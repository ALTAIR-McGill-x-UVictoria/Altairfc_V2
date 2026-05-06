from __future__ import annotations

import logging
import time
from collections import deque

from config.settings import FlightStageConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Hardware stubs — replace with real GPIO calls once pins are assigned
# ---------------------------------------------------------------------------

def _hw_read_arm_switch() -> bool:
    """
    TODO: Read the physical arm switch / GPIO input.

    Example (RPi.GPIO):
        import RPi.GPIO as GPIO
        ARM_SWITCH_PIN = 17          # BCM pin number — configure here
        GPIO.setup(ARM_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        return GPIO.input(ARM_SWITCH_PIN) == GPIO.HIGH

    Example (gpiozero):
        from gpiozero import Button
        _arm_button = Button(17)
        return _arm_button.is_pressed

    Returns False (unarmed) until implemented.
    """
    return False


def _hw_fire_cutdown() -> None:
    """
    TODO: Actuate the cutdown mechanism via GPIO output.

    Example (RPi.GPIO):
        import RPi.GPIO as GPIO
        CUTDOWN_PIN = 27             # BCM pin number — configure here
        GPIO.setup(CUTDOWN_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(CUTDOWN_PIN, GPIO.HIGH)
        # Hold high for the mechanism's required pulse duration, then release:
        # time.sleep(0.5)
        # GPIO.output(CUTDOWN_PIN, GPIO.LOW)

    Example (gpiozero):
        from gpiozero import OutputDevice
        _cutdown = OutputDevice(27, active_high=True, initial_value=False)
        _cutdown.on()

    Does nothing until implemented.
    """
    logger.warning("FlightStageTask: _hw_fire_cutdown() called — STUB, no GPIO action taken")

# Flight stage integer constants
STAGE_PREFLIGHT    = 0
STAGE_ARMED        = 1
STAGE_LAUNCH       = 2
STAGE_ASCENT       = 3
STAGE_TERMINATION  = 4
STAGE_BURST        = 5
STAGE_DESCENT      = 6
STAGE_LANDING      = 7
STAGE_RECOVERY     = 8

# Preflight: thresholds for "connected" checks
_MAVLINK_STALENESS_S  = 3.0   # mavlink data must have arrived within this many seconds
_VESC_RPM_TIMEOUT_S   = 5.0   # VESC data must have arrived within this many seconds

# Arm checks
_SPINUP_RPM           = 500   # minimum RPM both motors must reach during test spin
_SPINUP_DURATION_S    = 5.0   # how long to wait for spin-up response
_GPS_MIN_SV           = 4     # minimum satellites for a valid fix
_NEUTRAL_YAW_RATE     = 0.15  # rad/s — max yaw rate to consider orientation stable

# Altitude tolerance for "stationary" check (m)
_STATIONARY_BAND_M = 2.0

# Minimum climb rate to still consider ascending (m/s)
_ASCENDING_RATE_THRESHOLD = 0.5

# How long climb rate must be below threshold to declare burst (s)
_BURST_RATE_WINDOW_S = 5.0

# Altitude gain required over 10 s to confirm launch (m)
_LAUNCH_GAIN_M   = 20.0
_LAUNCH_WINDOW_S = 10.0


class FlightStageTask(BaseTask):
    """
    Autonomously drives the flight stage state machine by reading barometric
    altitude from the DataStore and writing event.* flags back.

    Stage sequence:
        Pre-flight (0) → Armed (1) → Launch (2) → Ascent (3)
            → Termination (4)  [if cutdown fires and is confirmed]
            → Burst (5)        [if natural burst or unconfirmed cutdown]
        → Descent (6) → Landing (7) → Recovery (8)

    DataStore keys read:
        mavlink.environment.baro_alt  — barometric altitude MSL (m)
        mavlink.environment.climb     — vertical speed (m/s)
        event.arm_state               — set externally to 1 to arm

    DataStore keys written (all under "event.*"):
        flight_stage, launch_detected, ascent_active,
        termination_fired, burst_detected, cutdown_fired,
        descent_active, landing_detected, recovery_active
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        config: FlightStageConfig,
    ) -> None:
        super().__init__(name, period_s, datastore)
        self._cfg = config

        # State machine
        self._stage: int = STAGE_PREFLIGHT
        self._measured_apogee: float = 0.0

        # Rolling history: deque of (timestamp, altitude) tuples
        self._alt_history: deque[tuple[float, float]] = deque()

        # Termination confirmation tracking
        self._cutdown_triggered_alt: float | None = None
        self._cutdown_trigger_time: float | None = None

        # Burst (slow-rate) detection
        self._low_rate_since: float | None = None

        # Recovery stationary tracking
        self._stationary_ref_alt: float | None = None
        self._stationary_since: float | None = None

        # Shadow of currently written flags (avoids redundant DS writes)
        self._flags: dict[str, int] = {
            "flight_stage":       0,
            "arm_state":          0,
            "preflight_ok":       0,
            "arm_checks_ok":      0,
            "launch_detected":    0,
            "ascent_active":      0,
            "termination_fired":  0,
            "burst_detected":     0,
            "descent_active":     0,
            "landing_detected":   0,
            "cutdown_fired":      0,
            "recovery_active":    0,
            "data_logging_active": 0,
        }

        # Arm check state
        self._arm_cmd_pending:  bool  = False
        self._arm_check_start:  float = 0.0

    # ------------------------------------------------------------------
    # BaseTask lifecycle
    # ------------------------------------------------------------------

    def setup(self) -> None:
        logger.info("FlightStageTask: initializing — writing all event.* keys to 0")
        for key, val in self._flags.items():
            self.datastore.write(f"event.{key}", val)
        self._write_flag("data_logging_active", 1)

    def execute(self) -> None:
        now = time.monotonic()

        baro_alt: float = self.datastore.read("mavlink.environment.baro_alt", default=0.0)
        climb:    float = self.datastore.read("mavlink.environment.climb",    default=0.0)

        # Poll GS ARM command — only accepted in PREFLIGHT once preflight_ok
        if float(self.datastore.read("command.arm", default=0.0)) >= 1.0:
            self.datastore.write("command.arm", 0.0)
            if self._stage == STAGE_PREFLIGHT and self._flags["preflight_ok"]:
                logger.info("FlightStageTask: ARM command received — starting arm checks")
                self._arm_cmd_pending = True
                self._arm_check_start = now
            else:
                logger.warning(
                    "FlightStageTask: ARM command rejected (stage=%d preflight_ok=%d)",
                    self._stage, self._flags["preflight_ok"],
                )

        # Poll GS LAUNCH_OK command — only valid in ARMED stage once arm_checks_ok
        if float(self.datastore.read("command.launch_ok", default=0.0)) >= 1.0:
            self.datastore.write("command.launch_ok", 0.0)
            if self._stage == STAGE_ARMED and self._flags["arm_checks_ok"]:
                self._write_flag("launch_detected", 1)
                self._stage = STAGE_LAUNCH
                logger.info("FlightStageTask: LAUNCH_OK received — advancing to STAGE_LAUNCH")
            else:
                logger.warning(
                    "FlightStageTask: LAUNCH_OK rejected (stage=%d arm_checks_ok=%d)",
                    self._stage, self._flags["arm_checks_ok"],
                )

        # Physical arm switch (stub — returns False until GPIO is configured)
        if _hw_read_arm_switch():
            self._write_flag("arm_state", 1)

        # Read current settings from DataStore (updated live by UpdateSettingCommand).
        # Fall back to the config object loaded at startup if a key is missing.
        cfg = FlightStageConfig(
            termination_altitude_m       = float(self.datastore.read("settings.termination_altitude_m",       default=self._cfg.termination_altitude_m)),
            burst_altitude_m             = float(self.datastore.read("settings.burst_altitude_m",             default=self._cfg.burst_altitude_m)),
            burst_altitude_uncertainty_m = float(self.datastore.read("settings.burst_altitude_uncertainty_m", default=self._cfg.burst_altitude_uncertainty_m)),
            ascent_detect_window_s       = float(self.datastore.read("settings.ascent_detect_window_s",       default=self._cfg.ascent_detect_window_s)),
            ascent_detect_gain_m         = float(self.datastore.read("settings.ascent_detect_gain_m",         default=self._cfg.ascent_detect_gain_m)),
            apogee_fraction              = float(self.datastore.read("settings.apogee_fraction",              default=self._cfg.apogee_fraction)),
            landing_fraction             = float(self.datastore.read("settings.landing_fraction",             default=self._cfg.landing_fraction)),
            recovery_stationary_s        = float(self.datastore.read("settings.recovery_stationary_s",        default=self._cfg.recovery_stationary_s)),
            termination_confirm_drop_m   = float(self.datastore.read("settings.termination_confirm_drop_m",   default=self._cfg.termination_confirm_drop_m)),
            termination_confirm_window_s = float(self.datastore.read("settings.termination_confirm_window_s", default=self._cfg.termination_confirm_window_s)),
        )

        # Keep rolling altitude history and update apogee
        self._alt_history.append((now, baro_alt))
        self._prune_history(now, max(
            cfg.ascent_detect_window_s,
            cfg.termination_confirm_window_s,
        ))
        if self._stage == STAGE_ASCENT and baro_alt > self._measured_apogee:
            self._measured_apogee = baro_alt

        # Run state transitions
        new_stage = self._transition(now, baro_alt, climb, cfg)
        if new_stage != self._stage:
            logger.info("FlightStageTask: stage %d → %d", self._stage, new_stage)
            self._stage = new_stage

        self._write_flag("flight_stage", self._stage)

    def teardown(self) -> None:
        pass

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _transition(
        self,
        now: float,
        baro_alt: float,
        climb: float,
        cfg: FlightStageConfig,
    ) -> int:
        stage = self._stage

        if stage == STAGE_PREFLIGHT:
            preflight_ok = self._check_preflight(now)
            self._write_flag("preflight_ok", 1 if preflight_ok else 0)

            if self._arm_cmd_pending:
                arm_ok, arm_failures = self._check_arm(now)
                if arm_ok:
                    self._arm_cmd_pending = False
                    self._write_flag("arm_checks_ok", 1)
                    self._write_flag("arm_state", 1)
                    logger.info("FlightStageTask: arm checks passed — advancing to STAGE_ARMED")
                    return STAGE_ARMED
                elif now - self._arm_check_start > _SPINUP_DURATION_S + 2.0:
                    # Timed out — report failures and cancel
                    self._arm_cmd_pending = False
                    logger.warning("FlightStageTask: arm checks FAILED: %s", ", ".join(arm_failures))

        elif stage == STAGE_ARMED:
            self._write_flag("arm_state", 1)
            if self._detect_launch(now, baro_alt):
                self._write_flag("launch_detected", 1)
                return STAGE_LAUNCH

        elif stage == STAGE_LAUNCH:
            if self._detect_ascent(now, baro_alt, cfg):
                self._write_flag("ascent_active", 1)
                self._measured_apogee = baro_alt
                return STAGE_ASCENT

        elif stage == STAGE_ASCENT:
            # Fire cutdown if above termination altitude
            if (
                baro_alt >= cfg.termination_altitude_m
                and not self._flags["cutdown_fired"]
            ):
                logger.warning(
                    "FlightStageTask: termination altitude %.1f m reached — firing cutdown",
                    cfg.termination_altitude_m,
                )
                _hw_fire_cutdown()  # TODO: stub — replace with real GPIO once pin is assigned
                self._write_flag("cutdown_fired", 1)
                self._cutdown_triggered_alt = baro_alt
                self._cutdown_trigger_time = now

            # Check termination confirmation (significant drop after cutdown)
            if self._flags["cutdown_fired"] and self._cutdown_triggered_alt is not None:
                drop = self._cutdown_triggered_alt - baro_alt
                elapsed = now - (self._cutdown_trigger_time or now)
                if (
                    drop >= cfg.termination_confirm_drop_m
                    and elapsed <= cfg.termination_confirm_window_s
                ):
                    self._write_flag("termination_fired", 1)
                    return STAGE_TERMINATION

                # Cutdown window expired without confirmation → treat as burst
                if elapsed > cfg.termination_confirm_window_s:
                    self._write_flag("burst_detected", 1)
                    return STAGE_BURST

            # Natural burst: in burst altitude zone and climbing very slowly
            in_burst_zone = baro_alt >= (
                cfg.burst_altitude_m - cfg.burst_altitude_uncertainty_m
            )
            if in_burst_zone and not self._flags["cutdown_fired"]:
                if climb < _ASCENDING_RATE_THRESHOLD:
                    if self._low_rate_since is None:
                        self._low_rate_since = now
                    elif now - self._low_rate_since >= _BURST_RATE_WINDOW_S:
                        self._write_flag("burst_detected", 1)
                        return STAGE_BURST
                else:
                    self._low_rate_since = None

        elif stage in (STAGE_TERMINATION, STAGE_BURST):
            if self._measured_apogee > 0:
                if baro_alt <= self._measured_apogee * cfg.apogee_fraction:
                    self._write_flag("ascent_active", 0)
                    self._write_flag("descent_active", 1)
                    return STAGE_DESCENT

        elif stage == STAGE_DESCENT:
            if self._measured_apogee > 0:
                if baro_alt <= self._measured_apogee * cfg.landing_fraction:
                    self._write_flag("descent_active", 0)
                    self._write_flag("landing_detected", 1)
                    return STAGE_LANDING

        elif stage == STAGE_LANDING:
            if self._stationary_ref_alt is None:
                self._stationary_ref_alt = baro_alt
                self._stationary_since = now
            elif abs(baro_alt - self._stationary_ref_alt) > _STATIONARY_BAND_M:
                # Still moving — reset reference
                self._stationary_ref_alt = baro_alt
                self._stationary_since = now
            else:
                stationary_duration = now - (self._stationary_since or now)
                if stationary_duration >= cfg.recovery_stationary_s:
                    self._write_flag("recovery_active", 1)
                    return STAGE_RECOVERY

        return stage

    # ------------------------------------------------------------------
    # Detection helpers
    # ------------------------------------------------------------------

    def _check_preflight(self, now: float) -> bool:
        """
        Verify all hardware is connected and reporting fresh data.
        Returns True only when every check passes.
        """
        failures: list[str] = []

        # MAVLink: attitude data must be arriving
        mavlink_entry = self.datastore.read_with_timestamp("mavlink.attitude.yaw")
        if mavlink_entry is None or (now - mavlink_entry[1]) > _MAVLINK_STALENESS_S:
            failures.append("mavlink_stale")

        # RW VESC: rpm key must exist and be fresh
        rw_entry = self.datastore.read_with_timestamp("rw.rpm")
        if rw_entry is None or (now - rw_entry[1]) > _VESC_RPM_TIMEOUT_S:
            failures.append("rw_vesc_missing")

        # MM VESC: rpm key must exist and be fresh
        mm_entry = self.datastore.read_with_timestamp("mm.rpm")
        if mm_entry is None or (now - mm_entry[1]) > _VESC_RPM_TIMEOUT_S:
            failures.append("mm_vesc_missing")

        # GPS: module must be responding
        gps_active = int(self.datastore.read("gps.active", default=0))
        if not gps_active:
            failures.append("gps_not_active")

        if failures:
            logger.debug("FlightStageTask: preflight failures: %s", ", ".join(failures))
            return False
        return True

    def _check_arm(self, now: float) -> tuple[bool, list[str]]:
        """
        Run arm checks after ARM command received.
        Returns (all_ok, list_of_failures).
        Waits _SPINUP_DURATION_S for motors to respond before evaluating RPM.
        """
        failures: list[str] = []
        elapsed = now - self._arm_check_start

        # GPS fix quality
        gps_valid  = int(self.datastore.read("gps.valid",   default=0))
        gps_num_sv = int(self.datastore.read("gps.num_sv",  default=0))
        if not gps_valid or gps_num_sv < _GPS_MIN_SV:
            failures.append(f"gps_no_fix(sv={gps_num_sv})")

        # Neutral orientation — low yaw rate
        yaw_rate = abs(float(self.datastore.read("mavlink.attitude.yawspeed", default=999.0)))
        if yaw_rate > _NEUTRAL_YAW_RATE:
            failures.append(f"yaw_rate_high({yaw_rate:.2f}rad/s)")

        # Motor spin-up — only evaluate after spin-up window
        if elapsed >= _SPINUP_DURATION_S:
            rw_rpm = abs(float(self.datastore.read("rw.rpm", default=0.0)))
            mm_rpm = abs(float(self.datastore.read("mm.rpm", default=0.0)))
            if rw_rpm < _SPINUP_RPM:
                failures.append(f"rw_no_spin(rpm={rw_rpm:.0f})")
            if mm_rpm < _SPINUP_RPM:
                failures.append(f"mm_no_spin(rpm={mm_rpm:.0f})")

            return len(failures) == 0, failures

        # Still within spin-up window — only report non-motor failures
        non_motor = [f for f in failures if "spin" not in f]
        return False, non_motor  # not ready yet

    def _detect_launch(self, now: float, baro_alt: float) -> bool:
        """True if altitude gained ≥ _LAUNCH_GAIN_M over the last _LAUNCH_WINDOW_S seconds."""
        cutoff = now - _LAUNCH_WINDOW_S
        old_pts = [(t, a) for t, a in self._alt_history if t <= cutoff]
        if not old_pts:
            return False
        oldest_alt = old_pts[-1][1]
        return (baro_alt - oldest_alt) >= _LAUNCH_GAIN_M

    def _detect_ascent(self, now: float, baro_alt: float, cfg: FlightStageConfig) -> bool:
        """True if altitude gained ≥ ascent_detect_gain_m over ascent_detect_window_s."""
        cutoff = now - cfg.ascent_detect_window_s
        old_pts = [(t, a) for t, a in self._alt_history if t <= cutoff]
        if not old_pts:
            return False
        oldest_alt = old_pts[-1][1]
        return (baro_alt - oldest_alt) >= cfg.ascent_detect_gain_m

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _write_flag(self, key: str, value: int) -> None:
        """Write event.{key} to DataStore only if the value changed."""
        if self._flags.get(key) != value:
            self._flags[key] = value
            self.datastore.write(f"event.{key}", value)

    def _prune_history(self, now: float, max_window_s: float) -> None:
        """Remove history entries older than max_window_s."""
        cutoff = now - max_window_s
        while self._alt_history and self._alt_history[0][0] < cutoff:
            self._alt_history.popleft()
