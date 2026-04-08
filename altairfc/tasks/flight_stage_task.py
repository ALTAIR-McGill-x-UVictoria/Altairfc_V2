from __future__ import annotations

import logging
import time
from collections import deque

from config.settings import FlightStageConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

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

    # ------------------------------------------------------------------
    # BaseTask lifecycle
    # ------------------------------------------------------------------

    def setup(self) -> None:
        logger.info("FlightStageTask: initializing — writing all event.* keys to 0")
        for key, val in self._flags.items():
            self.datastore.write(f"event.{key}", val)

    def execute(self) -> None:
        now = time.monotonic()

        baro_alt: float = self.datastore.read("mavlink.environment.baro_alt", default=0.0)
        climb:    float = self.datastore.read("mavlink.environment.climb",    default=0.0)
        arm_state: int  = int(self.datastore.read("event.arm_state",          default=0))

        # Keep rolling altitude history and update apogee
        self._alt_history.append((now, baro_alt))
        self._prune_history(now, max(
            self._cfg.ascent_detect_window_s,
            self._cfg.termination_confirm_window_s,
        ))
        if self._stage == STAGE_ASCENT and baro_alt > self._measured_apogee:
            self._measured_apogee = baro_alt

        # Run state transitions
        new_stage = self._transition(now, baro_alt, climb, arm_state)
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
        arm_state: int,
    ) -> int:
        stage = self._stage

        if stage == STAGE_PREFLIGHT:
            # Mirror arm_state from DataStore (set by external command interface)
            self._write_flag("arm_state", arm_state)
            if arm_state:
                return STAGE_ARMED

        elif stage == STAGE_ARMED:
            self._write_flag("arm_state", arm_state)
            if self._detect_launch(now, baro_alt):
                self._write_flag("launch_detected", 1)
                return STAGE_LAUNCH

        elif stage == STAGE_LAUNCH:
            if self._detect_ascent(now, baro_alt):
                self._write_flag("ascent_active", 1)
                self._measured_apogee = baro_alt
                return STAGE_ASCENT

        elif stage == STAGE_ASCENT:
            # Fire cutdown if above termination altitude
            if (
                baro_alt >= self._cfg.termination_altitude_m
                and not self._flags["cutdown_fired"]
            ):
                logger.warning(
                    "FlightStageTask: termination altitude %.1f m reached — firing cutdown",
                    self._cfg.termination_altitude_m,
                )
                self._write_flag("cutdown_fired", 1)
                self._cutdown_triggered_alt = baro_alt
                self._cutdown_trigger_time = now

            # Check termination confirmation (significant drop after cutdown)
            if self._flags["cutdown_fired"] and self._cutdown_triggered_alt is not None:
                drop = self._cutdown_triggered_alt - baro_alt
                elapsed = now - (self._cutdown_trigger_time or now)
                if (
                    drop >= self._cfg.termination_confirm_drop_m
                    and elapsed <= self._cfg.termination_confirm_window_s
                ):
                    self._write_flag("termination_fired", 1)
                    return STAGE_TERMINATION

                # Cutdown window expired without confirmation → treat as burst
                if elapsed > self._cfg.termination_confirm_window_s:
                    self._write_flag("burst_detected", 1)
                    return STAGE_BURST

            # Natural burst: in burst altitude zone and climbing very slowly
            in_burst_zone = baro_alt >= (
                self._cfg.burst_altitude_m - self._cfg.burst_altitude_uncertainty_m
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
                if baro_alt <= self._measured_apogee * self._cfg.apogee_fraction:
                    self._write_flag("ascent_active", 0)
                    self._write_flag("descent_active", 1)
                    return STAGE_DESCENT

        elif stage == STAGE_DESCENT:
            if self._measured_apogee > 0:
                if baro_alt <= self._measured_apogee * self._cfg.landing_fraction:
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
                if stationary_duration >= self._cfg.recovery_stationary_s:
                    self._write_flag("recovery_active", 1)
                    return STAGE_RECOVERY

        return stage

    # ------------------------------------------------------------------
    # Detection helpers
    # ------------------------------------------------------------------

    def _detect_launch(self, now: float, baro_alt: float) -> bool:
        """True if altitude gained ≥ _LAUNCH_GAIN_M over the last _LAUNCH_WINDOW_S seconds."""
        cutoff = now - _LAUNCH_WINDOW_S
        old_pts = [(t, a) for t, a in self._alt_history if t <= cutoff]
        if not old_pts:
            return False
        oldest_alt = old_pts[-1][1]
        return (baro_alt - oldest_alt) >= _LAUNCH_GAIN_M

    def _detect_ascent(self, now: float, baro_alt: float) -> bool:
        """True if altitude gained ≥ ascent_detect_gain_m over ascent_detect_window_s."""
        cutoff = now - self._cfg.ascent_detect_window_s
        old_pts = [(t, a) for t, a in self._alt_history if t <= cutoff]
        if not old_pts:
            return False
        oldest_alt = old_pts[-1][1]
        return (baro_alt - oldest_alt) >= self._cfg.ascent_detect_gain_m

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
