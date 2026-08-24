"""
ALTAIR V2 Flight Computer — Entry Point

Startup sequence:
  1. Load configuration from config/settings.toml
  2. Create the shared DataStore (blackboard)
  3. Import all packet types so the registry is populated before TelemetryTask starts
  4. Instantiate and register all enabled tasks with the TaskScheduler
  5. Install OS signal handlers (SIGINT, SIGTERM)
  6. Start all tasks
  7. Block on the shutdown event
  8. Stop all tasks gracefully
"""

from __future__ import annotations

import logging
import subprocess
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Bootstrap logging before importing project modules so their loggers work
# ---------------------------------------------------------------------------
from core.log_format import setup_logging
setup_logging("INFO")
logger = logging.getLogger("main")

# ---------------------------------------------------------------------------
# Project imports
# ---------------------------------------------------------------------------
from config.settings import ControllerConfig, GroundStationConfig, SerialPortConfig, SystemConfig
from core.datastore import DataStore
from core.photodiode_stream import PhotodiodeSampleBuffer
from core.lifecycle import install_signal_handlers, shutdown_event
from core.scheduler import TaskScheduler
from core.watchdog import WatchdogThread
from core.buzzer_player import BuzzerPlayer
from drivers.buzzer import TUNE_PENDING, TUNE_SUCCESS, TUNE_SUCCESS_REVERSE

# Import all packet modules so their @register decorators fire before
# TelemetryTask.execute() iterates the registry.
import telemetry.packets.heartbeat       # noqa: F401
import telemetry.packets.attitude        # noqa: F401
import telemetry.packets.power           # noqa: F401
import telemetry.packets.vesc            # noqa: F401
import telemetry.packets.photodiode      # noqa: F401
import telemetry.packets.gps             # noqa: F401
import telemetry.packets.environment     # noqa: F401
import telemetry.packets.events          # noqa: F401
import telemetry.packets.ack             # noqa: F401
import telemetry.packets.flight_settings  # noqa: F401
import telemetry.packets.pointing         # noqa: F401
import telemetry.packets.radio_config     # noqa: F401
import telemetry.packets.lighting         # noqa: F401

# Import command modules so their @register decorators populate command_registry
import telemetry.commands.arm            # noqa: F401
import telemetry.commands.launch_ok      # noqa: F401
import telemetry.commands.ping           # noqa: F401
import telemetry.commands.update_setting  # noqa: F401
import telemetry.commands.gs_gps         # noqa: F401
import telemetry.commands.radio_config    # noqa: F401

from tasks.gps_task import GpsTask
from tasks.mavlink_task import MavlinkTask
from tasks.command_receiver_task import CommandReceiverTask
from tasks.flight_stage_task import FlightStageTask
from tasks.photodiode_task import PhotodiodeTask
from tasks.power_task import PowerTask
from telemetry.telemetry_task import TelemetryTask
from telemetry.transport import SerialTransport
from telemetry.tee_server import TeeServer
from drivers.port_detect import find_lr900p_port
from tasks.pitch_task import PitchTask
from tasks.datalogger_task import DataLoggerTask
from tasks.radio_config_task import RadioConfigTask
from tasks.pointing_task import PointingTask
from tasks.lighting_task import LightingTask
from drivers.ads1115 import SENSE_RESISTOR_OHM
from drivers.beacon_led import BEACON_SENSE_RESISTOR_OHM



def main() -> None:
    buzzer = BuzzerPlayer()
    buzzer.start()
    buzzer.play(TUNE_PENDING)

    build_script = Path(__file__).parent / "drivers" / "build_all.sh"
    logger.info("Building C drivers via %s", build_script)
    subprocess.run(["bash", str(build_script)], check=True)

    config_path = Path(__file__).parent / "config" / "settings.toml"
    logger.info("Loading config from %s", config_path)
    config = SystemConfig.from_toml(config_path)

    # Create the per-session log directory now so both the file logger and
    # DataLoggerTask share the same timestamped folder. Explicit UTC (not
    # time.strftime's default localtime) so the folder name lines up with
    # the UTC timestamps recorded inside flight.log and the DataLogger CSVs
    # regardless of the Pi's system timezone setting.
    session_name = time.strftime("%Y-%m-%d_%H-%M-%SZ", time.gmtime())
    datalogger_enabled = config.tasks.get("datalogger", None)
    if datalogger_enabled and datalogger_enabled.enabled:
        session_dir = config.log_root / session_name
        session_dir.mkdir(parents=True, exist_ok=True)
        setup_logging(config.log_level, log_file=session_dir / "flight.log")
        logger.info("Log session: %s", session_dir)
    else:
        session_dir = None
        setup_logging(config.log_level)

    def _resync_log_session(gps_dt) -> None:
        # The Pi has no RTC and no network in flight, so session_dir above was
        # named from a stale system clock (whatever it was at boot). Called
        # once by GpsTask the first time GPS gives us real UTC time — rename
        # the directory to match. Safe to do while flight.log/CSV files are
        # open: renaming a directory doesn't affect already-open file
        # descriptors inside it on Linux.
        nonlocal session_dir
        if session_dir is None:
            return
        new_dir = session_dir.parent / gps_dt.strftime("%Y-%m-%d_%H-%M-%SZ")
        if new_dir == session_dir:
            return
        try:
            session_dir.rename(new_dir)
        except OSError as exc:
            logger.warning("Failed to rename log session %s -> %s: %s", session_dir, new_dir, exc)
            return
        logger.info("Log session renamed after GPS time sync: %s -> %s", session_dir, new_dir)
        session_dir = new_dir
        datastore.write("system.log_dir", session_dir.name)

    datastore = DataStore()
    datastore.write("system.log_dir", session_dir.name if session_dir is not None else "")
    photodiode_sample_buffer = (
        PhotodiodeSampleBuffer() if config.telemetry is not None else None
    )

    # Write all flight settings to DataStore before tasks start.
    # FlightStageTask, RWTask, and MMTask read these keys each cycle so that
    # an UpdateSettingCommand from the GS takes effect without a restart.
    _fs = config.flight_stage
    for _key, _val in {
        "settings.termination_altitude_m":       _fs.termination_altitude_m,
        "settings.burst_altitude_m":             _fs.burst_altitude_m,
        "settings.burst_altitude_uncertainty_m": _fs.burst_altitude_uncertainty_m,
        "settings.ascent_detect_window_s":       _fs.ascent_detect_window_s,
        "settings.ascent_detect_gain_m":         _fs.ascent_detect_gain_m,
        "settings.apogee_fraction":              _fs.apogee_fraction,
        "settings.landing_fraction":             _fs.landing_fraction,
        "settings.recovery_stationary_s":        _fs.recovery_stationary_s,
        "settings.termination_confirm_drop_m":   _fs.termination_confirm_drop_m,
        "settings.termination_confirm_window_s": _fs.termination_confirm_window_s,
        "settings.pointing_activate_altitude_m": _fs.pointing_activate_altitude_m,
        "settings.pointing_duration_min":        _fs.pointing_duration_min,
    }.items():
        datastore.write(_key, float(_val))
    datastore.write("settings.gs_use_hardcoded", 1.0 if config.ground_station.use_hardcoded else 0.0)
    datastore.write("settings.gs_lat", float(config.ground_station.latitude))
    datastore.write("settings.gs_lon", float(config.ground_station.longitude))
    datastore.write("settings.gs_alt", float(config.ground_station.altitude))
    logger.info("Wrote 18 flight settings to DataStore")

    scheduler = TaskScheduler(datastore, config)

    # ------------------------------------------------------------------
    # Register tasks — scheduler.register() silently skips disabled tasks
    # ------------------------------------------------------------------

    
    scheduler.register(
        MavlinkTask(
            name="mavlink",
            period_s=config.tasks["mavlink"].period_s,
            datastore=datastore,
            port_config=config.mavlink,
        )
    )

    scheduler.register(
        GpsTask(
            name="gps",
            period_s=config.tasks["gps"].period_s,
            datastore=datastore,
            on_time_sync=_resync_log_session,
        )
    )

    scheduler.register(
        PitchTask(
            name="sphere_pitch",
            period_s=config.tasks["sphere_pitch"].period_s,
            datastore=datastore,
            ground_station=config.ground_station,
        )
    )

    scheduler.register(
        PointingTask(
            name    = "pointing",
            period_s    = config.tasks["pointing"].period_s,
            datastore   = datastore,
            rw_port = config.rw_esc,
            rw_controller_config    = config.controller["reaction_wheel"],
            ground_station  = config.ground_station,
            pointing_config = config.pointing,
        )
    )

    telemetry_tee_server: TeeServer | None = None
    if config.telemetry is not None:
        telemetry_transport = SerialTransport(
            port=config.telemetry.port,
            baud=config.telemetry.baud,
            # Only when the operator asked for auto-detection — an explicit
            # fixed path is retried as exactly that path, never silently
            # swapped for whatever other CP210x device shows up later. See
            # SerialPortConfig.is_auto and SerialTransport's port_resolver.
            port_resolver=find_lr900p_port if config.telemetry.is_auto else None,
        )

        if config.telemetry_tee.enabled:
            # on_recv wires GS->FC command bytes arriving over the tunnel
            # straight into the same buffer CommandReceiverTask already
            # drains for serial commands (see SerialTransport.feed_command_bytes)
            # — the only surviving GS->FC path now that the radio link
            # itself is RF-unidirectional (FC->GS only).
            telemetry_tee_server = TeeServer(
                host=config.telemetry_tee.host,
                port=config.telemetry_tee.port,
                on_recv=telemetry_transport.feed_command_bytes,
            )
            telemetry_tee_server.start()
            telemetry_transport.attach_tee(telemetry_tee_server)
        scheduler.register(
            TelemetryTask(
                name="telemetry",
                period_s=config.tasks["telemetry"].period_s,
                datastore=datastore,
                transport=telemetry_transport,
                photodiode_samples=photodiode_sample_buffer,
                photodiode_batch_size=int(
                    config.tasks["photodiode"].extra.get("batch_size", 50)
                ),
                photodiode_batch_rate_hz=float(
                    config.tasks["photodiode"].extra.get(
                        "batch_tx_rate_hz", 2.0
                    )
                ),
            )
        )
        scheduler.register(
            CommandReceiverTask(
                name="command_receiver",
                period_s=config.tasks["command_receiver"].period_s,
                datastore=datastore,
                transport=telemetry_transport,
                buzzer=buzzer,
            )
        )
        scheduler.register(
            RadioConfigTask(
                name="radio_config",
                period_s=config.tasks["radio_config"].period_s,
                datastore=datastore,
                transport=telemetry_transport,
                radio_config=config.radio_config,
            )
        )
    else:
        logger.info("Telemetry radio not configured — TelemetryTask, CommandReceiverTask, and RadioConfigTask skipped")

    scheduler.register(
        FlightStageTask(
            name="flight_stage",
            period_s=config.tasks["flight_stage"].period_s,
            datastore=datastore,
            config=config.flight_stage,
            scheduler=scheduler,
        )
    )

    scheduler.register(
        PhotodiodeTask(
            name="photodiode",
            period_s=config.tasks["photodiode"].period_s,
            datastore=datastore,
            signal_data_rate=config.tasks["photodiode"].extra.get(
                "signal_data_rate", "SPS_800"
            ),
            temperature_data_rate=config.tasks["photodiode"].extra.get(
                "temperature_data_rate", "SPS_2000"
            ),
            temperature_period_s=float(
                config.tasks["photodiode"].extra.get(
                    "temperature_period_s", 1.0
                )
            ),
            bias_voltage_v=config.tasks["photodiode"].extra.get(
                "bias_voltage_v", 0.0
            ),
            sample_buffer=photodiode_sample_buffer,
            log_path=(
                session_dir / "PhotodiodeSamples.csv"
                if session_dir is not None
                else None
            ),
        )
    )

    scheduler.register(
        PowerTask(
            name="power",
            period_s=config.tasks["power"].period_s,
            datastore=datastore,
            i2c_dev=config.tasks["power"].extra.get("i2c_dev", "/dev/i2c-1"),
        )
    )

    scheduler.register(
        LightingTask(
            name="lighting",
            period_s=config.tasks["lighting"].period_s,
            datastore=datastore,
            beacon_windows=config.tasks["lighting"].extra.get("beacon_windows", []),
            i2c_dev=config.tasks["lighting"].extra.get("i2c_dev", "/dev/i2c-1"),
            sphere_target_current_a=config.tasks["lighting"].extra.get("sphere_target_current_a"),
            sphere_kp=config.tasks["lighting"].extra.get("sphere_kp"),
            sphere_ki=config.tasks["lighting"].extra.get("sphere_ki"),
            beacon_dac_code=config.tasks["lighting"].extra.get("beacon_dac_code", 1500),
            beacon_flash_hz=config.tasks["lighting"].extra.get("beacon_flash_hz", 2.0),
            sphere_sense_resistor_ohm=config.tasks["lighting"].extra.get(
                "sphere_sense_resistor_ohm", SENSE_RESISTOR_OHM
            ),
            beacon_sense_resistor_ohm=config.tasks["lighting"].extra.get(
                "beacon_sense_resistor_ohm", BEACON_SENSE_RESISTOR_OHM
            ),
        )
    )

    if session_dir is not None:
        scheduler.register(
            DataLoggerTask(
                name="datalogger",
                period_s=config.tasks["datalogger"].period_s,
                datastore=datastore,
                log_root=session_dir,
            )
        )

    # ------------------------------------------------------------------
    # Signal handlers + startup
    # ------------------------------------------------------------------
    install_signal_handlers(scheduler) # handles CTRL-C and kill signals for graceful shutdown
    logger.info("Starting ALTAIR V2 flight computer")
    scheduler.start_all()
    buzzer.play(TUNE_SUCCESS)

    watchdog = WatchdogThread(scheduler, watchdog_sec=config.watchdog_sec)
    watchdog.start()

    # Block main thread until SIGINT/SIGTERM or a critical task failure
    scheduler.shutdown_event.wait()
    logger.info("Shutdown event received — stopping all tasks")
    buzzer.play(TUNE_SUCCESS_REVERSE)
    watchdog.stop()
    scheduler.stop_all()
    buzzer.stop()
    if telemetry_tee_server is not None:
        telemetry_tee_server.stop()
    logger.info("ALTAIR V2 shutdown complete")


if __name__ == "__main__":
    main()
