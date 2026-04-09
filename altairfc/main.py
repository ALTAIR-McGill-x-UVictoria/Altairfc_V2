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
from config.settings import SystemConfig
from core.datastore import DataStore
from core.lifecycle import install_signal_handlers, shutdown_event
from core.scheduler import TaskScheduler

# Import all packet modules so their @register decorators fire before
# TelemetryTask.execute() iterates the registry.
import telemetry.packets.heartbeat     # noqa: F401
import telemetry.packets.attitude      # noqa: F401
import telemetry.packets.power         # noqa: F401
import telemetry.packets.vesc          # noqa: F401
import telemetry.packets.photodiode    # noqa: F401
import telemetry.packets.gps           # noqa: F401
import telemetry.packets.environment   # noqa: F401
import telemetry.packets.events        # noqa: F401
import telemetry.packets.ack           # noqa: F401

# Import command modules so their @register decorators populate command_registry
import telemetry.commands.arm          # noqa: F401
import telemetry.commands.launch_ok    # noqa: F401
import telemetry.commands.ping         # noqa: F401

from tasks.mavlink_task import MavlinkTask
from tasks.command_receiver_task import CommandReceiverTask
from tasks.flight_stage_task import FlightStageTask
from tasks.photodiode_task import PhotodiodeTask
from tasks.power_task import PowerTask
from tasks.control_task import ControlTask
from telemetry.telemetry_task import TelemetryTask
from telemetry.transport import SerialTransport


def main() -> None:
    config_path = Path(__file__).parent / "config" / "settings.toml"
    logger.info("Loading config from %s", config_path)
    config = SystemConfig.from_toml(config_path)

    setup_logging(config.log_level)

    datastore = DataStore()
    scheduler = TaskScheduler(datastore, config)

    # ------------------------------------------------------------------
    # Register tasks — scheduler.register() silently skips disabled tasks
    # ------------------------------------------------------------------
    scheduler.register(
        ControlTask(
            name="control",
            period_s=config.tasks["control"].period_s,
            datastore=datastore,
            rw_vesc_port=config.reaction_wheel,
        )
    )
    
    scheduler.register(
        MavlinkTask(
            name="mavlink",
            period_s=config.tasks["mavlink"].period_s,
            datastore=datastore,
            port_config=config.mavlink,
        )
    )

    telemetry_transport = SerialTransport(
        port=config.telemetry.port,
        baud=config.telemetry.baud,
    )
    scheduler.register(
        TelemetryTask(
            name="telemetry",
            period_s=config.tasks["telemetry"].period_s,
            datastore=datastore,
            transport=telemetry_transport,
        )
    )

    scheduler.register(
        CommandReceiverTask(
            name="command_receiver",
            period_s=config.tasks["command_receiver"].period_s,
            datastore=datastore,
            transport=telemetry_transport,
        )
    )

    scheduler.register(
        FlightStageTask(
            name="flight_stage",
            period_s=config.tasks["flight_stage"].period_s,
            datastore=datastore,
            config=config.flight_stage,
        )
    )

    scheduler.register(
        PhotodiodeTask(
            name="photodiode",
            period_s=config.tasks["photodiode"].period_s,
            datastore=datastore,
        )
    )

    scheduler.register(
        PowerTask(
            name="power",
            period_s=config.tasks["power"].period_s,
            datastore=datastore,
        )
    )

    # ------------------------------------------------------------------
    # Signal handlers + startup
    # ------------------------------------------------------------------
    install_signal_handlers(scheduler) # handles CTRL-C and kill signals for graceful shutdown
    logger.info("Starting ALTAIR V2 flight computer")
    scheduler.start_all()

    # Block main thread until SIGINT/SIGTERM or a critical task failure
    scheduler.shutdown_event.wait()
    logger.info("Shutdown event received — stopping all tasks")
    scheduler.stop_all()
    logger.info("ALTAIR V2 shutdown complete")


if __name__ == "__main__":
    main()
