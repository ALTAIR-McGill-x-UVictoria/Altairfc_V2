from __future__ import annotations

import logging
import time

from pymavlink import mavutil

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

# MAVLink message types this task subscribes to
_SUBSCRIBED_TYPES = ("ATTITUDE",)


class MavlinkTask(BaseTask):
    """
    Reads MAVLink messages from the Pixhawk 6X mini and writes them to the DataStore.

    Runs at 50 Hz (configurable). Uses non-blocking recv_match() so the scheduler
    controls cadence. Multiple message types can be added to _SUBSCRIBED_TYPES
    without changing the task loop.

    DataStore keys written:
        mavlink.attitude.roll        (float, rad)
        mavlink.attitude.pitch       (float, rad)
        mavlink.attitude.yaw         (float, rad)
        mavlink.attitude.rollspeed   (float, rad/s)
        mavlink.attitude.pitchspeed  (float, rad/s)
        mavlink.attitude.yawspeed    (float, rad/s)
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        port_config: SerialPortConfig,
        heartbeat_timeout_s: float = 10.0,
        connect_retry_s: float = 5.0,
    ) -> None:
        super().__init__(name, period_s, datastore)
        self._port_config = port_config
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._connect_retry_s = connect_retry_s
        self._master = None

    def setup(self) -> None:
        while not self._stop_event.is_set():
            try:
                logger.info(
                    "MavlinkTask: connecting to %s @ %d baud",
                    self._port_config.port,
                    self._port_config.baud,
                )
                self._master = mavutil.mavlink_connection(
                    self._port_config.port,
                    baud=self._port_config.baud,
                )
                self._master.wait_heartbeat(timeout=self._heartbeat_timeout_s)
                logger.info(
                    "MavlinkTask: heartbeat received (system %d, component %d)",
                    self._master.target_system,
                    self._master.target_component,
                )
                return  # connected successfully
            except Exception as e:
                logger.warning(
                    "MavlinkTask: connection failed (%s) — retrying in %.0fs",
                    e,
                    self._connect_retry_s,
                )
                self._stop_event.wait(timeout=self._connect_retry_s)

    def execute(self) -> None:
        if self._master is None:
            return
        msg = self._master.recv_match(type=list(_SUBSCRIBED_TYPES), blocking=False)
        if msg is None:
            return

        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            self.datastore.write("mavlink.attitude.roll",       msg.roll)
            self.datastore.write("mavlink.attitude.pitch",      msg.pitch)
            self.datastore.write("mavlink.attitude.yaw",        msg.yaw)
            self.datastore.write("mavlink.attitude.rollspeed",  msg.rollspeed)
            self.datastore.write("mavlink.attitude.pitchspeed", msg.pitchspeed)
            self.datastore.write("mavlink.attitude.yawspeed",   msg.yawspeed)

    def teardown(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
            logger.info("MavlinkTask: connection closed")
