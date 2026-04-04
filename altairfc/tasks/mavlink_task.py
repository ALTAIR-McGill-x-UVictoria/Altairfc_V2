from __future__ import annotations

import logging
import time

from pymavlink import mavutil

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

# MAVLink message types this task subscribes to
_SUBSCRIBED_TYPES = ("ATTITUDE", "GLOBAL_POSITION_INT")


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
        mavlink.gps.lat              (float, deg)   — from GLOBAL_POSITION_INT
        mavlink.gps.lon              (float, deg)
        mavlink.gps.alt              (float, m)     — MSL altitude
        mavlink.gps.relative_alt     (float, m)     — above home
        mavlink.gps.hdg              (float, deg)   — vehicle heading 0-360
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
                self._request_message_rates()
                return  # connected successfully
            except Exception as e:
                logger.warning(
                    "MavlinkTask: connection failed (%s) — retrying in %.0fs",
                    e,
                    self._connect_retry_s,
                )
                self._stop_event.wait(timeout=self._connect_retry_s)

    def _request_message_rates(self) -> None:
        """
        Explicitly ask the Pixhawk to stream required message types.
        Uses MAV_CMD_SET_MESSAGE_INTERVAL (command 511).
        Interval is in microseconds; -1 disables, 0 = default rate.
        """
        # (message_id, interval_us)
        requests = [
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            20_000),   # 50 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200_000),  #  5 Hz
        ]
        for msg_id, interval_us in requests:
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,           # confirmation
                msg_id,      # param1: message ID
                interval_us, # param2: interval in microseconds
                0, 0, 0, 0,  # params 3-6 unused
                0,           # param7 unused
            )
            logger.info(
                "MavlinkTask: requested MSG_ID=%d at %.1f Hz",
                msg_id, 1e6 / interval_us,
            )

    def execute(self) -> None:
        if self._master is None:
            return
        # Drain all queued messages each cycle so slow message types
        # (e.g. GLOBAL_POSITION_INT at 5 Hz) are not starved by faster ones.
        while True:
            msg = self._master.recv_match(type=list(_SUBSCRIBED_TYPES), blocking=False)
            if msg is None:
                break
            self._handle_message(msg)

    def _handle_message(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            self.datastore.write("mavlink.attitude.roll",       msg.roll)
            self.datastore.write("mavlink.attitude.pitch",      msg.pitch)
            self.datastore.write("mavlink.attitude.yaw",        msg.yaw)
            self.datastore.write("mavlink.attitude.rollspeed",  msg.rollspeed)
            self.datastore.write("mavlink.attitude.pitchspeed", msg.pitchspeed)
            self.datastore.write("mavlink.attitude.yawspeed",   msg.yawspeed)

        elif msg_type == "GLOBAL_POSITION_INT":
            # lat/lon are in 1e-7 degrees, alt/relative_alt in mm, hdg in cdeg
            self.datastore.write("mavlink.gps.lat",          msg.lat          / 1e7)
            self.datastore.write("mavlink.gps.lon",          msg.lon          / 1e7)
            self.datastore.write("mavlink.gps.alt",          msg.alt          / 1e3)
            self.datastore.write("mavlink.gps.relative_alt", msg.relative_alt / 1e3)
            self.datastore.write("mavlink.gps.hdg",          msg.hdg          / 1e2)

    def teardown(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
            logger.info("MavlinkTask: connection closed")
