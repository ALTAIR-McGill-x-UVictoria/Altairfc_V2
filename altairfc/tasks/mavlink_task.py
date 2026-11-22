from __future__ import annotations

import logging
import math
import time

from pymavlink import mavutil

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

# MAVLink message types this task subscribes to.
# Position comes from GLOBAL_POSITION_INT (PX4 EKF), which follows whichever GPS receiver
# PX4 has selected (SENS_GPS_PRIME / automatic failover), so every reader of mavlink.gps.*
# inherits that choice. GPS_RAW_INT / GPS2_RAW (the two raw receivers) supply fix quality only.
# LOCAL_POSITION_NED provides relative altitude (above home).
_SUBSCRIBED_TYPES = ("ATTITUDE", "ATTITUDE_QUATERNION", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "GPS2_RAW", "LOCAL_POSITION_NED", "SCALED_PRESSURE", "VFR_HUD", "SYS_STATUS")

# A receiver's fix report older than this is ignored when picking the best fix.
_GPS_FIX_STALE_S = 2.0


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
        mavlink.gps.lat              (float, deg)   — EKF position, from GLOBAL_POSITION_INT
        mavlink.gps.lon              (float, deg)
        mavlink.gps.alt              (float, m)     — EKF altitude AMSL (follows EKF2_HGT_REF)
        mavlink.gps.relative_alt     (float, m)     — above home, from LOCAL_POSITION_NED (-z)
        mavlink.gps.hdg              (float, deg)   — course over ground 0-360, from GPS_RAW_INT
        mavlink.gps.num_sv           (int)          — satellites used by the better of GPS_RAW_INT / GPS2_RAW
        mavlink.gps.fix_type         (int)          — 0=no GPS 1=no fix 2=2D 3=3D 4=DGPS 5/6=RTK, better of the two receivers
        mavlink.environment.press_abs    (float, hPa)  — from SCALED_PRESSURE
        mavlink.environment.press_diff   (float, hPa)
        mavlink.environment.temperature  (float, °C)   — centidegrees converted
        mavlink.environment.baro_alt     (float, m)    — from VFR_HUD
        mavlink.environment.climb        (float, m/s)
        mavlink.environment.airspeed     (float, m/s)
        mavlink.environment.groundspeed  (float, m/s)
        mavlink.battery.voltage          (float, V)    — from SYS_STATUS (mV converted; 0.0 if unknown)
        system.pixhawk_connected         (float, 0.0/1.0)
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
        self._nonfinite_warned: set[str] = set()
        # Latest (fix_type, num_sv, monotonic) per raw receiver message type
        self._gps_fix: dict[str, tuple[int, int, float]] = {}

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
                self.datastore.write("system.pixhawk_connected", 1.0)
                self._request_message_rates()
                return  # connected successfully
            except Exception as e:
                logger.debug(
                    "MavlinkTask: connection failed (%s) — retrying in %.0fs",
                    e,
                    self._connect_retry_s,
                )
                self.datastore.write("system.pixhawk_connected", 0.0)
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
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 20_000),   # 50 Hz 
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200_000),  #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,        200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW,           200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,    200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,            200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,         1_000_000), #  1 Hz
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
        try:
            msg = self._master.recv_match(type=list(_SUBSCRIBED_TYPES), blocking=True, timeout=self.period_s)
            if msg is not None:
                self._handle_message(msg)
                while True:
                    msg = self._master.recv_match(type=list(_SUBSCRIBED_TYPES), blocking=False)
                    if msg is None:
                        break
                    self._handle_message(msg)
        except Exception as e:
            logger.warning("MavlinkTask: serial error — reconnecting (%s)", e)
            self.datastore.write("system.pixhawk_connected", 0.0)
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
            self.setup()

    def _f(self, value: float, fallback: float = 0.0, field: str = "") -> float:
        """Return fallback if value is NaN or infinite, else value. Logs once per field."""
        if math.isfinite(value):
            return value
        if field and field not in self._nonfinite_warned:
            self._nonfinite_warned.add(field)
            logger.warning("MavlinkTask: %s is non-finite (%s) — writing %.1f instead", field, value, fallback)
        return fallback

    def _handle_message(self, msg) -> None:
        f = self._f
        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            self.datastore.write("mavlink.attitude.roll",       f(msg.roll,       field="mavlink.attitude.roll"))
            self.datastore.write("mavlink.attitude.pitch",      f(msg.pitch,      field="mavlink.attitude.pitch"))
            self.datastore.write("mavlink.attitude.yaw",        f(msg.yaw,        field="mavlink.attitude.yaw"))
            self.datastore.write("mavlink.attitude.rollspeed",  f(msg.rollspeed,  field="mavlink.attitude.rollspeed"))
            self.datastore.write("mavlink.attitude.pitchspeed", f(msg.pitchspeed, field="mavlink.attitude.pitchspeed"))
            self.datastore.write("mavlink.attitude.yawspeed",   f(msg.yawspeed,   field="mavlink.attitude.yawspeed"))

        elif msg_type == "ATTITUDE_QUATERNION":
            self.datastore.write("mavlink.quaternion.w", f(msg.q1, field="mavlink.quaternion.w"))
            self.datastore.write("mavlink.quaternion.x", f(msg.q2, field="mavlink.quaternion.x"))
            self.datastore.write("mavlink.quaternion.y", f(msg.q3, field="mavlink.quaternion.y"))
            self.datastore.write("mavlink.quaternion.z", f(msg.q4, field="mavlink.quaternion.z"))

        elif msg_type == "GLOBAL_POSITION_INT":
            # EKF position from the receiver PX4 has selected. lat/lon in 1e-7 deg, alt in mm AMSL.
            # alt is EKF height: set EKF2_HGT_REF = GPS so it does not follow the baro at altitude.
            self.datastore.write("mavlink.gps.lat", f(msg.lat / 1e7, field="mavlink.gps.lat"))
            self.datastore.write("mavlink.gps.lon", f(msg.lon / 1e7, field="mavlink.gps.lon"))
            self.datastore.write("mavlink.gps.alt", f(msg.alt / 1e3, field="mavlink.gps.alt"))

        elif msg_type in ("GPS_RAW_INT", "GPS2_RAW"):
            # Raw receivers (GPS1 / GPS2) supply fix quality only. Report the better of the two,
            # like PX4's own receiver selection, so the arm check matches what the EKF can use.
            now = time.monotonic()
            self._gps_fix[msg_type] = (int(msg.fix_type), int(msg.satellites_visible), now)
            fix_type, num_sv = max(
                (fix, sv) for fix, sv, t in self._gps_fix.values() if now - t < _GPS_FIX_STALE_S
            )
            self.datastore.write("mavlink.gps.fix_type", fix_type)
            self.datastore.write("mavlink.gps.num_sv", num_sv)
            if msg_type == "GPS_RAW_INT":
                # cog (course over ground) in cdeg
                self.datastore.write("mavlink.gps.hdg", f(msg.cog / 1e2, field="mavlink.gps.hdg"))

        elif msg_type == "LOCAL_POSITION_NED":
            # NED frame: z is positive downward, so relative_alt = -z
            self.datastore.write("mavlink.gps.relative_alt", f(-msg.z, field="mavlink.gps.relative_alt"))

        elif msg_type == "SCALED_PRESSURE":
            self.datastore.write("mavlink.environment.press_abs",   f(msg.press_abs, field="mavlink.environment.press_abs"))
            self.datastore.write("mavlink.environment.press_diff",  f(msg.press_diff, field="mavlink.environment.press_diff"))
            self.datastore.write("mavlink.environment.temperature", f(msg.temperature / 100.0, field="mavlink.environment.temperature"))

        elif msg_type == "VFR_HUD":
            self.datastore.write("mavlink.environment.baro_alt",    f(msg.alt, field="mavlink.environment.baro_alt"))
            self.datastore.write("mavlink.environment.climb",       f(msg.climb, field="mavlink.environment.climb"))
            self.datastore.write("mavlink.environment.airspeed",    f(msg.airspeed, field="mavlink.environment.airspeed"))
            self.datastore.write("mavlink.environment.groundspeed", f(msg.groundspeed, field="mavlink.environment.groundspeed"))

        elif msg_type == "SYS_STATUS":
            # voltage_battery in mV; UINT16_MAX (65535) means "not available"
            mv = msg.voltage_battery
            volts = mv / 1e3 if mv != 0xFFFF else 0.0
            self.datastore.write("mavlink.battery.voltage", f(volts, field="mavlink.battery.voltage"))

    def teardown(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
            logger.info("MavlinkTask: connection closed")
