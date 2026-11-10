from __future__ import annotations

import math
import sys
import types

import pymap3d as pm
import pytest

try:
    import drivers.vesc_interface  # noqa: F401
except ImportError:  # pyvesc/PyCRC only installed on the Pi; the RW is faked below anyway
    _stub = types.ModuleType("drivers.vesc_interface")
    _stub.VESCObject = object
    sys.modules["drivers.vesc_interface"] = _stub

from config.settings import ControllerConfig, GroundStationConfig, PointingConfig, SerialPortConfig
from controls.pointing_modes import KEY_MODE, PointingMode
from core.datastore import DataStore
from tasks.rw_pointing_task import RWPointingTask

BALLOON = (45.5048, -73.5772, 1060.0)
BIAS_RPM = 1500.0
KP = 100.0


class _FakeRW:
    connected = True

    def __init__(self) -> None:
        self.rpm_cmds: list[int] = []

    def read(self):
        return None

    def set_rpm(self, rpm) -> None:
        self.rpm_cmds.append(rpm)

    def set_current(self, current) -> None:
        pass


def _run_pointing_tick(gs_ned: tuple[float, float, float], yaw: float) -> RWPointingTask:
    """One POINTING tick with the GS at gs_ned metres (N, E, D) from BALLOON and a level
    payload with its nose at `yaw`."""
    gs = pm.ned2geodetic(*gs_ned, *BALLOON)
    task = RWPointingTask(
        "rw_pointing", 0.05, DataStore(),
        rw_port=SerialPortConfig(port="unused", baud=115200),
        rw_controller_config=ControllerConfig(Kp=KP, Ki=0.0, Kd=0.0, max=1e9, min=-1e9),
        pointing_config=PointingConfig(rw_bias_rpm=BIAS_RPM),
        ground_station=GroundStationConfig(latitude=gs[0], longitude=gs[1], altitude=gs[2]),
    )
    task.rw = _FakeRW()
    task._mode = None

    ds = task.datastore
    ds.write(KEY_MODE, int(PointingMode.POINTING))
    ds.write("mavlink.gps.lat", BALLOON[0])
    ds.write("mavlink.gps.lon", BALLOON[1])
    ds.write("mavlink.gps.alt", BALLOON[2])
    ds.write("mavlink.attitude.yawspeed", 0.0)
    # body->NED quaternion, [x, y, z, w]
    ds.write("mavlink.quaternion.x", 0.0)
    ds.write("mavlink.quaternion.y", 0.0)
    ds.write("mavlink.quaternion.z", math.sin(yaw / 2))
    ds.write("mavlink.quaternion.w", math.cos(yaw / 2))

    task.execute()
    return task


def test_gs_due_east_nose_north():
    """GS 90 deg right of the nose: az_err = +pi/2, controller sees -pi/2."""
    task = _run_pointing_tick(gs_ned=(0.0, 1000.0, 1000.0), yaw=0.0)

    assert task.datastore.read("pointing.az_error") == pytest.approx(math.pi / 2)
    assert task.rw.rpm_cmds == [int(KP * -math.pi / 2 + BIAS_RPM)]


def test_gs_due_north_matches_old_point_north_error():
    """With the GS due north the controller input equals yaw, as it did before."""
    yaw = math.radians(30.0)
    task = _run_pointing_tick(gs_ned=(1000.0, 0.0, 1000.0), yaw=yaw)

    assert task.rw.rpm_cmds == [int(KP * yaw + BIAS_RPM)]
