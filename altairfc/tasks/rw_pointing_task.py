from __future__ import annotations

import logging
from config.settings import ControllerConfig, SerialPortConfig
from controls.controller import Controller
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.rw_driver import RWDriver

logger = logging.getLogger(__name__)

class RWPointingTask(BaseTask):
    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        rw_port: SerialPortConfig,
        rw_controller_config: ControllerConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self.rw = RWDriver(rw_port.port)
        self.rw_controller = Controller(rw_controller_config, period_s)

    def setup(self) -> None:
        if not self.rw.connect():
            self.datastore.write("system.rw_vesc_connected", 0.0)
            raise ConnectionError("RW ESC enabled but not connected")
        self.datastore.write("system.rw_vesc_connected", 1.0)

    def execute(self) -> None:
        self._store()

        if int(self.datastore.read("event.pointing_active", default=0.0)) != 1:
            self.rw.set_current(0) # Outside the pointing window: motor released
            return

        yaw = float(self.datastore.read("mavlink.attitude.yaw", default=0.0))
        yaw_rate = float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0))
        self.datastore.write("pointing.heading_error", yaw)
        rw_rpm_cmd = self.rw_controller.output(yaw, yaw_rate) + 1500
        self.rw.set_rpm(int(rw_rpm_cmd))

    def teardown(self) -> None:
        self.rw.close()

    def _store(self) -> None:
        data = self.rw.read()
        self.datastore.write("system.rw_vesc_connected", 1.0 if self.rw.connected else 0.0)
        if data is not None:
            self._write("rw", data)
        elif self.rw.connected == False:
            raise ConnectionError("RW VESC disconnected during pointing task")

    def _write(self, prefix: str, data) -> None:
            for f in ('rpm', 'duty_now', 'current_motor', 'current_in',
                    'v_in', 'temp_pcb', 'amp_hours', 'tachometer',
                    'tachometer_abs'):
                self.datastore.write(f"{prefix}.{f}", getattr(data, f, 0.0))
            fault = getattr(data, 'mc_fault_code', b'\x00')
            self.datastore.write(f"{prefix}.mc_fault_code", fault[0] if isinstance(fault, (bytes, bytearray)) else int(fault))
