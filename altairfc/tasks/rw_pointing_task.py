from __future__ import annotations

import logging
from config.settings import ControllerConfig, PointingConfig, SerialPortConfig
from controls.controller import Controller
from controls.pointing_modes import KEY_MODE, PointingMode
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
        pointing_config: PointingConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._bias_rpm = pointing_config.rw_bias_rpm
        self.rw = RWDriver(rw_port.port)
        self.rw_controller = Controller(rw_controller_config, period_s)

    def setup(self) -> None:
        self._mode = None # Re-enter the commanded mode after a restart

        if not self.rw.connect():
            self.datastore.write("system.rw_vesc_connected", 0.0)
            raise ConnectionError("RW ESC enabled but not connected")
        self.datastore.write("system.rw_vesc_connected", 1.0)

    def execute(self) -> None:
        self._store()

        mode = PointingMode(self.datastore.read(KEY_MODE, default=PointingMode.SAFE))
        if mode != self._mode:
            self._on_enter(mode)
            self._mode = mode

        if mode == PointingMode.SPINUP:
            self.rw.set_rpm(self._bias_rpm) # Spin up to the bias speed before pointing

        elif mode == PointingMode.POINTING:
            yaw = float(self.datastore.read("mavlink.attitude.yaw", default=0.0))
            yaw_rate = float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0))
            rw_rpm_cmd = self.rw_controller.output(yaw, yaw_rate) + self._bias_rpm
            self.rw.set_rpm(int(rw_rpm_cmd))

        else:
            self.rw.set_current(0) # Motor released

    def teardown(self) -> None:
        self.rw.close()

    def _on_enter(self, mode: PointingMode) -> None:
        self.rw_controller.reset_integrator()

    def _store(self) -> None:
        data = self.rw.read()
        self.datastore.write("system.rw_vesc_connected", 1.0 if self.rw.connected else 0.0)
        if data is not None:
            self._write("rw", data)
        elif self.rw.connected == False:
            raise ConnectionError("RW VESC disconnected during pointing task")

    def _write(self, prefix: str, data) -> None:
            for f in ('rpm', 'duty_cycle_now', 'avg_motor_current', 'avg_input_current',
                    'v_in', 'temp_fet', 'amp_hours', 'tachometer',
                    'tachometer_abs'):
                self.datastore.write(f"{prefix}.{f}", getattr(data, f, 0.0))
            fault = getattr(data, 'mc_fault_code', b'\x00')
            self.datastore.write(f"{prefix}.mc_fault_code", fault[0] if isinstance(fault, (bytes, bytearray)) else int(fault))