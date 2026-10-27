from __future__ import annotations

import logging
import time
from config.settings import ControllerConfig, PointingConfig, SerialPortConfig
from controls.controller import Controller
from controls.pointing_modes import KEY_MODE, PointingMode
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.mm_driver import MMDriver

logger = logging.getLogger(__name__)

class MMPointingTask(BaseTask):
    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        mm_port: SerialPortConfig,
        mm_controller_config: ControllerConfig,
        pointing_config: PointingConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._mm_pulse_length_s = pointing_config.mm_pulse_length_s
        self._mm_pulse_period_s = pointing_config.mm_pulse_period_s
        self._rw_bias_rpm = pointing_config.rw_bias_rpm
        self.mm = MMDriver(mm_port.port)
        self.mm_controller = Controller(mm_controller_config, self._mm_pulse_period_s)

    def setup(self) -> None:
        self._mode = None # Re-enter the commanded mode after a restart

        if not self.mm.connect():
            self.datastore.write("system.mm_vesc_connected", 0.0)
            raise ConnectionError("MM ESC enabled but not connected")
        self.datastore.write("system.mm_vesc_connected", 1.0)

    def execute(self) -> None:
        self._store()

        mode = PointingMode(self.datastore.read(KEY_MODE, default=PointingMode.SAFE))
        if mode != self._mode:
            self._on_enter(mode)
            self._mode = mode

        if mode == PointingMode.POINTING:
            self._pulse()

        else:
            self.mm.set_current(0) # Motor released

    def teardown(self) -> None:
        self.mm.close()

    def _on_enter(self, mode: PointingMode) -> None:
        self.mm_controller.reset_integrator()
        self._last_mm_pulse_started = 0.0
        self._mm_pulse_ends_at = 0.0

    def _pulse(self) -> None:
        # heading_error = float(self.datastore.read("mavlink.attitude.yaw", default=0.0))
        # error_rate = float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0))
        rw_rpm = float(self.datastore.read("rw.rpm", default=0.0))/7
        rw_rpm_error = rw_rpm - self._rw_bias_rpm
        mm_current_cmd = self.mm_controller.output(rw_rpm_error)
        self.datastore.write("pointing.mm_current_cmd", mm_current_cmd)
        now = time.monotonic()

        if now - self._last_mm_pulse_started >= self._mm_pulse_period_s:
            self._last_mm_pulse_started = now
            self._mm_pulse_ends_at = now + self._mm_pulse_length_s

        if now < self._mm_pulse_ends_at:
            self.mm.set_rpm(3000)
        else:
            self.mm.set_current(0)

    def _store(self) -> None:
        data = self.mm.read()
        self.datastore.write("system.mm_vesc_connected", 1.0 if self.mm.connected else 0.0)
        if data is not None:
            self._write("mm", data)
        elif self.mm.connected == False:
            raise ConnectionError("MM VESC disconnected during pointing task")

    def _write(self, prefix: str, data) -> None:
            for f in ('rpm', 'duty_now', 'current_motor', 'current_in',
                    'v_in', 'temp_pcb', 'amp_hours', 'tachometer',
                    'tachometer_abs'):
                self.datastore.write(f"{prefix}.{f}", getattr(data, f, 0.0))
            fault = getattr(data, 'mc_fault_code', b'\x00')
            self.datastore.write(f"{prefix}.mc_fault_code", fault[0] if isinstance(fault, (bytes, bytearray)) else int(fault))