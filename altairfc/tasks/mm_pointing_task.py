from __future__ import annotations

import logging
import time
from config.settings import ModulatorConfig, PointingConfig, SerialPortConfig
from controls.modulator import Modulator
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
        modulator_config: ModulatorConfig,
        pointing_config: PointingConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self.mm = MMDriver(mm_port.port)
        self.modulator = Modulator(modulator_config, pointing_config.rw_bias_rpm, period_s)

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
        self.modulator.reset()

    def _pulse(self) -> None:
        rw_rpm = float(self.datastore.read("rw.rpm", default=0.0))/7
        rw_current = float(self.datastore.read("rw.current_motor", default=0.0))
        mm_current_cmd = self.modulator.modulate(rw_rpm, rw_current, time.monotonic())
        self.datastore.write("pointing.mm_current_cmd", mm_current_cmd)
        self.mm.set_current(int(mm_current_cmd * 1000)) # VESC takes mA

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