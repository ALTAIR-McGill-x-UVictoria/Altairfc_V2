from __future__ import annotations

import logging
import numpy as np
from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.vesc_interface import VESCObject
from controls.error_computation import compute_error
from controls.controller import Controller

logger = logging.getLogger(__name__)


class VescTask(BaseTask):

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        port_config: SerialPortConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._vesc_port = port_config.port

    def setup(self) -> None:
        try:
            self.vesc = VESCObject(self._vesc_port)
            logger.info("Initialized VESC motor interface on port %s", self._vesc_port)

        except Exception as e:
            logger.error("Failed to initialize VESC motor interface on port %s: %s", self._vesc_port, e)

    def execute(self) -> None:
        data = self.vesc.get_data()
        if data:
            # Write all GetValues fields into the datastore under the 'vesc.' namespace
            fields = [
                'temp_mos1', 'temp_mos2', 'temp_mos3', 'temp_mos4', 'temp_mos5', 'temp_mos6',
                'temp_pcb', 'current_motor', 'current_in', 'duty_now', 'rpm', 'v_in',
                'amp_hours', 'amp_hours_charged', 'watt_hours', 'watt_hours_charged',
                'tachometer', 'tachometer_abs', 'mc_fault_code',
            ]
            for f in fields:
                try:
                    val = getattr(data, f, None)
                except Exception:
                    val = None
                self.datastore.write(f"vesc.{f}", val)


    def teardown(self) -> None:
        # Close the serial port if it was opened by VESCObject
        try:
            if hasattr(self, 'vesc') and self.vesc is not None:
                port = getattr(self.vesc, 'port', None)
                if port is not None:
                    try:
                        port.close()
                        logger.info("Closed VESC serial port %s", getattr(port, 'name', ''))
                    except Exception:
                        logger.exception("Failed to close VESC serial port")
                self.vesc = None
        except Exception:
            logger.exception("Error during VescTask.teardown()")

