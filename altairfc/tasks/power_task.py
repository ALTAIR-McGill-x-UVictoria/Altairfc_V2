from __future__ import annotations

import logging

from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class PowerTask(BaseTask):
    """
    INA3221 three-channel power monitor task.

    CH1 → 24 V rail, CH2 → 12 V rail, CH3 → 5 V rail.

    DataStore keys written:
        power.voltage_24v  power.current_24v
        power.voltage_12v  power.current_12v
        power.voltage_5v   power.current_5v
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        i2c_dev: str = "/dev/i2c-1",
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._i2c_dev = i2c_dev

    def setup(self) -> None:
        self._ina = None
        try:
            from drivers.ina3221_driver import INA3221Driver
            self._ina = INA3221Driver(self._i2c_dev)
        except Exception as e:
            logger.error("PowerTask: failed to open INA3221 on %s: %s", self._i2c_dev, e)

    def execute(self) -> None:
        if self._ina is None:
            return
        reading = self._ina.read()
        if reading is None:
            return
        self.datastore.write("power.voltage_24v", float(reading.voltage_v[0]))
        self.datastore.write("power.current_24v", float(reading.current_a[0]))
        self.datastore.write("power.voltage_12v", float(reading.voltage_v[1]))
        self.datastore.write("power.current_12v", float(reading.current_a[1]))
        self.datastore.write("power.voltage_5v",  float(reading.voltage_v[2]))
        self.datastore.write("power.current_5v",  float(reading.current_a[2]))

    def teardown(self) -> None:
        if self._ina is not None:
            self._ina.close()
