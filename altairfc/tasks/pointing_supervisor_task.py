from __future__ import annotations

import logging
import time
from config.settings import PointingConfig
from controls.pointing_modes import KEY_MODE, PointingMode
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

class PointingSupervisorTask(BaseTask):
    """
    Owns the pointing mode shared by RWPointingTask and MMPointingTask.

    Only this task changes pointing.mode; the two motor tasks read it every cycle
    and run their own behaviour for it. No hardware I/O, so serial delays on
    either VESC never hold up a mode decision.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        pointing_config: PointingConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._spinup_s = pointing_config.spinup_s

    def setup(self) -> None:
        self._mode = PointingMode.SAFE
        self._mode_entered_at = time.monotonic()
        self._held_since: dict[str, float] = {}

    def execute(self) -> None:
        now = time.monotonic()

        active = int(self.datastore.read("event.pointing_active", default=0.0)) == 1
        # Default 1.0: a motor task disabled in config never writes its flag
        rw_connected = float(self.datastore.read("system.rw_vesc_connected", default=1.0)) >= 1.0
        mm_connected = float(self.datastore.read("system.mm_vesc_connected", default=1.0)) >= 1.0

        # The MM control law uses rw.rpm, which freezes if the RW drops out
        if not (active and rw_connected and mm_connected):
            new_mode = PointingMode.SAFE
        else:
            new_mode = self._transition(now)

        if new_mode != self._mode:
            logger.info("PointingSupervisor: mode %s -> %s", self._mode.name, new_mode.name)
            self._mode = new_mode
            self._mode_entered_at = now
            self._held_since.clear()

        self.datastore.write(KEY_MODE, int(self._mode))

    def _transition(self, now: float) -> PointingMode:
        mode = self._mode
        time_in_mode = now - self._mode_entered_at

        if mode == PointingMode.SAFE:
            return PointingMode.SPINUP

        elif mode == PointingMode.SPINUP:
            if time_in_mode >= self._spinup_s:
                return PointingMode.POINTING

        return mode

    def _held(self, name: str, condition: bool, duration_s: float, now: float) -> bool:
        """True once condition has been continuously true for duration_s. Cleared on every mode change."""
        if not condition:
            self._held_since.pop(name, None)
            return False
        since = self._held_since.setdefault(name, now)
        return now - since >= duration_s
