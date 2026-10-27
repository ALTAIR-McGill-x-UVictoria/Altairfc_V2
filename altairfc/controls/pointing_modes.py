from __future__ import annotations

from enum import IntEnum

# Written only by PointingSupervisorTask; read by RWPointingTask, MMPointingTask and telemetry
KEY_MODE = "pointing.mode"


class PointingMode(IntEnum):
    SAFE = 0      # Both motors released
    SPINUP = 1    # RW spins up to rw_bias_rpm, MM released
    POINTING = 2  # RW points, MM dumps momentum
