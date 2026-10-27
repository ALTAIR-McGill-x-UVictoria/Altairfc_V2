from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from config.settings import ModulatorConfig

RAD_S_TO_RPM = 30.0 / np.pi


class Modulator:
    """
    Pulse width modulator for the MM (pivot) motor, ported from HAB_ADCS_SIM (sim_tools/pwmodulator.py).

    Integrates the predicted RW speed offset from the bias and fires a fixed-current pulse
    once the accumulator crosses the threshold. The pulse width scales with the offset.
    After each pulse the motor is released for settle_time with the accumulator frozen.
    """

    def __init__(self, config: ModulatorConfig, reference_rpm: float, period: float):
        self.pulse_current = config.pulse_current
        self.settle_time = config.settle_time
        self.integration_threshold = config.integration_threshold
        self.deadband = config.deadband  # normalized offset
        self.loop_gain = config.loop_gain
        self.reference_speed = reference_rpm
        self.max_offset = config.rw_max_offset_rpm
        self.dt = period
        self.lookahead = config.lookahead_s
        self.wheel_inertia = config.rw_load_inertia + config.rw_rotor_inertia
        self.wheel_torque_constant = config.rw_torque_constant
        self.wheel_friction = config.rw_viscous_friction_coeff
        self.time_constant = (self.wheel_inertia * (self.max_offset / RAD_S_TO_RPM)) / (config.mm_torque_constant * self.pulse_current)
        self.reset()

    def modulate(self, signal: float, current: float, t: float) -> float:
        """Returns the MM current command (A).

        signal is the measured wheel speed (rpm) and current the wheel motor current (A).
        Pulsing is handled internally.
        """
        if self.pulse_start is not None:
            elapsed = t - self.pulse_start
            if elapsed < self.pulse_duration:
                return self.direction * self.pulse_current
            if elapsed < self.pulse_duration + self.settle_time:
                return 0.0  # settling, accumulator frozen
            self.pulse_start = None
            self.accumulator = 0.0

        offset = (signal - self.reference_speed) / self.max_offset
        accel = (self.wheel_torque_constant * current - self.wheel_friction * signal / RAD_S_TO_RPM) / (self.wheel_inertia * self.max_offset / RAD_S_TO_RPM)
        predicted = offset + self.lookahead * accel
        self.accumulator += np.sign(predicted) * max(abs(predicted) - self.deadband / 2.0, 0.0) * self.dt
        if (abs(self.accumulator) >= self.integration_threshold and np.sign(predicted) == np.sign(self.accumulator)
                and abs(predicted) > self.deadband / 2.0):
            impulse_bit = min(max(self.loop_gain * abs(predicted), self.deadband), self.loop_gain)
            self.pulse_duration = impulse_bit * self.time_constant
            self.direction = np.sign(predicted)
            self.pulse_start = t
            return self.direction * self.pulse_current
        return 0.0

    def reset(self) -> None:
        self.accumulator = 0.0
        self.pulse_duration = 0.0
        self.pulse_start = None
        self.direction = 0.0
