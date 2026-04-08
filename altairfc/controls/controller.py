import numpy as np


class Controller:
    def __init__(self, Kp: float, Kd: float, Ki: float, max_value: float, dt: float):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.max_value = max_value
        self.dt = dt

        self.e_prev = 0.0
        self.e_int = 0.0

    def output(self, error: float, error_derivative: float | None = None):
        P = self.Kp * error
        D = self.Kd * (error - self.e_prev)/self.dt if error_derivative is None else self.Kd * error_derivative

        self.e_int += error * self.dt
        I = self.Ki * self.e_int
        self.e_prev = error

        output = P + D + I
        output = np.clip(output, -self.max_value, self.max_value)

        return output
