from __future__ import annotations

# Buzzer on GPIO16 (pin 36), driven via software PWM.
# Each tune is a list of (frequency_hz, duration_s) pairs.
# frequency_hz = 0 means silence for that step.

GPIO_PIN = 16

# (freq_hz, duration_s)
Note = tuple[int, float]

TUNE_PENDING: list[Note] = [
    (440, 0.15),
    (0,   0.10),
    (440, 0.15),
    (0,   0.10),
    (440, 0.15),
]

TUNE_SUCCESS: list[Note] = [
    (523, 0.12),  # C5
    (659, 0.12),  # E5
    (784, 0.12),  # G5
    (1047, 0.25), # C6
]

TUNE_SUCCESS_REVERSE: list[Note] = list(reversed(TUNE_SUCCESS))

TUNE_PING: list[Note] = [
    (880, 0.06),
    (0,   0.04),
    (1174, 0.10),
]
