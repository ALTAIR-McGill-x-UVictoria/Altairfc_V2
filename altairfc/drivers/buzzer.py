from __future__ import annotations

# Buzzer on GPIO16 (pin 36), driven via pigpio software PWM.
# Each tune is a list of (frequency_hz, duration_s) pairs.
# frequency_hz = 0 means silence for that step.

GPIO_PIN = 16

# (freq_hz, duration_s)
Note = tuple[int, float]

# ---------------------------------------------------------------------------
# Note frequencies (Hz) — equal temperament, A4 = 440 Hz
# Octaves 3–7 cover the typical buzzer range (~130 Hz – 3950 Hz).
# Usage: C[5], Fs[4], Bb[6]  (s = sharp, b = flat)
# ---------------------------------------------------------------------------
C  = {3: 131, 4: 262, 5: 523,  6: 1047, 7: 2093}
Cs = {3: 139, 4: 277, 5: 554,  6: 1109, 7: 2217}
Db = Cs
D  = {3: 147, 4: 294, 5: 587,  6: 1175, 7: 2349}
Ds = {3: 156, 4: 311, 5: 622,  6: 1245, 7: 2489}
Eb = Ds
E  = {3: 165, 4: 330, 5: 659,  6: 1319, 7: 2637}
F  = {3: 175, 4: 349, 5: 698,  6: 1397, 7: 2794}
Fs = {3: 185, 4: 370, 5: 740,  6: 1480, 7: 2960}
Gb = Fs
G  = {3: 196, 4: 392, 5: 784,  6: 1568, 7: 3136}
Gs = {3: 208, 4: 415, 5: 831,  6: 1661, 7: 3322}
Ab = Gs
A  = {3: 220, 4: 440, 5: 880,  6: 1760, 7: 3520}
As = {3: 233, 4: 466, 5: 932,  6: 1865, 7: 3729}
Bb = As
B  = {3: 247, 4: 494, 5: 988,  6: 1976, 7: 3951}
REST = 0

# ---------------------------------------------------------------------------
# Tunes
# ---------------------------------------------------------------------------

# G major: G4=392→400 Hz, B4=494→500 Hz, D5=587→600 Hz (pigpio rounds cleanly)

d = 0.1

TUNE_PENDING: list[Note] = [
    (G[4], d),
    (REST, d),
    (G[4], d),
    (REST, d),
    (G[4], d),
    (REST, d),
    (G[4], d),
]

TUNE_SUCCESS: list[Note] = [
    (G[4], d),
    (REST, d),
    (B[4], d),
    (REST, d),
    (D[5], d),
    (REST, d),
    (G[5], d * 2),
]

TUNE_SUCCESS_REVERSE: list[Note] = list(reversed(TUNE_SUCCESS))

TUNE_PING: list[Note] = [
    (D[5], 0.06),
    (REST, 0.04),
    (G[5], 0.10),
]
