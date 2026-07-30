from __future__ import annotations

import pytest

from core.datastore import DataStore
from drivers.led_board import InterlockViolation
from tasks.lighting_task import (
    ImagingWindow,
    LightingTask,
    parse_windows,
    seconds_to_window,
    window_active,
)


# ---------------------------------------------------------------------------
# Pure window-matching functions
# ---------------------------------------------------------------------------

def test_window_active_simple():
    w = ImagingWindow(start_s=100.0, duration_s=10.0)
    assert not window_active(50.0, w)
    assert window_active(100.0, w)
    assert window_active(105.0, w)
    assert not window_active(110.0, w)


def test_window_active_midnight_wraparound():
    w = ImagingWindow(start_s=86395.0, duration_s=10.0)  # 23:59:55 -> 00:00:05 next day
    assert window_active(86398.0, w)   # 23:59:58
    assert window_active(2.0, w)       # 00:00:02
    assert not window_active(6.0, w)   # 00:00:06 -- past the window
    assert not window_active(86000.0, w)


def test_seconds_to_window():
    w = ImagingWindow(start_s=100.0, duration_s=10.0)
    assert seconds_to_window(0.0, w) == pytest.approx(100.0)
    assert seconds_to_window(105.0, w) == 0.0
    assert seconds_to_window(90.0, w) == pytest.approx(10.0)


def test_seconds_to_window_midnight_wraparound():
    w = ImagingWindow(start_s=86395.0, duration_s=10.0)
    assert seconds_to_window(86000.0, w) == pytest.approx(395.0)


def test_parse_windows():
    raw = [{"start_utc": "01:02:03", "duration_s": 5.0, "label": "a"}]
    windows = parse_windows(raw)
    assert windows[0].start_s == 1 * 3600 + 2 * 60 + 3
    assert windows[0].duration_s == 5.0
    assert windows[0].label == "a"


def test_parse_windows_default_label():
    raw = [{"start_utc": "00:00:00", "duration_s": 1.0}]
    windows = parse_windows(raw)
    assert windows[0].label == "window_0"


# ---------------------------------------------------------------------------
# LightingTask actuation logic (hardware replaced with fakes)
# ---------------------------------------------------------------------------

class _FakeInterlockedPair:
    """Mimics drivers.led_board.LedBoard's interlock for two fake channel objects."""

    def __init__(self) -> None:
        self.sphere_code = 0
        self.beacon_code = 0


class _FakeSphere:
    def __init__(self, pair: _FakeInterlockedPair) -> None:
        self._pair = pair

    def set_code(self, code: int) -> None:
        if self._pair.beacon_code > 0:
            raise InterlockViolation("beacon on")
        self._pair.sphere_code = code

    def all_off(self) -> None:
        self._pair.sphere_code = 0


class _FakeBeacon:
    def __init__(self, pair: _FakeInterlockedPair) -> None:
        self._pair = pair

    def set_code(self, code: int) -> None:
        if self._pair.sphere_code > 0:
            raise InterlockViolation("sphere on")
        self._pair.beacon_code = code

    def all_off(self) -> None:
        self._pair.beacon_code = 0


def _make_task(**kwargs) -> tuple[LightingTask, _FakeInterlockedPair]:
    task = LightingTask("lighting", 0.2, DataStore(), windows=[], **kwargs)
    pair = _FakeInterlockedPair()
    task._board = object()  # only needs to be non-None; _apply() doesn't touch it directly
    task._sphere = _FakeSphere(pair)
    task._beacon = _FakeBeacon(pair)
    return task, pair


def test_apply_turns_sphere_on_and_forces_beacon_off_during_window():
    task, pair = _make_task(sphere_dac_code=1000, beacon_dac_code=777)
    w = ImagingWindow(start_s=0.0, duration_s=10.0, label="w")

    task._apply(w)

    assert task._sphere_on is True
    assert task._beacon_on is False
    assert pair.sphere_code == 1000
    assert pair.beacon_code == 0


def test_apply_resumes_beacon_strobe_outside_window():
    task, pair = _make_task(sphere_dac_code=1000, beacon_dac_code=777)

    task._apply(None)

    assert task._sphere_on is False
    assert task._beacon_on is True
    assert pair.beacon_code == 777


def test_apply_leaves_sphere_off_when_no_dac_code_configured():
    """Per project decision: an unconfigured sphere_dac_code must never guess a value."""
    task, pair = _make_task(sphere_dac_code=None, beacon_dac_code=777)
    w = ImagingWindow(start_s=0.0, duration_s=10.0, label="w")

    task._apply(w)

    assert task._sphere_on is False
    assert task._beacon_on is True
    assert pair.sphere_code == 0


def test_apply_turns_sphere_off_when_window_ends():
    task, pair = _make_task(sphere_dac_code=1000)
    w = ImagingWindow(start_s=0.0, duration_s=10.0, label="w")

    task._apply(w)
    assert task._sphere_on is True

    task._apply(None)
    assert task._sphere_on is False
    assert pair.sphere_code == 0


def test_strobe_toggles_after_configured_durations():
    task, pair = _make_task(beacon_dac_code=42, beacon_on_s=1.0, beacon_off_s=2.0)

    task._advance_strobe(0.0)
    assert task._strobe_on is True
    assert pair.beacon_code == 42

    task._advance_strobe(0.5)
    assert task._strobe_on is True  # still inside the on-phase

    task._advance_strobe(1.5)
    assert task._strobe_on is False  # on-phase (1.0s) elapsed
    assert pair.beacon_code == 0

    task._advance_strobe(3.0)
    assert task._strobe_on is False  # still inside the 2.0s off-phase

    task._advance_strobe(3.6)
    assert task._strobe_on is True  # off-phase elapsed, back on
