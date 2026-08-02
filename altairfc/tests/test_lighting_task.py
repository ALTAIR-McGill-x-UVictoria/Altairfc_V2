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


def test_window_active_recurring_period():
    """period_s=60 recurs every minute, e.g. a 5s flash at :25."""
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0)
    assert not window_active(24.9, w)
    assert window_active(25.0, w)
    assert window_active(29.9, w)
    assert not window_active(30.0, w)
    assert window_active(85.0, w)      # wraps to the next minute


def test_seconds_to_window_recurring_period():
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0)
    assert seconds_to_window(30.0, w) == pytest.approx(55.0)
    assert seconds_to_window(20.0, w) == pytest.approx(5.0)
    assert seconds_to_window(25.0, w) == 0.0


def test_parse_windows_period_s():
    raw = [{"start_utc": "00:00:25", "duration_s": 5.0, "period_s": 60.0, "label": "beacon_25"}]
    windows = parse_windows(raw)
    assert windows[0].period_s == 60.0


def test_parse_windows_default_period_is_one_day():
    raw = [{"start_utc": "00:00:00", "duration_s": 1.0}]
    windows = parse_windows(raw)
    assert windows[0].period_s == 86400.0


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
    """Mimics drivers.led_board.LedBoard's interlock (sphere vs. relay) for two fake channel objects."""

    def __init__(self) -> None:
        self.sphere_code = 0
        self.relay_code = 0


class _FakeSphere:
    """Mimics SphereLedSource's hold_current()/update() current-loop interface."""

    def __init__(self, pair: _FakeInterlockedPair) -> None:
        self._pair = pair
        self.target_current_a: float | None = None

    def hold_current(self, target_a: float) -> None:
        self.target_current_a = target_a

    def update(self) -> None:
        if self._pair.relay_code > 0:
            raise InterlockViolation("beacon relay on")
        self._pair.sphere_code = 1   # stand-in for "DAC is now driving toward target"

    def all_off(self) -> None:
        self.target_current_a = None
        self._pair.sphere_code = 0


class _FakeBeacon:
    """Mimics BeaconChannel's brightness (ch1, not interlocked) + on()/off() (ch3 relay, interlocked)."""

    def __init__(self, pair: _FakeInterlockedPair) -> None:
        self._pair = pair
        self.brightness_code = 0

    def set_brightness(self, code: int) -> None:
        self.brightness_code = code

    def on(self) -> None:
        if self._pair.sphere_code > 0:
            raise InterlockViolation("sphere on")
        self._pair.relay_code = 1

    def off(self) -> None:
        self._pair.relay_code = 0

    def all_off(self) -> None:
        self.brightness_code = 0
        self._pair.relay_code = 0


def _make_task(**kwargs) -> tuple[LightingTask, _FakeInterlockedPair]:
    task = LightingTask("lighting", 0.2, DataStore(), beacon_windows=[], **kwargs)
    pair = _FakeInterlockedPair()
    task._board = object()  # only needs to be non-None; _apply() doesn't touch it directly
    task._sphere = _FakeSphere(pair)
    task._beacon = _FakeBeacon(pair)
    return task, pair


def test_apply_turns_sphere_on_when_observation_active():
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)

    task._apply(True, None)

    assert task._sphere_on is True
    assert task._beacon_on is False
    assert task._sphere.target_current_a == pytest.approx(0.2657)
    assert pair.sphere_code != 0
    assert pair.relay_code == 0


def test_apply_latches_sphere_on_even_after_observation_goes_inactive():
    """Per project decision: no internal apogee check — once on, only task teardown turns it off."""
    task, pair = _make_task(sphere_target_current_a=0.2657)

    task._apply(True, None)
    assert task._sphere_on is True

    task._apply(False, None)
    assert task._sphere_on is True
    assert pair.sphere_code != 0


def test_apply_strobes_beacon_in_window_before_observation_starts():
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0, label="beacon_25")

    task._apply(False, w)

    assert task._sphere_on is False
    assert task._beacon_on is True
    assert pair.relay_code == 1


def test_apply_leaves_beacon_off_outside_window():
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)

    task._apply(False, None)

    assert task._beacon_on is False
    assert pair.relay_code == 0


def test_apply_leaves_sphere_off_when_no_target_current_configured():
    """Per project decision: an unconfigured sphere_target_current_a must never guess a value."""
    task, pair = _make_task(sphere_target_current_a=None, beacon_dac_code=777)
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0)

    task._apply(True, w)

    assert task._sphere_on is False
    assert task._beacon_on is True
    assert pair.sphere_code == 0


def test_apply_forces_beacon_off_once_sphere_engages():
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0)

    task._apply(False, w)
    assert task._beacon_on is True

    task._apply(True, None)
    assert task._sphere_on is True
    assert task._beacon_on is False
    assert pair.relay_code == 0


def test_teardown_zeroes_sphere_and_beacon_including_relay():
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)
    task._apply(True, None)
    assert pair.sphere_code != 0

    task._board = None  # avoid LedBoard.close() touching the fake board object
    task.teardown()

    assert pair.sphere_code == 0
    assert pair.relay_code == 0
    assert task._beacon.brightness_code == 0
