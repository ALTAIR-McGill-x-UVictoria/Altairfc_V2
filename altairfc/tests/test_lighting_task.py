from __future__ import annotations

import sys
import types

import pytest

from core.datastore import DataStore
from drivers.sphere_led import LedState
from tasks.lighting_task import (
    ImagingWindow,
    LightingTask,
    flash_state,
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


def test_flash_state_50_percent_duty_cycle():
    """At 2 Hz (period 0.5s), on for the first 0.25s of every 0.5s, off for the rest."""
    assert flash_state(0.0, 2.0) is True
    assert flash_state(0.24, 2.0) is True
    assert flash_state(0.25, 2.0) is False
    assert flash_state(0.49, 2.0) is False
    assert flash_state(0.5, 2.0) is True   # next cycle


def test_flash_state_different_rates():
    assert flash_state(0.9, 1.0) is False   # 1 Hz: on [0, 0.5), off [0.5, 1.0)
    assert flash_state(1.0, 1.0) is True    # wraps to the next 1s cycle
    assert flash_state(0.05, 10.0) is False  # 10 Hz: on [0, 0.05), off [0.05, 0.1)


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

class _FakeChannelState:
    """Tracks the two fake channel objects' codes -- sphere and beacon relay are independent,
    no interlock between them (see drivers/led_board.py's module docstring)."""

    def __init__(self) -> None:
        self.sphere_code = 0
        self.relay_code = 0


class _FakeSphere:
    """Mimics SphereLedSource's hold_current()/update() current-loop interface."""

    def __init__(self, pair: _FakeChannelState) -> None:
        self._pair = pair
        self.target_current_a: float | None = None
        self.kp: float | None = None
        self.ki: float | None = None

    def hold_current(self, target_a: float, *, kp: float | None = None, ki: float | None = None) -> None:
        self.target_current_a = target_a
        self.kp = kp
        self.ki = ki

    def update(self) -> LedState:
        self._pair.sphere_code = 1   # stand-in for "DAC is now driving toward target"
        return LedState(
            code=self._pair.sphere_code,
            current_a=self.target_current_a or 0.0,
            temperature_c=20.0,
            target_current_a=self.target_current_a,
            settled=True,
        )

    def all_off(self) -> None:
        self.target_current_a = None
        self._pair.sphere_code = 0

    @property
    def code(self) -> int:
        return self._pair.sphere_code

    def read_current_a(self) -> float:
        return self.target_current_a or 0.0

    def read_bridge_temperature_c(self) -> float:
        return 20.0


class _FakeBeacon:
    """Mimics BeaconChannel's brightness (ch1) + on()/off() (ch3 relay) -- independent of the sphere."""

    def __init__(self, pair: _FakeChannelState) -> None:
        self._pair = pair
        self.brightness_code = 0

    def set_brightness(self, code: int) -> None:
        self.brightness_code = code

    def on(self) -> None:
        self._pair.relay_code = 1

    def off(self) -> None:
        self._pair.relay_code = 0

    def all_off(self) -> None:
        self.brightness_code = 0
        self._pair.relay_code = 0

    @property
    def code(self) -> int:
        return self.brightness_code

    def read_current_a(self) -> float:
        return 0.0


def _make_task(**kwargs) -> tuple[LightingTask, _FakeChannelState]:
    task = LightingTask("lighting", 0.2, DataStore(), beacon_windows=[], **kwargs)
    pair = _FakeChannelState()
    task._board = object()  # only needs to be non-None; _apply() doesn't touch it directly
    task._sphere = _FakeSphere(pair)
    task._beacon = _FakeBeacon(pair)
    return task, pair


def test_apply_turns_sphere_on_immediately():
    """Sphere engages on the very first _apply() call whenever sphere_target_current_a is
    configured — no longer gated on observation_active/event.ascent_active."""
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)

    task._apply(True, None, 0.0)  # no active beacon window

    assert task._sphere_on is True
    assert task._beacon_on is False
    assert task._sphere.target_current_a == pytest.approx(0.2657)
    assert pair.sphere_code != 0
    assert pair.relay_code == 0


def test_apply_latches_sphere_on_even_after_observation_goes_inactive():
    """Per project decision: no internal apogee check — once on, only task teardown turns it off."""
    task, pair = _make_task(sphere_target_current_a=0.2657)

    task._apply(True, None, 0.0)
    assert task._sphere_on is True

    task._apply(False, None, 0.0)
    assert task._sphere_on is True
    assert pair.sphere_code != 0


def test_apply_beacon_strobes_during_window_even_while_sphere_is_on():
    """No interlock: the beacon flashes on its schedule regardless of the sphere's state. Safe
    by construction because beacon_windows are defined as the non-observation windows — see
    LightingTask's class docstring. beacon_flash_hz defaults to 2.0 (period 0.5s, on for the
    first 0.25s of each cycle) — now_s=25.0 lands exactly on a cycle boundary, so it's on."""
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0, label="beacon_25")

    task._apply(True, None, 0.0)  # sphere latches on, no window yet
    assert task._sphere_on is True
    assert task._beacon_on is False

    task._apply(True, w, 25.0)  # beacon window now active — sphere stays on throughout
    assert task._sphere_on is True
    assert task._beacon_on is True
    assert pair.sphere_code != 0
    assert pair.relay_code == 1


def test_apply_beacon_toggles_off_between_flashes_within_an_active_window():
    """The window being active is necessary but not sufficient for the beacon to be on — it
    also has to be in the "on" half of the current flash cycle. now_s=25.3 is still inside the
    5s window (25-30) but past the 0.25s on-phase of the 0.5s (2 Hz) flash cycle."""
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0, label="beacon_25")

    task._apply(True, w, 25.3)

    assert task._beacon_on is False
    assert pair.relay_code == 0


def test_apply_leaves_beacon_off_outside_window():
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)

    task._apply(False, None, 0.0)

    assert task._beacon_on is False
    assert pair.relay_code == 0


def test_apply_leaves_sphere_off_when_no_target_current_configured():
    """Per project decision: an unconfigured sphere_target_current_a must never guess a value.
    The beacon still strobes on schedule regardless."""
    task, pair = _make_task(sphere_target_current_a=None, beacon_dac_code=777)
    w = ImagingWindow(start_s=25.0, duration_s=5.0, period_s=60.0)

    task._apply(True, w, 25.0)

    assert task._sphere_on is False
    assert task._beacon_on is True
    assert pair.sphere_code == 0


def test_teardown_zeroes_sphere_and_beacon_including_relay():
    task, pair = _make_task(sphere_target_current_a=0.2657, beacon_dac_code=777)
    task._apply(True, None, 0.0)
    assert pair.sphere_code != 0

    task._board = None  # avoid LedBoard.close() touching the fake board object
    task.teardown()

    assert pair.sphere_code == 0
    assert pair.relay_code == 0
    assert task._beacon.brightness_code == 0


def test_setup_resets_sphere_on_and_beacon_on_across_restarts(monkeypatch):
    """Regression test: core/task_base.py restarts a task (calls setup() again) after execute()
    raises (e.g. a transient I2C fault). setup() builds a brand-new SphereLedSource/BeaconChannel
    every time — if _sphere_on/_beacon_on weren't reset here too, the one-shot latch in _apply()
    would see _sphere_on already True from before the restart and never call hold_current() on
    the new object, leaving it silently spinning forever with no target (code stuck at 0,
    target_current_a stuck at None) while looking, from the loop-rate log alone, like a healthy
    running loop."""
    import tasks.lighting_task as lighting_task_module

    monkeypatch.setattr(lighting_task_module, "LedBoard", lambda **kwargs: object())
    monkeypatch.setattr(lighting_task_module, "SphereLedSource", lambda **kwargs: object())

    class _StubBeacon:
        def __init__(self, **kwargs) -> None:
            pass

        def set_brightness(self, code: int) -> None:
            pass

    monkeypatch.setattr(lighting_task_module, "BeaconChannel", _StubBeacon)

    fake_smbus2 = types.ModuleType("smbus2")
    fake_smbus2.SMBus = lambda *a, **k: object()
    monkeypatch.setitem(sys.modules, "smbus2", fake_smbus2)

    task = LightingTask("lighting", 0.2, DataStore(), beacon_windows=[], sphere_target_current_a=0.2657)
    task.setup()
    task._sphere_on = True   # simulate a prior run having already latched the sphere on
    task._beacon_on = True

    task.setup()  # simulates the task restarting after execute() raised

    assert task._sphere_on is False
    assert task._beacon_on is False
