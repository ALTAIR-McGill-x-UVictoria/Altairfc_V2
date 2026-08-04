from __future__ import annotations

from drivers.beacon_led import BeaconChannel
from drivers.led_board import BEACON_CHANNEL, RELAY_CHANNEL, LedBoard
from drivers.mcp4728_driver import MAX_CODE


class _FakeDac:
    def __init__(self) -> None:
        self.writes: list[list[int]] = []

    def set_vdd_reference(self, codes) -> bool:
        self.writes.append(list(codes))
        return True

    def close(self) -> None:
        pass


def _make_beacon() -> tuple[BeaconChannel, LedBoard]:
    dac = _FakeDac()
    board = LedBoard(bus=None, dac=dac, ads1115=object(), use_ldac=False)
    return BeaconChannel(board=board), board


def test_on_energizes_relay():
    """RELAY_CHANNEL is wired NO — confirmed on hardware 2026-08-03 — so on() must energize
    it (MAX_CODE, contact closed, beacon lit)."""
    beacon, board = _make_beacon()
    beacon.on()
    assert board.code(RELAY_CHANNEL) == MAX_CODE
    assert beacon.lit is True


def test_off_de_energizes_relay():
    beacon, board = _make_beacon()
    beacon.on()
    beacon.off()
    assert board.code(RELAY_CHANNEL) == 0
    assert beacon.lit is False


def test_all_off_leaves_beacon_dark():
    beacon, board = _make_beacon()
    beacon.set_brightness(1500)
    beacon.on()

    beacon.all_off()

    assert board.code(BEACON_CHANNEL) == 0
    assert board.code(RELAY_CHANNEL) == 0
    assert beacon.lit is False


def test_set_brightness_before_on_does_not_energize_channel_1():
    """Brightness set while dark must stay staged, not applied to channel 1 -- on() is what
    decides when current is allowed to flow, after the relay contact has closed."""
    beacon, board = _make_beacon()
    beacon.set_brightness(1500)
    assert board.code(BEACON_CHANNEL) == 0


def test_on_closes_relay_before_ramping_brightness_up():
    """The relay must close onto a currentless circuit -- channel 1 must be 0 at the moment
    channel 3 energizes, and only ramp up afterward, never jump straight to the setpoint."""
    dac = _FakeDac()
    board = LedBoard(bus=None, dac=dac, ads1115=object(), use_ldac=False)
    beacon = BeaconChannel(board=board)
    beacon.set_brightness(1500)

    n = len(dac.writes)
    beacon.on()
    seq = dac.writes[n:]

    relay_idx = next(i for i, w in enumerate(seq) if w[RELAY_CHANNEL] == MAX_CODE)
    assert seq[relay_idx][BEACON_CHANNEL] == 0

    ramp = [w[BEACON_CHANNEL] for w in seq[relay_idx:]]
    assert ramp == sorted(ramp)
    assert ramp[-1] == 1500


def test_off_ramps_brightness_down_before_opening_relay():
    """The relay must break a currentless circuit too -- channel 1 must reach 0 before
    channel 3 de-energizes, not the other way around."""
    dac = _FakeDac()
    board = LedBoard(bus=None, dac=dac, ads1115=object(), use_ldac=False)
    beacon = BeaconChannel(board=board)
    beacon.set_brightness(1500)
    beacon.on()

    n = len(dac.writes)
    beacon.off()
    seq = dac.writes[n:]

    relay_off_idx = next(i for i, w in enumerate(seq) if w[RELAY_CHANNEL] == 0)
    assert seq[relay_off_idx][BEACON_CHANNEL] == 0

    ramp = [w[BEACON_CHANNEL] for w in seq[: relay_off_idx + 1]]
    assert ramp == sorted(ramp, reverse=True)
