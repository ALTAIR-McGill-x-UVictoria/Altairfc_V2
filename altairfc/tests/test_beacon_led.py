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


def test_on_de_energizes_relay():
    """RELAY_CHANNEL is wired NC — confirmed on hardware 2026-08-03 — so on() must write code 0
    (de-energized, contact closed, beacon lit), not MAX_CODE."""
    beacon, board = _make_beacon()
    beacon.on()
    assert board.code(RELAY_CHANNEL) == 0
    assert beacon.lit is True


def test_off_energizes_relay():
    """off() must write MAX_CODE (energized, contact open, beacon dark) — the opposite of a
    typical NO-relay assumption."""
    beacon, board = _make_beacon()
    beacon.on()
    beacon.off()
    assert board.code(RELAY_CHANNEL) == MAX_CODE
    assert beacon.lit is False


def test_all_off_leaves_beacon_dark_not_lit():
    """Regression test: all_off() must energize the relay (MAX_CODE), not zero it — zeroing
    would actually turn the NC-wired beacon ON, the opposite of what "all off" should do."""
    beacon, board = _make_beacon()
    beacon.set_brightness(1500)
    beacon.on()

    beacon.all_off()

    assert board.code(BEACON_CHANNEL) == 0
    assert board.code(RELAY_CHANNEL) == MAX_CODE
    assert beacon.lit is False
