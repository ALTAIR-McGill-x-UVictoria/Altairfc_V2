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
