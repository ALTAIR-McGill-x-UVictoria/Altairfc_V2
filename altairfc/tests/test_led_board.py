from __future__ import annotations

import pytest

from drivers.led_board import BEACON_CHANNEL, SPHERE_CHANNEL, InterlockViolation, LedBoard
from drivers.mcp4728_driver import MAX_CODE


class _FakeDac:
    def __init__(self) -> None:
        self.writes: list[list[int]] = []
        self.closed = False

    def set_vdd_reference(self, codes) -> bool:
        self.writes.append(list(codes))
        return True

    def close(self) -> None:
        self.closed = True


def _make_board() -> tuple[LedBoard, _FakeDac]:
    dac = _FakeDac()
    board = LedBoard(bus=None, dac=dac, ads1115=object(), use_ldac=False)
    return board, dac


def test_write_channel_updates_code():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    assert board.code(SPHERE_CHANNEL) == 500
    assert dac.writes[-1][SPHERE_CHANNEL] == 500


def test_code_clamped_to_dac_range():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, MAX_CODE + 500)
    assert board.code(SPHERE_CHANNEL) == MAX_CODE
    board.write_channel(SPHERE_CHANNEL, -50)
    assert board.code(SPHERE_CHANNEL) == 0


def test_interlock_blocks_beacon_while_sphere_on():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    with pytest.raises(InterlockViolation):
        board.write_channel(BEACON_CHANNEL, 100)
    # the refused write must not have touched either channel
    assert board.code(SPHERE_CHANNEL) == 500
    assert board.code(BEACON_CHANNEL) == 0


def test_interlock_blocks_sphere_while_beacon_on():
    board, dac = _make_board()
    board.write_channel(BEACON_CHANNEL, 300)
    with pytest.raises(InterlockViolation):
        board.write_channel(SPHERE_CHANNEL, 500)
    assert board.code(BEACON_CHANNEL) == 300
    assert board.code(SPHERE_CHANNEL) == 0


def test_interlock_releases_after_all_off():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    board.all_off(SPHERE_CHANNEL)
    board.write_channel(BEACON_CHANNEL, 100)  # must not raise now
    assert board.code(BEACON_CHANNEL) == 100


def test_zero_writes_never_trip_interlock():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    board.write_channel(BEACON_CHANNEL, 0)  # writing 0 is always allowed
    assert board.code(BEACON_CHANNEL) == 0


def test_close_zeroes_all_channels_and_closes_dac():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    board.close()
    assert dac.writes[-1] == [0, 0, 0, 0]
    assert dac.closed
