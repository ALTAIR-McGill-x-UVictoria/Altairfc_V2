from __future__ import annotations

from drivers.led_board import BEACON_CHANNEL, RELAY_CHANNEL, SPHERE_CHANNEL, LedBoard
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


def test_beacon_brightness_channel_independent_of_sphere():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    board.write_channel(BEACON_CHANNEL, 1500)  # must not raise
    assert board.code(BEACON_CHANNEL) == 1500


def test_sphere_and_relay_can_be_energized_simultaneously():
    """No electrical constraint between the sphere and the beacon relay — see led_board.py's
    module docstring. Keeping the beacon dark during real imaging is a scheduling concern
    (tasks/lighting_task.py's beacon_windows), not something this driver enforces."""
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    board.write_channel(RELAY_CHANNEL, MAX_CODE)  # must not raise
    assert board.code(SPHERE_CHANNEL) == 500
    assert board.code(RELAY_CHANNEL) == MAX_CODE


def test_close_zeroes_all_channels_and_closes_dac():
    board, dac = _make_board()
    board.write_channel(SPHERE_CHANNEL, 500)
    board.close()
    assert dac.writes[-1] == [0, 0, 0, 0]
    assert dac.closed
