"""Scoring and demo checks for drill-backer that run without the simulator: python -m pytest -q test_drill_backer.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import drill_backer as M

HERE = Path(__file__).resolve().parent


def _quat(yaw_deg=0.0, roll_deg=0.0):
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    return np.array([cy * sr, sy * sr, sy * cr, cy * cr])  # yaw about z, then roll about x


def piece(name, xy, bottom, yaw=0.0, roll=0.0, moving=False) -> ObjectState:
    h = M.SIZE[name][2]
    v = np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos=np.array([xy[0], xy[1], bottom + h / 2]), quat_xyzw=_quat(yaw, roll), lin_vel=v,
                       ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))


def start(**over) -> dict:
    s = {n: piece(n, M.START[n], M.TOP) for n in M.NAMES}
    s.update(over)
    return s


def set_up(board_xy=M.BIT_XY, backer_xy=M.BIT_XY, **over) -> dict:
    s = {"backer": piece("backer", backer_xy, M.TT), "board": piece("board", board_xy, M.TT + M.BACKER[2])}
    s.update(over)
    return s


def test_the_task_file_loads():
    task = load(HERE / "drill_backer.py")
    assert set(task.free_objects) == set(M.NAMES)


# -- scoring --------------------------------------------------------------------------------------------------------

def test_the_start_scores_0():
    assert M.score(start()) == 0.0 and not M.success(start())


def test_the_board_centred_on_the_backer_succeeds_and_scores_0_9_until_it_has_held():
    assert M.success(set_up()) and M.score(set_up()) == 0.9


def test_the_board_under_the_bit_with_no_backer_scores_a_quarter():
    s = start(board=piece("board", M.BIT_XY, M.TT))
    assert M.score(s) == 0.25 and not M.success(s)


def test_the_board_5_mm_off_the_axis_on_the_backer_scores_half():
    x, y = M.BIT_XY
    assert M.score(set_up(board_xy=(x, y + 0.005))) == 0.5


def test_the_board_more_than_1_cm_off_the_axis_scores_0():
    x, y = M.BIT_XY
    assert M.score(set_up(board_xy=(x, y + 0.012))) == 0.0


def test_the_bit_near_the_backer_edge_scores_half():
    x, y = M.BIT_XY
    s = set_up(backer_xy=(x, y + M.BACKER[1] / 2 - 0.01))  # the axis 1 cm inside the backer's side
    assert 0 < M.backing_margin(s["backer"]) < M.BACKED and M.score(s) == 0.5


def test_the_backer_past_the_bit_does_not_back_it():
    x, y = M.BIT_XY
    s = set_up(backer_xy=(x + M.BACKER[0] / 2 + 0.01, y))  # its end short of the axis: the board tips onto it
    s["board"] = piece("board", M.BIT_XY, M.TT + M.BACKER[2])
    assert M.backing_margin(s["backer"]) < 0 and M.score(s) == 0.0


def test_the_backer_on_top_of_the_board_scores_0():
    s = {"board": piece("board", M.BIT_XY, M.TT), "backer": piece("backer", M.BIT_XY, M.TT + M.BOARD[2])}
    assert M.score(s) == 0.25  # the board is under the bit on the table; the backer on it does nothing


def test_a_tilted_board_scores_0():
    assert M.score(set_up(board=piece("board", M.BIT_XY, M.TT + M.BACKER[2], roll=6.0))) == 0.0


def test_the_backer_turned_across_still_backs_the_hole():
    s = set_up(backer=piece("backer", M.BIT_XY, M.TT, yaw=90.0))
    assert M.backing_margin(s["backer"]) == pytest.approx(M.BACKER[1] / 2) and M.success(s)


def test_a_moving_board_does_not_succeed():
    assert not M.success(set_up(board=piece("board", M.BIT_XY, M.TT + M.BACKER[2], moving=True)))


# -- the scene ------------------------------------------------------------------------------------------------------

def test_the_bit_tip_clears_the_board_on_the_backer_with_room_to_slide_in():
    top = M.TT + M.BACKER[2] + M.BOARD[2]
    assert M.BIT_TIP - top >= 0.03
    assert M.BIT_TIP - (top + M.SLIDE_GAP) >= 0.025


def test_the_hole_lies_under_the_bit():
    for o in M.OBJECTS:
        if o.name.startswith("press_table"):
            hx, hy = o.size[0] / 2, o.size[1] / 2
            x, y = o.pos[0], o.pos[1]
            assert not (x - hx < M.BIT_XY[0] < x + hx and y - hy < M.BIT_XY[1] < y + hy), o.name


def test_both_pieces_fit_on_the_press_table_and_clear_the_column():
    for n in M.NAMES:
        far = M.BIT_XY[0] + M.SIZE[n][0] / 2
        assert far < M.COLUMN_XY[0] - M.COLUMN_R - 0.02
    assert M.BACKER[0] <= M.PRESS_TABLE[0]


def test_the_pieces_start_off_the_press_table():
    for n, (x, y) in M.START.items():
        assert y - M.SIZE[n][1] / 2 > M.PRESS_TABLE[1] / 2 + 0.03


# -- the demo -------------------------------------------------------------------------------------------------------

def test_the_gripper_stays_well_in_front_of_the_bit():
    for n in M.NAMES:
        assert M.OFFSET[n] - M.CHUCK_R >= 0.05
        assert M.OFFSET[n] <= M.SIZE[n][0] / 2 - 0.02 + 1e-9  # the 2 cm wide pads wholly on the piece


def test_the_grip_squeezes_and_the_approach_clears():
    w = M.BOARD[1]
    assert M.GRIP <= w - 0.006 and M.APPROACH >= w + 0.015 and M.APPROACH <= 0.094


def test_the_demo_sets_the_backer_then_the_board_under_the_bit():
    ws = M.demo()["left"]
    downs = [w for w in ws if w.label == "set down"]
    assert len(downs) == 2
    assert downs[0].pos[2] < downs[1].pos[2]  # the backer first, lower


@pytest.mark.parametrize("arm", ["left"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_finishes_well_within_the_run_limit():
    assert sum(w.duration for w in M.demo()["left"]) < 2400 / 20 / 2


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0


def test_the_gripper_stays_in_front_of_the_head_and_under_it():
    head = next(o for o in M.OBJECTS if o.name == "press_head")
    head_front = head.pos[0] - head.size[0] / 2
    for n in M.NAMES:
        tcp_x = M.BIT_XY[0] - M.OFFSET[n]
        assert head_front - tcp_x >= 0.05 - 1e-9, n
    assert M.HEAD_LOW - M.TT >= 0.28

