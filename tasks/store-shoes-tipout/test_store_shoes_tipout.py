"""Scoring, geometry and demo checks for store-shoes-tipout that run without the simulator:
python -m pytest -q test_store_shoes_tipout.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import axis_angle_matrix

import store_shoes_tipout as M
from mechanism import MechanismState

HERE = Path(__file__).resolve().parent
Z3 = np.zeros(3)
Y = np.array([0.0, 1.0, 0.0])


def pads(x, y, z) -> M.Pads:
    return M.Pads(back=np.array([x, y, z + 0.04]), tips=np.array([x, y, z]))


AWAY = {"left_gripper": pads(*M.PARK_LEFT), "right_gripper": pads(*M.PARK_RIGHT)}


def quat_from_matrix(r) -> np.ndarray:
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


# A clog standing toe-down in the shut bin: its toe (+x) down, its sole toward the cabinet's back (+x world).
TOE_DOWN = axis_angle_matrix(Y, math.pi / 2)  # local +x -> world -z, local -z (sole) -> world +x... checked below
HEEL_DOWN = axis_angle_matrix(Y, -math.pi / 2)


def clog_state(pos, r=np.eye(3), moving=False) -> ObjectState:
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=quat_from_matrix(r), lin_vel=np.array([0.1, 0, 0]) if moving else Z3,
                       ang_vel=Z3, low=pos - 0.15, high=pos + 0.15)


def cabinet(open_deg=0.0) -> MechanismState:
    return MechanismState(pos=np.array([0, 0, M.TOP]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3, ang_vel=Z3,
                          low=Z3, high=Z3, joints={"door": -math.radians(open_deg)})


def standing_in_bin(y, r=TOE_DOWN, lift=0.0) -> ObjectState:
    """A clog standing in the shut bin against its floor (centre placed from the clog's own extent)."""
    corners = M.clog_corners(clog_state((0, 0, 0), r))
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    front = M.X0 + M.DOOR_T + 0.001
    x = front - lo[0]
    z = M.TOP + M.BOTTOM + 0.008 + 0.001 - lo[2] + lift
    return clog_state((x, y, z), r)


def objects(left=None, right=None, open_deg=0.0, grippers=None) -> dict:
    return {"cabinet": cabinet(open_deg), "left_clog": left if left is not None else standing_in_bin(0.11),
            "right_clog": right if right is not None else standing_in_bin(-0.11), **(grippers or AWAY)}


ON_TABLE = {n: clog_state(p, axis_angle_matrix(np.array([0, 0, 1.0]), math.pi)) for n, p in M.CLOG_START.items()}


def test_the_task_file_loads():
    task = load(HERE / "store_shoes_tipout.py")
    assert [o.name for o in task.objects] == ["cabinet", "left_clog", "right_clog"]
    assert task.scene == "station"


# -- the cabinet and the clogs at true size ------------------------------------------------------------------------------
def test_the_cabinet_is_real_size():
    assert (M.CAB_W, M.CAB_D, M.CAB_H) == (0.521, 0.181, 0.391)
    assert M.BIN_W == 0.47 and M.BIN_DEPTH_TOP == 0.15
    assert M.CABINET.size[0] == pytest.approx(M.CAB_D + M.LIP_OUT, abs=1e-6)  # the lip stands proud


def test_the_bin_fits_inside_the_cabinet_when_shut():
    assert M.X0 + M.DOOR_T + M.BIN_DEPTH_TOP + M.WALL < M.X0 + M.CAB_D - M.WALL
    assert M.LIP_Z < M.TOP_UNDER


def test_the_clog_is_thin_at_the_toe_and_thick_at_the_heel():
    corners = {i: np.array([np.asarray(c) + np.asarray(s) / 2 * np.array([sx, 1, sz])
                            for sx in (-1, 1) for sz in (-1, 1)]) for i, (c, s) in enumerate(M.CLOG_PARTS)}
    bottom = M.CLOG_PARTS[0][0][2] - M.CLOG_PARTS[0][1][2] / 2
    toe_top = corners[1][:, 2].max()
    heel_top = corners[3][:, 2].max()
    assert toe_top - bottom == pytest.approx(0.065) and heel_top - bottom == pytest.approx(0.095)
    assert M.CLOG_KG == 0.198


# -- the trap, from geometry alone ---------------------------------------------------------------------------------------
def lowest_top(toe_first: bool) -> float:
    """Stand a clog upright in the shut bin, toe or heel down, as low as its thickness lets it go at each height; return
    the height of its upper end. Thickness along the clog: 6.5 cm at the toe, 8 cm in the middle, 9.5 cm at the heel."""
    bottom = M.CLOG_PARTS[0][0][2] - M.CLOG_PARTS[0][1][2] / 2
    length = M.CLOG[0]
    xs = np.linspace(-length / 2, length / 2, 300)

    def thickness(x):
        tops = [c[2] + s[2] / 2 for c, s in M.CLOG_PARTS if c[0] - s[0] / 2 - 1e-9 <= x <= c[0] + s[0] / 2 + 1e-9]
        return max(tops) - bottom

    floor = M.BOTTOM + 0.008
    for h in np.arange(floor, 0.40, 0.001):
        heights = h + ((length / 2 - xs) if toe_first else (xs + length / 2))
        if all(thickness(x) <= M.bin_depth(z) + 1e-9 for x, z in zip(xs, heights)):
            return h + length
    return float("inf")


def test_toe_first_the_clog_sinks_below_the_lip_and_heel_first_it_stands_proud():
    toe, heel = lowest_top(True), lowest_top(False)
    assert toe <= M.LIP_Z, toe
    assert heel > M.LIP_Z + 0.05, heel  # 5 cm or more above the lip: the tray's front rim stops the door


# -- in the bin ----------------------------------------------------------------------------------------------------------
def test_a_clog_standing_toe_down_is_in_the_bin():
    c = standing_in_bin(0.11)
    assert M.in_bin(c, objects())
    top = M.clog_corners(c)[:, 2].max() - M.TOP
    assert top <= M.LIP_Z


def test_toe_down_really_puts_the_toe_down():
    c = standing_in_bin(0.11)
    r = M._rot(c.quat_xyzw)
    assert r[:, 0] @ np.array([0, 0, 1.0]) == pytest.approx(-1.0)


def test_a_clog_lifted_proud_of_the_lip_is_not_in():
    c = standing_in_bin(0.11)
    headroom = M.LIP_Z + M.LIP_TOL - (M.clog_corners(c)[:, 2].max() - M.TOP)
    assert headroom > 0.03  # toe first, a clog stands 3 cm or more below the lip
    assert M.in_bin(standing_in_bin(0.11, lift=headroom - 0.002), objects())
    assert not M.in_bin(standing_in_bin(0.11, lift=headroom + 0.002), objects())


def test_a_clog_heel_down_at_the_floor_does_not_fit_the_bin():
    assert not M.in_bin(standing_in_bin(0.11, r=HEEL_DOWN), objects())


def test_the_bin_moves_with_the_door():
    """A clog carried round the hinge with the door is still in the bin; left behind, it is not."""
    c = standing_in_bin(0.11)
    hinge = np.array([M.HINGE[0], M.HINGE[1], M.TOP + M.HINGE[2]])
    turn = axis_angle_matrix(Y, -math.radians(35.0))
    carried = clog_state(hinge + turn @ (np.asarray(c.pos) - hinge), turn @ M._rot(c.quat_xyzw))
    assert M.in_bin(carried, objects(left=carried, open_deg=35.0))
    assert not M.in_bin(c, objects(open_deg=35.0))


def test_clogs_on_the_table_are_not_in():
    assert not any(M.in_bin(ON_TABLE[n], objects()) for n in M.CLOG_START)


# -- the grade -----------------------------------------------------------------------------------------------------------
def test_both_in_and_shut_and_hands_off_is_full():
    assert M.grade(objects()) == 1.0
    assert M.success(objects())
    assert M.score(objects()) == M.COMPLETE_SCORE


def test_a_hand_on_the_door_or_a_clog_moving_is_three_quarters():
    on_door = {"left_gripper": pads(*M.PARK_LEFT), "right_gripper": pads(M.X0 - 0.02, -0.17, M.TOP + 0.35)}
    assert M.grade(objects(grippers=on_door)) == 0.75
    moving = standing_in_bin(0.11)
    moving = ObjectState(moving.pos, moving.quat_xyzw, np.array([0.1, 0, 0]), Z3, moving.low, moving.high)
    assert M.grade(objects(left=moving)) == 0.75


def test_door_angles():
    hinge = np.array([M.HINGE[0], M.HINGE[1], M.TOP + M.HINGE[2]])

    def at(deg):
        turn = axis_angle_matrix(Y, -math.radians(deg))
        out = {}
        for n, y in (("left_clog", 0.11), ("right_clog", -0.11)):
            c = standing_in_bin(y)
            out[n] = clog_state(hinge + turn @ (np.asarray(c.pos) - hinge), turn @ M._rot(c.quat_xyzw))
        return objects(left=out["left_clog"], right=out["right_clog"], open_deg=deg)

    assert M.grade(at(1.5)) == 1.0
    assert M.grade(at(10.0)) == 0.75
    assert M.grade(at(30.0)) == 0.5


def test_one_in_is_a_quarter_and_none_is_nothing():
    assert M.grade(objects(right=ON_TABLE["right_clog"])) == 0.25
    assert M.grade(objects(left=ON_TABLE["left_clog"], right=ON_TABLE["right_clog"])) == 0.0
    assert M.grade(objects(left=ON_TABLE["left_clog"], right=ON_TABLE["right_clog"], open_deg=40.0)) == 0.0


def test_the_start_scores_nothing():
    assert M.grade(objects(left=ON_TABLE["left_clog"], right=ON_TABLE["right_clog"])) == 0.0


# -- the demo ------------------------------------------------------------------------------------------------------------
def test_the_clogs_start_clear_of_the_opening_door():
    """The clogs lie beside the cabinet, outside the door's width, so the open door never hangs over them."""
    for name, (x, y, _) in M.CLOG_START.items():
        assert abs(y) - M.CLOG[1] / 2 > M.DOOR_Y + 0.02, name
        assert x + M.CLOG[0] / 2 < M.X0  # in front of the cabinet's front face


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(monkeypatch, arm):
    monkeypatch.delenv("SHOES_DEMO", raising=False)
    p = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, p.duration, 0.05):
        p.action7(t)
    assert p.worst_ik_mm < 3.0 and p.worst_ik_deg < 0.5, (p.worst_ik_mm, p.worst_ik_deg)  # 2.8 mm crossing in front of the lip


def test_the_demo_fits_the_run_limit():
    plans = M.demo()
    assert max(sum(w.duration for w in plans[a]) for a in ("left", "right")) * 20 < 1200


def test_stillness_comes_from_positions_over_six_steps():
    M._clear_history()
    at_rest = {n: standing_in_bin(0.11 if n == "left_clog" else -0.11) for n in M.CLOG_START}
    for _ in range(M.STILL_STEPS - 1):
        assert not any(M._record_stillness(at_rest).still.values())  # not enough history yet
    assert all(M._record_stillness(at_rest).still.values())
    moved = dict(at_rest)
    c = at_rest["left_clog"]
    moved["left_clog"] = clog_state(np.asarray(c.pos) + np.array([0.002, 0, 0]), M._rot(c.quat_xyzw))
    record = M._record_stillness(moved)
    assert not record.still["left_clog"] and record.still["right_clog"]
    # a stale reported velocity doesn't matter once the history says it is still
    stale = ObjectState(c.pos, c.quat_xyzw, np.array([0.087, 0, 0]), np.array([0.88, 0, 0]), c.low, c.high)
    assert M.grade({**objects(left=stale), "stillness": M.Stillness({"left_clog": True, "right_clog": True})}) == 1.0
    assert M.grade({**objects(), "stillness": M.Stillness({"left_clog": False, "right_clog": True})}) == 0.75
    M._clear_history()
