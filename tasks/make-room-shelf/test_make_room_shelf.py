"""Scoring and demo checks for make-room-shelf that run without the simulator: python -m pytest -q test_make_room_shelf.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import make_room_shelf as M

HERE = Path(__file__).resolve().parent


def _quat(yaw_deg=0.0, roll_deg=0.0):
    """About z by yaw, then about x by roll (xyzw)."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    # q = qz * qx
    return np.array([cy * sr, sy * sr, sy * cr, cy * cr])


def state(name, xy, bottom, yaw=0.0, roll=0.0, moving=False) -> ObjectState:
    h = M.SIZE[name][2]
    v = np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos=np.array([xy[0], xy[1], bottom + h / 2]), quat_xyzw=_quat(yaw, roll), lin_vel=v,
                       ang_vel=np.zeros(3), low=np.array([0.0, 0.0, bottom]), high=np.array([0.0, 0.0, bottom + h]))


def start(**over) -> dict:
    s = {n: state(n, M.START[n], M.SHELF) for n in M.BOXES}
    s["carton"] = state("carton", M.CARTON_START, M.TOP)
    s.update(over)
    return s


def packed(**over) -> dict:
    t = M.demo_targets()
    s = start(**{n: state(n, (M.ROW_X, t[n]), M.SHELF) for n in M.BOXES})
    s.update(over)
    return s


def done(**over) -> dict:
    s = packed(carton=state("carton", (M.ROW_X, M.demo_targets()["carton"]), M.SHELF))
    s.update(over)
    return s


def test_the_task_file_loads():
    task = load(HERE / "make_room_shelf.py")
    assert set(task.free_objects) == set(M.NAMES)


# -- the layout ------------------------------------------------------------------------------------------------------

def test_no_gap_at_the_start_fits_the_carton_but_the_free_space_does():
    assert M.widest_gap(start()) == pytest.approx(max(M.GAPS))
    assert max(M.GAPS) < M.CARTON[1] / 3
    assert sum(M.GAPS) + sum(M.WIDTH.values()) == pytest.approx(M.INNER)
    assert M.INNER - sum(M.WIDTH.values()) - M.CARTON[1] == pytest.approx(0.05)  # 5 cm to spare in all


def test_every_box_starts_on_the_shelf():
    s = start()
    assert all(M.on_shelf(s[n], M.SIZE[n]) for n in M.BOXES)


def test_the_carton_fits_on_the_board_front_to_back():
    assert M.CARTON[0] < M.BACK_X - M.FRONT_X and M.BOX_DEPTH < M.BACK_X - M.FRONT_X
    assert M.ROW_X - M.BOX_DEPTH / 2 > M.FRONT_X


# -- scoring ---------------------------------------------------------------------------------------------------------

def test_the_start_scores_0():
    assert M.score(start()) == 0.0 and not M.success(start())


def test_the_carton_on_the_shelf_succeeds_and_scores_0_9_until_it_has_held():
    assert M.success(done()) and M.score(done()) == 0.9


def test_packed_boxes_without_the_carton_score_a_quarter():
    assert M.score(packed()) == 0.25 and not M.success(packed())


def test_packing_all_four_boxes_at_one_end_also_makes_room():
    y, ys = M.HALF, {}
    for n in M.BOXES:
        ys[n] = y - M.WIDTH[n] / 2
        y -= M.WIDTH[n] + 0.005
    s = start(**{n: state(n, (M.ROW_X, ys[n]), M.SHELF) for n in M.BOXES})
    assert M.score(s) == 0.25
    s["carton"] = state("carton", (M.ROW_X, -M.HALF + 0.01 + M.CARTON[1] / 2), M.SHELF)
    assert M.success(s)


def test_a_box_taken_off_the_shelf_scores_0_even_with_the_carton_on():
    for n in M.BOXES:
        assert M.score(done(**{n: state(n, (0.20, 0.25), M.TOP)})) == 0.0, n


def test_a_box_set_on_another_box_scores_0():
    t = M.demo_targets()
    assert M.score(done(box_2=state("box_2", (M.ROW_X, t["box_1"]), M.SHELF + M.BOX_H))) == 0.0


def test_a_box_set_on_the_carton_scores_0():
    assert M.score(done(box_3=state("box_3", (M.ROW_X, 0.0), M.SHELF + M.CARTON[2]))) == 0.0


def test_a_knocked_over_box_scores_0():
    t = M.demo_targets()
    assert M.score(done(box_4=state("box_4", (M.ROW_X, t["box_4"]), M.SHELF, roll=90.0))) == 0.0


def test_a_box_hanging_over_the_front_edge_scores_0():
    t = M.demo_targets()
    x = M.FRONT_X + M.BOX_DEPTH / 2 - 0.01  # 1 cm over the edge
    assert M.score(done(box_1=state("box_1", (x, t["box_1"]), M.SHELF))) == 0.0


def test_the_carton_on_top_of_the_boxes_does_not_count():
    s = packed(carton=state("carton", (M.ROW_X, 0.13), M.SHELF + M.BOX_H))
    assert not M.success(s) and M.score(s) == 0.25


def test_the_carton_hanging_over_the_front_edge_does_not_count():
    x = M.FRONT_X + M.CARTON[0] / 2 - 0.01
    s = packed(carton=state("carton", (x, 0.0), M.SHELF))
    assert not M.success(s) and M.score(s) == 0.25


def test_the_carton_lying_on_its_side_on_the_shelf_counts():
    # Turned 90 degrees about y it lies on an 18 x 10 cm face: 10 cm deep, 7 cm tall, still wholly on the board.
    lying = np.array([0.0, math.sin(math.radians(45)), 0.0, math.cos(math.radians(45))])
    carton = ObjectState(pos=np.array([M.ROW_X, 0.0, M.SHELF + M.CARTON[0] / 2]), quat_xyzw=lying,
                         lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))
    assert M.success(packed(carton=carton))


def test_the_carton_leaning_does_not_count():
    s = packed(carton=state("carton", (M.ROW_X, 0.0), M.SHELF + 0.02, roll=20.0))
    assert not M.success(s)


def test_a_moving_carton_does_not_succeed():
    assert not M.success(done(carton=state("carton", (M.ROW_X, 0.0), M.SHELF, moving=True)))


def test_room_with_the_carton_dropped_off_the_table_scores_0():
    assert M.score(packed(carton=state("carton", (0.18, 0.0), M.TOP - 0.3))) == 0.0


def test_a_box_turned_on_the_shelf_still_counts_if_it_is_on_the_board():
    t = M.demo_targets()
    assert M.success(done(box_1=state("box_1", (M.ROW_X, t["box_1"] - 0.004), M.SHELF, yaw=8.0)))


# -- the demo --------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_targets_score_full():
    assert M.success(done())


def test_the_packed_boxes_leave_the_carton_room_each_side():
    t = M.demo_targets()
    room = (t["box_2"] - M.WIDTH["box_2"] / 2) - (t["box_3"] + M.WIDTH["box_3"] / 2)
    assert room - M.CARTON[1] == pytest.approx(0.034)  # 1.7 cm each side of the carton


def test_no_box_passes_over_another_and_carries_clear_the_board():
    # Carried only 2.5 cm above the board, a box must not sweep through any box already standing there.
    assert M.TRAVEL_Z - M.BOX_GRIP_ABOVE - M.SHELF >= 0.02
    assert M.TRAVEL_Z - M.CARTON_GRIP_ABOVE - M.SHELF >= 0.02
    t = M.demo_targets()
    where = dict(M.BOX_Y)
    for name in ("box_1", "box_2", "box_4", "box_3"):  # the order each arm moves them
        w = M.WIDTH[name] / 2
        sweep = (min(where[name], t[name]) - w, max(where[name], t[name]) + w)
        for other, y in where.items():
            if other != name:
                o = M.WIDTH[other] / 2
                assert sweep[1] < y - o or sweep[0] > y + o, (name, other)
        where[name] = t[name]


def test_the_carton_slides_in_clear_of_the_packed_boxes():
    t = M.demo_targets()
    left_face, right_face = t["box_2"] - M.WIDTH["box_2"] / 2, t["box_3"] + M.WIDTH["box_3"] / 2
    assert left_face - (t["carton"] + M.CARTON[1] / 2) >= 0.015
    assert (t["carton"] - M.CARTON[1] / 2) - right_face >= 0.015


def test_the_fingers_stay_clear_of_the_back_lip():
    open_half = 0.094 / 2 + 0.008  # open fingers and their thickness
    assert M.ROW_X + open_half <= M.BACK_X - 0.005


def test_the_two_arms_are_never_both_near_the_middle():
    plan = M.demo()

    def times(ws):
        t, out = 0.0, []
        for w in ws:
            t += w.duration
            out.append((t, w))
        return out

    left, right = times(plan["left"]), times(plan["right"])
    for tr, wr in right:
        # the left arm's target at that moment
        wl = next((w for t, w in left if t >= tr), plan["left"][-1])
        assert abs(wl.pos[1] - wr.pos[1]) >= 0.15 or math.dist(wl.pos[:2], wr.pos[:2]) >= 0.2, (tr, wl.label, wr.label)


def test_the_demo_finishes_well_within_the_run_limit():
    assert max(sum(w.duration for w in ws) for ws in M.demo().values()) < 2400 / 20 / 2


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0


def test_empty_fingers_come_and_go_above_every_box_and_the_carton():
    tips = M.CLEAR_Z - M.TCP_PAST
    assert tips - (M.SHELF + M.BOX_H) >= 0.02 and tips - (M.SHELF + M.CARTON[2]) >= 0.005
    for ws in M.demo().values():
        for w in ws:
            if w.label in ("over it", "lift away", "wait", "back to its own side"):
                assert w.pos[2] == M.CLEAR_Z, w.label


def test_the_demo_lets_go_of_the_carton_before_it_counts_as_on_the_shelf():
    assert M.CARTON_DROP > M.SHELF_TOL
    set_down = [w for w in M.demo()["left"] if w.label == "set down"][-1]
    assert set_down.pos[2] - M.CARTON_GRIP_ABOVE - M.SHELF == pytest.approx(M.CARTON_DROP)

