"""Scoring and demo checks for open-drink-can that run without the simulator:
python -m pytest -q test_open_drink_can.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from mechanism import MechanismState
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

import open_drink_can as M

HERE = Path(__file__).resolve().parent


def pads(x, y, tips_z) -> M.Pads:
    """A gripper pointing straight down, its pad tips at ``tips_z``."""
    return M.Pads(back=np.array([x, y, tips_z + M.PAD_TIP_PAST_TCP + M.PAD_BACK]), tips=np.array([x, y, tips_z]))


AWAY = {"left_gripper": pads(0.22, 0.25, M.TOP + 0.15), "right_gripper": pads(0.22, -0.30, M.TOP + 0.15)}
CAN_LOW = np.array([M.CAN_XY[0] - M.BODY_R / 1000, M.CAN_XY[1] - M.BODY_R / 1000, M.TOP])
CAN_HIGH = np.array([M.CAN_XY[0] + M.BODY_R / 1000, M.CAN_XY[1] + M.BODY_R / 1000, M.TOP + M.RIM_TOP / 1000])


def can(tab_deg=0.0, panel_deg=0.0, catch_mm=0.0, fresh=True, **grippers) -> dict:
    """The can with the tab's tail raised and the panel pushed in by these angles (degrees, positive); a fresh trial
    unless ``fresh`` is False."""
    if fresh:
        M._new_trial()
    joints = {"tab_hinge": -math.radians(tab_deg), "panel_hinge": -math.radians(panel_deg), "catch_slide": catch_mm / 1000}
    z = np.zeros(3)
    return {"can": MechanismState(pos=np.array([*M.CAN_XY, M.TOP]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z,
                                  ang_vel=z, low=CAN_LOW, high=CAN_HIGH, joints=joints), **AWAY, **grippers}


def test_the_task_file_loads():
    task = load(HERE / "open_drink_can.py")
    assert [o.name for o in task.objects] == ["can"]


# -- scoring --------------------------------------------------------------------------------------------------------

def test_the_closed_can_scores_0():
    assert M.score(can()) == 0.0 and not M.success(can())


def test_opened_with_the_tab_succeeds_and_scores_0_9_until_it_has_held():
    s = can(tab_deg=22, panel_deg=82)
    assert M.success(s) and M.score(s) == 0.9


@pytest.mark.parametrize("tab, panel", [(10, 45), (40, 90)])
def test_the_thresholds(tab, panel):
    assert M.success(can(tab_deg=tab, panel_deg=panel))


def test_a_panel_poked_in_with_the_tab_flat_scores_0():
    assert M.score(can(tab_deg=0, panel_deg=80)) == 0.0


def test_the_tab_folded_back_down_after_opening_still_counts():
    assert M.score(can(tab_deg=22, panel_deg=60)) == 0.9
    folded = can(tab_deg=0, panel_deg=82, fresh=False)
    assert M.success(folded) and M.score(folded) == 0.9


def test_a_reset_forgets_the_raised_tab():
    can(tab_deg=22, panel_deg=60)
    assert M.score(can(tab_deg=0, panel_deg=82)) == 0.0


def test_opened_with_the_fingers_still_on_the_tail_scores_three_quarters():
    tail = M.fold_tcp()
    s = can(tab_deg=22, panel_deg=82, left_gripper=pads(tail[0], tail[1], tail[2] - M.PAD_TIP_PAST_TCP))
    assert not M.success(s) and M.score(s) == 0.75


def test_without_the_grippers_state_it_is_not_full_credit():
    s = can(tab_deg=22, panel_deg=82)
    del s["left_gripper"], s["right_gripper"]
    assert not M.success(s) and M.score(s) == 0.75


def test_the_demo_ends_hovering_clear_of_the_can():
    end = M.demo()["left"][-1].pos
    assert end == pytest.approx(M.hover_tcp())
    s = can(tab_deg=22, panel_deg=82, left_gripper=pads(end[0], end[1], end[2] - M.PAD_TIP_PAST_TCP))
    assert M.success(s)


def test_the_demo_folds_the_tab_back_flat_before_letting_go():
    left = M.demo()["left"]
    i = next(k for k, w in enumerate(left) if w.label == "up")
    assert left[i - 1].label.startswith("fold") and left[i - 1].pos == pytest.approx(M.fold_tcp())
    # the closed fingertips come down onto the tab's top (aimed 1 mm into it), on the tail's side of the rivet
    assert left[i - 1].pos[2] - M.PAD_TIP_PAST_TCP == pytest.approx(M.LID_W + (M.TAB_GAP + M.TAB_T - 1.0) / 1000)
    assert 0 < 1000 * M.FOLD_X < M.TAB_TAIL_X


def test_the_seal_broken_but_not_open_scores_half():
    assert M.score(can(tab_deg=5, panel_deg=20)) == 0.5


def test_the_tab_raised_with_the_seal_holding_scores_a_quarter():
    assert M.score(can(tab_deg=15, panel_deg=2)) == 0.25


def test_a_barely_lifted_tab_scores_0():
    assert M.score(can(tab_deg=5, panel_deg=2)) == 0.0


# -- the can's geometry ---------------------------------------------------------------------------------------------

def test_the_can_is_a_real_12_oz_can():
    assert 2 * M.BODY_R == pytest.approx(66.0) and M.RIM_TOP == pytest.approx(122.7)
    assert M.RIM_TOP - M.LID_Z == pytest.approx(4.6)  # a 202 end's panel: 0.270 - 0.090 in below its top


def test_the_tab_is_a_real_tabs_size():
    # 0.985 x 0.620 in, its nose 0.305 to 0.317 in from the rivet, its curled edge about 1.5 mm thick
    assert M.TAB_TAIL_X - M.TAB_NOSE_X == pytest.approx(25.0) and 2 * M.TAB_HALF_W == pytest.approx(15.7)
    assert 7.7 <= -M.TAB_NOSE_X <= 8.1 and M.TAB_T == pytest.approx(1.5) and 2 * M.CURL_R == pytest.approx(M.TAB_T)
    # the tail's underside 0.050 to 0.100 in off the lid, and the whole tab below the rim's top
    assert 1.27 <= M.TAB_GAP <= 2.54
    assert M.LID_Z + M.NOSE_LIFT + M.NOSE_T < M.RIM_TOP and M.LID_Z + M.TAB_GAP + M.TAB_T < M.RIM_TOP
    lows, highs = zip(*(s.bounds() for s in M._tab()))
    size = 1000 * (np.max(highs, axis=0) - np.min(lows, axis=0))
    assert size[0] == pytest.approx(25.0) and size[1] == pytest.approx(15.7)


def tongue_corners(edge):
    """The lever fingertip's tongue (15.5 mm wide, 3.95 mm thick) where it meets the lid: its four corners, mm from
    the lid's centre, with its pad edge's middle at ``edge``."""
    out = M._FACING * -3.95 / math.cos(M.LEAN) / 1000
    pts = [edge + sx * 0.00775 * M._EDGE + k * out for sx in (1, -1) for k in (0, 1)]
    return [1000 * (q - M.CAN_W)[:2] for q in pts]


def test_the_lever_fingertip_fits_between_the_tail_and_the_rim():
    for back in (M.BACK, 0.0, -M.PUSH):
        assert max(np.hypot(*c) for c in tongue_corners(M.pad_edge(back))) < M.RIM_IN - 1.0
    # it comes down beside the tab, not onto it: its pad edge outside the tail's corner, on the tail's side of the rivet
    edge = 1000 * (M.pad_edge(M.BACK) - M.CAN_W)
    assert edge[1] > M.TAB_HALF_W and min(c[0] for c in tongue_corners(M.pad_edge(M.BACK))) > 0


def test_the_pad_edge_goes_under_the_tails_round_edge():
    # resting on the lid, the leaning fingertip's pad edge is under where the pad touches the tail's round edge
    edge_up = 1000 * (M.pad_edge(0.0)[2] - M.LID_W + M.PRESS)
    touch_up = M.TAB_GAP + M.CURL_R * (1 - math.sin(M.LEAN))
    assert 0 < edge_up < M.TAB_GAP < touch_up and touch_up - edge_up > 1.0
    # and at first touch the pad is against the tail's corner
    corner = M.CAN_W + np.array(M.TAIL_CORNER) / 1000
    assert (corner - M.pad_edge(0.0)) @ M._PAD == pytest.approx(M.CURL_R / 1000, abs=1e-6)


def test_only_the_nub_reaches_down_to_the_panel():
    assert M.NOSE_LIFT > M.TAB_GAP and M.NUB_X0 > M.PANEL_X0 and M.NUB_X1 < M.PANEL_X1
    assert M.BODY_X0 >= M.PANEL_X1  # the flat body stays off the panel


def test_the_catch_holds_the_panel_edge():
    assert 0 < M.CATCH_OVERLAP < M.CATCH_R and M.CATCH_TRAVEL > M.CATCH_OVERLAP / 1000


def test_the_opening_is_inside_the_lid():
    assert math.hypot(M.PANEL_X0 - M.OPEN_GAP, M.PANEL_HALF_W + M.OPEN_GAP) < M.RIM_IN


# -- the demo -------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_slides_a_fingertip_under_the_tail_and_levers_it_up():
    left = M.demo()["left"]
    down = next(w for w in left if w.label.startswith("one fingertip down"))
    slide = next(w for w in left if w.label.startswith("slide"))
    assert down.pos == pytest.approx(M.lever_tcp(M.BACK)) and slide.pos == pytest.approx(M.lever_tcp(-M.PUSH))
    lifts = [w for w in left if w.label.startswith("lever the tail up")]
    assert len(lifts) == len(M.LIFT_MM)
    assert [w.pos[2] for w in lifts] == sorted(w.pos[2] for w in lifts) and lifts[-1].pos[2] > slide.pos[2] + 0.006
    # the hand is wide open throughout (no pinch) and leans 15 degrees
    assert all(w.finger == M.FINGER_OPEN for w in (down, slide, *lifts))
    assert math.degrees(math.acos(-M.LEVER_GRIP[2, 2])) == pytest.approx(15.0)


def test_the_demo_finishes_well_within_the_run_limit():
    assert sum(w.duration for w in M.demo()["left"]) < 600 / 20


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0
