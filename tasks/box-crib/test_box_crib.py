"""Scoring and demo checks for box-crib that run without the simulator: python -m pytest -q test_box_crib.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import box_crib as M

HERE = Path(__file__).resolve().parent


def _quat(yaw_deg=0.0, pitch_deg=0.0):
    """About z by yaw, then about y by pitch (xyzw)."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    return np.array([-sy * sp, cy * sp, sy * cp, cy * cp])


def timber(xy, layer=0, heading=0.0, pitch=0.0, lift=0.0, moving=False) -> ObjectState:
    z = M.TOP + layer * M.W + M.W / 2 + lift
    v = np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos=np.array([xy[0], xy[1], z]), quat_xyzw=_quat(heading, pitch), lin_vel=v,
                       ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))


def start(**over) -> dict:
    s = {n: timber(xy) for n, xy in M.START.items()}
    s.update(over)
    return s


def crib(spacing=M.SPACING, centre=M.LOAD_POINT, layers=3, **over) -> dict:
    """The demo's crib with the pairs ``spacing`` apart, the first ``layers`` layers of it."""
    (cx, cy), h = centre, spacing / 2
    spots = [((cx, cy + h), 0.0), ((cx, cy - h), 0.0), ((cx + h, cy), 90.0), ((cx - h, cy), 90.0),
             ((cx, cy + h), 0.0), ((cx, cy - h), 0.0)]
    s = start()
    for (n, _, _, _, k), (xy, hd) in zip(M.crib_targets(), spots, strict=True):
        if k < layers:
            s[n] = timber(xy, k, hd)
    s.update(over)
    return s


def test_the_task_file_loads():
    task = load(HERE / "box_crib.py")
    assert set(task.free_objects) == set(M.TIMBERS)


# -- scoring ----------------------------------------------------------------------------------------------------------

def test_the_start_scores_0():
    assert M.score(start()) == 0.0 and not M.success(start())


def test_the_demo_crib_succeeds_and_scores_0_9_until_it_has_held():
    assert M.success(crib()) and M.score(crib()) == 0.9


@pytest.mark.parametrize("n", [1, 2])
def test_each_layer_built_right_counts_0_2(n):
    assert M.score(crib(layers=n)) == pytest.approx(0.2 * n)


def test_pairs_laid_at_the_ends_overlap_nothing_and_score_0_6():
    # The natural-looking crib: each pair at the very ends of the pair below, so no end sticks out past a corner.
    s = crib(spacing=M.L - M.W)
    assert M.score(s) == pytest.approx(0.6) and not M.success(s)


def test_the_overlap_threshold_is_50_8_mm():
    ok = M.L - M.W - 2 * (M.OVERLAP + 0.0005)  # 0.5 mm over the rule
    short = M.L - M.W - 2 * (M.OVERLAP - 0.0005)  # 0.5 mm short
    assert M.success(crib(spacing=ok)) and M.score(crib(spacing=short)) == pytest.approx(0.6)


def test_a_crib_off_the_load_point_scores_0_beyond_3_cm_and_no_full_credit_beyond_1_5_cm():
    x, y = M.LOAD_POINT
    assert M.score(crib(centre=(x + 0.02, y))) == pytest.approx(0.6)
    assert M.score(crib(centre=(x + 0.035, y))) == 0.0


def test_a_layer_not_square_to_the_one_below_stops_the_count():
    s = crib()
    s["timber_2"] = timber((M.LOAD_POINT[0], M.LOAD_POINT[1] + 0.04), 1, 0.0)
    s["timber_5"] = timber((M.LOAD_POINT[0], M.LOAD_POINT[1] - 0.04), 1, 0.0)  # layer 2 along x, like layer 1
    assert M.score(s) == pytest.approx(0.2)


def test_a_layer_with_one_timber_stops_the_count():
    s = crib()
    s["timber_5"] = timber(M.START["timber_5"])
    assert M.score(s) == pytest.approx(0.2)


def test_three_timbers_in_a_layer_do_not_count():
    s = crib(layers=1)
    s["timber_2"] = timber((M.LOAD_POINT[0] + 0.07, M.LOAD_POINT[1]), 0, 0.0)
    assert M.score(s) == 0.0


def test_a_tilted_timber_is_not_in_a_layer():
    s = crib()
    s["timber_6"] = timber((M.LOAD_POINT[0], M.LOAD_POINT[1] - 0.04), 2, 0.0, pitch=12.0)
    assert M.score(s) == pytest.approx(0.4)


def test_an_upper_pair_off_to_one_side_does_not_bear_on_both_below():
    x, y = M.LOAD_POINT
    s = crib(layers=2)
    s["timber_2"] = timber((x + 0.04, y + 0.17), 1, 90.0)  # its centre past the right timber's end: not bearing
    s["timber_5"] = timber((x - 0.04, y + 0.17), 1, 90.0)
    assert M.score(s) == pytest.approx(0.2)


def test_a_moving_timber_does_not_succeed():
    s = crib()
    s["timber_3"] = timber((M.LOAD_POINT[0], M.LOAD_POINT[1] + 0.04), 2, 0.0, moving=True)
    assert not M.success(s)


def test_the_overhang_is_measured_at_each_corner():
    s = crib()
    assert M.overhang(s["timber_2"], s["timber_1"]) == pytest.approx((M.L - M.SPACING - M.W) / 2)


# -- the demo ---------------------------------------------------------------------------------------------------------

def test_the_demo_spacing_clears_the_rule_by_14_mm():
    assert (M.L - M.SPACING - M.W) / 2 - M.OVERLAP == pytest.approx(0.01395, abs=1e-5)


def test_the_crib_stays_well_under_the_height_limit():
    # US&R: height at most 3 x the shortest width; this crib's width is the pair's outer faces apart
    assert M.LAYERS * M.W <= 3 * (M.SPACING + M.W) / 2  # under half the limit


def test_the_timbers_start_clear_of_the_crib_and_of_each_other():
    for n, (x, y) in M.START.items():
        assert math.dist((x, y), M.LOAD_POINT) > M.CRIB_RADIUS, n
        assert abs(y) - M.W / 2 > M.L / 2 + 0.04  # beyond the crib's crossing timbers' ends
    ys = sorted(y for _, y in M.START.values())
    assert min(b - a for a, b in zip(ys, ys[1:], strict=False)) >= 0.09 - 1e-9


def test_the_fingers_fit_between_a_pair():
    gap = M.SPACING - M.W  # between a pair's inner faces
    finger_reach = M.APPROACH / 2 + 0.008 - M.W / 2  # an open finger's outer side past the timber's face
    assert gap - finger_reach >= 0.01
    assert M.GRIP <= M.W - 0.006 and M.APPROACH >= M.W + 0.02


def test_the_pad_tips_stay_above_the_layer_below():
    assert M.GRIP_ABOVE - M.TCP_PAST >= 0.01


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_only_one_arm_is_at_the_crib_at_a_time():
    def at_crib(ws):
        t, spans = 0.0, []
        for w in ws:
            if w.label in ("carry", "hover", "set down", "hold", "release", "up"):
                spans.append((t, t + w.duration))
            t += w.duration
        return spans
    left, right = at_crib(M.demo()["left"]), at_crib(M.demo()["right"])
    for a0, a1 in left:
        for b0, b1 in right:
            assert a1 <= b0 + 1e-9 or b1 <= a0 + 1e-9


def test_the_demo_finishes_within_the_run_limit():
    assert max(sum(w.duration for w in ws) for ws in M.demo().values()) < 2400 / 20 - 20


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0


def test_a_carried_timber_clears_its_partner_already_in_the_layer():
    for timber, _, _, _, layer in M.crib_targets():
        carry = [w for w in M._place(timber, "left", (0.2, 0.0), 0.0, layer) if w.label == "carry"][0]
        bottom = carry.pos[2] - M.GRIP_ABOVE
        assert bottom - (M.TOP + (layer + 1) * M.W) >= 0.02, timber

