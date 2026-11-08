"""Scoring and demo checks for evidence-markers that run without the simulator: python -m pytest -q test_evidence_markers.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import evidence_markers as M

HERE = Path(__file__).resolve().parent


def state(xy, z_bottom, height, yaw_deg=0.0, tilt_deg=0.0, moving=False) -> ObjectState:
    h, t = math.radians(yaw_deg) / 2, math.radians(tilt_deg) / 2
    qz = np.array([0.0, 0.0, math.sin(h), math.cos(h)])
    qx = np.array([math.sin(t), 0.0, 0.0, math.cos(t)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    centre = np.array([xy[0], xy[1], z_bottom + height / 2])
    v = np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos=centre, quat_xyzw=q, lin_vel=v, ang_vel=np.zeros(3),
                       low=np.array([0.0, 0.0, z_bottom]), high=np.array([0.0, 0.0, z_bottom + height]))


def scene(**over) -> dict:
    """The start: evidence where it fell, markers in their row."""
    s = {}
    for n, (xy, yaw) in M.EVIDENCE.items():
        h = M.CASE_H if n.startswith("casing") else (M.PHONE if n == "phone" else M.KNIFE)[2]
        s[n] = state(xy, M.FLOOR, h, yaw)
    for n, xy in M.MARKERS.items():
        s[n] = state(xy, M.FLOOR, M.MARKER[2], M.MARKER_START_YAW)
    s.update(over)
    return s


def marker(xy, yaw=0.0, **kw):
    return state(xy, M.FLOOR, M.MARKER[2], yaw, **kw)


def demo_end(**over) -> dict:
    s = scene(**{m: marker(xy, yaw) for m, (xy, yaw) in M.demo_targets().items()})
    s.update(over)
    return s


def test_the_task_file_loads():
    task = load(HERE / "evidence_markers.py")
    assert set(task.free_objects) == {*M.ITEMS, *M.TAGS}


def test_the_start_scores_0():
    assert M.score(scene()) == 0.0 and not M.success(scene())
    s = scene()  # every marker starts more than 5 cm from every item: nothing counts as marked before the robot acts
    for m in M.TAGS:
        for n in M.ITEMS:
            assert M.gap(M.footprint(m, s[m]), M.footprint(n, s[n])) > M.GAP_MAX + 0.02, (m, n)


def test_every_item_marked_succeeds_and_scores_0_9_until_it_has_held():
    s = demo_end()
    assert M.marked(s) == set(M.ITEMS)
    assert M.success(s) and M.score(s) == 0.9


@pytest.mark.parametrize("n", [1, 2, 3])
def test_each_marked_item_earns_a_quarter(n):
    keep = list(M.demo_targets())[:n]
    s = scene(**{m: marker(*M.demo_targets()[m]) for m in keep})
    assert M.score(s) == pytest.approx(0.25 * n)


@pytest.mark.parametrize(("gap_mm", "counts"), [(0.0, False), (4.0, False), (6.0, True), (20.0, True), (49.0, True),
                                                (51.0, False)])
def test_beside_means_5_mm_to_5_cm_not_touching(gap_mm, counts):
    (x, y), _ = M.EVIDENCE["casing_1"]
    cx = x - M.CASE_R - gap_mm / 1000 - M.MARKER[1] / 2
    s = scene(marker_1=marker((cx, y), 90.0))
    assert ("casing_1" in M.marked(s)) == counts


@pytest.mark.parametrize("item", list(M.EVIDENCE))
def test_evidence_moved_3_mm_scores_0(item):
    (x, y), yaw = M.EVIDENCE[item]
    h = M.CASE_H if item.startswith("casing") else (M.PHONE if item == "phone" else M.KNIFE)[2]
    assert M.score(demo_end(**{item: state((x + 0.0029, y), M.FLOOR, h, yaw)})) == 0.9
    assert M.score(demo_end(**{item: state((x + 0.0031, y), M.FLOOR, h, yaw)})) == 0.0


@pytest.mark.parametrize("item", ["phone", "knife"])
def test_evidence_turned_3_degrees_scores_0(item):
    (xy, yaw), h = M.EVIDENCE[item], (M.PHONE if item == "phone" else M.KNIFE)[2]
    assert M.score(demo_end(**{item: state(xy, M.FLOOR, h, yaw + 2.9)})) == 0.9
    assert M.score(demo_end(**{item: state(xy, M.FLOOR, h, yaw + 3.1)})) == 0.0
    assert M.score(demo_end(**{item: state(xy, M.FLOOR, h, yaw + 180.0)})) == 0.9  # a box turned half way looks the same


def test_a_knocked_over_casing_scores_0():
    xy, _ = M.EVIDENCE["casing_2"]
    assert M.score(demo_end(casing_2=state(xy, M.FLOOR, M.CASE_H, tilt_deg=90.0))) == 0.0


def test_moving_evidence_scores_0():
    xy, yaw = M.EVIDENCE["phone"]
    assert M.score(demo_end(phone=state(xy, M.FLOOR, M.PHONE[2], yaw, moving=True))) == 0.0


def test_a_marker_lying_down_or_off_the_floor_does_not_count():
    xy, yaw = M.demo_targets()["marker_1"]
    assert "casing_1" not in M.marked(demo_end(marker_1=marker(xy, yaw, tilt_deg=90.0)))
    assert "casing_1" not in M.marked(demo_end(marker_1=state(xy, M.FLOOR + 0.01, M.MARKER[2], yaw)))
    assert "casing_1" not in M.marked(demo_end(marker_1=marker(xy, yaw, moving=True)))


def test_one_marker_counts_for_one_item_only():
    # Two markers on the phone, none on casing_1: 3 items marked, not 4.
    s = demo_end(marker_1=marker((0.40, -0.25), 0.0))
    assert "casing_1" not in M.marked(s) and len(M.marked(s)) == 3 and M.score(s) == 0.75


def test_markers_bunched_far_from_the_evidence_score_0():
    # All four set out together in a 2 x 2 block in the clearest corner of the mat, 9 cm from the nearest evidence.
    offs = [(-0.025, -0.035), (0.025, -0.035), (-0.025, 0.035), (0.025, 0.035)]
    s = scene(**{m: marker((0.18 + dx, -0.21 + dy)) for (dx, dy), m in zip(offs, M.TAGS, strict=True)})
    assert M.score(s) == 0.0


def test_gap_between_outlines():
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
    assert M.gap(sq, sq + [3, 0]) == pytest.approx(2.0)
    assert M.gap(sq, sq + [0.5, 0.5]) == 0.0
    assert M.gap(sq, sq + [2, 2]) == pytest.approx(math.sqrt(2))


# -- the demo -----------------------------------------------------------------------------------------------------

def test_the_demo_aims_two_cm_from_each_item():
    s = demo_end()
    for m, (n, _) in M.DEMO_PLAN.items():
        assert M.gap(M.footprint(m, s[m]), M.footprint(n, s[n])) == pytest.approx(M.DEMO_GAP, abs=1e-6)


def test_the_demo_markers_stay_clear_of_the_chair_leg_and_each_other():
    s = demo_end()
    leg = M.footprint("casing_1", state(M.CHAIR_LEG_XY, M.FLOOR, 0.1)) * (M.CHAIR_LEG_R / M.CASE_R) - (
        np.array(M.CHAIR_LEG_XY) * (M.CHAIR_LEG_R / M.CASE_R - 1))
    for m in M.TAGS:
        assert M.gap(M.footprint(m, s[m]), leg) > 0.01
        for o in M.TAGS:
            if o != m:
                assert M.gap(M.footprint(m, s[m]), M.footprint(o, s[o])) > 0.01


def test_every_start_and_target_is_on_the_mat():
    (mx, my), (sx, sy, _) = M.MAT_XY, M.MAT_SIZE
    s = demo_end()
    for n in (*M.ITEMS, *M.TAGS):
        for p in M.footprint(n, s[n]):
            assert abs(p[0] - mx) <= sx / 2 - 0.01 and abs(p[1] - my) <= sy / 2 - 0.01, n
        for p in M.footprint(n, scene()[n]):
            assert abs(p[0] - mx) <= sx / 2 - 0.01 and abs(p[1] - my) <= sy / 2 - 0.01, n


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_fingers_close_across_the_markers_width_never_toward_the_evidence():
    for arm, waypoints in M.demo().items():
        sets = [w for w in waypoints if w.label == "set down"]
        mine = [m for m, a in M.DEMO_ARM.items() if a == arm]
        for w, m in zip(sets, mine, strict=True):
            _, yaw = M.demo_targets()[m]
            close_dir = math.degrees(w.yaw) + 90.0  # the fingers close along the grip's y
            to_item = M.DEMO_PLAN[m][1] + 180.0
            d = (close_dir - to_item) % 180.0
            assert min(d, 180.0 - d) == pytest.approx(90.0)


def test_the_grip_squeezes_the_marker_and_lets_go_clear():
    assert M.GRIP_FINGER / M.FINGER_OPEN * 0.094 < M.MARKER[0] - 0.006
    assert (M.RELEASE_FINGER / M.FINGER_OPEN * 0.094 - M.MARKER[0]) / 2 >= 0.006


def test_the_carry_clears_the_markers_and_never_passes_over_the_chair_leg():
    assert M.TRAVEL_Z - M.GRIP_ABOVE - (M.FLOOR + M.MARKER[2]) >= 0.015
    leg = np.array(M.CHAIR_LEG_XY)
    for m, (xy, _) in M.demo_targets().items():
        a, b = np.array(M.MARKERS[m]), np.array(xy)
        assert M._seg_dist(leg, a, b) > M.CHAIR_LEG_R + 0.04, m  # the gripper (fingers open) passes 4 cm clear


def test_the_demo_finishes_well_within_the_run_limit():
    assert max(sum(w.duration for w in ws) for ws in M.demo().values()) < 2400 / 20 / 2
