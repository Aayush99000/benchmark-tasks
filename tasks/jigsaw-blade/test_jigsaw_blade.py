"""Offline checks for jigsaw_blade.py (no simulator): the clamp's catch, click and lock, the in-the-slot and teeth
tests, the scoring, the sizes, and the scripted demo against the YAM's real reach, step guard and collision hulls."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import jigsaw_blade as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402


def frame(pos, rot=np.eye(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.zeros(3), spin=np.zeros(3), com=p)


BACKWARDS = M.np.column_stack([-M.Y_AXIS, M.Z_AXIS, -M.X_AXIS])  # in line with the slot, teeth away from the nose


class Rig:
    def __init__(self):
        self.m = M.Clamp()
        self.m.log = self.m.trace = None

    def step(self, depth, cap_in, rot=M.R_BLADE_IN, n=24):
        acc = {}
        for _ in range(n):
            acc = {}
            blade = M.SLOT_MOUTH + np.array([depth, 0.0, 0.0])
            cap = M.CAP_AT + np.array([cap_in, 0.0, 0.0])
            self.m.update({"blade": frame(blade, rot), "cap": frame(cap)}, acc)
        return acc


def test_the_catch_pushes_the_blade_back_unless_the_cap_is_in():
    r = Rig()
    acc = r.step(5.0 * M.MM, 0.0)
    assert acc["blade"][0][0] < -10.0  # pushed back out along -x
    acc = r.step(5.0 * M.MM, 4.0 * M.MM)
    assert "blade" not in acc


def test_it_clicks_then_locks_when_the_cap_is_let_go():
    r = Rig()
    r.step(12.0 * M.MM, 4.5 * M.MM)
    assert r.m.clicked and not r.m.locked
    r.step(12.0 * M.MM, 0.0)
    assert r.m.locked and r.m.locked_forward and "blade" in r.m.poses


def test_letting_the_cap_out_slowly_locks_the_clicked_blade_and_never_pushes_it_out():
    r = Rig()
    r.step(14.0 * M.MM, 5.0 * M.MM)
    assert r.m.clicked
    for cap in np.linspace(5.0, 0.0, 30) * M.MM:
        acc = r.step(14.0 * M.MM, cap, n=4)
        assert "blade" not in acc  # no catch force on a clicked blade
    assert r.m.locked and r.m.locked_forward


def test_let_go_before_the_blade_is_in_it_doesnt_lock():
    r = Rig()
    r.step(6.0 * M.MM, 4.5 * M.MM)
    r.step(6.0 * M.MM, 0.0)
    assert not r.m.clicked and not r.m.locked


def test_backwards_locks_but_not_forward():
    r = Rig()
    r.step(12.0 * M.MM, 4.5 * M.MM, rot=BACKWARDS)
    r.step(12.0 * M.MM, 0.0, rot=BACKWARDS)
    assert r.m.locked and not r.m.locked_forward


def test_out_of_line_is_not_in_the_slot():
    tilted = M.axis_angle_matrix(M.Z_AXIS, np.radians(25.0)) @ M.R_BLADE_IN
    assert M.in_slot(frame(M.SLOT_MOUTH + np.array([0.01, 0, 0]), tilted)) == 0.0
    assert M.in_slot(frame(M.SLOT_MOUTH + np.array([0.01, 0.004, 0]), M.R_BLADE_IN)) == 0.0
    assert M.in_slot(frame(M.SLOT_MOUTH + np.array([0.01, 0, 0]), M.R_BLADE_IN)) == pytest.approx(0.01)


def scored(rig):
    M.MODEL.__dict__.update({k: v for k, v in rig.m.__dict__.items() if k not in ("log", "trace")})
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_the_tiers():
    r = Rig()
    r.step(12.0 * M.MM, 4.5 * M.MM)
    r.step(12.0 * M.MM, 0.0)
    assert scored(r) == (0.9, True)
    r = Rig()
    r.step(12.0 * M.MM, 4.5 * M.MM, rot=BACKWARDS)
    r.step(12.0 * M.MM, 0.0, rot=BACKWARDS)
    assert scored(r) == (0.5, False)
    r = Rig()
    r.step(12.0 * M.MM, 4.5 * M.MM)
    assert scored(r) == (0.25, False)  # in, the cap still held
    r = Rig()
    r.step(2.0 * M.MM, 0.0)
    assert scored(r) == (0.0, False)


def test_real_sizes_and_fits():
    assert M.BLADE_L == pytest.approx(0.100)
    assert M.BLADE_T < M.SLOT_T and M.BLADE_W < M.SLOT_W
    assert M.CATCH < M.SEAT < M.SLOT_DEPTH
    assert 3.0 < M.CAP_K * M.UNLOCK < 6.0  # N to unlock: a two-finger push
    assert M.UNLOCK < M.CAP_TRAVEL


def test_the_blade_starts_backwards_and_goes_in_forwards():
    assert not M.teeth_forward(frame(M.BLADE_AT, M.R_BLADE_START))
    assert M.teeth_forward(frame(M.SLOT_MOUTH, M.R_HAND_IN @ M._R_TB))


GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


EXACT = ("high over the blade", "just over it", "down to it", "grip", "turn it in line with the slot, finger to the cap",
         "push the cap in", "shank to the slot", "push it in", "click", "let the cap go", "let the blade go")


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    """Every pose that touches something is reached exactly; on the way (tipping the blade over in mid-air) the right
    hand may lag its orientation by a few degrees."""
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts, worst = [], {}
    ends, t = [], 0.0
    for w in M.demo()[arm]:
        t += w.duration
        ends.append((t, w.label))
    for t in np.arange(0.0, plan.duration, 0.05):
        before = (plan.worst_ik_mm, plan.worst_ik_deg)
        plan.worst_ik_mm = plan.worst_ik_deg = 0.0
        a, label = plan.action7(t)
        acts.append(a)
        worst[label] = max(worst.get(label, (0, 0)), (plan.worst_ik_mm, plan.worst_ik_deg))
        plan.worst_ik_mm, plan.worst_ik_deg = max(before[0], plan.worst_ik_mm), max(before[1], plan.worst_ik_deg)
    assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 6.0, (plan.worst_ik_mm, plan.worst_ik_deg)
    for label in EXACT:
        mm, deg = worst.get(label, (0, 0))
        assert mm < 0.5 and deg < 0.1, (label, mm, deg)
    assert (np.abs(np.diff(np.array(acts), axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_both_arms_follow_the_same_timeline():
    d = M.demo()
    assert [w.label for w in d["left"]] == [w.label for w in d["right"]]


@pytest.mark.slow
def test_the_demo_touches_only_the_blade_and_pushes_it_in_line():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not [h for h in out["hits"] if h[2] != "blade"], out["hits"][:5]
    assert all(out["in_line"])
