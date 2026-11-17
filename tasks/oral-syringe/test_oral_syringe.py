"""Offline checks for oral_syringe.py (no simulator): the dose log (squirts, pauses, medicine pushed outside the cheek),
the tip-in-the-cheek test, the scoring, the sizes, and the scripted demo against the YAM's real reach, step guard and
collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import oral_syringe as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402


def frame(pos, rot=np.eye(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.zeros(3), spin=np.zeros(3), com=p)


IN = M.syringe_origin(M.TIP_IN)  # the syringe's origin with its tip in the cheek
OUT = M.syringe_origin(M.TIP_IN - M.R_SYRINGE_FACE[:, 0] * 0.04)


class Rig:
    """Feeds DoseLog the syringe at ``at`` (face orientation) and its plunger ``travel`` in, at 240 Hz."""

    def __init__(self):
        self.m = M.DoseLog()
        self.m.log = self.m.trace = None
        self.travel = 0.0

    def hold(self, seconds, at=IN):
        self.push(0.0, seconds, at)

    def push(self, ml, seconds, at=IN):
        n = max(1, int(seconds * 240))
        r = M.R_SYRINGE_FACE
        for i in range(n):
            self.travel += ml * M.ML / n
            plunger = np.asarray(at) + r @ np.array([M.PLUNGER_AT + self.travel, 0.0, 0.0])
            self.m.update({"syringe": frame(at, r), "plunger": frame(plunger, r)}, {})


def dose(squirts, pause=2.0, at=IN):
    r = Rig()
    r.hold(0.5, at)
    for ml in squirts:
        r.push(ml, 1.5, at)
        r.hold(pause, at)
    return r


# -- the tip in the cheek ---------------------------------------------------------------------------------------------

def test_the_demo_pose_is_in_the_cheek_and_straight_in_or_too_deep_is_not():
    assert M.tip_in_cheek(frame(IN, M.R_SYRINGE_FACE))
    straight = M.rot_x(M.TILT)  # pointing straight into the mouth, tip in the middle
    tip_mid = M.MOUTH + np.array([0.015, 0.0, -0.004])
    assert not M.tip_in_cheek(frame(M.syringe_origin(tip_mid, straight), straight))
    deep = M.TIP_IN + M.R_SYRINGE_FACE[:, 0] * 0.015  # pushed on towards the back of the mouth
    assert not M.tip_in_cheek(frame(M.syringe_origin(deep), M.R_SYRINGE_FACE))
    assert M.out_of_mouth(frame(OUT, M.R_SYRINGE_FACE)) and not M.out_of_mouth(frame(IN, M.R_SYRINGE_FACE))


# -- the dose log ------------------------------------------------------------------------------------------------------

def test_squirts_are_split_by_pauses_of_a_second():
    r = dose([1.6, 1.7, 1.7])
    assert r.m.given == pytest.approx(5.0, abs=0.01)
    assert len(r.m.squirts) == 3 and r.m.biggest() == pytest.approx(1.7, abs=0.01)
    r = dose([1.6, 1.7], pause=0.5)  # too short a pause: one squirt of 3.3 ml
    assert r.m.biggest() == pytest.approx(3.3, abs=0.01)


def test_a_slow_creep_and_jitter_still_count_as_a_pause():
    r = Rig()
    r.hold(0.5)
    r.push(1.6, 1.5)
    for i in range(int(2.0 * 240)):  # creeping 0.25 mm/s, jittering +-0.05 mm substep to substep
        r.push(0.25 * M.MM / 240 / M.ML + (0.05 if i % 2 else -0.05) * M.MM / M.ML, 1.0 / 240)
    r.push(1.6, 1.5)
    r.hold(2.0)
    assert len(r.m.squirts) == 2 and r.m.biggest() < 2.0
    assert r.m.given == pytest.approx(3.2 + 0.25 * 2.0 * M.MM / M.ML, abs=0.03)  # jitter not counted as given


def test_medicine_pushed_outside_the_cheek_is_counted():
    r = dose([1.0], at=OUT)
    assert r.m.outside == pytest.approx(1.0, abs=0.01) and not r.m.clean()


# -- scoring -------------------------------------------------------------------------------------------------------------

def scored(rig):
    M.MODEL.__dict__.update({k: v for k, v in rig.m.__dict__.items() if k not in ("log", "trace")})
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_the_tiers():
    r = dose([1.6, 1.7, 1.7])
    r.hold(1.0, OUT)
    assert scored(r) == (0.9, True)
    assert scored(dose([1.6, 1.7, 1.7])) == (0.5, False)  # not taken out of the mouth
    r = dose([2.5, 2.5])
    r.hold(1.0, OUT)
    assert scored(r) == (0.5, False)  # a squirt over 2 ml
    assert scored(dose([1.5, 1.5])) == (0.25, False)  # 3 ml given
    assert scored(dose([1.5])) == (0.0, False)  # under 2 ml
    r = dose([1.5, 1.5])
    r.push(0.5, 1.0, OUT)
    assert scored(r) == (0.0, False)  # some squirted with the tip out of the cheek


# -- the sizes -----------------------------------------------------------------------------------------------------------

def test_real_sizes():
    assert M.BORE == pytest.approx(0.01206)
    assert M.STROKE == pytest.approx(0.0438, abs=0.0002)  # 5 ml
    assert 2.0 < M.DRAG * M.ML < 3.5  # N at 1 ml/s
    assert M.FULL_ML * M.ML < M.STROKE  # the stop is past the full dose


def test_the_thumb_rest_clears_the_flange_with_the_syringe_empty():
    assert -(M.PLUNGER_AT + M.STROKE) - M.THUMB_T / 2 - M.FLANGE_T == pytest.approx(M.THUMB_CLEAR)


# -- the demo --------------------------------------------------------------------------------------------------------

GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = np.array([plan.action7(t)[0] for t in np.arange(0.0, plan.duration, 0.05)])
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(acts, axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_both_arms_follow_the_same_timeline():
    d = M.demo()
    assert [w.label for w in d["left"]] == [w.label for w in d["right"]]


def test_each_planned_squirt_is_under_2_ml():
    assert M.STROKE / M.SQUIRTS / M.ML < M.SQUIRT_MAX_ML


@pytest.mark.slow
def test_the_demo_keeps_off_the_face_and_pushes_with_the_tip_in_the_cheek():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not [h for h in out["hits"] if h[2] == "manikin"], out["hits"][:5]
    assert all(out["squirt_in_cheek"])
