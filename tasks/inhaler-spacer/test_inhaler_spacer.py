"""Offline checks for inhaler_spacer.py (no simulator): the shake count, the puff log, the fitted and in-the-mouth tests,
the scoring, the sizes, and the scripted demo against the YAM's real reach, step guard and collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import inhaler_spacer as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402


def frame(pos, rot=np.eye(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.zeros(3), spin=np.zeros(3), com=p)


SPACER_IN = M.SPACER_IN
SPACER_OUT = M.SPACER_BACK  # 4 cm back from the mouth
FITTED = M.inhaler_in_spacer(SPACER_IN, M.FIT_IN)


class Rig:
    """Feeds InhalerLog the three bodies at 240 Hz: the inhaler at ``pos`` (upright), its canister ``depth`` in, the
    spacer at ``spacer``."""

    def __init__(self):
        self.m = M.InhalerLog()
        self.m.log = self.m.trace = None

    def step(self, pos, depth=0.0, spacer=SPACER_IN, n=1):
        pos = np.asarray(pos, float)
        for _ in range(n):
            self.m.update({"inhaler": frame(pos), "canister": frame(pos + [0, 0, M.CAN_ORIGIN - depth]),
                           "spacer": frame(spacer)}, {})

    def shake(self, k, at=None):
        at = np.asarray(FITTED if at is None else at, float)
        self.step(at, n=5)  # start from the top of the stroke
        for _ in range(k):
            self.step(at - [0, 0, 0.05], n=40)
            self.step(at, n=40)

    def puff(self, at=None, spacer=SPACER_IN):
        at = FITTED if at is None else at
        self.step(at, depth=3.0 * M.MM, spacer=spacer, n=24)
        self.step(at, depth=0.0, spacer=spacer, n=24)


# -- the log -----------------------------------------------------------------------------------------------------------

def test_shakes_count_down_and_up_strokes_of_3_cm():
    r = Rig()
    r.step(FITTED, n=10)
    r.shake(4)
    assert r.m.shakes == 4
    for dz in (0.02, 0.0):  # a 2 cm jiggle is no shake
        r.step(FITTED - [0, 0, dz], n=40)
    assert r.m.shakes == 4


def test_a_puff_registers_past_the_firing_point_once():
    r = Rig()
    for d in (1.0, 2.0, 2.4):
        r.step(FITTED, depth=d * M.MM, n=5)
    assert r.m.puffs == []
    r.step(FITTED, depth=2.6 * M.MM, n=5)
    r.step(FITTED, depth=1.5 * M.MM, n=5)  # not back above the re-arm point
    r.step(FITTED, depth=2.8 * M.MM, n=5)
    assert len(r.m.puffs) == 1
    r.step(FITTED, depth=0.5 * M.MM, n=5)
    r.step(FITTED, depth=2.8 * M.MM, n=5)
    assert len(r.m.puffs) == 2


def test_fitted_needs_the_mouthpiece_10_mm_in_and_centred():
    s = frame(SPACER_IN)
    assert M.fitted(frame(M.inhaler_in_spacer(SPACER_IN, 12.0 * M.MM)), s)
    assert not M.fitted(frame(M.inhaler_in_spacer(SPACER_IN, 8.0 * M.MM)), s)
    assert not M.fitted(frame(M.inhaler_in_spacer(SPACER_IN, 12.0 * M.MM) + [0, 0.006, 0]), s)


def test_in_the_mouth_needs_the_tip_8_mm_past_the_lips_centred_and_square():
    assert M.in_mouth(frame(SPACER_IN))
    assert not M.in_mouth(frame(SPACER_IN - [0.006, 0, 0]))  # only 6 mm in
    assert not M.in_mouth(frame(SPACER_IN + [0, 0, 0.008]))  # 8 mm high
    tilt = M.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(20.0))
    tip_at = SPACER_IN + M.SPACER_TIP - tilt @ M.SPACER_TIP  # the same tip, the spacer 20 degrees off square
    assert not M.in_mouth(frame(tip_at, tilt))
    assert M.in_mouth(frame(SPACER_IN, M.R_SPACER_ROLLED))  # rolled about its own axis is fine


# -- scoring -------------------------------------------------------------------------------------------------------------

def scored(rig):
    M.MODEL.__dict__.update({k: v for k, v in rig.m.__dict__.items() if k not in ("log", "trace")})
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_a_full_dose():
    r = Rig()
    r.shake(4)
    r.puff()
    r.step(FITTED, n=int(5.2 * 240))
    assert scored(r) == (0.9, True)


def test_the_tiers():
    r = Rig()  # not shaken
    r.puff()
    r.step(FITTED, n=int(5.2 * 240))
    assert scored(r) == (0.5, False)

    r = Rig()  # out of the mouth after 2 s
    r.shake(4)
    r.puff()
    r.step(FITTED, n=480)
    r.step(M.inhaler_in_spacer(SPACER_OUT, M.FIT_IN), spacer=SPACER_OUT, n=960)
    assert scored(r) == (0.5, False)

    r = Rig()  # fitted and in the mouth, no puff
    r.step(FITTED, n=10)
    assert scored(r) == (0.25, False)

    r = Rig()  # two puffs
    r.shake(4)
    r.puff()
    r.puff()
    r.step(FITTED, n=int(5.2 * 240))
    assert scored(r) == (0.0, False)

    r = Rig()  # fired with the spacer out of the mouth
    r.shake(4)
    r.puff(at=M.inhaler_in_spacer(SPACER_OUT, M.FIT_IN), spacer=SPACER_OUT)
    assert scored(r) == (0.0, False)

    r = Rig()  # fired not fitted
    r.shake(4)
    r.puff(at=M.inhaler_in_spacer(SPACER_IN, -20.0 * M.MM))
    assert scored(r) == (0.0, False)


# -- the sizes -----------------------------------------------------------------------------------------------------------

def test_real_sizes_and_force():
    assert M.CAN_TOP == pytest.approx(0.088)  # a Ventolin pMDI is about 9 cm tall
    assert M.CAN_L == pytest.approx(0.075)
    assert (M.SPACER_L, 2 * M.SPACER_R) == pytest.approx((0.145, 0.046))
    assert 22.0 <= M.CAN_K * M.FIRE <= 31.0  # measured actuation forces 22-31 N
    assert M.CAN_K * M.CAN_TRAVEL < 50.0  # within the YAM fingers' 50 N


def test_the_inhaler_fits_the_open_gripper_and_the_socket_takes_the_mouthpiece():
    assert M.CAN_TOP < 0.094 - 4.0 * M.MM  # pads open 94 mm
    w, h = M.MOUTHPIECE_W, M.MOUTHPIECE_H
    c, s = math.cos(abs(M.SPACER_TILT)), math.sin(abs(M.SPACER_TILT))
    assert w * c + h * s < M.SOCKET_W and w * s + h * c < M.SOCKET_H  # rolled 30 degrees, still in


def test_the_canister_clears_the_actuator_through_its_travel():
    # only its top 20 mm collides: its bottom stays above the boot's solid part when pushed fully in
    assert M.CAN_TOP - M.CAN_SOLID - M.CAN_TRAVEL > M.BOOT_H - M.COLLAR + 1.5 * M.MM


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


def test_the_squeeze_fires_and_the_hold_does_not():
    gap_hold, gap_squeeze = -2 * M.HOLD, -2 * M.SQUEEZE
    k_fingers = 5000.0 / 2
    hold_depth = k_fingers * (M.CAN_TOP - gap_hold) / (M.CAN_K + k_fingers)
    assert hold_depth < M.FIRE - 1.0 * M.MM
    # where the squeeze settles: the canister's spring against both fingers' 5,000 N/m (each takes half the squeeze)
    k_fingers = 5000.0 / 2
    depth = k_fingers * (M.CAN_TOP - gap_squeeze) / (M.CAN_K + k_fingers)
    assert M.FIRE < depth < M.CAN_TRAVEL  # fired, short of the canister's stop
    assert M.CAN_K * depth < 50.0


@pytest.mark.slow
def test_the_demo_touches_nothing_it_does_not_hold_and_squeezes_fitted_and_in_the_mouth():
    import plan_check

    out = plan_check.run(verbose=False)
    assert out["hits"] == [], out["hits"][:5]
    assert out["squeeze"]["fitted"] and out["squeeze"]["in_mouth"]
    assert out["min_hand_gap_mm"] > 20.0
