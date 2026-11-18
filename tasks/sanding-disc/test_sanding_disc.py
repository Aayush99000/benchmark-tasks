"""Offline checks for sanding_disc.py (no simulator): the hook-and-loop model (caught only flat, fuzzy side down, then
held where it landed), the hole alignment, pressing, the scoring, the sizes, and the scripted demo against the YAM's
real reach, step guard and collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sanding_disc as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402


def frame(pos, rot=np.eye(3), vel=np.zeros(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.asarray(vel, float), spin=np.zeros(3), com=p)


def on_pad(dx=0.0, dy=0.0, up=0.0, turn=M.PAD_TURN, tilt_deg=0.0, flip=False):
    """The disc's pose lying on the pad: its middle (dx, dy) from the pad's, its fuzzy side ``up`` over the face."""
    r = M.rot_z(turn)
    if tilt_deg:
        r = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(tilt_deg)) @ r
    if flip:
        r = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.pi) @ r
    t = math.radians(tilt_deg)
    lift = (M.DISC_T / 2) * math.cos(t) + M.DISC_R * math.sin(t)
    return M.PAD_AT + np.array([dx, dy, up + lift]), r


class Rig:
    def __init__(self):
        self.m = M.HookAndLoop()
        self.m.log = self.m.trace = None
        self.pose = (M.DISC_AT.copy(), M.R_DISC_START)

    def step(self, n=24):
        for _ in range(n):
            self.m.update({"disc": frame(*self.pose)}, {})
            for _, (p, q) in self.m.poses.items():
                self.pose = (np.asarray(p, float), M.fittingkit.quat_to_matrix(q))
        return self


def test_the_disc_starts_free_in_its_stand():
    r = Rig().step()
    assert not r.m.caught
    assert np.allclose(r.pose[0], M.DISC_AT)


def test_laid_flat_on_the_pad_it_is_caught_where_it_lands_and_cant_slide():
    r = Rig()
    r.pose = on_pad(0.0015, -0.001, up=0.0005, turn=M.PAD_TURN + math.radians(1.0))
    r.step()
    assert r.m.caught
    x, y, yaw = r.m.caught_at
    assert np.allclose((x, y), (0.0015, -0.001), atol=1e-6)
    assert math.degrees(yaw) == pytest.approx(math.degrees(M.PAD_TURN) + 1.0)
    r.pose = (r.pose[0] + np.array([0.01, 0.0, 0.02]), M.rot_z(0.3))  # pushed and turned: written back
    r.step(1)
    assert np.allclose(r.pose[0], M.PAD_AT + np.array([0.0015, -0.001, M.DISC_T / 2]))
    assert M.disc_yaw(r.pose[1]) == pytest.approx(yaw)


def test_tilted_over_the_pad_or_held_above_it_it_isnt_caught():
    for pose in (on_pad(tilt_deg=6.0), on_pad(up=0.003), on_pad(dx=0.025)):
        r = Rig()
        r.pose = pose
        r.step()
        assert not r.m.caught


def test_grit_side_down_it_isnt_caught():
    r = Rig()
    r.pose = on_pad(flip=True)
    r.step()
    assert not r.m.caught


def test_the_holes_line_up_only_turned_and_centred_right():
    assert M.hole_error_mm((0.0, 0.0), M.PAD_TURN) == pytest.approx(0.0, abs=1e-6)
    assert M.hole_error_mm((0.0, 0.0), M.PAD_TURN + math.radians(45.0)) == pytest.approx(0.0, abs=1e-6)  # any hole
    assert M.hole_error_mm((0.0, 0.0), M.PAD_TURN + math.radians(3.0)) < M.ALIGN_MM  # 2.0 mm at the holes
    assert M.hole_error_mm((0.0, 0.0), M.PAD_TURN + math.radians(5.0)) > M.ALIGN_MM  # 3.4 mm
    assert M.hole_error_mm((0.003, 0.0), M.PAD_TURN) > M.ALIGN_MM
    assert M.hole_error_mm((0.0, 0.0), 0.0) > 10.0  # picked up square and put on square: holes half way between


def test_lined_up_within_the_tolerance_each_hole_is_two_thirds_open():
    assert M.hole_open_fraction(0.0) == pytest.approx(1.0)
    assert M.hole_open_fraction(M.ALIGN_MM) > 2 / 3
    assert M.hole_open_fraction(10.0) == 0.0


def test_a_fingertip_on_the_caught_disc_presses_it():
    at = (0.0, 0.0, M.PAD_TURN)
    face = M.PAD_AT + np.array([0.0, 0.0, M.DISC_T])
    assert M.pressing(np.array([face + [0.01, 0.0, 0.002]]), at)
    assert not M.pressing(np.array([face + [0.01, 0.0, 0.008]]), at)  # above it
    assert not M.pressing(np.array([face + [0.07, 0.0, 0.002]]), at)  # beyond its edge


def scored(rig, pressed_s=0.0):
    M.MODEL.__dict__.update({k: v for k, v in rig.m.__dict__.items() if k not in ("log", "trace")})
    M.MODEL.pressed_s = pressed_s
    M.MODEL.frame = frame(*rig.pose)
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_the_tiers():
    r = Rig().step()
    assert scored(r) == (0.0, False)  # in its stand
    r.pose = on_pad(tilt_deg=8.0)
    r.step()
    assert scored(r) == (0.25, False)  # on the pad, not flat
    r.pose = on_pad(flip=True)
    r.step()
    assert scored(r) == (0.25, False)  # grit side down
    r = Rig()
    r.pose = on_pad(turn=M.PAD_TURN + math.radians(8.0))
    r.step()
    assert scored(r, pressed_s=1.0) == (0.5, False)  # caught and pressed, holes out of line
    r = Rig()
    r.pose = on_pad()
    r.step()
    assert scored(r) == (0.5, False)  # lined up, not pressed on
    assert scored(r, pressed_s=1.0) == (0.9, True)


def test_real_sizes():
    assert 2 * M.DISC_R == pytest.approx(0.125)  # a 5-inch disc
    assert M.DISC_R == M.PAD_R
    assert M.DISC_IN / math.cos(math.pi / 12) - M.DISC_R < 1.5 * M.MM  # the 12-sided disc within 1.5 mm of round
    assert M.HOLES == 8
    assert 2 * M.HOLE_R < M.DISC_R - M.HOLE_CIRCLE  # the holes inside the disc


GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = []
    for t in np.arange(0.0, plan.duration, 0.05):
        acts.append(plan.action7(t)[0])
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.5, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(np.array(acts), axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_both_arms_follow_the_same_timeline():
    d = M.demo()
    assert [w.label for w in d["left"]] == [w.label for w in d["right"]]
    assert sum(w.duration for w in d["right"]) < 50.0  # well inside the 60 s run


def test_the_demo_carries_the_disc_in_line_with_the_holes():
    import plan_check  # noqa: F401  (imports cleanly)

    d = M.demo()
    plan = ArmPlan(d["right"], station.BASES["right"])
    t = 0.0
    for w in d["right"]:
        t += w.duration
        if w.label == "far edge on the pad":
            break
    p, r, _, _ = plan.target(t - 1e-6)
    c, rot = M.disc_held(p, r)
    # 0.9 mm off centre (tipped, the middle is short of the far edge's line; it drops flat on it) and the hand turned
    # 0.8 degrees past, for the lag measured in the sim
    assert M.hole_error_mm((c - M.PAD_AT)[:2], M.disc_yaw(rot)) < 2.0
    assert rot[2, 2] > 0.9  # grit side up


@pytest.mark.slow
def test_the_demo_touches_nothing_it_shouldnt():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not out["hits"], out["hits"][:5]
    assert out["min_hand_gap_mm"] > 10.0
