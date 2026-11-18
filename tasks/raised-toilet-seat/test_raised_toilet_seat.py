"""Offline checks for raised_toilet_seat.py (no simulator): the fit (wings under the lip, resting, level, straight,
centred), the knob's turns driving the bracket onto the bowl's front wall, the clamp's force and the knob's torque,
the push once the hands are clear, the scoring, the sizes, and the scripted demo against the YAM's real reach, step
guard and collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import raised_toilet_seat as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402


def frame(pos, rot=np.eye(3), spin=np.zeros(3)) -> Frame:
    p = np.asarray(pos, float)
    r = np.asarray(rot, float)
    return Frame(pos=p, rot=r, vel=np.zeros(3), spin=np.asarray(spin, float), com=p + r @ np.array([0.0, 0.0, 0.05]))


def seat_at(dx=0.0, dy=0.0, dz=0.0, yaw_deg=0.0, tilt_deg=0.0):
    r = M.rot_z(math.radians(yaw_deg))
    if tilt_deg:
        r = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(tilt_deg)) @ r
    return M.FIT_AT + np.array([dx, dy, dz]), r


class Rig:
    """The clamp model fed the seat's and the knob's poses; the knob turned by hand, ``turn`` degrees a step."""

    def __init__(self, pose=None):
        self.m = M.SeatClamp()
        self.m.log = self.m.trace = None
        self.m.hands_off = False  # a hand on the knob, unless a test takes it away
        self.seat = pose or (M.SEAT_START.copy(), M.R_SEAT_START.copy())
        self.knob = 0.0  # rad, the knob's turn in the seat
        self.acc = {}

    def frames(self, spin=0.0):
        p, r = self.seat
        kr = r @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), self.knob) @ M.R_KNOB_IN_SEAT
        return {"seat": frame(p, r), "knob": frame(p + r @ M.KNOB_LOCAL, kr, spin * r[:, 0])}

    def step(self, n=1, turn_deg=0.0):
        for _ in range(n):
            self.knob += math.radians(turn_deg)
            self.acc = {}
            w = math.radians(turn_deg) * 240.0
            self.m.update(self.frames(w), self.acc)
        return self


def test_set_on_as_it_starts_it_isnt_fitted():
    assert not M.fitted(M.SEAT_START, M.R_SEAT_START)
    assert not M.hooks_under_lip(M.SEAT_START, M.R_SEAT_START)
    assert M.fitted(*seat_at())


def test_fitted_needs_the_wings_under_the_lip_resting_level_straight_and_centred():
    assert not M.fitted(*seat_at(dx=-0.003))  # 3 mm forward: the hooks only 2.5 mm under
    assert not M.fitted(*seat_at(dz=0.003))  # 3 mm up off the rim
    assert not M.fitted(*seat_at(tilt_deg=2.5))
    assert not M.fitted(*seat_at(yaw_deg=3.5))
    assert not M.fitted(*seat_at(dy=0.006))
    assert M.fitted(*seat_at(dy=0.002, yaw_deg=1.0))
    assert M.fitted(*seat_at(dx=-0.002))  # 3.5 mm under


def test_the_knob_drives_the_bracket_five_mm_a_turn():
    assert M.bracket_face(0.0) == pytest.approx(M.BRACKET_X0)
    assert M.bracket_face(2.0) == pytest.approx(M.BRACKET_X0 - 0.010)
    assert M.bracket_face(-1.0) == pytest.approx(M.BRACKET_X0)  # it can't open past fully open
    gap = -M.bracket_press(M.FIT_AT, np.eye(3), 0.0)
    assert gap == pytest.approx(M.UNDER + 3.0 * M.MM)  # 11 mm to the bowl's front wall, fitted
    assert M.bracket_press(M.FIT_AT, np.eye(3), gap / M.PITCH + 0.2) == pytest.approx(0.001)


def test_the_bracket_clears_the_lip_as_the_seat_goes_on():
    # opened, its face is inside the lip's edge with the seat fitted; set on 8 mm forward it is still clear of the wall
    assert M.BRACKET_X0 > -M.A_IN
    assert M.bracket_press(M.SEAT_START, M.R_SEAT_START, 0.0) < -1.0 * M.MM


def test_the_rig_counts_turns_through_whole_revolutions():
    r = Rig(seat_at())
    r.step(240 * 3, turn_deg=1.5)  # 3 s at 1.5 degrees a substep: 1,080 degrees
    assert r.m.turns == pytest.approx(3.0, abs=1e-6)
    r.step(240, turn_deg=-0.75)
    assert r.m.turns == pytest.approx(2.5, abs=1e-6)


def test_short_of_the_wall_nothing_presses_and_the_knob_turns_freely():
    r = Rig(seat_at())
    r.step(240, turn_deg=1.5 / 4)  # a quarter of a turn
    assert r.m.press < 0 and r.m.force == 0.0 and not r.m.tight
    assert "knob" not in r.acc


def test_past_the_wall_the_bracket_pushes_the_seat_back_and_the_knob_resists():
    r = Rig(seat_at())
    turns = (M.UNDER + 3.0 * M.MM + 2.0 * M.MM) / M.PITCH  # 2 mm past the wall
    r.step(int(round(turns * 240)), turn_deg=1.5)
    assert r.m.press == pytest.approx(0.002, abs=2e-4)
    assert r.m.force == pytest.approx(50.0, abs=5.0)
    f, _ = r.acc["seat"]
    assert f[0] > 40.0  # back (+x), on the seat
    _, tk = r.acc["knob"]
    assert np.dot(tk, M.X_AXIS) < 0  # against the tightening turn
    expected = r.m.force * M.PITCH / (2 * math.pi * M.EFFICIENCY)
    assert abs(np.dot(tk, M.X_AXIS)) == pytest.approx(expected, rel=0.05)


def test_tight_at_fifty_newtons_and_the_knobs_torque_stops_rising_at_a_hundred():
    r = Rig(seat_at())
    tight_turns = (M.UNDER + 3.0 * M.MM + M.TIGHT_N / M.K_BRACKET) / M.PITCH
    r.step(int(tight_turns * 240) - 4, turn_deg=1.5)
    assert not r.m.tight
    r.step(8, turn_deg=1.5)
    assert r.m.tight
    r.step(240 * 2, turn_deg=1.5)
    assert r.m.force > M.K_BRACKET * M.PRESS_MAX  # a stop: it goes on pressing harder
    _, tk = r.acc["knob"]
    assert abs(np.dot(tk, M.X_AXIS)) == pytest.approx(M.K_BRACKET * M.PRESS_MAX * M.PITCH / (2 * math.pi * M.EFFICIENCY), rel=0.01)  # the knob's torque doesn't


def test_the_screw_holds_its_place_once_let_go():
    r = Rig(seat_at())
    r.step(int(2.8 * 240), turn_deg=1.5)
    before = r.m.turns
    r.step(240, turn_deg=0.0)  # let go: nothing turns the knob back
    assert r.m.turns == pytest.approx(before)
    assert r.m.tight


def test_the_tight_bracket_sticks_to_the_bowl_across_it_up_to_half_its_force():
    r = Rig(seat_at())
    r.step(int(2.9 * 240), turn_deg=1.5)
    assert r.m.tight
    p, rot = r.seat
    r.seat = (p + np.array([0.0, 0.0005, 0.0]), rot)  # moved 0.5 mm sideways
    r.step(1)
    f, _ = r.acc["seat"]
    assert f[1] == pytest.approx(-M.K_STICK * 0.0005, rel=0.05)  # pulled back, like static friction
    r.seat = (p + np.array([0.0, 0.010, 0.0]), rot)  # 10 mm: it slips
    r.step(1)
    f, _ = r.acc["seat"]
    assert abs(f[1]) == pytest.approx(M.MU_BRACKET * r.m.force, rel=0.05)


def test_with_no_hand_on_it_the_screw_holds_where_it_was_left():
    r = Rig(seat_at())
    r.step(int(2.8 * 240), turn_deg=1.5)
    press = r.m.press
    r.m.hands_off = True
    r.step(240, turn_deg=-0.5)  # the knob creeping back, as the loaded one did in the sim
    assert r.m.turns == pytest.approx(2.8 - 1 / 3, abs=0.01)
    assert r.m.press == pytest.approx(press)  # the bracket didn't move
    assert r.m.tight


def tightened_rig():
    """Tightened 2.9 turns with a hand on the knob (no push meanwhile), then the hands taken away."""
    r = Rig(seat_at())
    r.m.hands_off = False
    r.step(int(2.9 * 240), turn_deg=1.5)
    assert r.m.tight and r.m.push_t is None
    r.m.hands_off = True
    return r


def test_with_the_hands_clear_and_the_seat_fitted_it_is_pushed_forwards_then_each_side():
    r = tightened_rig()
    assert r.m.hands_off  # offline: no hands
    r.step(1)
    assert r.m.push_t is not None
    bracket = np.array([r.m.force, 0.0, 0.0])  # the bracket's spring on the seat, nothing moving
    for d in (-M.X_AXIS, M.Y_AXIS, -M.Y_AXIS):
        r.step(119)  # half way through each 1 s push: its peak
        f, _ = r.acc["seat"]
        assert np.allclose(f - bracket, M.PUSH_N * d, atol=0.5)
        r.step(121)
    r.step(2)
    assert r.m.pushed and r.m.held  # the rig doesn't move the seat


def test_a_seat_that_shifts_under_the_push_isnt_held():
    r = tightened_rig()
    r.step(1)
    for i in range(int(3 * 240)):
        if i == 300:
            p, rot = r.seat
            r.seat = (p + np.array([-0.004, 0.0, 0.0]), rot)  # it slid 4 mm forwards
        r.step(1)
    r.step(2)
    assert r.m.pushed and not r.m.held


def test_a_hand_coming_back_stops_the_push():
    r = tightened_rig()
    r.step(10)
    r.m.hands_off = False
    r.step(1)
    assert r.m.push_t is None and not r.m.pushed


def score_of(rig):
    M.MODEL.__dict__.update({k: v for k, v in rig.m.__dict__.items() if k not in ("log", "trace")})
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_the_tiers():
    assert score_of(Rig()) == (0.0, False)  # set on, not fitted
    r = Rig(seat_at(yaw_deg=1.0))
    r.step(24)
    r.m.hands_off = False  # still being held: no push
    r.step(1)
    assert score_of(r) == (0.25, False)  # fitted, loose
    r = tightened_rig()
    r.m.hands_off = False
    r.step(1)
    assert score_of(r) == (0.5, False)  # fitted and tight, hands still on it
    r = tightened_rig()
    r.step(int(3.2 * 240))
    assert score_of(r) == (0.9, True)  # held under the push, hands clear
    loose = Rig(seat_at())
    loose.step(int(3.2 * 240))
    assert score_of(loose) == (0.25, False)  # a loose seat isn't scored as held, pushed or not


def test_real_sizes():
    assert 2 * M.SEAT_B == pytest.approx(17 * 0.0254, abs=0.001)  # Drive 12014: 17 x 16.5 x 4.5 in, 4 lb
    assert 2 * M.SEAT_A == pytest.approx(16.5 * 0.0254, abs=0.001)
    assert M.SEAT_H == pytest.approx(4.5 * 0.0254, abs=0.001)
    assert M.SEAT_MASS == pytest.approx(4 * 0.4536, abs=0.01)
    assert 2 * M.B_OUT == pytest.approx(14 * 0.0254, abs=0.002)  # a round bowl: about 14 in wide
    assert M.A_OUT + M.A_IN > 0.30  # and over 16 in from its middle's back to its front
    assert M.LIP_T <= 1.75 * 0.0254  # Invacare: not for a lip over 1-3/4 in
    assert 2 * M.KNOB_R < 0.094  # the gripper opens 94 mm


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
    assert sum(w.duration for w in d["right"]) < 80.0  # inside the 90 s run, with the push and the 1 s hold after


def test_the_demo_turns_the_knob_past_tight():
    turned = M.STROKES * (M.ROLL_TO - M.ROLL_FROM) / 360.0
    gap = M.UNDER + 3.0 * M.MM
    assert turned * M.PITCH - gap > M.TIGHT_N / M.K_BRACKET + 1.0 * M.MM  # 1 mm of margin past tight


@pytest.mark.slow
def test_the_demo_touches_nothing_it_shouldnt():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not out["hits"], out["hits"][:5]
    assert out["min_hand_gap_mm"] > 10.0
