"""Offline checks for oxygen_flowmeter.py (no simulator): the nipple model (on the nipple, centred and held by the
barbs against a push weaker than their push-back, moved on by a stronger one; seated past the last barb, then held),
the barbs' push-back estimate (barbfit.py), the flowmeter model (the knob's angle sets the flow, the ball rides on its line), the scoring, the
sizes, and the scripted demo against the YAM's real reach, step guard and collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import barbfit  # noqa: E402
import oxygen_flowmeter as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402


def frame(pos, rot=np.eye(3), vel=np.zeros(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.asarray(vel, float), spin=np.zeros(3), com=p)


def tube_on(depth, off_y=0.0, tilt_deg=0.0):
    """The tubing's pose with its mouth ``depth`` past the nipple's tip, ``off_y`` to the side, tilted ``tilt_deg``
    about z."""
    r = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(tilt_deg)) @ M.R_TUBE_START
    mouth = M.NIP_TIP + depth * M.NIP_AXIS + np.array([0.0, off_y, 0.0])
    return mouth - r @ np.array([0.0, 0.0, M.TUBE_L / 2]), r


def knob_turned(deg):
    """The knob's rotation turned ``deg`` anticlockwise, seen from the front."""
    return axis_angle_matrix(-M.X_AXIS, math.radians(deg)) @ M.R_KNOB


class Rig:
    def __init__(self):
        self.m = M.Oxygen()
        self.m.log = self.m.trace = None
        self.tube = (M.TUBE_AT.copy(), M.R_TUBE_START)
        self.tube_vel = np.zeros(3)
        self.knob = M.R_KNOB
        self.acc = {}

    def step(self, n=24):
        for _ in range(n):
            self.acc = {}
            self.m.update({"tubing": frame(*self.tube, vel=self.tube_vel), "knob": frame(M.KNOB_AT, self.knob),
                           "ball": frame(M.BALL_AT)}, self.acc)
            p, q = self.m.poses.get("tubing", (None, None))
            if p is not None:
                self.tube = (np.asarray(p, float), M.fittingkit.quat_to_matrix(q))
        return self


def test_at_the_start_the_tubing_is_off_and_the_flow_is_off():
    r = Rig().step()
    assert not r.m.seated and r.m.depth < 0
    assert r.m.flow == 0.0
    assert r.m.poses["ball"][0][2] == pytest.approx(M.BALL_AT[2])


def pushed(depth0, push_n, seconds=1.0):
    """The tubing on the nipple ``depth0`` on, pushed along it with ``push_n`` N, built up over 0.1 s as an arm's
    joint springs build it, moving along the nipple under that and the model's force (240 Hz, its 40 g): (its depth
    at the end, the model)."""
    m = M.Oxygen()
    m.log = m.trace = None
    x, v = depth0, 0.0
    for i in range(int(240 * seconds)):
        push = push_n * min(1.0, i / 24.0)
        acc = {}
        pos, rot = tube_on(x)
        m.update({"tubing": frame(pos, rot, vel=v * M.NIP_AXIS), "knob": frame(M.KNOB_AT, M.R_KNOB),
                  "ball": frame(M.BALL_AT)}, acc)
        if m.seated:
            break
        f = float(acc.get("tubing", [np.zeros(3)])[0] @ M.NIP_AXIS)
        v += (push + f) / M.TUBE_MASS / 240.0
        x += v / 240.0
    return x, m


def test_the_barbs_push_back_steps_up_at_each_barb():
    assert barbfit.F_BARB == pytest.approx(4.41, abs=0.01)
    d = np.linspace(0.0, 0.020, 401)
    f = np.array([barbfit.push_back(x) for x in d])
    assert (np.diff(f) >= -1e-12).all()  # never less further on
    assert barbfit.push_back(0.002) == 0.0  # the first cone is narrower than the bore there
    for k, crest in enumerate(barbfit.CRESTS):
        assert barbfit.push_back(crest) == pytest.approx((k + 1) * barbfit.F_BARB)
    assert barbfit.push_back(0.020) < M.PULL_OFF_N  # it goes on more easily than EN 13544-2's 30 N pulls it off
    lo, hi = min(v[3] for v in barbfit.estimate_range().values()), max(v[3] for v in barbfit.estimate_range().values())
    assert 8.0 < lo < hi < 28.0


def test_a_push_weaker_than_the_barbs_doesnt_move_it_a_stronger_one_moves_it_on():
    x, m = pushed(0.006, 3.0)  # over the first barb (4.4 N), pushed with 3 N
    assert x == pytest.approx(0.006, abs=2e-4) and m.engaged and not m.seated
    x, m = pushed(0.006, 6.0)  # 6 N: on up the second barb's cone, past where its push-back is 6 N (it has picked up
    # speed), stopping short of its crest
    assert barbfit.push_back(x) >= 6.0 and x < barbfit.CRESTS[1] and not m.seated
    x, m = pushed(0.006, 15.0)  # 15 N: over all three, seated
    assert m.seated and m.max_push > barbfit.push_back(0.009)


def test_pulled_back_past_a_barb_it_holds_up_to_30_n():
    x, m = pushed(0.010, -25.0)
    assert x == pytest.approx(0.010, abs=2e-4) and m.engaged
    x, m = pushed(0.010, -35.0, seconds=0.3)
    assert x < 0.0  # pulled off


def test_on_the_nipple_it_is_pulled_onto_its_axis():
    r = Rig()
    r.tube = tube_on(0.006, off_y=0.001)  # 1 mm to the left of the axis
    r.step(1)
    f, _ = r.acc["tubing"]
    assert r.m.engaged and f[1] == pytest.approx(-2 * M.CENTRE_K * 0.001, rel=0.05)


def test_past_the_last_barb_it_is_seated_and_held():
    r = Rig()
    r.tube = tube_on(0.016)
    r.step()
    assert r.m.seated
    assert r.m.seated_depth == pytest.approx(0.016)
    r.tube = (r.tube[0] - np.array([0.03, 0.0, 0.01]), r.tube[1])  # pulled off: written back
    r.step(1)
    depth, radial, angle = M.on_nipple(*r.tube)
    assert depth == pytest.approx(0.016) and radial < 1e-9 and angle < 1e-6


def test_part_way_on_it_isnt_seated():
    r = Rig()
    r.tube = tube_on(0.009)
    r.step()
    assert not r.m.seated and r.m.depth == pytest.approx(0.009)


def test_off_the_nipples_line_it_isnt_on():
    for pose in (tube_on(0.013, off_y=0.0025), tube_on(0.013, tilt_deg=12.0), tube_on(-0.005)):
        r = Rig()
        r.tube = pose
        r.step()
        assert not r.m.seated and r.m.depth < 0


def test_the_knob_sets_the_flow_and_the_ball_rides_on_its_line():
    r = Rig()
    for deg in np.linspace(0.0, 36.0, 40):  # turned slowly anticlockwise
        r.knob = knob_turned(deg)
        r.step(1)
    assert r.m.flow == pytest.approx(2.0)
    assert r.m.poses["ball"][0][2] - M.BALL_AT[2] == pytest.approx(0.020)
    r.knob = knob_turned(0.0)  # back off
    r.step(1)
    assert r.m.flow == pytest.approx(0.0, abs=1e-9)


def test_the_knob_angle_unwraps_past_half_a_turn():
    r = Rig()
    for deg in np.linspace(0.0, 250.0, 120):
        r.knob = knob_turned(deg)
        r.step(1)
    assert math.degrees(r.m.theta) == pytest.approx(250.0)
    assert r.m.flow == pytest.approx(15.0 * 250.0 / 270.0)


def test_the_scale_lines_and_tolerance():
    assert M.ball_rise(2.0) == pytest.approx(0.020)
    assert M.ball_rise(5.0) == pytest.approx(0.050)
    assert M.ball_rise(15.0) == pytest.approx(0.076)
    assert REST_TOP_FITS()
    # within 0.25 L/min: the ball's middle within 2.5 mm of the 2 line, half way to the next half line
    assert M.ball_rise(M.SET_LPM + M.SET_TOL) - M.ball_rise(M.SET_LPM) == pytest.approx(0.0025)
    assert M.flow_lpm(M.theta_for(2.0)) == pytest.approx(2.0)
    assert math.degrees(M.theta_for(2.0)) == pytest.approx(36.0)
    assert M.flow_lpm(math.radians(280.0)) == 65.0  # flush


def REST_TOP_FITS() -> bool:
    """The ball at 15 L/min stays inside the shroud."""
    return M.REST_Z + M.ball_rise(15.0) + M.BALL_R < M.SHROUD_TOP


def scored(seated=False, depth=-1.0, deg=0.0):
    M.MODEL.clear()
    M.MODEL.seated, M.MODEL.depth, M.MODEL.theta = seated, depth, math.radians(deg)
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_the_tiers():
    assert scored() == (0.0, False)  # nothing done
    assert scored(seated=True, deg=36.0) == (0.9, True)  # 2 L/min, seated (hands unknown offline: clear)
    assert scored(seated=True, deg=36.0 + 4.0) == (0.9, True)  # 2.22 L/min: within 0.25
    assert scored(seated=True, deg=36.0 + 5.0) == (0.5, False)  # 2.28 L/min: off the line
    assert scored(seated=True, deg=18.0) == (0.5, False)  # 1 L/min
    assert scored(seated=True, deg=0.0) == (0.25, False)  # seated, the flow off
    assert scored(depth=0.008, deg=36.0) == (0.25, False)  # the flow set, the tubing only part way on
    assert scored(deg=36.0) == (0.25, False)  # the flow set, the tubing not on
    assert scored(seated=True, deg=80.0) == (0.0, False)  # 4.4 L/min: far over
    assert scored(depth=0.008, deg=18.0) == (0.0, False)  # part way on, off the line


def test_real_sizes():
    assert 2 * M.KNOB_R == pytest.approx(0.027)
    assert M.FM_TOP - M.FM_BOT == pytest.approx(0.152)  # Ohio Medical 6700: 152 mm tall
    assert M.FM_D + M.SHROUD_D == pytest.approx(0.058)  # 58 mm deep
    assert M.FM_W == pytest.approx(0.056)  # 56 mm wide
    assert 2 * barbfit.CREST_R == pytest.approx(0.006)  # WHO: "6 mm (1/4 inch) barbed fitting"
    assert 2 * M.BORE_R == pytest.approx(0.005)  # WHO: "interior diameter 5mm"
    assert 2 * M.TUBE_R == pytest.approx(0.008)
    assert M.BORE_R - M.NIP_R == pytest.approx(0.0007)  # the bore's clearance round the nipple's core
    assert M.SEAT == pytest.approx(0.015) and M.SEAT < M.FULL_ON == M.NIP_L
    # the tubing's centre of mass rests on the packet, its mouth over the edge
    edge = M.PACKET_AT[0] + M.PACKET_SIZE[0] / 2
    assert M.MOUTH_START[0] - M.TUBE_COM < edge < M.MOUTH_START[0]


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


def test_the_demo_brings_the_tubing_onto_the_nipples_line_and_turns_the_knob_to_2():
    import plan_check  # noqa: F401  (imports cleanly)

    d = M.demo()
    right = ArmPlan(d["right"], station.BASES["right"])
    left = ArmPlan(d["left"], station.BASES["left"])
    ends, t = {}, 0.0
    for w in d["right"]:
        t += w.duration
        ends[w.label] = t
    p, r, _, _ = right.target(ends["onto its tip"] - 1e-6)
    depth, radial, angle = M.on_nipple(*M.tube_held(p, r))
    assert depth == pytest.approx(0.003, abs=1e-4) and radial == pytest.approx(M.AIM_UP, abs=1e-4) and angle < 0.1
    expect = ArmPlan(M.demo(expected=True)["right"], station.BASES["right"])
    p, r, _, _ = expect.target(ends["push it on"] - 1e-6)
    assert M.on_nipple(*M.tube_held(p, r))[0] == pytest.approx(M.FULL_ON, abs=1e-4)
    # aimed further on by the arm's give under the last push (about 6 mm per newton)
    p, r, _, _ = right.target(ends["push it on"] - 1e-6)
    give = M.on_nipple(*M.tube_held(p, r))[0] - M.FULL_ON
    assert 0.006 * (barbfit.push_back(M.FULL_ON) + M.PUSH_MARGIN - 1.0) < give < 0.007 * 18.0
    _, r0, _, _ = left.target(ends["close on it"] - 1e-6)
    _, r1, _, _ = left.target(ends["hold it there"] - 1e-6)
    turned = math.degrees(M.knob_angle(r1 @ r0.T @ M.R_KNOB))
    assert turned == pytest.approx(M.TURN_DEG + M.TURN_LAG, abs=0.05)


@pytest.mark.slow
def test_the_demo_touches_nothing_it_shouldnt():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not out["hits"], out["hits"][:5]
    assert out["min_hand_gap_mm"] > 10.0
