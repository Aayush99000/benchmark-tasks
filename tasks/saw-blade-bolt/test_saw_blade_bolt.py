"""Offline checks for saw_blade_bolt.py (no simulator): the lock and bolt model, the scoring, the geometry, the demo's
reach, and the demo's hand paths replayed through the model."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import saw_blade_bolt as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402

DT = 1.0 / 240.0
SEAT = M.AXIS_TOP - np.array([0.0, 0.0, M.KEY_DEPTH])


def key_frame(tip, psi, spin=0.0) -> Frame:
    tip = np.asarray(tip, float)
    return Frame(pos=tip, rot=M.key_rotation(psi), vel=np.zeros(3), spin=np.array([0.0, 0.0, spin]), com=tip)


def button_frame(press) -> Frame:
    p = M.BUTTON_REST - np.array([0.0, 0.0, press])
    return Frame(pos=p, rot=np.eye(3), vel=np.zeros(3), spin=np.zeros(3), com=p)


def run(model, psis, tip=SEAT, press=0.0, spin=-1.0) -> dict:
    acc = {}
    for psi in psis:
        acc = {}
        model.update({"key": key_frame(tip, psi, spin), "button": button_frame(press)}, acc)
    return acc


def turn(a, b, step=0.25):
    n = max(1, int(abs(b - a) / step))
    return list(np.linspace(a, b, n + 1))


@pytest.fixture
def model():
    m = M.SawBladeBolt()
    m.clear()
    return m


# -- the model --------------------------------------------------------------------------------------------------------

def test_the_key_goes_into_the_socket_lined_up(model):
    run(model, [M.KEY_IN_DEG])
    assert model.engaged and model.engagements == 1


def test_a_key_off_its_flats_is_pushed_back_out(model):
    acc = run(model, [M.KEY_IN_DEG + 30.0])
    assert not model.engaged
    assert acc["key"][0][2] > 0.0


@pytest.mark.parametrize("off, ok", [(1.0e-3, True), (2.5e-3, False)])
def test_the_key_must_be_on_the_axis(model, off, ok):
    run(model, [M.KEY_IN_DEG], tip=SEAT + np.array([off, 0.0, 0.0]))
    assert model.engaged is ok


def test_without_the_lock_the_key_turns_the_blade_and_bolt_together(model):
    run(model, [M.KEY_IN_DEG])
    run(model, turn(M.KEY_IN_DEG, M.KEY_IN_DEG - 120.0))
    assert model.tightened_deg() == pytest.approx(0.0, abs=1e-6)
    assert model.spindle == pytest.approx(M.SPINDLE0_DEG - 120.0)


def test_the_lock_held_in_catches_at_the_notch_then_the_bolt_turns(model):
    run(model, [M.KEY_IN_DEG], press=M.RIDE)
    assert not model.locked  # 25 degrees short of the notch, the pin rides on the gear
    run(model, turn(M.KEY_IN_DEG, M.KEY_IN_DEG - 115.0), press=M.RIDE)
    assert model.spindle == pytest.approx(0.0)
    assert model.tightened_deg() == pytest.approx(90.0, abs=0.3)


def test_the_lock_only_goes_in_at_a_notch(model):
    run(model, [M.KEY_IN_DEG], press=M.BUTTON_TRAVEL)
    assert not model.locked
    acc = {}
    model.update({"key": key_frame(SEAT, M.KEY_IN_DEG), "button": button_frame(M.BUTTON_TRAVEL)}, acc)
    assert acc["button"][0][2] > 0.0  # the gear's face pushes the button back to where the pin rides


def test_the_thread_resists_tightening_more_the_further_it_goes(model):
    run(model, [M.KEY_IN_DEG], press=M.RIDE)
    run(model, turn(M.KEY_IN_DEG, M.KEY_IN_DEG - 25.0 - 10.0), press=M.RIDE)
    low = run(model, [M.KEY_IN_DEG - 35.25], press=M.RIDE)["key"][1][2]
    run(model, turn(M.KEY_IN_DEG - 35.25, M.KEY_IN_DEG - 115.0), press=M.RIDE)
    high = run(model, [M.KEY_IN_DEG - 115.25], press=M.RIDE)["key"][1][2]
    assert 0.0 < low < high  # +z: against the clockwise turn
    assert high == pytest.approx(M.SawBladeBolt().torque_quarter * 90.25 / 90.0 + 0.01, rel=0.05)


def test_turning_back_with_the_lock_in_loosens_it(model):
    run(model, [M.KEY_IN_DEG], press=M.RIDE)
    run(model, turn(M.KEY_IN_DEG, M.KEY_IN_DEG - 115.0), press=M.RIDE)
    run(model, turn(M.KEY_IN_DEG - 115.0, M.KEY_IN_DEG - 75.0), press=M.RIDE, spin=1.0)
    assert model.tightened_deg() == pytest.approx(50.0, abs=0.3)


def test_letting_go_of_the_lock_frees_the_spindle(model):
    run(model, [M.KEY_IN_DEG], press=M.RIDE)
    run(model, turn(M.KEY_IN_DEG, M.KEY_IN_DEG - 60.0), press=M.RIDE)
    b = model.tightened_deg()
    run(model, turn(M.KEY_IN_DEG - 60.0, M.KEY_IN_DEG - 100.0), press=0.0)
    assert model.tightened_deg() == pytest.approx(b, abs=1e-6)


def test_lifting_the_key_takes_it_off(model):
    run(model, [M.KEY_IN_DEG])
    run(model, [M.KEY_IN_DEG], tip=M.AXIS_TOP + np.array([0.0, 0.0, 1.0e-3]))
    assert not model.engaged


# -- scoring ----------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("beta, expect", [(0.0, 0.0), (14.0, 0.0), (20.0, 0.25), (50.0, 0.5), (72.0, 0.9),
                                          (90.0, 0.9), (108.0, 0.9), (120.0, 0.5), (144.0, 0.5), (150.0, 0.0),
                                          (-20.0, 0.0)])
def test_the_tiers(beta, expect):
    M.MODEL.clear()
    M.MODEL.beta = beta
    assert M.score({}) == expect
    M.MODEL.clear()


def test_success_needs_the_key_off_put_down_and_the_lock_released():
    M.MODEL.clear()
    M.MODEL.beta = 90.0
    M.MODEL.key_off_by, M.MODEL.press = 0.1, 0.0
    M.MODEL.key_low = M.TOP + 1.0 * M.MM
    assert M.success({})
    M.MODEL.press = 2.0 * M.MM
    assert not M.success({})
    M.MODEL.press, M.MODEL.engaged = 0.0, True
    assert not M.success({})
    M.MODEL.engaged, M.MODEL.key_low = False, M.TOP + 30.0 * M.MM  # still in the hand, off the table
    assert not M.success({})
    M.MODEL.clear()


def test_a_gripper_still_on_the_key_is_not_put_down():
    m = M.SawBladeBolt()
    m.clear()
    down = np.array([0.30, 0.33, M.TOP])
    m.update({"key": key_frame(down, 0.0), "button": button_frame(0.0)}, {})
    assert m.key_down(np.zeros((0, 3)))
    assert not m.key_down(down + np.array([[M.GRIP_X, 8.0 * M.MM, M.KEY_ARM_Z]]))  # a finger beside the long arm
    assert m.key_down(down + np.array([[M.GRIP_X, 0.0, M.KEY_ARM_Z + 60.0 * M.MM]]))  # the hand clear above


# -- the geometry -----------------------------------------------------------------------------------------------------

def test_the_guards_cover_the_teeth_all_round():
    ring = [s for s in M.SAW.shapes if s.color in (M.GUARD, M.LOWER_GUARD) and s.kind == "box"
            and abs(s.size[2] - 28.0 * M.MM) < 1e-6]
    assert len(ring) == 18
    assert 76.0 * M.MM < M.BLADE_R < 101.0 * M.MM


def test_the_key_fits_the_socket_and_the_cap_is_wider_than_fingertips():
    assert 3.0 * M.MM < M.SOCKET_HOLE and M.KEY_AF / math.sqrt(3) < M.SOCKET_HOLE  # its corners clear
    assert M.BUTTON_R * 2 >= 24.0 * M.MM


def test_the_key_is_real_size():
    lo, hi = M.KEY.bounds_local()
    assert 0.095 < hi[0] - lo[0] < 0.11 and 0.03 < hi[2] - lo[2] < 0.04


# -- the demo -------------------------------------------------------------------------------------------------------

GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = np.array([plan.action7(t)[0] for t in np.arange(0.0, plan.duration, 0.05)])
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(acts, axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_the_hands_stay_apart():
    left = ArmPlan(M.demo()["left"], station.BASES["left"])
    right = ArmPlan(M.demo()["right"], station.BASES["right"])
    gap = min(np.linalg.norm(left.target(t)[0] - right.target(t)[0]) for t in np.arange(0.0, 20.0, 0.05))
    assert gap > 0.12, gap


def test_the_fingers_close_on_the_key_clear_of_the_holder():
    tcp = np.array(M.key_tcp(M.KEY_START, 0.0))
    pad = tcp[2] - M.PAD_PAST_TCP
    assert pad > M.TOP + M.HOLDER_BASE + 5.0 * M.MM
    grip_x = tcp[0] - M.HOLDER_XY[0]
    assert 25.0 * M.MM < grip_x < 80.0 * M.MM  # between the posts


def test_the_open_gripper_comes_down_on_the_key_clear_of_the_guard():
    """The open gripper is about 42 mm either side of the TCP across its fingers, 40-50 mm above the TCP (the
    finger-extent measurement plus 15 mm a side open); at the grip that height is the guard's top. Sim run 1 landed
    on it with the holder 33 mm from the guard."""
    tcp = np.array(M.key_tcp(M.KEY_START, 0.0))
    near = M.AXIS_XY[1] + M.GUARD_OUT  # the guard's edge nearest the holder (y)
    assert tcp[1] - 42.0 * M.MM > near + 10.0 * M.MM


def test_the_key_is_carried_over_the_guard_not_through_it():
    """Between gripping it and laying it down, wherever the key's tip is over the guard ring (with the short arm's 3 mm
    radius), it is at least 15 mm above the guard's top. Sim run 2 caught the guard and knocked the key over."""
    left = ArmPlan(M.demo()["left"], station.BASES["left"])
    labels = [w.label for w in left.waypoints]
    t0 = sum(w.duration for w in left.waypoints[:labels.index("grip the key") + 1])
    t1 = sum(w.duration for w in left.waypoints[:labels.index("let go of the key") + 1])
    guard_top = M.BLADE_Z + 14.0 * M.MM
    r0 = grip_rotation(0.0)
    tip_in_tcp = r0.T @ (M.KEY_START - np.array(M.key_tcp(M.KEY_START, 0.0)))
    for t in np.arange(t0, t1, 0.02):
        pos, rot, _, _ = left.target(t)
        tip = pos + rot @ tip_in_tcp
        r = np.hypot(*(tip[:2] - M.AXIS_XY))
        if M.GUARD_IN - 3.0 * M.MM < r < M.GUARD_OUT + 3.0 * M.MM:
            assert tip[2] > guard_top + 15.0 * M.MM, (t, r, tip[2] - guard_top)


def test_the_demo_tightens_the_bolt_a_quarter_turn_held_rigidly():
    """Replay both hands' paths through the model: the key rigid in the left hand from its grip, the button pressed as
    far as the right hand's pads reach (to its travel, and to where the pin rides away from a notch)."""
    model = M.SawBladeBolt()
    model.clear()
    left = ArmPlan(M.demo()["left"], station.BASES["left"])
    right = ArmPlan(M.demo()["right"], station.BASES["right"])
    labels = [w.label for w in left.waypoints]
    t_grip = sum(w.duration for w in left.waypoints[:labels.index("grip the key") + 1])
    t_let_go = sum(w.duration for w in left.waypoints[:labels.index("let go of the key") + 1])
    r0 = grip_rotation(0.0)
    tcp0 = np.array(M.key_tcp(M.KEY_START, 0.0))
    tip_in_tcp = r0.T @ (M.KEY_START - tcp0)
    rot_in_tcp = r0.T @ M.key_rotation(0.0)
    prev = None
    for t in np.arange(0.0, max(left.duration, right.duration) + 1.0, DT):
        if t < t_grip or t > t_let_go:
            tip, rot = (M.KEY_START, M.key_rotation(0.0)) if t < t_grip else (np.array([*M.KEY_DOWN_XY, M.TOP]), np.eye(3))
        else:
            pos, rot_t, _, _ = left.target(t)
            tip, rot = pos + rot_t @ tip_in_tcp, rot_t @ rot_in_tcp
        psi = math.atan2(rot[1, 0], rot[0, 0])
        spin = 0.0 if prev is None else _wrap_pi(psi - prev) / DT
        prev = psi
        rp = right.target(t)[0]
        press = 0.0
        if np.hypot(*(rp[:2] - M.LOCK_XY)) < M.BUTTON_R:
            press = min(max(M.BUTTON_TOP_REST - (rp[2] - M.PAD_PAST_TCP), 0.0), M.BUTTON_TRAVEL)
            if not model.at_notch():
                press = min(press, M.RIDE)
        fk = Frame(pos=np.asarray(tip, float), rot=rot, vel=np.zeros(3), spin=np.array([0.0, 0.0, spin]),
                   com=np.asarray(tip, float))
        model.update({"key": fk, "button": button_frame(press)}, {})
    assert model.engagements == 1
    assert M.WINDOW[0] <= model.tightened_deg() <= M.WINDOW[1], model.tightened_deg()
    assert model.key_clear() and model.lock_released() and model.key_down(np.zeros((0, 3)))


def _wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
