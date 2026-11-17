"""Offline checks for garlic_press.py (no simulator): the press's geometry, the crush model, the scoring, and the
scripted demo against the YAM's reach, the step guard and its real gripper collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import garlic_press as M  # noqa: E402
from fittingkit import BodyState, Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame(pos, rot=np.eye(3), spin=np.zeros(3)) -> Frame:
    pos = np.asarray(pos, float)
    return Frame(pos=pos, rot=rot, vel=np.zeros(3), spin=np.asarray(spin, float), com=pos)


def frames(theta, clove_local, clove_yaw=math.pi / 2, closing_speed=0.0):
    """The press at its start pose with the top handle open ``theta``; the clove's centre at ``clove_local`` in the
    press's frame, its length turned ``clove_yaw`` from x."""
    hinge = M.PRESS_POS + np.array([0.0, 0.0, M.HINGE_Z])
    return {
        "press": frame(M.PRESS_POS),
        "press_top": frame(hinge, rot_x(-theta), spin=np.array([closing_speed, 0.0, 0.0])),
        "clove": frame(M.PRESS_POS + np.asarray(clove_local, float), rot_z(clove_yaw)),
        "paste": frame(M.PARK["paste"]),
    }


IN_CHAMBER = (0.0, -0.023, M.GRATE_TOP + M.CLOVE_H / 2)  # lying flat on the floor, lengthways
ON_RIM = (0.0, -0.023, M.RIM_Z + M.CLOVE_H / 2)
ON_TABLE = (0.0, 0.11, M.CLOVE_H / 2)


def fresh() -> M.GarlicPress:
    m = M.GarlicPress()
    return m


def state(pos, tilt_deg=0.0) -> BodyState:
    q = (math.sin(math.radians(tilt_deg) / 2), 0.0, 0.0, math.cos(math.radians(tilt_deg) / 2))
    p = np.asarray(pos, float)
    return BodyState(pos=p, quat_xyzw=np.array(q), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)


def objects(tilt_deg=0.0, z=M.TOP):
    return {"press": state((*M.PRESS_XY, z), tilt_deg)}


# -- geometry --------------------------------------------------------------------------------------------------------

def test_the_press_is_real_size():
    length = M.HANDLE_END * -1 + 6.0 * M.MM  # the hinge's cheeks reach 6 mm past the axis
    assert length == pytest.approx(0.161, abs=0.006)  # a 15.5 cm press
    assert (M.CH_Y0 - M.CH_Y1, 2 * M.CH_X, M.RIM_Z - M.GRATE_TOP) == pytest.approx((0.034, 0.024, 0.024))


def test_the_clove_fits_the_chamber_only_lengthways():
    assert M.CLOVE_L < M.CH_Y0 - M.CH_Y1 and M.CLOVE_W < 2 * M.CH_X  # lengthways it fits
    assert M.CLOVE_L > 2 * M.CH_X  # across, it doesn't
    # turned 25 degrees off the chamber's length, its footprint is wider than the chamber
    a = math.radians(25.0)
    assert M.CLOVE_L * math.sin(a) + M.CLOVE_W * math.cos(a) > 2 * M.CH_X


def test_the_open_angle_reads_back():
    for deg in (0.0, 12.0, 45.0, 110.0):
        a = math.radians(deg)
        assert M.open_angle(np.eye(3), rot_x(-a)) == pytest.approx(a)
    # tilted press, same handle angle
    t = rot_z(0.3) @ rot_x(0.1)
    assert M.open_angle(t, t @ rot_x(-0.5)) == pytest.approx(0.5)


def test_the_plunger_comes_down_into_the_chamber():
    assert M.plunger_face_z(0.0) == pytest.approx(M.HINGE_Z - M.PL_DEPTH)  # shut: 5 mm above the floor
    assert M.GRATE_TOP < M.plunger_face_z(0.0) < M.GRATE_TOP + M.CRUSHED_GAP
    assert 0.0 < M.THETA_CRUSH < M.THETA_FLAT_CONTACT < math.radians(M.MASH_DEG) < M.OPEN
    assert math.degrees(M.THETA_FLAT_CONTACT) == pytest.approx(20.0, abs=3.0)


def test_the_upper_bar_clears_the_chamber_when_shut():
    bar = M.PRESS_TOP.shapes[0]
    assert M.HINGE_Z + bar.centre[2] - bar.size[2] / 2 > M.RIM_Z


def test_the_handle_rests_open_on_its_stop():
    """Its centre of mass is beyond the hinge's vertical when open, so gravity holds it on its stop."""
    com = rot_x(-M.OPEN) @ np.array(M.PRESS_TOP.com)
    assert com[1] > 0.0  # past upright, on the far side of the hinge from the handles


# -- the model -------------------------------------------------------------------------------------------------------

def test_an_open_press_with_the_clove_seated_does_nothing():
    m = fresh()
    acc = {}
    m.update(frames(M.OPEN, IN_CHAMBER), acc)
    assert m.seated and not m.contact and not m.crushed and m.torque == 0.0 and acc == {}


def test_the_clove_resists_and_yields_then_crushes():
    m = fresh()
    c0 = M.THETA_FLAT_CONTACT
    m.update(frames(c0 + 0.02, IN_CHAMBER), {})
    assert not m.contact
    m.update(frames(c0 - 1e-4, IN_CHAMBER), {})
    assert m.contact and m.yield_at is not None
    # pushed 1 degree past the contact: elastic, below the yield
    acc = {}
    m.update(frames(m.yield_at - math.radians(1.0), IN_CHAMBER), acc)
    assert 0.0 < m.torque < M.T_CRUSH
    assert acc["press_top"][1][0] < 0.0 and acc["press"][1][0] > 0.0  # it pushes the handle open
    # pushed much further: it yields, and pushes back no harder than T_CRUSH
    m.update(frames(M.THETA_CRUSH + 0.05, IN_CHAMBER, closing_speed=0.0), {})
    assert m.torque == pytest.approx(M.T_CRUSH)
    assert not m.crushed
    # let go: the squashed clove springs back only a little
    m.update(frames(M.THETA_CRUSH + 0.05 + M.T_CRUSH / M.K_CLOVE + 0.01, IN_CHAMBER), {})
    assert m.torque == 0.0
    # shut to the crush angle: crushed, the clove taken away, the paste under the floor
    m.update(frames(M.THETA_CRUSH - 1e-4, IN_CHAMBER), {})
    assert m.crushed and m.torque == 0.0
    assert np.allclose(m.poses["clove"][0], M.PARK["clove"])
    assert np.allclose(m.poses["paste"][0], M.PRESS_POS + M.PASTE_LOCAL)


def test_the_force_to_crush_is_f_crush_at_the_grip():
    assert M.T_CRUSH / M.GRIP_L == pytest.approx(M.F_CRUSH)
    assert 5.0 <= M.F_CRUSH <= 20.0


def test_an_empty_press_shut_is_nothing():
    m = fresh()
    for th in (M.OPEN, 0.5, 0.1, 0.0):
        m.update(frames(th, ON_TABLE), {})
    assert not (m.seated or m.contact or m.crushed or m.mashed)


def test_a_clove_on_the_rim_is_mashed():
    m = fresh()
    m.update(frames(math.radians(M.MASH_DEG) + 0.05, ON_RIM), {})
    assert not m.mashed and not m.seated
    m.update(frames(math.radians(M.MASH_DEG) - 0.05, ON_RIM), {})
    assert m.mashed and not m.crushed


def test_a_clove_across_the_chamber_on_the_walls_is_not_seated():
    # lying across (its length along x), it rests on the side walls: centre above the rim
    assert not M.seated(np.array([0.0, -0.023, M.RIM_Z + M.CLOVE_H / 2]))
    assert M.seated(np.array(IN_CHAMBER))


def test_clear_resets_the_log():
    m = fresh()
    m.update(frames(M.THETA_FLAT_CONTACT - 0.01, IN_CHAMBER), {})
    m.update(frames(0.0, IN_CHAMBER), {})
    assert m.crushed
    m.clear()
    assert not (m.crushed or m.contact or m.mashed) and m.yield_at is None and "clove" not in m.poses


# -- scoring ---------------------------------------------------------------------------------------------------------

@pytest.fixture
def model():
    M.MODEL.clear()
    yield M.MODEL
    M.MODEL.clear()


def test_score_tiers(model):
    assert M.score(objects()) == 0.0
    model.update(frames(M.OPEN, IN_CHAMBER), {})
    assert M.score(objects()) == 0.25 and not M.success(objects())
    model.update(frames(M.THETA_FLAT_CONTACT - 0.005, IN_CHAMBER), {})
    assert M.score(objects()) == 0.5
    model.update(frames(M.THETA_CRUSH - 0.001, IN_CHAMBER), {})
    assert M.score(objects()) == 1.0 and M.success(objects())


def test_the_press_knocked_over_scores_nothing(model):
    model.update(frames(M.OPEN, IN_CHAMBER), {})
    assert M.score(objects(tilt_deg=30.0)) == 0.0
    assert M.score(objects(z=M.TOP + 0.05)) == 0.0
    model.update(frames(0.0, IN_CHAMBER), {})
    assert model.crushed and not M.success(objects(tilt_deg=30.0))


def test_a_clove_taken_out_again_loses_its_credit(model):
    model.update(frames(M.THETA_FLAT_CONTACT - 0.005, IN_CHAMBER), {})
    assert M.score(objects()) == 0.5
    model.update(frames(M.OPEN, ON_TABLE), {})
    assert M.score(objects()) == 0.0


def test_mashing_on_the_rim_scores_nothing_even_if_seated_later(model):
    model.update(frames(0.2, ON_RIM), {})
    model.update(frames(M.OPEN, IN_CHAMBER), {})
    assert model.mashed and M.score(objects()) == 0.0


# -- the demo --------------------------------------------------------------------------------------------------------

GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = np.array([plan.action7(t)[0] for t in np.arange(0.0, plan.duration, 0.05)])
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(acts, axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_both_arms_finish_together():
    d = M.demo()
    assert sum(w.duration for w in d["left"]) == pytest.approx(sum(w.duration for w in d["right"]), abs=1.0)


def test_the_demo_fits_the_run_limit():
    d = M.demo()
    assert max(sum(w.duration for w in d[a]) for a in d) * 20 < 1200 - 100


def test_the_hands_keep_clear_of_the_press_and_each_other():
    import plan_check

    out = plan_check.run(verbose=False)
    assert out["left_lower_min_mm"] > 2.0 and out["right_lower_min_mm"] > 2.0
    assert out["left_upper_min_mm"] > 3.0 and out["right_upper_min_mm"] > 3.0
    assert out["arms_min_mm"] > 20.0
