"""Offline checks for folding_step_stool.py (no simulator): the legs' angle, the spring locks, the load, the scoring,
the sizes, and the scripted demo against the YAM's real reach, step guard and collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "hardware"))
import folding_step_stool as M  # noqa: E402
import stool_dimensions as HW  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402


def frame(pos, rot=np.eye(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.zeros(3), spin=np.zeros(3), com=p)


def leg_rot(leg, open_deg, top_rot=np.eye(3)):
    return M.leg_pose(0.0, 1, leg, open_deg)[1] if np.allclose(top_rot, np.eye(3)) else top_rot @ (
        M.rot_x(-math.radians(open_deg)) if leg == "leg_a" else M.rot_x(math.radians(open_deg)))


class Rig:
    def __init__(self):
        self.m = M.Locks()
        self.m.log = self.m.trace = None

    def step(self, open_a, open_b, top_z=M.TOP_Z, top_rot=np.eye(3), n=24):
        acc = {}
        for _ in range(n):
            acc = {}
            top = M.TOP_AT.copy()
            top[2] = top_z
            self.m.update({"stool_top": frame(top, top_rot), "leg_a": frame(top, leg_rot("leg_a", open_a, top_rot)),
                           "leg_b": frame(top, leg_rot("leg_b", open_b, top_rot))}, acc)
        return acc


def test_the_legs_angle_reads_folded_down_and_open():
    for leg in ("leg_a", "leg_b"):
        for opening in (0.0, 45.0, 90.0, 100.0):
            assert M.leg_angle_deg(np.eye(3), leg_rot(leg, opening), leg) == pytest.approx(opening, abs=1e-6)
        down = M._leg_dir(np.eye(3), leg, 90.0)
        assert np.allclose(down, [0.0, 0.0, -1.0])
        splay = M._leg_dir(np.eye(3), leg, 100.0)
        assert splay[1] * M.SIDE[leg] > 0  # at its stop the foot is outwards, towards its own end


def test_short_of_the_lock_the_spring_folds_it_back_and_past_98_it_locks():
    r = Rig()
    acc = r.step(60.0, 0.0)
    assert "leg_a" not in acc  # no spring below 80 degrees
    acc = r.step(95.0, 0.0)
    a = M.opening_axis(np.eye(3), "leg_a")
    assert acc["leg_a"][1] @ a < 0  # folding it back
    assert not r.m.locked["leg_a"]
    acc = r.step(98.5, 0.0)
    assert r.m.locked["leg_a"] and not r.m.locked["leg_b"]
    assert acc["leg_a"][1] @ a == pytest.approx(M.HOLD_TORQUE)  # held open against its stop
    acc = r.step(90.0, 0.0)
    assert r.m.locked["leg_a"] and acc["leg_a"][1] @ a > 0  # once locked it stays locked


def test_the_push_to_lock_is_light_and_the_lock_outholds_the_load():
    just_short = HW.fold_back_torque_nm(M.LATCH_DEG - 0.01)
    assert 0.05 < just_short < 0.3  # N m: under 3 N at the foot
    load_on_a_leg = M.LOAD_N / 2 * M.LEG_L * math.sin(math.radians(M.OPEN_DEG - 90.0))
    assert M.HOLD_TORQUE > 4 * load_on_a_leg


def test_the_load_comes_on_only_standing_level_at_its_height_with_the_hands_off():
    r = Rig()
    stand_z = M.TOP + M.STAND_H
    acc = r.step(100.0, 100.0, top_z=stand_z)
    assert r.m.loaded and acc["stool_top"][0][2] == pytest.approx(-M.LOAD_N)
    r.step(100.0, 100.0, top_z=M.TOP_Z)
    assert not r.m.loaded  # folded height
    r.step(100.0, 100.0, top_z=stand_z, top_rot=M.rot_x(math.radians(12.0)))
    assert not r.m.loaded  # tilted
    r.m.hands_off = False
    r.step(100.0, 100.0, top_z=stand_z)
    assert not r.m.loaded  # a hand still on it


def test_under_the_load_a_leg_that_isnt_locked_gives_way_and_a_locked_one_holds():
    r = Rig()
    stand_z = M.TOP + M.STAND_H
    acc = r.step(100.0, 81.0, top_z=stand_z)  # the +y leg sprang back short of its lock
    assert r.m.loaded and r.m.gave_way["leg_b"] and not r.m.gave_way["leg_a"]
    b = M.opening_axis(np.eye(3), "leg_b")
    assert acc["leg_b"][1] @ b == pytest.approx(-M.GIVE_WAY_TORQUE)
    a = M.opening_axis(np.eye(3), "leg_a")
    assert acc["leg_a"][1] @ a == pytest.approx(M.HOLD_TORQUE)


def scored(rig):
    M.MODEL.__dict__.update({k: v for k, v in rig.m.__dict__.items() if k not in ("log", "trace")})
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_the_tiers():
    stand_z = M.TOP + M.STAND_H
    r = Rig()
    r.step(100.0, 100.0, top_z=stand_z)
    assert scored(r) == (0.9, True)
    r = Rig()
    r.step(100.0, 100.0, top_z=stand_z + 0.05)  # both locked, still held up
    assert scored(r) == (0.5, False)
    r = Rig()
    r.step(100.0, 95.0, top_z=stand_z)  # one locked, the other only looks open
    assert scored(r) == (0.25, False)
    r = Rig()
    r.step(95.0, 95.0, top_z=stand_z)  # both look open, neither locked
    assert scored(r) == (0.0, False)


def test_real_sizes_and_fits():
    assert 2 * M.LEG_L < 2 * M.HINGE_Y  # folded, the two legs don't overlap
    assert M.FOOT_L > M.TOP_D + 0.05  # the foot rails stick out past the top, to be pushed
    assert M.GRIP_W < 0.094 - 0.015  # the grip rib fits between the open pads with room
    assert 0.10 < M.STAND_H + M.TOP_T / 2 < 0.15  # a low step stool, 10 to 15 cm high
    assert 0.5 < M.TOP_MASS + 2 * M.LEG_MASS < 1.2
    assert M.SPRING_FROM_DEG < 90.0 < M.LATCH_DEG < M.OPEN_DEG


def test_the_demo_plan_puts_the_hanging_legs_clear_of_the_table():
    assert M.turned(M.PIVOT_1, M.LIFT_1, M.hinge_local("leg_a"))[2] - M.TOP > M.LEG_L + 0.01
    assert M.turned2(M.LIFT_2, M.hinge_local("leg_b"))[2] - M.TOP > M.LEG_L + 0.01
    assert abs(math.degrees(M.LAND_2)) < 2.0  # let down, it ends level


GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = [plan.action7(t)[0] for t in np.arange(0.0, plan.duration, 0.05)]
    assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 3.0, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(np.array(acts), axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_both_arms_follow_the_same_timeline():
    d = M.demo()
    assert [w.label for w in d["left"]] == [w.label for w in d["right"]]
    assert len(M.PLAN) == len(d["left"])


@pytest.mark.slow
def test_the_demo_touches_the_stool_only_where_it_means_to():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not out["hits"], out["hits"][:5]
    assert out["min_hand_gap_mm"] > 10.0


def test_the_simulated_stool_is_the_buildable_one():
    mm = M.MM
    for sim, hw in ((M.TOP_D, HW.TOP_D), (M.TOP_L, HW.TOP_L), (M.TOP_T, HW.TOP_T), (M.HINGE_Y, HW.HINGE_Y),
                    (M.KNUCKLE_R, HW.KNUCKLE_R), (M.LEG_L, HW.LEG_L), (M.LEG_W, HW.LEG_W), (M.LEG_T, HW.LEG_T),
                    (M.FOOT_L, HW.FOOT_L), (M.FOOT_T, HW.FOOT_T), (M.GRIP_W, HW.RIB_W), (M.GRIP_T, HW.RIB_T),
                    (M.GRIP_H, HW.RIB_H), (M.RIB_LOCAL[1][1], HW.RIB_Y), (M.HINGE_Z - M.TOP_Z, HW.HINGE_Z)):
        assert sim == pytest.approx(hw * mm, abs=1e-9)
    assert (M.OPEN_DEG, M.LATCH_DEG, M.SPRING_FROM_DEG) == (HW.OPEN_DEG, HW.LATCH_DEG, HW.SPRING_FROM_DEG)
    assert M.STAND_H + M.TOP_T / 2 == pytest.approx(0.1259, abs=0.0005)  # 12.6 cm to the top's face, standing


def test_the_latch_spring_from_its_catalogue_numbers():
    # Lee Spring LC 026C 05 M: free 15.88 mm, 2.189 N/mm, solid 5.16 mm
    assert HW.SPRING_PRELOAD_N == pytest.approx(2.189 * (15.88 - 11.5))
    assert HW.SPRING_INSTALLED - HW.RELEASE_MM > HW.SPRING_SOLID + 1.0  # released, still well short of solid
    assert HW.fold_back_torque_nm(79.9) == 0.0 and HW.fold_back_torque_nm(98.0) == 0.0
    assert HW.fold_back_torque_nm(80.0) == pytest.approx(0.102, abs=0.001)
    assert HW.fold_back_torque_nm(97.99) == pytest.approx(0.165, abs=0.001)
    assert HW.S_AT_LATCH > 3.0  # the bolt drops 3.5 mm behind the wedge's high end


def test_the_sim_pushes_the_leg_back_with_the_latch_springs_torque():
    r = Rig()
    acc = r.step(90.0, 0.0)
    a = M.opening_axis(np.eye(3), "leg_a")
    assert acc["leg_a"][1] @ a == pytest.approx(-HW.fold_back_torque_nm(90.0))


def test_the_cad_parts_move_as_the_simulator_says():
    pytest.importorskip("manifold3d")
    import build_parts as B

    top, leg, bolt = B.top(), B.leg(), B.bolt()
    out = B.check(top, leg, bolt)
    over = dict(out["top_overlap"])
    assert max(v for d, v in over.items() if d <= 100.0) < 1e-3  # the leg turns clear of the top up to its stop
    assert over[101.0] > 1.0  # and stops there, on the pocket's end wall
    for d, push in out["push"]:
        if d < 78.0 or d >= 98.0:
            assert push == 0.0, d  # the bolt is out: short of the wedge, and dropped behind it from 98
        elif d >= 80.0:
            assert push == pytest.approx(HW.bolt_push_mm(d), abs=0.05), d
    assert out["lowest_folded"] == pytest.approx(0.0, abs=1e-6)  # folded, it rests on its knuckles and ears
    assert out["bolt_in_channel"] < 1e-3
