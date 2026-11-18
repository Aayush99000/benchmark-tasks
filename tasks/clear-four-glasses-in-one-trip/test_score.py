"""Scoring tests, no simulator: every band and each shortcut.  python -m pytest -q test_score.py
(run in the Python environment that has yam_arena; Isaac Sim is not started)."""
import math
import os
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import task as t  # noqa: E402
from glass_bussing import mech as M  # noqa: E402

DT = 0.05
FLOOR_Z = t.TOP + M.TUB_FLOOR
SLOTS = {"g0": (0.266, 0.119), "g1": (0.334, 0.071), "g2": (0.266, -0.119), "g3": (0.334, -0.071)}  # where the demo stands them
INSIDE = [(0.0, 0.0, 0.020), (0.0, 0.0, 0.046), (0.0, 0.0, 0.072), (0.0, 0.0, 0.090)]  # three cubes and the straw's middle, tumbler frame
AWAY = {"left": (0.20, 0.33, t.TOP + 0.20), "right": (0.20, -0.33, t.TOP + 0.20)}


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def body(p, R=None, v=(0.0, 0.0, 0.0), w=(0.0, 0.0, 0.0)):
    p = np.asarray(p, float)
    return NS(pos=p, quat_xyzw=np.asarray(t._quat(np.eye(3) if R is None else R)), lin_vel=np.asarray(v, float), ang_vel=np.asarray(w, float), low=p, high=p)


class World:
    """Bodies placed by hand. A tumbler's ice and straw ride inside it unless moved separately."""

    def __init__(self):
        t.reset_latch()
        self.g = {g: (np.array((*t.GLASS_XY[g], t.TOP)), np.eye(3)) for g in t.GLASSES}
        self.loose = {}  # content name -> world position, for contents that have left their tumbler
        self.vel = {}
        self.tcp = {a: np.array(p) for a, p in AWAY.items()}
        self.tub = (np.array((*t.TUB_XY, t.TOP)), np.eye(3))

    def objects(self):
        o = {"tub": body(*self.tub)}
        for g, (p, R) in self.g.items():
            o[g] = body(p, R, *self.vel.get(g, ((0, 0, 0), (0, 0, 0))))
            for n, q in zip(t.CONTENTS[g], INSIDE):
                o[n] = body(self.loose.get(n, p + R @ np.array(q)))
        for a, p in self.tcp.items():
            o[f"{a}_tcp"] = body(p)
        return o

    def step(self, n=1):
        for _ in range(n):
            t.note_step(self.objects(), DT)
        return self

    def wait(self, s):
        return self.step(int(round(s / DT)))

    def score(self):
        return t.score(self.objects())

    # -- moves -------------------------------------------------------------------------------------------------------
    def stand(self, g, xy=None, z=FLOOR_Z, R=None):
        xy = SLOTS[g] if xy is None else xy
        self.g[g] = (np.array((xy[0], xy[1], z)), np.eye(3) if R is None else R)

    def carry(self, arm, glasses, lean_deg=25.0):
        """One gripper lifts ``glasses`` together, takes them over the tub's rim and stands them in their slots."""
        R = rot_y(math.radians(lean_deg))
        s = 1 if arm == "left" else -1
        for y in (0.30, 0.22, 0.15, 0.10):  # across the long wall, whose rim's outer edge is at 0.1945
            for k, g in enumerate(glasses):
                self.g[g] = (np.array((0.28 + 0.07 * k, s * y, t.TOP + 0.17)), R)
            self.tcp[arm] = np.array((0.315, s * y, t.TOP + 0.31))
            self.step(4)
        for g in glasses:
            self.stand(g)
        self.step(4)
        self.tcp[arm] = np.array(AWAY[arm])
        self.step(4)
        return self

    def all_in_one_trip(self):
        self.carry("left", ("g0", "g1"))
        self.carry("right", ("g2", "g3"))
        return self


# -- the bands ----------------------------------------------------------------------------------------------------------
def test_nothing_done_scores_zero():
    w = World().wait(3.0)
    assert w.score() == 0.0 and not t.success(w.objects())


def test_full_score_needs_all_four_in_one_trip_hands_clear_and_still():
    w = World().all_in_one_trip()
    assert t.carries() == {"left": 1, "right": 1, "none": 0}
    assert w.score() == 0.5  # the hands have only just stopped
    w.wait(1.0)
    assert w.score() == 1.0 and t.success(w.objects())


def test_two_delivered():
    w = World().carry("left", ("g0", "g1")).wait(1.5)
    assert w.score() == 0.25


def test_three_delivered():
    w = World().carry("left", ("g0", "g1")).carry("right", ("g2",)).wait(1.5)
    assert w.score() == 0.25


def test_one_delivered_scores_zero():
    w = World().carry("left", ("g0",)).wait(1.5)
    assert w.score() == 0.0


def test_one_at_a_time_is_half():
    w = World()
    for arm, g in (("left", "g0"), ("right", "g2"), ("left", "g1"), ("right", "g3")):
        w.carry(arm, (g,)).wait(1.2)
    w.wait(1.5)
    assert t.carries() == {"left": 2, "right": 2, "none": 0}
    assert len(t.delivered(w.objects())) == 4 and w.score() == 0.5


def test_one_hand_in_one_trip_the_other_in_two_is_half():
    w = World().carry("left", ("g0", "g1")).carry("right", ("g2",)).wait(1.2).carry("right", ("g3",)).wait(1.5)
    assert t.carries() == {"left": 1, "right": 2, "none": 0} and w.score() == 0.5


def test_one_hand_carrying_all_four_in_two_trips_is_half():
    w = World()
    w.g["g2"] = (np.array((0.22, 0.25, t.TOP)), np.eye(3))  # as if moved across the table first
    w.g["g3"] = (np.array((0.40, 0.25, t.TOP)), np.eye(3))
    w.carry("left", ("g0", "g1")).wait(1.2).carry("left", ("g2", "g3")).wait(1.5)
    assert t.carries()["left"] == 2 and w.score() == 0.5


# -- carries ------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("gap_s, n", [(0.5, 1), (0.9, 1), (1.2, 2), (3.0, 2)])
def test_tumblers_over_the_rim_within_one_second_are_one_carry(gap_s, n):
    w = World()
    w.tcp["left"] = np.array((0.30, 0.15, t.TOP + 0.30))
    w.g["g0"] = (np.array((0.28, 0.15, t.TOP + 0.17)), np.eye(3))
    w.step()
    w.wait(gap_s - DT)
    w.g["g1"] = (np.array((0.35, 0.15, t.TOP + 0.17)), np.eye(3))
    w.step()
    assert t.carries()["left"] == n


def test_a_tumbler_that_goes_in_with_no_gripper_near_is_not_one_trip():
    w = World().carry("left", ("g0", "g1")).carry("right", ("g2",))
    w.stand("g3")  # it is in the tub, but no gripper brought it
    w.wait(1.5)
    assert t.carries()["none"] == 1 and len(t.delivered(w.objects())) == 4 and w.score() == 0.5


def test_taking_a_tumbler_out_and_bringing_it_back_is_a_second_carry():
    w = World().all_in_one_trip().wait(1.2)
    w.g["g0"] = (np.array((0.22, 0.32, t.TOP)), np.eye(3))
    w.wait(1.2)
    w.carry("left", ("g0",)).wait(1.5)
    assert t.carries()["left"] == 2 and w.score() == 0.5


def test_a_gripper_reaching_into_the_tub_empty_is_not_a_carry():
    w = World()
    w.tcp["left"] = np.array((0.30, 0.10, t.TOP + 0.20))
    w.wait(1.0)
    w.tcp["left"] = np.array(AWAY["left"])
    w.wait(1.0)
    assert t.carries() == {"left": 0, "right": 0, "none": 0}
    w.all_in_one_trip().wait(1.5)
    assert w.score() == 1.0


def test_setting_a_tumbler_straight_after_the_trip_inside_the_tub_costs_nothing():
    w = World().all_in_one_trip()
    w.tcp["left"] = np.array((0.27, 0.12, t.TOP + 0.20))  # back in over the rim, empty-handed
    w.wait(0.5)
    w.tcp["left"] = np.array(AWAY["left"])
    w.wait(1.5)
    assert w.score() == 1.0


# -- latched faults -----------------------------------------------------------------------------------------------------
def test_a_knocked_over_tumbler_zeroes_the_trial_even_if_stood_up_again():
    w = World()
    w.g["g1"] = (np.array((0.40, 0.32, t.TOP + 0.03)), rot_y(math.radians(80.0)))
    w.step()
    w.g["g1"] = (np.array((*t.GLASS_XY["g1"], t.TOP)), np.eye(3))
    w.all_in_one_trip().wait(1.5)
    assert len(t.delivered(w.objects())) == 4 and w.score() == 0.0


@pytest.mark.parametrize("deg, tipped", [(25.0, False), (44.0, False), (46.0, True)])
def test_knocked_over_means_past_45_degrees(deg, tipped):
    w = World()
    w.g["g0"] = (np.array((0.22, 0.32, t.TOP + 0.05)), rot_y(math.radians(deg)))
    w.step()
    assert t._trial["tipped"] is tipped


def test_the_carry_lean_does_not_count_as_knocked_over():
    w = World().all_in_one_trip().wait(1.5)
    assert not t._trial["tipped"] and w.score() == 1.0


def test_a_body_off_the_table_zeroes_the_trial():
    w = World().all_in_one_trip()
    w.loose["g2_i0"] = np.array((0.9, 0.9, t.TOP - 0.5))
    w.step()
    w.loose.clear()
    w.wait(1.5)
    assert t._trial["off"] and w.score() == 0.0


def test_nesting_is_not_one_trip():
    w = World()
    w.g["g0"] = (np.array((*t.GLASS_XY["g1"], t.TOP + 0.05)), np.eye(3))  # the near tumbler set down inside the far one
    w.step()
    assert t._trial["nested"]
    w.all_in_one_trip().wait(1.5)
    assert t.carries() == {"left": 1, "right": 1, "none": 0} and w.score() == 0.5


def test_rim_to_rim_is_not_nesting():
    w = World()
    w.g["g0"] = (np.array((t.FAR_X - M.RIM_D, t.ROW_Y, t.TOP)), np.eye(3))
    w.step()
    lean_a = M.TAPER
    w.g["g0"] = (np.array((t.FAR_X - M.BASE_D, t.ROW_Y, t.TOP + 0.10)), rot_y(-lean_a))  # hanging base to base
    w.g["g1"] = (np.array((t.FAR_X, t.ROW_Y, t.TOP + 0.10)), rot_y(lean_a))
    w.step()
    assert not t._trial["nested"]


# -- spills -------------------------------------------------------------------------------------------------------------
def test_ice_on_the_table_zeroes_the_trial():
    w = World().all_in_one_trip()
    w.loose["g1_i2"] = np.array((0.30, 0.30, t.TOP + 0.013))
    w.wait(1.5)
    assert t.spilled(w.objects()) == ["g1_i2"] and w.score() == 0.0


def test_a_straw_on_the_table_zeroes_the_trial():
    w = World().all_in_one_trip()
    w.loose["g3_s"] = np.array((0.20, -0.40, t.TOP + 0.003))
    w.wait(1.5)
    assert w.score() == 0.0


def test_ice_loose_in_the_tub_means_that_tumbler_is_not_delivered():
    w = World().all_in_one_trip()
    w.loose["g1_i2"] = np.array((0.45, 0.0, FLOOR_Z + 0.013))
    w.wait(1.5)
    assert t.spilled(w.objects()) == [] and len(t.delivered(w.objects())) == 3 and w.score() == 0.25


def test_ice_in_another_tumbler_is_not_on_the_table_but_its_own_tumbler_is_not_delivered():
    w = World().all_in_one_trip()
    w.loose["g0_i2"] = w.g["g1"][0] + np.array((0.0, 0.0, 0.10))
    w.wait(1.5)
    assert t.spilled(w.objects()) == [] and w.score() == 0.25


# -- what counts as delivered -------------------------------------------------------------------------------------------
@pytest.mark.parametrize("deg, ok", [(0.0, True), (4.6, True), (14.0, True), (16.0, False), (30.0, False)])
def test_delivered_means_within_15_degrees_of_upright(deg, ok):
    w = World().all_in_one_trip()
    w.stand("g0", R=rot_y(math.radians(deg)))
    w.wait(1.5)
    assert ("g0" in t.delivered(w.objects())) is ok


@pytest.mark.parametrize("dz, ok", [(0.0, True), (0.006, True), (0.012, False), (0.05, False)])
def test_delivered_means_standing_on_the_tubs_floor(dz, ok):
    w = World().all_in_one_trip()
    w.stand("g0", z=FLOOR_Z + dz)
    w.wait(1.5)
    assert ("g0" in t.delivered(w.objects())) is ok


def test_a_tumbler_still_moving_is_not_delivered():
    w = World().all_in_one_trip().wait(1.5)
    w.vel["g0"] = ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert "g0" not in t.delivered(w.objects()) and w.score() == 0.25


def test_a_tumbler_on_the_tubs_rim_or_on_the_table_is_not_delivered():
    w = World().all_in_one_trip()
    w.stand("g0", xy=(0.30, 0.19), z=t.TOP + M.TUB_H)
    w.wait(1.5)
    assert "g0" not in t.delivered(w.objects())
    w.stand("g0", xy=(0.30, 0.40), z=t.TOP)
    w.wait(1.5)
    assert "g0" not in t.delivered(w.objects()) and w.score() == 0.25


def test_the_tub_may_be_nudged_delivery_is_judged_in_the_tubs_frame():
    w = World()
    w.tub = (np.array((t.TUB_XY[0] + 0.03, 0.02, t.TOP)), rot_z(math.radians(8.0)))
    w.all_in_one_trip().wait(1.5)
    assert w.score() == 1.0


# -- hands ----------------------------------------------------------------------------------------------------------------
def test_a_hand_left_in_the_tumblers_is_half():
    w = World().all_in_one_trip()
    w.tcp["left"] = np.array((0.30, 0.095, FLOOR_Z + M.GLASS_H - 0.015))  # fingers still down inside the two rims
    w.wait(2.0)
    assert not t.hands_clear(w.objects()) and w.score() == 0.5


@pytest.mark.parametrize("up, ok", [(0.04, False), (0.08, True)])
def test_hands_clear_means_6_cm_from_every_tumbler(up, ok):
    w = World().all_in_one_trip()
    w.tcp["left"] = np.array((*SLOTS["g0"], FLOOR_Z + M.GLASS_H + up))
    w.wait(2.0)
    assert t.hands_clear(w.objects()) is ok and w.score() == (1.0 if ok else 0.5)


def test_a_moving_hand_is_not_still():
    w = World().all_in_one_trip().wait(1.5)
    assert w.score() == 1.0
    w.tcp["right"] = w.tcp["right"] + np.array((0.0, 0.0, 0.01))  # 20 cm/s
    w.step()
    assert w.score() == 0.5


def test_reset_clears_the_record():
    w = World()
    w.g["g1"] = (np.array((0.40, 0.32, t.TOP + 0.03)), rot_y(math.radians(80.0)))
    w.step()
    assert t._trial["tipped"]
    t.reset_latch()
    assert not t._trial["tipped"] and t.carries() == {"left": 0, "right": 0, "none": 0}


# -- geometry and the demo's plan ---------------------------------------------------------------------------------------
def test_the_tumbler_is_the_real_size_and_hollow():
    assert abs(2 * M.r_out(M.GLASS_H) - 0.0826) < 1e-9 and abs(2 * M.r_out(0.0) - 0.0603) < 1e-9 and abs(M.GLASS_H - 0.1381) < 1e-9
    assert M.WALL == 0.0025 and 2 * M.r_in(M.GLASS_H) > 0.075
    assert 0.05 < M.GLASS_COM_Z < 0.08 and abs(M.FULL_MASS - 0.1253) < 1e-6
    pts, faces = M.wall_panel(0)
    assert len(pts) == 8 and len(faces) == 6


def test_the_tub_is_the_real_size():
    assert (M.TUB_L, M.TUB_W, M.TUB_H) == (0.506, 0.389, 0.125)
    assert M.tub_inside(M.TUB_FLOOR) == M.TUB_BOT and M.tub_inside(M.TUB_H) == M.TUB_TOP
    assert M.TUB_H > 0.9 * M.GLASS_H  # the wall is nearly as tall as a tumbler: they have to be lifted over it


def test_the_layout_fits_and_the_contents_start_inside():
    o = World().objects()
    for g in t.GLASSES:
        assert t.holds_contents(o, g) is True
        assert not t.over_tub(o, o[g].pos)
        start = {n: body(p, t._rot(q)) for n, (p, q) in t.START.items()}
        assert all(t.in_glass(start, g, start[n].pos) for n in t.CONTENTS[g])
    assert abs((t.FAR_X - t.NEAR_X) - (M.RIM_D + 0.100)) < 1e-9
    assert len(t.OBJECTS) == 21 and len({ob.name for ob in t.OBJECTS}) == 21


def test_two_rims_do_not_fit_one_gripper_from_outside():
    assert 2 * M.BASE_D > 0.094  # the gripper opens 94 mm: two tumblers cannot be squeezed from outside, even at their bases


@pytest.mark.parametrize("variant", ["", "none", "half", "single", "high", "held"])
def test_the_demo_is_reachable_and_inside_the_run_limit(variant):
    t.set_variant(variant)
    try:
        wps = t.demo()
    finally:
        t.set_variant("")
    length = max(sum(w.duration for w in ws) for ws in wps.values())
    pe, re, bad, _ = t.ik_errors(wps)
    assert not bad and length * 20 + 60 < 1800
