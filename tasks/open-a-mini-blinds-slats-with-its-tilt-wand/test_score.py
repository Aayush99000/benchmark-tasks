"""Scorer and tilter unit tests: synthetic object states built from the blind's own geometry, no simulator.
Created 11_17_26.

Run: python -m pytest test_score.py (plain Python with NumPy and the yam_arena package importable)."""

import math

import numpy as np
import pytest

import task as T
from mini_blind import mech as M
from yam_arena.task_file import ObjectState

DT = 0.05
FAR_L = (0.10, 0.40, T.TOP + 0.20)  # parked hands, well away from the blind
FAR_R = (0.10, -0.40, T.TOP + 0.20)


def _st(p, q=(0.0, 0.0, 0.0, 1.0), low=None, high=None):
    p = np.asarray(p, float)
    z = np.zeros(3)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=z, ang_vel=z,
                       low=p if low is None else np.asarray(low, float), high=p if high is None else np.asarray(high, float))


def _qy(a):
    return np.array((0.0, math.sin(a / 2), 0.0, math.cos(a / 2)))


def objects(angle=0.0, left=FAR_L, right=FAR_R, shift=(0.0, 0.0), stand_tilt=0.0, wand_off=0.0, spread=0.0):
    """A state with the slats at `angle` (deg, mean; `spread` between least and greatest), the stand shifted (m) and
    tilted (deg, about y), and the wand `wand_off` degrees off plumb (swung toward the robot)."""
    qs = _qy(math.radians(stand_tilt))
    base = np.array((T.BLIND_XY[0] + shift[0], T.BLIND_XY[1] + shift[1], T.TOP))
    hook = base + T._rot(qs) @ M.HOOK
    w = math.radians(wand_off)
    tip = hook + M.WAND["len"] * np.array((-math.sin(w), 0.0, -math.cos(w)))
    a = math.radians(angle)
    s = math.radians(spread) / 2
    return {
        "left_tcp": _st(left), "right_tcp": _st(right),
        "blind": _st(base, qs),
        "slats": _st((0, 0, 0), low=(a, a - s, a + s), high=(a, 0.0, 0.0)),
        "wand": _st(hook, low=tip, high=tip),
    }


def run(seq):
    """Feed (objects, seconds) pairs through note_step; return the final score."""
    T.reset_latch()
    last = None
    for o, secs in seq:
        for _ in range(int(round(secs / DT))):
            T.note_step(o, DT)
        last = o
    return T.score(last)


def final(**kw):
    return run([(objects(**kw), 2.0)])


def wand_point(frac=0.5):
    """A world point on the hanging wand."""
    return np.array((T.BLIND_XY[0], T.BLIND_XY[1], T.TOP)) + M.HOOK + np.array((0.0, 0.0, -frac * M.WAND["len"]))


# -- the bands ---------------------------------------------------------------------------------------------------------
def test_untouched_start_scores_zero():
    assert final(angle=T.START_DEG) == 0.0


@pytest.mark.parametrize("angle,want", [
    (0.0, 1.0), (-10.0, 1.0), (10.0, 1.0), (-3.5, 1.0),
    (-10.1, 0.5), (10.1, 0.5), (-25.0, 0.5), (25.0, 0.5),
    (-25.1, 0.25), (25.1, 0.25), (-50.0, 0.25), (50.0, 0.25),
    (-50.1, 0.0), (50.1, 0.0), (-58.5, 0.0), (-70.0, 0.0), (70.0, 0.0),
])
def test_bands_same_either_side_of_level(angle, want):
    assert final(angle=angle) == want


def test_quarter_band_is_twenty_degrees_from_the_closed_start():
    assert abs(T.START_DEG) - T.MOVED_DEG == 50.0
    assert T.band(T.START_DEG + T.MOVED_DEG) == 0.25
    assert T.band(T.START_DEG + T.MOVED_DEG - 0.5) == 0.0


def test_one_stroke_as_measured_scores_zero():
    assert final(angle=-58.5) == 0.0  # the simulator's slat angle after one 95 degree stroke in an early trial


def test_over_rotation_to_the_other_closed_stop_scores_zero():
    assert final(angle=M.CLOSED_DEG) == 0.0


# -- wand and stand ----------------------------------------------------------------------------------------------------
def test_wand_off_plumb_drops_top_band_to_half():
    assert final(angle=0.0, wand_off=9.0) == 1.0
    assert final(angle=0.0, wand_off=12.0) == 0.5


def test_wand_off_plumb_leaves_lower_bands_alone():
    assert final(angle=-20.0, wand_off=30.0) == 0.5
    assert final(angle=-40.0, wand_off=30.0) == 0.25


def test_stand_shift_bands():
    assert final(angle=0.0, shift=(0.019, 0.0)) == 1.0
    assert final(angle=0.0, shift=(0.0, 0.021)) == 0.5
    assert final(angle=0.0, shift=(0.07, 0.07)) == 0.5
    assert final(angle=0.0, shift=(0.101, 0.0)) == 0.0
    assert final(angle=-40.0, shift=(0.05, 0.0)) == 0.25
    assert final(angle=-40.0, shift=(0.0, 0.11)) == 0.0


def test_stand_tipped_scores_zero():
    assert final(angle=0.0, stand_tilt=9.0, wand_off=0.0) > 0.0
    assert final(angle=0.0, stand_tilt=11.0) == 0.0
    assert final(angle=0.0, stand_tilt=90.0) == 0.0


# -- hands -------------------------------------------------------------------------------------------------------------
def test_hand_still_on_the_wand_scores_zero():
    assert final(angle=0.0, left=wand_point()) == 0.0


def test_hand_near_the_wand_scores_zero():
    p = wand_point() + np.array((-0.030, 0.0, 0.0))
    assert final(angle=0.0, left=p) == 0.0
    assert final(angle=0.0, left=wand_point() + np.array((-0.045, 0.0, 0.0))) == 1.0


def test_hand_at_the_slats_scores_zero():
    p = (T.BLIND_XY[0] - M.HEADRAIL["d"] / 2 - 0.02, 0.0, T.TOP + 0.15)
    assert final(angle=0.0, right=p) == 0.0
    assert final(angle=0.0, right=(T.BLIND_XY[0] - M.HEADRAIL["d"] / 2 - 0.05, -0.05, T.TOP + 0.15)) == 1.0


def test_clear_needs_a_full_last_second():
    held, free = objects(angle=0.0, left=wand_point()), objects(angle=0.0)
    assert run([(held, 2.0), (free, 0.9)]) == 0.0
    assert run([(held, 2.0), (free, 1.0)]) == 1.0


def test_touching_again_at_the_end_scores_zero():
    held, free = objects(angle=0.0, left=wand_point()), objects(angle=0.0)
    assert run([(free, 3.0), (held, 0.05)]) == 0.0


def test_a_passing_bump_is_not_remembered():
    """Everything is judged at the end: a stand that was shifted, a wand that swung and slats that were over-turned
    earlier in the trial do not count against a good final state."""
    bad = objects(angle=60.0, shift=(0.05, 0.0), wand_off=40.0, stand_tilt=5.0)
    assert run([(bad, 2.0), (objects(angle=0.0), 1.0)]) == 1.0


def test_best_so_far_is_not_kept():
    assert run([(objects(angle=0.0), 2.0), (objects(angle=40.0), 1.0)]) == 0.25
    assert run([(objects(angle=0.0), 2.0), (objects(angle=70.0), 1.0)]) == 0.0


def test_reset_forgets_the_trial():
    run([(objects(angle=0.0), 2.0)])
    T.reset_latch()
    assert T.score(objects(angle=0.0)) == 0.0  # no clear time yet


def test_success_never_stops_a_trial_early():
    assert T.success(objects(angle=0.0)) is False


def test_readouts():
    o = objects(angle=-33.0, wand_off=17.0, shift=(0.03, 0.04), stand_tilt=0.0)
    assert T.slat_angle_deg(o) == pytest.approx(-33.0)
    assert T.wand_plumb_deg(o) == pytest.approx(17.0, abs=1e-6)
    assert T.stand_shift(o) == pytest.approx(0.05)
    assert T.stand_tilt_deg(objects(stand_tilt=7.0)) == pytest.approx(7.0, abs=1e-6)


# -- the tilter (worm ratio and clutch) --------------------------------------------------------------------------------
def _turn(t, turns, steps=400, start=0.0):
    a = start
    for _ in range(steps):
        a += 2 * math.pi * turns / steps
        t.feed(M.wrap(a))  # the joint's reading wraps; the tilter unwraps it
    return a


def test_tilter_ratio_clockwise_opens():
    t = M.Tilter(T.START_DEG)
    t.feed(0.0)
    _turn(t, -1.0)
    assert math.degrees(t.tilt) == pytest.approx(T.START_DEG + 360.0 / M.RATIO)
    assert t.stem == pytest.approx(-2 * math.pi)


def test_tilter_turns_to_level():
    t = M.Tilter(T.START_DEG)
    t.feed(0.0)
    _turn(t, -M.turns_to_level())
    assert math.degrees(t.tilt) == pytest.approx(0.0, abs=1e-6)
    assert M.turns_to_level() == pytest.approx(70.0 * 10.0 / 360.0)


def test_one_wrist_stroke_is_far_from_enough():
    t = M.Tilter(T.START_DEG)
    t.feed(0.0)
    _turn(t, -120.0 / 360.0)  # more than the wrist's reach in one stroke
    assert T.band(math.degrees(t.tilt)) == 0.0


def test_clutch_slips_at_the_closed_start():
    t = M.Tilter(T.START_DEG)
    t.feed(0.0)
    a = _turn(t, +3.0)  # the wrong way: already at the stop
    assert math.degrees(t.tilt) == pytest.approx(T.START_DEG)
    _turn(t, -1.0, start=a)  # and it picks up at once on the way back
    assert math.degrees(t.tilt) == pytest.approx(T.START_DEG + 36.0)


def test_clutch_slips_at_the_far_stop():
    t = M.Tilter(T.START_DEG)
    t.feed(0.0)
    _turn(t, -6.0)
    assert math.degrees(t.tilt) == pytest.approx(M.CLOSED_DEG)
    assert T.band(math.degrees(t.tilt)) == 0.0


def test_tilter_takes_nothing_from_a_still_stem():
    t = M.Tilter(T.START_DEG)
    for _ in range(50):
        t.feed(1.234)
    assert math.degrees(t.tilt) == pytest.approx(T.START_DEG)
    assert t.stem == 0.0


# -- geometry ----------------------------------------------------------------------------------------------------------
def test_blind_is_the_products_smallest_size():
    assert M.WIDTH == pytest.approx(0.4572)  # 18 in wide
    assert M.DROP == pytest.approx(0.3048, abs=0.004)  # 12 in drop
    assert M.N_SLATS == 12 and M.PITCH == 0.0215
    assert (M.HEADRAIL["d"], M.HEADRAIL["h"]) == pytest.approx((0.0381, 0.0254))
    assert M.SLAT_METAL_T == pytest.approx(0.006 * 0.0254)  # 6 gauge
    assert M.TOP_Z <= 0.35 + 1e-9


def test_stand_carries_the_blind_inside_the_work_area():
    assert M.BOARD["y"] >= M.WIDTH + 0.05 and M.BOARD["y"] <= 0.60  # the base is wider than the blind, inside 0.6 m
    assert M.LINTEL["y"] >= M.WIDTH  # the head piece spans the headrail, for its two brackets
    assert M.POST["y"] + M.POST["s"] / 2 <= M.LINTEL["y"] / 2 + 1e-9
    assert M.POST["x"] - M.POST["s"] / 2 > M.SLAT["w"] / 2 + 0.005  # posts clear of the slats at any angle
    assert abs(T.BLIND_XY[1]) < 1e-9 and T.BLIND_XY[0] + M.BOARD["x"] / 2 <= 0.45  # centred, inside 0.45 m forward


def test_wand_is_at_the_tilter_end_within_the_left_arms_reach():
    assert M.TILTER_Y == pytest.approx(M.WIDTH / 2 - 0.052)
    assert T.HOOK_W[1] == pytest.approx(0.1766, abs=1e-3)
    worst, bad = T.ik_errors(T.demo())  # every waypoint of the scripted demo solves to within 1 mm and 1 degree
    assert worst[0] < 1.0 and worst[1] < 1.0 and not bad


def test_slats_never_touch_each_other_or_the_base():
    lim = math.radians(M.CLOSED_DEG)
    gap = M.PITCH * math.cos(lim)  # square distance between neighbouring slats' planes at the stop
    assert gap > M.SLAT["t"] + 0.003
    rail_low = M.RAIL_Z - (M.RAIL["w"] / 2 * math.sin(lim) + M.RAIL["h"] / 2 * math.cos(lim))
    assert rail_low > M.BOARD["t"] + 0.003


def test_wand_hangs_clear():
    assert M.wand_bottom()[2] > M.BOARD["t"] + 0.02
    assert M.WAND_STANDOFF - M.WAND["af"] / 2 - M.SLAT["w"] / 2 > 0.010  # clear of level slats
    assert M.WAND_MASS == pytest.approx(0.0103, abs=0.001)
    assert M.HOOK[2] < 24 * M.IN  # the supplied 24 in wand is longer than the hook is high, hence the sawn one


def test_demo_is_open_loop_and_long_enough():
    wps = T.demo()
    assert set(wps) == {"left", "right"}
    t = sum(w.duration for w in wps["left"])
    assert 30.0 < t < 150.0  # within the 3,000 step limit at 20 Hz
    assert sum(1 for w in wps["left"] if w.label == "pinch the wand") == T.STROKES
