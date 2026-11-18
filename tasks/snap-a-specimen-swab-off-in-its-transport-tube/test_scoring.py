"""Scoring and geometry rules of the swab task, checked without the simulator.

    python -m pytest test_scoring.py -q        (needs yam_arena on the path, as on the evaluation instance)
"""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import swabkit as K  # noqa: E402
import task as T  # noqa: E402

UPRIGHT = (0.0, 0.0, 0.0, 1.0)


def turn_x(deg):
    a = math.radians(deg)
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


# -- the scoring ladder ------------------------------------------------------------------------------------------------
def test_nothing_done_scores_zero():
    assert T.score_from(False, False, False, False, False, True, True) == 0.0


def test_valid_snap_ladder():
    assert T.score_from(True, True, False, False, False, True, True) == 0.5      # handle off, not yet thrown away
    assert T.score_from(True, True, False, False, True, True, False) == 0.75     # in the cup, a hand still on the tube
    assert T.score_from(True, True, False, False, True, False, True) == 0.75     # in the cup, the tube not standing
    assert T.score_from(True, True, False, False, True, True, True) == 1.0


def test_snap_off_the_floor_or_with_the_tube_over_scores_zero():
    assert T.score_from(True, False, False, False, True, True, True) == 0.0


def test_spilled_tube_scores_zero_whatever_follows():
    assert T.score_from(True, True, True, False, True, True, True) == 0.0


def test_tip_out_of_the_tube_scores_zero_whatever_follows():
    assert T.score_from(True, True, False, True, True, True, True) == 0.0


def test_binning_an_unbroken_swab_scores_zero():
    assert T.score_from(False, False, False, True, True, True, True) == 0.0


# -- the swab at rest in the tube ----------------------------------------------------------------------------------------
def _tube():
    return np.array([T.TUBE_XY[0], T.TUBE_XY[1], T.TOP]), UPRIGHT


def test_swab_starts_seated_joined_and_inside_the_bore():
    tube_pos, tube_quat = _tube()
    inside, above = T.tip_in_tube(T.tip_start(), tube_pos, tube_quat)
    assert inside and 0.0 <= above <= 0.001
    assert K.joined(T.tip_start(), T.SWAB_QUAT, T.breakpoint_start(), T.SWAB_QUAT)
    # the shaft leans on the far side of the rim without cutting into the wall
    at_rim = T.tip_start() + (K.TUBE_H - K.TUBE_FLOOR) / T.UP_SWAB[2] * T.UP_SWAB
    off = float(np.linalg.norm(at_rim[:2] - T.TUBE_XY))
    assert K.TUBE_ID / 2 - K.SHAFT_R - 0.0005 < off < K.TUBE_ID / 2 - K.SHAFT_R
    assert at_rim[0] > T.TUBE_XY[0]


def test_breakpoint_sits_just_inside_the_rim_and_handle_sticks_out():
    bp = T.breakpoint_start()[2] - T.TOP
    assert K.TUBE_H - 0.012 < bp < K.TUBE_H                           # inside the tube, within 12 mm of the rim
    top = (T.tip_start() + K.SWAB_L * T.UP_SWAB)[2] - T.TOP
    assert 0.155 < top < 0.165


def test_tip_in_tube_limits():
    tube_pos, tube_quat = _tube()
    floor = tube_pos + [0.0, 0.0, K.TUBE_FLOOR]
    assert T.tip_in_tube(floor + [0.0, 0.0, 0.030], tube_pos, tube_quat) == (True, pytest.approx(0.030))
    assert not T.tip_in_tube(floor + [0.0, 0.0, K.TUBE_H], tube_pos, tube_quat)[0]             # lifted right out
    assert not T.tip_in_tube(floor + [0.012, 0.0, 0.030], tube_pos, tube_quat)[0]              # beside the tube
    # in a tube lying on its side, the tip is judged in the tube's own frame
    lying = turn_x(90.0)
    inside, above = T.tip_in_tube(tube_pos + K._rotation(lying) @ np.array([0.0, 0.0, K.TUBE_FLOOR + 0.01]), tube_pos, lying)
    assert inside and above == pytest.approx(0.010)


def test_joined_tolerances():
    p0, q = T.tip_start(), T.SWAB_QUAT
    p1 = T.breakpoint_start()
    assert not K.joined(p0, q, p1 + [0.005, 0.0, 0.0], q)            # 5 mm apart
    assert K.joined(p0, q, p1 + [0.002, 0.0, 0.0], q)                # the joint's give under load is not a snap
    bent = K._quat(K._rotation(q) @ K._rotation(K.turn_y(math.radians(15.0))))
    assert not K.joined(p0, q, p1, bent)                             # bent 15 degrees at the breakpoint
    barely = K._quat(K._rotation(q) @ K._rotation(K.turn_y(math.radians(4.0))))
    assert K.joined(p0, q, p1, barely)
    shift, angle = K.apart(p0, q, p1 + [0.002, 0.0, 0.0], barely)
    assert shift == pytest.approx(0.002, abs=1e-6) and math.degrees(angle) == pytest.approx(4.0, abs=1e-3)


def test_quat_round_trip():
    for q in (UPRIGHT, K.turn_y(0.3), turn_x(100.0), (0.5, 0.5, 0.5, 0.5)):
        assert np.allclose(K._rotation(K._quat(K._rotation(q))), K._rotation(q), atol=1e-9)


def test_tube_tilt():
    assert T.tube_tilt(UPRIGHT) == pytest.approx(0.0)
    assert T.tube_tilt(turn_x(70.0)) == pytest.approx(70.0)
    assert T.tube_tilt(turn_x(70.0)) > T.SPILL_TILT > T.SNAP_TILT > T.STAND_TILT


# -- why it takes two hands ----------------------------------------------------------------------------------------------
def test_an_unheld_tube_tips_long_before_the_shaft_snaps():
    tipping = K.TUBE_MASS * 9.81 * K.TUBE_OD / 2                     # N m about the edge of its base
    assert K.BREAK_TORQUE > 20 * tipping


def test_a_straight_pull_cannot_snap_the_swab():
    assert K.BREAK_FORCE > 100 * (K.HEAD_MASS + K.HANDLE_MASS) * 9.81


# -- the waste cup and the scene -----------------------------------------------------------------------------------------
def test_in_cup():
    cup = np.array([T.CUP_XY[0], T.CUP_XY[1], T.TOP])
    assert T.in_cup(cup + [0.01, 0.0, 0.03], cup, UPRIGHT)
    assert not T.in_cup(cup + [0.06, 0.0, 0.03], cup, UPRIGHT)
    assert not T.in_cup(cup + [0.0, 0.0, T.CUP_H + 0.02], cup, UPRIGHT)
    assert not T.in_cup(cup + [0.0, 0.0, 0.03], cup, turn_x(80.0))
    assert K.HANDLE_L < T.CUP_H < T.CUP_IN                           # the handle can lie right inside


def test_scene_loads_and_names_are_unique():
    names = [o.name for o in T.OBJECTS]
    assert names == [T.TUBE, T.HEAD, T.HANDLE, T.CUP]
    for o in T.OBJECTS:
        o._check()
    head = next(o for o in T.OBJECTS if o.name == T.HEAD)
    assert head.handle == T.HANDLE
    assert np.linalg.norm(T.CUP_XY - T.TUBE_XY) > T.CUP_IN / 2 + 0.05
