"""Offline checks of the plumb-bob task: the scoring rule, the layout and the write-up. No simulator needed.

    python -m pytest test_scoring.py        (in the simulator's Python, which has yam_arena)
"""
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import plumbkit as K  # noqa: E402
import task as T  # noqa: E402


def mark(mm: float, plumbed: bool = True) -> dict:
    return {"d": mm / 1000.0, "plumbed": plumbed}


# -- the scoring rule ---------------------------------------------------------------------------------------------------

def test_nothing_done_scores_zero():
    assert T.score_from([], lifted=True, still=False, steadied=False) == 0.0


def test_steadying_alone_scores_a_quarter():
    assert T.score_from([], lifted=True, still=True, steadied=True) == 0.25


def test_a_good_mark_with_everything_left_right_scores_full():
    assert T.score_from([mark(2.0)], lifted=True, still=True, steadied=True) == 1.0
    assert T.score_from([mark(3.0)], lifted=True, still=True, steadied=True) == 1.0


def test_a_mark_made_while_the_bob_swings_or_is_held_earns_nothing():
    # the memorized spot, hit exactly, with the bob never steadied
    assert T.score_from([mark(0.0, plumbed=False)], lifted=True, still=False, steadied=False) == 0.0
    # ... and it spoils a later good mark: the worktop now carries a mark nothing vouches for
    assert T.score_from([mark(0.0, plumbed=False), mark(1.0)], lifted=True, still=True, steadied=True) == 0.5


def test_a_mark_between_3_and_10_mm_scores_half():
    assert T.score_from([mark(3.1)], lifted=True, still=True, steadied=True) == 0.5
    assert T.score_from([mark(10.0)], lifted=True, still=True, steadied=True) == 0.5


def test_a_mark_further_than_10_mm_earns_only_the_steadying():
    assert T.score_from([mark(10.1)], lifted=True, still=True, steadied=True) == 0.25


def test_a_stray_mark_costs_full_marks():
    assert T.score_from([mark(1.0), mark(40.0)], lifted=True, still=True, steadied=True) == 0.5


def test_two_good_marks_still_score_full():
    assert T.score_from([mark(1.0), mark(2.5)], lifted=True, still=True, steadied=True) == 1.0


def test_pencil_left_down_or_bob_knocked_afterwards_costs_full_marks():
    assert T.score_from([mark(1.0)], lifted=False, still=True, steadied=True) == 0.5
    assert T.score_from([mark(1.0)], lifted=True, still=False, steadied=True) == 0.5


# -- what makes a mark ----------------------------------------------------------------------------------------------------

def test_a_point_held_just_above_the_worktop_makes_no_mark():
    # the old rule marked anything within 0.5 mm of the worktop; now nothing marks without contact and pressure
    for gap_mm in (0.5, 0.4, 0.31):
        assert not T.marking(gap_mm / 1000.0, True, 0.0)
        assert not T.marking(gap_mm / 1000.0, True, 1.0)                  # not even with its lead pushed in by something else


def test_a_point_resting_on_the_worktop_too_lightly_makes_no_mark():
    assert not T.marking(0.0, True, 0.0)
    assert not T.marking(0.0, True, 0.1)                                  # the pencil's own weight is 0.1 N
    assert not T.marking(0.0, True, T.MARK_FORCE - 0.01)


def test_a_point_pressed_on_the_worktop_makes_a_mark():
    assert T.marking(0.0, True, T.MARK_FORCE)
    assert T.marking(0.0002, True, 0.6)
    assert not T.marking(0.0, False, 0.6)                                 # pressed on something off the worktop


def test_the_marking_force_is_a_light_stroke_the_lead_can_read():
    assert 0.2 <= T.MARK_FORCE <= 1.0                                     # light: well under the robot's 3 N push
    assert T.MARK_FORCE > 2 * K.PENCIL_MASS * 9.81                        # more than the pencil lying on its point
    assert K.lead_force(0.0) == 0.0 and K.lead_force(-0.001) == 0.0
    assert math.isclose(K.lead_force(0.0005), 0.3)
    assert K.lead_force(K.LEAD_TRAVEL) > T.MARK_FORCE                     # reached before the lead bottoms out
    assert K.LEAD_CLEAR > K.LEAD_TRAVEL                                   # the wood never reaches the worktop first
    assert T.ON_TOP < T.LIFT


# -- the layout -----------------------------------------------------------------------------------------------------------

def test_the_bob_hangs_6_mm_above_the_worktop_under_the_hole():
    point = T.pivot() + K.point_offset(T.DROP)
    assert np.allclose(point[:2], T.PLUMB_XY)
    assert math.isclose(point[2] - T.TOP, T.POINT_GAP, abs_tol=1e-9)
    assert math.isclose(T.SHELF_UNDER, 18 * 0.0254, abs_tol=5e-4)        # a wall shelf's usual height over a worktop


def test_the_bob_starts_2_cm_from_plumb_toward_the_arms():
    point = T.pivot() + K.point_offset(T.DROP, top_y=T.START_ANGLE)
    assert math.isclose(point[0] - T.PLUMB_XY[0], -T.START_OUT, abs_tol=1e-6)
    assert math.isclose(point[1], T.PLUMB_XY[1], abs_tol=1e-9)
    assert T.START_OUT > 5 * T.STILL                                      # far from still at the start


def test_the_swing_cannot_be_waited_out():
    """With the joint damping net of the integrator's gain, the free swing's amplitude after the whole run limit is
    still several times the 'still' threshold."""
    k = K.BOB_MASS * 9.81 * (T.DROP + K.BOB_L - K.BOB_COM)                # gravity stiffness at the hanging point
    inertia = K.BOB_MASS * (T.DROP + K.BOB_L - K.BOB_COM) ** 2
    net = K.TOP_DAMP - k / 240 / 2
    assert net > 0
    tau = 2 * inertia / net                                               # the amplitude's decay time, s
    assert 100 < tau < 400
    assert T.START_OUT * math.exp(-60.0 / tau) > 3 * T.STILL


def test_a_gripper_that_counts_as_clear_cannot_be_touching_the_bob():
    assert T.CLEAR >= K.BOB_R + 0.015                                     # finger points are checked against the centre line
    assert T.CLEAR < 0.0939 / 2                                           # an open hand round the bob does count as clear


def test_segment_distance():
    assert math.isclose(K.segment_distance((1.0, 0.0, 0.5), (0, 0, 0), (0, 0, 1)), 1.0)
    assert math.isclose(K.segment_distance((0.0, 3.0, 5.0), (0, 0, 0), (0, 0, 1)), 5.0)


def test_the_pencil_starts_in_its_rest_off_the_worktop_and_fits_under_the_bob():
    start = T.pencil_start()
    assert start[2] - T.TOP > T.LIFT                                      # no mark at the start
    assert T.TABLE_X[0] < start[0] < T.TABLE_X[1] and T.TABLE_Y[0] < start[1] < T.TABLE_Y[1]
    # brought in along its slant with its point on the mark, the pencil clears the bob's taper all the way up
    for s in np.linspace(0.001, 0.12, 60):
        along = s * T.UP_PENCIL                                           # a point on the pencil's axis, from the mark
        half = K.PENCIL_T / 2 * min(1.0, s / K.PENCIL_TIP)                # the pencil's half thickness there
        h = along[2] + half * math.cos(T.SLANT) - T.POINT_GAP             # its upper side's height above the bob's point
        if h < 0:
            continue                                                      # still below the bob's point: nothing to hit
        bob_r = min(K.BOB_R, K.BOB_R * h / K.CONE_L)
        assert abs(along[1]) - half / math.sin(T.SLANT) - bob_r > 0.002


def test_every_fixed_part_is_on_or_above_the_worktop():
    for o in T.OBJECTS:
        if isinstance(o, K.Fixed):
            r = K._rotation(o.quat)
            half = np.abs(r) @ (np.asarray(o.size) / 2)
            assert o.pos[2] - half[2] >= T.TOP - 1e-6, o.name


def test_both_layouts_load():
    import importlib
    import os

    for lot in ("A", "B"):
        os.environ["PLUMB_LOT"] = lot
        mod = importlib.reload(T)
        assert mod.LOT == lot and len(mod.OBJECTS) == 13 + mod.N_DOTS
    os.environ.pop("PLUMB_LOT")
    importlib.reload(T)


# -- the write-up ---------------------------------------------------------------------------------------------------------

def test_task_json_matches_the_code_and_the_intake_limits():
    path = HERE / "task.json"
    if not path.exists():
        return
    meta = json.loads(path.read_text())
    assert meta["instruction"] == T.INSTRUCTION
    for key, limit in (("title", 120), ("context", 1000), ("setup", 4000), ("fidelity", 500), ("scores", 300)):
        assert len(meta.get(key, "")) <= limit, (key, len(meta[key]))
    assert meta["category"] in ("Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                "Societal relatability", "Other")
