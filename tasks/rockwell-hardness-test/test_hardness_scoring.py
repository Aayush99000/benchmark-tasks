"""Unit tests of the scorer (pure NumPy):

    python -m pytest -q test_hardness_scoring.py
"""

from __future__ import annotations

import math

import pytest

import hardness_scoring as S

DEG = math.pi / 180
FWD, BACK = S.HANDLE_FORWARD, S.HANDLE_BACK
SPOT = (0.0, -0.006)
SET = S.wheel_for(S.PRE_TRAVEL)


def run(steps, rec=None, seated=True, spot=SPOT):
    """Feed (wheel, handle, seconds) segments, one control step at a time."""
    rec = rec or S.HardnessRecord()
    for wheel, handle, seconds in steps:
        for _ in range(int(round(seconds / S.DT))):
            rec.step(wheel, handle, seated, spot)
    return rec


def full_test(set_wheel=SET, dwell=5.0, spot=SPOT, rec=None):
    """Raise to ``set_wheel``, push the handle back, hold APPLY_S + ``dwell``, pull it forward."""
    steps = [(set_wheel * k / 20, FWD, 0.1) for k in range(1, 21)]
    steps += [(set_wheel, FWD, 1.0), (set_wheel, BACK, S.APPLY_S + dwell), (set_wheel, FWD, 1.0)]
    return run(steps, rec=rec, spot=spot)


def lower(rec, spot=SPOT):
    return run([(0.0, FWD, 1.0)], rec=rec, spot=spot)


def test_travel_geometry():
    assert S.travel(0.0) == pytest.approx(-S.START_GAP)
    assert S.travel(SET) == pytest.approx(S.PRE_TRAVEL)
    assert math.degrees(SET) == pytest.approx(162.0)  # (1.2 + 0.6) mm at 4 mm a turn
    one_div = math.degrees(S.wheel_for(S.PRE_TRAVEL + S.DIVISION) - SET)
    assert one_div == pytest.approx(0.18)  # one division of the dial: 0.18 deg of handwheel
    assert S.divisions_from_c(S.travel(SET + 5 * one_div * DEG)) == pytest.approx(5.0)


def test_a_clean_test_then_lowered_scores_full():
    rec = lower(full_test())
    assert S.grade(rec, 0.0, FWD, True, True) == 1.0
    assert S.score(rec, 0.0, FWD, True, True) == S.PARTIAL_CAP
    assert S.reading(rec) == S.BLOCK_HRC


@pytest.mark.parametrize("divs, expect", [(-5, 1.0), (4.9, 1.0), (-6, 0.0), (6, 0.0)])
def test_the_preliminary_window_is_five_divisions(divs, expect):
    w = S.wheel_for(S.PRE_TRAVEL + divs * S.DIVISION)
    rec = lower(full_test(set_wheel=w))
    assert S.grade(rec, 0.0, FWD, True, True) == expect


def test_overshoot_then_back_spoils_the_spot():
    over = S.wheel_for(S.PRE_TRAVEL + 8 * S.DIVISION)
    rec = run([(over, FWD, 1.0), (SET, FWD, 1.0), (SET, BACK, S.APPLY_S + 5), (SET, FWD, 1.0), (0.0, FWD, 1.0)])
    assert rec.attempts[0].overshot
    assert S.grade(rec, 0.0, FWD, True, True) == 0.0


def test_a_fresh_spot_after_an_overshoot_can_still_score():
    over = S.wheel_for(S.PRE_TRAVEL + 8 * S.DIVISION)
    rec = run([(over, FWD, 1.0), (0.0, FWD, 1.0)])
    rec = lower(full_test(rec=rec, spot=(0.0, -0.002)), spot=(0.0, -0.002))  # 4 mm from the spoiled spot
    assert S.grade(rec, 0.0, FWD, True, True) == 1.0


def test_a_spot_too_close_to_an_earlier_one_is_not_fresh():
    rec = lower(full_test())
    rec = lower(full_test(rec=rec, spot=(0.0, -0.004)), spot=(0.0, -0.004))
    assert not rec.attempts[1].fresh


def test_a_spot_near_the_edge_is_not_fresh():
    rec = lower(full_test(spot=(0.0, S.BLOCK_RADIUS - 0.002)), spot=(0.0, S.BLOCK_RADIUS - 0.002))
    assert S.grade(rec, 0.0, FWD, True, True) == 0.0


def test_load_without_the_preliminary_force_spoils_the_test():
    short = S.wheel_for(S.PRE_TRAVEL - 30 * S.DIVISION)
    rec = run([(short, FWD, 1.0), (short, BACK, S.APPLY_S + 5), (short, FWD, 1.0), (0.0, FWD, 1.0)])
    assert S.grade(rec, 0.0, FWD, True, True) == 0.0


def test_turning_the_wheel_under_load_spoils_the_test():
    nudge = S.wheel_for(S.PRE_TRAVEL + 3 * S.DIVISION)
    rec = run([(SET * k / 20, FWD, 0.1) for k in range(1, 21)] +
              [(SET, BACK, 3.0), (nudge, BACK, S.APPLY_S), (nudge, FWD, 1.0), (0.0, FWD, 1.0)])
    assert rec.attempts[0].wheel_moved
    assert S.grade(rec, 0.0, FWD, True, True) == 0.0


@pytest.mark.parametrize("dwell, expect", [(1.0, 0.5), (3.0, 1.0), (5.0, 1.0), (6.9, 1.0), (9.0, 0.5)])
def test_dwell_window(dwell, expect):
    rec = lower(full_test(dwell=dwell))
    assert S.grade(rec, 0.0, FWD, True, True) == expect


def test_taken_off_before_the_load_is_fully_on():
    rec = run([(SET * k / 20, FWD, 0.1) for k in range(1, 21)] + [(SET, BACK, 2.0), (SET, FWD, 1.0), (0.0, FWD, 1)])
    assert S.grade(rec, 0.0, FWD, True, True) == 0.25


def test_stages_in_order():
    rec = run([(SET * k / 20, FWD, 0.1) for k in range(1, 21)])
    assert S.grade(rec, SET, FWD, True, True) == 0.25
    rec = run([(SET, BACK, S.APPLY_S + 1)], rec=rec)
    assert S.grade(rec, SET, BACK, True, True) == 0.5
    rec = run([(SET, BACK, 4.0), (SET, FWD, 1.0)], rec=rec)
    assert S.grade(rec, SET, FWD, True, True) == 0.75  # valid reading, not lowered yet
    rec = lower(rec)
    assert S.grade(rec, 0.0, FWD, True, False) == 0.75  # a hand still on a knob
    assert S.grade(rec, 0.0, BACK, True, True) == 0.75  # handle not forward
    assert S.grade(rec, 0.0, FWD, False, True) == 0.75  # block knocked off the anvil
    assert S.grade(rec, 0.0, FWD, True, True) == 1.0


def test_block_not_on_the_anvil_scores_nothing():
    rec = S.HardnessRecord()
    rec = lower(full_test(rec=rec), spot=SPOT)
    rec2 = run([(SET * k / 20, FWD, 0.1) for k in range(1, 21)], seated=False)
    assert not rec2.attempts[0].seated
    assert S.grade(rec2, SET, FWD, False, True) == 0.0
    assert S.grade(rec, 0.0, FWD, True, True) == 1.0


def test_pointers():
    assert S.long_pointer(S.PRE_TRAVEL, 0.0) == pytest.approx(0.0, abs=1e-9)  # at C after three turns
    assert S.long_pointer(S.PRE_TRAVEL + 5 * S.DIVISION, 0.0) == pytest.approx(5 * 2 * math.pi / 100)
    assert S.short_pointer(S.PRE_TRAVEL) == pytest.approx(S.SHORT_RED)
    assert S.short_pointer(-0.001) == 0.0
    depth = S.permanent_depth()
    assert depth == pytest.approx((100 - S.BLOCK_HRC) * 0.002e-3)
    a = S.long_pointer(S.PRE_TRAVEL, depth)  # 38 divisions back from C: HRC 62
    assert (math.cos(a), math.sin(a)) == pytest.approx((math.cos(-0.38 * 2 * math.pi), math.sin(-0.38 * 2 * math.pi)))
