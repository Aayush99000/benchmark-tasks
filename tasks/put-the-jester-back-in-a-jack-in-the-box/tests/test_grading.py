"""The scorer's tiers and the cheap or wrong ways of doing the task, on hand-built states."""

import numpy as np

from rdb_jack_in_box import grading, layout as L
from rdb_jack_in_box.grading import JackGrade

BOX = np.array((L.BOX_X, 0., 0.))
UP = np.eye(3)
AWAY = [(.15, .30, .25), (.15, -.30, .25)]                 # both grippers far from the box
POPPED = L.head_centre(L.S_POPPED, 0.)
PRESSED = L.head_centre(L.S_SHUT - .004, 0.)               # head top 4 mm below the rim


def run(grade, steps, lid=0., head=PRESSED, box=BOX, rot=UP, tcp=AWAY, touching=False):
    result = None
    for _ in range(steps):
        result = grade.update(lid, head, box, rot, tcp, touching)
    return result


def rot_z(deg):
    a = np.radians(deg)
    return np.array(((np.cos(a), -np.sin(a), 0.), (np.sin(a), np.cos(a), 0.), (0., 0., 1.)))


def rot_x(deg):
    a = np.radians(deg)
    return np.array(((1., 0., 0.), (0., np.cos(a), -np.sin(a)), (0., np.sin(a), np.cos(a))))


def test_untouched_toy_scores_zero():
    assert run(JackGrade(), 100, lid=105., head=POPPED)["score"] == 0.


def test_full_score_needs_two_seconds_hands_off():
    grade = JackGrade()
    assert run(grade, grading.HOLD_TICKS - 1)["score"] == .6
    assert run(grade, 1)["score"] == 1.


def test_holding_the_lid_shut_scores_at_most_point_six():
    grade = JackGrade()
    assert run(grade, 200, tcp=[(L.BOX_X - .06, 0., .15), AWAY[1]], touching=True)["score"] == .6
    # near the box without touching it is not clear either
    grade = JackGrade()
    assert run(grade, 200, tcp=[(L.BOX_X - L.HALF - .03, 0., .10), AWAY[1]])["score"] == .6


def test_letting_go_restarts_the_hold():
    grade = JackGrade()
    run(grade, 30)
    run(grade, 1, touching=True)
    assert run(grade, grading.HOLD_TICKS - 1)["score"] == .6


def test_pressing_the_jester_with_the_lid_wide_open_earns_nothing():
    grade = JackGrade()
    assert run(grade, 50, lid=105., head=PRESSED)["score"] == 0.
    assert run(grade, 50, lid=45., head=PRESSED)["score"] == 0.


def test_head_below_the_rim_with_the_lid_nearly_shut_is_latched():
    grade = JackGrade()
    run(grade, 5, lid=30., head=PRESSED)
    assert run(grade, 100, lid=105., head=POPPED)["score"] == .3


def test_lid_propped_open_is_not_shut():
    grade = JackGrade()
    assert run(grade, 100, lid=grading.SHUT_DEG + .5)["score"] == .3


def test_lid_shut_with_the_jester_bent_out_over_the_front_is_not_shut():
    out = L.head_centre(.120, -np.radians(30.))                # bent over the front wall, head outside the box
    grade = JackGrade()
    result = run(grade, 100, lid=1., head=out)
    assert not result["jester_inside"] and result["score"] == 0.
    # the head sticking up 6 mm above the rim (the lid could not be shut over it): not inside either
    high = L.head_centre(L.S_SHUT + .006, 0.)
    assert run(JackGrade(), 100, lid=1., head=high)["score"] == 0.
    # 3 mm above the rim is under a lid 3 degrees open: inside, but it does not count as below the rim for 0.3
    assert run(JackGrade(), 100, lid=2., head=L.head_centre(L.S_SHUT + .003, 0.))["score"] == 1.
    assert not JackGrade().update(20., L.head_centre(L.S_SHUT + .003, 0.), BOX, UP, AWAY, False)["pressed_while_closing"]


def test_box_off_its_marks_caps_the_score():
    shifted = BOX + (.020, 0., 0.)
    grade = JackGrade()
    assert run(grade, 100, box=shifted, head=PRESSED + (.020, 0., 0.))["score"] == .6
    grade = JackGrade()
    assert run(grade, 100, rot=rot_z(12.))["score"] == .6
    grade = JackGrade()
    assert run(grade, 100, box=BOX + (.010, .008, 0.), head=PRESSED + (.010, .008, 0.), rot=rot_z(6.))["score"] == 1.


def test_box_knocked_over_scores_zero_for_the_rest_of_the_trial():
    grade = JackGrade()
    run(grade, 5, rot=rot_x(40.))
    assert run(grade, 100)["score"] == 0.


def test_unreadable_state_scores_zero():
    grade = JackGrade()
    assert grade.update(float("nan"), PRESSED, BOX, UP, AWAY, False)["score"] == 0.


def test_head_state_is_measured_in_the_box_frame():
    turned = rot_z(8.)
    head = BOX + turned @ (np.array(PRESSED) - BOX)
    top, within = grading.head_state(head, BOX, turned)
    assert top < 0. and within
    assert abs(grading.clearance([(L.BOX_X, 0., L.BOX_PUBLISHED[2] + .05)], BOX, UP) - .05) < 1e-9
