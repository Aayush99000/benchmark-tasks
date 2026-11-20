"""The scorer on hand-built end states: every tier, and every cheap way to a higher tier than deserved."""

import numpy as np

from rdb_till_note import grading, till_model as m

EYE = np.eye(3)
AWAY = [np.array((.15, .34, .25)), np.array((.15, -.34, .25))]       # both grippers far from everything
LANE = m.COMPARTMENT_Y[m.STOCKED]
HOME = m.STACK_X - m.NOTE_L / 2                                      # note's near edge when it lies on the stack


def state(*, near=HOME, lane=LANE, height=m.STACK_TOP_Z + m.NOTE_T / 2, drawer_open=0., hold=m.HOLD_ON_NOTE,
          tcp=AWAY, stack_shift=(0., 0., 0.), tilt=None, on_case=False):
    """A flat note whose near edge is ``near`` from the drawer front (drawer frame), unless on the case."""
    x0 = m.drawer_x(drawer_open)
    strips = m.NOTE_START.copy() if on_case else m.strip_centres_flat(x0 + near, lane, height)
    rotations = np.array([EYE] * m.STRIPS)
    if tilt is not None:                                              # stand the two nearest strips up
        c, s = np.cos(tilt), np.sin(tilt)
        up = np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))        # the strip rises toward the robot
        for i in (1, 0):
            rotations[i] = up
            strips[i] = strips[i + 1] - rotations[i + 1][:, 0] * m.STRIP_L / 2 - up[:, 0] * m.STRIP_L / 2
    stack = np.array((x0 + m.STACK_X, LANE, m.BILL_FLOOR_Z + m.STACK_SIZE[2] / 2)) + stack_shift
    angles = np.array([m.HOLD_ON_FLOOR] * 5)
    angles[m.STOCKED] = hold
    return dict(strips=strips, strip_rot=rotations, stack=stack, drawer_open=drawer_open, hold_angles=angles, tcp=tcp)


def final(**kwargs):
    grade = grading.TillNoteGrade()
    for _ in range(grading.HOLD_TICKS + 3):
        result = grade.update(**state(**kwargs))
    return result


def test_full_marks_need_the_one_second_hold():
    grade = grading.TillNoteGrade()
    scores = [grade.update(**state())["score"] for _ in range(grading.HOLD_TICKS + 2)]
    assert scores[0] == .75 and scores[grading.HOLD_TICKS - 1] == .75 and scores[-1] == 1.


def test_tiers():
    assert final(on_case=True)["score"] == 0.                                     # nothing done
    assert final(on_case=True, drawer_open=0.)["score"] == 0.                     # drawer shut, note left on the case
    assert final(drawer_open=m.TRAVEL)["score"] == .75                            # all but the drawer
    assert final()["score"] == 1.
    assert final(drawer_open=.004)["score"] == .75                                # not quite home
    assert final(drawer_open=.0015)["score"] == 1.


def test_note_merely_in_the_drawer():
    coins = final(near=m.COIN_X[0] + .002, height=m.RIM_Z + .002, hold=m.HOLD_ON_STACK)
    assert coins["score"] == .25 and not coins["note_in_middle_compartment"]      # lying across the coin cups
    other = final(lane=m.COMPARTMENT_Y[1], height=m.BILL_FLOOR_Z + m.NOTE_T / 2, hold=m.HOLD_ON_STACK)
    assert other["score"] == .25 and other["note_compartment"] == 1               # an empty compartment
    astride = final(lane=LANE + .030, height=m.RIM_Z + m.NOTE_T / 2, hold=m.HOLD_ON_STACK)
    assert astride["score"] == .25                                                # across the compartment wall
    assert final(near=-.050, height=m.PANEL_Z[1] + .001)["score"] == 0.           # hanging over the drawer front


def test_note_on_top_of_the_hold_down_is_not_under_it():
    # The arm rests on the stack; the note lies on the arm, 12 mm up at the tab end.
    on_top = final(height=m.STACK_TOP_Z + .012, hold=m.HOLD_ON_STACK)
    assert on_top["score"] == .5 and not on_top["note_under_hold_down"]
    # Dropped on the stack beside a hold-down that is still held up or stuck up.
    assert final(hold=np.radians(20.))["score"] == .5


def test_note_must_be_flat():
    standing = final(tilt=np.radians(30.))                       # its near end 20 mm up
    assert standing["score"] == .5 and not standing["note_flat"] and standing["note_under_hold_down"]


def test_other_notes_must_stay_put():
    assert final(stack_shift=(-.008, 0., 0.))["score"] == .5
    assert final(stack_shift=(-.004, 0., 0.))["score"] == 1.


def test_a_gripper_still_at_the_note_or_the_drawer():
    x0 = m.drawer_x(m.TRAVEL)
    holding = [AWAY[0], np.array((x0 + HOME + .01, LANE, m.STACK_TOP_Z + .01))]
    assert final(drawer_open=m.TRAVEL, tcp=holding)["score"] == 0.               # note still in the pads
    pinning = [AWAY[0], np.array((m.FRONT_CLOSED_X - .010, -.10, .06))]
    leaning = final(tcp=pinning)
    assert leaning["score"] == .75 and not leaning["grippers_clear"]              # holding the drawer shut


def test_drawer_or_note_still_moving():
    grade = grading.TillNoteGrade()
    for k in range(grading.HOLD_TICKS + 5):
        result = grade.update(**state(drawer_open=.0018 - .0001 * (k % 12)))      # the drawer keeps creeping
    assert result["score"] == .75 and not result["still_for_1s"]


def test_score_can_fall_again():
    grade = grading.TillNoteGrade()
    for _ in range(grading.HOLD_TICKS + 3):
        grade.update(**state())
    assert grade.update(**state(drawer_open=.05))["score"] == .75                 # pulled open again
    assert grade.update(**state(on_case=True))["score"] == 0.
