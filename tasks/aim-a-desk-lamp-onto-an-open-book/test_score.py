"""Scorer unit tests: synthetic object states built from the lamp's own kinematics, no simulator. Created 11_16_26.

Run: python -m pytest test_score.py (plain Python with NumPy and the yam_arena package importable)."""

import math

import numpy as np
import pytest

import task as T
from desk_lamp import mech as M
from yam_arena.task_file import ObjectState

DT = 0.05
FAR = (0.10, 0.40, T.TOP + 0.20)  # a parked hand, well away from the lamp


def _quat(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w < 1e-6:
        raise ValueError("rotation too close to 180 deg for this helper")
    return np.array(((r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w))


def _st(p, q=(0.0, 0.0, 0.0, 1.0)):
    p = np.asarray(p, float)
    z = np.zeros(3)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=z, ang_vel=z, low=p, high=p)


def objects(pose, left=FAR, right=(0.10, -0.40, T.TOP + 0.20), book_dxy=(0.0, 0.0), base_dxy=(0.0, 0.0)):
    w = M.fk(T.LAMP_XY, *pose)
    lift = np.array((0.0, 0.0, T.TOP))
    k, r = T.head_frame(pose)
    q = M.joints(*pose)
    qd = [math.degrees(q[j]) for j in ("swivel", "shoulder", "elbow", "head")]
    lamp_q = ObjectState(pos=np.radians(qd[:3]), quat_xyzw=np.array((0, 0, 0, 1.0)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=np.array((math.radians(qd[3]), 0, 0)), high=np.array((math.radians(qd[3]), 0, 0)))
    return {
        "left_tcp": _st(left), "right_tcp": _st(right),
        "lamp_base": _st((T.LAMP_XY[0] + base_dxy[0], T.LAMP_XY[1] + base_dxy[1], T.TOP)),
        "lamp_lower": _st(w["S"] + lift), "lamp_upper": _st(w["E"] + lift), "lamp_head": _st(k, _quat(r)),
        "lamp_q": lamp_q,
        "book": _st((T.BOOK_XY[0] + book_dxy[0], T.BOOK_XY[1] + book_dxy[1], T.TOP)),
    }


def run(seq):
    """Feed (objects, seconds) pairs through note_step; return the final score."""
    T.reset_latch()
    last = None
    for obj, secs in seq:
        for _ in range(int(round(secs / DT))):
            T.note_step(obj, DT)
            last = obj
    return T.score(last)


def aimed(err_mm, mouth_h=T.GOAL_MOUTH_H, tilt=T.GOAL_TILT):
    """A pose whose beam lands err_mm from the dot, straight out from the lamp along its swivel heading."""
    d = T.DOT_XY - T.LAMP_XY
    spot = T.DOT_XY + d / np.linalg.norm(d) * err_mm / 1000.0
    return M.aim_pose(T.LAMP_XY, spot, T.PAGE_H, mouth_h, tilt)


def test_goal_pose_reading():
    err, h = T.reading(objects(T.GOAL_POSE))
    assert err < 1e-6 and h == pytest.approx(T.GOAL_MOUTH_H, abs=1e-6)


def test_start_scores_zero_doing_nothing():
    err, _ = T.reading(objects(T.START_POSE))
    assert err > 0.100
    assert run([(objects(T.START_POSE), 10.0)]) == 0.0


@pytest.mark.parametrize("err_mm,expect", [(0, 1.0), (29, 1.0), (31, 0.5), (59, 0.5), (61, 0.25), (99, 0.25), (101, 0.0), (300, 0.0)])
def test_bands(err_mm, expect):
    assert run([(objects(aimed(err_mm)), 4.0)]) == expect


@pytest.mark.parametrize("err_mm,expect", [(0, 0.25), (50, 0.25), (90, 0.25), (120, 0.0)])
def test_low_shade_capped(err_mm, expect):
    assert run([(objects(aimed(err_mm, mouth_h=0.25)), 4.0)]) == expect


def test_needs_three_seconds_clear():
    assert run([(objects(T.GOAL_POSE), 2.5)]) == 0.0
    assert run([(objects(T.GOAL_POSE), 3.1)]) == 1.0


def test_hand_still_on_the_shade_scores_zero():
    k, r = T.head_frame(T.GOAL_POSE)
    on_shade = k + r @ np.array((M.KNUCKLE_TO_AXIS, 0.0, -0.05))
    assert run([(objects(T.GOAL_POSE, left=on_shade), 6.0)]) == 0.0


def test_hand_holding_the_arm_scores_zero():
    p, _ = T.hand_on_arm(T.GOAL_POSE)
    assert run([(objects(T.GOAL_POSE, right=p), 6.0)]) == 0.0


def test_hand_back_resets_the_clock():
    p, _ = T.hand_on_arm(T.GOAL_POSE)
    assert run([(objects(T.GOAL_POSE), 2.0), (objects(T.GOAL_POSE, right=p), 0.2), (objects(T.GOAL_POSE), 2.0)]) == 0.0


def test_lamp_still_moving_resets_the_clock():
    seq = [(objects(aimed(5 * i)), 0.5) for i in range(8)]  # the beam creeps 5 mm every 0.5 s
    assert run(seq) == 0.0


def test_book_moved_scores_zero():
    # the notebook slid under the beam: the dot ends up where the beam is, but the notebook moved 40 mm
    pose = aimed(40)
    d = T.DOT_XY - T.LAMP_XY
    shift = d / np.linalg.norm(d) * 0.040
    assert run([(objects(pose), 0.5), (objects(pose, book_dxy=shift), 4.0)]) == 0.0


def test_book_nudged_a_little_still_scores():
    assert run([(objects(T.GOAL_POSE, book_dxy=(0.005, 0.0)), 4.0)]) == 1.0


def test_base_dragged_scores_zero():
    assert run([(objects(T.GOAL_POSE), 0.5), (objects(T.GOAL_POSE, base_dxy=(0.0, 0.03)), 4.0)]) == 0.0


def test_transient_bump_that_returns_still_scores():
    # the reviewer's case: the notebook and the base are knocked 30 mm mid-trial but end where they started; only the
    # final poses count, as in the real trial
    seq = [(objects(T.GOAL_POSE), 0.5), (objects(T.GOAL_POSE, book_dxy=(0.03, 0.0), base_dxy=(0.0, 0.03)), 0.5),
           (objects(T.GOAL_POSE), 4.0)]
    assert run(seq) == 1.0


def test_shade_facing_up_has_no_spot():
    pose = (T.GOAL_POSE[0], T.GOAL_POSE[1], T.GOAL_POSE[2], math.radians(120.0))
    err, _ = T.reading(objects(pose))
    assert err == math.inf
    assert run([(objects(pose), 4.0)]) == 0.0


def test_score_bands_ordered():
    lims = [b[0] for b in T.BANDS]
    vals = [b[1] for b in T.BANDS]
    assert lims == sorted(lims) and vals == sorted(vals, reverse=True)
