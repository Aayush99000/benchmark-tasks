"""Offline tests of the clothes-peg task: how the photo hangs, what counts as a peg holding it, the trial's record,
the scoring rule and the layout.

    python -m pytest test_scoring.py

No simulator is needed.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import pegkit as K  # noqa: E402
import task as T  # noqa: E402

W, THICK, TALL = T.PHOTO
ON_PHOTO = math.atan((K.GAP - THICK) / K.PIVOT_Z)        # the spring joint's angle with the tips shut on the photo


def rot_about_x(deg: float) -> np.ndarray:
    return T.rot_x(math.radians(deg))


def level_photo():
    return T.photo_pose(0.0)


def hanging_peg(along: float, spring: float = ON_PHOTO, roll_deg: float = 0.0, photo=None):
    """A peg hanging on the wire with its tips' gap on the photo's sheet, ``along`` from the photo's middle toward its
    free end, rolled ``roll_deg`` along the wire about where its tips pinch."""
    pos, quat = level_photo() if photo is None else photo
    z = -K.PIVOT_Z + T.TIP_UP
    lo, hi = T.jaw_faces(z, spring)
    rot = rot_about_x(roll_deg) @ K._rotation(T.PEG_QUAT)
    pinch = np.array([T.beside(), float(pos[1]) - along, T.TOP + T.LINE_Z - T.CLIP_DEPTH + T.TIP_UP])
    return pinch - rot @ np.array([(lo + hi) / 2, 0.0, z]), K._quat(rot)


def fresh() -> dict:
    T._clear()
    return dict(T._STATE, past=[])


def run(s, n, **kw):
    base = dict(taken=False, angle=T.hang_angle(), corner_off=0.0, pegged=False, let_go=True, still=True)
    base.update(kw)
    for _ in range(n):
        T.advance(s, **base)


# -- how the photo hangs ---------------------------------------------------------------------------------------------------

def test_held_by_one_corner_the_photo_hangs_with_its_middle_below_the_peg():
    assert math.degrees(T.hang_angle()) == pytest.approx(60.9, abs=0.1)
    pos, quat = T.photo_pose(T.hang_angle())
    assert np.allclose(T.photo_point(pos, quat, T.pivot_local()), T.pivot_point())
    assert pos[1] == pytest.approx(T.PIVOT_Y, abs=1e-6)                      # its middle straight below its peg
    assert math.degrees(T.photo_angle(quat)) == pytest.approx(60.9, abs=0.1)


def test_its_free_corner_hangs_ten_centimetres_below_the_wire_and_nothing_touches_the_table():
    pos, quat = T.photo_pose(T.hang_angle())
    free_top = T.photo_point(pos, quat, (W / 2, 0.0, TALL / 2))
    assert T.TOP + T.LINE_Z - free_top[2] == pytest.approx(0.101, abs=0.002)
    corners = [T.photo_point(pos, quat, (a * W / 2, 0.0, b * TALL / 2)) for a in (-1, 1) for b in (-1, 1)]
    assert min(c[2] for c in corners) - T.TOP > 0.010


def test_level_it_reaches_twelve_millimetres_above_the_wire_just_in_front_of_it():
    pos, quat = level_photo()
    assert T.photo_angle(quat) == pytest.approx(0.0, abs=1e-9)
    assert T.photo_top_edge(pos, quat)[2] - (T.TOP + T.LINE_Z) == pytest.approx(T.ABOVE)
    assert T.LINE_X - T.LINE_R - (pos[0] + THICK / 2) == pytest.approx(T.IN_FRONT)
    assert K._rotation(quat)[:, 1] @ np.array([1.0, 0.0, 0.0]) == pytest.approx(1.0)   # its picture faces the arms


def test_photo_angle_reads_any_swing():
    for deg in (-5.0, 0.0, 10.0, 45.0, 60.9):
        assert math.degrees(T.photo_angle(T.photo_pose(math.radians(deg))[1])) == pytest.approx(deg, abs=1e-6)


# -- the pegs ---------------------------------------------------------------------------------------------------------

def test_a_peg_shuts_until_its_tips_meet_and_presses_them_together():
    assert K.GAP - K.PIVOT_Z * math.tan(K.SHUT) == pytest.approx(0.0001)     # 0.1 mm between the tips, shut on nothing
    force = K.Peg("p", pos=(0, 0, 0), size=(0.02, 0.01, 0.07), mass=0.006).spring * math.radians(7.0) / K.PIVOT_Z
    assert force == pytest.approx(1.0, abs=0.05)                             # about 1 N at the tips
    assert 0 < ON_PHOTO < K.SHUT


def tipped(deg: float) -> tuple:
    """A peg's quaternion when it is tipped ``deg`` across the rim."""
    c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return K._quat(np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]]) @ K._rotation(T.PEG_QUAT))


def astride(across: float, up: float, tip_deg: float = 0.0, y: float = T.PEGS_Y[0]) -> tuple:
    """A peg astride the basket's rim: the rim's top edge ``across`` from the peg's middle plane toward its sprung leg
    and ``up`` from its jaw tips, the peg tipped ``tip_deg`` across the rim."""
    quat = tipped(tip_deg)
    rim = np.array([T.BASKET_X, y, T.TOP + T.RIM_Z])
    return rim - K._rotation(quat) @ np.array([across, 0.0, -K.PIVOT_Z + up]), quat


def test_a_spare_peg_is_taken_only_when_it_is_off_the_rim_and_not_in_the_basket():
    rest, squeezed = math.radians(-5.0), math.radians(-14.5)
    start = T.spare_start(0)
    assert T.peg_on_rim(start, T.PEG_QUAT, rest) and not T.taken_off(start, T.PEG_QUAT, rest)
    assert not T.taken_off(start + [0.0, 0.025, 0.0], T.PEG_QUAT, rest)      # slid 2.5 cm along the rim
    for tip in (-10.0, 0.0, 10.0):                                           # squeezed open and tipped, not lifted:
        for across in (0.0012, 0.003, 0.005):                                # the rim anywhere between the open jaws
            pos, quat = astride(across, T.RIM_DEPTH, tip)
            assert not T.taken_off(pos, quat, squeezed), (tip, across)
            pos, quat = astride(across, T.RIM_DEPTH, tip, y=T.PEGS_Y[0] + 0.025)   # ... and slid along the rim like that
            assert not T.taken_off(pos, quat, squeezed), (tip, across)
    pos, quat = astride(0.003, -0.005)                                       # lifted: its tips 5 mm clear of the rim
    assert T.taken_off(pos, quat, squeezed)
    assert T.taken_off(start + [0.0, 0.0, 0.040], T.PEG_QUAT, squeezed)
    assert not T.taken_off(start + [0.050, 0.0, -0.060], T.PEG_QUAT, K.SHUT)  # dropped into the basket
    assert T.taken_off(start + [-0.050, 0.0, -0.060], T.PEG_QUAT, K.SHUT)    # lying on the table in front of it


def test_a_peg_hanging_on_the_wire_and_pinching_the_free_corner_holds_it():
    photo = level_photo()
    pos, quat = hanging_peg(W / 2 - 0.010)
    g = T.grip_check(pos, quat, ON_PHOTO, *photo)
    assert g["closed"] and g["wire"] and g["photo"]
    assert T.holds_free_corner(pos, quat, ON_PHOTO, *photo)


def test_a_crooked_peg_is_judged_where_the_wire_passes_it():
    photo = level_photo()
    pos, quat = hanging_peg(W / 2 - 0.010, roll_deg=20.0)
    assert abs(pos[1] - (photo[0][1] - (W / 2 - 0.010))) > 0.005             # its pivot is well to one side of its tips
    assert T.wire_in_peg(pos, quat) is not None and abs(T.wire_in_peg(pos, quat)[1]) < 1e-9
    assert T.holds_free_corner(pos, quat, ON_PHOTO, *photo)


def test_a_peg_on_the_wire_beside_the_photo_does_not_hold_it():
    photo = level_photo()
    pos, quat = hanging_peg(W / 2 + 0.010, spring=K.SHUT)                    # 1 cm past the free end, shut on nothing
    g = T.grip_check(pos, quat, K.SHUT, *photo)
    assert g["wire"] and not g["photo"]
    assert not T.holds_free_corner(pos, quat, K.SHUT, *photo)


def test_a_peg_on_the_wire_with_the_photo_hanging_below_does_not_hold_it():
    hanging = T.photo_pose(T.hang_angle())
    pos, quat = hanging_peg(W / 2 - 0.010, spring=K.SHUT)                    # where the corner would be, photo not lifted
    assert not T.holds_free_corner(pos, quat, K.SHUT, *hanging)


def test_a_peg_in_the_middle_of_the_photo_is_not_at_its_corner():
    photo = level_photo()
    pos, quat = hanging_peg(0.0)
    g = T.grip_check(pos, quat, ON_PHOTO, *photo)
    assert g["wire"] and g["photo"] and not T.holds_free_corner(pos, quat, ON_PHOTO, *photo)


def test_a_peg_still_squeezed_open_holds_nothing():
    photo = level_photo()
    spring = math.radians(-14.0)
    pos, quat = hanging_peg(W / 2 - 0.010, spring=spring)
    assert not T.grip_check(pos, quat, spring, *photo)["closed"]
    assert not T.holds_free_corner(pos, quat, spring, *photo)


def test_tips_shut_in_front_of_the_photo_do_not_pinch_it():
    photo = level_photo()
    pos, quat = hanging_peg(W / 2 - 0.010, spring=K.SHUT)
    pos = pos - [0.0010, 0.0, 0.0]                                            # the shut tips 1 mm in front of the sheet
    assert not T.grip_check(pos, quat, K.SHUT, *photo)["photo"]


def test_no_wire_beyond_the_posts():
    pos, quat = hanging_peg(0.0)
    pos = pos + [0.0, T.LINE_Y[1] + 0.05 - pos[1], 0.0]
    assert T.wire_in_peg(pos, quat) is None


def test_distance_to_the_photo():
    pos, quat = level_photo()
    assert T.gap_to_photo(pos, pos, quat) == 0.0
    assert T.gap_to_photo(pos + [-0.050, 0.0, 0.0], pos, quat) == pytest.approx(0.050 - THICK / 2)
    assert T.gap_to_photo(pos + [0.0, 0.0, TALL / 2 + 0.030], pos, quat) == pytest.approx(0.030)


# -- the trial's record ----------------------------------------------------------------------------------------------------

def test_taken_needs_a_quarter_second_off_the_rim():
    s = fresh()
    run(s, T.TAKEN_STEPS - 1, taken=True)
    run(s, 1, taken=False)                                                    # a momentary reading does not count
    run(s, T.TAKEN_STEPS - 1, taken=True)
    assert not s["taken"]
    run(s, 1, taken=True)
    assert s["taken"]


def test_raised_needs_a_second_within_ten_degrees_of_level():
    s = fresh()
    run(s, T.RAISED_STEPS - 1, angle=math.radians(9.0))
    assert not s["raised"]
    run(s, 1, angle=math.radians(9.0))
    assert s["raised"]
    s = fresh()
    run(s, 100, angle=math.radians(11.0))                                     # never close enough
    assert not s["raised"]
    s = fresh()
    for _ in range(5):                                                        # swung through level, never held there
        run(s, T.RAISED_STEPS - 1, angle=0.0)
        run(s, 1, angle=math.radians(30.0))
    assert not s["raised"]


def test_full_marks_need_two_seconds_pegged_level_at_rest_and_let_go():
    good = dict(taken=True, angle=math.radians(1.0), pegged=True, let_go=True, still=True)
    s = fresh()
    run(s, T.HANG_STEPS - 1, **good)
    assert not s["held"]
    run(s, 1, **good)
    assert s["held"]
    for bad in (dict(pegged=False), dict(let_go=False), dict(still=False), dict(angle=math.radians(6.0))):
        s = fresh()
        run(s, 200, **{**good, **bad})
        assert not s["held"], bad


def test_a_photo_off_its_first_peg_is_latched_and_earns_nothing_more():
    s = fresh()
    run(s, 1, corner_off=T.FALLEN - 0.001)
    assert not s["fallen"]
    run(s, 1, corner_off=T.FALLEN)
    assert s["fallen"]
    run(s, 200, angle=0.0, pegged=True)                                       # lying level on the table
    assert s["fallen"] and not s["raised"] and not s["held"]


# -- scoring --------------------------------------------------------------------------------------------------------------

def test_the_score_ladder():
    f = T.score_from
    assert f(False, False, False, False) == 0.0
    assert f(True, False, False, False) == 0.25                               # a peg taken
    assert f(False, True, False, False) == 0.25                               # the photo raised
    assert f(True, True, False, False) == 0.5                                 # both, but not pegged
    assert f(True, True, True, False) == 1.0
    assert f(True, True, True, True) == 0.0                                   # the photo came off its first peg
    assert f(True, True, False, True) == 0.0


def test_score_reads_the_trial_record():
    T._clear()
    T._STATE.update(taken=True, raised=True)
    assert T.score({}) == 0.5 and not T.success({})
    T._STATE.update(held=True)
    assert T.score({}) == 1.0 and T.success({})
    T._STATE.update(fallen=True)
    assert T.score({}) == 0.0
    T._clear()


# -- the layout ------------------------------------------------------------------------------------------------------------

def test_the_scene_starts_with_the_photo_hanging_and_the_spares_on_the_rim():
    by = {o.name: o for o in T.OBJECTS}
    assert len(by) == len(T.OBJECTS)
    photo = by["photo"]
    assert T.photo_angle(photo.quat) == pytest.approx(T.hang_angle())
    assert np.allclose(T.photo_point(photo.pos, photo.quat, T.pivot_local()), T.pivot_point())
    assert photo.no_touch == (T.FIRST,)
    first = by[T.FIRST]
    assert np.allclose(first.pos, T.peg_place(+1))
    wire = T.wire_in_peg(first.pos, first.quat)
    assert -K.PIVOT_Z + K.NOTCH_Z[0] < wire[2] < -K.PIVOT_Z + K.NOTCH_Z[1]   # the wire is in its jaws' groove
    for i, name in enumerate(T.SPARES):
        assert T.rim_in_peg(by[name].pos, by[name].quat) is not None         # the rim passes through its jaws
        assert not T.in_basket(by[name].pos)
    assert not T.holds_free_corner(first.pos, first.quat, K.SHUT, photo.pos, photo.quat)


def test_the_first_pegs_hold_is_declared():
    assert set(K._JOINTS) == {"hold", "pivot"}
    assert K._JOINTS["pivot"].kind == "pivot" and K._JOINTS["pivot"].force == T.PHOTO_HOLD
    assert np.allclose(K._JOINTS["pivot"].anchor1, T.pivot_local())
    assert K._JOINTS["hold"].body1 == (T.FIRST, "leg_a") and K._JOINTS["hold"].force == T.PEG_HOLD


def test_the_second_place_is_within_two_centimetres_of_the_free_end():
    free_end = T.PIVOT_Y - (W - T.PEG_IN)
    assert T.peg_place(-1)[1] - free_end == pytest.approx(T.PEG_IN)
    assert T.LINE_Y[0] < T.peg_place(-1)[1] < T.peg_place(+1)[1] < T.LINE_Y[1]
