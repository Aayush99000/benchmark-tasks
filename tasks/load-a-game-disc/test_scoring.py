"""Offline checks of the scoring rule, the case's snap and the disc geometry (no simulator): python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import disckit as K  # noqa: E402
import task as T  # noqa: E402


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def case():
    return next(o for o in T.OBJECTS if o.name == "game_case")


def test_nothing_done_scores_zero():
    assert T.score({}) == 0.0
    assert T._STATE["latched"]


def test_tiers():
    T._STATE["opened"] = True
    assert T.score({}) == 0.25
    T._STATE["off_hub"] = True
    assert T.score({}) == 0.5
    T._STATE["in_slot"] = True
    assert T.score({}) == 0.75
    T._STATE["taken"] = True
    assert T.score({}) == 1.0
    assert T.success({})


def test_a_freed_catch_alone_earns_nothing():
    T._STATE["latched"] = False                                     # the lip lifted, the cover not swung open
    assert T.score({}) == 0.0


def test_opened_means_the_cover_swung_upright_with_the_tray_flat_on_the_box():
    home, level = np.array([T.HUB_XY[0], T.HUB_XY[1], T.TOP + T.STAND_H]), (0.0, 0.0, 0.0, 1.0)
    assert not T.opened(math.radians(60), home, level)
    assert T.opened(math.radians(90), home, level)
    assert T.opened(math.radians(180), home, level)
    tipped = (0.0, math.sin(math.radians(-20)), 0.0, math.cos(math.radians(-20)))       # tray tipped up 40 degrees
    assert not T.opened(math.radians(180), home, tipped)
    assert not T.opened(math.radians(180), home - [0.0, 0.0, T.STAND_H], level)          # knocked off the box


def test_a_dropped_disc_zeroes_the_trial():
    T._STATE.update(opened=True, off_hub=True, in_slot=True, taken=True, dropped=True)
    assert T.score({}) == 0.0


def test_the_snap_needs_the_tray_held_down():
    # the catch holds to SNAP_PULL at the lip; the whole closed case tips up at well under half of that
    assert T.SNAP_HOLD * T.SNAP_ANGLE / T.SNAP_ARM == pytest.approx(T.SNAP_PULL)
    assert T.lift_to_tip_the_case() < 0.4 * T.SNAP_PULL
    assert T.SNAP_ARM * math.sin(T.SNAP_ANGLE) < 0.005              # the catch lets go within 5 mm of lift at the lip
    assert T.OPEN_AT > 10 * T.SNAP_ANGLE                            # "opened" is far past "catch freed"


def test_the_closed_case_fits_together():
    c = case()
    assert c.thick - c.plate > K.BUTTON_TOP                         # the cover clears the hub's button
    assert c.wall > K.FLOOR + K.SEAT_H + K.DISC_T                   # the tray's rim stands above the disc
    assert c.mass > c.cover_mass + c.spine_mass
    corner = (abs(c.recess_x) - c.ledge[0] / 2, c.tray[1] / 2 - c.ledge[1])      # the ledge's corner nearest the hub
    assert math.hypot(*corner) > K.DISC_R + 0.002                   # the finger ledge is clear of the disc
    assert c.recess[0] > c.ledge[0] and c.recess[1] > c.ledge[1]    # the ledge shows through the cover's recess
    assert c.notch[1] > c.tray[0] / 2 - K.DISC_R                    # the thumb cut-out reaches under the disc's edge


def test_geometry_of_a_flat_disc():
    pos = T.disc_home()
    axis, low, depth = T.disc_geometry(pos, (0.0, 0.0, 0.0, 1.0))
    assert axis[2] == pytest.approx(1.0)
    assert low == pytest.approx(pos[2] - K.DISC_T / 2)
    assert depth == pytest.approx(pos[0] + K.DISC_R - T.FACE_X)


def test_geometry_of_an_upright_disc_in_the_slot():
    pos = np.array([T.FACE_X + 0.02, T.SLOT_Y, T.TOP + T.SLOT_Z])
    axis, low, depth = T.disc_geometry(pos, T.UPRIGHT_Q)
    assert abs(axis[1]) == pytest.approx(1.0)
    assert low == pytest.approx(pos[2] - K.DISC_R)
    assert depth == pytest.approx(0.08)
    assert T.centred(pos, axis)
    assert low > T.TOP + T.SLOT_Z - T.SLOT_H / 2                    # a centred disc clears the slot's bottom
    off = pos + [0.0, 0.005, 0.0]
    assert not T.centred(off, axis)
    tilted = (math.sin(math.radians(60)), 0.0, 0.0, math.cos(math.radians(60)))   # 30 degrees off upright
    assert not T.centred(pos, T.disc_geometry(pos, tilted)[0])


def flat_in_its_case(x, y, z):
    """The disc lying flat (as it does on the hub of a shut case) with its centre at x, y and z above the table."""
    return np.array([x, y, T.TOP + z]), (0.0, 0.0, 0.0, 1.0)


def test_a_shut_case_carried_over_the_console_is_not_in_the_slot():
    # the disc still on its hub, flat, 2 cm over the console's top, its leading edge 5 cm beyond the front face and
    # its centre on the slot's line: the old rule (forward distance and sideways position only) called this "inserted"
    pos, quat = flat_in_its_case(T.FACE_X + 0.05 - K.DISC_R, T.SLOT_Y, T.CONSOLE["height"] + 0.02)
    axis, low, forward = T.disc_geometry(pos, quat)
    assert forward == pytest.approx(0.05) and abs(pos[1] - T.SLOT_Y) < 0.004
    assert T.slot_depth(pos, quat) == -1.0
    assert not T.went_in(pos, quat)
    # ... and the same at every height from the table to well over the console, and at every forward distance
    for z in np.arange(0.0, T.CONSOLE["height"] + 0.10, 0.005):
        for ahead in (0.02, 0.05, 0.08, 0.12):
            assert not T.went_in(*flat_in_its_case(T.FACE_X + ahead - K.DISC_R, T.SLOT_Y, z))


def test_an_upright_disc_over_or_beside_the_slot_is_not_in_it():
    over = np.array([T.FACE_X - K.DISC_R + 0.05, T.SLOT_Y, T.TOP + T.CONSOLE["height"] + K.DISC_R + 0.01])
    assert not T.went_in(over, T.UPRIGHT_Q)                                   # upright, but over the console
    high = np.array([T.FACE_X - K.DISC_R + 0.05, T.SLOT_Y, T.TOP + T.SLOT_Z + 0.03])
    assert not T.went_in(high, T.UPRIGHT_Q)                                   # 3 cm too high for the slot's top end
    beside = np.array([T.FACE_X - K.DISC_R + 0.05, T.SLOT_Y + 0.010, T.TOP + T.SLOT_Z])
    assert not T.went_in(beside, T.UPRIGHT_Q)                                 # 1 cm to one side of the slot


def test_only_an_upright_disc_inside_the_slot_counts_as_in_it():
    def at(depth, dy=0.0, dz=0.0):
        return np.array([T.FACE_X - K.DISC_R + depth, T.SLOT_Y + dy, T.TOP + T.SLOT_Z + dz])

    assert T.slot_depth(at(0.030), T.UPRIGHT_Q) == pytest.approx(0.030)
    assert T.went_in(at(0.020), T.UPRIGHT_Q) and T.went_in(at(0.069), T.UPRIGHT_Q)
    assert not T.went_in(at(0.019), T.UPRIGHT_Q)                              # not yet 2 cm in
    assert T.went_in(at(0.030, dy=0.002, dz=0.004), T.UPRIGHT_Q)              # within the slot's clearances
    assert not T.went_in(at(0.030, dy=0.004), T.UPRIGHT_Q)                    # through the slot's side
    assert T.went_in(at(0.030, dz=0.008), T.UPRIGHT_Q)                        # 3 cm in, the disc's cap still clears the top end
    assert not T.went_in(at(0.030, dz=0.030), T.UPRIGHT_Q)                    # through the slot's top end
    lean = math.radians(20)                                                   # leaning 20 degrees out of upright
    leaning = (math.sin(math.pi / 4 + lean / 2), 0.0, 0.0, math.cos(math.pi / 4 + lean / 2))
    assert abs(T.disc_geometry(at(0.030), leaning)[0][1]) == pytest.approx(math.cos(lean))
    assert not T.went_in(at(0.030), leaning)
    assert not T.went_in(at(0.030), (0.0, 0.0, 0.0, 1.0))                     # lying flat at the slot's height


def test_constants_are_consistent():
    assert T.IN_SLOT < T.TAKE_AT < 2 * K.DISC_R
    assert T.TAKE_TILT < T.IN_TILT                                   # a disc the drive can take is also "in the slot"
    assert T.TAKE_OFF[0] <= T.SLOT_W / 2 + T.SLOT_SLACK and T.TAKE_OFF[1] + K.DISC_R <= T.SLOT_H / 2 + 0.008
    assert T.SEATED - K.DISC_R > T.FACE_X                            # a seated disc is wholly inside
    assert T.SEATED + K.DISC_R < T.FACE_X + T.BAY                    # ... and inside the drive bay
    assert 2 * K.DISC_R < T.SLOT_H and K.DISC_T < T.SLOT_W < T.MOUTH_W
    assert K.HOLE_R > K.POST_R and K.TOOTH_OUT > K.HOLE_R > K.TOOTH_OUT - K.TOOTH_STROKE   # the teeth hold, then let go
    assert K.BUTTON_TRAVEL > -T.PRESSED
