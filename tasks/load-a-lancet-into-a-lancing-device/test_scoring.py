"""Offline tests of the lancing-device task: the fits, the device's own behaviour, the scoring rule and the layout.

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

import lancetkit as K  # noqa: E402
import task as T  # noqa: E402

D0, DQ, LQ = T.device_start(), T.DEVICE_QUAT, T.LANCET_QUAT


def turned(quat, axis: str, deg: float) -> tuple:
    """``quat`` followed by a turn of ``deg`` about the part's own x, y or z."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    r = {"x": [[1, 0, 0], [0, c, -s], [0, s, c]], "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]], "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
    return K._quat(K._rotation(quat) @ np.array(r, float))


def fresh() -> dict:
    T._clear()
    return dict(T._STATE)


HOME = {"depth": K.SOCKET_DEPTH, "off": 0.0, "angle": 0.0}
OUT = {"depth": -0.05, "off": 0.0, "angle": 0.0}
ON = {"gap": 0.0, "off": 0.0, "angle": 0.0}
OFF = {"gap": 0.05, "off": 0.0, "angle": 0.0}


# -- the fits -------------------------------------------------------------------------------------------------------------

def test_a_lancet_pushed_home_is_seated_and_one_short_of_home_is_not():
    home = K.place(D0, DQ, (0.0, 0.0, K.SOCKET_DEPTH))
    assert K.seated(K.seat_fit(D0, DQ, home, LQ))
    assert not K.seated(K.seat_fit(D0, DQ, home - 0.002 * T.ALONG, LQ))                 # 2 mm short of the floor
    assert not K.seated(K.seat_fit(D0, DQ, home + [0.003, 0.0, 0.0], LQ))               # beside the socket
    assert not K.seated(K.seat_fit(D0, DQ, home, turned(LQ, "x", 15.0)))                # 15 degrees out of line
    assert not K.seated(K.seat_fit(D0, DQ, home, DQ))                                   # needle end first


def test_the_cap_starts_on_and_is_off_two_millimetres_out():
    on = D0 + K.NOSE_L * T.ALONG
    assert K.cap_on(K.cap_fit(D0, DQ, on, DQ))
    assert not K.cap_on(K.cap_fit(D0, DQ, on - 0.002 * T.ALONG, DQ))
    assert K.cap_fit(D0, DQ, on - 0.002 * T.ALONG, DQ)["gap"] == pytest.approx(0.002)


def test_the_barrel_reads_how_far_it_is_pulled_back():
    rest = D0 + K.BODY_END * T.ALONG
    assert K.barrel_pull(D0, DQ, rest) == pytest.approx(0.0, abs=1e-9)
    assert K.barrel_pull(D0, DQ, rest + 0.007 * T.ALONG) == pytest.approx(0.007)
    assert K.COCK_AT < K.COCK_TRAVEL


def test_twist_is_the_turn_about_the_lancets_own_axis_only():
    assert math.degrees(K.twist(LQ, turned(LQ, "z", 30.0))) == pytest.approx(30.0)
    assert math.degrees(K.twist(LQ, turned(LQ, "z", -50.0))) == pytest.approx(-50.0)
    assert abs(math.degrees(K.twist(LQ, turned(LQ, "x", 10.0)))) < 0.1                  # a bend is not a twist


def test_the_tab_is_joined_in_place_and_not_when_moved_off_or_bent_over():
    l0 = T.lancet_start()
    tab = K.place(l0, LQ, (0.0, 0.0, K.LANCET_L))
    assert K.joined(l0, LQ, tab, LQ)
    assert K.joined(l0, LQ, tab, turned(LQ, "z", 60.0))                                 # twisted on its neck: still in place
    assert not K.joined(l0, LQ, tab - 0.006 * T.ALONG, LQ)
    assert not K.joined(l0, LQ, tab, turned(LQ, "x", 30.0))


def test_the_needle_stands_out_of_the_lancets_front():
    base, tip = K.needle(T.lancet_start(), LQ)
    assert np.linalg.norm(tip - base) == pytest.approx(K.NEEDLE_L)
    assert np.allclose((tip - base) / K.NEEDLE_L, -T.ALONG)                             # in the tray it points away from the device


# -- the device's own behaviour ------------------------------------------------------------------------------------------------

def test_a_lancet_pushed_home_is_gripped_and_one_pulled_out_is_let_go():
    s = fresh()
    assert T.advance(s, HOME, OFF, True, 0.0, 0.0, 0.0, 1.0) == ["seat"] and s["seat"]
    assert T.advance(s, HOME, OFF, True, 0.0, 0.0, 0.0, 1.0) == []                      # made once
    T.advance(s, {"depth": 0.002, "off": 0.0, "angle": 0.0}, OFF, True, 0.0, 0.0, 0.0, 1.0)
    assert not s["seat"]


def test_the_tab_comes_off_when_twisted_far_enough_or_pulled_away():
    s = fresh()
    assert T.advance(s, HOME, OFF, True, math.radians(30.0), 0.0, 0.0, 1.0) == ["seat"] and not s["tab_off"]
    assert T.advance(s, HOME, OFF, True, math.radians(46.0), 0.0, 0.0, 1.0) == ["shear"] and s["tab_off"]
    s = fresh()
    T.advance(s, OUT, OFF, False, 0.0, 0.02, 0.0, 1.0)                                  # pulled apart
    assert s["tab_off"] and s["tab_clear"]


def test_the_cap_snaps_on_only_when_fully_home_and_lets_go_when_pulled():
    s = fresh()
    assert s["click"]                                                                    # it starts on
    T.advance(s, OUT, OFF, True, 0.0, 0.0, 0.0, 1.0)
    assert not s["click"]
    T.advance(s, OUT, {"gap": 0.004, "off": 0.0, "angle": 0.0}, True, 0.0, 0.0, 0.0, 1.0)
    assert not s["click"]
    assert T.advance(s, OUT, ON, True, 0.0, 0.0, 0.0, 1.0) == ["click"] and s["click"]


def test_cocking_latches_at_its_click():
    s = fresh()
    T.advance(s, OUT, ON, True, 0.0, 0.0, K.COCK_AT - 0.0005, 1.0)
    assert not s["cocked"]
    T.advance(s, OUT, ON, True, 0.0, 0.0, K.COCK_AT, 1.0)
    T.advance(s, OUT, ON, True, 0.0, 0.0, 0.0, 1.0)                                     # the barrel springs back
    assert s["cocked"] and s["pull_max"] == pytest.approx(K.COCK_AT)


def test_only_a_bare_needle_can_be_touched():
    s = fresh()
    T.advance(s, HOME, OFF, True, 0.0, 0.0, 0.0, 0.001)                                 # the tab is still on
    assert not s["stuck"]
    T.advance(s, HOME, OFF, True, math.radians(50.0), 0.003, 0.0, 0.001)                # off, but still over the needle
    assert s["tab_off"] and not s["stuck"]
    T.advance(s, HOME, OFF, False, 0.0, 0.02, 0.0, 0.0025)                              # bare, a gripper 2.5 mm away
    assert not s["stuck"]
    T.advance(s, HOME, OFF, False, 0.0, 0.02, 0.0, 0.0015)                              # 1.5 mm away
    assert s["stuck"]
    s = fresh()
    T.advance(s, HOME, OFF, False, 0.0, 0.02, 0.0, 1.0)
    T.advance(s, HOME, ON, False, 0.0, 0.02, 0.0, 0.0)                                  # the cap is back on: nothing to touch
    assert not s["stuck"]


# -- scoring --------------------------------------------------------------------------------------------------------------

def test_the_score_ladder():
    f = T.score_from
    assert f(False, False, True, False, False, False, True, True) == 0.0                # nothing done
    assert f(True, False, False, False, False, False, False, False) == 0.25             # lancet home, tab on
    assert f(True, True, False, False, False, False, False, False) == 0.5               # tab off, needle bare
    assert f(True, True, False, True, False, False, True, True) == 0.5                  # cocked with the cap left off
    assert f(True, True, True, False, False, False, True, True) == 0.75                 # capped, not cocked
    assert f(True, True, True, True, False, False, True, False) == 0.75                 # still in hand
    assert f(True, True, True, True, False, False, False, True) == 0.75                 # not laid down
    assert f(True, True, True, True, False, False, True, True) == 1.0


def test_what_zeroes_a_trial():
    f = T.score_from
    assert f(True, True, True, True, True, False, True, True) == 0.0                    # the bare needle was touched
    assert f(True, True, True, True, False, True, True, True) == 0.0                    # the device went off the table
    assert f(False, True, True, True, False, False, True, True) == 0.0                  # capped and cocked with no lancet in it
    assert f(False, True, False, False, False, False, True, True) == 0.0                # the tab pulled the lancet out


def test_score_reads_the_trial_record():
    T._clear()
    T._STATE.update(seat=True, tab_off=True, click=True, cocked=True, laid=True, settled=True)
    assert T.score({}) == 1.0 and T.success({})
    T._STATE.update(stuck=True)
    assert T.score({}) == 0.0 and not T.success({})
    T._clear()


# -- why it takes two hands, and why the tab is twisted ------------------------------------------------------------------------

def test_a_pull_on_the_cap_drags_a_loose_device():
    weight = (K.BODY_MASS + K.BARREL_MASS + K.CAP_MASS) * 9.81
    assert K.CAP_HOLD > 5 * weight                                                       # the snap holds far more than friction can


def test_a_pulled_tab_brings_the_lancet_out():
    assert K.NECK_FORCE > 2 * K.SEAT_HOLD


def test_the_neck_twists_off_at_a_torque_fingertips_can_give():
    grip = 1.0 * 0.5 * K.TAB_R * 2                                                       # 1 N of pinch at friction 0.5, both sides
    assert K.NECK_TORQUE < grip + 0.002
    assert math.degrees(K.NECK_TWIST) > 3 * K.NECK_YIELD


# -- the parts fit each other ----------------------------------------------------------------------------------------------

def test_the_lancet_fits_the_socket_and_the_cap_fits_over_both():
    assert K.LANCET_R < K.SOCKET_R < K.NOSE_R < K.CAP_BORE < K.CAP_LEAD < K.CAP_R
    bare = K.LANCET_L - K.SOCKET_DEPTH + K.NEEDLE_L                                      # what stands out of the nose
    assert K.CAP_L - 0.002 - K.NOSE_L > bare                                             # the chamber is deep enough
    assert K.CAP_CHAMBER > K.LANCET_R + 0.001
    assert K.TAB_R > K.LANCET_R


def test_the_layout_starts_capped_with_the_lancet_in_the_tray():
    by = {o.name: o for o in T.OBJECTS}
    assert len(by) == len(T.OBJECTS)
    dev, cap, lan, tab, bar = (by[n] for n in (T.DEVICE, T.CAP, T.LANCET, T.TAB, T.BARREL))
    assert K.cap_on(K.cap_fit(dev.pos, dev.quat, cap.pos, cap.quat))
    assert K.barrel_pull(dev.pos, dev.quat, bar.pos) == pytest.approx(0.0, abs=1e-9)
    assert K.joined(lan.pos, lan.quat, tab.pos, tab.quat)
    assert not K.seated(K.seat_fit(dev.pos, dev.quat, lan.pos, lan.quat))
    assert T.tilt_from_level(dev.quat) < 1e-6
    assert dev.pos[2] - T.TOP == pytest.approx(K.BODY_R, abs=0.001)                      # lying on the table
    assert lan.pos[2] - T.TOP == pytest.approx(T.TRAY[2] + T.RIDGE[2] + K.LANCET_R, abs=0.001)
    assert np.allclose(K._rotation(lan.quat)[:, 2], -K._rotation(dev.quat)[:, 2])        # the lancet already points the right way


def test_both_layouts_keep_everything_on_the_table():
    for name in ("device", "barrel", "cap", "lancet", "tab", "dish", "tray"):
        o = next(o for o in T.OBJECTS if o.name == name)
        assert -0.15 < o.pos[0] < 0.85 and -0.6 < o.pos[1] < 0.6
