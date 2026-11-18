"""Offline checks of the reading rule and the scoring (no simulator):  python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bandkit as K  # noqa: E402
import task as T  # noqa: E402

DT = 1.0 / T.CONTROL_HZ
QUARTER = math.pi / 2
FAR = [{"tcp": np.array([0.2, 0.5, T.TOP + 0.3]), "rot": np.eye(3), "gap": 0.09},
       {"tcp": np.array([0.2, -0.6, T.TOP + 0.3]), "rot": np.eye(3), "gap": 0.09}]


def aimed(standoff=0.09, turn=QUARTER, yaw=0.0):
    """The scanner's origin and orientation with its window ``standoff`` from the symbol, square on to it (the forearm
    turned ``turn``), then swung ``yaw`` about the vertical through its window."""
    pts, normals = K.symbol_points(T.ELBOW, turn)
    lying = K._rotation(T.LYING)
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ lying
    window = pts[0] + standoff * np.array([-1.0, 0.0, 0.0])
    pos = window - rot @ np.array(K.WINDOW)
    w, x, y, z = _quat(rot)
    return pos, (x, y, z, w)


def _quat(r):
    """wxyz of a rotation matrix (any rotation)."""
    k = np.array([[r[0, 0] - r[1, 1] - r[2, 2], 0, 0, 0], [r[0, 1] + r[1, 0], r[1, 1] - r[0, 0] - r[2, 2], 0, 0],
                  [r[0, 2] + r[2, 0], r[1, 2] + r[2, 1], r[2, 2] - r[0, 0] - r[1, 1], 0],
                  [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], r[0, 0] + r[1, 1] + r[2, 2]]]) / 3
    vals, vecs = np.linalg.eigh(k + k.T - np.diag(np.diag(k)))
    x, y, z, w = vecs[:, np.argmax(vals)]
    return (w, x, y, z) if w >= 0 else (-w, -x, -y, -z)


def any_pose_reads(turn) -> int:
    """How many of 1,632 scanner poses all round the symbol (every 15 deg round it, every 10 deg of height, 2 to
    14 cm away, each pointed straight at it) could read it with the forearm turned ``turn``."""
    centre = K.symbol_points(T.ELBOW, turn)[0][0]
    found = 0
    for az in np.radians(np.arange(0, 360, 15)):
        for el in np.radians(np.arange(-80, 81, 10)):
            for dist in (0.02, 0.05, 0.09, 0.14):
                d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
                aim = -d
                up = np.array([0.0, 0.0, 1.0])
                y = np.cross(up, aim)
                y /= np.linalg.norm(y)
                rot = np.column_stack([aim, y, np.cross(aim, y)])
                w, qx, qy, qz = _quat(rot)
                found += T.readable(centre + dist * d - rot @ np.array(K.WINDOW), (qx, qy, qz, w), turn)[0]
    return found


def lying_low(pos, quat):
    return np.vstack([K.box_corners(b) for b in K.scanner_boxes(pos, quat)]).min(0)


def play(steps, turn=QUARTER, trigger=-K.TRIGGER_TRAVEL, standoff=0.09, rate=0.0, s=None, grips=FAR, pose=None):
    s = s if s is not None else fresh()
    pos, quat = pose if pose is not None else aimed(standoff, QUARTER)
    for _ in range(steps):
        T.advance(s, turn, rate, trigger, pos, quat, lying_low(pos, quat), np.zeros(6), grips, DT)
    return s


def fresh():
    T._clear()
    return T._STATE


def at_rest(s, turn=0.0, grips=FAR):
    """The forearm rolled gently (45 deg/s) to ``turn``, then a second of it there with the scanner lying where the
    layout puts it."""
    pos, quat = np.array(T.SCANNER_POS, float), T.LYING
    now = s["turn"]
    while abs(now - turn) > 1e-9:
        now += max(-math.radians(45) * DT, min(math.radians(45) * DT, turn - now))
        T.advance(s, now, 0.0, 0.0, pos, quat, lying_low(pos, quat), np.zeros(6), grips, DT)
    for _ in range(20):
        T.advance(s, turn, 0.0, 0.0, pos, quat, lying_low(pos, quat), np.zeros(6), grips, DT)
    return s


def test_task_file_loads():
    from yam_arena import task_file

    task_file.load(str(HERE / "task.py"))
    assert [o.name for o in T.OBJECTS] == ["arm_board", "training_arm", "scanner"]
    assert T.SCORE_OVER == "end" and len(T.INSTRUCTION) > 20


def test_symbol_is_a_real_square_matrix():
    assert len(K.MATRIX) == 22 and all(len(r) == 22 for r in K.MATRIX)
    assert set(K.MATRIX[-1]) == {"1"} and all(r[0] == "1" for r in K.MATRIX)      # the finder's solid edges
    assert abs(K.SYMBOL - 0.005588) < 1e-6


def test_palm_down_the_label_cannot_be_read_from_anywhere():
    pos, quat = aimed(0.09, QUARTER)
    ok, why, _ = T.readable(pos, quat, 0.0)
    assert not ok and "tilt" in why
    assert any_pose_reads(0.0) == 0                                        # face down on the board: no pose reads it
    assert any_pose_reads(math.radians(10)) == 0
    assert any_pose_reads(QUARTER) > 100                                   # a quarter turn over: plenty do


def test_turned_and_aimed_it_reads():
    pos, quat = aimed(0.09, QUARTER)
    ok, why, view = T.readable(pos, quat, QUARTER)
    assert ok and not why
    assert abs(view["range_cm"] - 9.0) < 0.05 and view["tilt_deg"] < 10


def test_range_limits():
    for standoff, good in ((0.005, False), (0.020, True), (0.150, True), (0.165, False)):
        ok, why, _ = T.readable(*aimed(standoff, QUARTER), QUARTER)
        assert ok is good, (standoff, why)
        assert good or "range" in why


def test_field_of_view():
    assert T.readable(*aimed(0.09, QUARTER, math.radians(10)), QUARTER)[0]
    ok, why, _ = T.readable(*aimed(0.09, QUARTER, math.radians(20)), QUARTER)
    assert not ok and why == ["field of view"]


def test_tilt_needs_the_forearm_turned_about_a_third_of_the_way():
    pos, quat = aimed(0.09, QUARTER)
    assert not T.readable(pos, quat, math.radians(25))[0]
    assert "tilt" in T.readable(pos, quat, math.radians(25))[1]
    assert T.readable(pos, quat, math.radians(45))[0]


def test_a_gripper_in_the_way_blocks_the_read():
    pos, quat = aimed(0.09, QUARTER)
    window = K.scanner_optics(pos, quat)[0]
    mid = (window + K.symbol_points(T.ELBOW, QUARTER)[0][0]) / 2
    down = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    ok, why, _ = T.readable(pos, quat, QUARTER, [{"tcp": mid, "rot": down, "gap": 0.0}])
    assert not ok and why == ["line of sight"]


def test_no_trigger_no_read():
    s = play(40, trigger=0.0)
    assert not s["read"] and s["pulls"] == 0


def test_a_read_takes_a_quarter_second_of_steady_view():
    s = play(4)
    assert not s["read"]
    s = play(1, s=s)
    assert s["read"] and s["lit"]
    s = fresh()
    for _ in range(6):                                                     # readable in bursts of 0.2 s: never long enough
        play(4, s=s)
        play(1, s=s, turn=0.0)
    assert not s["read"]


def test_decode_session_times_out_and_needs_a_new_pull():
    s = play(int(T.SESSION_S * T.CONTROL_HZ) + 5, turn=0.0)                 # trigger held 10 s with nothing to read
    assert s["timed_out"] == 1 and not s["session_live"]
    s = play(20, s=s)                                                      # now readable, same pull: no read
    assert not s["read"]
    s = play(2, s=s, trigger=0.0)
    s = play(6, s=s)                                                       # a new pull reads
    assert s["read"] and s["pulls"] == 2


def test_a_symbol_swept_through_the_view_does_not_read():
    s = fresh()
    for k in range(40):
        pos, quat = aimed(0.09, QUARTER)
        pos = pos + [0.0, 0.010 * (k % 4 - 1.5), 0.0]                       # 20 cm/s to and fro, inside the view
        T.advance(s, QUARTER, 0.0, -K.TRIGGER_TRAVEL, pos, quat, lying_low(pos, quat), np.zeros(6), FAR, DT)
    assert not s["read"] and "motion" in s["why"]


def test_scores():
    s = fresh()
    assert T.grade(s) == 0.0
    s = play(19, trigger=0.0)                                              # turned and held, under a second
    assert T.grade(s) == 0.0
    s = play(1, s=s, trigger=0.0)
    assert s["shown"] and T.grade(s) == 0.25
    s = play(6, s=s)                                                       # the read
    assert T.grade(s) == 0.75
    s = at_rest(s)                                                         # everything back, let go
    assert s["down"] and s["laid"] and T.grade(s) == 1.0


def test_a_dropped_forearm_caps_the_score():
    s = play(30)
    assert T.grade(s) == 0.75
    T.advance(s, 0.0, 0.0, 0.0, *aimed(), lying_low(*aimed()), np.zeros(6), FAR, DT)   # a quarter turn back in one step
    assert s["fast"] and T.grade(s) == 0.5
    assert T.grade(at_rest(s)) == 0.5
    s = play(30, trigger=0.0)                                              # shown, then dropped, never read
    play(1, s=s, turn=0.0, trigger=0.0)
    assert T.grade(s) == 0.0


def test_speed_limit_is_200_degrees_a_second():
    s = play(5, turn=0.0, trigger=0.0, rate=math.radians(190))
    assert not s["fast"]
    s = play(1, s=s, turn=0.0, trigger=0.0, rate=math.radians(210))
    assert s["fast"]


def test_not_back_until_let_go_and_laid_down():
    s = play(30)
    rest, quat = np.array(T.SCANNER_POS, float), T.LYING
    holding = [FAR[0], {"tcp": rest + K._rotation(quat) @ np.array(K.GRIP), "rot": np.eye(3), "gap": 0.035}]
    s = at_rest(s, grips=holding)                                          # scanner on the table, hand still on it
    assert not s["down"] and s["laid"] and T.grade(s) == 0.75
    s = at_rest(s, turn=math.radians(15))                                  # forearm left 15 deg over
    assert s["down"] and not s["laid"] and T.grade(s) == 0.75
    high = rest + [0.0, 0.0, 0.05]
    for _ in range(20):                                                    # scanner let go in mid air (as if propped up)
        T.advance(s, math.radians(15), 0.0, 0.0, high, quat, lying_low(high, quat), np.zeros(6), FAR, DT)
    assert not s["down"]
    assert T.grade(at_rest(s)) == 1.0
