"""On the station table, straight in front of the arms, a bench drill press: a grey base plate and table with a 3 cm
hole under the spindle, a column behind, and a head over the table with the chuck and a drill bit pointing down. The
press doesn't run. To the robot's left on the table lie a long oak board and a shorter piece of scrap pine, both along
the table's depth.
"""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the drill press: fixed ----------------------------------------------------------------------------------------
BIT_XY = (0.26, 0.0)  # the spindle's axis
PRESS_T = 0.03  # the press table, 3 cm thick, standing on the station table
TT = TOP + PRESS_T  # the press table's top
PRESS_TABLE = (0.26, 0.26)  # x, y extent, centred on the spindle
HOLE = 0.03  # the square hole through the table under the spindle
BIT_TIP = TT + 0.10  # the bit's tip, 10 cm above the table (the quill all the way up)
BIT_R, CHUCK_R = 0.004, 0.022
# The head and chuck stand well up, and the head stops 3 cm in front of the spindle: the gripper's body rises straight
# up from the fingers, and with the head lower and reaching further forward it blocked the arm 6 cm short of the
# press (traced 2026-11-08).
CHUCK_LOW, HEAD_LOW, HEAD_TOP = TT + 0.20, TT + 0.30, TT + 0.39
COLUMN_XY, COLUMN_R = (BIT_XY[0] + 0.20, 0.0), 0.035
STEEL, DARK, METAL = (0.55, 0.57, 0.60), (0.20, 0.21, 0.23), (0.78, 0.78, 0.80)


def _press() -> list:
    (bx, by), (tx, ty), h = BIT_XY, PRESS_TABLE, HOLE / 2
    z = TOP + PRESS_T / 2
    side_y = (ty / 2 - h)
    side_x = (tx / 2 - h)
    head_x0, head_x1 = bx - 0.03, COLUMN_XY[0] + 0.05
    return [
        # the table, as four slabs round the hole
        Box("press_table_near", pos=(bx - h - side_x / 2, by, z), size=(side_x, ty, PRESS_T), color=STEEL),
        Box("press_table_far", pos=(bx + h + side_x / 2, by, z), size=(side_x, ty, PRESS_T), color=STEEL),
        Box("press_table_left", pos=(bx, by + h + side_y / 2, z), size=(HOLE, side_y, PRESS_T), color=STEEL),
        Box("press_table_right", pos=(bx, by - h - side_y / 2, z), size=(HOLE, side_y, PRESS_T), color=STEEL),
        Cylinder("press_column", pos=(*COLUMN_XY, (TOP + HEAD_TOP) / 2), radius=COLUMN_R, height=HEAD_TOP - TOP,
                 color=DARK),
        Box("press_head", pos=((head_x0 + head_x1) / 2, by, (HEAD_LOW + HEAD_TOP) / 2),
            size=(head_x1 - head_x0, 0.12, HEAD_TOP - HEAD_LOW), color=DARK),
        Cylinder("press_chuck", pos=(bx, by, (CHUCK_LOW + HEAD_LOW) / 2), radius=CHUCK_R,
                 height=HEAD_LOW - CHUCK_LOW, color=METAL),
        Cylinder("drill_bit", pos=(bx, by, (BIT_TIP + CHUCK_LOW) / 2), radius=BIT_R, height=CHUCK_LOW - BIT_TIP,
                 color=METAL),
    ]  # fixed: no mass


# -- the board and the backer: loose ------------------------------------------------------------------------------
BOARD, BOARD_KG = (0.24, 0.06, 0.02), 0.10  # oak, 24 x 6 x 2 cm, lying along its length (x)
BACKER, BACKER_KG = (0.24, 0.06, 0.02), 0.07  # scrap pine, 24 x 6 x 2 cm
OAK, PINE = (0.62, 0.42, 0.22), (0.86, 0.74, 0.52)
SIZE = {"board": BOARD, "backer": BACKER}
START = {"backer": (0.25, 0.21), "board": (0.25, 0.30)}  # both along x, on the station table to the robot's left
NAMES = ("board", "backer")

OBJECTS = [
    *_press(),
    Box("backer", pos=(*START["backer"], TOP + BACKER[2] / 2 + 0.001), size=BACKER, mass=BACKER_KG, color=PINE),
    Box("board", pos=(*START["board"], TOP + BOARD[2] / 2 + 0.001), size=BOARD, mass=BOARD_KG, color=OAK),
]

# -- scoring ----------------------------------------------------------------------------------------------------------
LEVEL_DEG = 3.0  # flat: tilted at most 3 degrees
REST_TOL = 0.003  # resting on a surface: its lowest point within 3 mm of it
CENTRE_TOL = 0.003  # the board's middle within 3 mm of the spindle's axis (across), for full credit
NEAR_TOL = 0.010  # and within 1 cm for partial credit
BACKED = 0.02  # the spindle's axis at least 2 cm inside every edge of the backer, so the bit exits into it
COMPLETE_SCORE = 0.9  # score() for the complete state: 1.0 comes only from success() holding for 1 s, which ends
# the trial (task_score takes the last step's score() when success hasn't fired, a hole reviewers reject)


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _bottom(s, size) -> float:
    half = np.array(size) / 2
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    corners = np.asarray(s.pos, dtype=float) + (signs * half) @ _rot(s.quat_xyzw).T
    return float(corners[:, 2].min())


def _level(s) -> bool:
    return float(_rot(s.quat_xyzw)[2, 2]) >= math.cos(math.radians(LEVEL_DEG))  # lying flat, its 2 cm side up


def _off_axis(s) -> float:
    return math.dist(np.asarray(s.pos[:2], dtype=float), BIT_XY)


def backing_margin(backer) -> float:
    """How far inside the backer's nearest edge the spindle's axis is (m); negative if outside it."""
    r = _rot(backer.quat_xyzw)
    d = np.array([*BIT_XY, 0.0]) - np.asarray(backer.pos, dtype=float)
    local = r.T @ d
    return float(min(BACKER[0] / 2 - abs(local[0]), BACKER[1] / 2 - abs(local[1])))


def _grade(objects) -> float:
    board, backer = objects["board"], objects["backer"]
    if not _level(board) or _off_axis(board) > NEAR_TOL:
        return 0.0
    board_bottom = _bottom(board, BOARD)
    backer_ok = _level(backer) and abs(_bottom(backer, BACKER) - TT) <= REST_TOL
    on_backer = backer_ok and abs(board_bottom - (TT + BACKER[2])) <= REST_TOL and backing_margin(backer) > 0
    if on_backer:
        full = _off_axis(board) <= CENTRE_TOL and backing_margin(backer) >= BACKED
        return 1.0 if full else 0.5
    if abs(board_bottom - TT) <= REST_TOL:
        return 0.25  # under the bit, but straight on the table: the bit would tear out the underside into the hole
    return 0.0


def success(objects):
    """The board flat on the backer, its middle within 3 mm of the spindle's axis, the backer flat on the press table
    with the axis at least 2 cm inside its edges, both still. Must hold for 1 s."""
    return _grade(objects) == 1.0 and all(objects[n].resting() for n in NAMES)


def score(objects):
    """0.9 for the board centred on the backer under the bit (1.0 only once success() has held 1 s); 0.5 if it is on
    the backer under the bit but up to 1 cm off, or the bit is within 2 cm of the backer's edge; 0.25 if it is under
    the bit straight on the table, with no backer; 0 otherwise."""
    return min(_grade(objects), COMPLETE_SCORE)


# -- the demo -----------------------------------------------------------------------------------------------------
# The bit is in the way of a grip from above, so each piece is held near its end nearer the arms (fingers closing
# across its 6 cm width), carried in from the left low over the press table, set down and let go: first the backer,
# centred under the bit, then the board on it. The gripper stays OFFSET in front of the bit the whole time.
OFFSET = {"backer": 0.10, "board": 0.10}  # grip point this far from the piece's middle, towards the arms (-x)
# (8 cm, the arm's upper links met the head's front corner as the gripper neared the middle, traced 2026-11-08)
TCP_PAST = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP
GRIP_ABOVE = 0.012  # the TCP 1.2 cm above a piece's bottom: the pad tips just above its underside
SLIDE_GAP = 0.025  # carried in with its bottom 2.5 cm above what it will rest on: held off-centre, a piece droops a
# few degrees, and 8 mm up the scrap's far end caught the press table and it fell (traced 2026-11-08)
LIFT_Z = TT + 0.10  # over the station table, a carried piece's bottom 10 cm up, clear of the press table
DROP = 0.002
HEADING = 0.0  # fingers closing along y, across the pieces' width
APPROACH, GRIP = 0.080, 0.044  # finger gap coming down, and aimed 8 mm inside each side, for a firm grip off-centre
DEMO_AIM_SHIFT_MM = 0.0  # both pieces set down this far from the axis (+x and +y), for margin runs
# Where each piece came to rest, aimed straight at the axis (mm, x and y; traced 2026-11-08 on the official stack):
# the demo aims that much the other way.
DEMO_LANDING = {"backer": (-0.4, 3.2), "board": (-1.6, 4.2)}


def _finger(gap: float) -> float:
    return FINGER_OPEN * gap / 0.094


def _rest_z(name: str) -> float:
    return TT if name == "backer" else TT + BACKER[2]


def _place(name: str) -> list[Waypoint]:
    (sx, sy), off = START[name], OFFSET[name]
    shift = DEMO_AIM_SHIFT_MM / 1000
    lx, ly = DEMO_LANDING[name]
    tx, ty = BIT_XY[0] + shift - lx / 1000, BIT_XY[1] + shift - ly / 1000
    pick = (sx - off, sy)
    rest = _rest_z(name)
    side = (tx - off, ty + 0.15)  # beside the press table, the piece clear of the bit
    above_z = LIFT_Z + GRIP_ABOVE
    slide_z = rest + SLIDE_GAP + GRIP_ABOVE
    a, open_, grip = HEADING, _finger(APPROACH), _finger(GRIP)
    return [
        Waypoint(1.5, (*pick, above_z), open_, "over the piece", a),
        Waypoint(1.5, (*pick, TOP + GRIP_ABOVE), open_, "descend", a),
        Waypoint(0.4, (*pick, TOP + GRIP_ABOVE), open_, "steady", a),
        Waypoint(0.8, (*pick, TOP + GRIP_ABOVE), grip, "close", a),
        Waypoint(1.2, (*pick, above_z), grip, "lift", a),
        Waypoint(1.5, (*side, above_z), grip, "beside the press", a),
        Waypoint(1.2, (*side, slide_z), grip, "down to just over the table", a),
        Waypoint(2.0, (tx - off, ty, slide_z), grip, "in under the bit", a),
        Waypoint(0.4, (tx - off, ty, slide_z), grip, "steady", a),
        Waypoint(1.0, (tx - off, ty, rest + DROP + GRIP_ABOVE), grip, "set down", a),
        Waypoint(0.3, (tx - off, ty, rest + DROP + GRIP_ABOVE), grip, "hold", a),
        Waypoint(1.0, (tx - off, ty, rest + DROP + GRIP_ABOVE), open_, "release", a),
        Waypoint(1.0, (tx - off - 0.06, ty, rest + 0.05), open_, "back out", a),
        Waypoint(1.2, (*side, above_z), open_, "clear", a),
    ]


def demo():
    """The left arm sets the backer under the bit, then the board on it; the right arm doesn't move."""
    return {"left": _place("backer") + _place("board")}
