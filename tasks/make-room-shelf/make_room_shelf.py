"""On the station table, straight in front of the arms, a low shelf runs across the table: a white board 45 cm long
between two end walls, with a lip along its back. Four product boxes stand on it, spread out with a few centimetres
between each other and the ends. A brown carton stands on the table in front of the shelf. It is 18 cm long: the
free space on the shelf adds up to 23 cm, but no single gap is wider than 5.5 cm, so the boxes have to be moved
together before the carton fits.
"""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the shelf: fixed --------------------------------------------------------------------------------------------------
# A 2 cm board on the table, 13 cm deep, 45 cm between its end walls; the end walls and the back lip stand 6 cm above
# the board (below the 8 cm boxes, so a gripper working beside one clears them).
BOARD_T = 0.02
SHELF = TOP + BOARD_T  # the board's top
FRONT_X, BACK_X = 0.265, 0.395  # the board's front edge and the back lip's inner face
INNER = 0.45  # between the end walls' inner faces, centred on y = 0
HALF = INNER / 2
WALL_T, WALL_H = 0.012, 0.06
WHITE, WALL_GREY = (0.90, 0.89, 0.86), (0.62, 0.62, 0.64)
_DEPTH = BACK_X + WALL_T - FRONT_X
SHELF_PARTS = [
    Box("shelf_board", pos=((FRONT_X + BACK_X + WALL_T) / 2, 0.0, TOP + BOARD_T / 2),
        size=(_DEPTH, INNER + 2 * WALL_T, BOARD_T), color=WHITE),
    Box("shelf_back", pos=(BACK_X + WALL_T / 2, 0.0, SHELF + WALL_H / 2), size=(WALL_T, INNER, WALL_H),
        color=WALL_GREY),
    Box("shelf_end_left", pos=((FRONT_X + BACK_X + WALL_T) / 2, HALF + WALL_T / 2, SHELF + WALL_H / 2),
        size=(_DEPTH, WALL_T, WALL_H), color=WALL_GREY),
    Box("shelf_end_right", pos=((FRONT_X + BACK_X + WALL_T) / 2, -HALF - WALL_T / 2, SHELF + WALL_H / 2),
        size=(_DEPTH, WALL_T, WALL_H), color=WALL_GREY),
]  # fixed: no mass

# -- the boxes and the carton: loose -----------------------------------------------------------------------------------
ROW_X = 0.33  # every box stands with its centre this far out, 3.5 cm behind the board's front edge
BOX_DEPTH, BOX_H, BOX_KG = 0.06, 0.08, 0.12
# Along the shelf (y), from the left end: each box's width and colour, and the gaps (m) from the left end wall to the
# first box, between the boxes, and from the last box to the right end wall. 23 cm free in all, none wider than 5.5 cm.
WIDTH = {"box_1": 0.05, "box_2": 0.06, "box_3": 0.05, "box_4": 0.06}
COLOR = {"box_1": (0.80, 0.15, 0.12), "box_2": (0.15, 0.55, 0.25), "box_3": (0.95, 0.78, 0.15),
         "box_4": (0.20, 0.40, 0.80)}
GAPS = (0.030, 0.055, 0.050, 0.055, 0.040)
BOXES = tuple(WIDTH)


def _box_ys() -> dict[str, float]:
    y, out = HALF, {}
    for name, gap in zip(BOXES, GAPS, strict=False):
        y -= gap
        out[name] = y - WIDTH[name] / 2
        y -= WIDTH[name]
    return out


BOX_Y = _box_ys()
START = {n: (ROW_X, y) for n, y in BOX_Y.items()}
CARTON = (0.07, 0.18, 0.10)  # x (depth), y (length), z: it stands with its 18 cm length across the table
CARTON_KG, KRAFT = 0.25, (0.62, 0.45, 0.28)
CARTON_START = (0.18, 0.0)
SIZE = {**{n: (BOX_DEPTH, WIDTH[n], BOX_H) for n in BOXES}, "carton": CARTON}
NAMES = (*BOXES, "carton")

OBJECTS = [
    *SHELF_PARTS,
    *[Box(n, pos=(*START[n], SHELF + BOX_H / 2 + 0.001), size=SIZE[n], mass=BOX_KG, color=COLOR[n]) for n in BOXES],
    Box("carton", pos=(*CARTON_START, TOP + CARTON[2] / 2 + 0.001), size=CARTON, mass=CARTON_KG, color=KRAFT),
]  # each starts 1 mm above what it stands on, and settles

# -- scoring -----------------------------------------------------------------------------------------------------------
FLAT_DEG = 10.0  # resting on a face: one of its axes within 10 degrees of vertical
SHELF_TOL = 0.003  # on the board: its lowest corner within 3 mm of the board's top
EDGE_TOL = 0.002  # every corner inside the board's front edge, back lip and end walls, to 2 mm
COMPLETE_SCORE = 0.9  # score() for the complete state: 1.0 comes only from success() holding for 1 s, which ends
# the trial (task_score takes the last step's score() when success hasn't fired, a hole reviewers reject)

# Where a box is comes from its centre and rotation, not from ``low``/``high``: Arena builds those from the box's
# corners, turned with it, so they grow when a box turns about the vertical.


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(s, size) -> np.ndarray:
    half = np.array(size) / 2
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return np.asarray(s.pos, dtype=float) + (signs * half) @ _rot(s.quat_xyzw).T


def _upright(s) -> bool:
    """Its own height axis within FLAT_DEG of vertical, either way up."""
    return abs(_rot(s.quat_xyzw)[2, 2]) >= math.cos(math.radians(FLAT_DEG))


def _flat(s) -> bool:
    """Resting on any of its faces: one of its axes within FLAT_DEG of vertical."""
    return float(np.abs(_rot(s.quat_xyzw)[2]).max()) >= math.cos(math.radians(FLAT_DEG))


def on_shelf(s, size) -> bool:
    """Standing on the board itself (not on another box), wholly inside its front edge, back lip and end walls."""
    c = _corners(s, size)
    return bool(abs(c[:, 2].min() - SHELF) <= SHELF_TOL
                and c[:, 0].min() >= FRONT_X - EDGE_TOL and c[:, 0].max() <= BACK_X + EDGE_TOL
                and c[:, 1].min() >= -HALF - EDGE_TOL and c[:, 1].max() <= HALF + EDGE_TOL)


def widest_gap(objects) -> float:
    """The widest stretch of the shelf (between the end walls, along y) that no box reaches into (m)."""
    spans = sorted((float(c[:, 1].min()), float(c[:, 1].max()))
                   for c in (_corners(objects[n], SIZE[n]) for n in BOXES))
    edge, best = -HALF, 0.0
    for lo, hi in spans:
        best = max(best, lo - edge)
        edge = max(edge, hi)
    return max(best, HALF - edge)


def _grade(objects) -> float:
    boxes_ok = all(_upright(objects[n]) and on_shelf(objects[n], SIZE[n]) for n in BOXES)
    if not boxes_ok:
        return 0.0  # at the end, a box off the shelf, on another, or lying over
    carton = objects["carton"]
    if _flat(carton) and on_shelf(carton, CARTON):
        return 1.0
    carton_kept = float(_corners(carton, CARTON)[:, 2].min()) >= TOP - 0.01  # not dropped off the table
    if carton_kept and widest_gap(objects) >= CARTON[1]:
        return 0.25  # room made, carton not on the shelf yet
    return 0.0


def success(objects):
    """The carton resting flat on the shelf board, wholly on it, and all four boxes standing on the board, every
    object still. Must hold for 1 s."""
    return _grade(objects) == 1.0 and all(objects[n].resting() for n in NAMES)


def score(objects):
    """0.9 for the carton on the shelf with all four boxes still standing on it (1.0 only once success() has held 1 s);
    0.25 if the boxes are still standing on the shelf with an 18 cm stretch cleared but the carton not on it; 0 if any
    box is off the shelf, on another box or lying over at the end (only the end state counts)."""
    return min(_grade(objects), COMPLETE_SCORE)


# -- the demo ----------------------------------------------------------------------------------------------------------
# Each arm packs the two boxes on its own half against its end wall, outer box first, aiming 4 mm between neighbours:
# that opens 21.4 cm in the middle. Then the left arm slides the carton in from the front, 2 cm above the board, and
# sets it down there. Every grip closes along x (across the table), across the boxes' 6 cm depth and the carton's
# 7 cm, so no finger goes into a gap between two boxes.
PACK_GAP = 0.004
TCP_PAST = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP
BOX_GRIP_ABOVE = 0.045  # the TCP 4.5 cm above a box's bottom: the pad tips 3.2 cm up, 3.5 cm below its top
CARTON_GRIP_ABOVE = 0.05  # the TCP 5 cm above the carton's bottom
# Carried 2.5 cm above the board, a box (2 cm, the carton) clears it; no box passes over another, and the carton goes
# in between the packed boxes from the front. With the grip turned across the table, the middle of the shelf is out of
# reach much higher (offline IK at 35 cm out: 6 mm short at 11 cm above the board, so the shelf stands at 33 cm).
TRAVEL_Z = SHELF + 0.07
# Empty-handed the fingers come and go 12 cm above the board: their tips 2.7 cm above every box top and 0.7 cm above
# the carton's. On the official stack an arm that moved off at TRAVEL_Z swept a box off the end of the shelf (traced
# 2026-11-08).
CLEAR_Z = SHELF + 0.12
DROP = 0.001  # let go of a box 1 mm above the board
# The carton is let go above the 3 mm that counts as on the board, so success can't fire while it is held: let go
# 1 mm up, success fired 1.6 s before the fingers opened. Carrying it, the arm sags about 6 mm below its target, so it
# is aimed 16 mm up and opens about 1 cm above the board (traced 2026-11-08: aimed 6 mm up, it reached the board).
CARTON_DROP = 0.016
# The grip turned a quarter turn, closing along x: the left arm turns clockwise, the right anticlockwise (the other
# way round, the left arm misses the carton's pick by 3.4 mm).
HEADING = {"left": math.radians(-90.0), "right": math.radians(90.0)}
# Aim the fingers 4 mm inside each face, so each presses well under its limit and the pair centres the item (as in
# clear-beam); open slowly to 76 mm (box) or 86 mm (carton) to let go.
GRIP = {"box": BOX_DEPTH - 0.008, "carton": CARTON[0] - 0.008}
RELEASE = {"box": 0.076, "carton": 0.086}
DEMO_ARM = {"box_1": "left", "box_2": "left", "box_3": "right", "box_4": "right", "carton": "left"}


def _finger(gap: float) -> float:
    return FINGER_OPEN * gap / 0.094


def demo_targets() -> dict[str, float]:
    """Where the demo sets each box and the carton down (y): boxes packed against their end walls, the carton centred
    in the room that opens."""
    out, y = {}, HALF
    for n in ("box_1", "box_2"):
        y -= PACK_GAP
        out[n] = y - WIDTH[n] / 2
        y -= WIDTH[n]
    left_edge, y = y, -HALF
    for n in ("box_4", "box_3"):
        y += PACK_GAP
        out[n] = y + WIDTH[n] / 2
        y += WIDTH[n]
    out["carton"] = (left_edge + y) / 2
    return out


DEMO_AIM_SHIFT_MM = 0.0  # the carton set down this far toward the left (+y), for margin runs


def _move(name: str, pick, place, wait_s: float = 0.0) -> list[Waypoint]:
    """A top-down pick and place: grip, lift, carry, set down slowly, open slowly, lift away."""
    (px, py, pz), (qx, qy, qz) = pick, place
    kind = "carton" if name == "carton" else "box"
    a, open_, grip, rel = HEADING[DEMO_ARM[name]], FINGER_OPEN, _finger(GRIP[kind]), _finger(RELEASE[kind])
    out = [Waypoint(wait_s, (px, py, CLEAR_Z), open_, "wait", a)] if wait_s else []
    return out + [
        Waypoint(1.5, (px, py, CLEAR_Z), open_, "over it", a),
        Waypoint(1.5, (px, py, pz), open_, "descend", a),
        Waypoint(0.5, (px, py, pz), open_, "steady", a),
        Waypoint(0.8, (px, py, pz), grip, "close", a),
        Waypoint(1.2, (px, py, TRAVEL_Z), grip, "lift", a),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), grip, "carry", a),
        Waypoint(0.5, (qx, qy, TRAVEL_Z), grip, "hover", a),
        Waypoint(2.0, (qx, qy, qz), grip, "set down", a),
        Waypoint(0.3, (qx, qy, qz), grip, "hold", a),
        Waypoint(1.5, (qx, qy, qz), rel, "release", a),
        Waypoint(0.3, (qx, qy, qz), rel, "settle", a),
        Waypoint(1.5, (qx, qy, CLEAR_Z), rel, "lift away", a),
    ]


def _box_move(name: str, wait_s: float = 0.0) -> list[Waypoint]:
    z = SHELF + BOX_GRIP_ABOVE
    return _move(name, (ROW_X, BOX_Y[name], z), (ROW_X, demo_targets()[name], z + DROP), wait_s)


# Each arm ends back over its own side of the table, clear of the shelf's middle and the carton.
PARK = {"left": (0.20, 0.24), "right": (0.20, -0.24)}


def _park(arm: str) -> list[Waypoint]:
    x, y = PARK[arm]
    return [Waypoint(1.5, (x, y, CLEAR_Z), FINGER_OPEN, "back to its own side", 0.0)]


def demo():
    """Left arm: box_1 then box_2 against the left end wall, then it parks. Right arm: waits one move (so the two
    grippers are never both near the middle), box_4 then box_3 against the right end wall, then it parks. Then the
    left arm takes the carton from the table, slides it in between the packed boxes and sets it down."""
    left_pack = _box_move("box_1") + _box_move("box_2") + _park("left")
    first = sum(w.duration for w in _box_move("box_1"))
    right = _box_move("box_4", wait_s=first) + _box_move("box_3") + _park("right")
    wait = sum(w.duration for w in right) - sum(w.duration for w in left_pack)
    cx, cy = CARTON_START
    ty = demo_targets()["carton"] + DEMO_AIM_SHIFT_MM / 1000
    hold = [Waypoint(max(wait, 0.0) + 0.5, (*PARK["left"], CLEAR_Z), FINGER_OPEN, "wait", 0.0)]
    carton = _move("carton", (cx, cy, TOP + CARTON_GRIP_ABOVE), (ROW_X, ty, SHELF + CARTON_DROP + CARTON_GRIP_ABOVE))
    return {"left": left_pack + hold + carton + _park("left"), "right": right}
