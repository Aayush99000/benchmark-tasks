"""On the station table, straight in front of the arms, a small red disc marks the load point: the spot a load will be
lowered onto. Six cribbing timbers lie on the table, three on each side, all along the table's depth. They are a
half-scale 4x4 x 20 in: 4.45 cm square, 25.4 cm long, 250 g.
"""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

LOAD_POINT = (0.23, 0.0)
RED, PINE = (0.85, 0.12, 0.10), (0.80, 0.64, 0.40)
MARK = Cylinder("load_point", pos=(*LOAD_POINT, TOP + 0.00025), radius=0.012, height=0.0005, color=RED)  # fixed

# Half scale: a 4x4 is 3.5 in (89 mm) square, so 44.5 mm; 20 in long, so 254 mm. The US&R Shoring Operations Guide's
# "Overlap corners by at least 4 inches" becomes 50.8 mm.
W, L = 0.0445, 0.254
TIMBER_KG = 0.25
OVERLAP = 0.0508
LAYERS = 3
# Where each timber lies at the start: (x, y) centre, all along x (the table's depth), 9 cm apart across.
START = {
    "timber_1": (0.25, 0.20), "timber_2": (0.25, 0.29), "timber_3": (0.25, 0.38),
    "timber_4": (0.25, -0.20), "timber_5": (0.25, -0.29), "timber_6": (0.25, -0.38),
}
TIMBERS = tuple(START)
OBJECTS = [
    MARK,
    *[Box(n, pos=(*xy, TOP + W / 2 + 0.001), size=(L, W, W), mass=TIMBER_KG, color=PINE) for n, xy in START.items()],
]  # each timber starts 1 mm above the table, and settles

# -- scoring ----------------------------------------------------------------------------------------------------------
CRIB_RADIUS = 0.16  # a timber is part of the crib when its centre is within 16 cm of the load point (across)
LAYER_TOL = 0.004  # its lowest point within 4 mm of a whole number of timber widths above the table
FLAT_DEG = 5.0  # its long axis within 5 degrees of level, and the crib's layers within 5 degrees of square
CENTRE_TOL = 0.015  # the crib's centre within 1.5 cm of the load point, for full credit
CENTRE_MAX = 0.03  # and within 3 cm for any credit
COMPLETE_SCORE = 0.9  # score() for the complete state: 1.0 comes only from success() holding for 1 s, which ends
# the trial (task_score takes the last step's score() when success hasn't fired, a hole reviewers reject)


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _axis(s) -> np.ndarray:
    """The timber's long axis (its own x) in the world."""
    return _rot(s.quat_xyzw)[:, 0]


def _bottom(s) -> float:
    half = np.array([L, W, W]) / 2
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    corners = np.asarray(s.pos, dtype=float) + (signs * half) @ _rot(s.quat_xyzw).T
    return float(corners[:, 2].min())


def _level(s) -> bool:
    return abs(float(_axis(s)[2])) <= math.sin(math.radians(FLAT_DEG))


def _heading(s) -> float:
    """The long axis's direction across the table, in degrees, 0 to 180."""
    a = _axis(s)
    return math.degrees(math.atan2(a[1], a[0])) % 180.0


def _angle_between(h1: float, h2: float) -> float:
    d = abs(h1 - h2) % 180.0
    return min(d, 180.0 - d)


def layer_of(s) -> int | None:
    """Which layer a level timber lies in (0 on the table), from the height of its lowest point; None if between."""
    if not _level(s):
        return None
    k = (_bottom(s) - TOP) / W
    n = round(k)
    return n if n >= 0 and abs(k - n) * W <= LAYER_TOL else None


def _xy(s) -> np.ndarray:
    return np.asarray(s.pos[:2], dtype=float)


def _a2(s) -> np.ndarray:
    a = _axis(s)[:2]
    return a / np.linalg.norm(a)


def _rests_on(u, lower) -> bool:
    """The upper timber crosses the lower one and bears on it: each one's centre line passes over the other."""
    return (abs(float((_xy(lower) - _xy(u)) @ _a2(u))) <= L / 2
            and abs(float((_xy(u) - _xy(lower)) @ _a2(lower))) <= L / 2)


def overhang(u, lower) -> float:
    """At the corner where two crossing timbers meet, the shorter of the two ends past the other's outer face (m)."""
    over_u = L / 2 - (abs(float((_xy(lower) - _xy(u)) @ _a2(u))) + W / 2)
    over_l = L / 2 - (abs(float((_xy(u) - _xy(lower)) @ _a2(lower))) + W / 2)
    return min(over_u, over_l)


def _crib(objects) -> tuple[list[list[str]], float]:
    """The crib's layers that are built right, from the bottom (each two level timbers, parallel, square to the layer
    below and bearing on both its timbers), and its centre's distance from the load point."""
    near = [n for n in TIMBERS if math.dist(_xy(objects[n]), LOAD_POINT) <= CRIB_RADIUS]
    by_layer: dict[int, list[str]] = {}
    for n in near:
        k = layer_of(objects[n])
        by_layer.setdefault(-1 if k is None else k, []).append(n)
    layers: list[list[str]] = []
    for k in range(LAYERS):
        pair = by_layer.get(k, [])
        if len(pair) != 2:
            break
        a, b = (objects[n] for n in pair)
        if _angle_between(_heading(a), _heading(b)) > FLAT_DEG:
            break
        if layers:
            below = [objects[n] for n in layers[-1]]
            if _angle_between(_heading(a), _heading(below[0])) < 90.0 - FLAT_DEG:
                break
            if not all(_rests_on(u, lo) for u in (a, b) for lo in below):
                break
        layers.append(pair)
    if not layers:
        return [], math.inf
    centre = np.mean([_xy(objects[n]) for n in layers[0]], axis=0)
    return layers, float(np.linalg.norm(centre - np.array(LOAD_POINT)))


def _grade(objects) -> float:
    layers, off = _crib(objects)
    if not layers or off > CENTRE_MAX:
        return 0.0
    if len(layers) == LAYERS and off <= CENTRE_TOL:
        corners = [overhang(objects[u], objects[lo])
                   for k in range(1, LAYERS) for u in layers[k] for lo in layers[k - 1]]
        if min(corners) >= OVERLAP:
            return 1.0
    return round(0.2 * len(layers), 6)


def success(objects):
    """A 2 x 2 box crib three layers high on the load point (centred within 1.5 cm), each layer square to the one
    below, every corner overlapping by at least 50.8 mm (4 in at full scale), every timber still. Must hold 1 s."""
    return _grade(objects) == 1.0 and all(objects[n].resting() for n in TIMBERS)


def score(objects):
    """0.2 per layer built right from the bottom, on the load point (within 3 cm); 0.9 for the complete crib with
    every corner overlapping by 50.8 mm (1.0 only once success() has held 1 s)."""
    return min(_grade(objects), COMPLETE_SCORE)


# -- the demo -----------------------------------------------------------------------------------------------------------
# Pairs 8 cm apart, centre to centre: each end then overhangs the timbers it crosses by 64.75 mm, 14 mm past the rule.
# Layers 1 and 3 run along x, layer 2 along y. The arms take turns, each taking timbers from its own side.
SPACING = 0.08
TCP_PAST = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP
GRIP_ABOVE = W / 2 + 0.006  # the TCP 6 mm above a timber's middle: the pad tips 15 mm above its bottom
DROP = 0.001  # set down 1 mm above where it rests
APPROACH, GRIP = 0.070, W - 0.008  # fingers come down 70 mm apart and close to 4 mm inside each face
PARK = {"left": (0.20, 0.30), "right": (0.20, -0.30)}


def crib_targets() -> list[tuple[str, str, tuple[float, float], float, int]]:
    """(timber, arm, (x, y) centre, heading in degrees, layer), in the order the demo builds the crib."""
    (cx, cy), h = LOAD_POINT, SPACING / 2
    return [
        ("timber_1", "left", (cx, cy + h), 0.0, 0),
        ("timber_4", "right", (cx, cy - h), 0.0, 0),
        ("timber_2", "left", (cx + h, cy), 90.0, 1),
        ("timber_5", "right", (cx - h, cy), 90.0, 1),
        ("timber_3", "left", (cx, cy + h), 0.0, 2),
        ("timber_6", "right", (cx, cy - h), 0.0, 2),
    ]


DEMO_AIM_SHIFT_MM = 0.0  # every set-down aimed this far towards +x and +y, for margin runs


def _clear_z(layer: int) -> float:
    """Carry height for a timber going into ``layer``: its bottom 2 cm above that layer's top, so it clears its partner
    already set there. (Carried only 2 cm above the layers below, the right arm swept the left arm's timber off layer 2
    on the official stack, traced 2026-11-08.)"""
    return TOP + (layer + 1) * W + 0.02 + GRIP_ABOVE


def _grip_heading(timber_heading: float, arm: str) -> float:
    """The fingers close across the timber. Along x the grip is turned 0; along y a quarter turn, the left arm
    clockwise and the right anticlockwise (each the way its wrist reaches the crib)."""
    if timber_heading == 0.0:
        return 0.0
    return math.radians(-90.0 if arm == "left" else 90.0)


def _finger(gap: float) -> float:
    return FINGER_OPEN * gap / 0.094


def _place(timber: str, arm: str, xy, heading: float, layer: int) -> list[Waypoint]:
    (px, py), (qx, qy) = START[timber], xy
    qx += DEMO_AIM_SHIFT_MM / 1000
    qy += DEMO_AIM_SHIFT_MM / 1000
    z_hi = _clear_z(layer)
    pick_z, place_z = TOP + GRIP_ABOVE, TOP + layer * W + DROP + GRIP_ABOVE
    a0, a1 = 0.0, _grip_heading(heading, arm)
    open_, grip = _finger(APPROACH), _finger(GRIP)
    return [
        Waypoint(1.5, (px, py, z_hi), open_, "over the timber", a0),
        Waypoint(1.5, (px, py, pick_z), open_, "descend", a0),
        Waypoint(0.4, (px, py, pick_z), open_, "steady", a0),
        Waypoint(0.8, (px, py, pick_z), grip, "close", a0),
        Waypoint(1.2, (px, py, z_hi), grip, "lift", a0),
        Waypoint(2.0, (qx, qy, z_hi), grip, "carry", a1),
        Waypoint(0.5, (qx, qy, z_hi), grip, "hover", a1),
        Waypoint(1.8, (qx, qy, place_z), grip, "set down", a1),
        Waypoint(0.3, (qx, qy, place_z), grip, "hold", a1),
        Waypoint(1.2, (qx, qy, place_z), open_, "release", a1),
        Waypoint(1.0, (qx, qy, z_hi), open_, "up", a1),
        Waypoint(1.5, (*PARK[arm], _clear_z(LAYERS - 1)), open_, "back to its own side", 0.0),
    ]


def demo():
    """The arms take turns, one timber each turn, in crib_targets()'s order; the arm not placing waits over its own
    side."""
    plan = {"left": [], "right": []}
    clock = {"left": 0.0, "right": 0.0}
    t = 0.0
    for timber, arm, xy, heading, layer in crib_targets():
        if clock[arm] < t:
            plan[arm].append(Waypoint(t - clock[arm], (*PARK[arm], _clear_z(LAYERS - 1)), FINGER_OPEN, "wait", 0.0))
        move = _place(timber, arm, xy, heading, layer)
        plan[arm] += move
        t += sum(w.duration for w in move)
        clock[arm] = t
    return plan
