"""On the station table, a crime-scene floor: a grey mat fixed on the table, 40 x 56 cm, with four pieces of evidence
lying where they fell: two spent 9 mm shell casings (brass, standing on their bases), a black phone and a closed
folding knife. One casing lies 1.8 cm from a fixed dark chair leg. Four yellow evidence-marker tents stand in a row at
the near edge of the mat. Mark each piece of evidence with one marker beside it, ready for the scene photographs.
Evidence must stay exactly where it lies until it has been photographed.
"""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The floor: a fixed grey mat (concrete) on the table.
MAT_XY, MAT_SIZE = (0.32, 0.0), (0.40, 0.56, 0.004)
FLOOR = TOP + MAT_SIZE[2]
CONCRETE = (0.52, 0.52, 0.50)

# Evidence: (x, y), heading in degrees. A 9 mm Luger case is 9.9 mm across and 19.15 mm long, about 4 g of brass.
CASE_R, CASE_H, CASE_KG = 0.00495, 0.01915, 0.004
PHONE, PHONE_KG = (0.147, 0.071, 0.008), 0.17
KNIFE, KNIFE_KG = (0.110, 0.028, 0.014), 0.12  # a closed folding knife
BRASS, BLACK, KNIFE_GREY = (0.78, 0.60, 0.22), (0.05, 0.05, 0.06), (0.30, 0.30, 0.32)
EVIDENCE = {
    "casing_1": ((0.44, 0.08), 0.0),
    "knife": ((0.30, 0.20), -30.0),
    "phone": ((0.40, -0.15), 20.0),
    "casing_2": ((0.27, -0.05), 0.0),
}
CHAIR_LEG_XY, CHAIR_LEG_R, CHAIR_LEG_H = (0.27, -0.012), 0.015, 0.12  # 1.8 cm from casing_2, on its left

# Markers: yellow plastic A-frame tents, as a fixed-shape block: 5 cm wide (their number faces), 3 cm deep, 6 cm tall.
MARKER, MARKER_KG = (0.05, 0.03, 0.06), 0.02
YELLOW = (0.98, 0.82, 0.05)
MARKERS = {f"marker_{i}": (0.15, y) for i, y in enumerate((0.15, 0.07, -0.07, -0.15), start=1)}
MARKER_START_YAW = 90.0  # number faces toward the arms


def _xyz(xy, z) -> tuple[float, float, float]:
    return float(xy[0]), float(xy[1]), float(z)


OBJECTS = [
    Box("floor_mat", pos=_xyz(MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=CONCRETE),
    Cylinder("chair_leg", pos=_xyz(CHAIR_LEG_XY, FLOOR + CHAIR_LEG_H / 2), radius=CHAIR_LEG_R, height=CHAIR_LEG_H,
             color=(0.12, 0.10, 0.09)),
    Cylinder("casing_1", pos=_xyz(EVIDENCE["casing_1"][0], FLOOR + CASE_H / 2 + 0.0005), radius=CASE_R, height=CASE_H,
             mass=CASE_KG, color=BRASS),
    Cylinder("casing_2", pos=_xyz(EVIDENCE["casing_2"][0], FLOOR + CASE_H / 2 + 0.0005), radius=CASE_R, height=CASE_H,
             mass=CASE_KG, color=BRASS),
    Box("phone", pos=_xyz(EVIDENCE["phone"][0], FLOOR + PHONE[2] / 2 + 0.0005), size=PHONE, mass=PHONE_KG, color=BLACK,
        yaw=math.radians(EVIDENCE["phone"][1])),
    Box("knife", pos=_xyz(EVIDENCE["knife"][0], FLOOR + KNIFE[2] / 2 + 0.0005), size=KNIFE, mass=KNIFE_KG,
        color=KNIFE_GREY, yaw=math.radians(EVIDENCE["knife"][1])),
    *[Box(n, pos=_xyz(xy, FLOOR + MARKER[2] / 2 + 0.0005), size=MARKER, mass=MARKER_KG, color=YELLOW,
          yaw=math.radians(MARKER_START_YAW)) for n, xy in MARKERS.items()],
]
ITEMS = tuple(EVIDENCE)
TAGS = tuple(MARKERS)

# -- scoring ---------------------------------------------------------------------------------------------------------
UNTOUCHED_M, UNTOUCHED_DEG = 0.003, 3.0  # evidence still where it lay: within 3 mm, turned at most 3 degrees
STANDING_DEG = 10.0  # a casing still on its base; a marker standing
FLOOR_TOL = 0.003  # a marker standing on the floor: its bottom within 3 mm of the mat
GAP_MIN, GAP_MAX = 0.005, 0.05  # beside: 5 mm to 5 cm from the evidence, never touching it
COMPLETE_SCORE = 0.9  # score() for the complete state: 1.0 comes only from success() holding for 1 s, which ends
# the trial (task_score takes the last step's score() when success hasn't fired, a hole reviewers reject)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _tilt_deg(s) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, _rot(s.quat_xyzw)[2, 2]))))


def _yaw_deg(s) -> float:
    r = _rot(s.quat_xyzw)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _turn(a: float, b: float, period: float = 360.0) -> float:
    d = (a - b) % period
    return min(d, period - d)


def footprint(name: str, s) -> np.ndarray:
    """The object's outline on the floor, seen from above (a convex polygon, counterclockwise, metres)."""
    c = np.asarray(s.pos[:2], dtype=float)
    if name.startswith("casing"):
        t = np.linspace(0.0, 2 * math.pi, 16, endpoint=False)
        return c + CASE_R * np.c_[np.cos(t), np.sin(t)]
    size = PHONE if name == "phone" else KNIFE if name == "knife" else MARKER
    r = _rot(s.quat_xyzw)[:2, :2]
    hx, hy = size[0] / 2, size[1] / 2
    return c + np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]]) @ r.T


def _seg_dist(p, a, b) -> float:
    ab, ap = b - a, p - a
    t = 0.0 if not ab.any() else min(1.0, max(0.0, float(ap @ ab) / float(ab @ ab)))
    return float(np.linalg.norm(ap - t * ab))


def _overlap(p: np.ndarray, q: np.ndarray) -> bool:
    for poly in (p, q):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            a, b = p @ n, q @ n
            if a.max() < b.min() or b.max() < a.min():
                return False
    return True


def gap(p: np.ndarray, q: np.ndarray) -> float:
    """The shortest distance between two convex outlines (0 if they overlap)."""
    if _overlap(p, q):
        return 0.0
    d = [_seg_dist(v, q[i], q[(i + 1) % len(q)]) for v in p for i in range(len(q))]
    d += [_seg_dist(v, p[i], p[(i + 1) % len(p)]) for v in q for i in range(len(p))]
    return min(d)


def disturbed(objects) -> str | None:
    """Why the evidence no longer lies where it fell, or None."""
    for n, ((x, y), yaw) in EVIDENCE.items():
        s = objects[n]
        if float(np.hypot(s.pos[0] - x, s.pos[1] - y)) > UNTOUCHED_M:
            return f"{n} moved"
        if n.startswith("casing"):
            if _tilt_deg(s) > STANDING_DEG:
                return f"{n} knocked over"
        elif _turn(_yaw_deg(s), yaw, 180.0) > UNTOUCHED_DEG or _tilt_deg(s) > STANDING_DEG:
            return f"{n} turned"
    return None


def _marker_ok(s) -> bool:
    return (_tilt_deg(s) <= STANDING_DEG and abs(float(s.low[2]) - FLOOR) <= FLOOR_TOL and s.resting())


def marked(objects) -> set[str]:
    """The evidence items that have a marker beside them: standing on the floor and still, 5 mm to 5 cm from the
    item and not touching it. Each marker counts for the item nearest it only."""
    items = {n: footprint(n, objects[n]) for n in ITEMS}
    out = set()
    for t in TAGS:
        s = objects[t]
        if not _marker_ok(s):
            continue
        ft = footprint(t, s)
        dists = {n: gap(ft, p) for n, p in items.items()}
        nearest = min(dists, key=dists.get)
        if GAP_MIN <= dists[nearest] <= GAP_MAX:
            out.add(nearest)
    return out


def _grade(objects) -> float:
    if disturbed(objects) is not None:
        return 0.0
    if not all(objects[n].resting() for n in ITEMS):
        return 0.0
    return round(len(marked(objects)) / len(ITEMS), 6)


def success(objects):
    """Every piece of evidence has a marker beside it (5 mm to 5 cm, not touching), every marker that counts standing
    and still, and no evidence moved. Must hold for 1 s."""
    return _grade(objects) == 1.0


def score(objects):
    """0.25 per piece of evidence with a marker beside it (0.9 when complete; 1.0 only once success() has held 1 s).
    0 if any evidence was moved more than 3 mm, turned more than 3 degrees or knocked over."""
    return min(_grade(objects), COMPLETE_SCORE)


# -- the demo --------------------------------------------------------------------------------------------------------
# Each arm takes the two markers on its side, gripped across their 5 cm width (the fingers on the side faces, so no
# finger ever comes between a marker and the evidence), and sets each down with its number face toward the evidence,
# DEMO_GAP from it, on the side away from anything else.
DEMO_GAP = 0.02
# Official traces (2026-11-08): with DEMO_GAP 2 cm and 1 cm, both 1.0 at 23 s, every marker within 1.1 mm of its aim,
# no evidence moved (0.0 mm, 0 degrees) at any logged step.
GRIP_ABOVE = 0.045  # grip point above a marker's bottom: 1.5 cm below its top
TRAVEL_Z = FLOOR + GRIP_ABOVE + 0.08  # a carried marker's bottom 2 cm above every marker top (no carry passes over
# the chair leg)
DROP = 0.001
GRIP_FINGER = FINGER_OPEN * 42 / 94  # aim 42 mm apart, 4 mm inside each face of the 50 mm marker
RELEASE_FINGER = FINGER_OPEN * 66 / 94  # let go gently: 8 mm clear each side
DEMO_PLAN = {  # marker: (evidence, direction from the evidence to the marker, degrees)
    "marker_1": ("casing_1", 180.0),  # on its near side, toward the arms
    "marker_2": ("knife", 180.0),
    "marker_3": ("casing_2", 180.0),  # the chair leg is on its left
    "marker_4": ("phone", 180.0),
}
DEMO_ARM = {"marker_1": "left", "marker_2": "left", "marker_3": "right", "marker_4": "right"}


def _extent(name: str, direction_deg: float) -> float:
    """How far the evidence's outline reaches from its centre in a direction (m)."""
    if name.startswith("casing"):
        return CASE_R
    size = PHONE if name == "phone" else KNIFE
    a = math.radians(direction_deg - EVIDENCE[name][1])
    return abs(size[0] / 2 * math.cos(a)) + abs(size[1] / 2 * math.sin(a))


def demo_targets() -> dict[str, tuple[tuple[float, float], float]]:
    """Each marker's centre and heading (degrees) as the demo sets it down: DEMO_GAP from its evidence's outline, its
    3 cm depth along the line to the evidence."""
    out = {}
    for m, (n, direction) in DEMO_PLAN.items():
        (x, y), a = EVIDENCE[n][0], math.radians(direction)
        r = _extent(n, direction) + DEMO_GAP + MARKER[1] / 2
        out[m] = ((x + r * math.cos(a), y + r * math.sin(a)), direction - 90.0)
    return out


def _move(pick, place, yaw_deg: float) -> list[Waypoint]:
    (px, py), (qx, qy) = pick, place
    pz, qz = FLOOR + GRIP_ABOVE, FLOOR + GRIP_ABOVE + DROP
    # The fingers close along the grip's y; across a marker's 5 cm width the grip is turned 90 degrees from the marker
    # (or -90, the same grip): the one nearer zero, so the wrist stays well inside its +-120 degrees.
    def grip(marker_yaw):
        g = marker_yaw % 180.0 - 90.0  # in [-90, 90): marker at 90 degrees -> grip at 0
        return math.radians(g)

    a, b = grip(MARKER_START_YAW), grip(yaw_deg)
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "to the marker", a),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", a),
        Waypoint(0.8, (px, py, pz), GRIP_FINGER, "close", a),
        Waypoint(1.2, (px, py, TRAVEL_Z), GRIP_FINGER, "lift", a),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), GRIP_FINGER, "carry", b),
        Waypoint(0.5, (qx, qy, TRAVEL_Z), GRIP_FINGER, "hover", b),
        Waypoint(2.0, (qx, qy, qz), GRIP_FINGER, "set down", b),
        Waypoint(0.3, (qx, qy, qz), GRIP_FINGER, "hold", b),
        Waypoint(1.0, (qx, qy, qz), RELEASE_FINGER, "release", b),
        Waypoint(1.2, (qx, qy, TRAVEL_Z), RELEASE_FINGER, "retreat", b),
    ]


def demo():
    """Left arm: markers 1 and 2 to the casing and the knife on the left; right arm: markers 3 and 4 to the casing by
    the chair leg and the phone. Both arms work at once."""
    tgt = demo_targets()
    plans: dict[str, list[Waypoint]] = {"left": [], "right": []}
    for m, arm in DEMO_ARM.items():
        plans[arm] += _move(MARKERS[m], *tgt[m])
    return plans
