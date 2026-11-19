"""The props' geometry for draw_a_splinter_out_with_tweezers.py: numbers, frames and the rotations the boxes use.

World frame: x forward from the arm bases, y left, z up, metres; the table top is TOP. Every size here is the real
item's (see README, "Props"), except where a comment says the simulator needs otherwise.
"""

from __future__ import annotations

import math

import numpy as np
from benchkit import matrix_quat, quat_matrix
from yam_arena.task_file import TABLE_TOP

TOP = TABLE_TOP["station"]


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def q(r) -> tuple:
    """A rotation matrix as a box turn (w, x, y, z)."""
    return matrix_quat(r)


def xyzw(r) -> tuple:
    w, x, y, z = matrix_quat(r)
    return (x, y, z, w)


# -- the splinter: a 15 mm piece of a 1.8 mm round birch toothpick, pointed end in -------------------------------------
ENTRY_ANGLE = math.radians(25.0)  # to the skin
SPLINTER_LONG = 0.015
SPLINTER_DIA = 0.0018
BURIED = 0.011  # under the skin at reset, along its axis
PROUD = SPLINTER_LONG - BURIED  # 4 mm stands out
PROXY = 0.0024  # the splinter collides as a 2.4 mm square stick: under 2 mm, PhysX contacts tunnel and jitter
IN = np.array([math.cos(ENTRY_ANGLE), 0.0, -math.sin(ENTRY_ANGLE)])  # the way it went in: away from the robot, down
OUT = -IN
TRACK = rot_y(ENTRY_ANGLE)  # the track's frame: its X runs along IN, its Y is world y
TRACK_Q = q(TRACK)

# -- the wooden block and the hand ------------------------------------------------------------------------------------
BLOCK = (0.150, 0.150, 0.020)  # pine, screwed to the table
FINGER_T = 0.017  # the index finger, back of the finger to pad
ENTRY = np.array([0.270, -0.060, TOP + BLOCK[2] + FINGER_T])  # the ink line on the index fingertip's pad
BLOCK_POS = (0.335, -0.050, TOP + BLOCK[2] / 2)
HAND_MASS = 0.50  # kg, to weigh on the real prop
# Hand frame: origin at ENTRY, axes as the world's. Palm up, fingers pointing at the robot (-x), thumb to -y.
# (name, x from, x to, y centre, width, top above the pad, colour key)
FINGERS = (
    ("index", 0.020, 0.072, 0.000, 0.018, 0.0),
    ("middle", -0.016, 0.072, 0.0195, 0.018, 0.0),
    ("ring", -0.008, 0.072, 0.0385, 0.017, 0.0),
    ("little", 0.010, 0.072, 0.0565, 0.015, -0.002),
)
PALM = ((0.072, 0.168), (-0.010, 0.066), 0.008)  # x span, y span, top above the pad (the palm is 25 mm thick)
THUMB = ((0.070, 0.135), (-0.046, -0.012), -0.003)
WRIST = ((0.168, 0.194), (-0.004, 0.058), 0.006)
HAND_BOTTOM = -FINGER_T  # every part lies on the block
# The index fingertip: its last 28 mm, under a 4 mm silicone pad that dents on a spring where something presses it
TIP_X = (-0.008, 0.020)
PAD_DEPTH = 0.004
PAD_RATE = 1000.0  # N/m: the sim pad dents 1 mm under 1 N (a model value: the real bench reads the dent on video)
PAD_GIVE = 0.003

# -- the tweezers: Tweezerman slant tweezer, stainless steel, 95 mm long, 10 mm wide ----------------------------------
# Tool frame: origin at the middle of the joined end's top, z along the tweezers toward the joined end (the tips are
# at -z), the legs either side along y, their 10 mm width along x.
TW_LONG = 0.095
TW_WIDE = 0.010
TW_MASS = 0.015  # kg
LEG_T = 0.002  # real legs are about 1 mm thick; the sim's are 2 mm so their contacts don't tunnel. Inner faces are real
ROOT_LONG = 0.008  # the joined end
PIVOT_Z = -ROOT_LONG
UPPER_TO = 0.058  # along a leg from the joined end: the 10 mm wide part
LOWER_TO = 0.078  # then 6 mm wide to here
SLANT = math.radians(25.0)  # the tips' slant: their edge runs 25 degrees off the legs' line
JAW = (0.003, LEG_T, 0.007)  # the slanted tip: 3 mm deep, 7 mm along the slant edge
JAW_X = -0.0003
JAW_ALONG = 0.0832  # the jaw's centre along the leg from the joined end
JAW_REACH = JAW[2] / 2 * math.cos(SLANT) + JAW[0] / 2 * math.sin(SLANT)  # jaw centre to the very tip, along the leg
TIP_GAP = 0.006  # between the tips, open at rest (about 6 mm: to measure on the real pair)
SPLAY = math.asin(TIP_GAP / 2 / JAW_ALONG)  # each leg runs this far out from the line (2.1 degrees)
# The grip section: where fingers hold tweezers, a third of the way down, each leg's outer face runs straight for 13 mm,
# 7.2 mm across the pair. In the sim each leg slides straight in on a sprung slide, and its grip section touches the
# fingers through five 3 x 3 mm raised pads (the corners and the middle), so the simulator holds it at five points
# rather than one; see the fidelity note.
GRIP_FROM, GRIP_TO = 0.022, 0.035  # below the top
GRIP_ACROSS = 0.0072
GRIP_T = 0.002
GRIP_BUMP = (0.003, GRIP_T, 0.003)
GRIP_BUMPS = tuple((sx * (TW_WIDE / 2 - GRIP_BUMP[0] / 2), 0.0, sz * ((GRIP_TO - GRIP_FROM) / 2 - GRIP_BUMP[2] / 2))
                   for sx, sz in ((-1, -1), (-1, 1), (1, -1), (1, 1)))  # around the middle one, in the slide's frame
LEG_RATE = 3000.0  # N/m per leg: 5.4 N squeezes the tips onto the splinter (a sim model value; not scored)
LEG_DAMPING = 20.0
LEG_MASS = 0.006
JAW_TURN = rot_y(-SLANT)  # in a leg's own frame (before its splay)
CLOSE_TRAVEL = TIP_GAP / 2 - 0.0002  # each leg's slide stops just short of the tips meeting


def leg_turn(side: int) -> np.ndarray:
    """A leg's splay: side +1 is the left leg (+y), -1 the right."""
    return rot_x(side * SPLAY)


def leg_rest(side: int) -> np.ndarray:
    """A leg's slide origin in the tool frame: the middle of its grip section."""
    return np.array([0.0, side * (GRIP_ACROSS / 2 - GRIP_T / 2), -(GRIP_FROM + GRIP_TO) / 2])


def _pivot(side: int) -> np.ndarray:
    return np.array([0.0, side * LEG_T / 2, PIVOT_Z])


def leg_boxes(side: int, colour) -> tuple:
    """A leg's boxes in its slide's frame (origin at leg_rest), legs open: the grip section's four corner pads (its
    middle one is the slide's own box), the grip section itself drawn only, and the leg."""
    r = leg_turn(side)
    off = _pivot(side) - leg_rest(side)

    def box(centre, size, turn=np.eye(3), name=""):
        return (tuple(float(v) for v in off + r @ np.asarray(centre)), tuple(size), colour, q(r @ turn), name)

    tag = "l" if side > 0 else "r"
    bumps = tuple((c, GRIP_BUMP, colour, 0.0, f"grip_{tag}_{i}") for i, c in enumerate(GRIP_BUMPS))
    plate = ((0.0, 0.0, 0.0), (TW_WIDE, GRIP_T * 0.9, GRIP_TO - GRIP_FROM), colour, 0.0, f"grip_{tag}_plate", 1.0, True)
    return bumps + (plate,) + (
        box((0.0, 0.0, -UPPER_TO / 2), (TW_WIDE, LEG_T, UPPER_TO), name=f"leg_{tag}_upper"),
        box((0.0, 0.0, -(UPPER_TO + LOWER_TO) / 2), (0.006, LEG_T, LOWER_TO - UPPER_TO), name=f"leg_{tag}_lower"),
        box((JAW_X, 0.0, -JAW_ALONG), JAW, r.T @ JAW_TURN, name=f"jaw_{tag}"),  # tips bent back parallel
    )


def jaw_corners(side: int, travel: float = 0.0) -> np.ndarray:
    """The jaw box's 8 corners in the tool frame, with the leg slid ``travel`` (m, along y) from rest. The tips are bent
    back from the legs' splay, so their inner faces meet flat, as a real pair's do."""
    r = leg_turn(side)
    half = np.array(JAW) / 2
    corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * half
    centre = _pivot(side) + r @ np.array([JAW_X, 0.0, -JAW_ALONG])
    return centre + corners @ JAW_TURN.T + np.array([0.0, travel, 0.0])


TIP_END = PIVOT_Z - JAW_ALONG - JAW_REACH  # the very tips, in the tool frame (about -95 mm)

# -- the cup: Choice 1 oz graduated polypropylene medicine cup, 47.6 mm across the top, 28.6 mm at the bottom, 34.9
# mm tall, taped to the table. The sim's cup steps out in three rings of twelve flat sides (the real one tapers):
# the bottom ring as wide as the real floor, the middle one as the real cup halfway up, the top one as its rim, so the
# tweezers lean on the same floor and rim as in the real cup.
CUP_XY = (0.220, -0.200)
CUP_TOP_D = 0.0476
CUP_BOTTOM_D = 0.0286
CUP_TALL = 0.0349
CUP_WALL = 0.0005  # real wall; the sim's walls are SIM_WALL thick, outward
SIM_WALL = 0.0021  # 2 mm or more: thinner walls tunnel in PhysX
CUP_FLOOR = 0.0010
CUP_SIDES = 12
CUP_RINGS = 3


def cup_radius(z: float) -> float:
    """The real cup's inside radius at height z above the table."""
    top, bottom = CUP_TOP_D / 2 - CUP_WALL, CUP_BOTTOM_D / 2 - CUP_WALL
    return bottom + (top - bottom) * min(max(z, 0.0), CUP_TALL) / CUP_TALL


def cup_rings() -> list[tuple[float, float, float]]:
    """(from, to, inside radius) of each sim ring, heights above the table."""
    step = CUP_TALL / CUP_RINGS
    at = (0.0, CUP_TALL / 2, CUP_TALL)
    return [(i * step, (i + 1) * step, cup_radius(at[i])) for i in range(CUP_RINGS)]


# The tweezers lean in the cup, tips down against its far side, resting on its near rim: lean_pose() finds the angle and
# position; the start uses the fixed values below (a test checks they match), so it never changes.
LEAN = math.radians(40.15)
TWEEZER_POS = (0.17052, -0.200, TOP + 0.07369)
TWEEZER_TURN = rot_y(-LEAN)  # the joined end leans back toward the robot (-x), the tips sit at the far (+x) side


def _corners(centre, size, r) -> np.ndarray:
    half = np.array(size) / 2
    k = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * half
    return np.asarray(centre) + k @ np.asarray(r).T


def tweezer_points() -> np.ndarray:
    """Corners of every tweezer box in the tool frame, legs open."""
    pts = [_corners((0.0, 0.0, -ROOT_LONG / 2), (TW_WIDE, 2 * LEG_T, ROOT_LONG), np.eye(3))]
    for side in (1, -1):
        rest = leg_rest(side)
        pts.append(rest + _corners((0.0, 0.0, 0.0), GRIP_BUMP, np.eye(3)))
        for spec in leg_boxes(side, (0.0, 0.0, 0.0)):
            turn = spec[3] if isinstance(spec[3], tuple) else (1.0, 0.0, 0.0, 0.0)
            pts.append(rest + _corners(spec[0], spec[1], quat_matrix(turn)))
    return np.vstack(pts)


def lean_pose() -> tuple[float, np.ndarray]:
    """The lean (rad) and joined-end position at which the tweezers rest in the sim's cup: tips on its floor against its
    far wall, the legs' back edges on its near rim."""
    pts = tweezer_points()
    cx, cy = CUP_XY
    rings = cup_rings()
    rim = np.array([cx - rings[-1][2], cy, TOP + CUP_TALL])

    def place(lean):
        r = rot_y(-lean)
        w = pts @ r.T
        z = TOP + CUP_FLOOR - w[:, 2].min()
        dx = np.inf
        for lo, hi, radius in rings:  # every point inside the cup within its ring's wall
            inside = (w[:, 2] + z - TOP >= lo - 1e-9) & (w[:, 2] + z - TOP < hi)
            if inside.any():
                dx = min(dx, np.min(np.sqrt(np.maximum(radius**2 - w[inside, 1] ** 2, 0.0)) - w[inside, 0]))
        return r, np.array([cx + dx, cy, z])

    lo, hi = math.radians(1.0), math.radians(70.0)
    for _ in range(60):
        mid = (lo + hi) / 2
        r, pos = place(mid)
        if (r.T @ (rim - pos))[0] + TW_WIDE / 2 > 0:
            hi = mid
        else:
            lo = mid
    lean = (lo + hi) / 2
    return lean, place(lean)[1]
