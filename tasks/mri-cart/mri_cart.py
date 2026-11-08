"""On the station table, a patient cart about to go into an MRI scanner room. Its top shelf (light grey, with a 2.5 cm
rim) is fixed on the table straight in front of the arms. On it lie six loose items: steel bandage scissors, tucked
1.7 cm from each of two blue foam positioning pads; the silver chest piece of a stethoscope; the patient's phone; and a white
plastic water bottle. Nearer the arms, between them, stands an open grey bin for whatever can't go into the scanner
room. Steel and electronics must stay out: the magnet pulls them in.
"""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]


def _xyz(xy, z) -> tuple[float, float, float]:
    return float(xy[0]), float(xy[1]), float(z)


def _tray(prefix: str, xy, inner, wall_h: float, wall_t: float, floor_t: float, color) -> list[Box]:
    """An open fixed tray: a floor and four walls around an inner (x, y) space."""
    (x, y), (ix, iy) = xy, inner
    ox, oy = ix + 2 * wall_t, iy + 2 * wall_t
    z = TOP + floor_t + wall_h / 2
    return [
        Box(f"{prefix}_floor", pos=_xyz(xy, TOP + floor_t / 2), size=(ox, oy, floor_t), color=color),
        Box(f"{prefix}_wall_far", pos=_xyz((x + (ix + wall_t) / 2, y), z), size=(wall_t, oy, wall_h), color=color),
        Box(f"{prefix}_wall_near", pos=_xyz((x - (ix + wall_t) / 2, y), z), size=(wall_t, oy, wall_h), color=color),
        Box(f"{prefix}_wall_left", pos=_xyz((x, y + (iy + wall_t) / 2), z), size=(ix, wall_t, wall_h), color=color),
        Box(f"{prefix}_wall_right", pos=_xyz((x, y - (iy + wall_t) / 2), z), size=(ix, wall_t, wall_h), color=color),
    ]


# The cart's top shelf: 24 x 36 cm inside, a 2.5 cm rim, fixed. The bin: 20 x 20 cm inside, walls 6 cm tall.
CART_XY, CART_IN, CART_RIM, CART_FLOOR = (0.37, 0.0), (0.24, 0.36), 0.025, 0.01
BIN_XY, BIN_IN, BIN_WALL, BIN_FLOOR = (0.13, 0.0), (0.20, 0.20), 0.06, 0.005
WALL_T = 0.008
SHELF = TOP + CART_FLOOR  # the cart shelf's surface
BIN_BOTTOM = TOP + BIN_FLOOR
CART_GREY, BIN_GREY = (0.80, 0.80, 0.78), (0.35, 0.35, 0.37)

# The items: (x, y) centre, heading in degrees.
SCISSORS, SCISSORS_KG = (0.14, 0.04, 0.012), 0.08  # steel bandage scissors, closed
PHONE, PHONE_KG = (0.147, 0.072, 0.011), 0.19  # in its case
CHEST_R, CHEST_H, CHEST_KG = 0.022, 0.013, 0.10  # a stethoscope's chest piece, steel
PAD, PAD_KG = (0.10, 0.05, 0.04), 0.025  # foam positioning pads, MR safe
BOTTLE_R, BOTTLE_H, BOTTLE_KG = 0.03, 0.10, 0.12  # a plastic water bottle, part full, MR safe
STEEL, BLACK, BLUE, WHITE = (0.72, 0.73, 0.75), (0.05, 0.05, 0.06), (0.15, 0.35, 0.80), (0.92, 0.92, 0.90)
GAP = 0.017  # the scissors lie 1.7 cm from each foam pad
SCISSORS_XY = (0.40, 0.08)
LAYOUT = {
    "scissors": (SCISSORS_XY, 0.0),
    "pad_left": ((0.40, SCISSORS_XY[1] + SCISSORS[1] / 2 + GAP + PAD[1] / 2), 0.0),
    "pad_right": ((0.40, SCISSORS_XY[1] - SCISSORS[1] / 2 - GAP - PAD[1] / 2), 0.0),
    "phone": ((0.37, -0.12), 0.0),
    "chest_piece": ((0.30, -0.05), 90.0),
    "bottle": ((0.29, 0.13), 0.0),
}
UNSAFE = ("scissors", "phone", "chest_piece")  # ferromagnetic or electronic: never into the scanner room
SAFE = ("pad_left", "pad_right", "bottle")  # MR safe: they go in with the patient
ITEMS = UNSAFE + SAFE
HEIGHT = {"scissors": SCISSORS[2], "phone": PHONE[2], "chest_piece": CHEST_H, "pad_left": PAD[2], "pad_right": PAD[2],
          "bottle": BOTTLE_H}


def _box(name, size, kg, color):
    (xy, yaw) = LAYOUT[name]
    return Box(name, pos=_xyz(xy, SHELF + size[2] / 2 + 0.0005), size=size, mass=kg, color=color, yaw=math.radians(yaw))


OBJECTS = [
    *_tray("cart", CART_XY, CART_IN, CART_RIM, WALL_T, CART_FLOOR, CART_GREY),
    *_tray("bin", BIN_XY, BIN_IN, BIN_WALL, WALL_T, BIN_FLOOR, BIN_GREY),
    _box("scissors", SCISSORS, SCISSORS_KG, STEEL),
    _box("pad_left", PAD, PAD_KG, BLUE),
    _box("pad_right", PAD, PAD_KG, BLUE),
    _box("phone", PHONE, PHONE_KG, BLACK),
    Cylinder("chest_piece", pos=_xyz(LAYOUT["chest_piece"][0], SHELF + CHEST_H / 2 + 0.0005), radius=CHEST_R,
             height=CHEST_H, mass=CHEST_KG, color=STEEL),
    Cylinder("bottle", pos=_xyz(LAYOUT["bottle"][0], SHELF + BOTTLE_H / 2 + 0.0005), radius=BOTTLE_R,
             height=BOTTLE_H, mass=BOTTLE_KG, color=WHITE),
]

# -- scoring ---------------------------------------------------------------------------------------------------------
FLOOR_TOL = 0.015  # resting on a tray's floor: the item's lowest point within 1.5 cm of it (items may lie on each other)
COMPLETE_SCORE = 0.9  # score() for the complete state: 1.0 comes only from success() holding for 1 s, which ends
# the trial (task_score takes the last step's score() when success hasn't fired, a hole reviewers reject)


def _inside(s, xy, inner, floor_z) -> bool:
    """The item's centre inside the tray's walls and its lowest point within FLOOR_TOL of the tray's floor."""
    d = np.abs(np.asarray(s.pos[:2], dtype=float) - np.asarray(xy))
    return bool(np.all(d < np.asarray(inner) / 2) and float(s.low[2]) - floor_z <= FLOOR_TOL
                and float(s.low[2]) >= floor_z - 0.003)


def on_cart(s) -> bool:
    return _inside(s, CART_XY, CART_IN, SHELF)


def in_bin(s) -> bool:
    return _inside(s, BIN_XY, BIN_IN, BIN_BOTTOM)


def _grade(objects) -> float:
    """A third for each steel or electronic item in the bin, still; 0 if any MR-safe item has left the cart (taking
    everything off is not getting the cart ready: the pads and the water go in with the patient)."""
    if not all(on_cart(objects[n]) for n in SAFE):
        return 0.0
    out = sum(in_bin(objects[n]) and objects[n].resting() for n in UNSAFE)
    return round(out / len(UNSAFE), 6)


def success(objects):
    """All three steel or electronic items in the bin and still, the MR-safe items still on the cart. Must hold 1 s."""
    return _grade(objects) == 1.0 and all(objects[n].resting() for n in ITEMS)


def score(objects):
    """A third per steel or electronic item in the bin (0.9 when complete; 1.0 only once success() has held 1 s); 0 if
    a foam pad or the water bottle has left the cart."""
    return min(_grade(objects), COMPLETE_SCORE)


# -- the demo --------------------------------------------------------------------------------------------------------
# The left arm takes the scissors from between the foam pads: the fingers come down only 54 mm apart, 7 mm clear of
# the 40 mm scissors and 10 mm from each pad, grip across the scissors' width, lift them straight up and drop them into
# the bin from 3 cm above its floor. Then the right arm takes the phone (across its 72 mm width, the fingers 7 mm inside the rim) and the chest piece
# (across its 44 mm diameter, the fingers closing along x so they stay 4 mm from the rim and from a pad) the same
# way.
PAD_ABOVE = 0.003  # pad tips 3 mm above the shelf
TCP_PAST = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP
TRAVEL_Z = SHELF + 0.12  # a carried item's bottom 4 cm above the bin's walls; no carry passes over the bottle
DROP_Z = BIN_BOTTOM + 0.03  # let go 3 cm above the bin's floor
APPROACH = {"scissors": 0.054, "phone": 0.090, "chest_piece": 0.056}  # finger gap coming down (m)
GRIP = {"scissors": 0.032, "phone": 0.064, "chest_piece": 0.036}  # aimed 4 mm inside each face
DROP_XY = {"scissors": (0.13, 0.05), "phone": (0.13, -0.055), "chest_piece": (0.13, 0.004)}
DROP_YAW = {"scissors": 0.0, "phone": 0.0, "chest_piece": 0.0}  # the phone turned flat across the bin as it is carried
DEMO_ARM = {"scissors": "left", "phone": "right", "chest_piece": "right"}


def _finger(gap: float) -> float:
    return FINGER_OPEN * gap / 0.094


DEMO_AIM_SHIFT_MM = 0.0  # the scissors pick aimed this far toward the left pad (+y), for margin runs
# Margin runs (2026-11-08, traced on the official stack): aimed 3 mm toward either pad, both 1.0, the pads moved at
# most 0.74 mm. Official demo 1.0 three times (one before, two after the instruction was reworded for Dream's API).


def _take(name: str, wait_s: float = 0.0) -> list[Waypoint]:
    (x, y), yaw = LAYOUT[name]
    if name == "scissors":
        y += DEMO_AIM_SHIFT_MM / 1000
    a = math.radians(yaw)
    z = SHELF + PAD_ABOVE + TCP_PAST
    bx, by = DROP_XY[name]
    b = math.radians(DROP_YAW[name])
    open_, grip = _finger(APPROACH[name]), _finger(GRIP[name])
    out = [Waypoint(wait_s, (x, y, TRAVEL_Z), open_, "wait", a)] if wait_s else []
    return out + [
        Waypoint(1.5, (x, y, TRAVEL_Z), open_, "over the item", a),
        Waypoint(1.5, (x, y, z), open_, "descend", a),
        Waypoint(0.5, (x, y, z), open_, "steady", a),
        Waypoint(0.8, (x, y, z), grip, "close", a),
        Waypoint(1.5, (x, y, TRAVEL_Z), grip, "lift straight up", a),
        Waypoint(2.0, (bx, by, TRAVEL_Z), grip, "carry", b),
        Waypoint(1.5, (bx, by, DROP_Z + TCP_PAST), grip, "lower into the bin", b),
        Waypoint(0.8, (bx, by, DROP_Z + TCP_PAST), FINGER_OPEN, "let go", b),
        Waypoint(1.2, (bx, by, TRAVEL_Z), FINGER_OPEN, "out of the bin", b),
    ]


# After its last drop each arm moves back over its own side of the table, clear of the bin: on the official stack the
# left arm left hovering over the bin kept the right arm 4-6 cm short of it, and the phone landed against the bin's
# far wall, tilted 26 degrees (traced 2026-11-08).
PARK = {"left": (0.22, 0.24), "right": (0.22, -0.24)}


def _park(arm: str) -> list[Waypoint]:
    x, y = PARK[arm]
    return [Waypoint(1.5, (x, y, TRAVEL_Z), FINGER_OPEN, "back to its own side", 0.0)]


def demo():
    """The left arm clears the scissors and moves back to its side; the right arm waits at the ready pose until then,
    then clears the phone and the chest piece. The foam pads and the bottle stay on the cart."""
    left = _take("scissors") + _park("left")
    wait = sum(w.duration for w in left)
    right = _take("phone", wait_s=wait) + _take("chest_piece") + _park("right")
    return {"left": left, "right": right}
