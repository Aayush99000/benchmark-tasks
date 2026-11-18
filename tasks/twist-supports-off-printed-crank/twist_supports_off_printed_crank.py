"""On the station table: an open-frame 3D printer (black aluminium frame, black textured build sheet) at the far
middle of the bench, its bed slid forward to the front and its print head parked high. On the bed stands a finished
orange print, a crank link: two round bosses with holes, joined by a flat bar; one boss stands on the bed, the bar
runs across from its top, and the other boss stands on top of the bar's far end. Thin orange tree-shaped supports,
printed with it, stand on the bed under the bar and under the raised boss's rim. A white open bin stands on the table
beside the printer's bed, on the right, in front of its display; a blue tray is in front of the right arm, nearer
the robot. The printer, bin and tray are fixed; the print is loose.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # labkit sits next to this file

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

from labkit import (  # noqa: E402
    Container, Model, Pose, Prim, Shaped, Trial, all_resting, body_point, forward_grip, ready_pose, released, rolled,
    snapped_off,
)

SCENE = "station"
TOP = TABLE_TOP[SCENE]
ASSETS = HERE / "assets"

# -- colours (sRGB 0..1) ------------------------------------------------------------------------------------------------
ORANGE, WHITE, BLUE, BLACK = (0.93, 0.42, 0.12), (0.93, 0.93, 0.91), (0.15, 0.35, 0.85), (0.09, 0.09, 0.10)

# -- the printer: the manufacturer's Ender-3 CAD (GPL-3.0, assets/LICENSE-ender3.txt) --------------------------------
# Its mesh (assets/ender3.usda) has its origin at its bounding-box centre; the colliders are boxes round the parts a
# print or an arm can touch, from the CAD's part bounds (world, m, heights from the table top):
PRINTER_CENTRE = (0.3468, -0.0099, 0.3052)
PRINTER_SIZE = (0.4486, 0.4424, 0.6104)
_PRINTER_BOXES = {  # (x0, x1), (y0, y1), (z0, z1)
    "base": ((0.256, 0.546), (-0.165, 0.165), (0.0, 0.0425)),
    "bed": ((0.123, 0.358), (-0.118, 0.118), (0.0795, 0.1035)),  # hot bed with its build sheet, on the Y carriage
    "display": ((0.254, 0.366), (-0.227, -0.125), (0.0, 0.0965)),
    "upright_l": ((0.390, 0.430), (0.150, 0.170), (0.0425, 0.4645)),
    "upright_r": ((0.390, 0.430), (-0.170, -0.150), (0.0425, 0.4645)),
    "top_bar": ((0.391, 0.411), (-0.170, 0.170), (0.4435, 0.4645)),
    "x_beam": ((0.349, 0.405), (-0.182, 0.204), (0.2435, 0.2895)),  # raised 10 cm, as after a print
    "print_head": ((0.307, 0.392), (-0.073, 0.005), (0.1835, 0.2985)),
    "power_supply": ((0.411, 0.524), (-0.173, -0.123), (0.0905, 0.3045)),
}
PRINTER = Model(
    "printer", pos=(PRINTER_CENTRE[0], PRINTER_CENTRE[1], TOP + PRINTER_CENTRE[2]), size=PRINTER_SIZE, color=BLACK,
    usd=str(ASSETS / "ender3.usda"),
    colliders=tuple(Prim("box", tuple((a + b) / 2 - c for (a, b), c in zip(box, PRINTER_CENTRE)),
                         tuple(b - a for a, b in box)) for box in _PRINTER_BOXES.values()),
)
BED_TOP = TOP + _PRINTER_BOXES["bed"][2][1]

# -- the print: a crank link and its four tree supports, as printed (assets/make_print.py) ------------------------------
# Every body's origin is the print's origin on the bed, at (20, 0) cm (the bed's centre is at (24, 0)), body axes as the table's: the bar runs
# along y, the down boss at y +4 cm, the up boss at y -4 cm. Colliders are the part's real cylinders and bar; a
# support is its trunk (a cylinder) and the spread of its branches (a box) up to its tips, 0.6 mm under the part.
PRINT_AT = (0.20, 0.0, BED_TOP)  # printed 4 cm in front of the bed's centre, clear of the raised print head
BOSS_R, BOSS_H, BAR = 0.014, 0.025, (0.016, 0.08, 0.008)
CRANK = Shaped(
    "crank", pos=PRINT_AT, size=(0.028, 0.108, 0.058), box_at=(0.0, 0.0, 0.029), mass=0.025, color=ORANGE,
    mesh=str(ASSETS / "crank.usda"),
    parts=(Prim("cylinder", (0.0, 0.04, BOSS_H / 2), (BOSS_R, BOSS_R, BOSS_H)),  # down boss, on the bed
           Prim("box", (0.0, 0.0, BOSS_H + BAR[2] / 2), BAR),  # the bar, 25 mm up
           Prim("cylinder", (0.0, -0.04, BOSS_H + BAR[2] + BOSS_H / 2), (BOSS_R, BOSS_R, BOSS_H))),  # up boss
)
# Break thresholds (estimates, not yet measured; the README gives the procedure to measure them on real prints): a
# support holds a straight pull of 15 N (60 times the part's weight) but lets go at a bend of 0.10 N m.
BREAK_FORCE, BREAK_TORQUE = 15.0, 0.10
# The trees as assets/make_print.py builds them (print frame, mm: x along the bar, y across it, z up from the bed):
# trunk foot (x, y), split height, trunk radius, and each branch's tip (x, y, z). The tips stop 0.2 mm under the part.
Z_TIP, Z_RIM = 24.8, 32.8  # under the bar (its underside 25 mm up); under the up boss's rim (33 mm up)
TREES = [
    ((-35, 0), 20, 3.2, [(-41, -5, Z_TIP), (-41, 5, Z_TIP), (-35, -5, Z_TIP), (-35, 5, Z_TIP), (-29, -5, Z_TIP),
                         (-29, 5, Z_TIP)]),  # under the bar, at the up boss's end
    ((0, 0), 20, 3.2, [(-6, -5, Z_TIP), (-6, 5, Z_TIP), (0, -5, Z_TIP), (0, 5, Z_TIP), (6, -5, Z_TIP),
                       (6, 5, Z_TIP)]),  # under the bar's middle
    ((-40, 18), 18, 2.6, [(-45, 12, Z_RIM), (-35, 12, Z_RIM), (-50, 9.5, Z_RIM)]),  # under the up boss's rim
    ((-40, -18), 18, 2.6, [(-45, -12, Z_RIM), (-35, -12, Z_RIM), (-31, -9.5, Z_RIM)]),
]


def _body(px: float, py: float, pz: float = 0.0) -> np.ndarray:
    """Print frame (mm) to body frame (m): the print is turned 90 degrees, its x along the table's y."""
    return np.array([-py, px, pz]) / 1000.0


def _seg_box(a, b, r: float) -> Prim:
    """A box round the segment a-b (body frame, m) of radius r: a branch limb, as an axis-aligned box."""
    a, b = np.asarray(a), np.asarray(b)
    lo, hi = np.minimum(a, b) - r, np.maximum(a, b) + r
    return Prim("box", tuple(float(v) for v in (lo + hi) / 2), tuple(float(v) for v in hi - lo))


def _tree_parts(k: int) -> tuple:
    """Colliders for tree k: its trunk (a cylinder from the bed to the split) and each branch in two limbs (split to
    mid, mid to tip), as make_print.py's tree() lays them out."""
    (bx, by), split, rb, tips = TREES[k]
    base = _body(bx, by)
    parts = [Prim("cylinder", (float(base[0]), float(base[1]), split / 2000), (rb / 1000, rb / 1000, split / 1000))]
    sp = _body(bx, by, split)
    for tx, ty, tz in tips:
        mid = _body((bx + tx) / 2, (by + ty) / 2, split + (tz - split) * 0.55)
        tip = _body(tx, ty, tz - 1.2)  # the 0.8 mm tip sphere's centre: with the 1 mm limb box, 0.2 mm under the part
        for (a, b, r) in ((sp, mid, 0.0016), (mid, tip, 0.0010)):  # each limb in three boxes, to follow its slant
            for i in range(3):
                parts.append(_seg_box(a + (b - a) * i / 3, a + (b - a) * (i + 1) / 3, r))
    return tuple(parts)


def _support(k: int) -> Shaped:
    parts = _tree_parts(k)
    corners = np.vstack([p.corners() for p in parts])
    lo, hi = corners.min(0), corners.max(0)
    (bx, by), split, rb = TREES[k][:3]
    return Shaped(
        f"support_{k}", pos=PRINT_AT, size=tuple(float(v) for v in hi - lo), box_at=tuple(float(v) for v in (hi + lo) / 2),
        mass=0.0018 if len(TREES[k][3]) > 3 else 0.0012, color=ORANGE, mesh=str(ASSETS / f"support_{k}.usda"),
        parts=parts, snap_to="crank", break_force=BREAK_FORCE, break_torque=BREAK_TORQUE,
    )


# (trunk x, trunk y, split height, trunk radius) per support, body frame, m: where the left hand grips
_SUPPORTS = [tuple(_body(*TREES[k][0])[:2]) + (TREES[k][1] / 1000, TREES[k][2] / 1000) for k in range(len(TREES))]


SUPPORTS = [_support(k) for k in range(len(_SUPPORTS))]
SUPPORT_NAMES = [s.name for s in SUPPORTS]

# -- bin and tray -------------------------------------------------------------------------------------------------------
BIN_POS, BIN_SIZE, WALL = (0.20, -0.17), (0.09, 0.09, 0.055), 0.003  # beside the bed, in front of the display
TRAY_POS, TRAY_SIZE = (0.16, -0.30), (0.10, 0.14, 0.025)  # long along y, as the part lies
BIN = Container("bin", pos=(*BIN_POS, TOP + WALL / 2), size=BIN_SIZE, wall=WALL, color=WHITE)
TRAY = Container("tray", pos=(*TRAY_POS, TOP + WALL / 2), size=TRAY_SIZE, wall=WALL, color=BLUE)

OBJECTS = [PRINTER, CRANK, *SUPPORTS, BIN, TRAY]
SPECS = {o.name: o for o in OBJECTS}

# -- scoring ----------------------------------------------------------------------------------------------------------
# A support is off once its joint to the part has let go: its centre has moved over 5 mm in the part's frame. In the
# bin: its centre inside the bin's walls and below the rim. The part in the tray: its centre inside the tray's walls
# and its lowest point on the tray's floor (within 1 cm; the part stands 58 mm, taller than the tray's 25 mm rim). Gates (score 0 for the rest of the trial): the part falls
# off the table, falls freely (faster than a 10 cm drop lands: 1.4 m/s down), or lands in the bin.
TRIAL = Trial(gate=False)
DROP_SPEED = float(np.sqrt(2 * 9.81 * 0.10))
PER_OFF, PER_BINNED, TRAY_CREDIT = 0.10, 0.05, 0.40
PER_SUPPORT = PER_OFF + PER_BINNED  # 0.15 for a support off and in the bin


def centre(name: str, state) -> np.ndarray:
    """An object's bounding-box centre in the world (a Shaped body's origin is on the bed, not at its centre)."""
    spec = SPECS[name]
    return body_point(state, spec.box_at) if isinstance(spec, Shaped) else np.asarray(state.pos, float)


def holds(container: Container, objects, name: str) -> bool:
    return container.holds(objects[container.name], centre(name, objects[name]))


def on_tray_floor(objects, name: str = "crank") -> bool:
    tray = objects["tray"]
    local = np.asarray(centre(name, objects[name])) - np.asarray(tray.pos)
    half = np.asarray(TRAY_SIZE[:2]) / 2 - WALL
    return bool(np.all(np.abs(local[:2]) < half) and objects[name].low[2] < TOP + WALL + 0.01)


def supports_off(objects) -> list[bool]:
    return [snapped_off(objects[s.name], objects["crank"], s, CRANK) for s in SUPPORTS]


def score(objects) -> float:
    """0.10 for each support broken off the part, +0.05 more if it is in the bin; +0.40 for the part on the tray's
    floor with all four supports off. 0 for the trial once the part falls off the table, drops faster than a 10 cm
    fall, or lands in the bin."""
    crank = objects["crank"]
    c = centre("crank", crank)
    TRIAL["gate"] |= bool(c[2] < TOP - 0.02 or crank.lin_vel[2] < -DROP_SPEED or holds(BIN, objects, "crank"))
    if TRIAL["gate"]:
        return 0.0
    off = supports_off(objects)
    binned = sum(o and holds(BIN, objects, s) for o, s in zip(off, SUPPORT_NAMES))
    tray = TRAY_CREDIT if all(off) and on_tray_floor(objects) else 0.0
    return round(PER_OFF * sum(off) + PER_BINNED * binned + tray, 6)


def success(objects) -> bool:
    """Full score, everything still, and no gripper holding the part; must hold for 1 s."""
    return (score(objects) >= 1.0 - 1e-9 and all_resting(objects, ["crank", *SUPPORT_NAMES])
            and released(objects["crank"]))


# -- scripted demo ------------------------------------------------------------------------------------------------------
# The right arm takes the print off the bed by its bar and holds it up over the scrap bin; the left arm grips each
# support's trunk from the front (a forward grip), bends it until it snaps, and lets it drop into the bin. For the last support, which sits behind the up boss (beside the held bar), the right arm first turns the part 60 degrees so that
# support faces the left arm. Then the right arm sets the part in the tray. Open loop: run with -E object_jitter=0.
# The right hand pinches the flat bar across its 16 mm width (fingers closing along x), just off the part's middle:
# flat faces stop the part spinning in the grip (in Isaac Sim, 16 Nov, a pinch on the round boss let it swing 70
# degrees when the left hand touched it) and the part's weight hangs almost under the grip.
GRIP_LOCAL = np.array([0.0, -0.004, 0.040])  # the right TCP on the bar, body frame (bar 25-33 mm up; pads to 27 mm)
GRIP_Z = float(GRIP_LOCAL[2])
GRIP_YAW = np.pi / 2  # fingers closing along x
HOLD = np.array([0.23, -0.1565, TOP + 0.20])  # the right TCP while the supports come off: the part held up so all
# four supports hang over the bin, 10 cm above its rim (snapped off, each drops in)
BAR_GRIP = np.asarray(PRINT_AT) + GRIP_LOCAL
UP_BOSS = BAR_GRIP  # (kept for the tools that read it)
HALF_OPEN = 0.5 * FINGER_OPEN  # 47 mm across the 16 mm bar: wide open, the far fingertip reached the print head
TURN = -np.pi / 3  # the right wrist's turn that brings the last support (behind the up boss) round to the front
LAST_SHIFT = np.array([0.0, float(os.environ.get("DEMO_LAST_DY", "0.045")), 0.0])  # and the part slid toward the middle
# for it, so the left arm needn't reach as far across into the right arm's space (still over the bin)
# Tuning for a GPU session (unset, the demo is exactly as below): DEMO_AIM="dx,dy,dz" (mm) shifts every left grip,
# DEMO_ROCK (deg) and DEMO_AJAR (scale) change the rock and the finger opening, DEMO_ONLY=1 stops the demo after the
# first support (a 1-minute check of the grip and the snap).
_AIM = np.array([float(v) for v in os.environ.get("DEMO_AIM", "0,0,0").split(",")]) / 1000.0
ROCK = float(os.environ.get("DEMO_ROCK", "12"))  # degrees each way, about the support's top (where it meets the part)
# Rolling the wrist about its own axis only twists the fingers round the 6 mm trunk, which slips (Isaac Sim, 16 Nov:
# the support turned 7 degrees and sprang back); swinging the grip in an arc about the support's top bends it
# sideways with the pads' clamping force, as a hand does.
ROCK_BY = {3: float(os.environ.get("DEMO_ROCK_LAST", "20"))}  # the last support, after the right wrist's turn: the
# wrist is near its limit and gives, so a 12-degree bend only tilted it 5 degrees and it held (Isaac Sim, 16 Nov)
ORDER = (1, 2, 0)  # 1 first (its fingers need the room 0 would take), then the one in front of the up boss (its
# branches are in the way of the fingers closing on 0), then 0; support_3 after the turn
LAST = 3  # the support behind the up boss
OVER_BIN = np.array([BIN_POS[0], BIN_POS[1], TOP + 0.12])
BACK_OFF = 0.04  # how far in front of a trunk the left fingers wait before closing on it
DROP = 0.006  # how far the left hand pulls a broken-off support down before drawing it out (the bed is 11 mm below)
GRIP_TRUNK = 0.004  # the left pads close on the trunk 4 mm above its foot (higher, the fingers reach the branches)
# How far the left fingers open round each support's trunk (fraction of fully open, 94 mm): from tools/grip_search.py,
# the openings at which the finger hulls clear the tree's own branches, its neighbours and the part (at 16% the
# fingertips hit the tree's branches: Isaac Sim, 16 Nov). Support 1 sits between support 0 and the down boss.
AJAR_BY = {k: v * float(os.environ.get("DEMO_AJAR", "1")) for k, v in {0: 0.45, 1: 0.35, 2: 0.45, 3: 0.25}.items()}
# support 3 sits under the up boss's rim: at 45% the far finger caught the boss and lifted the part 18 mm (Isaac Sim,
# 16 Nov); at 25% (23 mm, its trunk is 5 mm) the grip search still clears its branches and the part by 6-8 mm
AJAR = 0.45 * FINGER_OPEN


# Measured in Isaac Sim (16 Nov, the same in two runs with object_jitter=0): the part settles 3.6 degrees in
# the right hand when lifted, so after the turn its last support sits about 5.5 mm toward +y of the plan; aim there.
AIM_BY = {3: np.array([0.0, float(os.environ.get("DEMO_AIM_LAST_DY", "-0.0055")), 0.0])}


def _hold(k: int) -> np.ndarray:
    return HOLD + (LAST_SHIFT if k == LAST else 0.0)


def _trunk_grip(k: int, yaw: float = 0.0) -> np.ndarray:
    """Where the left TCP grips support k's trunk (GRIP_TRUNK above its foot) while the right arm holds the part at HOLD,
    turned ``yaw`` about the vertical through the grip."""
    tx, ty = _SUPPORTS[k][:2]
    c, s_ = np.cos(yaw), np.sin(yaw)
    dx, dy = tx - GRIP_LOCAL[0], ty - GRIP_LOCAL[1]  # from the grip
    return _hold(k) + np.array([c * dx - s_ * dy, s_ * dx + c * dy, GRIP_TRUNK - GRIP_Z]) + _AIM + AIM_BY.get(k, 0.0)


def _arc(point, pivot, deg: float) -> np.ndarray:
    """``point`` swung ``deg`` about the world x axis through ``pivot`` (as labkit.rolled turns the grip)."""
    from yam_arena.urdf import axis_angle_matrix

    return np.asarray(pivot) + axis_angle_matrix(np.eye(3)[0], np.radians(deg)) @ (np.asarray(point) - np.asarray(pivot))


def _ready(arm: str, duration: float, finger: float, label: str) -> Pose:
    """The arm's ready pose with the fingers at ``finger`` (labkit's ready_pose opens them)."""
    w = ready_pose(arm, duration, label)
    return Pose(w.duration, w.pos, finger, label, rot=w.rot)


def _one_support(k: int, yaw: float = 0.0) -> list:
    """Left hand: come in level to the trunk with the fingers part open, close, bend the support about its top one way
    and the other until it snaps, pull it down clear of the part, and open over the bin so it drops in; back out."""
    g = _trunk_grip(k, yaw)
    back = g - [BACK_OFF, 0.0, 0.0]
    line_up = back - [min(0.012, max(0.0, back[0] - 0.172)), 0.0, 0.0]  # a forward grip reaches x 17 cm here
    rot = forward_grip(0.0)
    ajar = AJAR_BY[k] * FINGER_OPEN
    rock = ROCK_BY.get(k, ROCK)
    top = max(p.at[2] + p.size[2] / 2 for p in SUPPORTS[k].parts)  # the support's top, where its tips meet the part
    pivot = g + np.array([0.0, 0.0, top - GRIP_TRUNK])
    low = g - [0.0, 0.0, DROP]
    return [
        Pose(1.5, tuple(line_up), ajar, f"support {k}: line up in front of it", rot=rot),
        Pose(0.6, tuple(back), ajar, f"support {k}: in front of its trunk", rot=rot),
        Pose(1.0, tuple(g), ajar, f"support {k}: fingers round the trunk", rot=rot),
        Pose(0.6, tuple(g), FINGER_CLOSED, f"support {k}: grip", rot=rot),
        Pose(0.8, tuple(_arc(g, pivot, rock)), FINGER_CLOSED, f"support {k}: rock one way", rot=rolled(rot, rock)),
        Pose(0.8, tuple(_arc(g, pivot, -rock)), FINGER_CLOSED, f"support {k}: rock the other way",
             rot=rolled(rot, -rock)),
        # bend it once more (a borderline grip may not snap it the first time: Isaac Sim's contacts vary run to run)
        Pose(0.8, tuple(_arc(g, pivot, rock)), FINGER_CLOSED, f"support {k}: rock again", rot=rolled(rot, rock)),
        Pose(0.8, tuple(_arc(g, pivot, -rock)), FINGER_CLOSED, f"support {k}: and back", rot=rolled(rot, -rock)),
        Pose(0.6, tuple(low), FINGER_CLOSED, f"support {k}: pull it down off the part", rot=rot),
        Pose(0.5, tuple(low), ajar, f"support {k}: let it drop into the bin", rot=rot),
        Pose(0.8, tuple(back - [0.0, 0.0, DROP]), ajar, f"support {k}: draw back", rot=rot),
    ]


def _span(ws) -> float:
    return sum(w.duration for w in ws)


RIGHT_LIFT = 2.0 + 1.5 + 0.8 + 1.2 + 2.0  # the right arm's moves until it holds the part up
TURN_S = 1.5


def _right() -> list:
    above = UP_BOSS + [0, 0, 0.03]
    tray_tcp = np.array([TRAY_POS[0], TRAY_POS[1] + GRIP_LOCAL[1], TOP + WALL + GRIP_Z + 0.006])  # part's middle on tray's
    first = sum(_span(_one_support(k)) for k in ORDER)
    return [
        Waypoint(2.0, tuple(above), HALF_OPEN, "over the bar", GRIP_YAW),
        Waypoint(1.5, tuple(BAR_GRIP), HALF_OPEN, "down onto the bar", GRIP_YAW),
        Waypoint(0.8, tuple(BAR_GRIP), FINGER_CLOSED, "grip the bar", GRIP_YAW),
        Waypoint(1.2, tuple(above), FINGER_CLOSED, "lift the print off the bed", GRIP_YAW),
        Waypoint(2.0, tuple(HOLD), FINGER_CLOSED, "hold it up over the scrap bin", GRIP_YAW),
        Waypoint(first, tuple(HOLD), FINGER_CLOSED, "hold while three supports come off", GRIP_YAW),
        Waypoint(TURN_S, tuple(_hold(LAST)), FINGER_CLOSED, "turn the last support to the front", GRIP_YAW + TURN),
        Waypoint(_span(_one_support(LAST, TURN)), tuple(_hold(LAST)), FINGER_CLOSED, "hold while it comes off",
                 GRIP_YAW + TURN),
        Waypoint(TURN_S, tuple(HOLD), FINGER_CLOSED, "turn back", GRIP_YAW),
        Waypoint(2.0, tuple(tray_tcp + [0, 0, 0.08]), FINGER_CLOSED, "over the tray", GRIP_YAW),
        Waypoint(1.5, tuple(tray_tcp), FINGER_CLOSED, "lower into the tray", GRIP_YAW),
        Waypoint(0.8, tuple(tray_tcp), FINGER_OPEN, "let go", GRIP_YAW),
        Waypoint(1.5, tuple(tray_tcp + [0, 0, 0.10]), FINGER_OPEN, "clear of the part", GRIP_YAW),
    ]


def demo():
    if os.environ.get("DEMO_ONLY"):  # the lift and the first support only, then both arms hold still
        right = _right()[:5] + [Waypoint(_span(_one_support(ORDER[0])) + 2.0, tuple(HOLD), FINGER_CLOSED, "hold",
                                         GRIP_YAW)]
        left = [ready_pose("left", RIGHT_LIFT, "wait for the print to be held up")] + _one_support(ORDER[0])
        return {"left": left, "right": right}
    left = [ready_pose("left", RIGHT_LIFT, "wait for the print to be held up")]
    for k in ORDER:
        left += _one_support(k)
    left.append(ready_pose("left", TURN_S, "wait for the turn"))
    left += _one_support(LAST, TURN)
    left.append(ready_pose("left", 4.0))  # from the far right of its reach: slow enough for the IK to follow
    return {"left": left, "right": _right()}
