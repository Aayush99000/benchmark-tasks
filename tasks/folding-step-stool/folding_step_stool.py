"""On the station table: a child's low plastic folding step stool lies folded flat, its two legs folded under its top.
Unfold it: open each leg until its lock clicks, then stand the stool on its feet.

Each leg turns on a hinge under one end of the top and has its own spring latch, which catches only once the leg is
pushed a little past straight down (98 degrees from folded; its stop is at 100). A leg that is short of its latch is
pushed back by the latch's spring, so it looks open but folds again under a load. The stool is the buildable one in
hardware/ (stool_dimensions.py; build_parts.py makes its printed parts and checks the latch's travel). Standing on its feet with both hands off it, the stool takes a
40 N push down on its top (a stand-in for a child stepping on it).

Run it:  inspect-robots run --embodiment yam_arena -E task=folding_step_stool.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "hardware"))
import fittingkit  # noqa: E402
import stool_dimensions as HW  # noqa: E402  the buildable stool's numbers (mm), shared with its CAD
from fittingkit import Hinge, body, box, cyl  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

SEAT_BLUE = (0.16, 0.50, 0.80)  # the top, a saturated plastic blue (seen from the top camera at 320 px)
LEG_WHITE = (0.90, 0.90, 0.88)
GRIP_GREY = (0.30, 0.32, 0.35)
TREAD = (0.10, 0.36, 0.62)
KNOB_BLACK = (0.12, 0.12, 0.13)


def rot_x(angle: float) -> np.ndarray:
    return axis_angle_matrix(X_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the stool -------------------------------------------------------------------------------------------------------
# Its long side along y (the legs at the -y and +y ends), the middle 31 cm out. The sizes are those of the buildable
# stool (hardware/stool_dimensions.py, in mm; test_the_simulated_stool_is_the_buildable_one holds them equal), a low
# children's folding step stool scaled to the arms' reach; 0.75 kg in all (the top is ballasted to 0.55 kg).
STOOL_X = 0.31
TOP_L, TOP_D, TOP_T = 0.246, 0.20, 0.018  # the top: along y, along x, thick
HINGE_Y = 0.105  # each leg's hinge, this far from the middle, under the top
KNUCKLE_R = 8.0 * MM  # the hinge's knuckle (on the leg), round its axis
LEG_L, LEG_W, LEG_T = 0.100, 0.15, 0.010  # a leg from its hinge axis to its foot's far face; wide (x); its panel's thickness
FOOT_L, FOOT_T = 0.34, 0.016  # the foot rail along the leg's end, longer than the top is deep (7 cm out each side)
GRIP_W, GRIP_T, GRIP_H = 0.07, 0.014, 0.020  # a raised grip rib across each end of the top
HINGE_Z = TOP + KNUCKLE_R  # folded, the knuckles and the foot rails rest on the table
TOP_Z = HINGE_Z + KNUCKLE_R + TOP_T / 2  # the top's middle, folded
TOP_AT = np.array([STOOL_X, 0.0, TOP_Z])
TOP_MASS, LEG_MASS = 0.55, 0.10

OPEN_DEG, LATCH_DEG = HW.OPEN_DEG, HW.LATCH_DEG  # a leg opens to 100 degrees (10 past straight down); it latches at 98
SPRING_FROM_DEG = HW.SPRING_FROM_DEG  # from 80 degrees the latch's bolt rides the leg's wedge and pushes it back
# The push back: the latch spring's force (Lee Spring LC 026C 05 M, 2.189 N/mm, 9.6 N preload) times the wedge's rise
# per radian, 0.10 N m at 80 degrees to 0.165 N m (1.6 N at the foot) just short of 98 (HW.fold_back_torque_nm).
HOLD_TORQUE = 2.0  # N m: a locked leg is held open against its stop [estimate: well above what a 40 N load puts on it]
LOAD_N = 40.0  # the push down on the top once the stool stands on its own [estimate: a stand-in for a step]
GIVE_WAY_TORQUE = 6.0  # N m folding a leg that isn't locked once the stool has taken the load: it gives way
# WHY a rule, not left to the contacts: with both feet held by friction the stool and an unlocked leg 9 degrees short of
# straight make a rigid triangle, and the stool stood under the push (probe run fsP: the +y leg sprang back to 81 degrees
# and the stool still stood level at its height)


def _top_shapes() -> list:
    """In the top's frame (origin at its middle): the top, the grip ribs at its ends, the hinges' ears under it, the
    latches' bosses and release knobs on its +x side (away from the arms), the treads drawn on it. The latches' bolts
    and the legs' wedges are inside the top's pockets and aren't drawn: the latch is modelled in Locks."""
    rib_y = TOP_L / 2 - GRIP_T / 2 - 1.0 * MM
    solid = [
        box((0.0, 0.0, 0.0), (TOP_D, TOP_L, TOP_T), SEAT_BLUE),
        box((0.0, rib_y, TOP_T / 2 + GRIP_H / 2), (GRIP_W, GRIP_T, GRIP_H), GRIP_GREY),
        box((0.0, -rib_y, TOP_T / 2 + GRIP_H / 2), (GRIP_W, GRIP_T, GRIP_H), GRIP_GREY),
    ]
    hz = HW.HINGE_Z * MM
    for s in (-1.0, 1.0):
        hy = s * HW.HINGE_Y * MM
        for x0, x1 in HW.EAR_X:
            xc, w = (x0 + x1) / 2 * MM, (x1 - x0) * MM
            solid.append(cyl((xc, hy, hz), KNUCKLE_R, w, SEAT_BLUE, axis="X"))
            solid.append(box((xc, hy, (hz - TOP_T / 2) / 2), (w, 2 * KNUCKLE_R, -TOP_T / 2 - hz), SEAT_BLUE))
        bx0, bx1 = HW.BOSS_X
        by = hy + s * sum(HW.BOSS_Y) / 2 * MM
        solid.append(box(((bx0 + bx1) / 2 * MM, by, 0.0), ((bx1 - bx0) * MM, (HW.BOSS_Y[1] - HW.BOSS_Y[0]) * MM, TOP_T),
                         SEAT_BLUE))
        kx0, kx1 = HW.KNOB_X
        solid.append(cyl(((kx0 + kx1) / 2 * MM, hy + s * HW.BOLT_Y * MM, HW.BOLT_AXIS_Z * MM), HW.KNOB_D / 2 * MM,
                         (kx1 - kx0) * MM, KNOB_BLACK, axis="X"))
    seen = [box((x, 0.0, TOP_T / 2 + 0.0005), (0.008, 0.15, 0.001), TREAD) for x in (-0.06, -0.03, 0.0, 0.03, 0.06)]
    return solid + seen


_TS = _top_shapes()
STOOL_TOP = body("stool_top", pos=tuple(TOP_AT), mass=TOP_MASS, friction=0.8, shapes=_TS,
                 visual_only=tuple(range(len(_TS) - 5, len(_TS))), color=SEAT_BLUE)
RIB_LOCAL = {s: np.array([0.0, s * (TOP_L / 2 - GRIP_T / 2 - 1.0 * MM), TOP_T / 2 + GRIP_H / 2]) for s in (-1, 1)}

# A leg's frame: origin on its hinge axis; folded, it lies under the top pointing to the stool's middle, along its own
# +y for the -y leg ("leg_a") and its own -y for the +y leg ("leg_b"); the leg frames start with the world's axes.
INWARD = {"leg_a": 1.0, "leg_b": -1.0}  # the folded leg's direction (y) from its hinge
SIDE = {"leg_a": -1.0, "leg_b": 1.0}  # which end of the stool


def _leg_shapes(s: float) -> list:
    """In the leg's frame; ``s`` is its inward direction (y)."""
    k0, k1 = HW.KNUCKLE_X  # past the panel, at +x, the knuckle carries the latch's wedge (not drawn)
    return [
        cyl(((k0 + k1) / 2 * MM, 0.0, 0.0), KNUCKLE_R, (k1 - k0) * MM, LEG_WHITE, axis="X"),
        box((0.0, s * (KNUCKLE_R + (LEG_L - FOOT_T - KNUCKLE_R) / 2), 1.0 * MM),
            (LEG_W, LEG_L - FOOT_T - KNUCKLE_R, LEG_T), LEG_WHITE),
        box((0.0, s * (LEG_L - FOOT_T / 2), 0.0), (FOOT_L, FOOT_T, FOOT_T), GRIP_GREY),
    ]


def hinge_local(leg: str) -> np.ndarray:
    """The leg's hinge axis point in the top's frame."""
    return np.array([0.0, SIDE[leg] * HINGE_Y, HINGE_Z - TOP_Z])


LEGS = {}
for _leg in ("leg_a", "leg_b"):
    _at = TOP_AT + hinge_local(_leg)
    LEGS[_leg] = body(_leg, pos=tuple(_at), mass=LEG_MASS, friction=1.0, shapes=_leg_shapes(INWARD[_leg]),
                      com=(0.0, INWARD[_leg] * 0.06, 0.0), color=LEG_WHITE)
    # the -y leg opens turning about -x, the +y leg about +x (a PhysX revolute angle is the child's turn about +x)
    _lo, _hi = (-math.radians(OPEN_DEG), 0.0) if _leg == "leg_a" else (0.0, math.radians(OPEN_DEG))
    fittingkit.add_hinge(Hinge("stool_top", _leg, anchor=tuple(hinge_local(_leg)), axis=(1.0, 0.0, 0.0), damping=0.002,
                               lower=_lo, upper=_hi))

OBJECTS = [STOOL_TOP, LEGS["leg_a"], LEGS["leg_b"]]


# -- the locks ------------------------------------------------------------------------------------------------------
def leg_angle_deg(top_rot: np.ndarray, leg_rot: np.ndarray, leg: str) -> float:
    """How far (degrees) the leg is open from folded: 0 folded flat under the top, 90 straight down from it, 100 at
    its stop."""
    s = INWARD[leg]
    d = top_rot.T @ leg_rot @ np.array([0.0, s, 0.0])  # the leg's direction, hinge to foot, in the top's frame
    return float(np.degrees(math.atan2(-d[2], s * d[1])))


def opening_axis(top_rot: np.ndarray, leg: str) -> np.ndarray:
    """The world axis about which the leg turns to open (right hand)."""
    return top_rot @ np.array([-INWARD[leg], 0.0, 0.0])


class Locks:
    """Runs before every physics substep (240 Hz): each leg's spring lock from the legs' and the top's poses. A leg
    short of its lock is folded back by a spring from 80 degrees on; at 98 degrees its lock catches, and from then on
    it is held open against its stop. Once the stool stands on its own (its top level and at its standing height, both
    hands off it), it takes a 40 N push down on its top."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("STOOL_LOG")
        self.trace = os.environ.get("STOOL_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["stool_top", "leg_a", "leg_b"]

    def clear(self) -> None:
        self._n = 0
        self.locked = {"leg_a": False, "leg_b": False}
        self.angle = {"leg_a": 0.0, "leg_b": 0.0}
        self.top_pos = TOP_AT.copy()
        self.top_rot = np.eye(3)
        self.hands_off = True  # updated by the scorer's 10 Hz check; offline, no hands
        self.loaded = False
        self.gave_way = {"leg_a": False, "leg_b": False}
        self.poses = {}

    def update(self, frames: dict, acc: dict) -> None:
        top = frames.get("stool_top")
        if top is None:
            return
        self._n += 1
        self.top_pos, self.top_rot = top.pos - self.origin, top.rot
        for leg in ("leg_a", "leg_b"):
            f = frames.get(leg)
            if f is None:
                continue
            opening = leg_angle_deg(top.rot, f.rot, leg)
            self.angle[leg] = opening
            if not self.locked[leg] and opening >= LATCH_DEG:
                self.locked[leg] = True
                self._note(f"{self._n / 240:7.2f}s {leg} locked at {opening:5.1f} deg")
            if self.loaded and not self.locked[leg] and not self.gave_way[leg]:
                self.gave_way[leg] = True
                self._note(f"{self._n / 240:7.2f}s {leg} gives way under the load at {opening:5.1f} deg")
            if self.locked[leg]:
                torque = HOLD_TORQUE
            elif self.gave_way[leg]:
                torque = -GIVE_WAY_TORQUE
            elif SPRING_FROM_DEG <= opening < LATCH_DEG:
                torque = -HW.fold_back_torque_nm(opening)
            else:
                continue
            axis = opening_axis(top.rot, leg)
            fittingkit._torque(acc, leg, torque * axis)
            fittingkit._torque(acc, "stool_top", -torque * axis)
        if self._n % 24 == 0 and fittingkit._ENV["env"] is not None:
            self.hands_off = hands_clear_of(self.top_pos, self.top_rot, frames, self.origin)
        was = self.loaded
        self.loaded = self.hands_off and standing(self.top_pos, self.top_rot)
        if self.loaded:
            fittingkit._wrench(acc, "stool_top", top, np.array([0.0, 0.0, -LOAD_N]), top.com)
        if self.loaded != was:
            self._note(f"{self._n / 240:7.2f}s load {'on' if self.loaded else 'off'}")
        if self.trace and self._n % 24 == 0:
            self._write_trace(top)

    def _write_trace(self, top) -> None:
        """Debug only (STOOL_TRACE=<file>): every 0.1 s, the top's pose, the legs' angles and both TCPs (mm)."""
        p = 1000 * (top.pos - self.origin)
        tilt = math.degrees(math.atan2(top.rot[2, 1], top.rot[2, 2]))
        yaw = math.degrees(math.atan2(top.rot[1, 0], top.rot[0, 0]))
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s top ({p[0]:6.1f},{p[1]:6.1f},{p[2] - 1000 * TOP:6.1f}) roll {tilt:6.1f} yaw {yaw:5.1f} | "
                     f"a {self.angle['leg_a']:6.1f} {int(self.locked['leg_a'])} b {self.angle['leg_b']:6.1f} "
                     f"{int(self.locked['leg_b'])} | off {int(self.hands_off)} load {int(self.loaded)} |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


def _leg_dir(top_rot: np.ndarray, leg: str, open_deg: float) -> np.ndarray:
    """The leg's direction (hinge to foot) in the world, open ``open_deg`` from folded on a top turned ``top_rot``."""
    s, p = INWARD[leg], math.radians(open_deg)
    return top_rot @ np.array([0.0, s * math.cos(p), -math.sin(p)])


def _standing_height() -> float:
    """The top's middle above the table with both legs at their stop, level: from the foot rails' lowest edges."""
    d = _leg_dir(np.eye(3), "leg_a", OPEN_DEG)
    foot = d * (LEG_L - FOOT_T / 2)  # the foot rail's middle from the hinge
    low = foot[2] - FOOT_T / 2 * (abs(d[2]) + abs(d[1]))  # its lowest edge
    return -low + (TOP_Z - HINGE_Z)


STAND_H = _standing_height()
LEVEL_DEG = 5.0
CLEAR = 0.02


def standing(top_pos, top_rot) -> bool:
    """The top level (within 5 degrees) at its standing height (within 1 cm), over the table."""
    up = float(np.clip(top_rot[2, 2], -1.0, 1.0))
    h = float(top_pos[2] - TOP)
    return math.degrees(math.acos(up)) < LEVEL_DEG and abs(h - STAND_H) < 0.01


def _part_boxes(top_pos, top_rot, frames, origin):
    """(centre, rotation, half sizes) of the top and of each leg's panel and foot rail, in table coordinates."""
    out = [(top_pos, top_rot, np.array([TOP_D, TOP_L, TOP_T + 2 * GRIP_H]) / 2)]
    for s in (-1.0, 1.0):  # the latches' bosses and knobs on the +x side
        c = np.array([(HW.BOSS_X[0] + HW.KNOB_X[1]) / 2, s * (HW.HINGE_Y + sum(HW.BOSS_Y) / 2), 0.0]) * MM
        half = np.array([HW.KNOB_X[1] - HW.BOSS_X[0], HW.BOSS_Y[1] - HW.BOSS_Y[0], HW.TOP_T]) * MM / 2
        out.append((top_pos + top_rot @ c, top_rot, half))
    for leg in ("leg_a", "leg_b"):
        f = frames.get(leg)
        if f is None:
            continue
        s = INWARD[leg]
        for c, size in (((0.0, s * LEG_L / 2, 0.0), (LEG_W, LEG_L, 2 * KNUCKLE_R)),
                        ((0.0, s * (LEG_L - FOOT_T / 2), 0.0), (FOOT_L, FOOT_T, FOOT_T))):
            out.append((f.pos - origin + f.rot @ np.array(c), f.rot, np.array(size) / 2))
    return out


def hands_clear_of(top_pos, top_rot, frames, origin) -> bool:
    """No gripper point within 2 cm of the top or a leg; unknown (offline) counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0:
        return True
    pts = pts - origin
    for c, r, half in _part_boxes(top_pos, top_rot, frames, origin):
        local = np.abs((pts - c) @ r) - half
        if (np.max(local, axis=1) < CLEAR).any():
            return False
    return True


MODEL = fittingkit.add_model(Locks())

# -- scoring -------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def both_locked() -> bool:
    return MODEL.locked["leg_a"] and MODEL.locked["leg_b"]


def success(objects) -> bool:
    """Both legs locked, the stool standing on its feet, level at its height under the 40 N load, both hands clear.
    Must hold for 1 s."""
    return both_locked() and MODEL.loaded and standing(MODEL.top_pos, MODEL.top_rot)


def score(objects) -> float:
    """0.9 for both legs locked and the stool standing on its own under the load (1.0 once success() has held 1 s);
    0.5 for both legs locked; 0.25 for one; 0 otherwise. A leg that only looks open earns nothing."""
    if both_locked():
        return COMPLETE_SCORE if (MODEL.loaded and standing(MODEL.top_pos, MODEL.top_rot)) else 0.5
    return 0.25 if (MODEL.locked["leg_a"] or MODEL.locked["leg_b"]) else 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# One end at a time: one hand pinches the grip rib at that end by its two ends and lifts it, the stool turning on its
# other end (and between the pads), so the leg there hangs from its hinge; the other hand, fingers closed and pointing
# down, pushes the leg's foot rail outwards where it sticks out past the top, round past straight down until the lock
# clicks; then the end is let down onto the leg. The right hand lifts the -y end and the left pushes its leg; then the
# left lifts the +y end and the right pushes.
class RotWaypoint(Waypoint):
    def __init__(self, duration, pos, finger, label="", rot=None):
        super().__init__(duration, pos, finger, label, 0.0)
        self.rot = None if rot is None else np.asarray(rot, float)


def tcp_rotation(w) -> np.ndarray:
    rot = getattr(w, "rot", None)
    return grip_rotation(w.yaw) if rot is None else rot


def _target(self, t: float):
    """yam_arena.demo.ArmPlan.target, honouring a RotWaypoint's orientation."""
    from yam_arena import ik

    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal, rot = np.asarray(w.pos, dtype=float), tcp_rotation(w)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_demo.ArmPlan.target = _target

OPEN, CLOSED = -0.04695, 0.0
PAD_TIP = 13.0 * MM  # the pads' tips past the TCP
DOWN = grip_rotation(0.0)  # pointing down, the fingers closing along y


def turned(pivot, angle: float, local) -> np.ndarray:
    """Where a point of the top (``local``, in its frame at the start) is with the stool turned ``angle`` about x
    round ``pivot`` (a world point)."""
    return np.asarray(pivot) + rot_x(angle) @ (TOP_AT + np.asarray(local, float) - np.asarray(pivot))


def foot_low_z(angle: float, pivot, leg: str, open_deg: float) -> float:
    """The lowest edge of the leg's foot rail (world z) with the stool turned ``angle`` round ``pivot`` and the leg
    open ``open_deg``."""
    h = turned(pivot, angle, hinge_local(leg))
    d = _leg_dir(rot_x(angle), leg, open_deg)
    c = h + d * (LEG_L - FOOT_T / 2)
    return float(c[2] - FOOT_T / 2 * (abs(d[1]) + abs(d[2])))


def _solve_angle(f, lo: float, hi: float) -> float:
    """The angle in [lo, hi] where f crosses zero (bisection)."""
    flo = f(lo)
    for _ in range(60):
        mid = (lo + hi) / 2
        if (f(mid) > 0) == (flo > 0):
            lo, flo = mid, f(mid)
        else:
            hi = mid
    return (lo + hi) / 2


# Stage 1: the stool turned on its +y knuckle (on the table), the -y end lifted so the -y leg hangs clear.
PIVOT_1 = TOP_AT + hinge_local("leg_b")
LIFT_1 = math.radians(-34.0)  # the top turned about x: the -y end up
# WHY 34 degrees, 31 cm out, aimed 12 mm high: lifted to 32 degrees 34 cm out, the top got to 27.6 (the arm sagged 11 mm
# and the stool slid 24 mm on its pivot) and the hanging leg's rail was 12 mm lower than planned, so the push went over
# it (run fs1); higher than 34 degrees is out of the right arm's reach
LIFT_SAG = 0.012  # the lifting hand aimed this far higher while it holds the end up (run fs1: 11 mm low) [measured]
# Stage 1 ends with the -y end let down until the -y leg's foot (locked, 100 degrees) is on the table.
DOWN_1 = _solve_angle(lambda a: foot_low_z(a, PIVOT_1, "leg_a", OPEN_DEG) - TOP, LIFT_1, 0.0)
# Stage 2: turned on the -y foot rail, the +y end lifted until the +y leg hangs clear.
PIVOT_2 = None  # set below from DOWN_1
LIFT_2 = math.radians(4.0)


def _foot_centre(angle, pivot, leg, open_deg) -> np.ndarray:
    h = turned(pivot, angle, hinge_local(leg))
    return h + _leg_dir(rot_x(angle), leg, open_deg) * (LEG_L - FOOT_T / 2)


PIVOT_2 = _foot_centre(DOWN_1, PIVOT_1, "leg_a", OPEN_DEG)


def turned2(angle: float, local) -> np.ndarray:
    """A point of the top with the stool turned to ``angle`` (about x) round the -y foot rail, from DOWN_1."""
    p_down = turned(PIVOT_1, DOWN_1, local)
    return PIVOT_2 + rot_x(angle - DOWN_1) @ (p_down - PIVOT_2)


LAND_2 = _solve_angle(lambda a: float(turned2(a, hinge_local("leg_b"))[2] + (_leg_dir(rot_x(a), "leg_b", OPEN_DEG)
                      * (LEG_L - FOOT_T / 2))[2] - FOOT_T / 2 - TOP), DOWN_1, LIFT_2)

GRIP_DEPTH = 8.0 * MM  # the pads' tips this far above the top's face, either side of the rib
PUSH_LOCAL_X = -(TOP_D / 2 + 0.06)  # the TCP over the foot rail where it sticks out on the arms' side
# WHY 6 cm out from the top's edge: the closed fingers reach 3.4 cm either side of the TCP along x; pushing 2 cm out the
# hand ran 9 mm into the tilted top's edge (plan_check), and 4.8 cm out it brushed the edge's near corner, the stool
# slewed 5 degrees (runs fs5 to fs7)
FINGER_HALF = 10.0 * MM  # half the closed fingers' thickness, 1 to 4 cm above their tips (URDF hulls)
FACE_BELOW = 20.0 * MM  # the fingertips this far below the rail's middle, so the fingers' side meets the rail
# WHY below: with the tips level with the rail, the rail turned 15 degrees and then the tips rose over it (run fs3: it
# met the rail 10 mm further out than aimed, the stool slewed as it slid); the closed fingers' side is 4 cm tall
PUSH_PAST_DEG = 20.0  # the push aimed this far past the stop (fs4: aimed 12 past, the leg got to 86 of 98)


# WHY pinched along x, the hand kept pointing straight down: with the hand turning with the stool as it lifts, the arm
# on that side misses the lifted pose by about 70 mm (and the other arm misses it low down by about 20 mm); pinched
# across the rib's ends the stool turns between the pads, so the hand need not turn, and the arm reaches every pose.
LIFT_GRIP = grip_rotation(math.pi / 2)  # pointing down, the fingers closing along x (the right hand, at the -y end)
LIFT_GRIP_LEFT = grip_rotation(-math.pi / 2)  # the same, turned half a turn (the left hand, at the +y end)
PINCH_UP = 14.0 * MM  # the pads' middle on the rib, above the top's face


def grip_pose(side: float, angle: float, stage: int, lift: float = 0.0):
    """The TCP pinching the rib at the ``side`` end with the stool turned ``angle`` (stage 1 or 2), ``lift`` above."""
    pinch_local = RIB_LOCAL[int(side)] + np.array([0.0, 0.0, -GRIP_H / 2 + PINCH_UP])
    pinch = turned(PIVOT_1, angle, pinch_local) if stage == 1 else turned2(angle, pinch_local)
    rot = LIFT_GRIP if side < 0 else LIFT_GRIP_LEFT
    return pinch + np.array([0.0, 0.0, GRIP_DEPTH - PINCH_UP + PAD_TIP + lift]), rot


# Where the stool really is once an end is lifted, for aiming the push: lifting the -y end as planned, the top turned
# about 34 degrees and the stool slid about 25 mm along y on its other end and slewed 6 degrees about the vertical (runs
# fs1 to fs7 all slid). Aimed at the planned pose the push missed the rail or rode over it (fs2, fs3). [measured: the
# pose in fs3 and fs4 once lifted, before the pushing hand came near]
LIFTED_1 = (np.array([0.3167, 0.0276, TOP + 0.0807]),
            axis_angle_matrix(Z_AXIS, math.radians(4.9)) @ rot_x(math.radians(-32.9)))


def push_pose(leg: str, top, world_from_plumb_deg: float, back: float = 0.0, up: float = 0.0):
    """The pushing TCP (pointing down, fingers closed), its fingertips against the inner face of the leg's foot rail
    where it sticks out on the arms' side, the stool's top at ``top`` (position, rotation) and the leg turned
    ``world_from_plumb_deg`` outwards from hanging straight down; ``back`` towards the stool's middle, ``up`` above."""
    h = top[0] + top[1] @ (hinge_local(leg) + np.array([PUSH_LOCAL_X, 0.0, 0.0]))
    out = SIDE[leg]  # outwards along y
    a = math.radians(world_from_plumb_deg)
    rail = h + np.array([0.0, out * math.sin(a), -math.cos(a)]) * (LEG_L - FOOT_T / 2)
    half = FOOT_T / 2 * (abs(math.cos(a)) + abs(math.sin(a)))  # the turned rail's half width, across y
    tip = rail + np.array([0.0, -out * (half + FINGER_HALF + back), up - FACE_BELOW])
    return tip - DOWN[:, 2] * PAD_TIP, DOWN


def roll_of(rot: np.ndarray) -> float:
    """The top's turn about x (rad)."""
    return math.atan2(rot[2, 1], rot[2, 2])


LIFTED_2 = None  # set in demo() (planned until measured)


def plumb_from_folded(angle: float, leg: str) -> float:
    """The leg's opening (degrees from folded) when it hangs straight down from a top turned ``angle``."""
    return 90.0 - SIDE[leg] * math.degrees(angle)


def top_pose(angle: float, stage: int) -> tuple[np.ndarray, np.ndarray]:
    """The top's middle and rotation with the stool turned ``angle`` in stage 1 or 2 (stage 3: lifted in stage 1, where
    the sim puts it, ``LIFTED_1``)."""
    if stage == 3:
        return LIFTED_1
    p = turned(PIVOT_1, angle, np.zeros(3)) if stage == 1 else turned2(angle, np.zeros(3))
    return p, rot_x(angle)


def leg_pose(angle: float, stage: int, leg: str, open_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """The leg's hinge point and rotation, open ``open_deg`` on a top turned ``angle``."""
    p, r = top_pose(angle, stage)
    turn = rot_x(-math.radians(open_deg)) if leg == "leg_a" else rot_x(math.radians(open_deg))
    return p + r @ hinge_local(leg), r @ turn


def resting_angle(angle: float, stage: int, leg: str, cap_deg: float = 90.0) -> float:
    """Where a free leg rests (degrees from folded) on a top turned ``angle``: hanging straight down (or at
    ``cap_deg`` if less, held by its spring), or, if that would put its foot through the table, on the table."""
    want = min(plumb_from_folded(angle, leg), cap_deg)

    def low(opening):
        h, r = leg_pose(angle, stage, leg, opening)
        d = r @ np.array([0.0, INWARD[leg], 0.0])
        return float((h + d * (LEG_L - FOOT_T / 2))[2] - FOOT_T / 2 * (abs(d[1]) + abs(d[2])) - TOP)

    if low(want) >= 0.0:
        return want
    lo = 0.0
    if low(lo) < 0.0:
        return 0.0
    return math.degrees(_solve_angle(lambda a: low(math.degrees(a)), 0.0, math.radians(want)))


PLAN: list = []  # the stool's expected state (stage, angle, leg_a and leg_b openings) at the end of each waypoint


def demo():
    left, right = [], []
    PLAN.clear()

    def both(dur, lpose, lf, rpose, rf, label, state):
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        PLAN.append(state)

    def lying(angle):  # stage 1: the +y leg stays flat on the table as the top turns on its hinge
        return -math.degrees(angle)

    up8 = np.array([0.0, 0.0, 0.08])
    # stage 1: the right hand lifts the -y end; the left pushes the -y leg
    flat = (1, 0.0, 0.0, 0.0)
    r_grip = grip_pose(-1, 0.0, 1)
    l_wait = (np.array([0.16, 0.10, TOP + 0.16]), DOWN)  # out on the arms' side of the stool
    both(2.5, l_wait, CLOSED, (r_grip[0] + up8, r_grip[1]), OPEN, "right over the -y grip", flat)
    both(1.2, l_wait, CLOSED, r_grip, OPEN, "right down to it", flat)
    both(2.0, l_wait, CLOSED, r_grip, CLOSED, "right grips", flat)
    for k, frac in enumerate((0.35, 0.7, 1.0)):
        a = LIFT_1 * frac
        both(1.0, l_wait, CLOSED, grip_pose(-1, a, 1, lift=LIFT_SAG * frac), CLOSED, f"lift the -y end {k + 1}",
             (1, a, resting_angle(a, 1, "leg_a"), lying(a)))
    held = grip_pose(-1, LIFT_1, 1, lift=LIFT_SAG)
    hang = plumb_from_folded(roll_of(LIFTED_1[1]), "leg_a")
    full = OPEN_DEG - hang + PUSH_PAST_DEG  # the turn from hanging straight down
    hung = (3, LIFT_1, hang, lying(LIFT_1))
    both(1.5, push_pose("leg_a", LIFTED_1, 0.0, back=0.04, up=0.04), CLOSED, held, CLOSED, "left over the -y foot", hung)
    both(1.0, push_pose("leg_a", LIFTED_1, 0.0, back=0.04), CLOSED, held, CLOSED, "left down beside it", hung)
    both(0.8, push_pose("leg_a", LIFTED_1, 0.0), CLOSED, held, CLOSED, "left to the rail", hung)
    for k, frac in enumerate((0.33, 0.66, 1.0)):
        both(1.0, push_pose("leg_a", LIFTED_1, full * frac), CLOSED, held, CLOSED, f"push the -y leg out {k + 1}",
             (3, LIFT_1, min(OPEN_DEG, hang + full * frac), lying(LIFT_1)))
    clicked = (3, LIFT_1, OPEN_DEG, lying(LIFT_1))
    both(0.6, push_pose("leg_a", LIFTED_1, full), CLOSED, held, CLOSED, "click", clicked)
    l_back = (push_pose("leg_a", LIFTED_1, full)[0] + np.array([-0.03, 0.03, 0.04]), DOWN)  # back, within reach
    both(1.0, l_back, CLOSED, held, CLOSED, "left back", clicked)
    for k, frac in enumerate((0.5, 1.0)):
        a = LIFT_1 + (DOWN_1 - LIFT_1) * frac
        both(1.2, l_back, CLOSED, grip_pose(-1, a, 1, lift=LIFT_SAG * (1.0 - frac)), CLOSED, f"let the -y end down {k + 1}",
             (1, a, OPEN_DEG, lying(a)))
    on_a = (2, DOWN_1, OPEN_DEG, lying(DOWN_1))
    both(2.0, l_back, CLOSED, grip_pose(-1, DOWN_1, 1), OPEN, "right lets go", on_a)
    r_away = (np.array([0.15, -0.18, TOP + 0.15]), DOWN)  # out on the arms' side, clear of the top's edge
    # stage 2: the left hand lifts the +y end; the right pushes the +y leg
    l_grip = grip_pose(1, DOWN_1, 2)
    l_high = (l_back[0] + np.array([0.03, 0.06, 0.03]), DOWN)
    # WHY up first: going straight across, the left hand swept the stool 15 cm along the table (run fs1)
    r_up1 = (grip_pose(-1, DOWN_1, 1, lift=0.05)[0] + np.array([0.0, -0.04, 0.0]), LIFT_GRIP)
    # WHY the right hand rises first: crossing to its waiting place while the left rose, the hands' hulls met (plan_check)
    both(1.2, l_high, CLOSED, r_up1, OPEN, "left up", on_a)
    # WHY the right hand waits up there until the left is at the +y end: crossing at the same time the arms met (run fs10:
    # the left arm was knocked off its path and threw the stool aside)
    both(2.0, (l_grip[0] + up8, l_grip[1]), OPEN, r_up1, OPEN, "left over the +y grip", on_a)
    both(1.5, l_grip, OPEN, r_away, OPEN, "left down to it", on_a)
    both(2.0, l_grip, CLOSED, r_away, CLOSED, "left grips", on_a)
    hang2 = 80.0  # the +y leg hangs short of straight down: its spring holds it at about 80 degrees [estimate]
    for k, frac in enumerate((0.5, 1.0)):
        a = DOWN_1 + (LIFT_2 - DOWN_1) * frac
        both(1.0, grip_pose(1, a, 2, lift=LIFT_SAG * frac), CLOSED, r_away, CLOSED, f"lift the +y end {k + 1}",
             (2, a, OPEN_DEG, resting_angle(a, 2, "leg_b", hang2)))
    held2 = grip_pose(1, LIFT_2, 2, lift=LIFT_SAG)
    global LIFTED_2
    if LIFTED_2 is None:
        LIFTED_2 = top_pose(LIFT_2, 2)
    start2 = resting_angle(LIFT_2, 2, "leg_b", hang2)
    base2 = start2 - plumb_from_folded(LIFT_2, "leg_b")  # where it starts, from hanging straight down (inwards)
    full2 = OPEN_DEG - start2 + PUSH_PAST_DEG
    hung2 = (2, LIFT_2, OPEN_DEG, start2)
    both(1.5, held2, CLOSED, push_pose("leg_b", LIFTED_2, base2, back=0.04, up=0.04), CLOSED, "right over the +y foot",
         hung2)
    both(1.0, held2, CLOSED, push_pose("leg_b", LIFTED_2, base2, back=0.04), CLOSED, "right down beside it", hung2)
    both(0.8, held2, CLOSED, push_pose("leg_b", LIFTED_2, base2), CLOSED, "right to the rail", hung2)
    for k, frac in enumerate((0.5, 1.0)):
        both(1.0, held2, CLOSED, push_pose("leg_b", LIFTED_2, base2 + full2 * frac), CLOSED,
             f"push the +y leg out {k + 1}", (2, LIFT_2, OPEN_DEG, min(OPEN_DEG, start2 + full2 * frac)))
    clicked2 = (2, LIFT_2, OPEN_DEG, OPEN_DEG)
    both(0.6, held2, CLOSED, push_pose("leg_b", LIFTED_2, base2 + full2), CLOSED, "click 2", clicked2)
    r_back = push_pose("leg_b", LIFTED_2, base2 + full2, back=0.03, up=0.06)
    both(1.0, held2, CLOSED, r_back, CLOSED, "right back", clicked2)
    landed = (2, LAND_2, OPEN_DEG, OPEN_DEG)
    both(1.2, grip_pose(1, LAND_2, 2), CLOSED, r_back, CLOSED, "let the +y end down", landed)
    both(2.0, grip_pose(1, LAND_2, 2), OPEN, r_back, CLOSED, "left lets go", landed)
    l_away = (grip_pose(1, LAND_2, 2, lift=0.04)[0] + np.array([0.0, 0.03, 0.0]), LIFT_GRIP_LEFT)
    r_up = (r_back[0] + np.array([-0.04, 0.0, 0.05]), DOWN)  # back towards the arms: clear of the top by more than 2 cm (fs6)
    both(1.5, l_away, OPEN, r_up, CLOSED, "hands clear", landed)
    both(2.0, l_away, OPEN, r_up, CLOSED, "wait", landed)
    return {"left": left, "right": right}
