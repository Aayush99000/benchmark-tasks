"""On the station table: a 5-inch random orbital sander lying upside down, its hook-and-loop backing pad facing up, and
a new sanding disc standing on edge in a slotted disc stand beside it. Fit the disc to the pad: line the disc's 8 dust holes
up with the pad's 8 holes, put it on, press it on firmly, and let go of it.

The pad's face is a model of hook-and-loop: a disc that comes down flat on it, its fuzzy (loop) side down, is caught
by the hooks where it lands and can't slide or turn on the pad any more; pressed on, it is held firmly. So the holes
have to be lined up before the disc goes on: a disc put on turned or off centre blocks the dust holes.

Run it:  inspect-robots run --embodiment yam_arena -E task=sanding_disc.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import body, box, cyl, prism12  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

HOUSING = (0.16, 0.42, 0.33)
RUBBER = (0.10, 0.10, 0.11)
HOOK_GREY = (0.30, 0.31, 0.33)
HOLE_BLACK = (0.02, 0.02, 0.02)
GRIT = (0.66, 0.24, 0.12)
FLEECE = (0.88, 0.87, 0.84)
STAND = (0.55, 0.40, 0.25)
WHITE = (0.92, 0.92, 0.90)


def rot_z(angle: float) -> np.ndarray:
    return axis_angle_matrix(Z_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the hole pattern ---------------------------------------------------------------------------------------------------
# A 5-inch (125 mm) 8-hole pad and disc: 8 dust holes 45 degrees apart.
# [estimate: holes 10 mm across on a circle 78 mm across; the makers don't publish the pattern's sizes]
HOLES = 8
HOLE_R = 5.0 * MM  # a hole's radius
HOLE_CIRCLE = 39.0 * MM  # the holes' centres from the middle
HOLE_PITCH = 2 * math.pi / HOLES


def hole_centres(angle: float = 0.0) -> np.ndarray:
    """The 8 holes' centres (x, y), from the middle, the first ``angle`` round from x."""
    a = angle + HOLE_PITCH * np.arange(HOLES)
    return HOLE_CIRCLE * np.column_stack([np.cos(a), np.sin(a)])


# -- the sander (fixed): upside down, the pad facing up ------------------------------------------------------------------
# [estimate: a compact 5-inch random orbital sander, 12 cm tall from its top cap to the pad's face; its dust port at the
# back, here pointing away from the arms (+x)]
PAD = np.array([0.28, 0.0])  # the pad's middle, on the table
FACE_H = 0.120  # the pad's face, above the table
PAD_AT = np.array([PAD[0], PAD[1], TOP + FACE_H])
PAD_R = 62.5 * MM  # 125 mm across
PAD_IN = 61.5 * MM  # its collision shape: a 12-sided prism this far to its flats (63.7 mm to its corners)
PAD_T = 20.0 * MM  # the foam pad
# WHY turned: square to the table, the holes would line up with a disc picked up square; turned 20 degrees, the holes
# have to be looked at and the disc turned to them
PAD_TURN = math.radians(20.0)
HOUSING_IN = 57.0 * MM


def _sander_shapes() -> tuple[list, list]:
    """In the sander's frame (origin on the table under the pad's middle; the world's axes)."""
    body_top = FACE_H - PAD_T
    solid = [
        *prism12((0.0, 0.0, body_top / 2), HOUSING_IN, body_top, HOUSING),  # the motor housing and top cap
        *prism12((0.0, 0.0, FACE_H - PAD_T / 2), PAD_IN, PAD_T, HOOK_GREY),  # the backing pad
        cyl((0.090, 0.0, body_top - 0.022), 17.0 * MM, 0.070, RUBBER, axis="X"),  # the dust port, at the back
    ]
    seen = [cyl((x, y, FACE_H + 0.0002), HOLE_R, 0.0004, HOLE_BLACK) for x, y in hole_centres(PAD_TURN)]
    seen += [
        cyl((0.0, 0.0, FACE_H - PAD_T - 0.004), HOUSING_IN + 3.0 * MM, 0.006, RUBBER),  # the shroud's rubber skirt
        box((-0.030, 0.050, 0.060), (0.024, 0.012, 0.012), RUBBER),  # the switch
        cyl((-0.040, -0.052, 0.035), 6.0 * MM, 0.040, RUBBER, axis="Y"),  # the cord's boot
        box((0.0, -HOUSING_IN - 0.0005, 0.050), (0.040, 0.001, 0.020), WHITE),  # the label
    ]
    return solid, seen


_SS, _SV = _sander_shapes()
SANDER = body("sander", pos=(PAD[0], PAD[1], TOP), friction=1.2, shapes=_SS + _SV,
              visual_only=tuple(range(len(_SS), len(_SS) + len(_SV))))

# -- the disc stand and the disc ----------------------------------------------------------------------------------------
# A wooden stand that keeps discs on edge: a block with a slot across it. The disc stands in the slot, its grit side
# facing away from the arms (+x), its fuzzy side towards them.
STAND_AT = np.array([0.20, -0.14, TOP])
STAND_SIZE = (0.050, 0.090, 0.025)  # x, y, z
SLOT_W, SLOT_FLOOR = 4.0 * MM, 0.010  # the slot: 4 mm wide, its floor 1 cm up (15 mm deep)
_wall = (STAND_SIZE[0] - SLOT_W) / 2
_wall_h = STAND_SIZE[2] - SLOT_FLOOR
STAND_BODY = body("stand", pos=tuple(STAND_AT), friction=0.6, shapes=[
    box((0.0, 0.0, SLOT_FLOOR / 2), (STAND_SIZE[0], STAND_SIZE[1], SLOT_FLOOR), STAND),
    box((-(SLOT_W + _wall) / 2, 0.0, SLOT_FLOOR + _wall_h / 2), (_wall, STAND_SIZE[1], _wall_h), STAND),
    box(((SLOT_W + _wall) / 2, 0.0, SLOT_FLOOR + _wall_h / 2), (_wall, STAND_SIZE[1], _wall_h), STAND),
])

DISC_R = PAD_R
DISC_IN = PAD_IN
# WHY 2 mm and 40 g: a real disc is about 1 mm thick and 6 g; between the YAM's fingers a part much under 30 g is
# squeezed through (our garlic-press runs), and the fingers hold a 2 mm plate with 5 N each (5,000 N/m, 1 mm short of
# closed)
DISC_T = 2.0 * MM
DISC_MASS = 0.040
# The disc's frame: z its grit side's normal, x along one of its holes. Standing in the stand: z to +x (the grit facing
# away from the arms), x straight up, so one hole is at its top; a flat of its 12 sides on the slot's floor.
R_DISC_START = np.column_stack([Z_AXIS, -Y_AXIS, X_AXIS])
DISC_AT = STAND_AT + np.array([0.0, 0.0, SLOT_FLOOR + DISC_IN])
DISC_TOP = DISC_AT[2] + DISC_IN  # its top edge


def _disc_shapes() -> tuple[list, list]:
    solid = prism12((0.0, 0.0, 0.0), DISC_IN, DISC_T, GRIT)
    seen = [cyl((x, y, DISC_T / 2 + 0.0002), HOLE_R, 0.0004, HOLE_BLACK) for x, y in hole_centres(0.0)]
    seen.append(cyl((0.0, 0.0, -DISC_T / 2 - 0.0002), DISC_IN, 0.0004, FLEECE))  # the fuzzy side
    seen.append(box((0.0, 0.0, DISC_T / 2 + 0.0002), (0.006, 0.018, 0.0004), WHITE))  # the grit number
    return solid, seen


_DS, _DV = _disc_shapes()
DISC = body("disc", pos=tuple(DISC_AT), mass=DISC_MASS, friction=1.0, shapes=_DS + _DV,
            visual_only=tuple(range(len(_DS), len(_DS) + len(_DV))), quat=quat_xyzw(R_DISC_START))

OBJECTS = [SANDER, STAND_BODY, DISC]

# -- hook and loop -------------------------------------------------------------------------------------------------------
STICK_GAP = 1.0 * MM  # the disc's lowest point this close to the pad's face,
STICK_DEG = 4.0  # tilted under this,
STICK_OFF = 20.0 * MM  # and its middle within this of the pad's, it is caught
PRESS_GAP = 5.0 * MM  # a fingertip within this above the disc's face, over the pad, is pressing it
PRESS_S = 0.3  # pressed on: pressed this long (s) after it is caught
ALIGN_MM = 2.5  # every disc hole within this of a pad hole: each hole at least two-thirds open
CLEAR = 0.02  # both grippers this far from the disc at the end
TRACE_EVERY = 24  # substeps (0.1 s)


def disc_yaw(rot: np.ndarray) -> float:
    return math.atan2(rot[1, 0], rot[0, 0])


def disc_lie(f, origin=np.zeros(3)) -> tuple[np.ndarray, float, float, bool]:
    """(the disc's middle from the pad's face middle, its tilt (deg), its lowest point above the pad's face, grit side
    up)."""
    rel = f.pos - origin - PAD_AT
    up = f.rot[:, 2]
    tilt = math.degrees(math.acos(float(np.clip(abs(up[2]), -1.0, 1.0))))
    t = math.radians(tilt)
    lowest = rel[2] - (DISC_T / 2) * math.cos(t) - DISC_R * math.sin(t)
    return rel, tilt, float(lowest), bool(up[2] > 0.0)


def hole_error_mm(centre_xy, yaw: float) -> float:
    """How far (mm) the disc's worst hole is from the nearest pad hole, the disc's middle at ``centre_xy`` from the
    pad's and turned ``yaw``."""
    disc = hole_centres(yaw) + np.asarray(centre_xy, float)[:2]
    pad = hole_centres(PAD_TURN)
    d = np.linalg.norm(disc[:, None, :] - pad[None, :, :], axis=2)
    return float(1000 * d.min(axis=1).max())


def hole_open_fraction(error_mm: float) -> float:
    """The share of a 10 mm hole left open, the disc's hole ``error_mm`` off the pad's."""
    r, d = 1000 * HOLE_R, error_mm
    if d >= 2 * r:
        return 0.0
    lens = 2 * r * r * math.acos(d / (2 * r)) - (d / 2) * math.sqrt(4 * r * r - d * d)
    return lens / (math.pi * r * r)


class HookAndLoop:
    """Runs before every physics substep (240 Hz): the pad's hook face, from the disc's pose and the fingers.

    Until it is caught the disc is a free body. When it comes down flat on the pad (its lowest point within 1 mm of the
    face, tilted under 4 degrees, its middle within 20 mm of the pad's), fuzzy side down, the hooks catch it: from then
    on its pose is written back every substep, on the pad's face where it landed, turned as it landed. A fingertip held
    on it over the pad for 0.3 s in all presses it on."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.dt = 1.0 / 240.0
        self.log = os.environ.get("DISC_LOG")
        self.trace = os.environ.get("DISC_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["disc"]

    def clear(self) -> None:
        self._n = 0
        self.caught = False
        self.caught_at = None  # (x, y) from the pad's middle, and yaw
        self.pressed_s = 0.0
        self.frame = None
        self.poses = {}

    @property
    def pressed(self) -> bool:
        return self.caught and self.pressed_s >= PRESS_S

    def update(self, frames: dict, acc: dict) -> None:
        f = frames.get("disc")
        if f is None:
            return
        self._n += 1
        self.frame = f
        if not self.caught:
            rel, tilt, lowest, grit_up = disc_lie(f, self.origin)
            if grit_up and tilt < STICK_DEG and lowest < STICK_GAP and math.hypot(rel[0], rel[1]) < STICK_OFF:
                self.caught = True
                self.caught_at = (float(rel[0]), float(rel[1]), disc_yaw(f.rot))
                err = hole_error_mm(rel[:2], self.caught_at[2])
                self._note(f"{self._n / 240:7.2f}s caught: middle ({1000 * rel[0]:5.1f},{1000 * rel[1]:5.1f}) mm off, "
                           f"turned {math.degrees(self.caught_at[2]):6.2f} deg, tilt {tilt:4.1f} deg, holes off "
                           f"{err:4.2f} mm")
        if self.caught:
            x, y, yaw = self.caught_at
            self.poses = {"disc": (PAD_AT + self.origin + np.array([x, y, DISC_T / 2]), quat_xyzw(rot_z(yaw)))}
        if self._n % TRACE_EVERY == 0:
            if self.caught and self.pressed_s < PRESS_S and self._pressing():
                self.pressed_s += TRACE_EVERY * self.dt
                if self.pressed_s >= PRESS_S:
                    self._note(f"{self._n / 240:7.2f}s pressed on")
            if self.trace:
                self._write_trace(f)

    def _pressing(self) -> bool:
        pts = fittingkit.finger_points()
        if len(pts) == 0:
            return False
        return pressing(pts - self.origin)

    def _write_trace(self, f) -> None:
        """Debug only (DISC_TRACE=<file>): every 0.1 s, the disc and both TCPs (mm, from the table)."""
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        p = 1000 * (f.pos - self.origin)
        rel, tilt, lowest, _ = disc_lie(f, self.origin)
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s disc ({p[0]:6.1f},{p[1]:6.1f},{p[2] - 1000 * TOP:6.1f}) yaw "
                     f"{math.degrees(disc_yaw(f.rot)):6.1f} tilt {tilt:5.1f} low {1000 * lowest:6.1f} c{int(self.caught)} "
                     f"p{self.pressed_s:3.1f} |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


def pressing(points: np.ndarray, caught_at=None) -> bool:
    """Any of ``points`` (from the table's origin, world) within 5 mm above the caught disc's face, over the pad."""
    x, y, _ = MODEL.caught_at if caught_at is None else caught_at
    face = PAD_AT + np.array([x, y, DISC_T])
    rel = points - face
    over = np.hypot(rel[:, 0], rel[:, 1]) < DISC_R - 5.0 * MM
    return bool(np.any(over & (rel[:, 2] > -1.0 * MM) & (rel[:, 2] < PRESS_GAP)))


MODEL = fittingkit.add_model(HookAndLoop())

# -- scoring -------------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def holes_lined_up() -> bool:
    if not MODEL.caught:
        return False
    x, y, yaw = MODEL.caught_at
    return hole_error_mm((x, y), yaw) <= ALIGN_MM


def on_the_pad() -> bool:
    """The disc lying on the pad, caught or not (tilted, or upside down): its middle within 40 mm of the pad's axis and
    under 3 cm above its face, still."""
    if MODEL.caught:
        return True
    f = MODEL.frame
    if f is None:
        return False
    rel = f.pos - MODEL.origin - PAD_AT
    return bool(math.hypot(rel[0], rel[1]) < 0.040 and 0.0 < rel[2] < 0.030 and np.linalg.norm(f.vel) < 0.02)


def hands_clear() -> bool:
    """No gripper point within 2 cm of the disc; unknown offline counts as clear."""
    pts = fittingkit.finger_points()
    f = MODEL.frame
    if len(pts) == 0 or f is None:
        return True
    rel = (pts - f.pos) @ f.rot
    radial = np.maximum(np.hypot(rel[:, 0], rel[:, 1]) - DISC_R, 0.0)
    axial = np.maximum(np.abs(rel[:, 2]) - DISC_T / 2, 0.0)
    return bool(np.hypot(radial, axial).min() > CLEAR)


def success(objects) -> bool:
    """The disc caught on the pad with every hole lined up, pressed on, and both grippers clear of it. Must hold 1 s."""
    return MODEL.pressed and holes_lined_up() and hands_clear()


def score(objects) -> float:
    """0.9 for the disc on the pad, holes lined up, pressed on (1.0 once success() has held 1 s, the hands clear); 0.5
    for the disc caught on the pad otherwise (holes out of line, or not pressed on); 0.25 for the disc lying on the pad
    but not caught (tilted, or fuzzy side up); 0 otherwise."""
    if MODEL.pressed and holes_lined_up():
        return COMPLETE_SCORE
    if MODEL.caught:
        return 0.5
    return 0.25 if on_the_pad() else 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# The right hand comes down over the disc's top edge, pointing down, its fingers closing across the disc's faces, and
# pinches it 16 mm in from the edge; it lifts it out of the stand and tips it forward to level (the hand pointing away
# from the arms, the grit side coming up), turns it 20 degrees to the pad's holes and brings it over the pad; it tips it
# 10 degrees down at its far side and touches that edge on the pad's far edge. The left hand, fingers closed and
# pointing down, holds the far edge there while the right lets go and backs away, and the disc drops flat and is
# caught; the left hand then presses it on in the middle, and both hands move clear.
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
PAD_TIP = 13.0 * MM  # the closed fingertips this far past the TCP
TIP_IN = 16.0 * MM  # the right fingertips this far in from the disc's edge (the stand's walls 13 mm lower)


def r_level(turn: float, tip_deg: float = 0.0) -> np.ndarray:
    """The right TCP's orientation level, pointing along x turned ``turn`` about the vertical, its fingers closing up
    and down (the TCP's x up), tipped ``tip_deg`` down."""
    a = rot_z(turn) @ X_AXIS
    r = np.column_stack([Z_AXIS, np.cross(a, Z_AXIS), a])
    if tip_deg:
        r = axis_angle_matrix(np.cross(Z_AXIS, a), math.radians(tip_deg)) @ r
    return r


# Picking it up: pointing straight down, the fingers closing along x across the disc's faces; tipped forward 90 degrees
# about y, that is the level hand pointing along x, with the disc's grit side up.
R_PICK = axis_angle_matrix(Y_AXIS, math.pi / 2) @ r_level(0.0)
PICK_TCP = np.array([DISC_AT[0], DISC_AT[1], DISC_TOP - TIP_IN + PAD_TIP])
# the disc in the hand: its pose in the right TCP's frame, as the fingers close on it
# WHY 0.7 mm further out: held level in run sand1, the disc's middle was 59.2 mm ahead of the TCP, not 58.5
HELD_OUT = 0.7 * MM
_R_TD = R_PICK.T @ R_DISC_START
_P_TD = R_PICK.T @ (DISC_AT - PICK_TCP) + np.array([0.0, 0.0, HELD_OUT])
# the disc's holes as it lies level in the hand, unturned: from R_DISC_START tipped forward with the hand
DISC_TURN = math.atan2(*(r_level(0.0) @ _R_TD)[1::-1, 0])
TURN = PAD_TURN - DISC_TURN  # the hand turned this far (anticlockwise from above) to put the disc's holes on the pad's
TURN = (TURN + HOLE_PITCH / 2) % HOLE_PITCH - HOLE_PITCH / 2  # the least turn: the holes repeat every 45 degrees
# WHY 0.8 degrees more: turned 20 degrees by the plan, the disc in the hand turned 19.2 (run sand1: caught at 0.77
# degrees short of the pad's holes, 1.59 mm off at the holes)
TURN_LAG = math.radians(0.8)
TIP_DEG = 10.0  # the disc tipped this far down at its far side, touching on
TOUCH_PAST = 1.0 * MM  # its far edge aimed this far under the pad's face


def disc_held(tcp, r) -> tuple[np.ndarray, np.ndarray]:
    """The disc's middle and rotation held by the right TCP at ``tcp``, ``r``."""
    return np.asarray(tcp, float) + r @ _P_TD, r @ _R_TD


def tcp_for_disc(centre, r) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP's pose ``r`` holding the disc with its middle at ``centre``."""
    return np.asarray(centre, float) - r @ _P_TD, r


def touch_centre(tip_deg: float = TIP_DEG, past: float = TOUCH_PAST) -> np.ndarray:
    """The disc's middle with its far edge (along the turned hand) on the pad's far edge, tipped ``tip_deg`` down."""
    a = rot_z(TURN) @ X_AXIS
    t = math.radians(tip_deg)
    far = PAD_AT + DISC_R * a - np.array([0.0, 0.0, past])  # the far edge's lowest corner
    return far - DISC_R * math.cos(t) * a + np.array([0.0, 0.0, DISC_R * math.sin(t) + (DISC_T / 2) * math.cos(t)])


# The left hand: closed fingertips pointing straight down.
R_PRESS = grip_rotation(0.0)
HOLD_IN = 25.0 * MM  # holding the far edge on: the fingertips this far in from the disc's far edge
HOLD_PAST = 4.0 * MM  # and aimed this far under its face [estimate: the arm's compliance]
PRESS_PAST = 6.0 * MM  # pressing it on: the fingertips aimed this far under its face


def press_pose(point, depth: float, up: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose with its closed fingertips ``depth`` under ``point`` (or ``up`` above it)."""
    tip = np.asarray(point, float) - np.array([0.0, 0.0, depth - up])
    return tip + np.array([0.0, 0.0, PAD_TIP]), R_PRESS


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    up = lambda h: np.array([0.0, 0.0, h])  # noqa: E731
    a1 = rot_z(TURN) @ X_AXIS
    far_hold = PAD_AT + (DISC_R - HOLD_IN) * a1 + up(DISC_T + HOLD_IN * math.sin(math.radians(TIP_DEG)))
    middle = PAD_AT + up(DISC_T)
    # WHY to the side: straight over the far edge 8 cm up is out of the left arm's reach (45 mm off)
    l_wait = (np.array([far_hold[0], 0.13, TOP + 0.16]), R_PRESS)
    both(2.5, (PICK_TCP + up(0.04), R_PICK), OPEN, l_wait, CLOSED, "over the disc's top edge")
    both(1.2, (PICK_TCP, R_PICK), OPEN, l_wait, CLOSED, "fingers either side of it")
    both(2.0, (PICK_TCP, R_PICK), CLOSED, l_wait, CLOSED, "pinch it")
    lift = PICK_TCP + up(0.02)
    both(1.5, (lift, R_PICK), CLOSED, l_wait, CLOSED, "lift it out of the stand")
    # WHY tipped level in two halves, rising: in one go the slerp's midpoint path is the same, but the disc's lower edge
    # sweeps 9 cm over the table at the half way pose, clear of the stand
    from yam_arena.ik import slerp_matrix
    both(1.5, (lift + up(0.02), slerp_matrix(R_PICK, r_level(0.0), 0.5)), CLOSED, l_wait, CLOSED, "tip it forward")
    level = (lift + up(0.04), r_level(0.0))
    both(2.0, level, CLOSED, l_wait, CLOSED, "level, grit side up")
    turn = TURN + TURN_LAG
    over = tcp_for_disc(PAD_AT + up(0.015 + DISC_T / 2), r_level(turn))
    both(2.0, over, CLOSED, l_wait, CLOSED, "over the pad, holes to its holes")
    tc = touch_centre()
    r_tip = r_level(turn, TIP_DEG)
    both(1.5, tcp_for_disc(tc + up(0.012), r_tip), CLOSED, l_wait, CLOSED, "tip it, far edge down")
    touch = tcp_for_disc(tc, r_tip)
    both(1.2, touch, CLOSED, l_wait, CLOSED, "far edge on the pad")
    both(1.5, touch, CLOSED, press_pose(far_hold, 0.0, up=0.015), CLOSED, "finger over the far edge")
    both(1.0, touch, CLOSED, press_pose(far_hold, HOLD_PAST), CLOSED, "hold the far edge on")
    ajar = -0.012  # the fingers opened 12 mm each side of the disc: enough to let it drop, not to reach the pad
    both(1.5, touch, ajar, press_pose(far_hold, HOLD_PAST), CLOSED, "let it go")
    back = lambda pose, d, h: (pose[0] - d * pose[1][:, 2] + up(h), pose[1])  # noqa: E731  (back along the hand)
    both(1.5, back(touch, 0.05, 0.01), ajar, press_pose(far_hold, HOLD_PAST), CLOSED, "back away")
    r_away = back(touch, 0.08, 0.03)
    # WHY only 8 mm up: lifted 2 cm off the edge, the left hand counted as clear and the trial ended there (run sand1),
    # before the press in the middle
    both(1.5, r_away, OPEN, press_pose(far_hold, 0.0, up=0.008), CLOSED, "lift off the edge")
    both(1.2, r_away, OPEN, press_pose(middle, 0.0, up=0.008), CLOSED, "over its middle")
    both(1.0, r_away, OPEN, press_pose(middle, PRESS_PAST), CLOSED, "press it on")
    both(1.0, r_away, OPEN, press_pose(middle, PRESS_PAST), CLOSED, "hold it pressed")
    both(1.2, r_away, OPEN, l_wait, CLOSED, "hands clear")
    both(1.5, r_away, OPEN, l_wait, CLOSED, "wait")
    return {"left": left, "right": right}
