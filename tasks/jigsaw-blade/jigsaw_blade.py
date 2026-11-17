"""On the station table: a corded jigsaw, unplugged, lying on its side in its case's foam insert with its blade clamp's
mouth facing the arms, and a new T-shank wood blade standing in a holder with its teeth facing away from the saw's
nose. Fit the blade: hold the
clamp's spring-loaded cap pushed in, push the blade's shank into the clamp with its teeth towards the saw's nose until it
clicks, then let go of the cap and of the blade.

The clamp is a model of a tool-free (SDS-type) blade clamp: with the cap not pushed in, its catch stops the blade 3 mm
into the slot; with the cap pushed 3.5 mm in, the blade goes on to the bottom of the slot and clicks, and once the cap
is let go with the blade fully in, the blade is locked.

Run it:  inspect-robots run --embodiment yam_arena -E task=jigsaw_blade.py -E object_jitter=0 -E control_hz=20 \\
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
from fittingkit import Slider, body, box  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

FOAM = (0.16, 0.16, 0.17)
HOUSING = (0.10, 0.42, 0.40)  # a power tool's teal
BLACK = (0.08, 0.08, 0.09)
STEEL = (0.70, 0.71, 0.73)
CAP_RED = (0.80, 0.12, 0.10)
BLADE_STEEL = (0.62, 0.64, 0.68)
HOLDER = (0.85, 0.75, 0.30)


def rot_z(angle: float) -> np.ndarray:
    return axis_angle_matrix(Z_AXIS, angle)


def rot_y(angle: float) -> np.ndarray:
    return axis_angle_matrix(Y_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the jigsaw (fixed): on its right side in its case's foam, its plunger pointing at the arms (-x), its nose to +y --
# Lying like this the clamp's slot opens towards the arms and the blade goes in along +x, teeth towards the nose (+y).
SLOT_MOUTH = np.array([0.34, 0.0, TOP + 0.09])  # the middle of the clamp slot's mouth
FOAM_H = 0.055  # the open case's foam insert the saw lies on
# WHY 9 cm up: with the clamp 4.7 cm up, the right arm can't hold the blade level in line with the slot (it misses
# by 8-40 mm); at 9 cm and 34 cm out it reaches every pose within 0.2 mm
SLOT_W, SLOT_T, SLOT_DEPTH = 10.0 * MM, 3.5 * MM, 14.0 * MM  # along y (the blade's width), z (its thickness), in (+x)
CLAMP_HALF = 12.0 * MM  # the clamp block round the slot, half its size across (y and z)
CAP_TRAVEL, UNLOCK = 5.0 * MM, 3.5 * MM  # the cap's travel, and how far in it unlocks the clamp
CAP_K = 1250.0  # N/m: 4.4 N at the unlock point [estimate: a two-finger push]
CATCH, SEAT = 3.0 * MM, 10.0 * MM  # the catch stops the blade this far in; this far in it clicks
SAW_AT = np.array([0.42, 0.0, TOP])


def _saw_shapes() -> list:
    """In the saw's frame (origin on the table under its middle; the same axes as the world)."""
    o = SAW_AT
    mx, my, mz = SLOT_MOUTH - o
    floor = mx + SLOT_DEPTH
    c, d = CLAMP_HALF, SLOT_DEPTH
    return [
        box((0.0, 0.0, FOAM_H / 2), (0.20, 0.24, FOAM_H), FOAM),  # the open case and its foam insert
        box((floor + 0.004 + 0.08, 0.02, mz), (0.16, 0.16, 0.070), HOUSING),  # motor and handle, on its side
        # the clamp block round the slot (its back closes the slot)
        box((mx + d / 2, my + (SLOT_W / 2 + c) / 2, mz), (d, c - SLOT_W / 2, 2 * c), STEEL),
        box((mx + d / 2, my - (SLOT_W / 2 + c) / 2, mz), (d, c - SLOT_W / 2, 2 * c), STEEL),
        box((mx + d / 2, my, mz + (SLOT_T / 2 + c) / 2), (d, SLOT_W, c - SLOT_T / 2), STEEL),
        box((mx + d / 2, my, mz - (SLOT_T / 2 + c) / 2), (d, SLOT_W, c - SLOT_T / 2), STEEL),
        box((floor + 0.002, my, mz), (0.004, 2 * c, 2 * c), BLACK),  # the plunger behind the clamp
        # the shoe (footplate), standing on edge on the arms' side; its open front leaves the slot and the cap clear
        box((mx - 0.02, my - 0.065, mz), (0.004, 0.07, 0.070), STEEL),
        # WHY 3 cm clear of the slot: 1 cm clear, the blade's edge caught the shoe going in and was pushed down and
        # aside, under the clamp (run jb1)
    ]


def _saw_looks() -> list:
    """Drawn only (in the saw's frame): what makes it read as a jigsaw from above and from the arms: the D-handle's
    opening and grip, the trigger, the cord and the motor's vents, on the upturned side."""
    o = SAW_AT
    mx, my, mz = SLOT_MOUTH - o
    top = mz + 0.035 + 0.0005  # the upturned side of the housing
    hx = mx + SLOT_DEPTH + 0.004 + 0.13  # the handle, at the saw's back (far from the arms, towards the nose)
    return [
        box((hx, 0.03, top), (0.05, 0.07, 0.001), BLACK),  # the D-handle's opening, seen from above
        box((hx + 0.03, 0.03, top), (0.012, 0.11, 0.001), (0.20, 0.20, 0.21)),  # the grip
        box((hx - 0.01, 0.075, top + 0.0005), (0.018, 0.008, 0.001), CAP_RED),  # the trigger
        box((mx + 0.05, 0.07, top), (0.04, 0.012, 0.001), BLACK),  # motor vents
        box((mx + 0.05, 0.045, top), (0.04, 0.004, 0.001), BLACK),
        box((hx + 0.075, 0.03, mz), (0.05, 0.008, 0.008), BLACK),  # the cord leaving the back
    ]


_SAW_SOLID = _saw_shapes()
_SAW_SEEN = _saw_looks()
SAW = body("jigsaw", pos=tuple(SAW_AT), friction=0.6, shapes=_SAW_SOLID + _SAW_SEEN,
           visual_only=tuple(range(len(_SAW_SOLID), len(_SAW_SOLID) + len(_SAW_SEEN))))
CAP_SIZE = (10.0 * MM, 16.0 * MM, 20.0 * MM)  # along x (its travel), y, z
CAP_AT = SLOT_MOUTH + np.array([CAP_SIZE[0] / 2, CLAMP_HALF + 2.0 * MM + CAP_SIZE[1] / 2, 0.0])  # on the nose side
CAP = body("cap", pos=tuple(CAP_AT), mass=0.01, friction=0.8, shapes=[box((0.0, 0.0, 0.0), CAP_SIZE, CAP_RED)])
fittingkit.add_slider(Slider("jigsaw", "cap", anchor=tuple(CAP_AT - SAW_AT), axis=(1, 0, 0), lower=0.0, upper=CAP_TRAVEL,
                             stiffness=CAP_K, damping=2.0, target=0.0))

# -- the blade: a T-shank wood blade, 100 mm (Bosch T 144 D: 100 mm long) ------------------------------------------------
# Its frame: origin at the shank's end, z along it towards the tip, the teeth on its +x edge.
# [estimate: 7.5 mm wide, 1.5 mm thick, 8 g]
BLADE_L, BLADE_W, BLADE_T = 100.0 * MM, 7.5 * MM, 1.5 * MM
SHANK_L = 12.0 * MM


def _blade_shapes() -> tuple[list, list]:
    solid = [box((0.0, 0.0, BLADE_L / 2), (BLADE_W, BLADE_T, BLADE_L), BLADE_STEEL)]
    seen = [box((BLADE_W / 2 + 0.6 * MM, 0.0, z), (1.2 * MM, BLADE_T * 0.6, 2.4 * MM), BLADE_STEEL)
            for z in np.arange(SHANK_L + 6 * MM, BLADE_L - 2 * MM, 4.5 * MM)]  # the teeth along the +x edge
    seen.append(box((0.0, 0.0, SHANK_L - 1.0 * MM), (BLADE_W + 1.5 * MM, BLADE_T + 0.2 * MM, 2.0 * MM), BLACK))  # the T
    return solid, seen


HOLDER_AT = np.array([0.20, -0.16, TOP])
HOLDER_FLOOR, HOLDER_H = 10.0 * MM, 30.0 * MM
BLADE_AT = HOLDER_AT + np.array([0.0, 0.0, HOLDER_FLOOR])  # standing shank down in the holder
R_BLADE_START = rot_z(math.pi)  # its teeth towards the arms (-x): it has to be turned a quarter turn to go in
R_BLADE_IN = np.column_stack([Y_AXIS, -Z_AXIS, -X_AXIS])  # in the clamp: tip towards the arms, teeth towards the nose


def _holder_shapes() -> list:
    """In the holder's frame (origin on the table): a block with a slot the blade stands in, 2 mm clear each side
    (the blade's width along x, its thickness along y)."""
    gx, gy = BLADE_W / 2 + 2.0 * MM, BLADE_T / 2 + 2.0 * MM
    h, f = HOLDER_H, HOLDER_FLOOR
    return [
        box((0.0, 0.0, f / 2), (0.04, 0.03, f), HOLDER),
        box(((gx + 0.02) / 2, 0.0, (f + h) / 2), (0.02 - gx, 0.03, h - f), HOLDER),
        box((-(gx + 0.02) / 2, 0.0, (f + h) / 2), (0.02 - gx, 0.03, h - f), HOLDER),
        box((0.0, (gy + 0.015) / 2, (f + h) / 2), (2 * gx, 0.015 - gy, h - f), HOLDER),
        box((0.0, -(gy + 0.015) / 2, (f + h) / 2), (2 * gx, 0.015 - gy, h - f), HOLDER),
    ]


BLADE_HOLDER = body("blade_holder", pos=tuple(HOLDER_AT), friction=0.6, shapes=_holder_shapes())
_BS, _BV = _blade_shapes()
BLADE = body("blade", pos=tuple(BLADE_AT), mass=0.008, friction=0.6, shapes=_BS + _BV,
             visual_only=tuple(range(len(_BS), len(_BS) + len(_BV))), quat=quat_xyzw(R_BLADE_START),
             com=(0.0, 0.0, BLADE_L / 2), contact_offset=0.0005)

OBJECTS = [SAW, CAP, BLADE_HOLDER, BLADE]


# -- the clamp ---------------------------------------------------------------------------------------------------------
IN_SLOT_OFF = 2.5 * MM  # the shank's end within this of the slot's middle (across)
UPRIGHT_DEG = 15.0  # and the blade within this of pointing straight out of the slot
CATCH_K, CATCH_MAX = 20000.0, 40.0  # the catch: a stiff stop (N/m), up to 40 N
CLEAR = 0.02  # both grippers this far from the blade at the end


def in_slot(blade, origin=np.zeros(3)) -> float:
    """How far (m) the blade's shank is into the slot (0 if it isn't lined up with the slot)."""
    end = blade.pos - origin
    out = blade.rot[:, 2]  # from the shank towards the tip: out of the slot, towards the arms (-x)
    if math.degrees(math.acos(float(np.clip(-out[0], -1.0, 1.0)))) > UPRIGHT_DEG:
        return 0.0
    if math.hypot(end[1] - SLOT_MOUTH[1], end[2] - SLOT_MOUTH[2]) > IN_SLOT_OFF:
        return 0.0
    return max(0.0, float(end[0] - SLOT_MOUTH[0]))


def teeth_forward(blade) -> bool:
    """The blade's teeth towards the saw's nose (+y)."""
    return bool(blade.rot[1, 0] > 0.7)


class Clamp:
    """Runs before every physics substep (240 Hz): the catch, the click and the lock, from the cap's and the blade's
    poses. Not locked, with the cap not pushed in, the catch pushes the blade back out past 3 mm. With the cap pushed in
    and the blade 10 mm in, it clicks; when the cap is then let go, the blade is locked where it is."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("JIGSAW_LOG")
        self.trace = os.environ.get("JIGSAW_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["blade", "cap"]

    def clear(self) -> None:
        self._n = 0
        self.clicked = False
        self.locked = False
        self.locked_forward = False
        self.cap_in = 0.0
        self.depth = 0.0
        self.poses = {}
        self._lock_pose = None

    def update(self, frames: dict, acc: dict) -> None:
        b, c = frames.get("blade"), frames.get("cap")
        if b is None or c is None:
            return
        self._n += 1
        self.cap_in = float(c.pos[0] - self.origin[0] - CAP_AT[0])
        if self.locked:
            self.poses = {"blade": self._lock_pose}
            self.depth = SEAT
            return
        self.depth = in_slot(b, self.origin)
        unlocked = self.cap_in >= UNLOCK
        if self.clicked and self.depth < SEAT - 3.0 * MM:
            self.clicked = False  # pulled back out while the cap was still in
        if not self.clicked and not unlocked and self.depth > CATCH:
            push = min(CATCH_MAX, CATCH_K * (self.depth - CATCH))
            fittingkit._wrench(acc, "blade", b, b.rot[:, 2] * push, b.pos)
        if unlocked and self.depth >= SEAT and not self.clicked:
            self.clicked = True
            self._note(f"{self._n / 240:7.2f}s click: {1000 * self.depth:4.1f} mm in, teeth forward {int(teeth_forward(b))}")
        # WHY lock at the unlock point: locking only once the cap was fully back out (under 1 mm), the catch acted on the
        # clicked blade on the way and threw it 12 cm out of the clamp (run jb3)
        if self.clicked and not unlocked:
            self.locked = True
            self.locked_forward = teeth_forward(b)
            self._lock_pose = (b.pos.copy(), quat_xyzw(b.rot))
            self.poses = {"blade": self._lock_pose}
            self._note(f"{self._n / 240:7.2f}s locked, teeth forward {int(self.locked_forward)}")
        if self.trace and self._n % 24 == 0:
            self._write_trace(b)

    def _write_trace(self, b) -> None:
        """Debug only (JIGSAW_TRACE=<file>): every 0.1 s, the cap, the blade and both TCPs (mm, from the table)."""
        end = 1000 * (b.pos - self.origin)
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s cap in {1000 * self.cap_in:4.1f} | blade end ({end[0]:6.1f},{end[1]:6.1f},"
                     f"{end[2] - 1000 * TOP:6.1f}) out-x {-b.rot[0, 2]:5.2f} teeth y {b.rot[1, 0]:5.2f} in slot "
                     f"{1000 * self.depth:4.1f} clicked {int(self.clicked)} locked {int(self.locked)} |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


MODEL = fittingkit.add_model(Clamp())

# -- scoring -------------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def hands_clear() -> bool:
    """No gripper point within 2 cm of the blade (its axis from end to tip); unknown offline counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0 or MODEL._lock_pose is None:
        return True
    p, q = MODEL._lock_pose
    r = fittingkit.quat_to_matrix(q)
    a, d = np.asarray(p) - MODEL.origin, r[:, 2]
    rel = pts - MODEL.origin - a
    t = np.clip(rel @ d, 0.0, BLADE_L)
    dist = np.linalg.norm(rel - np.outer(t, d), axis=1)
    return bool(dist.min() > CLEAR)


def success(objects) -> bool:
    """The blade locked in the clamp with its teeth towards the saw's nose, the cap let go, and both grippers clear of
    the blade. Must hold for 1 s."""
    return MODEL.locked and MODEL.locked_forward and MODEL.cap_in < 1.0 * MM and hands_clear()


def score(objects) -> float:
    """0.9 for the blade locked teeth forward with the cap let go (1.0 once success() has held 1 s, the hands clear);
    0.5 for it locked teeth backwards; 0.25 for the blade fully in the slot but not locked; 0 otherwise."""
    if MODEL.locked:
        return COMPLETE_SCORE if MODEL.locked_forward else 0.5
    return 0.25 if MODEL.depth >= SEAT else 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# The left hand, fingers closed and pointing at the saw, pushes the red cap in with its fingertips and holds it. The right
# hand picks the blade up by its top, turns it down to lie pointing at the saw, shank first, its teeth towards the nose,
# and pushes the shank into the slot to the bottom. The left hand lets the cap go (the blade locks), then the right lets
# go of the blade, and both move clear.
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
PAD_TIP = 13.0 * MM

GRIP_BELOW_TOP = 18.0 * MM  # the right TCP on the blade's axis, this far from its tip (the pads over 30 mm of it)
# The right hand points level at the slot, along the blade, its fingers closing up and down across the blade's flats,
# turned half a turn about its axis (so the arm can reach it).
# WHY level and in line: a pinch between parallel pads holds the blade flat against them, so the blade always lies at
# the hand's own pitch (run jb2: the hand pitched 10 degrees down, the blade went in 11 degrees down, under the slot);
# with the slot 28 cm out the arm reaches the level pose turned half a turn within 0.1 mm.
R_HAND_IN = axis_angle_matrix(X_AXIS, math.pi) @ np.column_stack([Z_AXIS, -Y_AXIS, X_AXIS])
R_HAND_IN = axis_angle_matrix(X_AXIS, math.pi) @ R_HAND_IN @ axis_angle_matrix(Z_AXIS, math.pi)
R_UPRIGHT = rot_y(math.pi / 2) @ R_HAND_IN  # the hand pointing down, holding the blade upright, teeth to the nose
R_PICK = rot_z(math.pi / 2) @ R_UPRIGHT  # picking the blade up from its holder, its teeth towards the arms
# WHY two plain turns: from R_PICK straight to R_HAND_IN is a tip and a turn at once, and on the way the right arm's IK
# lost the pose by up to 48 mm; turned first about the vertical, then tipped over about y, it follows. The blade starts
# a quarter turn wrong (teeth to the arms), not a half: a half turn about the vertical is past the wrist's range.
_R_TB = R_PICK.T @ R_BLADE_START  # the blade's orientation in the hand
_P_TB = -_R_TB @ np.array([0.0, 0.0, BLADE_L - GRIP_BELOW_TOP])  # the blade's shank end, from the TCP, in the hand
PICK_TCP = BLADE_AT + R_BLADE_START @ np.array([0.0, 0.0, BLADE_L - GRIP_BELOW_TOP])


def tcp_for_blade(end_at) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP's pose that puts the blade's shank end (held as at the pick) at ``end_at``, in line with the slot."""
    return np.asarray(end_at) - R_HAND_IN @ _P_TB, R_HAND_IN


PRESS_YAW = math.radians(-35.0)  # the left hand points at the cap from the nose side, clear of the right hand
PRESS_DOWN = math.radians(20.0)  # and a little down
_press_dir = np.array([math.cos(PRESS_DOWN) * math.cos(PRESS_YAW), math.cos(PRESS_DOWN) * math.sin(PRESS_YAW),
                       -math.sin(PRESS_DOWN)])
_px = np.cross(Z_AXIS, _press_dir)
_px /= np.linalg.norm(_px)
R_PRESS = axis_angle_matrix(_press_dir, math.pi / 2) @ np.column_stack([_px, np.cross(_press_dir, _px), _press_dir])
# WHY rolled 90 degrees: with its fingers closing sideways the left arm misses the press pose by about 70 degrees;
# rolled a quarter turn (closing up and down) it reaches it
PRESS_AT = CAP_AT - np.array([CAP_SIZE[0] / 2, 0.0, 0.0])  # the middle of the cap's face
PRESS_PAST = 7.0 * MM  # the fingertips aimed this far past the cap's face (it stops at 5 mm) [estimate: compliance]
INSERT_PAST = 1.5 * MM  # the shank aimed this far past the slot's bottom [estimate: compliance]


def press_pose(depth: float, back: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose with its closed fingertips ``depth`` past the cap's face along x (or ``back`` along the hand
    before it)."""
    tip = PRESS_AT + np.array([depth, 0.0, 0.0]) - R_PRESS[:, 2] * back
    return tip - R_PRESS[:, 2] * PAD_TIP, R_PRESS


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    high, near = np.array([0.0, 0.0, 0.12]), np.array([0.0, 0.0, 0.02])
    lift = np.array([0.0, 0.0, 0.13])
    l_wait = press_pose(0.0, back=0.06)
    both(2.5, (PICK_TCP + high, R_PICK), OPEN, l_wait, CLOSED, "high over the blade")
    both(1.2, (PICK_TCP + near, R_PICK), OPEN, l_wait, CLOSED, "just over it")
    both(0.5, (PICK_TCP + near, R_PICK), OPEN, l_wait, CLOSED, "settle over it")
    both(0.8, (PICK_TCP, R_PICK), OPEN, l_wait, CLOSED, "down to it")
    both(2.2, (PICK_TCP, R_PICK), CLOSED, l_wait, CLOSED, "grip")
    both(1.5, (PICK_TCP + lift, R_PICK), CLOSED, l_wait, CLOSED, "lift it out")
    both(2.0, (PICK_TCP + lift, R_UPRIGHT), CLOSED, l_wait, CLOSED, "turn it, teeth to the nose")
    # WHY by way of a pose turned 30 degrees: tipped straight over from upright to level, the right arm's IK loses the
    # pose on the way (up to 27 degrees); half tipped and turned 30 degrees about the vertical, it follows
    from yam_arena import ik as _ik
    tip_mid = rot_z(math.radians(30.0)) @ _ik.slerp_matrix(R_UPRIGHT, R_HAND_IN, 0.5)
    both(1.8, (PICK_TCP + lift + np.array([0.0, 0.0, 0.05]), tip_mid), CLOSED, l_wait, CLOSED, "tip it over")
    both(3.6, tcp_for_blade(SLOT_MOUTH - np.array([0.03, 0.0, 0.0])), CLOSED, press_pose(0.0, back=0.01), CLOSED,
         "turn it in line with the slot, finger to the cap")
    both(1.0, tcp_for_blade(SLOT_MOUTH - np.array([0.03, 0.0, 0.0])), CLOSED, press_pose(PRESS_PAST), CLOSED,
         "push the cap in")
    both(0.5, tcp_for_blade(SLOT_MOUTH - np.array([0.03, 0.0, 0.0])), CLOSED, press_pose(PRESS_PAST), CLOSED, "hold it")
    both(1.2, tcp_for_blade(SLOT_MOUTH - np.array([0.004, 0.0, 0.0])), CLOSED, press_pose(PRESS_PAST), CLOSED,
         "shank to the slot")
    pin = tcp_for_blade(SLOT_MOUTH + np.array([SLOT_DEPTH + INSERT_PAST, 0.0, 0.0]))
    both(1.5, pin, CLOSED, press_pose(PRESS_PAST), CLOSED, "push it in")
    both(0.6, pin, CLOSED, press_pose(PRESS_PAST), CLOSED, "click")
    both(1.0, pin, CLOSED, press_pose(0.0, back=0.03), CLOSED, "let the cap go")
    both(2.2, pin, OPEN, press_pose(0.0, back=0.03), CLOSED, "let the blade go")
    away = (pin[0] - R_HAND_IN[:, 2] * 0.06, pin[1])
    both(1.5, away, OPEN, press_pose(0.0, back=0.08), CLOSED, "hands clear")
    both(1.5, away, OPEN, press_pose(0.0, back=0.08), CLOSED, "wait")
    return {"left": left, "right": right}
