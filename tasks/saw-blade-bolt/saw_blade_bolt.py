"""On the station table, a little left of centre: a cordless circular saw for 184 mm (7-1/4 in) blades, lying on its
side with its blade facing up. A new blade is on, and the clamping bolt at its centre (a hex socket for a 6 mm key,
on a clamping flange) is hand-tight: a white paint line across the bolt's head lines up with one on the flange. The
blade's teeth are covered all round by the guards. On the gear housing, on the robot's right of the blade, is the
spring-loaded spindle lock button. A 6 mm hex key stands in its holder on the left. Tighten the bolt a further
quarter turn with the key, as the saw's maker says, then put the key down.

The bolt turns on the spindle's own axis, so the key turns the blade and bolt together unless the spindle lock is
held in; the lock only catches when the spindle comes round to one of its two notches.

Run it:  inspect-robots run --embodiment yam_arena -E task=saw_blade_bolt.py -E object_jitter=0 -E control_hz=20 \\
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
from fittingkit import Slider, _torque, _wrench, body, box, cyl, matrix_to_quat_rel, ring  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
HOUSING = (0.20, 0.21, 0.23)  # dark grey plastic
GUARD = (0.55, 0.57, 0.60)  # cast aluminium
LOWER_GUARD = (0.35, 0.36, 0.38)
BLADE = (0.30, 0.31, 0.33)
STEEL = (0.62, 0.63, 0.65)
KEY_STEEL = (0.18, 0.18, 0.19)  # black-oxide hex key
WOOD = (0.55, 0.40, 0.25)
WHITE = (0.95, 0.95, 0.93)
RED = (0.75, 0.10, 0.10)
DARK = (0.06, 0.06, 0.07)
fittingkit.METALLIC.update({STEEL: 0.25, GUARD: 0.45, BLADE: 0.35})

# -- the frame -------------------------------------------------------------------------------------------------------
# The spindle's axis is vertical through AXIS_XY. Angles are about +z (counterclockwise seen from above) from world +x.
# Tightening the bolt is clockwise seen from above: the key's angle decreases.
# WHY here: lifting the key 4-6 cm off the bolt at the end of the turn, the left arm ran out of reach 2-4 cm further
# out (at (0.32, 0.10) the IK missed by 19 mm; yam_arena IK, 2026-11-16); here it reaches the whole turn 6 cm up.
AXIS_XY = np.array([0.26, 0.14])
BLADE_Z = TOP + 95.0 * MM  # the blade's mid-plane
BLADE_R, BLADE_T = 92.0 * MM, 1.6 * MM  # a 184 mm (7-1/4 in) blade
GUARD_IN, GUARD_OUT = 76.0 * MM, 101.0 * MM  # the guard ring round the teeth (radii)
FLANGE_R, FLANGE_T = 15.0 * MM, 4.0 * MM  # the outer clamping flange, on the blade
HEAD_R, HEAD_T = 6.5 * MM, 8.0 * MM  # the bolt's round socket head (13 mm across)
SOCKET_DEPTH = 5.0 * MM  # the 6 mm hex socket
# its collision hole's radius: the key's arm collides as a 3 mm round, so 1.5 mm of play (the model's engagement
# tolerance). WHY not 4.2: 1.2 mm of play was less than the arm holds still to (sim run 4: 1.4 mm), so the key caught the rim
SOCKET_HOLE = 4.5 * MM
FLANGE_TOP = BLADE_Z + BLADE_T / 2 + FLANGE_T
HEAD_TOP = FLANGE_TOP + HEAD_T
AXIS_TOP = np.array([*AXIS_XY, HEAD_TOP])  # where the spindle's axis leaves the bolt's head

# The spindle lock: a button on a pad of the gear housing, on the robot's right of the blade (its pin goes into a
# notch in the spindle's gear, under the blade, at two positions half a turn apart).
LOCK_XY = AXIS_XY + np.array([0.0, -0.165])
PAD_TOP = TOP + 75.0 * MM
BUTTON_R, BUTTON_PROUD = 12.0 * MM, 7.0 * MM  # a wide cap, so the closed fingertips press it and not the pad
BUTTON_TRAVEL = 4.5 * MM
# WHY a light spring: the YAM presses with its joints' position springs; 2-5 N is what it holds steadily. In the
# official sim the right hand, aimed 12 mm below the cap, held it in 2.1 mm on the gear's face and 3.8 mm once caught.
BUTTON_K, BUTTON_PRELOAD_M = 600.0, 3.0 * MM  # N/m, and the drive's target above rest (a 1.8 N preload)
PIN_IN = 1.2 * MM  # pressed this far, the pin is on the gear; at a notch it goes in and the spindle is locked
RIDE = 1.5 * MM  # away from a notch, the pin rides on the gear's face and stops the button here
NOTCH_EVERY = 180.0  # degrees: the spindle's two notches
NOTCH_TOL = 0.05  # degrees: the pin goes in only as the spindle reaches a notch (it is then held exactly there)

# The blade's angle at the start, from a notch: turning the key clockwise, the spindle meets a notch after 25 degrees.
SPINDLE0_DEG = 25.0
# Hand-tight, the bolt's socket lines up with a key whose long arm points this way (degrees from +x).
KEY_IN_DEG = 75.0
HEX_OFF_DEG = KEY_IN_DEG - SPINDLE0_DEG  # the socket's flats relative to the blade, hand-tight
PAINT_DEG = 210.0  # where the paint lines point at the start (towards the robot's camera side)


def _sector_ring(r_in, r_out, z0, z1, color, segments=36) -> list:
    """A ring of boxes about the body's z axis."""
    rm, w = (r_in + r_out) / 2, r_out - r_in
    chord = 2 * r_out * math.tan(math.pi / segments) + 1.0 * MM
    return [box((rm * math.cos(a), rm * math.sin(a), (z0 + z1) / 2), (w, chord, z1 - z0), color, yaw=a)
            for a in (2 * math.pi * k / segments for k in range(segments))]


# -- the saw (fixed) -------------------------------------------------------------------------------------------------
# Its frame: origin on the table under the spindle's axis, world-aligned.
def _saw_shapes() -> tuple[list, list]:
    bz = BLADE_Z - TOP
    pad_z = PAD_TOP - TOP
    lx, ly = LOCK_XY - AXIS_XY
    hole = BUTTON_R + 2.0 * MM
    solid = [
        cyl((0.0, 0.0, (bz - 5.0 * MM) / 2), 50.0 * MM, bz - 5.0 * MM, HOUSING),  # the motor and gear housing
        cyl((0.0, 0.0, bz - 3.0 * MM), 15.0 * MM, 2.0 * MM, STEEL),  # the inner flange, under the blade
        # the gear housing's arm out to the lock button, and its pad round the button's hole
        box((lx, ly / 2 - 10.0 * MM, (pad_z - 12.0 * MM) / 2), (50.0 * MM, abs(ly) + 20.0 * MM, pad_z - 12.0 * MM),
            HOUSING),
        box((lx, ly, pad_z - 6.0 * MM - hole / 2), (50.0 * MM, 50.0 * MM, 12.0 * MM - hole), HOUSING),
        *[box((lx + sx * (hole + (25.0 * MM - hole) / 2), ly, pad_z - 6.0 * MM), (25.0 * MM - hole, 50.0 * MM,
              12.0 * MM), HOUSING) for sx in (1, -1)],
        *[box((lx, ly + sy * (hole + (25.0 * MM - hole) / 2), pad_z - 6.0 * MM), (2 * hole, 25.0 * MM - hole,
              12.0 * MM), HOUSING) for sy in (1, -1)],
        # the guards: closed round the blade's rim, 76-101 mm from the axis, 14 mm above and below it
        *_sector_ring(GUARD_IN, GUARD_OUT, bz - 14.0 * MM, bz + 14.0 * MM, GUARD, segments=18)[:9],
        *_sector_ring(GUARD_IN, GUARD_OUT, bz - 14.0 * MM, bz + 14.0 * MM, LOWER_GUARD, segments=18)[9:],
        box((-115.0 * MM, 0.0, 45.0 * MM), (40.0 * MM, 36.0 * MM, 90.0 * MM), HOUSING),  # the rear handle
        box((108.0 * MM, 30.0 * MM, 65.0 * MM), (5.0 * MM, 230.0 * MM, 130.0 * MM), GUARD),  # the shoe, on edge
    ]
    seen = [
        box((lx, ly + BUTTON_R + 9.0 * MM, pad_z + 0.05 * MM), (14.0 * MM, 3.0 * MM, 0.1 * MM), WHITE),  # lock icon
    ]
    return solid, seen


_SS, _SV = _saw_shapes()
SAW = body("saw", pos=(*AXIS_XY, TOP), friction=0.6, shapes=_SS + _SV,
           visual_only=tuple(range(len(_SS), len(_SS) + len(_SV))))

# -- the blade, with the outer clamping flange (posed by the model below) -------------------------------------------
# Its frame: origin on the axis at the blade's mid-plane; x the direction its flange's paint line points.


def _blade_shapes() -> tuple[list, list]:
    solid = [cyl((0.0, 0.0, 0.0), BLADE_R, BLADE_T, BLADE),
             cyl((0.0, 0.0, BLADE_T / 2 + FLANGE_T / 2), FLANGE_R, FLANGE_T, STEEL)]
    seen = [
        box(((FLANGE_R + HEAD_R) / 2 + 1.0 * MM, 0.0, BLADE_T / 2 + FLANGE_T + 0.05 * MM),
            (FLANGE_R - HEAD_R - 1.0 * MM, 1.6 * MM, 0.1 * MM), WHITE),  # the flange's paint line
        box((45.0 * MM, 0.0, BLADE_T / 2 + 0.05 * MM), (30.0 * MM, 8.0 * MM, 0.1 * MM), RED),  # the blade's label
    ]
    return solid, seen


def blade_rotation(spindle_deg: float) -> np.ndarray:
    a = math.radians(spindle_deg + PAINT_DEG)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


_BLS, _BLV = _blade_shapes()
BLADE_CENTRE = np.array([*AXIS_XY, BLADE_Z])
BLADE_BODY = body("blade", pos=tuple(BLADE_CENTRE), quat=matrix_to_quat_rel(blade_rotation(SPINDLE0_DEG)), mass=0.15,
                  friction=0.4, shapes=_BLS + _BLV, visual_only=tuple(range(len(_BLS), len(_BLS) + len(_BLV))))
fittingkit.no_gravity("blade")

# -- the clamping bolt (posed by the model) ---------------------------------------------------------------------------
# Its frame: origin on the axis at the top of its head; x its paint line's direction (lined up with the flange's when
# hand-tight). The socket's flats are HEX_OFF_DEG - PAINT_DEG round from x.


def _bolt_shapes() -> tuple[list, list]:
    solid = [*ring(-HEAD_T, 0.0, SOCKET_HOLE, HEAD_R, STEEL, segments=16),
             cyl((0.0, 0.0, -SOCKET_DEPTH - 1.5 * MM), SOCKET_HOLE + 0.5 * MM, 3.0 * MM, STEEL)]  # the socket floor
    seen = [
        box(((HEAD_R + SOCKET_HOLE) / 2, 0.0, 0.05 * MM), (HEAD_R - SOCKET_HOLE, 1.6 * MM, 0.1 * MM), WHITE),
        cyl((0.0, 0.0, -SOCKET_DEPTH + 0.05 * MM), SOCKET_HOLE, 0.1 * MM, DARK),
    ]
    return solid, seen


def bolt_rotation(spindle_deg: float, beta_deg: float) -> np.ndarray:
    return blade_rotation(spindle_deg - beta_deg)


_BOS, _BOV = _bolt_shapes()
BOLT = body("bolt", pos=tuple(AXIS_TOP), quat=matrix_to_quat_rel(bolt_rotation(SPINDLE0_DEG, 0.0)), mass=0.03,
            friction=0.6, shapes=_BOS + _BOV, visual_only=tuple(range(len(_BOS), len(_BOS) + len(_BOV))))
fittingkit.no_gravity("bolt")

# -- the spindle lock button (on a spring slider in the saw) ---------------------------------------------------------
BUTTON_REST = np.array([*LOCK_XY, PAD_TOP + BUTTON_PROUD - 6.0 * MM])  # its origin: 6 mm below its cap's top
BUTTON = body("button", pos=tuple(BUTTON_REST), mass=0.02, friction=0.9,
              shapes=[cyl((0.0, 0.0, 0.0), BUTTON_R, 12.0 * MM, RED)])
fittingkit.add_slider(Slider("saw", "button", anchor=tuple(BUTTON_REST - np.array([*AXIS_XY, TOP])), axis=(0, 0, 1),
                             lower=-BUTTON_TRAVEL, upper=0.0, stiffness=BUTTON_K, damping=8.0,
                             target=BUTTON_PRELOAD_M))
BUTTON_TOP_REST = BUTTON_REST[2] + 6.0 * MM

# -- the hex key: 6 mm across the flats, L-shaped -------------------------------------------------------------------
# Its frame: origin at the short arm's tip, z up the short arm, x along the long arm.
KEY_AF = 6.0 * MM
KEY_SHORT, KEY_LONG = 30.0 * MM, 100.0 * MM
KEY_ARM_Z = KEY_SHORT  # the long arm's centreline above the tip
GRIP_X = 55.0 * MM  # the demo grips the long arm this far from the short arm's axis


def hexagon(across_flats, length, color, centre=(0.0, 0.0, 0.0), axis="Z") -> list:
    """A hexagonal prism (three boxes turned 60 degrees apart) along z, or along x."""
    side = across_flats / math.sqrt(3)
    if axis == "Z":
        return [box(centre, (across_flats, side, length), color, yaw=k * math.pi / 3) for k in range(3)]
    return [box(centre, (length, across_flats, across_flats), color)]  # along x: drawn square (seen from above)


def _key_shapes() -> tuple[list, list]:
    solid = [cyl((0.0, 0.0, (KEY_SHORT + 3.0 * MM) / 2), 3.0 * MM, KEY_SHORT + 3.0 * MM, KEY_STEEL),
             box((3.0 * MM + KEY_LONG / 2, 0.0, KEY_ARM_Z), (KEY_LONG, KEY_AF, KEY_AF), KEY_STEEL)]
    seen = hexagon(KEY_AF, KEY_SHORT, KEY_STEEL, centre=(0.0, 0.0, KEY_SHORT / 2))
    return solid, seen


def key_rotation(psi_deg: float) -> np.ndarray:
    a = math.radians(psi_deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


# -- the key's holder (fixed): a base, a post with a hole for the short arm, a post under the long arm's end -------
# WHY y 0.30: at 0.27 the open gripper's body, coming down on the key, landed on the guard's top (sim run 1)
HOLDER_XY = np.array([0.20, 0.30])
HOLDER_BASE = 4.0 * MM
POST_H = KEY_ARM_Z - KEY_AF / 2  # the long arm rests on the posts


def _holder_shapes() -> list:
    hole, wall = 4.5 * MM, 7.0 * MM
    zc = HOLDER_BASE + POST_H / 2
    return [
        box((45.0 * MM, 0.0, HOLDER_BASE / 2), (130.0 * MM, 26.0 * MM, HOLDER_BASE), WOOD),
        *[box((sx * (hole + wall / 2), 0.0, zc), (wall, 2 * (hole + wall), POST_H), WOOD) for sx in (1, -1)],
        *[box((0.0, sy * (hole + wall / 2), zc), (2 * hole, wall, POST_H), WOOD) for sy in (1, -1)],
        box((95.0 * MM, 0.0, zc), (10.0 * MM, 16.0 * MM, POST_H), WOOD),
    ]


HOLDER = body("holder", pos=(*HOLDER_XY, TOP), friction=0.6, shapes=_holder_shapes())
KEY_START = np.array([*HOLDER_XY, TOP + HOLDER_BASE + 0.2 * MM])
_KS, _KV = _key_shapes()
KEY = body("key", pos=tuple(KEY_START), mass=0.032, com=(40.0 * MM, 0.0, 25.0 * MM), friction=0.9,
           shapes=_KS + _KV, visual_only=tuple(range(len(_KS), len(_KS) + len(_KV))), solver_iterations=32)

KEY_BOUNDS = KEY.bounds_local()
KEY_CORNERS = np.array([[x, y, z] for x in (KEY_BOUNDS[0][0], KEY_BOUNDS[1][0])
                        for y in (KEY_BOUNDS[0][1], KEY_BOUNDS[1][1]) for z in (KEY_BOUNDS[0][2], KEY_BOUNDS[1][2])])
OBJECTS = [SAW, BLADE_BODY, BOLT, BUTTON, KEY, HOLDER]


# -- the model: the spindle lock, the key in the bolt's socket, the bolt on the spindle ----------------------------
def _wrap(a: float, period: float) -> float:
    return (a + period / 2) % period - period / 2


class SawBladeBolt:
    """The spindle, its lock and the clamping bolt, run before every physics substep (240 Hz).

    The spindle (with the blade and flange) turns freely about the axis unless locked: the lock is in when the button
    is pressed at least ``PIN_IN`` and the spindle is at one of its notches (every ``NOTCH_EVERY`` degrees). Away from
    a notch, the pin rides on the gear's face and stops the button at ``RIDE``.

    The key is in the socket when its short arm is within ``tilt_tol`` of the axis, its tip within ``across`` of it
    and at least ``depth_on`` down in the socket, and its flats lined up with the socket's within ``align_tol`` (they
    repeat every 60 degrees). While in, the key is held on the axis, and turning it turns the bolt: with the spindle
    locked, the bolt turns on the spindle (``beta``, degrees past hand-tight; clockwise from above tightens) against a
    torque rising to ``torque_quarter`` at a quarter turn; unlocked, the spindle turns with it against a light drag. A
    misaligned key is pushed back out of the socket. Lifted out, or pulled off the axis, the key comes off."""

    def __init__(self, key="key", button="button", spindle0=SPINDLE0_DEG, across=1.5e-3, depth_on=1.5e-3, depth_off=0.5e-3,
                 tilt_tol=10.0, align_tol=12.0, k_hold=800.0, k_push=600.0, drag=0.01, torque_quarter=0.5,
                 beta_min=-45.0, beta_max=200.0):
        self.key, self.button = key, button
        self.spindle0 = spindle0
        self.across, self.depth_on, self.depth_off = across, depth_on, depth_off
        self.tilt_tol, self.align_tol = math.radians(tilt_tol), align_tol
        self.k_hold, self.k_push, self.drag, self.torque_quarter = k_hold, k_push, drag, torque_quarter
        self.beta_min, self.beta_max = beta_min, beta_max
        self.origin = np.zeros(3)
        self.log = os.environ.get("SAW_LOG")  # a file to log key and lock events to (sim traces)
        self._n = 0
        self.poses = {}
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return [self.key, self.button, "blade", "bolt"]

    def clear(self) -> None:
        self._n = 0
        self.spindle, self.beta = self.spindle0, 0.0
        self.engaged, self.locked = False, False
        self._prev = 0.0
        self.press = 0.0  # the button's depth (m)
        self.key_off_by = 1.0  # the key's tip's distance from the socket (m)
        self.key_low, self.key_speed = 1.0, 0.0  # the key's lowest point (m, world z) and its speed (m/s)
        self.key_pose = None  # (position, rotation) of the key's frame
        self.engagements = 0
        self.max_lock_press = 0.0
        self._pose()

    # -- state the scorer reads ----------------------------------------------------------------------------------
    def tightened_deg(self) -> float:
        return round(self.beta, 6)  # (so a set angle is not read a hair either side of a tier's edge)

    def key_clear(self) -> bool:
        return not self.engaged and self.key_off_by > 20.0 * MM

    def lock_released(self) -> bool:
        return self.press < 1.0 * MM

    def key_down(self, fingers) -> bool:
        """The key put down: lying on the table (its lowest point within 6 mm of it), still, and no gripper within
        1 cm of it. ``fingers``: the grippers' collision-hull points (world, (n, 3))."""
        if self.key_low - (TOP + self.origin[2]) > 6.0 * MM or self.key_speed > 0.05:
            return False
        fingers = np.asarray(fingers, float).reshape(-1, 3)
        if self.key_pose is None or len(fingers) == 0:
            return True
        pos, rot = self.key_pose
        local = (fingers - pos) @ rot  # each point in the key's frame
        lo, hi = KEY_BOUNDS
        near = np.all((local > lo - 10.0 * MM) & (local < hi + 10.0 * MM), axis=1)
        return not bool(near.any())

    # -- helpers -------------------------------------------------------------------------------------------------
    def at_notch(self) -> bool:
        return abs(_wrap(self.spindle, NOTCH_EVERY)) <= NOTCH_TOL

    def socket_deg(self) -> float:
        """The direction (degrees) a key's long arm must point to line up with the socket's flats."""
        return self.spindle + HEX_OFF_DEG - self.beta

    def _event(self, what: str, psi: float) -> None:
        if self.log:
            with open(self.log, "a") as f:
                f.write(f"{self._n / 240.0:7.2f}s {what:6s} key {psi:7.1f} spindle {self.spindle:7.1f} "
                        f"bolt past hand-tight {self.beta:6.1f} button {1000 * self.press:4.1f} mm "
                        f"locked {int(self.locked)}\n")

    def update(self, frames: dict, acc: dict) -> None:
        self._n += 1
        fb, fk = frames.get(self.button), frames.get(self.key)
        if fb is not None and np.all(np.isfinite(fb.pos)):
            self.press = float(BUTTON_REST[2] + self.origin[2] - fb.pos[2])
            if not self.at_notch() and self.press > RIDE:  # the pin on the gear's face
                v = float(fb.vel[2])
                _wrench(acc, self.button, fb, np.array([0.0, 0.0, 3000.0 * (self.press - RIDE) - 15.0 * v]), fb.com)
        was = self.locked
        self.locked = self.press >= PIN_IN and self.at_notch()
        if self.locked:  # the pin in its notch holds the spindle there
            self.spindle = round(self.spindle / NOTCH_EVERY) * NOTCH_EVERY
        if fk is None or not np.all(np.isfinite(fk.pos)):
            self._pose()
            return
        axis_top = AXIS_TOP + self.origin
        tip = np.asarray(fk.pos, float)
        d = tip - axis_top
        lateral = float(np.hypot(d[0], d[1]))
        depth = float(-d[2])  # how far the tip is down in the socket
        self.key_off_by = float(np.linalg.norm(d))
        rot = np.asarray(fk.rot, float)
        self.key_low = float(np.min((KEY_CORNERS @ rot.T)[:, 2])) + float(tip[2])
        self.key_speed = float(np.linalg.norm(fk.vel))
        self.key_pose = (tip, rot)
        up = fk.rot[:, 2]
        tilt = math.acos(max(-1.0, min(1.0, float(up[2]))))
        psi = math.degrees(math.atan2(float(fk.rot[1, 0]), float(fk.rot[0, 0])))
        mis = _wrap(psi - self.socket_deg(), 60.0)
        if not self.engaged:
            if lateral <= self.across and depth >= self.depth_on and tilt <= self.tilt_tol and abs(mis) <= self.align_tol:
                self.engaged, self._prev = True, psi
                self.engagements += 1
                self._event("in", psi)
            elif lateral <= SOCKET_HOLE and depth > 0.3 * MM:  # its flats against the socket's: pushed back out
                _wrench(acc, self.key, fk, np.array([0.0, 0.0, self.k_push * depth]), tip)
        else:
            if depth < self.depth_off or lateral > 2 * self.across + 1.5 * MM or tilt > 2 * self.tilt_tol:
                self.engaged = False
                self._event("out", psi)
            else:
                v = np.asarray(fk.point_vel(tip), float)
                f = -self.k_hold * np.array([d[0], d[1], 0.0]) - 2.0 * np.array([v[0], v[1], 0.0])
                _wrench(acc, self.key, fk, f, tip)
                dpsi = _wrap(psi - self._prev, 360.0)
                spin = float(fk.spin[2])
                if self.locked:
                    self._turn_bolt(dpsi, spin, fk, acc)
                else:
                    target = self.spindle + dpsi
                    notch = self._notch_between(self.spindle, target)
                    if notch is not None and self.press >= PIN_IN:  # the pin drops into the notch on the way
                        self.spindle = notch
                        self.locked = True
                        self._event("lock", psi)
                        self._turn_bolt(target - notch, spin, fk, acc)
                    else:
                        self.spindle = target
                        _torque(acc, self.key, np.array([0.0, 0.0, -self.drag * math.tanh(spin / 0.5)]))
                self._prev = psi
        if self.locked and not was:
            self.max_lock_press = max(self.max_lock_press, self.press)
        self._pose()

    def _turn_bolt(self, dpsi: float, spin: float, fk, acc) -> None:
        new = min(max(self.beta - dpsi, self.beta_min), self.beta_max)
        self.beta = new
        # the thread's torque against the key: rising with the angle past hand-tight, against the turning
        t = self.torque_quarter * max(0.0, self.beta) / 90.0 + self.drag
        _torque(acc, self.key, np.array([0.0, 0.0, t * math.tanh(-spin / 0.3)]))  # +z resists tightening

    @staticmethod
    def _notch_between(a: float, b: float) -> float | None:
        """The first notch the spindle reaches turning from ``a`` to ``b`` (degrees), not counting ``a`` itself."""
        if b < a:
            n = math.floor((a - 1e-9) / NOTCH_EVERY) * NOTCH_EVERY
            return n if n >= b else None
        if b > a:
            n = math.ceil((a + 1e-9) / NOTCH_EVERY) * NOTCH_EVERY
            return n if n <= b else None
        return None

    def _pose(self) -> None:
        o = self.origin
        self.poses = {
            "blade": (BLADE_CENTRE + o, matrix_to_quat_rel(blade_rotation(self.spindle))),
            "bolt": (AXIS_TOP + o, matrix_to_quat_rel(bolt_rotation(self.spindle, self.beta))),
        }


MODEL = fittingkit.add_model(SawBladeBolt())


# -- scoring --------------------------------------------------------------------------------------------------------
# By angle past hand-tight: the maker gives 6-9 N m as "hand-tight plus 1/4 turn", so a quarter turn (90 degrees) is
# the mark; 72-108 degrees is 6-9 N m on a torque that rises with the angle.
COMPLETE_SCORE = 0.9  # score() in the window; 1.0 comes from success() holding for 1 s, which ends the trial
WINDOW = (72.0, 108.0)


def success(objects) -> bool:
    """The bolt a quarter turn past hand-tight (72-108 degrees), the key off it and put down on the table (no gripper
    on it), the spindle lock released. Must hold for 1 s."""
    b = MODEL.tightened_deg()
    return (WINDOW[0] <= b <= WINDOW[1] and MODEL.key_clear() and MODEL.lock_released()
            and MODEL.key_down(fittingkit.finger_points()))


def score(objects) -> float:
    """0.9 at 72-108 degrees (1.0 once success() has held 1 s); 0.5 at 36-72 or 108-144 (under- or over-tight); 0.25
    at 15-36; 0 otherwise (short of 15, past 144, or loosened)."""
    b = MODEL.tightened_deg()
    if WINDOW[0] <= b <= WINDOW[1]:
        return COMPLETE_SCORE
    if 36.0 <= b < WINDOW[0] or WINDOW[1] < b <= 144.0:
        return 0.5
    return 0.25 if 15.0 <= b < 36.0 else 0.0


# -- the scripted demo -------------------------------------------------------------------------------------------
# Open loop, straight-down grips only. The left hand takes the key out of its holder by the long arm (its fingers
# closing across it), puts the short arm into the bolt's socket, and, once the right hand is holding the spindle lock
# in, turns the key clockwise: the blade turns with it until the lock catches 25 degrees on, then the bolt turns a
# quarter turn past hand-tight. The left hand lifts the key out and lays it down; the right hand lets go of the lock.
HOLD = FINGER_OPEN * 1.0 / 94.0  # closed on the 6 mm arm
OPEN = FINGER_OPEN * 30.0 / 94.0
CLOSED = 0.0
PAD_PAST_TCP = 13.0 * MM  # the fingertips' pads reach this far below the TCP
TURN_TO_DEG = KEY_IN_DEG - (SPINDLE0_DEG + 90.0) - 3.0  # aimed 3 degrees past a quarter turn [estimate: arm lag]
KEY_DEPTH = 4.0 * MM  # how far the short arm goes into the socket
KEY_DOWN_XY = (0.30, 0.33)  # where the demo lays the key down, clear of the saw
# the left hand held still over the socket sat this far off its aim (tip, x and y; sim run 4 trace, 10.2-10.8 s), so
# the insertion aims the other way
SOCKET_AIM = np.array([1.0 * MM, -1.0 * MM, 0.0])
PRESS_BELOW = 12.0 * MM  # the right hand's aim below the button's cap [estimate: arm compliance]


def key_tcp(tip, psi_deg: float) -> tuple:
    """The TCP holding the key's long arm at GRIP_X, its tip at ``tip``, its long arm pointing ``psi_deg``."""
    a = math.radians(psi_deg)
    p = np.asarray(tip, float) + np.array([GRIP_X * math.cos(a), GRIP_X * math.sin(a), KEY_ARM_Z])
    return tuple(float(v) for v in p)


def _wl(dur, tip, psi, finger, label):
    return Waypoint(dur, key_tcp(tip, psi), finger, label, math.radians(psi))


def left_waypoints() -> list:
    start = KEY_START
    over = start + np.array([0.0, 0.0, 60.0 * MM])
    seat = AXIS_TOP - np.array([0.0, 0.0, KEY_DEPTH])
    above = seat + np.array([0.0, 0.0, 40.0 * MM])
    # WHY: the key travels with its tip at ``above``'s height, 35 mm over the guard's top; in sim run 2 it went across
    # lower and its short arm caught the guard, which knocked the key over in the hand
    lifted = np.array([start[0], start[1], above[2]])
    ws = [
        _wl(2.0, over, 0.0, OPEN, "over the key"),
        _wl(1.5, start, 0.0, OPEN, "round the key's long arm"),
        _wl(0.5, start, 0.0, OPEN, "settle round it"),
        _wl(0.8, start, 0.0, HOLD, "grip the key"),
        _wl(1.6, lifted, 0.0, HOLD, "lift it out of its holder"),
        _wl(2.0, above, KEY_IN_DEG, HOLD, "over the bolt"),
        _wl(0.6, above, KEY_IN_DEG, HOLD, "settle over the bolt"),
        # WHY in two steps: coming straight down, the arm lagged 4-5 mm sideways and the key landed on the head beside
        # the socket (sim run 3); it settles just above the socket, then goes in slowly
        _wl(1.0, AXIS_TOP + SOCKET_AIM + np.array([0.0, 0.0, 5.0 * MM]), KEY_IN_DEG, HOLD, "down to the socket"),
        _wl(0.8, AXIS_TOP + SOCKET_AIM + np.array([0.0, 0.0, 5.0 * MM]), KEY_IN_DEG, HOLD, "line up over the socket"),
        _wl(1.0, seat + SOCKET_AIM, KEY_IN_DEG, HOLD, "into the socket"),
        _wl(1.4, seat + SOCKET_AIM, KEY_IN_DEG, HOLD, "wait for the lock"),
    ]
    # the turn, in 7.5-degree steps (a straight line between waypoints would cut the circle's chord)
    n = int(math.ceil((KEY_IN_DEG - TURN_TO_DEG) / 7.5))
    for k in range(1, n + 1):
        psi = KEY_IN_DEG + (TURN_TO_DEG - KEY_IN_DEG) * k / n
        ws.append(_wl(0.35, seat, psi, HOLD, f"turn {k}"))
    ws += [
        _wl(0.6, seat, TURN_TO_DEG, HOLD, "end of the turn"),
        _wl(1.0, above, TURN_TO_DEG, HOLD, "lift the key out"),
    ]
    down = np.array([*KEY_DOWN_XY, TOP + 8.0 * MM])
    ws += [
        _wl(1.5, np.array([*KEY_DOWN_XY, above[2]]), 0.0, HOLD, "over the table"),
        _wl(1.5, down, 0.0, HOLD, "down by the table"),
        _wl(0.6, down, 0.0, OPEN, "let go of the key"),
        _wl(1.0, down + np.array([0.0, 0.0, 60.0 * MM]), 0.0, OPEN, "clear"),
    ]
    return ws


def right_waypoints() -> list:
    over = (float(LOCK_XY[0]), float(LOCK_XY[1]), BUTTON_TOP_REST + PAD_PAST_TCP + 40.0 * MM)
    touch = (over[0], over[1], BUTTON_TOP_REST + PAD_PAST_TCP)
    press = (over[0], over[1], BUTTON_TOP_REST + PAD_PAST_TCP - PRESS_BELOW)
    t_left = sum(w.duration for w in left_waypoints()[:7])  # until the key is over the bolt
    t_out = sum(w.duration for w in left_waypoints() if not w.label.startswith(("over the table", "down by", "let go",
                                                                                   "clear")))
    ws = [
        Waypoint(3.0, over, CLOSED, "over the lock"),
        Waypoint(max(0.5, t_left - 3.0), over, CLOSED, "wait over the lock"),
        Waypoint(1.0, touch, CLOSED, "onto the lock"),
        Waypoint(1.2, press, CLOSED, "press the lock in"),
    ]
    held = t_out - sum(w.duration for w in ws)
    ws += [
        Waypoint(max(0.5, held), press, CLOSED, "hold the lock in"),
        Waypoint(1.0, over, CLOSED, "let go of the lock"),
        Waypoint(1.0, (0.22, -0.30, TOP + 0.15), CLOSED, "clear"),
    ]
    return ws


def demo():
    return {"left": left_waypoints(), "right": right_waypoints()}
