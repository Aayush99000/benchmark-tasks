"""On the station table: the top of a round-front toilet bowl, and on it a front-lock raised toilet seat that has been
set on but not fitted. Fit it: push it back and straight so the two wings under its back go under the bowl's rear lip,
then, holding it there, turn the knob on its front clockwise until the bracket under its front is tight against the
front of the bowl, and let go of it.

The seat is a rigid body that rests on the bowl's rim. Its rear wings are part of it: two tabs that hang down inside
the bowl's opening at the back, each with a hook pointing backwards; pushed back, the hooks go under the rim's inner
edge (the lip). The bracket under its front is a model: the knob turns on a hinge in the seat, and each clockwise turn
drives the bracket 5 mm forwards through a self-locking screw; once the bracket reaches the inside of the bowl's front
wall it presses it like a stiff spring (with friction), which pushes the seat back, and the knob then takes the
screw's torque to turn. Once both grippers are clear of the seat, a 40 N push is put on its top, as someone leaning on
it would: a fitted, tight seat stays put; a loose one shifts.

Run it:  inspect-robots run --embodiment yam_arena -E task=raised_toilet_seat.py -E object_jitter=0 -E control_hz=20 \\
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
from fittingkit import Hinge, body, box, cyl, prism12  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

# The bowl: vitreous china in a biscuit colour (a standard toilet colour), so the white seat on it reads apart from it
# from the top camera
CHINA = (0.88, 0.82, 0.70)
BOWL_IN = (0.72, 0.67, 0.57)  # its inside, in shade
WATER = (0.36, 0.52, 0.60)
SEAT = (0.97, 0.97, 0.99)  # the seat: white molded plastic
SEAT_EDGE = (0.70, 0.72, 0.76)
KNOB = (0.45, 0.47, 0.50)  # grey
MARK = (0.10, 0.10, 0.10)


def rot_z(angle: float) -> np.ndarray:
    return axis_angle_matrix(Z_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


def _oval_point(a: float, b: float, t: float) -> np.ndarray:
    return np.array([a * math.cos(t), b * math.sin(t)])


def _oval_normal(a: float, b: float, t: float) -> np.ndarray:
    """The oval's outward unit normal at parameter angle ``t``."""
    n = np.array([b * math.cos(t), a * math.sin(t)])
    return n / np.linalg.norm(n)


def _oval_layer(z0, z1, edge, mid, outward: float, color, n: int, centre) -> list:
    """Boxes square to the ``edge`` oval (semi-axes), each reaching from it to just past the ``mid`` oval, ``outward``
    +1 (an inner edge, boxes reaching out) or -1 (an outer edge, boxes reaching in)."""
    cx, cy = centre
    out = []
    for k in range(n):
        ta, tb = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        t = (ta + tb) / 2
        p, nrm = _oval_point(*edge, t), _oval_normal(*edge, t)
        w = abs(float(np.dot(_oval_point(*mid, t) - p, nrm))) + 2.0 * MM
        chord = max(float(np.linalg.norm(_oval_point(*edge, tb) - _oval_point(*edge, ta))),
                    float(np.linalg.norm(_oval_point(*mid, tb) - _oval_point(*mid, ta)))) + 0.5 * MM
        c = p + outward * nrm * w / 2
        out.append(box((cx + c[0], cy + c[1], (z0 + z1) / 2), (w, chord, z1 - z0), color,
                       yaw=math.atan2(nrm[1], nrm[0])))
    return out


def oval_ring(z0: float, z1: float, a_in: float, b_in: float, a_out: float, b_out: float, color, n: int = 48,
              centre=(0.0, 0.0)) -> list:
    """A flat oval ring from height ``z0`` to ``z1``: inside the oval with semi-axes ``a_out`` (along x) and ``b_out``
    (along y), outside the one with ``a_in``, ``b_in``. Two layers of ``n`` boxes each: the inner one square to the
    inner oval, the outer one square to the outer oval, meeting (overlapping 2 mm) at the oval half way between, so
    both edges are smooth 2n-sided outlines within a fraction of a millimetre of the ovals.

    WHY two layers (2026-11-18, run rts1): one layer of boxes each spanning the ring, from the inner oval to the outer
    along the same parameter angle, is skewed to both ovals when they aren't the same shape: the seat's front came out
    as a staircase of fins that a fingertip caught in."""
    mid = ((a_in + a_out) / 2, (b_in + b_out) / 2)
    if min(a_in, b_in) < 0.002:  # a filled oval: the outer layer alone, reaching to the middle
        return _oval_layer(z0, z1, (a_out, b_out), (0.0, 0.0), -1.0, color, n, centre)
    return (_oval_layer(z0, z1, (a_in, b_in), mid, 1.0, color, n, centre)
            + _oval_layer(z0, z1, (a_out, b_out), mid, -1.0, color, n, centre))


def oval_x(a: float, b: float, y: float) -> float:
    """The oval's x (> 0) at ``y``."""
    return a * math.sqrt(max(0.0, 1.0 - (y / b) ** 2))


# -- the bowl (fixed): the top 10 cm of a round-front toilet bowl, its front towards the arms (-x) -------------------
# Sizes: a round-front bowl is about 16-1/2 in (419 mm) from its seat-bolt holes to the front of its rim and about 14 in
# (356 mm) wide (Bemis); the opening and the rim's section are [estimate].
BOWL = np.array([0.43, 0.0])  # the bowl's middle, on the table
RIM_H = 0.100  # the rim's top above the table
RIM_AT = np.array([BOWL[0], BOWL[1], TOP + RIM_H])
A_OUT, B_OUT = 0.215, 0.178  # the rim's outside: 430 x 356 mm
A_IN, B_IN = 0.145, 0.120  # the opening: 290 x 240 mm [estimate]
LIP_T = 25.0 * MM  # the rim's thickness at its inner edge: the "toilet lip" (Invacare: at most 1-3/4 in)
UNDER = 8.0 * MM  # under the lip, the bowl's inner wall is this far further out (the rim's undercut) [estimate]
UNDER_D = 45.0 * MM  # for this far down


def _bowl_shapes() -> tuple[list, list]:
    """In the bowl's frame (origin on the table under its middle; the world's axes)."""
    z_lip = RIM_H - LIP_T
    z_under = z_lip - UNDER_D
    solid = [
        *oval_ring(z_lip, RIM_H, A_IN, B_IN, A_OUT, B_OUT, CHINA),  # the rim
        *oval_ring(z_under, z_lip, A_IN + UNDER, B_IN + UNDER, A_OUT - 2 * MM, B_OUT - 2 * MM, CHINA, n=40),
        *oval_ring(0.0, z_under, A_IN - 0.020, B_IN - 0.020, A_OUT - 8 * MM, B_OUT - 8 * MM, CHINA, n=32),
    ]
    seen = [
        *oval_ring(z_under - 0.002, z_under, 0.0005, 0.0005, A_IN - 0.019, B_IN - 0.019, BOWL_IN, n=24),
        *oval_ring(z_under + 0.001, z_under + 0.003, 0.0005, 0.0005, 0.060, 0.050, WATER, n=16),
    ]
    return solid, seen


_BS, _BV = _bowl_shapes()
BOWL_BODY = body("bowl", pos=(BOWL[0], BOWL[1], TOP), friction=0.6, shapes=_BS + _BV,
                 visual_only=tuple(range(len(_BS), len(_BS) + len(_BV))), color=CHINA)


def lip_x(y: float) -> float:
    """The lip's inner edge (x from the bowl's middle, > 0 at the back) at ``y``."""
    return oval_x(A_IN, B_IN, y)


def front_wall_x(y: float) -> float:
    """The inside of the bowl's front wall under the lip (x from the bowl's middle, < 0) at ``y``."""
    return -oval_x(A_IN + UNDER, B_IN + UNDER, y)


# -- the seat (loose): a front-lock raised toilet seat -----------------------------------------------------------------
# Size and weight from a seat of the same front-lock kind, Drive's 12014: 17 in (W) x 16.5 in (D) x 4.5 in (H), 4 lb
# (Invacare publishes none for the 1391). The seat's hole and the wings are [estimate].
SEAT_A, SEAT_B = 0.2095, 0.216  # half its depth (x) and width (y): 419 x 432 mm
SEAT_H = 0.114  # 4.5 in
HOLE_A, HOLE_B = 0.130, 0.105  # the hole it is sat on: 260 x 210 mm
SEAT_MASS = 1.81  # 4 lb
# The rear wings: tabs 6 mm thick and 2 cm wide, 4 cm deep below the seat, where the lip's edge is at y = +-6 cm, each
# turned to face the edge there (square to its normal, 35 degrees off the x axis), with a hook 6 mm long pointing out
# under the lip at its foot. Fitted, a tab's back is 1 mm inside the lip's edge at its middle (0.6 mm at its corners,
# the edge curving away), and its hook is 5 mm under the lip.
# WHY turned (2026-11-18, runs rts1, rts2): square to x, a 3 cm tab's outer corner was 9 mm inside the rim where the
# edge curves at the back; PhysX pushed the seat 3.5 mm forward at spawn and the tabs jammed it there.
TAB_Y = 0.060
# WHY 33.5 mm deep: the hook's top 0.5 mm under the lip, snug; 7 mm under it, the seat's back lifted 2.2 mm as the
# bracket tightened (run rts4)
TAB_T, TAB_W, TAB_D = 6.0 * MM, 20.0 * MM, 33.5 * MM
HOOK_L, HOOK_T = 6.0 * MM, 8.0 * MM
TAB_GAP = 1.0 * MM  # a tab's back from the lip's edge, fitted
HOOKED = 3.0 * MM  # a hook is under the lip when its end is at least this far out past the lip's edge


def _tab_frame(s: float) -> tuple[np.ndarray, np.ndarray]:
    """For the wing on side ``s`` (+1: the robot's left): the lip's edge point it faces and the edge's outward normal
    there (the seat's frame, fitted)."""
    t = s * math.asin(TAB_Y / B_IN)
    return _oval_point(A_IN, B_IN, t), _oval_normal(A_IN, B_IN, t)


def past_oval(x: float, y: float, a: float, b: float) -> float:
    """About how far (m) the point is outside the oval with semi-axes ``a``, ``b`` (negative: inside); to a fraction of
    a millimetre within a centimetre of it."""
    f2 = (x / a) ** 2 + (y / b) ** 2
    g = 2.0 * math.hypot(x / a ** 2, y / b ** 2)
    return (f2 - 1.0) / g
# The front bracket (a model, drawn fixed): a plate hanging under the seat's front, inside the opening; its front face
# opened (screwed all the way back) 3 mm inside the lip's edge, so it passes the lip as the seat is put on.
BRACKET_X0 = -A_IN + 3.0 * MM
BRACKET_Z = -30.0 * MM  # its middle below the seat's bottom: 5 mm under the lip
PITCH = 5.0 * MM  # the bracket's travel per clockwise turn of the knob [estimate: a coarse self-locking thread]
# The knob: on the seat's front face, 5 cm up, turning about the seat's x axis
KNOB_R, KNOB_L = 20.0 * MM, 22.0 * MM
KNOB_LOCAL = np.array([-SEAT_A - KNOB_L / 2, 0.0, 0.050])
KNOB_MASS = 0.04

# The seat's frame: origin at its middle on its bottom face; fitted, that is the rim's middle on its top.
FIT_AT = RIM_AT.copy()
START_SHIFT = -8.0 * MM  # set on 8 mm forward of fitted,
START_TURN = math.radians(3.0)  # and turned 3 degrees (anticlockwise from above)
R_SEAT_START = rot_z(START_TURN)
SEAT_START = FIT_AT + np.array([START_SHIFT, 0.0, 0.0002])


def _seat_shapes() -> tuple[list, list]:
    solid = oval_ring(0.0, SEAT_H, HOLE_A, HOLE_B, SEAT_A, SEAT_B, SEAT, n=48)
    for s in (1, -1):
        edge, n = _tab_frame(s)
        yaw = math.atan2(n[1], n[0])
        back = edge - n * TAB_GAP
        c = back - n * TAB_T / 2
        solid.append(box((c[0], c[1], -TAB_D / 2), (TAB_T, TAB_W, TAB_D), SEAT_EDGE, yaw=yaw))
        c = back + n * HOOK_L / 2
        solid.append(box((c[0], c[1], -TAB_D + HOOK_T / 2), (HOOK_L, TAB_W, HOOK_T), SEAT_EDGE, yaw=yaw))
    seen = [
        box((BRACKET_X0 + 4.0 * MM, 0.0, BRACKET_Z), (8.0 * MM, 0.040, 0.020), KNOB),  # the bracket's plate
        box((BRACKET_X0 + 12.0 * MM, 0.0, -0.012), (6.0 * MM, 0.020, 0.026), KNOB),  # its stem
        box((-SEAT_A + 0.002, 0.0, KNOB_LOCAL[2]), (0.004, 0.050, 0.040), SEAT_EDGE),  # the knob's boss
        box((0.0, 0.0, SEAT_H + 0.0004), (0.004, 0.10, 0.0008), SEAT_EDGE),  # a moulding line on the top
    ]
    seen += _oval_layer(0.0, 0.006, (SEAT_A + 0.0008, SEAT_B + 0.0008), (SEAT_A - 0.004, SEAT_B - 0.004), -1.0,
                        SEAT_EDGE, 48, (0.0, 0.0))  # a grey seam along its bottom edge
    return solid, seen


_SS, _SV = _seat_shapes()
SEAT_BODY = body("seat", pos=tuple(SEAT_START), mass=SEAT_MASS, friction=0.8, shapes=_SS + _SV,
                 visual_only=tuple(range(len(_SS), len(_SS) + len(_SV))), com=(0.0, 0.0, 0.050),
                 quat=quat_xyzw(R_SEAT_START), inertia=(0.030, 0.028, 0.055), color=SEAT)

# The knob's own frame has its z axis along the seat's x (its turning axis), so it can be a 12-sided prism: a fluted
# knob, like the real one. WHY not a cylinder (2026-11-18, run rts3): the fingers slipped on a PhysX cylinder, and the
# knob turned only about 55 degrees for each 105 the wrist turned.
R_KNOB_IN_SEAT = axis_angle_matrix(Y_AXIS, math.pi / 2)  # the knob's z along the seat's x
KNOB_START = SEAT_START + R_SEAT_START @ KNOB_LOCAL
KNOB_BODY = body("knob", pos=tuple(KNOB_START), mass=KNOB_MASS, friction=1.0,
                 quat=quat_xyzw(R_SEAT_START @ R_KNOB_IN_SEAT), inertia=(6e-6, 6e-6, 8e-6), color=KNOB, shapes=[
                     *prism12((0.0, 0.0, 0.0), KNOB_R * math.cos(math.pi / 12), KNOB_L, KNOB),
                     box((KNOB_R * 0.55, 0.0, -KNOB_L / 2 - 0.0004), (KNOB_R * 0.7, 0.004, 0.0008), MARK),
                 ], visual_only=(6,))
fittingkit.add_hinge(Hinge("seat", "knob", anchor=tuple(KNOB_LOCAL), axis=(1.0, 0.0, 0.0), damping=0.02))

OBJECTS = [BOWL_BODY, SEAT_BODY, KNOB_BODY]

# -- the clamp and the push --------------------------------------------------------------------------------------------
K_BRACKET = 25000.0  # N/m: the bracket's give against the bowl [estimate]
# WHY damped this hard (about critical for the seat): at 150 N s/m a loose seat pushed into the wall at 0.5 m/s bounced
# 20 mm up off the bowl (probe rtsP2)
C_BRACKET = 400.0  # N s/m
BRACKET_MAX = 300.0  # N: the most the bracket bears [estimate: a plastic bracket]
# The bracket presses the bowl like a spring however hard it is pushed (it is a stop), but turning the knob takes no
# more torque past 100 N: past that its thread slips, as the manual warns ("DO NOT overtighten knob").
# WHY no cap on the force (2026-11-18, probe rtsP): capped at 100 N, a loose seat pushed forwards ran on 35 mm past
# the wall for a moment
PRESS_MAX = 4.0 * MM  # the knob's torque stops rising here (100 N)
MU_BRACKET = 0.5  # friction of the bracket's plate on the china
# Its friction is sticky: a stiff spring from where the plate first touched, slipping on once it would pull more than
# 0.5 times the bracket's force. WHY (run rts6): a friction that grew with the sliding speed let the 40 N push walk the
# tight seat 3 mm sideways and turn it 3.5 degrees
K_STICK, C_STICK = 40000.0, 200.0  # N/m, N s/m
EFFICIENCY = 0.25  # the screw's (self-locking) [estimate]
TIGHT_N = 50.0  # "securely fastened": the bracket pressing the bowl this hard [estimate: hand tight]
W_TURN = 0.2  # rad/s: the knob's friction torque reaches its full value by this speed
PUSH_N = 40.0  # the push once the hands are clear [estimate: someone leaning on the seat to stand]
PUSH_S = 1.0  # forwards, then to the robot's left, then to its right, this long each
HOLD_MM, HOLD_DEG = 3.0, 1.0  # holding: the seat moves less than this under the push
CLEAR = 0.02
TRACE_EVERY = 24  # substeps (0.1 s)


def seat_yaw(rot: np.ndarray) -> float:
    return math.atan2(rot[1, 0], rot[0, 0])


def seat_tilt_deg(rot: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


def knob_angle(seat_rot: np.ndarray, knob_rot: np.ndarray) -> float:
    """The knob's turn about the seat's x axis (rad, right hand about +x: clockwise seen from in front of the seat)."""
    rel = seat_rot.T @ knob_rot @ R_KNOB_IN_SEAT.T  # the knob's turn about the seat's x
    return math.atan2(rel[2, 1], rel[1, 1])


def bracket_face(turns: float) -> float:
    """The bracket's front face (seat frame x) after ``turns`` clockwise turns from opened."""
    return BRACKET_X0 - PITCH * max(turns, 0.0)


def bracket_press(seat_pos, seat_rot, turns: float) -> float:
    """How far (m) the bracket is pressed into the bowl's front wall (<= 0: not touching), the seat's origin at
    ``seat_pos`` (table coordinates)."""
    p = seat_pos + seat_rot @ np.array([bracket_face(turns), 0.0, BRACKET_Z]) - np.array([BOWL[0], BOWL[1], TOP])
    z_lip = RIM_H - LIP_T
    if not (z_lip - UNDER_D < p[2] < z_lip):
        return 0.0
    return float(front_wall_x(p[1]) - p[0])


def hooks_under_lip(seat_pos, seat_rot) -> bool:
    """Both wings' hooks reach at least 3 mm out under the lip, below it (fitted, 5 mm)."""
    for s in (1, -1):
        edge, n = _tab_frame(s)
        tip = edge + n * (HOOK_L - TAB_GAP)
        # WHY its middle height: its top is only 0.5 mm under the lip, so a seat pressed up against the lip by the
        # bracket (0.3 mm, run rts5) would count its hooks as out
        end = seat_pos + seat_rot @ np.array([tip[0], tip[1], -TAB_D + HOOK_T / 2])
        rel = end - np.array([BOWL[0], BOWL[1], TOP])
        if not (rel[2] < RIM_H - LIP_T and past_oval(rel[0], rel[1], A_IN, B_IN) >= HOOKED):
            return False
    return True


def fitted(seat_pos, seat_rot) -> bool:
    """The wings under the lip, the seat resting on the rim (its bottom within 2 mm of the rim's top), level (2
    degrees), straight (3 degrees) and centred (5 mm)."""
    rel = seat_pos - FIT_AT
    return (hooks_under_lip(seat_pos, seat_rot) and abs(rel[2]) < 2.0 * MM and seat_tilt_deg(seat_rot) < 2.0
            and abs(math.degrees(seat_yaw(seat_rot))) < 3.0 and abs(rel[1]) < 5.0 * MM)


class SeatClamp:
    """Runs before every physics substep (240 Hz): the knob's turns, the bracket's press on the bowl, and the push.

    The knob's turn on its hinge (from the seat's and the knob's poses) drives the bracket forwards 5 mm a turn. Once
    the bracket's face is past the inside of the bowl's front wall it presses it like a stiff spring (25,000 N/m, up
    to 100 N, damped), with friction across it: that pushes the seat back at the bracket, and the knob then takes the
    screw's torque to turn either way (a self-locking screw: the bracket never turns the knob back). Tight is the
    bracket pressing with 50 N or more. Once both grippers are 2 cm clear of the seat and knob and the seat is fitted,
    a 40 N push goes on its top, 1 s forwards, then 1 s to each side; it holds if the seat moves under 3 mm and tilts
    under 1 degree all the while."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.dt = 1.0 / 240.0
        self.log = os.environ.get("SEAT_LOG")
        self.trace = os.environ.get("SEAT_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["seat", "knob"]

    def clear(self) -> None:
        self._n = 0
        self.turns = 0.0  # the knob's turns
        self.screw = 0.0  # the turns the bracket has been driven: the knob's, frozen while no hand is on it
        self._last_angle = 0.0
        self.press = 0.0  # m
        self.force = 0.0  # N, the spring's part
        self.seat_pos = SEAT_START.copy()
        self.seat_rot = R_SEAT_START.copy()
        self.hands_off = True  # updated every 0.1 s in the sim; offline, no hands
        self.push_t = None  # s into the push, while it is on
        self.push_from = None  # the seat's pose when it began
        self.moved_mm = 0.0
        self.tilted_deg = 0.0
        self.held = False  # the last push ended with the seat held
        self.pushed = False  # a push has run to its end
        self.ever_tight = False
        self._stick = None  # where the bracket's plate is stuck on the bowl (world), while it presses
        self.poses = {}

    @property
    def tight(self) -> bool:
        return self.force >= TIGHT_N

    def update(self, frames: dict, acc: dict) -> None:
        seat, knob = frames.get("seat"), frames.get("knob")
        if seat is None or knob is None:
            return
        self._n += 1
        self.seat_pos, self.seat_rot = seat.pos - self.origin, seat.rot
        a = knob_angle(seat.rot, knob.rot)
        self.turns += ((a - self._last_angle + math.pi) % (2 * math.pi) - math.pi) / (2 * math.pi)
        self._last_angle = a
        # WHY frozen with no hand on it: a self-locking screw doesn't turn by itself, but in the sim the loaded knob
        # crept back 0.1 turn in 8 s once let go (run rts4)
        if not self.hands_off:
            self.screw = self.turns
        axis = seat.rot[:, 0]
        w_rel = float(np.dot(knob.spin - seat.spin, axis))
        self.press = bracket_press(self.seat_pos, seat.rot, self.screw)
        self.force = 0.0
        if self.press > 0.0:
            at = seat.world(np.array([bracket_face(self.screw), 0.0, BRACKET_Z]))
            v = seat.point_vel(at)
            spring = K_BRACKET * self.press
            normal = min(max(spring - C_BRACKET * float(v[0]), 0.0), BRACKET_MAX)
            self.force = spring
            f = np.array([normal, 0.0, 0.0]) + self._friction(at, v, normal)
            fittingkit._wrench(acc, "seat", seat, f, at)
            torque = (K_BRACKET * min(self.press, PRESS_MAX) * PITCH / (2 * math.pi * EFFICIENCY)
                      * float(np.clip(w_rel / W_TURN, -1.0, 1.0)))
            fittingkit._torque(acc, "knob", -torque * axis)
            fittingkit._torque(acc, "seat", torque * axis)
            if self.tight and not self.ever_tight:
                self.ever_tight = True
                self._note(f"{self._n / 240:7.2f}s tight: {self.turns:5.2f} turns, {spring:5.1f} N")
        else:
            self._stick = None
        if self._n % TRACE_EVERY == 0:
            if fittingkit._ENV["env"] is not None:
                self.hands_off = hands_clear(self.seat_pos, seat.rot, knob.pos - self.origin, knob.rot, self.origin)
            if self.trace:
                self._write_trace()
        self._push(seat, acc)

    def _friction(self, at, v, normal: float) -> np.ndarray:
        """The bracket plate's friction on the bowl's wall (world, across the wall): a spring from where it stuck,
        slipping once past 0.5 times ``normal``."""
        if self._stick is None:
            self._stick = at.copy()
        d = at - self._stick
        d[0] = 0.0
        f = -K_STICK * d - C_STICK * np.array([0.0, v[1], v[2]])
        limit = MU_BRACKET * normal
        mag = float(np.linalg.norm(f))
        if mag > limit:
            f *= limit / mag
            dn = float(np.linalg.norm(d))
            if dn > 1e-9:  # slipping: the stuck point follows, so the spring pulls with the limit
                self._stick = at - d * (limit / K_STICK) / dn
                self._stick[0] = at[0]
        return f

    def _push(self, seat, acc) -> None:
        if self.push_t is None:
            if self.hands_off and fitted(self.seat_pos, self.seat_rot):
                self.push_t, self.push_from = 0.0, (self.seat_pos.copy(), self.seat_rot.copy())
                self.moved_mm = self.tilted_deg = 0.0
                self.pushed = self.held = False
                self._note(f"{self._n / 240:7.2f}s push on ({self.force:5.1f} N at the bracket)")
            return
        if not self.hands_off:
            self.push_t = None
            self._note(f"{self._n / 240:7.2f}s push off: a hand came back")
            return
        if self.pushed:
            return  # done; it runs again once a hand has come back and gone
        p0, r0 = self.push_from
        self.moved_mm = max(self.moved_mm, 1000 * float(np.linalg.norm(self.seat_pos - p0)))
        rel = r0.T @ self.seat_rot
        self.tilted_deg = max(self.tilted_deg, math.degrees(math.acos(float(np.clip((np.trace(rel) - 1) / 2, -1, 1)))))
        phase = int(self.push_t // PUSH_S)
        if phase < 3:
            d = (-X_AXIS, Y_AXIS, -Y_AXIS)[phase]
            # WHY a smooth rise and fall, at half the seat's height: switched on at once at its top, 40 N threw a loose
            # seat 20 mm forwards and 21 mm up for a moment (probe rtsP3); a person's lean builds up
            ramp = math.sin(math.pi * (self.push_t % PUSH_S) / PUSH_S) ** 2
            fittingkit._wrench(acc, "seat", seat, PUSH_N * ramp * d, seat.world(np.array([-0.10, 0.0, SEAT_H / 2])))
            self.push_t += self.dt
            return
        self.held = self.moved_mm < HOLD_MM and self.tilted_deg < HOLD_DEG
        self.pushed = True
        self._note(f"{self._n / 240:7.2f}s push done: moved {self.moved_mm:4.1f} mm, turned {self.tilted_deg:4.2f} deg, "
                   f"{'held' if self.held else 'NOT held'}")

    def _write_trace(self) -> None:
        """Debug only (SEAT_TRACE=<file>): every 0.1 s, the seat's pose, the knob, the bracket and both TCPs."""
        p = 1000 * (self.seat_pos - FIT_AT)
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s seat ({p[0]:6.1f},{p[1]:6.1f},{p[2]:6.1f}) yaw "
                     f"{math.degrees(seat_yaw(self.seat_rot)):5.2f} tilt {seat_tilt_deg(self.seat_rot):4.2f} hooks "
                     f"{int(hooks_under_lip(self.seat_pos, self.seat_rot))} | turns {self.turns:5.2f} press "
                     f"{1000 * self.press:5.1f} F {self.force:5.1f} | off {int(self.hands_off)} push "
                     f"{-1 if self.push_t is None else min(self.push_t, 9.9):4.1f} moved {self.moved_mm:4.1f} |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


def near_parts(pts, seat_pos, seat_rot, knob_pos, knob_rot) -> np.ndarray:
    """Which of ``pts`` (table coordinates) are within 2 cm of the seat (its outline, from under its wings to its
    top) or of the knob's surface."""
    local = (pts - seat_pos) @ seat_rot
    near = ((local[:, 0] / (SEAT_A + CLEAR)) ** 2 + (local[:, 1] / (SEAT_B + CLEAR)) ** 2 <= 1.0)
    near &= (local[:, 2] > -TAB_D - CLEAR) & (local[:, 2] < SEAT_H + CLEAR)
    k = (pts - knob_pos) @ knob_rot  # the knob's frame: its axis is z
    axial = np.maximum(np.abs(k[:, 2]) - KNOB_L / 2, 0.0)
    radial = np.maximum(np.hypot(k[:, 0], k[:, 1]) - KNOB_R, 0.0)
    return near | (np.hypot(axial, radial) < CLEAR)


def hands_clear(seat_pos, seat_rot, knob_pos, knob_rot, origin) -> bool:
    """No gripper point within 2 cm of the seat or the knob; unknown (offline) counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0:
        return True
    return not bool(np.any(near_parts(pts - origin, seat_pos, seat_rot, knob_pos, knob_rot)))


MODEL = fittingkit.add_model(SeatClamp())

# -- scoring -----------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def is_fitted() -> bool:
    return fitted(MODEL.seat_pos, MODEL.seat_rot)


def success(objects) -> bool:
    """Fitted, tight, and held under the push with both grippers clear. Must hold 1 s."""
    return is_fitted() and MODEL.tight and MODEL.pushed and MODEL.held and MODEL.hands_off


def score(objects) -> float:
    """0.9 for the seat fitted, tight and held under the push (1.0 once success() has held 1 s); 0.5 for fitted and
    tight; 0.25 for fitted; 0 otherwise."""
    if not is_fitted():
        return 0.0
    if not MODEL.tight:
        return 0.25
    return COMPLETE_SCORE if (MODEL.pushed and MODEL.held) else 0.5


# -- the scripted demo -------------------------------------------------------------------------------------------------
# The left hand, fingers closed and pointing forwards, comes in front of the seat's front face left of the knob and
# pushes the seat back with its fingertips, so the seat turns straight and goes back until its wings' hooks are under
# the bowl's lip; it holds it there. The right hand, pointing forwards (+x) with its fingers closing up and down,
# comes onto the knob and turns it clockwise 105 degrees at a time: it closes on the knob, turns, opens a little, turns
# back and closes again, ten times. Then both hands move clear and wait for the push.
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


def r_forward(roll_deg: float) -> np.ndarray:
    """The right TCP pointing forwards (+x), its fingers closing up and down (its x up) at no roll; rolled
    ``roll_deg`` about +x (clockwise seen from behind the hand, as the knob tightens)."""
    r0 = np.column_stack([Z_AXIS, np.cross(X_AXIS, Z_AXIS), X_AXIS])
    return axis_angle_matrix(X_AXIS, math.radians(roll_deg)) @ r0


# The left hand pushes the seat's front face with its closed fingertips, pointing forwards like the right, its fingers
# closing up and down. WHY not pointing down: the hand is 10 cm wide 4 cm above its fingertips, so pointing down at the
# 11.4 cm tall face its palm went 10 mm into the seat's top (plan_check)
PUSH_Y = 0.120  # the left fingertips push the seat's front face this far to the robot's left of its middle
PUSH_Z = 0.055  # at this height above the rim
# WHY 25 mm: the arm gives about 550 N/m at the fingertips, and sliding the 1.8 kg seat on the china takes about 12 N
PUSH_PAST = 20.0 * MM  # aimed this far past the face's fitted line [estimate]
# WHY ease off: held at 25 mm past once the seat was home, the push went on turning it, to 6.9 degrees (run rts3)
EASE_PAST = 5.0 * MM  # then this far past, holding it there lightly
GRIP_ON = 14.0 * MM  # the right fingertips this far onto the knob from its front
ROLL_FROM, ROLL_TO = -10.0, 95.0  # each stroke turns the knob this far (105 degrees)
STROKES = 12  # 1,260 degrees: 3.5 turns, past tight (2.7) with room for the knob slipping in the fingers
AJAR = -0.026  # between strokes the right fingers open to 26 mm from closed: 6 mm clear of the 40 mm knob
# WHY the finger moves take 1 s or more: the step guard lets a gripper move 5% of its travel a step (1 a second)


R_PUSH = r_forward(0.0)  # the left hand's orientation


def seat_front_x(y: float, shift: float = 0.0) -> float:
    """The seat's front face (world x) at ``y``, fitted and moved ``shift`` back."""
    return FIT_AT[0] - oval_x(SEAT_A, SEAT_B, y) + shift


def knob_tcp() -> np.ndarray:
    """The right TCP on the knob's axis, its fingertips 14 mm onto it, the seat fitted."""
    face = FIT_AT + KNOB_LOCAL + np.array([-KNOB_L / 2, 0.0, 0.0])
    return face + np.array([GRIP_ON - PAD_TIP, 0.0, 0.0])


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    z = FIT_AT[2] + PUSH_Z
    face_start = FIT_AT[0] + START_SHIFT - (oval_x(SEAT_A, SEAT_B, PUSH_Y) * math.cos(START_TURN)
                                             + PUSH_Y * math.sin(START_TURN))
    l_tcp = lambda tip_x, dy=0.0, dz=0.0: (np.array([tip_x - PAD_TIP, PUSH_Y + dy, z + dz]), R_PUSH)  # noqa: E731
    l_over = l_tcp(face_start - 0.030)
    l_down = l_tcp(face_start - 0.002)
    l_push = l_tcp(seat_front_x(PUSH_Y) + PUSH_PAST)
    # WHY only 4 cm back: further back the left arm is 6 mm off its pose (probe)
    l_away = l_tcp(seat_front_x(PUSH_Y) - 0.040, dy=0.020, dz=0.010)
    k = knob_tcp()
    # WHY 2 cm back: straight over the knob, the closed fingertips touched the seat's top front edge before it was pushed back
    r_wait = (k + np.array([-0.02, 0.0, 0.05]), r_forward(ROLL_FROM))
    r_front = (k + np.array([-0.03, 0.0, 0.0]), r_forward(ROLL_FROM))
    r_clear = (k + np.array([-0.03, 0.0, 0.07]), r_forward(ROLL_FROM))
    r_up = (k + np.array([0.0, 0.0, 0.10]), r_forward(ROLL_FROM))
    # WHY the right fingers stay closed until the hand is in front of the knob: waiting above it open, the lower
    # finger hung onto the knob as the seat came back (plan_check)
    # WHY up first: straight from the ready pose to the wait, the right fingers dipped 7 mm into the seat's top front edge
    # as the hand turned to point forwards (plan_check)
    both(2.0, r_up, CLOSED, l_over, CLOSED, "hands up")
    both(2.0, r_wait, CLOSED, l_over, CLOSED, "in front of the seat")
    both(1.5, r_wait, CLOSED, l_down, CLOSED, "fingertips to its face")
    both(2.5, r_wait, CLOSED, l_push, CLOSED, "push it back and straight")
    l_push = l_tcp(seat_front_x(PUSH_Y) + EASE_PAST)
    both(1.0, r_wait, CLOSED, l_push, CLOSED, "ease off")
    both(1.5, r_front, CLOSED, l_push, CLOSED, "in front of the knob")
    both(1.8, r_front, OPEN, l_push, CLOSED, "open the fingers")
    both(1.2, (k, r_forward(ROLL_FROM)), OPEN, l_push, CLOSED, "onto the knob")
    for i in range(STROKES):
        n = f" {i + 1}"
        both(1.8 if i == 0 else 1.0, (k, r_forward(ROLL_FROM)), CLOSED, l_push, CLOSED, "close on the knob" + n)
        both(1.2, (k, r_forward(ROLL_TO)), CLOSED, l_push, CLOSED, "turn it clockwise" + n)
        both(1.0, (k, r_forward(ROLL_TO)), AJAR, l_push, CLOSED, "let it go" + n)
        both(0.8, (k, r_forward(ROLL_FROM)), AJAR, l_push, CLOSED, "turn back" + n)
    both(1.2, r_front, OPEN, l_push, CLOSED, "off the knob")
    both(2.0, r_clear, CLOSED, l_away, CLOSED, "hands clear")
    both(5.0, r_clear, CLOSED, l_away, CLOSED, "wait for the push")
    return {"left": left, "right": right}
