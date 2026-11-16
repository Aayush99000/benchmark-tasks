"""On the station table, a little left of centre: a steel bracket with a narrow upright slot. At the bottom of the slot
an M8 bolt sticks out of the bracket's back plate towards the robot's left arm, with a steel nylon-insert locknut
(13 mm across the flats) on it, half a turn short of seated: a white paint line on the nut's face points up, and a
red mark on the plate below the nut shows where it points once seated. The slot's two walls leave a wrench's handle
only about 44 degrees of swing, pointing up. Further left, a 10 x 13 mm double open-end wrench stands head down in a
wooden rack, its 13 mm head in the rack's pocket and its handle straight up. Tighten the nut with the wrench.

Run it:  inspect-robots run --embodiment yam_arena -E task=wrench_flip.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Tighten the nut with the wrench." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import _torque, _wrench, body, box, cyl, matrix_to_quat_rel, ring  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
STEEL = (0.62, 0.63, 0.65)
ZINC = (0.78, 0.79, 0.80)
CHROME = (0.82, 0.83, 0.85)
WOOD = (0.55, 0.40, 0.25)
WHITE = (0.95, 0.95, 0.93)
RED = (0.75, 0.10, 0.10)
DARK = (0.10, 0.10, 0.11)
fittingkit.METALLIC.update({CHROME: 0.12, ZINC: 0.3})

# -- the frame of the joint -------------------------------------------------------------------------------------------
# The bolt's axis B is level, pointing out of the bracket towards the left arm; H is level across the slot; Z is up.
# The wrench swings in the plane of H and Z. Angles in that plane are measured from Z, positive turning about +B
# (from Z towards H); tightening (clockwise, seen from the end of the bolt) is negative.
# WHY the bolt points at the left arm, 30 degrees off -x: the wrench is held by a straight-down grip with its fingers
# closing along B, and turned over by turning that grip half round (yaw -120 to +60 degrees): both are inside the left
# wrist's range over the slot (yam_arena IK, 2026-11-15), so the demo needs no other grip.
BOLT_YAW = math.radians(150.0)
B = np.array([math.cos(BOLT_YAW), math.sin(BOLT_YAW), 0.0])
Z = np.array([0.0, 0.0, 1.0])
H = np.cross(B, Z)
NUT_XY = (0.28, 0.12)
NUT_H = 45.0 * MM  # the bolt's axis above the table
N = np.array([*NUT_XY, TOP + NUT_H])  # the nut's centre (world, m)


def in_plane(deg: float) -> np.ndarray:
    """The unit vector in the wrench's plane at ``deg`` from straight up (positive towards H)."""
    a = math.radians(deg)
    return math.cos(a) * Z + math.sin(a) * H


# -- the bracket (fixed) ----------------------------------------------------------------------------------------------
# Its own frame: origin on the table under the nut, x along B, y along -H (the body is turned 150 degrees), z up.
NUT_LEN, NUT_AF = 8.0 * MM, 13.0 * MM  # M8 nylon-insert locknut (DIN 985): 13 mm across the flats
PLATE_FACE = -5.6 * MM  # the back plate's face behind the nut (x), a 1.6 mm washer between them
SLOT_HALF = 32.7 * MM  # the slot's walls' inner faces from the bolt's axis
WALL_TOP = NUT_H + 70.0 * MM
BASE_TOP = NUT_H - 25.0 * MM  # the slot's floor
WALL_FRONT = 20.0 * MM  # the walls and the base reach this far out along B
# The handle is 14 mm wide: at the walls' top (70 mm out from the bolt) it touches a wall at
# 70 sin(a) + 7 cos(a) = 32.7 mm, so a = 22 degrees each way of straight up.
# WHY 22 and not 20: at 20, the strokes with the wrench face up went on at 13-14 degrees, and with the wrench a few mm off
# in the gripper after the first strokes, the handle caught the wall's front edge going in and the stroke missed
# (official sim trace, run wf22, 2026-11-16). At 22 every stroke goes on at 11. WHY not more: the jaw goes on up to 7
# degrees off a fit and the nut keeps that offset, so in the model one face alone tightened it fully with 24 each way,
# and stopped at 39 degrees with 23 (checked 2026-11-16 with the one-face strategy of the tests).
SWING_DEG = 22.0

# The paint line's start angle (nearly straight up). WHY -4: then the first stroke goes on at 11 degrees, like the rest.
# Starting at +3, the first stroke went on at 18 degrees, where sliding the jaw on pushed the handle into the slot's wall
# and the stroke failed (official sim trace, 2026-11-16, with the wall at 20).
NU0_DEG = -4.0
SEAT_DEG = 180.0  # tightening this far from the start seats it (on the stop)
LOOSEN_DEG = 30.0  # it can be backed off this far past its start


def _bracket_shapes() -> tuple[list, list]:
    z0 = NUT_H
    depth = WALL_FRONT - PLATE_FACE + 10.0 * MM  # walls from the plate's back to their front
    wall_x = (WALL_FRONT + PLATE_FACE - 10.0 * MM) / 2
    solid = [
        box((wall_x, 0.0, BASE_TOP / 2), (depth, 2 * (SLOT_HALF + 10.0 * MM), BASE_TOP), STEEL),  # the base
        box((PLATE_FACE - 5.0 * MM, 0.0, (BASE_TOP + z0 + 40.0 * MM) / 2),  # the back plate, to 4 cm over the bolt
            (10.0 * MM, 2 * SLOT_HALF, z0 + 40.0 * MM - BASE_TOP), STEEL),
        *[box((wall_x, sy * (SLOT_HALF + 5.0 * MM), (BASE_TOP + WALL_TOP) / 2),
              (depth, 10.0 * MM, WALL_TOP - BASE_TOP), STEEL) for sy in (1, -1)],  # the slot's walls
    ]
    seen = [
        cyl((PLATE_FACE + 0.8 * MM, 0.0, z0), 8.0 * MM, 1.6 * MM, ZINC, axis="X"),  # the washer
        cyl((PLATE_FACE + 9.8 * MM, 0.0, z0), 4.0 * MM, 19.6 * MM, ZINC, axis="X"),  # the stud, 10 mm past the nut
        # the seated mark: a red line on the plate below the nut, where the nut's paint line points once seated
        # (the bracket's own y is -H)
        box((PLATE_FACE + 0.05 * MM, -13.5 * MM * math.sin(math.radians(NU0_DEG - SEAT_DEG)),
             z0 + 13.5 * MM * math.cos(math.radians(NU0_DEG - SEAT_DEG))), (0.1 * MM, 1.6 * MM, 6.0 * MM), RED),
    ]
    return solid, seen


_BS, _BV = _bracket_shapes()
BRACKET = body("bracket", pos=(*NUT_XY, TOP), yaw=BOLT_YAW, friction=0.6, shapes=_BS + _BV,
               visual_only=tuple(range(len(_BS), len(_BS) + len(_BV))))

# -- the nut: turned only by the wrench model below (it is posed every physics substep) -------------------------------
# Its frame: origin at its centre, z along B, x the direction its paint line points (the normal of one flat).
# WHY round for collisions: as in the accepted compression-joint task, the hexagon is drawn but the nut collides as a
# round core inside its flats, so a jaw that the model holds on it never fights the contact solver.


def hexagon(across_flats, length, color) -> list:
    """A hexagonal prism along the body's z, as three boxes turned 60 degrees apart (two flats facing +-x)."""
    side = across_flats / math.sqrt(3)
    return [box((0.0, 0.0, 0.0), (across_flats, side, length), color, yaw=k * math.pi / 3) for k in range(3)]


# WHY a 4.3 mm core (just round the 4 mm stud): at 6.2 mm, then 5 mm, the jaw's prongs (6.9 mm from its middle) landed
# on it with the hand 2-4 mm off across the mouth, and the jaw stopped short of the nut (official sim, 2026-11-16).
NUT_CORE = 4.3 * MM


def _nut_shapes() -> tuple[list, list]:
    solid = ring(-NUT_LEN / 2, NUT_LEN / 2, 3.0 * MM, NUT_CORE, ZINC, segments=12)
    seen = [*hexagon(NUT_AF, NUT_LEN, ZINC),
            box((3.3 * MM, 0.0, NUT_LEN / 2 + 0.05 * MM), (6.0 * MM, 1.6 * MM, 0.1 * MM), WHITE),  # the paint line
            box((NUT_AF / 2 + 0.05 * MM, 0.0, 0.0), (0.1 * MM, 1.6 * MM, NUT_LEN), WHITE)]  # and down its flat
    return solid, seen


def nut_rotation(nu_deg: float) -> np.ndarray:
    """The nut's rotation with its paint line at ``nu_deg`` in the plane."""
    x = in_plane(nu_deg)
    return np.column_stack([x, np.cross(B, x), B])


_NS, _NV = _nut_shapes()
NUT_CENTRE = N + (PLATE_FACE + 1.6 * MM + NUT_LEN / 2) * B  # just off the washer
NUT = body("nut", pos=tuple(NUT_CENTRE), quat=matrix_to_quat_rel(nut_rotation(NU0_DEG)), mass=0.02, friction=0.6,
           shapes=_NS + _NV, visual_only=tuple(range(len(_NS), len(_NS) + len(_NV))), solver_iterations=32)
fittingkit.no_gravity("nut")

# -- the wrench: a 10 x 13 mm double open-end wrench, both heads angled 15 degrees -----------------------------------
# Its frame: origin at the middle of the 13 mm jaw's seat (where the nut's centre sits when the jaw is on it), x along
# the handle towards the 10 mm end, z through its face. The 13 mm jaw opens away from the handle, turned +15 degrees
# about z: its mouth points along (-cos 15, -sin 15, 0).
WR_T = 5.0 * MM  # handle thickness
HEAD_T = 5.5 * MM
HEAD_ANGLE = 15.0  # degrees
JAW_13 = 13.8 * MM  # opening (a real 13 mm jaw is about 13.2: wider here so the rigid jaw never binds)
GRIP = 110.0 * MM  # the demo's grip on the handle, this far from the seat


def _rot(c, deg):
    a = math.radians(deg)
    return (c[0] * math.cos(a) - c[1] * math.sin(a), c[0] * math.sin(a) + c[1] * math.cos(a), c[2])


def _head(origin_x, deg, opening, back_x, depth, width, prong_w):
    """An open-end head: a back behind the seat and two prongs either side of the opening, in a frame turned ``deg``
    whose -x is the mouth. ``back_x``: the back's inner face; ``depth``: the prongs' length from the mouth."""
    half = opening / 2
    back_len = 5.5 * MM
    parts = [((back_x + back_len / 2, 0.0, 0.0), (back_len, width, HEAD_T))]
    for sy in (1, -1):
        parts.append(((back_x - depth / 2, sy * (half + prong_w / 2), 0.0), (depth, prong_w, HEAD_T)))
    out = []
    for c, s in parts:
        rc = _rot(c, deg)
        out.append(box((origin_x + rc[0], rc[1], rc[2]), s, CHROME, yaw=math.radians(deg)))
    return out


def _wrench_shapes() -> tuple[list, list]:
    solid = [
        box((70.0 * MM, 0.0, 0.0), (100.0 * MM, 14.0 * MM, WR_T), CHROME),  # the handle
        box((17.0 * MM, 0.0, 0.0), (10.0 * MM, 18.0 * MM, HEAD_T), CHROME),  # the 13 mm head's neck
        box((124.0 * MM, 0.0, 0.0), (8.0 * MM, 15.0 * MM, HEAD_T), CHROME),  # the 10 mm head's neck
        # the 13 mm head: the back just clears the hex's corner (7.5 mm from its centre)
        *_head(0.0, HEAD_ANGLE, JAW_13, 7.8 * MM, 18.6 * MM, 30.0 * MM, 9.0 * MM),
        # the 10 mm head at the far end, its mouth pointing the other way
        *_head(134.0 * MM, 180.0 + HEAD_ANGLE, 10.2 * MM, 6.0 * MM, 14.0 * MM, 24.0 * MM, 7.0 * MM),
    ]
    seen = [box((40.0 * MM, 0.0, s * (WR_T / 2 + 0.05 * MM)), (12.0 * MM, 6.0 * MM, 0.1 * MM), DARK)
            for s in (1, -1)]  # "13" stamped on both faces
    return solid, seen


def wrench_rotation(face: int, deg: float = 0.0) -> np.ndarray:
    """The wrench's rotation with its handle at ``deg`` in the plane and its face +B (``face`` +1) or -B (-1)."""
    x = in_plane(deg)
    z = face * B
    return np.column_stack([x, np.cross(z, x), z])


# -- the rack (fixed): the wrench stands head down in its pocket ---------------------------------------------------
RACK_XY = (0.20, 0.26)
RACK_H, POCKET_DEPTH = 42.0 * MM, 32.0 * MM
# WHY 32 mm deep: in a 20 mm pocket the head stood on one prong tip with its upper corners above the rim, and the
# wrench leaned over 39 degrees while the scene settled (official sim, 2026-11-16); the whole head (to 13.3 mm above
# its seat, 29.4 mm above the floor) now sits inside.
POCKET_B, POCKET_H = 7.0 * MM, 37.0 * MM  # the pocket along B (round the 5.5 mm head) and across (round its 36 mm)
RACK_B, RACK_HW = 50.0 * MM, 70.0 * MM
# The head's lowest point, standing straight up: one prong's tip, 14.55 mm below the seat and 12.6 mm to the +H side.
# The other prong's lowest point (its inner corner, 8.64 mm below the seat, 9.5 mm to the -H side) is 5.9 mm higher.
# WHY a ledge under it: on a flat floor the head stood on one tip and the wrench toppled in its plane until it jammed,
# leaning 30-39 degrees (official sim, 2026-11-16); on the tip and the ledge its weight is between its supports.
HEAD_BELOW_SEAT = 14.55 * MM
LEDGE_UP = (14.55 - 8.64) * MM  # the ledge's top above the pocket's floor
LEDGE_FROM = 6.0 * MM  # it runs from 6 mm on the -H side of the seat to the pocket's end (the corner is at 9.5)
SEAT0 = np.array([*RACK_XY, TOP + RACK_H - POCKET_DEPTH + HEAD_BELOW_SEAT + 0.5 * MM])


def _rack_shapes() -> list:
    floor = RACK_H - POCKET_DEPTH
    side_b = (RACK_B - POCKET_B) / 2
    side_h = (RACK_HW - POCKET_H) / 2
    return [
        box((0.0, 0.0, floor / 2), (RACK_B, RACK_HW, floor), WOOD),
        *[box((s * (POCKET_B + side_b) / 2, 0.0, (floor + RACK_H) / 2), (side_b, RACK_HW, POCKET_DEPTH), WOOD)
          for s in (1, -1)],
        *[box((0.0, s * (POCKET_H + side_h) / 2, (floor + RACK_H) / 2), (POCKET_B, side_h, POCKET_DEPTH), WOOD)
          for s in (1, -1)],
        # the ledge under the -H prong (the rack's own y is -H)
        box((0.0, (LEDGE_FROM + POCKET_H / 2) / 2, floor + LEDGE_UP / 2), (POCKET_B, POCKET_H / 2 - LEDGE_FROM, LEDGE_UP),
            WOOD),
    ]


RACK = body("rack", pos=(*RACK_XY, TOP), yaw=BOLT_YAW, friction=0.6, shapes=_rack_shapes())
_WS, _WV = _wrench_shapes()
WRENCH = body("wrench", pos=tuple(SEAT0), quat=matrix_to_quat_rel(wrench_rotation(+1)), mass=0.07,
              com=(70.0 * MM, 0.0, 0.0), friction=1.0, shapes=_WS + _WV,
              visual_only=tuple(range(len(_WS), len(_WS) + len(_WV))), solver_iterations=32)

OBJECTS = [BRACKET, NUT, RACK, WRENCH]
fittingkit.register(*OBJECTS)


# -- the wrench on the nut: our own model ------------------------------------------------------------------------------
def _wrap(a: float, period: float) -> float:
    return (a + period / 2) % period - period / 2


class WrenchOnNut:
    """The 13 mm jaw on the nut's flats, run before every physics substep (240 Hz).

    The jaw is on when the wrench lies in the nut's plane (its face within ``plane_tol`` of the bolt's axis, either
    face out), within ``axial`` of the nut's middle along the bolt, the nut between its prongs (at most ``across`` off
    its middle across the mouth, and from ``deep`` past its seat to ``mouth`` out towards the mouth), and its jaw lined
    up with a pair of flats within ``align_tol`` (the flats repeat every 60 degrees). Its jaw is turned 15 degrees to
    the handle, one way with one face out and the other way with the wrench turned over, so turning it over moves the
    angle at which the handle fits the flats by 30 degrees.

    While on, the jaw is held across the mouth on the nut, drawn onto it along the mouth by at most ``pull``, and the
    nut turns with it,
    against     ``drag`` (the nylon insert); it stops ``seat`` degrees tightened (seated) and ``loosen`` degrees backed off. Slid
    ``release`` out towards its mouth (the nut nearly out of it), or off the nut along the bolt, it comes off."""

    # WHY the nut may sit up to 8 mm out in the mouth: the prongs are 18.6 mm deep, so the nut's flats are between them
    # from its seat to well out; across the mouth only 3 mm (the prongs clear the nut's core by 2.6). A round 4.5 mm
    # capture made the jaw go on only once the hand came within it, late or not at all (official sim, 2026-11-16).
    # WHY a light pull (at most ``pull`` N) into the mouth while on: free along the mouth, the jaw drifted off mid-stroke
    # (official sim, 2026-11-16); a hand sliding it off on purpose overcomes it.
    def __init__(self, wrench, nut, centre, nu0=NU0_DEG, seat=SEAT_DEG, loosen=LOOSEN_DEG, plane_tol=12.0,
                 across=3.0e-3, deep=1.5e-3, mouth=8.0e-3, axial=4.5e-3, align_tol=7.0, release=10.0e-3, drag=0.5,
                 k_hold=800.0, k_stop=2.0, pull=1.5):
        self.wrench, self.nut = wrench, nut
        self.centre = np.asarray(centre, float)
        self.nu0 = math.radians(nu0)
        self.lo, self.hi = self.nu0 - math.radians(seat), self.nu0 + math.radians(loosen)
        self.plane_tol, self.axial = math.radians(plane_tol), axial
        self.across, self.deep, self.mouth = across, deep, mouth
        self.align_tol, self.release, self.drag = math.radians(align_tol), release, drag
        self.k_hold, self.k_stop, self.pull = k_hold, k_stop, pull
        self.origin = np.zeros(3)
        self.log = os.environ.get("WRENCH_LOG")  # a file to log every engagement and release to (sim traces)
        self._n = 0
        self.poses = {}
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return [self.wrench, self.nut]

    def clear(self) -> None:
        self._n = 0
        self.nu = self.nu0
        self.engaged = False
        self.face = 0
        self._prev = 0.0
        self.engagements = 0
        self.faces_used: set[int] = set()
        self._pose_nut()

    def tightened_deg(self) -> float:
        return math.degrees(self.nu0 - self.nu) + 1e-9  # (so a set angle is not read a hair short)

    def seated(self) -> bool:
        return self.tightened_deg() >= SEAT_DEG - 2.0

    def jaw_state(self, pos, rot):
        """(handle angle, face, misalignment to the flats, offset of the seat from the nut's centre), all in the
        nut's plane; angles in radians."""
        x, z = rot[:, 0], rot[:, 2]
        face = 1 if float(z @ B) >= 0.0 else -1
        handle = math.atan2(float(x @ H), float(x @ Z))
        mouth = handle + math.pi + face * math.radians(HEAD_ANGLE)  # the jaw's mouth's direction in the plane
        # the jaw's faces are square to its mouth; a pair of flats is square to the paint line (nu) plus k 60 degrees
        mis = _wrap(mouth + math.pi / 2 - self.nu, math.pi / 3)
        d = np.asarray(pos, float) - (self.centre + self.origin)
        return handle, face, mis, d

    def _event(self, what, handle, face, mis, dp, along) -> None:
        if self.log:
            with open(self.log, "a") as f:
                f.write(f"{self._n / 240.0:7.2f}s {what:3s} face {face:+d} handle {math.degrees(handle):6.1f} "
                        f"mis {math.degrees(mis):5.1f} off {1000 * float(np.linalg.norm(dp)):4.1f} mm "
                        f"along {1000 * along:4.1f} mm tightened {self.tightened_deg():6.1f}\n")

    def update(self, frames: dict, acc: dict) -> None:
        self._n += 1
        fw = frames.get(self.wrench)
        if fw is None or not np.all(np.isfinite(fw.pos)):
            self._pose_nut()
            return
        handle, face, mis, d = self.jaw_state(fw.pos, fw.rot)
        along = float(d @ B)
        dp = d - along * B
        in_plane = abs(float(fw.rot[:, 2] @ B)) >= math.cos(self.plane_tol)
        out = in_plane_unit(handle + face * math.radians(HEAD_ANGLE))  # the way the jaw slides off the nut
        o = float(dp @ out)  # how far the nut sits out from the seat, towards the mouth
        side = float(np.linalg.norm(dp - o * out))  # and across the mouth
        if not self.engaged:
            if (-self.deep <= o <= self.mouth and side <= self.across and abs(along) < self.axial and in_plane
                    and abs(mis) <= self.align_tol):
                self.engaged, self.face, self._prev = True, face, handle
                self.engagements += 1
                self.faces_used.add(face)
                self._event("on", handle, face, mis, dp, along)
        else:
            if (o > self.release or abs(along) > 1.5 * self.axial
                    or abs(float(fw.rot[:, 2] @ B)) < math.cos(1.6 * self.plane_tol) or face != self.face):
                self.engaged = False
                self._event("off", handle, face, mis, dp, along)
            else:
                perp = dp - out * max(0.0, o)  # held on the nut across the mouth
                v = np.asarray(fw.vel, float)
                f = -self.k_hold * perp - 2.0 * (v - float(v @ B) * B)
                f = f - min(self.pull, self.k_hold * max(0.0, o)) * out  # and drawn lightly onto it along the mouth
                _wrench(acc, self.wrench, fw, f, fw.pos)
                new = self.nu + _wrap(handle - self._prev, 2 * math.pi)
                held = min(max(new, self.lo), self.hi)
                if held != new:  # on a stop: the nut holds the wrench where it met it
                    _torque(acc, self.wrench, self.k_stop * _wrap(self._prev - handle, 2 * math.pi) * B)
                else:
                    self._prev = handle
                    spin = float(np.asarray(fw.spin, float) @ B)
                    _torque(acc, self.wrench, -self.drag * math.tanh(spin / 0.5) * B)  # the nylon insert's drag
                self.nu = held
        self._pose_nut()

    def _pose_nut(self) -> None:
        self.poses = {self.nut: (NUT_CENTRE + self.origin, matrix_to_quat_rel(nut_rotation(math.degrees(self.nu))))}


def in_plane_unit(angle_rad: float) -> np.ndarray:
    return math.cos(angle_rad) * Z + math.sin(angle_rad) * H


MODEL = fittingkit.add_model(WrenchOnNut("wrench", "nut", NUT_CENTRE))


# -- scoring ----------------------------------------------------------------------------------------------------------
# The tightening angle is the model's (the nut is only ever turned through it). A nut short of seated is a loose
# fastening: the photo shows the paint line short of the red mark.
COMPLETE_SCORE = 0.9  # score() once seated; 1.0 comes from success() holding for 1 s, which ends the trial
TIERS = ((150.0, 0.75), (100.0, 0.5), (60.0, 0.25))
# WHY 0.25 only from 60 degrees: one stroke without turning the wrench over turns the nut at most about 51 degrees (the
# 44 degree swing, plus up to the 7 degree alignment tolerance), so any credit needs the wrench turned over at least
# once.


def tightened_deg(objects=None) -> float:
    return MODEL.tightened_deg()


def success(objects) -> bool:
    """The nut seated: tightened its full half turn (within 2 degrees of the stop). Must hold for 1 s."""
    return MODEL.seated()


def score(objects) -> float:
    """0.9 seated (1.0 once success() has held 1 s); 0.75 from 150 degrees tightened; 0.5 from 100; 0.25 from 60;
    0 otherwise."""
    if MODEL.seated():
        return COMPLETE_SCORE
    t = MODEL.tightened_deg()
    return next((s for limit, s in TIERS if t >= limit), 0.0)


# -- the scripted demo (left arm) -----------------------------------------------------------------------------------
# Open loop. The left hand takes the wrench out of its rack by the handle, its fingers closing along B (across the
# handle's 5 mm thickness). For each stroke it brings the jaw into the slot above the nut, slides it down onto the
# flats, swings the handle clockwise (seen from the bolt's end) to near the slot's wall, backs it off along the bolt, out of
# the slot, and turns the wrench over by turning its grip half round about the vertical, then goes in again.
# WHY the grip tilts with the handle: the hand turns the wrench about the bolt; the stock waypoints only point
# straight down, so this file adds a tilt about B (TiltWaypoint, below).


class TiltWaypoint(Waypoint):
    """A Waypoint whose straight-down grip is also tilted ``tilt`` (rad) about B, after its yaw."""

    def __init__(self, duration, pos, finger, label="", yaw=0.0, tilt=0.0):
        super().__init__(duration, pos, finger, label, yaw)
        self.tilt = float(tilt)


def tcp_rotation(w) -> np.ndarray:
    rot = grip_rotation(w.yaw)
    tilt = getattr(w, "tilt", 0.0)
    return rot if tilt == 0.0 else axis_angle_matrix(B, tilt) @ rot


def _target(self, t: float):
    """yam_arena.demo.ArmPlan.target, honouring a TiltWaypoint's tilt."""
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

YAW = {+1: math.radians(-120.0), -1: math.radians(60.0)}  # the grip for each face out: fingers along B either way
YAW_MID = math.radians(-30.0)  # turning over passes through here (a half turn is otherwise ambiguous)
HOLD = FINGER_OPEN * 1.0 / 94.0  # aimed 1 mm apart across the 5 mm handle: a firm grip
AROUND = FINGER_OPEN * 40.0 / 94.0  # 40 mm open round the handle
# WHY the hand starts 60 mm above its grip: the wrench's top (its 10 mm head) is 35 mm above the grip and the fingertips
# 13 mm below the TCP; from 30 mm above, a fingertip came down on the head and the hand rested there (official sim
# trace, 2026-11-16). At 60 mm the fingertips clear the head by 12 mm before they come down round it.
OVER_GRIP = 60.0 * MM
OFF = 25.0 * MM  # out of the slot, the jaw clears the nut's axis by this much
# WHY the jaw comes down beside the nut, 16 mm out, and then slides on across it: from 25 mm out a stroke going on at 12.5
# degrees started with the handle against the wall (then at 20 degrees) and never reached the nut (official sim,
# 2026-11-16). From 10 mm out the prongs' tips were already over the nut, and with the jaw 5-7 mm to the side (the
# wrench a few degrees round in the gripper) it came down on the nut's end and the stroke missed (traces, runs wf21-wf23).
# From 16 mm the tips (10.8 mm past the seat) pass beside the nut's 4.3 mm core, and sliding on, a prong meets the round
# core and is steered onto it. With the wall at 22 degrees the handle still clears it (test below).
ON_FROM = 16.0 * MM
SETTLE = 0.8  # seconds above the nut before the jaw goes on
CLEAR_END = 10.0 * MM  # out along the bolt from there, the head clears the nut's end
OUT = 60.0 * MM  # out of the slot along B to turn the wrench over
OUT_UP = 15.0 * MM
# WHY each stroke aims 1 degree inside the wall: aimed past it, at the end of each stroke the arm pressed the handle into
# the wall and the wrench shifted in the gripper (official sim traces, runs wf20-wf22, 2026-11-16).
STROKE_AIM = {+1: -SWING_DEG + 1.0, -1: -SWING_DEG + 1.0}
STROKE_PAST = 2.0  # a last, short stroke aims this far past where it is planned to end
# WHY the plan is made from where the strokes really end and the jaw really goes on: planned from where they were aimed,
# the nut fell 1.5 degrees behind the plan each stroke, 6 degrees after four, the jaw then came at it 11 degrees off its
# flats, beyond the 7 that fit, and strokes 5 to 7 missed (official sim, run wf17, 2026-11-16). The jaw went on 0.3,
# 1.7, 2.3 and 0.8 degrees below its aim (mean 1.3, run wf17). The strokes stopped 0.5-2.5 degrees short of their aim
# against the nut's drag (runs wf16 and wf17, with the wall at 20); aimed at -21 with the wall at 22 they ended at -19.6,
# -19.5, -20.0 and -20.0 (run wf23, mean -19.8; planned at -19, the jaw met the flats 1 degree further off each stroke).
ON_LAG = 1.3
STROKE_REACHED = {f: aim + 1.2 for f, aim in STROKE_AIM.items()}
# WHY the jaw aimed 2.5 mm into the bolt from the nut's middle: at the nut the hand settled about 5 mm short along B, and
# the jaw only went on once it came within the model's 4.5 mm, sometimes not at all (official sim, 2026-11-16: four of
# six strokes went on, each at 4.4-4.5 mm). Aimed 2.5 mm deeper, its face still clears the back plate by 0.3 mm.
SEAT_AIM = N - 2.5 * MM * B


def engage_angle(nu_deg: float, face: int) -> float:
    """The handle angle (degrees) at which the jaw fits the flats with the paint line at ``nu_deg``, in the swing."""
    a = _wrap(nu_deg - 30.0 - 15.0 * face, 60.0)
    while a > SWING_DEG - 1.0:
        a -= 60.0
    while a < -SWING_DEG:
        a += 60.0
    return a


def stroke_plan() -> list[tuple[int, float, float]]:
    """(face, handle angle the jaw goes on at, handle angle the stroke ends at) for each stroke, until seated.

    These are where the sim's arm put the handle; the demo aims ``ON_LAG`` above the first and at ``STROKE_AIM``."""
    plan, nu, face = [], NU0_DEG, +1
    while NU0_DEG - nu < SEAT_DEG - 0.5:
        on = engage_angle(nu, face)
        end = max(STROKE_REACHED[face], on - (SEAT_DEG - (NU0_DEG - nu)))
        plan.append((face, on, end))
        nu -= on - end
        face = -face
    return plan


def tcp_for(seat, deg) -> tuple:
    """The TCP holding the wrench at the grip, its seat at ``seat`` and its handle at ``deg``."""
    p = np.asarray(seat, float) + GRIP * in_plane(deg)
    return tuple(float(v) for v in p)


def _w(dur, seat, deg, finger, label, face, yaw=None, fix=None):
    pos = tcp_for(seat, deg) if fix is None else tuple(float(v) for v in np.asarray(tcp_for(seat, deg)) + fix)
    return TiltWaypoint(dur, pos, finger, label, YAW[face] if yaw is None else yaw, math.radians(deg))


def _t(ws) -> float:
    return sum(w.duration for w in ws)


def demo():
    lift = SEAT0 + np.array([0.0, 0.0, POCKET_DEPTH + 15.0 * MM])
    left = [
        _w(2.0, SEAT0 + [0.0, 0.0, OVER_GRIP], 0.0, AROUND, "over the wrench", +1),
        _w(0.6, SEAT0 + [0.0, 0.0, OVER_GRIP], 0.0, AROUND, "settle over the wrench", +1),
        _w(1.5, SEAT0, 0.0, AROUND, "round its handle", +1),
        # WHY a pause: coming down, the arm lagged its aim by up to 22 mm along B, more than half the fingers' opening,
        # and closed beside the handle (official sim, 2026-11-16)
        _w(1.0, SEAT0, 0.0, AROUND, "settle round its handle", +1),
        _w(1.0, SEAT0, 0.0, HOLD, "grip the handle", +1),
        _w(0.5, SEAT0, 0.0, HOLD, "hold the grip", +1),
        _w(1.2, lift, 0.0, HOLD, "lift it out of the rack", +1),
    ]
    plan = stroke_plan()
    for k, (face, on, end) in enumerate(plan, start=1):
        on_l = on + ON_LAG
        end_l = STROKE_AIM[face] if end <= STROKE_REACHED[face] else end - STROKE_PAST
        near = SEAT_AIM + ON_FROM * in_plane_unit(math.radians(on_l + face * HEAD_ANGLE))
        front = near + OUT * B + OUT_UP * Z
        left += [
            _w(1.6 if k == 1 else 1.0, front, on_l, HOLD, f"in front of the slot {k}", face),
            # WHY a pause in front of the slot and the last 10 mm along the bolt taken slowly: going in straight from
            # turning the wrench over, the handle was still 3 degrees short of its angle, the jaw came in 5-7 mm to the
            # side, landed on the nut's end and the stroke missed (official sim trace, run wf21, 2026-11-16: strokes 3
            # and 5; stroke 1, which had 1.6 s to get there, went on)
            _w(0.5, front, on_l, HOLD, f"line up in front of the slot {k}", face),
            _w(1.0, near + CLEAR_END * B, on_l, HOLD, f"into the slot {k}", face),
            _w(SETTLE, near, on_l, HOLD, f"settle above the nut {k}", face),
            _w(0.8, SEAT_AIM, on_l, HOLD, f"onto the nut {k}", face),
            _w(1.6, SEAT_AIM, end_l, HOLD, f"stroke {k}", face),
            _w(0.5, SEAT_AIM, end_l, HOLD, f"end of stroke {k}", face),
        ]
        off = SEAT_AIM + OFF * in_plane_unit(math.radians(end_l + face * HEAD_ANGLE))
        out = off + OUT * B + OUT_UP * Z
        # WHY the jaw comes straight off along the bolt, over the nut's end, not slid off across it: slid off with the
        # wrench turned over, the head's corner ran into the slot's wall (31 mm out against the wall's 30.5), the arm fell
        # 8-12 mm behind and the wrench slipped 3 mm along and 7 mm across in the gripper; the next stroke then came down
        # on the nut's end and missed (official sim trace, run wf20, 2026-11-16).
        left += [
            _w(1.2, SEAT_AIM + OUT * B, end_l, HOLD, f"back off along the bolt {k}", face),
            _w(0.8, out, 0.0, HOLD, f"out of the slot {k}", face),
        ]
        if k < len(plan):
            left += [
                _w(0.7, out, 0.0, HOLD, f"turn it over {k}", face, yaw=YAW_MID),
                _w(0.7, out, 0.0, HOLD, f"turned over {k}", -face),
            ]
    last_face = plan[-1][0]
    left.append(_w(2.0, left_end_seat(), 0.0, HOLD, "hold it clear", last_face))
    right = [Waypoint(1.0, (0.22, -0.30, TOP + 0.15), FINGER_OPEN, "stay clear")]
    return {"left": left, "right": right}


def left_end_seat() -> np.ndarray:
    return N + OUT * B + (OUT_UP + 40.0 * MM) * Z
