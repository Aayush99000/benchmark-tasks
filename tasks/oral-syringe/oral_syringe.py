"""On the station table: a child care-training manikin's head facing the arms, and a filled 5 ml oral syringe lying in a
plastic tray. Give the manikin the medicine: the syringe's tip inside its cheek, between the gums and the cheek, the
plunger pushed in small squirts with a pause between them for it to swallow, until the syringe is empty; then take the
syringe out of its mouth.

The plunger is a damped slider in the barrel: pushing it is like pushing liquid out through the tip, about 2.6 N at
1 ml/s. 5 ml is 44 mm of plunger (12.06 mm bore).

Run it:  inspect-robots run --embodiment yam_arena -E task=oral_syringe.py -E object_jitter=0 -E control_hz=20 \\
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
from fittingkit import Slider, body, box, cyl  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Z_AXIS = np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])

SKIN = (0.93, 0.79, 0.68)
LIPS = (0.80, 0.45, 0.45)
DARK = (0.12, 0.10, 0.10)
CLEAR_PLASTIC = (0.88, 0.93, 0.96, 0.35)
MEDICINE = (0.92, 0.55, 0.20)  # an orange syrup, seen through the barrel
PLUNGER_WHITE = (0.94, 0.94, 0.92)
TRAY_BLUE = (0.55, 0.70, 0.85)


def rot_x(angle: float) -> np.ndarray:
    return axis_angle_matrix(X_AXIS, angle)


def rot_z(angle: float) -> np.ndarray:
    return axis_angle_matrix(Z_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the manikin (fixed): a child's head on a neck post, facing the arms (-x) ------------------------------------------
MOUTH = np.array([0.45, 0.0, TOP + 0.14])  # the middle of the mouth, in the plane of the lips
MOUTH_W, MOUTH_H = 40.0 * MM, 32.0 * MM  # the open mouth [estimate: a child manikin's open mouth]
FACE_DEPTH = 45.0 * MM  # how deep the mouth opening goes


def _manikin_shapes() -> tuple[list, list]:
    """In the manikin's frame: origin on the table under the lips' plane, x into the head."""
    mz = MOUTH[2] - TOP
    lo, hi, half = 0.08, 0.26, 0.075
    hw, hh = MOUTH_W / 2, MOUTH_H / 2
    d = FACE_DEPTH
    solid = [
        box((d / 2, (hw + half) / 2, (lo + hi) / 2), (d, half - hw, hi - lo), SKIN),
        box((d / 2, -(hw + half) / 2, (lo + hi) / 2), (d, half - hw, hi - lo), SKIN),
        box((d / 2, 0.0, (mz + hh + hi) / 2), (d, 2 * hw, hi - mz - hh), SKIN),
        box((d / 2, 0.0, (lo + mz - hh) / 2), (d, 2 * hw, mz - hh - lo), SKIN),
        box((d + 0.0575, 0.0, (lo + hi) / 2), (0.115, 2 * half, hi - lo), SKIN),
        box((0.08, 0.0, lo / 2), (0.10, 0.07, lo), SKIN),
    ]
    seen = [
        box((-0.001, 0.0, mz + hh + 0.002), (0.002, 2 * hw + 0.006, 0.004), LIPS),
        box((-0.001, 0.0, mz - hh - 0.002), (0.002, 2 * hw + 0.006, 0.004), LIPS),
        box((-0.001, hw + 0.002, mz), (0.002, 0.004, 2 * hh), LIPS),
        box((-0.001, -hw - 0.002, mz), (0.002, 0.004, 2 * hh), LIPS),
        box((-0.006, 0.0, mz + 0.045), (0.012, 0.018, 0.030), SKIN),
        box((-0.001, 0.030, mz + 0.085), (0.002, 0.016, 0.010), DARK),
        box((-0.001, -0.030, mz + 0.085), (0.002, 0.016, 0.010), DARK),
    ]
    return solid, seen


_MS, _MV = _manikin_shapes()
MANIKIN = body("manikin", pos=(MOUTH[0], MOUTH[1], TOP), friction=0.8, shapes=_MS + _MV,
               visual_only=tuple(range(len(_MS), len(_MS) + len(_MV))))

# -- the syringe: a barrel with its tip and finger flange, and a plunger on a damped slider in it ----------------------
# Its frame: origin at the middle of the barrel's back (the flange's front face), x along it towards the tip.
# 5 ml oral syringe: 12.06 mm bore (BD plastic 5 ml, syringe-pump diameter tables), so 1 ml is 8.756 mm of plunger.
# [estimate: the barrel's 14.5 mm outside, 58 mm length, the 10 mm tip and the 25 x 8 mm flange]
BORE = 12.06 * MM
ML = 1e-6 / (math.pi * (BORE / 2) ** 2)  # m of plunger per ml: 8.756 mm
DOSE_ML = 5.0
STROKE = DOSE_ML * ML  # 43.8 mm
BARREL_R, BARREL_L = 7.25 * MM, 58.0 * MM
NOZZLE_R, NOZZLE_L = 2.5 * MM, 10.0 * MM
SYRINGE_TIP = np.array([BARREL_L + NOZZLE_L, 0.0, 0.0])
FLANGE_T, FLANGE_W, FLANGE_H = 2.5 * MM, 25.0 * MM, 8.0 * MM
THUMB_R, THUMB_T = 8.5 * MM, 2.5 * MM
THUMB_CLEAR = 2.0 * MM  # the thumb rest stops this far short of the flange with the syringe empty
PLUNGER_AT = -(FLANGE_T + THUMB_CLEAR + THUMB_T / 2 + STROKE)  # the thumb rest's middle, filled (syringe frame, x)
DRAG = 300.0  # N s/m: about 2.6 N at 1 ml/s (8.8 mm/s) [estimate: a syrup through a 2 mm tip]
SYRINGE_MASS, PLUNGER_MASS = 0.011, 0.003  # barrel and 5 g of medicine; plunger [estimate]


def _syringe_shapes() -> tuple[list, list]:
    solid = [
        cyl((BARREL_L / 2, 0.0, 0.0), BARREL_R, BARREL_L, CLEAR_PLASTIC, axis="X"),
        cyl((BARREL_L + NOZZLE_L / 2, 0.0, 0.0), NOZZLE_R, NOZZLE_L, CLEAR_PLASTIC, axis="X"),
        box((-FLANGE_T / 2, 0.0, 0.0), (FLANGE_T, FLANGE_W, FLANGE_H), CLEAR_PLASTIC),
    ]
    seen = [cyl((BARREL_L / 2, 0.0, 0.0), BORE / 2 - 0.2 * MM, BARREL_L - 2 * MM, MEDICINE, axis="X")]
    return solid, seen


def _plunger_shapes() -> tuple[list, list]:
    """In the plunger's frame (origin at the thumb rest's middle): only the thumb rest collides; the rod and the
    rubber seal, inside and behind the barrel, are drawn only."""
    solid = [cyl((0.0, 0.0, 0.0), THUMB_R, THUMB_T, PLUNGER_WHITE, axis="X")]
    rod = -PLUNGER_AT
    seen = [box((rod / 2, 0.0, 0.0), (rod, 2.0 * MM, 9.0 * MM), PLUNGER_WHITE),
            cyl((rod, 0.0, 0.0), BORE / 2 - 0.3 * MM, 3.0 * MM, DARK, axis="X")]
    return solid, seen


# -- the tray (fixed): a floor under the barrel and two stops beside its front --------------------------------------------
SYRINGE_AT = np.array([0.24, 0.17, TOP + 3.0 * MM + BARREL_R])  # filled, lying along x, its tip towards the manikin
TRAY_FLOOR = 3.0 * MM


def _tray_shapes() -> list:
    """In the tray's frame (origin on the table under the syringe's back)."""
    gap = BARREL_R + 1.0 * MM
    return [
        box((0.034, 0.0, TRAY_FLOOR / 2), (0.054, 0.012, TRAY_FLOOR), TRAY_BLUE),  # narrow: the fingers pass beside it
        box((0.052, gap + 2.0 * MM, TRAY_FLOOR + 4.0 * MM), (0.012, 4.0 * MM, 8.0 * MM), TRAY_BLUE),
        box((0.052, -gap - 2.0 * MM, TRAY_FLOOR + 4.0 * MM), (0.012, 4.0 * MM, 8.0 * MM), TRAY_BLUE),
    ]


TRAY = body("tray", pos=(SYRINGE_AT[0], SYRINGE_AT[1], TOP), friction=0.6, shapes=_tray_shapes())
_SS, _SV = _syringe_shapes()
SYRINGE = body("syringe", pos=tuple(SYRINGE_AT), mass=SYRINGE_MASS, friction=0.8, shapes=_SS + _SV,
               visual_only=tuple(range(len(_SS), len(_SS) + len(_SV))), com=(0.030, 0.0, 0.0))
_PS, _PV = _plunger_shapes()
PLUNGER = body("plunger", pos=tuple(SYRINGE_AT + np.array([PLUNGER_AT, 0.0, 0.0])), mass=PLUNGER_MASS, friction=0.8,
               shapes=_PS + _PV, visual_only=tuple(range(len(_PS), len(_PS) + len(_PV))), com=(0.03, 0.0, 0.0))
fittingkit.add_slider(Slider("syringe", "plunger", anchor=(PLUNGER_AT, 0.0, 0.0), axis=(1, 0, 0), lower=-0.5 * MM,
                             upper=STROKE, stiffness=0.0, damping=DRAG, target=0.0))

OBJECTS = [MANIKIN, TRAY, SYRINGE, PLUNGER]


# -- the checks ----------------------------------------------------------------------------------------------------------
CHEEK_DEPTH = (8.0 * MM, 25.0 * MM)  # the tip this far past the lips
CHEEK_SIDE = (8.0 * MM, 18.0 * MM)  # and this far to one side of the mouth's middle
CHEEK_TOWARDS_DEG = 10.0  # the syringe pointing at least this far towards that cheek, off straight in
CHEEK_HEIGHT = MOUTH_H / 2  # the tip within the mouth's height
OUT_OF_MOUTH = 5.0 * MM  # out: the tip this far back from the lips
FULL_ML = 4.8
SQUIRT_MAX_ML = 2.0
PAUSE_S = 1.0  # a squirt ends once the plunger has been still this long
PAUSE_CREEP = 0.5 * MM  # a pause: the plunger's furthest point advances less than this (0.06 ml) in PAUSE_S
# WHY a window: run os1's plunger crept about 0.3 mm/s while the arm held still, and the substep-to-substep jitter of
# its position read as motion, so no pause ever counted and its three squirts were logged as one
PARTIAL_ML = 2.0
OUTSIDE_SLACK_ML = 0.1  # pushed with the tip not in the cheek, more than this, scores 0


def tip_in_cheek(syringe, origin=np.zeros(3)) -> bool:
    """The tip 8-25 mm past the lips, 8-18 mm to one side of the mouth's middle and within the mouth's height, the
    syringe pointing at least 10 degrees towards that side."""
    tip = syringe.world(SYRINGE_TIP) - origin
    axis = syringe.rot[:, 0]
    depth, side, up = tip[0] - MOUTH[0], tip[1] - MOUTH[1], tip[2] - MOUTH[2]
    towards = math.degrees(math.atan2(axis[1] * math.copysign(1.0, side), axis[0]))
    return bool(CHEEK_DEPTH[0] <= depth <= CHEEK_DEPTH[1] and CHEEK_SIDE[0] <= abs(side) <= CHEEK_SIDE[1]
                and abs(up) <= CHEEK_HEIGHT and towards >= CHEEK_TOWARDS_DEG)


def out_of_mouth(syringe, origin=np.zeros(3)) -> bool:
    return bool(syringe.world(SYRINGE_TIP)[0] - origin[0] <= MOUTH[0] - OUT_OF_MOUTH)


class DoseLog:
    """Runs before every physics substep (240 Hz): how much medicine has been pushed out (from the furthest the plunger
    has gone), how much of it with the tip not in the cheek, and the biggest squirt (between pauses of 1 s)."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("SYRINGE_LOG")
        self.trace = os.environ.get("SYRINGE_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["syringe", "plunger"]

    def clear(self) -> None:
        self._n = 0
        self.given = 0.0  # ml
        self.outside = 0.0  # ml pushed with the tip not in the cheek
        self.squirts: list[float] = []  # ml in each finished squirt
        self.squirt = 0.0  # ml in the squirt going on
        self.peak = 0.0  # the furthest the plunger has gone (m)
        self._window: list[tuple[float, float]] = []  # (time, peak) over the last PAUSE_S
        self.now_in_cheek = self.now_out = False

    def update(self, frames: dict, acc: dict) -> None:
        s, p = frames.get("syringe"), frames.get("plunger")
        if s is None or p is None:
            return
        self._n += 1
        dt = 1.0 / 240.0
        travel = float(s.local(p.pos)[0] - PLUNGER_AT)
        self.now_in_cheek = tip_in_cheek(s, self.origin)
        self.now_out = out_of_mouth(s, self.origin)
        t = self._n * dt
        if travel > self.peak:
            ml = (travel - self.peak) / ML
            self.peak = travel
            self.given += ml
            self.squirt += ml
            if not self.now_in_cheek:
                self.outside += ml
        self._window.append((t, self.peak))
        while self._window and self._window[0][0] < t - PAUSE_S:
            self._window.pop(0)
        paused = t - self._window[0][0] >= PAUSE_S - 2 * dt and self.peak - self._window[0][1] < PAUSE_CREEP
        if paused and self.squirt > 0.1:
            self.squirts.append(self.squirt)
            self._note(f"{t:7.2f}s squirt {self.squirt:4.2f} ml, given {self.given:4.2f} ml, "
                       f"outside the cheek {self.outside:4.2f} ml")
            self.squirt = 0.0
        if self.trace and self._n % 24 == 0:
            self._write_trace(s, travel)

    def _write_trace(self, s, travel) -> None:
        """Debug only (SYRINGE_TRACE=<file>): every 0.1 s, the dose, the tip and both TCPs (mm, from the table's origin)."""
        tip = s.world(SYRINGE_TIP) - self.origin
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        axis = s.rot[:, 0]
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240.0:6.2f}s travel {1000 * travel:5.1f} given {self.given:4.2f} squirt "
                     f"{self.squirt:4.2f} max {self.biggest():4.2f} out {self.outside:4.2f} | tip past lips "
                     f"{1000 * (tip[0] - MOUTH[0]):6.1f} side {1000 * (tip[1] - MOUTH[1]):6.1f} up "
                     f"{1000 * (tip[2] - MOUTH[2]):6.1f} yaw {math.degrees(math.atan2(axis[1], axis[0])):5.1f} "
                     f"cheek {int(self.now_in_cheek)} |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")

    # -- what the scorer reads
    def biggest(self) -> float:
        return max(self.squirts + [self.squirt])

    def clean(self) -> bool:
        """Nothing (beyond 0.1 ml of jitter) pushed out with the tip not in the cheek."""
        return self.outside <= OUTSIDE_SLACK_ML


MODEL = fittingkit.add_model(DoseLog())

# -- scoring -------------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def success(objects) -> bool:
    """The whole 5 ml given with the tip in the cheek, no squirt over 2 ml, and the syringe out of the mouth. Must hold
    for 1 s."""
    return MODEL.given >= FULL_ML and MODEL.clean() and MODEL.biggest() <= SQUIRT_MAX_ML and MODEL.now_out


def score(objects) -> float:
    """0.9 for the whole dose in small squirts in the cheek and the syringe out (1.0 once success() has held 1 s); 0.5
    for the whole dose in the cheek but a squirt over 2 ml (or the syringe not taken out); 0.25 for at least 2 ml in the
    cheek; 0 for anything pushed out with the tip not in the cheek, or less than 2 ml."""
    if not MODEL.clean() or MODEL.given < PARTIAL_ML:
        return 0.0
    if MODEL.given >= FULL_ML:
        return COMPLETE_SCORE if (MODEL.biggest() <= SQUIRT_MAX_ML and MODEL.now_out) else 0.5
    return 0.25


# -- the scripted demo -----------------------------------------------------------------------------------------------
# The left hand picks the syringe up by its barrel and puts its tip into the manikin's cheek at the corner of the mouth,
# the syringe turned 25 degrees towards that cheek. The right hand, fingers closed and pointing along the syringe, pushes
# the plunger's thumb rest in three squirts of 1.7 ml with 2 s between them, then backs off, and the left hand takes the
# syringe out.
class RotWaypoint(Waypoint):
    """A Waypoint with its TCP's orientation given as a rotation matrix (columns: the TCP's x, y, z in the world; z
    points along the fingers)."""

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
PAD_TIP = 13.0 * MM  # the closed fingertips reach this far past the TCP along the hand


def finger_for_gap(gap: float) -> float:
    return -gap / 2


BARREL_GRIP = finger_for_gap(2 * BARREL_R - 6.0 * MM)  # about 15 N on the barrel
GRIP_AT = np.array([0.016, 0.0, 0.0])  # where the left hand holds the barrel (16 mm in front of the flange)
GRIP_ABOVE = 4.0 * MM
R_PICK = grip_rotation(0.0)  # pointing down, the fingers closing along y, across the barrel

YAW = math.radians(25.0)  # the syringe turned towards the manikin's cheek on the robot's left (+y)
TILT = math.radians(-45.0)  # the left hand leans in from its own side at the face (it can't reach straight down there)
# WHY 45 degrees and 16 mm: holding the barrel's middle and leaning 30 degrees, the hand went 8 mm into the manikin's
# face (plan_check.py); 10 mm further back and leaning 45 degrees it clears the face by about 9 mm
R_SYRINGE_FACE = rot_z(YAW) @ rot_x(TILT)  # the syringe's orientation at the face: turned, and rolled with the hand
TIP_IN = MOUTH + np.array([15.0 * MM, 12.0 * MM, -4.0 * MM])  # in the cheek, between the lower gum and the cheek
APPROACH = 20.0 * MM  # the tip this far short of the lips (along the syringe) before it goes in
OUT = 40.0 * MM  # taken out this far along the syringe


def _rel(p_tcp, r_tcp, p_obj, r_obj):
    return r_tcp.T @ (np.asarray(p_obj) - np.asarray(p_tcp)), r_tcp.T @ r_obj


PICK_TCP = SYRINGE_AT + GRIP_AT + np.array([0.0, 0.0, GRIP_ABOVE])
_P_TS, _R_TS = _rel(PICK_TCP, R_PICK, SYRINGE_AT, np.eye(3))


def syringe_origin(tip_at, rot=R_SYRINGE_FACE) -> np.ndarray:
    return np.asarray(tip_at) - np.asarray(rot) @ SYRINGE_TIP


def tcp_for_syringe(origin, rot=R_SYRINGE_FACE) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose that puts the syringe (held as at the pick) at ``origin``, ``rot``."""
    r_tcp = np.asarray(rot) @ _R_TS.T
    return np.asarray(origin) - r_tcp @ _P_TS, r_tcp


def push_pose(origin, travel, rot=R_SYRINGE_FACE, back=0.0) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP's pose with its closed fingertips on the thumb rest's middle (``travel`` pushed in), pointing
    along the syringe, ``back`` behind it, the fingers one above the other."""
    axis = np.asarray(rot)[:, 0]
    thumb = np.asarray(origin) + axis * (PLUNGER_AT - THUMB_T / 2 + travel)
    z = axis
    x = np.cross(Z_AXIS, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    r = axis_angle_matrix(z, PUSH_ROLL) @ np.column_stack([x, y, z])
    return thumb - z * (PAD_TIP + back), r


PUSH_ROLL = math.radians(90.0)  # the pushing hand turned so its fingers close up and down
# WHY: with the fingers closing sideways the right arm misses the push poses by 60 degrees; turned 60-300 degrees
# about the push it reaches them all within 0.1 mm
SQUIRTS = 3
SQUIRT_S, SWALLOW_S = 1.8, 3.0  # the plunger creeps on as the lagging arm catches up, so the pauses are long
PUSH_PAST = 4.0 * MM  # the last push aimed this far past the plunger's stop [estimate: arm compliance]


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    high, near = np.array([0.0, 0.0, 0.12]), np.array([0.0, 0.0, 0.02])
    face_out = syringe_origin(TIP_IN - R_SYRINGE_FACE[:, 0] * (APPROACH + (TIP_IN[0] - MOUTH[0]) / math.cos(YAW)))
    face_in = syringe_origin(TIP_IN)
    park = push_pose(face_out, 0.0, back=0.05)
    both(2.5, park, CLOSED, (PICK_TCP + high, R_PICK), OPEN, "high over the syringe")
    both(1.2, park, CLOSED, (PICK_TCP + near, R_PICK), OPEN, "just over it")
    both(0.5, park, CLOSED, (PICK_TCP + near, R_PICK), OPEN, "settle over it")
    both(0.8, park, CLOSED, (PICK_TCP, R_PICK), OPEN, "down to it")
    both(1.8, park, CLOSED, (PICK_TCP, R_PICK), BARREL_GRIP, "grip")
    both(1.5, park, CLOSED, (PICK_TCP + np.array([0.0, 0.0, 0.08]), R_PICK), BARREL_GRIP, "lift")
    both(3.0, push_pose(face_out, 0.0, back=0.03), CLOSED, tcp_for_syringe(face_out), BARREL_GRIP,
         "to the face, the right hand behind the plunger")
    both(1.5, push_pose(face_in, 0.0, back=0.03), CLOSED, tcp_for_syringe(face_in), BARREL_GRIP, "tip into the cheek")
    both(1.0, push_pose(face_in, 0.0, back=2.0 * MM), CLOSED, tcp_for_syringe(face_in), BARREL_GRIP,
         "finger to the plunger")
    both(0.5, push_pose(face_in, 0.0, back=2.0 * MM), CLOSED, tcp_for_syringe(face_in), BARREL_GRIP, "settle")
    for k in range(1, SQUIRTS + 1):
        travel = STROKE * k / SQUIRTS + (PUSH_PAST if k == SQUIRTS else 0.0)
        both(SQUIRT_S, push_pose(face_in, travel), CLOSED, tcp_for_syringe(face_in), BARREL_GRIP, f"squirt {k}")
        both(SWALLOW_S, push_pose(face_in, travel), CLOSED, tcp_for_syringe(face_in), BARREL_GRIP, f"swallow {k}")
    both(0.8, push_pose(face_in, STROKE, back=0.03), CLOSED, tcp_for_syringe(face_in), BARREL_GRIP, "finger off")
    both(1.5, push_pose(face_out, STROKE, back=0.06), CLOSED, tcp_for_syringe(face_out), BARREL_GRIP,
         "out of the mouth")
    both(1.5, push_pose(face_out, STROKE, back=0.06), CLOSED, tcp_for_syringe(face_out), BARREL_GRIP, "wait")
    return {"left": left, "right": right}
