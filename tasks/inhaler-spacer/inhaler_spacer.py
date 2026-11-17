"""On the station table: a child care-training manikin head facing the arms, a pressurised metered-dose inhaler (cap
off) lying on its side, and a spacer the size of an AeroChamber Plus lying beside it. Give the manikin one puff through
the spacer: shake the inhaler, fit it into the spacer's back, put the spacer's mouthpiece in the manikin's mouth, press
the inhaler once, and keep the spacer in its mouth for its five breaths.

The canister is a sprung slider in the inhaler's plastic actuator: it fires (a puff) when pushed 2.5 mm in, which takes
25 N, and is ready again once back above 1 mm. The YAM's fingers squeeze with up to 50 N, so the hand holding the
inhaler end to end, as a thumb and index finger do, can fire it while the other hand keeps the spacer in the mouth.

Run it:  inspect-robots run --embodiment yam_arena -E task=inhaler_spacer.py -E object_jitter=0 -E control_hz=20 \\
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
X_AXIS = np.array([1.0, 0.0, 0.0])

SKIN = (0.93, 0.79, 0.68)
LIPS = (0.80, 0.45, 0.45)
DARK = (0.12, 0.10, 0.10)
ACTUATOR_BLUE = (0.25, 0.45, 0.80)
ALUMINIUM = (0.78, 0.79, 0.80)
CLEAR_PLASTIC = (0.85, 0.92, 0.95, 0.35)
RUBBER = (0.30, 0.32, 0.34)


def rot_x(angle: float) -> np.ndarray:
    return axis_angle_matrix(X_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    q = fittingkit.matrix_to_quat_rel(r)
    return tuple(float(v) for v in q)


# -- the manikin (fixed): a child's head on a neck post, facing the arms (-x) ------------------------------------------
MOUTH = np.array([0.45, 0.0, TOP + 0.14])  # the middle of the mouth, in the plane of the lips
MOUTH_W, MOUTH_H = 40.0 * MM, 32.0 * MM  # the open mouth [estimate: a child manikin's open mouth]
FACE_DEPTH = 45.0 * MM  # how deep the mouth opening goes before the back of the throat


def _manikin_shapes() -> tuple[list, list]:
    """In the manikin's frame: origin on the table under the lips' plane, x into the head."""
    mz = MOUTH[2] - TOP
    lo, hi, half = 0.08, 0.26, 0.075  # the head from 8 to 26 cm above the table, 15 cm wide
    hw, hh = MOUTH_W / 2, MOUTH_H / 2
    d = FACE_DEPTH
    solid = [
        box((d / 2, (hw + half) / 2, (lo + hi) / 2), (d, half - hw, hi - lo), SKIN),  # face, left of the mouth
        box((d / 2, -(hw + half) / 2, (lo + hi) / 2), (d, half - hw, hi - lo), SKIN),  # right of it
        box((d / 2, 0.0, (mz + hh + hi) / 2), (d, 2 * hw, hi - mz - hh), SKIN),  # above it
        box((d / 2, 0.0, (lo + mz - hh) / 2), (d, 2 * hw, mz - hh - lo), SKIN),  # below it
        box((d + 0.0575, 0.0, (lo + hi) / 2), (0.115, 2 * half, hi - lo), SKIN),  # the back of the head
        box((0.08, 0.0, lo / 2), (0.10, 0.07, lo), SKIN),  # the neck post
    ]
    seen = [
        box((-0.001, 0.0, mz + hh + 0.002), (0.002, 2 * hw + 0.006, 0.004), LIPS),
        box((-0.001, 0.0, mz - hh - 0.002), (0.002, 2 * hw + 0.006, 0.004), LIPS),
        box((-0.001, hw + 0.002, mz), (0.002, 0.004, 2 * hh), LIPS),
        box((-0.001, -hw - 0.002, mz), (0.002, 0.004, 2 * hh), LIPS),
        box((-0.006, 0.0, mz + 0.045), (0.012, 0.018, 0.030), SKIN),  # nose
        box((-0.001, 0.030, mz + 0.085), (0.002, 0.016, 0.010), DARK),  # eyes
        box((-0.001, -0.030, mz + 0.085), (0.002, 0.016, 0.010), DARK),
    ]
    return solid, seen


_MS, _MV = _manikin_shapes()
MANIKIN = body("manikin", pos=(MOUTH[0], MOUTH[1], TOP), friction=0.8, shapes=_MS + _MV,
               visual_only=tuple(range(len(_MS), len(_MS) + len(_MV))))

# -- the inhaler: a plastic actuator and an aluminium canister on a sprung slider in it --------------------------------
# Its frame: origin at the middle of the actuator's base, z up the canister, x out through the mouthpiece.
# Sizes: a Ventolin pMDI is about 9 cm tall, 4 cm deep and 3 cm wide (University of Toronto Scientific Instruments
# Collection); canister 75 mm long. [estimate: the canister's 21 mm diameter and the mouthpiece, 24 x 17 mm and 24 mm long]
BOOT_D, BOOT_W, BOOT_H = 28.0 * MM, 30.0 * MM, 73.0 * MM  # the actuator's upright boot: along x, y, z
MOUTHPIECE_L, MOUTHPIECE_W, MOUTHPIECE_H = 24.0 * MM, 24.0 * MM, 17.0 * MM
MOUTHPIECE_Z = (3.0 * MM, 20.0 * MM)
MOUTHPIECE_TIP = np.array([BOOT_D / 2 + MOUTHPIECE_L, 0.0, sum(MOUTHPIECE_Z) / 2])
CAN_R, CAN_L, CAN_PROUD = 10.5 * MM, 75.0 * MM, 15.0 * MM
CAN_TOP = BOOT_H + CAN_PROUD  # 88 mm: the inhaler end to end
CAN_SEEN_FROM = CAN_TOP - CAN_L  # its bottom, inside the boot
CAN_SOLID = 20.0 * MM  # only the canister's top 20 mm collides; the rest, inside the boot, is drawn only
CAN_ORIGIN = CAN_TOP - CAN_SOLID / 2  # the canister body's origin in the inhaler's frame (z)
COLLAR = 13.0 * MM  # the boot's top 13 mm is walls round the canister (2 mm clear); below that it is solid
FIRE, REARM = 2.5 * MM, 1.0 * MM  # a puff when pushed this far in; ready again once back above REARM
CAN_TRAVEL = 3.0 * MM
CAN_K = 10000.0  # N/m: 25 N at the firing point (measured actuation forces 22-31 N, Drug Dev Ind Pharm 2008)
INHALER_MASS, CANISTER_MASS = 0.012, 0.018  # kg [estimate]


def _inhaler_shapes() -> tuple[list, list]:
    hd, hw = BOOT_D / 2, BOOT_W / 2
    solid_top = BOOT_H - COLLAR
    gap = CAN_R + 2.0 * MM  # the collar's hole half-width
    shapes = [
        box((0.0, 0.0, solid_top / 2), (BOOT_D, BOOT_W, solid_top), ACTUATOR_BLUE),
        box(((hd + gap) / 2, 0.0, solid_top + COLLAR / 2), (hd - gap, BOOT_W, COLLAR), ACTUATOR_BLUE),
        box((-(hd + gap) / 2, 0.0, solid_top + COLLAR / 2), (hd - gap, BOOT_W, COLLAR), ACTUATOR_BLUE),
        box((0.0, (hw + gap) / 2, solid_top + COLLAR / 2), (2 * gap, hw - gap, COLLAR), ACTUATOR_BLUE),
        box((0.0, -(hw + gap) / 2, solid_top + COLLAR / 2), (2 * gap, hw - gap, COLLAR), ACTUATOR_BLUE),
        box((hd + MOUTHPIECE_L / 2, 0.0, sum(MOUTHPIECE_Z) / 2), (MOUTHPIECE_L, MOUTHPIECE_W, MOUTHPIECE_H),
            ACTUATOR_BLUE),
    ]
    return shapes, []


def _canister_shapes() -> tuple[list, list]:
    """In the canister body's frame (origin CAN_ORIGIN up the inhaler's axis)."""
    solid = [cyl((0.0, 0.0, 0.0), CAN_R, CAN_SOLID, ALUMINIUM)]
    inside = CAN_TOP - CAN_SOLID - CAN_SEEN_FROM
    seen = [cyl((0.0, 0.0, -CAN_SOLID / 2 - inside / 2), CAN_R, inside, ALUMINIUM)]
    return solid, seen


R_LYING = rot_x(-math.pi / 2)  # on its side: the canister towards +y, the mouthpiece towards the manikin (+x)
INHALER_AT = np.array([0.26, -0.16, TOP + BOOT_W / 2])
_IS, _ = _inhaler_shapes()
INHALER = body("inhaler", pos=tuple(INHALER_AT), mass=INHALER_MASS, friction=0.8, shapes=_IS,
               quat=quat_xyzw(R_LYING), com=(0.004, 0.0, 0.030))
_CS, _CV = _canister_shapes()
CANISTER = body("canister", pos=tuple(INHALER_AT + R_LYING @ np.array([0.0, 0.0, CAN_ORIGIN])), mass=CANISTER_MASS,
                com=(0.0, 0.0, CAN_SEEN_FROM + CAN_L / 2 - CAN_ORIGIN), friction=0.8, shapes=_CS + _CV, visual_only=tuple(range(len(_CS), len(_CS) + len(_CV))),
                quat=quat_xyzw(R_LYING))
fittingkit.add_slider(Slider("inhaler", "canister", anchor=(0.0, 0.0, CAN_ORIGIN), axis=(0, 0, 1), lower=-CAN_TRAVEL,
                             upper=0.0, stiffness=CAN_K, damping=5.0, target=0.0))

# -- the spacer: clear barrel, rubber back socket, mouthpiece ------------------------------------------------------------
# Its frame: origin at the middle of its back face, x along it towards the mouthpiece.
# The size of an AeroChamber Plus (14.5 x 4.6 cm; 149 ml). Its soft back takes most inhalers' mouthpieces; here a
# rigid square socket 32 mm across, which takes the 24 x 17 mm mouthpiece at any roll of the spacer up to 30 degrees.
SPACER_L, SPACER_R = 145.0 * MM, 23.0 * MM
SOCKET_DEPTH, SOCKET_W, SOCKET_H = 20.0 * MM, 32.0 * MM, 32.0 * MM
SPACER_MOUTH_R, SPACER_MOUTH_L = 12.0 * MM, 20.0 * MM
SPACER_TIP = np.array([SPACER_L, 0.0, 0.0])
SPACER_MASS = 0.05  # kg [estimate]


def _spacer_shapes() -> list:
    r, d = SPACER_R, SOCKET_DEPTH
    bar_z = (r - SOCKET_H / 2)
    bar_y = (r - SOCKET_W / 2)
    barrel_l = SPACER_L - d - SPACER_MOUTH_L
    return [
        box((d / 2, 0.0, (SOCKET_H / 2 + r) / 2), (d, 2 * r, bar_z), RUBBER),
        box((d / 2, 0.0, -(SOCKET_H / 2 + r) / 2), (d, 2 * r, bar_z), RUBBER),
        box((d / 2, (SOCKET_W / 2 + r) / 2, 0.0), (d, bar_y, SOCKET_H), RUBBER),
        box((d / 2, -(SOCKET_W / 2 + r) / 2, 0.0), (d, bar_y, SOCKET_H), RUBBER),
        cyl((d + barrel_l / 2, 0.0, 0.0), r, barrel_l, CLEAR_PLASTIC, axis="X"),
        cyl((SPACER_L - SPACER_MOUTH_L / 2, 0.0, 0.0), SPACER_MOUTH_R, SPACER_MOUTH_L, RUBBER, axis="X"),
    ]


SPACER_AT = np.array([0.28, 0.16, TOP + SPACER_R])
SPACER = body("spacer", pos=tuple(SPACER_AT), mass=SPACER_MASS, friction=0.8, shapes=_spacer_shapes(),
              com=(0.065, 0.0, 0.0))
# WHY the centres of mass: left at each body's origin, the spacer's sat on the back edge of its footprint and it
# stood itself up on its back end 1.5 s in (run inh2)

OBJECTS = [MANIKIN, INHALER, CANISTER, SPACER]


# -- the checks ----------------------------------------------------------------------------------------------------------
SHAKE_STROKE = 30.0 * MM  # a shake: down at least this far, then back up at least this far
SHAKES_NEEDED = 4
FITTED_DEPTH = 10.0 * MM  # the mouthpiece this far into the spacer's socket
FITTED_OFF = 4.0 * MM  # and its tip this close to the spacer's axis
IN_MOUTH_DEPTH = 8.0 * MM  # the spacer's tip this far past the lips
IN_MOUTH_OFF = 6.0 * MM  # within this of the mouth's middle
SQUARE_DEG = 15.0  # the spacer within this of square to the face
BREATHS_S = 5.0  # kept in the mouth this long after the puff


def fitted(inhaler, spacer) -> bool:
    """The inhaler's mouthpiece at least 10 mm into the spacer's socket, its tip within 4 mm of the spacer's axis."""
    tip = spacer.local(inhaler.world(MOUTHPIECE_TIP))
    return bool(tip[0] >= FITTED_DEPTH and math.hypot(tip[1], tip[2]) <= FITTED_OFF)


def in_mouth(spacer, origin=np.zeros(3)) -> bool:
    """The spacer's tip at least 8 mm past the lips and within 6 mm of the mouth's middle, the spacer within 15
    degrees of square to the face."""
    tip = spacer.world(SPACER_TIP) - origin
    axis = spacer.rot[:, 0]
    square = math.degrees(math.acos(float(np.clip(axis[0], -1.0, 1.0))))
    return bool(tip[0] - MOUTH[0] >= IN_MOUTH_DEPTH and math.hypot(tip[1] - MOUTH[1], tip[2] - MOUTH[2]) <= IN_MOUTH_OFF
                and square <= SQUARE_DEG)


class InhalerLog:
    """Runs before every physics substep (240 Hz): counts shakes, logs each puff with whether the inhaler was fitted
    and the spacer in the mouth, and times how long the spacer then stays in the mouth."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("INHALER_LOG")
        self.trace = os.environ.get("INHALER_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["inhaler", "canister", "spacer"]

    def clear(self) -> None:
        self._n = 0
        self.shakes = 0
        self._dir, self._hi, self._lo = 1, -np.inf, np.inf
        self.down = False
        self.depth = 0.0
        self.puffs: list[dict] = []
        self.kept = False  # kept in the mouth for BREATHS_S after the (first) puff
        self._broken = False
        self.now_fitted = self.now_in_mouth = False

    def update(self, frames: dict, acc: dict) -> None:
        a, c, s = (frames.get(n) for n in ("inhaler", "canister", "spacer"))
        if a is None or c is None or s is None:
            return
        self._n += 1
        t = self._n / 240.0
        # shakes: the inhaler down at least SHAKE_STROKE, then up at least SHAKE_STROKE
        z = float(a.pos[2])
        if self._dir > 0:
            self._hi = max(self._hi, z)
            if z < self._hi - SHAKE_STROKE:
                self._dir, self._lo = -1, z
        else:
            self._lo = min(self._lo, z)
            if z > self._lo + SHAKE_STROKE:
                self._dir, self._hi = 1, z
                self.shakes += 1
        self.now_fitted = fitted(a, s)
        self.now_in_mouth = in_mouth(s, self.origin)
        self.depth = float(CAN_ORIGIN - a.local(c.pos)[2])
        if not self.down and self.depth >= FIRE:
            self.down = True
            self.puffs.append(dict(t=t, fitted=self.now_fitted, in_mouth=self.now_in_mouth, shakes=self.shakes))
            self._note(f"{t:7.2f}s puff fitted {int(self.now_fitted)} in-mouth {int(self.now_in_mouth)} "
                       f"shakes {self.shakes}")
        elif self.down and self.depth < REARM:
            self.down = False
        if len(self.puffs) == 1 and not self.kept:
            if not self.now_in_mouth:
                self._broken = True
            elif not self._broken and t - self.puffs[0]["t"] >= BREATHS_S:
                self.kept = True
                self._note(f"{t:7.2f}s kept in the mouth {BREATHS_S:.0f} s")
        if self.trace and self._n % 24 == 0:
            self._write_trace(t, z, a, s)

    def _write_trace(self, t, z, a, s) -> None:
        """Debug only (INHALER_TRACE=<file>): every 0.1 s, the parts and both TCPs (mm, from the table's origin)."""
        tip = s.world(SPACER_TIP) - self.origin
        mtip = s.local(a.world(MOUTHPIECE_TIP))
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                p = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({p[0]:6.1f},{p[1]:6.1f},{p[2] - 1000 * TOP:6.1f})"
        with open(self.trace, "a") as fh:
            fh.write(f"{t:6.2f}s z {1000 * (z - self.origin[2] - TOP):6.1f} shakes {self.shakes} depth "
                     f"{1000 * self.depth:5.2f} | socket {1000 * mtip[0]:6.1f} off {1000 * math.hypot(*mtip[1:]):5.1f} "
                     f"fitted {int(self.now_fitted)} | tip past lips {1000 * (tip[0] - MOUTH[0]):6.1f} off "
                     f"{1000 * math.hypot(tip[1] - MOUTH[1], tip[2] - MOUTH[2]):5.1f} in {int(self.now_in_mouth)} |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")

    # -- what the scorer reads
    def good_puff(self) -> bool:
        """Exactly one puff, fired with the inhaler fitted and the spacer in the mouth."""
        return len(self.puffs) == 1 and self.puffs[0]["fitted"] and self.puffs[0]["in_mouth"]


MODEL = fittingkit.add_model(InhalerLog())

# -- scoring -------------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def success(objects) -> bool:
    """Shaken at least 4 times, then exactly one puff, fired with the inhaler fitted and the spacer in the mouth, and
    the spacer kept in the mouth for 5 s after it. Must hold for 1 s."""
    return MODEL.good_puff() and MODEL.puffs[0]["shakes"] >= SHAKES_NEEDED and MODEL.kept


def score(objects) -> float:
    """0.9 for a full dose (1.0 once success() has held 1 s); 0.5 for one good puff not shaken first or not kept in
    for 5 s; 0.25 for the inhaler fitted and the spacer in the mouth with no puff; 0 otherwise (two puffs, or a puff
    with the inhaler not fitted or the spacer not in the mouth)."""
    if MODEL.puffs:
        if not MODEL.good_puff():
            return 0.0
        return COMPLETE_SCORE if (MODEL.puffs[0]["shakes"] >= SHAKES_NEEDED and MODEL.kept) else 0.5
    return 0.25 if (MODEL.now_fitted and MODEL.now_in_mouth) else 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# The right hand holds the inhaler end to end, one finger on the canister's end and the other on the actuator's base,
# as a thumb and index finger do: it picks it up lying on its side from above, then turns it upright (a quarter turn
# of the hand about x), shakes it, fits it into the spacer the left hand holds, and fires it by closing harder. The left
# hand holds the spacer's barrel from above and puts its mouthpiece in the manikin's mouth.
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

OPEN = -0.04695  # finger joint: pads 94 mm apart


def finger_for_gap(gap: float) -> float:
    """The finger joint value that puts the pads ``gap`` (m) apart (0: touching; -0.04695: 94 mm)."""
    return -gap / 2


HOLD = finger_for_gap(CAN_TOP - 5.0 * MM)  # pads 5 mm inside the inhaler's length: about 10 N, short of firing
# WHY 10 N: held at about 5 N, the inhaler swung out of the end-to-end pinch as the hand turned it (run inh1)
SQUEEZE = finger_for_gap(CAN_TOP - 14.0 * MM)  # the canister settles about 2.8 mm in at about 28 N: fired, short
# of its 3 mm stop (squeezing harder, 18 mm inside, pushed it 4.3 mm, through the joint's stop: run inh2)
SPACER_GRIP = finger_for_gap(2 * SPACER_R - 6.0 * MM)
GRIP_ABOVE = 3.0 * MM  # the right TCP this far above the inhaler's axis at the pick (its pads clear the table)

R_PICK = grip_rotation(0.0)  # pointing down, the fingers closing along y
R_SPACER = grip_rotation(0.0)  # picking the spacer up: pointing down, the fingers either side of the barrel
SPACER_TILT = math.radians(-30.0)  # at the face the left hand leans 30 degrees in from its own side
# WHY: pointing straight down at mouth height, 35-39 cm out on the middle line, the left arm misses by 9-18 mm;
# leaning 15-60 degrees towards +y it reaches within 0.1 mm. The spacer, held, rolls 30 degrees with it.
R_SPACER_ROLLED = rot_x(SPACER_TILT)  # the spacer's own orientation then
INHALER_MID = np.array([0.0, 0.0, CAN_TOP / 2])  # the right TCP on the inhaler's axis, half way up


def _rel(p_tcp, r_tcp, p_obj, r_obj):
    """The object's pose in the TCP's frame."""
    return r_tcp.T @ (np.asarray(p_obj) - np.asarray(p_tcp)), r_tcp.T @ r_obj


PICK_TCP = INHALER_AT + R_LYING @ INHALER_MID + np.array([0.0, 0.0, GRIP_ABOVE])
_P_TI, _R_TI = _rel(PICK_TCP, R_PICK, INHALER_AT, R_LYING)
SPACER_GRIP_AT = np.array([0.075, 0.0, 0.0])  # where the left hand holds the barrel, in the spacer's frame
SPACER_TCP_ABOVE = 5.0 * MM
SPACER_PICK_TCP = SPACER_AT + SPACER_GRIP_AT + np.array([0.0, 0.0, SPACER_TCP_ABOVE])
_P_TS, _R_TS = _rel(SPACER_PICK_TCP, R_SPACER, SPACER_AT, np.eye(3))


def tcp_for_inhaler(pos, rot=np.eye(3)) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP's pose that puts the inhaler (held as at the pick) at ``pos``, ``rot``."""
    r_tcp = np.asarray(rot) @ _R_TI.T
    return np.asarray(pos) - r_tcp @ _P_TI, r_tcp


def tcp_for_spacer(pos, rot=R_SPACER_ROLLED) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose that puts the spacer (held as at the pick) at ``pos``, ``rot`` (level, along x, rolled)."""
    r_tcp = np.asarray(rot) @ _R_TS.T
    return np.asarray(pos) - r_tcp @ _P_TS, r_tcp


SPACER_AIM_UP = 3.0 * MM  # aimed this much above the mouth's middle: the left hand settles about 3 mm low
# (run inh4: the tip 5.3 mm off the middle, of the 6 mm allowed)
SPACER_IN = np.array([MOUTH[0] + 12.0 * MM - SPACER_L, 0.0, MOUTH[2] + SPACER_AIM_UP])  # its tip 12 mm into the mouth
SPACER_BACK = SPACER_IN - np.array([0.04, 0.0, 0.0])  # 4 cm back, before it goes in
FIT_IN = 17.0 * MM  # the mouthpiece aimed this far into the 20 mm socket
# WHY 17: aimed 14 mm in, it reached 10.0-10.7 mm (the inhaler sat about 3 mm further back in the grip than planned
# after the turn and the shakes), on the 10 mm line, and the puff counted as not fitted (run inh3)


def inhaler_in_spacer(spacer_at, depth: float) -> np.ndarray:
    """The inhaler's origin, upright, with its mouthpiece ``depth`` into the socket of a spacer at ``spacer_at``."""
    return np.asarray(spacer_at) + np.array([depth, 0.0, 0.0]) - MOUTHPIECE_TIP


SHAKE_AT = np.array([0.24, -0.09, TOP + 0.135])  # upright, clear of everything
SHAKE_STROKE_DEMO = 50.0 * MM


def demo():
    right, left = [], []

    def both(dur, rp, rr, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rp), rf, label, rr))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    sg = SPACER_PICK_TCP
    high, near = np.array([0.0, 0.0, 0.12]), np.array([0.0, 0.0, 0.02])
    # WHY high: on the way from the ready pose at 6 cm, the left hand's open fingers swept the spacer aside (run inh1)
    both(2.5, PICK_TCP + high, R_PICK, OPEN, (sg + high, R_SPACER), OPEN, "high over the inhaler and the spacer")
    both(1.2, PICK_TCP + near, R_PICK, OPEN, (sg + near, R_SPACER), OPEN, "just over them")
    both(0.5, PICK_TCP + near, R_PICK, OPEN, (sg + near, R_SPACER), OPEN, "settle over them")
    both(0.8, PICK_TCP, R_PICK, OPEN, (sg, R_SPACER), OPEN, "down to them")
    both(1.2, PICK_TCP, R_PICK, HOLD, (sg, R_SPACER), SPACER_GRIP, "grip")
    lift = np.array([0.0, 0.0, 0.08])
    both(1.5, PICK_TCP + lift, R_PICK, HOLD, (sg + lift, R_SPACER), SPACER_GRIP, "lift")
    p, r = tcp_for_inhaler(SHAKE_AT)
    both(3.0, p, r, HOLD, tcp_for_spacer(SPACER_BACK), SPACER_GRIP, "inhaler upright, spacer up to the face")
    for k in range(SHAKES_NEEDED):
        pd, _ = tcp_for_inhaler(SHAKE_AT - np.array([0.0, 0.0, SHAKE_STROKE_DEMO]))
        both(0.35, pd, r, HOLD, tcp_for_spacer(SPACER_BACK), SPACER_GRIP, f"shake {k + 1} down")
        both(0.35, p, r, HOLD, tcp_for_spacer(SPACER_BACK), SPACER_GRIP, f"shake {k + 1} up")
    p, r = tcp_for_inhaler(inhaler_in_spacer(SPACER_BACK, -10.0 * MM))
    both(1.5, p, r, HOLD, tcp_for_spacer(SPACER_BACK), SPACER_GRIP, "behind the socket")
    p, r = tcp_for_inhaler(inhaler_in_spacer(SPACER_BACK, FIT_IN))
    both(1.0, p, r, HOLD, tcp_for_spacer(SPACER_BACK), SPACER_GRIP, "fit it in")
    p, r = tcp_for_inhaler(inhaler_in_spacer(SPACER_IN, FIT_IN))
    both(1.8, p, r, HOLD, tcp_for_spacer(SPACER_IN), SPACER_GRIP, "into the mouth")
    both(0.5, p, r, HOLD, tcp_for_spacer(SPACER_IN), SPACER_GRIP, "settle")
    both(0.6, p, r, SQUEEZE, tcp_for_spacer(SPACER_IN), SPACER_GRIP, "squeeze once")
    both(0.4, p, r, SQUEEZE, tcp_for_spacer(SPACER_IN), SPACER_GRIP, "hold the squeeze")
    both(0.4, p, r, HOLD, tcp_for_spacer(SPACER_IN), SPACER_GRIP, "ease off")
    both(6.0, p, r, HOLD, tcp_for_spacer(SPACER_IN), SPACER_GRIP, "five breaths")
    return {"left": left, "right": right}
