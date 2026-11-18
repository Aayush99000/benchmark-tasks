"""On the station table: an interchangeable-lens camera body lying on its back, its lens mount facing up, with its kit
zoom lens on it, and a new 50 mm lens standing on a lens cloth beside it. Change the lens: while pressing the lens
release button, turn the zoom lens until it stops and lift it off, and set it down on the cloth; then line the new
lens's red mount index up with the camera's red index, set it on the mount, and turn it until it clicks.

The mount is a model of a Canon-type bayonet: a lens goes onto the mount only with its index within 6 degrees of the
camera's (otherwise its tabs rest on the mount's, 2.5 mm up); seated, it turns clockwise (seen from the front) through
60 degrees to a stop, where the lock pin clicks into it; a locked lens turns back only while the release button is
pressed in.

Run it:  inspect-robots run --embodiment yam_arena -E task=camera_lens.py -E object_jitter=0 -E control_hz=20 \\
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
from fittingkit import Slider, body, box, cyl, ring  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

BODY_BLACK = (0.07, 0.07, 0.08)
RUBBER = (0.12, 0.12, 0.13)
MOUNT_STEEL = (0.72, 0.73, 0.75)
INDEX_RED = (0.85, 0.08, 0.08)
GLASS = (0.10, 0.14, 0.22)
CLOTH = (0.55, 0.62, 0.70)
WHITE = (0.92, 0.92, 0.90)


def rot_z(angle: float) -> np.ndarray:
    return axis_angle_matrix(Z_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the camera (fixed): on its back, the mount facing up ---------------------------------------------------------------
# The camera's top points to the robot's right (-y), so its red mount index is on the mount's -y side; the lens release
# button is on the mount's side towards the arms (-x), and the grip on the far side (+x).
# [estimate: a Canon APS-C body, 130 x 100 x 78 mm; the release button 50 mm from the mount's middle]
MOUNT = np.array([0.27, 0.0])  # the mount's middle, on the table
# WHY 27 cm out: at 30 cm the right arm reaches a straight-down grip over the mount only up to 16 cm above the table
FACE_H = 0.066  # the mount's face, above the table
MOUNT_AT = np.array([MOUNT[0], MOUNT[1], TOP + FACE_H])
THROAT_R, RING_R = 27.0 * MM, 36.0 * MM  # the mount ring: the throat a lens's bayonet goes into, and its outside
BODY_FRONT = 0.060  # the body's front (facing up), 6 mm under the mount's face
INDEX_ANGLE = -math.pi / 2  # the camera's red index: on the mount's -y side (the camera's top)
BUTTON_R, BUTTON_ANGLE = 50.0 * MM, math.pi  # the lens release button: 50 mm out, on the -x side
BUTTON_RADIUS, BUTTON_TRAVEL, UNLOCK = 5.5 * MM, 3.0 * MM, 1.5 * MM
BUTTON_K = 1800.0  # N/m: 2.7 N at the unlock point [estimate: a thumb press]
BUTTON_AT = MOUNT_AT + np.array([BUTTON_R * math.cos(BUTTON_ANGLE), BUTTON_R * math.sin(BUTTON_ANGLE),
                                 BODY_FRONT + 0.003 - FACE_H])  # its middle; its top 6 mm above the body's front
CAMERA_AT = np.array([MOUNT[0], MOUNT[1], TOP])
X0, X1, Y0, Y1 = -0.060, 0.072, -0.056, 0.046  # the body's extent from the mount's middle (x: button side to grip)


def _round_well(x0, x1, y0, y1, wx, wy, hx, hy, z0, z1) -> list:
    """A slab from x0..x1, y0..y1 and z0..z1 with a hole 2 * ``hx`` by 2 * ``hy`` round (wx, wy), as up to four boxes."""
    z, t = (z0 + z1) / 2, z1 - z0
    a, b = max(x0, wx - hx), wx + hx
    out = [box(((x0 + a) / 2, (y0 + y1) / 2, z), (a - x0, y1 - y0, t), BODY_BLACK)] if a > x0 else []
    return out + [
        box(((b + x1) / 2, (y0 + y1) / 2, z), (x1 - b, y1 - y0, t), BODY_BLACK),
        box(((a + b) / 2, (y0 + wy - hy) / 2, z), (b - a, wy - hy - y0, t), BODY_BLACK),
        box(((a + b) / 2, (wy + hy + y1) / 2, z), (b - a, y1 - wy - hy, t), BODY_BLACK),
    ]


def _camera_shapes() -> list:
    """In the camera's frame (origin on the table under the mount's middle; the world's axes)."""
    hole = 0.034  # the square opening in the body's front round the mount ring
    cx, cy = (X0 + X1) / 2, (Y0 + Y1) / 2
    lo = 0.050  # the body's solid part, under the mount's throat
    t = BODY_FRONT - lo
    z = (lo + BODY_FRONT) / 2
    return [
        box((cx, cy, lo / 2), (X1 - X0, Y1 - Y0, lo), BODY_BLACK),
        box(((hole + X1) / 2, cy, z), (X1 - hole, Y1 - Y0, t), BODY_BLACK),
        # the front on the button's side, round a well the button goes down into
        # WHY a well: a joint doesn't keep the button from colliding with the fixed body it slides in, and sitting on the
        # front the button couldn't be pressed at all (run cl1: 0.0 mm all run, the zoom lens stayed locked)
        # WHY 18 x 30 mm: with a well 4 mm wider than the button, the fingertips' 26 mm width landed on the front with
        # the button only 1.5 mm in (run cl2)
        *_round_well(X0, -hole, Y0, Y1, BUTTON_AT[0] - MOUNT[0], BUTTON_AT[1] - MOUNT[1], 9.0 * MM, 15.0 * MM,
                     lo, BODY_FRONT),
        box((0.0, (hole + Y1) / 2, z), (2 * hole, Y1 - hole, t), BODY_BLACK),
        box((0.0, (Y0 - hole) / 2, z), (2 * hole, -hole - Y0, t), BODY_BLACK),
        *ring(lo, FACE_H, THROAT_R, RING_R, MOUNT_STEEL),
        # the grip, on the far side, its front 4 mm under the mount's face
        box((X1 - 0.016, cy, (FACE_H - 0.004) / 2), (0.032, Y1 - Y0, FACE_H - 0.004), RUBBER),
    ]


def _camera_looks() -> list:
    """Drawn only: the red index, the viewfinder hump and hot shoe at the camera's top (-y), the shutter button and
    dial on the grip, the lens contacts in the throat."""
    r = RING_R + 4.0 * MM
    return [
        cyl((r * math.cos(INDEX_ANGLE), r * math.sin(INDEX_ANGLE), BODY_FRONT + 0.0004), 2.5 * MM, 0.0008, INDEX_RED),
        box((0.0, Y0 - 0.008, 0.032), (0.060, 0.016, 0.040), BODY_BLACK),  # the viewfinder hump on the top
        box((0.0, Y0 - 0.0165, 0.032), (0.020, 0.001, 0.018), MOUNT_STEEL),  # the hot shoe
        cyl((X1 - 0.016, Y0 + 0.012, FACE_H - 0.004 + 0.002), 5.0 * MM, 0.004, MOUNT_STEEL),  # the shutter button
        cyl((X1 - 0.040, Y0 - 0.002, 0.040), 11.0 * MM, 0.010, RUBBER),  # the mode dial
        box((0.0, -THROAT_R + 3.0 * MM, 0.052), (0.020, 0.003, 0.002), (0.75, 0.62, 0.25)),  # the contacts
    ]


_CAM_SOLID = _camera_shapes()
_CAM_SEEN = _camera_looks()
CAMERA = body("camera", pos=tuple(CAMERA_AT), friction=0.6, shapes=_CAM_SOLID + _CAM_SEEN,
              visual_only=tuple(range(len(_CAM_SOLID), len(_CAM_SOLID) + len(_CAM_SEEN))))
BUTTON = body("release_button", pos=tuple(BUTTON_AT), mass=0.005, friction=0.8,
              shapes=[cyl((0.0, 0.0, 0.0), BUTTON_RADIUS, 0.006, MOUNT_STEEL)])
fittingkit.add_slider(Slider("camera", "release_button", anchor=tuple(BUTTON_AT - CAMERA_AT), axis=(0, 0, 1),
                             lower=-BUTTON_TRAVEL, upper=0.0, stiffness=BUTTON_K, damping=2.0, target=0.0))

# -- the lenses ---------------------------------------------------------------------------------------------------------
# A lens's frame: origin in the middle of its mount face (which sits on the camera's mount face), z along its axis
# towards its front, x towards its red index. Its bayonet (25 mm round, 8 mm deep) goes into the camera's throat.
LENS_R = 34.6 * MM  # both 69.2 mm across (Canon EF-S 18-55 mm IS STM: 69 x 75.2 mm; EF 50 mm f/1.8 STM: 69.2 x 39.3 mm)
BAYONET_R, BAYONET_H = 25.0 * MM, 8.0 * MM
OLD_L, NEW_L = 75.0 * MM, 39.3 * MM


def _lens_shapes(length: float, zoom: bool) -> tuple[list, list]:
    solid = [cyl((0.0, 0.0, -BAYONET_H / 2), BAYONET_R, BAYONET_H, MOUNT_STEEL),
             cyl((0.0, 0.0, length / 2), LENS_R, length, BODY_BLACK)]
    seen = [
        box((LENS_R + 1.2 * MM, 0.0, 6.0 * MM), (3.0 * MM, 5.0 * MM, 6.0 * MM), INDEX_RED),  # the red index
        cyl((0.0, 0.0, length - 1.0 * MM), LENS_R - 6.0 * MM, 0.0024, GLASS),  # the front element
        cyl((0.0, 0.0, length - 0.4 * MM), LENS_R - 2.0 * MM, 0.0010, RUBBER),  # the front ring (under the glass)
    ]
    if zoom:  # the zoom ring's rubber grip and its focal-length marks
        seen.append(cyl((0.0, 0.0, 0.040), LENS_R + 0.6 * MM, 0.022, RUBBER))
        seen += [box((math.cos(a) * (LENS_R + 0.7 * MM), math.sin(a) * (LENS_R + 0.7 * MM), 0.054),
                     (1.5 * MM, 1.5 * MM, 3.0 * MM), WHITE) for a in (0.0, 0.5, 1.0)]
    else:  # the focus ring
        seen.append(cyl((0.0, 0.0, 0.020), LENS_R + 0.5 * MM, 0.012, RUBBER))
    return solid, seen


LOCK_DEG = 60.0  # the turn from the index to the lock [estimate: Canon EF about 60 degrees]
R_INSERT = rot_z(INDEX_ANGLE)  # a lens's orientation with its index on the camera's: as it goes on
R_LOCKED = rot_z(INDEX_ANGLE - math.radians(LOCK_DEG))  # turned clockwise, seen from the front (above), to the lock

_OS, _OV = _lens_shapes(OLD_L, zoom=True)
OLD_LENS = body("old_lens", pos=tuple(MOUNT_AT), mass=0.205, friction=0.8, shapes=_OS + _OV,
                visual_only=tuple(range(len(_OS), len(_OS) + len(_OV))), quat=quat_xyzw(R_LOCKED),
                com=(0.0, 0.0, 0.032))

CLOTH_AT = np.array([0.25, -0.175, TOP])
CLOTH_SIZE = (0.24, 0.12, 0.004)
CLOTH_TOP = TOP + CLOTH_SIZE[2]
LENS_CLOTH = body("lens_cloth", pos=tuple(CLOTH_AT), friction=0.8,
                  shapes=[box((0.0, 0.0, CLOTH_SIZE[2] / 2), CLOTH_SIZE, CLOTH)])
NEW_AT = np.array([0.19, -0.175, CLOTH_TOP + BAYONET_H])  # standing on its back end, its index to -y
SET_DOWN_AT = np.array([0.31, -0.175, CLOTH_TOP + BAYONET_H])  # where the demo stands the old lens
_NS, _NV = _lens_shapes(NEW_L, zoom=False)
NEW_LENS = body("new_lens", pos=tuple(NEW_AT), mass=0.160, friction=0.8, shapes=_NS + _NV,
                visual_only=tuple(range(len(_NS), len(_NS) + len(_NV))), quat=quat_xyzw(R_INSERT),
                com=(0.0, 0.0, 0.016))

OBJECTS = [CAMERA, BUTTON, OLD_LENS, LENS_CLOTH, NEW_LENS]
LENSES = ("old_lens", "new_lens")
LENGTH = {"old_lens": OLD_L, "new_lens": NEW_L}


# -- the mount ----------------------------------------------------------------------------------------------------------
INSERT_DEG = 6.0  # a lens goes on only with its index within this of the camera's
CENTRED, SEATED, TILT_DEG = 2.5 * MM, 1.0 * MM, 6.0  # seated: within these of the mount's axis and face
TABS_H = 2.5 * MM  # a lens not lined up rests on the mount's tabs this far up
TABS_K, TABS_MAX = 20000.0, 40.0  # the tabs: a stiff stop (N/m), up to 40 N
LEAD_R, LEAD_K, LEAD_MAX = 6.0 * MM, 1500.0, 6.0  # the throat's chamfer: within 6 mm of the axis it steers a lens's back in
ENGAGE_DEG, FREE_DEG = 4.0, 2.0  # turned this far towards the lock it is in the bayonet; back within this, it's free
CLICK_DEG = 0.5  # within this of the stop it locks
CLEAR = 0.02  # both grippers this far from both lenses at the end
DOWN_Z = TOP + 0.06  # a lens set down: its centre of mass under this (on the table or the cloth, not on the camera)


def twist_deg(rot: np.ndarray) -> float:
    """How far (degrees) a lens's index is turned from the camera's, about the mount's axis; negative is clockwise
    seen from the front (towards the lock)."""
    x = rot[:, 0]
    a = math.atan2(x[1], x[0]) - INDEX_ANGLE
    return math.degrees((a + math.pi) % (2 * math.pi) - math.pi)


def seat_offset(lens, origin=np.zeros(3)) -> tuple[float, float, float]:
    """(how far its mount face is above the camera's, how far its axis is off the mount's, how far it is tilted)."""
    p = lens.pos - origin - MOUNT_AT
    tilt = math.degrees(math.acos(float(np.clip(lens.rot[2, 2], -1.0, 1.0))))
    return float(p[2]), float(math.hypot(p[0], p[1])), tilt


class Mount:
    """Runs before every physics substep (240 Hz): the bayonet, from the lenses' and the release button's poses.

    A lens over the mount not lined up with the index rests on its tabs (a stiff push up). Lined up, it goes down onto
    the face; turned clockwise from there it is in the bayonet: held on the mount's axis and face, free only to turn,
    between the index and the stop 60 degrees on. At the stop the lock pin clicks in and holds it there; with the
    release button pressed in, it turns back, and at the index it comes off."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("LENS_LOG")
        self.trace = os.environ.get("LENS_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["old_lens", "new_lens", "release_button"]

    def clear(self) -> None:
        self._n = 0
        self.engaged = {"old_lens": True, "new_lens": False}
        self.locked = {"old_lens": True, "new_lens": False}
        self.twist = {"old_lens": -LOCK_DEG, "new_lens": 0.0}
        self.seated = {"old_lens": True, "new_lens": False}
        self.clicks = {"old_lens": 0, "new_lens": 0}
        self.button_in = 0.0
        self.frames = {}
        self.poses = {}

    def pressed(self) -> bool:
        return self.button_in >= UNLOCK

    def update(self, frames: dict, acc: dict) -> None:
        btn = frames.get("release_button")
        if btn is None or any(frames.get(n) is None for n in LENSES):
            return
        self._n += 1
        self.button_in = float(BUTTON_AT[2] - (btn.pos[2] - self.origin[2]))
        self.poses = {}
        for name in LENSES:
            self._lens(name, frames[name], acc)
        self.frames = {n: frames[n] for n in LENSES}
        if self.trace and self._n % 24 == 0:
            self._write_trace()

    def _lens(self, name: str, f, acc: dict) -> None:
        up, off, tilt = seat_offset(f, self.origin)
        tw = twist_deg(f.rot)
        self.twist[name] = tw
        on_axis = off < CENTRED and tilt < TILT_DEG
        self.seated[name] = self.engaged[name] or (on_axis and abs(up) < SEATED)
        if not self.engaged[name]:
            # WHY a lead-in: with 2 mm between a lens's back and the throat, a lens brought down 4 mm off the axis sat
            # on the ring (run cl3); a real mount's throat and a lens's back are chamfered, and steer it in
            rel = f.pos - self.origin - MOUNT_AT
            if off < LEAD_R and -1.0 * MM < up < BAYONET_H + 4.0 * MM and tilt < 15.0:
                push = -LEAD_K * np.array([rel[0], rel[1], 0.0])
                n = np.linalg.norm(push)
                if n > LEAD_MAX:
                    push *= LEAD_MAX / n
                if n > 1e-3:
                    fittingkit._wrench(acc, name, f, push, f.pos - f.rot[:, 2] * (BAYONET_H / 2))
            if on_axis and abs(up) < SEATED and tw < -ENGAGE_DEG:
                self.engaged[name] = True
                self._note(f"{self._n / 240:7.2f}s {name} into the bayonet ({tw:5.1f} deg)")
            elif off < THROAT_R - BAYONET_R + 1.0 * MM and tilt < 15.0 and -3.0 * MM < up < TABS_H \
                    and abs(tw) > INSERT_DEG:
                lift = min(TABS_MAX, TABS_K * (TABS_H - up))  # not lined up: on the tabs
                fittingkit._wrench(acc, name, f, Z_AXIS * lift, f.pos)
                return
            else:
                return
        # in the bayonet: on the axis and the face, turned between the index and the stop
        # WHY the pin lets go as soon as the button is in: held at the stop every substep, a locked lens could never be
        # seen to turn off it; let go of at the stop, the pin clicks straight back in
        if self.locked[name]:
            if self.pressed():
                self.locked[name] = False
                self._note(f"{self._n / 240:7.2f}s {name} lock pin out (button in {1000 * self.button_in:3.1f} mm)")
            else:
                tw = -LOCK_DEG
        tw = min(0.0, max(-LOCK_DEG, tw))
        if not self.locked[name] and tw <= -LOCK_DEG + CLICK_DEG and not self.pressed():
            self.locked[name] = True
            self.clicks[name] += 1
            tw = -LOCK_DEG
            self._note(f"{self._n / 240:7.2f}s {name} clicks, locked")
        if not self.locked[name] and tw > -FREE_DEG:
            self.engaged[name] = False
            self._note(f"{self._n / 240:7.2f}s {name} out of the bayonet, free to lift off")
            return
        self.twist[name] = tw
        rot = rot_z(INDEX_ANGLE + math.radians(tw))
        self.poses[name] = (MOUNT_AT + self.origin, quat_xyzw(rot))

    def _write_trace(self) -> None:
        """Debug only (LENS_TRACE=<file>): every 0.1 s, the button, each lens and both TCPs (mm, from the table)."""
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        parts = []
        for n in LENSES:
            p = 1000 * (self.frames[n].pos - self.origin)
            parts.append(f"{n[:3]} ({p[0]:6.1f},{p[1]:6.1f},{p[2] - 1000 * TOP:6.1f}) tw {self.twist[n]:6.1f} "
                         f"e{int(self.engaged[n])} l{int(self.locked[n])}")
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s btn {1000 * self.button_in:4.1f} | " + " | ".join(parts) + f" |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


MODEL = fittingkit.add_model(Mount())

# -- scoring -------------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def _lens_frame(name: str):
    return MODEL.frames.get(name)


def old_lens_off() -> bool:
    """The old lens out of the bayonet and off the mount (its mount face 5 mm above it, or 5 mm off its axis)."""
    if MODEL.engaged["old_lens"]:
        return False
    f = _lens_frame("old_lens")
    if f is None:
        return False
    up, off, _ = seat_offset(f, MODEL.origin)
    return up > 5.0 * MM or off > 5.0 * MM


def old_lens_set_down() -> bool:
    """Off the camera and resting low, on the table or the cloth (its centre of mass under 6 cm up, still)."""
    f = _lens_frame("old_lens")
    if f is None or not old_lens_off():
        return False
    com = f.world(OLD_LENS.com) - MODEL.origin
    return bool(com[2] < DOWN_Z and np.linalg.norm(f.vel) < 0.02)


def hands_clear() -> bool:
    """No gripper point within 2 cm of either lens's barrel; unknown offline counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0:
        return True
    for name in LENSES:
        f = _lens_frame(name)
        if f is None:
            continue
        rel = pts - f.pos
        t = np.clip(rel @ f.rot[:, 2], -BAYONET_H, LENGTH[name])
        radial = np.linalg.norm(rel - np.outer(t, f.rot[:, 2]), axis=1)
        if (radial - LENS_R).min() < CLEAR:
            return False
    return True


def success(objects) -> bool:
    """The new lens locked on the camera, the old lens set down off it, the release button let go, and both grippers
    clear of both lenses. Must hold for 1 s."""
    return (MODEL.locked["new_lens"] and old_lens_set_down() and MODEL.button_in < 0.5 * MM and hands_clear())


def score(objects) -> float:
    """0.9 for the new lens locked with the old one set down and the button let go (1.0 once success() has held 1 s,
    the hands clear); 0.5 for the new lens on the mount otherwise; 0.25 for the old lens off the mount; 0 otherwise."""
    if MODEL.locked["new_lens"] and old_lens_set_down() and MODEL.button_in < 0.5 * MM:
        return COMPLETE_SCORE
    if MODEL.seated["new_lens"]:
        return 0.5
    return 0.25 if old_lens_off() else 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# The left hand, fingers closed, presses the release button in and holds it while the right hand, holding the zoom lens
# near its front, turns it 60 degrees anticlockwise (seen from above) to its index and lifts it off. The left hand lets
# the button go; the right stands the zoom lens on the cloth, picks up the new lens, sets it on the mount with its index
# on the camera's, turns it 60 degrees clockwise until it clicks, and lets go.
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

# The right hand holds each lens from above, its fingers closing across it along y (turned up to 30 degrees either way),
# clear of the release button (-x) and the grip (+x).
OLD_GRIP = 60.0 * MM  # the TCP this far up the zoom lens's axis (the fingertips 47 mm up, near its front)
NEW_GRIP = 28.0 * MM  # and up the 50 mm lens's (the fingertips 15 mm up)
TURN_YAW = math.radians(30.0)
UNTURN_PAST = math.radians(2.0)  # turned this far past the index taking the zoom lens off, so it's surely free
TURN_PAST = math.radians(1.5)  # and past the stop putting the new lens on [estimate: the lock holds it at the stop]
SEAT_PAST = 1.5 * MM  # the new lens aimed this far under the mount's face [estimate: compliance]
SET_PAST = 1.0 * MM  # the zoom lens aimed this far under the cloth, standing it down
HAND_TWIST = math.radians(4.5)  # the new lens turns this far (anticlockwise) in the fingers as they close (runs cl2, cl3)

# The left hand presses the button with its closed fingertips, pointing down and leaning 30 degrees out towards the
# arms (-x), its fingers closing along x, so its fingers stay clear of the lens and of the right hand.
PRESS_LEAN = math.radians(30.0)
_d = np.array([math.sin(PRESS_LEAN), 0.0, -math.cos(PRESS_LEAN)])  # from the palm to the fingertips
# WHY from the straight-down grip turned -90 degrees: built with the fingers closing along +x instead (that grip turned
# 90 degrees), the left arm missed the pose by 60 mm and 150 degrees
R_PRESS = axis_angle_matrix(Y_AXIS, -PRESS_LEAN) @ grip_rotation(-math.pi / 2)
assert np.allclose(R_PRESS[:, 2], _d)
BUTTON_TOP = BUTTON_AT + np.array([0.0, 0.0, 0.003])
# WHY 9 mm: the arm pushes like a spring of about 800 N/m; aimed 5.5 mm under the top of a 2,500 N/m button, it held it
# only 1.5 mm in (runs cl2, cl3); aimed 9 mm under this one it goes in about 2.8 mm (it stops at 3)
PRESS_PAST = 9.0 * MM  # the fingertips aimed this far under the button's top (it unlocks at 1.5, stops at 3)


def press_pose(depth: float, back: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose with its closed fingertips ``depth`` under the button's top (or ``back`` away from it along
    the hand)."""
    tip = BUTTON_TOP - np.array([0.0, 0.0, depth]) - _d * back
    return tip - _d * PAD_TIP, R_PRESS


# WHY the hand off the lens's axis: in the sim only one of the YAM's fingers closes on a lens this wide; the other stays
# at its open stop and the lens is pushed against it, so a held lens's axis ends 12.4 mm (47 mm open, less its 34.6 mm
# radius) from the TCP along the fingers' line (runs cl2: the new lens ended 12.2 mm off, and the hand holding the locked
# zoom lens was pushed 11.6 mm). Centred over the mount, the new lens landed 13.7 mm off its axis, on the ring.
IN_HAND = 12.4 * MM  # a held lens's axis from the TCP, towards the finger that stays open
REACH_IN = 10.0 * MM  # coming down round a lens, the TCP this far off its axis (the open finger 2.4 mm clear of it)
LET_GO = 9.0 * MM  # opening the fingers, the TCP moved to this far off (the open finger 3.4 mm clear of it)


def grip(axis_at, yaw: float, off: float = IN_HAND) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP's pose straight down, turned ``yaw``, with a lens's axis at ``axis_at`` (at the TCP's height)
    ``off`` from the TCP towards the finger that stays open (the TCP's -x)."""
    r = grip_rotation(yaw)
    return np.asarray(axis_at, float) - off * r[:, 0], r


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    up = lambda h: np.array([0.0, 0.0, h])  # noqa: E731
    old_on = MOUNT_AT + up(OLD_GRIP)
    y0, y1 = -TURN_YAW, TURN_YAW + UNTURN_PAST
    l_wait, l_near = press_pose(0.0, back=0.08), press_pose(0.0, back=0.015)
    both(2.5, grip(old_on + up(0.04), y0, REACH_IN), OPEN, l_wait, CLOSED, "high over the zoom lens")
    # WHY the left hand waits until the right has closed: the open right fingers, turned 30 degrees, reach out over the
    # button's side, and with the left fingertip 15 mm over the button the hands came within 2.3 mm (plan_check)
    both(1.5, grip(old_on + up(0.01), y0, REACH_IN), OPEN, l_wait, CLOSED, "just over it")
    both(0.8, grip(old_on, y0, REACH_IN), OPEN, l_wait, CLOSED, "round it")
    both(2.0, grip(old_on, y0), CLOSED, l_wait, CLOSED, "hold it")
    both(1.2, grip(old_on, y0), CLOSED, l_near, CLOSED, "finger over the button")
    both(1.0, grip(old_on, y0), CLOSED, press_pose(PRESS_PAST), CLOSED, "press the release button")
    both(0.5, grip(old_on, y0), CLOSED, press_pose(PRESS_PAST), CLOSED, "hold the button in")
    # WHY the button let go half way round: the lock pin only catches at the stop, and held in to the end, the left
    # fingers were within 8.5 mm of the right hand's, turned 32 degrees the other way
    both(1.5, grip(old_on, 0.0), CLOSED, press_pose(PRESS_PAST), CLOSED, "turn it off the lock")
    both(1.5, grip(old_on, y1), CLOSED, press_pose(0.0, back=0.05), CLOSED, "let the button go, turn it to the stop")
    both(1.2, grip(old_on + up(0.035), y1), CLOSED, l_wait, CLOSED, "lift it off")
    set_at = SET_DOWN_AT + up(OLD_GRIP)
    both(2.5, grip(set_at + up(0.035), y1), CLOSED, l_wait, CLOSED, "over the cloth")
    both(1.2, grip(set_at - up(SET_PAST), y1), CLOSED, l_wait, CLOSED, "stand it on the cloth")
    both(2.2, grip(set_at - up(SET_PAST), y1, LET_GO), OPEN, l_wait, CLOSED, "let it go")
    both(1.0, grip(set_at + up(0.06), y1, LET_GO), OPEN, l_wait, CLOSED, "up from it")
    new_at = NEW_AT + up(NEW_GRIP)
    y2 = TURN_YAW
    y2c = y2 - HAND_TWIST  # carried turned back by what the lens turned in the fingers, so it arrives on the index
    y3 = y2c - math.radians(LOCK_DEG) - TURN_PAST
    both(2.0, grip(new_at + up(0.08), y2, REACH_IN), OPEN, l_wait, CLOSED, "over the new lens")
    both(1.2, grip(new_at, y2, REACH_IN), OPEN, l_wait, CLOSED, "round the new lens")
    both(2.0, grip(new_at, y2), CLOSED, l_wait, CLOSED, "hold the new lens")
    both(1.2, grip(new_at + up(0.10), y2), CLOSED, l_wait, CLOSED, "lift it")
    new_on = MOUNT_AT + up(NEW_GRIP)
    both(2.5, grip(new_on + up(0.04), y2c), CLOSED, l_wait, CLOSED, "over the mount, index to the red dot")
    both(1.5, grip(new_on - up(SEAT_PAST), y2c), CLOSED, l_wait, CLOSED, "set it on the mount")
    both(0.5, grip(new_on - up(SEAT_PAST), y2c), CLOSED, l_wait, CLOSED, "settle it")
    # turned about the lens's axis, not the TCP: in two halves, so the TCP's straight path between them stays within
    # 0.4 mm of the arc round the axis
    both(1.25, grip(new_on - up(SEAT_PAST), (y2c + y3) / 2), CLOSED, l_wait, CLOSED, "turn it towards the click")
    both(1.25, grip(new_on - up(SEAT_PAST), y3), CLOSED, l_wait, CLOSED, "turn it until it clicks")
    both(2.2, grip(new_on, y3, LET_GO), OPEN, l_wait, CLOSED, "let the new lens go")
    both(1.2, grip(new_on + up(0.07), y3, LET_GO), OPEN, press_pose(0.0, back=0.12), CLOSED, "hands clear")
    both(1.5, grip(new_on + up(0.07), y3, LET_GO), OPEN, press_pose(0.0, back=0.12), CLOSED, "wait")
    return {"left": left, "right": right}
