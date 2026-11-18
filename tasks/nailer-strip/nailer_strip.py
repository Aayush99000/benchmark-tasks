"""On the station table: a pneumatic 21-degree framing nailer, not connected to air, lying on its side with its
magazine empty, and a strip of 25 plastic-collated 3 in x .131 framing nails lying beside it. Load the nailer: feed
the strip into the rear of the magazine, the nail heads in the magazine's head track, and push it in past the nail
stop; then slide the pusher back past the strip and ease it forward against the nails, without letting it snap.

The magazine is a model of a stick nailer's: its track takes the strip only with the heads in the head track (the
heads are thicker than the slot the shanks run in); a strip pushed in past the nail stop, 35 mm in, can't slide back
out; the spring pusher's finger rides over the strip when pulled back over it (dragging a strip that isn't past the
stop back out with it) and drops in behind the last nail once pulled back past it; let forward, it pushes the strip
up to the driver. A pusher that hits the strip, or reaches the driver, faster than 0.25 m/s has snapped.

Run it:  inspect-robots run --embodiment yam_arena -E task=nailer_strip.py -E object_jitter=0 -E control_hz=20 \\
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
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

TOOL_RED = (0.72, 0.10, 0.08)
MAG_ALU = (0.62, 0.64, 0.67)
DARK = (0.10, 0.10, 0.11)
RUBBER = (0.16, 0.16, 0.17)
STEEL = (0.78, 0.79, 0.81)
COLLATION = (0.95, 0.48, 0.08)  # the strip's orange plastic collation
PUSHER_BLACK = (0.08, 0.08, 0.09)
YELLOW = (0.95, 0.80, 0.10)


def rot_z(angle: float) -> np.ndarray:
    return axis_angle_matrix(Z_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the strip ------------------------------------------------------------------------------------------------------------
# 25 plastic-collated 21-degree full round head nails, 3 in x .131 in (Grabber PS300131BR: 25 a strip). The strip's frame:
# x along the strip towards its front (the end that goes into the magazine first), y along the nails towards their
# heads, z through the strip's face. Seen face up, the nails lean 21 degrees: each point leads its head along +x.
NAILS = 25
NAIL_L = 76.2 * MM  # 3 in, head included
SHANK_D = 3.33 * MM  # .131 in
HEAD_D, HEAD_T = 7.14 * MM, 1.7 * MM  # .281 in full round head [estimate: the head's thickness]
PITCH = 7.6 * MM  # along the strip, head to head [estimate: the heads just clear each other]
LEAN = NAIL_L * math.tan(math.radians(21.0))  # 29.3 mm: how far a point leads its head
STRIP_L = HEAD_D / 2 + (NAILS - 1) * PITCH + LEAN  # 215 mm, from the last head's back to the first point
BAND_T = 5.0 * MM  # the plastic collation round the shanks [estimate]
STRIP_MASS = 0.13  # 25 x 5.1 g of steel, and the plastic


def _strip_shapes() -> tuple[list, list]:
    half = STRIP_L / 2
    y_head = NAIL_L / 2 - HEAD_T  # the heads' underside
    solid = [
        # the shanks and their collation, as one band; the heads, as a thicker band along the heads' edge
        box((0.0, (-NAIL_L / 2 + y_head) / 2, 0.0), (STRIP_L, y_head + NAIL_L / 2, BAND_T), COLLATION),
        box((-LEAN / 2, NAIL_L / 2 - HEAD_T / 2, 0.0), (STRIP_L - LEAN, HEAD_T, HEAD_D), STEEL),
    ]
    seen = []
    lean = math.radians(21.0)
    for i in range(NAILS):
        xh = -half + HEAD_D / 2 + i * PITCH  # the head's middle
        seen.append(cyl((xh, NAIL_L / 2 - HEAD_T / 2, 0.0), HEAD_D / 2, HEAD_T, STEEL, axis="Y", yaw=lean))
        seen.append(box((xh + LEAN / 2, -HEAD_T / 2, 0.0), (SHANK_D, NAIL_L - HEAD_T, SHANK_D), STEEL, yaw=lean))
    for y in (0.016, -0.012):  # the two plastic bands across the shanks
        seen.append(box((LEAN * (0.5 - (y + NAIL_L / 2) / NAIL_L), y, 0.0), (STRIP_L - LEAN, 6.0 * MM, BAND_T + 0.3 * MM),
                        COLLATION))
    return solid, seen


# -- the nailer (fixed): lying on its side, its magazine along the table's y, its nose to the robot's right --------------
# The magazine frame: origin in the middle of the track's rear end; a into the magazine (towards the driver, -y), w
# towards the heads' side of the track (+x, the side the tool's body and handle are on), n up.
X_C = 0.17  # the track's middle line
Y_REAR = 0.13  # the magazine's rear end, where the strip goes in
A_DRIVE = 0.34  # from the rear end to the driver, where the first nail stops (the track's length)
Z_S = TOP + 0.034  # the middle of the shanks' slot
R_MAG = np.column_stack([-Y_AXIS, X_AXIS, Z_AXIS])  # the magazine's (a, w, n) axes in the world
MAG_AT = np.array([X_C, Y_REAR, Z_S])
SLOT_T = 7.5 * MM  # the shanks' slot (the strip's band is 5 mm)
HEAD_TRACK_T = 11.5 * MM  # the head track (the heads are 7.14 mm)
# WHY 83 mm: with 81, a strip held 2.6 mm off the track's line caught its corner on a wall's end (run ns7)
TRACK_W = 83.0 * MM  # the track across, wall to wall (the strip is 76.2 mm)
FLARE_L, FLARE = 15.0 * MM, math.radians(15.0)  # the walls' last 15 mm open out 15 degrees: the rear end's lead-in
HEAD_TRACK_W = 7.5 * MM  # the head track's width along the nails, against the heads' wall
WALL = 6.5 * MM
WINDOW = 55.0 * MM  # the track's last 55 mm is open on top, for loading
# WHY 35 mm: riding back over the strip, the pusher's finger drags it back onto the nail stop; with the stop 28 mm in,
# the finger, pulled back as far as the right arm reaches (29 mm), was still over the strip's last nail (run ns16)
TAB = 35.0 * MM  # the nail stop, this far into the track
RAIL_W = 15.0 * MM  # the pusher's rail, beside the heads' wall


def mag(a: float, w: float, n: float) -> np.ndarray:
    """A point given in the magazine's frame, in the world."""
    return MAG_AT + R_MAG @ np.array([a, w, n])


def _xb(x0, x1, y0, y1, z0, z1, color) -> object:
    """A box from world x0..x1, y0..y1, z0..z1, in the nailer's frame (origin on the table under the track's rear end)."""
    o = np.array([X_C, Y_REAR, TOP])
    c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]) - o
    return box(tuple(c), (x1 - x0, y1 - y0, z1 - z0), color)


Y_DRIVE = Y_REAR - A_DRIVE
W_IN = TRACK_W / 2  # the walls' inner faces, from the track's middle line
X_WALL_P, X_WALL_H = X_C - W_IN, X_C + W_IN  # the points' wall and the heads' wall
Z_FLOOR, Z_CEIL = Z_S - SLOT_T / 2, Z_S + SLOT_T / 2
Z_HFLOOR, Z_HCEIL = Z_S - HEAD_TRACK_T / 2, Z_S + HEAD_TRACK_T / 2
X_HTRACK = X_WALL_H - HEAD_TRACK_W  # where the head track starts
Z_COVER = Z_HCEIL + 3.0 * MM  # the top of the magazine's cover (and of the rail)
X_RAIL0, X_RAIL1 = X_WALL_H + WALL, X_WALL_H + WALL + RAIL_W
Y_WIN = Y_REAR - WINDOW
# WHY a pocket: the strip's back end is only 5 mm thick, and with the fingertips pushing it from the slot's floor their
# rounded ends rode up over it (run ns5). WHY only a pocket: with the whole window's floor lowered, the strip's sagging
# front caught that floor's rear edge (runs ns6 to ns8, and probe nsP1), then, 15 mm lower, the slot's start 55 mm in
# (runs ns12, ns13); the rest of the window's floor is the slot's
Z_WFLOOR = Z_FLOOR - 15.0 * MM  # the fingertips' pocket's floor
POCKET = (0.0, 32.0 * MM)  # across the track (w), under the loading window


def _flares() -> list:
    """The walls' last 15 mm, each turned 15 degrees out from the track: their inner faces lead a strip in."""
    out = []
    length = FLARE_L / math.cos(FLARE)
    z0, z1 = Z_WFLOOR, Z_COVER
    for side in (-1.0, 1.0):  # the points' wall, the heads' wall
        theta = -side * FLARE  # the box's long axis (-sin t, cos t) runs from the wall's line out to the rear end
        axis = np.array([-math.sin(theta), math.cos(theta)])
        start = np.array([X_C + side * W_IN, Y_REAR - FLARE_L])
        mid = start + axis * length / 2
        out_dir = side * np.array([math.cos(theta), math.sin(theta)])
        c = np.array([*(mid + out_dir * WALL / 2), (z0 + z1) / 2]) - np.array([X_C, Y_REAR, TOP])
        out.append(box(tuple(c), (WALL, length, z1 - z0), MAG_ALU, yaw=theta))
    return out


# WHY chamfers: the strip pivots in a pinch and its front comes in 2 to 3 mm low; a square edge it meets face on holds
# it there however it is steered (runs ns6 to ns15). A 45-degree chamfer lifts it as it is pushed, as on a real
# magazine's entry.
FLOOR_CH, COVER_CH = 5.0 * MM, 2.5 * MM  # the floor's edge at the rear end, the cover's at the window's front


def _edge(x0, x1, y0, y1, z0, z1, ch, floor: bool) -> list:
    """A slab from x0..x1, y0..y1, z0..z1 with its top (``floor``) or bottom edge at y1 chamfered 45 degrees by ``ch``:
    two boxes and a box turned about x that fills the chamfer's triangle."""
    t = ch / math.sqrt(2.0)  # the triangle's height from its long side
    if floor:  # the top edge: the face runs down from (y1 - ch, z1) to (y1, z1 - ch)
        boxes = [_xb(x0, x1, y0, y1, z0, z1 - ch, MAG_ALU), _xb(x0, x1, y0, y1 - ch, z1 - ch, z1, MAG_ALU)]
        roll, cy, cz = -math.pi / 4, y1 - 0.75 * ch, z1 - 0.75 * ch
    else:  # the bottom edge: the face runs up from (y1 - ch, z0) to (y1, z0 + ch)
        boxes = [_xb(x0, x1, y0, y1, z0 + ch, z1, MAG_ALU), _xb(x0, x1, y0, y1 - ch, z0, z0 + ch, MAG_ALU)]
        roll, cy, cz = -3 * math.pi / 4, y1 - 0.75 * ch, z0 + 0.75 * ch
    c = np.array([(x0 + x1) / 2, cy, cz]) - np.array([X_C, Y_REAR, TOP])
    return boxes + [box(tuple(c), (x1 - x0, ch * math.sqrt(2.0), t), MAG_ALU, roll=roll)]


def _nailer_shapes() -> list:
    y0, y1 = Y_DRIVE, Y_REAR
    xp0, xh1 = X_WALL_P - WALL, X_WALL_H + WALL
    return [
        # the magazine: its lower side (on the bench), the slot's floor, and the head track's lower floor
        _xb(xp0, X_RAIL1, y0, y1, TOP, Z_WFLOOR, MAG_ALU),
        _xb(xp0, X_RAIL1, y0, Y_WIN, Z_WFLOOR, Z_HFLOOR, MAG_ALU),
        _xb(xp0, X_HTRACK, y0, Y_WIN, Z_HFLOOR, Z_FLOOR, MAG_ALU),
        # under the loading window, the floor beside the fingertips' pocket (the pocket is lower, so fingertips can get
        # below the strip's back end), its edge at the rear end chamfered
        *_edge(xp0, X_C + POCKET[0], Y_WIN, y1, Z_WFLOOR, Z_FLOOR, FLOOR_CH, floor=True),
        *_edge(X_C + POCKET[1], X_HTRACK, Y_WIN, y1, Z_WFLOOR, Z_FLOOR, FLOOR_CH, floor=True),
        *_edge(X_HTRACK, X_RAIL1, Y_WIN, y1, Z_WFLOOR, Z_HFLOOR, FLOOR_CH, floor=True),
        # its walls, the points' and the heads'
        _xb(xp0, X_WALL_P, y0, y1 - FLARE_L, Z_WFLOOR, Z_COVER, MAG_ALU),
        _xb(X_WALL_H, xh1, y0, y1 - FLARE_L, Z_WFLOOR, Z_COVER, MAG_ALU),
        *_flares(),
        # its cover, over the slot and over the head track, except over the loading window at the rear
        *_edge(X_WALL_P, X_HTRACK, y0, Y_WIN, Z_CEIL, Z_COVER, COVER_CH, floor=False),
        *_edge(X_HTRACK, X_WALL_H, y0, Y_WIN, Z_HCEIL, Z_COVER, COVER_CH, floor=False),
        # the pusher's rail beside the heads' wall
        _xb(xh1, X_RAIL1, y0, y1, Z_WFLOOR, Z_COVER - 1.0 * MM, PUSHER_BLACK),
        # the nose and the driver housing at the front, the cylinder head on the heads' side, the handle
        _xb(X_C - 0.075, X_C + 0.065, Y_DRIVE - 0.036, Y_DRIVE, TOP, TOP + 0.070, TOOL_RED),
        _xb(X_C + 0.065, X_C + 0.200, Y_DRIVE - 0.090, Y_DRIVE - 0.002, TOP, TOP + 0.092, TOOL_RED),
        _xb(X_C + 0.140, X_C + 0.175, Y_DRIVE, Y_REAR - 0.070, TOP + 0.016, TOP + 0.052, RUBBER),
        # the handle's end, joined to the magazine by a bracket under the rail
        _xb(X_C + 0.135, X_C + 0.180, Y_REAR - 0.085, Y_REAR - 0.045, TOP, TOP + 0.055, DARK),
        _xb(X_RAIL1, X_C + 0.135, Y_REAR - 0.080, Y_REAR - 0.050, TOP, TOP + 0.020, DARK),
    ]


def _nailer_looks() -> list:
    """Drawn only: the nose's contact tip and the trigger, the air fitting, a depth wheel, a yellow warning label."""
    return [
        _xb(X_C - 0.105, X_C - 0.075, Y_DRIVE - 0.024, Y_DRIVE - 0.010, TOP + 0.020, TOP + 0.042, STEEL),
        _xb(X_C + 0.120, X_C + 0.140, Y_DRIVE + 0.020, Y_DRIVE + 0.040, TOP + 0.020, TOP + 0.045, DARK),
        _xb(X_C + 0.150, X_C + 0.165, Y_REAR - 0.045, Y_REAR - 0.030, TOP + 0.025, TOP + 0.040, STEEL),
        _xb(X_C - 0.060, X_C - 0.030, Y_DRIVE - 0.030, Y_DRIVE - 0.010, TOP + 0.070, TOP + 0.074, DARK),
        _xb(X_C + 0.090, X_C + 0.170, Y_DRIVE - 0.070, Y_DRIVE - 0.020, TOP + 0.092, TOP + 0.0935, YELLOW),
        _xb(X_WALL_P + 0.010, X_HTRACK - 0.010, Y_DRIVE + 0.150, Y_DRIVE + 0.200, Z_COVER, Z_COVER + 0.0005, YELLOW),
    ]


_NS = _nailer_shapes()
_NV = _nailer_looks()
NAILER = body("nailer", pos=(X_C, Y_REAR, TOP), friction=0.5, shapes=_NS + _NV,
              visual_only=tuple(range(len(_NS), len(_NS) + len(_NV))))

# -- the pusher: on the rail, a spring pulling it towards the driver ------------------------------------------------------
# Its finger reaches into the track; its knob is 25 mm behind the finger. Empty, the finger rests at the driver.
KNOB_BACK = 25.0 * MM
KNOB = (12.0 * MM, 14.0 * MM, 16.0 * MM)  # along the track, across, up
PUSHER_BODY = (30.0 * MM, RAIL_W - 1.0 * MM, 6.0 * MM)
PUSHER_K, PUSHER_PRELOAD = 10.0, 0.30  # N/m, and how far past the driver its target is: 3 N there, about 6 N at the rear
PUSHER_MASS = 0.05
FINGER_MIN = 10.0 * MM  # the furthest back the finger goes (the rail runs on past the track's rear end)


def knob_y(finger_a: float) -> float:
    """The knob's (and the pusher body's) world y with the finger ``finger_a`` into the track."""
    return Y_REAR - (finger_a - KNOB_BACK)


PUSHER_X = (X_RAIL0 + X_RAIL1) / 2
PUSHER_Z = Z_COVER + 1.0 * MM + PUSHER_BODY[2] / 2  # 1 mm clear of the rail: a joint doesn't stop it colliding
PUSHER_AT = np.array([PUSHER_X, knob_y(A_DRIVE), PUSHER_Z])
KNOB_Z = PUSHER_BODY[2] / 2 + KNOB[2] / 2  # the knob's middle, above the pusher body's
PUSHER = body("pusher", pos=tuple(PUSHER_AT), mass=PUSHER_MASS, friction=1.0, shapes=[
    box((0.0, 0.0, 0.0), PUSHER_BODY, PUSHER_BLACK),
    box((0.0, 0.0, KNOB_Z), KNOB, PUSHER_BLACK),
    # drawn only: the finger, over the heads' wall into the track
    box(((X_HTRACK + PUSHER_X) / 2 - PUSHER_X, -KNOB_BACK, (Z_S - PUSHER_Z) / 2), (PUSHER_X - X_HTRACK, 3.0 * MM,
        PUSHER_Z - Z_S), PUSHER_BLACK),
], visual_only=(2,))
fittingkit.add_slider(Slider("nailer", "pusher", anchor=tuple(PUSHER_AT - np.array([X_C, Y_REAR, TOP])),
                             axis=(0, 1, 0), lower=0.0, upper=A_DRIVE - FINGER_MIN, stiffness=PUSHER_K, damping=0.5,
                             target=-PUSHER_PRELOAD))

# -- the strip on the table: lying flat along x, its front away from the arms, its heads to the robot's left ---------------
# WHY this way round: the left hand then picks it with its fingers along y and turns it a quarter turn clockwise; turned
# the other way, the left wrist would have to start turned 180 degrees, which it can't reach here (6 mm off, 64 degrees)
R_STRIP_START = np.eye(3)
# WHY on an offcut: lying on the table the strip is 5 to 7 mm thick, and the closing fingertips rode up over its edges
# (run ns2); on a 2x4 offcut standing on edge, 38 mm wide, its edges stand out 19 mm either side, clear for the fingers
WOOD = (0.80, 0.66, 0.44)
OFFCUT_SIZE = (0.200, 0.038, 0.089)  # a 2x4 offcut (1.5 x 3.5 in), on edge, along x
# WHY 38.5 cm out: at 36 cm the left hand, feeding the strip into the rear end, came within 3 to 9 mm of it (plan_check)
OFFCUT_AT = np.array([0.385, 0.26, TOP])
OFFCUT = body("offcut", pos=tuple(OFFCUT_AT), friction=0.8,
              shapes=[box((0.0, 0.0, OFFCUT_SIZE[2] / 2), OFFCUT_SIZE, WOOD)])
STRIP_AT = np.array([0.385, 0.26, TOP + OFFCUT_SIZE[2] + HEAD_D / 2 + 0.5 * MM])
_SS, _SV = _strip_shapes()
STRIP = body("nail_strip", pos=tuple(STRIP_AT), mass=STRIP_MASS, friction=0.6, shapes=_SS + _SV,
             visual_only=tuple(range(len(_SS), len(_SS) + len(_SV))), quat=quat_xyzw(R_STRIP_START))
R_STRIP_IN = R_MAG  # in the track the right way: its front towards the driver, its heads in the head track

OBJECTS = [NAILER, PUSHER, OFFCUT, STRIP]

# -- the magazine ---------------------------------------------------------------------------------------------------------
REST_N = -SLOT_T / 2 + BAND_T / 2  # the strip's middle, lying on the slot's floor
IN_W, IN_N, IN_DEG = 4.0 * MM, 4.0 * MM, 10.0  # in the track: within these of its line and level
# the rear end's lead-in and the window steer a strip's front in line with the slot (its front is lifted to the slot's
# height before the slot starts, 55 mm in)
# WHY these are soft and damped, and the two hard stops are poses: a model's forces act once per 240 Hz substep, and
# a stiff spring on a light part rings there; the nail stop at 20,000 N/m threw the strip from 28 mm in to the driver,
# and the driver stop bounced the 50 g pusher at 4.5 m/s (run ns11)
LEAD_W, LEAD_N, LEAD_K, LEAD_C, LEAD_MAX = 8.0 * MM, 9.0 * MM, 1500.0, 20.0, 10.0
STOP_K, STOP_C, STOP_MAX = 1500.0, 25.0, 40.0  # a wrong-way strip against the slot
DRAG, DRAG_C = 2.0, 20.0  # the pusher's finger, riding back over the strip, drags it towards its own speed, up to 2 N
STRIP_DRAG, STRIP_DRAG_C = 1.3, 10.0  # the strip's friction in the track, felt by the pusher pushing it, up to 1.3 N
V_SNAP = 0.25  # m/s: faster than this, the pusher has snapped
CLEAR = 0.02  # both grippers this far from the strip and the pusher's knob at the end


def strip_in_mag(f, origin=np.zeros(3)) -> tuple[np.ndarray, np.ndarray]:
    """The strip's middle in the magazine's frame (a, w, n) and its rotation relative to the magazine's."""
    return R_MAG.T @ (f.pos - origin - MAG_AT), R_MAG.T @ f.rot


def finger_a(pusher_pos, origin=np.zeros(3)) -> float:
    """How far into the track the pusher's finger is."""
    return float(Y_REAR - (pusher_pos[1] - origin[1]) + KNOB_BACK)


class Magazine:
    """Runs before every physics substep (240 Hz): the magazine, from the strip's and the pusher's poses.

    A strip coming into the rear end lined up with the track is steered in by its chamfer; with its heads on the
    points' side it stops there (the heads don't fit the shanks' slot). Once its back is past the nail stop it can't come
    back out. The pusher's finger, pulled back over the strip, rides on it and drags it back; pulled back past its last
    nail, it drops in behind it, and let forward it meets the strip (its speed then is noted) and pushes it, held in the
    track, up to the driver. The strip only feeds the right way round: a strip in back to front doesn't."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("NAILER_LOG")
        self.trace = os.environ.get("NAILER_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["nail_strip", "pusher"]

    def clear(self) -> None:
        self._n = 0
        self.past_stop = False  # the strip's back past the nail stop
        self.armed = False  # the finger behind the strip's last nail
        self.engaged = False  # the finger against it, pushing it
        self.snapped = False
        self.hit_speed = 0.0  # how fast the finger met the strip
        self.peak_speed = 0.0  # the fastest the pusher went pushing it
        self.home = False  # the strip's first nail at the driver
        self.in_track = False
        self.right_way = False
        self.depth = (0.0, 0.0)  # the strip's back and front, into the track
        self.finger = A_DRIVE
        self.frames = {}
        self.poses = {}

    def update(self, frames: dict, acc: dict) -> None:
        s, p = frames.get("nail_strip"), frames.get("pusher")
        if s is None or p is None:
            return
        self._n += 1
        self.poses = {}
        rel, rr = strip_in_mag(s, self.origin)
        rear, front = rel[0] - STRIP_L / 2, rel[0] + STRIP_L / 2
        self.depth = (float(rear), float(front))
        along = float(rr[0, 0])  # the strip's front direction along the track
        heads = float(rr[1, 1])  # its heads' direction across, towards the heads' wall
        level = float(rr[2, 2])
        ok_angle = abs(along) > math.cos(math.radians(IN_DEG)) and abs(level) > math.cos(math.radians(IN_DEG))
        # WHY the heads' side counts: a strip with its heads on the points' side can't get in at all (stopped below)
        self.in_track = bool(front > 0.0 and abs(rel[1]) < IN_W and abs(rel[2] - REST_N) < IN_N and ok_angle
                             and heads > 0.0)
        self.right_way = bool(along > 0.0 and heads > 0.0)
        fa = finger_a(p.pos, self.origin)
        va = float(-p.vel[1])  # the finger's speed into the track (towards the driver)
        self.finger = fa
        if self.engaged:
            self._pushing(s, p, fa, va, acc)
        else:
            self._loose(s, rel, rear, front, along, heads, level, fa, va, acc)
        self.frames = {"nail_strip": s, "pusher": p}
        if self.trace and self._n % 24 == 0:
            self._write_trace(s, p)

    def _loose(self, s, rel, rear, front, along, heads, level, fa, va, acc) -> None:
        a_dir = R_MAG[:, 0]
        # coming in at the rear end: the chamfer steers its front in; the wrong way round, it stops there
        lined_up = abs(along) > math.cos(math.radians(15.0)) and abs(level) > math.cos(math.radians(15.0))
        tip = s.pos + s.rot[:, 0] * (STRIP_L / 2) * (1 if along > 0 else -1)
        # WHY the front, not the middle: the strip pivots in a pinch and its front sags; steered by its middle, its front
        # met the slot's start 55 mm in low and stopped there (run ns12)
        tip_rel = R_MAG.T @ (tip - self.origin - MAG_AT)  # its front in (a, w, n)
        vel = R_MAG.T @ s.point_vel(tip)  # and its velocity
        near = abs(tip_rel[1]) < LEAD_W and abs(tip_rel[2] - REST_N) < LEAD_N and lined_up
        if near and heads < 0.0 and front > -0.002 and rear < 0.0:  # its heads against the shanks' slot
            push = -min(STOP_MAX, max(0.0, STOP_K * (front + 0.002) + STOP_C * vel[0]))
            fittingkit._wrench(acc, "nail_strip", s, a_dir * push, tip)
            self._note_once("wrong", f"{self._n / 240:7.2f}s heads on the points' side: it won't go in")
        elif near and heads > 0.0 and -0.02 < front < WINDOW + 0.015:  # through the lead-in and the window: steered in
            f = -(LEAD_K * np.array([0.0, tip_rel[1], tip_rel[2] - REST_N]) + LEAD_C * np.array([0.0, vel[1], vel[2]]))
            n = np.linalg.norm(f)
            if n > LEAD_MAX:
                f *= LEAD_MAX / n
            fittingkit._wrench(acc, "nail_strip", s, R_MAG @ f, tip)
        if self.in_track and self.right_way and rear > TAB + 1.0 * MM and not self.past_stop:
            self.past_stop = True
            self._note(f"{self._n / 240:7.2f}s past the nail stop")
        if self.past_stop and self.in_track and rear < TAB:  # held at the nail stop: it can't come back out
            at = s.pos - self.origin + a_dir * (TAB - rear)
            self.poses["nail_strip"] = (at + self.origin, quat_xyzw(s.rot))
            self.depth = (TAB, TAB + STRIP_L)
        if not (self.in_track and self.right_way):
            self.armed = False
            return
        # the pusher's finger
        if rear < fa < front and va < -0.005:  # riding back over it
            vs = float(a_dir @ s.vel)
            f = float(np.clip(-DRAG_C * (vs - va), -DRAG, DRAG))
            fittingkit._wrench(acc, "nail_strip", s, a_dir * f, s.pos)
        if fa < rear - 2.0 * MM and not self.armed:
            self.armed = True
            self._note(f"{self._n / 240:7.2f}s the pusher's finger drops in behind the strip")
        if self.armed and fa >= rear - 0.5 * MM:
            self.engaged = True
            self.hit_speed = max(0.0, va)
            if self.hit_speed > V_SNAP:
                self.snapped = True
            self._note(f"{self._n / 240:7.2f}s the pusher meets the strip at {self.hit_speed:.2f} m/s")

    def _pushing(self, s, p, fa, va, acc) -> None:
        a_dir = R_MAG[:, 0]
        rear = self.depth[0]
        if fa < rear - 3.0 * MM:  # pulled back off it again
            self.engaged = False
            self._note(f"{self._n / 240:7.2f}s the pusher pulled back off the strip")
            return
        over = fa + STRIP_L - A_DRIVE
        # WHY forward and short of the driver only: held at the driver, the pusher's pose is written back every substep
        # while the hand still presses on its knob, and its measured speed there reads 0.44 m/s (run ns17)
        if over < -0.5 * MM:
            self.peak_speed = max(self.peak_speed, va)
            if va > V_SNAP and not self.snapped:
                self.snapped = True
                self._note(f"{self._n / 240:7.2f}s the pusher snaps forward ({va:.2f} m/s)")
        if over > 0.0:  # the first nail against the driver: the pusher stops there
            stop = np.array([PUSHER_X, knob_y(A_DRIVE - STRIP_L), PUSHER_Z])
            self.poses["pusher"] = (stop + self.origin, quat_xyzw(p.rot))
            fa = A_DRIVE - STRIP_L
        else:
            f = float(np.clip(-STRIP_DRAG_C * va, -STRIP_DRAG, STRIP_DRAG))
            fittingkit._wrench(acc, "pusher", p, a_dir * f, p.pos)
        a = fa + STRIP_L / 2
        self.home = bool(fa + STRIP_L >= A_DRIVE - 1.0 * MM)
        if self.home and not getattr(self, "_home_noted", False):
            self._home_noted = True
            self._note(f"{self._n / 240:7.2f}s the strip is up to the driver")
        self.depth = (a - STRIP_L / 2, a + STRIP_L / 2)
        self.poses["nail_strip"] = (mag(a, 0.0, REST_N) + self.origin, quat_xyzw(R_STRIP_IN))

    def _write_trace(self, s, p) -> None:
        """Debug only (NAILER_TRACE=<file>): every 0.1 s, the strip, the finger and both TCPs (mm, from the table)."""
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        rel, rr = strip_in_mag(s, self.origin)
        q = 1000 * (s.pos - self.origin)
        yaw = math.degrees(math.atan2(rr[1, 0], rr[0, 0]))
        tilt = math.degrees(math.acos(min(1.0, abs(float(rr[2, 2])))))
        line = (f"{self._n / 240:6.2f}s strip ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:5.1f}) yaw {yaw:6.1f} tilt {tilt:4.1f} in a "
                f"{1000 * self.depth[0]:6.1f}..{1000 * self.depth[1]:6.1f} w {1000 * rel[1]:5.1f} n {1000 * rel[2]:5.1f} "
                f"t{int(self.in_track)} r{int(self.right_way)} s{int(self.past_stop)} | finger {1000 * self.finger:6.1f} "
                f"v {-p.vel[1]:5.2f} a{int(self.armed)} e{int(self.engaged)} h{int(self.home)} x{int(self.snapped)} |"
                f"{tcps}\n")
        with open(self.trace, "a") as fh:
            fh.write(line)

    def _note_once(self, key: str, line: str) -> None:
        done = getattr(self, "_noted", set())
        if key not in done:
            done.add(key)
            self._noted = done
            self._note(line)

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


MODEL = fittingkit.add_model(Magazine())


@fittingkit.on_reset
def _reset() -> None:
    MODEL.clear()
    MODEL._home_noted = False
    MODEL._noted = set()


# -- scoring ----------------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def loaded() -> bool:
    """The strip in the track the right way round and past the nail stop."""
    return bool(MODEL.in_track and MODEL.right_way and MODEL.past_stop) or MODEL.engaged


def fed() -> bool:
    """The pusher against the strip and the strip up to the driver."""
    return bool(MODEL.engaged and MODEL.home)


def _box_gap(points: np.ndarray, centre, rot, half) -> float:
    local = (points - centre) @ rot
    out = np.maximum(np.abs(local) - half, 0.0)
    return float(np.linalg.norm(out, axis=1).min())


def hands_clear() -> bool:
    """No gripper point within 2 cm of the strip or of the pusher's knob; unknown offline counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0:
        return True
    s, p = MODEL.frames.get("nail_strip"), MODEL.frames.get("pusher")
    if s is None or p is None:
        return True
    strip_half = np.array([STRIP_L / 2, NAIL_L / 2, HEAD_D / 2])
    knob = p.pos + p.rot @ np.array([0.0, 0.0, KNOB_Z])
    return (_box_gap(pts, s.pos, s.rot, strip_half) > CLEAR
            and _box_gap(pts, knob, p.rot, np.array(KNOB) / 2) > CLEAR)


def success(objects) -> bool:
    """The strip up to the driver with the pusher against it, the pusher never having snapped, and both grippers clear
    of the strip and the pusher's knob. Must hold for 1 s."""
    return fed() and not MODEL.snapped and hands_clear()


def score(objects) -> float:
    """0.9 for the strip fed up to the driver with the pusher against it, unsnapped (1.0 once success() has held 1 s,
    the hands clear); 0.5 for it fed but the pusher snapped; 0.25 for the strip in the track the right way round past
    the nail stop; 0 otherwise."""
    if fed():
        return 0.5 if MODEL.snapped else COMPLETE_SCORE
    return 0.25 if loaded() else 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------------
# The left hand picks the strip up by its back end, turns it a quarter turn so its front points at the driver and its
# heads at the heads' wall, feeds it into the rear end of the magazine as far as its fingers allow, lets go, and pushes
# the strip's back past the nail stop with its closed fingertips through the loading window. Then the right hand takes
# the pusher's knob, pulls it back past the strip, and eases it forward until the strip is up to the driver.
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
PAD_TIP = 13.0 * MM  # the TCP to the closed fingertips' ends
TIP_T = 8.0 * MM  # the closed fingertips' thickness, along the fingers' line

# WHY the hand off the strip's middle: on a part this wide one of the YAM's fingers stays at its open stop and the
# closing one pushes the part against it, so the strip's middle line ends 47 mm less half its width (8.9 mm) from the
# TCP, towards the TCP's +x finger here (run ns4: 9 mm; the camera-lens task found the same for its 69 mm lenses)
IN_HAND = 47.0 * MM - NAIL_L / 2
REACH_IN = IN_HAND - 2.5 * MM  # coming down round it, the finger that stays open 2.5 mm clear of its edge
LET_GO = IN_HAND - 1.0 * MM
# WHY turned further: the strip lags the hand's quarter turn by 3.3 degrees in the fingers and stays so (run ns4)
YAW_LAG = math.radians(3.3)
GRIP_BELOW = 10.0 * MM  # the fingertips this far under the strip's middle
# WHY 70 mm: held 16 mm from its back end, the strip turned 6 degrees in the fingers while turned and carried (its
# pinched edges are only 5 to 7 mm thick), which swung its front 12 mm off the track's line (run ns3); held by its
# middle, it went in only to 88 mm before the fingers met the rear end, its centre of mass 20 mm outside, and once let
# go its back end sagged out of reach of a second grip (run ns9). At 70 mm its front goes 127 mm in, its centre of mass
# inside, and the fingertips push the rest.
GRIP_FROM_BACK = 70.0 * MM  # the fingers' middle this far from the strip's back end
FINGER_HALF = 13.0 * MM
# WHY 4 mm over: the strip sits 1.4 mm lower in the hand than aimed (run ns4), and its front sags another 2 mm; with
# its middle aimed 1 mm over, its front met the slot's floor at the rear end 1.75 mm low (run ns14). Aimed 4 mm over,
# its front comes in about level with the slot's middle, clear of the floor at the rear end and the cover 55 mm in.
HOLD_UP = 4.0 * MM  # the strip's middle aimed this far over where it lies in the slot
FEED_SHORT = 5.0 * MM  # feeding it in, the fingers stop this far short of the magazine's rear end
# WHY 42 mm: pushed to 50 mm, the fingers' wider part above the tips met the cover at the window's front (plan_check)
PUSH_TO = 42.0 * MM  # the strip's back pushed this far in (the nail stop is 35 mm in; the window 55 mm)
PUSH_PAST = 3.0 * MM  # the fingertips aimed this far further [estimate: compliance]
PUSH_W = 15.0 * MM  # the fingertips push the strip's back this far towards its heads' side
PULL_TO = 32.0 * MM  # the pusher's finger pulled back to this far in (the hand aimed 12 mm further)
PULL_LAG = 12.0 * MM  # the hand aimed this much further: it lags about 6 N on an arm that gives about 550 N/m
EASE_PAST = 3.0 * MM  # easing forward, the hand aimed this far past where the driver stops the pusher
KNOB_PINCH = 5.0 * MM  # the fingertips this far under the knob's top


def strip_tcp(centre, rot, off: float = IN_HAND) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose holding the strip near its back end: straight down, its fingers across the strip (along the
    strip's y), the strip's middle line ``off`` from the TCP towards the TCP's +x, the fingertips 10 mm under its
    middle (so it is held between the pads' flat faces, not their rounded ends)."""
    centre, rot = np.asarray(centre, float), np.asarray(rot, float)
    fingers = rot[:, 1]  # the fingers' line, across the strip
    yaw = math.atan2(-fingers[0], fingers[1])  # grip_rotation(yaw)'s x is (sin yaw, -cos yaw); here -heads
    r = grip_rotation(yaw)
    along = centre + rot[:, 0] * (-STRIP_L / 2 + GRIP_FROM_BACK)
    tip_z = along[2] - GRIP_BELOW
    p = np.array([along[0], along[1], tip_z + PAD_TIP]) - off * r[:, 0]
    return p, r


def demo():
    right, left = [], []

    def both(dur, lpose, lf, rpose, rf, label):
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))

    up = lambda h: np.array([0.0, 0.0, h])  # noqa: E731
    r_wait = (np.array([0.22, -0.20, TOP + 0.20]), grip_rotation(math.pi / 2))
    # the left hand: pick the strip up 70 mm from its back end
    start = (STRIP_AT, R_STRIP_START)
    g0 = strip_tcp(*start, off=REACH_IN)
    g1 = strip_tcp(*start)
    both(2.0, (g0[0] + up(0.08), g0[1]), OPEN, r_wait, OPEN, "high over the strip")
    both(1.2, (g0[0] + up(0.015), g0[1]), OPEN, r_wait, OPEN, "just over it")
    both(1.0, g0, OPEN, r_wait, OPEN, "round it")
    both(2.0, g1, CLOSED, r_wait, OPEN, "hold it")
    lift = up(0.06)
    both(1.0, (g1[0] + lift, g1[1]), CLOSED, r_wait, OPEN, "lift it")
    # turned a quarter turn clockwise (seen from above) about the TCP, in two halves
    half = rot_z(-math.pi / 4)
    both(1.3, (g1[0] + lift, half @ g1[1]), CLOSED, r_wait, OPEN, "turn it")
    turned = rot_z(-math.pi / 2 - YAW_LAG) @ g1[1]
    both(1.3, (g1[0] + lift, turned), CLOSED, r_wait, OPEN, "front to the driver, heads to the heads' wall")
    # in line with the track, its front 16 mm behind the rear end, 8 cm up, then down to the slot's height
    in_line = mag(-0.016 - STRIP_L / 2, 0.0, REST_N + HOLD_UP)
    f0 = strip_tcp(in_line, R_STRIP_IN)
    f0 = (f0[0], rot_z(-YAW_LAG) @ f0[1])
    behind = np.array([g1[0][0], f0[0][1], g1[0][2] + 0.06])
    both(1.3, (behind, turned), CLOSED, r_wait, OPEN, "behind the magazine")
    both(2.0, (f0[0] + up(0.08), f0[1]), CLOSED, r_wait, OPEN, "over the rear end")
    both(1.5, f0, CLOSED, r_wait, OPEN, "in line with the track")
    # fed in until the fingers are 3 mm short of the rear end
    fed_back = FEED_SHORT + FINGER_HALF + GRIP_FROM_BACK  # the strip's back then this far behind the rear end
    f1 = strip_tcp(mag(-fed_back + STRIP_L / 2, 0.0, REST_N + HOLD_UP), R_STRIP_IN)
    f1 = (f1[0], rot_z(-YAW_LAG) @ f1[1])
    both(2.0, f1, CLOSED, r_wait, OPEN, "feed it into the magazine")
    f2 = strip_tcp(mag(-fed_back + STRIP_L / 2, 0.0, REST_N + HOLD_UP), R_STRIP_IN, off=LET_GO)
    f2 = (f2[0], rot_z(-YAW_LAG) @ f2[1])
    both(2.0, f2, OPEN, r_wait, OPEN, "let the strip go")
    both(1.0, (f2[0] + up(0.05), f2[1]), OPEN, r_wait, OPEN, "up from it")
    # the closed fingertips behind the strip's back, their thickness along the track, and push it in past the stop;
    # meanwhile the right hand goes to the pusher's knob, its fingers across it (along x)
    r_push = grip_rotation(0.0)  # the fingers' line along y, the track's line
    tip_z = Z_WFLOOR + 1.5 * MM
    # WHY 20 mm behind: the strip slips back in the fingers as it is fed (8 mm in run ns10), and the fingertips, brought
    # down 4 mm behind where its back should have been, landed on it and slid over it
    behind_back = mag(-fed_back - TIP_T / 2 - 20.0 * MM, PUSH_W, 0.0)
    p0 = np.array([behind_back[0], behind_back[1], tip_z + PAD_TIP])
    r_knob = grip_rotation(math.pi / 2)

    def knob_tcp(fa: float) -> np.ndarray:
        top = PUSHER_Z + KNOB_Z + KNOB[2] / 2
        return np.array([PUSHER_X, knob_y(fa), top - KNOB_PINCH + PAD_TIP])

    k0 = knob_tcp(A_DRIVE)
    both(2.0, (p0 + up(0.05), r_push), CLOSED, r_wait, OPEN, "fingertips behind the strip")
    both(1.2, (p0, r_push), CLOSED, (k0 + up(0.08), r_knob), OPEN, "down behind it")
    pushed = mag(PUSH_TO + PUSH_PAST - TIP_T / 2, PUSH_W, 0.0)
    p1 = np.array([pushed[0], pushed[1], tip_z + PAD_TIP])
    both(4.0, (p1, r_push), CLOSED, (k0 + up(0.08), r_knob), OPEN, "push it in past the nail stop")
    both(1.0, (p1 + up(0.06), r_push), CLOSED, (k0, r_knob), OPEN, "up out of the window")
    l_away = (np.array([0.22, 0.30, TOP + 0.20]), r_push)
    both(2.0, l_away, CLOSED, (k0, r_knob), CLOSED, "left hand away; hold the knob")
    k1 = knob_tcp(PULL_TO - PULL_LAG)
    both(2.5, l_away, CLOSED, (k1, r_knob), CLOSED, "pull the pusher back past the strip")
    both(0.5, l_away, CLOSED, (k1, r_knob), CLOSED, "hold it back")
    k2 = knob_tcp(A_DRIVE - STRIP_L + EASE_PAST)
    both(3.5, l_away, CLOSED, (k2, r_knob), CLOSED, "ease it forward against the strip")
    both(0.5, l_away, CLOSED, (k2, r_knob), CLOSED, "hold it there")
    both(2.0, l_away, CLOSED, (k2, r_knob), OPEN, "let the knob go")
    both(1.2, l_away, CLOSED, (k2 + up(0.07), r_knob), OPEN, "hands clear")
    both(1.5, l_away, CLOSED, (k2 + up(0.07), r_knob), OPEN, "wait")
    return {"left": left, "right": right}
