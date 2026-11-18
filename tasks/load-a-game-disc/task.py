"""On the station table: a games console standing upright, with a slot-loading disc drive whose slot is in its front
face, and beside it, lying on a storage box, a closed game case with the game's disc inside on its hub.

The disc has to go from its case into the console. The case is snapped shut: its catch takes a 1.2 N pull at the
cover's lip to open, three times what it takes to lift that edge of the whole case (0.4 N), so pulling the lip alone
lifts the case and leaves it shut. One hand has to hold the tray half down, by the finger ledge that shows through
the recess in the cover's left edge beside the lip, while the other lifts the lip; the cover then swings up and over the spine and
lies open beyond it. Inside, the case holds its disc on a hub with spring teeth: the disc comes off only while the
hub's centre button is pressed (the teeth draw in), and lifting it without pressing the button just lifts the case
with it. The disc is 120 mm across, wider than the gripper opens, and 1.2 mm thick, so it can only be pinched at its
edge, which the case's thumb cut-out leaves free. The console's slot is 5 mm wide behind a flared mouth: the disc has
to go in upright, square to the slot and centred, until the drive takes it (7 cm in).

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable DISC_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import disckit as K  # noqa: E402
from disckit import Case, Disc, Fixed, body_pose, on_reset, on_step, write_joint, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Open the game case, take the disc out and put it into the console."

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("DISC_LOT", "A")
LOTS = {"A": dict(hub=(0.430, 0.210)),
        "B": dict(hub=(0.420, 0.220))}
if LOT not in LOTS:
    raise ValueError(f"DISC_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
HUB_XY = np.array(LOTS[LOT]["hub"])

# -- the case, and the storage box it lies on ---------------------------------------------------------------------------
STAND_H = 0.085
TRAY = (0.135, 0.171)                   # the case's footprint (x, y); the spine is on the far (+x) long edge
CASE_T = 0.015                          # the closed case's thickness
CASE_MASS, COVER_MASS, SPINE_MASS = 0.070, 0.026, 0.004     # the whole empty case, and the cover's and spine's share
LIP = 0.004                             # the cover's lip stands this proud of the tray's near edge, over the thumb cut-out
RECESS_X = -0.0295                      # the finger recess and ledge on the left edge, from the hub's axis along x

# The snap: a catch at the opening edge holds the cover shut until its lip is pulled up with SNAP_PULL. Here that is a
# spring at the cover's hinge that lets go for good once the cover has turned SNAP_ANGLE (see _watch below); the spine
# cannot turn while the catch holds.
SNAP_PULL = 1.2                         # N at the cover's lip
SNAP_ANGLE = math.radians(1.5)          # 3.7 mm of lift at the lip
SNAP_ARM = TRAY[0] + K.RIM + LIP        # 0.141 m from the cover's hinge to its lip
SNAP_HOLD = SNAP_PULL * SNAP_ARM / SNAP_ANGLE               # N m/rad
OPEN_AT = math.radians(90)              # the cover swung this far from shut (upright or beyond): the case is open ...
OPEN_FLAT = math.radians(20)            # ... if its tray half still lies this flat ...
OPEN_ON_BOX = 0.010                     # ... and this close to its height on the box (not tipped up or knocked off)

# -- the console: standing upright, slot in its front face (facing the arms) -------------------------------------------
FACE_X = 0.470                          # the front face
SLOT_Y = 0.060                          # the slot's centre line
SLOT_Z = 0.100                          # height of the slot's middle (and of a loaded disc's centre) above the table
SLOT_W, SLOT_H, BAY = 0.005, 0.132, 0.135   # slot width and height; the drive bay behind it is as deep as this
MOUTH_W, MOUTH_D = 0.012, 0.008         # the slot's flared mouth: this wide at the face, narrowing to SLOT_W this deep
CONSOLE = dict(width=0.104, depth=0.260, height=0.300)

# -- the disc and its drive ------------------------------------------------------------------------------------------
DISC_MASS = 0.016
TAKE_AT = 0.070                         # the drive takes the disc once its leading edge is this far in ...
TAKE_TILT = math.radians(10)            # ... standing within this of upright (square to the slot) ...
TAKE_OFF = (0.0025, 0.012)              # ... and centred: across the slot, and up and down
SEATED = FACE_X + 0.072                 # where the drive draws the disc's centre to (its trailing edge 12 mm inside)
DRAW = 0.004                            # m per control step the drive draws it in (8 cm/s)
IN_SLOT = 0.020                         # the disc is "in the slot" with its leading edge this far behind the face ...
IN_TILT = math.radians(15)              # ... standing within this of upright and square to the slot ...
SLOT_SLACK = 0.001                      # ... and all of it that is behind the face between the slot's sides and its
                                        # top and bottom ends, to within this
OFF_HUB = 0.010                         # disc centre this far from its place on the hub: off the hub
DROPPED = 0.003                         # disc's lowest point this close to the table, outside the console: dropped
PRESSED = -0.0015                       # the hub button pressed this far: the teeth draw in
UPRIGHT_Q = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))   # xyzw: disc axis along world -y, upright in the x-z plane


def disc_home() -> np.ndarray:
    """The disc's centre on the hub at the start."""
    return np.array([HUB_XY[0], HUB_XY[1], TOP + STAND_H + K.FLOOR + K.SEAT_H + K.DISC_T / 2])


def lift_to_tip_the_case() -> float:
    """The pull (N) at the cover's lip that lifts the near edge of the whole closed case, disc inside, off the box."""
    arm = TRAY[0] / 2 + K.RIM                                           # from the far bottom edge to the middle
    return (CASE_MASS + DISC_MASS) * 9.81 * arm / SNAP_ARM


def _stand() -> Fixed:
    """The storage box: its front edge behind the thumb cut-out, long enough for the open cover to lie on."""
    x0 = HUB_XY[0] - TRAY[0] / 2 + 0.0275
    x1 = HUB_XY[0] + TRAY[0] / 2 + K.RIM + CASE_T + TRAY[0] + 0.020
    y0, y1 = HUB_XY[1] - TRAY[1] / 2 - 0.004, HUB_XY[1] + TRAY[1] / 2 + 0.005
    return Fixed("storage_box", pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + STAND_H / 2),
                 size=(x1 - x0, y1 - y0, STAND_H), color=(0.62, 0.55, 0.46), roughness=0.8)


def _console() -> list:
    """The console's body as fixed blocks round the slot and the drive bay behind it."""
    w, d, h = CONSOLE["width"], CONSOLE["depth"], CONSOLE["height"]
    y0, y1 = SLOT_Y - w / 2, SLOT_Y + w / 2
    sy0, sy1 = SLOT_Y - SLOT_W / 2, SLOT_Y + SLOT_W / 2
    z0, z1 = SLOT_Z - SLOT_H / 2, SLOT_Z + SLOT_H / 2
    black, white = (0.06, 0.06, 0.07), (0.92, 0.92, 0.93)
    parts = []

    def block(name, xa, xb, ya, yb, za, zb, color=black, rough=0.5):
        parts.append(Fixed(name, pos=((xa + xb) / 2, (ya + yb) / 2, TOP + (za + zb) / 2),
                           size=(xb - xa, yb - ya, zb - za), color=color, roughness=rough,
                           static_friction=0.4, dynamic_friction=0.3))

    x0, x1 = FACE_X, FACE_X + d
    xm = x0 + MOUTH_D                                                   # behind the flared mouth
    m = MOUTH_W / 2 + 0.003
    block("console_left", xm, x1, sy1, y1, 0.0, h)                      # the slot's two sides, full height
    block("console_right", xm, x1, y0, sy0, 0.0, h)
    block("console_face_l", x0, xm, SLOT_Y + m, y1, 0.0, h)              # the face beside the mouth
    block("console_face_r", x0, xm, y0, SLOT_Y - m, 0.0, h)
    block("console_under_slot", x0, x1, SLOT_Y - m, SLOT_Y + m, 0.0, z0)
    block("console_over_slot", x0, x1, SLOT_Y - m, SLOT_Y + m, z1, h)
    run = math.hypot(MOUTH_D, (MOUTH_W - SLOT_W) / 2)
    ang = math.atan2((MOUTH_W - SLOT_W) / 2, MOUTH_D)
    for side in (1, -1):                                                # the mouth's two sloping lips
        mid = np.array([x0 + MOUTH_D / 2, SLOT_Y + side * (MOUTH_W + SLOT_W) / 4])
        normal = np.array([math.sin(ang), side * math.cos(ang)])
        c = mid + normal * 0.002
        q = (0.0, 0.0, math.sin(-side * ang / 2), math.cos(-side * ang / 2))
        parts.append(Fixed(f"console_lip_{'lr'[side < 0]}", pos=(float(c[0]), float(c[1]), TOP + SLOT_Z),
                           size=(run + 0.002, 0.004, SLOT_H), color=(0.03, 0.03, 0.03), roughness=0.4, quat=q,
                           static_friction=0.3, dynamic_friction=0.2))
    block("console_bay_back", x0 + BAY, x1, sy0, sy1, z0, z1)
    block("console_panel_l", x0 + 0.004, x1 - 0.010, y1, y1 + 0.006, 0.015, h - 0.010, white, 0.35)   # side panels
    block("console_panel_r", x0 + 0.004, x1 - 0.010, y0 - 0.006, y0, 0.015, h - 0.010, white, 0.35)
    block("console_feet", x0 + 0.02, x1 - 0.02, y0 - 0.006, y1 + 0.006, -0.0005, 0.0005, (0.2, 0.2, 0.2))
    block("console_light", x0 - 0.0005, x0, sy1 + 0.004, sy1 + 0.007, z1 + 0.02, h - 0.03, (0.35, 0.6, 1.0), 0.2)
    return parts


OBJECTS = [
    _stand(),
    *_console(),
    Case("game_case", pos=(float(HUB_XY[0]), float(HUB_XY[1]), TOP + STAND_H), size=(TRAY[0], TRAY[1], CASE_T),
         mass=CASE_MASS, cover_mass=COVER_MASS, spine_mass=SPINE_MASS, tray=TRAY, thick=CASE_T, lip=LIP,
         recess_x=RECESS_X, snap=SNAP_HOLD),
    Disc("disc", pos=tuple(float(v) for v in disc_home()), size=(0.12, 0.12, K.DISC_T), mass=DISC_MASS),
]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(latched=True, opened=False, off_hub=False, in_slot=False, taking=False, taken=False, dropped=False,
                  teeth_in=False, draw_x=None, pressed_max=0.0, depth_max=-1.0, slot_max=-1.0, cover_max=0.0)


_clear()


def disc_geometry(pos, quat):
    """From the disc's pose: its axis, its lowest point's height, and how far its leading edge is into the slot."""
    axis = K._rotation(quat)[:, 2]
    low = pos[2] - K.DISC_R * math.sqrt(max(0.0, 1 - axis[2] ** 2)) - K.DISC_T / 2 * abs(axis[2])
    front = pos[0] + K.DISC_R * math.sqrt(max(0.0, 1 - axis[0] ** 2))
    return axis, low, front - FACE_X


def centred(pos, axis) -> bool:
    return (abs(pos[1] - SLOT_Y) <= TAKE_OFF[0] and abs(pos[2] - (TOP + SLOT_Z)) <= TAKE_OFF[1]
            and abs(axis[1]) >= math.cos(TAKE_TILT))


def slot_depth(pos, quat) -> float:
    """How far the disc is in the slot (m): how far its leading edge is behind the console's front face, counted only
    while the disc is physically in the slot. That needs it standing within IN_TILT of upright and square to the slot,
    and every part of it that is behind the face lying inside the slot's opening: between the slot's two sides (the
    flared mouth's at the face) and between its top and bottom ends. Anything else gives -1: a disc lying flat, a disc
    held over the console or beside the slot, however far "forward" it is."""
    pos = np.asarray(pos, float)
    rot = K._rotation(quat)
    axis = rot[:, 2]
    if abs(axis[1]) < math.cos(IN_TILT):
        return -1.0
    turn = np.linspace(0.0, 2 * math.pi, 144, endpoint=False)
    rim = pos + K.DISC_R * (np.outer(np.cos(turn), rot[:, 0]) + np.outer(np.sin(turn), rot[:, 1]))
    pts = np.vstack([rim + K.DISC_T / 2 * axis, rim - K.DISC_T / 2 * axis])
    behind = pts[pts[:, 0] > FACE_X]
    if len(behind) == 0:
        return -1.0
    half = np.where(behind[:, 0] > FACE_X + MOUTH_D, SLOT_W / 2, MOUTH_W / 2) + SLOT_SLACK
    across = np.abs(behind[:, 1] - SLOT_Y) <= half
    along = np.abs(behind[:, 2] - (TOP + SLOT_Z)) <= SLOT_H / 2 + SLOT_SLACK
    if not (across.all() and along.all()):
        return -1.0
    return float(pts[:, 0].max() - FACE_X)


def went_in(pos, quat) -> bool:
    """The disc is at least IN_SLOT (2 cm) inside the slot: upright, and physically in it (see ``slot_depth``)."""
    return slot_depth(pos, quat) >= IN_SLOT


def opened(swing, pos, quat) -> bool:
    """The case counts as opened when its cover has swung ``swing`` (rad) to upright or beyond while the tray half
    lies flat on the box, where ``pos`` and ``quat`` say the tray is."""
    flat = K._rotation(quat)[2, 2] >= math.cos(OPEN_FLAT) and abs(pos[2] - (TOP + STAND_H)) <= OPEN_ON_BOX
    return bool(swing >= OPEN_AT and flat)


@on_reset
def _reset(environment) -> None:
    _clear()
    write_joint(environment, "game_case", "cover", stiffness=SNAP_HOLD, position=0.0)      # snapped shut
    write_joint(environment, "game_case", "spine", limits=(0.0, 0.0), position=0.0)
    for i in range(K.TEETH):
        write_joint(environment, "game_case", f"tooth_{i}", limits=(0.0, 0.0), position=0.0)
    write_joint(environment, "game_case", "button", position=0.0)


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    case = object_states(environment, ["game_case"])["game_case"]
    cover, spine = case.joints["cover"], case.joints["spine"]
    if s["latched"] and cover > SNAP_ANGLE:                             # the catch lets go
        s["latched"] = False
        write_joint(environment, "game_case", "cover", stiffness=0.0)
        write_joint(environment, "game_case", "spine", limits=(0.0, math.pi / 2))
    s["cover_max"] = max(s["cover_max"], cover + spine)
    s["opened"] = s["opened"] or opened(cover + spine, case.pos, case.quat_xyzw)

    button = case.joints["button"]
    s["pressed_max"] = min(s["pressed_max"], button)
    teeth_in = button <= PRESSED
    if teeth_in != s["teeth_in"]:
        s["teeth_in"] = teeth_in
        lim = (-K.TOOTH_STROKE, -K.TOOTH_STROKE) if teeth_in else (0.0, 0.0)
        for i in range(K.TEETH):
            write_joint(environment, "game_case", f"tooth_{i}", limits=lim)

    pos, quat = body_pose(environment, "disc")
    axis, low, depth = disc_geometry(pos, quat)
    s["depth_max"] = max(s["depth_max"], depth)
    hub_home = np.asarray(case.pos) + K._rotation(case.quat_xyzw) @ np.array([0.0, 0.0, K.FLOOR + K.SEAT_H + K.DISC_T / 2])
    s["off_hub"] = s["off_hub"] or float(np.linalg.norm(pos - hub_home)) > OFF_HUB
    inside = depth > 0.0 and centred(pos, axis)
    s["slot_max"] = max(s["slot_max"], slot_depth(pos, quat))
    if went_in(pos, quat):
        s["in_slot"] = True
    if not inside and not s["taking"] and low <= TOP + DROPPED:
        s["dropped"] = True
    if not s["taking"] and depth >= TAKE_AT and centred(pos, axis):
        s["taking"], s["draw_x"] = True, float(pos[0])
    if s["taking"]:                                                     # the drive draws the disc in and holds it
        s["draw_x"] = min(SEATED, s["draw_x"] + DRAW)
        write_pose(environment, "disc", (s["draw_x"], SLOT_Y, TOP + SLOT_Z), UPRIGHT_Q)
        s["taken"] = s["draw_x"] >= SEATED


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """0 if the disc ever touched the table outside the console (dropped). Otherwise 1.0 once the drive has taken
    it; 0.75 if it stood in the slot at least 2 cm deep (upright and square to the slot within 15 degrees, and all of
    it that was behind the console's face inside the slot's opening: a disc over the console, beside the slot or lying
    flat does not count, however far forward it is); 0.5 if it came off the hub; 0.25 if the case was opened (its
    cover swung upright or beyond while its tray half lay flat on the box); 0 otherwise."""
    s = _STATE
    if s["dropped"]:
        return 0.0
    if s["taken"]:
        return 1.0
    if s["in_slot"]:
        return 0.75
    if s["off_hub"]:
        return 0.5
    return 0.25 if s["opened"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    s = _STATE
    return {"snap_let_go": not s["latched"], "opened": s["opened"], "cover_max_deg": round(math.degrees(s["cover_max"]), 1),
            "off_hub": s["off_hub"], "in_slot": s["in_slot"], "taking": s["taking"], "taken": s["taken"],
            "dropped": s["dropped"], "button_min_mm": round(1000 * s["pressed_max"], 2),
            "depth_max_mm": round(1000 * s["depth_max"], 1), "in_slot_max_mm": round(1000 * s["slot_max"], 1),
            "score": score(objects)}


def demo(objects, seed):
    return {}
