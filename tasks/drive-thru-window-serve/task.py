"""Hand a drink out a self-closing drive-thru window: one hand holds the sliding panel open by its pull bar, the other carries a full, lidded cup out
through the opening and sets it on the stainless hand-off shelf outside; the serving arm comes back inside, and only then is the window let go, so
that it shuts by itself without its edge touching either arm or the cup.

A painted plywood wall, 10 cm thick, stands across the counter. Set in it is a Quikserv SC-4030 self-closing horizontal slider: an aluminium frame
48 x 36 in with a 20-1/4 x 29 in service opening and a fixed pane beside it. The sliding panel (1/4 in tempered glass in an aluminium frame, about
8 kg) hangs inside the fixed pane and pulls itself shut. Under the window on the outside wall is a stainless hand-off shelf (Ready Access 65198701,
53-1/2 x 10 in).

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --stills 40 --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Serve the drink through the drive-thru window. With your left hand, slide the window open by its pull bar and hold it. With your right "
               "hand, carry the cup out through the opening and set it down upright on the shelf outside, then let go. Bring your right arm back "
               "inside, then let go of the window so it closes by itself without its edge touching either arm or the cup, and draw your left hand back.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold: score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the wall and the window, station metres (x forward from the arm bases, y left, z up from the counter, which is the table top)
WALL_X = (0.36, 0.46)  # inner and outer faces: the inner face clear of both grippers at their start pose (x 0.32)
WALL_Y, WALL_H = (-0.60, 1.20), 1.30
ROUGH_Y, ROUGH_Z = (-0.180, 1.049), (0.035, 0.959)  # the rough opening, 48-3/8 x 36-3/8 in
FRAME_Y, FRAME_Z, FRAME_X = (-0.175, 1.044), (0.040, 0.954), (0.365, 0.440)  # the frame, 48 x 36 in
OPEN_Y = (-0.150, 0.364)  # the service opening, 20-1/4 in wide
OPEN_Z = (0.1285, 0.8655)  # and 29 in high
MULLION_Y = (0.364, 0.389)
FIXED_X = (0.4125, 0.4175)  # the fixed pane's glass

# -- the sliding panel: in the plane x 0.39, its leading stile meeting the right jamb at y -0.15 when shut; it slides toward +y behind the fixed pane
PANEL_X = (0.3775, 0.4025)  # its aluminium frame, 25 mm deep; the 1/4 in glass at its middle
PANEL_Y = (-0.149, 0.401)  # 55 cm wide, closed (1 mm off the jamb)
PANEL_Z = (0.110, 0.880)  # 77 cm tall, its top and bottom edges running in the frame's tracks
PANEL_COL_Z = (0.135, 0.860)  # its colliders: clear of the track lips
STILE_W = 0.040
TRAVEL = 0.48  # its travel toward +y, the pull bar staying inside the service opening
PANEL_MASS = 8.0  # 6.7 kg of glass (0.55 x 0.77 m x 6.35 mm at 2,500 kg/m^3) and its frame
PULL_N = 10.0  # its closing pull: a simulation setting (a real unit's is read with a spring scale on the bar)
DAMP = 22.0  # N s/m: its damper, a setting, so that it shuts from 30 cm open in about 1 s and does not slam
BAR_R, BAR_X, BAR_Y0, BAR_Z = 0.0125, 0.3443, -0.120, (0.20, 0.50)  # the pull bar: 25 mm round, 30 cm long, 3 cm off the glass, 3 cm from the stile's face

# -- the cup: a 16 oz paper cold cup with a flat polystyrene lid, holding 0.47 kg of water; its origin at the centre of its base
CUP_R0, CUP_R1, CUP_H = 0.030, 0.045, 0.140
LID_R, LID_H = 0.0465, 0.008
CUP_MASS = 0.49  # 0.47 kg of water, the cup and the lid
CUP_START = np.array([0.28, -0.15])  # on the counter just inside the window

# -- the hand-off shelf outside: Ready Access 65198701, 13-gauge stainless, 53-1/2 x 10 in, on the outer wall under the window
SHELF_X = (0.46, 0.714)
SHELF_Y = (-0.20, 1.159)
SHELF_TOP, SHELF_T = 0.125, 0.0023

# -- the demo's plan
BAR_GRIP_Z = 0.36  # the left hand holds the bar here
OPEN_TO = 0.30  # and slides the panel this far open
CUP_TCP_H = 0.10  # the right hand's tool point on the cup, above its base: the pads under the lid
CUP_W = 2 * (CUP_R0 + (CUP_R1 - CUP_R0) * (CUP_TCP_H + 0.019) / CUP_H)  # the cup's width at the pads' top edge, where a flat pad meets the tapered cup
CUP_SQUEEZE = 12.0  # N: the right hand's squeeze on the cup, under the crushing setting
JAW_BAR = -0.024  # each finger this far open round the bar: the far finger clear of the panel
PICK_BACK = 0.004  # the tool point this far behind the cup's axis: the gripper's palm clear of the cup's back
SERVE = np.array([0.515, -0.075])  # where the cup is set on the shelf
CARRY_BASE = 0.25  # the cup's base this high while carried out, clear over the sill

# -- the scorer's thresholds
OPENED = 0.15  # the panel opened at least 15 cm at some time
LIFTED = 0.02  # the cup lifted at least 2 cm off the counter
UPRIGHT, TIPPED = 15.0, 30.0
FALL_V = 1.0  # m/s downward: a fall
CRUSH_N = 20.0  # a squeeze above this on the rigid cup counts as crushing it (a setting)
TOUCH_STILE = 0.004  # any part of an arm, or the cup, within 4 mm of the stile's leading face is a touch
LET_GO = 0.008  # every fingertip of both hands at least 8 mm off the cup: let go
SHUT = 0.005  # gap under 5 mm
OFF_PANEL = 0.010  # every fingertip at least 1 cm off the pull bar
STILL = 0.001

LOOKS = {
    "paint": m.Look(color=m.rgb(0.90, 0.88, 0.83), roughness=0.85),
    "siding": m.Look(color=m.rgb(0.62, 0.30, 0.22), roughness=0.9),
    "alu": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.85, roughness=0.35),
    "glass": m.Look(glass=(0.93, 0.97, 0.98)),
    "dark": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.6),
    "paper": m.Look(color=m.rgb(0.96, 0.96, 0.94), roughness=0.7),
    "band": m.Look(color=m.rgb(0.72, 0.10, 0.10), roughness=0.6),
    "lid": m.Look(color=m.rgb(0.90, 0.92, 0.93), roughness=0.3, opacity=0.85),
    "skin": m.Look(color=m.rgb(0.80, 0.62, 0.50), roughness=0.6),
    "nail": m.Look(color=m.rgb(0.88, 0.74, 0.68), roughness=0.4),
    "steel": m.Look(color=m.rgb(0.55, 0.56, 0.58), metallic=0.9, roughness=0.35),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.5),
}


def _bx(x, y, z) -> m.Mesh:
    return m.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _cbox(name, x, y, z) -> str:
    return m.col_box(name, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def cup_r(h):
    """The cup's outside radius at height h above its base."""
    return CUP_R0 + (CUP_R1 - CUP_R0) * np.clip(h, 0.0, CUP_H) / CUP_H


# -- the wall and the window's frame (fixed) ---------------------------------------------------------------------------------------------------------

def _wall():
    x = WALL_X
    pieces = {
        "right": ((WALL_Y[0], ROUGH_Y[0]), (0.0, WALL_H)),
        "left": ((ROUGH_Y[1], WALL_Y[1]), (0.0, WALL_H)),
        "below": (ROUGH_Y, (0.0, ROUGH_Z[0])),
        "above": (ROUGH_Y, (ROUGH_Z[1], WALL_H)),
    }
    parts, cols = [], []
    for n, (y, z) in pieces.items():
        parts.append((n, _bx((x[0], x[1] - 0.004), y, z), "paint"))
        parts.append((f"{n}_ext", _bx((x[1] - 0.004, x[1]), y, z), "siding"))  # the outside face
        cols.append(_cbox(f"{n}_c", x, y, z))
    # the window's aluminium frame, with tracks for the sliding panel: its colliders leave the panel's band free
    fx, inner, outer = FRAME_X, (FRAME_X[0], PANEL_X[0] - 0.0015), (PANEL_X[1] + 0.0015, FRAME_X[1])
    members = {
        "head": (FRAME_Y, (OPEN_Z[1], FRAME_Z[1])),
        "sill": (FRAME_Y, (FRAME_Z[0], OPEN_Z[0])),
    }
    for n, (y, z) in members.items():
        parts.append((n, _bx(fx, y, z), "alu"))
        cols.append(_cbox(f"{n}_ci", inner, y, z))
        cols.append(_cbox(f"{n}_co", outer, y, z))
    cols.append(_cbox("head_cap", fx, FRAME_Y, (PANEL_Z[1] + 0.008, FRAME_Z[1])))
    cols.append(_cbox("sill_cap", fx, FRAME_Y, (FRAME_Z[0], PANEL_Z[0] - 0.008)))
    for n, y in {"jamb_r": (FRAME_Y[0], OPEN_Y[0]), "jamb_l": (FRAME_Y[1] - 0.025, FRAME_Y[1])}.items():
        parts.append((n, _bx(fx, y, OPEN_Z), "alu"))
        cols.append(_cbox(f"{n}_c", fx, y, OPEN_Z))
    parts.append(("mullion", _bx(outer, MULLION_Y, OPEN_Z), "alu"))
    cols.append(_cbox("mullion_c", outer, MULLION_Y, OPEN_Z))
    fixed_y = (MULLION_Y[1], FRAME_Y[1] - 0.025)
    parts.append(("fixed_pane", _bx(FIXED_X, fixed_y, OPEN_Z), "glass"))
    cols.append(_cbox("fixed_pane_c", FIXED_X, fixed_y, OPEN_Z))
    for k, z in enumerate((OPEN_Z[0] + 0.004, OPEN_Z[1] - 0.004)):  # the panel's two tracks, seen from inside
        parts.append((f"track{k}", _bx((PANEL_X[0] - 0.0015, PANEL_X[0]), (OPEN_Y[0], fixed_y[1]), (z - 0.004, z + 0.004)), "dark"))
    return m.fixed("wall", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.4, 0.3))


# -- the sliding panel, on a slider along +y, with a constant closing pull (Trial.step) and a damper ------------------------------------------------------

def _panel() -> m.Link:
    x, (y0, y1), z = PANEL_X, PANEL_Y, PANEL_Z
    gx = (0.39 - 0.0032, 0.39 + 0.0032)
    parts = [
        ("stile_lead", _bx(x, (y0, y0 + STILE_W), z), "alu"),
        ("stile_trail", _bx(x, (y1 - STILE_W, y1), z), "alu"),
        ("rail_top", _bx(x, (y0 + STILE_W, y1 - STILE_W), (z[1] - 0.040, z[1])), "alu"),
        ("rail_bottom", _bx(x, (y0 + STILE_W, y1 - STILE_W), (z[0], z[0] + 0.040)), "alu"),
        ("glass", _bx(gx, (y0 + STILE_W, y1 - STILE_W), (z[0] + 0.040, z[1] - 0.040)), "glass"),
        ("bar", m.cylinder(BAR_R, BAR_Z[1] - BAR_Z[0], (BAR_X, BAR_Y0, (BAR_Z[0] + BAR_Z[1]) / 2)), "alu"),
    ]
    cz = PANEL_COL_Z
    cols = [
        _cbox("stile_lead_c", x, (y0, y0 + STILE_W), cz),
        _cbox("stile_trail_c", x, (y1 - STILE_W, y1), cz),
        _cbox("glass_c", (gx[0] - 0.002, gx[1] + 0.002), (y0 + STILE_W, y1 - STILE_W), cz),
        m.col_cylinder("bar_c", BAR_R, BAR_Z[1] - BAR_Z[0], (BAR_X, BAR_Y0, (BAR_Z[0] + BAR_Z[1]) / 2)),
    ]
    for k, zz in enumerate((BAR_Z[0] + 0.02, BAR_Z[1] - 0.02)):  # the bar's two standoffs to the stile
        sx = (BAR_X + BAR_R * 0.5, x[0])
        mesh = m.cylinder(0.007, sx[1] - sx[0], ((sx[0] + sx[1]) / 2, BAR_Y0, zz), axis="x")
        parts.append((f"standoff{k}", mesh, "alu"))
        cols.append(m.col_hull(f"standoff{k}_c", mesh))
    return m.slider("panel", "Y", 0.0, TRAVEL, parts, cols, mass=PANEL_MASS, spring=0.0, damping=DAMP, effort=80.0, friction=0.0, armature=0.0,
                    inertia=(PANEL_MASS * 0.55 ** 2 / 12, PANEL_MASS * 0.77 ** 2 / 12, PANEL_MASS * 0.55 ** 2 / 12))


def _window():
    anchor = [("anchor", _bx((WALL_X[0] + 0.01, WALL_X[0] + 0.02), (WALL_Y[0] + 0.01, WALL_Y[0] + 0.02), (1.0, 1.01)), "paint", True)]
    return m.mechanism("window", (0.0, 0.0, TOP), [_panel()], LOOKS, frame_parts=anchor, friction=(0.4, 0.3), solver_iters=32)


# -- the cup ----------------------------------------------------------------------------------------------------------------------------------------

def _cup():
    prof = [(0.0, 0.0), (CUP_R0 - 0.002, 0.0), (CUP_R0, 0.002)] + [(float(cup_r(h)), h) for h in np.linspace(0.004, CUP_H - 0.004, 6)] + \
           [(CUP_R1 + 0.0012, CUP_H - 0.002), (CUP_R1 + 0.0012, CUP_H), (0.0, CUP_H)]
    band = [(float(cup_r(h)) + 0.0003, h) for h in (0.050, 0.095)]
    lid = [(0.0, CUP_H - 0.001), (LID_R, CUP_H - 0.001), (LID_R, CUP_H + 0.004), (LID_R - 0.004, CUP_H + 0.006), (LID_R - 0.012, CUP_H + 0.006),
           (LID_R - 0.014, CUP_H + LID_H), (0.0, CUP_H + LID_H)]
    parts = [
        ("cup", m.lathe(prof, 40), "paper"),
        ("band", m.lathe([(0.0, band[0][1]), *band, (0.0, band[1][1])], 40), "band"),
        ("lid", m.lathe(lid, 40), "lid"),
        ("sip", m.cylinder(0.004, 0.001, (0.024, 0.0, CUP_H + 0.0055)), "dark"),
    ]
    hull = m.lathe([(0.0, 0.0), (CUP_R0, 0.0)] + [(float(cup_r(h)), h) for h in (0.05, 0.10, CUP_H - 0.002)] + [(LID_R, CUP_H + 0.004), (LID_R - 0.014, CUP_H + LID_H), (0.0, CUP_H + LID_H)], 24)
    return m.free("cup", (CUP_START[0], CUP_START[1], TOP + 0.0005), parts, [m.col_hull("cup_c", hull)], LOOKS, mass=CUP_MASS, friction=(0.7, 0.6),
                  torsional_patch=0.03)  # a pad pressed on the paper wall grips over an area, so the cup does not pivot in the jaws


# -- the shelf ------------------------------------------------------------------------------------------------------------------------------------

def _shelf():
    x, y = SHELF_X, SHELF_Y
    parts = [("top", _bx(x, y, (SHELF_TOP - SHELF_T, SHELF_TOP)), "steel"),
             ("lip", _bx((x[1] - SHELF_T, x[1]), y, (SHELF_TOP - 0.025, SHELF_TOP)), "steel"),  # its front edge, turned down
             ("back", _bx((x[0], x[0] + SHELF_T), y, (SHELF_TOP - 0.06, SHELF_TOP)), "steel")]
    for k, yy in enumerate((y[0] + 0.05, 0.47, y[1] - 0.05)):  # its brackets against the wall
        parts.append((f"bracket{k}", m.prism([(x[0], SHELF_TOP - 0.06), (x[0] + 0.20, SHELF_TOP - SHELF_T), (x[0], SHELF_TOP - SHELF_T)], yy - 0.0015, yy + 0.0015).turned(90.0, "x").moved((0.0, 2 * yy, 0.0)), "steel"))
    cols = [_cbox("top_c", x, y, (SHELF_TOP - 0.01, SHELF_TOP)), _cbox("lip_c", (x[1] - SHELF_T, x[1]), y, (SHELF_TOP - 0.025, SHELF_TOP))]
    return m.fixed("shelf", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


def _side_camera():
    """The real trial's side camera, outside the wall looking along +y across the opening."""
    body = m.box((0.05, 0.08, 0.05), (0.52, -0.45, 0.40))
    lens = m.cylinder(0.016, 0.03, (0.52, -0.395, 0.40), axis="y")
    post = m.cylinder(0.008, 0.375, (0.52, -0.45, 0.1875))
    foot = m.cylinder(0.05, 0.01, (0.52, -0.45, 0.005))
    return m.fixed("side_camera", (0.0, 0.0, TOP), [("body", body, "black"), ("lens", lens, "dark"), ("post", post, "steel"), ("foot", foot, "steel")],
                   [], LOOKS)


OBJECTS = [_wall(), _window(), _cup(), _shelf(), _side_camera()]


# -- the scorer -------------------------------------------------------------------------------------------------------------------------------------

def stile_gap(p: np.ndarray, q: float, radius: float = 0.0) -> float:
    """How far a point (a ball of `radius`) is from the face of the panel's leading stile, the face that meets the jamb, with the panel open `q`
    metres; inf for points behind the face (inside the panel's own outline)."""
    ys = PANEL_Y[0] + q
    if p[1] > ys + 0.002:
        return math.inf
    dx = max(PANEL_X[0] - p[0], 0.0, p[0] - PANEL_X[1])
    dz = max(PANEL_Z[0] - p[2], 0.0, p[2] - PANEL_Z[1])
    return math.sqrt(dx * dx + (ys - p[1]) ** 2 + dz * dz) - radius


def cup_points(cup, rings: int = 6, around: int = 32) -> np.ndarray:
    """Points on the cup's side and lid, in station metres."""
    th = np.linspace(0.0, 2 * math.pi, around, endpoint=False)
    pts = []
    for h in np.linspace(0.0, CUP_H, rings):
        r = float(cup_r(h))
        pts += [(r * math.cos(t), r * math.sin(t), h) for t in th]
    pts += [(LID_R * math.cos(t), LID_R * math.sin(t), CUP_H + 0.004) for t in th[::2]]
    return cup.to_world(np.array(pts))


def cup_surface_gap(cup, pts: np.ndarray) -> np.ndarray:
    """Each point's distance outside the cup (and its lid); negative inside."""
    loc = cup.to_local(pts)
    h = loc[:, 2]
    rad = np.hypot(loc[:, 0], loc[:, 1])
    r = np.where(h > CUP_H, LID_R, cup_r(h))
    dr = rad - r
    dh = np.maximum(np.maximum(-h, h - CUP_H - LID_H), 0.0)
    return np.where(dh > 0, np.hypot(np.maximum(dr, 0.0), dh), dr)


ARM_CHAIN = (("link3", 0.035), ("link4", 0.032), ("link5", 0.030), ("gripper", 0.030))


def arm_points(env):
    """Points along each arm, with a radius each: the link origins from the elbow out, the points between them, the wrist camera and the
    tool point. Off the simulator: none."""
    if env is None or not hasattr(env, "unwrapped"):
        return []
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = m._np(robot.data.body_link_pose_w)[0]
        chain = []
        for name, r in ARM_CHAIN:
            p = poses[robot.find_bodies(name)[0][0]]
            chain.append((p[:3] - origin, r, p))
        g = chain[-1][2]
        R = m.quat_matrix(g[3:7])
        tcp = g[:3] - origin + R @ np.array([0.0, 0.0, 0.134])
        cam = g[:3] - origin + R @ np.array([0.0, 0.0697, 0.072])  # the wrist camera (README 7)
        for (a, ra, _), (b, rb, _) in zip(chain[:-1], chain[1:]):
            for s in np.linspace(0.0, 1.0, 5)[:-1]:
                out.append((a + s * (b - a), ra + s * (rb - ra), side))
        out += [(chain[-1][0], chain[-1][1], side), (chain[-1][0] + 0.5 * (tcp - chain[-1][0]), 0.025, side), (tcp, 0.015, side), (cam, 0.022, side)]
    return out


def finger_squeeze(env) -> dict:
    """Each hand's squeeze, N: the fingers' drive pushing them closed past where they stand (zero with nothing between them)."""
    if env is None or not hasattr(env, "unwrapped"):
        return {"left": 0.0, "right": 0.0}
    from yam_arena.constants import GRIPPER

    out = {}
    for side in ("left", "right"):
        robot = env.unwrapped.scene[f"{side}_robot"]
        js = [robot.find_joints(j)[0][0] for j in ("joint7", "joint8")]
        q = m._np(robot.data.joint_pos)[0]
        t = m._np(robot.data.joint_pos_target)[0]
        out[side] = float(min(GRIPPER["effort"], max(GRIPPER["stiffness"] * max(0.0, abs(q[j]) - abs(t[j])) for j in js)))
    return out


def moment(c: dict) -> float:
    """The scoring text on one moment's facts."""
    if c["failed"]:
        return 0.0
    if not (c["opened"] and c["lifted"]):
        return 0.0
    if not (c["cup_x"] > WALL_X[1] and c["tilt"] <= UPRIGHT):
        return 0.25
    if not (c["on_shelf"] and c["let_go"]):
        return 0.5
    if c["arms_in"] and c["gap"] < SHUT and c["off_panel"] and c["still"]:
        return 1.0
    return 0.75


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.struck = gk.Latch("the stile touched an arm or the cup")
        self.tipped = gk.Latch("the cup tipped past 30 degrees")
        self.fell = gk.Latch("the cup fell")
        self.crushed = gk.Latch("the cup was squeezed past the crushing setting")
        self.opened = 0.0
        self.lifted = False
        self.stall = 0
        self.prev_q = 0.0
        self.end = gk.EndHold(HOLD_STEPS)
        self.trail: list[tuple] = []
        self.score = 0.0
        self.report: dict = {}
        self.why = ""

    def step(self, env) -> None:
        self.steps += 1
        if self.steps == 1 and env is not None and hasattr(env, "unwrapped"):
            m.push(env, "window", (0.0, -PULL_N, 0.0), body="panel")
        win, cup = m.state(env, "window"), m.state(env, "cup")
        q = win.joint("panel")
        self.opened = max(self.opened, q)
        tips = m.fingertips(env)
        tilt = cup.tilt_deg()
        base_h = float(cup.pos[2] - TOP)
        mid = cup.to_world(np.array([[0.0, 0.0, CUP_H / 2]]))[0]
        self.lifted = self.lifted or base_h >= LIFTED
        cpts = cup_points(cup)
        arms = arm_points(env)

        # latched failures
        hit = [f"{s} arm" for p, r, s in arms if stile_gap(p, q, r) <= TOUCH_STILE]
        hit += [f"{'left' if i < 12 else 'right'} fingertip" for i, p in enumerate(tips) if stile_gap(p, q) <= TOUCH_STILE]
        if min(stile_gap(p, q) for p in cpts) <= TOUCH_STILE:
            hit.append("cup")
        if hit and not self.struck:
            self.why = f"stile touched the {hit[0]} at step {self.steps} (panel open {1000 * q:.0f} mm)"
            print(f"[serve] {self.why}", flush=True)
        self.struck.trip(bool(hit))
        self.tipped.trip(tilt > TIPPED)
        self.fell.trip(float(cup.lin_vel[2]) <= -FALL_V or (float(mid[0]) > WALL_X[1] and base_h < SHELF_TOP - 0.03))
        gaps = cup_surface_gap(cup, tips)
        sq = finger_squeeze(env)
        on_cup = {"left": bool(gaps[:12].min() <= 0.006), "right": bool(gaps[12:].min() <= 0.006)}
        self.crushed.trip(any(on_cup[s] and sq[s] > CRUSH_N for s in ("left", "right")))

        # standing on the shelf: its base within 1 cm over the top, the whole base disc over the shelf, upright
        cxy = cup.pos[:2]
        on_shelf = bool(-0.003 <= base_h - SHELF_TOP <= 0.010 and tilt <= UPRIGHT and SHELF_X[0] + CUP_R0 <= cxy[0] <= SHELF_X[1] - CUP_R0
                        and SHELF_Y[0] + CUP_R0 <= cxy[1] <= SHELF_Y[1] - CUP_R0)

        let_go = bool(gaps.min() >= LET_GO)
        arms_in = bool(tips[:, 0].max() < WALL_X[0] and all(p[0] < WALL_X[0] for p, _, _ in arms))
        bar_d = np.hypot(tips[:, 0] - BAR_X, tips[:, 1] - (BAR_Y0 + q)) - BAR_R
        bar_d = np.where((tips[:, 2] - TOP >= BAR_Z[0] - 0.02) & (tips[:, 2] - TOP <= BAR_Z[1] + 0.02), bar_d, math.inf)
        off_panel = bool(bar_d.min() >= OFF_PANEL)
        # trapped: with no hand within 1 cm of the panel, it closes under its pull; stopped short of shut for 0.25 s, it rests on an arm or the cup
        lo = np.array([PANEL_X[0], PANEL_Y[0] + q, PANEL_Z[0] + TOP])
        hi = np.array([PANEL_X[1], PANEL_Y[1] + q, PANEL_Z[1] + TOP])
        hands_off = float(np.linalg.norm(np.maximum(np.maximum(lo - tips, tips - hi), 0.0), axis=1).min()) >= 0.01 and off_panel
        self.stall = self.stall + 1 if (hands_off and q > SHUT and abs(q - self.prev_q) < 0.0003) else 0
        self.prev_q = q
        if self.stall >= 5 and not self.struck:
            self.why = f"the closing panel stopped {1000 * q:.0f} mm short of shut, on an arm or the cup (step {self.steps})"
            print(f"[serve] {self.why}", flush=True)
            self.struck.trip(True)
        self.trail = (self.trail + [(q, cup.pos.copy())])[-(HOLD_STEPS + 1):]
        still = len(self.trail) > HOLD_STEPS and max(max(abs(a - q), float(np.linalg.norm(b - cup.pos))) for a, b in self.trail) <= STILL
        failed = bool(self.struck or self.tipped or self.fell or self.crushed)
        c = {"failed": failed, "opened": self.opened >= OPENED, "lifted": self.lifted, "cup_x": float(mid[0]), "tilt": tilt, "on_shelf": on_shelf,
             "let_go": let_go, "arms_in": arms_in, "gap": q, "off_panel": off_panel, "still": still}
        mo = moment(c)
        self.end.step(mo == 1.0)
        self.score = 1.0 if self.end.done else (0.75 if mo == 1.0 else mo)
        self.report = {"step": self.steps, "score": self.score, "moment": mo, "panel_mm": round(1000 * q, 1), "opened_mm": round(1000 * self.opened, 1),
                       "cup_mm": np.round(1000 * (cup.pos - [0, 0, TOP]), 1).tolist(), "tilt": round(tilt, 1), "lifted": self.lifted,
                       "on_shelf": on_shelf, "let_go": let_go,
                       "arms_in": arms_in, "off_panel": off_panel, "still": still, "squeeze": {k: round(v, 1) for k, v in sq.items()},
                       "struck": bool(self.struck), "tipped": bool(self.tipped), "fell": bool(self.fell), "crushed": bool(self.crushed), "why": self.why,
                       "nearest_stile_mm": round(1000 * min([stile_gap(p, q, r) for p, r, _ in arms] + [stile_gap(p, q) for p in tips]), 1)}
        if self.steps % 20 == 0:
            d = os.environ.get("SERVE_SCORE_DIR", "serve_scores")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{os.environ.get('TASK_DEMO', 'run')}.json"), "w") as f:
                json.dump(self.report, f, indent=0, default=str)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.end.done


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------------------------

def demo_script(ctx):
    """The left hand takes the pull bar with its fingers pointing 45 degrees to the right and closing across it, and slides the panel 30 cm open.
    The right hand takes the cup by its upper body, fingers 5 degrees below level, lifts it, carries it out through the opening and
    sets it down on the shelf outside. It lets go and comes back inside.
    Then the left hand opens its fingers and draws back, and the bar slides out of them as the panel shuts.

    Check modes (TASK_DEMO): `one_arm`: the right hand opens the panel, lets go and fetches the cup, so the panel shuts first; `early_release`: the
    window is let go while the right arm is still out; `no_close`: the window is held open to the end."""
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode or "full"
    bar = d.level(-45)
    ab = bar.rot[:, 2]
    cupg = d.tilted(85, 0)  # fingers 5 degrees below level: the wrist camera rides behind and above the lid

    def bar_at(y):
        return np.array([BAR_X, y, TOP + BAR_GRIP_Z])

    def bar_now():
        return bar_at(BAR_Y0 + ctx.state("window").joint("panel"))

    def cup_tcp(xy, base):
        return np.array([xy[0], xy[1], TOP + base + CUP_TCP_H])

    for _ in range(5):
        yield
    park_r = np.array([0.18, -0.20, TOP + 0.30])
    if mode == "one_arm":
        yield from r.move(bar_at(BAR_Y0) - 0.07 * ab, grip=bar, finger=JAW_BAR, t=2.5, label="right: to the bar")
        yield from r.move(bar_at(BAR_Y0), t=1.0, label="right: round the bar")
        yield from r.close(2 * BAR_R, 25.0, t=0.6, label="right: hold the bar")
        yield from r.move(bar_at(BAR_Y0 + 0.20), t=2.5, label="right: slide it open")
        yield from r.move(r.at(), finger=JAW_BAR, t=0.4, label="right: let go of the bar to fetch the cup")
        yield from r.move(r.at() - 0.07 * ab, t=0.6, label="right: off the bar")
        yield from r.move(cup_tcp(CUP_START + [-0.08, -0.10], 0.16), grip=cupg, finger=d.FINGER_OPEN, t=2.0, label="right: up and round")
        yield from r.move(cup_tcp(CUP_START - [0.08, 0.0], 0.0), t=1.5, label="right: behind the cup")
        yield from r.move(cup_tcp(CUP_START - [PICK_BACK, 0.0], 0.0), t=1.2, label="right: round the cup")
        yield from r.close(CUP_W, CUP_SQUEEZE, t=0.6, label="right: hold the cup")
        yield from r.move(cup_tcp(CUP_START, CARRY_BASE), t=1.5, label="right: lift")
        yield from r.move(cup_tcp((0.30, SERVE[1]), CARRY_BASE), t=1.5, label="right: toward the shut window")
        yield from r.wait(2.0)
        print(f"[serve] one_arm end: {TRIAL.report}", flush=True)
        return

    yield from ctx.together(l.move(bar_at(BAR_Y0) - 0.07 * ab, grip=bar, finger=JAW_BAR, t=2.5, label="left: to the bar, jaws half open"),
                            r.move(cup_tcp(CUP_START + [-0.08, -0.10], 0.16), grip=cupg, t=1.5, label="right: up and round, clear of the cup"))
    yield from ctx.together(l.move(bar_at(BAR_Y0), t=1.0, label="left: round the bar"),
                            r.move(cup_tcp(CUP_START - [0.08, 0.0], 0.0), t=1.5, label="right: behind the cup"))
    yield from r.move(cup_tcp(CUP_START - [PICK_BACK, 0.0], 0.0), t=1.2, label="right: round the cup")
    yield from ctx.together(l.close(2 * BAR_R, 25.0, t=0.6, label="left: hold the bar"), r.close(CUP_W, CUP_SQUEEZE, t=0.6, label="right: hold the cup"))
    yield from ctx.together(l.move(bar_at(BAR_Y0 + OPEN_TO), t=2.5, label="left: slide the window open"),
                            r.move(cup_tcp(CUP_START, CARRY_BASE), t=1.6, label="right: lift the cup"))
    cup_pt = lambda: ctx.state("cup").pos + [0.0, 0.0, CUP_TCP_H]  # noqa: E731  (the cup's axis at the hand's height: the arm sags under it)
    yield from ctx.together(l.servo(bar_at(BAR_Y0 + OPEN_TO), read=bar_now, tol=0.004, timeout=1.5, label="left: hold it open"),
                            r.servo(cup_tcp(CUP_START, CARRY_BASE), read=cup_pt, tol=0.004, timeout=1.5, label="right: cup up to the carry height"))
    print(f"[serve] open: {TRIAL.report}", flush=True)
    yield from r.move(cup_tcp((0.30, SERVE[1]), CARRY_BASE), t=1.2, label="right: to the opening")
    yield from r.move(cup_tcp(SERVE, CARRY_BASE), t=2.2, label="right: out through the opening")
    yield from r.servo(cup_tcp(SERVE, CARRY_BASE), read=cup_pt, tol=0.004, timeout=1.5, label="right: cup over the shelf")
    yield from r.move(r.at() + [0.0, 0.0, SHELF_TOP + 0.01 - CARRY_BASE], t=1.4, label="right: down toward the shelf")
    for _ in range(30):  # lower it 1.5 mm a step until its base sits on the shelf
        if float(ctx.state("cup").pos[2] - TOP) - SHELF_TOP <= 0.002:
            break
        yield from r.move(r.at() - [0.0, 0.0, 0.0015], t=0.1, label="right: set the cup down")
    yield from r.wait(0.4)
    print(f"[serve] on the shelf: {TRIAL.report}", flush=True)
    yield from r.open(t=0.5, label="right: let go of the cup")
    out = r.at()
    if mode == "early_release":
        yield from l.move(l.at(), finger=JAW_BAR, t=0.4, label="left: let go of the window too early")
        yield from l.move(l.at() - 0.07 * ab, t=0.6, label="left: off the bar")
        yield from r.wait(2.0)
        print(f"[serve] early_release end: {TRIAL.report}", flush=True)
        return
    yield from r.move(out - [0.06, 0.0, 0.0], t=0.8, label="right: fingers off the cup")
    yield from r.move(np.array([0.22, out[1], out[2]]), t=1.8, label="right: back in through the opening")
    yield from r.move(park_r, t=1.2, label="right: clear of the window")
    if mode == "no_close":
        yield from r.wait(3.0)
        print(f"[serve] no_close end: {TRIAL.report}", flush=True)
        return
    yield from l.move(l.at(), finger=JAW_BAR, t=0.4, label="left: let the window go")
    yield from l.move(l.at() - 0.08 * ab, t=0.7, label="left: draw back; the bar slides out of the fingers")
    yield from l.move(np.array([0.16, 0.22, TOP + 0.36]), t=1.2, label="left: clear")
    yield from l.wait(3.0)
    print(f"[serve] end: {TRIAL.report}", flush=True)
