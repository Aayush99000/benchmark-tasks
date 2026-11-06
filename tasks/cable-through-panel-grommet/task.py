"""On the station table: a grey upright panel (280 mm long, 100 mm tall, 6 mm thick) stands on the midline between
the two arms (its faces look towards the arms, y = 0), running away from the robot from x 16 cm to x 44 cm. At
x 30 cm, low in the panel, a black square grommet with a 26 x 26 mm bore (bottom of the bore 10 mm above the
table) passes through it. On the left-arm side, a grey junction box (40 x 40 x 30 mm) at x 15, y 13 holds the fixed
end of a black 6 mm cable (40 cm long) that lies straight on the table diagonally away from the robot and to the
left (45 deg) and ends in a black 60 x 16 x 12 mm plug. A 20 mm red band is taped on the cable, its centre 25 cm from the junction box. The plug
must be pushed through the grommet from the left side, taken by the right arm on the far side, carried over a white
30 mm-tall duct wall that runs parallel to the panel 13-14 cm behind it, and the cable pulled through until the red
band sits at the grommet (within 2 cm). Neither arm can reach the other side of the panel,
so the plug has to be handed over through the hole.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Push the cable's plug through the black grommet in the grey upright panel, take it on the other "
               "side with the other arm, and pull the cable through until the red band on the cable sits at the "
               "grommet.")

# -- panel with grommet (panel plane y = 0; near side = left arm, y > 0; far side = right arm, y < 0) ---------------
PANEL_X0, PANEL_X1 = 0.16, 0.44
PANEL_T = 0.006  # panel thickness (y)
PANEL_H = 0.10
HOLE_X = 0.30
HOLE_W = 0.026  # square bore
HOLE_Z0 = TOP + 0.010  # bottom of the bore
HOLE_Z1 = HOLE_Z0 + HOLE_W
HOLE_ZC = (HOLE_Z0 + HOLE_Z1) / 2
GROM_WALL = 0.006  # grommet frame wall
GROM_T = 0.010  # grommet depth (y): sticks out 2 mm from each panel face
GROM_FACE = GROM_T / 2  # |y| of the grommet faces
GREY = (0.62, 0.64, 0.66)
BLACK = (0.04, 0.04, 0.04)


def _box(name, x0, x1, z0, z1, t, color):
    return Box(name, pos=((x0 + x1) / 2, 0.0, (z0 + z1) / 2), size=(x1 - x0, t, z1 - z0), color=color)


_gx0, _gx1 = HOLE_X - HOLE_W / 2 - GROM_WALL, HOLE_X + HOLE_W / 2 + GROM_WALL
PANEL = [
    _box("panel_left", PANEL_X0, _gx0, TOP, TOP + PANEL_H, PANEL_T, GREY),
    _box("panel_right", _gx1, PANEL_X1, TOP, TOP + PANEL_H, PANEL_T, GREY),
    _box("panel_below", _gx0, _gx1, TOP, HOLE_Z0 - GROM_WALL, PANEL_T, GREY),
    _box("panel_above", _gx0, _gx1, HOLE_Z1 + GROM_WALL, TOP + PANEL_H, PANEL_T, GREY),
    _box("grommet_bottom", _gx0, _gx1, HOLE_Z0 - GROM_WALL, HOLE_Z0, GROM_T, BLACK),
    _box("grommet_top", _gx0, _gx1, HOLE_Z1, HOLE_Z1 + GROM_WALL, GROM_T, BLACK),
    _box("grommet_near", _gx0, HOLE_X - HOLE_W / 2, HOLE_Z0, HOLE_Z1, GROM_T, BLACK),
    _box("grommet_far", HOLE_X + HOLE_W / 2, _gx1, HOLE_Z0, HOLE_Z1, GROM_T, BLACK),
]

# -- cable from the junction box, with a plug and a red band ---------------------------------------------------------
R = 0.003  # 6 mm cable
CORD_LEN = 0.40  # 20 segments of 2 cm
PLUG = Plug(size=(0.060, 0.016, 0.012), mass=0.012, color=(0.06, 0.06, 0.06))
CORD_YAW = math.radians(45.0)  # away from the robot, towards the left
_DIR = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW)])
CORD_START = (0.16, 0.14, TOP + PLUG.size[2] / 2 + 0.0005)  # raised so the plug does not start inside the table
BOX_SIZE = (0.04, 0.04, 0.03)
_bc = np.array(CORD_START[:2]) - _DIR * BOX_SIZE[0] / 2
JBOX = Box("junction_box", pos=(float(_bc[0]), float(_bc[1]), TOP + BOX_SIZE[2] / 2), size=BOX_SIZE,
           yaw=CORD_YAW, color=(0.45, 0.47, 0.50))
CORD = cable("cord", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=True,
             end_plug=PLUG, color=(0.05, 0.05, 0.05), bend_stiffness=0.01, bend_damping=0.001)
BAND_SEG = 12  # the red band is segment 12: arclength 24-26 cm from the junction box
BAND_S = (BAND_SEG + 0.5) * CORD.seg_len  # 0.25 m: band centre
BAND_COLOR = (0.85, 0.06, 0.05)

# -- far side: a wiring-duct wall (30 mm tall) 13-14 cm behind the panel; the cable continues over it -----------------
RAIL_Y0, RAIL_Y1, RAIL_H = -0.140, -0.130, 0.030
RAIL = Box("duct_wall", pos=(0.30, (RAIL_Y0 + RAIL_Y1) / 2, TOP + RAIL_H / 2), size=(0.24, RAIL_Y1 - RAIL_Y0, RAIL_H),
           color=(0.92, 0.92, 0.90))

OBJECTS = [*PANEL, JBOX, CORD, RAIL]


# cablekit has no per-segment colour: wrap its spawner (looked up when the scene is built) to paint the band
# segment red. Visual only; the physics is untouched.
def _install_band() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_band", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.sim.utils import bind_visual_material
            from isaaclab.sim.utils.stage import get_current_stage
            from yam_arena.station import linear_rgb

            if cfg.cable_name == CORD.name:
                look = f"{prim_path}/Looks/band"
                mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BAND_COLOR), roughness=0.5)
                mat.func(look, mat)
                bind_visual_material(f"{prim_path}/seg_{BAND_SEG}/geom", look, stage=get_current_stage())
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[grommet task] band colour not applied: {e}", flush=True)
        return prim

    spawn._band = True
    cablekit._spawn_cable = spawn


_install_band()

# -- scoring ---------------------------------------------------------------------------------------------------------
BAND_TOL = 0.02  # band centre within 2 cm (along the cable) of the panel's mid-plane
APERTURE_TOL = 0.002


def _line(objects) -> tuple[np.ndarray, np.ndarray]:
    """Cable centre line from the junction box to the plug's front face, and the arclength of each vertex."""
    c = objects["cord"]
    poly = c.polyline()
    if c.end_plug is not None:
        poly = np.vstack([poly, c.end_tip + c.end_dir * PLUG.size[0]])
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    return poly, s


def crossings(objects) -> list[tuple[float, np.ndarray]]:
    """Every place where the cable (or plug) crosses the panel's mid-plane y = 0: (arclength, point)."""
    poly, s = _line(objects)
    out = []
    for i in range(len(poly) - 1):
        a, b = poly[i, 1], poly[i + 1, 1]
        if (a > 0) != (b > 0):
            t = a / (a - b)
            out.append((float(s[i] + t * (s[i + 1] - s[i])), poly[i] + t * (poly[i + 1] - poly[i])))
    return out


def _in_bore(p: np.ndarray) -> bool:
    return bool(abs(p[0] - HOLE_X) <= HOLE_W / 2 + APERTURE_TOL and HOLE_Z0 - APERTURE_TOL <= p[2]
                <= HOLE_Z1 + APERTURE_TOL)


def threaded(objects) -> float | None:
    """If the cable passes through the grommet once (and nowhere else crosses the panel line, e.g. round its end
    or over its top) and the whole plug is beyond the grommet's far face: the arclength (m from the junction box)
    of the cable at the panel. Otherwise None."""
    xs = crossings(objects)
    c = objects["cord"]
    if len(xs) != 1 or not _in_bore(xs[0][1]) or c.end_plug is None:
        return None
    if c.end_tip[1] > -GROM_FACE:  # plug's cable-side end still in / before the grommet
        return None
    return xs[0][0]


def band_offset(objects) -> float | None:
    """Distance along the cable (m) from the red band's centre to the panel: > 0 band still on the near side,
    < 0 pulled past. None if the cable is not threaded."""
    s = threaded(objects)
    return None if s is None else s - BAND_S


FULL_D = CORD_LEN - BAND_S  # 0.15 m: band-to-panel distance when the cable's end has just come through
DRAWN_Y = RAIL_Y0 - 0.005  # plug centre beyond the duct wall (>= 14.5 cm behind the panel): drawn out over the wall


def drawn_out(objects) -> bool:
    """Threaded, and the plug lifted over the duct wall: its centre at least 14.5 cm behind the panel's mid-plane
    (5 mm past the wall's far face)."""
    p = objects["cord"].end_plug
    return threaded(objects) is not None and p is not None and p[1] <= DRAWN_Y


def success(objects) -> bool:
    d = band_offset(objects)
    return d is not None and drawn_out(objects) and abs(d) <= BAND_TOL


def score(objects) -> float:
    """0 unless the cable passes through the grommet (and crosses the panel line nowhere else) with the plug centre
    beyond the duct wall (>= 14.5 cm behind the panel's mid-plane). Then: band centre within 2 cm of the panel (along the cable):
    1.0; otherwise 0.3 + 0.5 x max(0, (15 cm - band distance) / 13 cm)."""
    d = band_offset(objects)
    if d is None or not drawn_out(objects):
        return 0.0
    if abs(d) <= BAND_TOL:
        return 1.0
    return 0.3 + 0.5 * float(np.clip((FULL_D - abs(d)) / (FULL_D - BAND_TOL), 0.0, 1.0))


def distance(objects) -> float:
    """|band offset| (m) once threaded; else 0.5 + plug-to-bore distance."""
    d = band_offset(objects)
    if d is not None:
        return abs(d)
    p = objects["cord"].end_plug
    return 0.5 + float(np.linalg.norm(p - np.array([HOLE_X, 0.0, HOLE_ZC]))) if p is not None else 1.0


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _cord_at(objects, a: float) -> np.ndarray:
    poly, s = _line(objects)
    return np.array([np.interp(a, s, poly[:, k]) for k in range(3)])


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    hi = TOP + 0.07
    feed_z = HOLE_ZC + 0.010  # TCP when holding the plug / cable at bore height
    YL, YR = -math.pi / 2, math.pi / 2  # IK-friendly grip yaws near the panel for left / right

    def log(tag):
        st = ctx.states()
        c = st["cord"]
        xs = [(round(a, 3), np.round(p, 3).tolist()) for a, p in crossings(st)]
        print(f"[demo] {tag}: plug={np.round(c.end_plug, 3)} tip={np.round(c.end_tip, 3)} cross={xs} "
              f"band={band_offset(st)} score={score(st):.3f}", flush=True)

    # Cheat variants (scoring checks, must score 0): GROMMET_CHEAT=nothing | drop (one arm: push the plug through,
    # keep pushing the cable, let go; no hand-off) | around (hand the plug round the panel's near end instead of
    # through the grommet).
    cheat = os.environ.get("GROMMET_CHEAT", "")
    log("start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        log("end")
        return
    # 1) Left arm: take the plug by its cable-side end and carry it to the grommet, plug pointing through.
    st = ctx.states()
    c = st["cord"]
    g = c.end_tip + c.end_dir * 0.008  # 8 mm into the plug from its cable-side end
    gy = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    yield from L.move((g[0], g[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((g[0], g[1], c.end_plug[2] + 0.010), yaw=gy, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    yield from L.move((g[0], g[1], hi), yaw=gy, duration=0.8)
    if cheat == "around":
        ax = PANEL_X0 - 0.04
        yield from L.move((ax, 0.07, hi), yaw=YL, duration=1.5)
        yield from L.move((ax, 0.07, feed_z), yaw=YL, duration=1.0)
        yield from L.move((ax, 0.005, feed_z), yaw=YL, duration=1.2)
        pp = ctx.states()["cord"].end_plug
        yield from Rt.move((pp[0], pp[1] - 0.012, hi), yaw=YR, finger=FINGER_OPEN, duration=1.5)
        pp = ctx.states()["cord"].end_plug
        yield from Rt.move((pp[0], pp[1] - 0.012, max(pp[2] + 0.010, TOP + 0.013)), yaw=YR, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED)
        yield from L.grip(FINGER_OPEN, duration=0.4)
        yield from ctx.both(L.move((ax, 0.10, hi), yaw=YL, duration=0.8), Rt.wait(0.1))
        yield from Rt.move((0.22, -0.20, feed_z), yaw=YR, duration=2.5)
        yield from Rt.wait(1.5)
        log("end")
        return
    grip_y0 = 0.090  # plug front 3 cm before the grommet
    yield from L.move((HOLE_X, grip_y0, hi), yaw=YL, duration=1.5)
    yield from L.move((HOLE_X, grip_y0, feed_z), yaw=YL, duration=1.0)
    log("plug at grommet")
    push_y = 0.046  # closest the left TCP comes to the panel (the fingers are ~7 cm wide along the cable)
    yield from L.move((HOLE_X, push_y, feed_z), yaw=YL, duration=1.5)
    log("pushed 1")
    # 2) Re-grip the cable behind the plug and push again until the whole plug is through.
    n_push = 7 if cheat == "drop" else 4
    for k in range(n_push):
        st = ctx.states()
        if cheat != "drop" and st["cord"].end_plug[1] < -0.060:  # plug centre far enough out for the right arm
            break
        yield from L.grip(FINGER_OPEN, duration=0.4)
        yield from L.move((HOLE_X, push_y, feed_z + 0.02), yaw=YL, duration=0.4)
        yield from L.move((HOLE_X, push_y + 0.030, feed_z + 0.02), yaw=YL, duration=0.6)
        st = ctx.states()
        # where the cable now is, 3 cm behind where the fingers were
        poly, s = _line(st)
        i = int(np.argmin(np.abs(poly[:, 1] - (push_y + 0.030)) + 10 * (s < 0.1)))
        p = poly[i]
        yield from L.move((HOLE_X, push_y + 0.030, max(p[2], TOP + R) + 0.010), yaw=YL, duration=0.6)
        yield from L.grip(FINGER_CLOSED, duration=0.5)
        yield from L.move((HOLE_X, push_y, feed_z), yaw=YL, duration=1.2)
        log(f"pushed {k + 2}")
    if cheat == "drop":
        yield from L.grip(FINGER_OPEN, duration=0.4)
        yield from L.move((HOLE_X, 0.08, hi), yaw=YL, duration=0.8)
        yield from L.wait(2.0)
        log("end")
        return
    # 3) Left arm lets go and backs off (the two wrists would collide above the panel), right arm takes the plug on
    #    the far side.
    yield from L.grip(FINGER_OPEN, duration=0.4)
    yield from L.move((HOLE_X, push_y, hi), yaw=YL, duration=0.5)
    st = ctx.states()
    pp = st["cord"].end_plug
    yield from ctx.both(Rt.move((pp[0], pp[1] - 0.01, hi), yaw=YR, finger=FINGER_OPEN, duration=1.5),
                        L.move((HOLE_X - 0.05, 0.16, hi + 0.03), yaw=YL, duration=1.0))
    st = ctx.states()
    pp = st["cord"].end_plug
    c = st["cord"]
    gp = c.end_tip + c.end_dir * (PLUG.size[0] - 0.015)  # 15 mm from the plug's front end
    gz = max(pp[2] + 0.010, TOP + 0.013)
    yield from Rt.move((gp[0], gp[1], hi), yaw=YR, duration=0.6)
    yield from Rt.move((gp[0], gp[1], gz), yaw=YR, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    log("right has plug")
    pull_z = TOP + 0.065  # carry the plug over the duct wall
    yield from Rt.move((gp[0], gp[1], pull_z), yaw=YR, duration=0.8)
    # 4) Right arm pulls straight away from the panel, over the duct wall, until the band is at the grommet.
    y = gp[1]
    for k in range(8):
        st = ctx.states()
        d = band_offset(st)
        log(f"pull {k}")
        if d is not None and abs(d) <= 0.006:
            break
        step = 0.04 if d is None else float(np.clip(d, -0.03, 0.06))
        y = max(y - step, -0.27)
        yield from Rt.move((HOLE_X, y, pull_z), yaw=YR, duration=max(0.6, abs(step) * 25))
        yield from Rt.wait(0.3)
    log("pulled")
    yield from Rt.wait(1.5)
    log("end")
