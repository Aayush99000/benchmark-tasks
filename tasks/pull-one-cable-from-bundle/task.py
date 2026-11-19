"""On the station table, in front of the robot, a white open-ended cable tray (40 cm long left-right, 8 cm wide
inside, 3 cm walls) holds four 6 mm cables lying side by side on its floor, 15 mm apart, held in by two black
bridge clips spanning the tray (10 mm clearance under each). From the robot outwards: grey A, blue, grey B, grey C.
The three grey cables are live; the blue one is decommissioned. All four run under both clips and out of the
tray's right end, each ending in an identical black plug on the table; the plugs are staggered so that neither the
plug nearest the robot (grey A) nor the one sticking out furthest (grey C) is the blue one. The blue cable's left
end lies inside the tray, left of the left clip, so it can only come out lengthwise through the clips. It must be
drawn out of the tray's right end and left, with its plug, lying on the table entirely outside the tray (not
hanging off the table, not draped over other cables), without pulling the live cables out of the clips or moving
their plugs more than 3 cm, and then let go. This is the data-centre job of removing abandoned cable from a live
tray (see README).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The blue cable in the white tray is decommissioned. Pull it out of the tray through the two black "
               "clips and lay it, with its plug, on the table completely outside the tray, then let go. The three "
               "grey cables are live: they must stay in the tray under both clips.")

# -- tray (open at both ends), fixed -----------------------------------------------------------------------------
TX0, TX1 = 0.26, 0.34  # inner faces of the near / far wall (x)
TY0, TY1 = -0.20, 0.20  # tray ends (y)
WT = 0.004  # wall thickness
FLOOR_T = 0.003
FLOOR = TOP + FLOOR_T  # tray floor surface
WALL_H = 0.03  # wall height above the floor
WALL_TOP = FLOOR + WALL_H
TRAY_C = ((TX0 + TX1) / 2, (TY0 + TY1) / 2)
WHITE = (0.93, 0.93, 0.91)
TRAY = [
    Box("tray_floor", pos=(TRAY_C[0], TRAY_C[1], TOP + FLOOR_T / 2), size=(TX1 - TX0 + 2 * WT, TY1 - TY0, FLOOR_T),
        color=WHITE),
    Box("tray_wall_near", pos=(TX0 - WT / 2, TRAY_C[1], TOP + (FLOOR_T + WALL_H) / 2),
        size=(WT, TY1 - TY0, FLOOR_T + WALL_H), color=WHITE),
    Box("tray_wall_far", pos=(TX1 + WT / 2, TRAY_C[1], TOP + (FLOOR_T + WALL_H) / 2),
        size=(WT, TY1 - TY0, FLOOR_T + WALL_H), color=WHITE),
]

# -- clips: bridges spanning the tray between the walls ----------------------------------------------------------
CLIP_Y = (0.06, -0.10)  # left, right
CLIP_W = 0.02  # along y
CLIP_CLEAR = 0.010  # floor to clip underside
CLIP_T = 0.005
CLIPS = [Box(f"clip_{k}", pos=(TRAY_C[0], y, FLOOR + CLIP_CLEAR + CLIP_T / 2), size=(TX1 - TX0, CLIP_W, CLIP_T),
             color=(0.08, 0.08, 0.08)) for k, y in zip(("left", "right"), CLIP_Y)]

# -- cables: four 6 mm cables side by side on the tray floor, all leaving the tray's right end with a plug ----------
R = 0.003  # 6 mm cables
PITCH = 0.015  # loosely laid: 9 mm gaps between neighbours
NAMES = ("grey_a", "blue", "grey_b", "grey_c")  # from the robot outwards
GREYS = ("grey_a", "grey_b", "grey_c")
ROW_X = {n: 0.2775 + i * PITCH for i, n in enumerate(NAMES)}
Z0 = FLOOR + R + 0.0005
START_Y = {"grey_a": 0.18, "blue": 0.09, "grey_b": 0.18, "grey_c": 0.18}  # left (free) ends, inside the tray
END_Y = {"grey_a": -0.28, "blue": -0.34, "grey_b": -0.28, "grey_c": -0.40}  # right cable ends (plugs beyond)
COLOR = {"grey_a": (0.55, 0.56, 0.58), "blue": (0.08, 0.30, 0.85), "grey_b": (0.50, 0.51, 0.53),
         "grey_c": (0.60, 0.60, 0.62)}
PLUG = Plug(size=(0.03, 0.010, 0.010), mass=0.008, color=(0.06, 0.06, 0.06))
CABLES = {n: cable(n, start=(ROW_X[n], START_Y[n], Z0), yaw=-math.pi / 2, length=START_Y[n] - END_Y[n], radius=R,
                   end_plug=PLUG, color=COLOR[n], bend_stiffness=0.01, bend_damping=0.001) for n in NAMES}
PLUG_START = {n: np.array([ROW_X[n], END_Y[n] - PLUG.size[0] / 2]) for n in NAMES}

OBJECTS = [*TRAY, *CLIPS, *CABLES.values()]

# -- scoring --------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.04  # every gripper TCP at least this far from every cable point at the end


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _pts(objects, name) -> np.ndarray:
    c = objects[name]
    extra = [p[None] for p in (c.start_plug, c.end_plug) if p is not None]
    return np.vstack([_dense(c.polyline())] + extra)


def under_clip(pts: np.ndarray, clip_y: float) -> bool:
    """Some cable point lies under the clip: within its width, between the walls, below its underside."""
    ok = (np.abs(pts[:, 1] - clip_y) <= CLIP_W / 2) & (pts[:, 0] >= TX0) & (pts[:, 0] <= TX1) \
        & (pts[:, 2] < FLOOR + CLIP_CLEAR)
    return bool(ok.any())


def over_tray(pts: np.ndarray) -> np.ndarray:
    """Per point: over the tray's footprint in top view (walls included, any height)."""
    return (pts[:, 0] >= TX0 - WT - R) & (pts[:, 0] <= TX1 + WT + R) & (pts[:, 1] >= TY0 - R) \
        & (pts[:, 1] <= TY1 + R)


PLUG_TOL = 0.03  # a live cable's plug may move at most this far (horizontally) from its starting mark


def grey_intact(objects, name) -> bool:
    """A live cable still passes under both clips and its plug is within 3 cm of its starting mark."""
    c = objects[name]
    p = _pts(objects, name)
    return all(under_clip(p, y) for y in CLIP_Y) and c.end_plug is not None \
        and float(np.linalg.norm(c.end_plug[:2] - PLUG_START[name])) <= PLUG_TOL


def blue_unclipped(objects) -> bool:
    p = _pts(objects, "blue")
    return not any(under_clip(p, y) for y in CLIP_Y)


def blue_out(objects) -> bool:
    return not bool(over_tray(_pts(objects, "blue")).any())


# -- "supported" / "resting on the table" -------------------------------------------------------------------------
# A blue cable point (or the blue plug) counts as supported only if it is in contact with a real support surface: the
# bare table top, the tray floor between the walls, or a wall top. Contact is a height test against that surface
# (cable centre within 3 mm of lying on it, i.e. at most radius + 3 mm above it) plus a check that no grey cable or
# grey plug lies directly beneath it (overlapping it in top view and lower), so a blue cable or plug resting on the
# live cables, in the tray or on the table, is never supported.
# The station table top spans x -0.19..0.91, y -0.65..0.65 (yam_arena station.TABLE_CENTER / TABLE_SIZE).
TABLE_X = (-0.19, 0.91)
TABLE_Y = (-0.65, 0.65)
CONTACT_TOL = 0.003  # centre of a cable point / plug face within 3 mm (above or below) of lying on the surface
UNDER_MARGIN = 0.002  # a grey item is "beneath" if its centre is this much lower than the blue point / plug centre
PH = (PLUG.size[0] / 2, PLUG.size[2] / 2)  # plug half length (along the cable), half width / height
FLAT_SIN = 0.10  # a plug lies flat if its long axis is within ~6 deg of horizontal
REST_SPEED = 0.02  # m/s, mean speed of the blue cable's segments (CableState.resting)


def _on_table_xy(xy: np.ndarray, margin: float) -> np.ndarray:
    return (xy[:, 0] >= TABLE_X[0] + margin) & (xy[:, 0] <= TABLE_X[1] - margin) & (xy[:, 1] >= TABLE_Y[0] + margin) \
        & (xy[:, 1] <= TABLE_Y[1] - margin)


def _tray_box(xy: np.ndarray) -> np.ndarray:
    """Per point: over the tray's physical footprint (floor plate and walls), no margin."""
    return (xy[:, 0] >= TX0 - WT) & (xy[:, 0] <= TX1 + WT) & (xy[:, 1] >= TY0) & (xy[:, 1] <= TY1)


def _surface(xy: np.ndarray) -> np.ndarray:
    """Per point: height of the support surface directly below it (table top, tray floor between the walls, or a wall
    top); NaN beyond the table's edges."""
    z = np.full(len(xy), np.nan)
    z[_on_table_xy(xy, 0.0)] = TOP
    tray = _tray_box(xy)
    z[tray] = FLOOR
    wall = tray & ((xy[:, 0] < TX0) | (xy[:, 0] > TX1))
    z[wall] = WALL_TOP
    return z


def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Top-view distance from each point p (N, 2) to the segment a-b."""
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab + 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def _plug_axis(c) -> tuple[np.ndarray, np.ndarray] | None:
    """The plug's two end-face centres. It is welded coaxially onto the cable's last segment, so its long axis runs
    through that segment's centre and its own."""
    if c.end_plug is None:
        return None
    u = c.end_plug - c.points[-1]
    u = u / (np.linalg.norm(u) + 1e-12)
    return c.end_plug - u * PH[0], c.end_plug + u * PH[0]


def _grey_beneath(objects, pts: np.ndarray, half_width: float) -> np.ndarray:
    """Per point (N, 3) of a blue body whose top-view half width is ``half_width``: some grey cable point or grey
    plug overlaps it in top view and is lower, i.e. the blue body lies on a live cable or plug there."""
    hit = np.zeros(len(pts), bool)
    for n in GREYS:
        g = _dense(objects[n].polyline(), 0.002)
        d = np.linalg.norm(pts[:, None, :2] - g[None, :, :2], axis=2)
        lower = g[None, :, 2] < pts[:, None, 2] - UNDER_MARGIN
        hit |= ((d < half_width + R - 0.001) & lower).any(axis=1)
        ax = _plug_axis(objects[n])
        if ax is not None:
            d = _seg_dist(pts[:, :2], ax[0][:2], ax[1][:2])
            hit |= (d < half_width + PH[1] - 0.001) & (objects[n].end_plug[2] < pts[:, 2] - UNDER_MARGIN)
    return hit


def blue_points_supported(objects) -> np.ndarray:
    """Per blue cable point (dense along the cable, plug excluded): in contact with the table top, the tray floor or
    a wall top (centre within 3 mm of surface + radius) and not lying on a grey cable or plug."""
    p = _dense(objects["blue"].polyline())
    s = _surface(p[:, :2])
    contact = np.abs(p[:, 2] - (s + R)) <= CONTACT_TOL  # False where s is NaN (beyond the table)
    return contact & ~_grey_beneath(objects, p, R)


def blue_points_on_table(objects) -> np.ndarray:
    """Per blue cable point: supported (as above) by the bare table top, inside the table's edges by the cable radius
    and outside the tray's footprint (walls included, plus the cable radius)."""
    p = _dense(objects["blue"].polyline())
    return blue_points_supported(objects) & _on_table_xy(p[:, :2], R) & ~over_tray(p)


def _plug_contact(objects):
    """(supported, on_table) for the blue plug. It is supported if it lies flat (long axis within ~6 deg of
    horizontal) with both end faces' bottoms within 3 mm of the surface below them, or stands on end with its lower
    face within 3 mm of the surface; and no grey cable or plug lies beneath it. On the table: supported, both end
    faces' centres at least half the plug's width inside the table's edges, and outside the tray's footprint."""
    c = objects["blue"]
    ax = _plug_axis(c)
    if ax is None:
        return False, False
    ends = np.vstack(ax)
    uz = abs(float(ends[1, 2] - ends[0, 2])) / (2 * PH[0])
    s = _surface(ends[:, :2])
    if uz <= FLAT_SIN:
        contact = bool(np.all(np.abs(ends[:, 2] - PH[1] - s) <= CONTACT_TOL))
    elif uz >= 1.0 - FLAT_SIN:
        lo = int(np.argmin(ends[:, 2]))
        contact = bool(abs(ends[lo, 2] - s[lo]) <= CONTACT_TOL)
    else:
        contact = False  # tilted: propped on something
    axis_pts = ends[0] + (ends[1] - ends[0]) * np.linspace(0, 1, 7)[:, None]
    axis_pts[:, 2] = c.end_plug[2]
    supported = contact and not bool(_grey_beneath(objects, axis_pts, PH[1]).any())
    on_table = supported and bool(np.all(_on_table_xy(ends[:, :2], PH[1]))) and not bool(over_tray(ends).any()) \
        and bool(np.all(s == TOP))
    return supported, on_table


def blue_plug_supported(objects) -> bool:
    return _plug_contact(objects)[0]


def blue_plug_on_table(objects) -> bool:
    return _plug_contact(objects)[1]


def blue_still(objects) -> bool:
    return objects["blue"].resting(max_speed=REST_SPEED)


def blue_supported(objects) -> bool:
    """Every point of the blue cable and its plug is in contact with the table, the tray floor or a wall top, none of
    it lies on a grey cable or plug, and the cable is still."""
    return bool(blue_points_supported(objects).all()) and blue_plug_supported(objects) and blue_still(objects)


def blue_on_table(objects) -> bool:
    """The whole blue cable and its plug rest on the table top outside the tray's footprint, and the cable is still."""
    return bool(blue_points_on_table(objects).all()) and blue_plug_on_table(objects) and blue_still(objects)


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_bundle_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._bundle_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(objects) -> bool:
    pts = np.vstack([_pts(objects, n) for n in NAMES])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless both grippers are >= 4 cm from every cable, every grey cable still passes under both clips with its
    plug within 3 cm of its starting mark, and every point of the blue cable and its plug is in contact with the
    table, the tray floor or a wall top, none of it lying on a grey cable or plug, and the cable is still. Then 1.0 if
    the whole blue cable and its plug rest on the bare table top outside the tray's footprint; 0.5 if the blue cable
    passes under neither clip (part of it still resting on the tray floor); else 0 (still under a clip)."""
    if not released(objects):
        return 0.0
    if not all(grey_intact(objects, n) for n in GREYS):
        return 0.0
    if not blue_supported(objects):
        return 0.0
    if blue_on_table(objects):
        return 1.0
    return 0.5 if blue_unclipped(objects) else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Blue cable length (m) not yet lying on the bare table outside the tray, plus 0.05 m if its plug is not
    (1 m if a grey cable has been disturbed)."""
    if not all(grey_intact(objects, n) for n in GREYS):
        return 1.0
    return float((~blue_points_on_table(objects)).sum() * 0.004) + (0.0 if blue_plug_on_table(objects) else 0.05)


# -- camera-based estimates (top camera only) ------------------------------------------------------------------------
# The scripted demo sees the scene only through the top camera image (ctx.obs.images["top_cam"]) and knows the arms'
# own poses and the fixed layout from the setup (tray, clips, where the left hand presses); it never reads cable or
# plug poses from the simulator. vision.py (the kit's camera helpers) back-projects pixels onto horizontal planes with
# the station's published top-camera calibration. A hue/saturation mask picks the blue cable; dark pixels pick plugs;
# grey or dark pixels next to the blue cable are treated as other cables (obstacles for a regrasp).
import vision  # noqa: E402

CABLE_Z = TOP + R  # centre height of a cable lying on the table (the tray floor is only 3 mm higher)
PLUG_TOP_Z = TOP + PLUG.size[2]  # back-project plug pixels onto their top face


def _blue(img):  # thin blue cable: blends with the table, so a low saturation floor
    return vision.hsv_mask(img, 222, 22, min_sat=0.15)


def _dark(img):  # black plugs (and clips, arm parts: use only inside a region of interest)
    return np.asarray(img)[..., :3].max(axis=-1) < 70


def _greyish(img):  # grey cables, black plugs: unsaturated, not white (the oak table is saturated ~0.3-0.45)
    f = np.asarray(img, dtype=float)[..., :3]
    mx, mn = f.max(axis=-1), f.min(axis=-1)
    return ((mx - mn) / (mx + 1e-9) < 0.16) & (mx < 200)


def see_blue(ctx) -> np.ndarray:
    """(N, 2) table-plane positions of every blue pixel in the top camera image."""
    img = vision.top_image(ctx)
    return vision.top_camera(img.shape).to_plane(_blue(img), CABLE_Z)


def see_blue_plug(ctx):
    """Image estimate of the blue cable's plug centre (x, y): the blue cable's line outside the tray's right end gives
    x and the end of the blue run gives y; the dark plug pixels just beyond that end refine it. None if the blue
    cable is not seen outside the tray."""
    img = vision.top_image(ctx)
    cam = vision.top_camera(img.shape)
    pts = cam.to_plane(_blue(img), CABLE_Z)
    out = pts[(pts[:, 1] < TY0 - 0.02) & (pts[:, 1] > TY0 - 0.30) & (pts[:, 0] > TX0 - 0.03)
              & (pts[:, 0] < TX1 + 0.03)]
    if len(out) < 5:
        return None
    x, y_end = float(np.median(out[:, 0])), float(np.percentile(out[:, 1], 3))
    guess = np.array([x, y_end - PLUG.size[0] / 2])
    dk = cam.to_plane(_dark(img), PLUG_TOP_Z)
    near = dk[(np.abs(dk[:, 0] - x) < 0.010) & (dk[:, 1] < y_end + 0.005) & (dk[:, 1] > y_end - PLUG.size[0] - 0.01)]
    return near.mean(axis=0) if len(near) >= 3 else guess


def see_blue_over_tray(ctx) -> tuple[int, int]:
    """(blue pixels over the tray's footprint, blue pixels seen in all)."""
    pts = see_blue(ctx)
    over = (pts[:, 0] >= TX0 - WT - 0.005) & (pts[:, 0] <= TX1 + WT + 0.005) & (pts[:, 1] >= TY0 - 0.005) \
        & (pts[:, 1] <= TY1 + 0.005)
    return int(over.sum()), len(pts)


def see_regrasp(ctx):
    """Image estimate of where to re-grasp the blue cable: walking the blue pixels outward from where the cable
    leaves the tray's right end, the first point at least 3 cm beyond the tray end and at least 3 cm from every
    grey or dark pixel that is not part of the blue cable (other cables, plugs). Returns (xy, cable heading) or None."""
    img = vision.top_image(ctx)
    cam = vision.top_camera(img.shape)
    blue = cam.to_plane(_blue(img), CABLE_Z)
    out = blue[blue[:, 1] < TY0 + 0.01]
    exit_pts = out[out[:, 1] > TY0 - 0.02]
    if len(out) < 10 or len(exit_pts) < 2:
        return None
    anchor = np.array([np.median(exit_pts[:, 0]), TY0])
    poly = vision.trace_cable(img, _blue(img) & (_pixel_y(img, cam) < TY0 + 0.01), CABLE_Z, anchor,
                              start_ring=(0.01, 0.04), exclude=0.0)
    if poly is None:
        return None
    grey = cam.to_plane(_greyish(img) & ~_dark(img), CABLE_Z)
    if len(grey):  # drop the mixed edge pixels along the blue cable itself
        d_blue = np.array([np.linalg.norm(blue - o, axis=1).min() for o in grey])
        grey = grey[d_blue > 0.007]
    oth = np.vstack([grey, cam.to_plane(_dark(img), CABLE_Z)])
    oth = oth[(oth[:, 1] < TY0 + 0.02) & (oth[:, 1] > TY0 - 0.35) & (oth[:, 0] > 0.15) & (oth[:, 0] < 0.45)]
    for s in np.arange(0.03, poly.length - 0.03, 0.005):
        xy, h = poly.at(s)
        if xy[1] > TY0 - 0.03:
            continue
        if len(oth) and float(np.linalg.norm(oth - xy, axis=1).min()) < 0.03:
            continue
        return xy, h
    return None


BLUE_LEN = START_Y["blue"] - END_Y["blue"]  # 0.43 m, from the setup
PLUG_PRIOR = np.array([0.20, -0.52])  # where the right hand left the plug and pulled the cable (its own motions)


def see_tail_grasp(ctx):
    """Image estimate of where to pick the extracted blue cable up near its free (tail) end, to lift it off anything
    it lies across: the blue pixel farthest from where the hand left the plug is taken as the tail; the blue cable is
    traced from there towards the plug, and the first point at least 2 cm along it and at least 3 cm from every grey or
    dark pixel that is not blue (other cables, plugs, arm) is chosen. Returns (xy, cable heading, arc length from the
    tail) or None."""
    img = vision.top_image(ctx)
    cam = vision.top_camera(img.shape)
    mask = _blue(img)
    blue = cam.to_plane(mask, CABLE_Z)
    if len(blue) < 10:
        return None
    tail = blue[int(np.argmax(np.linalg.norm(blue - PLUG_PRIOR, axis=1)))]
    poly = vision.trace_cable(img, mask, CABLE_Z, tail, start_ring=(0.005, 0.03), exclude=0.0)
    if poly is None:
        return None
    grey = cam.to_plane(_greyish(img) & ~_dark(img), CABLE_Z)
    if len(grey):
        d_blue = np.array([np.linalg.norm(blue - o, axis=1).min() for o in grey])
        grey = grey[d_blue > 0.007]
    oth = np.vstack([grey, cam.to_plane(_dark(img), CABLE_Z)])
    oth = oth[np.linalg.norm(oth - tail, axis=1) < 0.25]
    for s in np.arange(0.02, min(poly.length, 0.20), 0.005):
        r = poly.at(s)
        if r is None:
            break
        xy, h = r
        if over_tray(np.array([[xy[0], xy[1], CABLE_Z]]))[0]:
            continue
        if len(oth) and float(np.linalg.norm(oth - xy, axis=1).min()) < 0.03:
            continue
        return xy, h, float(s)
    return None


def _pixel_y(img, cam):
    """Per pixel: table-plane y (for masking image regions by y)."""
    h, w = img.shape[:2]
    full = np.ones((h, w), bool)
    return cam.to_plane(full, CABLE_Z)[:, 1].reshape(h, w)


# -- closed-loop scripted demo (cabledemo.py): camera estimates + the arms' own poses only ---------------------------
PARK_R = (0.10, -0.55, TOP + 0.06)  # right arm out of the top camera's view of the tray end and the pulled cable
PRESS = (0.30, 0.135)  # where the left hand holds the bundle down (fixed layout: left of the blue cable's end)
PRESS_Z = FLOOR + 0.005 + 0.013  # pad tips 5 mm above the floor (1 mm into the 6 mm cables)
LAY_X = 0.14  # the line (x) the extracted cable is laid along: clear table between the robot and the tray / grey plugs
PARK_END = (0.0, -0.60, TOP + 0.10)  # right arm out of the top camera's view of the laid-out cable
PULL_TO = [(0.22, -0.50, TOP + 0.12), (0.20, -0.52, TOP + 0.10), (0.22, -0.50, TOP + 0.10)]


def _log(ctx, tag):
    """Print what the camera sees; with BUNDLE_DEBUG=1 also the simulator's truth (diagnostics and offline checks
    only: nothing printed here feeds back into the demo's motions). BUNDLE_DUMP=dir saves the raw top image."""
    over, seen = see_blue_over_tray(ctx)
    line = f"[demo t={ctx.t:5.2f}] {tag}: blue px seen={seen} over tray={over}"
    if os.environ.get("BUNDLE_DEBUG"):
        st = ctx.states()
        g = {n: (int(grey_intact(st, n)), np.round(st[n].end_plug[:2] - PLUG_START[n], 3).tolist()) for n in GREYS}
        b = st["blue"]
        line += (f" | TRUE score={score(st):.2f} blue_out={blue_out(st)} unclipped={blue_unclipped(st)} "
                 f"blue_start={np.round(b.start_tip, 3)} plug={np.round(b.end_plug, 3)} greys={g} "
                 f"dist={distance(st):.3f} on_table={blue_points_on_table(st).mean():.3f} "
                 f"supported={blue_points_supported(st).mean():.3f} plug_on_table={blue_plug_on_table(st)} "
                 f"plug_dz={1000 * (b.end_plug[2] - TOP):.1f}mm "
                 f"pt_dz_mm=[{1000 * (_dense(b.polyline())[:, 2] - TOP - R).min():.1f},"
                 f"{1000 * (_dense(b.polyline())[:, 2] - TOP - R).max():.1f}] speed={np.linalg.norm(b.lin_vel):.3f}")
    print(line, flush=True)
    d = os.environ.get("BUNDLE_DUMP")
    if d:
        os.makedirs(d, exist_ok=True)
        np.save(os.path.join(d, f"{tag.replace(' ', '_')}_{ctx.t:06.2f}.npy"), np.asarray(ctx.obs.images["top_cam"]))


def _debug_truth(ctx, what, est):
    if os.environ.get("BUNDLE_DEBUG") and est is not None:
        st = ctx.states()
        if what == "tail":
            pts = _dense(st["blue"].polyline(), 0.002)
            others = np.vstack([_pts(st, n) for n in GREYS])
            print(f"[demo] tail grasp est={np.round(est[0], 3)} s={est[2]:.3f} TRUE tail={np.round(st['blue'].start_tip, 3)} "
                  f"off_blue_mm={1000 * np.linalg.norm(pts[:, :2] - est[0], axis=1).min():.0f} "
                  f"to_grey_mm={1000 * np.linalg.norm(others[:, :2] - est[0], axis=1).min():.0f}", flush=True)
        elif what == "plug":
            print(f"[demo] plug est={np.round(est, 3)} TRUE={np.round(st['blue'].end_plug, 3)}", flush=True)
        else:
            pts = _dense(st["blue"].polyline(), 0.002)
            others = np.vstack([_pts(st, n) for n in GREYS])
            print(f"[demo] regrasp est={np.round(est[0], 3)} yaw={math.degrees(est[1]):.0f} "
                  f"off_blue_mm={1000 * np.linalg.norm(pts[:, :2] - est[0], axis=1).min():.0f} "
                  f"to_grey_mm={1000 * np.linalg.norm(others[:, :2] - est[0], axis=1).min():.0f}", flush=True)


def _seen(ctx, fn, tries=4):
    """Look up to ``tries`` times (0.3 s apart): the first estimate that is not None."""
    for _ in range(tries):
        est = fn(ctx)
        if est is not None:
            return est
        yield from ctx.left.wait(0.3)
    return None


def demo_script(ctx):
    """Left arm holds the live cables down on the tray floor, left of the blue cable's end (fingers straddling the
    bundle, pads on grey A and grey C) at a fixed spot from the layout. Right arm, closed loop on the top camera
    only: it parks out of the camera's view, finds the blue cable's plug in the image (blue run outside the tray,
    dark plug at its end), grasps it, pulls it up and to the right, towards the robot, lets go; then looks again and
    re-grasps the blue cable outside the tray at the first point at least 3 cm from every other cable and plug, pulls
    again, until the camera sees no blue over the tray. Then the left arm lets go, and the right arm picks the blue
    cable up near its free end (seen in the image), carries it to a clear line on the robot's side of the tray and
    the grey plugs, lowers it there flat and lets go.
    BUNDLE_CHEAT=nothing|nearest|longest|off_table|partly_off|in_air|propped|yank runs a shortcut instead (each must
    score 0), BUNDLE_CHEAT=partial a partial extraction (0.5);
    BUNDLE_CHEAT_NOLAY=1 runs this demo without the final lay-out (the cable is left where the pulls left it)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    cheat = os.environ.get("BUNDLE_CHEAT", "")
    if cheat in ("partial", "propped"):
        yield from _cheat_partial(ctx, cheat)
        return
    if cheat == "yank":
        yield from _cheat_yank(ctx)
        return
    if cheat in ("nothing", "nearest", "longest"):
        yield from _cheat_script(ctx, cheat)
        return
    assert cheat in ("", "off_table", "partly_off", "in_air"), cheat
    L, Rt = ctx.left, ctx.right
    hi = TOP + 0.10
    over = 0.010
    ry = -math.pi / 2  # right grip: fingers close along x, across a cable running along y

    # 1) left arm to its hold spot; right arm out of the camera's way
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=-0.0225, duration=1.2),
                        Rt.move(PARK_R, yaw=ry, finger=-0.014, duration=1.2))
    yield from ctx.both(L.move((PRESS[0], PRESS[1], PRESS_Z), yaw=math.pi / 2, duration=0.8), Rt.wait(0.8))
    _log(ctx, "held")

    # 2) find the blue plug in the image, grab it, pull up/right/towards the robot
    pp = yield from _seen(ctx, see_blue_plug)
    _debug_truth(ctx, "plug", pp)
    if pp is None:
        print("[demo] blue plug not seen", flush=True)
        return
    yield from Rt.move((pp[0], pp[1], hi), yaw=ry, duration=1.0)
    yield from Rt.move((pp[0], pp[1], TOP + PLUG.size[2] / 2 + over), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((pp[0], pp[1] - 0.02, TOP + 0.04), yaw=ry, duration=0.5)
    yield from Rt.move(PULL_TO[0], yaw=ry, duration=2.0)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], TOP + 0.025), yaw=ry, duration=0.6)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], hi), yaw=ry, duration=0.5)

    for k in range(5):
        yield from Rt.move(PARK_R, yaw=ry, finger=-0.014, duration=0.8)
        yield from Rt.wait(0.3)
        _log(ctx, f"look {k + 1}")
        over_px, seen = see_blue_over_tray(ctx)
        if seen >= 20 and over_px <= 2:
            break
        g = yield from _seen(ctx, see_regrasp)
        _debug_truth(ctx, "regrasp", g)
        if g is None:
            print("[demo] no regrasp point seen", flush=True)
            break
        p, h = g
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], hi), yaw=gy, duration=1.0)
        yield from Rt.move((p[0], p[1], CABLE_Z + over), yaw=gy, duration=0.7)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1], TOP + 0.05), yaw=gy, duration=0.4)
        tgt = PULL_TO[1 + k % 2]
        yield from Rt.move(tgt, yaw=ry, duration=1.8)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.025), yaw=ry, duration=0.6)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((tgt[0], tgt[1], hi), yaw=ry, duration=0.5)

    # 3) let go of the live cables; then lay the blue cable flat on clear table, off any cable or plug it crosses:
    # pick it up near its free end, carry it (plug end trailing) to a line on the robot's side of the tray and the
    # grey plugs, lower it there and let go
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=FINGER_OPEN, duration=0.8),
                        Rt.move(PARK_R, yaw=ry, finger=-0.014, duration=0.8))
    yield from Rt.wait(0.3)
    _log(ctx, "before lay")
    g = None if os.environ.get("BUNDLE_CHEAT_NOLAY") else (yield from _seen(ctx, see_tail_grasp))
    _debug_truth(ctx, "tail", g)
    if g is None:
        print("[demo] no tail grasp point seen", flush=True)
    else:
        p, h, t = g
        gy = _grip_yaw(h)
        lift = TOP + 0.13
        y_far = PLUG_PRIOR[1] + (BLUE_LEN - t) + 0.02  # the cable from the grasp to the plug, taut along LAY_X
        yield from Rt.move((p[0], p[1], hi), yaw=gy, duration=1.0)
        yield from Rt.move((p[0], p[1], CABLE_Z + over), yaw=gy, duration=0.7)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1], lift), yaw=gy, duration=0.8)
        yield from Rt.move((LAY_X, p[1], lift), yaw=ry, duration=1.2)
        yield from Rt.move((LAY_X, y_far, lift), yaw=ry, duration=1.5)
        # lower while moving towards the robot by the free end's length, so the free end is laid out sideways
        yield from Rt.move((LAY_X - t - 0.02, y_far, TOP + 0.015), yaw=ry, duration=1.2)
        if cheat:  # test-only shortcut continuing from the laid-out cable (see _cheat_after_lay)
            yield from _cheat_after_lay(ctx, cheat)
            return
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((LAY_X - t - 0.02, y_far, hi), yaw=ry, duration=0.6)
        _log(ctx, "laid")
    yield from Rt.move(PARK_END, yaw=ry, duration=0.8)
    _log(ctx, "released")
    yield from L.wait(1.0)
    _log(ctx, "end")


# -- shortcut variants (test only; they may read the simulator, they only have to score 0) --------------------------


def _poly(objects, name) -> tuple[np.ndarray, np.ndarray]:
    pts = _dense(objects[name].polyline(), 0.002)
    return pts, np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _regrasp_point(objects, name: str) -> tuple[np.ndarray, float] | None:
    """A point on cable ``name`` outside the tray (>= 3 cm beyond its right end), >= 3 cm from every other cable and
    plug, lying low; the one closest to the tray along the cable. Returns (point, cable heading) or None."""
    pts, s = _poly(objects, name)
    others = np.vstack([_pts(objects, n) for n in NAMES if n != name])
    for j in range(len(pts)):
        p = pts[j]
        if p[1] > TY0 - 0.03 or p[2] > TOP + 0.02:
            continue
        if float(np.linalg.norm(others[:, :2] - p[:2], axis=1).min()) < 0.03:
            continue
        if s[j] > s[-1] - 0.03:  # keep off the plug
            continue
        d = pts[min(j + 4, len(pts) - 1)] - pts[max(j - 4, 0)]
        return p, math.atan2(d[1], d[0])
    return None


def _cheat_after_lay(ctx, cheat):
    """off_table / partly_off / in_air, starting from the demo's laid-out cable (right hand still holding it near the
    free end): the right hand opens and, so that the laid-out state (which scores 1.0) does not end the episode, keeps
    within 2.5 cm of the cable while it follows the cable (read from the simulator) to its plug; it takes the plug and
    drags it off the table's right edge (y -0.65) to y -0.80 and drops it / leaves it just past the edge (centre
    y -0.672, the rest of the cable on the table) / holds it up 12 cm above the table and keeps holding it."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    Rt = ctx.right
    hi, over, ry, skim = TOP + 0.10, 0.010, -math.pi / 2, TOP + 0.025
    yield from Rt.grip(FINGER_OPEN, 0.4)
    cur = Rt.tcp()
    yield from Rt.move((cur[0], cur[1], skim), yaw=ry, duration=0.3)
    pts = _dense(ctx.states()["blue"].polyline(), 0.03)
    k = int(np.argmin(np.linalg.norm(pts[:, :2] - cur[:2], axis=1)))
    for q in pts[k + 1:-1]:
        yield from Rt.move((q[0], q[1], skim), yaw=ry, duration=0.2)
    st = ctx.states()["blue"]
    pp, ed = st.end_plug, st.end_dir
    gy = _grip_yaw(math.atan2(ed[1], ed[0]))
    yield from Rt.move((pp[0], pp[1], skim), yaw=gy, duration=0.4)
    yield from Rt.move((pp[0], pp[1], pp[2] + over), yaw=gy, duration=0.4)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((pp[0], pp[1], TOP + 0.06), yaw=gy, duration=0.5)
    if cheat == "in_air":
        yield from Rt.move((pp[0], pp[1], TOP + 0.12), yaw=gy, duration=0.8)
        _log(ctx, "holding up")
        yield from Rt.wait(2.0)
        _log(ctx, "end")
        return
    dst = (0.15, -0.80) if cheat == "off_table" else (0.15, -0.672)
    yield from Rt.move((dst[0], dst[1], TOP + 0.06), yaw=ry, duration=2.0)
    yield from Rt.move((dst[0], dst[1], TOP + 0.02), yaw=ry, duration=0.6)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((dst[0], dst[1] + 0.05, hi), yaw=ry, duration=0.6)
    _log(ctx, "placed")
    yield from Rt.move(PARK_END, yaw=ry, duration=0.8)
    _log(ctx, "released")
    yield from Rt.wait(1.0)
    _log(ctx, "end")


def _cheat_yank(ctx):
    """yank: the careless way. No hand holds the live cables; the right hand takes the blue plug (read from the
    simulator) and pulls it straight towards the robot, across the end of the tray, until the blue cable is out, then
    lets go. Pulled round the tray's open end, the blue cable sweeps across grey A where it leaves the tray."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    Rt = ctx.right
    hi, over, ry = TOP + 0.10, 0.010, -math.pi / 2
    _log(ctx, "start")
    pp = ctx.states()["blue"].end_plug
    yield from Rt.move((pp[0], pp[1], hi), yaw=ry, finger=-0.014, duration=1.2)
    yield from Rt.move((pp[0], pp[1], pp[2] + over), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((pp[0], pp[1], TOP + 0.03), yaw=ry, duration=0.4)
    yield from Rt.move((0.02, pp[1], TOP + 0.03), yaw=ry, duration=3.0)
    yield from Rt.move((0.02, pp[1] - 0.15, TOP + 0.03), yaw=ry, duration=2.0)
    _log(ctx, "pulled")
    yield from Rt.move((0.02, pp[1] - 0.15, TOP + 0.015), yaw=ry, duration=0.4)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move(PARK_END, yaw=ry, duration=0.8)
    _log(ctx, "released")
    yield from Rt.wait(1.0)
    _log(ctx, "end")


def _cheat_partial(ctx, cheat):
    """partial: hold the live cables down as the demo does, grasp the blue plug (read from the simulator) and pull it
    straight out along its row until the blue cable has left both clips but its free end still lies in the tray, lay
    it down and let go (part in the tray on its floor, the rest on the table between the grey plugs: 0.5).
    propped: the same pull, but instead of letting go the hand carries the plug back over the tray and sets it down
    across grey cables B and C inside the tray (right of the right clip), long side across them, and lets go: the plug
    rests on the live cables, not on the tray (must score 0)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    hi, over, ry = TOP + 0.10, 0.010, -math.pi / 2
    _log(ctx, "start")
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=-0.0225, duration=1.2),
                        Rt.move(PARK_R, yaw=ry, finger=-0.014, duration=1.2))
    yield from ctx.both(L.move((PRESS[0], PRESS[1], PRESS_Z), yaw=math.pi / 2, duration=0.8), Rt.wait(0.8))
    pp = ctx.states()["blue"].end_plug
    yield from Rt.move((pp[0], pp[1], hi), yaw=ry, duration=1.0)
    yield from Rt.move((pp[0], pp[1], pp[2] + over), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    x, y = ROW_X["blue"], pp[1]
    yield from Rt.move((x, y, TOP + 0.02), yaw=ry, duration=0.4)
    for _ in range(4):  # 21 cm, then 2 cm more at a time until the free end has left the right clip
        y -= 0.21 if y == pp[1] else 0.02
        yield from Rt.move((x, y, TOP + 0.02), yaw=ry, duration=3.0 if y < pp[1] - 0.1 else 0.6)
        if blue_unclipped(ctx.states()):
            break
    _log(ctx, "pulled")
    if cheat == "propped":
        yield from Rt.move((x, y, TOP + 0.08), yaw=ry, duration=0.6)
        yield from Rt.move((0.31, -0.165, TOP + 0.08), yaw=0.0, duration=2.0)
        yield from Rt.move((0.31, -0.165, FLOOR + 2 * R + PH[1] + over + 0.002), yaw=0.0, duration=1.0)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((0.31, -0.165, hi), yaw=0.0, duration=0.6)
    else:
        yield from Rt.move((x, y, TOP + 0.015), yaw=ry, duration=0.5)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((x, y, hi), yaw=ry, duration=0.6)
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=FINGER_OPEN, duration=0.8),
                        Rt.move(PARK_END, yaw=ry, duration=0.8))
    _log(ctx, "released")
    yield from L.wait(1.0)
    _log(ctx, "end")


def _cheat_script(ctx, cheat):
    """nothing: wait. nearest/longest: pull the cable nearest the robot (grey A) / with the plug sticking out
    furthest (grey C) exactly the way the demo pulls blue (positions read from the simulator: a shortcut only has to
    show what it scores)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    target = {"nearest": "grey_a", "longest": "grey_c"}.get(cheat, "blue")
    hi = TOP + 0.10
    over = 0.010
    ry = -math.pi / 2
    _log(ctx, "start")
    if cheat == "nothing":
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=-0.0225, duration=1.2)
    yield from L.move((PRESS[0], PRESS[1], PRESS_Z), yaw=math.pi / 2, duration=0.8)
    pp = ctx.states()[target].end_plug
    yield from Rt.move((pp[0], pp[1], hi), yaw=ry, finger=-0.014, duration=1.2)
    pp = ctx.states()[target].end_plug
    yield from Rt.move((pp[0], pp[1], pp[2] + over), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((pp[0], pp[1] - 0.02, TOP + 0.04), yaw=ry, duration=0.5)
    yield from Rt.move(PULL_TO[0], yaw=ry, duration=2.0)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], TOP + 0.025), yaw=ry, duration=0.6)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], hi), yaw=ry, duration=0.5)
    _log(ctx, "pull 1")
    for k in range(5):
        g = _regrasp_point(ctx.states(), target)
        if g is None:
            print("[demo] no regrasp point", flush=True)
            break
        p, h = g
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=-0.014, duration=1.0)
        yield from Rt.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.7)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1], TOP + 0.05), yaw=gy, duration=0.4)
        tgt = PULL_TO[1 + k % 2]
        yield from Rt.move(tgt, yaw=ry, duration=1.8)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.025), yaw=ry, duration=0.6)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((tgt[0], tgt[1], hi), yaw=ry, duration=0.5)
        _log(ctx, f"pull {k + 2}")
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=FINGER_OPEN, duration=0.8),
                        Rt.move((0.18, -0.35, hi), yaw=ry, duration=0.8))
    _log(ctx, "released")
    yield from L.wait(1.0)
    _log(ctx, "end")
