"""Operating-room bench (no patient) on the station table. On the left half of the table stands a raised sterile
field: a 5 cm tall box covered with a blue drape (48 x 52 cm), whose right-hand edge runs along the table's midline
(y = 0) in front of the robot. A white sterile camera cord (8 mm, 47 cm) leaves a white holster fixed near the
field's front-right corner and lies straight across the drape; it ends in a black 70 x 16 x 14 mm plug. A grey cord
holder sits on a line running from the field edge diagonally (-60 deg) towards a grey console (camera control unit)
on the right half of the table: an open-top slot on the drape (14 mm wide, 30 mm walls, 8.5-12.5 cm in from the edge)
and a flat 6 cm wide tray at drape height from 2.5 cm inside the edge to the console. The console's jack (a 24 x 21 mm
opening, 30 mm deep) faces the holder, 15 cm beyond the edge.

The left arm is the scrubbed (sterile) team member, the right arm the non-sterile circulator. Contamination is
followed like a tracer: a material point of the cord or plug becomes non-sterile when it touches anything non-sterile
(anything outside the drape's footprint at or below drape height, i.e. the tray past the edge, the drape's hanging
sides and the table; the drape's outer inch along its edges; the console; the right arm) or non-sterile cord. Score 0
if, at any step: any part of the right arm crosses the edge plane; the arms touch; any part of the left arm (base
excepted) is at or below drape height anywhere but over the drape's sterile zone, or touches the console; the left
arm touches non-sterile cord or plug; or non-sterile cord or plug touches the drape top more than 3 cm in from its
edges (past the outer inch and a 5 mm tape line), the slot walls or the holster. Otherwise, with the cord in the slot
at the end: 1.0 when the plug is at least 15 mm into the jack, lined up, and both grippers are clear of cord and
plug; 0.5 when the whole plug is past the edge and the left gripper is clear (passed off the field); else 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var STERILE_CHEAT): nothing | pass_only | reach_in | lean_over | glove_touch | tray_set |
regrip | noclip | hold.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Plug the white cord into the grey console's jack, with the cord lying in the grey holder's slot. The "
               "left arm and the blue drape are sterile, the right arm is not. Let go with both arms at the end.")

# -- colours ----------------------------------------------------------------------------------------------------------
DRAPE_BLUE = (0.16, 0.42, 0.68)
WHITE = (0.92, 0.92, 0.90)
HOLDER_GREY = (0.62, 0.64, 0.66)
CONSOLE_GREY = (0.78, 0.79, 0.80)
JACK_DARK = (0.22, 0.23, 0.25)
PLUG_BLACK = (0.07, 0.07, 0.08)

# -- sterile field: a draped raised box, right edge on y = 0 ------------------------------------------------------------
FIELD_H = 0.05
FT = TOP + FIELD_H  # drape top (sterile surface)
FIELD_X0, FIELD_X1 = 0.08, 0.56
FIELD_Y0, FIELD_Y1 = 0.0, 0.52  # y0 = the field's right edge (the boundary between the two arms' zones)
FIELD = Box("field", pos=((FIELD_X0 + FIELD_X1) / 2, (FIELD_Y0 + FIELD_Y1) / 2, TOP + FIELD_H / 2),
            size=(FIELD_X1 - FIELD_X0, FIELD_Y1 - FIELD_Y0, FIELD_H), color=DRAPE_BLUE)

# -- hand-off axis: holder slot -> tray -> jack, all on one line through E at heading -60 deg ----------------------------
AX_YAW = math.radians(-60.0)
U = np.array([math.cos(AX_YAW), math.sin(AX_YAW)])  # along the axis, off the field towards the console
NRM = np.array([-math.sin(AX_YAW), math.cos(AX_YAW)])  # sideways (left of the axis)
E = np.array([0.30, 0.0])  # where the axis crosses the field edge


def ax(d: float, b: float = 0.0) -> np.ndarray:
    """World xy of the point d m along the hand-off axis from the field edge (positive = off the field), b sideways."""
    return E + U * d + NRM * b


def ax_coords(xy) -> tuple[float, float]:
    v = np.asarray(xy[:2], dtype=float) - E
    return float(v @ U), float(v @ NRM)


def _abox(name, d0, d1, b, w, z0, z1, color):
    """Box aligned with the hand-off axis: from d0 to d1 along it, centred b sideways, w wide, z0..z1 high."""
    c = ax((d0 + d1) / 2, b)
    return Box(name, pos=(float(c[0]), float(c[1]), (z0 + z1) / 2), size=(d1 - d0, w, z1 - z0), yaw=AX_YAW,
               color=color)


# cord holder: open-top U-slot on the drape (sterile, part of the field) + flat tray at drape height that runs from just
# inside the edge out to the console (non-sterile beyond the edge line)
SLOT_D0, SLOT_D1 = -0.125, -0.085  # slot extent along the axis (on the field)
SLOT_W, SLOT_WALL, SLOT_H = 0.014, 0.004, 0.030
LEDGE_D0, LEDGE_D1, LEDGE_W, LEDGE_T = -0.025, 0.145, 0.060, 0.010
HOLDER = [
    _abox("slot_wall_l", SLOT_D0, SLOT_D1, (SLOT_W + SLOT_WALL) / 2, SLOT_WALL, FT, FT + SLOT_H, HOLDER_GREY),
    _abox("slot_wall_r", SLOT_D0, SLOT_D1, -(SLOT_W + SLOT_WALL) / 2, SLOT_WALL, FT, FT + SLOT_H, HOLDER_GREY),
    _abox("ledge", LEDGE_D0, LEDGE_D1, 0.0, LEDGE_W, FT - LEDGE_T, FT, HOLDER_GREY),
]

# console (camera control unit) with a jack facing the holder
PL, PW, PH = 0.070, 0.016, 0.014  # plug: length along the cord, width, height
CAV_W, CAV_H, CAV_D = PW + 0.008, PH + 0.007, 0.030  # jack opening 24 x 21 mm, 30 mm deep
JACK_D = 0.15  # jack face, m along the axis from the field edge
SLEEVE_LEN, SLEEVE_T = 0.040, 0.004
JACK_ZC = FT + PH / 2 + 0.002  # cavity centre height (= plug centre carried 2 mm above the tray)
CAV_Z0 = JACK_ZC - CAV_H / 2
CONSOLE_LEN, CONSOLE_W, CONSOLE_TOP = 0.12, 0.16, FT + 0.025
_SW = CAV_W + 2 * SLEEVE_T
CONSOLE = [
    _abox("console_body", JACK_D + SLEEVE_LEN, JACK_D + SLEEVE_LEN + CONSOLE_LEN, 0.0, CONSOLE_W, TOP, CONSOLE_TOP,
          CONSOLE_GREY),
    _abox("jack_base", JACK_D, JACK_D + SLEEVE_LEN, 0.0, _SW, TOP, CAV_Z0, CONSOLE_GREY),
    _abox("jack_wall_l", JACK_D, JACK_D + CAV_D, (CAV_W + SLEEVE_T) / 2, SLEEVE_T, CAV_Z0, CAV_Z0 + CAV_H, JACK_DARK),
    _abox("jack_wall_r", JACK_D, JACK_D + CAV_D, -(CAV_W + SLEEVE_T) / 2, SLEEVE_T, CAV_Z0, CAV_Z0 + CAV_H, JACK_DARK),
    _abox("jack_roof", JACK_D, JACK_D + CAV_D, 0.0, _SW, CAV_Z0 + CAV_H, CAV_Z0 + CAV_H + SLEEVE_T, JACK_DARK),
    _abox("jack_back", JACK_D + CAV_D, JACK_D + SLEEVE_LEN, 0.0, _SW, CAV_Z0, CAV_Z0 + CAV_H + SLEEVE_T, JACK_DARK),
]

# sterile camera cord: device end fixed in a holster on the field, plug on the free end
R = 0.004  # 8 mm cord
CORD_LEN = 0.47
A = np.array([0.14, 0.06])  # cord leaves the holster here
CORD_YAW = math.radians(48.0)
_DIR = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW)])
PLUG = Plug(size=(PL, PW, PH), mass=0.025, color=PLUG_BLACK)
CORD = cable("cord", start=(float(A[0]), float(A[1]), FT + PH / 2 + 0.0005), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=PLUG, color=WHITE, cable_mass=0.060, bend_stiffness=0.005, bend_damping=0.001)
HOLSTER_SIZE = (0.050, 0.035, 0.030)
_hc = A - _DIR * (0.006 + HOLSTER_SIZE[0] / 2)
HOLSTER = Box("holster", pos=(float(_hc[0]), float(_hc[1]), FT + HOLSTER_SIZE[2] / 2), size=HOLSTER_SIZE,
              yaw=CORD_YAW, color=WHITE)

OBJECTS = [FIELD, *HOLDER, *CONSOLE, HOLSTER, CORD]
TWIN_EXTRA: list = []  # everything is in OBJECTS (the cord's device end is glued into the holster)

# -- robot geometry: the collision SURFACES of every link, read once from the arm's USD asset ---------------------------
SURF_H = 0.0012  # surface sample spacing (m): every face of every convex collision piece gets a triangular grid this fine
SURF_COVER = 0.001  # every collision surface point lies within this of a sample (h/sqrt(3) = 0.69 mm + 0.26 mm thinning)
_HULLS: dict = {}


def _face_samples(T: np.ndarray) -> np.ndarray:
    """Points covering triangles T (M, 3, 3): a barycentric grid with all sub-triangle edges <= SURF_H (corners,
    edges and interior of every face)."""
    out = []
    e = np.max(np.linalg.norm(T[:, [1, 2, 2]] - T[:, [0, 0, 1]], axis=2), axis=1)
    n = np.maximum(1, np.ceil(e / SURF_H)).astype(int)
    for k in np.unique(n):
        t = T[n == k]
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1))
        m = (i + j) <= k
        a, b = (i[m] / k)[None, :, None], (j[m] / k)[None, :, None]
        out.append((t[:, None, 0] + a * (t[:, None, 1] - t[:, None, 0]) + b * (t[:, None, 2] - t[:, None, 0])).reshape(-1, 3))
    return np.vstack(out)


def _load_hulls() -> dict:
    """{body name: (S, V, centre, radius)} for every rigid body of the YAM arm (base, link1-5, gripper, both fingers), in that body's
    frame. V: the vertices of its convex collision pieces. S: points covering the whole SURFACE of those pieces, i.e.
    every face of every piece sampled on a grid with spacing <= 1.2 mm (then thinned to one per 0.15 mm cell), so every surface point is within SURF_COVER of
    a sample (corners and edges included). Samples more than 1 mm inside another piece of the same body are dropped:
    they are not on the body's outer surface, and no outer surface point is within SURF_COVER of a dropped sample.
    centre, radius: a sphere round S (to skip bodies far from what is being tested)."""
    if _HULLS:
        return _HULLS
    import yam_arena
    from pxr import Gf, Usd, UsdGeom
    from scipy.spatial import ConvexHull

    path = pathlib.Path(yam_arena.__file__).parent / "assets/yam/usd/yam_arm/yam_arm.usda"
    stage = Usd.Stage.Open(str(path))
    xc = UsdGeom.XformCache()
    bodies = [p for p in stage.Traverse() if "PhysicsRigidBodyAPI" in p.GetAppliedSchemas()]
    for b in bodies:
        inv = xc.GetLocalToWorldTransform(b).GetInverse()
        verts, samples, eqs = [], [], []
        for p in Usd.PrimRange(b, Usd.TraverseInstanceProxies()):
            q, other = p, False
            while q != b:
                if "PhysicsRigidBodyAPI" in q.GetAppliedSchemas():
                    other = True
                    break
                q = q.GetParent()
            if other or not p.IsA(UsdGeom.Mesh) or "PhysicsCollisionAPI" not in p.GetAppliedSchemas():
                continue
            m = xc.GetLocalToWorldTransform(p) * inv
            v = np.array([list(m.Transform(Gf.Vec3d(*map(float, x)))) for x in UsdGeom.Mesh(p).GetPointsAttr().Get()])
            h = ConvexHull(v)  # a flat (degenerate) piece raises: the rule could not be checked
            verts.append(v[h.vertices])
            samples.append(_face_samples(v[h.simplices]))
            eqs.append((v.min(axis=0), v.max(axis=0), h.equations))
        if not samples:
            continue
        S = np.vstack(samples)
        S = S[np.sort(np.unique(np.floor(S / 1.5e-4).astype(np.int64), axis=0, return_index=True)[1])]  # 0.26 mm cells
        deep = np.zeros(len(S), dtype=bool)
        for lo, hi, eq in eqs:
            k = np.nonzero(np.all((S >= lo + 0.001) & (S <= hi - 0.001), axis=1))[0]
            if len(k):
                deep[k[(S[k] @ eq[:, :3].T + eq[:, 3]).max(axis=1) <= -0.001]] = True
        S = S[~deep]
        c = (S.min(axis=0) + S.max(axis=0)) / 2
        _HULLS[b.GetName()] = (S, np.vstack(verts), c, float(np.linalg.norm(S - c, axis=1).max()))
    if len(_HULLS) < 9:
        raise RuntimeError(f"arm hulls: only {sorted(_HULLS)}")
    return _HULLS


def _quat_wxyz(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _t(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return (x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)).astype(np.float64)


def arm_frames(key: str) -> dict:
    """{link name: (rotation, position)} of every rigid body of one arm ("left_robot" or "right_robot") from the sim's
    live body poses (env-origin corrected). Raises when the sim is not available."""
    env = _ENV["env"]
    hulls = _load_hulls()
    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0]
    art = base.scene[key]
    names = list(art.body_names)
    pos = _t(art.data.body_pos_w)[0] - origin
    quat = _t(art.data.body_quat_w)[0]
    # quaternion layout check, every call: the gripper body's frame must put the TCP (134 mm along its z axis)
    # where the embodiment reports it; whichever layout (wxyz or xyzw) does so is used, neither = error
    from yam_arena.runtime import tcp_world

    tcp = np.asarray(tcp_world(art), dtype=np.float64) - origin
    gi = names.index("gripper")
    rot = None
    for conv in (lambda q: q, lambda q: np.r_[q[3], q[:3]]):
        if np.linalg.norm(pos[gi] + _quat_wxyz(conv(quat[gi]))[:, 2] * 0.134 - tcp) < 0.004:
            rot = conv
            break
    if rot is None:
        raise RuntimeError("arm body frames do not match the reported TCP")
    out = {}
    for n in hulls:
        i = names.index(n)  # a missing body raises: the rule could not be checked
        out[n] = (_quat_wxyz(rot(quat[i])), pos[i])
    return out


def body_points(frames: dict, n: str, verts: bool = False) -> np.ndarray:
    """World surface samples (verts=True: hull vertices) of body n."""
    m, p = frames[n]
    return _load_hulls()[n][1 if verts else 0] @ m.T + p


def in_field(p, margin: float = 0.0) -> np.ndarray:
    """Points over the drape's footprint, at any height above the table (the sterile field volume): x within the
    drape, y > FIELD_Y0 + margin."""
    p = np.atleast_2d(p)
    return (p[:, 0] >= FIELD_X0) & (p[:, 0] <= FIELD_X1) & (p[:, 1] > FIELD_Y0 + margin) & (p[:, 1] <= FIELD_Y1) & (
        p[:, 2] >= TOP)


def arm_in_field(p) -> np.ndarray:
    """Surface samples of the non-sterile arm within SURF_COVER of the field side of the edge plane (y > 0), at any
    x and any height above the table (so any surface point across the plane is caught)."""
    return (p[:, 1] > FIELD_Y0 - SURF_COVER) & (p[:, 2] >= TOP)


def glove_low(p) -> np.ndarray:
    """Surface samples of the sterile arm at or below drape height + 2 mm anywhere but over the drape top beyond its
    outer inch (on that inch, beside the drape box, on the tray beyond the edge, on the table)."""
    c = SURF_COVER
    return ~_inset(p, BORDER + c) & (p[:, 2] <= FT + TOUCH + c)


def _inset(p, m: float) -> np.ndarray:
    """Points over the drape's footprint at least ``m`` in from all four of its edges."""
    return (p[:, 0] >= FIELD_X0 + m) & (p[:, 0] <= FIELD_X1 - m) & (p[:, 1] >= FIELD_Y0 + m) & (p[:, 1] <= FIELD_Y1 - m)


def off_footprint(p) -> np.ndarray:
    return ~((p[:, 0] >= FIELD_X0) & (p[:, 0] <= FIELD_X1) & (p[:, 1] >= FIELD_Y0) & (p[:, 1] <= FIELD_Y1))


# -- per-step monitor (latched) -------------------------------------------------------------------------------------------
TOUCH = 0.002  # a surface this close to another = touching it
BORDER = 0.025  # the drape's outer inch, along all four edges, is not sterile (contaminates like the tray)
TAPE = 0.005  # a 5 mm tape line inside it is neither; the sterile zone starts 3 cm in from every edge
_ENV: dict = {"env": None}
_MON: dict = {}
_CORD_SUB = 4  # cord material points per segment gap (5 mm apart)
_PLUG_SAMPLES = np.linspace(-0.5, 0.5, 15)  # along the plug (every 5 mm), as fractions of its length
GLOVE = ("gripper", "tip_left", "tip_right")  # the scrub arm's "gloved hand": gripper housing and both fingers
ARMS_GAP = 0.002  # right-arm surface this close to any left-arm surface = the arms touch (as "touch" everywhere)
_HALF = np.array([PL / 2, PW / 2, PH / 2])


def _plug_surface() -> np.ndarray:
    """Points covering the plug box's surface (plug frame, <= 2 mm apart)."""
    out = []
    for ax_i in range(3):
        o = [i for i in range(3) if i != ax_i]
        g = [np.linspace(-_HALF[k], _HALF[k], max(2, int(np.ceil(2 * _HALF[k] / 0.002)) + 1)) for k in o]
        u, v = np.meshgrid(*g)
        for sgn in (-1, 1):
            q = np.zeros((u.size, 3))
            q[:, o[0]], q[:, o[1]], q[:, ax_i] = u.ravel(), v.ravel(), sgn * _HALF[ax_i]
            out.append(q)
    return np.vstack(out)


_PLUG_SURF = _plug_surface()


def _plug_idx(x) -> np.ndarray:
    """Index of the plug material point (along its length) nearest plug-frame position(s) x."""
    return np.clip(np.rint((np.asarray(x) / PL + 0.5) * (len(_PLUG_SAMPLES) - 1)), 0, len(_PLUG_SAMPLES) - 1).astype(int)


_PLUG_SURF_IDX = _plug_idx(_PLUG_SURF[:, 0])


def _reset_monitor() -> None:
    _MON.update(err=False, events=[], steps=0, right_in=False, right_link=None, arms_touch=False, min_arms_gap=9.0,
                max_right_y=-9.0, glove_low=False, glove_part=None, left_dirty=False, field_dirty=False, dirty=None,
                n_dirty=0, left_touch=False, right_touch=False, min_glove_z=9.0, dsrc=None)


_reset_monitor()


def grippers() -> list[np.ndarray]:
    """World TCPs (between the finger pads, 13 mm above the pad tips) of the left and right gripper; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0]
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def plug_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    """(plug centre, unit axis from the cord through the plug tip)."""
    c = objects[CORD.name]
    p = np.asarray(c.end_plug, dtype=float)
    a = p - np.asarray(c.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def plug_frame(objects) -> tuple[np.ndarray, np.ndarray]:
    """(rotation, centre) of the connector box from the sim's live body pose: rotation columns are its length (cord to
    tip), width and height axes. The quaternion layout is the one whose length axis matches the cord-to-plug
    direction; neither = error."""
    env = _ENV["env"]
    base = env.unwrapped
    art = base.scene[CORD.name]
    i = list(art.body_names).index("end_plug")
    q = _t(art.data.body_quat_w)[0][i]
    p, a = plug_pose(objects)
    for conv in (lambda q: q, lambda q: np.r_[q[3], q[:3]]):
        m = _quat_wxyz(conv(q))
        if float(m[:, 0] @ a) > 0.97:
            return m, p
    raise RuntimeError("connector body frame does not match the cord direction")


def _box_dist(P, m, p) -> tuple[np.ndarray, np.ndarray]:
    """Distance of world points P to the connector box, and their (clamped) position along it (plug frame x)."""
    loc = (P - p) @ m
    return np.linalg.norm(np.maximum(np.abs(loc) - _HALF, 0.0), axis=1), np.clip(loc[:, 0], -PL / 2, PL / 2)


def _near_box(P, bx, margin: float) -> np.ndarray:
    """Points within ``margin`` of fixed box ``bx`` (any yaw)."""
    c, s = math.cos(bx.yaw), math.sin(bx.yaw)
    v = P - np.asarray(bx.pos)
    h = np.asarray(bx.size) / 2 + margin
    return (np.abs(v[:, 0] * c + v[:, 1] * s) <= h[0]) & (np.abs(-v[:, 0] * s + v[:, 1] * c) <= h[1]) & (
        np.abs(v[:, 2]) <= h[2])


def _near_any(P, boxes, margin: float) -> np.ndarray:
    hit = np.zeros(len(P), dtype=bool)
    for bx in boxes:
        hit |= _near_box(P, bx, margin)
    return hit


def _console_near(P, margin: float) -> np.ndarray:
    """Points within ``margin`` of a console box (console housing and jack)."""
    return _near_any(P, CONSOLE, margin)


STERILE_PROPS = [HOLDER[0], HOLDER[1], HOLSTER]  # slot walls and holster (on the field)


def sterile_zone(p) -> np.ndarray:
    """Points over the drape top's sterile zone: at least BORDER + TAPE in from every edge of the drape."""
    return _inset(np.atleast_2d(p), BORDER + TAPE)


def border(p) -> np.ndarray:
    """Points over the drape's outer inch (inside its footprint, less than BORDER from any of its four edges): not
    sterile."""
    p = np.atleast_2d(p)
    return ~off_footprint(p) & ~_inset(p, BORDER)


def cord_samples(objects) -> tuple[np.ndarray, np.ndarray]:
    """Material points of the cord and plug, in a fixed order (index i is always the same piece of cord):
    (N, 3) centres and (N,) half-thickness below the centre. Cord: segment centres and 3 points between each pair;
    plug: 15 points along its axis (5 mm apart)."""
    c = objects[CORD.name]
    pts = np.asarray(c.points, dtype=float)
    f = np.arange(_CORD_SUB) / _CORD_SUB
    cord = (pts[:-1, None, :] * (1 - f)[None, :, None] + pts[1:, None, :] * f[None, :, None]).reshape(-1, 3)
    cord = np.vstack([cord, pts[-1:]])
    p, a = plug_pose(objects)
    plug = p[None, :] + _PLUG_SAMPLES[:, None] * PL * a[None, :]
    half = np.r_[np.full(len(cord), R), np.full(len(plug), PH / 2)]
    return np.vstack([cord, plug]), half


def _event(key: str, msg: str) -> None:
    if not _MON[key]:
        _MON[key] = True
        _MON["events"].append((msg, _MON["steps"]))


def _monitor(objects) -> None:
    """One control step of the rules (latched). Contamination is followed like a tracer: a material point of the cord
    or plug becomes non-sterile ("dirty") when it touches anything non-sterile (anything outside the drape's footprint
    at or below drape height: the tray past the edge, the drape's hanging sides, the table; the console; any part of
    the right arm) or, for the cord, another stretch of cord that is dirty."""
    from scipy.spatial import cKDTree

    g = grippers()
    if len(g) != 2:
        raise RuntimeError("gripper poses unavailable")
    m, p = plug_frame(objects)
    k = _MON["steps"]
    tb = TOUCH + SURF_COVER
    s, half = cord_samples(objects)
    nc = len(s) - len(_PLUG_SAMPLES)
    if _MON["dirty"] is None or len(_MON["dirty"]) != len(s):
        _MON["dirty"] = np.zeros(len(s), dtype=bool)
    D = _MON["dirty"]
    D0 = D.copy()
    cs = s[:nc]
    W = _PLUG_SURF @ m.T + p
    # (a) material touching a non-sterile surface becomes dirty
    D[:nc] |= ((off_footprint(cs) | border(cs)) & (cs[:, 2] - R <= FT + TOUCH)) | _console_near(cs, R + TOUCH)
    hw = ((off_footprint(W) | border(W)) & (W[:, 2] <= FT + TOUCH)) | _console_near(W, TOUCH)
    if hw.any():
        D[nc + np.unique(_PLUG_SURF_IDX[hw])] = True
    ctree = cKDTree(cs)
    cmin, cmax = cs.min(axis=0) - R, cs.max(axis=0) + R
    # sterile arm: never at or below drape height outside the drape's footprint, never on the console
    lf = arm_frames("left_robot")
    G = {n: body_points(lf, n) for n in GLOVE}
    gp = np.vstack(list(G.values()))
    _MON["min_glove_z"] = min(_MON["min_glove_z"], float(gp[:, 2].min()))
    H = _load_hulls()
    lsph = {n: (lf[n][0] @ H[n][2] + lf[n][1], H[n][3]) for n in lf}
    lpts = dict(G)
    for ln in lf:
        if ln == "base":  # stands on the table beside the field
            continue
        lw, lr = lsph[ln]
        near_cord = bool(np.all(lw + lr >= cmin - 0.01) and np.all(lw - lr <= cmax + 0.01))
        near_plug = float(np.linalg.norm(lw - p)) <= lr + PL / 2 + 0.01
        if ln not in lpts and (lw[2] - lr <= FT + 0.01 or near_cord or near_plug):
            lpts[ln] = body_points(lf, ln)
        if ln in lpts and (glove_low(lpts[ln]) | _console_near(lpts[ln], tb)).any() and not _MON["glove_low"]:
            _MON["glove_part"] = ln
            _event("glove_low", f"left arm ({ln}) at or below drape height outside the field, or on the console")
    # non-sterile arm: no point of any link across the edge plane; never touching the left arm; what it touches
    # becomes dirty
    rf = arm_frames("right_robot")
    bound = ARMS_GAP + 2 * SURF_COVER
    right_touch = False
    for n in rf:
        _, _, cc, r = H[n]
        w = rf[n][0] @ cc + rf[n][1]
        far = w[1] + r < FIELD_Y0 - SURF_COVER - 0.05  # bounding sphere more than 5 cm off the plane
        near_plug = float(np.linalg.norm(w - p)) <= r + PL / 2 + 0.02
        near_cord = bool(np.all(w + r >= cmin - 0.01) and np.all(w - r <= cmax + 0.01))
        near_left = [ln for ln, (lw, lr) in lsph.items() if float(np.linalg.norm(w - lw)) <= r + lr + 0.02]
        if far and not near_plug and not near_left and not near_cord:
            continue
        P = body_points(rf, n)
        _MON["max_right_y"] = max(_MON["max_right_y"], float(P[:, 1].max()))
        if arm_in_field(P).any() and not _MON["right_in"]:
            _MON["right_link"] = n
            _event("right_in", f"right arm ({n}) across the field edge")
        for ln in near_left:
            if ln not in lpts:
                lpts[ln] = body_points(lf, ln)
            L_ = lpts[ln]
            lo, hi_ = L_.min(axis=0) - 0.02, L_.max(axis=0) + 0.02
            Q = P[np.all((P >= lo) & (P <= hi_), axis=1)]
            if len(Q):  # (gaps up to 2 cm are logged)
                dd = float(cKDTree(L_).query(Q, distance_upper_bound=0.02)[0].min())
                _MON["min_arms_gap"] = min(_MON["min_arms_gap"], dd)
                if dd <= bound:
                    _event("arms_touch", f"right arm ({n}) touched the left arm ({ln})")
        if near_cord:
            dd, ii = ctree.query(P, distance_upper_bound=R + tb)
            hit = ii[np.isfinite(dd)]
            if len(hit):
                D[np.unique(hit)] = True
                right_touch = True
        if near_plug:
            dd, x = _box_dist(P, m, p)
            t = dd <= tb
            if t.any():
                D[nc + np.unique(_plug_idx(x[t]))] = True
                right_touch = True
    # (b) dirty cord passes it on to cord it touches (stretches more than 4 cm apart along the cord)
    if D[:nc].any() and not D[:nc].all():
        pr = ctree.query_pairs(2 * R + TOUCH, output_type="ndarray")
        pr = pr[np.abs(pr[:, 0] - pr[:, 1]) > 2 * _CORD_SUB] if len(pr) else pr
        if len(pr):
            hit = D[pr[:, 0]] | D[pr[:, 1]]
            D[pr[hit].ravel()] = True
    _MON["n_dirty"] = int(D.sum())
    if _MON["dsrc"] is None or len(_MON["dsrc"]) != len(D):
        _MON["dsrc"] = [None] * len(D)
    for i in np.nonzero(D & ~D0)[0]:
        _MON["dsrc"][i] = (k, np.round(s[i], 3).tolist())
    # (c) the sterile arm touches nothing dirty (any link, base excepted)
    Lp = np.vstack([v for kk, v in lpts.items()])
    dd, x = _box_dist(Lp, m, p)
    t = dd <= tb
    left_touch = bool(t[: len(gp)].any())
    if t.any() and D[nc + _plug_idx(x[t])].any():
        _event("left_dirty", "left arm touched plug material that had touched something non-sterile")
    dd, ii = ctree.query(Lp, distance_upper_bound=R + tb)
    hit = ii[np.isfinite(dd)]
    if len(hit) and D[hit].any():
        _event("left_dirty", "left arm touched cord that had touched something non-sterile")
    _MON["left_touch"], _MON["right_touch"] = left_touch, right_touch
    # (d) dirty material never touches the sterile zone of the drape top, the slot walls or the holster
    dc = D[:nc]
    bad = dc & ((sterile_zone(cs) & (cs[:, 2] - R <= FT + TOUCH)) | _near_any(cs, STERILE_PROPS, R + TOUCH))
    dw = D[nc + _PLUG_SURF_IDX]
    bad_p = dw & ((sterile_zone(W) & (W[:, 2] <= FT + TOUCH)) | _near_any(W, STERILE_PROPS, TOUCH))
    if (bad.any() or bad_p.any()) and not _MON["field_dirty"]:
        bi = np.nonzero(bad)[0].tolist()
        print(f"[sterile] field_dirty at step {k}: cord points {bi} at {np.round(cs[bi], 3).tolist()} first dirty "
              f"{[_MON['dsrc'][i] for i in bi]}; plug points {np.unique(_PLUG_SURF_IDX[bad_p]).tolist()}", flush=True)
    if bad.any() or bad_p.any():
        _event("field_dirty", "cord or plug that had touched something non-sterile touched the sterile field")
    if k % 200 == 0:
        print(f"[sterile] step {k}: right arm max y {_MON['max_right_y']:.3f}, dirty points {_MON['n_dirty']}",
              flush=True)
    _MON["steps"] += 1


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_sterile_hooked", False):
        return
    base_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        try:
            from yam_arena.scene import object_states

            _monitor(object_states(self._env, [CORD.name]))
        except Exception as e:  # pragma: no cover
            _MON["err"] = True  # a step the rules could not be checked on never passes
            print(f"[sterile] monitor error: {e!r}", flush=True)
        return base_step(self, *a, **k)

    def reset(self, *a, **k):
        out = base_reset(self, *a, **k)
        _reset_monitor()  # arms start at READY (left over the field, right off it), plug on the drape
        return out

    _task_step._sterile_hooked = True
    reset._sterile_hooked = True
    cls._task_step = _task_step
    cls.reset = reset


_install_hooks()

# -- scoring ----------------------------------------------------------------------------------------------------------
SEATED_MIN = 0.015  # plug tip at least 15 mm inside the jack
RELEASE_PLUG = 0.045  # let go = gripper TCP at least this far from the plug's centre line, end to end,
RELEASE_CORD = 0.03  # and at least this far from every point of the cord

_RULES = (("right_in", "right_in"), ("arms_touch", "arms_touch"), ("glove_low", "glove_low"),
          ("left_dirty", "left_dirty"), ("field_dirty", "field_dirty"))


def violations() -> list[str]:
    out = []
    if _MON["err"] or _MON["steps"] == 0:
        out.append("monitor_error")  # no step was checked, or one could not be: never a pass
    out += [name for key, name in _RULES if _MON[key]]
    return out


def jack_depth(objects) -> float:
    """How far (m) the plug tip is inside the jack; -1 if the plug is not lined up with the opening (centre > 8 mm
    off the jack axis sideways or vertically, or pointing more than ~25 deg away from it)."""
    p, a = plug_pose(objects)
    if not np.isfinite(p).all():
        return -1.0
    d, b = ax_coords(p)
    if abs(b) > 0.008 or abs(p[2] - JACK_ZC) > 0.008 or float(a[:2] @ U) < 0.9:
        return -1.0
    return float(d + PL / 2 - JACK_D)


def cord_in_slot(objects) -> bool:
    """Some cord segment centre lies inside the holder slot (between its walls, below their tops)."""
    for q in objects[CORD.name].points:
        d, b = ax_coords(q)
        if SLOT_D0 - 0.002 <= d <= SLOT_D1 + 0.002 and abs(b) <= SLOT_W / 2 and FT <= q[2] <= FT + SLOT_H:
            return True
    return False


def passed_off(objects) -> bool:
    """The whole plug (its four corners in plan) is past the field-edge plane."""
    p, a = plug_pose(objects)
    a2 = a[:2] / (np.linalg.norm(a[:2]) + 1e-12)
    n2 = np.array([-a2[1], a2[0]])
    ys = [(p[:2] + a2 * sa * PL / 2 + n2 * sn * PW / 2)[1] for sa in (-1, 1) for sn in (-1, 1)]
    return bool(np.isfinite(ys).all() and max(ys) < FIELD_Y0)


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def released(objects, arms=(0, 1)) -> bool:
    """Each gripper centre in ``arms`` (0 left, 1 right) at least 4.5 cm from the plug's centre line and 3 cm from the
    cord; False when the gripper poses cannot be read."""
    g = grippers()
    if len(g) != 2:
        return False
    p, a = plug_pose(objects)
    rear, tip = p - a * PL / 2, p + a * PL / 2
    poly = np.asarray(objects[CORD.name].polyline(), dtype=float)
    for i in arms:
        x = g[i]
        if _seg_dist(x, rear, tip) < RELEASE_PLUG:
            return False
        if min(_seg_dist(x, u, v) for u, v in zip(poly[:-1], poly[1:])) < RELEASE_CORD:
            return False
    return True


def score(objects) -> float:
    """0 if, at any step (latched in _MON until reset()): any point of the collision surface of a right-arm link crossed
    the field-edge plane (y > 0, any height); any right-arm surface came within 2 mm of any left-arm surface; any
    left-arm surface (base excepted) was at or below drape height + 2 mm anywhere but over the drape's sterile zone
    (beyond its outer inch), or touched the console; the left arm touched cord or plug material that had touched something non-sterile; or such material
    touched the drape top more than 3 cm in from its edges, the slot walls or the holster. Contamination is followed
    per material point (cord every 5 mm, plug every 5 mm): a point becomes non-sterile when it touches anything outside
    the drape's footprint at or below drape height (tray past the edge, drape sides, table), the drape's outer inch
    along its edges, the console, any part of the right arm, or non-sterile cord.
    Otherwise, with the cord in the holder slot at the end: 1.0 if the plug tip is >= 15 mm inside the jack (lined up)
    and both gripper centres are >= 4.5 cm from the plug's centre line and 3 cm from the cord; 0.5 if the whole plug is
    past the edge plane and the left gripper centre is that far clear (passed off the field); else 0."""
    if violations() or not cord_in_slot(objects):
        return 0.0
    if jack_depth(objects) >= SEATED_MIN and released(objects):
        return 1.0
    if passed_off(objects) and released(objects, (0,)):
        return 0.5
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CORD.name].resting(max_speed=0.02)


def distance(objects) -> float:
    """Plug tip's remaining distance to 15 mm inside the jack (m)."""
    p, a = plug_pose(objects)
    tip = p[:2] + a[:2] * PL / 2
    goal = ax(JACK_D + SEATED_MIN)
    return float(np.linalg.norm(tip - goal))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    return {
        "cord_in_slot": int(cord_in_slot(objects)),
        "passed_off": int(passed_off(objects)),
        "plug_in_jack": round(float(np.clip(jack_depth(objects) / SEATED_MIN, 0.0, 1.0)), 3),
        "released": int(released(objects)),
        "rules_kept": int(not violations()),
    }


REAL_MARGIN = 0.005  # real-trial joint-log replay: every limit widened by 5 mm (model, calibration and cover errors)


def replay_joint_log(q_log, margin: float = REAL_MARGIN) -> dict:
    """Rules 1-3 for a REAL trial, from the robot's own joint log: ``q_log`` is a (T, 16) array, one row per control
    step, of the measured joint positions joint1-joint8 of the left arm then joint1-joint8 of the right arm (rad for
    joints 1-6, m for the finger joints 7-8), e.g. read from the controller's log with np.loadtxt. Each row places
    both arms' collision surfaces (the same sampled meshes the simulation scorer uses) by forward kinematics on the
    arm's URDF from the base positions of the station, and applies the simulation's tests widened by ``margin``:
    any right-arm point within ``margin`` of the edge plane; the arms within ARMS_GAP + ``margin``; any left-arm point
    (base excepted) at or below drape height + 2 mm + ``margin`` anywhere but over the sterile zone, or within
    ``margin`` of the console. No camera is involved. Returns the latched breaks and the closest approaches."""
    from scipy.spatial import cKDTree
    from yam_arena import YAM_URDF, station
    from yam_arena.constants import ALL_JOINTS
    from yam_arena.urdf import forward_kinematics

    H = _load_hulls()
    q_log = np.atleast_2d(np.asarray(q_log, dtype=float))
    if q_log.shape[1] != 2 * len(ALL_JOINTS):
        raise ValueError(f"joint log must have {2 * len(ALL_JOINTS)} columns, got {q_log.shape[1]}")
    out = dict(right_in=False, arms_touch=False, glove_low=False, first_step=None, min_plane_gap=9.0,
               min_arms_gap=9.0, min_left_low_z=9.0)
    bases = {a: np.asarray(station.BASES[a], dtype=float) for a in ("left", "right")}

    def surfaces(arm, q):
        fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ALL_JOINTS, q, strict=True)})
        return {n: H[n][0] @ fk[n][:3, :3].T + fk[n][:3, 3] + bases[arm] for n in H if n in fk}

    for k, row in enumerate(q_log):
        Ls, Rs = surfaces("left", row[:8]), surfaces("right", row[8:])
        R_ = np.vstack(list(Rs.values()))
        L_ = np.vstack([v for n, v in Ls.items() if n != "base"])
        gap = float(-(R_[:, 1].max() - FIELD_Y0))
        out["min_plane_gap"] = min(out["min_plane_gap"], gap)
        dd = float(cKDTree(L_).query(R_, distance_upper_bound=0.05)[0].min())
        out["min_arms_gap"] = min(out["min_arms_gap"], dd)
        low = glove_low(L_ - np.array([0.0, 0.0, margin]))  # widened: points up to ``margin`` higher count as low
        if low.any():
            out["min_left_low_z"] = min(out["min_left_low_z"], float(L_[low, 2].min() - FT))
        brk = {"right_in": gap <= SURF_COVER + margin, "arms_touch": dd <= ARMS_GAP + 2 * SURF_COVER + margin,
               "glove_low": bool(low.any() or _console_near(L_, TOUCH + SURF_COVER + margin).any())}
        for key, b in brk.items():
            if b and not out[key]:
                out[key] = True
                if out["first_step"] is None:
                    out["first_step"] = k
    out["score_ok"] = not (out["right_in"] or out["arms_touch"] or out["glove_low"])
    return out


VARIATION = {
    "cable:cord": {"dx": 0.015, "dy": 0.010, "yaw": 0.05},
}

# -- closed-loop scripted demo (cabledemo.py): camera images + the arms' own poses + the fixed layout only -------------
SCRUB_GRIP_BACK = 0.015  # left arm pinches the plug this far from its cord end
GRASP_DZ = 0.010  # TCP above the plug centre when gripping it
DROP_TCP_D = 0.060  # left TCP when it lets the plug go: the whole plug 4.5-11.5 cm past the edge, over the tray
DROP_H = 0.012  # plug underside this far above the tray when the left lets go
PULL_GRIP = 0.010  # right arm first pinches the plug this far from its tip ...
PULL_DIST = 0.022  # ... and draws it this far further out along the tray
SEAT_GRIP = 0.038  # then re-grips it this far from its tip (near its middle) for the push into the jack
SEAT_DEPTH = 0.022
CAM_Z = FT + PH  # the plug's top face (plane used to back-project its pixels)
CAM_BIAS = np.zeros(2)  # constant camera offset (filled after the offline check against the sim)


def _hd(v) -> float:
    return math.atan2(float(v[1]), float(v[0]))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _plug_pixels(ctx, z: float):
    """Top-camera pixels of the black plug, back-projected to height z; None in coarse
    debug renders."""
    import vision

    img = vision.top_image(ctx)
    if img.shape[1] < 160:  # debug runs without recording render 64x36 frames: too coarse, use the layout instead
        return None
    im = img.astype(int)
    dark = (im.max(axis=-1) < 45) & (im.max(axis=-1) - im.min(axis=-1) < 25)
    return vision.top_camera(img.shape).to_plane(dark, z)


def see_plug(ctx, near_xy, toward_xy, radius: float = 0.13, on_axis: bool = False):
    """Camera estimate of the connector lying flat near ``near_xy``: (centre xy, unit axis xy pointing to the end
    nearer ``toward_xy`` (its tip), length seen). Uses only the top camera image: connector pixels back-projected to
    the plug's top-face plane, kept within ``radius`` of ``near_xy``; ends from the 3rd/97th percentiles along the
    main axis."""
    pts = _plug_pixels(ctx, CAM_Z)
    if pts is None or len(pts) == 0:
        return None
    pts = pts[np.linalg.norm(pts - np.asarray(near_xy)[:2], axis=1) <= radius]
    if on_axis:  # on the tray: keep the strip along the hand-off line, short of the (dark) jack sleeve, and of it the
        # largest unbroken run along the line (shaded jack pixels beyond a gap are dropped)
        v = pts - E
        pts = pts[(np.abs(v @ NRM) <= 0.03) & (v @ U <= JACK_D - 0.001)]
        if len(pts) >= 10:
            s_ = (pts - E) @ U
            o = np.argsort(s_)
            cuts = np.nonzero(np.diff(s_[o]) > 0.006)[0] + 1
            runs = np.split(o, cuts)
            pts = pts[max(runs, key=len)]
    if len(pts) < 10:
        return None
    c = pts.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((pts - c).T))
    a = v[:, -1]
    s = (pts - c) @ a
    lo, hi = np.percentile(s, 3), np.percentile(s, 97)
    centre = c + a * (lo + hi) / 2 + CAM_BIAS
    if float((np.asarray(toward_xy)[:2] - centre) @ a) < 0:
        a = -a
    return centre, a, float(hi - lo)


def demo_script(ctx):
    """Left (scrub) arm looks for the plug with the top camera, pinches it near its cord end, turns it to point along
    the hand-off line, brings it in from behind the holder so the trailing cord drops into the slot, carries it out
    over the tray until the whole plug is past the edge, opens its fingers 12 mm above the tray (the plug drops onto
    it, untouched by the glove) and moves back over the field. Right (circulating) arm looks again, pinches the plug
    1 cm from its tip (its whole arm stays clear of the field), draws it 2.2 cm further out, lets go, looks again,
    re-grips 38 mm from the tip, pushes it along the line into the jack, lets go and backs off."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("STERILE_CHEAT", "")

    def log(tag, est=None):
        st = ctx.states()  # logging only: never used to choose a motion
        p, a = plug_pose(st)
        extra = ""
        if est is not None:
            extra = (f" cam=({est[0][0]:.3f},{est[0][1]:.3f}) cam_err={np.round(est[0] - p[:2], 4).tolist()} "
                     f"cam_hd_err={math.degrees(_wrap(_hd(est[1]) - _hd(a))):.1f} len={est[2]:.3f}")
        print(f"[demo t={ctx.t:5.1f}] {tag}: plug={np.round(p, 3).tolist()} hd={math.degrees(_hd(a)):.0f} "
              f"ax={np.round(ax_coords(p), 3).tolist()} depth={jack_depth(st):.3f} slot={cord_in_slot(st)} "
              f"off={passed_off(st)} relL={released(st, (0,))} rel={released(st)} viol={violations()} "
              f"Ltouch={int(_MON['left_touch'])} Rtouch={int(_MON['right_touch'])} dirty={_MON['n_dirty']} "
              f"maxRY={_MON['max_right_y']:.4f} minGloveZ={_MON['min_glove_z'] - FT:.4f} glove_part={_MON['glove_part']} "
              f"right_link={_MON['right_link']} arms_gap={_MON['min_arms_gap']:.3f} score={score(st):.2f}{extra} "
              f"ev={_MON['events'][-3:]} Ltcp={np.round(ax_coords(L.tcp()), 3).tolist()} "
              f"Rtcp={np.round(ax_coords(Rt.tcp()), 3).tolist()}", flush=True)

    def look(near, toward, on_axis=True, tries=4):
        est = None
        for _ in range(tries):
            yield from L.wait(0.3)
            est = see_plug(ctx, near, toward, on_axis=on_axis)
            if est is not None and PL - 0.025 <= est[2] <= PL + 0.03:
                break
        return est

    log("start")
    if cheat == "nothing":
        yield from L.wait(5.0)
        log("end")
        return
    if cheat == "glove_touch":
        # the scrub gripper brushes the bare table just beyond the drape's far-left edge, lifts, then does the full run
        t0 = (0.30, FIELD_Y1 + 0.09)
        yield from L.move((t0[0], t0[1], FT + 0.08), yaw=0.0, duration=1.5)
        yield from L.move((t0[0], t0[1], TOP + 0.014), yaw=0.0, duration=1.5)
        yield from L.wait(0.3)
        log("scrub gripper touching the table beside the drape")
        yield from L.move((t0[0], t0[1], FT + 0.10), yaw=0.0, duration=1.2)

    hi = FT + 0.06
    park_r = (0.25, -0.30, hi + 0.04)
    if cheat == "reach_in":
        # the right gripper reaches across the edge plane and closes on the sterile cord on the drape between the
        # holster and the slot, lets go and backs off; then the full run
        q = A + _DIR * 0.05
        yield from Rt.move((q[0], q[1], hi), yaw=CORD_YAW, finger=FINGER_OPEN, duration=1.8)
        yield from Rt.move((q[0], q[1], FT + R + 0.012), yaw=CORD_YAW, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED)
        yield from Rt.wait(0.3)
        log("right gripper closed on the sterile cord over the field")
        yield from Rt.grip(FINGER_OPEN, 0.5)
        yield from Rt.move((q[0], q[1], hi), yaw=CORD_YAW, duration=0.8)
        yield from Rt.move(park_r, yaw=0.0, duration=1.5)
    if cheat == "lean_over":
        # right gripper centre stays off the field (y = -3 cm) but its wrist leans over the drape; then the full run
        yield from Rt.move((0.26, -0.03, FT + 0.12), yaw=0.0, duration=1.5)
        yield from Rt.wait(0.5)
        log("right wrist over the field, gripper centre off it")
        yield from Rt.move(park_r, yaw=0.0, duration=1.2)

    # 0) left arm out of the camera's line of sight, then find the plug (fixed layout: it lies at the cord's end)
    plug0 = A + _DIR * (CORD_LEN + PL / 2)
    yield from L.move((0.14, 0.40, FT + 0.10), yaw=0.0, duration=1.2)
    est = yield from look(plug0, plug0 + _DIR, on_axis=False)  # tip = the end away from the holster
    log("seen on drape", est)
    if est is None:
        est = (plug0, _DIR, PL)
    c0, a0, _ = est
    rear = c0 - a0 * PL / 2
    gy = _wrap(_hd(a0))
    if abs(gy) > math.pi / 2:
        gy = _wrap(gy + math.pi)
    g = rear + a0 * SCRUB_GRIP_BACK
    yield from L.move((g[0], g[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((g[0], g[1], FT + PH / 2 + GRASP_DZ), yaw=gy, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    yield from L.move((g[0], g[1], hi), yaw=gy, duration=0.8)
    log("scrub has plug")
    # 1) turn the plug to point along the axis and come in from behind the holder, then out over the tray
    tcp_d = DROP_TCP_D
    if cheat == "noclip":  # comes in sideways from the left of the axis: the trailing cord misses the slot
        back, pre = ax(tcp_d - 0.06, 0.16), ax(tcp_d - 0.06, 0.02)
    else:  # comes in along the axis from behind the holder: the trailing cord lies over the slot
        back, pre = ax(tcp_d - 0.24), ax(tcp_d - 0.06)
    zd = FT + PH / 2 + DROP_H + GRASP_DZ
    yield from L.move((back[0], back[1], hi), yaw=AX_YAW, duration=2.0)
    yield from L.move((pre[0], pre[1], FT + 0.035), yaw=AX_YAW, duration=1.5)
    tgt = ax(tcp_d)
    yield from L.move((tgt[0], tgt[1], zd), yaw=AX_YAW, duration=1.2)
    yield from L.wait(0.3)
    if cheat == "tray_set":  # sets it down on the tray past the edge, still holding it, then lets go there
        yield from L.move((tgt[0], tgt[1], FT + PH / 2 + GRASP_DZ + 0.0005), yaw=AX_YAW, duration=0.8)
        yield from L.wait(0.5)
        log("scrub holding it on the tray")
    log("plug held out over the tray")
    yield from L.grip(FINGER_OPEN, 0.3)
    yield from L.wait(0.3)
    log("plug dropped onto the tray")
    yield from L.move((tgt[0], tgt[1], hi + 0.02), yaw=AX_YAW, duration=0.8)
    yield from L.move((0.20, 0.30, hi + 0.05), yaw=0.0, duration=1.2)

    if cheat == "regrip":  # the scrub arm picks the plug up off the tray again and puts it back down
        est = yield from look(ax(0.08), ax(0.3))
        log("seen on tray (regrip)", est)
        c, a, _ = est if est is not None else (ax(tcp_d - SCRUB_GRIP_BACK + PL / 2), U, PL)
        gg = c - a * (PL / 2 - SCRUB_GRIP_BACK)
        yield from L.move((gg[0], gg[1], hi), yaw=AX_YAW, finger=FINGER_OPEN, duration=1.2)
        yield from L.move((gg[0], gg[1], FT + PH / 2 + GRASP_DZ), yaw=AX_YAW, duration=0.8)
        yield from L.grip(FINGER_CLOSED)
        yield from L.move((gg[0], gg[1], zd), yaw=AX_YAW, duration=0.6)
        log("scrub holding the plug again")
        yield from L.grip(FINGER_OPEN, 0.3)
        yield from L.move((gg[0], gg[1], hi + 0.02), yaw=AX_YAW, duration=0.8)
        yield from L.move((0.20, 0.30, hi + 0.05), yaw=0.0, duration=1.2)
    if cheat == "pass_only":  # the scrub's half only: the plug stays on the tray, both arms clear
        yield from L.wait(3.0)
        log("end")
        return

    # 2) look at the plug on the tray (both arms out of the line of sight)
    yield from Rt.move(park_r, yaw=0.0, duration=1.2)
    est = yield from look(ax(0.08), ax(0.3))
    log("seen on tray", est)
    if est is None:
        est = (ax(tcp_d - SCRUB_GRIP_BACK + PL / 2), U, PL)
    c1, a1, _ = est
    tip1 = c1 + a1 * PL / 2
    ry = _wrap(_hd(a1))
    if abs(ry) > math.pi / 2:
        ry = _wrap(ry + math.pi)
    g = tip1 - a1 * PULL_GRIP
    yield from Rt.move((g[0], g[1], hi), yaw=ry, finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move((g[0], g[1], FT + PH / 2 + GRASP_DZ), yaw=ry, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    log("circulator has plug by its tip")

    # 3) draw it further out along the tray, set it down, let go and clear the camera's view
    g2 = g + U * PULL_DIST
    yield from Rt.move((g[0], g[1], FT + PH / 2 + GRASP_DZ + 0.003), yaw=ry, duration=0.4)
    yield from Rt.move((g2[0], g2[1], FT + PH / 2 + GRASP_DZ + 0.003), yaw=ry, duration=1.5)
    yield from Rt.move((g2[0], g2[1], FT + PH / 2 + GRASP_DZ), yaw=ry, duration=0.4)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((g2[0], g2[1], hi), yaw=ry, duration=0.6)
    yield from Rt.move(park_r, yaw=0.0, duration=1.0)
    est = yield from look(ax(0.13), ax(0.4))
    log("seen drawn out", est)
    if est is None:
        est = (ax(tcp_d - SCRUB_GRIP_BACK + PL / 2 + PULL_DIST), U, PL)
    c2, a2, _ = est
    tip2 = c2 + a2 * PL / 2

    # 4) re-grip further back, turn onto the axis, push into the jack (plug held centred between the pads)
    g3 = tip2 - a2 * SEAT_GRIP
    yield from Rt.move((g3[0], g3[1], hi), yaw=ry, finger=FINGER_OPEN, duration=1.2)
    yield from Rt.move((g3[0], g3[1], FT + PH / 2 + GRASP_DZ), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED)
    log("circulator re-gripped")
    tip_d = ax_coords(tip2)[0]
    yaw_ax = AX_YAW if abs(_wrap(AX_YAW - ry)) < math.pi / 2 else _wrap(AX_YAW + math.pi)

    def tcp_for_tip(d_tip: float, dz: float = 0.0):
        x = ax(d_tip - SEAT_GRIP)
        return (x[0], x[1], JACK_ZC + GRASP_DZ + dz)

    yield from Rt.move(tcp_for_tip(tip_d, 0.004), yaw=ry, duration=0.5)
    yield from Rt.move(tcp_for_tip(max(tip_d, JACK_D - 0.025), 0.002), yaw=yaw_ax, duration=1.5)
    yield from Rt.move(tcp_for_tip(JACK_D - 0.008, 0.0), yaw=yaw_ax, duration=1.0)
    log("at jack mouth")
    yield from Rt.move(tcp_for_tip(JACK_D + SEAT_DEPTH, 0.0), yaw=yaw_ax, duration=1.5)
    yield from Rt.wait(0.3)
    log("seated (held)")
    if cheat == "hold":
        yield from Rt.wait(3.0)
        log("end")
        return
    yield from Rt.grip(FINGER_OPEN, 0.5)
    cur = Rt.tcp()
    yield from Rt.move((cur[0], cur[1], cur[2] + 0.12), yaw=yaw_ax, duration=0.8)
    yield from Rt.move(park_r, yaw=0.0, duration=1.2)
    yield from Rt.wait(1.5)
    log("end")
