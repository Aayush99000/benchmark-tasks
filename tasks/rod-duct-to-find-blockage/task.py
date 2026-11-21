"""Push a fish tape through a conduit until its tip comes out in the far box, then draw the tip out past a mark.

An electrician pulling wire through conduit first pushes a fish tape through the run, from one box to the other, and
fastens the wire to its tip where it comes out. Here a short run of 3/4-inch EMT (20.9 mm bore) lies strapped on the
table in front of the robot: an open near mouth with a plastic bushing faces the robot's left; the run goes 27 cm
towards the robot's right, turns towards the robot through one 90-degree bend (127 mm centreline radius, a 3/4-inch
EMT hand bender) and runs 2 cm into the knockout of a 4-inch square steel box, 1-1/2 inches deep, cover off. A yellow
4.8 mm (3/16 in) fibreglass fish tape lies on the table out of the near mouth with its brass tip inside it; how far in
is drawn afresh at every reset (3-20 cm, seeded). The conduit and box are fixed. A red line is taped on the table 3 cm
beyond the box's back wall, across the line the tape comes out on.

Two-hand feed, as electricians do it: the left hand holds the tape on the conduit's axis 20 cm out from the mouth and
the right hand pinches it 5.5 cm out; both push 3.5 cm towards the mouth together; the right hand reopens and goes
back while the left holds, then the left does the same while the right holds, so the tape is never let go. Once the
camera sees the tape 5 cm into the box, the right hand takes it there, draws the tip up out of the box and lays it on
the table past the red line. Both let go.

Score (end state; 1.0 must hold 1 s): both grippers at least 3 cm from the tape, and the tape running through the
whole conduit (in the bore within 3 cm of both mouths), else 0. Then: 1.0 if the tip lies on the table past the red
line; 0.5 if the tip is out of the far mouth but not past the line; 0.25 if it is through the bend but not out; else 0.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Push the fish tape through the conduit into the far box, then pull its tip past the red line and let go."

# -- conduit: 3/4-inch EMT (OD 23.4 mm, ID 20.9 mm) strapped on the table ---------------------------------------------
OD, ID = 0.02342, 0.02093
RI, RO = ID / 2, OD / 2
ZA = TOP + RO  # axis height: the conduit lies on the table
R_BEND = 0.127  # centreline radius of the 90-degree bend (5 in, a 3/4-inch EMT hand bender)
DX = 0.35  # near leg runs along -y at this x
YM = 0.0  # near mouth (open, faces +y, the robot's left)
Y_B = -0.27  # the bend starts here
L_FAR = 0.02  # short far leg from the end of the bend to the box (the connector)
H0 = -math.pi / 2  # heading at the near mouth (along -y)
PIECES = [("line", YM - Y_B), ("arc", R_BEND, -math.pi / 2), ("line", L_FAR)]  # arc: (radius, signed turn; - = clockwise)
SEGS = [(p[0], p[1] if p[0] == "line" else abs(p[2]) * p[1]) for p in PIECES]
S_EDGES = np.cumsum([0.0] + [ln for _, ln in SEGS])
LC = float(S_EDGES[-1])  # conduit length along its axis (m)
S_BEND_END = float(S_EDGES[2])  # past the bend
GALV = (0.70, 0.72, 0.74)
BUSH_L, BUSH_R, BUSH_FLARE = 0.009, 0.0150, 0.0160  # near-mouth plastic bushing, flared throat


def _dir(h):
    return np.array([math.cos(h), math.sin(h)])


def axis_at(s: float) -> tuple[np.ndarray, np.ndarray]:
    """Conduit axis point and unit direction at arc length ``s`` (0..LC) from the near mouth."""
    p, h = np.array([DX, YM]), H0
    for piece in PIECES:
        if piece[0] == "line":
            ln = piece[1]
            if s <= ln:
                q = p + s * _dir(h)
                return np.array([q[0], q[1], ZA]), np.array([*_dir(h), 0.0])
            p, s = p + ln * _dir(h), s - ln
        else:
            r, turn = piece[1], piece[2]
            sg = math.copysign(1.0, turn)
            ln = r * abs(turn)
            left = np.array([-math.sin(h), math.cos(h)])
            c = p + sg * r * left
            u = min(s, ln)
            ph = h + sg * u / r
            q = c - sg * r * np.array([-math.sin(ph), math.cos(ph)])
            if s <= ln:
                return np.array([q[0], q[1], ZA]), np.array([*_dir(ph), 0.0])
            p, h, s = q, ph, s - ln
    return np.array([p[0], p[1], ZA]), np.array([*_dir(h), 0.0])


_SS = np.linspace(-0.6, LC + 0.6, int((LC + 1.2) / 0.001) + 1)
_AX = np.array([axis_at(float(s))[0] if 0 <= s <= LC else
                (axis_at(0.0)[0] + s * axis_at(0.0)[1] if s < 0 else axis_at(LC)[0] + (s - LC) * axis_at(LC)[1])
                for s in _SS])


def project(p) -> tuple[float, float]:
    """(s, r): arc length of the nearest axis point (end legs extended 60 cm) and the distance from it (m)."""
    d = np.linalg.norm(_AX - np.asarray(p, float)[None, :], axis=1)
    i = int(np.argmin(d))
    return float(_SS[i]), float(d[i])


# -- far box: 4-inch square steel box, 1-1/2 in deep, cover off; the conduit enters one wall through a knockout -------
BOX_IN = 0.1016  # inside width
BOX_D = 0.038  # inside depth
BOX_T = 0.0015
KO = 0.028  # 3/4-inch knockout, about 1-1/8 in
STEEL = (0.62, 0.64, 0.66)
FM = axis_at(LC)[0][:2]  # far mouth (inner face of the knockout wall)
FD = axis_at(LC)[1][:2]  # into the box
FL = np.array([-FD[1], FD[0]])  # across the box
BOX_C = FM + FD * BOX_IN / 2  # inside centre
_bz = TOP + BOX_D / 2
GRAB = tuple(float(v) for v in FM + 0.03 * FD)  # where the right hand pinches the tape in the box
GRAB_YAW = (math.atan2(FD[1], FD[0]) + math.pi / 2) % math.pi - math.pi / 2  # fingers close across the tape there


def to_box(p) -> np.ndarray:
    """(a, b) of plane points: a = depth into the box from the knockout wall's inner face, b = across."""
    v = np.asarray(p, float)[..., :2] - FM
    return np.stack([v @ FD, v @ FL], axis=-1)


def _bpiece(name, a, b, z, sa, sb, sz):
    c = FM + a * FD + b * FL
    size = (sa, sb, sz) if abs(FD[0]) > 0.5 else (sb, sa, sz)
    return Box(name, pos=(float(c[0]), float(c[1]), z), size=size, color=STEEL)


_ka = (BOX_IN / 2 + KO / 2) / 2  # centre of each knockout-wall piece beside the knockout
BOX = [
    _bpiece("box_floor", BOX_IN / 2, 0.0, TOP + 0.0005, BOX_IN + 2 * BOX_T, BOX_IN + 2 * BOX_T, 0.001),
    _bpiece("box_wall_l", BOX_IN / 2, BOX_IN / 2 + BOX_T / 2, _bz, BOX_IN + 2 * BOX_T, BOX_T, BOX_D),
    _bpiece("box_wall_r", BOX_IN / 2, -BOX_IN / 2 - BOX_T / 2, _bz, BOX_IN + 2 * BOX_T, BOX_T, BOX_D),
    _bpiece("box_wall_back", BOX_IN + BOX_T / 2, 0.0, _bz, BOX_T, BOX_IN, BOX_D),
    # the knockout wall, in three pieces round the 28 mm knockout the conduit's connector fills
    _bpiece("box_ko_l", -BOX_T / 2, _ka, _bz, BOX_T, BOX_IN / 2 - KO / 2, BOX_D),
    _bpiece("box_ko_r", -BOX_T / 2, -_ka, _bz, BOX_T, BOX_IN / 2 - KO / 2, BOX_D),
    _bpiece("box_ko_top", -BOX_T / 2, 0.0, (ZA + KO / 2 + TOP + BOX_D) / 2, BOX_T, KO, TOP + BOX_D - ZA - KO / 2),
]
# straps: two-hole steel straps holding the conduit down on the near leg (visual and contact only)
STRAPS = [
    Box("strap_1", pos=(DX, YM - 0.08, TOP + OD + 0.0008), size=(0.03, 0.016, 0.0016), color=STEEL),
    Box("strap_2", pos=(DX, Y_B + 0.08, TOP + OD + 0.0008), size=(0.03, 0.016, 0.0016), color=STEEL),
]
# red line taped on the table beyond the box's back wall (the side facing the knockout), 3 cm out from it, across
# the line the tape comes out of the conduit on: the tip must be laid on the table past it
X_MARK = float(BOX_C[0] - BOX_IN / 2 - BOX_T - 0.03)
MARK_Y0, MARK_Y1 = float(BOX_C[1] - 0.07), float(BOX_C[1] + 0.07)
MARK = Box("red_line", pos=(X_MARK, float(BOX_C[1]), TOP + 0.0004), size=(0.012, 0.14, 0.0008),
           color=(0.85, 0.08, 0.06))

# -- fish tape: start = brass tip (leading end, in the conduit), end = black handle (the case's stand-in) -------------
TAPE_R = 0.0024  # 4.8 mm (3/16 in) round fibreglass
TAPE_LEN = 1.40
TIP = Plug(size=(0.012, 0.006, 0.006), mass=0.004, color=(0.78, 0.62, 0.25))  # brass leader tip
HANDLE = Plug(size=(0.030, 0.026, 0.012), mass=0.020, color=(0.08, 0.08, 0.08))  # wider than the bore
Z_IN = ZA - RI + TAPE_R + 0.0005
Z_OUT = TOP + TAPE_R + 0.0005
D0_MIN, D0_MAX = 0.03, 0.20  # tip front this far inside the near mouth at the start, drawn at every reset
TAPE = cable("fish_tape", start=(DX, YM - 0.10, Z_IN), yaw=math.pi / 2, length=TAPE_LEN, radius=TAPE_R,
             start_plug=TIP, end_plug=HANDLE, cable_mass=0.045, color=(0.95, 0.80, 0.08),
             bend_stiffness=0.03, bend_damping=0.003,
             friction=0.25)

OBJECTS = [*BOX, *STRAPS, MARK, TAPE]


# -- the conduit's bore: static 12-sided collider lining, plus the visual tube mesh -----------------------------------
def _rot_cols(x, z) -> np.ndarray:
    x = np.asarray(x, float) / np.linalg.norm(x)
    z = np.asarray(z, float)
    z = z - x * (z @ x)
    z /= np.linalg.norm(z)
    return np.stack([x, np.cross(z, x), z], axis=1)


def _quat_wxyz(m) -> tuple[float, float, float, float]:
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    i = int(np.argmax(np.diag(m)))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)


N_SIDES = 12
WALL_COLL = 0.003


def _ring_frame(s):
    if s < 0:
        p, t = axis_at(0.0)
        p = p + s * t
    elif s > LC:
        p, t = axis_at(LC)
        p = p + (s - LC) * t
    else:
        p, t = axis_at(s)
    u = np.cross([0.0, 0.0, 1.0], t)
    u /= np.linalg.norm(u)
    return p, t, u, np.array([0.0, 0.0, 1.0])


def _breaks() -> list[float]:
    out = [0.0]
    for (kind, ln), s0 in zip(SEGS, S_EDGES[:-1]):
        if ln <= 1e-6:
            continue
        n = 40 if kind.startswith("arc") else 1  # collider pieces per bend
        out += [float(s0 + ln * (j + 1) / n) for j in range(n)]
    return out


def conduit_slabs() -> list[tuple[np.ndarray, np.ndarray, tuple]]:
    """Static collider boxes lining the bore (12-sided, flats at the bore radius) and the near bushing's throat."""
    out = []
    half_w = RI * math.tan(math.pi / N_SIDES) + 0.0006

    def tube(s0, s1, r0, r1):
        pa, ta, ua, va = _ring_frame(s0)
        pb, tb, ub, vb = _ring_frame(s1)
        for k in range(N_SIDES):
            ph = 2 * math.pi * k / N_SIDES
            na = math.cos(ph) * ua + math.sin(ph) * va
            nb = math.cos(ph) * ub + math.sin(ph) * vb
            a = pa + (r0 + WALL_COLL / 2) * na
            b = pb + (r1 + WALL_COLL / 2) * nb
            ln = float(np.linalg.norm(b - a))
            m = _rot_cols(b - a, (na + nb) / 2)
            out.append(((a + b) / 2, m, (ln + 0.0015, 2 * half_w * (r0 + r1) / (2 * RI), WALL_COLL)))

    br = _breaks()
    for s0, s1 in zip(br[:-1], br[1:]):
        tube(s0, s1, RI, RI)
    tube(-BUSH_L, 0.0, BUSH_FLARE, RI)
    return out


def _tube_mesh(samples, r_in, r_out, n=32):
    pts, quads = [], []
    m = len(samples)
    for p, t, u, v in samples:
        for r in (r_out, r_in):
            for k in range(n):
                ph = 2 * math.pi * k / n
                pts.append(p + r * (math.cos(ph) * u + math.sin(ph) * v))

    def idx(i, layer, k):
        return (i * 2 + layer) * n + k % n

    for i in range(m - 1):
        for k in range(n):
            quads.append([idx(i, 0, k), idx(i + 1, 0, k), idx(i + 1, 0, k + 1), idx(i, 0, k + 1)])
            quads.append([idx(i, 1, k), idx(i, 1, k + 1), idx(i + 1, 1, k + 1), idx(i + 1, 1, k)])
    for i, sgn in ((0, 1), (m - 1, -1)):
        for k in range(n):
            q = [idx(i, 0, k), idx(i, 0, k + 1), idx(i, 1, k + 1), idx(i, 1, k)]
            quads.append(q if sgn > 0 else q[::-1])
    return np.array(pts), quads


def _author_conduit(stage, parent: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    root = f"{parent}/emt_conduit"
    UsdGeom.Xform.Define(stage, root)
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(GALV), roughness=0.35, metallic=0.4)
    look.func(f"{root}/Looks/galv", look)
    blk = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb((0.05, 0.05, 0.05)), roughness=0.6)
    blk.func(f"{root}/Looks/bushing", blk)
    cf = 0.15  # bore friction (chosen, not measured)
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=cf, dynamic_friction=cf)
    pm.func(f"{root}/Looks/phys", pm)
    for i, (c, m, size) in enumerate(conduit_slabs()):
        g = UsdGeom.Cube.Define(stage, f"{root}/wall_{i}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, c)))
        w, x, y, z = _quat_wxyz(m)
        g.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        g.CreatePurposeAttr("guide")
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(g.GetPath()), f"{root}/Looks/phys", stage=stage)

    def add_mesh(name, samples, r_in, r_out, mat):
        pts, quads = _tube_mesh(samples, r_in, r_out)
        mesh = UsdGeom.Mesh.Define(stage, f"{root}/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
        mesh.CreateFaceVertexCountsAttr([4] * len(quads))
        mesh.CreateFaceVertexIndicesAttr([int(i) for q in quads for i in q])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        bind_visual_material(str(mesh.GetPath()), mat, stage=stage)

    ss = sorted(set(_breaks() + [float(s) for s in np.linspace(S_EDGES[1], S_EDGES[2], 25)]))
    add_mesh("tube", [_ring_frame(s) for s in ss], RI, RO, f"{root}/Looks/galv")
    add_mesh("bushing", [_ring_frame(-BUSH_L), _ring_frame(0.002)], RI + 0.0008, BUSH_R, f"{root}/Looks/bushing")
    # set-screw connector body and locknut at the box (visual only; the knockout wall pieces carry the contact)
    add_mesh("connector", [_ring_frame(LC - 0.025), _ring_frame(LC)], RI + 0.0008, RO + 0.0025, f"{root}/Looks/galv")


def _install_parts() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_ft_parts", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") == TAPE.name:
            from isaaclab.sim.utils.stage import get_current_stage

            _author_conduit(get_current_stage(), prim_path.rsplit("/", 1)[0])
        return prim

    spawn._ft_parts = True
    cablekit._spawn_cable = spawn


_install_parts()


# -- starting lay: tip D0 inside the near mouth; straight out of the mouth along +y for 33 cm, one 180-degree bend of
#    13 cm radius away from the robot, then straight back along -y ---------------------------------------------------
R_U = 0.13
Y_BEND = min(YM + 0.33, 0.535 - R_U)  # straight lead out of the mouth, where the hands feed


def _arc(c, r, a0, a1, z, n=60):
    a = np.linspace(a0, a1, n)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a), np.full(n, z)]


def start_path(d0: float) -> np.ndarray:
    s0 = YM - d0 + TIP.size[0]  # first segment starts behind the tip
    inside = np.c_[np.full(20, DX), np.linspace(s0, YM + 0.005, 20), np.full(20, Z_IN)]
    drop = np.c_[np.full(10, DX), np.linspace(YM + 0.01, YM + 0.05, 10), np.linspace(Z_IN, Z_OUT, 10)]
    lead = np.c_[np.full(20, DX), np.linspace(YM + 0.055, Y_BEND, 20), np.full(20, Z_OUT)]
    u = _arc((DX + R_U, Y_BEND), R_U, math.pi, 0.0, Z_OUT)
    tail = np.c_[np.full(60, DX + 2 * R_U), np.linspace(Y_BEND - 0.005, Y_BEND - 1.4, 60), np.full(60, Z_OUT)]
    return np.vstack([inside, drop, lead, u, tail])


# -- episode state ----------------------------------------------------------------------------------------------------
_ST: dict = {"env": None, "seed": None, "resets": 0, "d0": D0_MIN, "nan": False, "max_s": -1.0}


def draw_d0(seed: int | None, n_reset: int) -> float:
    """How far the tip starts inside the near mouth: the episode seed if the runner gives one, else FT_SEED (default
    0) plus the number of earlier resets in this process. Uniform in [D0_MIN, D0_MAX], to 1 mm."""
    s = int(seed) if seed is not None else int(os.environ.get("FT_SEED", "0")) + n_reset
    return round(float(np.random.default_rng([s, 1101]).uniform(D0_MIN, D0_MAX)), 3)


def _shape(emb):
    _ST.update(nan=False, max_s=-1.0, env=emb._env)
    _ST["d0"] = draw_d0(_ST["seed"], _ST["resets"])
    _ST["resets"] += 1
    print(f"[fish tape task] tip starts {_ST['d0'] * 100:.1f} cm inside the near mouth", flush=True)
    cableshape.preshape(emb, TAPE, start_path(_ST["d0"]), hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)


def _install_hooks() -> None:
    """Keep the episode seed the runner passes to reset(), and the env for the grippers' positions."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_ft_seed", False):
        inner = cls.reset

        def reset(self, *a, **k):
            _ST["seed"] = k.get("seed")
            return inner(self, *a, **k)

        reset._ft_seed = True
        cls.reset = reset
    if not getattr(cls._task_step, "_ft_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ST["env"] = self._env
            return base(self, *a, **k)

        _task_step._ft_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    env = _ST["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- scoring geometry -------------------------------------------------------------------------------------------------
BORE_TOL = 0.004
RELEASE = 0.03  # every gripper TCP at least this far from every point of the tape
END_IN = 0.03  # "runs through the whole run": tape inside the bore within this of both mouths
ON_TABLE = 0.02  # tip centre at most this above the table top


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def tape_points(objects) -> np.ndarray:
    c = objects["fish_tape"]
    pts = c.polyline()
    if c.start_plug is not None:
        pts = np.vstack([c.start_plug[None, :], pts])
    return _dense(pts)


def tip(objects) -> np.ndarray:
    return np.asarray(objects["fish_tape"].start_plug, dtype=float)


def in_bore(p, tol: float = BORE_TOL) -> bool:
    s, r = project(p)
    return bool(0.0 <= s <= LC and r <= RI + tol)


def tip_s(objects) -> float:
    """How far along the run the tip is (m from the near mouth along the axis; > LC once out in the box)."""
    p = tip(objects)
    s, r = project(p)
    if s > LC:  # past the far mouth: count straight-line distance from it
        return LC + float(np.linalg.norm(p[:2] - axis_at(LC)[0][:2]))
    return s


def threaded(objects) -> bool:
    """The tape runs through the whole run: some of it in the bore within END_IN of each mouth."""
    ss = [project(p)[0] for p in tape_points(objects) if in_bore(p)]
    return bool(ss) and min(ss) <= END_IN and max(ss) >= LC - END_IN


def tip_out(objects) -> bool:
    """The tip is out of the far mouth (in the box or beyond)."""
    return tip_s(objects) > LC + 0.005


def tip_past_mark(objects) -> bool:
    p = tip(objects)
    return bool(p[0] < X_MARK and MARK_Y0 - 0.05 <= p[1] <= MARK_Y1 + 0.05 and p[2] <= TOP + ON_TABLE)


def released(objects) -> bool:
    g = grippers()
    if not g:
        return True
    pts = tape_points(objects)
    return all(float(np.linalg.norm(pts - x, axis=1).min()) >= RELEASE for x in g)


def _finite(objects) -> bool:
    return bool(np.isfinite(objects["fish_tape"].points).all())


def _track(objects) -> None:
    if not _finite(objects):
        _ST["nan"] = True
        return
    _ST["max_s"] = max(_ST["max_s"], tip_s(objects))


def score(objects) -> float:
    """0 unless both grippers are clear of the tape and the tape runs through the whole run. Then 1.0 if the tip lies
    on the table past the red line; 0.5 if the tip is out of the far mouth; 0.25 if it is through the bend."""
    _track(objects)
    if _ST["nan"] or not released(objects) or not threaded(objects):
        return 0.0
    if tip_past_mark(objects):
        return 1.0
    if tip_out(objects):
        return 0.5
    return 0.25 if tip_s(objects) >= S_BEND_END else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """How far the tip still has to go: along the run to the far mouth, then on to the red line (m)."""
    _track(objects)
    s = tip_s(objects)
    p = tip(objects)
    return float(max(0.0, LC - s) + max(0.0, p[0] - X_MARK))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    if not _finite(objects):
        return {"nan": 1}
    return {
        "tip_along_run": float(np.clip(tip_s(objects) / LC, 0.0, 1.5)),
        "through_bends": int(tip_s(objects) >= S_BEND_END),
        "tip_out": int(tip_out(objects)),
        "past_line": int(tip_past_mark(objects)),
        "threaded": int(threaded(objects)),
        "released": int(released(objects)),
    }


VARIATION = {"cable:fish_tape": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}

TWIN_EXTRA = [
    {"name": "tape_tip", "kind": "body", "size_mm": [12, 6, 6], "colour": "brass", "note": "brass leader tip"},
    {"name": "tape_handle", "kind": "body", "size_mm": [30, 26, 12], "colour": "black",
     "note": "stand-in for the fish tape's case at the tape's end; wider than the bore"},
]

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------------
# Actions come only from the top camera image (vision.py), the arms' own TCPs and the fixed layout (the conduit, its
# mouths, the box, the red line). ctx.states() is read only by _log() (printed for the run log, never used to choose a
# motion).
HI = TOP + 0.07
GRASP_OPEN = -0.014
YAW_L = -math.pi / 2  # left gripper across a tape that runs along y
PARK_L = (0.10, 0.44, TOP + 0.10)  # left arm out of the top camera's view while looking
PARK_R = (0.10, -0.30, TOP + 0.12)
TAPE_HUE, TAPE_HUE_TOL = 52.0, 12.0
TAPE_SAT = 0.45
IN_BOX = 0.05  # stop feeding once the tape seen in the box reaches this far from the knockout


def _tape_mask(img):
    import vision

    return vision.hsv_mask(img, TAPE_HUE, TAPE_HUE_TOL, min_sat=TAPE_SAT, min_val=90)


def _see_tape(ctx):
    """Top camera: the yellow tape traced from the near mouth outwards (vision.Polyline, arc length from the mouth)."""
    import vision

    img = vision.top_image(ctx)
    return vision.trace_cable(img, _tape_mask(img), Z_OUT, (DX, YM))


def _box_tape(ctx) -> np.ndarray:
    """Top camera: tape pixels inside the far box, back-projected onto the box floor plane (N, 2)."""
    import vision

    img = vision.top_image(ctx)
    p = vision.top_camera(img.shape).to_plane(_tape_mask(img), Z_OUT)
    ab = to_box(p)
    m = (np.abs(ab[:, 1]) < BOX_IN / 2 - 0.004) & (ab[:, 0] > 0.004) & (ab[:, 0] < BOX_IN - 0.004)
    return p[m]


def _in_box_reach(ctx) -> float:
    """How far into the box (from the knockout wall) the tape is seen; 0 if none."""
    p = _box_tape(ctx)
    return float(to_box(p)[:, 0].max()) if len(p) >= 3 else 0.0


def _look(ctx, L):
    """Park the left arm out of the camera's line of sight, wait, and trace the tape (3 tries)."""
    p = L.tcp()
    if np.linalg.norm(p - np.array(PARK_L)) > 0.02:
        yield from L.move((p[0], p[1], max(p[2], TOP + 0.05)), yaw=L._yaw, duration=0.25)
        yield from L.move(PARK_L, yaw=0.0, duration=0.7)
    poly = None
    for _ in range(3):
        yield from L.wait(0.3)
        poly = _see_tape(ctx)
        if poly is not None:
            break
    return poly


_FEED: dict = {}


YAW_R = math.pi / 2  # right gripper across a tape that runs along y (yaw is world-frame for both arms)
Y_R0, Y_R1 = YM + 0.055, YM + 0.02  # front (right) hand: pinch 5.5 cm from the mouth, push to 2 cm from it
Y_L0 = YM + 0.205  # back (left) hand: holds the tape 20.5 cm from the mouth (15 cm behind the front hand: the grippers clear each other)
STROKE = Y_R0 - Y_R1  # tape fed per stroke (3.5 cm)


def _move_turn(arm, pos, yaw, duration):
    """Generator: move to ``pos`` with a straight-down grip turned ``yaw``, blending position and wrist rotation from
    the arm's last command and solving IK every step from the previous solution (no branch jumps on a wrist turn)."""
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    p0, r0 = ik.tcp_pose(arm.q, arm.base)
    goal, r1 = np.asarray(pos, dtype=float), grip_rotation(yaw)
    n = max(1, round(duration * 20))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        q, pe, re = ik.solve(p0 + s * (goal - p0), ik.matrix_to_quat_xyzw(ik.slerp_matrix(r0, r1, s)), arm.q,
                             base_pos=arm.base, iters=60)
        arm.q = q
        arm.cmd = np.r_[q, arm.cmd[-1]]
        yield
    arm._yaw = yaw


def _dbg_tcp(ctx, arm, i, tag):
    """Log only: commanded TCP (from the IK solution) against the simulated TCP."""
    g = grippers()
    m = g[i] if g else np.full(3, np.nan)
    print(f"[dbg t={ctx.t:6.2f}] {tag}: {arm.name} cmd {np.round(arm.tcp(), 3)} sim {np.round(m, 3)}", flush=True)


def _feed2(ctx, L, R, fc, fo, *, max_strokes=30, stop_after=None):
    """Two-hand feed. Camera: trace the tape out of the mouth; the left (back) hand pinches it 14 cm out and lays it on
    the conduit's axis, the right (front) hand pinches it 6.5 cm out. Each stroke: both hands push 4.5 cm towards the
    mouth together; the front hand reopens and goes back to 6.5 cm while the back hand holds the tape, then pinches;
    the back hand reopens and goes back to 14 cm while the front hand holds it, then pinches. The tape is always held
    by at least one hand on the axis. Stop once the camera sees the tape IN_BOX into the far box."""
    z1 = Z_IN + 0.010
    zu = z1 + 0.02
    poly = yield from _look(ctx, L)
    # the back-hand pinch: the tape centre the camera sees nearest the axis point Y_L0 out from the mouth
    xy = poly.snap((DX, Y_L0), radius=0.03) if poly is not None else np.array([DX, Y_L0])
    print(f"[demo] back-hand pinch at {np.round(xy, 3)} (camera)", flush=True)
    gy = fy = YAW_L
    yield from L.grip(GRASP_OPEN, 0.3)
    yield from _move_turn(L, (xy[0], xy[1], TOP + 0.06), gy, 1.6)
    yield from L.move((xy[0], xy[1], Z_OUT + 0.010), yaw=gy, duration=0.6)
    yield from L.grip(fc, 0.35)
    yield from L.move((DX, Y_L0, z1), yaw=fy, duration=0.5)
    _dbg_tcp(ctx, L, 0, "left holding the tape on the axis")
    if fy != YAW_L:  # turn the grip so the left wrist faces the mouth
        yield from L.grip(GRASP_OPEN, 0.25)
        yield from L.move((DX, Y_L0, zu), yaw=fy, duration=0.25)
        yield from L.move((DX, Y_L0, zu), yaw=YAW_L, duration=0.35)
        yield from L.move((DX, Y_L0, Z_OUT + 0.010), yaw=YAW_L, duration=0.3)
        yield from L.grip(fc, 0.3)
        yield from L.move((DX, Y_L0, z1), yaw=YAW_L, duration=0.25)
    yield from R.grip(fo, 0.3)
    yield from _move_turn(R, (DX - 0.05, Y_R0 - 0.06, TOP + 0.08), YAW_R, 2.0)
    _dbg_tcp(ctx, R, 1, "right above the mouth, wrist turned")
    yield from R.move((DX, Y_R0, TOP + 0.035), yaw=YAW_R, duration=1.0)
    yield from R.move((DX, Y_R0, z1), yaw=YAW_R, duration=0.6)
    _dbg_tcp(ctx, R, 1, "right at its pinch")
    _dbg_tcp(ctx, L, 0, "left at its pinch")
    yield from R.grip(fc, 0.3)
    _dbg(ctx, L, "both hands on the tape")
    dl = Y_L0 - Y_R0
    log = []
    for n in range(max_strokes):
        if stop_after is not None and n >= stop_after:
            break
        if _in_box_reach(ctx) >= IN_BOX:
            break
        yield from ctx.both(R.move((DX, Y_R1, z1), yaw=YAW_R, duration=0.6),
                            L.move((DX, Y_R1 + dl, z1), yaw=YAW_L, duration=0.6))
        _dbg(ctx, L, f"stroke {n + 1} pushed")
        yield from R.grip(fo, 0.2)
        yield from R.move((DX, Y_R1, zu), yaw=YAW_R, duration=0.15)
        yield from R.move((DX, Y_R0, zu), yaw=YAW_R, duration=0.3)
        yield from R.move((DX, Y_R0, z1), yaw=YAW_R, duration=0.15)
        yield from R.grip(fc, 0.25)
        yield from L.grip(fo, 0.2)
        yield from L.move((DX, Y_R1 + dl, zu), yaw=YAW_L, duration=0.15)
        yield from L.move((DX, Y_L0, zu), yaw=YAW_L, duration=0.3)
        yield from L.move((DX, Y_L0, z1), yaw=YAW_L, duration=0.15)
        yield from L.grip(fc, 0.25)
        log.append(n + 1)
        _dbg(ctx, L, f"stroke {n + 1}, tape seen {_in_box_reach(ctx):.3f} into the box")
    yield from R.grip(fo, 0.25)
    yield from L.grip(fo, 0.25)
    yield from ctx.both(R.move((DX, Y_R0, TOP + 0.035), yaw=YAW_R, duration=0.4), L.move((DX, Y_L0, HI), yaw=YAW_L, duration=0.4))
    _FEED.update(log=log, in_box=_in_box_reach(ctx))


def _feed_one(ctx, R, fc, fo, max_strokes=30):
    """Variant only: the right hand feeds alone, with the same strokes as the two-hand feed but nothing holding the tape
    while it reopens and goes back."""
    z1 = Z_IN + 0.010
    zu = z1 + 0.02
    yield from R.grip(fo, 0.3)
    yield from _move_turn(R, (DX - 0.05, Y_R0 - 0.06, TOP + 0.08), YAW_R, 2.0)
    yield from R.move((DX, Y_R0, TOP + 0.035), yaw=YAW_R, duration=1.0)
    yield from R.move((DX, Y_R0, z1), yaw=YAW_R, duration=0.6)
    log = []
    for n in range(max_strokes):
        if _in_box_reach(ctx) >= IN_BOX:
            break
        yield from R.grip(fc, 0.25)
        yield from R.move((DX, Y_R1, z1), yaw=YAW_R, duration=0.6)
        yield from R.grip(fo, 0.2)
        yield from R.move((DX, Y_R1, zu), yaw=YAW_R, duration=0.15)
        yield from R.move((DX, Y_R0, zu), yaw=YAW_R, duration=0.3)
        yield from R.move((DX, Y_R0, z1), yaw=YAW_R, duration=0.15)
        log.append(n + 1)
        _dbg(ctx, R, f"one-hand stroke {n + 1}, tape seen {_in_box_reach(ctx):.3f} into the box")
    yield from R.move((DX, Y_R0, TOP + 0.035), yaw=YAW_R, duration=0.4)
    _FEED.update(log=log, in_box=_in_box_reach(ctx))


def _draw_out(ctx, Rt, fo, fc):
    """Right arm: find the tape's free end in the box (camera), pinch the tape 1.5 cm back from it with the fingers only
    a little open (the box is 10 cm wide), lift it clear of the box's back wall, draw it on the way it comes out of the
    conduit, past the red line, and lay it down."""
    p = _box_tape(ctx)
    if len(p) < 3:
        print("[demo] no tape seen in the box", flush=True)
        return
    end = p[int(np.argmax(to_box(p)[:, 0]))]  # the seen tape point deepest in the box: the tip end
    back = FM - end
    back /= np.linalg.norm(back) + 1e-9
    xy = end + 0.015 * back
    near = p[np.linalg.norm(p - xy, axis=1) < 0.006]
    if len(near):
        xy = near.mean(axis=0)
    yaw = (math.atan2(back[1], back[0]) + math.pi / 2) % math.pi - math.pi / 2  # fingers close across the tape
    print(f"[demo] tape end seen in the box at {np.round(end, 3)}; pinch at {np.round(xy, 3)}, yaw {yaw:.2f}", flush=True)
    yield from Rt.grip(GRASP_OPEN, 0.3)
    yield from _move_turn(Rt, (xy[0], xy[1], HI), yaw, 1.5)
    yield from Rt.move((xy[0], xy[1], Z_OUT + 0.010), yaw=yaw, duration=0.8)
    _dbg_tcp(ctx, Rt, 1, "right at the box pinch")
    yield from Rt.grip(fc, 0.4)
    yield from Rt.move((xy[0], xy[1], TOP + BOX_D + 0.04), yaw=yaw, duration=0.8)
    yield from Rt.move((X_MARK - 0.02, xy[1] - 0.02, TOP + BOX_D + 0.04), yaw=yaw, duration=1.8)
    yield from Rt.move((X_MARK - 0.035, xy[1] - 0.04, TOP + 0.015), yaw=yaw, duration=0.8)
    yield from Rt.grip(fo, 0.4)
    q = Rt.tcp()
    yield from Rt.move((q[0], q[1], HI), yaw=yaw, duration=0.6)
    yield from Rt.move(PARK_R, yaw=GRAB_YAW, duration=1.0)


def _park_left(ctx, L):
    p = L.tcp()
    yield from L.move((p[0], p[1], HI), yaw=L._yaw, duration=0.5)
    yield from L.move((0.15, 0.22, HI + 0.05), yaw=0.0, duration=1.0)


def _dbg(ctx, L, tag):
    """Log only: the true tip depth along the run and each hand's distance to the nearest tape point."""
    st = ctx.states()
    pts = tape_points(st)
    g = grippers()
    d = [float(np.linalg.norm(pts - x, axis=1).min()) * 1000 for x in g] if g else []
    dd = " ".join(f"{v:.1f}" for v in d)
    print(f"[dbg t={ctx.t:6.2f}] {tag}: TRUE tip_s={tip_s(st):.3f}/{LC:.3f} hand-tape mm {dd}", flush=True)


def _log(ctx, tag=""):
    """Run-log line (simulator state printed for the log only; never used to choose a motion)."""
    st = ctx.states()
    print(f"[t={ctx.t:6.2f}] {tag}: TRUE tip_s={tip_s(st):.3f}/{LC:.3f} tip={np.round(tip(st), 3)} "
          f"threaded={threaded(st)} out={tip_out(st)} past_line={tip_past_mark(st)} rel={released(st)} "
          f"score={score(st):.2f}", flush=True)


def demo_script(ctx):
    """FT_DEMO=correct (default) | nothing | onearm_right (the right hand alone: the same feed strokes with no back
    hand holding the tape, then the draw-out) | partway (two-hand feed for 6 strokes, then both let go) | hold
    (correct, but the right hand keeps holding the tape at the end)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("FT_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, f"start mode={mode}")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "onearm_right":  # the right hand alone: the same strokes with no back hand, then the draw-out
        yield from _feed_one(ctx, Rt, FINGER_CLOSED, GRASP_OPEN)
        _log(ctx, f"fed one-handed: {len(_FEED['log'])} strokes, tape seen {_FEED['in_box']:.3f} into the box")
        yield from _draw_out(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    yield from _feed2(ctx, L, Rt, FINGER_CLOSED, GRASP_OPEN, stop_after=6 if mode == "partway" else None)
    _log(ctx, f"fed: {len(_FEED['log'])} strokes, tape seen {_FEED['in_box']:.3f} into the box")
    yield from _park_left(ctx, L)
    if mode == "partway":
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    if mode == "hold":
        p = _box_tape(ctx)
        yield from Rt.move((*GRAB, HI), yaw=GRAB_YAW, finger=FINGER_OPEN, duration=1.2)
        yield from Rt.move((*GRAB, Z_OUT + 0.010), yaw=GRAB_YAW, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.4)
        yield from Rt.move((X_MARK - 0.035, GRAB[1] - 0.04, TOP + 0.02), yaw=GRAB_YAW, duration=2.0)
        yield from Rt.wait(3.0)
        _log(ctx, f"end (held; {len(p)} tape pixels were seen in the box)")
        return
    yield from _draw_out(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
    yield from L.wait(2.0)
    _log(ctx, "end")
