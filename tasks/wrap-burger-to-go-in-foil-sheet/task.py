"""A burger on a white prep board sits in the middle of a foil-laminated wrap sheet (foil side up, 37.5 cm front to
back along x, 25 cm left to right). The robot must wrap it for takeout: lift the sheet's front edge over the bun and
crease it, bring the back edge over that, then fold each side up against the bun and over the top, and let go.

The sheet is a reduced position-based model (sheet.py) stepped once per control step from the rigid simulator's
burger pose and the measured gripper poses and jaw openings; score() reads its end state.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --scorer task_score --instruction "Wrap the burger in the sheet for takeout."
"""

from __future__ import annotations

import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
WRAP_BURGER_TASK = True  # marks this module for the embodiment hooks below

# -- layout (m; x forward from the arm-base line, y left) --------------------------------------------------------
CX, CY = 0.28, 0.0
BOARD_LX, BOARD_LY, BOARD_T = 0.28, 0.16, 0.015
BOARD_TOP = TOP + BOARD_T
R, H, RR = 0.045, 0.05, 0.012  # bun radius, height, rounded crown edge (sheet contact shape)
BURGER_MASS = 0.12
SX, SY = 0.375, 0.25  # sheet

WHITE = (0.95, 0.95, 0.93)
BUN = (0.62, 0.30, 0.08)

OBJECTS = [
    Box("prep_board", pos=(CX, CY, TOP + BOARD_T / 2), size=(BOARD_LX, BOARD_LY, BOARD_T), color=WHITE),
    Cylinder("burger", pos=(CX, CY, BOARD_TOP + H / 2 + 0.0005), radius=R, height=H, mass=BURGER_MASS, color=BUN),
]

VARIATION = {"obj:burger": {"dx": 0.01, "dy": 0.01, "yaw": 3.14}}  # the sheet is laid centred on the burger
TWIN_EXTRA = [
    {"name": "wrap_sheet", "kind": "body", "size_mm": [375, 250, 0.05], "colour": "silver",
     "real": "foil-laminated paper sandwich wrap (dead-fold), cut to 375 x 250 mm", "pose": [CX, CY, 0.0153]},
]

# -- scoring constants -----------------------------------------------------------------------------------------------
MOVE_MAX = 0.03  # burger may shift at most 3 cm on the board
TILT_MAX = 20.0  # deg
CLEAR = 0.03  # grippers 3 cm clear of the sheet
OPEN_GAP = 0.02  # a jaw counts as open from a 2 cm finger gap
HOLD_S = 3.0  # the ladder must hold for 3 s after release
OVER_MIN = 0.8  # fraction of the middle 9 cm of the front / back edge on top of the bun
END_UP = 0.025  # height of the middle of a side edge above the board (half the bun)
END_HALF = 0.0125  # "middle": the central 2.5 cm of the side edge (median of its 3 nodes)
LADDER_EVERY = 4  # control steps between ladder evaluations while the hands are clear


def _quat_xyzw_to_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def burger_frame(pos_centre, quat_xyzw):
    """(base point, rotation) of the burger from its body-centre position and xyzw orientation."""
    Rm = _quat_xyzw_to_matrix(quat_xyzw)
    return np.asarray(pos_centre, float) - Rm @ np.array([0.0, 0.0, H / 2]), Rm


def tilt_deg(Rm) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(Rm[2, 2])))))


def wrap_state(x, faces, ids, rest_x0, rest_y0, bp, bR):
    """End-state geometry of the wrap. x: (N,3) sheet nodes; rest_x0/rest_y0: flat-sheet row/column offsets from
    the burger centre."""
    from sheet import coverage

    cov = coverage(x[faces], bp, bR, R, H, n_ring=36, n_h=5, n_top=7)
    crown = float((bp + bR @ np.array([0.0, 0.0, H]))[2])
    mid_cols = np.flatnonzero(np.abs(rest_y0) <= R)
    mid_rows = np.flatnonzero(np.abs(rest_x0) <= END_HALF + 1e-6)
    over = lambda e: float(np.mean(e[:, 2] > crown - 0.015))  # noqa: E731
    up = lambda e: float(np.median(e[:, 2]) - BOARD_TOP)  # noqa: E731
    return dict(cov=cov, near_over=over(x[ids[0, mid_cols]]), far_over=over(x[ids[-1, mid_cols]]),
                right_up=up(x[ids[mid_rows, 0]]), left_up=up(x[ids[mid_rows, -1]]))


def ladder(st) -> float:
    c = st["cov"]
    if c["top"] < 1.0:
        return 0.0
    s = 0.25
    if st["near_over"] >= OVER_MIN and st["far_over"] >= OVER_MIN and c["+x"] >= 1.0 and c["-x"] >= 1.0:
        s = 0.5
        ends = int(st["left_up"] >= END_UP) + int(st["right_up"] >= END_UP)
        if ends >= 1:
            s = 0.75
        if ends == 2 and min(c.values()) >= 1.0:
            s = 1.0
    return s


def _box_dist(p, T, lo, hi):
    """Distance of points p (N,3) to a box [lo, hi] in frame T (4x4)."""
    loc = (p - T[:3, 3]) @ T[:3, :3]
    d = np.maximum(np.maximum(lo - loc, loc - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def finger_boxes(tcp, rot, aperture):
    """Both pads of one gripper as boxes in its TCP frame (x: closing axis, z: approach): 12 mm pads, 22 mm wide,
    running from the pad tips (13 mm past the TCP) 6 cm back toward the wrist."""
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rot, tcp
    a = aperture / 2
    return [(T, np.array([a, -0.011, -0.06]), np.array([a + 0.012, 0.011, 0.013])),
            (T, np.array([-a - 0.012, -0.011, -0.06]), np.array([-a, 0.011, 0.013]))]


# -- the sheet, stepped from the embodiment ---------------------------------------------------------------------------
def _arr(t):
    t = t.torch if hasattr(t, "torch") else t
    return t.detach().cpu().numpy().astype(np.float64)


class Tracker:
    def __init__(self):
        self.S = None
        self.mesh = None
        self.decorated = False

    # sensors ---------------------------------------------------------------------------------------------------
    def _origin(self, base):
        return _arr(base.scene.env_origins[0])

    def _burger(self, base):
        pose = _arr(base.arena_world.get_pose_w("burger")[0])
        return burger_frame(pose[:3] - self._origin(base), pose[3:7])  # get_pose_w: position + xyzw

    def _hands(self, base):
        from yam_arena.runtime import link_poses, tcp_world

        o = self._origin(base)
        tcp, rot, ap = [], [], []
        for k in ("left_robot", "right_robot"):
            rob = base.scene[k]
            lp = link_poses(rob)
            g = lp["gripper"]
            z = g[:3, 2] / np.linalg.norm(g[:3, 2])
            xc = lp["tip_left"][:3, 3] - lp["tip_right"][:3, 3]
            xc = xc - z * (xc @ z)
            xc = xc / np.linalg.norm(xc) if np.linalg.norm(xc) > 1e-4 else g[:3, 0]
            rot.append(np.stack([xc, np.cross(z, xc), z], 1))
            tcp.append(np.asarray(tcp_world(rob), float) - o)
            ids = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
            q = _arr(rob.data.joint_pos)[0, ids]
            ap.append(float(abs(q[0]) + abs(q[1])))
        return np.array(tcp), np.array(rot), np.array(ap)

    # lifecycle -------------------------------------------------------------------------------------------------
    def reset(self, env):
        from sheet import DX, Sheet

        base = env.unwrapped
        self.dt = float(base.step_dt)
        if not getattr(self, "_said", False):
            import warp

            print(f"[wrap] sheet model on warp {warp.__version__}, numpy {np.__version__}", flush=True)
            self._said = True
        bp, bR = self._burger(base)
        self.bp0 = bp.copy()
        self.S = Sheet((bp[0], bp[1]), (SX, SY), BOARD_TOP + 0.0008,
                       board=dict(cx=CX, cy=CY, hx=BOARD_LX / 2, hy=BOARD_LY / 2, top=BOARD_TOP, t=BOARD_T),
                       burger=dict(r=R, h=H, rr=RR), table_z=TOP)
        self.rest_x0 = (np.arange(self.S.nx) - (self.S.nx - 1) / 2) * DX
        self.rest_y0 = (np.arange(self.S.ny) - (self.S.ny - 1) / 2) * DX
        tcp, rot, ap = self._hands(base)
        for _ in range(30):  # let the sheet settle onto the board and drape over its edges
            self.S.step(tcp, rot, ap, bp, bR, [b for k in range(2) for b in finger_boxes(tcp[k], rot[k], ap[k])],
                        dt=self.dt)
        self.t = 0.0
        self.n = 0
        self.moved = self.tipped = False
        self.clear_t = 0.0
        self.hist = deque()
        self.last = dict(ladder=0.0, why="start", clear=False, held=False)
        self.state = None
        self._tr0 = self.S.x.copy()
        self._render(base)
        self._decorate(base)

    def step(self, env):
        if self.S is None:
            return
        base = env.unwrapped
        bp, bR = self._burger(base)
        tcp, rot, ap = self._hands(base)
        boxes = [b for k in range(2) for b in finger_boxes(tcp[k], rot[k], ap[k])]
        self.S.step(tcp, rot, ap, bp, bR, boxes, dt=self.dt)
        self.t += self.dt
        self.n += 1
        self._trace(tcp, rot, ap, bp, bR)
        if np.hypot(*(bp[:2] - self.bp0[:2])) > MOVE_MAX:
            self.moved = True
        if tilt_deg(bR) > TILT_MAX:
            self.tipped = True
        held = bool(self.S.pins)
        x = self.S.x
        dist = min(float(_box_dist(x, T, lo, hi).min()) for T, lo, hi in boxes)
        clear = (not held) and bool(np.all(ap >= OPEN_GAP)) and dist >= CLEAR
        self.clear_t = self.clear_t + self.dt if clear else 0.0
        if not clear:
            self.hist.clear()
        elif self.n % LADDER_EVERY == 0 or not self.hist:
            self.state = wrap_state(x, self.S.faces, self.S.ids, self.rest_x0, self.rest_y0, bp, bR)
            self.hist.append((self.t, ladder(self.state)))
            while self.hist and self.hist[0][0] < self.t - HOLD_S:
                self.hist.popleft()
        self.last = dict(held=held, clear=clear, hand_dist=dist, aperture=ap.tolist())
        self._render(base)

    def _trace(self, tcp, rot, ap, bp, bR):
        """Debug only (WRAP_TRACE=path): record the sheet's inputs so a run can be replayed offline."""
        import os

        path = os.environ.get("WRAP_TRACE")
        if not path:
            return
        if self.n == 1:
            self._tr = {k: [] for k in ("tcp", "rot", "ap", "bp", "bR")}
        for k, v in zip(("tcp", "rot", "ap", "bp", "bR"), (tcp, rot, ap, bp, bR)):
            self._tr[k].append(np.array(v))
        if self.n % 50 == 0:
            np.savez_compressed(path, x0=self._tr0, **{k: np.array(v) for k, v in self._tr.items()})

    # evaluation ------------------------------------------------------------------------------------------------
    def gate(self) -> str:
        if self.S is None:
            return "no sheet"
        if self.S.torn or self.S.bad:
            return "sheet torn"
        if self.moved:
            return "burger moved"
        if self.tipped:
            return "burger tipped"
        if self.last.get("held"):
            return "gripper holds the sheet"
        if not self.last.get("clear"):
            return "grippers not open and clear"
        if self.clear_t < HOLD_S - 1e-6:
            return "released < 3 s ago"
        return ""

    def score(self) -> float:
        if self.gate() or not self.hist:
            return 0.0
        return float(min(v for _, v in self.hist))

    def summary(self) -> str:
        st = self.state or {}
        cov = {k: round(v, 3) for k, v in st.get("cov", {}).items()}
        return (f"gate={self.gate() or 'ok'} score={self.score()} pins={ {k: len(v) for k, v in self.S.pins.items()} } "
                f"cov={cov} near={st.get('near_over', 0):.2f} far={st.get('far_over', 0):.2f} "
                f"L={st.get('left_up', 0):.3f} R={st.get('right_up', 0):.3f} strain={self.S.max_strain:.2f} "
                f"clear_t={self.clear_t:.2f} hand_dist={self.last.get('hand_dist', 0):.3f}")

    # rendering -------------------------------------------------------------------------------------------------
    def _render(self, base):
        """Sheet as two meshes sharing its nodes: a 2.5 cm green-and-silver check printed on the foil (as on
        checker-print burger wraps), so it reads as a wrap sheet, not plain foil or the white board."""
        try:
            from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
            from isaaclab.sim.utils.stage import get_current_stage

            pts = Vt.Vec3fArray.FromNumpy((self.S.x + self._origin(base)).astype(np.float32))
            if self.mesh is None or not self.mesh[0].GetPrim().IsValid():
                stage = get_current_stage()
                f = self.S.faces
                i, j = f // self.S.ny, f % self.S.ny
                cell = ((i.min(1) // 2) + (j.min(1) // 2)) % 2
                self.mesh = []
                for k, (rgb, metal, rough) in enumerate((((0.86, 0.87, 0.89), 0.7, 0.3), ((0.10, 0.48, 0.22), 0.2, 0.5))):
                    m = UsdGeom.Mesh.Define(stage, f"/World/WrapSheet_{k}")
                    m.CreateDoubleSidedAttr(True)
                    m.CreateSubdivisionSchemeAttr("none")
                    fk = f[cell == k]
                    m.GetFaceVertexCountsAttr().Set([3] * len(fk))
                    m.GetFaceVertexIndicesAttr().Set(fk.ravel().tolist())
                    mat = UsdShade.Material.Define(stage, f"/World/WrapSheetLook_{k}")
                    sh = UsdShade.Shader.Define(stage, f"/World/WrapSheetLook_{k}/Shader")
                    sh.CreateIdAttr("UsdPreviewSurface")
                    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
                    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
                    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
                    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
                    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
                    self.mesh.append(m)
            for m in self.mesh:
                m.GetPointsAttr().Set(pts)
        except Exception as e:  # noqa: BLE001
            if not getattr(self, "_render_warned", False):
                print(f"[wrap] sheet render failed: {e!r}", flush=True)
                self._render_warned = True

    def _decorate(self, base):
        """Bun, lettuce, patty and cheese bands and sesame seeds (visual only, children of the burger body, inside its collision shape)."""
        if self.decorated:
            return
        self.decorated = True
        try:
            _decorate_burger(base)
        except Exception as e:  # noqa: BLE001
            print(f"[wrap] burger decoration failed: {e!r}", flush=True)


def _lathe(profile, n=48):
    """Triangle mesh of a surface of revolution; profile = [(rho, z), ...] from bottom to top."""
    pts, faces = [], []
    m = len(profile)
    for k in range(n):
        a = 2 * math.pi * k / n
        for rho, z in profile:
            pts.append((rho * math.cos(a), rho * math.sin(a), z))
    for k in range(n):
        k2 = (k + 1) % n
        for i in range(m - 1):
            a0, a1, b0, b1 = k * m + i, k * m + i + 1, k2 * m + i, k2 * m + i + 1
            faces += [(a0, b0, b1), (a0, b1, a1)]
    return np.array(pts), np.array(faces)


def _bands():
    zc = -H / 2  # body frame origin is the cylinder centre
    arc = [(R - RR + RR * math.cos(t), H - RR + RR * math.sin(t)) for t in np.linspace(0, math.pi / 2, 7)]
    return {
        "heel": ([(0.0, 0.0), (R - 0.004, 0.0), (R - 0.0012, 0.0012), (R, 0.004), (R, 0.014)], (0.86, 0.60, 0.30), zc),
        "lettuce": ([(R, 0.014), (R, 0.018)], (0.25, 0.65, 0.12), zc),
        "patty": ([(R, 0.018), (R, 0.027)], (0.22, 0.10, 0.05), zc),
        "cheese": ([(R, 0.027), (R, 0.031)], (1.00, 0.72, 0.02), zc),
        "crown": ([(R, 0.031), *arc, (0.0, H)], (0.62, 0.30, 0.08), zc),
    }


def _decorate_burger(base):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    path = base.scene.rigid_objects["burger"].cfg.prim_path.replace("env_[^/]+", "env_0").replace("env_.*", "env_0")
    root = stage.GetPrimAtPath(path)
    if not root or not root.IsValid() or stage.GetPrimAtPath(f"{path}/look_crown").IsValid():
        return
    for p in Usd.PrimRange(root):  # hide the plain collision cylinder's own look; it still collides
        if p != root and p.IsA(UsdGeom.Gprim):
            UsdGeom.Imageable(p).MakeInvisible()

    def look(name, rgb, rough=0.6):
        mat = UsdShade.Material.Define(stage, f"{path}/Looks_{name}")
        sh = UsdShade.Shader.Define(stage, f"{path}/Looks_{name}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    for name, (prof, rgb, zc) in _bands().items():
        pts, faces = _lathe(prof)
        pts[:, 2] += zc
        mesh = UsdGeom.Mesh.Define(stage, f"{path}/look_{name}")
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
        mesh.GetFaceVertexCountsAttr().Set([3] * len(faces))
        mesh.GetFaceVertexIndicesAttr().Set(faces.ravel().tolist())
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look(name, rgb))
    seed = look("seed", (0.97, 0.93, 0.80), 0.4)
    rng = np.random.default_rng(7)
    for k in range(36):  # sesame seeds on the crown (0.8 mm proud, inside the sheet's contact offset)
        rho, a = 0.031 * math.sqrt(rng.uniform(0.02, 1.0)), rng.uniform(0, 2 * math.pi)
        s = UsdGeom.Sphere.Define(stage, f"{path}/look_seed_{k}")
        s.CreateRadiusAttr(1.0)
        xf = UsdGeom.Xformable(s)
        xf.AddTranslateOp().Set(Gf.Vec3d(rho * math.cos(a), rho * math.sin(a), H / 2))
        xf.AddRotateZOp().Set(float(rng.uniform(0, 360)))
        xf.AddScaleOp().Set(Gf.Vec3f(0.0035, 0.0018, 0.0008))
        UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(seed)


# -- embodiment hooks (shared through the class: the task module can be loaded more than once) ------------------------
def tracker():
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return None
    return getattr(ie.YamArenaEmbodiment, "_wrap_tracker", None)


def _ours(emb) -> bool:
    task = getattr(emb, "task", None)
    return bool(task is not None and getattr(getattr(task, "module", None), "WRAP_BURGER_TASK", False))


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_wrap_hooked", False):
        return
    cls._wrap_tracker = Tracker()
    base_reset, base_step = cls.reset, cls._task_step

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        if not _ours(self):
            return obs
        cls._wrap_tracker.reset(self._env)
        for _ in range(2):  # two more settle steps at READY so the cameras' buffers show the new sheet and decorations
            self._env.step(self._ready)
            cls._wrap_tracker.step(self._env)
        for _ in range(ie.RESET_RENDERS):
            self._base.sim.render()
        return self._observation()

    def _task_step(self, *a, **k):
        if _ours(self):
            cls._wrap_tracker.step(self._env)  # move the sheet before the step is scored
        return base_step(self, *a, **k)

    cls.reset, cls._task_step, cls._wrap_hooked = reset, _task_step, True


_install_hooks()


# -- scoring ---------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    tr = tracker()
    return tr.score() if tr is not None else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["burger"].resting(max_speed=0.02)


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    tr = tracker()
    if tr is None or tr.S is None:
        return {"sheet": 0}
    st = tr.state or {}
    cov = st.get("cov", {})
    return {"crown_covered": float(cov.get("top", 0.0) >= 1.0),
            "front_back_over": float(st.get("near_over", 0) >= OVER_MIN and st.get("far_over", 0) >= OVER_MIN),
            "ends_up": 0.5 * (int(st.get("left_up", 0) >= END_UP) + int(st.get("right_up", 0) >= END_UP)),
            "released_clear": float(tr.clear_t >= HOLD_S), "intact": float(not tr.gate().startswith(("sheet", "burger")))}
