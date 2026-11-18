"""On the station table, centred between the arms: two white cupboard walls standing parallel, front to back, with
a clear gap between their inner faces. A short yellow band is marked on the front edge of each wall. In front of
the walls, a spring tension rod lies across two small grey risers, pointing across the gap: a blue round tube with
a black rubber tip, and a thinner grey inner tube with a second black tip that slides into the outer tube
against a spring. The rod is longer than the gap; squeezed, it gets shorter.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
WALL_X0, WALL_X1 = 0.250, 0.420  # front and back edges of both walls
WALL_CX = (WALL_X0 + WALL_X1) / 2
GAP = 0.300  # between the inner faces
WALL_T, WALL_H = 0.020, 0.200
INNER = GAP / 2  # inner faces at y = +-0.150
BAND_Z0, BAND_Z1 = 0.120, 0.150  # the yellow band on each wall's front edge

WHITE, YELLOW, BLUE, GREY, BLACK = (0.94, 0.94, 0.92), (0.95, 0.80, 0.10), (0.15, 0.30, 0.80), (0.6, 0.6, 0.62), (
    0.08, 0.08, 0.08)

CUPBOARD = parts.Part(
    "cupboard",
    pos=(WALL_CX, 0.0, TOP),
    shapes=[
        *(parts.box((WALL_X1 - WALL_X0, WALL_T, WALL_H), at=(0.0, s * (INNER + WALL_T / 2), WALL_H / 2), color=WHITE)
          for s in (1, -1)),
        # The band: a 2 mm proud yellow strip on each wall's front edge (the face toward the robot).
        *(parts.box((0.002, WALL_T, BAND_Z1 - BAND_Z0),
                    at=(-(WALL_X1 - WALL_X0) / 2 - 0.001, s * (INNER + WALL_T / 2), (BAND_Z0 + BAND_Z1) / 2),
                    color=YELLOW)
          for s in (1, -1)),
    ],
    friction=(0.6, 0.5),  # painted board
)

# The rod, written at its free length (joint 0) in its own frame: origin at its middle, axis along y. Sizes are those
# of a 7/16 in (11.1 mm) round spring tension rod: an outer tube, a 13/32 in (10.3 mm) inner tube sliding in it, and a
# rubber tip 19 mm across on each end, standing 10 mm proud of the tube end.
TUBE_D, TUBE_L = 0.0111, 0.180
BAR_D, BAR_L = 0.0103, 0.170
CAP_D, CAP_T = 0.019, 0.010
CAP_SIZE = (CAP_D, CAP_T, CAP_D)  # a cap's own box (x, y, z) when level
FREE = 0.325  # cap face to cap face: 25 mm (about 1 in) longer than the gap
TRAVEL = 0.050  # squeezed fully: 275 mm
SPRING = 200.0  # N/m: fitted, squeezed 25 mm, each tip presses its wall with 5 N
TUBE_Y0 = FREE / 2 - CAP_T - TUBE_L  # outer tube from -0.0275 to +0.1525, its cap beyond (+y)
BAR_Y1 = -FREE / 2 + CAP_T + BAR_L  # inner tube from -0.1525 to +0.0175, its cap beyond (-y)
RISER_H = 0.030
ROD_START = (0.170, 0.0, TOP + RISER_H + TUBE_D / 2)  # outer tube resting on its riser, 30 mm up
SLIDE = parts.Slider(
    "slide",
    shapes=[
        parts.cylinder(BAR_D / 2, BAR_L, at=(0.0, (BAR_Y1 + -FREE / 2 + CAP_T) / 2, 0.0), color=GREY, axis="y"),
        parts.cylinder(CAP_D / 2, CAP_T, at=(0.0, -FREE / 2 + CAP_T / 2, 0.0), color=BLACK, axis="y"),
    ],
    axis=(0.0, 1.0, 0.0),  # squeezing moves the inner tube and its cap into the outer tube (+y)
    travel=TRAVEL,
    mass=0.04,
    spring=SPRING,
    damping=1.0,
)
ROD = parts.Part(
    "rod",
    pos=ROD_START,
    shapes=[
        parts.cylinder(TUBE_D / 2, TUBE_L, at=(0.0, TUBE_Y0 + TUBE_L / 2, 0.0), color=BLUE, axis="y"),
        parts.cylinder(CAP_D / 2, CAP_T, at=(0.0, FREE / 2 - CAP_T / 2, 0.0), color=BLACK, axis="y"),
    ],
    joints=[SLIDE],
    mass=0.06,  # outer tube and its cap; with the inner tube and cap, 100 g
    friction=(1.0, 0.8),  # rubber caps
)
RISER_XY = ((0.170, 0.030), (0.170, -0.060))  # under the outer tube, and under the inner tube clear of the outer one
RISERS = [Box(f"riser_{k}", pos=(*RISER_XY[i], TOP + RISER_H / 2), size=(0.040, 0.020, RISER_H), color=GREY)
          for i, k in enumerate(("tube", "bar"))]
OBJECTS = [CUPBOARD, ROD, *RISERS]

SUCCESS_HOLD_S = 1.0  # after the 10 s hold below: the 1.0 placement must also hold for 1 s more
SEAT = 0.0005  # m: a tip face within 0.5 mm of a wall's inner face touches it (contact offsets; a released, squeezed
# rod cannot rest with a gap, since only the walls can push back on its spring)
SQUEEZED = 0.005  # m: the rod at least 5 mm shorter than its free length, so its spring is pressing both caps
HOLD_S = 10.0  # s: hands off, holding itself up, before any credit
LOAD_AT, LOAD_N = 5.0, 2.94  # s into the hold, N: a 300 g test load hung at the rod's middle for the last 5 s
LOAD_RAMP = 0.5  # s: the load is hung and taken off gently, over 0.5 s each
SLIP = 0.002  # m: neither tip may move more than 2 mm under the test load
LEVEL = 0.010  # m: cap centres within 10 mm of the same height for full credit
SQUARE = 0.010  # m: cap centres within 10 mm of each other front to back for full credit
CLEAR = 0.003  # m: the rod's lowest point at least 3 mm above the table (held by the walls, not resting on it)
RELEASE_GAP = 0.005  # m: no part of either arm within 5 mm of the rod
PARTIAL = 0.5


def _rot(q):
    return parts._quat_to_matrix(q)


def _poses(rod):
    poses = {"base": (np.asarray(rod.pos, float), _rot(rod.quat_xyzw))}
    if "slide" in rod.links:
        lp, lq = rod.links["slide"]
        poses["slide"] = (np.asarray(lp, float), _rot(lq))
    return poses


def _caps(rod) -> tuple[np.ndarray, np.ndarray]:
    """World centres of the outer tube's cap (+y end) and the inner tube's cap (-y end)."""
    boxes = ROD.world_boxes(_poses(rod))
    caps = [c for _, c, _, s in boxes if np.allclose(s, CAP_SIZE)]
    return caps[0], caps[1]


def _on_walls(rod, still: bool = True) -> bool:
    """Both tip faces on the walls' inner faces with the spring squeezed, each tip centre on its wall's face (between
    its front and back edges, not above its top), and the rod clear of the table and (unless ``still`` is False)
    still. Nothing but the walls can then hold it (there is nothing between the walls but the table)."""
    a, b = _caps(rod)
    return bool(
        INNER - SEAT - 1e-6 <= rod.high[1] <= INNER + SEAT + 1e-6
        and -INNER - SEAT - 1e-6 <= rod.low[1] <= -INNER + SEAT + 1e-6
        and rod.joints.get("slide", 0.0) >= SQUEEZED - 1e-6
        and all(WALL_X0 - 1e-6 <= c[0] <= WALL_X1 + 1e-6 for c in (a, b))
        and all(c[2] <= TOP + WALL_H + 1e-6 for c in (a, b))
        and rod.low[2] >= TOP + CLEAR - 1e-6
        and (rod.resting() or not still)
    )


# -- release and hold: no part of the robot within 5 mm of the rod, and the rod holding itself up for 10 s --------
# In the simulator an on_step hook measures, every step, the rod's distance to every link of both arms, and times how
# long the rod has held itself up with no robot part near it. 5 s into that hold it hangs a 300 g test load (2.94 N, down)
# at the rod's middle for 5 s; if either cap moves more than 2 mm, or the rod stops holding itself up, the hold starts
# again. Before the first measurement after a reset everything fails closed (no credit). Offline (near_misses.py sets
# OFFLINE_FLAG; no simulator, no robot) each case stands for a state with the grippers away and the hold passed.
_GAP: dict[str, float] = {}  # "rod" -> closest distance to any arm link at the last step (m)
_HOLD: dict = {}  # "since": time the current hold began, "held": its length (s), "caps": cap centres at load-on
OFFLINE_FLAG = "TENSION_ROD_TASK_OFFLINE_CHECK"


def _released() -> bool:
    if "rod" not in _GAP:
        return _offline()
    return _GAP["rod"] >= RELEASE_GAP - 1e-6


def _held() -> bool:
    if "held" not in _HOLD:
        return _offline()
    return _HOLD["held"] >= HOLD_S - 1e-6


def hold_step(t: float, rod, released: bool) -> float:
    """Advance the hold clock to time ``t`` (s) for this rod state; return the test load to apply now (N, down).
    The load is hung gently: it rises over 0.5 s from 5 s into the hold and falls over the last 0.5 s before 10 s.
    While it is on, the rod need not be still (hanging a weight jiggles it), but neither tip may move more than 2 mm
    from where it was when the load went on; before and after, the rod must be still."""
    since = _HOLD.get("since")
    loading = since is not None and LOAD_AT - 1e-6 <= t - since < HOLD_S - 1e-6
    up = released and _on_walls(rod, still=not loading)
    if up and loading and "caps" in _HOLD:
        moved = max(float(np.linalg.norm(c - c0)) for c, c0 in zip(_caps(rod), _HOLD["caps"]))
        up = moved <= SLIP + 1e-6
    if not up:
        _HOLD.update(since=None, held=0.0)
        _HOLD.pop("caps", None)
        return 0.0
    if since is None:
        _HOLD["since"] = since = t
    held = _HOLD["held"] = t - since
    if loading:
        _HOLD.setdefault("caps", _caps(rod))
        return LOAD_N * max(0.0, min(1.0, (held - LOAD_AT) / LOAD_RAMP + 1 / 20 / LOAD_RAMP,
                                     (HOLD_S - held) / LOAD_RAMP))
    _HOLD.pop("caps", None)  # the load is off at 10 s; the hold has passed
    return 0.0


def _offline() -> bool:
    import os

    return os.environ.get(OFFLINE_FLAG) == "1"


_LOAD = {"n": 0.0}  # the load the simulator is applying (N)


def _apply_load(sim, newtons: float, rod) -> None:
    import torch

    asset = sim.env.unwrapped.scene["rod"]
    dev = asset.device
    a, b = (_caps(rod) if newtons else (np.zeros(3), np.zeros(3)))
    mid = (a + b) / 2 + sim.env.unwrapped.scene.env_origins[0].cpu().numpy()
    f = torch.tensor([[[0.0, 0.0, -float(newtons)]]], dtype=torch.float32, device=dev)
    p = torch.tensor([[list(map(float, mid))]], dtype=torch.float32, device=dev)
    ids = [list(asset.body_names).index("base")]
    asset.set_external_force_and_torque(f, torch.zeros_like(f), positions=p if newtons else None, body_ids=ids,
                                        env_ids=torch.tensor([0], device=dev), is_global=True)
    if not _LOAD["n"] or not newtons or newtons >= LOAD_N - 1e-9:
        print(f"[hold] t={sim.t:.2f}s test load {newtons:.2f} N", flush=True)
    _LOAD["n"] = newtons


@parts.on_reset
def _clear(sim):
    _GAP.clear()
    _HOLD.clear()
    if _LOAD["n"]:
        _apply_load(sim, 0.0, None)


# Distance from the robot to the rod: every link's collision hulls (the URDF's), sampled over their whole surface
# (faces, edges and corners, at most 2 mm apart) once and cached, then checked each step against the rod's boxes. A
# link face lying flat on the rod therefore reads as touching even when all its corners are far away. Sampling
# overstates a distance by at most 2 mm / sqrt(3), about 1.2 mm (every point of a face is that close to a sample).
SAMPLE = 0.002  # m


def _sample(tris: np.ndarray, step: float = SAMPLE) -> np.ndarray:
    """Points covering triangles (n, 3, 3): vertices, edges and faces, no two neighbours more than ``step`` apart."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    edge = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)], 0)
    n = np.maximum(1, np.ceil(edge / step)).astype(int)
    out = [tris.reshape(-1, 3)]
    for k in np.unique(n):
        sel = n == k
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing="ij")
        m = (i + j) <= k
        u, v = (i[m] / k)[None, :, None], (j[m] / k)[None, :, None]
        out.append((a[sel][:, None] + (b - a)[sel][:, None] * u + (c - a)[sel][:, None] * v).reshape(-1, 3))
    pts = np.concatenate(out)
    return np.unique(np.round(pts / 2e-4).astype(np.int64), axis=0) * 2e-4  # merge repeats within 0.2 mm


_SURF: dict[str, np.ndarray] = {}


def _arm_points() -> dict[str, np.ndarray]:
    """Surface samples of every link's collision hulls, in the link frame (the same hulls the URDF gives PhysX)."""
    if not _SURF:
        import xml.etree.ElementTree as ET

        from yam_arena import ASSET_DIR, YAM_URDF, analysis
        from yam_arena.urdf import rpy_matrix

        for link in ET.parse(YAM_URDF).getroot().iter("link"):
            pts = []
            for col in link.findall("collision"):
                origin = col.find("origin")
                r, t = np.eye(3), np.zeros(3)
                if origin is not None:
                    r = rpy_matrix(np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]))
                    t = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
                tris = analysis.read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename"))
                pts.append(_sample(tris @ r.T + t))
            if pts:
                _SURF[link.get("name")] = np.concatenate(pts)
    return _SURF


_COARSE: dict = {}
COARSE = 0.008  # m: one coarse point per 8 mm cube of samples; every sample is within 8 * sqrt(3) mm of one
NEAR = 0.010  # m: only distances under this are measured exactly (the let-go rule needs 5 mm)


def _coarse(name: str, pts: np.ndarray) -> np.ndarray:
    key = (name, len(pts), float(pts[0, 0]))
    if key not in _COARSE:
        _, idx = np.unique(np.floor(pts / COARSE).astype(np.int64), axis=0, return_index=True)
        _COARSE[key] = pts[idx]
    return _COARSE[key]


def robot_gap(link_poses: dict[str, np.ndarray], rod, points: dict[str, np.ndarray] | None = None) -> float:
    """Closest distance (m; negative inside) from any sampled robot surface point to any of the rod's boxes, exact
    below 10 mm (anything farther reads as at least 10 mm). ``link_poses``: link name -> 4x4 world pose.
    ``points``: link-frame samples (default: the arms' hulls)."""
    from yam_arena.analysis import box_signed_distance

    points = _arm_points() if points is None else points
    boxes = ROD.world_boxes(_poses(rod))

    def gap(w):
        return min(float(box_signed_distance(w, c, r, sz).min()) for _, c, r, sz in boxes) if len(w) else np.inf

    best = np.inf
    for name, t in link_poses.items():
        if name not in points:
            continue
        rot, off = t[:3, :3], t[:3, 3]
        if gap(_coarse(name, points[name]) @ rot.T + off) > NEAR + COARSE * 3 ** 0.5:
            best = min(best, NEAR)  # no sample of this link can be within 10 mm
            continue
        w = points[name] @ rot.T + off
        w = w[np.all((w >= rod.low - NEAR) & (w <= rod.high + NEAR), axis=1)]  # only points that could be near
        best = min(best, gap(w) if len(w) else NEAR)
    return best


@parts.on_step
def _record(sim):
    base = sim.env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    links = {}
    for arm in ("left_robot", "right_robot"):
        rob = base.scene[arm]
        poses = rob.data.body_link_pose_w
        poses = (poses.torch if hasattr(poses, "torch") else poses)[0].cpu().numpy().astype(float)
        for i, n in enumerate(rob.body_names):  # every link of the arm, base to fingertips
            t = np.eye(4)
            t[:3, :3] = _rot(poses[i, 3:])
            t[:3, 3] = poses[i, :3] - origin
            links[(arm, n)] = t
    rod = sim.states()["rod"]
    pts = _arm_points()
    _GAP["rod"] = min(robot_gap({n: t}, rod, {n: pts[n]}) if n in pts else np.inf for (_, n), t in links.items())
    was = _HOLD.get("held", 0.0)
    load = hold_step(float(sim.t), rod, _GAP["rod"] >= RELEASE_GAP - 1e-6)
    if load != _LOAD["n"]:
        _apply_load(sim, load, rod)
    if was < HOLD_S - 1e-6 <= _HOLD["held"]:
        print(f"[hold] t={sim.t:.2f}s held itself up for {HOLD_S:.0f} s, test load passed", flush=True)


def _bearing(rod) -> bool:
    """Holds itself up: on the walls (above), let go, and the 10 s hold with its test load passed."""
    return bool(_on_walls(rod) and _released() and _held())


def _held_across(rod) -> bool:
    """Holds itself up, and each cap wholly on its wall: centre at least a cap radius inside both edges, top not above
    the wall top."""
    a, b = _caps(rod)
    r = CAP_D / 2
    return bool(
        _bearing(rod)
        and all(WALL_X0 + r - 1e-6 <= c[0] <= WALL_X1 - r + 1e-6 for c in (a, b))
        and all(c[2] + r <= TOP + WALL_H + 1e-6 for c in (a, b))
    )


def _heights(rod):
    a, b = _caps(rod)
    return a[2] - TOP, b[2] - TOP


def success(objects):
    rod = objects["rod"]
    if not _held_across(rod):
        return False
    za, zb = _heights(rod)
    a, b = _caps(rod)
    return bool(
        all(BAND_Z0 - 1e-6 <= z <= BAND_Z1 + 1e-6 for z in (za, zb))
        and abs(za - zb) <= LEVEL + 1e-6
        and abs(a[0] - b[0]) <= SQUARE + 1e-6
    )


def score(objects):
    """1.0 fitted level and square in the band; 0.5 let go and holding itself up between the walls otherwise; else 0."""
    if success(objects):
        return 1.0
    return PARTIAL if _bearing(objects["rod"]) else 0.0


# -- demo: both arms grip the rod near its ends (fingers closing across it), squeeze it shorter, lift it to the band,
# carry it straight back between the walls and let go together.
TIP = 0.013
YAW = math.pi / 2  # fingers close along x, across the rod (turned -90 deg on the left arm, +90 on the right)
GRIP_Y = 0.135  # left arm on the outer tube at +y, right arm on the inner tube at -y
SQUEEZE = 0.050  # each arm moves this far inward; the arms give, so the rod shortens by less than 100 mm (the slide stops at 50 mm)
SAG_Z = 0.004
SHIFT_Y = 0.0035  # both arms carry the squeezed rod 3.5 mm to the left: in GPU traces it sat that far right
FIT_Z = (BAND_Z0 + BAND_Z1) / 2  # rod axis height when fitted
TCP_ABOVE_AXIS = TIP - (TUBE_D / 2 - 0.003)  # pad tips 3 mm above the tube's underside


def demo():
    x0, z0 = ROD_START[0], ROD_START[2]
    grip_z = z0 + TCP_ABOVE_AXIS
    fit_z = TOP + FIT_Z + TCP_ABOVE_AXIS + SAG_Z
    up_z = TOP + 0.12
    pre_x = WALL_X0 - CAP_D / 2 - 0.025  # caps 25 mm in front of the walls

    def arm(s):
        yaw = -s * YAW
        y0, y1 = s * GRIP_Y, s * (GRIP_Y - SQUEEZE) + SHIFT_Y
        return [
            Waypoint(1.5, (x0 - 0.06, y0, up_z), FINGER_OPEN, "in front of the rod", yaw),
            Waypoint(1.0, (x0, y0, up_z), FINGER_OPEN, "over the rod", yaw),
            Waypoint(1.5, (x0, y0, grip_z), FINGER_OPEN, "descend", yaw),
            Waypoint(0.8, (x0, y0, grip_z), FINGER_CLOSED, "grip", yaw),
            Waypoint(1.5, (x0, y0, fit_z), FINGER_CLOSED, "lift to the band", yaw),
            Waypoint(1.0, (pre_x, y0, fit_z), FINGER_CLOSED, "up to the walls", yaw),
            Waypoint(2.0, (pre_x, y1, fit_z), FINGER_CLOSED, "squeeze", yaw),
            Waypoint(2.0, (WALL_CX, y1, fit_z), FINGER_CLOSED, "carry between the walls", yaw),
            Waypoint(0.8, (WALL_CX, y1, fit_z), FINGER_CLOSED, "settle", yaw),
            Waypoint(0.8, (WALL_CX, y1, fit_z), FINGER_OPEN, "let go", yaw),
            Waypoint(0.7, (WALL_CX, y1, fit_z + 0.025), FINGER_OPEN, "up", yaw),
            Waypoint(1.0, (0.17, y1, fit_z + 0.025), FINGER_OPEN, "back out of the gap", yaw),
            Waypoint(0.8, (0.17, s * 0.20, fit_z + 0.025), FINGER_OPEN, "aside, clear of the walls", yaw),
            Waypoint(14.0, (0.17, s * 0.20, fit_z + 0.025), FINGER_OPEN, "wait while the rod holds", yaw),
        ]

    return {"left": arm(1), "right": arm(-1)}
