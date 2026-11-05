"""On the station table, a yellow taped walkway stripe (150 mm wide) runs left-right across the whole table, 22.5 cm
in front of the robot. A grey junction box stands just in front of the walkway's near edge, left of centre. A red
"live" cable (8 mm, 30 cm, black 35x16x12 mm plug on its free end) leaves the junction box and lies straight and
diagonally across the walkway (heading forward and to the right), with its plug just past the far edge. On the right,
in front of the right arm, a green taped rectangle (10 x 12 cm) marks the cable parking zone, on the near side of the
walkway. A free blue insulated stick (hollow square fibreglass rod, 300 x 20 x 20 mm, 60 g) lies on the right,
parallel to the robot's forward axis. Using only the stick, the robot must push the cable completely off the walkway
so that its plug ends up inside the green zone; the grippers (and the rest of the arms) must never touch the cable
or its plug.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
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

INSTRUCTION = ("The red cable is live. Without touching the cable or its plug with the grippers or any part of the "
               "arms, use the blue insulated stick to push the whole cable off the yellow walkway so that its plug "
               "ends up inside the green parking zone.")

# -- walkway: a flush yellow tape stripe across the table (x = forward) ---------------------------------------------
WALK_X0, WALK_X1 = 0.225, 0.375  # near and far edge (150 mm wide)
WALK_HALF_Y = 0.48
TAPE_T = 0.0006  # tape thickness: flush, the cable and stick slide over it
WALKWAY = Box("walkway", pos=((WALK_X0 + WALK_X1) / 2, 0.0, TOP + TAPE_T / 2),
              size=(WALK_X1 - WALK_X0, 2 * WALK_HALF_Y, TAPE_T), color=(0.98, 0.80, 0.05))

# -- junction box and live cable --------------------------------------------------------------------------------------
R = 0.004  # 8 mm cable
CORD_LEN = 0.30
A = np.array([0.17, 0.12])  # cable leaves the junction box here (5.5 cm before the walkway's near edge)
CORD_YAW = math.radians(-40.0)  # heading forward-right, diagonally across the walkway
DIR0 = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW)])
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.010, color=(0.06, 0.06, 0.06))
CORD = cable("cord", start=(float(A[0]), float(A[1]), TOP + R + 0.0005), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=PLUG, color=(0.85, 0.06, 0.04), bend_stiffness=0.01, bend_damping=0.001)
JBOX_SIZE = (0.05, 0.07, 0.04)  # x along the cable exit, y across it
_jc = A - DIR0 * (0.006 + JBOX_SIZE[0] / 2)
JBOX = Box("junction_box", pos=(float(_jc[0]), float(_jc[1]), TOP + JBOX_SIZE[2] / 2), size=JBOX_SIZE,
           yaw=CORD_YAW, color=(0.55, 0.57, 0.60))

# -- parking zone: green tape outline (inner rectangle) on the near side, in front of the right arm ------------------
ZONE_X0, ZONE_X1 = 0.115, 0.215
ZONE_Y0, ZONE_Y1 = -0.255, -0.135
ZT = 0.012  # tape width (outside the zone)
_GREEN = (0.10, 0.65, 0.20)
ZONE = [
    Box("zone_tape_near", pos=(ZONE_X0 - ZT / 2, (ZONE_Y0 + ZONE_Y1) / 2, TOP + TAPE_T / 2),
        size=(ZT, ZONE_Y1 - ZONE_Y0 + 2 * ZT, TAPE_T), color=_GREEN),
    Box("zone_tape_far", pos=(ZONE_X1 + ZT / 2, (ZONE_Y0 + ZONE_Y1) / 2, TOP + TAPE_T / 2),
        size=(ZT, ZONE_Y1 - ZONE_Y0 + 2 * ZT, TAPE_T), color=_GREEN),
    Box("zone_tape_left", pos=((ZONE_X0 + ZONE_X1) / 2, ZONE_Y1 + ZT / 2, TOP + TAPE_T / 2),
        size=(ZONE_X1 - ZONE_X0, ZT, TAPE_T), color=_GREEN),
    Box("zone_tape_right", pos=((ZONE_X0 + ZONE_X1) / 2, ZONE_Y0 - ZT / 2, TOP + TAPE_T / 2),
        size=(ZONE_X1 - ZONE_X0, ZT, TAPE_T), color=_GREEN),
]

# -- insulated stick (free) --------------------------------------------------------------------------------------------
STICK_SIZE = (0.30, 0.02, 0.02)
STICK_XY = (0.33, -0.315)
STICK = Box("stick", pos=(STICK_XY[0], STICK_XY[1], TOP + STICK_SIZE[2] / 2 + 0.001), size=STICK_SIZE, mass=0.06,
            color=(0.10, 0.30, 0.85))

OBJECTS = [WALKWAY, *ZONE, JBOX, CORD, STICK]

# -- robot-cable contact monitor ---------------------------------------------------------------------------------------
# A PhysX rigid-contact view (the mechanism behind Isaac Lab's ContactSensor) is read after EVERY physics substep
# (240 Hz): its sensors are all robot links (the YAM USD has contact reporting on every link) and its filters are
# every cable segment and the plug. Any robot-link/cable contact point with a separation <= TOUCH_SEP marks the
# episode as "touched". Only robot<->cable pairs are in the view, so the stick touching the fingers (or the cable)
# never triggers it, and since every substep is checked, fast motions cannot skip a touch.
TOUCH_SEP = 0.001  # m
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
_MON: dict = {"touched": False, "view": None, "first": None, "min_sep": float("inf"), "polls": 0, "hits": 0,
              "err": None}


def _reset_monitor() -> None:
    _MON.update(touched=False, first=None, min_sep=float("inf"), polls=0, hits=0)


def _enable_contact_processing() -> None:
    """Isaac Lab turns PhysX contact processing off unless a ContactSensor exists (this embodiment builds none);
    switch it back on, exactly as Isaac Lab's own ContactSensor does."""
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover  (outside Isaac Sim)
        pass


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _make_view(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    sensors, filters = [], []
    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" not in p:
            continue
        if any(r in p for r in _ROBOT_PRIMS) and "PhysxContactReportAPI" in prim.GetAppliedSchemas():
            sensors.append(p)
        elif "/cord/" in p and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            filters.append(p)
    if not sensors or not filters:
        raise RuntimeError(f"live-wire contact monitor: {len(sensors)} robot links, {len(filters)} cable bodies")
    view = sv.create_rigid_contact_view(sensors, filter_patterns=[filters] * len(sensors),
                                        max_contact_data_count=64 * len(sensors) * len(filters))
    _MON["view"], _MON["stage"] = view, stage
    print(f"[live-wire] contact monitor: {len(sensors)} robot links x {len(filters)} cable bodies "
          f"(e.g. {sensors[-1].rsplit('/', 2)[-2:]}, {filters[-1].rsplit('/', 1)[-1]})", flush=True)


def _poll_contacts(mgr) -> None:
    """Called after every physics substep."""
    try:
        if _MON["view"] is None or _MON.get("stage") is None:
            _make_view(mgr)
        view = _MON["view"]
        if view is None:
            return
        _, _, _, sep, counts, starts = view.get_contact_data(dt=mgr.get_physics_dt())
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[live-wire] contact monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    counts = _np(counts).reshape(-1)
    if not counts.any():
        return
    starts = _np(starts).reshape(-1)
    sep = _np(sep).reshape(-1)
    idx = np.concatenate([np.arange(s, s + c) for s, c in zip(starts[counts > 0], counts[counts > 0])])
    m = float(sep[idx].min())
    _MON["hits"] += 1
    _MON["min_sep"] = min(_MON["min_sep"], m)
    if m <= TOUCH_SEP and not _MON["touched"]:
        _MON["touched"] = True
        print(f"[live-wire] robot touched the cable (separation {m * 1000:.2f} mm)", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_live_wire_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._live_wire_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def _patch_physics() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())


def _install_hooks() -> None:
    _patch_physics()
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_live_wire_hooked", False):
        return
    base_reset = cls.reset

    def reset(self, *a, **k):
        _MON["view"] = None  # rebuilt on the next physics step (the env may have been rebuilt)
        out = base_reset(self, *a, **k)
        _reset_monitor()  # the settle steps at reset hold the arms at READY, far from the cable
        return out

    reset._live_wire_hooked = True
    cls.reset = reset


_install_hooks()


def touched() -> bool:
    return bool(_MON["touched"])


# -- scoring -----------------------------------------------------------------------------------------------------------
ON_TABLE_Z = TOP + 0.02
MIN_GAIN = 0.03  # m of cable that must have left the walkway before any partial credit


def _dense(poly: np.ndarray, step: float = 0.002) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _over_walkway(x: np.ndarray, half_w: float) -> np.ndarray:
    return (x + half_w > WALK_X0) & (x - half_w < WALK_X1)


def _on_len(poly: np.ndarray, plug: np.ndarray | None, plug_dir: np.ndarray) -> float:
    pts = _dense(poly)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    mid = (pts[1:] + pts[:-1]) / 2
    on = float(seg[_over_walkway(mid[:, 0], R)].sum())
    if plug is not None:
        ps = np.array([plug + plug_dir * PLUG.size[0] * (t - 0.5) for t in np.linspace(0, 1, 8)])
        on += PLUG.size[0] * float(_over_walkway(ps[:, 0], PLUG.size[1] / 2).mean())
    return on


def length_on_walkway(objects) -> float:
    """Length (m) of cable + plug that overlaps the walkway stripe in top view (cable surface or plug body over the
    tape; height ignored, so a cable held up above the walkway still counts)."""
    c = objects["cord"]
    return _on_len(c.polyline(), c.end_plug, c.end_dir)


def _initial_on_walkway() -> float:
    pts = CORD.start_points()
    z = pts[0, 2]
    poly = np.vstack([np.r_[A, z], pts, np.r_[A + DIR0 * CORD_LEN, z]])
    plug = np.r_[A + DIR0 * (CORD_LEN + PLUG.size[0] / 2), z]
    return _on_len(poly, plug, np.r_[DIR0, 0.0])


L0 = _initial_on_walkway()  # cable length on the walkway at the start


def plug_in_zone(objects) -> bool:
    p = objects["cord"].end_plug
    return bool(p is not None and ZONE_X0 <= p[0] <= ZONE_X1 and ZONE_Y0 <= p[1] <= ZONE_Y1)


def _on_table(objects) -> bool:
    return bool(objects["cord"].points[:, 2].max() <= ON_TABLE_Z)


def score(objects) -> float:
    """0 if a robot link ever touched the cable or plug. Full 1.0: nothing of the cable or plug over the walkway,
    plug centre inside the green zone, cable lying on the table. Otherwise partial credit 0.8 x (fraction of the
    initially covered walkway length that has been cleared), given only once at least 3 cm has been cleared."""
    if touched():
        return 0.0
    on = length_on_walkway(objects)
    if on <= 0.0 and plug_in_zone(objects) and _on_table(objects):
        return 1.0
    gain = L0 - on
    if gain < MIN_GAIN:
        return 0.0
    return round(0.8 * min(gain / L0, 1.0), 4)


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Cable length still on the walkway (m); 1 m once the robot has touched the cable."""
    return 1.0 if touched() else length_on_walkway(objects)


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------------

STICK_GRIP_FROM_END = 0.015  # TCP this far from the stick's +x end
GRIP_OVER = 0.012  # TCP above the stick centre when holding it (pad tips 1 mm below the centre)
SWEEP_LIFT = 0.003  # stick held this far above the table while sweeping
RG = 0.375  # gripper distance from the cable's anchor while sweeping (beyond the plug)


def _cord_heading(st) -> float:
    p = st["cord"].end_plug
    return math.atan2(p[1] - A[1], p[0] - A[0])


def _sweep_pose(th: float, off: float, rg: float = RG) -> tuple[float, float]:
    d = np.array([math.cos(th), math.sin(th)])
    n = np.array([-math.sin(th), math.cos(th)])  # side the cable moves away from when th decreases
    p = A + rg * d + off * n
    return float(p[0]), float(p[1])


def demo_script(ctx):
    """Right arm picks the stick up by its far end, lays it alongside the cable on its far (+x) side with the
    gripper beyond the plug, and sweeps it round the junction box (clockwise) so the stick's side pushes the
    cable off the walkway; the plug ends in the green zone. Closed loop: each sweep step reads the live plug and
    keeps the stick's side against the cable."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    Rt, L = ctx.right, ctx.left
    cheat = os.environ.get("LIVEWIRE_CHEAT", "")  # test only: "none" | "hand" | "sweep_hand"
    hi = TOP + 0.07

    def log(tag):
        st = ctx.states()
        p = st["cord"].end_plug
        print(f"[demo t={ctx.t:5.1f}] {tag}: on_walk={length_on_walkway(st):.3f}/{L0:.3f} zone={plug_in_zone(st)} "
              f"touched={touched()} minsep={_MON['min_sep']:.4f} score={score(st):.3f} "
              f"plug={np.round(p, 3)} head={math.degrees(_cord_heading(st)):.1f} "
              f"stick={np.round(st['stick'].pos, 3)} polls={_MON['polls']} hits={_MON['hits']}", flush=True)

    log("start")
    if cheat == "none":
        yield from Rt.wait(4.0)
        log("end")
        return
    if cheat == "hand":  # grasp the plug by hand and carry it into the zone
        st = ctx.states()
        p = st["cord"].end_plug
        h = math.atan2(*(st["cord"].end_dir[[1, 0]]))
        gy = (h + math.pi / 2) % math.pi - math.pi / 2
        yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED)
        log("plug grasped")
        yield from Rt.move((p[0], p[1], hi), yaw=gy, duration=0.8)
        for th in np.radians(np.arange(-46, -91, -6)):
            q = A + 0.32 * np.array([math.cos(th), math.sin(th)])
            yield from Rt.move((q[0], q[1], hi), yaw=float(np.clip(th + math.pi / 2, -1.5, 1.5)), duration=0.4)
        zc = ((ZONE_X0 + ZONE_X1) / 2, (ZONE_Y0 + ZONE_Y1) / 2)
        yield from Rt.move((zc[0], zc[1], TOP + 0.025), yaw=0.0, duration=1.0)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((zc[0], zc[1], hi), yaw=0.0, duration=0.8)
        yield from Rt.move((0.20, -0.33, hi), yaw=0.0, duration=0.8)
        yield from Rt.wait(1.0)
        log("end")
        return

    sweep_z = TOP + STICK_SIZE[2] / 2 + SWEEP_LIFT + GRIP_OVER
    if cheat != "sweep_hand":
        # 1) pick up the stick by its +x end
        st = ctx.states()
        s = st["stick"].pos
        gx = s[0] + STICK_SIZE[0] / 2 - STICK_GRIP_FROM_END
        yield from Rt.move((gx, s[1], hi), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move((gx, s[1], s[2] + GRIP_OVER), yaw=0.0, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED)
        yield from Rt.move((gx, s[1], hi), yaw=0.0, duration=0.8)
        log("stick lifted")
    else:  # same sweep, but pushing the cable's middle with the closed bare fingers at cable height
        yield from Rt.grip(FINGER_CLOSED)
        sweep_z = TOP + 0.016
    rg = RG if cheat != "sweep_hand" else 0.24

    # 2) lay the stick alongside the cable on its far side, gripper beyond the plug
    th = _cord_heading(ctx.states())
    off0 = 0.032
    x, y = _sweep_pose(th, off0, rg)
    yield from Rt.move((x, y, hi), yaw=th, duration=1.5)
    yield from Rt.move((x, y, sweep_z), yaw=th, duration=1.0)
    log("stick down")

    # 3) sweep clockwise round the junction box; the stick's side pushes the cable
    off = R + STICK_SIZE[1] / 2 + 0.002
    target = math.radians(-90.0)
    step = math.radians(4.0)
    while th > target + 1e-6:
        th = max(th - step, target)
        x, y = _sweep_pose(th, off, rg)
        yield from Rt.move((x, y, sweep_z), yaw=th, duration=0.35)
    yield from Rt.wait(0.5)
    log("swept")

    # 4) back the stick off the cable, lift it and put it back down to the right
    x, y = _sweep_pose(th, off + 0.03, rg)
    yield from Rt.move((x, y, sweep_z), yaw=th, duration=0.6)
    yield from Rt.move((x, y, hi), yaw=th, duration=0.6)
    if cheat != "sweep_hand":
        yield from Rt.move((0.33, -0.33, hi), yaw=0.0, duration=1.2)
        yield from Rt.move((0.33, -0.33, TOP + STICK_SIZE[2] / 2 + GRIP_OVER + 0.004), yaw=0.0, duration=0.8)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((0.33, -0.33, hi), yaw=0.0, duration=0.6)
    log("released")
    yield from Rt.wait(1.5)
    log("end")
