"""On the station table: a two-piece patch-cord tester. The MAIN unit (a dark blue block, 80 mm deep x 70 mm wide x
40 mm tall, fixed) stands on the left and the REMOTE unit (same size, orange, fixed) on the right; their test jacks
(rectangular openings 17.5 x 15.5 mm, 26 mm deep, at table level) face each other across a 41 cm gap. A loose
(unanchored) 6 mm grey patch cord, 40 cm long, with an identical dark grey rigid plug (55 x 16 x 14 mm) on EACH end,
lies straight across the table in front of the tester, parallel to the gap, with each plug pointing towards one unit.
The cord is only ~5 cm longer than the straight line between the two seated plugs, so the robot has to bring both ends
over together and seat the second plug without pulling the first one out: both plugs fully into the jacks (either end
into either unit), then let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var BOTHENDS_DEMO): correct (default) | nothing | near | hold | one.
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

INSTRUCTION = ("Connect the loose patch cord to the cable tester for certification: push one plug fully into the jack "
               "of the main unit (blue, left) and the other plug fully into the jack of the remote unit (orange, "
               "right), without pulling the first plug out, then let go of both plugs.")

BLUE = (0.12, 0.24, 0.62)
ORANGE = (0.93, 0.50, 0.10)
PLUG_GREY = (0.16, 0.16, 0.17)
CORD_GREY = (0.62, 0.64, 0.66)
JACK_GREY = (0.30, 0.31, 0.33)

# -- geometry (x forward, y left) ----------------------------------------------------------------------------------
PL, PW, PH = 0.055, 0.016, 0.014  # plug body: length along the cord, width, height
CAV_W, CAV_H, CAV_D = PW + 0.004, PH + 0.0025, 0.026  # jack opening (along x, z) and depth (along y): 20 x 16.5 x 26 mm
PORT_X = 0.36  # jack centre line (x)
FACE_Y = 0.205  # jack faces at y = +FACE_Y (main, faces -y) and y = -FACE_Y (remote, faces +y)
UNIT_X, UNIT_Y, UNIT_H = 0.08, 0.07, 0.040  # each unit body's footprint (x, y) and height
ADAPTER_T = 0.004  # jack adapter wall/roof thickness
ADAPTER_W, ADAPTER_H, ADAPTER_LEN = CAV_W + 2 * ADAPTER_T, CAV_H + ADAPTER_T, 0.040  # adapter sleeve (x, z, along y)
R = 0.003  # 6 mm cord
CORD_LEN = 0.40
CORD_X = 0.22  # the cord's starting line (x)
PLUG_Z = TOP + PH / 2 + 0.0005  # cord and plug centre height at spawn (plugs rest on the table)

CORD = cable("cord", start=(CORD_X, CORD_LEN / 2, PLUG_Z), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             anchor_start=False, start_plug=Plug(size=(PL, PW, PH), mass=0.015, color=PLUG_GREY),
             end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=PLUG_GREY), color=CORD_GREY,
             bend_stiffness=0.01, bend_damping=0.001)

UNITS = {"main": +1.0, "remote": -1.0}  # sign of y of each unit
UNIT_COLOR = {"main": BLUE, "remote": ORANGE}


def _unit(name: str) -> list[Box]:
    """The unit body plus its jack adapter: a narrow sleeve (cavity, roof, two side walls, solid back) that sticks out
    of the body's inner side face towards the other unit, with the jack opening at its tip."""
    s, c = UNITS[name], UNIT_COLOR[name]
    y0 = s * FACE_Y  # jack face (tip of the adapter)
    body_y0 = y0 + s * ADAPTER_LEN
    return [
        Box(f"{name}_body", pos=(PORT_X, body_y0 + s * UNIT_Y / 2, TOP + UNIT_H / 2), size=(UNIT_X, UNIT_Y, UNIT_H),
            color=c),
        Box(f"{name}_jack_back", pos=(PORT_X, y0 + s * (CAV_D + (ADAPTER_LEN - CAV_D) / 2), TOP + ADAPTER_H / 2),
            size=(ADAPTER_W, ADAPTER_LEN - CAV_D, ADAPTER_H), color=JACK_GREY),
        Box(f"{name}_jack_roof", pos=(PORT_X, y0 + s * CAV_D / 2, TOP + (CAV_H + ADAPTER_H) / 2),
            size=(ADAPTER_W, CAV_D, ADAPTER_H - CAV_H), color=JACK_GREY),
        Box(f"{name}_jack_wall_f", pos=(PORT_X + (CAV_W + ADAPTER_T) / 2, y0 + s * CAV_D / 2, TOP + CAV_H / 2),
            size=(ADAPTER_T, CAV_D, CAV_H), color=JACK_GREY),
        Box(f"{name}_jack_wall_n", pos=(PORT_X - (CAV_W + ADAPTER_T) / 2, y0 + s * CAV_D / 2, TOP + CAV_H / 2),
            size=(ADAPTER_T, CAV_D, CAV_H), color=JACK_GREY),
    ]


TESTER = [b for u in UNITS for b in _unit(u)]
OBJECTS = [CORD, *TESTER]

# -- scoring -------------------------------------------------------------------------------------------------------
SEATED_MIN = 0.015  # m: a plug counts when its tip is at least 15 mm inside a jack
RELEASE_DIST = 0.045  # m: both grippers' TCP at least this far from both plug centres = let go
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see where the grippers are."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_bothends_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._bothends_hooked = True
        cls._task_step = _task_step


_install_hooks()


def _plugs(objects) -> list[tuple[np.ndarray, np.ndarray]]:
    """(centre, unit axis pointing out of the cord through the plug tip) for both plugs."""
    st = objects[CORD.name]
    out = []
    for p, seg in ((st.start_plug, st.points[0]), (st.end_plug, st.points[-1])):
        p = np.asarray(p, dtype=float)
        a = p - np.asarray(seg, dtype=float)
        out.append((p, a / (np.linalg.norm(a) + 1e-12)))
    return out


def depth_in(unit: str, plug: tuple[np.ndarray, np.ndarray]) -> float:
    """How far (m) the plug's tip is inside ``unit``'s jack; -1 if the plug is not lined up in the jack opening
    (centre more than 8 mm off the jack axis sideways, above the opening, or not pointing into the jack)."""
    p, a = plug
    s = UNITS[unit]
    if not np.isfinite(p).all():
        return -1.0
    if abs(p[0] - PORT_X) > 0.008 or p[2] > TOP + CAV_H + 0.005 or a[1] * s < 0.9:
        return -1.0
    return float(s * (p[1] + s * PL / 2) - FACE_Y)


def depths(objects) -> dict[str, float]:
    plugs = _plugs(objects)
    return {u: max(depth_in(u, pl) for pl in plugs) for u in UNITS}


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
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
    return all(float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in grippers() for p, _ in _plugs(objects))


def score(objects) -> float:
    """0 if a gripper is still within 45 mm of either plug at the end (still holding). Otherwise 0.5 for each unit
    (main, remote) whose jack has a plug tip at least 15 mm inside it, lined up with the jack."""
    if not released(objects):
        return 0.0
    return 0.5 * sum(d >= SEATED_MIN for d in depths(objects).values())


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and np.linalg.norm(objects[CORD.name].lin_vel) < 0.02)


def distance(objects) -> float:
    """Sum over the two jacks of how far (m) the best plug still is from 15 mm deep (0.1 if not lined up)."""
    return float(sum(max(0.0, SEATED_MIN - d) if d > -1 else 0.1 for d in depths(objects).values()))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Both arms pick up their plug together (pinching it 10 mm from its cord end) and carry the two ends forward side by
# side, so the short cord is never dragged by one arm. The left arm then seats its plug in the main unit and keeps
# holding it; only then does the right arm push the other plug into the remote unit (the held first plug can't be
# pulled out by the cord). Both release and lift away.

FINGER_PINCH = -0.020  # finger opening for the plug pinch (pads ~44 mm apart)
GRASP_BACK = 0.010  # pinch this far from the plug's cord end
SEAT_TO = 0.0245  # target tip depth (cavity is 26 mm deep)
HI = TOP + 0.08


def _log(ctx, tag=""):
    st = ctx.states()
    ds = {u: round(d * 1000, 1) for u, d in depths(st).items()}
    pl = [np.round(p, 3).tolist() for p, _ in _plugs(st)]
    print(f"[t={ctx.t:5.2f}] {tag} depth_mm={ds} plugs={pl} rel={released(st)} score={score(st)} "
          f"tcp={[np.round(g, 3).tolist() for g in grippers()]} cmd={[np.round(a.tcp(), 3).tolist() for a in (ctx.left, ctx.right)]}",
          flush=True)


def _plug_for(st, sign):
    """The plug currently on the ``sign`` side (y) of the table: (centre, axis)."""
    return max(_plugs(st), key=lambda pa: sign * pa[0][1])


def _pick(arm, st, sign, yaw):
    """Pinch the plug on side ``sign`` 10 mm from its cord end and lift it 2.5 cm."""
    from yam_arena.constants import FINGER_CLOSED

    p, a = _plug_for(st, sign)
    g = p - a * (PL / 2 - GRASP_BACK)
    g = np.array([g[0], g[1], TOP + PH + 0.003])
    yield from arm.move((g[0], g[1], HI), yaw=yaw, finger=FINGER_PINCH, duration=1.5)
    yield from arm.move(g, yaw=yaw, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move(g + np.array([0.0, 0.0, 0.025]), yaw=yaw, duration=0.6)


def _stage(arm, sign, yaw, pre=0.008, z=0.025, duration=2.5):
    """Carry the held plug in front of its jack: tip ``pre`` before the face, lined up with the jack axis."""
    tip_y = sign * (FACE_Y - pre)
    gy = tip_y - sign * (PL - GRASP_BACK)
    yield from arm.move((PORT_X, gy, TOP + PH + 0.003 + z), yaw=yaw, duration=duration)


def _dbg(ctx, tag):
    if os.environ.get("BOTHENDS_DBG"):
        _log(ctx, tag)


def _insert(arm, sign, yaw, ctx):
    """Lower the plug to jack height and slide it in until its tip is SEAT_TO deep (closed loop on the plug)."""
    gz = TOP + PH + 0.003
    gy0 = sign * (FACE_Y - 0.008) - sign * (PL - GRASP_BACK)
    yield from arm.move((PORT_X, gy0, gz), yaw=yaw, duration=0.8)
    yield from arm.wait(0.3)
    _dbg(ctx, "lowered")
    unit = "main" if sign > 0 else "remote"
    # line the plug up with the jack axis (closed loop on the plug: the arm sags a few mm under load)
    dx = 0.0
    for _ in range(3):
        p, a = _plug_for(ctx.states(), sign)
        err = PORT_X - p[0]
        if abs(err) < 0.0007:
            break
        dx = float(np.clip(dx + err, -0.02, 0.02))
        yield from arm.move((PORT_X + dx, gy0, gz), yaw=yaw, duration=0.5)
        yield from arm.wait(0.2)
    _dbg(ctx, "aligned")
    gy = gy0 + sign * (SEAT_TO + 0.008)
    yield from arm.move((PORT_X + dx, gy, gz), yaw=yaw, duration=2.0)
    _dbg(ctx, "pushed")
    for _ in range(2):  # top up if the plug slipped back in the fingers
        d = depths(ctx.states())[unit]
        if d >= SEAT_TO - 0.002:
            break
        gy += sign * (SEAT_TO - max(d, 0.0))
        yield from arm.move((PORT_X + dx, gy, gz), yaw=yaw, duration=1.0)
    yield from arm.wait(0.3)
    return gy


def _release(arm, gy, yaw, sign):
    yield from arm.grip(FINGER_PINCH, 0.6)
    yield from arm.move((PORT_X, gy - sign * 0.005, HI), yaw=yaw, duration=1.0)
    yield from arm.move((0.20, sign * 0.20, TOP + 0.14), yaw=yaw, duration=1.2)


def demo_script(ctx):
    mode = os.environ.get("BOTHENDS_DEMO", "correct")  # test only
    L, Rt = ctx.left, ctx.right
    yl, yr = -math.pi / 2, -math.pi / 2
    _log(ctx, "start")
    if mode == "nothing":  # cheat: do nothing
        for _ in range(6):
            yield from L.wait(1.0)
            _log(ctx)
        return
    st = ctx.states()
    yield from ctx.both(_pick(L, st, +1, yl), _pick(Rt, st, -1, yr))
    _log(ctx, "picked")
    yield from ctx.both(_stage(L, +1, yl), _stage(Rt, -1, yr))
    _log(ctx, "staged")
    if mode == "near":  # cheat: put both plugs down right at the jacks (against the faces) without inserting them
        # main: lined up with the jack, tip 4 mm short of the face; remote: pressed against the face 20 mm beside
        # the opening
        def drop(arm, sign, yaw, dx, gap):
            gy = sign * (FACE_Y - gap - PL + GRASP_BACK)
            yield from arm.move((PORT_X + dx, gy, TOP + PH + 0.006), yaw=yaw, duration=1.0)
            yield from _release(arm, gy, yaw, sign)
        yield from ctx.both(drop(L, +1, yl, 0.0, 0.004), drop(Rt, -1, yr, 0.02, -0.002))
        for _ in range(3):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return
    gl = yield from _insert(L, +1, yl, ctx)
    _log(ctx, "main seated (held)")
    if mode == "one":  # partial credit: seat only the main-unit plug, drop the other end
        yield from Rt.grip(FINGER_PINCH, 0.6)
        yield from Rt.move((0.20, -0.20, TOP + 0.14), yaw=yr, duration=1.2)
        yield from _release(L, gl, yl, +1)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    gr = yield from _insert(Rt, -1, yr, ctx)
    _log(ctx, "remote seated")
    if mode == "hold":  # cheat: seat both but keep holding them
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from ctx.both(_release(L, gl, yl, +1), _release(Rt, gr, yr, -1))
    yield from L.wait(1.5)
    _log(ctx, "end")
