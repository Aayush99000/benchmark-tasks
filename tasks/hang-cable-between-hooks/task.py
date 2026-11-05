"""On the station table, 28 cm in front of the robot, stand two grey hook posts 30 cm apart (one in front of each
arm, at y +15 cm and y -15 cm). Each post is a 20 mm round rod topped by an open-top U cradle: a 32 x 25 mm floor
whose top is 15 cm above the table (the hook line), with two 4 mm side walls 12 mm tall (inner gap 24 mm), so a
cable can lie in it running left-right. Two thin orange bands on each post mark 4 cm and 6 cm below the hook line.
A free black 8 mm cable, 50 cm long, with a small black plug (35 x 16 x 12 mm, 10 g) on each end, lies straight on
the table across the front of the posts (8 cm closer to the robot than the posts), from in front of the left arm
to in front of the right arm. The cable must be hung across both cradles with each end hanging down on the outer
side of its hook, and the middle sagging so that its lowest point is between the two orange bands (4-6 cm below
the hook line); then the robot lets go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Hang the black cable across the two hooks: lay it in both cradles with each end hanging down on the "
               "outer side of its hook, so that the middle sags between the hooks with its lowest point between "
               "the two orange bands on the posts (4 to 6 cm below the hooks). Then let go of the cable.")

# -- hooks: post + open-top U cradle (cable runs along y through the cradle) ----------------------------------------
HOOK_X = 0.28
HOOK_Y = 0.15  # hooks at y = +HOOK_Y (left arm) and -HOOK_Y (right arm)
LINE_Z = TOP + 0.15  # top of the cradle floor = hook line
POST_R = 0.010
FLOOR_T = 0.008
CRADLE_LX = 0.032  # floor size across the cable (x)
CRADLE_LY = 0.025  # floor size along the cable (y)
SLOT_W = 0.024  # inner gap between the side walls
WALL_T, WALL_H = 0.004, 0.012  # wall height above the floor top
GREY = (0.55, 0.57, 0.60)
ORANGE = (1.0, 0.45, 0.0)
BAND_DEPTHS = (0.04, 0.06)


def _hook(tag: str, y: float) -> list:
    post_h = LINE_Z - FLOOR_T - TOP
    parts = [
        Cylinder(f"post_{tag}", pos=(HOOK_X, y, TOP + post_h / 2), radius=POST_R, height=post_h, color=GREY),
        Box(f"cradle_floor_{tag}", pos=(HOOK_X, y, LINE_Z - FLOOR_T / 2), size=(CRADLE_LX, CRADLE_LY, FLOOR_T),
            color=GREY),
    ]
    for s, sg in (("front", -1), ("back", 1)):
        parts.append(Box(f"cradle_wall_{s}_{tag}", pos=(HOOK_X + sg * (SLOT_W / 2 + WALL_T / 2), y,
                                                       LINE_Z - FLOOR_T + (FLOOR_T + WALL_H) / 2),
                         size=(WALL_T, CRADLE_LY, FLOOR_T + WALL_H), color=GREY))
    for k, d in enumerate(BAND_DEPTHS):
        parts.append(Cylinder(f"band_{k}_{tag}", pos=(HOOK_X, y, LINE_Z - d), radius=POST_R + 0.0015, height=0.004,
                              color=ORANGE))
    return parts


HOOKS = _hook("l", HOOK_Y) + _hook("r", -HOOK_Y)

# -- free cable with a plug (end weight) on each end, lying straight on the table in front of the posts ------------
R = 0.004
CORD_LEN = 0.50
CORD_X = 0.20
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.010, color=(0.08, 0.08, 0.08))
CORD = cable("cord", start=(CORD_X, CORD_LEN / 2, TOP + PLUG.size[2] / 2 + 0.0005), yaw=-math.pi / 2,
             length=CORD_LEN, radius=R, start_plug=PLUG, end_plug=PLUG, color=(0.05, 0.05, 0.05),
             bend_stiffness=0.01, bend_damping=0.001)

OBJECTS = [*HOOKS, CORD]

# -- scoring ---------------------------------------------------------------------------------------------------------
SAG_LO, SAG_HI = 0.04, 0.06  # full credit band (lowest point below the hook line)
SAG_TOL = 0.02  # partial credit up to 2 cm outside the band (but see SAG_MIN)
SAG_MIN = 0.03  # less than this = pulled taut / laid straight over the hooks: not hung with sag -> 0
END_DROP = 0.05  # each end must hang at least this far below the hook line, outside its hook
RELEASE_DIST = 0.03  # every gripper (centre between the pads) at least this far from the cable


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def in_cradle(objects, sign: int) -> bool:
    """Some part of the cable lies inside the cradle's slot (between the walls, over the floor, below the wall
    tops)."""
    pts = _dense(objects["cord"].polyline())
    ok = (np.abs(pts[:, 0] - HOOK_X) <= SLOT_W / 2) & (np.abs(pts[:, 1] - sign * HOOK_Y) <= CRADLE_LY / 2) \
        & (pts[:, 2] >= LINE_Z) & (pts[:, 2] <= LINE_Z + WALL_H)
    return bool(ok.any())


def _end_hangs_out(tip: np.ndarray, sign: int) -> bool:
    return bool(sign * tip[1] > HOOK_Y and tip[2] <= LINE_Z - END_DROP)


def hung(objects) -> tuple[bool, bool]:
    """(left hook hung, right hook hung): the cable lies in the cradle and one cable end (its plug centre) hangs
    >= 5 cm below the hook line on the hook's outer side."""
    c = objects["cord"]
    tips = (c.start_plug if c.start_plug is not None else c.start_tip,
            c.end_plug if c.end_plug is not None else c.end_tip)
    l_ends = {i for i, t in enumerate(tips) if _end_hangs_out(t, +1)} if in_cradle(objects, +1) else set()
    r_ends = {i for i, t in enumerate(tips) if _end_hangs_out(t, -1)} if in_cradle(objects, -1) else set()
    return bool(l_ends), bool(r_ends)  # an end cannot be outside both hooks, so they are different ends


def sag(objects) -> float:
    """Hook line minus the lowest point of the cable's underside between the two cradles (m)."""
    pts = _dense(objects["cord"].polyline())
    mid = np.abs(pts[:, 1]) < HOOK_Y - CRADLE_LY / 2
    if not mid.any():
        return float("inf")
    return float(LINE_Z - (pts[mid, 2].min() - R))


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_hang_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._hang_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown.
    The approach axis is taken from the gripper link to the midpoint of the two finger links."""
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
    c = objects["cord"]
    pts = np.vstack([_dense(c.polyline())] + [p[None] for p in (c.start_plug, c.end_plug) if p is not None])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless the cable is released. 0.2 per hook hung (cable in the cradle, its end >= 5 cm below the hook line
    on the outer side). Both hung: 0 if the sag is < 3 cm (taut / laid straight); else + 0.6 for a sag of 4-6 cm,
    + 0.3 for 3-4 or 6-8 cm, + 0 beyond 8 cm."""
    if not released(objects):
        return 0.0
    hl, hr = hung(objects)
    if not (hl and hr):
        return 0.2 * (hl + hr)
    d = sag(objects)
    if d < SAG_MIN:
        return 0.0
    if SAG_LO <= d <= SAG_HI:
        return 1.0
    if SAG_LO - SAG_TOL <= d <= SAG_HI + SAG_TOL:
        return 0.7
    return 0.4


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["cord"].resting(max_speed=0.02)


def distance(objects) -> float:
    """|sag - 5 cm| (m) once both hooks are hung, else 1 m."""
    hl, hr = hung(objects)
    if not (hl and hr):
        return 1.0
    return abs(sag(objects) - 0.05)


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

PLUG_OUT = 0.09  # the arms hold the plugs this far outside the cradle centres while laying the cable in
SAG_TARGET = 0.05
HOLD_SAG = 0.062  # sag set while holding (the cable settles ~1 cm when the plugs are let go)


def demo_script(ctx):
    """Each arm takes one end plug, lifts the cable, carries its plug past the outside of its hook so the cable
    drops into the cradle, and lowers the plug on the outer side. Closed loop: while holding, both arms lower or
    raise their plugs (pulling cable over the cradles) until the measured sag is 5 cm, then let go."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("HANG_CHEAT", "")  # test only: "none" | "hold" | "short" | "taut"
    yl, yr = -math.pi / 2, math.pi / 2  # fingers close along x, across the plug / cable (which run along y)
    over = 0.010
    hi = TOP + 0.06
    carry = TOP + 0.21

    def log(tag):
        st = ctx.states()
        c = st["cord"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: hung={hung(st)} sag={sag(st):.3f} rel={released(st)} "
              f"score={score(st):.2f} plugs={np.round(c.start_plug, 3)} {np.round(c.end_plug, 3)}", flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return
    st = ctx.states()
    pl, pr = st["cord"].start_plug, st["cord"].end_plug
    # 1) both arms take the end plugs
    yield from ctx.both(L.move((pl[0], pl[1], hi), yaw=yl, finger=FINGER_OPEN, duration=1.5),
                        Rt.move((pr[0], pr[1], hi), yaw=yr, finger=FINGER_OPEN, duration=1.5))
    yield from ctx.both(L.move((pl[0], pl[1], pl[2] + over), yaw=yl, duration=1.0),
                        Rt.move((pr[0], pr[1], pr[2] + over), yaw=yr, duration=1.0))
    yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
    # 2) lift, bring the plugs in front of the hooks on their inner side, then out past the hooks (the cable now
    #    runs over the cradles) and down on the outer side, so the cable drops into both cradles
    yield from ctx.both(L.move((pl[0], pl[1], carry), yaw=yl, duration=1.2),
                        Rt.move((pr[0], pr[1], carry), yaw=yr, duration=1.2))
    log("lifted")
    inner = HOOK_Y - 0.04
    yield from ctx.both(L.move((HOOK_X, inner, carry), yaw=yl, duration=1.5),
                        Rt.move((HOOK_X, -inner, carry), yaw=yr, duration=1.5))
    out_y = HOOK_Y + PLUG_OUT
    lay = LINE_Z + 0.018  # TCP height: plug centre ~8 mm above the hook line, cable running level into the cradle
    if cheat == "short":  # drape it over the hooks but keep the ends up on the cradles (nothing hangs past them)
        out_y = HOOK_Y + 0.03
    yield from ctx.both(L.move((HOOK_X, out_y, carry), yaw=yl, duration=1.5),
                        Rt.move((HOOK_X, -out_y, carry), yaw=yr, duration=1.5))
    yield from ctx.both(L.move((HOOK_X, out_y, lay), yaw=yl, duration=1.5),
                        Rt.move((HOOK_X, -out_y, lay), yaw=yr, duration=1.5))
    yield from L.wait(0.5)
    log("laid in")
    # 3) closed loop: move both plugs outwards (less sag) or inwards (more sag) until the sag is on target
    if cheat != "short":
        for _ in range(4):
            d = sag(ctx.states())
            if abs(d - HOLD_SAG) < 0.004 or not np.isfinite(d):
                break
            out_y = float(np.clip(out_y + 0.45 * (d - HOLD_SAG), HOOK_Y + 0.07, HOOK_Y + 0.13))
            yield from ctx.both(L.move((HOOK_X, out_y, lay), yaw=yl, duration=0.8),
                                Rt.move((HOOK_X, -out_y, lay), yaw=yr, duration=0.8))
            yield from L.wait(0.5)
            log(f"adjust out={out_y - HOOK_Y:.3f}")
    if cheat == "taut":  # pull the cable as straight as the arms can across the hooks, then let go
        out_y = HOOK_Y + 0.14
        yield from ctx.both(L.move((HOOK_X, out_y, lay), yaw=yl, duration=1.5),
                            Rt.move((HOOK_X, -out_y, lay), yaw=yr, duration=1.5))
        log("pulled taut")
    # 4) let go and move clear
    yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))
    if cheat == "hold":  # let the plugs drop into place, then the left arm follows its plug down and grips it
        yield from L.move((HOOK_X, HOOK_Y + 0.06, LINE_Z - 0.05), yaw=yl, duration=0.8)
        st = ctx.states()
        p = st["cord"].start_plug
        yield from L.move((p[0], p[1], p[2] + 0.008), yaw=yl, duration=0.6)
        yield from L.grip(FINGER_CLOSED)
        yield from Rt.move((HOOK_X - 0.08, -out_y, lay + 0.04), yaw=yr, duration=1.0)
        log("holding the hanging left plug")
        yield from L.wait(4.0)
        log("end (held)")
        return
    yield from ctx.both(L.move((HOOK_X, out_y, lay + 0.04), yaw=yl, duration=0.8),
                        Rt.move((HOOK_X, -out_y, lay + 0.04), yaw=yr, duration=0.8))
    yield from ctx.both(L.move((HOOK_X - 0.08, out_y, lay + 0.04), yaw=yl, duration=1.0),
                        Rt.move((HOOK_X - 0.08, -out_y, lay + 0.04), yaw=yr, duration=1.0))
    log("released")
    yield from L.wait(1.5)
    log("end")
