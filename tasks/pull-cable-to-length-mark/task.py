"""On the station table: a grey closed cable feed channel (a straight duct, 400 mm long, 14 x 12 mm inside) lies on
the table in front of the left arm, running left-right (along y) at x 30 cm from y +10 to y +50 cm, its open exit
facing the right arm. A black 6 mm round cable (45 cm) lies inside it; only its free end sticks 6 cm out of the exit.
A yellow ruler strip (400 x 30 mm, 1 mm thick) is taped to the table just beyond the cable line (away from the
robot), its zero at the exit face, running towards the right arm, with black ticks every 5 cm (longer every 10 cm)
and a red line at 30 cm (also marked on the bench on the near side of the cable). The robot must pull the cable out
of the channel along the ruler until its end lies on the red 30 cm line (within 1 cm), then let go. The 30 cm line is
beyond the left arm's reach and the cable end starts beyond the right arm's reach, so the end has to be handed over:
the left arm draws the first stretch out, the right arm takes the end and pulls it to the mark.

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
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Pull the black cable out of the grey feed channel along the yellow ruler until the cable's end lies "
               "on the table at the red 30 cm line, then let go, leaving the pulled-out cable lying straight beside the "
               "ruler. Never pull the end more than 5 cm past the red line.")

# -- feed channel (cable line x = CX; exit face at y = EXIT_Y, cable pulled out towards -y) ------------------------
CX = 0.30
EXIT_Y = 0.10
BACK_Y = 0.50
IN_W, IN_H = 0.014, 0.012  # inside width (x) and height
WALL, ROOF = 0.008, 0.006
GREY = (0.55, 0.57, 0.60)
_cl = BACK_Y - EXIT_Y
_cy = (EXIT_Y + BACK_Y) / 2
CHANNEL = [
    Box("channel_near", pos=(CX - IN_W / 2 - WALL / 2, _cy, TOP + IN_H / 2), size=(WALL, _cl, IN_H), color=GREY),
    Box("channel_far", pos=(CX + IN_W / 2 + WALL / 2, _cy, TOP + IN_H / 2), size=(WALL, _cl, IN_H), color=GREY),
    Box("channel_roof", pos=(CX, _cy, TOP + IN_H + ROOF / 2), size=(IN_W + 2 * WALL, _cl, ROOF), color=GREY),
    Box("channel_back", pos=(CX, BACK_Y + WALL / 2, TOP + (IN_H + ROOF) / 2), size=(IN_W + 2 * WALL, WALL, IN_H + ROOF),
        color=GREY),
]

# -- cable: free (not anchored), tail inside the channel, end 6 cm out of the exit ---------------------------------
R = 0.003
CORD_LEN = 0.45
OUT0 = 0.06  # starting protrusion
CORD_START = (CX, EXIT_Y - OUT0 + CORD_LEN, TOP + R + 0.0005)  # tail end, deep inside the channel
CORD = cable("cord", start=CORD_START, yaw=-math.pi / 2, length=CORD_LEN, radius=R, anchor_start=False,
             color=(0.05, 0.05, 0.05), bend_stiffness=0.01, bend_damping=0.001)

# -- ruler (zero at the exit face, reading grows towards -y) -----------------------------------------------------------
RULER_LEN, RULER_W, RULER_T = 0.40, 0.030, 0.001
RULER_X0 = CX + 0.009  # near edge of the ruler (cable surface at CX + 0.003)
RULER = Box("ruler", pos=(RULER_X0 + RULER_W / 2, EXIT_Y - RULER_LEN / 2, TOP + RULER_T / 2),
            size=(RULER_W, RULER_LEN, RULER_T), color=(0.95, 0.80, 0.10))
TARGET = 0.30  # m along the ruler
TICKS = []
for _k in range(0, 9):
    _cm = 5 * _k
    if _cm == 30:
        continue
    _l = 0.016 if _cm % 10 == 0 else 0.009
    TICKS.append(Box(f"tick_{_cm}", pos=(RULER_X0 + _l / 2, EXIT_Y - _cm / 100, TOP + 0.0007), size=(_l, 0.002, 0.0014),
                     color=(0.05, 0.05, 0.05)))
MARK_W = 0.005
MARKS = [
    Box("mark_ruler", pos=(RULER_X0 + RULER_W / 2, EXIT_Y - TARGET, TOP + 0.0008), size=(RULER_W, MARK_W, 0.0016),
        color=(0.85, 0.06, 0.05)),
    Box("mark_bench", pos=(CX - 0.006 - 0.0125, EXIT_Y - TARGET, TOP + 0.0004), size=(0.025, MARK_W, 0.0008),
        color=(0.85, 0.06, 0.05)),
]

OBJECTS = [*CHANNEL, CORD, RULER, *TICKS, *MARKS]

# -- scoring ---------------------------------------------------------------------------------------------------------
FULL_TOL, MID_TOL, LOW_TOL = 0.01, 0.03, 0.05
OVERPULL = TARGET + LOW_TOL  # the end must never be pulled past the 35 cm line
LANE = 0.02  # every point of the pulled-out cable within 2 cm (sideways) of the cable line
ON_TABLE_Z = TOP + 0.015  # ... and at most 1.5 cm above the table
MIN_INSIDE = 0.05  # the cable must still run at least 5 cm into the channel


def reading(objects) -> float:
    """Ruler reading (m) of the cable's end: its distance from the exit face along the ruler (horizontal)."""
    return float(EXIT_Y - objects["cord"].end_tip[1])


# Over-pull memory: the end must never pass the 35 cm line during the episode. Reset at every episode start (the
# embodiment's reset is wrapped below; as a fallback, a cable back in its exact starting layout also resets it).
_STATE = {"max_reading": -1.0}


def _reset_memory() -> None:
    _STATE["max_reading"] = -1.0


def _install_reset_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_length_mark_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._length_mark_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._length_mark_hooks = []
        cls.reset = reset
    cls.reset._length_mark_hooks.append(_reset_memory)


_install_reset_hook()
_START_PTS = CORD.start_points()


def _track(objects) -> None:
    c = objects["cord"]
    if np.abs(c.points - _START_PTS).max() < 0.003:  # untouched starting layout => new episode
        _reset_memory()
    _STATE["max_reading"] = max(_STATE["max_reading"], reading(objects))


def over_pulled(objects) -> bool:
    _track(objects)
    return _STATE["max_reading"] > OVERPULL


def laid_along_ruler(objects) -> bool:
    """The cable still runs >= 5 cm into the channel, and all of it that is out of the channel lies on the table
    (<= 1.5 cm up) within 2 cm sideways of the cable line beside the ruler."""
    c = objects["cord"]
    if c.start_tip[1] < EXIT_Y + MIN_INSIDE:
        return False
    poly = c.polyline()
    out = poly[poly[:, 1] < EXIT_Y]
    if len(out) == 0:
        return False
    return bool(np.abs(out[:, 0] - CX).max() <= LANE and out[:, 2].max() <= ON_TABLE_Z)


_ENV: dict = {"env": None}
RELEASE_DIST = 0.03  # both grippers (centre between the pads) at least this far from every point of the cable


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_length_mark_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._length_mark_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown."""
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


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def released(objects) -> bool:
    """Neither gripper within 3 cm of the cable (it has been let go, not held at the mark)."""
    pts = _dense(objects["cord"].polyline())
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def error(objects) -> float | None:
    """|reading - 30 cm| (m) if the cable is released, laid along the ruler and was never over-pulled, else None."""
    if over_pulled(objects) or not laid_along_ruler(objects) or not released(objects):
        return None
    return abs(reading(objects) - TARGET)


def success(objects) -> bool:
    e = error(objects)
    return e is not None and e <= FULL_TOL


def score(objects) -> float:
    """0 if the cable is still held (a gripper within 3 cm of it), or the end was ever pulled past the 35 cm line, or
    the pulled-out cable is not lying on the table within
    2 cm of the cable line, or less than 5 cm is left in the channel. Otherwise by the end's ruler reading:
    within 1 cm of 30 cm = 1.0; within 3 cm = 0.6; within 5 cm = 0.3; else 0."""
    e = error(objects)
    if e is None:
        return 0.0
    if e <= FULL_TOL:
        return 1.0
    if e <= MID_TOL:
        return 0.6
    if e <= LOW_TOL:
        return 0.3
    return 0.0


def distance(objects) -> float:
    """|reading - 30 cm| (m); 1 m once over-pulled."""
    if over_pulled(objects):
        return 1.0
    return abs(reading(objects) - TARGET)


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    YL, YR = -math.pi / 2, math.pi / 2  # IK-friendly grip yaws (fingers close across the cable, along x)
    hi = TOP + 0.06
    lift = TOP + 0.030  # carry height of the end while pulling
    GRIP_BACK = 0.015  # grasp this far behind the end tip

    def log(tag):
        st = ctx.states()
        c = st["cord"]
        print(f"[demo] {tag}: tip={np.round(c.end_tip, 3)} tail={np.round(c.start_tip, 3)} "
              f"read={reading(st):.3f} max={_STATE['max_reading']:.3f} lane={laid_along_ruler(st)} rel={released(st)} "
              f"score={score(st):.2f}", flush=True)

    # Cheat variants (scoring checks, must score 0): LENGTH_CHEAT=nothing | yank (pull far past the mark, then put
    # the end back on the line) | sideways (pull the end to the 30 cm reading but dragged 6 cm off the cable line) |
    # hold (pull the end exactly to the line and keep holding it there).
    cheat = os.environ.get("LENGTH_CHEAT", "")
    log("start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        log("end")
        return

    def grasp(arm, yaw, tip):
        g = (CX, tip[1] + GRIP_BACK)
        yield from arm.move((g[0], g[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.2)
        yield from arm.move((g[0], g[1], TOP + R + 0.010), yaw=yaw, duration=0.8)
        yield from arm.grip(FINGER_CLOSED)
        yield from arm.move((g[0], g[1], lift), yaw=yaw, duration=0.5)
        return g

    # 1) Left arm takes the end just outside the exit and draws the cable out to the midline.
    st = ctx.states()
    yield from grasp(L, YL, st["cord"].end_tip)
    yield from L.move((CX, 0.0, lift), yaw=YL, duration=1.5)
    yield from L.move((CX, 0.0, TOP + R + 0.010), yaw=YL, duration=0.5)
    yield from L.grip(FINGER_OPEN, duration=0.4)
    yield from L.move((CX, 0.0, hi), yaw=YL, duration=0.4)
    log("left drew")
    # 2) Left arm backs off; the right arm takes the end and pulls it towards the mark.
    st = ctx.states()
    tip = st["cord"].end_tip
    yield from ctx.both(L.move((CX - 0.08, 0.20, hi + 0.03), yaw=YL, duration=1.0),
                        Rt.move((CX, tip[1] + GRIP_BACK - 0.03, hi), yaw=YR, finger=FINGER_OPEN, duration=1.0))
    st = ctx.states()
    gy = st["cord"].end_tip[1] + GRIP_BACK
    yield from Rt.move((CX, gy, hi), yaw=YR, duration=0.5)
    yield from Rt.move((CX, gy, TOP + R + 0.010), yaw=YR, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED)
    yield from Rt.move((CX, gy, lift), yaw=YR, duration=0.5)
    log("right has end")
    if cheat == "yank":
        yield from Rt.move((CX, EXIT_Y - 0.42 + GRIP_BACK, lift), yaw=YR, duration=2.0)
        log("yanked")
        tgt_tcp = EXIT_Y - TARGET + GRIP_BACK
        yield from Rt.move((CX, tgt_tcp, lift), yaw=YR, duration=1.5)
    elif cheat == "sideways":
        tgt_tcp = EXIT_Y - TARGET + GRIP_BACK
        yield from Rt.move((CX - 0.06, tgt_tcp, lift), yaw=YR, duration=2.5)
        yield from Rt.move((CX - 0.06, tgt_tcp, TOP + R + 0.010), yaw=YR, duration=0.6)
        yield from Rt.grip(FINGER_OPEN, duration=0.4)
        yield from Rt.move((CX - 0.06, tgt_tcp, hi), yaw=YR, duration=0.5)
        yield from Rt.wait(2.0)
        log("end")
        return
    # 3) Closed loop: lay the end down, read the ruler, and correct with short strokes until within 4 mm; let go
    #    and re-read; re-grip and correct if letting go moved the end.
    y = gy
    for attempt in range(3):
        for k in range(6):
            st = ctx.states()
            e = TARGET - reading(st)  # > 0: still short
            log(f"pull {attempt}.{k}")
            if abs(e) <= 0.004 and k > 0:
                break
            y = y - float(np.clip(e, -0.03, 0.12))
            yield from Rt.move((CX, y, lift), yaw=YR, duration=max(0.6, abs(e) * 12))
            yield from Rt.move((CX, y, TOP + R + 0.010), yaw=YR, duration=0.5)
            yield from Rt.wait(0.3)
        if cheat == "hold":  # keep holding the end down at the line, never let go
            yield from Rt.wait(3.0)
            log("end")
            return
        yield from Rt.grip(FINGER_OPEN, duration=0.5)
        yield from Rt.move((CX, y, hi), yaw=YR, duration=0.6)
        yield from Rt.wait(0.5)
        st = ctx.states()
        log("released")
        if abs(TARGET - reading(st)) <= 0.006:
            break
        y = st["cord"].end_tip[1] + GRIP_BACK
        yield from Rt.move((CX, y, TOP + R + 0.010), yaw=YR, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED)
        yield from Rt.move((CX, y, lift), yaw=YR, duration=0.4)
    yield from Rt.wait(2.0)
    log("end")
