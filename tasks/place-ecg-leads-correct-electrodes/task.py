"""On the station table lies a flat, skin-coloured patient manikin, on its back (face up), with its head AWAY from the
robot and its feet towards the robot: a torso board (20 cm long, 22 cm wide, 1 cm thick) whose far end joins a neck
and a round head (13 cm across) with a nose on top, so the robot stands at the patient's feet. Four identical white
electrode cups (square, 40 mm inner, 6 mm walls) are fixed on the torso, unlabelled, at the four limb-lead
positions: two near the shoulders (far end, 11 cm apart left-right) and two at the lower rib cage (near end).
Standing at the feet of a patient lying on its back, the patient's RIGHT side is on the robot's LEFT (+y):
  RA (right arm) = far cup on the robot's left,   LA (left arm) = far cup on the robot's right,
  RL (right leg) = near cup on the robot's left,  LL (left leg) = near cup on the robot's right.
A grey patient monitor stands on the robot's left; three 5 mm ECG lead wires (white, black, red, 50 cm) leave its
right-hand face and lie straight across the table in front of the manikin, each ending in a clip of its own colour
(22 x 14 x 12 mm) on the robot's right. AHA 3-lead colour code: white -> RA, black -> LA, red -> LL; RL stays empty.
Each clip must end up dropped into the cup of its electrode and released.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Attach the 3-lead ECG cable to the patient manikin, which lies on its back with its head away from "
               "you. Following the AHA colour code, drop each lead's clip into the white electrode cup at the "
               "correct body position for that lead; one cup stays empty. Then let go of the clips.")

# -- manikin: torso board, neck, head (with a nose: face up) ---------------------------------------------------------
SKIN = (0.78, 0.52, 0.42)
SKIN_DARK = (0.55, 0.33, 0.25)
BOARD_X0, BOARD_X1, BOARD_W, BOARD_T = 0.255, 0.455, 0.22, 0.010
BOARD_TOP = TOP + BOARD_T
TORSO = Box("torso", pos=((BOARD_X0 + BOARD_X1) / 2, 0.0, TOP + BOARD_T / 2),
            size=(BOARD_X1 - BOARD_X0, BOARD_W, BOARD_T), color=SKIN)
NECK = Box("neck", pos=(BOARD_X1 + 0.015, 0.0, TOP + 0.015), size=(0.03, 0.07, 0.03), color=SKIN)
HEAD_C, HEAD_R, HEAD_H = (0.545, 0.0), 0.065, 0.06
HEAD = Cylinder("head", pos=(HEAD_C[0], HEAD_C[1], TOP + HEAD_H / 2), radius=HEAD_R, height=HEAD_H, color=SKIN)
NOSE = Box("nose", pos=(HEAD_C[0] + 0.012, 0.0, TOP + HEAD_H + 0.008), size=(0.03, 0.016, 0.016), color=SKIN_DARK)
ARM_X0, ARM_X1, ARM_W = 0.27, 0.45, 0.04  # upper arms alongside the torso, shoulders at the far end
ARMS = [Box(f"arm_{s}", pos=((ARM_X0 + ARM_X1) / 2, sg * (BOARD_W / 2 + 0.005 + ARM_W / 2), TOP + BOARD_T / 2),
            size=(ARM_X1 - ARM_X0, ARM_W, BOARD_T), color=SKIN) for s, sg in (("l", 1), ("r", -1))]
MANIKIN = [TORSO, NECK, HEAD, NOSE, *ARMS]

# -- electrode cups (white, unlabelled). Patient's right = robot's left (+y): see the docstring --------------------
CUP_IN, CUP_WALL_T, CUP_WALL_H = 0.040, 0.004, 0.006
CUP_FAR_X, CUP_NEAR_X, CUP_Y = 0.40, 0.29, 0.065
CUPS = {  # electrode position -> cup centre (x, y)
    "RA": (CUP_FAR_X, +CUP_Y),
    "LA": (CUP_FAR_X, -CUP_Y),
    "RL": (CUP_NEAR_X, +CUP_Y),
    "LL": (CUP_NEAR_X, -CUP_Y),
}
WHITE_CUP = (0.97, 0.97, 0.97)


def _cup(site: str) -> list[Box]:
    cx, cy = CUPS[site]
    z = BOARD_TOP + CUP_WALL_H / 2
    o = CUP_IN / 2 + CUP_WALL_T / 2
    full = CUP_IN + 2 * CUP_WALL_T
    n = site.lower()
    return [
        Box(f"cup_{n}_xn", pos=(cx - o, cy, z), size=(CUP_WALL_T, full, CUP_WALL_H), color=WHITE_CUP),
        Box(f"cup_{n}_xp", pos=(cx + o, cy, z), size=(CUP_WALL_T, full, CUP_WALL_H), color=WHITE_CUP),
        Box(f"cup_{n}_yn", pos=(cx, cy - o, z), size=(CUP_IN, CUP_WALL_T, CUP_WALL_H), color=WHITE_CUP),
        Box(f"cup_{n}_yp", pos=(cx, cy + o, z), size=(CUP_IN, CUP_WALL_T, CUP_WALL_H), color=WHITE_CUP),
        # the electrode's gel pad: a 1 mm white disc on the cup floor (visual)
        Cylinder(f"cup_{n}_pad", pos=(cx, cy, BOARD_TOP + 0.0005), radius=0.012, height=0.001, color=(0.85, 0.85, 0.80)),
    ]


# -- patient monitor (robot's left) and the three leads ---------------------------------------------------------------
MON_SIZE = (0.18, 0.08, 0.06)
MON_C = (0.17, 0.34)
MON_FACE_Y = MON_C[1] - MON_SIZE[1] / 2  # its right-hand face, where the leads leave it
MONITOR = Box("monitor", pos=(MON_C[0], MON_C[1], TOP + MON_SIZE[2] / 2), size=MON_SIZE, color=(0.45, 0.47, 0.50))
SCREEN = Box("monitor_screen", pos=(MON_C[0] - 0.01, MON_C[1], TOP + MON_SIZE[2] + 0.001), size=(0.12, 0.06, 0.002),
             color=(0.05, 0.08, 0.10))

R = 0.0025  # 5 mm lead wire
LEAD_LEN = 0.50
CLIP = (0.022, 0.014, 0.012)  # along the lead, width, height
CLIP_MASS = 0.008
LEAD_Z = TOP + CLIP[2] / 2 + 0.0005
COLORS = {"white": (0.96, 0.96, 0.96), "black": (0.06, 0.06, 0.06), "red": (0.85, 0.08, 0.06)}
ROW_X = {"black": 0.23, "white": 0.17, "red": 0.11}  # leads run along -y at these x (far to near)
TARGET = {"white": "RA", "black": "LA", "red": "LL"}  # AHA 3-lead code


def _lead(c: str):
    return cable(f"lead_{c}", start=(ROW_X[c], MON_FACE_Y - 0.006, LEAD_Z), yaw=-math.pi / 2, length=LEAD_LEN,
                 radius=R, anchor_start=True, end_plug=Plug(size=CLIP, mass=CLIP_MASS, color=COLORS[c]),
                 color=COLORS[c], bend_stiffness=0.01, bend_damping=0.001)


LEADS = {c: _lead(c) for c in ("white", "black", "red")}
OBJECTS = [*MANIKIN, *(b for s in CUPS for b in _cup(s)), MONITOR, SCREEN, *LEADS.values()]

# -- scoring ---------------------------------------------------------------------------------------------------------
SEAT_XY = CUP_IN / 2  # clip centre within the cup's inner square (top view)
SEAT_DZ = 0.012  # clip centre at most 12 mm above the cup floor (lying in the cup, not perched on a wall)
RELEASE_DIST = 0.03


def cup_of(clip: np.ndarray | None) -> str | None:
    """The cup a clip lies in, or None."""
    if clip is None or clip[2] > BOARD_TOP + SEAT_DZ:
        return None
    for site, (cx, cy) in CUPS.items():
        if abs(clip[0] - cx) <= SEAT_XY and abs(clip[1] - cy) <= SEAT_XY:
            return site
    return None


def placement(objects) -> dict[str, str | None]:
    return {c: cup_of(objects[f"lead_{c}"].end_plug) for c in LEADS}


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_ecg_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._ecg_hooked = True
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
    """Both grippers at least 3 cm from every clip."""
    clips = [objects[f"lead_{c}"].end_plug for c in LEADS]
    return all(float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in grippers() for p in clips if p is not None)


def tally(objects) -> tuple[int, int]:
    """(clips in their own cup, clips in any other cup)."""
    pl = placement(objects)
    right = sum(pl[c] == TARGET[c] for c in LEADS)
    wrong = sum(pl[c] is not None and pl[c] != TARGET[c] for c in LEADS)
    return right, wrong


def score(objects) -> float:
    """0 if a gripper is within 3 cm of a clip, or if ANY clip lies in a wrong cup (including the unused RL cup).
    Otherwise 1/3 per clip lying in its own cup (white RA, black LA, red LL)."""
    if not released(objects):
        return 0.0
    right, wrong = tally(objects)
    if wrong:
        return 0.0
    return right / 3.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9 and all(objects[f"lead_{c}"].resting(max_speed=0.02) for c in LEADS)


def distance(objects) -> float:
    """Clips not in their own cup, plus 3 if any clip is in a wrong cup, plus 1 if a gripper still holds a clip."""
    right, wrong = tally(objects)
    return float(len(LEADS) - right) + 3.0 * (wrong > 0) + (0.0 if released(objects) else 1.0)


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------

GRASP_OPEN = -0.030  # finger opening for clip grasps (FINGER_OPEN is -0.047)
HANDOVER = (0.345, 0.0)  # midline spot on the torso where the right arm sets a clip down for the left arm
HI = TOP + 0.08


def _log(ctx, tag):
    st = ctx.states()
    parts = [f"{c}:{cup_of(st[f'lead_{c}'].end_plug)} {np.round(st[f'lead_{c}'].end_plug, 3)}" for c in LEADS]
    print(f"[demo t={ctx.t:5.1f}] {tag}: {' | '.join(parts)} rel={released(st)} score={score(st):.2f}", flush=True)


def _yaw_for(arm, ctx, d) -> float:
    """Grip yaw closing the fingers across a clip whose axis is d (fingers close along yaw + 90 deg)."""
    y = math.atan2(d[1], d[0])
    y = (y + math.pi / 2) % math.pi - math.pi / 2  # [-pi/2, pi/2)
    if arm is ctx.right and y < -1.2:
        y += math.pi
    if arm is ctx.left and y > 1.2:
        y -= math.pi
    return y


def _pick(ctx, arm, c):
    from yam_arena.constants import FINGER_CLOSED

    cs = ctx.states()[f"lead_{c}"]
    p = cs.end_plug
    yaw = _yaw_for(arm, ctx, cs.end_dir)
    yield from arm.move((p[0], p[1], HI), yaw=yaw, finger=GRASP_OPEN, duration=1.4)
    cs = ctx.states()[f"lead_{c}"]  # closed loop: the clip may have shifted
    p = cs.end_plug
    yield from arm.move((p[0], p[1], p[2] + 0.010), yaw=yaw, duration=0.9)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move((p[0], p[1], HI), yaw=yaw, duration=0.8)
    return yaw


def _put(ctx, arm, xy, yaw, floor):
    """Carry the held clip above xy and let it go with the fingertips 1 mm above ``floor`` + wall height."""
    yield from arm.move((xy[0], xy[1], HI), yaw=yaw, duration=1.4)
    yield from arm.move((xy[0], xy[1], floor + 0.013 + 0.001), yaw=yaw, duration=0.9)
    yield from arm.grip(GRASP_OPEN, 0.5)
    yield from arm.move((xy[0], xy[1], HI), yaw=yaw, duration=0.7)


def _to_cup(ctx, arm, c, site):
    yaw = yield from _pick(ctx, arm, c)
    yield from _put(ctx, arm, CUPS[site], yaw, BOARD_TOP + CUP_WALL_H)
    _log(ctx, f"{c} -> {site}")


def _via_handover(ctx, c, site):
    """Right arm takes clip c to the midline spot; the left arm picks it up there and drops it in a left-side cup."""
    yaw = yield from _pick(ctx, ctx.right, c)
    yield from _put(ctx, ctx.right, HANDOVER, yaw, BOARD_TOP)
    _log(ctx, f"{c} at handover")


def demo_script(ctx):
    """ECG_DEMO = correct (default) | nothing | mirror (patient's right taken as the robot's right: white LA, black RA,
    red RL) | swap (white and black exchanged: white LA, black RA, red LL) | hold (correct, but the left arm keeps
    holding the white clip in its cup)."""
    mode = os.environ.get("ECG_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    park_l, park_r = (0.15, 0.22, TOP + 0.16), (0.15, -0.22, TOP + 0.16)
    _log(ctx, f"start ({mode})")
    if mode == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return
    plan = {"correct": {"white": "RA", "black": "LA", "red": "LL"}, "hold": {"white": "RA", "black": "LA", "red": "LL"},
            "mirror": {"white": "LA", "black": "RA", "red": "RL"},
            "swap": {"white": "LA", "black": "RA", "red": "LL"}}[mode]
    # every clip starts on the robot's right (right arm); a clip bound for a cup on the robot's left (+y) is set down
    # on the torso midline by the right arm and picked up there by the left arm.
    cross = [c for c in ("black", "white", "red") if CUPS[plan[c]][1] > 0]
    direct = [c for c in ("black", "white", "red") if CUPS[plan[c]][1] < 0]

    def right_arm():
        for c in cross:
            yield from _via_handover(ctx, c, plan[c])
            yield from Rt.move(park_r, yaw=math.pi / 2, duration=1.0)
            yield from Rt.wait(0.1)
            _handed.append(c)
            # wait until the left arm has taken the clip off the midline spot
            while c not in _taken:
                yield
        for c in direct:
            yield from _to_cup(ctx, Rt, c, plan[c])
        yield from Rt.move(park_r, yaw=math.pi / 2, duration=1.0)

    def left_arm():
        for i, c in enumerate(cross):
            while c not in _handed:
                yield
            yaw = yield from _pick(ctx, L, c)
            _taken.append(c)
            last = i == len(cross) - 1
            if mode == "hold" and last:
                yield from L.move((CUPS[plan[c]][0], CUPS[plan[c]][1], HI), yaw=yaw, duration=1.4)
                yield from L.move((CUPS[plan[c]][0], CUPS[plan[c]][1], BOARD_TOP + CUP_WALL_H + 0.016), yaw=yaw,
                                  duration=0.9)
                continue
            yield from _put(ctx, L, CUPS[plan[c]], yaw, BOARD_TOP + CUP_WALL_H)
            _log(ctx, f"{c} -> {plan[c]}")
        if mode != "hold":
            yield from L.move(park_l, yaw=-math.pi / 2, duration=1.0)

    _handed: list[str] = []
    _taken: list[str] = []
    yield from ctx.both(right_arm(), left_arm())
    _log(ctx, "all placed")
    yield from L.wait(2.0)
    _log(ctx, "end")
