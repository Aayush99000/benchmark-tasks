"""Move an oven's wire rack down one rung: tilt the front up past the stop, slide it out, slide it back in one rung lower, level and home.

A Cuisinart Chef's Convection Toaster Oven TOB-260N1 at its true size: cavity about 374 x 330 x 190 mm, four rack positions, a stop on the upper two. The wire rack,
368 x 305 mm, 0.35 kg (assumed), has two side rods, a front wire, a rear wire and four cross wires, all 5 mm, and rests on position 3. Each side rod has a small
outward tab; a 4 mm stop on the ledge catches it about halfway out, and the rack comes out only if its front is lifted so the tab rides over the stop.

    python demokit.py task.py OUT --no-record --mode smoke       settle, grip the front strip with both hands, lift past the stop, slide part way out
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Move the oven rack down one rung."
TABLE = TABLE_TOP[SCENE]
BASE = float(os.environ.get("OVEN_BASE", "0.12"))  # the oven stands on a stand this high (a layout move: the arms reach the rack at a workable height)
TOP = TABLE + BASE  # the oven floor
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 20

XD = float(os.environ.get("OVEN_XD", "0.60"))  # the door line, x from the arm bases
# Cuisinart Chef's Convection Toaster Oven TOB-260N1, true size (sources and assumed figures in README.md)
CAV_W, CAV_D, CAV_H = 0.374, 0.330, 0.190  # inside, at the rack grooves: about 14.5 x 13 x 7.5 in
WALL = 0.012
PITCH = float(os.environ.get("OVEN_PITCH", "0.035"))  # rung pitch, ASSUMED (four positions in 190 mm)
Z0 = 0.040  # position 1 ledge top, above the cavity floor, ASSUMED
RUNGS = [Z0 + PITCH * k for k in range(4)]  # positions 1 to 4, bottom up
START_RUNG, GOAL_RUNG = 2, 1  # position 3 to position 2
STOP_RUNGS = (2, 3)  # the manual: "the upper positions have a stop feature so the rack stops halfway out"
LEDGE_Y0, LEDGE_Y1 = 0.172, 0.187  # groove ledge from the side wall inward, |y|
LEDGE_T = 0.006
STOP_X = XD + 0.10  # the stop's front face, about halfway along the rack
STOP_H, STOP_L = 0.004, 0.006  # ASSUMED
ROD = 0.005  # wire, modelled as a 5 mm square section
RACK_W, RACK_D = 0.368, 0.305  # 14.5 x 12 in
RACK_MASS = float(os.environ.get("RACK_MASS", "0.35"))  # ASSUMED, estimated 0.2 to 0.35 kg (wire gauge x length x steel density); demo verified at 0.35 and 0.45, fails at 0.6
STRIP_H, STRIP_T = ROD, ROD  # every wire is 5 mm; no added rail
GRIP_BACK = float(os.environ.get("GRIP_BACK", "0.0625"))  # the hands pinch the first cross wire, this far behind the front wire (0 = the front wire)
LOOKS = {"oven": m.Look(color=m.rgb(0.55, 0.57, 0.60), metallic=0.6, roughness=0.4), "ledge": m.Look(color=m.rgb(0.45, 0.47, 0.50), metallic=0.7, roughness=0.35),
         "rack": m.Look(color=m.rgb(0.80, 0.80, 0.82), metallic=0.9, roughness=0.25), "stop": m.Look(color=m.rgb(0.75, 0.30, 0.20), roughness=0.5)}


def _box(parts, cols, name, size, centre, look):
    parts.append((name, m.box(size, centre), look))
    cols.append(m.col_box(f"{name}_c", size, centre))


def _oven() -> m.Part:
    parts, cols = [], []
    xm = XD + CAV_D / 2
    for k, sy in enumerate((-1, 1)):
        _box(parts, cols, f"side{k}", (CAV_D, WALL, CAV_H + 2 * WALL), (xm, sy * (CAV_W / 2 + WALL / 2), CAV_H / 2), "oven")
    _box(parts, cols, "floor", (CAV_D, CAV_W, WALL), (xm, 0.0, -WALL / 2 + 0.0), "oven")
    _box(parts, cols, "roof", (CAV_D, CAV_W, WALL), (xm, 0.0, CAV_H + WALL / 2), "oven")
    _box(parts, cols, "back", (WALL, CAV_W + 2 * WALL, CAV_H + 2 * WALL), (XD + CAV_D + WALL / 2, 0.0, CAV_H / 2), "oven")
    lx0, lx1 = XD + 0.02, XD + CAV_D
    for r, zt in enumerate(RUNGS):
        for k, sy in enumerate((-1, 1)):
            yc = sy * (LEDGE_Y0 + LEDGE_Y1) / 2
            _box(parts, cols, f"ledge{r}_{k}", (lx1 - lx0, LEDGE_Y1 - LEDGE_Y0, LEDGE_T), ((lx0 + lx1) / 2, yc, zt - LEDGE_T / 2), "ledge")
            if r in STOP_RUNGS:
                _box(parts, cols, f"stop{r}_{k}", (STOP_L, 0.0025, STOP_H), (STOP_X - STOP_L / 2, sy * (LEDGE_Y1 - 0.00125), zt + STOP_H / 2), "stop")
    _box(parts, cols, "stand", (CAV_D + 0.02, CAV_W + 0.06, BASE), (xm, 0.0, -BASE / 2), "oven")
    return m.fixed("oven", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.45))


# the rack frame: origin at the rack's centre, rods' bottom at local z = -ROD/2 (it sits on a ledge top at rod bottom)
RACK0 = np.array([XD + 0.012 + RACK_D / 2, 0.0, TOP + RUNGS[START_RUNG] + ROD / 2 + 0.0005])
TAB_X = -RACK_D / 2 + 0.25  # the tab's centre, 25 cm behind the front


def _rack() -> m.Part:
    parts, cols = [], []
    for k, sy in enumerate((-1, 1)):
        _box(parts, cols, f"rod{k}", (RACK_D, ROD, ROD), (0.0, sy * (RACK_W / 2 - ROD / 2), 0.0), "rack")
        _box(parts, cols, f"tab{k}", (0.010, 0.0028, 0.005), (TAB_X, sy * (RACK_W / 2 + 0.0014), 0.0), "rack")  # outward from the rod, 6 mm tall, at stop height
    _box(parts, cols, "front", (STRIP_T, RACK_W, STRIP_H), (-RACK_D / 2 + STRIP_T / 2, 0.0, STRIP_H / 2 - ROD / 2), "rack")
    _box(parts, cols, "rear", (ROD, RACK_W, ROD), (RACK_D / 2 - ROD / 2, 0.0, 0.0), "rack")
    for k, xc in enumerate((-0.09, -0.03, 0.03, 0.09)):
        _box(parts, cols, f"cross{k}", (ROD, RACK_W, ROD), (xc, 0.0, 0.0), "rack")
    return m.free("rack", tuple(RACK0), parts, cols, LOOKS, mass=RACK_MASS, friction=(0.5, 0.45), torsional_patch=None)


OBJECTS = [_oven(), _rack()]


def measure(s) -> dict:
    rot = s.rotation()
    pitch = math.degrees(math.asin(-float(np.clip(rot[2, 0], -1, 1))))
    roll = math.degrees(math.atan2(rot[2, 1], rot[2, 2]))
    yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    zr = float(s.pos[2] - TOP - ROD / 2)  # rod bottom above the table
    rung = int(np.argmin([abs(zr - z) for z in RUNGS]))
    front = float(s.pos[0] - RACK_D / 2)
    return {"x": round(float(s.pos[0]), 4), "y": round(float(s.pos[1]), 4), "z_rod_mm": round(zr * 1000, 1), "rung": rung, "on_rung": abs(zr - RUNGS[rung]) < 0.004,
            "pitch": round(pitch, 2), "roll": round(roll, 2), "yaw": round(yaw, 2), "front_out_mm": round((XD - front) * 1000, 1)}


class Tracker:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.last, self.hold, self.phase, self.grip = 0, {}, 0, "settle", None
        self.out, self.dropped, self.clear, self.seated, self.home, self.level = False, False, False, False, False, False

    def step(self, env):
        self.steps += 1
        s = m.state(env, "rack")
        r = measure(s)
        tips = m.fingertips(env)
        local = (tips - s.pos) @ s.rotation()
        v = float(np.linalg.norm(s.lin_vel))
        slip = None
        if self.phase in ("hold", "lift", "slide") and self.grip is None:
            near = np.where(np.linalg.norm(local - np.array([-RACK_D / 2, 0.0, 0.0]) * 1.0, axis=1) < 0.35)[0]
            # the tips within 25 mm of the front strip's box
            box = np.maximum(np.abs(local - np.array([-RACK_D / 2 + GRIP_BACK + STRIP_T / 2, 0.0, STRIP_H / 2 - ROD / 2])) - np.array([STRIP_T / 2, RACK_W / 2, STRIP_H / 2]), 0.0)
            idx = np.where(np.linalg.norm(box, axis=1) < 0.025)[0]
            self.grip = (idx, local[idx].copy())
        if self.grip is not None and len(self.grip[0]):
            slip = float(np.max(np.linalg.norm(local[self.grip[0]] - self.grip[1], axis=1)))
        self.last = {**r, "v": round(v, 4), "slip_mm": None if slip is None else round(slip * 1000, 2), "n_grip_tips": 0 if self.grip is None else len(self.grip[0]), "phase": self.phase}
        clear = bool(np.min(np.linalg.norm(np.maximum(np.abs(local) - np.array([RACK_D / 2, RACK_W / 2, 0.006]), 0.0), axis=1)) > 0.03)
        self.clear = clear
        if r["front_out_mm"] >= 150.0:
            self.out = True  # past the stop (it holds the rack at about 125 mm)
        if r["z_rod_mm"] < RUNGS[0] * 1000 - 15.0:
            self.dropped = True  # fell below the lowest ledge
        seated = r["rung"] == GOAL_RUNG and abs(r["z_rod_mm"] / 1000 - RUNGS[GOAL_RUNG]) < 0.003 and r["front_out_mm"] < 150.0 and v < 0.02
        home = -30.0 <= r["front_out_mm"] <= 20.0
        level = abs(r["pitch"]) <= 2.0 and abs(r["roll"]) <= 2.0
        self.seated, self.home, self.level = seated, home, level
        self.hold = self.hold + 1 if (seated and home and level and clear and not self.dropped) else 0
        self.last.update(hold=self.hold, out=self.out, dropped=self.dropped, seated=seated, clear=clear)
        if self.steps % 5 == 0:
            print("[rack] " + json.dumps({"t": round(self.steps * float(env.unwrapped.step_dt), 2), **self.last}), flush=True)


TRACK = Tracker()
m.on_step(TRACK.step)
m.on_reset(TRACK.reset)


def score(objects) -> float:
    """By effect: 0.25 once the rack has come out past the stop; 0.5 while it rests on the lower rung inside the oven with both hands clear; 1.0 after it has also been level
    (within 2 degrees), home (front strip within 20 mm of the door line) and never dropped, for HOLD_STEPS steps. A dropped rack scores 0."""
    if TRACK.hold >= HOLD_STEPS:
        return 1.0
    if not TRACK.last or TRACK.dropped:
        return 0.0
    if TRACK.seated and TRACK.clear:
        return 0.5
    return 0.25 if TRACK.out else 0.0


def success(objects) -> bool:
    return TRACK.hold >= HOLD_STEPS


# -- the scripted demo --------------------------------------------------------------------------

LIFT_DEG = float(os.environ.get("RACK_LIFT_DEG", "9"))
REL_DZ = float(os.environ.get("REL_DZ", "-0.012"))  # the hands rise (+) or drop (-) this much after letting go, before drawing back
REL_T = float(os.environ.get("REL_T", "1.2"))
OUT1 = float(os.environ.get("RACK_OUT1", "0.165"))  # pulled level until the tab meets the stop (162 mm travel)


def _pinch():
    g = d.tilted(90.0, 0.0)
    return d._as_grip(g.rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi / 2))


def demo_script(ctx):
    s = ctx.pos("rack")
    arms = [("left", 0.09), ("right", -0.09)]
    depth = 0.0133 + STRIP_T / 2  # the fingertips on the wire
    front = s[0] - RACK_D / 2 + GRIP_BACK  # the wire gripped
    zs = s[2] + STRIP_H / 2 - ROD / 2  # the strip's middle
    TRACK.phase = "settle"
    yield from ctx.left.wait(2.0)
    print("[rack] SETTLED " + json.dumps(TRACK.last), flush=True)

    def grip_at(xf, y, z):
        return np.array([xf + depth - 0.0133, y, z])

    TRACK.phase = "approach"
    yield from ctx.together(*[getattr(ctx, a).move(grip_at(min(front - 0.03, XD - 0.06), y, zs + 0.07), grip=_pinch(), finger=d.FINGER_OPEN, t=2.5, label="above, in front of the strip") for a, y in arms])
    yield from ctx.together(*[getattr(ctx, a).move(grip_at(min(front - 0.03, XD - 0.06), y, zs), t=1.2, label="down in front of the strip") for a, y in arms])
    yield from ctx.together(*[getattr(ctx, a).move(grip_at(front, y, zs), t=1.2, label="fingers over and under the strip") for a, y in arms])
    yield from ctx.together(*[getattr(ctx, a).close(STRIP_H, force=45.0, label="pinch") for a, _ in arms])
    TRACK.phase = "hold"
    yield from ctx.left.wait(0.5)
    TRACK.phase = "slide"
    yield from ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array([-OUT1, 0.0, 0.0]), t=2.0, label="slide out level to the stop") for a, _ in arms])
    print("[rack] AT_STOP " + json.dumps(TRACK.last), flush=True)
    TRACK.phase = "lift"
    st = ctx.state("rack")
    piv = np.array([st.pos[0] + RACK_D / 2, 0.0, st.pos[2]])  # the rear rod, the rack's rest point
    yield from ctx.together(*[getattr(ctx, a).arc(piv, np.array([0.0, 1.0, 0.0]), LIFT_DEG, t=1.5, label="lift the front past the stop") for a, _ in arms])
    print("[rack] LIFTED " + json.dumps(TRACK.last), flush=True)
    mode = ctx.mode
    goal = START_RUNG if "upper" in mode else GOAL_RUNG
    TRACK.phase = "slide"

    def hands_move(delta, t, label):
        return ctx.together(*[getattr(ctx, a).move(getattr(ctx, a).at() + np.array(delta), t=t, label=label) for a, _ in arms])

    # tilted, on out until the rear rod is clear of the ledge tips
    st = ctx.state("rack")
    yield from hands_move([(XD - 0.16) - float(st.pos[0]) , 0.0, 0.0], 3.0, "slide on out, tilted, until clear of the ledges")
    print("[rack] CLEAR " + json.dumps(TRACK.last), flush=True)
    TRACK.phase = "level"

    def zrod():
        return float(ctx.state("rack").pos[2]) - TOP - ROD / 2

    def level_and_height(target, tol=0.004):
        """Feedback on the observed pose, by taking out the pitch about the rack's centre, then close the height error."""
        for k in range(3):
            st = ctx.state("rack")
            rot = m.quat_matrix(st.quat_xyzw)
            pitch = math.degrees(math.asin(-float(np.clip(rot[2, 0], -1, 1))))
            err = target - zrod()
            if abs(pitch) < 1.0 and abs(err) < tol:
                break
            if abs(pitch) >= 1.0:
                yield from ctx.together(*[getattr(ctx, a).arc(np.asarray(st.pos, float), np.array([0.0, 1.0, 0.0]), -pitch, t=0.7, label="take out the pitch") for a, _ in arms])
            err = target - zrod()
            if abs(err) >= tol:
                yield from hands_move([0.0, 0.0, 0.7 * err], 0.7, "correct the height")

    CLR = float(os.environ.get("RACK_CLR", "0.012"))  # rod bottom this far over the ledge while the tabs pass the stops (they need 8.5 mm)
    yield from level_and_height(RUNGS[goal] + CLR)
    print("[rack] LEVEL " + json.dumps(TRACK.last), flush=True)
    TRACK.phase = "in"
    st = ctx.state("rack")
    xt = XD + 0.16
    x0 = float(ctx.state("rack").pos[0])
    for k in (1, 2, 3):
        yield from hands_move([(x0 + (xt - x0) * k / 3.0) - float(ctx.state("rack").pos[0]), 0.0, 0.0], 1.2, "slide in level, over the stops")
        yield from level_and_height(RUNGS[goal] + CLR)
        print("[rack] IN" + str(k) + " " + json.dumps(TRACK.last), flush=True)
    TRACK.phase = "seat"
    yield from level_and_height(RUNGS[goal] + 0.0008, tol=0.003)
    print("[rack] SEATED " + json.dumps(TRACK.last), flush=True)
    st = ctx.state("rack")
    home_x = RACK0[0] - 0.004 - (0.10 if "short" in mode else 0.0)  # pushed home, the front wire 8 mm inside the door line; sliding the fingers off draws it out about 15 mm
    yield from hands_move([home_x - float(st.pos[0]), 0.0, 0.0], 3.0, "push home")
    if "tilted" in mode:
        st = ctx.state("rack")
        yield from ctx.together(*[getattr(ctx, a).arc(np.array([st.pos[0] + RACK_D / 2, 0.0, st.pos[2]]), np.array([0.0, 1.0, 0.0]), 5.0, t=1.0, label="tilt it up") for a, _ in arms])
    TRACK.phase = "release"
    yield from ctx.together(*[getattr(ctx, a).open(label="let go") for a, _ in arms])
    yield from ctx.left.wait(0.8)
    if REL_DZ:
        yield from hands_move([0.0, 0.0, REL_DZ], 0.8, "clear the wire")
    yield from hands_move([-0.06, 0.0, 0.0], REL_T, "fingers off the wire")
    yield from hands_move([-0.04, 0.0, 0.06], 1.0, "hands away")
    yield from ctx.left.wait(1.5)
    print("[rack] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
