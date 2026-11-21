"""Raise an office chair's seat to 53 cm by holding its gas-lift lever up as the seat rises, and let go on the height.

An IKEA MARKUS office chair (listed 70 x 70 cm, seat 57 x 49 cm, seat height 47 to 58 cm) stands on the bench, centred at (56, -22.5) cm, facing -y, its right side towards the robot, the seat at its lowest.
Under the seat's right edge a black lever paddle (12 x 3 x 0.8 cm, estimate) is hinged at its inner end, its tip 3 cm beyond the seat's side; it lifts 10 mm at the tip against a return spring (12 N at the tip, estimate).
The gas lift is code (estimate): while the paddle is lifted past 6 mm the column raises the seat at 8 cm/s to its 11 cm stroke; with the paddle below 6 mm the column locks where it is. The seat cannot be pushed down.
A 60 cm steel ruler stands in a weighted block 3 cm in front of the seat's front edge. The chair's base is fixed to the bench (rubber caster cups). Its meshes are chairmesh.py's; the colliders are boxes and cylinders.

    python demokit.py task.py OUT --no-record --mode full,none,overshoot,hand,early,undo
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import chairmesh as cm  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Raise the chair's seat so its front edge is 53 cm above the bench top, by the ruler beside it."
SUCCESS_HOLD_S = 0.0  # the scorer holds its own 1 s
SCORE_OVER = "end"
TOP = d.TABLE_TOP
RUN_STEPS = 280  # task.json's run limit

CX0, CY0 = 0.56, -0.225  # chair centre
CX, CY = CX0 + float(os.environ.get("OC_DX", "0")), CY0 + float(os.environ.get("OC_DY", "0"))  # a jitter run shifts the chair and ruler; the demo still aims at the nominal place
SEAT_LOW = 0.47  # the seat's front edge top, at its lowest
TARGET = 0.53
STROKE = 0.11
RISE = 0.08  # m/s while the valve is open
VALVE_MM = 6.0  # lever tip lift that opens the valve
LEVER_L = 0.12
LEVER_TRAVEL_MM = 10.0
LEVER_DEG = math.degrees(math.asin(LEVER_TRAVEL_MM / 1000 / LEVER_L))
LEVER_K = 12.0 * LEVER_L / math.radians(LEVER_DEG)  # N m/rad: 12 N at the tip at the stop
HINGE = (-0.195, -0.08, 0.41)  # the lever's hinge, in the chair's frame (origin: centre, on the bench)
CLEAR = 0.03

LOOKS = {"frame": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.5), "seat": m.Look(m.rgb(0.16, 0.16, 0.17), roughness=0.9), "mesh": m.Look(m.rgb(0.22, 0.23, 0.25), roughness=0.95),
         "steel": m.Look(m.rgb(0.72, 0.72, 0.74), metallic=0.8, roughness=0.35), "paddle": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.4), "block": m.Look(m.rgb(0.35, 0.35, 0.37), metallic=0.5, roughness=0.5),
         "red": m.Look(m.rgb(0.85, 0.1, 0.1), roughness=0.5),
         "wheel": m.Look(m.rgb(0.07, 0.07, 0.08), roughness=0.8), "chrome": m.Look(m.rgb(0.8, 0.8, 0.82), metallic=0.9, roughness=0.25)}


def _chair() -> m.Part:
    """Colliders: boxes and cylinders, unchanged. Visuals: the MARKUS meshes of chairmesh.py (built from IKEA's dimensions and photos)."""
    fp, fc = [("hub", cm.hub(), "frame"), ("outer", cm.column_outer(), "frame")], []
    fc.append(m.col_cylinder("hub_c", 0.04, 0.02, (0, 0, 0.075)))
    fc.append(m.col_cylinder("outer_c", 0.038, 0.23, (0, 0, 0.19)))
    for i in range(5):
        ang = 72 * i + 90
        leg = m.box((0.31, 0.05, 0.03), (0.17, 0, 0.065)).turned(ang)
        fp.append((f"leg{i}", cm.star_leg(ang), "frame"))
        fc.append(m.col_hull(f"leg{i}_c", leg))
        c = np.array([0.31 * math.cos(math.radians(ang)), 0.31 * math.sin(math.radians(ang))])
        fp += [(f"caster{i}_{n}", mesh, look) for n, mesh, look in cm.caster(ang)]
        fc.append(m.col_cylinder(f"caster{i}_c", 0.025, 0.05, (c[0], c[1], 0.025)))
    # the seat: column (inner tube) and its cover, mechanism, shell, cushion, back (mesh, frame, headrest, spine), armrests
    sp = [("column", m.cylinder(0.027, 0.20, (0, 0, 0.31)), "chrome"), ("shroud", cm.column_shroud(), "frame"), ("housing", cm.mechanism_housing(), "frame"),
          ("bracket", cm.slab(-0.145, HINGE[1], 0.05, 0.015, 0.418, lambda x, y: np.full_like(x, 0.43), n=8, round_t=0.8), "frame"),
          ("shell", cm.seat_shell(), "frame"), ("cushion", cm.seat_cushion(), "seat"),
          ("back", cm.back_panel(cm.BACK_Z0, cm.BACK_Z1, 0.008), "mesh"), ("backframe", cm.back_frame(), "frame"),
          ("headrest", cm.back_panel(1.13, cm.BACK_Z1 - 0.004, 0.034, nu=18, nv=12, bulge=0.010), "seat"), ("spine", cm.back_spine(), "frame")]
    for sx in (-1, 1):
        sp += [(n, mesh, look) for n, mesh, look in cm.armrest(sx)]
    sc = [m.col_box("pan_c", (0.57, 0.49, 0.04), (0, 0, 0.45)), m.col_box("housing_c", (0.20, 0.20, 0.04), (0, 0, 0.41)), m.col_box("back_c", (0.50, 0.05, 0.79), (0, 0.24, 0.895))]
    seat = m.slider("seat", "Z", 0.0, STROKE, sp, sc, mass=8.0, spring=20000.0, damping=800.0, effort=200.0, friction=0.5, parent=None)
    pad_c = (HINGE[0] - 0.06, HINGE[1], HINGE[2])
    pad = cm.lever_paddle(LEVER_L, 0.03, 0.008, pad_c)
    lever = m.hinge("lever", "Y", HINGE, 0.0, LEVER_DEG, [("paddle", pad, "paddle")], [m.col_box("paddle_c", (LEVER_L, 0.03, 0.008), pad_c)], mass=0.05, spring=LEVER_K, damping=0.3, effort=50.0, friction=0.0, parent="seat")
    return m.mechanism("chair", (CX, CY, TOP), [seat, lever], LOOKS, frame_parts=fp, frame_colliders=fc, friction=(0.8, 0.8), solver_iters=64)


def _ruler() -> m.Part:
    bx, by = CX, CY - 0.245 - 0.03
    parts = [("block", m.box((0.05, 0.05, 0.02), (0, 0, 0.01)), "block"), ("ruler", m.box((0.025, 0.003, 0.60), (0, 0, 0.32)), "steel"), ("mark", m.box((0.026, 0.0035, 0.004), (0, 0, TARGET)), "red")]
    cols = [m.col_box("block_c", (0.05, 0.05, 0.02), (0, 0, 0.01))]
    return m.fixed("ruler", (bx, by, TOP), parts, cols, LOOKS)


OBJECTS = [_chair(), _ruler()]


# -- the scorer ---------------------------------------------------------------------------------------------------------------------------------------------------------


def lever_mm(joint_rad: float) -> float:
    return 1000.0 * LEVER_L * math.sin(joint_rad)


def seat_h(joint_m: float) -> float:
    return SEAT_LOW + joint_m


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.tgt = 0.0
        self.open = False
        self.quiet = 0
        self.score = 0.0
        self.prev_h = None
        self.last = {}
        self.engaged = False

    def step(self, env):
        ch = m.state(env, "chair")
        q = float(ch.joint("seat"))
        lv = lever_mm(float(ch.joint("lever")))
        self.n += 1
        # the gas lift: while the valve is open the target runs ahead at RISE; else the column holds where it is
        if lv >= VALVE_MM:
            if not self.open:
                self.tgt = q
            self.open = True
            self.tgt = min(STROKE, self.tgt + RISE / 20.0, q + 0.012)
        else:
            if self.open or self.n == 1:
                self.tgt = q
            self.open = False
        m.set_target(env, "chair", "seat", self.tgt)
        h = seat_h(q)
        released = lv < 1.0
        speed = 0.0 if self.prev_h is None else abs(h - self.prev_h)
        self.prev_h = h
        still = speed < 0.0003
        tips = m.fingertips(env)
        loc = tips - np.array([CX, CY, TOP + q])  # in the seat's frame (the lever's small tilt is ignored)
        pad = np.array([HINGE[0] - 0.06, HINGE[1], HINGE[2]])
        dp = float(np.min(np.linalg.norm(np.maximum(np.abs(loc - pad) - np.array([LEVER_L / 2, 0.015, 0.004]), 0.0), axis=1)))
        dseat = float(np.min(np.linalg.norm(np.maximum(np.abs(loc - np.array([0.0, 0.0, 0.45])) - np.array([0.285, 0.245, 0.02]), 0.0), axis=1)))
        clear = min(dp, dseat) >= CLEAR
        if min(dp, dseat) < 0.02:
            self.engaged = True
        self.quiet = self.quiet + 1 if (released and still) else 0
        band1 = abs(h - TARGET) <= 0.01
        band2 = abs(h - TARGET) <= 0.025
        s = 0.0
        if h - SEAT_LOW >= 0.02:
            s = 0.25
        if band2 and self.quiet >= 20:
            s = 0.5
        if band1 and self.quiet >= 20 and clear and self.engaged:
            s = 1.0
        self.score = s
        self.last = {"h_cm": round(h * 100, 2), "lever_mm": round(lv, 2), "open": self.open, "clear": clear, "quiet": self.quiet, "score": s}
        if self.n % 10 == 0:
            print("[oc] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "n": self.n, **self.last}), flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.score


def success(objects) -> bool:
    # The end state, not a latch: true only at the run limit, if the seat then still scores 1.0 (stopped in the band, lever let go, hands clear, still for 1 s).
    # A seat that was set and then raised out of the band again scores its current tier. score() is read at the end either way.
    return TRIAL.n >= RUN_STEPS and TRIAL.score >= 1.0


# -- the scripted demo --------------------------------------------------------------------------------------------------------------------------------------------------

LEAD = float(os.environ.get("OC_LEAD", "0.008"))  # release this far below the height: the hand's lag at 8 cm/s
HOLD_MM = float(os.environ.get("OC_HOLD", "8.0"))
KP = float(os.environ.get("OC_KP", "0.25"))
Z0 = float(os.environ.get("OC_Z0", "0.385"))
X_IN = float(os.environ.get("OC_X", "0.30"))
YAW = os.environ.get("OC_GRIP", "level")
PADDLE_Y = CY0 + HINGE[1]


def _grip():
    return d.level(0.0)


def _read(ctx):
    s = ctx.state("chair")
    return float(s.joint("seat")), lever_mm(float(s.joint("lever")))


def _pull(ctx, R, grip, h_rel, first):
    """Slide under the paddle, find it by its first 1.5 mm of lift, hold it at HOLD_MM while the seat rises to h_rel, let go and draw back."""
    q0, _ = _read(ctx)
    z0 = TOP + Z0 + q0
    if first:
        yield from R.move((X_IN - 0.12, PADDLE_Y, z0), grip=grip, finger=d.FINGER_CLOSED, label="beside the paddle")
    else:
        yield from R.move((X_IN - 0.12, PADDLE_Y, z0 - 0.02), label="beside the paddle again")
        yield from R.move((X_IN - 0.12, PADDLE_Y, z0), label="level with the paddle")
    yield from R.move((X_IN, PADDLE_Y, z0), t=1.5, label="under the paddle")
    box = {"z": z0}
    for _ in range(200):
        _, lv = _read(ctx)
        if lv >= 1.5:
            break
        box["z"] += 0.0006
        yield from R.follow(lambda: (X_IN, PADDLE_Y, box["z"]), t=0.05, speed=0.3)
    print(f"[oc] touched the paddle at z {box['z'] - TOP:.4f}, lever {_read(ctx)[1]:.2f} mm", flush=True)
    prev_q = _read(ctx)[0]
    for _ in range(400):
        q, lv = _read(ctx)
        if SEAT_LOW + q >= h_rel:
            break
        box["z"] += (q - prev_q) + KP * (HOLD_MM - lv) / 1000.0
        prev_q = q
        yield from R.follow(lambda: (X_IN, PADDLE_Y, box["z"]), t=0.05, speed=0.4)
    q, lv = _read(ctx)
    print(f"[oc] release at h {100 * (SEAT_LOW + q):.2f} cm, lever {lv:.2f} mm", flush=True)
    yield from R.move(R.at() + [0.0, 0.0, -0.03], t=0.25, label="let go")
    yield from R.move(R.at() + [-0.10, 0.0, 0.0], t=0.8, label="back")


def demo_script(ctx):
    """Modes: full. none: nothing. overshoot: the lever held until the seat tops out. hand: the seat pushed up from beneath, the lever untouched. early: let go at 50 cm. undo: set at 53 as in full, then the lever lifted again until the seat is out of the band (56.5 cm)."""
    mode = ctx.mode
    if mode not in ("full", "none", "overshoot", "hand", "early", "undo"):
        raise ValueError(f"unknown mode {mode!r}")
    R = ctx.right
    if mode == "none":
        yield from R.wait(3.0)
        return
    grip = _grip()
    if mode == "hand":
        # a pad under the seat's pan, 12 cm in front of the lever, pushed up at 20 N for 3 s
        yield from R.move((X_IN - 0.08, PADDLE_Y - 0.12, TOP + 0.37), grip=grip, finger=d.FINGER_CLOSED, label="to the seat's side")
        yield from R.move((X_IN, PADDLE_Y - 0.12, TOP + 0.37), t=1.5, label="under the pan")
        res = yield from R.press((0.0, 0.0, 1.0), 20.0, 0.06, speed=0.02, settle=3.0, timeout=7.0)
        print(f"[oc] hand pushed {res}", flush=True)
        yield from R.wait(1.0)
        yield from R.move(R.at() + [-0.10, 0.0, -0.03], t=1.0)
        yield from R.wait(2.0)
        print("[oc] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    h_rel = {"full": TARGET - LEAD, "undo": TARGET - LEAD, "early": 0.50 - LEAD, "overshoot": SEAT_LOW + STROKE - 0.004}[mode]
    yield from _pull(ctx, R, grip, h_rel, first=True)
    if mode == "undo":
        yield from R.wait(1.5)
        yield from _pull(ctx, R, grip, 0.565 - LEAD, first=False)
    yield from R.wait(2.0)
    print("[oc] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
