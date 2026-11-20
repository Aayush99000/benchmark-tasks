"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object: state},
expected score, expected success); states are ready PartStates from the parts' own geometry. Cases run in order and
share the stillness count (a moved gauge restarts it).

Offline there is no robot and no physics. The arms' measured clearance is read from a mark on the gauge's state
(default: measured clear); in the simulator it is measured every step and is None, no credit, until then. The air
rule that drives the stick (task.air_step) is checked directly at the end of this file, on import.
"""

import math

import numpy as np

import parts
import task
from task import CRADLE, GAUGE, GAUGE_MID, REST_Z, STILL_STEPS, TOP, WHEEL, D, TIP_W, GASKET_X


def _offline_clear(objects):
    g = objects["gauge"]
    # Offline fixtures describe the air-driven history; the simulator obtains it only from air_step.
    task._stick_air["target"] = getattr(g, "_offline_air_target", 0.0)
    pts = getattr(g, "_offline_points", None)
    if pts is not None:
        return task.arms_clear(pts, task._gauge_boxes(g))
    return getattr(g, "_offline_clear", True)


parts.__dict__.setdefault("_gauge_task_grip", {})["offline_clear"] = _offline_clear


def ext(psi):
    return (psi - task.PSI_ZERO) * task.MM_PER_PSI


def gauge(psi=10.0, pos=None, quat=(0.0, 0.0, 0.0, 1.0), vel=None, clear=True, points=None, air_psi=32.0):
    st = GAUGE.state(joints={"stick": ext(psi)}, pos=tuple(pos) if pos is not None else None, quat_xyzw=tuple(quat),
                     lin_vel=vel)
    object.__setattr__(st, "_offline_clear", clear)
    object.__setattr__(st, "_offline_air_target", ext(air_psi))
    if points is not None:
        object.__setattr__(st, "_offline_points", points)
    return {"gauge": st}


def in_cradle(psi, dx=0.0, dy=0.0, dz=0.0, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, **kw):
    """The gauge with its middle at the cradle's middle (offset dx, dy, dz), turned yaw/pitch degrees about it."""
    r = (parts._rot((0, 0, 1), math.radians(yaw_deg)) @ parts._rot((0, 1, 0), math.radians(pitch_deg))
         @ parts._rot((1, 0, 0), math.radians(roll_deg)))
    mid = np.array([CRADLE.pos[0] + dx, CRADLE.pos[1] + dy, TOP + REST_Z + dz])
    pos = mid - r @ np.array([GAUGE_MID, 0.0, 0.0])
    return gauge(psi, pos=pos, quat=tuple(parts._matrix_to_quat(r)), **kw)


def on_valve(psi, **kw):
    """The gauge seated on the valve: gasket face on the barrel's tip, along the stem."""
    lean = math.degrees(math.acos(D[2]))
    r = parts._rot((0, 1, 0), -math.radians(90.0 - lean))
    pos = TIP_W - D * GASKET_X
    return gauge(psi, pos=pos, quat=tuple(parts._matrix_to_quat(r)), **kw)


def plate(gap):
    """A flat 120 x 120 mm arm face lying level over the gauge in its cradle, `gap` above the barrel's top: its
    corners are all about 60 mm from the gauge, only the face itself is near. Given as the scorer's sample points."""
    h = np.array([0.060, 0.060, 0.001])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    planes = np.array([[1, 0, 0, h[0]], [-1, 0, 0, h[0]], [0, 1, 0, h[1]], [0, -1, 0, h[1]], [0, 0, 1, h[2]],
                       [0, 0, -1, h[2]]], float)
    centre = np.array([CRADLE.pos[0], CRADLE.pos[1], TOP + REST_Z + task.HEAD_R + gap + h[2]])
    return [task._face_points(corners, planes) + centre]


def held_still(label, state, score):
    """A moving step (restarts the count), then the state held still until it has been still for 1 s."""
    moving = {k: v for k, v in state.items()}
    g = state["gauge"]
    mv = GAUGE.state(joints=dict(g.joints), pos=tuple(np.asarray(g.pos) + (0.0, 0.0025, 0.0)),
                     quat_xyzw=tuple(g.quat_xyzw), lin_vel=(0.0, 0.05, 0.0))  # 2.5 mm away: moving
    for k in ("_offline_clear", "_offline_points", "_offline_air_target"):
        if hasattr(g, k):
            object.__setattr__(mv, k, getattr(g, k))
    moving["gauge"] = mv
    partial = 0.5 if score == 1.0 else score
    out = [(f"{label}: still moving", moving, partial, False)]
    out += [(f"{label} (still, step {k + 1})", state, partial, False) for k in range(STILL_STEPS)]
    out.append((f"{label} (still 1 s)", state, score, score == 1.0))
    return out


R = in_cradle(32.0)
CASES = [
    ("start: gauge in its cradle, stick in (reads nothing)", {}, 0.0, False),
    *held_still("back in the cradle, never used: stick in", in_cradle(10.0), 0.0),
    *held_still("back in the cradle, a leaking press: reads 12 psi", in_cradle(12.0), 0.0),
    *held_still("back in the cradle, reads 29.9 psi", in_cradle(29.9), 0.0),
    *held_still("back in the cradle, reads 34.1 psi", in_cradle(34.1), 0.0),
    *held_still("stick manually drawn to 32 psi, no air history", in_cradle(32.0, air_psi=10.0), 0.0),
    *held_still("stick manually drawn to 32 psi after only a leaking press", in_cradle(32.0, air_psi=12.0), 0.0),
    *held_still("stick manually drawn to 32 psi after a short press below 30 psi", in_cradle(32.0, air_psi=29.9), 0.0),
    *held_still("back in the cradle, reads exactly 30 psi", in_cradle(30.0), 1.0),
    *held_still("back in the cradle, reads exactly 34 psi", in_cradle(34.0), 1.0),
    *held_still("back in the cradle, reads 32 psi", R, 1.0),
    *held_still("reads 32, in the cradle the other way round", in_cradle(32.0, yaw_deg=180.0), 1.0),
    *held_still("reads 32, still held in the gripper (fail closed)", in_cradle(32.0, clear=False), 0.0),
    *held_still("reads 32, arms not yet measured (fail closed)", in_cradle(32.0, clear=None), 0.0),
    *held_still("reads 32, a flat arm face resting 0.5 mm over the gauge, no corner near",
                in_cradle(32.0, points=plate(0.0005)), 0.0),
    *held_still("reads 32, the same face 6 mm over the gauge: clear", in_cradle(32.0, points=plate(0.006)), 1.0),
    *held_still("reads 32, lying on the table beside the cradle", gauge(32.0, pos=(0.20, -0.25, TOP + task.HEAD_R)),
                0.5),
    *held_still("reads 32, left hanging on the valve", on_valve(32.0), 0.5),
    *held_still("reads 32, in the cradle but 5 mm to one side (on a saddle wall)", in_cradle(32.0, dy=0.005), 0.5),
    *held_still("reads 32, 4.5 mm to one side: in", in_cradle(32.0, dy=0.0045), 1.0),
    *held_still("reads 32, 13 mm along from its place (off a saddle)", in_cradle(32.0, dx=0.013), 0.5),
    *held_still("reads 32, propped 3 mm high", in_cradle(32.0, dz=0.003), 0.5),
    *held_still("reads 32, tilted 7 degrees in the cradle", in_cradle(32.0, pitch_deg=7.0), 0.5),
    *held_still("reads 32, turned 7 degrees across the cradle", in_cradle(32.0, yaw_deg=7.0), 0.5),
    ("reads 32, dropped off the table", gauge(32.0, pos=(0.20, -0.25, TOP - 0.30)), 0.0, False),
    *held_still("reads 32, back in the cradle, settling (moved 0.6 mm)", in_cradle(32.0, dy=0.0006), 1.0),
    ("reads 32, then creeping 0.6 mm in one step: the count restarts", in_cradle(32.0), 0.5, False),
    *held_still("reads 32, rolled 21 degrees about its own axis in the cradle", in_cradle(32.0, roll_deg=21.0), 1.0),
    *[(f"reads 32, rocking a degree about its own axis (step {k + 1})", in_cradle(32.0, roll_deg=21.0 + (k % 2) * 1.2),
       1.0, True) for k in range(4)],
    ("reads 32, its axis then turned 0.6 degrees: the count restarts", in_cradle(32.0, roll_deg=21.0, pitch_deg=0.6),
     0.5, False),
]


# --- the air rule (task.air_step), checked on import -----------------------------------------------------------------
def _air_run(gauge_state, core, steps=30, start=0.0):
    task._stick_air["target"] = start
    w = WHEEL.state(joints={"core": core})
    t = start
    for _ in range(steps):
        t = task.air_step(w, gauge_state)
    return t


def _g(pos, quat):
    return GAUGE.state(pos=tuple(pos), quat_xyzw=tuple(quat))


_seat = on_valve(10.0)["gauge"]
_q = tuple(_seat.quat_xyzw)
assert task.sealed(_seat)
assert abs(_air_run(_seat, 0.0012) - ext(32.0)) < 1e-9, "seated and core open: reads the tyre's 32 psi"
assert _air_run(_seat, 0.0012, steps=3) < ext(30.0), "a press too short reads low"
assert _air_run(_seat, 0.0005) == 0.0, "seated but the core not pushed far enough: no air, no reading"
assert _air_run(_g(np.asarray(_seat.pos) + D * 0.0007, _q), 0.0012) == ext(12.0), "gasket 0.7 mm off the tip: leaks"
_side = np.cross(D, [0.0, 1.0, 0.0])
assert _air_run(_g(np.asarray(_seat.pos) + _side * 0.0013, _q), 0.0012) == ext(12.0), "1.3 mm off the axis: leaks"
def _tilted(deg):
    q = parts._matrix_to_quat(parts._rot((0, 1, 0), math.radians(deg)) @ parts._quat_to_matrix(_q))
    return _g(TIP_W - (parts._quat_to_matrix(q) @ [1.0, 0.0, 0.0]) * GASKET_X, q)


assert not task.sealed(_tilted(4.1)) and task.sealed(_tilted(3.9)), "seated only within 4 degrees of the stem"
assert _air_run(_seat, 0.0012, start=ext(33.0)) == ext(33.0), "the stick never goes back in"
assert _air_run(_g(np.asarray(_seat.pos) + D * 0.0007, _q), 0.0012, start=ext(20.0)) == ext(20.0)
task._stick_air["target"] = 0.0
