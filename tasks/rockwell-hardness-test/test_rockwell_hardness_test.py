"""Checks of the task file that run without the simulator:

    PYTHONPATH=<yam-isaac-lab-arena checkout>:. python -m pytest -q test_rockwell_hardness_test.py

- it loads as a yam_arena task file and its mechanism is well formed;
- the scripted demo's IK stays within 2 mm and 2 degrees on both arms;
- the demo's plan replayed through the scorer, the handwheel turning with the left hand's knob and the handle with
  the right hand's: it ends at full score, the preliminary load within the manual's 5 divisions; and the probes
  (an overshoot, a short hold) score what the README says.
It checks geometry and the plan, not PhysX.
"""

from __future__ import annotations

import importlib
import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

import hardness_scoring as S
import rockwell_hardness_test as T

HERE = Path(__file__).resolve().parent


def test_loads_as_a_task_file():
    task = load(HERE / "rockwell_hardness_test.py")
    assert task.scene == "station" and [o.name for o in task.objects] == ["tester", "block"]
    assert task.success is not None and task.score is not None and task.demo is not None
    T.TESTER._check()


def test_geometry():
    assert T.TIP_Z - T.BLOCK_TOP == pytest.approx(S.START_GAP)
    # The block starts flat on the anvil, centred over it, the indenter meeting it at a fresh spot.
    assert np.hypot(T.BLOCK_XY[0] - T.AX, T.BLOCK_XY[1]) < T.ANVIL_R
    assert np.linalg.norm(np.array([T.AX, 0.0]) - T.BLOCK_XY) <= S.BLOCK_RADIUS - S.SPOT_GAP
    # The knobs stand clear of the base casting and below the head; the anvil's screw stays above the wheel's hub.
    assert T.KNOB_LO > T.BASE_TOP and T.KNOB_HI < T.HEAD_LO
    lo, hi = next(j for j in T.JOINTS if j.name == "wheel").lower, next(j for j in T.JOINTS if j.name == "wheel").upper
    assert -hi <= 0.0 <= -lo and -lo == pytest.approx(S.wheel_for(S.TRAVEL_STOP))
    assert max(abs(lo), abs(hi)) < 2 * math.pi  # PhysX revolute limits


def plans():
    d = T.demo()
    return ArmPlan(d["left"], station.BASES["left"]), ArmPlan(d["right"], station.BASES["right"])


def test_demo_ik_within_two_mm():
    left, right = plans()
    for plan in (left, right):
        t = 0.0
        while t < plan.duration:
            plan.action7(t)
            t += S.DT
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (plan.worst_ik_mm, plan.worst_ik_deg)


def replay(module=T):
    """The demo's plan, the arms at their IK poses; the handwheel follows a closed left hand on a knob, the handle a
    closed right hand on its knob. Returns the record and the final (wheel, handle, tcps)."""
    d = module.demo()
    left, right = ArmPlan(d["left"], station.BASES["left"]), ArmPlan(d["right"], station.BASES["right"])
    rec = S.HardnessRecord()
    wheel, handle = 0.0, S.HANDLE_FORWARD
    held = {"left": None, "right": None}  # (knob index, wheel angle minus the hand's angle) while a hand holds one
    spot = np.array([module.AX, 0.0]) - np.array(module.BLOCK_XY)
    t, end = 0.0, max(left.duration, right.duration)
    worst_div = None
    tcps = []
    while t < end:
        tcps = []
        for side, plan in (("left", left), ("right", right)):
            q7, _ = plan.action7(min(t, plan.duration))
            pos, _ = ik.tcp_pose(q7[:6], station.BASES[side])
            closed = q7[6] < 0.5
            tcps.append(pos)
            v = pos[:2] - np.array([module.AX, 0.0])
            ang_now = math.atan2(v[1], -v[0])
            if not closed:
                held[side] = None
            elif held[side] is None:
                for i, ang in enumerate(module.knob_angles(wheel)):
                    k = np.array([*module.knob_xy(ang), module.GRIP_Z])
                    if np.linalg.norm(pos - k) < 0.005:
                        held[side] = (i, wheel - ang_now)
            if held[side] is not None:
                wheel = ang_now + held[side][1]  # the knob turns with the hand
            if side == "right":
                k = module.handle_knob(handle)
                if closed and np.linalg.norm(pos - k) < 0.006:
                    handle = float(np.clip(math.atan2(pos[0] - module.PIVOT[0], pos[2] - module.PIVOT[2]),
                                           S.HANDLE_FORWARD, S.HANDLE_BACK))
        rec.step(wheel, handle, True, spot)
        if handle >= S.HANDLE_RELEASED and worst_div is None:
            worst_div = S.divisions_from_c(S.travel(wheel))
        t += S.DT
    return rec, wheel, handle, tcps, worst_div


@pytest.fixture
def untrimmed(monkeypatch):
    """The demo without its official-sim settle trim: here the hand reaches its IK pose exactly."""
    monkeypatch.setenv("ROCKWELL_DEMO_TRIM_DEG", "0")
    yield importlib.reload(T)
    monkeypatch.delenv("ROCKWELL_DEMO_TRIM_DEG")
    importlib.reload(T)


def test_the_demo_replays_to_full_score(untrimmed):
    rec, wheel, handle, tcps, div_at_push = replay(untrimmed)
    assert div_at_push is not None and abs(div_at_push) <= 1.0, div_at_push  # the plan sets it within a division
    assert len(rec.attempts) == 1 and rec.attempts[0].stage() == 0.75
    a = rec.attempts[0]
    assert S.DWELL[0] <= a.left_t - a.full_t <= S.DWELL[1]
    assert S.travel(wheel) <= S.LOWERED
    assert S.grade(rec, wheel, handle, True, untrimmed.hands_clear(tcps, _state(wheel, handle))) == 1.0


def test_the_trim_is_within_the_window_of_a_short_settle():
    # The default aims 1.6 degrees (about 9 divisions) past C: it lands inside the window only if the hand stops
    # about that much short, as it does in the official sim.
    assert T.SET_CW - S.wheel_for(S.PRE_TRAVEL) == pytest.approx(math.radians(T.SETTLE_TRIM_DEG))


def _state(wheel, handle):
    from mechanism import MechanismState

    z = np.zeros(3)
    return MechanismState(pos=z, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=z, high=z,
                          joints={"wheel": -wheel, "handle": handle})


@pytest.mark.parametrize("env, expect", [({"ROCKWELL_DEMO_OVERSHOOT_DIV": "8", "ROCKWELL_DEMO_TRIM_DEG": "0"}, 0.0),
                                          ({"ROCKWELL_DEMO_HOLD_S": "5.0", "ROCKWELL_DEMO_TRIM_DEG": "0"}, 0.5)])
def test_probes(monkeypatch, env, expect):
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    module = importlib.reload(T)
    try:
        rec, wheel, handle, tcps, _ = replay(module)
        assert S.grade(rec, wheel, handle, True, True) == expect
    finally:
        for k in env:
            monkeypatch.delenv(k)
        importlib.reload(T)


def test_fingers():
    assert FINGER_CLOSED > FINGER_OPEN  # the replay reads "closed" from the normalised grip below 0.5
