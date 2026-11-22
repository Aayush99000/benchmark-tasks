"""Checks of the task file that run without the simulator:

    PYTHONPATH=<yam-isaac-lab-arena checkout>:. python -m pytest -q test_close_door_quietly.py

- it loads as a yam_arena task file;
- the latch in plan view (the bolt's outline against the frame at bolt height): a free bolt is pushed in by the
  strike's lip as the door closes, a bolt held flush passes it, and at the stop the bolt fits fully out;
- the scripted demo's IK, and its plan replayed through the scorer with the door, knob and bolt following the hand.
It checks geometry and the plan, not PhysX.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

import close_door_quietly as T
import door_scoring as S
from mechanism import MechanismState

HERE = Path(__file__).resolve().parent
DEG = math.pi / 180


def test_loads_as_a_task_file():
    task = load(HERE / "close_door_quietly.py")
    assert task.scene == "station" and [o.name for o in task.objects] == ["door_unit"]
    assert task.success is not None and task.score is not None and task.demo is not None
    T.DOOR_UNIT._check()


def test_starts_ajar_and_the_knob_and_bolt_ride_on_the_door():
    joints = {j.name: j for j in T.JOINTS}
    assert joints["door"].start == pytest.approx(6 * DEG)
    assert joints["knob"].parent == "door" and joints["bolt"].parent == "door"
    assert T.DOOR_UNIT.base_contacts == ("bolt",)
    assert S.edge_gap(T.DOOR_START) == pytest.approx(0.064, abs=0.001)


def test_frame_stays_off_the_table():
    lows, highs = zip(*(s.bounds() for s in T.BASE.shapes))
    assert max(h[1] for h in highs) <= -0.65  # the table's right edge
    # the door opens towards the robot: at its start the slab's robot-side latch corner stays 15 mm or more off the
    # table's side edge (y -0.65)
    corner = np.array([S.HINGE_XY[0], S.HINGE_XY[1]]) + S.WIDTH * np.array([math.cos(T.DOOR_START), math.sin(T.DOOR_START)]) \
        + S.THICK / 2 * np.array([-math.sin(T.DOOR_START), math.cos(T.DOOR_START)])
    assert corner[1] <= -0.665


DOOR_ANGLES = [d * DEG / 10 for d in range(60, -1, -1)]  # 6 deg to shut in 0.1 deg steps


def test_a_free_bolt_is_pushed_in_by_the_lip_then_latches():
    pushes = [T.push_needed(d) for d in DOOR_ANGLES]
    assert pushes[0] == 0.0  # ajar: the bolt stands clear of the frame
    assert all(math.isfinite(p) for p in pushes)  # never jams: the lip always lets it pass
    assert max(pushes) >= S.SCRAPE + 0.005  # pushed well past the scrape threshold
    assert pushes[-1] == 0.0  # shut: fully out in the hole, latched
    assert T.bolt_fits(0.0, 0.0)


def test_a_bolt_held_flush_passes_the_lip():
    assert all(T.bolt_fits(d, S.knob_holds_in(T.TURN)) for d in DOOR_ANGLES)


def test_a_knob_turned_short_of_45_still_clears():
    # wrist compliance may leave the knob a few degrees short; the lip asks for at most 11.5 mm of the 13, which the
    # cam gives by 40 deg
    worst = max(T.push_needed(d) for d in DOOR_ANGLES)
    assert worst <= 0.0115
    least = min(d for d in range(46) if S.knob_holds_in(d * DEG) >= worst)
    assert least <= 40


def test_the_bolt_cannot_be_pulled_shut_without_latching_at_the_stop():
    # at the stop the bolt sits in the hole: pushed in fully it still fits (held), and fully out it fits (latched)
    for b in (0.0, S.THROW):
        assert T.bolt_fits(0.0, b)


def _plan():
    plans = T.demo()
    return {side: ArmPlan(plans[side], station.BASES[side]) for side in ("left", "right")}  # the station's bases


def test_demo_ik():
    arms = _plan()
    t_end = max(p.duration for p in arms.values())
    for k in range(int(t_end / S.DT) + 1):
        for plan in arms.values():
            plan.action7(min(k * S.DT, plan.duration - 1e-6))
    for side, plan in arms.items():
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 1.0, side


def _door_from_hand(pos) -> float:
    """The door angle whose knob centre is nearest the TCP (a firm grip carries the knob with the hand)."""
    grid = np.linspace(0.0, 10 * DEG, 1001)
    return float(grid[np.argmin([np.linalg.norm(S.knob_centre(d) - pos) for d in grid])])


def _turn(rot, door) -> float:
    """How far the TCP is rolled about its pointing axis past the grip that leaves the knob at rest."""
    ref = T.grip(door, 0.0)
    return math.atan2(float(np.dot(np.cross(ref[:, 0], rot[:, 0]), rot[:, 2])), float(np.dot(ref[:, 0], rot[:, 0])))


def _state(door, knob, bolt) -> MechanismState:
    z = np.zeros(3)
    return MechanismState(pos=z, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=z, high=z,
                          joints={"door": door, "knob": knob, "bolt": -bolt})


def replay(arms, turn_scale: float = 1.0):
    """Play the demo's plan through a kinematic model of the door and the scorer; return the final grade."""
    rec = S.DoorRecord()
    door, knob, held = T.DOOR_START, 0.0, False
    t_end = max(p.duration for p in arms.values())
    grade = None
    for k in range(int(t_end / S.DT) + 1):
        t = k * S.DT
        pos, rot, finger, label = arms["right"].target(min(t, arms["right"].duration - 1e-6))
        # closed to within 30% of the way from closed to open, and not yet opening
        held = abs(finger - FINGER_CLOSED) <= 0.3 * abs(FINGER_OPEN - FINGER_CLOSED) and "let go" not in label
        if held:
            door = _door_from_hand(pos)
            knob = turn_scale * _turn(rot, door)
        else:
            knob = 0.0
        bolt = max(S.knob_holds_in(knob), T.push_needed(door))
        pads = [np.array([pos]), np.array([np.asarray(T.PARK_LEFT)])]
        rec.step(door, knob, bolt, pads)
        objects = {"door_unit": _state(door, knob, bolt), "door_record": rec,
                   "right_gripper": T.Pads(back=pos - 0.025 * rot[:, 2], tips=pos + 0.013 * rot[:, 2]),
                   "left_gripper": T.Pads(back=np.asarray(T.PARK_LEFT), tips=np.asarray(T.PARK_LEFT))}
        grade = S.grade(*T._args(objects))
    return grade, rec


def test_demo_plan_scores_full():
    grade, rec = replay(_plan())
    assert not rec.scraped and not rec.hand_at_latch
    assert rec.snap_speed < S.SNAP and rec.close_speed < S.GENTLE
    assert grade == 1.0


def test_demo_plan_with_the_knob_not_turned_clicks():
    grade, rec = replay(_plan(), turn_scale=0.0)
    assert rec.scraped and grade == 0.5


def test_record_counts_each_control_step_once():
    T._clear_record()
    st = {"door_unit": _state(T.DOOR_START, 0.0, 0.0)}
    T.update_record(st, 0.05)
    T.update_record(st, 0.05)  # read again in the same step
    T.update_record({"door_unit": _state(T.DOOR_START - 0.01, 0.0, 0.0)}, 0.10)
    assert T.RECORD.steps == 2
    assert T.RECORD.edge_speed == pytest.approx(0.01 * S.WIDTH / 0.05)
    T._clear_record()
    assert T.RECORD.steps == 0
