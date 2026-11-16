"""Checks for wrench-flip that run without the simulator: python -m pytest -q test_wrench_flip.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from fittingkit import Frame
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

import wrench_flip as M

HERE = Path(__file__).resolve().parent
DT = 1.0 / 240.0


ON0 = 11.0  # the handle angle the first stroke goes on at

def frame(seat, face, deg, vel=(0.0, 0.0, 0.0), spin=(0.0, 0.0, 0.0)) -> Frame:
    rot = M.wrench_rotation(face, deg)
    pos = np.asarray(seat, float)
    return Frame(pos=pos, rot=rot, vel=np.asarray(vel, float), spin=np.asarray(spin, float), com=pos)


def run(model, seat, face, degs) -> dict:
    """Feed the model the wrench at ``seat`` with its handle at each angle in turn; the forces of the last step."""
    acc = {}
    for d in degs:
        acc = {}
        model.update({"wrench": frame(seat, face, d)}, acc)
    return acc


def swing(a, b, step=0.25):
    n = max(1, int(abs(b - a) / step))
    return list(np.linspace(a, b, n + 1))


@pytest.fixture
def model():
    m = M.WrenchOnNut("wrench", "nut", M.NUT_CENTRE)
    m.clear()
    return m


def test_the_task_file_loads():
    task = load(HERE / "wrench_flip.py")
    assert [o.name for o in task.objects] == ["bracket", "nut", "rack", "wrench"]


# -- the model --------------------------------------------------------------------------------------------------------

def test_the_jaw_goes_on_where_it_fits_and_turns_the_nut(model):
    on = M.engage_angle(M.NU0_DEG, +1)
    assert on == pytest.approx(ON0)
    run(model, M.NUT_CENTRE, +1, [on])
    assert model.engaged
    run(model, M.NUT_CENTRE, +1, swing(on, on - 30.0))
    assert model.tightened_deg() == pytest.approx(30.0, abs=0.01)


def test_the_jaw_does_not_go_on_out_of_line_with_the_flats(model):
    run(model, M.NUT_CENTRE, +1, [ON0 + 8.0])
    assert not model.engaged
    run(model, M.NUT_CENTRE, -1, [ON0])  # the other face out: 30 degrees off
    assert not model.engaged


def _jaw_axes(deg=ON0, face=+1):
    out = M.in_plane_unit(math.radians(deg + face * M.HEAD_ANGLE))
    return out, M.in_plane_unit(math.radians(deg + face * M.HEAD_ANGLE + 90.0))


@pytest.mark.parametrize("o, side, along, on", [(7.0e-3, 0.0, 0.0, True), (9.0e-3, 0.0, 0.0, False),
                                                (0.0, 2.5e-3, 0.0, True), (0.0, 3.5e-3, 0.0, False),
                                                (-2.5e-3, 0.0, 0.0, False), (0.0, 0.0, 5.0e-3, False)])
def test_the_nut_must_be_between_the_prongs(model, o, side, along, on):
    out, across = _jaw_axes()
    run(model, M.NUT_CENTRE + o * out + side * across + along * M.B, +1, [ON0])
    assert model.engaged == on


def test_the_wrench_must_lie_in_the_nut_s_plane(model):
    rot = M.wrench_rotation(+1, ON0)
    tilt = np.array(__import__("yam_arena.urdf", fromlist=["x"]).axis_angle_matrix(M.H, math.radians(20.0)))
    f = Frame(pos=M.NUT_CENTRE, rot=tilt @ rot, vel=np.zeros(3), spin=np.zeros(3), com=M.NUT_CENTRE)
    model.update({"wrench": f}, {})
    assert not model.engaged


def test_sliding_it_off_out_of_its_mouth_takes_it_off(model):
    run(model, M.NUT_CENTRE, +1, [ON0])
    out = M.in_plane_unit(math.radians(ON0 + M.HEAD_ANGLE))
    run(model, M.NUT_CENTRE + 13.0e-3 * out, +1, [ON0])
    assert not model.engaged


def test_the_jaw_is_held_on_the_nut_but_free_to_slide_off(model):
    run(model, M.NUT_CENTRE, +1, [ON0])
    side = M.in_plane_unit(math.radians(ON0 + M.HEAD_ANGLE + 90.0))
    acc = run(model, M.NUT_CENTRE + 2.0e-3 * side, +1, [ON0])
    assert float(acc["wrench"][0] @ side) < 0  # pulled back on
    out = M.in_plane_unit(math.radians(ON0 + M.HEAD_ANGLE))
    acc = run(model, M.NUT_CENTRE + 6.0e-3 * out, +1, [ON0])
    assert float(acc["wrench"][0] @ out) == pytest.approx(-model.pull)  # drawn on lightly: a hand can slide it off


def test_turning_the_wrench_over_lets_it_go_on_30_degrees_further_round(model):
    run(model, M.NUT_CENTRE, +1, [ON0])
    run(model, M.NUT_CENTRE, +1, swing(ON0, ON0 - 30.0))
    run(model, M.NUT_CENTRE + 0.03 * M.Z, +1, [-19.5])  # off
    assert not model.engaged
    run(model, M.NUT_CENTRE, +1, [ON0])  # the same face out does not fit at ON0
    assert not model.engaged
    run(model, M.NUT_CENTRE + 0.03 * M.Z, -1, [ON0])
    run(model, M.NUT_CENTRE, -1, [ON0])  # turned over, it does
    assert model.engaged and model.faces_used == {1, -1}


def test_without_turning_the_wrench_over_it_never_reaches_a_quarter(model):
    """Any sequence of strokes with one face out: on wherever the jaw fits (to the 7 degree tolerance), to the wall."""
    for face in (+1, -1):
        model.clear()
        for _ in range(20):
            nu = math.degrees(model.nu)
            fits = [a for a in np.arange(-M.SWING_DEG, M.SWING_DEG + 0.01, 0.1)
                    if abs(M._wrap(a + 30.0 + 15.0 * face - nu, 60.0)) <= 7.0]
            if not fits:
                break
            on = max(fits)
            run(model, M.NUT_CENTRE + 0.03 * M.Z, face, [on])
            run(model, M.NUT_CENTRE, face, [on])
            run(model, M.NUT_CENTRE, face, swing(on, -M.SWING_DEG))
            run(model, M.NUT_CENTRE + 0.03 * M.Z, face, [-M.SWING_DEG])
        assert model.tightened_deg() < 60.0 - 5.0, face


def test_the_nut_stops_when_seated(model):
    model.nu = model.nu0 - math.radians(170.0)
    run(model, M.NUT_CENTRE, +1, [M.engage_angle(math.degrees(model.nu), +1)])
    on = M.engage_angle(math.degrees(model.nu), +1)
    acc = run(model, M.NUT_CENTRE, +1, swing(on, on - 30.0))
    assert model.tightened_deg() == pytest.approx(M.SEAT_DEG)
    assert float(acc["wrench"][1] @ M.B) > 0  # the stop pushes the handle back


def test_the_nut_is_posed_by_the_model(model):
    model.nu = model.nu0 - math.pi
    model._pose_nut()
    pos, quat = model.poses["nut"]
    assert np.allclose(pos, M.NUT_CENTRE)


# -- scoring ----------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("deg, expected", [(0, 0.0), (45, 0.0), (60, 0.25), (99, 0.25), (100, 0.5), (149, 0.5),
                                           (150, 0.75), (177, 0.75), (178, 0.9), (180, 0.9)])
def test_the_tiers(deg, expected):
    M.MODEL.clear()
    M.MODEL.nu = M.MODEL.nu0 - math.radians(deg)
    assert M.score({}) == expected
    assert M.success({}) == (deg >= 178)
    M.MODEL.clear()


def test_backing_it_off_scores_nothing():
    M.MODEL.clear()
    M.MODEL.nu = M.MODEL.nu0 + math.radians(20.0)
    assert M.score({}) == 0.0
    M.MODEL.clear()


# -- the geometry ---------------------------------------------------------------------------------------------------

def test_the_slot_leaves_about_20_degrees_each_way():
    r, half_w = M.WALL_TOP - M.NUT_H, 7.0 * M.MM
    edge = lambda a: r * math.sin(math.radians(a)) + half_w * math.cos(math.radians(a))  # noqa: E731
    assert edge(M.SWING_DEG) == pytest.approx(M.SLOT_HALF, abs=0.1 * M.MM)
    assert edge(abs(M.STROKE_AIM[+1])) < M.SLOT_HALF and edge(ON0 + M.ON_LAG) < M.SLOT_HALF - 10.0 * M.MM


def test_the_jaw_fits_the_nut():
    assert M.JAW_13 > M.NUT_AF
    corner = M.NUT_AF / math.sqrt(3)
    assert 7.8 * M.MM > corner  # the jaw's back clears the hex's corner


def test_the_head_fits_the_rack_pocket():
    lo, hi = M.WRENCH.bounds_local()
    head = [s for s in M.WRENCH.shapes if s.centre[0] < 25 * M.MM]
    ys = [abs(s.centre[1]) + s.half_extent()[1] for s in head]
    assert max(ys) < M.POCKET_H / 2
    assert M.HEAD_T < M.POCKET_B


def test_the_wrench_is_real_size():
    lo, hi = M.WRENCH.bounds_local()
    assert 0.14 < hi[0] - lo[0] < 0.17  # a 10 x 13 mm double open-end wrench is about 15 to 16 cm


# -- the demo -------------------------------------------------------------------------------------------------------

GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = np.array([plan.action7(t)[0] for t in np.arange(0.0, plan.duration, 0.05)])
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(acts, axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_the_plan_turns_the_wrench_over_between_strokes():
    plan = M.stroke_plan()
    assert len(plan) == 6
    assert [f for f, _, _ in plan] == [1, -1, 1, -1, 1, -1]
    assert sum(on - end for _, on, end in plan) == pytest.approx(M.SEAT_DEG)
    assert all(-M.SWING_DEG < end < on + M.ON_LAG < M.SWING_DEG - 5.0 for _, on, end in plan)
    assert STROKE_PAST_OK(M.STROKE_AIM[+1])


def test_the_jaw_slides_on_inside_the_wall():
    """Where the handle crosses the wall's top, slid out ON_FROM across the nut, at the highest stroke start."""
    top_on = max(on for _, on, _ in M.stroke_plan()) + M.ON_LAG
    a, out = math.radians(top_on), math.radians(top_on + M.HEAD_ANGLE)
    up, across = M.ON_FROM * math.cos(out), M.ON_FROM * math.sin(out)
    r = M.WALL_TOP - M.NUT_H - up
    assert r * math.sin(a) + 7.0 * M.MM * math.cos(a) + across < M.SLOT_HALF - 3.5 * M.MM


def STROKE_PAST_OK(aim):  # noqa: N802
    return -M.SWING_DEG < aim <= -M.SWING_DEG + 2.0  # aimed just inside the wall


def test_the_demo_tightens_the_nut_to_seated_held_rigidly():
    """Replay the demo's hand path through the model, the wrench held rigidly where the hand gripped it."""
    model = M.WrenchOnNut("wrench", "nut", M.NUT_CENTRE)
    model.clear()
    plan = ArmPlan(M.demo()["left"], station.BASES["left"])
    labels = [w.label for w in plan.waypoints]
    t_grip = sum(w.duration for w in plan.waypoints[:labels.index("grip the handle") + 1])
    r_t0 = M.grip_rotation(M.YAW[+1])
    tcp0 = np.array(M.tcp_for(M.SEAT0, 0.0))
    seat_in_tcp = r_t0.T @ (M.SEAT0 - tcp0)
    w_in_tcp = r_t0.T @ M.wrench_rotation(+1)
    seated_at = None
    from yam_arena.urdf import axis_angle_matrix

    strokes = M.stroke_plan()
    for t in np.arange(t_grip, plan.duration, DT):
        pos, rot, _, label = plan.target(t)
        seat, wrot = pos + rot @ seat_in_tcp, rot @ w_in_tcp
        # the arm as the sim measured it: the handle ON_LAG below its aim, and the strokes stopping where they did
        wrot = axis_angle_matrix(M.B, math.radians(-M.ON_LAG)) @ wrot
        handle = math.degrees(math.atan2(wrot[:, 0] @ M.H, wrot[:, 0] @ M.Z))
        k = label.rsplit(" ", 1)[-1]
        stop = M.STROKE_REACHED[strokes[int(k) - 1][0]] if k.isdigit() else -M.SWING_DEG
        if np.linalg.norm(seat - M.NUT_CENTRE) < 0.03 and handle < stop:
            wrot = axis_angle_matrix(M.B, math.radians(stop - handle)) @ wrot
        f = Frame(pos=seat, rot=wrot, vel=np.zeros(3), spin=np.zeros(3), com=seat)
        model.update({"wrench": f}, {})
        if model.seated() and seated_at is None:
            seated_at = t
    assert seated_at is not None, model.tightened_deg()
    assert model.engagements >= 6 and model.faces_used == {1, -1}
    assert seated_at * 20 < 2400 * 0.6  # well within the 2,400-step run limit (the sim seated it at 62.6 s, run wf24)


def test_the_demo_starts_by_gripping_the_wrench_in_its_rack():
    left = M.demo()["left"]
    grip = next(w for w in left if w.label == "grip the handle")
    assert grip.pos == pytest.approx(M.tcp_for(M.SEAT0, 0.0))
    over = next(w for w in left if w.label == "over the wrench")
    lo, hi = M.WRENCH.bounds_local()
    assert over.pos[2] - 13.0 * M.MM > M.SEAT0[2] + hi[0] + 10.0 * M.MM  # the fingertips clear the wrench's top
