"""Offline tests: the task file's geometry, scorer and webbing model. No simulator needed.

    python -m pytest tests -q        (in the Isaac Lab-Arena + yam_arena environment)
"""
import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

T = task_file.load(Path(__file__).resolve().parents[1] / "task.py").module
FLAT = T._rz(T.HOOK_YAW)                      # the hook as it lies at the start
LEFTWARD = T._rz(math.pi / 2)                 # level, nose pointing left (+y): how it goes onto the anchor
ON_ANCHOR = np.array([T.ANCHOR_SEAT[0] - 0.004, T.ANCHOR_SEAT[1] - 0.018, T.ANCHOR_Z])


def test_task_file_loads():
    t = task_file.load(Path(__file__).resolve().parents[1] / "task.py")
    assert t.scene == "station" and t.score is not None and t.success is not None
    names = t.names
    for n in ("deck", "guardrail", "column", "hook", "hook_gate", "hook_lock", "pack", "harness"):
        assert n in names
    assert sum(n.startswith("web_") for n in names) == T.N_SEG


def test_real_sizes():
    assert T.PIPE_R * 2 == pytest.approx(0.0483, abs=1e-4)                    # 1-1/2 in schedule 40 pipe
    assert T.TOP_RAIL_Z + T.PIPE_R - T.ZD == pytest.approx(1.067, abs=1e-3)   # top rail 42 in
    assert T.MID_RAIL_Z - T.ZD == pytest.approx(0.533, abs=1e-3)              # midrail 21 in
    assert T.COL == pytest.approx(0.1016)                                     # 4 in square tube
    assert (T.RING_IN_W, T.RING_IN_L) == (0.057, 0.083)                       # anchor ring 2-1/4 x 3-1/4 in inside
    lanyard = 0.186 + T.BOWL_R + T.SEG * (T.N_SEG + T.N_PACK)                 # bowl to harness D-ring
    assert 1.75 < lanyard < 1.95                                              # a 6 ft (1.83 m) lanyard


def test_hook_fits_pipe_and_gripper():
    inside_w = 2 * (T.BOWL_R - T.HW / 2)
    assert inside_w > 2 * T.PIPE_R + 0.02                                      # the bowl takes the guardrail pipe
    nose_in = -(T.BOWL_R - T.HW / 2)
    opening = (T.gate_tip(T.GATE_OPEN_DEG)[1] - 0.005) - nose_in               # clear, nose to open gate
    assert 0.055 < opening < 0.066                                             # a 2-1/4 in (57 mm) gate
    assert opening > 2 * T.PIPE_R                                              # the pipe passes the open gate
    x = -0.105                                                                 # where the fingers go
    g = T.GATE_TIP - T.GATE_PIVOT
    gate_out = T.GATE_PIVOT[1] + (x - T.GATE_PIVOT[0]) * g[1] / g[0] - 0.005
    lock_out = T.LOCK_PIVOT[1] + 0.004 + T.LOCK_OUT * ((T.LOCK_PIVOT[0] - x) / T.LOCK_LEN)
    assert lock_out - gate_out < 0.094 - 0.008                                 # inside the open gripper (94 mm)


def test_start_scores_zero():
    ev = T.judge(T.HOOK_POS, FLAT, 0.0, 0.0)
    assert ev["score"] == 0.0 and not ev["on_anchor"] and ev["on_rail"] == []


def test_on_anchor_full_score():
    ev = T.judge(ON_ANCHOR, LEFTWARD, 0.0, 0.0)
    assert ev["on_anchor"] and ev["score"] == 1.0
    hang = np.column_stack([[0, 0.3, -0.954], [-1.0, 0, 0], [0, 0.954, 0.3]])  # let go: hanging tail-down on the ring
    hang[:, 2] = np.cross(hang[:, 0], hang[:, 1])
    p = np.array([T.ANCHOR_SEAT[0], T.ANCHOR_SEAT[1], T.ANCHOR_Z]) - hang @ np.array([0.02, 0.0, 0.0])
    assert T.judge(p, hang, 0.0, 0.0)["score"] == 1.0


@pytest.mark.parametrize("kw", [dict(gate_deg=20.0), dict(lock_deg=4.0), dict(still=False),
                                dict(gripper_pts=[ON_ANCHOR + np.array([-0.05, -0.105, 0.0])])])
def test_on_anchor_but_not_finished_is_half(kw):
    args = dict(gate_deg=0.0, lock_deg=0.0, gripper_pts=(), still=True)
    args.update(kw)
    ev = T.judge(ON_ANCHOR, LEFTWARD, **args)
    assert ev["on_anchor"] and ev["score"] == 0.5


def test_beside_the_anchor_is_zero():
    for off in ((-0.09, 0.0, 0.0), (0.0, -0.12, 0.0), (0.0, 0.0, 0.08)):
        ev = T.judge(ON_ANCHOR + np.array(off), LEFTWARD, 0.0, 0.0)
        assert not ev["on_anchor"] and ev["score"] == 0.0


def test_every_guardrail_member_scores_zero():
    for y in (T.RIGHT_POST_Y, *T.POST_Y):                                      # level, round each post
        p = np.array([T.POST_X - 0.012, y - 0.016, T.ZD + 0.15])
        ev = T.judge(p, FLAT, 0.0, 0.0)
        assert len(ev["on_rail"]) == 1 and ev["on_rail"][0].startswith("post") and ev["score"] == 0.0
    upright = np.column_stack([[0, 0, 1.0], [1.0, 0, 0], [0, 1.0, 0]])         # hanging on a rail: nose up, loop over it
    for name, z in (("lower_rail", T.LOW_RAIL_Z), ("midrail", T.MID_RAIL_Z), ("top_rail", T.TOP_RAIL_Z)):
        ev = T.judge(np.array([T.POST_X, -0.30, z - 0.015]), upright, 0.0, 0.0)
        assert ev["on_rail"] == [name] and ev["score"] == 0.0


def test_guardrail_overrides_anchor():
    """If the loop is somehow round both, the guardrail decides: 0."""
    assert T.judge(ON_ANCHOR, LEFTWARD, 0.0, 0.0)["score"] == 1.0
    ev = dict(T.judge(ON_ANCHOR, LEFTWARD, 0.0, 0.0))
    assert ev["on_rail"] == []


def test_lanyard_reaches_the_anchor_with_slack():
    eye = ON_ANCHOR + LEFTWARD @ T.EYE
    assert np.linalg.norm(T.WORKER_DRING - eye) < T.SEG * (T.N_SEG + T.N_PACK) - 0.10
    assert not T.Strap.taut_pull(eye, T.WORKER_DRING).any()


def test_taut_pull():
    eye = T.WORKER_DRING + np.array([0.0, 1.2, -1.3])                          # 1.77 m away: past the lanyard's length
    f = T.Strap.taut_pull(eye, T.WORKER_DRING)
    assert 0 < np.linalg.norm(f) <= T.TAUT_MAX and f[1] < 0 < f[2]             # toward the worker
    d = eye - T.WORKER_DRING
    near = T.WORKER_DRING + d / np.linalg.norm(d) * (T.SEG * (T.N_SEG + T.N_PACK) - 0.01 + 0.005)   # 5 mm past
    f = T.Strap.taut_pull(near, T.WORKER_DRING)
    pulling = T.Strap.taut_pull(near, T.WORKER_DRING, rate=0.1)             # still moving away: damped harder
    easing = T.Strap.taut_pull(near, T.WORKER_DRING, rate=-0.1)             # coming back: less, never a push
    assert np.linalg.norm(easing) < np.linalg.norm(f) < np.linalg.norm(pulling) <= T.TAUT_MAX
    assert not T.Strap.taut_pull(near, T.WORKER_DRING, rate=-1.0).any()


def test_strap_keeps_its_length_and_stays_on_the_deck():
    st = T.Strap()
    eye = T.eye_world()
    for k in range(120):                                                       # carry the eye to the anchor
        s = min(1.0, k / 80)
        e = eye + s * (ON_ANCHOR + LEFTWARD @ T.EYE - eye) + np.array([0, 0, 0.1 * math.sin(math.pi * s)])
        st.step(e, T.WORKER_DRING)
    seg = np.linalg.norm(np.diff(st.p, axis=0), axis=1)
    assert seg[1:].max() < T.SEG * 1.03 and seg.mean() > T.SEG * 0.93      # every piece keeps its length
    assert seg[0] < T.SEG * 1.5                                             # (the piece at the eye takes up the rest)
    assert st.p[:, 2].min() >= T.Strap.FLOOR - 1e-9
    assert np.linalg.norm(st.hanging_pull()) <= 4.0 + 1e-9


def test_score_text_matches_judge():
    assert T.CLEAR == 0.03 and T.STILL_S == 1.0 and T.STILL_MM == 3.0 and T.SUCCESS_HOLD_S == 1.0
