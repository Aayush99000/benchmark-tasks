"""Checks without the simulator: the starting layout, the score of a made-up end state for every outcome, and the
scripted demo's shape. Run: python test_roll_silverware_in_a_napkin.py (or with pytest)."""

from __future__ import annotations

import math
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import roll_silverware_in_a_napkin as task  # noqa: E402
import labkit  # noqa: E402
from labkit import start_state  # noqa: E402

EDGE, TOP = task.BENCH_X[0], task.BENCH_TOP_Z
ROWS, COLS = task.NAPKIN.cells[0] + 1, task.NAPKIN.cells[1] + 1    # mesh nodes across the napkin's depth, and along its width
KNIFE_X = EDGE + task.KNIFE_IN                                       # the knife lies along the near edge, this far in


def _piece(x, y, z, yaw=-math.pi / 2, speed=0.0):
    return types.SimpleNamespace(pos=np.array([x, y, z]), quat_xyzw=np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]),
                                 lin_vel=np.array([speed, 0.0, 0.0]), ang_vel=np.zeros(3))


def _sheet(profile):
    """A napkin whose every line across its depth follows ``profile``: (x, z) points, any spacing, 23.5 cm long."""
    p = np.asarray(profile, float); seg = np.linalg.norm(np.diff(p, axis=0), axis=1); s = np.concatenate([[0.0], np.cumsum(seg)])
    at = np.linspace(0.0, task.NAPKIN_DEPTH, ROWS)
    assert s[-1] >= task.NAPKIN_DEPTH - 1e-6, f"profile is only {s[-1]:.3f} m long"
    line = np.stack([np.interp(at, s, p[:, 0]), np.interp(at, s, p[:, 1])], 1)
    ys = np.linspace(-task.NAPKIN_WIDTH / 2, task.NAPKIN_WIDTH / 2, COLS)
    nodes = np.array([[x, y, z] for x, z in line for y in ys])        # node (i, j) = i * COLS + j, as the simulator orders them
    return types.SimpleNamespace(nodes=nodes)


def _wound(times, centre_x=KNIFE_X, base=TOP):
    """The napkin wound ``times`` round a centre line 2 cm above ``base``, its free end inside, the rest lying flat
    on ``base`` away from the robot."""
    r_out, per_turn, c = 0.020, 0.003, (centre_x, base + 0.020 + 0.001)
    ang = np.linspace(-math.pi / 2 - 2 * math.pi * times, -math.pi / 2, 400)
    r = r_out - per_turn * (-math.pi / 2 - ang) / (2 * math.pi)
    spiral = np.stack([c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)], 1)
    return np.vstack([spiral, [[centre_x + 0.40, base + 0.001]]]), c


def _trial(times, fork_dx=0.0, fork_yaw=-math.pi / 2, base=TOP, speed=0.0):
    profile, c = _wound(times, base=base)
    return {"napkin": _sheet(profile), "knife": _piece(c[0], 0.0, c[1], speed=speed), "fork": _piece(c[0] + fork_dx, task.FORK_Y, c[1] + 0.004, yaw=fork_yaw)}


AWAY = np.array([[0.0, 0.305, TOP + 0.30], [0.0, -0.305, TOP + 0.30]])    # finger pads up at the ready poses, far from the roll


def _score_after(objs, seconds, pads=AWAY):
    """The score once the finger pads have been at ``pads`` for ``seconds`` (None: finger pads not known)."""
    task._new_trial(); labkit._FINGERS["pts"] = pads
    labkit.tick(0.0); task.score(objs)
    labkit.tick(seconds)
    try:
        return task.score(objs), task.success(objs)
    finally:
        labkit._FINGERS["pts"] = None; labkit.tick(0.0); task._new_trial()


def test_layout_matches_the_setup_text():
    nap, knife, fork = (start_state(o) for o in (task.NAPKIN, task.KNIFE, task.FORK))
    assert abs(nap.low[0] - (EDGE + 0.005)) < 0.002 and abs(nap.high[0] - nap.low[0] - 0.235) < 0.002 and abs(nap.high[1] - 0.14) < 0.002
    assert nap.high[0] < task.BENCH_X[1] and 0.0 < nap.low[2] - TOP < 0.004, "the napkin lies flat, wholly on the side table"
    for s in (knife, fork):
        assert abs(s.low[2] - TOP) < 0.002 and EDGE < s.low[0] and s.high[0] < task.BENCH_X[1] and abs(s.low[1]) < task.BENCH_Y / 2 and abs(s.high[1]) < task.BENCH_Y / 2
    assert knife.low[1] > nap.high[1] + 0.01 and fork.high[1] < nap.low[1] - 0.01, "knife left of the napkin, fork right of it"
    assert abs(task.BENCH_TOP_Z - task.TOP - 0.20) < 1e-9 and abs(task.BENCH_TOP.pos[2] + task.SLAB / 2 - TOP) < 1e-9
    assert 0.060 < task.NAPKIN.density * 0.235 * 0.28 * 0.0025 < 0.075, "the napkin weighs about 68 g"


def test_doing_nothing_scores_zero():
    start = {o.name: start_state(o) for o in task.OBJECTS}
    assert task.score(start) == 0.0 and not task.success(start) and not task.success({})


def test_knife_and_fork_laid_on_the_open_napkin_score_zero():
    flat = [(EDGE - 0.001, TOP - 0.061), (EDGE - 0.001, TOP + 0.001), (EDGE + 0.30, TOP + 0.001)]     # a quarter hangs over the edge
    objs = {"napkin": _sheet(flat), "knife": _piece(KNIFE_X, 0.0, TOP + 0.006), "fork": _piece(KNIFE_X, task.FORK_Y, TOP + 0.012)}
    m = task._measure(objs)
    assert m["knife_in"] == 0.0 and m["turns"] < 0.7 and task.score(objs) == 0.0, m


def test_napkin_dropped_over_them_on_the_bare_table_scores_zero():
    over = [(KNIFE_X - 0.10, TOP + 0.001), (KNIFE_X - 0.02, TOP + 0.001), (KNIFE_X - 0.012, TOP + 0.018), (KNIFE_X + 0.012, TOP + 0.018),
            (KNIFE_X + 0.02, TOP + 0.001), (KNIFE_X + 0.30, TOP + 0.001)]
    objs = {"napkin": _sheet(over), "knife": _piece(KNIFE_X, 0.0, TOP + 0.004), "fork": _piece(KNIFE_X, task.FORK_Y, TOP + 0.010)}
    assert task._measure(objs)["knife_in"] < task.ROUND_SHARE and task.score(objs) == 0.0


def test_each_stage_scores_its_step():
    for times, want in ((0.80, 0.3), (1.10, 0.6), (1.60, 0.8)):
        objs = _trial(times); m = task._measure(objs)
        assert task.score(objs) == want, (times, want, m)
    folded, rolled_on, rolled_up = (task._measure(_trial(t)) for t in (0.80, 1.10, 1.60))
    assert task.TURNS_FOLDED < folded["turns"] < task.TURNS_ROLLED < rolled_on["turns"] < task.TURNS_DONE < rolled_up["turns"]
    assert rolled_up["width"] < task.ROLL_WIDTH < rolled_on["width"]


def test_full_credit_needs_the_roll_let_go_and_lying_on_the_side_table():
    roll = _trial(1.60)
    assert _score_after(roll, 2.5) == (1.0, True), "rolled up, grippers clear for 2.5 s"
    assert _score_after(roll, 1.5)[0] == 0.8, "clear for only 1.5 s"
    assert _score_after(roll, 2.5, pads=None)[0] == 0.8, "finger pads not known"
    knife = roll["knife"].pos
    assert _score_after(roll, 2.5, pads=np.vstack([AWAY, [knife + [0.0, 0.0, 0.015]]]))[0] == 0.8, "a finger pad 1.5 cm from the knife: still held"
    tail = roll["napkin"].nodes[-1]
    assert _score_after(roll, 2.5, pads=np.vstack([AWAY, [tail + [0.0, 0.0, 0.010]]]))[0] == 0.8, "a finger pad pressing on the napkin's end"
    assert _score_after(_trial(1.60, base=TOP + 0.05), 2.5)[0] == 0.8, "rolled up but held 5 cm above the side table"
    assert _score_after(_trial(1.10), 2.5)[0] == 0.6, "letting go of a half-rolled bundle earns nothing more"


def test_fork_outside_the_roll_scores_zero():
    assert _score_after(_trial(1.60, fork_dx=0.06), 2.5)[0] == 0.0, "fork lying beside the roll"
    assert _score_after(_trial(1.60, fork_yaw=-math.pi / 2 + math.radians(35.0)), 2.5)[0] == 0.0, "fork lying across the knife"


def test_roll_that_fell_off_the_side_table_scores_zero():
    assert _score_after(_trial(1.60, base=task.TOP), 2.5)[0] == 0.0


def test_success_needs_the_roll_lying_still():
    assert _score_after(_trial(1.60, speed=0.05), 2.5) == (1.0, False), "let go but still moving"
    assert not task.success(_trial(1.60)) and task.SUCCESS_HOLD_S == 2.0 and task.CLEAR_S == 2.0 and task.CLEAR_GAP == 0.02


def test_demo_is_the_five_steps_in_order():
    plan = task.demo(); labels = [p.label for p in plan["left"]]
    order = [next(i for i, l in enumerate(labels) if l.startswith(s)) for s in
             ("press the fingertips on the napkin", "close on the knife", "forward: lower fingertip under the hanging edge",
              "turn the wrist: the bundle slides round", "roll: turn the wrist")]
    assert order == sorted(order), order
    lengths = {arm: sum(p.duration for p in plan[arm]) for arm in ("left", "right")}
    assert abs(lengths["left"] - lengths["right"]) < 1e-6 and 100.0 < lengths["left"] < 110.0, lengths
    assert sum(l.startswith("pinch the handles") for l in labels) == 2, "two half turns, each with a fresh hold"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print("ok  ", fn.__name__)
    print(f"{len(fns)} passed")
