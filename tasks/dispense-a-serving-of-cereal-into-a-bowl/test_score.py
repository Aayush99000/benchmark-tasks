"""Unit tests for the scoring criteria: every band and the shortcuts. Plain Python and NumPy, no simulator.
Created 11_16_26 (America/New_York).

    python -m pytest test_score.py
"""

import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

import task
from cereal_dispenser import geometry as G

UP = np.array((0.0, 0.0, 0.0, 1.0))


def _bowl(pos=None, quat=UP, vel=(0.0, 0.0, 0.0)):
    p = task.BOWL_TARGET if pos is None else np.asarray(pos, float)
    return NS(pos=np.asarray(p, float), quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float))


def _in_bowl_points(n, bowl_pos=None):
    """n piece centres spread inside the bowl (bowl upright)."""
    c = task.BOWL_TARGET if bowl_pos is None else np.asarray(bowl_pos, float)
    out = []
    k = 0
    while len(out) < n:
        z = G.BOWL_FLOOR_Z + 0.003 + 0.004 * (k % 8)
        r = (G.bowl_inner_radius(z) - 0.006) * math.sqrt(((k * 37) % 101) / 101.0)
        a = 2.399963 * k
        out.append(c + (r * math.cos(a), r * math.sin(a), z))
        k += 1
    return np.array(out)


def _dispenser_points(n):
    """n piece centres inside the canister (dispenser frame mapped to the world)."""
    d = np.c_[np.linspace(-0.03, 0.03, n), np.zeros(n), np.full(n, G.FUNNEL_Z1 + 0.02)]
    return np.c_[task.D_XY[0] + d[:, 0] * task.X_D[0] + d[:, 1] * task.Y_D[0],
                 task.D_XY[1] + d[:, 0] * task.X_D[1] + d[:, 1] * task.Y_D[1], task.TOP + d[:, 2]]


def _counter_points(n):
    """n piece centres lying on the counter, well away from the bowl and the dispenser."""
    return np.c_[np.full(n, 0.20), np.linspace(-0.10, -0.05, n), np.full(n, task.TOP + 0.0025)]


def _state(grams=0.0, spilled=0, bowl=None, tcp_left=(0.0, 0.305, 1.0), tcp_right=(0.0, -0.305, 1.0)):
    n_in = int(round(grams / (G.PIECE_MASS * 1000)))
    bowl = bowl or _bowl()
    pts = [_in_bowl_points(n_in, bowl.pos)] if n_in else []
    if spilled:
        pts.append(_counter_points(spilled))
    rest = G.N_PIECES - n_in - spilled
    pts.append(_dispenser_points(rest))
    pos = np.concatenate(pts)
    return {"bowl": bowl, "cereal": NS(pos=pos, vel=np.zeros_like(pos)), "wheel": NS(angle=0.0, rate=0.0),
            "left_tcp": NS(pos=np.asarray(tcp_left, float)), "right_tcp": NS(pos=np.asarray(tcp_right, float))}


@pytest.fixture(autouse=True)
def fresh():
    task.reset_trial()
    yield
    task.reset_trial()


def settle(st, seconds=1.2):
    for _ in range(round(seconds / 0.05)):
        task.note_step(st, 0.05)


# -- geometry of the criteria ------------------------------------------------------------------------------------------
def test_counts_match_construction():
    st = _state(grams=39.0, spilled=4)
    c = task.counts(st)
    assert c["spilled"] == 4
    assert abs(c["grams"] - 39.0) < 0.11
    assert c["bowl"] + c["spilled"] + c["dispenser"] == G.N_PIECES


def test_piece_above_rim_or_outside_wall_is_not_in_bowl():
    b = _bowl()
    above = task.BOWL_TARGET + (0.0, 0.0, G.BOWL["h"] + 0.010)
    outside = task.BOWL_TARGET + (G.BOWL["r_rim"] + 0.005, 0.0, 0.03)
    floor = task.BOWL_TARGET + (0.0, 0.0, G.BOWL_FLOOR_Z + 0.0025)
    got = task.in_bowl({"bowl": b}, np.array((above, outside, floor)))
    assert got.tolist() == [False, False, True]


def test_in_bowl_follows_a_moved_and_tilted_bowl():
    q = (math.sin(math.radians(10) / 2), 0.0, 0.0, math.cos(math.radians(10) / 2))
    b = _bowl(pos=task.BOWL_START, quat=q)
    p = task.BOWL_START + (0.0, 0.0, G.BOWL_FLOOR_Z + 0.004)
    assert task.in_bowl({"bowl": b}, p[None])[0]
    assert not task.in_bowl({"bowl": _bowl()}, p[None])[0]


def test_spout_is_at_the_dispenser_origin_and_inside_the_outline():
    spout_world = np.array((*task.D_XY, task.TOP + G.SPOUT_Z + 0.005))
    d = task.to_dispenser(spout_world)[0]
    assert np.allclose(d[:2], 0.0, atol=1e-9)
    assert task.in_dispenser(spout_world[None])[0]
    assert not task.in_dispenser(np.array((*task.D_XY, task.TOP + G.SPOUT_Z - 0.01))[None])[0]


def test_bowl_target_is_under_the_spout_and_start_is_not():
    for p, under in ((task.BOWL_TARGET, True), (task.BOWL_START, False)):
        d = task.to_dispenser(p)[0]
        spout_corners = [(sx * G.X_END, sy * G.OUTLET_Y) for sx in (-1, 1) for sy in (-1, 1)]
        inside = all(math.hypot(x - d[0], y - d[1]) < G.bowl_inner_radius(G.BOWL["h"]) - 0.01 for x, y in spout_corners)
        assert inside == under


def test_piece_mass_gives_the_labelled_serving():
    assert abs(1.5 * G.CUP_COUNT * G.PIECE_MASS * 1000 - G.SERVING_G) < 0.1


# -- bands -------------------------------------------------------------------------------------------------------------
def test_do_nothing_scores_zero():
    st = _state()
    settle(st)
    assert task.score(st) == 0.0


def test_knob_turned_with_no_bowl_under_the_spout_scores_zero():
    st = _state(grams=0.0, spilled=300)
    settle(st)
    assert task.score(st) == 0.0


def test_one_piece_in_the_bowl_scores_quarter():
    st = _state(grams=G.PIECE_MASS * 1000)
    settle(st)
    assert task.score(st) == 0.25


def test_under_half_a_serving_scores_quarter():
    st = _state(grams=15.0)
    settle(st)
    assert task.score(st) == 0.25


def test_half_to_short_of_a_serving_scores_half():
    st = _state(grams=25.0)
    settle(st)
    assert task.score(st) == 0.5


def test_over_serving_scores_half():
    st = _state(grams=60.0)
    settle(st)
    assert task.score(st) == 0.5


@pytest.mark.parametrize("grams", (31.5, 39.0, 46.5))
def test_a_serving_finished_scores_full(grams):
    st = _state(grams=grams, spilled=task.SPILL_FULL)
    settle(st)
    assert task.score(st) == 1.0
    assert task.success(st)


def test_a_serving_with_some_spill_scores_three_quarters():
    st = _state(grams=39.0, spilled=task.SPILL_FULL + 1)
    settle(st)
    assert task.score(st) == 0.75


def test_heavy_spill_caps_at_quarter_even_with_a_serving():
    st = _state(grams=39.0, spilled=task.SPILL_CAP + 1)
    settle(st)
    assert task.score(st) == 0.25


def test_a_tipped_bowl_caps_at_quarter():
    q = (math.sin(math.radians(30) / 2), 0.0, 0.0, math.cos(math.radians(30) / 2))
    b = _bowl(quat=q)
    st = _state(grams=39.0, bowl=b)
    settle(st)
    assert task.score(st) == 0.25


def test_hand_still_on_the_bowl_scores_three_quarters():
    st = _state(grams=39.0, tcp_left=task.BOWL_TARGET + (0.0, 0.0, G.BOWL["h"] + 0.02))
    settle(st)
    assert task.score(st) == 0.75


def test_hand_beside_the_bowl_rim_is_not_clear_but_far_away_is():
    near = task.BOWL_TARGET + (G.BOWL["r_rim"] + 0.02, 0.0, 0.03)
    far = task.BOWL_TARGET + (G.BOWL["r_rim"] + 0.08, 0.0, 0.03)
    assert not task.hands_clear(_state(grams=39.0, tcp_right=near))
    assert task.hands_clear(_state(grams=39.0, tcp_right=far))


def test_not_yet_at_rest_scores_three_quarters():
    st = _state(grams=39.0)
    settle(st, 0.5)
    assert task.score(st) == 0.75


# -- at rest ------------------------------------------------------------------------------------------------------------
def test_a_moving_bowl_restarts_the_rest_clock():
    st = _state(grams=39.0)
    settle(st)
    st["bowl"].lin_vel = np.array((0.05, 0.0, 0.0))
    task.note_step(st, 0.05)
    assert task.trial()["still"] == 0.0


def test_two_jittering_pieces_are_tolerated_three_are_not():
    st = _state(grams=39.0)
    inside = np.flatnonzero(task.in_bowl(st, st["cereal"].pos))
    st["cereal"].vel[inside[:2]] = (0.1, 0.0, 0.0)
    settle(st)
    assert task.trial()["still"] >= task.STILL_S
    st["cereal"].vel[inside[2]] = (0.1, 0.0, 0.0)
    task.note_step(st, 0.05)
    assert task.trial()["still"] == 0.0


def test_reset_clears_the_trial():
    st = _state(grams=39.0)
    settle(st)
    task.reset_trial()
    assert task.trial()["still"] == 0.0


# -- demo -------------------------------------------------------------------------------------------------------------
def test_demo_plans_are_reachable_and_within_the_run_limit():
    pytest.importorskip("yam_arena")
    plans = task.demo()
    for arm in ("left", "right"):
        assert plans[arm][0].duration < 2400 * task.DT
    q6 = plans["right"][0].q[:, 5]
    assert q6.min() > -2.0944 and q6.max() < 2.0944


def test_demo_turns_the_knob_by_the_planned_amount():
    pytest.importorskip("yam_arena")
    p = task.demo()["right"][0]
    turning = [i for i, lab in enumerate(p.labels) if lab == "turn"]
    total = 0.0
    for a, b in zip(turning[:-1], turning[1:]):
        if b == a + 1:
            total += p.q[b, 5] - p.q[a, 5]
    assert abs(math.degrees(total) - math.degrees(task.TURN_TOTAL)) < 15.0
