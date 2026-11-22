"""Scoring checks for close-door-quietly that run without the simulator: python -m pytest -q test_door_scoring.py

Trials are synthetic: a door, knob and bolt driven step by step through a simple latch model (the knob holds the bolt
in; within the last THROW of the close the strike's beveled lip pushes it in; at the stop it is free to come out into
the hole). The model only stands in for the simulator's latch, to feed the scorer realistic joint traces.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import door_scoring as S

OPEN = math.radians(8)  # a synthetic start, wider than the scene's 6 deg: latch edge about 85 mm off the stop
TURNED = S.KNOB_FULL


def latch_bolt(door: float, knob: float) -> float:
    """The bolt's push-in (m) for this door angle and knob turn: the knob's hold or the strike lip's push."""
    gap = S.edge_gap(door)
    lip = S.THROW - gap if 0.0 < gap < S.THROW else 0.0
    return max(S.knob_holds_in(knob), lip)


def on_knob(door: float) -> list:
    """Right gripper's pad points round the knob's rim; left gripper parked by its base."""
    c = S.knob_centre(door)
    return [np.array([c + (0, 0, S.KNOB_R), c - (0, 0, S.KNOB_R)]), np.array([[0.2, 0.3, 0.9], [0.2, 0.32, 0.9]])]


PARKED = [np.array([[0.1, -0.3, 0.95], [0.1, -0.28, 0.95]]), np.array([[0.2, 0.3, 0.9], [0.2, 0.32, 0.9]])]


def ramp(a: float, b: float, n: int) -> list[float]:
    return [a + (b - a) * (k + 1) / n for k in range(n)]


def run(segments):
    """Play (door, knob, pads) steps through the latch model; return the record and the final state."""
    rec = S.DoorRecord()
    door = knob = bolt = 0.0
    pads = PARKED
    for door, knob, pads in segments:
        bolt = latch_bolt(door, knob)
        rec.step(door, knob, bolt, pads)
    return rec, door, knob, bolt, pads


def quiet_close(close_steps=40, release_steps=20, release_at=0.0, end_pads=PARKED, hold=20):
    steps = [(OPEN, k, on_knob(OPEN)) for k in ramp(0.0, TURNED, 10)]  # grip and turn
    steps += [(d, TURNED, on_knob(d)) for d in ramp(OPEN, release_at, close_steps)]  # draw shut, knob held
    steps += [(release_at, k, on_knob(release_at)) for k in ramp(TURNED, 0.0, release_steps)]  # ease the knob back
    if release_at > 0.0:
        steps += [(d, 0.0, on_knob(d)) for d in ramp(release_at, 0.0, 10)]  # then pushed the rest of the way
    steps += [(0.0, 0.0, end_pads)] * hold
    return steps


def test_quiet_close_succeeds_and_score_is_capped():
    rec, *final = run(quiet_close())
    assert not rec.scraped and not rec.hand_at_latch
    assert 0.0 < rec.snap_speed < S.SNAP  # 13 mm over 20 steps (1 s): under SNAP at its fastest
    assert S.grade(rec, *final) == 1.0
    assert S.success(rec, *final)
    assert S.score(rec, *final) == S.PARTIAL_CAP


def test_push_shut_clicks_for_half():
    steps = [(d, 0.0, on_knob(d)) for d in ramp(OPEN, 0.0, 30)] + [(0.0, 0.0, PARKED)] * 20
    rec, *final = run(steps)
    assert rec.scraped
    assert S.grade(rec, *final) == 0.5


def test_fast_release_snaps_for_three_quarters():
    rec, *final = run(quiet_close(release_steps=1))
    assert not rec.scraped and rec.snap_speed > S.SNAP
    assert S.grade(rec, *final) == 0.75
    assert not S.success(rec, *final)


def fastest_out(release_steps: int) -> float:
    """The bolt's fastest outward speed (m/s) when the knob goes back from 45 deg to 0 evenly over n steps."""
    knobs = [TURNED] + ramp(TURNED, 0.0, release_steps)
    ins = [S.knob_holds_in(k) for k in knobs]
    return max(a - b for a, b in zip(ins, ins[1:])) / S.DT


def test_cam_holds_the_bolt_flush_at_45_and_lets_it_out_at_rest():
    assert S.knob_holds_in(TURNED) == S.THROW and S.knob_holds_in(-TURNED) == S.THROW
    assert S.knob_holds_in(0.0) == 0.0
    ins = [S.knob_holds_in(math.radians(d)) for d in range(0, 46)]
    assert all(b >= a for a, b in zip(ins, ins[1:]))  # turning further never lets the bolt out


def test_snap_threshold():
    # the fewest even release steps that stay under SNAP pass; one step fewer snaps
    n = min(k for k in range(1, 60) if fastest_out(k) <= S.SNAP)
    assert fastest_out(n) <= S.SNAP < fastest_out(n - 1)
    rec, *final = run(quiet_close(release_steps=n))
    assert rec.snap_speed == pytest.approx(fastest_out(n)) and S.grade(rec, *final) == 1.0
    rec, *final = run(quiet_close(release_steps=n - 1))
    assert rec.snap_speed == pytest.approx(fastest_out(n - 1)) and S.grade(rec, *final) == 0.75


def test_measured_releases_fall_either_side_of_snap():
    # official sim, 2026-11-22: the demo's 2 s release peaked at 10 mm/s, a one-step twist back at 49 mm/s
    assert 0.010 < S.SNAP < 0.049


def test_door_meets_stop_too_fast_for_three_quarters():
    # 85 mm in 5 steps: 0.34 m/s average, but the last step 17 mm in 0.05 s is 0.34 m/s > SLAM -> 0
    rec, *final = run(quiet_close(close_steps=5))
    assert rec.close_speed > S.SLAM and S.grade(rec, *final) == 0.0
    # 85 mm in 12 steps: 0.14 m/s at the stop, between GENTLE and SLAM -> 0.75
    rec, *final = run(quiet_close(close_steps=12))
    assert S.GENTLE < rec.close_speed <= S.SLAM
    assert S.grade(rec, *final) == 0.75


def test_knob_still_held_at_the_end_is_not_latched():
    steps = quiet_close(release_steps=1)[:-21]  # drop the release and the hold
    steps += [(0.0, TURNED, on_knob(0.0))] * 5
    rec, *final = run(steps)
    assert final[2] > S.LATCHED_IN
    assert S.grade(rec, *final) == 0.25


def test_door_left_ajar_scores_zero():
    steps = [(OPEN, k, on_knob(OPEN)) for k in ramp(0.0, TURNED, 10)]
    steps += [(d, TURNED, on_knob(d)) for d in ramp(OPEN, math.radians(1), 30)]
    steps += [(math.radians(1), k, PARKED) for k in ramp(TURNED, 0.0, 20)]
    rec, *final = run(steps)
    assert S.edge_gap(final[0]) > S.SHUT and S.grade(rec, *final) == 0.0


def test_released_before_the_stop_scrapes():
    # knob let go with the latch edge 5 mm off the stop: the bolt comes out onto the strike's lip, then rides it
    rec, *final = run(quiet_close(release_at=0.005 / S.WIDTH))
    assert rec.scraped and S.grade(rec, *final) == 0.5


def test_hand_at_the_latch_scores_zero():
    poke = [np.array([S.bolt_nose(0.0) + (0.0, 0.005, 0.0)]), PARKED[1]]
    steps = quiet_close()
    steps.insert(30, (steps[30][0], steps[30][1], poke))
    rec, *final = run(steps)
    assert rec.hand_at_latch and S.grade(rec, *final) == 0.0


def test_hand_on_the_knob_is_not_at_the_latch():
    for door in (OPEN, 0.0):
        assert not S.hand_at_latch(on_knob(door), door)


def test_gripper_still_on_the_knob_at_the_end_is_not_full():
    rec, *final = run(quiet_close(end_pads=on_knob(0.0)))
    assert S.grade(rec, *final) == 0.75 and not S.success(rec, *final)


def test_still_moving_is_not_full():
    rec, *final = run(quiet_close(hold=0))
    rec.edge_speed = 2 * S.STILL
    assert S.grade(rec, *final) == 0.75


def test_reset_clears_the_record():
    rec, *_ = run([(d, 0.0, on_knob(d)) for d in ramp(OPEN, 0.0, 30)])
    assert rec.scraped
    rec.reset()
    assert (rec.steps, rec.scraped, rec.snap_speed, rec.close_speed, rec.prev, rec.first) == (0, False, 0.0, 0.0,
                                                                                              None, {})


def test_geometry_matches_the_reach_check():
    # tools/check_reach.py's chosen layout puts the shut door's knob at x 0.2997, y -0.6826, 0.154 m above the table
    c = S.knob_centre(0.0)
    assert c == pytest.approx([-0.25 + 0.610 - 0.0603, -0.75 + 0.0349 / 2 + 0.050, -0.0125 + 0.914])
    assert c[2] - 0.7475 == pytest.approx(0.154, abs=1e-3)
    assert S.bolt_nose(0.0) == pytest.approx(S.STRIKE)
    assert S.edge_gap(OPEN) == pytest.approx(0.085, abs=0.001)
