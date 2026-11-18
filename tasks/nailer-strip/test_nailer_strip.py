"""Offline checks for nailer_strip.py (no simulator): the magazine (the wrong way round, the nail stop, the pusher's
drag, dropping in behind the strip, feeding it, a snap), the scoring, the sizes, and the scripted demo against the YAM's
real reach and step guard."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nailer_strip as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

A = M.R_MAG[:, 0]  # into the track, towards the driver


def frame(pos, rot=np.eye(3), vel=np.zeros(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.asarray(vel, float), spin=np.zeros(3), com=p)


class Rig:
    """Feeds the magazine model poses: the strip with its back so far into the track (or anywhere), and the pusher's
    finger so far in, moving at a speed (towards the driver positive)."""

    def __init__(self):
        self.m = M.Magazine()
        self.m.log = self.m.trace = None
        self.strip = (M.STRIP_AT.copy(), M.R_STRIP_START)
        self.finger, self.speed = M.A_DRIVE, 0.0

    def put(self, rear: float, rot=M.R_STRIP_IN, w=0.0, n=M.REST_N):
        self.strip = (M.mag(rear + M.STRIP_L / 2, w, n), rot)

    def step(self, n=24) -> dict:
        acc = {}
        for _ in range(n):
            acc = {}
            pusher = np.array([M.PUSHER_X, M.knob_y(self.finger), M.PUSHER_Z])
            self.m.update({"nail_strip": frame(*self.strip),
                           "pusher": frame(pusher, vel=np.array([0.0, -self.speed, 0.0]))}, acc)
            for name, (p, q) in self.m.poses.items():
                self.strip = (np.asarray(p, float), M.fittingkit.quat_to_matrix(q))
        return acc

    def force_on(self, name: str, acc: dict) -> np.ndarray:
        return acc.get(name, [np.zeros(3), np.zeros(3)])[0]


def test_the_sizes_are_a_real_strip_and_magazine():
    assert M.STRIP_L == pytest.approx(0.215, abs=0.002)
    assert M.LEAN == pytest.approx(0.0293, abs=0.0005)
    assert M.NAIL_L == pytest.approx(0.0762)  # 3 in
    # the heads don't fit the shanks' slot; the band and the heads fit their tracks
    assert M.HEAD_D > M.SLOT_T - 0.5e-3 and M.BAND_T < M.SLOT_T and M.HEAD_D < M.HEAD_TRACK_T
    assert M.TRACK_W > M.NAIL_L
    # the gripper opens 94 mm: wide enough round the strip
    assert 2 * 0.047 > M.NAIL_L + 0.01


def test_a_strip_the_wrong_way_round_is_stopped_at_the_rear_end():
    r = Rig()
    flipped = M.rot_z(np.pi) @ M.R_STRIP_IN  # heads on the points' side (and back to front)
    r.put(-M.STRIP_L + 0.010, rot=flipped)
    acc = r.step(1)
    assert r.force_on("nail_strip", acc) @ A < -5.0  # pushed back out
    assert not r.m.right_way


def test_a_strip_lined_up_is_steered_into_the_rear_end():
    r = Rig()
    r.put(-M.STRIP_L - 0.005, w=0.004)
    acc = r.step(1)
    f = r.force_on("nail_strip", acc)
    assert f @ M.R_MAG[:, 1] < 0.0 and np.linalg.norm(f) <= M.LEAD_MAX + 1e-9


def test_the_nail_stop_holds_a_strip_pushed_past_it():
    r = Rig()
    r.put(0.020)
    r.step()
    assert r.m.in_track and r.m.right_way and not r.m.past_stop
    r.put(0.040)
    r.step()
    assert r.m.past_stop
    r.put(0.020)  # pulled back out: held at the stop
    r.step(1)
    rel, _ = M.strip_in_mag(frame(*r.strip))
    assert rel[0] - M.STRIP_L / 2 == pytest.approx(M.TAB)


def test_the_pusher_drags_a_strip_back_as_it_rides_over_it():
    r = Rig()
    r.put(0.010)
    r.finger, r.speed = 0.15, -0.1  # riding back over the strip
    acc = r.step(1)
    assert r.force_on("nail_strip", acc) @ A == pytest.approx(-M.DRAG)
    assert not r.m.armed


def test_the_finger_drops_in_behind_and_feeds_the_strip_to_the_driver():
    r = Rig()
    r.put(0.050)
    r.step()
    r.finger, r.speed = 0.032, -0.05  # pulled back past the strip
    r.step()
    assert r.m.armed and not r.m.engaged
    r.finger, r.speed = 0.0505, 0.04  # eased forward onto it
    r.step(2)
    assert r.m.engaged and not r.m.snapped and r.m.hit_speed == pytest.approx(0.04)
    for fa in np.linspace(0.051, M.A_DRIVE - M.STRIP_L, 20):
        r.finger = fa
        r.step(4)
    assert r.m.home
    rel, rr = M.strip_in_mag(frame(*r.strip))
    assert rel[0] + M.STRIP_L / 2 == pytest.approx(M.A_DRIVE, abs=1e-6) and np.allclose(rr, np.eye(3))
    r.finger = M.A_DRIVE - M.STRIP_L + 0.005  # pushed on past where the driver stops it: held there
    r.step(1)
    assert r.m.poses["pusher"][0][1] == pytest.approx(M.knob_y(M.A_DRIVE - M.STRIP_L))
    assert r.m.home


def test_a_pusher_let_go_snaps():
    r = Rig()
    r.put(0.050)
    r.step()
    r.finger, r.speed = 0.032, -0.05
    r.step()
    r.finger, r.speed = 0.0505, 1.5  # let go: it flies forward onto the strip
    r.step(1)
    assert r.m.engaged and r.m.snapped


def test_a_strip_in_back_to_front_doesnt_feed():
    r = Rig()
    r.put(0.050, rot=M.R_STRIP_IN @ np.diag([-1.0, 1.0, -1.0]))  # turned over end to end: heads still to the wall
    r.step()
    assert r.m.in_track and not r.m.right_way
    r.finger, r.speed = 0.030, -0.05
    r.step()
    r.finger, r.speed = 0.051, 0.04
    r.step()
    assert not r.m.armed and not r.m.engaged


def _set(m, **kw):
    M.MODEL.clear()
    for k, v in kw.items():
        setattr(M.MODEL, k, v)


def test_the_scores():
    _set(M.MODEL)
    assert M.score(None) == 0.0 and not M.success(None)
    _set(M.MODEL, in_track=True, right_way=True, past_stop=True)
    assert M.score(None) == 0.25
    _set(M.MODEL, in_track=True, right_way=False, past_stop=False)
    assert M.score(None) == 0.0
    _set(M.MODEL, engaged=True, home=False)
    assert M.score(None) == 0.25 and not M.success(None)
    _set(M.MODEL, engaged=True, home=True, snapped=True)
    assert M.score(None) == 0.5 and not M.success(None)
    _set(M.MODEL, engaged=True, home=True)
    assert M.score(None) == M.COMPLETE_SCORE and M.success(None)  # offline the hands count as clear
    M.MODEL.clear()


G = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])  # the run's per-step guard


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_reaches_every_pose_and_stays_under_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = []
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.worst_ik_mm = plan.worst_ik_deg = 0.0
        a, label = plan.action7(t)
        acts.append(a)
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.2, (label, plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(np.array(acts), axis=0)) / G).max() < 0.9


def test_the_demo_fits_the_run():
    d = M.demo()
    assert sum(w.duration for w in d["left"]) == pytest.approx(sum(w.duration for w in d["right"]))
    assert sum(w.duration for w in d["left"]) < 1200 / 20 - 5
