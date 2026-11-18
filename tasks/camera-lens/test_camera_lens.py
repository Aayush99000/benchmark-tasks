"""Offline checks for camera_lens.py (no simulator): the bayonet (tabs, turn, stop, lock, release), the scoring, the
sizes, and the scripted demo against the YAM's real reach, step guard and collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import camera_lens as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

AWAY = np.array([0.31, -0.175, M.CLOTH_TOP + M.BAYONET_H])  # standing on the cloth


def frame(pos, rot=np.eye(3), vel=np.zeros(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.asarray(vel, float), spin=np.zeros(3), com=p)


def turned(deg: float) -> np.ndarray:
    return M.rot_z(M.INDEX_ANGLE + math.radians(deg))


class Rig:
    """Feeds the mount model poses: each lens at a height over the mount's face (or at a place) and turned so many
    degrees from the index, and the button pushed in so far."""

    def __init__(self):
        self.m = M.Mount()
        self.m.log = self.m.trace = None
        self.old = (M.MOUNT_AT.copy(), turned(-M.LOCK_DEG))
        self.new = (M.NEW_AT.copy(), M.R_INSERT)
        self.button = 0.0

    def step(self, n=24):
        acc = {}
        for _ in range(n):
            acc = {}
            btn = M.BUTTON_AT - np.array([0.0, 0.0, self.button])
            self.m.update({"old_lens": frame(*self.old), "new_lens": frame(*self.new), "release_button": frame(btn)},
                          acc)
            # the model's written poses are where the lenses are next substep
            for name, (p, q) in self.m.poses.items():
                pose = (np.asarray(p, float), M.fittingkit.quat_to_matrix(q))
                if name == "old_lens":
                    self.old = pose
                else:
                    self.new = pose
        return acc

    def old_off(self):
        """The zoom lens already off, standing on the cloth."""
        self.m.engaged["old_lens"] = self.m.locked["old_lens"] = False
        self.old = (AWAY, M.R_INSERT)

    def turn(self, name, deg, up=0.0, n=24):
        pose = (M.MOUNT_AT + np.array([0.0, 0.0, up]), turned(deg))
        if name == "old_lens":
            self.old = pose
        else:
            self.new = pose
        return self.step(n)


def test_the_zoom_lens_starts_locked_and_doesnt_turn_without_the_button():
    r = Rig()
    r.step()
    assert r.m.locked["old_lens"]
    r.turn("old_lens", -40.0)
    assert r.m.locked["old_lens"] and r.m.twist["old_lens"] == pytest.approx(-M.LOCK_DEG)
    assert M.twist_deg(r.old[1]) == pytest.approx(-M.LOCK_DEG)  # held at the stop
    r.turn("old_lens", 0.0, up=0.02)  # pulled straight up and turned: still held on the mount
    assert r.old[0][2] == pytest.approx(M.MOUNT_AT[2]) and r.m.engaged["old_lens"]


def test_with_the_button_in_it_turns_back_to_the_index_and_comes_off():
    r = Rig()
    r.button = 2.5 * M.MM
    r.step()
    for deg in np.linspace(-58.0, -10.0, 8):
        r.turn("old_lens", deg, n=6)
        assert r.m.engaged["old_lens"] and not r.m.locked["old_lens"]
    r.turn("old_lens", -1.0)
    assert not r.m.engaged["old_lens"]
    r.button = 0.0
    r.old = (M.MOUNT_AT + np.array([0.0, 0.0, 0.03]), turned(1.0))
    r.step()
    assert not r.m.engaged["old_lens"] and not r.m.seated["old_lens"]


def test_let_go_of_the_button_at_the_stop_it_locks_again():
    r = Rig()
    r.button = 2.5 * M.MM
    r.step()
    assert not r.m.locked["old_lens"]
    r.button = 0.0
    r.step()
    assert r.m.locked["old_lens"]


def test_a_lens_not_lined_up_rests_on_the_tabs():
    r = Rig()
    r.old_off()
    acc = r.turn("new_lens", 20.0, up=1.0 * M.MM)
    assert acc["new_lens"][0][2] > 10.0  # pushed up
    assert not r.m.engaged["new_lens"]
    acc = r.turn("new_lens", 3.0, up=0.5 * M.MM)
    assert "new_lens" not in acc  # lined up: no push, it goes down onto the face


def test_the_throat_steers_a_lens_a_little_off_its_axis_in():
    r = Rig()
    r.old_off()
    r.new = (M.MOUNT_AT + np.array([0.004, 0.0, 0.005]), M.R_INSERT)
    acc = r.step(1)
    f = acc["new_lens"][0]
    assert f[0] < -3.0 and abs(f[2]) < 1e-9  # towards the axis, and no lift (lined up)
    r.new = (M.MOUNT_AT + np.array([0.010, 0.0, 0.005]), M.R_INSERT)
    assert "new_lens" not in r.step(1)  # too far off: it's on the ring


def test_the_new_lens_turns_in_and_clicks_at_the_stop():
    r = Rig()
    r.old_off()
    r.turn("new_lens", 0.0, up=0.3 * M.MM)
    assert r.m.seated["new_lens"] and not r.m.engaged["new_lens"]
    r.turn("new_lens", -10.0)
    assert r.m.engaged["new_lens"] and not r.m.locked["new_lens"]
    r.turn("new_lens", -30.0, up=0.01)  # held on the face while it turns
    assert r.new[0][2] == pytest.approx(M.MOUNT_AT[2])
    r.turn("new_lens", -62.0)
    assert r.m.locked["new_lens"] and M.twist_deg(r.new[1]) == pytest.approx(-M.LOCK_DEG)
    assert r.m.clicks["new_lens"] == 1


def test_turned_the_wrong_way_it_stays_free_and_rides_on_the_tabs():
    r = Rig()
    r.old_off()
    acc = r.turn("new_lens", 15.0, up=0.5 * M.MM)
    assert not r.m.engaged["new_lens"] and acc["new_lens"][0][2] > 0.0


def scored(rig, old_vel=np.zeros(3)):
    M.MODEL.__dict__.update({k: v for k, v in rig.m.__dict__.items() if k not in ("log", "trace", "frames")})
    M.MODEL.frames = {"old_lens": frame(*rig.old, vel=old_vel), "new_lens": frame(*rig.new)}
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    M.MODEL.frames = {}
    return s, ok


def test_the_tiers():
    r = Rig()
    r.step()
    assert scored(r) == (0.0, False)  # the zoom lens still on
    r.button = 2.5 * M.MM
    r.turn("old_lens", -1.0)
    r.button = 0.0
    r.old = (AWAY, M.R_INSERT)
    r.step()
    assert scored(r) == (0.25, False)  # the zoom lens off and down
    r.turn("new_lens", 0.0)
    assert scored(r) == (0.5, False)  # the new lens on, not turned
    r.turn("new_lens", -30.0)
    assert scored(r) == (0.5, False)  # turned part way
    r.turn("new_lens", -62.0)
    assert scored(r) == (0.9, True)  # clicked
    assert scored(r, old_vel=np.array([0.0, 0.0, 0.1]))[0] == 0.5  # the zoom lens still moving: not set down
    r.button = 2.5 * M.MM
    r.step()
    assert scored(r)[1] is False  # the button still held in


def test_the_zoom_lens_held_in_the_air_is_not_set_down():
    r = Rig()
    r.button = 2.5 * M.MM
    r.turn("old_lens", -1.0)
    r.button = 0.0
    r.old = (M.MOUNT_AT + np.array([0.0, -0.1, 0.05]), M.R_INSERT)
    r.step()
    r.turn("new_lens", 0.0)
    r.turn("new_lens", -62.0)
    assert scored(r) == (0.5, False)


def test_real_sizes_and_fits():
    assert 2 * M.LENS_R == pytest.approx(0.0692)
    assert 2 * M.LENS_R < 0.094 - 0.02  # the open gripper passes round a lens with 1 cm either side
    assert M.BAYONET_R < M.THROAT_R < M.LENS_R
    assert M.UNLOCK < M.BUTTON_TRAVEL
    assert 2.0 < M.BUTTON_K * M.UNLOCK < 6.0  # N to unlock: a thumb press
    assert M.BUTTON_R - M.BUTTON_RADIUS > M.LENS_R + 5.0 * M.MM  # the button clear of the lens


def test_both_lenses_start_where_the_story_says():
    assert M.twist_deg(M.R_LOCKED) == pytest.approx(-M.LOCK_DEG)
    assert M.twist_deg(M.R_INSERT) == pytest.approx(0.0)


GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = []
    for t in np.arange(0.0, plan.duration, 0.05):
        acts.append(plan.action7(t)[0])
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(np.array(acts), axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_both_arms_follow_the_same_timeline():
    d = M.demo()
    assert [w.label for w in d["left"]] == [w.label for w in d["right"]]
    assert sum(w.duration for w in d["right"]) < 50.0  # well inside the 60 s run


@pytest.mark.slow
def test_the_demo_touches_nothing_it_doesnt_hold_and_turns_each_lens_right():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not out["hits"], out["hits"][:5]
    assert out["min_hand_gap_mm"] > 10.0
    assert out["twist"]["let the button go, turn it to the stop"] > M.FREE_DEG - 4.0  # back past the index (it comes free at -2)
    assert abs(out["twist"]["set it on the mount"]) < M.INSERT_DEG / 2
    assert out["twist"]["turn it until it clicks"] < -M.LOCK_DEG + M.CLICK_DEG
