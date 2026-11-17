"""Offline checks for shielded_pin.py (no simulator): the keypad's press log, the line-of-sight test, the scoring, the
layout, and the scripted demo against the YAM's real gripper collision hulls (URDF forward kinematics)."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import shielded_pin as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402


def key_frame(name, depth) -> Frame:
    x, y = M.key_xy(name)
    p = np.array([x, y, M.KEY_REST_Z - depth])
    return Frame(pos=p, rot=np.eye(3), vel=np.zeros(3), spin=np.zeros(3), com=p)


def box_planes(lo, hi) -> np.ndarray:
    """A box as outward planes n . x = d."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    out = []
    for i in range(3):
        n = np.zeros(3)
        n[i] = 1.0
        out += [np.r_[n, hi[i]], np.r_[-n, -lo[i]]]
    return np.array(out)


def press(model, *names, depth=1.0 * M.MM, together=()):
    """Press each key in turn (and let it up), the keys in ``together`` held down meanwhile."""
    for name in names:
        frames = {k.name: key_frame(k.name[4:], depth if (k.name[4:] == name or k.name[4:] in together) else 0.0)
                  for k in M.KEYS}
        model.update(frames, {})
        model.update({k.name: key_frame(k.name[4:], depth if k.name[4:] in together else 0.0) for k in M.KEYS}, {})


@pytest.fixture
def model():
    m = M.PinPad()
    m.occluders = [box_planes((0.0, -1.0, 0.0), (2.0, 1.0, 2.0))]  # a wall hiding everything
    m.log = None
    return m


# -- the press log ----------------------------------------------------------------------------------------------------

def test_a_key_registers_past_its_click_once(model):
    for d in (0.5, 0.7, 0.9, 1.1, 0.9, 0.5):
        model.update({"key_7": key_frame("7", d * M.MM)}, {})
    assert model.keys() == ("7",)
    for d in (0.3, 0.9):  # let up past the release point, then pressed again: a second press
        model.update({"key_7": key_frame("7", d * M.MM)}, {})
    assert model.keys() == ("7", "7")


def test_short_of_the_click_is_no_press(model):
    model.update({"key_7": key_frame("7", 0.7 * M.MM)}, {})
    assert model.keys() == ()


def test_two_keys_down_at_once_is_flagged(model):
    press(model, "8", together=("7",))
    assert not model.clean()


def test_the_pin_logged_in_order(model):
    press(model, *M.EXPECTED)
    assert model.keys() == M.EXPECTED and model.clean() and model.all_hidden()


# -- the line of sight -----------------------------------------------------------------------------------------------

def test_a_segment_through_a_box_hits_it_and_one_beside_it_does_not():
    planes = box_planes((-1, -1, -1), (1, 1, 1))
    assert M.segment_hits_hull(np.array([-5.0, 0, 0]), np.array([5.0, 0, 0]), planes)
    assert not M.segment_hits_hull(np.array([-5.0, 2, 0]), np.array([5.0, 2, 0]), planes)
    assert not M.segment_hits_hull(np.array([-5.0, 0, 0]), np.array([-2.0, 0, 0]), planes)  # stops short


def test_nothing_in_the_way_means_seen():
    corners = M.key_corners("5")
    assert not M.hidden(corners, [])
    assert not M.hidden(corners, [box_planes((-1, 2, 0), (1, 3, 1))])  # a box off to the side


def test_a_press_with_no_screen_is_seen(model):
    model.occluders = []
    press(model, *M.EXPECTED)
    assert model.keys() == M.EXPECTED and not model.all_hidden()


# -- scoring ---------------------------------------------------------------------------------------------------------

def scored(keys, hidden=True, clean=True):
    M.MODEL.clear()
    M.MODEL.presses = [(k, hidden, not clean) for k in keys]
    s, ok = M.score({}), M.success({})
    M.MODEL.clear()
    return s, ok


def test_the_tiers():
    assert scored(M.EXPECTED) == (0.9, True)
    assert scored(M.EXPECTED, hidden=False) == (0.5, False)
    assert scored(M.PIN) == (0.25, False)
    assert scored(M.EXPECTED, clean=False) == (0.0, False)
    assert scored(("7", "3", "2", "1", "9", "green")) == (0.0, False)  # a wrong key
    assert scored(("7", "3", "9", "green")) == (0.0, False)  # a digit missing
    assert scored(M.EXPECTED + ("green",)) == (0.0, False)  # an extra press


# -- the layout ------------------------------------------------------------------------------------------------------

def test_the_keypad_is_a_standard_one_seen_from_the_customer():
    # 1 2 3 across the top (farthest from the customer), 1 on the customer's left (+y); the coloured keys nearest
    assert M.key_xy("1")[0] > M.key_xy("4")[0] > M.key_xy("7")[0] > M.key_xy("0")[0] > M.key_xy("green")[0]
    assert M.key_xy("1")[1] > M.key_xy("2")[1] > M.key_xy("3")[1]
    assert M.key_xy("0")[1] == pytest.approx(M.key_xy("5")[1])
    assert len(M.KEYS) == 15


def test_the_terminal_is_real_size_and_the_keys_fit():
    assert (M.TERM_L, M.TERM_W) == pytest.approx((0.180, 0.083))
    assert M.KEY_W < 22.0 * M.MM and M.KEY_D < 16.0 * M.MM  # gaps between keys
    assert M.CLICK < M.KEY_TRAVEL < M.KEY_PROUD


def test_the_case_keeps_clear_of_every_key_through_its_travel():
    """The keys collide with the case in the sim (their joints don't stop it), so no case box may come near a key."""
    origin = np.array([*M.TERM_XY, M.TOP])
    boxes = [(origin + np.array(s.centre) - np.array(s.size) / 2, origin + np.array(s.centre) + np.array(s.size) / 2)
             for s in M.TERMINAL.shapes if s.kind == "box"]
    boxes = boxes[:len(M._CS)]  # the collision shapes, not the screen's visual
    for name in [k.name[4:] for k in M.KEYS]:
        x, y = M.key_xy(name)
        klo = np.array([x - M.KEY_D / 2, y - M.KEY_W / 2, M.KEY_TOP - M.KEY_TRAVEL - M.KEY_H])
        khi = np.array([x + M.KEY_D / 2, y + M.KEY_W / 2, M.KEY_TOP])
        for lo, hi in boxes:
            gap = np.maximum(0.0, np.maximum(lo - khi, klo - hi))
            assert np.linalg.norm(gap) >= M.KEY_CLEAR - 1e-9, (name, lo, hi)


def test_a_key_takes_about_2_5_n_to_click():
    force = M.KEY_K * (M.KEY_PRELOAD + M.CLICK)
    assert 2.0 < force < 3.5


def test_the_onlooker_looks_down_from_across_the_counter():
    d = M.EYE - M.KEYPAD_MID
    assert d[0] == pytest.approx(0.60) and d[2] == pytest.approx(0.45)


# -- the demo --------------------------------------------------------------------------------------------------------

GUARD_MAX_DELTA = np.array([0.287979, 0.1832595, 0.1570795, 0.1631885, 0.15708, 0.20944, 0.05])


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach_and_the_step_guard(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    acts = np.array([plan.action7(t)[0] for t in np.arange(0.0, plan.duration, 0.05)])
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert (np.abs(np.diff(acts, axis=0)) / GUARD_MAX_DELTA).max() < 0.95


def test_both_arms_follow_the_same_timeline():
    d = M.demo()
    assert [w.label for w in d["left"]] == [w.label for w in d["right"]]
    assert sum(w.duration for w in d["left"]) == pytest.approx(sum(w.duration for w in d["right"]))


@pytest.mark.slow
def test_the_demo_hides_every_press_behind_the_screen_and_the_hands_never_meet():
    """At each press, with the YAM's real gripper hulls: every corner of the key is hidden by the screening hand
    alone; the pressing fingertips touch only their key; the screen stays above the keys; the hands stay apart."""
    import plan_check

    out = plan_check.run(verbose=False)
    for p in out["presses"]:
        assert all(p["shield_alone"]), p
        assert p["only_on_its_key"], p
        assert p["shield_above_keys_mm"] > 2.0, p
        assert p["tip_below_top_mm"] > M.CLICK * 1000, p
    # closest at the end of each approach, the pressing fingertips 6 mm above the key: 2.7 mm (2026-11-17); in the sim
    # the screening hand held still through every press (run pin8)
    assert out["min_gap_mm"] > 2.0
