"""Offline checks for soap_dispenser_refill.py (no simulator): the pump house (groove, detent, click, lock, release),
the cover's latch, the scoring, the sizes, and the scripted demo against the YAM's real reach, step guard and
collision hulls."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import soap_dispenser_refill as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

SEAT = np.array([M.XS, 0.0, M.SEAT_Z])
# the empty refill lying in the recycling box: its frame turned on its side, its centre of mass low over the box's middle
IN_BOX_ROT = M.rot_y(math.pi / 2)
IN_BOX = np.array([M.BOX_AT[0], M.BOX_AT[1], M.TOP + 0.05]) - IN_BOX_ROT @ np.array(M.OLD_REFILL.com)


def frame(pos, rot=np.eye(3), vel=np.zeros(3)) -> Frame:
    p = np.asarray(pos, float)
    return Frame(pos=p, rot=np.asarray(rot, float), vel=np.asarray(vel, float), spin=np.zeros(3), com=p)


class Rig:
    """Feeds the pump house model poses: each refill somewhere, the latch pushed in so far, the cover open so far."""

    def __init__(self):
        self.m = M.PumpHouse()
        self.m.log = self.m.trace = None
        self.old = (SEAT.copy(), np.eye(3))
        self.new = (M.NEW_AT.copy(), np.eye(3))
        self.latch = 0.0
        self.cover = M.OPEN_DEG

    def step(self, n=24):
        acc = {}
        for _ in range(n):
            acc = {}
            cover_rot = M.rot_y(-math.radians(self.cover))
            self.m.update({"old_refill": frame(*self.old), "new_refill": frame(*self.new),
                           "release_latch": frame(M.LATCH_AT + np.array([self.latch, 0.0, 0.0])),
                           "cover": frame(M.HINGE_AT, cover_rot)}, acc)
            for name, (p, q) in self.m.poses.items():  # the model's written poses are where they are next substep
                pose = (np.asarray(p, float), M.fittingkit.quat_to_matrix(q))
                if name == "old_refill":
                    self.old = pose
                elif name == "new_refill":
                    self.new = pose
                elif name == "cover":
                    self.cover = M.cover_open_deg(pose[1])
        return acc

    def put(self, name, depth, off=0.0, up=0.0, n=24):
        """A refill's collar ``depth`` past the pump house's front, ``off`` the groove's line, ``up`` off its floor."""
        pose = (np.array([M.XF + depth, off, M.SEAT_Z + up]), np.eye(3))
        if name == "old_refill":
            self.old = pose
        else:
            self.new = pose
        return self.step(n)

    def take_old_out(self):
        self.latch = 3.0 * M.MM
        self.step()
        self.latch = 0.0
        self.old = (IN_BOX.copy(), IN_BOX_ROT)
        self.step()


def scored(r):
    M.MODEL.locked = dict(r.m.locked)
    M.MODEL.offset = dict(r.m.offset)
    M.MODEL.frames = {"old_refill": frame(*r.old), "new_refill": frame(*r.new),
                      "cover": frame(M.HINGE_AT, M.rot_y(-math.radians(r.cover)))}
    M.MODEL.cover_latched = r.m.cover_latched
    return M.score(None), M.success(None)


def test_the_empty_refill_starts_locked_and_stays_without_the_latch():
    r = Rig()
    r.step()
    assert r.m.locked["old_refill"]
    acc = r.put("old_refill", 0.040, n=1)  # pulled 4 mm out: held, pulled back to its seat
    k = min(M.HOLD_K, M.OLD_REFILL.mass / (4.0 * M.HOLD_DT ** 2))  # the hold's spring, capped for stability
    assert r.m.locked["old_refill"] and acc["old_refill"][0][0] > 0.8 * k * 0.004
    acc = r.put("old_refill", 0.0, n=1)  # pulled 44 mm out: the spring, capped at 40 N
    assert np.linalg.norm(acc["old_refill"][0]) == pytest.approx(min(M.HOLD_MAX, k * 0.044))
    r.latch = 2.0 * M.MM  # pushed in, but short of the unlock point
    r.put("old_refill", 0.0)
    assert r.m.locked["old_refill"]


def test_the_hold_is_stable_at_the_substep():
    """The hold is an explicit force: each axis's gains must leave c dt / I and k dt^2 / I small (run soap25)."""
    dt = M.HOLD_DT
    for spec in (M.OLD_REFILL, M.NEW_REFILL):
        k = min(M.HOLD_K, spec.mass / (4.0 * dt ** 2))
        assert k * dt ** 2 / spec.mass <= 0.25 + 1e-9 and 2.0 * (k * spec.mass) ** 0.5 * dt / spec.mass <= 1.0 + 1e-9
        for inertia in spec.inertia:
            k_turn = min(M.HOLD_TURN_K, inertia / (4.0 * dt ** 2))
            c_turn = min(M.HOLD_TURN_C, 2.0 * (k_turn * inertia) ** 0.5)
            assert k_turn * dt ** 2 / inertia <= 0.25 + 1e-9 and c_turn * dt / inertia <= 1.0 + 1e-9


def test_the_latch_lets_the_empty_refill_go_and_it_clicks_again_if_left_home():
    r = Rig()
    r.latch = 2.6 * M.MM
    r.step()
    assert not r.m.locked["old_refill"]
    r.latch = 0.0
    r.step()  # still at the back of the groove when the latch comes out: it clicks again
    assert r.m.locked["old_refill"] and r.m.clicks["old_refill"] == 1
    r.latch = 2.6 * M.MM
    r.put("old_refill", 0.02)  # drawn 24 mm off its seat while the latch is in
    r.latch = 0.0
    r.step()
    assert not r.m.locked["old_refill"] and r.m.clicks["old_refill"] == 1


def test_the_detent_pushes_back_in_the_last_3_mm_and_the_collar_clicks_2_mm_short_of_the_seat():
    r = Rig()
    r.take_old_out()
    seat = M.XS - M.XF
    acc = r.put("new_refill", seat - 10.0 * M.MM)  # 10 mm short: free
    assert "new_refill" not in acc and not r.m.locked["new_refill"]
    acc = r.put("new_refill", seat - 2.5 * M.MM, n=1)  # in the detent: pushed back, 1.5 N
    assert acc["new_refill"][0][0] == pytest.approx(-M.DETENT_K * 0.5 * M.MM)
    assert not r.m.locked["new_refill"]
    r.put("new_refill", seat - 1.9 * M.MM)
    assert r.m.locked["new_refill"] and r.m.clicks["new_refill"] == 1
    acc = r.put("new_refill", seat - 3.0 * M.MM, n=1)  # locked: drawn 3 mm off its seat, it is pulled back
    assert acc["new_refill"][0][0] > 5.0 and r.m.locked["new_refill"]


def test_a_collar_off_the_groove_line_or_tilted_doesnt_click():
    seat = M.XS - M.XF
    for off, up, tilt in ((6.0 * M.MM, 0.0, 0.0), (0.0, 6.0 * M.MM, 0.0), (0.0, 0.0, 12.0)):
        r = Rig()
        r.take_old_out()
        rot = M.rot_y(math.radians(tilt))
        r.new = (np.array([M.XF + seat, off, M.SEAT_Z + up]), rot)
        r.step()
        assert not r.m.locked["new_refill"], (off, up, tilt)


def test_the_groove_mouth_steers_a_collar_to_its_line():
    r = Rig()
    r.take_old_out()
    acc = r.put("new_refill", 0.010, off=3.0 * M.MM, n=1)
    assert acc["new_refill"][0][1] == pytest.approx(-M.LEAD_K * 3.0 * M.MM)
    acc = r.put("new_refill", 0.010, off=8.0 * M.MM, n=1)  # capped
    assert acc["new_refill"][0][1] == pytest.approx(-M.LEAD_MAX)
    acc = r.put("new_refill", 0.010, off=20.0 * M.MM, n=1)  # beyond the chamfer
    assert "new_refill" not in acc


def test_the_cover_latches_only_within_1_5_degrees_and_is_held_shut():
    r = Rig()
    r.cover = 3.0
    r.step()
    assert not r.m.cover_latched
    r.cover = 1.0
    r.step()
    assert r.m.cover_latched
    r.cover = 20.0  # swung open again: held shut
    r.step()
    assert r.cover == pytest.approx(0.0, abs=1e-6)


def test_the_cover_starts_open_on_its_stop():
    assert M.cover_open_deg(M.R_COVER_OPEN) == pytest.approx(M.OPEN_DEG)
    assert M.cover_open_deg(np.eye(3)) == pytest.approx(0.0)


def test_scoring_orders_the_steps():
    r = Rig()
    r.step()
    assert scored(r) == (0.0, False)  # nothing done
    r.cover = 0.5
    r.step()
    assert scored(r) == (0.0, False)  # the cover shut over the empty refill
    r = Rig()
    r.take_old_out()
    assert scored(r) == (0.15, False)
    r.put("new_refill", 0.030)  # in the pump house, not clicked
    assert scored(r) == (0.35, False)
    r.put("new_refill", M.XS - M.XF)
    assert scored(r) == (0.6, False)  # clicked in, the cover open
    r.cover = 1.0
    r.step()
    assert scored(r) == (0.9, True)  # offline the hands count as clear: success() holds


def test_an_empty_refill_still_moving_isnt_put_down():
    r = Rig()
    r.take_old_out()
    r.put("new_refill", M.XS - M.XF)
    r.cover = 1.0
    r.step()
    M.MODEL.locked = dict(r.m.locked)
    M.MODEL.offset = dict(r.m.offset)
    M.MODEL.cover_latched = True
    M.MODEL.frames = {"old_refill": frame(IN_BOX, IN_BOX_ROT, vel=np.array([0.0, 0.0, -0.2])),
                      "new_refill": frame(SEAT), "cover": frame(M.HINGE_AT)}
    assert M.score(None) == 0.6 and not M.success(None)


def test_an_empty_refill_out_but_not_in_the_box_doesnt_finish_it():
    r = Rig()
    r.take_old_out()
    r.old = (np.array([0.22, -0.05, M.TOP + 0.003]), IN_BOX_ROT)  # put down on the table, not in the box
    r.step()
    r.put("new_refill", M.XS - M.XF)
    r.cover = 1.0
    r.step()
    assert scored(r) == (0.6, False)


def test_real_sizes_and_fits():
    assert M.WIDTH == pytest.approx(0.100) and M.HEIGHT == pytest.approx(0.249)
    # the ADX-7's 94 mm, 5.5 mm deeper here so the shut cover clears the latch
    assert M.DEPTH + M.COVER_GAP + M.COVER_T == pytest.approx(0.1025, abs=0.001)
    assert M.Z0 >= 0.076  # at least GOJO's 3 in above a counter
    assert M.BOTTLE_Y < 0.094 - 0.008  # the open gripper passes round a refill with 4.5 mm either side
    assert M.COLLAR_R < M.GROOVE_HALF < M.COLLAR_R + 3.0 * M.MM
    assert M.SEAT_Z - M.TOP + M.COLLAR_H + M.BOTTLE_H < M.Z0 + M.HEIGHT - M.CAP_H  # a seated refill under the top cap
    assert M.HINGE_AT[0] < M.XF - M.LATCH_OUT  # the shut cover clear of the latch
    assert 1.5 < M.LATCH_K * M.UNLOCK < 5.0  # N to let a refill go: a thumb push
    assert M.UNLOCK < M.LATCH_TRAVEL < M.WELL_D - M.LATCH_D + M.LATCH_OUT
    assert 2.0 <= M.DETENT_MAX < 10.0  # N: the most the sim's arm pushed the held refill against (runs soap23-26)


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
    assert sum(w.duration for w in d["right"]) < 56.5  # inside the 60 s run, with the 1 s hold


def test_the_demo_brings_each_refill_where_the_model_wants_it():
    d = M.demo()
    plan = ArmPlan(d["right"], station.BASES["right"])  # the right hand handles both refills
    ends, t = {}, 0.0
    for w in d["right"]:
        t += w.duration
        ends[w.label] = t
    p, r, _, _ = plan.target(ends["push it into the pump house"] - 1e-6)
    p = p - M.CARRY_DP  # the carry's targets are moved against the arm's measured give
    collar = p + np.array([M.BOTTLE_X / 2 - M.GRIP_IN, M.HELD_Y, -M.GRIP_UP_NEW - M.SAG])
    assert collar[0] - M.XS == pytest.approx(M.PUSH_PAST)  # aimed past the click
    assert abs(collar[1]) < 1e-9 and 0.0 < collar[2] - M.SEAT_Z < M.IN_Z
    p, _, _, _ = plan.target(ends["pump to the groove"] - 1e-6)
    collar = p - M.CARRY_DP + np.array([M.BOTTLE_X / 2 - M.GRIP_IN, M.HELD_Y, -M.GRIP_UP_NEW - M.SAG])
    assert collar[0] < M.XF - M.COLLAR_R  # in front of the groove before it goes in
    p, _, _, _ = plan.target(ends["over the recycling bin"] - 1e-6)
    middle = p + np.array([M.BOTTLE_X / 2 - M.GRIP_IN, M.HELD_Y, 0.0])
    assert np.allclose(middle[:2], M.BOX_AT[:2])  # the empty refill dropped over the box's middle
    lp, _, _, _ = ArmPlan(d["left"], station.BASES["left"]).target(ends["push the release latch"] - 1e-6)
    tip = lp + M._d * M.PAD_TIP
    assert tip[0] == pytest.approx(M.LATCH_FACE[0] + M.PRESS_PAST) and abs(tip[1]) < 1e-9


@pytest.mark.slow
def test_the_demo_touches_nothing_it_doesnt_hold():
    import plan_check

    out = plan_check.run(verbose=False)
    assert not out["hits"], out["hits"][:5]
    assert not out["carried_hits"], out["carried_hits"][:5]
    assert all(c[-1] < plan_check.PRESS_CONTACT_MM for c in out["press_contacts"]), out["press_contacts"]
    assert out["min_hand_gap_mm"] > 10.0
