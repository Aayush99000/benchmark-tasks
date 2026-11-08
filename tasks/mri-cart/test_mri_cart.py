"""Scoring and demo checks for mri-cart that run without the simulator: python -m pytest -q test_mri_cart.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import mri_cart as M

HERE = Path(__file__).resolve().parent


def state(name, xy, bottom, moving=False) -> ObjectState:
    h = M.HEIGHT[name]
    v = np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos=np.array([xy[0], xy[1], bottom + h / 2]), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                       lin_vel=v, ang_vel=np.zeros(3), low=np.array([0.0, 0.0, bottom]),
                       high=np.array([0.0, 0.0, bottom + h]))


def start(**over) -> dict:
    s = {n: state(n, xy, M.SHELF) for n, (xy, _) in M.LAYOUT.items()}
    s.update(over)
    return s


def binned(name, k=0):
    return state(name, (M.BIN_XY[0] - 0.04 + 0.04 * k, M.BIN_XY[1]), M.BIN_BOTTOM)


def cleared(**over) -> dict:
    s = start(**{n: binned(n, k) for k, n in enumerate(M.UNSAFE)})
    s.update(over)
    return s


def test_the_task_file_loads():
    task = load(HERE / "mri_cart.py")
    assert set(task.free_objects) == set(M.ITEMS)


def test_the_start_scores_0():
    assert M.score(start()) == 0.0 and not M.success(start())


def test_the_cleared_cart_succeeds_and_scores_0_9_until_it_has_held():
    assert M.success(cleared()) and M.score(cleared()) == 0.9


@pytest.mark.parametrize("n", [1, 2])
def test_each_unsafe_item_in_the_bin_earns_a_third(n):
    s = start(**{x: binned(x, k) for k, x in enumerate(M.UNSAFE[:n])})
    assert M.score(s) == pytest.approx(n / 3, abs=1e-6)


@pytest.mark.parametrize("safe", M.SAFE)
def test_taking_an_mr_safe_item_off_the_cart_scores_0(safe):
    assert M.score(cleared(**{safe: binned(safe, 3)})) == 0.0  # into the bin
    assert M.score(cleared(**{safe: state(safe, (0.20, 0.25), M.TOP)})) == 0.0  # onto the table


def test_dumping_everything_in_the_bin_scores_0():
    assert M.score(start(**{n: binned(n, k % 4) for k, n in enumerate(M.ITEMS)})) == 0.0


def test_an_unsafe_item_put_on_the_table_does_not_count():
    s = cleared(phone=state("phone", (0.20, -0.22), M.TOP))
    assert M.score(s) == pytest.approx(2 / 3, abs=1e-6)


def test_an_item_held_above_the_bin_does_not_count():
    s = cleared(scissors=state("scissors", M.BIN_XY, M.BIN_BOTTOM + 0.03))
    assert M.score(s) == pytest.approx(2 / 3, abs=1e-6)


def test_a_moving_item_does_not_count():
    s = cleared(chest_piece=state("chest_piece", M.BIN_XY, M.BIN_BOTTOM, moving=True))
    assert M.score(s) == pytest.approx(2 / 3, abs=1e-6) and not M.success(s)


def test_items_lying_on_each_other_in_the_bin_count():
    s = cleared(chest_piece=state("chest_piece", M.BIN_XY, M.BIN_BOTTOM + M.PHONE[2]))
    assert M.success(s)


def test_an_item_on_the_bin_rim_does_not_count():
    edge = (M.BIN_XY[0] + M.BIN_IN[0] / 2 + M.WALL_T / 2, M.BIN_XY[1])
    assert M.score(cleared(scissors=state("scissors", edge, M.BIN_BOTTOM + M.BIN_WALL))) == pytest.approx(2 / 3, abs=1e-6)


# -- the layout -----------------------------------------------------------------------------------------------------

def _footprint(name):
    (x, y), yaw = M.LAYOUT[name]
    if name in ("chest_piece", "bottle"):
        r = M.CHEST_R if name == "chest_piece" else M.BOTTLE_R
        return x - r, x + r, y - r, y + r
    sx, sy = {"scissors": M.SCISSORS, "phone": M.PHONE, "pad_left": M.PAD, "pad_right": M.PAD}[name][:2]
    if round(yaw) % 180 == 90:
        sx, sy = sy, sx
    return x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2


def test_every_item_starts_on_the_cart_clear_of_the_rim_and_the_others():
    (cx, cy), (ix, iy) = M.CART_XY, M.CART_IN
    for n in M.ITEMS:
        x0, x1, y0, y1 = _footprint(n)
        assert cx - ix / 2 + 0.01 <= x0 and x1 <= cx + ix / 2 - 0.01 and cy - iy / 2 + 0.01 <= y0 and y1 <= cy + iy / 2 - 0.01, n
        for m in M.ITEMS:
            if m != n:
                a, b = _footprint(n), _footprint(m)
                apart = a[1] < b[0] or b[1] < a[0] or a[3] < b[2] or b[3] < a[2]
                assert apart, (n, m)


def test_the_scissors_are_tucked_1_7_cm_from_each_pad():
    _, _, sy0, sy1 = _footprint("scissors")  # pads 5 cm wide
    assert _footprint("pad_left")[2] - sy1 == pytest.approx(0.017)
    assert sy0 - _footprint("pad_right")[3] == pytest.approx(0.017)


def test_the_fingers_fit_between_the_pads_on_the_way_down():
    # Coming down 54 mm apart over the 40 mm scissors: 7 mm clear of the scissors, and a finger (at most about 8 mm
    # thick near its tip) still 2 mm clear of each pad, 17 mm from the scissors.
    half = M.APPROACH["scissors"] / 2
    assert half - M.SCISSORS[1] / 2 >= 0.0069
    assert M.SCISSORS[1] / 2 + M.GAP - (half + 0.008) >= 0.0019


def test_the_fingers_stay_clear_of_the_rim_and_the_other_items():
    # Each finger, open on the way down, at the item's centre +- half the approach gap along the grip's closing axis
    # (y at heading 0, x at 90), about 8 mm thick and 2 cm wide: it must not land on the rim or on another item.
    (cx, cy), (ix, iy) = M.CART_XY, M.CART_IN
    for n in M.UNSAFE:
        (x, y), yaw = M.LAYOUT[n]
        along_x = round(yaw) % 180 == 90
        for s in (-1, 1):
            c = s * (M.APPROACH[n] / 2 + 0.004)
            fx, fy = (x + c, y) if along_x else (x, y + c)
            hx, hy = (0.004, 0.01) if along_x else (0.01, 0.004)
            finger = (fx - hx, fx + hx, fy - hy, fy + hy)
            assert cx - ix / 2 + 0.002 <= finger[0] and finger[1] <= cx + ix / 2 - 0.002, n
            assert cy - iy / 2 + 0.002 <= finger[2] and finger[3] <= cy + iy / 2 - 0.002, n
            for m in M.ITEMS:
                if m != n:
                    b = _footprint(m)
                    assert finger[1] < b[0] or b[1] < finger[0] or finger[3] < b[2] or b[3] < finger[2], (n, m)


def test_the_grips_squeeze_and_the_approaches_clear():
    width = {"scissors": M.SCISSORS[1], "phone": M.PHONE[1], "chest_piece": 2 * M.CHEST_R}
    for n, w in width.items():
        assert M.GRIP[n] <= w - 0.006 and M.APPROACH[n] >= w + 0.012 and M.APPROACH[n] <= 0.094, n


def test_the_demo_drops_inside_the_bin_without_piling_up():
    (bx, by), (ix, iy) = M.BIN_XY, M.BIN_IN
    boxes = {}
    for n, (x, y) in M.DROP_XY.items():
        if n == "chest_piece":
            hx = hy = M.CHEST_R
        else:
            sx, sy = (M.SCISSORS if n == "scissors" else M.PHONE)[:2]
            hx, hy = (sx / 2, sy / 2) if round(M.DROP_YAW[n]) % 180 == 0 else (sy / 2, sx / 2)
        assert abs(x - bx) + hx <= ix / 2 - 0.005 and abs(y - by) + hy <= iy / 2 - 0.005, n
        boxes[n] = (x - hx, x + hx, y - hy, y + hy)
    for a in boxes:
        for b in boxes:
            if a < b:
                p, q = boxes[a], boxes[b]
                assert p[1] < q[0] or q[1] < p[0] or p[3] < q[2] or q[3] < p[2], (a, b)
    assert M.DROP_Z - M.BIN_BOTTOM < M.BIN_WALL  # let go below the bin's rim


def test_the_carry_clears_the_bin_walls_and_passes_beside_the_bottle():
    clear = M.TRAVEL_Z - M.TCP_PAST - M.PAD_ABOVE  # the bottom of a carried item, gripped 3 mm above its bottom
    assert clear - (M.BIN_BOTTOM + M.BIN_WALL) >= 0.015
    bottle = np.array(M.LAYOUT["bottle"][0])
    for n in M.UNSAFE:
        a, b = np.array(M.LAYOUT[n][0]), np.array(M.DROP_XY[n])
        t = np.clip((bottle - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0)
        # Each item is carried with its long side along its path, so it sweeps its width (the chest piece its
        # diameter) to either side of the line.
        half = min((M.SCISSORS if n == "scissors" else M.PHONE if n == "phone" else (2 * M.CHEST_R,) * 2)[:2]) / 2
        assert np.linalg.norm(a + t * (b - a) - bottle) - M.BOTTLE_R - half >= 0.01, n  # 1 cm clear


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_never_touches_an_mr_safe_item():
    taken = {w.label for ws in M.demo().values() for w in ws}
    assert "close" in taken
    for arm, ws in M.demo().items():
        grips = [w for w in ws if w.label == "close"]
        for w in grips:
            nearest = min(M.ITEMS, key=lambda n: math.dist(M.LAYOUT[n][0], w.pos[:2]))
            assert nearest in M.UNSAFE


def test_the_demo_finishes_well_within_the_run_limit():
    assert max(sum(w.duration for w in ws) for ws in M.demo().values()) < 2400 / 20 / 2


def test_each_arm_leaves_the_bin_clear_when_it_is_done():
    (bx, by), (ix, iy) = M.BIN_XY, M.BIN_IN
    for arm, ws in M.demo().items():
        x, y, _ = ws[-1].pos
        assert abs(y - by) >= iy / 2 + 0.10, arm  # 10 cm beyond the bin's side


def test_the_right_arm_starts_only_once_the_left_has_left():
    plan = M.demo()
    assert plan["right"][0].label == "wait"
    assert plan["right"][0].duration == pytest.approx(sum(w.duration for w in plan["left"]))


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0
