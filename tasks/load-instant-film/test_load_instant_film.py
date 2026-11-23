"""Scoring and demo checks for load-instant-film that run without the simulator: python -m pytest -q"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import load_instant_film as M
from mechanism import MechanismState

HERE = Path(__file__).resolve().parent
Z3 = np.zeros(3)


def pads(x, y, z) -> M.Pads:
    return M.Pads(back=np.array([x, y, z + 0.04]), tips=np.array([x, y, z]))


AWAY = {"left_gripper": pads(*M.PARK_LEFT), "right_gripper": pads(*M.PARK_RIGHT)}


def quat_of(r: np.ndarray) -> np.ndarray:
    """A rotation matrix's quaternion (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    q = np.zeros(4)
    q[i] = math.sqrt(max(0.0, 1 + r[i, i] - r[j, j] - r[k, k])) / 2
    q[j] = (r[j, i] + r[i, j]) / (4 * q[i])
    q[k] = (r[k, i] + r[i, k]) / (4 * q[i])
    q[3] = (r[k, j] - r[j, k]) / (4 * q[i])
    return q


LOADED = np.column_stack([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]).astype(float)  # length up, width to -x... back out
SEATED_POS = np.array([M.CH_X, M.CH_WALL - M.PACK[2] / 2, M.TOP + M.CH_Z0 + M.PACK[0] / 2])


def pack(pos=SEATED_POS, r=LOADED, still=True) -> ObjectState:
    v = Z3 if still else np.array([0.05, 0, 0])
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat_of(r), lin_vel=v, ang_vel=Z3, low=p - 0.05, high=p + 0.05)


def camera(cover=0.0, lock=0.0, ring=0.0, shutter=0.0, sheet=0.0) -> MechanismState:
    return MechanismState(pos=np.array([0, 0, M.TOP]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3, ang_vel=Z3,
                          low=Z3, high=Z3, joints={"back_cover": cover, "lock": lock, "lens_ring": ring,
                                                   "shutter": shutter, "film_cover": sheet})


def objs(p=None, cam=None, rec=None, hands=None):
    return {"film_pack": p or pack(), "camera": cam or camera(), "film_record": rec or M.FilmRecord(),
            **(hands or AWAY)}


ON = -math.radians(40)


def test_task_file_loads():
    load(HERE / "load_instant_film.py")


def test_loaded_frame_is_a_rotation_and_right_way():
    assert np.allclose(LOADED @ LOADED.T, np.eye(3)) and np.isclose(np.linalg.det(LOADED), 1.0)
    assert M.right_way(pack()) and M.seated(pack())


def test_carry_brings_yellow_end_up():
    r = M._carry_turn() @ np.array([[math.cos(M.PACK_YAW), -math.sin(M.PACK_YAW), 0],
                                    [math.sin(M.PACK_YAW), math.cos(M.PACK_YAW), 0], [0, 0, 1]])
    assert r[2, 0] > 0.99 and -r[1, 2] > 0.99


def test_scores_in_order():
    done = M.FilmRecord(locked=True, ejected=True, presses=1)
    assert M.grade(objs(cam=camera(ring=ON, sheet=M.EJECT), rec=done)) == 1.0
    assert M.score(objs(cam=camera(ring=ON, sheet=M.EJECT), rec=done)) == M.COMPLETE_SCORE
    assert M.grade(objs(cam=camera(ring=ON))) == 0.75
    assert M.grade(objs(cam=camera())) == 0.5
    assert M.grade(objs(cam=camera(cover=math.radians(40)), rec=M.FilmRecord(locked=False))) == 0.25
    assert M.grade(objs(p=pack(pos=M.PACK_START, r=np.eye(3)))) == 0.0


def test_hand_near_camera_holds_full_score_back():
    done = M.FilmRecord(locked=True, ejected=True, presses=1)
    near = {**AWAY, "left_gripper": pads(M.CX, M.YF + 0.03, M.TOP + 0.09)}
    assert M.grade(objs(cam=camera(ring=ON, sheet=M.EJECT), rec=done, hands=near)) == 0.75


def test_wrong_way_round_scores_zero():
    upside_down = np.column_stack([[0, 0, -1], [1, 0, 0], [0, -1, 0]]).astype(float)  # yellow end down
    back_in = np.column_stack([[0, 0, 1], [1, 0, 0], [0, 1, 0]]).astype(float)  # holes towards the lens
    for r in (upside_down, back_in):
        p = pack(r=r)
        assert M.in_chamber(p) and not M.right_way(p)
        assert M.grade(objs(p=p, cam=camera(ring=ON))) == 0.0


def test_not_pushed_home_scores_zero():
    proud = SEATED_POS + np.array([0, -0.004, 0])
    assert M.grade(objs(p=pack(pos=proud))) == 0.0


def test_holes_and_forced_lock_score_zero():
    for rec in (M.FilmRecord(holes_pressed=True), M.FilmRecord(forced=True)):
        assert M.grade(objs(cam=camera(ring=ON), rec=rec)) == 0.0


def test_fingertip_on_a_hole_is_seen():
    p = pack()
    (hx, hy), _ = M.HOLES[0]
    local = np.array([hx, hy, M.PACK[2] / 2 + 0.001])
    q = LOADED @ local + SEATED_POS
    on_hole = M.Pads(back=q, tips=q, fingertips=(q,))
    margin = LOADED @ np.array([0.030, 0, M.PACK[2] / 2 + 0.001]) + SEATED_POS
    on_margin = M.Pads(back=margin, tips=margin, fingertips=(margin,))
    assert M.presses_holes(on_hole, p) and not M.presses_holes(on_margin, p)


def test_lock_logic():
    rec = {"locked": True, "popping": False, "forced": False}
    assert M.lock_step(rec, 0.0, 0.0) == 0.0 and rec["locked"]
    assert M.lock_step(rec, math.radians(8), 0.0) == 0.0 and rec["forced"]
    rec = {"locked": True, "popping": False, "forced": False}
    assert M.lock_step(rec, 0.0, -0.0025) == pytest.approx(math.radians(M.POP_DEG)) and not rec["locked"]
    popped = math.radians(M.POP_DEG - 1)
    assert M.lock_step(rec, popped, 0.0) == pytest.approx(popped)  # popped: now free
    assert M.lock_step(rec, math.radians(60), 0.0) == pytest.approx(math.radians(60))
    assert M.lock_step(rec, math.radians(1.5), -0.0025) == pytest.approx(math.radians(1.5))  # tab held down: no lock
    assert M.lock_step(rec, math.radians(1.5), 0.0) == 0.0 and rec["locked"]


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reach(arm):
    plan = ArmPlan(M.demo()[arm], LEFT_BASE_POS if arm == "left" else RIGHT_BASE_POS)
    worst_mm, worst_deg, worst_at = 0.0, 0.0, ""
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
        if plan.worst_ik_mm > worst_mm or plan.worst_ik_deg > worst_deg:
            worst_mm, worst_deg = plan.worst_ik_mm, plan.worst_ik_deg
            worst_at = plan.target(t)[3]
    print(f"{arm}: worst IK {worst_mm:.1f} mm, {worst_deg:.1f} deg at {worst_at!r}")
    assert worst_mm < 3.0 and worst_deg < 3.0, (worst_mm, worst_deg, worst_at)
