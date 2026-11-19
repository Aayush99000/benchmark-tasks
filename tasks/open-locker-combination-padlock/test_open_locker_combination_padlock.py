"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest open-locker-combination-padlock    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.modules.pop("yam_parts", None)
import yam_parts
sys.modules.pop("yam_joints", None)
import yam_joints
sys.modules.pop("yam_mesh", None)
import yam_mesh
from dataclasses import replace

TASK = load(Path(__file__).with_name("open_locker_combination_padlock.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 5000


def _rot(yaw_deg, tilt_deg, axis):
    y, t = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    c, s = math.cos(t), math.sin(t)
    rt = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) if axis == "x" else np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return rz @ rt


def _half(o, r):
    if hasattr(o, "size"):
        return np.abs(r) @ (np.asarray(o.size) / 2)
    if hasattr(o, "height"):
        a = r[:, 2]
        return o.height / 2 * np.abs(a) + o.radius * np.sqrt(np.clip(1 - a * a, 0, 1))
    return np.full(3, o.radius)


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None, joints=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
    if isinstance(OBJ[name], yam_joints.Mechanism):
        q = {link.name: float((joints or {}).get(link.name, getattr(link.joint, "initial", 0.0))) *
             (math.pi / 180 if isinstance(link.joint, yam_joints.Hinge) else 1.0)
             for link in OBJ[name].links}
        state = OBJ[name].offline_state(pos, math.radians(yaw_deg), q, R=r)
        return replace(state, lin_vel=np.asarray(vel, float))
    if isinstance(OBJ[name], yam_parts.Part):
        origin = np.asarray(pos, float)
        low, high = OBJ[name].aabb(origin, r)
        return yam_parts.PartState((low + high) / 2, np.asarray(ik.matrix_to_quat_xyzw(r), float),
                                   np.asarray(vel, float), np.zeros(3), low, high, origin)
    pos, half = np.asarray(pos, float), _half(OBJ[name], r)
    q = ik.matrix_to_quat_xyzw(r)
    return ObjectState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def layout(poses):
    """Every object's state: the loose ones as given, the fixed ones where the task file puts them."""
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 0.0, 'wheel2': -21.50969293753206, 'wheel1': -43.01938587506412, 'shackle': 0.0, 'bolt': 0.0}},
}
GOAL_POSES = {
    'padlock': {'pos': (0.2395, -0.06, 0.8805), 'joints': {'dial': 441.0, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.012, 'bolt': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'padlock'}
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)


def test_success_waits_for_the_objects_to_settle(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    moving = layout({n: {**p, "vel": (0.05, 0.0, 0.0)} for n, p in GOAL_POSES.items()})
    assert not T.success(moving)

# (name, the layout it starts from, what's moved, the score it should get, module overrides)
CASES = [
    ('untouched start', START_POSES, {
    }, 0, {}),
    ('first number set (right past two turns to 24)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 864.0, 'wheel2': 884.6, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.25, {}),  # Wheel 1's gate sits under the fence; wheel 2 and the cam do not.
    ('first and second numbers set (left one turn past 24 to 8)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 288.0, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.5, {}),
    ('all three numbers set, not pulled', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 441.0, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.75, {}),
    ('second number two marks off, third set', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 441.0, 'wheel2': 645.4, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.25, {}),  # Wheel 2's gate is 18 degrees from the fence, far outside half a mark.
    ('each gate just inside half a mark', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 445.4, 'wheel2': 623.0, 'wheel1': 909.6, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.75, {}),
    ('each gate off by half the tolerance', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 438.75, 'wheel2': 629.65, 'wheel1': 902.95, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.75, {}),
    ('exact combination, opened, released, at rest', GOAL_POSES, {
    }, 1.0, {}),
    ('first number one mark off (25 instead of 24)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 441.0, 'wheel2': 627.4, 'wheel1': 914.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0, {}),  # Wheel 1's gate 9 degrees from the fence: not set, so nothing scores.
    ('first number one mark off the other way (23)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 441.0, 'wheel2': 627.4, 'wheel1': 896.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0, {}),
    ('second number one mark off (9 instead of 8)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 441.0, 'wheel2': 618.4, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.25, {}),
    ('second number one mark off the other way (7)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 441.0, 'wheel2': 636.4, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.25, {}),
    ('third number one mark off (30 instead of 31)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 432.0, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.5, {}),
    ('third number one mark off the other way (32)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 450.0, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0.5, {}),
    ('shackle open with the third number a mark and a half off (forced)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8805), 'joints': {'dial': 454.5, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.012, 'bolt': 0.0}},
    }, 0, {}),  # The fence cannot enter a gate this far off, so an open shackle there means the lock was forced.
    ('opened but the dial still pinched', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': "{'left': (np.array([0.10, 0.25, TOP + 0.20]), 1.0), 'right': (np.array(TCP_DIAL) - [0, 0, SHACKLE_TRAVEL], 0.35)}"}),
    ('opened, released, but the lock still swinging', GOAL_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8805), 'vel': (0.0, 0.03, 0.0), 'joints': {'dial': 441.0, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.012, 'bolt': 0.0}},
    }, 0.75, {}),
    ('dial spun right only (first gate set by the right turns, the rest scrambled)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 1000.0000000000001, 'wheel2': 1020.6, 'wheel1': 1041.2, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0, {}),
    ('opened, then the lock knocked off the hasp', GOAL_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8325), 'joints': {'dial': 441.0, 'wheel2': 627.4, 'wheel1': 905.2, 'shackle': 0.012, 'bolt': 0.0}},
    }, 0, {'TEST_STICKY': "('lock off the hasp',)"}),
    ('lock off the hasp earlier, put back open', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('lock off the hasp',)"}),  # Violate-then-restore; a lock that fell stays at zero.
    ('shackle open with the wheel pack scrambled (forced)', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8805), 'joints': {'dial': 0.0, 'wheel2': -21.50969293753206, 'wheel1': -43.01938587506412, 'shackle': 0.012, 'bolt': 0.0}},
    }, 0, {}),  # The bolt cannot move unless all three gates are under the fence; an open shackle without them means the lock was forced.
    ('dialled straight right to 24 without the full turns', START_POSES, {
        'padlock': {'pos': (0.2395, -0.06, 0.8925), 'joints': {'dial': 144.0, 'wheel2': -21.50969293753206, 'wheel1': -43.01938587506412, 'shackle': 0.0, 'bolt': 0.0}},
    }, 0, {}),  # From the reset (pack cleared to the left) the cam has not yet picked up wheel 2, so wheel 1 is not set.
]


@pytest.mark.parametrize(("name", "base", "moved", "expected", "module"), CASES, ids=[c[0] for c in CASES])
def test_named_layouts(name, base, moved, expected, module, monkeypatch):
    module_values = {}
    for name, expr in module.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    objects = layout({**base, **moved})
    assert T.score(objects) == pytest.approx(expected), name
    assert bool(T.success(objects)) == (expected == 1.0), name


def test_the_demo_is_reachable_within_the_joint_limits():
    plan = TASK.demo()
    assert set(plan) <= {"left", "right"}
    for side, waypoints in plan.items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = round(arm.duration * 20) + 5
        actions = np.array([arm.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(actions[:, :6] >= ik.LOWER - 1e-6) and np.all(actions[:, :6] <= ik.UPPER + 1e-6), side


def test_the_demo_fits_in_the_run_limit():
    longest = max(ArmPlan(w, station.BASES[s]).duration for s, w in TASK.demo().items())
    assert (longest + TASK.success_hold_s) * 20 < RUN_LIMIT_STEPS * 0.6


# ------------------------------------------------------------------ the gates' physical clearance
def _rot2(poly, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return [(y * c - z * s, y * s + z * c) for y, z in poly]


def _overlap(p, q):
    """Separating-axis test for two convex polygons (strict overlap; touching counts as clear)."""
    for poly in (p, q):
        for i in range(len(poly)):
            (y0, z0), (y1, z1) = poly[i], poly[(i + 1) % len(poly)]
            n = (z1 - z0, y0 - y1)
            a = [n[0] * y + n[1] * z for y, z in p]
            b = [n[0] * y + n[1] * z for y, z in q]
            if max(a) <= min(b) + 1e-9 or max(b) <= min(a) + 1e-9:
                return False
    return True


def _fence_reaches_release(link, offset_deg):
    """Can the fence go RELEASE_DEPTH deep (inner face at 10.2 mm radius) with this link's gate offset_deg off it?"""
    r0 = T.FENCE_R_RELEASE
    fence = [(r0, -T.FENCE_HALF_T), (15.5, -T.FENCE_HALF_T), (15.5, T.FENCE_HALF_T), (r0, T.FENCE_HALF_T)]
    q = offset_deg - T.GATE_PSI[link]            # the link angle that puts its gate offset_deg from the fence
    return not any(_overlap(fence, _rot2(prof, q)) for prof in T.rim_profiles(link))


@pytest.mark.parametrize("link", ["wheel1", "wheel2", "dial"])
def test_the_fence_enters_only_within_the_tolerance(link):
    for off in (0.0, T.TOL_DEG / 2, -T.TOL_DEG / 2, T.OPEN_WINDOW_DEG - 0.05, -(T.OPEN_WINDOW_DEG - 0.05)):
        assert _fence_reaches_release(link, off), (link, off)
    for off in (T.TOL_DEG, -T.TOL_DEG, T.MARK_DEG, -T.MARK_DEG, 2 * T.MARK_DEG, 90.0):
        assert not _fence_reaches_release(link, off), (link, off)
    assert T.OPEN_WINDOW_DEG <= T.TOL_DEG <= T.MARK_DEG / 2


def test_the_idle_fence_clears_every_rim():
    fence = [(T.FENCE_R_IDLE, -T.FENCE_HALF_T), (15.5, -T.FENCE_HALF_T), (15.5, T.FENCE_HALF_T),
             (T.FENCE_R_IDLE, T.FENCE_HALF_T)]
    for link in ("wheel1", "wheel2", "dial"):
        for q in range(0, 360, 15):
            assert not any(_overlap(fence, _rot2(prof, q)) for prof in T.rim_profiles(link))


def test_the_carried_wheels_land_on_their_gates_when_the_dial_stops_on_the_number():
    """Dialled by the rule with the pins touching (pick-up angle A_PIN), each gate lands exactly under the fence, and
    a dial stop a whole mark away moves the gate a whole mark, outside the tolerance."""
    a = T.A_PIN
    w1 = -T.MARK_DEG * 24 + 2 * a          # right to 24, both pin pairs pushing
    w2 = -T.MARK_DEG * 8 - a               # left to 8, the cam's pin pushing
    cam = -T.MARK_DEG * 31
    for link, ang in (("wheel1", w1), ("wheel2", w2), ("dial", cam)):
        assert abs(T._wrap(T.GATE_PSI[link] + ang)) < 1e-9
        assert abs(T._wrap(T.GATE_PSI[link] + ang + T.MARK_DEG)) > T.TOL_DEG


# ------------------------------------------------------------------ the printed dial face
def _turn(yz, deg):
    a = math.radians(deg)
    y, z = yz
    return (y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a))


@pytest.mark.parametrize(("number", "dial_deg"), [(24, 864.0), (8, 288.0), (31, 441.0), (0, 0.0)])
def test_the_printed_mark_under_the_index_is_the_dialled_number(number, dial_deg):
    """At each stop of the maker's rule the printed tick of that number (and no other) is straight up, under the
    index, and number_at_index() reads the same number."""
    assert T.number_at_index(dial_deg) == pytest.approx(number)
    y, z = _turn(T.DIAL_MARKS[number]["tick"], dial_deg)
    assert abs(y) < 1e-6 and z > 0
    for other in (number - 1, number + 1):
        y2, z2 = _turn(T.DIAL_MARKS[other % 40]["tick"], dial_deg)
        assert abs(math.degrees(math.atan2(-y2, z2))) == pytest.approx(T.MARK_DEG, abs=1e-6)


def test_the_numbers_rise_clockwise_seen_from_the_robot():
    """Seen from the robot (screen right = -y), 10 is at three o'clock, 20 at the bottom and 30 at nine o'clock,
    and turning right (+ about x) brings lower numbers to the index."""
    y, z = T.DIAL_MARKS[10]["label"]
    assert -y > 10 and abs(z) < 1e-6
    y, z = T.DIAL_MARKS[20]["label"]
    assert z < -10
    y, z = T.DIAL_MARKS[30]["label"]
    assert y > 10
    assert T.number_at_index(T.MARK_DEG) == pytest.approx(39)


def test_the_print_sits_on_the_disc_clear_of_the_knob_and_the_pads():
    pts = np.asarray(T.PRINT_FACE.points) / T.MM
    r = np.hypot(pts[:, 1], pts[:, 2])
    assert r.max() < T.DISC_R and r.min() > T.KNOB_R
    assert pts[:, 0].min() >= T.FACE_X - T.PRINT_T - 1e-9          # 0.15 mm proud at most
    pad_tip_x = (T.TCP_DIAL[0] + T.PAD_TIP - T.LOCK_X) / T.MM
    assert pad_tip_x < T.FACE_X - 1.0                                # the pinching pads stay over 1 mm off the print
