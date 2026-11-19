"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest pinch-zoom-map-on-tablet-to-fit-route    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("pinch_zoom_map_on_tablet_to_fit_route.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 800


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


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
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
    'tablet': {'pos': (0.2, -0.06, 0.7475)},
}
GOAL_POSES = {
    'tablet': {'pos': (0.2, -0.06, 0.7475)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_VIEW': '(0.2778, (20.8, 0.0))'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'tablet'}
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

# (name, the layout it starts from, what's moved, the score it should get, module overrides)
CASES = [
    ('untouched start (route 2.8 screen widths, only its middle on screen)', START_POSES, {
    }, 0, {}),
    ("route fits and fills, but the cafe's place card is open", START_POSES, {
    }, 0.75, {'TEST_VIEW': '(0.2778, (20.8, 0.0))', 'TEST_CARD': "'cafe'"}),
    ('route fits and fills, but the tablet was pushed 30 mm across the desk', START_POSES, {
        'tablet': {'pos': (0.2, -0.09, 0.7475)},
    }, 0.75, {'TEST_VIEW': '(0.2778, (20.8, 0.0))'}),
    ('route fits and fills, the tablet turned 8 degrees on the desk', START_POSES, {
        'tablet': {'pos': (0.2, -0.06, 0.7475), 'yaw_deg': 8},
    }, 0.75, {'TEST_VIEW': '(0.2778, (20.8, 0.0))'}),
    ('zoomed out too far (x0.15, route only 90 mm wide)', START_POSES, {
    }, 0.5, {'TEST_VIEW': '(0.15, (20.0, 0.0))', 'TEST_PINCHED': 'True'}),
    ("zoomed out 3x but the route's right end runs off the screen", START_POSES, {
    }, 0.5, {'TEST_VIEW': '(0.33, (-60.0, 0.0))', 'TEST_PINCHED': 'True'}),
    ('one small pinch only (x0.7), route still off the screen', START_POSES, {
    }, 0.25, {'TEST_VIEW': '(0.7, (0.0, 0.0))', 'TEST_PINCHED': 'True'}),
    ('route fits and fills with the narrowest allowed margins (x0.19, centred)', START_POSES, {
    }, 1, {'TEST_VIEW': '(0.19, (20.0, 0.0))'}),
    ('route fits but the fingertips are still on the glass', START_POSES, {
    }, 0, {'TEST_VIEW': '(0.2778, (20.8, 0.0))', 'TEST_NIBS': 'nibs_at(-8.0, 0.0, 0.0)'}),  # no credit while the hand is still on the screen
    ('route fits but the fingertips hover 5 mm over the glass', START_POSES, {
    }, 0, {'TEST_VIEW': '(0.2778, (20.8, 0.0))', 'TEST_NIBS': 'nibs_at(-8.0, 0.0, 5.0)'}),  # hovering is not hands off
    ('route fits, but a touch swiped in from the screen edge earlier (sticky)', START_POSES, {
    }, 0, {'TEST_VIEW': '(0.2778, (20.8, 0.0))', 'TEST_STICKY': "('edge swipe',)"}),  # an edge swipe can leave the map app
    ('tablet turned over', START_POSES, {
        'tablet': {'pos': (0.2, -0.06, 0.7561), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 0, {'TEST_VIEW': '(0.2778, (20.8, 0.0))'}),  # the tablet must stay screen up on the desk
    ('tablet pushed off the desk', START_POSES, {
        'tablet': {'pos': (0.2, -0.06, 0.6475)},
    }, 0, {'TEST_VIEW': '(0.2778, (20.8, 0.0))'}),  # dropped tablet
    ('zoomed in 2x by a double tap instead of out', START_POSES, {
    }, 0, {'TEST_VIEW': '(2.0, (0.0, 0.0))'}),  # wrong direction, no pinch
    ('route fits but 1 mm too close to the right edge (x0.346, shifted)', START_POSES, {
    }, 0.5, {'TEST_VIEW': '(0.346, (17.0, 0.0))'}),  # the 5 mm margin is part of fitting
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
