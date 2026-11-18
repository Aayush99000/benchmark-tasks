"""Conservative 20 mm dispenser clearance from measured collision geometry.

Robot and tool CollisionBoxes have world centres. Rigid ObjectState bounds are
in task coordinates (Arena's object_states already subtracts env_origin).
This observer subtracts the origin from box centres exactly once. It never
reads TCPs, changes simulation state, or requires releasing/returning the tool.
A tool held away from the dispenser is a valid final condition.
"""
from __future__ import annotations

from dataclasses import dataclass
from numbers import Integral
import numpy as np

from surface_access_geometry import rotation_matrix_xyzw

CLEARANCE_M = .020
REQUIRED_RIGIDS = ('stub', 'roll', 'cover', 'shutter', 'latch_bolt')
ARMS = ('left_robot', 'right_robot')
_SIGNS = np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)], dtype=float)


@dataclass(frozen=True)
class ClearanceReport:
    valid: bool
    clear: bool
    blocked_pairs: tuple[tuple[str, str], ...] = ()
    robot_piece_count: int = 0
    tool_piece_count: int = 0
    error: str | None = None


def validate_robot_coverage(body_names_by_arm, pieces_per_link):
    """Return all required piece names, rejecting missing/duplicate links.

    Pass actual articulation.body_names for each arm and the piece counts from
    surface_paper._robot_bounds(). This validates measurement coverage before
    those same live poses build the snapshot's CollisionBoxes. The clearance
    evaluator independently checks every resulting piece name as well.
    """
    if not pieces_per_link:
        raise ValueError('No declared robot collision geometry')
    counts = {}
    for name, count in pieces_per_link.items():
        if not isinstance(name, str) or not name or not isinstance(count, Integral) or count < 0:
            raise ValueError('Invalid declared robot collision piece count')
        if count:
            counts[name] = int(count)
    if not counts:
        raise ValueError('No required robot collision links')
    expected = set()
    for arm in ARMS:
        if arm not in body_names_by_arm:
            raise ValueError(f'Missing measured robot {arm}')
        names = tuple(body_names_by_arm[arm])
        if any(not isinstance(name, str) for name in names) or len(set(names)) != len(names):
            raise ValueError(f'Duplicate/invalid measured links for {arm}')
        missing = set(counts) - set(names)
        if missing:
            raise ValueError(f'Missing measured collision links for {arm}: {sorted(missing)}')
        expected.update(f'{arm}/{name}/{i}' for name, count in counts.items() for i in range(count))
    return frozenset(expected)


def _expected_names(names, label):
    names = tuple(names)
    if not names or any(not isinstance(name, str) or not name for name in names):
        raise ValueError(f'Missing/invalid expected {label} pieces')
    if len(set(names)) != len(names):
        raise ValueError(f'Duplicate expected {label} piece')
    return frozenset(names)


def _parse_boxes(boxes, expected, origin, label):
    boxes = tuple(boxes)
    names = tuple(box.name for box in boxes)
    if len(set(names)) != len(names) or set(names) != expected:
        raise ValueError(f'Incomplete/duplicate {label} collision pieces')
    centers, halves, rotations = [], [], []
    for box in boxes:
        center = np.asarray(box.center, dtype=float)
        half = np.asarray(box.half_extents, dtype=float)
        quat = np.asarray(box.quat_xyzw, dtype=float)
        if (center.shape != (3,) or half.shape != (3,) or quat.shape != (4,)
                or not all(np.isfinite(v).all() for v in (center, half, quat))
                or np.any(half <= 0) or abs(np.linalg.norm(quat)-1.) > 1e-3):
            raise ValueError(f'Invalid measured {label} collision geometry: {box.name}')
        centers.append(center-origin)
        halves.append(half)
        rotations.append(rotation_matrix_xyzw(quat))
    return names, np.asarray(centers), np.asarray(halves), np.asarray(rotations)


def evaluate_release_clearance(*, robot_boxes, tool_boxes, rigid_states, env_origin,
                               expected_robot_names, expected_tool_names):
    """Check all measured robot/tool pieces against task-local rigid bounds.

    Project all eight corners of every rigid state's AABB into each measured
    collision piece's local frame. Its local box is expanded by the existing
    20 mm on every axis. This deliberately conservative test can reject a near
    pass; a successful separation cannot hide contact with the rigid material
    inside its AABB. Exactly 20 mm remains blocked, matching the old inclusive
    TCP-boundary rule. Paper-triangle clearance stays a separate prerequisite.

    The six actual cover_tool boxes are included as extensions of the robot
    for dispenser clearance, even if the tool is no longer grasped. They are
    not checked against the holding fingers, and no rack/hand-opening condition
    is imposed. Missing or nonfinite observations return valid=False,clear=False.
    """
    try:
        origin = np.asarray(env_origin, dtype=float)
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('Invalid measured environment origin')
        required_robot = _expected_names(expected_robot_names, 'robot')
        required_tool = _expected_names(expected_tool_names, 'tool')
        if len(required_tool) != 6:
            raise ValueError('The declared cover_tool must contain all six collision pieces')
        robot = _parse_boxes(robot_boxes, required_robot, origin, 'robot')
        tool = _parse_boxes(tool_boxes, required_tool, origin, 'tool')
        if set(robot[0]) & set(tool[0]):
            raise ValueError('Robot and tool piece identities overlap')
        missing = set(REQUIRED_RIGIDS) - set(rigid_states)
        if missing:
            raise ValueError(f'Missing required rigid states: {sorted(missing)}')
        corners = []
        for name in REQUIRED_RIGIDS:
            state = rigid_states[name]
            low, high = np.asarray(state.low, float), np.asarray(state.high, float)
            if (low.shape != (3,) or high.shape != (3,) or not np.isfinite(low).all()
                    or not np.isfinite(high).all() or np.any(high <= low)):
                raise ValueError(f'Invalid measured rigid bounds: {name}')
            corners.append((low+high)/2 + _SIGNS*(high-low)/2)
        names = robot[0] + tool[0]
        centers = np.concatenate((robot[1], tool[1]))
        halves = np.concatenate((robot[2], tool[2]))
        rotations = np.concatenate((robot[3], tool[3]))
        relative = np.asarray(corners)[None, :, :, :] - centers[:, None, None, :]
        local = np.einsum('btvi,bij->btvj', relative, rotations)
        low, high = local.min(axis=2), local.max(axis=2)
        expanded = halves[:, None, :] + CLEARANCE_M
        # The guard only expands rejection at floating-point boundaries.
        guard = 64*np.finfo(float).eps*(1+np.abs(relative).max()+halves.max())
        overlap = np.all((high >= -expanded-guard) & (low <= expanded+guard), axis=2)
        pairs = tuple((names[i], REQUIRED_RIGIDS[j]) for i, j in np.argwhere(overlap))
        return ClearanceReport(True, not pairs, pairs, len(robot[0]), len(tool[0]))
    except (ValueError, TypeError, AttributeError, KeyError, IndexError) as error:
        return ClearanceReport(False, False, error=type(error).__name__+': '+str(error))
