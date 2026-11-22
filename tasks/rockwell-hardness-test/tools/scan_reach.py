"""Offline reach scan for rockwell-hardness-test (no Isaac): where can the tester stand so that
(1) the left arm can pinch the elevating handwheel's rim from the front and sweep it through a stroke, and
(2) the right arm can push the main-load handle on the tester's right side back and pull it forward?

    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 tools/scan_reach.py

Poses only (yam_arena's NumPy IK on the URDF, joint limits clipped); no collisions are checked.

Found 2026-11-22: a horizontal rim pinch pointing at the wheel's axis is out of reach anywhere (30-120 mm IK error),
and a down grip only reaches below about 11 cm above the table. A grip pointing straight forward (+x), fingers closing
along y, on a free-spinning knob reaches the front arc (-60 to +60 deg) at 15-21 cm, wheel axis x 0.30-0.38, with
10-23 deg of joint margin: that is the layout the task uses. The handle's knob is reachable with a +y grip along its
whole arc (pivot x 0.38-0.44, 22-34 cm up).
"""
import itertools
import math

import numpy as np
from yam_arena import ik, station
from yam_arena.ik import matrix_to_quat_xyzw

TOP = station.TABLE_TOP
BASES = station.BASES


def rim_rot(ang: float) -> np.ndarray:
    """TCP frame pinching the rim at angle ang (0 = the rim point nearest the robot, -x; positive towards +y):
    approach (+z) horizontal and radially inward, fingers closing vertically (TCP x = world z)."""
    radial = np.array([-math.cos(ang), math.sin(ang), 0.0])  # from the axis out to the rim point
    z = -radial
    x = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def solve_path(side, points, rots):
    q = None
    worst_mm = worst_deg = 0.0
    margin = 9.0
    for p, r in zip(points, rots):
        q, pe, re = ik.solve(p, matrix_to_quat_xyzw(r), q, base_pos=BASES[side], iters=300)
        worst_mm, worst_deg = max(worst_mm, pe * 1000), max(worst_deg, math.degrees(re))
        margin = min(margin, float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q))))
    return worst_mm, worst_deg, math.degrees(margin)


def wheel(wx, wz, r, angs):
    c = np.array([wx, 0.0, TOP + wz])
    pts = [c + r * np.array([-math.cos(p), math.sin(p), 0.0]) for p in angs]
    return solve_path("left", pts, [rim_rot(p) for p in angs])


def lever_rot(theta):
    """Gripping the handle's knob from the front-right, approach along +y (towards the tester), fingers vertical."""
    z = np.array([0.0, 1.0, 0.0])
    x = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def lever(px, py, pz, length, thetas):
    """Handle pivot at (px, py, TOP+pz) about the y axis; theta 0 = straight up, positive = knob pushed back (+x)."""
    pts = [np.array([px + length * math.sin(t), py, TOP + pz + length * math.cos(t)]) for t in thetas]
    return solve_path("right", pts, [lever_rot(t) for t in thetas])


def spinner_stroke(wx, kz, r=0.08):
    """The task's grip: forward (+x), fingers along y, fixed while a spinning knob is carried round the front arc."""
    rot = np.column_stack([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    angs = np.radians(np.linspace(-60, 60, 13))
    c = np.array([wx, 0.0, TOP + kz])
    pts = [c + r * np.array([-math.cos(a), math.sin(a), 0.0]) for a in angs]
    return solve_path("left", pts, [rot] * len(pts))


if __name__ == "__main__":
    print("spinner knob stroke (the task's grip): wheel x, knob height -> worst IK mm, deg, margin deg")
    for wx, kz in itertools.product((0.32, 0.36, 0.40), (0.15, 0.17, 0.19)):
        print(f"  x {wx:.2f} z {kz:.2f}:", "%.1f mm %.1f deg margin %.0f" % spinner_stroke(wx, kz))
    angs = np.radians(np.linspace(-50, 50, 11))
    print("wheel: x, z above table, radius -> worst IK mm, deg, least joint margin deg")
    for wx, wz, r in itertools.product((0.30, 0.34, 0.38, 0.42), (0.12, 0.16, 0.20, 0.24), (0.09,)):
        print(f"  x {wx:.2f} z {wz:.2f} r {r:.2f}:", "%.1f mm %.1f deg margin %.0f" % wheel(wx, wz, r, angs))
    th = np.radians(np.linspace(-10, 60, 8))
    print("lever: pivot x, y, z, length -> worst IK mm, deg, margin deg")
    for px, pz in itertools.product((0.38, 0.44, 0.50), (0.22, 0.28, 0.34)):
        print(f"  pivot x {px:.2f} z {pz:.2f}:", "%.1f mm %.1f deg margin %.0f" % lever(px, -0.12, pz, 0.12, th))
