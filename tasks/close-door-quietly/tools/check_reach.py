"""Offline reach check for close-door-quietly (no Isaac): can the right arm grip the door's knob, turn it 45 deg and
hold it through the last degrees of the close, for a true-size door standing on the floor beside the table?

Run from a yam-isaac-lab-arena checkout:
    PYTHONPATH=. python3 <this file>
Uses yam_arena's NumPy IK (DLS on the URDF, joint limits clipped) and the station's table, floor and top camera.
It checks poses, not physics: no collisions between the arm and the door are tested here.
"""
import itertools
import math

import numpy as np
from yam_arena import ik, station
from yam_arena.urdf import quat_xyzw_to_matrix

FLOOR = station.FLOOR_Z
TABLE_TOP = station.TABLE_TOP
TX0 = station.TABLE_CENTER[0] - station.TABLE_SIZE[0] / 2  # table's back edge (x)
TY = station.TABLE_SIZE[1] / 2  # table's side edges at y = +-TY
RIGHT = station.BASES["right"]

# The door (spec, all [UNVERIFIED] until measured): 1-3/8 in slab, 2-3/8 in backset, knob 36 in above the floor.
THICK = 0.0349
BACKSET = 0.0603
KNOB_H = 0.914
KNOB_OUT = 0.050  # knob body's centre from the door's face (door_scoring.KNOB_OUT)
TURN = math.radians(45)  # knob turn that pulls the bolt in [estimate]


def knob_pose(hinge_x, plane_y, width, theta):
    """Knob centre and the door's outward normal (toward the robot) with the door open theta (towards the robot)."""
    along = np.array([math.cos(theta), math.sin(theta), 0.0])  # opening turns the door about +z, towards the robot
    normal = np.array([-math.sin(theta), math.cos(theta), 0.0])
    hinge = np.array([hinge_x, plane_y, FLOOR + KNOB_H])
    return hinge + (width - BACKSET) * along + (THICK / 2 + KNOB_OUT) * normal, normal


def grip_rot(normal, roll):
    """TCP frame: approach (+z) into the door, along -normal; roll about the approach axis."""
    z = -normal
    x0 = np.array([0.0, 0.0, 1.0])
    y0 = np.cross(z, x0)
    c, s = math.cos(roll), math.sin(roll)
    x, y = c * x0 + s * y0, -s * x0 + c * y0
    return np.column_stack([x, y, z])


def solve(pos, rot, q0):
    q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q0, base_pos=RIGHT, iters=300)
    return q, pe * 1000, math.degrees(re)


def check(hinge_x, plane_y, width, thetas_deg, base_roll):
    """Worst IK error over a close from the widest theta to 0 with the knob turned, then the release at theta 0."""
    worst_mm = worst_deg = 0.0
    q = np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])
    margin = 9.0
    steps = [(math.radians(t), TURN) for t in thetas_deg]  # grip turned, door drawn shut
    steps = [(steps[0][0], 0.0)] + steps + [(0.0, TURN * (1 - k / 6)) for k in range(1, 7)]  # grip, close, release
    for theta, turn in steps:
        pos, n = knob_pose(hinge_x, plane_y, width, theta)
        q, mm, deg = solve(pos, grip_rot(n, base_roll + turn), q)
        worst_mm, worst_deg = max(worst_mm, mm), max(worst_deg, deg)
        margin = min(margin, float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q))))
    return worst_mm, worst_deg, math.degrees(margin)


def top_cam_sees(points):
    cam = station.top_camera()
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw if hasattr(cam, "rot_xyzw") else cam[1]))
    pos = np.array(cam.pos if hasattr(cam, "pos") else cam[0])
    fx, fy, cx, cy = station.TOP_CAM_K
    out = []
    for p in points:
        c = rot.T @ (np.asarray(p) - pos)  # OpenGL camera: looks along -z, +y up
        if c[2] >= 0:
            out.append(None)
            continue
        u, v = cx + fx * c[0] / -c[2], cy - fy * c[1] / -c[2]
        out.append((round(u), round(v)))
    return out


def link_clearance(hinge_x, plane_y, width, open_deg, base_roll_deg):
    """Along the same close and release: each link origin's least distance past the door's robot-side face (in its
    plane's normal direction) and its lowest height above the table top."""
    from yam_arena.constants import ARM_JOINTS
    from yam_arena import YAM_URDF
    from yam_arena.urdf import forward_kinematics

    q = np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])
    worst_face, worst_table = 9.0, 9.0
    thetas = [math.radians(t) for t in range(open_deg, -1, -1)]
    for theta, turn in [(t, TURN) for t in thetas] + [(0.0, TURN * (1 - k / 6)) for k in range(1, 7)]:
        pos, n = knob_pose(hinge_x, plane_y, width, theta)
        q, _, _ = solve(pos, grip_rot(n, math.radians(base_roll_deg) + turn), q)
        fk = forward_kinematics(YAM_URDF, {j: float(v) for j, v in zip(ARM_JOINTS, q, strict=True)})
        hinge = np.array([hinge_x, plane_y])
        for name, tf in fk.items():
            p = np.asarray(tf)[:3, 3] + np.asarray(RIGHT)
            face = float(np.dot(p[:2] - hinge, n[:2])) - THICK / 2  # + = on the robot's side of the door
            worst_face = min(worst_face, face)
            worst_table = min(worst_table, p[2] - TABLE_TOP)
    return worst_face, worst_table


if __name__ == "__main__":
    import sys

    print(f"table top {TABLE_TOP:.4f}, floor {FLOOR:.4f}, table x from {TX0:.3f}, sides y +-{TY:.3f}; "
          f"knob {KNOB_H - (TABLE_TOP - FLOOR):.3f} m above the table top")
    if "grid" not in sys.argv:
        # 24 in door, hinge 0.25 m behind the bases, door's centre plane 10 cm outside the table's side edge (so the
        # 116 mm jamb clears the table)
        hx, py, width, open_deg, roll = -0.25, -0.75, 0.610, int(sys.argv[1]) if len(sys.argv) > 1 else 6, 90
        mm, deg, mg = check(hx, py, width, list(range(open_deg, -1, -1)), math.radians(roll))
        face, table = link_clearance(hx, py, width, open_deg, roll)
        knob, _ = knob_pose(hx, py, width, 0.0)
        cam = station.top_camera()
        print(f"hinge x {hx}, plane y {py}, width {width}, open {open_deg} deg, roll {roll}: "
              f"worst IK {mm:.1f} mm / {deg:.1f} deg, joint margin {mg:.1f} deg")
        print(f"link origins: closest {face * 1000:.0f} mm in front of the door's face, lowest {table * 1000:.0f} mm "
              f"above the table top")
        print(f"top camera pixels ({cam.lens.width}x{cam.lens.height}, None = behind):",
              dict(zip(("knob", "latch edge at the bolt"),
                       top_cam_sees([knob, (hx + width, py, FLOOR + KNOB_H + 0.03)]))))
        sys.exit()
    print("width  hinge_x  plane_y  open  roll  ->  worst IK mm / deg, joint margin deg, knob dist from base m")
    rows = []
    for width, hx, py, open_deg in itertools.product(
        (0.610, 0.711), (-0.35, -0.30, -0.25, -0.20), (-0.665, -0.68, -0.70), (4, 8, 12)
    ):
        if hx + width * 0 > TX0 and py > -TY:  # hinge would stand inside the table's footprint
            continue
        best = None
        for roll_deg in (-90, -45, 0, 45, 90, 180):
            r = check(hx, py, width, list(range(open_deg, -1, -1)), math.radians(roll_deg))
            if best is None or (r[0], r[1]) < (best[1][0], best[1][1]):
                best = (roll_deg, r)
        pos, _ = knob_pose(hx, py, width, math.radians(open_deg))
        dist = float(np.linalg.norm(pos[:2] - np.array(RIGHT[:2])))
        rows.append((width, hx, py, open_deg, best[0], *best[1], dist))
    for w, hx, py, o, roll, mm, deg, mg, d in sorted(rows, key=lambda r: (r[5] > 2 or r[6] > 2, -r[3], r[5])):
        ok = "OK " if mm <= 2 and deg <= 2 else "-- "
        print(f"{ok}{w:.3f}  {hx:+.2f}  {py:+.3f}  {o:2d}deg  {roll:+4d}  ->  {mm:6.1f} mm / {deg:5.1f} deg, "
              f"margin {mg:5.1f} deg, {d:.3f} m")

    # What the top camera sees of the latch edge and the knob, for the first OK layout.
    good = [r for r in rows if r[5] <= 2 and r[6] <= 2]
    if good:
        w, hx, py = good[0][:3]
        latch_top = (hx + w, py, FLOOR + KNOB_H + 0.03)
        knob, _ = knob_pose(hx, py, w, 0.0)
        door_top = (hx + w, py, FLOOR + 2.032)
        print("top camera pixels (640x480, None = behind):",
              dict(zip(("knob", "latch edge at the bolt", "door top corner"), top_cam_sees([knob, latch_top, door_top]))))
