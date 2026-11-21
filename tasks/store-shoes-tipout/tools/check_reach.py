"""Offline reach scan for the tip-out shoe cabinet: the door's top edge through its swing, and shoe drop points.

Run from a yam-isaac-lab-arena checkout: PYTHONPATH=. python3 <this file>. Needs only numpy (no Isaac).
IKEA TRONES: 52.1 cm wide, 18.1 cm deep, 39.1 cm tall (IKEA US page). The front is one door hinged along its
bottom edge that tips out toward the robot; its top edge is DOOR_R from the hinge [estimate 0.37 m]. The cabinet
stands on the table with its front at x X0, centred on y 0. Pass = IK within 2 mm and 2 degrees.
"""
import numpy as np
from yam_arena import ik
from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS, TABLE_HEIGHT
from yam_arena.demo import grip_rotation
from yam_arena.urdf import axis_angle_matrix

T = TABLE_HEIGHT
DOOR_R = 0.37
down = grip_rotation(0.0)
Y = np.array([0.0, 1.0, 0.0])
grips = {"down": down, "tilt45": axis_angle_matrix(Y, -np.pi / 4) @ down,
         "level": axis_angle_matrix(Y, -np.pi / 2) @ down,
         "back45": axis_angle_matrix(Y, np.pi / 4) @ down}  # pointing down and back toward the arms (pulling the lip)
seeds = ([0, 1, 1, -0.6, 0, 0], [0, 0.5, 0.5, 0, 0, 0], [0, 1.5, 1.5, -1, 0, 0], [0, 1.0, 0.5, 0.5, 0, 0],
         [0, 2.0, 1.5, 0, 0, 0])


def ok(p, r, base) -> bool:
    quat = ik.matrix_to_quat_xyzw(r)
    best = min((ik.solve(p, quat, q_init=s, base_pos=base, iters=400)[1:] for s in seeds),
               key=lambda e: e[0] + 0.05 * e[1])
    return best[0] < 0.002 and best[1] < np.radians(2)


def cell(p, base) -> str:
    return "".join(g[0] if g != "back45" else "b" for g, r in grips.items() if ok(p, r, base)) or "-"


for x0 in (0.35, 0.40, 0.45):
    print(f"== cabinet front at x {x0:.2f}")
    for deg in (0, 15, 30, 45):
        a = np.radians(deg)
        lip = (x0 - DOOR_R * np.sin(a) - 0.02, None, T + DOOR_R * np.cos(a))  # TCP 2 cm in front of the door's lip
        print(f"  door lip at {deg:2d} deg (x {lip[0]:.2f}, {lip[2] - T:.2f} up): "
              f"left y+0.13 {cell((lip[0], 0.13, lip[2]), LEFT_BASE_POS)}  right y-0.13 {cell((lip[0], -0.13, lip[2]), RIGHT_BASE_POS)}")
    for z in (0.42, 0.48, 0.54):
        for dx in (-0.10, -0.05, 0.0):  # shoe held by its heel over the open bin, in front of the cabinet's front plane
            p = x0 + dx
            print(f"  shoe drop x {p:.2f}, {z:.2f} up: left y+0.13 {cell((p, 0.13, T + z), LEFT_BASE_POS)}  "
                  f"right y-0.13 {cell((p, -0.13, T + z), RIGHT_BASE_POS)}")
