"""Offline scan for rockwell-hardness-test (no Isaac): where can the right arm hold its wrist camera so that it sees
the dial square-on and close, while the left hand turns the handwheel below it?

    PYTHONPATH=<yam-isaac-lab-arena checkout>:. python3 tools/scan_dial_view.py

For each camera distance, yaw and pitch of the line of sight and roll about it, the camera is put on the line through
the dial's centre and the right arm's IK solved for the gripper pose that carries it there (yam_arena's NumPy IK on the
URDF; no collisions checked). It prints the best poses: IK error, joint margin, and the dial's size in pixels at the
policy's 320 x 176 camera (the 640 x 360 render halved).
"""
import itertools
import math

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import TCP_OFFSET
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.urdf import quat_xyzw_to_matrix

import rockwell_hardness_test as T

CAM = station.wrist_camera("right")
CAM_POS = np.array(CAM.pos)  # in the gripper link frame
CAM_ROT = quat_xyzw_to_matrix(np.array(CAM.rot_xyzw))  # opengl: looks along its -z, up is its +y
FX_POLICY = CAM.lens.fx * 320 / 640


def view_rotation(look: np.ndarray, roll: float) -> np.ndarray:
    """The camera's world rotation looking along ``look``, its up vector rolled ``roll`` from world +z."""
    z = -look / np.linalg.norm(look)
    up0 = np.array([0.0, 0.0, 1.0]) - np.dot([0.0, 0.0, 1.0], z) * z
    up0 /= np.linalg.norm(up0)
    x0 = np.cross(up0, z)
    up = math.cos(roll) * up0 + math.sin(roll) * x0
    return np.column_stack([np.cross(up, z), up, z])


def pose_for(dist: float, yaw: float, pitch: float, roll: float):
    """TCP position and rotation that put the right wrist camera ``dist`` from the dial's centre, looking at it."""
    look = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
    r_cam = view_rotation(look, roll)
    r_grip = r_cam @ CAM_ROT.T
    cam_world = T.DIAL_C - dist * look
    grip_origin = cam_world - r_grip @ CAM_POS
    return grip_origin + r_grip @ np.array(TCP_OFFSET), r_grip


def main() -> None:
    rows = []
    for dist, yaw_d, pitch_d, roll_d in itertools.product((0.14, 0.17, 0.20, 0.24), range(-10, 41, 10),
                                                        range(-10, 31, 10), range(-90, 91, 15)):
        p, r = pose_for(dist, math.radians(yaw_d), math.radians(pitch_d), math.radians(roll_d))
        q, pe, re = ik.solve(p, matrix_to_quat_xyzw(r), None, base_pos=station.BASES["right"], iters=300)
        margin = float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q)))
        rows.append((pe * 1000, math.degrees(re), math.degrees(margin), dist, yaw_d, pitch_d, roll_d, p))
    ok = sorted((r for r in rows if r[0] < 2.0 and r[1] < 2.0), key=lambda r: (r[3], -r[2]))
    print(f"{len(ok)} of {len(rows)} poses within 2 mm / 2 deg")
    for pe, re, m, dist, yaw_d, pitch_d, roll_d, p in ok[:25]:
        px = FX_POLICY * 2 * T.DIAL_R / dist
        print(f"dist {dist:.2f} yaw {yaw_d:+3d} pitch {pitch_d:+3d} roll {roll_d:+4d}: IK {pe:.1f} mm {re:.1f} deg, "
              f"margin {m:.0f} deg, dial {px:.0f} px across; tcp {np.round(p - [0, 0, T.TOP], 3)} above the table")


if __name__ == "__main__":
    main()
