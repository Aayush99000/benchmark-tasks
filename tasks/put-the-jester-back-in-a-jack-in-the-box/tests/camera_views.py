"""Where things fall in the three policy cameras, by pinhole projection. NumPy only (plus yam_arena constants).

The policy sees the overhead camera and the two wrist cameras at 320 x 176. These helpers project points of
the scene into those images to say how many pixels wide an object is, and whether it is in view.
"""

import numpy as np
from yam_arena import station
from yam_arena.constants import ALL_JOINTS
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix
from yam_arena import YAM_URDF

WIDTH, HEIGHT = 320, 176
TOP = station.TABLE_TOP
RIG = station.rig("calibrated")


class View:
    """A camera at ``position`` with rotation ``rotation`` (OpenGL axes: looks along -z, +y up), tabletop frame."""

    def __init__(self, position, rotation, lens):
        self.position, self.rotation = np.asarray(position, float), np.asarray(rotation, float)
        self.lens = lens.resized(WIDTH, HEIGHT)

    def pixel(self, point):
        c = self.rotation.T @ (np.asarray(point, float) - self.position)
        depth = -c[2]
        return np.array((self.lens.cx + self.lens.fx * c[0] / depth, self.lens.cy - self.lens.fy * c[1] / depth)), depth

    def in_view(self, point):
        (u, v), depth = self.pixel(point)
        return bool(depth > 0 and 0 <= u < WIDTH and 0 <= v < HEIGHT)

    def length(self, a, b):
        """Pixels between the images of two points."""
        return float(np.linalg.norm(self.pixel(a)[0] - self.pixel(b)[0]))

    def thickness(self, point, across, size):
        """Pixels covered by something ``size`` metres thick at ``point``, measured along the unit vector ``across``."""
        half = .5 * size * np.asarray(across, float)
        return self.length(np.asarray(point) - half, np.asarray(point) + half)

    def best_thickness(self, point, along, size):
        """Pixels across a thin round thing (a wire, a dowel) lying along ``along`` at ``point``."""
        sight = np.asarray(point, float) - self.position
        across = np.cross(along, sight)
        return self.thickness(point, across / np.linalg.norm(across), size)


def overhead():
    cam = RIG.top_cam
    return View(np.array(cam.pos) - (0., 0., TOP), quat_xyzw_to_matrix(np.array(cam.rot_xyzw)), cam.lens)


def wrist(side, joints, base):
    """The wrist camera of one arm at the given joint angles; ``base`` is the arm base in the tabletop frame."""
    q = dict(zip(ALL_JOINTS[:6], joints, strict=True))
    q["joint7"] = q["joint8"] = 0.
    link = forward_kinematics(YAM_URDF, q)["gripper"]
    cam = RIG.wrist_cams[side]
    position = link[:3, :3] @ np.array(cam.pos) + link[:3, 3] + np.asarray(base)
    return View(position, link[:3, :3] @ quat_xyzw_to_matrix(np.array(cam.rot_xyzw)), cam.lens)
