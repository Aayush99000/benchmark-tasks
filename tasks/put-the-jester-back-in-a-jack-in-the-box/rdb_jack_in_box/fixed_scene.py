"""Fixed scene: one popped-open jack-in-the-box on the calibrated dual-arm YAM station."""

from pathlib import Path

import numpy as np

from . import layout

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("jack_in_box.usda")
CONTROL_HZ = 20
MAX_STEPS = 1600
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table
ACTORS = layout.ACTORS
START = layout.start_poses()                    # tabletop frame: (position, xyzw quaternion)
WORLD_START = {name: ((p[0], p[1], p[2] + TOP), q) for name, (p, q) in START.items()}

INSTRUCTION = (
    "The jack-in-the-box has popped open. Push the jester back down into the box and shut the lid over it so "
    "that the catch holds the lid shut, then let go and move both arms away. Leave the box where it is."
)


def side_camera():
    """Review camera: to the left of the box, a little behind it, looking across at the lid, the jester and the
    front catch. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((layout.BOX_X + .10, .50, TOP + .30))
    back = eye - (layout.BOX_X - .01, 0., TOP + .13)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(24.))
