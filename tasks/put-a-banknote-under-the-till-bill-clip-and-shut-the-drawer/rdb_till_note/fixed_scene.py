"""Fixed scene: an open cash drawer with its till, and one banknote lying on top of the drawer, on the YAM station."""

from pathlib import Path

import numpy as np

from . import till_model as till

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

TASK_ID = "put-a-banknote-under-the-till-bill-clip-and-shut-the-drawer"
SCORER = "till_note_score"
USD = Path(__file__).with_name("till.usda")
CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 2400
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

STRIPS = tuple(f"strip_{i}" for i in range(till.STRIPS))
LOOSE = ("stack", *STRIPS)                      # free bodies (the note's strips are hinged to each other)
HOLDS = tuple(f"hold_{i}" for i in range(5))
TILL_LINKS = ("case", "drawer", *HOLDS)
TILL_JOINTS = ("slide", *(f"hinge_{i}" for i in range(5)))
NOTE_JOINTS = tuple(f"fold_{i}" for i in range(1, till.STRIPS))

INSTRUCTION = (
    "Put the banknote that is lying on top of the cash drawer into the till: lay it flat in the middle note "
    "compartment, on top of the notes already there and under that compartment's hold-down clip, with the clip "
    "let down on it. Then push the drawer shut and take both arms away."
)


def side_camera():
    """Review camera: above the robot's left shoulder, looking down into the open till. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.02, .30, TOP + .50))
    back = eye - (.36, -.02, TOP + .06)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(22.))
