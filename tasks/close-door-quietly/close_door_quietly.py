"""To the robot's right, beyond the table's right edge, a real interior door stands in its frame on the floor: a white
hollow-core door 61 cm wide and 2.03 m tall, hinged at the back (behind the arms) and opening towards the robot, so it
closes by being pushed away. It stands ajar, its latch edge about 6.4 cm short of the frame. On the robot's side of the
door, 15 cm above the table top and about 30 cm forward of the arms, is a round brass doorknob. Turning the knob either way draws the
latch bolt (the tongue in the door's edge, 13 mm proud) in; let go, it springs back out. The frame's latch side has a
brass strike plate with a hole for the bolt. Pushed shut, the door's bolt rides over the strike's lip with a click and
snaps into the hole.

Shut the door so it latches, without a sound.
"""

import math
import os
import pathlib
import sys
from dataclasses import dataclass

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import door_scoring as S  # noqa: E402
import mechanism as mech  # noqa: E402
from mechanism import Joint, Link, Shape  # noqa: E402
from yam_arena import demo as yam_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the door unit, in the world frame (the mechanism's frame is the world's: pos 0, no yaw) --------------------------
HX, Y0 = S.HINGE_XY  # hinge axis; the door's centre plane when shut
F = S.FLOOR_Z
KZ = F + S.KNOB_H  # knob and bolt height
EDGE = HX + S.WIDTH  # the latch edge, shut
KX = EDGE - S.BACKSET  # the knob's axis
FACE = Y0 + S.THICK / 2  # the door's robot-side face, shut
DOOR_H = 2.032  # 80 in
UNDERCUT = 0.015
DOOR_KG = 9.0  # hollow-core 24 x 80 slab [estimate]

JAMB_X = EDGE + 0.0035  # the latch jamb's face: a 3.5 mm gap to the door's edge
JAMB_T = 0.019  # 3/4 in jamb stock
JAMB_SWING = FACE + 0.003  # the jamb's swing-side edge, facing the robot: a 3 mm reveal past the door's robot-side face
JAMB_FAR = JAMB_SWING - 0.116  # 116 mm deep, running away from the robot
JAMB_H = DOOR_H + UNDERCUT + 0.003
HOLE_Y = (Y0 - 0.008, Y0 + 0.010)  # the strike's hole across the jamb (the bolt is 10 mm thick)
HOLE_Z = 0.020  # half height
STOP_T, STOP_W = 0.0095, 0.0127  # door stop moulding: 3/8 in thick, standing 1/2 in off the jamb

# The strike's lip: a ramp at 45 degrees (a square turned 45 degrees, its corner 2 mm past the door's edge) on the
# swing side of the hole (the robot's side). A bolt standing out meets its face towards the robot and is pushed in as
# the door closes, then springs out into the hole; a bolt held flush passes in front of it.
LIP_A = 0.016  # half diagonal
LIP_C = (EDGE + 0.002 + LIP_A, Y0 + 0.020)

# The bolt, in the door (shut): from 25 mm inside the edge to 13 mm proud of it; 10 mm thick, 22 mm tall.
BOLT_BACK = EDGE - 0.025
BOLT_T, BOLT_H = 0.010, 0.022
TAIL_Z = 0.030  # two tail bars, above and below the cam's sweep, to the yoke behind the spindle
YOKE_FACE = KX - S.CAM_GAP  # the yoke's face towards the bolt; the cam pushes it in (towards the hinge)

WHITE, BRASS, PINE, GREY = (0.93, 0.93, 0.91), (0.78, 0.64, 0.30), (0.86, 0.78, 0.62), (0.55, 0.55, 0.55)
MATERIALS = {"wood": (0.5, 0.4), "brass": (0.5, 0.4), "latch": (0.1, 0.08)}  # latch parts greased [estimate]


def _box(lo, hi, color, material="wood", yaw=0.0) -> Shape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple((lo + hi) / 2), tuple(hi - lo), color, material=material, yaw=yaw)


def _jamb_shapes() -> tuple:
    jx = (JAMB_X, JAMB_X + JAMB_T)
    zl, zh = KZ - HOLE_Z, KZ + HOLE_Z
    return (
        _box((jx[0], JAMB_FAR, F), (jx[1], JAMB_SWING, zl), PINE),  # latch jamb below the hole
        _box((jx[0], JAMB_FAR, zh), (jx[1], JAMB_SWING, F + JAMB_H), PINE),  # above it
        _box((jx[0], HOLE_Y[1], zl), (jx[1], JAMB_SWING, zh), PINE),  # swing side of the hole (the robot's)
        _box((jx[0], JAMB_FAR, zl), (jx[1], HOLE_Y[0], zh), PINE),  # far side of the hole
        _box((JAMB_X - 0.0015, HOLE_Y[0] - 0.012, KZ - 0.034), (JAMB_X, HOLE_Y[0], KZ + 0.034), BRASS, "brass"),
        Shape("box", (LIP_C[0], LIP_C[1], KZ), (LIP_A * math.sqrt(2), LIP_A * math.sqrt(2), 0.050), BRASS,
              material="latch", yaw=math.pi / 4),  # the strike's lip
        _box((JAMB_X - STOP_W, Y0 - S.THICK / 2 - STOP_T, F), (JAMB_X, Y0 - S.THICK / 2, F + JAMB_H), WHITE),  # stop
        _box((HX - 0.02 - JAMB_T, JAMB_FAR, F), (HX - 0.02, JAMB_SWING, F + JAMB_H), PINE),  # hinge jamb
        _box((HX - 0.02 - JAMB_T, JAMB_FAR, F + JAMB_H), (JAMB_X + JAMB_T, JAMB_SWING, F + JAMB_H + JAMB_T), PINE),
    )


BASE = Link("frame", 20.0, _jamb_shapes())
DOOR = Link("door", DOOR_KG, (
    _box((HX + 0.004, Y0 - S.THICK / 2, F + UNDERCUT), (EDGE, Y0 + S.THICK / 2, F + UNDERCUT + DOOR_H), WHITE),
    Shape("cylinder", (KX, FACE + 0.003, KZ), (0.034, 0.006), BRASS, axis="Y", material="brass"),  # the knob's rose
))
KNOB = Link("knob", 0.25, (
    Shape("cylinder", (KX, FACE + (S.KNOB_OUT - S.KNOB_LEN / 2) / 2, KZ), (0.011, S.KNOB_OUT - S.KNOB_LEN / 2), BRASS,
          axis="Y", material="brass"),  # neck
    Shape("cylinder", (KX, FACE + S.KNOB_OUT, KZ), (S.KNOB_R, S.KNOB_LEN), BRASS, axis="Y", material="brass"),  # knob
    _box((KX - S.CAM_T / 2, Y0 - 0.003, KZ - S.CAM_R), (KX + S.CAM_T / 2, Y0 + 0.003, KZ + S.CAM_R), GREY, "latch"),
))
BOLT = Link("bolt", 0.05, (
    _box((BOLT_BACK, Y0 - BOLT_T / 2, KZ - BOLT_H / 2), (EDGE + S.THROW, Y0 + BOLT_T / 2, KZ + BOLT_H / 2), BRASS,
         "latch"),
    *(_box((YOKE_FACE - 0.003, Y0 - 0.003, KZ + s * TAIL_Z - 0.003), (BOLT_BACK, Y0 + 0.003, KZ + s * TAIL_Z + 0.003),
           GREY, "latch") for s in (-1, 1)),
    _box((YOKE_FACE - 0.003, Y0 - 0.005, KZ - TAIL_Z - 0.003), (YOKE_FACE, Y0 + 0.005, KZ + TAIL_Z + 0.003), GREY,
         "latch"),  # the yoke
))
# 6 deg ajar: the latch edge 64 mm off the stop, and the slab's corner 19 mm clear of the table's side edge (it would
# meet the table past about 7 deg).
DOOR_START = math.radians(6.0)
JOINTS = (
    # Positive turns about +z swing the latch edge towards the robot: the door opens to positive angles; 0 is shut
    # against the stop (the hinge's lower limit; the door's slab never touches the frame).
    Joint("door", "revolute", "door", (HX, Y0, KZ), "Z", 0.0, 1.4, damping=1.0, friction=0.3, start=DOOR_START),
    Joint("knob", "revolute", "knob", (KX, Y0, KZ), "Y", -0.80, 0.80, stiffness=0.5, damping=0.01, parent="door"),
    # Positive slides out; the spring's target 4 mm past the stop preloads the bolt out (1.2 N, 5.1 N held in).
    Joint("bolt", "prismatic", "bolt", (EDGE, Y0, KZ), "X", -S.THROW, 0.0, stiffness=300.0, damping=1.0,
          target=0.004, parent="door"),
)
DOOR_UNIT = mech.mechanism("door_unit", (0.0, 0.0, 0.0), BASE, (DOOR, KNOB, BOLT), JOINTS, MATERIALS,
                           color=WHITE, base_contacts=("bolt",))
OBJECTS = [DOOR_UNIT]


def joints_for_scoring(state) -> tuple[float, float, float]:
    """The scorer's (door open, knob turn, bolt in) from the mechanism's joints."""
    j = state.joints
    return float(j.get("door", 0.0)), float(j.get("knob", 0.0)), -float(j.get("bolt", 0.0))


# -- plan-view check of the latch (pure geometry, for tools/ and the tests) ----------------------------------------------
def _convex_overlap(a: np.ndarray, b: np.ndarray) -> bool:
    """Separating-axis test for two convex polygons (rows are vertices, in order)."""
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() <= pb.min() + 1e-9 or pb.max() <= pa.min() + 1e-9:
                return False
    return True


def _rect(x0, x1, y0, y1) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


FRAME_AT_BOLT = (
    _rect(JAMB_X, JAMB_X + JAMB_T, HOLE_Y[1], JAMB_SWING),
    _rect(JAMB_X, JAMB_X + JAMB_T, JAMB_FAR, HOLE_Y[0]),
    _rect(JAMB_X - 0.0015, JAMB_X, HOLE_Y[0] - 0.012, HOLE_Y[0]),
    np.array([[LIP_C[0] - LIP_A, LIP_C[1]], [LIP_C[0], LIP_C[1] - LIP_A], [LIP_C[0] + LIP_A, LIP_C[1]],
              [LIP_C[0], LIP_C[1] + LIP_A]]),
)


def bolt_outline(door: float, bolt_in: float) -> np.ndarray:
    """The bolt's plan outline (world xy) with the door open ``door`` rad and the bolt pushed ``bolt_in`` m in."""
    local = _rect(BOLT_BACK - HX, S.WIDTH + S.THROW - bolt_in, -BOLT_T / 2, BOLT_T / 2)  # from the hinge
    c, s = math.cos(door), math.sin(door)
    rot = np.array([[c, -s], [s, c]])  # opening turns the door about +z, towards the robot
    return local @ rot.T + np.array([HX, Y0])


def bolt_fits(door: float, bolt_in: float) -> bool:
    return not any(_convex_overlap(bolt_outline(door, bolt_in), p) for p in FRAME_AT_BOLT)


def push_needed(door: float, steps: int = 131) -> float:
    """The least the bolt must be pushed in (m) to fit the frame with the door at ``door``; inf if even flush jams."""
    for b in np.linspace(0.0, S.THROW, steps):
        if bolt_fits(door, float(b)):
            return float(b)
    return math.inf


# -- the record: the door's joints and the grippers, once per control step -----------------------------------------------
@dataclass(frozen=True)
class Pads:
    """One gripper after a control step (world frame, m): the span along its pointing axis from the pads' back to
    their tips, and points across the end of each of its two fingertips."""

    back: np.ndarray
    tips: np.ndarray
    fingertips: tuple = ()

    def points(self) -> np.ndarray:
        along = [self.back + s * (self.tips - self.back) for s in np.linspace(0.0, 1.0, 6)]
        return np.array([*along, *(np.asarray(p, float) for p in self.fingertips)])


PAD_BACK = 0.025
GRIPPERS = ("left_gripper", "right_gripper")
FINGERTIP_ENDS = {  # as in take-throat-swab and break-chocolate-row: each fingertip's end in its link's frame (m)
    "tip_left": [(x, -0.04454, -0.09016) for x in (0.01799, 0.02399, 0.02999)],
    "tip_right": [(x, 0.04454, -0.09016) for x in (-0.02999, -0.02399, -0.01799)],
}

RECORD = S.DoorRecord()
_LAST = {"t": None}


def _clear_record() -> None:
    RECORD.reset()
    _LAST["t"] = None


mech.on_reset(_clear_record)


def pad_points(objects) -> list:
    return [objects[g].points() for g in GRIPPERS if objects.get(g) is not None]


def update_record(states, t: float | None) -> S.DoorRecord:
    """Add a control step to the record, once per simulated time ``t`` (object states are read more than once a
    step). With no clock, every call counts as a step of ``S.DT``."""
    if t is not None and _LAST["t"] is not None and t <= _LAST["t"] + 1e-9:
        return RECORD
    dt = S.DT if t is None or _LAST["t"] is None else t - _LAST["t"]
    _LAST["t"] = t
    RECORD.step(*joints_for_scoring(states["door_unit"]), pad_points(states), dt=dt)
    return RECORD


def _sim_time(env) -> float | None:
    base = env.unwrapped
    try:
        return float(base.episode_length_buf[0]) * float(base.step_dt)
    except (AttributeError, TypeError, IndexError):
        sim = getattr(base, "sim", None)
        return float(sim.current_time) if sim is not None and hasattr(sim, "current_time") else None


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = runtime.link_poses(robot)
        axis = poses["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        ends = tuple(poses[link][:3, :3] @ np.array(p) + poses[link][:3, 3] - origin
                     for link, points in FINGERTIP_ENDS.items() for p in points)
        out[f"{side}_gripper"] = Pads(back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, fingertips=ends)
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_close_door_patched", False):
        return
    stock_states = scene.object_states

    def object_states(env, names):
        states = {**stock_states(env, names), **_gripper_states(env)}
        return {**states, "door_record": update_record(states, _sim_time(env))}

    scene.object_states = object_states
    scene._close_door_patched = True


# WHY through mechanism's own import hook: two hooks that each look the module up through the other finders called
# each other without end (copy-id-card, official sim, 2026-11-20).
_mech_scene_patch = mech._PATCHES["yam_arena.scene"]


def _scene_patches(scene) -> None:
    _mech_scene_patch(scene)
    _patch_scene(scene)


mech._PATCHES["yam_arena.scene"] = _scene_patches
if "yam_arena.scene" in sys.modules:
    _scene_patches(sys.modules["yam_arena.scene"])


# -- scoring --------------------------------------------------------------------------------------------------------
def _args(objects):
    rec = objects.get("door_record") or RECORD
    return (rec, *joints_for_scoring(objects["door_unit"]), pad_points(objects))


def _debug(objects, g) -> None:
    path = os.environ.get("DOOR_DEBUG")
    if not path:
        return
    rec, door, knob, bolt, _ = _args(objects)
    hand = ""
    pads = objects.get("right_gripper")
    if pads is not None:
        axis = (pads.tips - pads.back) / np.linalg.norm(pads.tips - pads.back)
        tcp = pads.tips - PAD_TIP_PAST_TCP * axis
        off = (tcp - S.knob_centre(door)) * 1000
        hand = (f"tcp=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) tcp_minus_knob_mm=({off[0]:.1f},{off[1]:.1f},{off[2]:.1f}) "
                f"axis=({axis[0]:.2f},{axis[1]:.2f},{axis[2]:.2f}) ")
    with open(path, "a") as f:
        f.write(hand)
        f.write(f"grade={g} door_deg={math.degrees(door):.2f} gap_mm={S.edge_gap(door) * 1000:.1f} "
                f"knob_deg={math.degrees(knob):.1f} bolt_in_mm={bolt * 1000:.2f} "
                f"holds_mm={S.knob_holds_in(knob) * 1000:.2f} scraped={rec.scraped} snap={rec.snap_speed:.3f} "
                f"close={rec.close_speed:.3f} edge={rec.edge_speed:.4f} hand={rec.hand_at_latch} first={rec.first}\n")


def score(objects) -> float:
    g = S.grade(*_args(objects))
    _debug(objects, g)
    return min(g, S.PARTIAL_CAP)


def success(objects) -> bool:
    return S.grade(*_args(objects)) == 1.0


# -- the scripted demo: the right hand takes the knob, turns it, pushes the door shut, lets the knob back slowly --------
def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle (as in take-throat-swab)."""
    if getattr(yam_demo, "_matrix_grips", False):
        return
    plain = yam_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    yam_demo.grip_rotation = grip_rotation
    yam_demo._matrix_grips = True


_allow_matrix_grips()

GRIP_ROLL = math.radians(90)  # fingers closing across the knob's top and bottom (tools/check_reach.py: best roll)
# Probe settings for the README's checks (the submitted demo uses the defaults): DOOR_DEMO_TURN_DEG=0 pushes the door
# shut without turning the knob (the ordinary click); DOOR_DEMO_RELEASE_S=0.05 lets the knob go in one step (the snap).
TURN = math.radians(float(os.environ.get("DOOR_DEMO_TURN_DEG", math.degrees(S.KNOB_FULL))))
RELEASE_S = float(os.environ.get("DOOR_DEMO_RELEASE_S", "2.0"))


def knob_pose(door: float) -> tuple[np.ndarray, np.ndarray]:
    """The knob's centre and the door's outward normal (towards the robot), the door open ``door`` rad."""
    return S.knob_centre(door), np.array([-math.sin(door), math.cos(door), 0.0])


def grip(door: float, turn: float) -> np.ndarray:
    """TCP rotation: pointing into the door along its normal, rolled ``GRIP_ROLL + turn`` about that axis."""
    _, n = knob_pose(door)
    z = -n
    x0 = np.array([0.0, 0.0, 1.0])
    y0 = np.cross(z, x0)
    roll = GRIP_ROLL + turn
    x = math.cos(roll) * x0 + math.sin(roll) * y0
    return np.column_stack([x, np.cross(z, x), z])


def _pt(v) -> tuple:
    return tuple(float(a) for a in v)


PARK_LEFT = (0.16, 0.33, TOP + 0.20)
BACK_OFF = 0.08
# The right hand parks 15 cm back from the shut knob and 5 cm up, still pointing at the door: turning it to point down
# from there needs joint5 past its limit.
CLEAR_BACK, CLEAR_UP = 0.15, 0.05


def _right() -> list:
    c0, n0 = knob_pose(DOOR_START)
    shut, n1 = knob_pose(0.0)
    start_deg = round(math.degrees(DOOR_START))
    close = [Waypoint(0.5, _pt(knob_pose(math.radians(d))[0]), FINGER_CLOSED, f"push the door to {d} deg",
                      grip(math.radians(d), TURN)) for d in range(start_deg - 1, -1, -1)]
    return [
        Waypoint(2.0, _pt(c0 + BACK_OFF * n0), FINGER_OPEN, "in front of the knob", grip(DOOR_START, 0.0)),
        Waypoint(1.5, _pt(c0), FINGER_OPEN, "round the knob", grip(DOOR_START, 0.0)),
        Waypoint(1.2, _pt(c0), FINGER_CLOSED, "grip it", grip(DOOR_START, 0.0)),
        Waypoint(1.0, _pt(c0), FINGER_CLOSED, "turn it: the bolt is drawn in", grip(DOOR_START, TURN)),
        *close,
        Waypoint(0.5, _pt(shut), FINGER_CLOSED, "hold, the door against its stop", grip(0.0, TURN)),
        Waypoint(RELEASE_S, _pt(shut), FINGER_CLOSED, "let the knob back slowly: the bolt slides into the strike",
                 grip(0.0, 0.0)),
        Waypoint(1.2, _pt(shut), FINGER_OPEN, "let go", grip(0.0, 0.0)),
        Waypoint(1.5, _pt(shut + BACK_OFF * n1), FINGER_OPEN, "back off the knob", grip(0.0, 0.0)),
        Waypoint(1.5, _pt(shut + CLEAR_BACK * n1 + (0.0, 0.0, CLEAR_UP)), FINGER_OPEN, "clear", grip(0.0, 0.0)),
        Waypoint(3.0, _pt(shut + CLEAR_BACK * n1 + (0.0, 0.0, CLEAR_UP)), FINGER_OPEN, "stay", grip(0.0, 0.0)),
    ]


def _left() -> list:
    down = yam_demo.grip_rotation(0.0)
    return [Waypoint(1.0, PARK_LEFT, FINGER_OPEN, "out of the way", down),
            Waypoint(20.0, PARK_LEFT, FINGER_OPEN, "stay", down)]


def demo() -> dict:
    return {"left": _left(), "right": _right()}
