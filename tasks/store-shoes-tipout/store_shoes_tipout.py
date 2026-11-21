"""On the station table, 45 cm in front of the arms' bases and centred between them, stands a white tip-out shoe
cabinet (52 cm wide, 18 cm deep, 39 cm tall), its front towards the arms. Its whole front is one door, hinged along its
bottom edge, with a lip along its top edge to pull it by. Pulled, the door tips forward and down to about 40 degrees,
opening a bin behind it that is 7 cm deep at the bottom and 15 cm deep at the top. Pushed back past about 15 degrees,
it falls shut by its own weight. On top is a shallow tray.

On the table, one either side of the cabinet and just in front of it, lie a pair of red foam clogs (29 cm long, 10 cm
wide), soles down, toes towards the arms and heels towards the cabinet. A clog's toe is thin (6.5 cm) and its heel thick
(9.5 cm), and its heel cup is open on top.

Put both clogs away in the cabinet: tip the door open by its lip, stand each clog in the bin toe first, so that it sinks
below the bin's top edge, and tip the door shut. A clog put in heel first stands too high, and the door won't shut on
it.
"""

import math
import os
import pathlib
import sys
from dataclasses import dataclass

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import mechanism as mech  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from yam_arena import demo as yam_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cabinet --------------------------------------------------------------------------------------------------------
# IKEA TRONES shoe/storage cabinet: 52.1 x 18.1 x 39.1 cm outside (IKEA US product page, 20 1/2 x 7 1/8 x 15 3/8 in);
# IKEA's dimension drawing gives a 2 cm deep tray on top, the bin about 47 cm wide inside, its opening 15 cm deep, and
# 37 cm tall inside. The wall thickness, the door's open angle and the bin's depth at the bottom are estimates until a
# cabinet is measured. Heights are above the table; x, y are world (x forward from the arm bases, y left).
X0 = 0.45  # the cabinet's front face
CAB_W, CAB_D, CAB_H = 0.521, 0.181, 0.391
WALL = 0.006
TRAY_DEPTH = 0.020
TOP_UNDER = CAB_H - TRAY_DEPTH - WALL  # the underside of the tray's floor: the bin must pass below it to shut
BOTTOM = 0.020  # top of the cabinet's bottom panel
HINGE = (X0 + 0.003, 0.0, BOTTOM + 0.002)  # the door's hinge line, along y at the bottom front
DOOR_T = 0.006
DOOR_Y = CAB_W / 2 - WALL - 0.002
LIP_Z = TOP_UNDER - 0.003  # top of the door's lip
LIP_OUT = 0.020  # how far the lip stands proud of the door [estimate: TRONES' top edge curls outward]
BIN_W = 0.47
BIN_Y = BIN_W / 2
OPEN_DEG = 40.0  # [estimate from IKEA's photo]
OPEN = math.radians(OPEN_DEG)
# The bin's depth (door's inner face to its back) at the bottom and top; its back steps out in between.
BIN_DEPTH_BOTTOM, BIN_DEPTH_TOP = 0.070, 0.150
BIN_STEPS = 8
DOOR_KG = 0.7  # [estimate]
WHITE, GREY_TRAY, BIN_GREY, RED = (0.93, 0.93, 0.92), (0.80, 0.80, 0.79), (0.62, 0.64, 0.66), (0.85, 0.10, 0.08)
MATERIALS = {"plastic": (0.35, 0.30)}  # polypropylene on foam [estimate]; combined with "min" (below)


def _box(x0, x1, y0, y1, z0, z1, color, material="plastic") -> Shape:
    return Shape("box", ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color,
                 material=material)


def bin_depth(z: float) -> float:
    """The bin's depth at height z above the table, door shut: its back is a staircase of BIN_STEPS steps."""
    s = min(BIN_STEPS - 1, max(0, int((z - BOTTOM) / ((LIP_Z - BOTTOM) / BIN_STEPS))))
    return BIN_DEPTH_BOTTOM + (BIN_DEPTH_TOP - BIN_DEPTH_BOTTOM) * s / (BIN_STEPS - 1)


def _cabinet():
    yw = CAB_W / 2
    x1 = X0 + CAB_D
    base = Link("body", 2.0, (
        _box(X0, x1, yw - WALL, yw, 0.0, CAB_H, WHITE),
        _box(X0, x1, -yw, -yw + WALL, 0.0, CAB_H, WHITE),
        _box(x1 - WALL, x1, -yw + WALL, yw - WALL, 0.0, CAB_H, WHITE),
        _box(X0, x1 - WALL, -yw + WALL, yw - WALL, 0.0, BOTTOM, WHITE),
        _box(X0, x1 - WALL, -yw + WALL, yw - WALL, TOP_UNDER, TOP_UNDER + WALL, GREY_TRAY),  # the tray's floor
        _box(X0, X0 + 0.010, -yw + WALL, yw - WALL, TOP_UNDER + WALL, CAB_H, WHITE),  # the tray's front rim
    ))
    front = X0 + DOOR_T
    shapes = [
        _box(X0, front, -DOOR_Y, DOOR_Y, BOTTOM + 0.002, LIP_Z - 0.012, WHITE),  # the door
        _box(X0 - LIP_OUT, front, -DOOR_Y, DOOR_Y, LIP_Z - 0.012, LIP_Z, WHITE),  # its lip, 2 cm proud
        _box(front, front + BIN_DEPTH_BOTTOM, -BIN_Y, BIN_Y, BOTTOM + 0.002, BOTTOM + 0.008, BIN_GREY),  # bin floor
    ]
    step_h = (LIP_Z - BOTTOM) / BIN_STEPS
    for s in range(BIN_STEPS):
        z0, z1 = BOTTOM + s * step_h, BOTTOM + (s + 1) * step_h
        d = bin_depth(z0 + 1e-6)
        shapes.append(_box(front + d, front + d + WALL, -BIN_Y - WALL, BIN_Y + WALL, z0, z1, BIN_GREY))  # back
        if s:  # the ledge joining this step to the one below
            shapes.append(_box(front + bin_depth(z0 - 1e-6), front + d + WALL, -BIN_Y, BIN_Y, z0, z0 + WALL, BIN_GREY))
        for side in (-1, 1):  # the bin's sides, as deep as the back at this height
            y = side * (BIN_Y + WALL / 2)
            shapes.append(_box(front, front + d, y - WALL / 2, y + WALL / 2, z0, z1, BIN_GREY))
    door = Link("door", DOOR_KG, tuple(shapes))
    joints = [Joint("door", "revolute", "door", HINGE, "Y", -OPEN, 0.0, damping=0.05)]
    return mechanism("cabinet", (0.0, 0.0, TOP), base, (door,), joints, {**MATERIALS, "default": (0.8, 0.7)})


# -- the clogs ----------------------------------------------------------------------------------------------------------
# Crocs Classic Clog, men's US 9: about 198 g each (Zappos listing, "based on a single item"). The Crocs charts give
# 263 mm heel to toe for the foot; the clog's outside (29 x 10 x 9.5 cm, toe 6.5 cm, heel 8.5 cm wide) is an
# estimate until a pair is measured. Each clog is one rigid body of boxes, in its own frame (x toward the toe, z up,
# origin at the middle of its outside box): sole, toe box, middle, and a heel cup of three walls.
CLOG = (0.29, 0.10, 0.095)
CLOG_KG = 0.198
HEEL_WALL = 0.008
CLOG_PARTS = (  # (centre, size). The heel cup is open on top, as a clog's is: a back wall and two side walls.
    ((0.0, 0.0, -0.0325), (0.29, 0.10, 0.030)),  # sole
    ((0.080, 0.0, 0.0), (0.130, 0.095, 0.035)),  # toe box
    ((-0.020, 0.0, 0.0075), (0.070, 0.100, 0.050)),  # middle
    ((-0.145 + HEEL_WALL / 2, 0.0, 0.015), (HEEL_WALL, 0.085, 0.065)),  # heel cup: back wall
    ((-0.096, 0.0425 - HEEL_WALL / 2, 0.015), (0.082, HEEL_WALL, 0.065)),  # heel cup: side walls
    ((-0.096, -0.0425 + HEEL_WALL / 2, 0.015), (0.082, HEEL_WALL, 0.065)),
)
CLOG_X = 0.25  # the clogs' middles on the table
# WHY beside the cabinet, not in front of it: in front, the open door hangs over the heels and the hands can't reach
# them (official sim, 2026-11-20). At y +-0.33 they lie just outside the door's width.
CLOG_Y = 0.33
BIN_Y_AT = 0.11  # where each clog goes into the bin
CLOG_START = {"left_clog": (CLOG_X, CLOG_Y, TOP + CLOG[2] / 2 + 0.0005),
              "right_clog": (CLOG_X, -CLOG_Y, TOP + CLOG[2] / 2 + 0.0005)}


@dataclass(frozen=True)
class Clog(Box):
    """A clog: a loose box whose spawner builds the clog from its parts (the outside box itself doesn't collide)."""

    clog: bool = True


CABINET = _cabinet()
OBJECTS = [
    CABINET,
    *(Clog(name, pos, color=RED, mass=CLOG_KG, size=CLOG, yaw=math.pi) for name, pos in CLOG_START.items()),
]


# -- the simulator side: the clogs' parts, the plastic's friction, and the grippers ------------------------------------
def _clog_spawner():
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material, clone
    import isaaclab.sim as sim_utils
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    @clone
    def spawn_clog(prim_path, cfg, translation=None, orientation=None, **kwargs):
        prim = spawn_cuboid.__wrapped__(prim_path, cfg, translation, orientation, **kwargs)
        stage = prim.GetStage()
        # WHY every descendant, with the attributes created: the outside box's collider sits below the prim and its
        # collisionEnabled attribute isn't authored, so setting the bare attribute did nothing (official sim,
        # 2026-11-20: the fingers landed on the outside box's top).
        for p in Usd.PrimRange(prim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr().Set(False)
            if p != prim and p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        look = f"{prim_path}/clog_look"
        surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(RED), roughness=0.6)
        surface.func(look, surface)
        grip = f"{prim_path}/clog_grip"
        material = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0)
        material.func(grip, material)
        for i, (centre, size) in enumerate(CLOG_PARTS):
            cube = UsdGeom.Cube.Define(stage, f"{prim_path}/part_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            part = cube.GetPrim()
            UsdPhysics.CollisionAPI.Apply(part)
            physx = PhysxSchema.PhysxCollisionAPI.Apply(part)
            physx.CreateContactOffsetAttr(0.002)
            physx.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(part.GetPath()), grip, stage=stage)
            bind_visual_material(str(part.GetPath()), look, stage=stage)
        return prim

    return spawn_clog


def _install_plastic_friction() -> None:
    """Foam on polypropylene is slippery: the cabinet's plastic combines its friction with "min", so a clog slides down
    the tipped bin at 0.35 / 0.30 while the grippers keep 1.2 / 1.0 on it (the task-file format gives every loose
    object 1.2 / 1.0, which PhysX's default averaging would turn into 0.78 on the plastic)."""
    if getattr(mech, "_plastic_min", False):
        return
    plain = mech._spawn

    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        prim = plain(prim_path, cfg, translation, orientation, **kwargs)
        from pxr import PhysxSchema

        material = prim.GetStage().GetPrimAtPath(f"{prim_path}/Looks/physics_plastic")
        if material.IsValid():
            PhysxSchema.PhysxMaterialAPI.Apply(material).CreateFrictionCombineModeAttr().Set("min")
        return prim

    mech._spawn = spawn
    mech._plastic_min = True


_install_plastic_friction()


@dataclass(frozen=True)
class Pads:
    """One gripper after a control step (world frame, m): the span along its pointing axis from the pads' back to
    their tips, and points across the end of each of its two fingertips."""

    back: np.ndarray
    tips: np.ndarray
    fingertips: tuple = ()


PAD_BACK = 0.025
GRIPPERS = ("left_gripper", "right_gripper")
FINGERTIP_ENDS = {  # as in break-chocolate-row: the end of each fingertip in its link's frame (m)
    "tip_left": [(x, -0.04454, -0.09016) for x in (0.01799, 0.02399, 0.02999)],
    "tip_right": [(x, 0.04454, -0.09016) for x in (-0.02999, -0.02399, -0.01799)],
}


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


@dataclass(frozen=True)
class Stillness:
    """Whether each clog has stayed put over the last STILL_STEPS control steps: moved under STILL_MM and turned under
    STILL_DEG. WHY from positions rather than the reported velocity: a clog at rest in the bin kept reporting 87 mm/s
    and 0.9 rad/s, unchanged for 12 s while its position didn't move (official sim, 2026-11-20; a sleeping body's stale
    velocity)."""

    still: dict


STILL_STEPS, STILL_MM, STILL_DEG = 6, 1.0, 1.0
_HISTORY: dict = {}


def _clear_history() -> None:
    _HISTORY.clear()


mech.on_reset(_clear_history)


def _quat_angle_deg(a, b) -> float:
    d = abs(float(np.dot(np.asarray(a, float), np.asarray(b, float))))
    return math.degrees(2 * math.acos(min(1.0, d)))


def _record_stillness(states) -> Stillness:
    out = {}
    for name in CLOG_START:
        c = states.get(name)
        if c is None:
            continue
        h = _HISTORY.setdefault(name, [])
        h.append((np.asarray(c.pos, float).copy(), np.asarray(c.quat_xyzw, float).copy()))
        del h[:-STILL_STEPS]
        out[name] = len(h) == STILL_STEPS and all(
            np.linalg.norm(p - h[-1][0]) * 1000 < STILL_MM and _quat_angle_deg(q, h[-1][1]) < STILL_DEG for p, q in h)
    return Stillness(out)


def still(objects, name) -> bool:
    record = objects.get("stillness")
    return record.still.get(name, False) if record is not None else objects[name].resting()


def _patch_scene(scene) -> None:
    if getattr(scene, "_store_shoes_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        obj = stock_object(spec)
        if not getattr(spec, "clog", False):
            return obj
        # WHY a new object rather than swapping the spawner on the stock one: Arena reads the spawner when it builds the
        # object, so a func set afterwards never ran (official sim, 2026-11-20: the clogs came out as plain boxes).
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose

        cfg = obj.spawner_cfg.replace(func=_clog_spawner())
        return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                      initial_pose=Pose(position_xyz=tuple(spec.pos),
                                        rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))

    def object_states(env, names):
        states = {**stock_states(env, names), **_gripper_states(env)}
        return {**states, "stillness": _record_stillness(states)}

    scene.task_object, scene.object_states = task_object, object_states
    scene._store_shoes_patched = True


# WHY through mechanism's own import hook rather than a second one: two hooks that each look the module up through
# the other finders called each other without end (copy-id-card, official sim, 2026-11-20).
_mech_scene_patch = mech._PATCHES["yam_arena.scene"]


def _scene_patches(scene) -> None:
    _mech_scene_patch(scene)
    _patch_scene(scene)


mech._PATCHES["yam_arena.scene"] = _scene_patches
if "yam_arena.scene" in sys.modules:
    _scene_patches(sys.modules["yam_arena.scene"])


# -- scoring: a function of the object states (the clogs, the door's angle, the grippers) ------------------------------
SHUT_DEG = 2.0
NEARLY_SHUT_DEG = 15.0
LIP_TOL = 0.005  # a clog may stand this far above the bin's lip (the tray's floor is 6 mm above it)
CLEAR = 0.03  # hands off: no point along either gripper's pads or fingertips within 3 cm of the cabinet's front face
COMPLETE_SCORE = 0.9  # score() for a finished task; 1.0 comes only from success() holding 1 s, which ends the trial


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def clog_corners(clog) -> np.ndarray:
    """The corners of a clog's parts in the world, from its pose and true size."""
    r, p = _rot(clog.quat_xyzw), np.asarray(clog.pos, float)
    pts = []
    for centre, size in CLOG_PARTS:
        c, h = np.asarray(centre), np.asarray(size) / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    pts.append(p + r @ (c + h * np.array([sx, sy, sz])))
    return np.array(pts)


def door_open_rad(objects) -> float:
    return -float(objects["cabinet"].joints["door"])


def to_door_frame(points: np.ndarray, open_rad: float) -> np.ndarray:
    """World points as they would be with the door shut: turned back about the hinge by the door's opening (table
    heights, world x and y)."""
    hinge = np.array([HINGE[0], HINGE[1], TOP + HINGE[2]])
    back = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), open_rad)  # the door opens by -open_rad about +y
    local = (points - hinge) @ back.T + hinge
    local[:, 2] -= TOP
    return local


def in_bin(clog, objects) -> bool:
    """Every corner of the clog inside the bin: behind the door, in front of the bin's back at its height, between the
    bin's sides, above its floor and no higher than its lip."""
    pts = to_door_frame(clog_corners(clog), door_open_rad(objects))
    front = X0 + DOOR_T - 0.003
    for x, y, z in pts:
        if not (BOTTOM - 0.003 <= z <= LIP_Z + LIP_TOL and abs(y) <= BIN_Y + 0.003):
            return False
        if not (front <= x <= front + 0.003 + bin_depth(min(max(z, BOTTOM), LIP_Z - 1e-6)) + 0.003):
            return False
    return True


def _touches(pads: Pads) -> bool:
    lo = np.array([X0 - LIP_OUT - CLEAR, -CAB_W / 2 - CLEAR, TOP])
    hi = np.array([X0 + CAB_D + CLEAR, CAB_W / 2 + CLEAR, TOP + CAB_H + CLEAR])
    along = [pads.back + s * (pads.tips - pads.back) for s in np.linspace(0.0, 1.0, 11)]
    return any(bool(np.all(p >= lo) and np.all(p <= hi)) for p in (*along, *(np.asarray(q, float) for q in pads.fingertips)))


def hands_off(objects) -> bool:
    pads = [objects.get(g) for g in GRIPPERS]
    return all(p is not None for p in pads) and not any(_touches(p) for p in pads)


def grade(objects) -> float:
    """1.0: both clogs in the bin, the door within 2 degrees of shut, both grippers 3 cm clear of the cabinet, the clogs
    still (each moved under 1 mm and turned under 1 degree over the last 6 control steps). 0.75: both in, the door within 15 degrees of shut (or shut with a hand still near). 0.5: both in, the door
    open. 0.25: one in. 0: neither."""
    clogs = [objects[n] for n in CLOG_START]
    inside = sum(in_bin(c, objects) for c in clogs)
    if inside == 2:
        open_deg = math.degrees(door_open_rad(objects))
        if open_deg <= SHUT_DEG and hands_off(objects) and all(still(objects, n) for n in CLOG_START):
            return 1.0
        if open_deg <= NEARLY_SHUT_DEG:
            return 0.75
        return 0.5
    return 0.25 if inside == 1 else 0.0


def _debug(objects, g) -> None:
    """With SHOES_DEBUG=path set, one line per control step."""
    path = os.environ.get("SHOES_DEBUG")
    if not path:
        return
    parts = [f"grade={g} door_deg={math.degrees(door_open_rad(objects)):.1f} hands_off={hands_off(objects)}"]
    for n in CLOG_START:
        c = objects[n]
        pts = to_door_frame(clog_corners(c), door_open_rad(objects))
        p = np.asarray(c.pos) * 1000
        parts.append(f"{n}=({p[0]:.0f},{p[1]:.0f},{p[2] - TOP * 1000:.0f}) in={in_bin(c, objects)} "
                     f"still={still(objects, n)} v_mm_s={np.linalg.norm(c.lin_vel) * 1000:.0f} "
                     f"top_mm={pts[:, 2].max() * 1000:.0f} low_mm={pts[:, 2].min() * 1000:.0f} "
                     f"back_mm={(pts[:, 0].max() - X0) * 1000:.0f}")
    for g_ in GRIPPERS:
        pads = objects.get(g_)
        if pads is not None:
            axis = (pads.tips - pads.back) / np.linalg.norm(pads.tips - pads.back)
            reach = max(float((np.asarray(q) - pads.tips) @ axis) for q in pads.fingertips) if pads.fingertips else 0.0
            ends = np.array(pads.fingertips) if pads.fingertips else np.zeros((1, 3))
            parts.append(f"{g_[0]}=({','.join(f'{v * 1000:.0f}' for v in pads.tips)}) ends_past_tips_mm={reach * 1000:.0f} "
                         f"ends_lo=({','.join(f'{v * 1000:.0f}' for v in ends.min(axis=0))})")
    with open(path, "a") as f:
        f.write(" ".join(parts) + "\n")


def score(objects) -> float:
    g = grade(objects)
    _debug(objects, g)
    return min(g, COMPLETE_SCORE)


def success(objects) -> bool:
    return grade(objects) == 1.0


# -- the scripted demo --------------------------------------------------------------------------------------------------
def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle (yam_arena.demo builds each waypoint's
    rotation with grip_rotation(yaw), which this wraps)."""
    if getattr(yam_demo, "_matrix_grips", False):
        return
    plain = yam_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    yam_demo.grip_rotation = grip_rotation
    yam_demo._matrix_grips = True


_allow_matrix_grips()

Y_AXIS = np.array([0.0, 1.0, 0.0])
DOWN = yam_demo.grip_rotation(0.0)
LEVEL = axis_angle_matrix(Y_AXIS, -math.pi / 2) @ DOWN  # pointing at the cabinet (+x), fingers closing along y
TILT = axis_angle_matrix(Y_AXIS, -math.pi / 4) @ DOWN  # pointing down and toward the cabinet
X_AXIS, Z_AXIS = np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
# Pinching a clog's heel back wall: fingers closing along x (one inside the heel cup, one behind it), then tipped
# toe-down about y in two steps.
DOWN_X = axis_angle_matrix(Z_AXIS, math.pi / 2) @ DOWN
TILT_X = axis_angle_matrix(Y_AXIS, -math.pi / 4) @ DOWN_X
LEVEL_X = axis_angle_matrix(Y_AXIS, -math.pi / 2) @ DOWN_X
# Pinching the door's lip: level, rolled a quarter turn so the fingers close vertically on the lip.
LIP_ROLL = float(os.environ.get("SHOES_LIP_ROLL", "1")) * math.pi / 2
LIP_PINCH = axis_angle_matrix(X_AXIS, LIP_ROLL) @ LEVEL


def _pt(*v) -> tuple:
    return tuple(float(a) for a in v)


def down_tcp(x, y, tips_z) -> tuple:
    return _pt(x, y, tips_z + PAD_TIP_PAST_TCP)


HINGE_W = np.array([HINGE[0], HINGE[1], TOP + HINGE[2]])


def on_door(point, deg: float) -> tuple:
    """A point fixed to the door (given with it shut) carried round the hinge with the door opened ``deg``."""
    p = np.asarray(point, float)
    return _pt(*(HINGE_W + axis_angle_matrix(Y_AXIS, -math.radians(deg)) @ (p - HINGE_W)))


# Opening: the lip pinched between the fingertips, carried round the hinge's arc with the grip turning as the door
# does, let go past the door's balance; it falls open to its stop.
LIP_PULL_Y = float(os.environ.get("SHOES_LIP_Y", "0.235"))  # near the lip's end, to slip off it sideways
PINCH_LIP_TCP = np.array([X0 - PAD_TIP_PAST_TCP - 0.002, LIP_PULL_Y, TOP + LIP_Z - 0.006])  # pad tips at the door face
LET_GO_DEG = float(os.environ.get("SHOES_LET_GO_DEG", "24"))


def lip_grip(deg: float) -> np.ndarray:
    return axis_angle_matrix(Y_AXIS, -math.radians(deg)) @ LIP_PINCH


def _open_door(side: float) -> list:
    y = side * LIP_PULL_Y
    at = PINCH_LIP_TCP * np.array([1.0, side, 1.0])
    before = _pt(at[0] - 0.05, y, at[2])
    arc = [Waypoint(0.35, on_door(at, d), FINGER_CLOSED, f"door {d:g} degrees", lip_grip(d))
           for d in np.arange(3.0, LET_GO_DEG + 0.1, 3.0)]
    last = on_door(at, arc and float(np.arange(3.0, LET_GO_DEG + 0.1, 3.0)[-1]) or 0.0)
    g = lip_grip(float(np.arange(3.0, LET_GO_DEG + 0.1, 3.0)[-1]))
    return [
        Waypoint(1.5, _pt(before[0], y, before[2] + 0.04), FINGER_OPEN, "in front of the lip", LIP_PINCH),
        Waypoint(0.8, before, FINGER_OPEN, "level with the lip", LIP_PINCH),
        Waypoint(0.8, _pt(*at), FINGER_OPEN, "fingers astride the lip", LIP_PINCH),
        Waypoint(0.6, _pt(*at), FINGER_CLOSED, "pinch the lip", LIP_PINCH),
        *arc,
        Waypoint(0.3, last, FINGER_OPEN, "let go", g),
        # WHY sideways, off the lip's end: lifted straight up, the lower finger pushed the lip back up and the door shut
        # again (official sim, 2026-11-20).
        Waypoint(0.4, _pt(last[0], side * (CAB_W / 2 + 0.05), last[2]), FINGER_OPEN, "off the lip's end: it falls open", g),
        Waypoint(0.6, _pt(last[0] - 0.03, side * (CAB_W / 2 + 0.05), last[2] + 0.06), FINGER_OPEN, "up", g),
    ]


# Placing: each hand pinches its clog's heel cup across its width from above, lifts it, tips it toe-down and lowers it
# into the bin. WHY across the cup, not on its back wall: on the 8 mm wall the fingers closed beside it (official sim,
# 2026-11-20); the 8.5 cm cup fits inside the fingers' 9.4 cm opening.
HEEL_LOCAL_X = -0.096  # the heel cup's middle along the clog
PINCH_Z = TOP + CLOG[2] - 0.022  # pad tips this far down the heel cup's sides
DROP_X = float(os.environ.get("SHOES_DROP_X", "0.27"))
TOE_ON_Z = float(os.environ.get("SHOES_TOE_ON_Z", "0.47"))
CARRY_Z = 0.56  # a hanging clog's toe clears the open door at this hand height
ACROSS_X = 0.18  # in front of the open lip, where nothing stands this high
RELEASE_Z = float(os.environ.get("SHOES_RELEASE_Z", "0.40"))


def _place(name: str, variant: str) -> list:
    cx, cy, _ = CLOG_START[name]
    by = math.copysign(BIN_Y_AT, cy)
    # The clog lies toe toward the arms (yaw pi), so its heel cup is ahead of its middle.
    heel_x = cx - HEEL_LOCAL_X
    grab_x = heel_x
    above = down_tcp(grab_x, cy, PINCH_Z + 0.07)
    pinch = down_tcp(grab_x, cy, PINCH_Z)
    # WHY turned in three steps: straight down reaches only up to about 20 cm above the table here, and a level grip
    # only from about 40 cm, where the hanging clog clears the table; the 45-degree grip bridges the two (offline IK,
    # 2026-11-20).
    if variant == "flat":  # a probe: carried flat, as picked up, and dropped over the bin
        return [
            Waypoint(1.5, above, FINGER_OPEN, f"over {name}'s heel", DOWN),
            Waypoint(0.8, pinch, FINGER_OPEN, "down astride the heel cup", DOWN),
            Waypoint(0.6, pinch, FINGER_CLOSED, "pinch", DOWN),
            Waypoint(0.8, _pt(pinch[0], cy, TOP + 0.20), FINGER_CLOSED, "lift it", DOWN),
            Waypoint(1.5, _pt(DROP_X, by, TOP + 0.35), FINGER_CLOSED, "over the bin, flat", TILT),
            Waypoint(0.5, _pt(DROP_X, by, TOP + 0.35), FINGER_OPEN, "drop it", TILT),
            Waypoint(0.8, _pt(DROP_X - 0.08, by, TOP + 0.38), FINGER_OPEN, "out and up", TILT),
        ]
    return [
        Waypoint(1.5, above, FINGER_OPEN, f"over {name}'s heel", DOWN),
        Waypoint(0.8, pinch, FINGER_OPEN, "down astride the heel cup", DOWN),
        Waypoint(0.6, pinch, FINGER_CLOSED, "pinch", DOWN),
        Waypoint(0.8, _pt(pinch[0], cy, TOP + 0.19), FINGER_CLOSED, "lift it", DOWN),
        Waypoint(1.0, _pt(0.30, cy, TOP + 0.30), FINGER_CLOSED, "tip it toe-down halfway", TILT),
        Waypoint(1.0, _pt(0.29, cy, TOP + 0.46), FINGER_CLOSED, "toe-down", LEVEL),
        Waypoint(0.8, _pt(0.29, cy, TOP + CARRY_Z), FINGER_CLOSED, "up", LEVEL),
        Waypoint(0.8, _pt(ACROSS_X, cy, TOP + CARRY_Z), FINGER_CLOSED, "toward the arms", LEVEL),
        # WHY across in front of the lip first: crossing over the open bin, the hanging toe caught the bin's side wall
        # and the hand pushed the door shut (official sim, 2026-11-20).
        Waypoint(1.5, _pt(ACROSS_X, by, TOP + CARRY_Z), FINGER_CLOSED, "across, in front of the open lip", LEVEL),
        Waypoint(0.8, _pt(DROP_X, by, TOP + CARRY_Z), FINGER_CLOSED, "in over the opening", LEVEL),
        # WHY lowered onto the door and slid in, not dropped: pinched across its heel cup the clog swings freely about
        # the fingers' axis and hangs straight down; dropped that way it lands on the lip and falls out, and a tilted
        # grip can't hold it tilted (official sim, 2026-11-20). With its toe on the tipped door and the hand moving in
        # and down along the bin, the toe slides down the door and the clog swings into line with the bin.
        Waypoint(1.2, _pt(DROP_X, by, TOP + TOE_ON_Z), FINGER_CLOSED, "toe onto the door", LEVEL),
        Waypoint(0.8, _pt(DROP_X + 0.03, by, TOP + TOE_ON_Z - 0.035), FINGER_CLOSED, "slide it in along the door", LEVEL),
        Waypoint(0.8, _pt(DROP_X + 0.06, by, TOP + RELEASE_Z), FINGER_CLOSED, "further in", LEVEL),
        Waypoint(0.4, _pt(DROP_X + 0.06, by, TOP + RELEASE_Z), FINGER_OPEN, "let go", LEVEL),
        Waypoint(1.0, _pt(DROP_X + 0.03, by, TOP + CARRY_Z), FINGER_OPEN, "up and out", LEVEL),
        Waypoint(0.8, _pt(DROP_X, by, TOP + CARRY_Z), FINGER_OPEN, "untwist", LEVEL),
        Waypoint(1.0, _pt(0.24, by, TOP + 0.34), FINGER_OPEN, "down and back", TILT),
    ]


# Shutting: the right hand pushes the lip back round the arc until the door tips past its balance and falls shut.
def _shut_door() -> list:
    y = -LIP_PULL_Y
    face = np.array([X0 - LIP_OUT - 0.008 - PAD_TIP_PAST_TCP, y, TOP + LIP_Z - 0.006])  # TCP just in front of the lip
    arc = [Waypoint(0.35, on_door(face, d), FINGER_CLOSED, f"push to {d:g} degrees", LEVEL)
           for d in (OPEN_DEG - 2, 30, 22, 14, 8, 3, 0)]
    return [
        Waypoint(1.2, _pt(*(np.asarray(on_door(face, OPEN_DEG)) + np.array([-0.012, 0, 0.04]))), FINGER_CLOSED,
                 "in front of the open lip", LEVEL),
        *arc,
        Waypoint(0.8, _pt(face[0] - 0.05, y, face[2] + 0.02), FINGER_CLOSED, "away from the door", LEVEL),
    ]


PARK_LEFT, PARK_RIGHT = _pt(0.16, 0.33, TOP + 0.20), _pt(0.16, -0.33, TOP + 0.20)


def _left(variant: str) -> list:
    return [
        *_open_door(+1.0),
        Waypoint(1.0, _pt(0.25, 0.36, TOP + 0.22), FINGER_OPEN, "clear", DOWN),
        *_place("left_clog", variant),
        Waypoint(1.5, PARK_LEFT, FINGER_OPEN, "park", DOWN),
        Waypoint(12.0, PARK_LEFT, FINGER_OPEN, "stay", DOWN),
    ]


def _right(variant: str) -> list:
    wait = sum(w.duration for w in _open_door(+1.0)) + 1.0
    return [
        Waypoint(wait, PARK_RIGHT, FINGER_OPEN, "wait for the door", DOWN),
        *_place("right_clog", variant),
        Waypoint(2.0, _pt(0.22, -0.20, TOP + 0.40), FINGER_CLOSED, "wait, up", LEVEL),
        *_shut_door(),
        Waypoint(1.5, PARK_RIGHT, FINGER_OPEN, "park", DOWN),
        Waypoint(6.0, PARK_RIGHT, FINGER_OPEN, "stay", DOWN),
    ]


def demo():
    """SHOES_DEMO=flat drops both clogs in as picked up, lying flat (a probe: the door can't shut on them)."""
    variant = os.environ.get("SHOES_DEMO", "")
    return {"left": _left(variant), "right": _right(variant)}
