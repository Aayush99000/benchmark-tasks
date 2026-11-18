"""swabkit: a transport tube, a flocked swab with a moulded breakpoint, and a waste cup, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from swabkit import Cup, SwabHandle, SwabHead, Tube, on_reset, on_step

- :class:`Tube` is a loose, free-standing 16 x 100 mm transport tube with its cap off (``pos`` is the middle of its
  base). Its skirted base is solid up to the inside floor; its wall is a ring of flat pieces. The medium in it is
  drawn, not simulated; its weight is part of the tube's.
- :class:`SwabHead` and :class:`SwabHandle` are the two lengths of one swab: from its flocked tip up to the moulded
  breakpoint, and from the breakpoint to its end. Each is a loose body with its length along its own z (``pos`` is the
  tip for the head, the breakpoint for the handle). The kit joins them at the breakpoint with a joint that lets go for
  good once the bending moment or the pull across it passes ``BREAK_TORQUE`` or ``BREAK_FORCE``: the shaft snaps. The
  joint is made whole again at every reset.
- :class:`Cup` is a loose, open waste cup (``pos`` the middle of its base, ``size`` its inside diameter twice and its
  height).

``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``, a passive breakable joint re-armed on reset). Pure
Python at import time; Isaac Sim is reached only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Cup", "KitState", "SwabHandle", "SwabHead", "Tube", "env", "finger_points", "gripper_points", "install",
           "joined", "on_reset", "on_step", "segment_distance", "to_local", "turn_y", "write_pose"]

# -- a 16 x 100 mm skirted transport tube, cap off (m) ----------------------------------------------------------------
TUBE_OD, TUBE_H = 0.016, 0.100
TUBE_WALL = 0.00125
TUBE_ID = TUBE_OD - 2 * TUBE_WALL
TUBE_FLOOR = 0.012                      # the inside floor, this far above the table (the skirt is below it)
MEDIUM_DEEP = 0.021                     # 3 mL of medium stands this deep
TUBE_MASS = 0.010                       # the tube (6 g) with its medium and beads
TUBE_FRICTION = (0.5, 0.4)

# -- a regular flocked swab with a moulded breakpoint 80 mm from its tip (m) -------------------------------------------
SWAB_L = 0.1505                         # tip to end
BREAK_AT = 0.080                        # the breakpoint, from the tip
HANDLE_L = SWAB_L - BREAK_AT
SHAFT_R = 0.00125                       # a 2.5 mm shaft
TIP_R, TIP_L = 0.00275, 0.016           # the flocked tip
HEAD_MASS, HANDLE_MASS = 0.0006, 0.0005
SWAB_FRICTION = (0.6, 0.5)
# The breakpoint, as a joint that lets go: the bending moment a 1.5 mm polystyrene neck carries at about 90 MPa
# (pi d^3 / 32 times the stress), and its pull at about 35 MPa. Estimates from the material, not measurements.
BREAK_TORQUE = 0.030                    # N m
BREAK_FORCE = 60.0                      # N
# The two lengths count as one piece within these. A whole swab's joint gives a little under load (two half-gram
# lengths, one pinched and lifted while the other drags in the tube), so the limits sit well outside that give; a
# snapped swab passes them within a control step, because its lower length drops free.
JOINED_SHIFT, JOINED_ANGLE = 0.004, math.radians(12.0)

PALM_BACK = 0.065                       # a gripper's palm, this far back from its grasp point
FINGER_THICK = 0.012                    # a finger's outer face, this far out from its pad


class _Part(Box):
    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class Tube(_Part):
    """A loose transport tube standing on its skirt, cap off. ``medium`` is the colour of the liquid drawn in it."""

    mass: float | None = TUBE_MASS
    medium: tuple = (0.93, 0.42, 0.30)
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class SwabHead(_Part):
    """The swab from its tip (``pos``) up to the breakpoint; ``quat`` (xyzw) turns its own z (tip to breakpoint) into
    the world. ``handle`` names the :class:`SwabHandle` it is joined to."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = HEAD_MASS
    handle: str = "swab_handle"


@dataclass(frozen=True)
class SwabHandle(_Part):
    """The swab from the breakpoint (``pos``) to its end; ``quat`` (xyzw) turns its own z into the world."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = HANDLE_MASS


@dataclass(frozen=True)
class Cup(_Part):
    """A loose open cup. ``size``: inside diameter, inside diameter, height."""

    mass: float | None = 0.02
    wall: float = 0.002
    friction: tuple = (0.6, 0.5)


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``pos`` is its own origin (see each class)."""

    joints: dict = field(default_factory=dict)


# -- geometry (pure) -------------------------------------------------------------------------------------------------

def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat(rot) -> tuple:
    """The quaternion (xyzw) of a rotation matrix."""
    m = np.asarray(rot, float)
    t = float(np.trace(m))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i], q[j], q[k], q[3] = s / 4, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s, (m[k, j] - m[j, k]) / s
    return tuple(q)


def turn_y(angle: float) -> tuple:
    """The quaternion (xyzw) of a turn by ``angle`` about y (it leans an upright part toward +x)."""
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


def to_local(point, pos, quat) -> np.ndarray:
    """``point`` (world) in the frame of a body at ``pos``, ``quat``."""
    return _rotation(quat).T @ (np.asarray(point, float) - np.asarray(pos, float))


def segment_distance(p, a, b) -> float:
    """The distance from point ``p`` to the straight piece from ``a`` to ``b``."""
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def apart(head_pos, head_quat, handle_pos, handle_quat) -> tuple:
    """How far the handle's lower end is from the head's upper end (m), and how far out of line the two are (rad)."""
    r0, r1 = _rotation(head_quat), _rotation(handle_quat)
    shift = float(np.linalg.norm(np.asarray(head_pos, float) + r0 @ np.array([0.0, 0.0, BREAK_AT]) - np.asarray(handle_pos, float)))
    angle = math.acos(float(np.clip((np.trace(r0.T @ r1) - 1.0) / 2.0, -1.0, 1.0)))
    return shift, angle


def joined(head_pos, head_quat, handle_pos, handle_quat) -> bool:
    """The swab is still one piece: the handle's lower end is at the head's upper end and the two are in line."""
    shift, angle = apart(head_pos, head_quat, handle_pos, handle_quat)
    return shift <= JOINED_SHIFT and angle <= JOINED_ANGLE


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


# -- authoring helpers -----------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.5, metal=0.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def glass(self, color, ior=1.49, thin=True) -> str:
        """A see-through look (the real-time renderer ignores a plain surface's opacity)."""
        key = ("glass", tuple(color), ior)
        if key not in self.looks:
            path = f"{self.path}/Looks/g{len(self.looks)}"
            try:
                vis = self.sim_utils.GlassMdlCfg(glass_color=tuple(float(c) for c in color), frosting_roughness=0.15,
                                                 thin_walled=thin, glass_ior=ior)
                vis.func(path, vis)
            except Exception as err:                                          # noqa: BLE001
                print(f"[swabkit] glass look unavailable ({err!r}); drawn as a plain surface", file=sys.stderr, flush=True)
                from yam_arena.station import linear_rgb

                vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.3)
                vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, mass, lin_damp=0.05, ang_damp=0.1, iterations=None):
        from pxr import PhysxSchema, UsdPhysics

        prim = self.UsdGeom.Xform.Define(self.stage, path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        if iterations:
            rb.CreateSolverPositionIterationCountAttr(int(iterations))
            rb.CreateSolverVelocityIterationCountAttr(4)
        return prim

    def collide(self, prim, mat, offset=0.0008):
        from isaaclab.sim.utils import bind_physics_material
        from pxr import PhysxSchema, UsdPhysics

        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)

    def paint(self, prim, look):
        from isaaclab.sim.utils import bind_visual_material

        bind_visual_material(str(prim.GetPath()), look, stage=self.stage)

    def _finish(self, prim, look, mat, offset=0.0008):
        if look:
            self.paint(prim, look)
        else:
            self.UsdGeom.Imageable(prim).MakeInvisible()
        if mat:
            self.collide(prim, mat, offset)
        return prim

    def box(self, path, centre, size, look, mat=None, yaw=0.0, offset=0.0008):
        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if yaw:
            cube.AddOrientOp().Set(self.Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        return self._finish(cube.GetPrim(), look, mat, offset)

    def cyl(self, path, r, z0, z1, look, mat=None, offset=0.0008):
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(z1 - z0))
        c.AddTranslateOp().Set(self.Gf.Vec3d(0.0, 0.0, float(z0 + z1) / 2))
        return self._finish(c.GetPrim(), look, mat, offset)

    def ring(self, stem, r_in, wall, z0, z1, look, mat, pieces=16):
        """A round wall from flat pieces, each turned to face the axis."""
        width = 2 * (r_in + wall) * math.tan(math.pi / pieces) + 0.0002
        for k in range(pieces):
            t = 2 * math.pi * k / pieces
            self.box(f"{stem}_{k}", ((r_in + wall / 2) * math.cos(t), (r_in + wall / 2) * math.sin(t), (z0 + z1) / 2),
                     (wall, width, z1 - z0), look, mat, yaw=t)


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned
_JOINTS: dict = {}                     # breakpoint joint path -> (head name, handle name)


def _spawn_tube(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A solid skirted base, a see-through wall and the medium drawn inside; one loose body with its base at the origin."""
    spec: Tube = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass), iterations=16)
    m = a.mat("tube", *TUBE_FRICTION)
    white = a.look((0.94, 0.94, 0.92), rough=0.5)
    a.cyl(f"{prim_path}/skirt", TUBE_OD / 2, 0.0, TUBE_FLOOR, white, m)
    a.ring(f"{prim_path}/wall", TUBE_ID / 2, TUBE_WALL, TUBE_FLOOR, TUBE_H, a.glass((0.93, 0.95, 0.96)), m)
    a.cyl(f"{prim_path}/medium", TUBE_ID / 2 - 0.0003, TUBE_FLOOR, TUBE_FLOOR + MEDIUM_DEEP, a.look(spec.medium, rough=0.2))
    a.cyl(f"{prim_path}/thread", TUBE_OD / 2 + 0.0004, TUBE_H - 0.011, TUBE_H - 0.002, white)       # the screw thread's band
    return a.root


def _joint(head_path: str, handle_path: str, local1=(0.0, 0.0, 0.0), quat1=(0.0, 0.0, 0.0, 1.0)) -> str:
    """Author the breakpoint between the two lengths: disabled first, its frames set, enabled last. ``local1`` and
    ``quat1`` place the head's upper end in the handle's frame (identity when the two were spawned in line)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    path = f"{head_path}/breakpoint"
    joint = UsdPhysics.FixedJoint.Define(get_current_stage(), path)
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([head_path])
    joint.CreateBody1Rel().SetTargets([handle_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, BREAK_AT))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*(float(v) for v in local1)))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(float(quat1[3]), Gf.Vec3f(*(float(v) for v in quat1[:3]))))
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBreakForceAttr().Set(BREAK_FORCE)
    joint.CreateBreakTorqueAttr().Set(BREAK_TORQUE)
    joint.CreateJointEnabledAttr().Set(True)       # last write, after the frames
    return path


def _join_if_both(head_name: str) -> None:
    head: SwabHead = _SPECS[head_name]
    if head_name in _PATHS and head.handle in _PATHS:
        _JOINTS[_joint(_PATHS[head_name], _PATHS[head.handle])] = (head_name, head.handle)


def _spawn_head(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The flocked tip and the shaft up to the breakpoint: tip at the origin, length along z."""
    spec: SwabHead = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass), iterations=32)
    m = a.mat("swab", *SWAB_FRICTION)
    a.cyl(f"{prim_path}/tip", TIP_R, 0.0, TIP_L, a.look((0.97, 0.97, 0.95), rough=0.95), m)
    a.cyl(f"{prim_path}/shaft", SHAFT_R, TIP_L, BREAK_AT, a.look((0.96, 0.96, 0.96), rough=0.4), m)
    _join_if_both(spec.name)
    return a.root


def _spawn_handle(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The shaft from the breakpoint to its end: breakpoint at the origin, length along z. A coloured ring marks the
    breakpoint, as on the real swab."""
    spec: SwabHandle = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass), iterations=32)
    m = a.mat("swab", *SWAB_FRICTION)
    a.cyl(f"{prim_path}/shaft", SHAFT_R, 0.0, HANDLE_L, a.look((0.96, 0.96, 0.96), rough=0.4), m)
    a.cyl(f"{prim_path}/mark", SHAFT_R + 0.0002, 0.0005, 0.0025, a.look((0.85, 0.15, 0.15), rough=0.5))
    for name, spec0 in _SPECS.items():
        if isinstance(spec0, SwabHead) and spec0.handle == spec.name:
            _join_if_both(name)
    return a.root


def _spawn_cup(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A round open cup: a floor disc and a ring of flat wall pieces, one loose body with its base at the origin."""
    spec: Cup = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass))
    m = a.mat("cup", *spec.friction)
    look = a.look(spec.color, rough=0.5)
    r_in, h, w = float(spec.size[0]) / 2, float(spec.size[2]), float(spec.wall)
    a.cyl(f"{prim_path}/floor", r_in + w, 0.0, w, look, m)
    a.ring(f"{prim_path}/wall", r_in, w, 0.0, h, look, m, pieces=20)
    return a.root


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    _PATHS.pop(spec.name, None)
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


_SPAWNERS = {Tube: _spawn_tube, SwabHead: _spawn_head, SwabHandle: _spawn_handle, Cup: _spawn_cup}


# -- run time ----------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def write_pose(environment, name: str, pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    asset = environment.unwrapped.scene[name]
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + _origin(environment)), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def _pose(environment, name: str):
    pose = _np(environment.unwrapped.scene[name].data.root_link_pose_w)[0]
    return pose[:3], pose[3:7]


def _disarm() -> None:
    """Switch every breakpoint joint off (before a reset puts the two lengths back)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    stage = get_current_stage()
    for path in _JOINTS:
        joint = UsdPhysics.FixedJoint.Get(stage, path)
        if joint:
            joint.CreateJointEnabledAttr().Set(False)


def _rearm(environment) -> None:
    """Make every breakpoint whole again where the reset left the two lengths: the joint is authored afresh, its
    frames taken from the bodies' poses, and enabled last."""
    for head, handle in list(_JOINTS.values()):
        p0, q0 = _pose(environment, head)
        p1, q1 = _pose(environment, handle)
        r0, r1 = _rotation(q0), _rotation(q1)
        anchor = p0 + r0 @ np.array([0.0, 0.0, BREAK_AT])
        _joint(_PATHS[head], _PATHS[handle], local1=r1.T @ (anchor - p1), quat1=_quat(r1.T @ r0))


def _box_bounds(pos, rot, lo, hi):
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ rot.T + pos
    return pts.min(0), pts.max(0)


def _extent(spec) -> tuple:
    if isinstance(spec, Tube):
        return (TUBE_OD / 2, TUBE_H)
    if isinstance(spec, SwabHead):
        return (TIP_R, BREAK_AT)
    if isinstance(spec, SwabHandle):
        return (SHAFT_R, HANDLE_L)
    return (float(spec.size[0]) / 2 + float(spec.wall), float(spec.size[2]))


def _kit_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    r, h = _extent(spec)
    low, high = _box_bounds(pos, _rotation(quat), (-r, -r, 0.0), (r, r, h))
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=low, high=high)


def finger_points(environment) -> list:
    """The grasp point of each gripper, left then right (world, relative to the env origin)."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


def gripper_points(environment) -> list:
    """Points on each gripper that could touch something held between or beside its fingers, left gripper then right
    (world, relative to the env origin): along both fingers' inner and outer faces, from their tips back to the palm,
    and the palm's middle. The fingers' opening is read from their joints."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        rot = _rotation(pose[3:7])
        tcp = pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin
        q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        pts = [tcp - PALM_BACK * rot[:, 2]]
        # joint7 carries the finger on the gripper's -x side, joint8 the one on its +x side; each pad is |q| from the middle
        for side, joint in ((-1.0, "joint7"), (1.0, "joint8")):
            half = abs(float(q[names.index(joint)]))
            for depth in (PAD_TIP_PAST_TCP, 0.0, -PALM_BACK / 2, -PALM_BACK):
                for face in (0.0, FINGER_THICK):
                    pts.append(tcp + side * (half + face) * rot[:, 0] + depth * rot[:, 2])
        out.append(pts)
    return out


# -- hooking yam_arena -------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_swabkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        for kind, spawn in _SPAWNERS.items():
            if isinstance(spec, kind):
                return _rigid_object(spec, spawn)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), tuple(_SPAWNERS))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._swabkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_swabkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        # The breakpoint has to be whole before the reset's settling steps and first picture: switch the joint off,
        # let the environment put both lengths back, then author the joint afresh from where they are.
        base, saved, had = None, None, False
        try:
            self._ensure_sim(None)
            base = self._env.unwrapped
            env_reset = base.reset
            saved, had = vars(base).get("reset"), "reset" in vars(base)

            def reset_then_join(*aa, **kk):
                _disarm()
                result = env_reset(*aa, **kk)
                _rearm(self._env)
                return result

            base.reset = reset_then_join
        except Exception as err:                     # noqa: BLE001
            print(f"[swabkit] breakpoint not re-armed around the reset: {err!r}", file=sys.stderr, flush=True)
            base = None
        try:
            obs = stock_reset(self, *a, **k)
        finally:
            if base is not None:
                if had:
                    base.reset = saved
                else:
                    del base.reset
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._swabkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
