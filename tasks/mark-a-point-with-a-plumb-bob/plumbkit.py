"""plumbkit: a plumb bob on its line, a carpenter's pencil, pencil dots and fixed boards, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from plumbkit import Bob, Dot, Fixed, Pencil, on_reset, on_step

- :class:`Fixed` is a fixed box with its own friction, look and orientation (boards, the pencil's rest).
- :class:`Bob` is a 16 oz brass plumb bob hanging on a line from a fixed point (``pos``, the lower edge of the hole the
  line comes through). One articulation with a fixed base: the line turns freely about the hanging point (joints
  ``top_x``, ``top_y``) and the bob turns freely on the line's end (``bob_x``, ``bob_y``), so it swings as a pendulum
  and rocks on its line. The line is a thin stiff rod: it cannot go slack or bend. Nothing drives the joints; a little
  damping stands for the air and the line's own friction (see ``TOP_DAMP`` and ``KNEE_DAMP``).
- :class:`Pencil` is a loose carpenter's pencil; ``pos`` is its point, its length runs along its own z. Its lead's
  point rides on a short stiff spring, so the force it is pressed with can be read (``lead_force``).
- :class:`Dot` is one pencil dot: a flat graphite-coloured disc that touches nothing, moved by the task file to where
  the pencil's point touches the worktop.

``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``). Pure Python at import time; Isaac Sim is reached
only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Bob", "Dot", "Fixed", "KitState", "Pencil", "env", "finger_points", "gripper_points", "install", "lead_force", "on_reset",
           "on_step", "point_offset", "segment_distance", "turn_x", "write_joints", "write_pose"]

# -- a 16 oz (454 g) brass plumb bob (m): from its point up to where the line is tied --------------------------------
BOB_R = 0.016                                    # the body's radius
CONE_L = 0.060                                   # the taper, from the point up to the body
CYL_L = 0.045                                    # the body
CAP_R, CAP_L = 0.008, 0.015                      # the cap the line is tied into
BOB_L = CONE_L + CYL_L + CAP_L                   # 12 cm, point to line
BOB_MASS = 0.454
BOB_COM = 0.071                                  # its centre of mass, this far up from its point
LINE_R = 0.001                                   # a 2 mm braided line
LINE_MASS = 0.002
LINE_TOP_GAP = 0.015                             # the line touches nothing over this length below the hole
# Joint damping (N m s/rad). The simulator's integrator (240 Hz, joint positions advanced with the mean of the old and
# new speeds) feeds a free swing energy as a damping of -k dt / 2 would, k being the joint's gravity stiffness
# (1.70 N m/rad at the hanging point, 0.22 where the line meets the bob). Each value below is that amount plus the real
# loss: the air and the line at the hanging point (the swing halves in about two minutes), the line's bend at the cap
# (rocking dies away in a few seconds).
TOP_DAMP = 3.55e-3 + 6.6e-4
KNEE_DAMP = 4.5e-4 + 7.5e-4

# -- a carpenter's pencil (m): flat, sharpened to a point ---------------------------------------------------------------
PENCIL_W, PENCIL_T = 0.015, 0.007                # wide (its x) and thick (its y)
PENCIL_TIP = 0.025                               # the sharpened length
PENCIL_L = 0.175                                 # point to end
PENCIL_MASS = 0.010
# The lead's point rides on a short stiff spring in the pencil's wood, so the force it is pressed with can be read: the
# spring's compression times its stiffness, along the pencil. A real lead is rigid; the spring is the measuring device.
LEAD_TRAVEL = 0.0015                             # the point can be pushed this far in before the wood would touch
LEAD_K = 600.0                                   # N/m: 0.3 N pushes it in 0.5 mm
LEAD_DAMP = 2.0                                  # N s/m
LEAD_MASS = 0.001                                # of the pencil's 10 g
LEAD_ARMATURE = 0.004                            # kg added to the slide: keeps the 1 g point steady at 240 Hz
LEAD_CLEAR = 0.0020                              # the wood's taper ends this far behind the point

DOT_R = 0.0015
PALM_BACK = 0.065                                # a gripper's palm, this far back from its grasp point
FINGER_THICK = 0.012                             # a finger's outer face, this far out from its pad


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction, look and orientation (``quat``, xyzw)."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.6
    metallic: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Bob(Box):
    """A plumb bob on its line. ``pos`` is the point the line hangs from, ``drop`` the line's length from there to the
    bob's cap. ``mass`` is the bob's."""

    drop: float = 0.30
    mass: float | None = BOB_MASS

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if not self.drop > 0.05:
            raise ValueError(f"{self.name}: the line must be longer than 5 cm")


@dataclass(frozen=True)
class Pencil(Box):
    """A loose carpenter's pencil. ``pos`` is its point; ``quat`` (xyzw) turns its own z (point to end) and x (its
    width) into the world."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = PENCIL_MASS
    friction: tuple = (0.9, 0.8)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class Dot(Box):
    """A pencil dot (touches nothing; the task file moves it)."""

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it. For a :class:`Bob`, ``pos`` is the bob's point and ``joints`` maps its
    joints to their angles (rad); for a :class:`Pencil`, ``pos`` is its point and ``joints['lead']`` how far the
    point is pushed in against its spring (m)."""

    joints: dict = field(default_factory=dict)


# -- geometry (pure) -----------------------------------------------------------------------------------------------------

def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def turn_x(angle: float) -> tuple:
    """The quaternion (xyzw) of a turn by ``angle`` about x."""
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def point_offset(drop: float, top_x: float = 0.0, top_y: float = 0.0, bob_x: float = 0.0, bob_y: float = 0.0):
    """Where the bob's point is, measured from the point the line hangs from, at these joint angles."""
    r1 = _rx(top_x) @ _ry(top_y)
    r2 = r1 @ _rx(bob_x) @ _ry(bob_y)
    return r1 @ np.array([0.0, 0.0, -drop]) + r2 @ np.array([0.0, 0.0, -BOB_L])


def segment_distance(p, a, b) -> float:
    """The distance from point ``p`` to the straight piece from ``a`` to ``b``."""
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


# -- authoring helpers --------------------------------------------------------------------------------------------------

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

    def look(self, color, rough=0.5, metal=0.0, opacity=1.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal, opacity)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal,
                                                   opacity=opacity)
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

    def body(self, path, pos, mass, lin_damp=0.0, ang_damp=0.0, inertia=None, kinematic=False):
        from pxr import PhysxSchema, UsdPhysics

        xf = self.UsdGeom.Xform.Define(self.stage, path)
        if path != self.path and pos is not None:
            xf.AddTranslateOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*pos))
            xf.AddOrientOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rigid.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(self.Gf.Vec3f(*(float(v) for v in inertia)))
            m.CreateCenterOfMassAttr(self.Gf.Vec3f(0.0, 0.0, 0.0))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        rb.CreateSleepThresholdAttr(0.0)             # a slow swing must not be put to sleep
        rb.CreateStabilizationThresholdAttr(0.0)
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

    def _finish(self, prim, look, mat):
        if look:
            self.paint(prim, look)
        else:
            self.UsdGeom.Imageable(prim).MakeInvisible()
        if mat:
            self.collide(prim, mat)
        return prim

    def box(self, path, centre, size, look, mat=None):
        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        return self._finish(cube.GetPrim(), look, mat)

    def cyl(self, path, r, z0, z1, look, mat=None):
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(z1 - z0))
        c.AddTranslateOp().Set(self.Gf.Vec3d(0.0, 0.0, float(z0 + z1) / 2))
        return self._finish(c.GetPrim(), look, mat)

    def cone(self, path, r, z_apex, z_base, look, mat=None):
        """A cone along z with its point at ``z_apex`` and its round end (radius ``r``) at ``z_base``."""
        c = self.UsdGeom.Cone.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(abs(float(z_base - z_apex)))
        c.AddTranslateOp().Set(self.Gf.Vec3d(0.0, 0.0, float(z_apex + z_base) / 2))
        if z_apex < z_base:                                               # USD's cone points up: turn it over
            c.AddOrientOp().Set(self.Gf.Quatf(0.0, 1.0, 0.0, 0.0))
        return self._finish(c.GetPrim(), look, mat)


# -- spawning ------------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
JOINTS = ("top_x", "top_y", "bob_x", "bob_y")


def _spawn_bob(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A fixed anchor at the hanging point; the line on two crossed hinges under it; the bob on two crossed hinges at
    the line's end. Bodies' origins: the anchor and the line at the hanging point, the bob where the line is tied."""
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    spec: Bob = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    brass = a.look((0.80, 0.62, 0.22), rough=0.28, metal=0.9)
    steel = a.look((0.72, 0.73, 0.75), rough=0.3, metal=0.9)
    cord = a.look((0.95, 0.38, 0.08), rough=0.8)                          # orange braided mason's line
    m_brass = a.mat("brass", 0.45, 0.35)
    m_cord = a.mat("cord", 0.6, 0.5)
    drop = float(spec.drop)
    tiny = (1.0e-7, 1.0e-7, 1.0e-7)

    anchor, t1, line, k1, bob = (f"{prim_path}/{n}" for n in ("anchor", "t1", "line", "k1", "bob"))
    a.body(anchor, (0.0, 0.0, 0.0), 0.005, inertia=tiny)
    a.body(t1, (0.0, 0.0, 0.0), 0.002, inertia=tiny)
    a.body(line, (0.0, 0.0, 0.0), LINE_MASS)
    a.cyl(f"{line}/cord", LINE_R, -drop, 0.0, cord)                       # drawn over its whole length
    a.cyl(f"{line}/hit", LINE_R, -drop, -LINE_TOP_GAP, None, m_cord)      # collides, except right under the hole
    a.body(k1, (0.0, 0.0, -drop), 0.002, inertia=tiny)
    a.body(bob, (0.0, 0.0, -drop), float(spec.mass))
    a.cyl(f"{bob}/cap", CAP_R, -CAP_L, 0.0, brass, m_brass)
    a.cyl(f"{bob}/barrel", BOB_R, -CAP_L - CYL_L, -CAP_L, brass, m_brass)
    a.cone(f"{bob}/taper", BOB_R, -BOB_L, -CAP_L - CYL_L, brass, m_brass)
    a.cone(f"{bob}/tip", BOB_R * 0.0125 / CONE_L + 0.0003, -BOB_L - 0.0003, -BOB_L + 0.0125, steel)   # hardened steel tip

    stage = a.stage
    for name, parent, child, axis, at in (("top_x", anchor, t1, "X", (0.0, 0.0, 0.0)),
                                          ("top_y", t1, line, "Y", (0.0, 0.0, 0.0)),
                                          ("bob_x", line, k1, "X", (0.0, 0.0, -drop)),
                                          ("bob_y", k1, bob, "Y", (0.0, 0.0, 0.0))):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{child}/{name}")
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*at))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
    # the anchor is fixed to the world where it stands
    world = UsdGeom.Xformable(a.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p, q = world.ExtractTranslation(), world.ExtractRotationQuat()
    fix = UsdPhysics.FixedJoint.Define(stage, f"{anchor}/fix")
    fix.CreateBody1Rel().SetTargets([Sdf.Path(anchor)])
    fix.CreateLocalPos0Attr(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
    fix.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), *(float(v) for v in q.GetImaginary())))
    fix.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fix.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(a.root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(a.root)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    art.CreateSleepThresholdAttr(0.0)
    art.CreateStabilizationThresholdAttr(0.0)
    return a.root


def _spawn_pencil(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A flat pencil: its point at the origin, its length along z, its width along x. Two bodies: the stock (wood and
    barrel) and the lead's point, which slides up to LEAD_TRAVEL into the stock against a spring (joint ``lead``)."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Pencil = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    stock, point = f"{prim_path}/stock", f"{prim_path}/point"
    a.body(stock, (0.0, 0.0, 0.0), float(spec.mass) - LEAD_MASS, lin_damp=0.05, ang_damp=0.1)
    a.body(point, (0.0, 0.0, 0.0), LEAD_MASS, lin_damp=0.05, ang_damp=0.1)
    m = a.mat("pencil", *spec.friction)
    red = a.look((0.80, 0.13, 0.10), rough=0.5)
    wood = a.look((0.86, 0.70, 0.48), rough=0.8)
    lead = a.look((0.10, 0.10, 0.11), rough=0.5)
    a.box(f"{stock}/barrel", (0.0, 0.0, (PENCIL_TIP + PENCIL_L) / 2), (PENCIL_W, PENCIL_T, PENCIL_L - PENCIL_TIP), red, m)
    a.cone(f"{stock}/taper", PENCIL_T / 2, LEAD_CLEAR, PENCIL_TIP, wood, m)
    lead_r = PENCIL_T / 2 * 0.007 / PENCIL_TIP + 0.0002
    a.cone(f"{point}/lead", lead_r, 0.0, 0.007, lead, m)                  # the graphite: what touches the worktop

    stage = a.stage
    j = UsdPhysics.PrismaticJoint.Define(stage, f"{point}/lead_slide")
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(stock)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(point)])
    for attr in (j.CreateLocalPos0Attr, j.CreateLocalPos1Attr):
        attr(Gf.Vec3f(0.0, 0.0, 0.0))
    for attr in (j.CreateLocalRot0Attr, j.CreateLocalRot1Attr):
        attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(float(LEAD_TRAVEL))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(LEAD_K))
    drive.CreateDampingAttr(float(LEAD_DAMP))
    drive.CreateTargetPositionAttr(0.0)
    root = stage.GetPrimAtPath(stock)                                     # the stock is the root link: its pose is the pencil's
    UsdPhysics.ArticulationRootAPI.Apply(root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    art.CreateSleepThresholdAttr(0.0)
    return a.root


def _spawn_dot(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, None, 0.001, inertia=(1.0e-8, 1.0e-8, 1.0e-8), kinematic=True)
    a.cyl(f"{prim_path}/disc", DOT_R, -0.0001, 0.0001, a.look((0.12, 0.12, 0.14), rough=0.6))
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
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


def _bob_object(spec: Bob):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BobSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bob)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"top": ImplicitActuatorCfg(joint_names_expr=["top_.*"], stiffness=0.0, damping=TOP_DAMP),
                         "knee": ImplicitActuatorCfg(joint_names_expr=["bob_.*"], stiffness=0.0, damping=KNEE_DAMP)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=BobSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _pencil_object(spec: Pencil):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PencilSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_pencil)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"lead": ImplicitActuatorCfg(joint_names_expr=["lead_slide"], stiffness=LEAD_K, damping=LEAD_DAMP,
                                                     armature=LEAD_ARMATURE)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=PencilSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _fixed_object(scene, spec: Fixed):
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness,
                                       metallic=spec.metallic)
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size),
                              physics_material=_material(sim_utils, spec.static_friction, spec.dynamic_friction),
                              visual_material=look, collision_props=scene.PhysxCollisionPropertiesCfg())
    return scene.Object(name=spec.name, object_type=scene.ObjectType.BASE, spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                                rotation_xyzw=tuple(float(v) for v in spec.quat)))


# -- run time ------------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def write_pose(environment, name: str, pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    """Put a loose or moved-by-hand object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
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


def write_joints(environment, name: str, angles: dict) -> None:
    """Set the joints of bob ``name`` to ``angles`` (rad; the others to 0), all at rest."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    dev = _t(asset.data.joint_pos).device
    pos = torch.tensor([[float(angles.get(n, 0.0)) for n in names]], dtype=torch.float32, device=dev)
    fn = getattr(asset, "write_joint_state_to_sim_index", None)
    if fn is not None:
        fn(position=pos, velocity=torch.zeros_like(pos), joint_ids=list(range(len(names))), env_ids=[0])
    else:
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos))


def _box_bounds(pos, rot, lo, hi):
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ rot.T + pos
    return pts.min(0), pts.max(0)


def _bob_state(environment, spec: Bob) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    origin = _origin(environment)
    bodies = list(asset.body_names)
    pose = _np(asset.data.body_link_pose_w)[0][bodies.index("bob")]
    rot = _rotation(pose[3:7])
    knee = pose[:3] - origin
    point = knee + rot @ np.array([0.0, 0.0, -BOB_L])
    vel = np.zeros(6)
    try:
        vel = _np(asset.data.body_com_vel_w)[0][bodies.index("bob")]
    except Exception:                                                      # noqa: BLE001  (not every version reports it)
        pass
    names = list(asset.joint_names)
    q = _np(asset.data.joint_pos)[0]
    low, high = _box_bounds(knee, rot, (-BOB_R, -BOB_R, -BOB_L), (BOB_R, BOB_R, 0.0))
    return KitState(pos=point, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=low, high=high,
                    joints={n: float(q[names.index(n)]) for n in names})


def lead_force(pushed_in: float) -> float:
    """The force (N, along the pencil) that holds the lead's point pushed in by ``pushed_in`` metres."""
    return LEAD_K * max(0.0, min(float(pushed_in), LEAD_TRAVEL))


def _pencil_state(environment, spec: Pencil) -> KitState:
    """``pos``: the lead's point. ``quat_xyzw``: the stock's. ``joints['lead']``: how far the point is pushed in (m)."""
    asset = environment.unwrapped.scene[spec.name]
    origin = _origin(environment)
    bodies = list(asset.body_names)
    poses = _np(asset.data.body_link_pose_w)[0]
    stock, point = poses[bodies.index("stock")], poses[bodies.index("point")]
    vel = np.zeros(6)
    try:
        vel = _np(asset.data.body_com_vel_w)[0][bodies.index("stock")]
    except Exception:                                                      # noqa: BLE001
        pass
    names = list(asset.joint_names)
    q = float(_np(asset.data.joint_pos)[0][names.index("lead_slide")])
    low, high = _box_bounds(stock[:3] - origin, _rotation(stock[3:7]), (-PENCIL_W / 2, -PENCIL_T / 2, 0.0),
                            (PENCIL_W / 2, PENCIL_T / 2, PENCIL_L))
    return KitState(pos=point[:3] - origin, quat_xyzw=stock[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=low, high=high,
                    joints={"lead": q})


def _loose_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    if isinstance(spec, Pencil):
        low, high = _box_bounds(pos, _rotation(quat), (-PENCIL_W / 2, -PENCIL_T / 2, 0.0), (PENCIL_W / 2, PENCIL_T / 2, PENCIL_L))
    else:
        low, high = pos - DOT_R, pos + DOT_R
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


# -- hooking yam_arena ------------------------------------------------------------------------------------------------

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
    if getattr(scene, "_plumbkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Bob):
            return _bob_object(spec)
        if isinstance(spec, Pencil):
            return _pencil_object(spec)
        if isinstance(spec, Dot):
            return _rigid_object(spec, _spawn_dot)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), (Bob, Pencil, Dot))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            spec = _SPECS[n]
            if isinstance(spec, Bob):
                out[n] = _bob_state(environment, spec)
            elif isinstance(spec, Pencil):
                out[n] = _pencil_state(environment, spec)
            else:
                out[n] = _loose_state(environment, spec)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._plumbkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_plumbkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        if _RESET:                                   # show the scene as the reset callbacks left it
            try:
                base = self._env.unwrapped
                base.sim.step(render=False)          # one physics step (1/240 s) carries the written state to the bodies
                base.scene.update(base.physics_dt)
                for _ in range(3):
                    base.sim.render()
                obs = self._observation()
            except Exception as err:                 # noqa: BLE001
                print(f"[plumbkit] first frame not refreshed: {err!r}", file=sys.stderr, flush=True)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._plumbkit_patched = True


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
