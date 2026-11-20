"""rigkit: compound rigid parts, hinges and step callbacks for a yam_arena task file.

Keep this file next to the task file. yam_arena's task-file objects are single primitives; a ``Part`` here is one
rigid body made of several box and cylinder colliders at any orientation. A part can be

- free (it has a mass and falls),
- ``fixed`` (kinematic, with colliders: it stays where it is put),
- ``ghost`` (kinematic, no collider: the task moves it, e.g. the pieces that draw the lanyard's webbing), or
- hinged (``hinge=Hinge(...)``) to another part or to the world, with limits and a spring.

Importing this module hooks ``yam_arena.scene`` and the embodiment without modifying yam_arena. ``on_substep(fn)``
registers ``fn(env)`` to run before every physics step (240 Hz), ``on_step(fn)`` after every control step and
``on_reset(fn)`` after every reset. Pure Python at import time; Isaac Sim is imported lazily.

The hooking technique (a meta-path finder that patches ``yam_arena.scene.task_object`` and the embodiment's step)
follows the ``partkit.py`` helper published with this repository's ``luggage-cart-bungee`` task (MIT licence).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["Hinge", "Part", "part", "rod", "arc", "quat_from_matrix", "rotm", "on_reset", "on_step", "on_substep",
           "pose", "set_pose", "add_forces", "hinge_angle", "gripper_points"]


# -- small maths ------------------------------------------------------------------------------------------------------
def rotm(q) -> np.ndarray:
    """Rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_from_matrix(m) -> tuple:
    """Quaternion (x, y, z, w) of a rotation matrix."""
    m = np.asarray(m, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)
    q = np.array(q)
    return tuple((q / np.linalg.norm(q)).tolist())


def _frame_with_z(direction) -> np.ndarray:
    """A rotation whose z axis points along ``direction``."""
    z = np.asarray(direction, float)
    z = z / np.linalg.norm(z)
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = ref - np.dot(ref, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _frame_with_x(direction) -> np.ndarray:
    """A rotation whose x axis points along ``direction``."""
    x = np.asarray(direction, float)
    x = x / np.linalg.norm(x)
    ref = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
    y = ref - np.dot(ref, x) * x
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def rod(p0, p1, radius, color):
    """A round bar from ``p0`` to ``p1`` (part frame), as a cylinder entry for :func:`part`."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    return (tuple(((p0 + p1) / 2).tolist()), float(radius), float(np.linalg.norm(d)), color, "Z",
            quat_from_matrix(_frame_with_z(d)))


def arc(centre, radius, a0, a1, n, section, color, plane="xy"):
    """A curved flat bar as ``n`` box entries: centre-line radius ``radius`` from angle ``a0`` to ``a1`` (rad) round
    ``centre`` in the part's ``plane``; ``section`` is (radial width, thickness along the plane's normal)."""
    if plane != "xy":
        raise ValueError(plane)
    out = []
    step = (a1 - a0) / n
    chord = 2 * radius * math.sin(abs(step) / 2)
    rc = radius * math.cos(abs(step) / 2)
    for k in range(n):
        a = a0 + (k + 0.5) * step
        c, s = math.cos(a), math.sin(a)
        pos = (centre[0] + rc * c, centre[1] + rc * s, centre[2])
        rot = np.column_stack([[-s, c, 0.0], [-c, -s, 0.0], [0.0, 0.0, 1.0]])    # x: tangent, y: inward, z: normal
        size = (chord + section[0] * math.tan(abs(step) / 2), section[0], section[1])   # outer corners meet
        out.append((pos, size, color, quat_from_matrix(rot)))
    return out


# -- parts ------------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Hinge:
    """A revolute joint holding a part. ``anchor`` and ``axis`` are in the part's own frame. ``parent`` names
    another part (None: the world). Limits and ``target`` are degrees from the start pose; ``stiffness`` is the
    spring's rate in N m/rad toward ``target`` and ``damping`` in N m s/rad."""

    anchor: tuple
    axis: tuple
    parent: str | None = None
    lower: float | None = None
    upper: float | None = None
    stiffness: float = 0.0
    damping: float = 0.0
    target: float = 0.0


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is a bounding box
    and ``pos`` the body origin. ``boxes`` are (centre, size, colour[, quat xyzw]) and ``cylinders`` are
    (centre, radius, length, colour, axis "X" | "Y" | "Z"[, quat xyzw]) in the part frame. ``quat`` (x, y, z, w) is
    the part's start orientation."""

    boxes: tuple = ()
    cylinders: tuple = ()
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    friction: float = 1.0
    ghost: bool = False        # kinematic, no collider (moved by the task)
    fixed: bool = False        # kinematic, with colliders (stays where it is put)
    damping: float = 0.5       # angular damping (1/s)
    hinge: Hinge | None = None
    roughness: float = 0.45
    metallic: float = 0.0


PARTS: dict[str, Part] = {}


def part(name, pos, boxes=(), mass=1.0, quat=(0.0, 0.0, 0.0, 1.0), color=(0.5, 0.5, 0.5), friction=1.0, ghost=False,
         cylinders=(), fixed=False, damping=0.5, hinge=None, roughness=0.45, metallic=0.0):
    half = [np.array(b[1]) / 2 * (1.0 if len(b) < 4 else 1.42) for b in boxes]
    for c in cylinders:
        r, h, ax = c[1], c[2], c[4]
        if len(c) > 5:
            half.append(np.full(3, max(r, h / 2)))
        else:
            half.append(np.array([h / 2 if ax == "X" else r, h / 2 if ax == "Y" else r, h / 2 if ax == "Z" else r]))
    centres = np.array([b[0] for b in boxes] + [c[0] for c in cylinders], float)
    ext = np.max(np.abs(centres) + np.array(half), axis=0) * 2
    p = Part(name=name, pos=tuple(float(v) for v in pos), size=tuple(ext.tolist()), mass=mass, color=color,
             boxes=tuple(boxes), cylinders=tuple(cylinders), quat=tuple(float(v) for v in quat), friction=friction,
             ghost=ghost, fixed=fixed, damping=damping, hinge=hinge, roughness=roughness, metallic=metallic)
    PARTS[name] = p
    return p


def _author_hinge(stage, prim_path: str, spec: Part) -> None:
    from pxr import Gf, PhysxSchema, UsdPhysics

    h = spec.hinge
    rc = rotm(spec.quat)
    anchor_w = np.asarray(spec.pos, float) + rc @ np.asarray(h.anchor, float)
    frame_w = _frame_with_x(rc @ np.asarray(h.axis, float))            # joint frame: x along the hinge axis
    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/hinge")
    joint.CreateAxisAttr("X")

    def quatf(m):
        x, y, z, w = quat_from_matrix(m)
        return Gf.Quatf(float(w), float(x), float(y), float(z))

    if h.parent is None:
        joint.CreateLocalPos0Attr(Gf.Vec3f(*anchor_w.tolist()))
        joint.CreateLocalRot0Attr(quatf(frame_w))
    else:
        parent = PARTS[h.parent]
        rp = rotm(parent.quat)
        root = prim_path.rsplit("/", 1)[0]
        joint.CreateBody0Rel().SetTargets([f"{root}/{h.parent}/body"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(rp.T @ (anchor_w - np.asarray(parent.pos, float))).tolist()))
        joint.CreateLocalRot0Attr(quatf(rp.T @ frame_w))
    joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
    joint.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in h.anchor]))
    joint.CreateLocalRot1Attr(quatf(rc.T @ frame_w))
    if h.lower is not None:
        joint.CreateLowerLimitAttr(float(h.lower))
    if h.upper is not None:
        joint.CreateUpperLimitAttr(float(h.upper))
    if h.stiffness or h.damping:
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(h.stiffness) * math.pi / 180.0)     # USD angular drives are per degree
        drive.CreateDampingAttr(float(h.damping) * math.pi / 180.0)
        drive.CreateTargetPositionAttr(float(h.target))
    PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = PARTS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    looks: dict = {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color[:3]), roughness=spec.roughness,
                                              metallic=spec.metallic)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    main = f"{prim_path}/body"
    xform = UsdGeom.Xform.Define(stage, main)
    xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    kinematic = spec.ghost or spec.fixed
    if kinematic:
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(4)
    rb.CreateAngularDampingAttr(spec.damping)
    rb.CreateLinearDampingAttr(1.0 if spec.damping > 0.5 else 0.0)
    rb.CreateEnableCCDAttr(not kinematic)
    shapes = []
    for i, entry in enumerate(spec.boxes):
        centre, size, color = entry[:3]
        cube = UsdGeom.Cube.Define(stage, f"{main}/box_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if len(entry) > 3:                         # optional orientation (x, y, z, w) of this box in the part
            qx, qy, qz, qw = entry[3]
            cube.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        shapes.append((cube.GetPrim(), color))
    for i, entry in enumerate(spec.cylinders):
        centre, radius, length, color, axis = entry[:5]
        cyl = UsdGeom.Cylinder.Define(stage, f"{main}/cyl_{i}")
        cyl.CreateAxisAttr(axis)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(length)
        cyl.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if len(entry) > 5:
            qx, qy, qz, qw = entry[5]
            cyl.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
        shapes.append((cyl.GetPrim(), color))
    for p, color in shapes:
        if not spec.ghost:
            UsdPhysics.CollisionAPI.Apply(p)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(p)
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(p.GetPath()), phys, stage=stage)
        bind_visual_material(str(p.GetPath()), look(color), stage=stage)
    if spec.hinge is not None:
        _author_hinge(stage, prim_path, spec)
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=PartSpawnerCfg(part_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=tuple(spec.quat)))


# -- reading and moving parts -----------------------------------------------------------------------------------------
def _t(x):
    return getattr(x, "torch", x)


def pose(environment, name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Body origin (env frame), quaternion xyzw and linear velocity of a part (env 0)."""
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins)[0].detach().cpu().numpy().astype(float)
    p = _t(asset.data.root_link_pose_w)[0].detach().cpu().numpy().astype(float)
    v = _t(asset.data.root_lin_vel_w)[0].detach().cpu().numpy().astype(float)
    return p[:3] - origin, p[3:7], v


def set_pose(environment, name: str, pos, quat_xyzw) -> None:
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins)[0]
    t = torch.tensor([[*pos, *quat_xyzw]], dtype=torch.float32, device=origin.device)
    t[:, :3] += origin
    asset.write_root_pose_to_sim_index(root_pose=t)


def add_forces(environment, name: str, pairs, torque=(0.0, 0.0, 0.0)) -> None:
    """Apply several (force, point) pairs (N, world; env frame) and a free torque (N m, world) on a part for the
    next physics step."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins)[0].detach().cpu().numpy().astype(float)
    com = _t(asset.data.root_com_pos_w)[0].detach().cpu().numpy().astype(float) - origin
    f, tq = np.zeros(3), np.asarray(torque, float).copy()
    for force, point in pairs:
        force = np.asarray(force, float)
        f += force
        tq += np.cross(np.asarray(point, float) - com, force)
    dev = asset.device
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[f.tolist()]], dtype=torch.float32, device=dev),
        torques=torch.tensor([[tq.tolist()]], dtype=torch.float32, device=dev), is_global=True)


def hinge_angle(environment, name: str) -> float:
    """Degrees a hinged part has turned from its start pose about its hinge axis (relative to its parent)."""
    spec = PARTS[name]
    h = spec.hinge
    _, q, _ = pose(environment, name)
    r_now = rotm(q)
    r0 = rotm(spec.quat)
    if h.parent is not None:
        _, qp, _ = pose(environment, h.parent)
        r_now = rotm(qp).T @ r_now
        r0 = rotm(PARTS[h.parent].quat).T @ r0
    d = r0.T @ r_now                                   # rotation since the start, in the part's own frame
    axis = np.asarray(h.axis, float) / np.linalg.norm(h.axis)
    v = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]]) / 2.0     # sin(angle) x axis
    ang = math.atan2(float(np.dot(v, axis)), (np.trace(d) - 1.0) / 2.0)
    return math.degrees(ang)


def gripper_points(environment) -> dict:
    """Points (env frame) on each gripper: the palm, the centre between the finger pads and each pad (tip and
    middle), keyed by arm; and each gripper's opening (m between the pads)."""
    base = environment.unwrapped
    origin = _t(base.scene.env_origins)[0].detach().cpu().numpy().astype(float)
    out = {}
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        names = list(robot.body_names)
        g = _t(robot.data.body_link_pose_w)[0].detach().cpu().numpy().astype(float)[names.index("gripper")]
        R, p = rotm(g[3:7]), g[:3] - origin
        jn = list(robot.joint_names)
        half = abs(float(_t(robot.data.joint_pos)[0].detach().cpu().numpy()[jn.index("joint7")])) + 0.008
        pts = [p + R @ np.array(local, float) for local in ((0, 0, 0.06), (0, 0, 0.134), (half, 0, 0.147),
                                                              (-half, 0, 0.147), (half, 0, 0.12), (-half, 0, 0.12))]
        out[key.split("_")[0]] = (pts, 2 * half - 0.016)
    return out


def _state(environment, spec: Part) -> ObjectState:
    p, q, v = pose(environment, spec.name)
    r = rotm(q)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = p + corners @ r.T
    return ObjectState(pos=p, quat_xyzw=q, lin_vel=v, ang_vel=np.zeros(3), low=pts.min(0), high=pts.max(0))


# -- hooking yam_arena ------------------------------------------------------------------------------------------------
_STEP: list = []
_RESET: list = []
_SUB: list = []


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def on_substep(fn):
    _SUB.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_rigkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            PARTS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in PARTS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in PARTS:
                out[n] = _state(environment, PARTS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._rigkit_patched = True


def _wrap_scene_writes(environment) -> None:
    """Run the substep callbacks just before the scene writes its data to the simulator (every physics step)."""
    sc = environment.unwrapped.scene
    if getattr(sc, "_rigkit_wrapped", False):
        return
    orig = sc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for fn in _SUB:
            fn(environment)
        return orig(*a, **k)

    sc.write_data_to_sim = write_data_to_sim
    sc._rigkit_wrapped = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_rigkit_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        out = base_ensure(self, *a, **k)
        if self._env is not None:
            for fn in _RESET:
                fn(self._env)
            _wrap_scene_writes(self._env)
        return out

    def _task_step(self, *a, **k):
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        if self._env is not None:   # clear the task's state before the reset's settling steps
            for fn in _RESET:
                fn(self._env)
        obs = base_reset(self, *a, **k)
        for fn in _STEP:
            fn(self._env)
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._rigkit_patched = True


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
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
