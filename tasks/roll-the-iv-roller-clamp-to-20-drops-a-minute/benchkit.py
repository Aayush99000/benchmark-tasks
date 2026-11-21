"""benchkit: lab-bench parts for yam_arena task files. Tools with spring-loaded sliders, and per-step hooks.

yam_arena's task-file objects are single primitives. This module adds :class:`Tool`, one loose rigid body made of
several box colliders that can carry :class:`Slider` parts: boxes on prismatic joints that a spring pushes back to
their rest stops. A squeeze-bulb transfer pipette is a Tool with two sliders, the bulb's sides, which the gripper's
fingers push in.

Importing this module hooks ``yam_arena.scene`` and the ``yam_arena`` embodiment's step and reset, without changing
yam_arena itself:

- ``success()`` and ``score()`` receive a :class:`ToolState` for each Tool (its body pose, bounds, and slider
  positions), and an ordinary ``ObjectState`` for everything else.
- :func:`on_step` and :func:`on_reset` register callbacks that receive the object states after every control step
  (before the task file is scored) and after every reset.
- :func:`recolor` changes an object's colour mid-trial, to show what the simulator can't (liquid taking up dye).
- A box can carry a :class:`Part` as a fourth element: a turn (roll, pitch, yaw) for a sloping face, ``ghost`` for a
  mark that is drawn but touches nothing (a drop of blood), and ``label`` for a prim name :func:`recolor` can find.
- A Tool can start turned about the world x and y axes (``roll``, ``pitch``), for an item lying on its side.
- A :class:`Slider` can carry more boxes (``boxes``) that move with it: a plunger's rod, thumb rest and stopper.

Nothing that needs Isaac Sim is imported until the simulator builds the scene, so the task file and its tests load
without a GPU.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
BoxSpec = tuple  # (centre, size, colour) or (centre, size, colour, Part): tool frame, m, sRGB


@dataclass(frozen=True)
class Part:
    """Extras for one box of a Tool, given as the fourth element of its BoxSpec. ``turn`` is (roll, pitch, yaw) in rad
    about the tool frame's x, y and z, applied in that order. A ``ghost`` box is drawn but has no collider. ``label``
    names the box's prim and gives it a material of its own, so :func:`recolor` can change it alone."""

    turn: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ghost: bool = False
    label: str = ""


def part(box) -> Part:
    return box[3] if len(box) > 3 else Part()


def rpy_matrix(turn) -> np.ndarray:
    """Rotation for (roll, pitch, yaw): roll about x first, then pitch about y, then yaw about z."""
    r, p, y = turn
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def quat_xyzw(rot) -> tuple[float, float, float, float]:
    """A rotation matrix as a unit quaternion (x, y, z, w)."""
    m = np.asarray(rot, dtype=float)
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return (x, y, z, w)


def box_corners(box) -> np.ndarray:
    """The eight corners of a BoxSpec in the tool frame, turned if it has a turn."""
    centre, size = np.asarray(box[0], dtype=float), np.asarray(box[1], dtype=float)
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], dtype=float)
    return centre + (rpy_matrix(part(box).turn) @ (signs * size / 2).T).T


@dataclass(frozen=True)
class Slider:
    """A box on a prismatic joint along the tool frame's ``axis`` ("X", "Y" or "Z"). Its frame rests at ``rest``
    (tool frame); it moves between ``lower`` and ``upper`` (m, 0 is the rest stop) and a spring of ``rate`` N/m
    returns it to 0. ``friction`` (N) is the joint's breakaway friction, passed to the implicit actuator as its static
    and its sliding (dynamic) friction effort: with ``rate`` 0 the slider stays put until pushed harder than that, and
    the same force holds it back while it moves. ``weightless`` turns gravity off for the slider's own body, so that
    on a tool tipped a little its weight puts no load along the joint (PhysX lets a load under the breakaway creep a
    joint through its friction; the IV clamp's wheel crept shut that way while held)."""

    name: str
    rest: tuple[float, float, float]
    size: tuple[float, float, float]
    axis: str = "Z"
    lower: float = -0.010
    upper: float = 0.0
    color: Color = (0.85, 0.85, 0.88)
    rate: float = 600.0
    damping: float = 8.0
    mass: float = 0.004
    boxes: tuple = ()  # more BoxSpecs in the slider's own frame, moving with it
    friction: float = 0.0
    weightless: bool = False  # gravity off for the slider's body: its weight never loads its joint (see the clamp)


@dataclass(frozen=True)
class Tool(Box):
    """A loose rigid tool made of ``boxes`` (tool frame). It subclasses Box so yam_arena's task-file checks accept
    it: ``pos`` is the body origin and ``size`` its bounding box. ``points`` names tool-frame points (a tip, say)
    that :meth:`ToolState.point` puts in the world. ``weightless`` turns gravity off for the body."""

    boxes: tuple[BoxSpec, ...] = ()
    sliders: tuple[Slider, ...] = ()
    friction: float = 1.0
    points: tuple[tuple[str, tuple[float, float, float]], ...] = ()
    weightless: bool = False
    pitch: float = 0.0  # rad about the world y axis, applied after the roll and before the yaw
    roll: float = 0.0  # rad about the world x axis, applied first

    def start_rotation(self) -> np.ndarray:
        return rpy_matrix((self.roll, self.pitch, self.yaw))

    def _check(self) -> None:
        super()._check()
        if not self.boxes:
            raise ValueError(f"{self.name}: a Tool needs at least one box")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Tool must be loose (give it a mass)")


def tool(name, pos, boxes, mass, *, sliders=(), yaw=0.0, color=(0.5, 0.5, 0.5), friction=1.0, points=(),
         weightless=False, pitch=0.0, roll=0.0) -> Tool:
    """Build a Tool; its bounding box is taken from the boxes and sliders."""
    shapes = list(boxes) + [(s.rest, s.size, s.color) for s in sliders]
    shapes += [(tuple(np.add(s.rest, b[0])), b[1], b[2]) for s in sliders for b in s.boxes]
    corners = np.concatenate([box_corners(b) for b in shapes])
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    return Tool(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), sliders=tuple(sliders), friction=friction, points=tuple(points),
                weightless=weightless, pitch=pitch, roll=roll)


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass(frozen=True)
class ToolState(ObjectState):
    """A tool's body pose (``pos`` is the body origin, not the bounding-box centre), velocities, world bounds, each
    slider's position (m from its rest stop) and its named points in the world."""

    sliders: dict = field(default_factory=dict)
    named: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        return _rotation(self.quat_xyzw)

    def to_world(self, local) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local, dtype=float)

    def point(self, name: str) -> np.ndarray:
        return self.named[name]

    def tilt_deg(self) -> float:
        """Angle between the tool's z axis and the vertical."""
        return float(np.degrees(np.arccos(np.clip(self.rotation()[2, 2], -1.0, 1.0))))


def tool_state(spec: Tool, pos, quat_xyzw, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0), sliders=None) -> ToolState:
    """A ToolState from a body pose: what the simulator reports, and what tests build by hand."""
    pos, quat = np.asarray(pos, dtype=float), np.asarray(quat_xyzw, dtype=float)
    rot = _rotation(quat)
    solid = [b for b in spec.boxes if not part(b).ghost] or list(spec.boxes)
    corners = np.concatenate([box_corners(b) for b in solid])
    pts = pos + (rot @ corners.T).T
    named = {n: pos + rot @ np.asarray(p, dtype=float) for n, p in spec.points}
    return ToolState(pos=pos, quat_xyzw=quat, lin_vel=np.asarray(lin_vel, dtype=float),
                     ang_vel=np.asarray(ang_vel, dtype=float), low=pts.min(0), high=pts.max(0),
                     sliders=dict(sliders or {s.name: 0.0 for s in spec.sliders}), named=named)


# -- spawning (Isaac Sim, imported lazily) ------------------------------------------------------------------------

_TOOLS: dict[str, Tool] = {}


def joint_name(slider: Slider) -> str:
    return f"{slider.name}_slide"


def _spawn_tool(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the tool under ``prim_path``: an Xform holding the body and each slider body with its joint."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec = _TOOLS[cfg.tool_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    material = f"{prim_path}/Looks/grip"
    grip = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    grip.func(material, grip)
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    def rigid_body(path: str, pos, mass: float, weightless: bool = False):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=double).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=double).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        body.CreateMaxDepenetrationVelocityAttr(1.0)
        body.CreateSolverPositionIterationCountAttr(32)
        if spec.weightless or weightless:
            body.CreateDisableGravityAttr(True)
        return prim

    def own_look(label: str, color: Color) -> str:
        path = f"{prim_path}/Looks/{label}"
        surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
        surface.func(path, surface)
        return path

    def add_box(parent: str, i: int, centre, size, color, extra: Part = Part()):
        cube = UsdGeom.Cube.Define(stage, f"{parent}/{extra.label or f'box_{i}'}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if any(extra.turn):
            x, y, z, w = quat_xyzw(rpy_matrix(extra.turn))
            cube.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        prim = cube.GetPrim()
        if not extra.ghost:
            UsdPhysics.CollisionAPI.Apply(prim)
            contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            contact.CreateContactOffsetAttr(0.002)
            contact.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(prim.GetPath()), material, stage=stage)
        shade = own_look(extra.label, color) if extra.label else look(color)
        bind_visual_material(str(prim.GetPath()), shade, stage=stage)

    body = f"{prim_path}/body"
    rigid_body(body, (0.0, 0.0, 0.0), spec.mass)
    for i, box in enumerate(spec.boxes):
        add_box(body, i, box[0], box[1], box[2], part(box))
    for p in spec.sliders:
        path = f"{prim_path}/{p.name}"
        rigid_body(path, p.rest, p.mass, p.weightless)
        add_box(path, 0, (0.0, 0.0, 0.0), p.size, p.color)
        for k, extra in enumerate(p.boxes, start=1):
            add_box(path, k, extra[0], extra[1], extra[2], part(extra))
        joint = UsdPhysics.PrismaticJoint.Define(stage, f"{path}/{joint_name(p)}")
        joint.CreateAxisAttr(p.axis)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*p.rest))
        joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLowerLimitAttr(p.lower)
        joint.CreateUpperLimitAttr(p.upper)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(p.rate)
        drive.CreateDampingAttr(p.damping)
        drive.CreateTargetPositionAttr(0.0)
    if spec.sliders:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body))
        articulation = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(body))
        articulation.CreateEnabledSelfCollisionsAttr(False)
        articulation.CreateSolverPositionIterationCountAttr(32)
        articulation.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Tool):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class ToolSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_tool)
        tool_name: str = ""

    class ToolObject(Object):
        def _generate_articulation_cfg(self):
            # Implicit actuators are the sliders' springs: their default targets are the rest stops. Isaac Lab 3.0
            # (Isaac Sim 5+) takes ``friction`` as the static friction effort (N) only; its dynamic effort stays at the
            # USD default, 0, so a slider would run free once it broke loose. The sliding friction is set equal to it,
            # as physcheck's frictionloss and the tasks' fidelity text have it.
            sliding = _has_field(ImplicitActuatorCfg, "dynamic_friction")
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[joint_name(s)], stiffness=s.rate, damping=s.damping,
                                                     friction=s.friction,
                                                     **({"dynamic_friction": s.friction} if sliding else {}))
                         for s in spec.sliders}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return ToolObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION if spec.sliders else ObjectType.RIGID,
        spawner_cfg=ToolSpawnerCfg(tool_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=quat_xyzw(spec.start_rotation())),
    )


def _has_field(cfg_class, name: str) -> bool:
    """Whether an Isaac Lab configclass declares ``name`` (older versions lack some actuator fields)."""
    return name in getattr(cfg_class, "__dataclass_fields__", {}) or hasattr(cfg_class, name)


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _read_tool(environment, spec: Tool) -> ToolState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    data = asset.data
    pose = _np(data.root_link_pose_w)[0]
    if hasattr(data, "root_lin_vel_w") and hasattr(data, "root_ang_vel_w"):
        lin, ang = _np(data.root_lin_vel_w)[0], _np(data.root_ang_vel_w)[0]
    elif hasattr(data, "root_vel_w"):
        vel = _np(data.root_vel_w)[0]
        lin, ang = vel[:3], vel[3:6]
    else:  # a silent zero would make every tool look at rest
        raise RuntimeError(f"benchkit: can't read {spec.name}'s velocity from this Isaac Lab version")
    sliders = {}
    if spec.sliders:
        names = list(asset.joint_names)
        missing = [joint_name(s) for s in spec.sliders if joint_name(s) not in names]
        if missing:
            raise RuntimeError(f"benchkit: {spec.name}'s articulation has joints {names}, missing {missing}")
        q = _np(data.joint_pos)[0]
        sliders = {s.name: float(q[names.index(joint_name(s))]) for s in spec.sliders}
    return tool_state(spec, pose[:3] - origin, pose[3:7], lin, ang, sliders)


# -- hooks into yam_arena -----------------------------------------------------------------------------------------

_STEP: dict = {}
_RESET: dict = {}
_ENV: dict = {"env": None}
_SHADERS: dict = {}
_WARNED: set = set()


def on_step(fn):
    """Register ``fn(states)``: called with every object's state after each control step, before scoring. A task
    file loaded again replaces its earlier hooks rather than adding to them."""
    _STEP[(fn.__module__, fn.__qualname__)] = fn
    return fn


def on_reset(fn):
    """Register ``fn(states)``: called after every reset, once objects have settled."""
    _RESET[(fn.__module__, fn.__qualname__)] = fn
    return fn


def recolor(name: str, srgb: Color) -> None:
    """Change an object's colour while the simulation runs (e.g. liquid taking up dye). Does nothing outside the
    simulator, and only warns once if the object's material can't be found: it never stops a trial."""
    env = _ENV["env"]
    if env is None:
        return
    try:
        from pxr import Gf, Usd, UsdShade

        from yam_arena.station import linear_rgb

        if name not in _SHADERS:
            stage = env.unwrapped.sim.stage
            prims = [p for p in stage.Traverse() if p.GetName() == name and "env_0" in str(p.GetPath())]
            shaders = []
            for root in prims[:1]:
                for part in Usd.PrimRange(root):
                    if part.IsA(UsdShade.Shader):
                        shaders.append(UsdShade.Shader(part))
                    material = UsdShade.MaterialBindingAPI(part).ComputeBoundMaterial()[0]
                    if material:
                        shaders += [UsdShade.Shader(q) for q in Usd.PrimRange(material.GetPrim())
                                    if q.IsA(UsdShade.Shader)]
            _SHADERS[name] = list({str(sh.GetPath()): sh for sh in shaders}.values())
        colour = Gf.Vec3f(*linear_rgb(srgb))
        for shader in _SHADERS[name]:
            for key in ("diffuseColor", "diffuse_color_constant"):
                port = shader.GetInput(key)
                if port:
                    port.Set(colour)
        if not _SHADERS[name] and name not in _WARNED:
            _WARNED.add(name)
            print(f"[benchkit] no material found to recolour {name!r}")
    except Exception as err:  # colour is cosmetic: never let it end a trial
        if name not in _WARNED:
            _WARNED.add(name)
            print(f"[benchkit] could not recolour {name!r}: {err}")


def _patch_scene(scene) -> None:
    if getattr(scene, "_benchkit", False):
        return
    plain_object, plain_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Tool):
            _TOOLS[spec.name] = spec
            return _arena_object(spec)
        return plain_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _TOOLS]
        out = plain_states(environment, plain) if plain else {}
        for n in names:
            if n in _TOOLS:
                out[n] = _read_tool(environment, _TOOLS[n])
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._benchkit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_benchkit", False):
        return
    plain_step, plain_reset = cls._task_step, cls.reset

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self._env, self.task.names)

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        if _STEP:
            now = states(self)
            for fn in list(_STEP.values()):
                fn(now)
        return plain_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = plain_reset(self, *a, **k)
        _ENV["env"] = self._env
        if _RESET:
            now = states(self)
            for fn in list(_RESET.values()):
                fn(now)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._benchkit = True


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
        plain_exec = loader.exec_module

        def exec_module(module):
            plain_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena: now for modules already imported, otherwise when they are. Safe to call twice."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
