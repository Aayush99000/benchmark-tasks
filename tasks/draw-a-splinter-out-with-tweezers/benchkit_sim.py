"""benchkit's simulator side: authoring the Tools in USD and reading their state back. Imported only once Isaac Sim
builds the scene (see benchkit.py), so the task file and its tests load without a GPU."""

from __future__ import annotations

import math

from benchkit import _TOOLS, Color, Tool, ToolState, _np, box_look, box_parts, joint_name, tool_state


def _spawn_tool(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the tool under ``prim_path``: an Xform holding the body and each slider and hinge body with its joint."""
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
    grip_material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    grip_material.func(material, grip_material)
    looks: dict[tuple, str] = {}

    def look(color: Color, opacity: float = 1.0) -> str:
        key = (color, opacity)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            try:
                surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5,
                                                      **({"opacity": opacity} if opacity < 1.0 else {}))
            except TypeError:  # an Isaac Lab without the opacity field: the part renders opaque
                print(f"[benchkit] {spec.name}: this Isaac Lab can't set opacity; rendering it opaque")
                surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            surface.func(path, surface)
            looks[key] = path
        return looks[key]

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

    def add_box(parent: str, i: int, centre, size, color, yaw=0.0, opacity: float = 1.0, name: str = "",
                ghost: bool = False):
        cube = UsdGeom.Cube.Define(stage, f"{parent}/{name or f'box_{i}'}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if isinstance(yaw, tuple):  # a full rotation (w, x, y, z)
            if tuple(yaw) != (1.0, 0.0, 0.0, 0.0):
                cube.AddOrientOp().Set(Gf.Quatf(*(float(v) for v in yaw)))
        elif yaw:
            cube.AddOrientOp().Set(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        prim = cube.GetPrim()
        if ghost:  # drawn only: no collider
            bind_visual_material(str(prim.GetPath()), look(color, opacity), stage=stage)
            return
        UsdPhysics.CollisionAPI.Apply(prim)
        contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        contact.CreateContactOffsetAttr(0.002)
        contact.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material, stage=stage)
        if opacity <= 0.0:  # a collision proxy: solid, but not drawn
            UsdGeom.Imageable(prim).MakeInvisible()
            return
        bind_visual_material(str(prim.GetPath()), look(color, opacity), stage=stage)

    def joint_frame(parent_name: str, rest) -> tuple[str, tuple]:
        """The parent body's path and the anchor in its frame: the tool body, or a hinge or slider given by name."""
        parent = next((p for p in (*spec.hinges, *spec.sliders) if p.name == parent_name), None)
        if parent is None:
            return body, tuple(rest)
        return f"{prim_path}/{parent.name}", tuple(a - b for a, b in zip(rest, parent.rest))

    body = f"{prim_path}/body"
    rigid_body(body, (0.0, 0.0, 0.0), spec.mass)
    for i, b in enumerate(spec.boxes):
        centre, size, color, yaw, name = box_parts(b)
        opacity, ghost = box_look(b)
        add_box(body, i, centre, size, color, yaw, opacity, name=name, ghost=ghost)
    for h in spec.hinges:  # hinges first: a slider may ride on one
        path = f"{prim_path}/{h.name}"
        rigid_body(path, h.rest, h.mass, h.weightless)
        for i, b in enumerate(h.boxes):
            centre, size, color, yaw, name = box_parts(b)
            opacity, ghost = box_look(b)
            add_box(path, i, centre, size, color, yaw, opacity, name=name, ghost=ghost)
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{joint_name(h)}")
        joint.CreateAxisAttr(h.axis)
        parent_path, anchor = joint_frame(h.parent, h.rest)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*anchor))
        joint.CreateLocalRot0Attr(Gf.Quatf(*(float(v) for v in h.frame)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(*(float(v) for v in h.frame)))
        per_degree = math.pi / 180  # USD gives revolute limits and drive gains in degrees
        joint.CreateLowerLimitAttr(math.degrees(h.lower))
        joint.CreateUpperLimitAttr(math.degrees(h.upper))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate * per_degree)
        drive.CreateDampingAttr(h.damping * per_degree)
        drive.CreateTargetPositionAttr(0.0)
    for p in spec.sliders:
        path = f"{prim_path}/{p.name}"
        rigid_body(path, p.rest, p.mass, p.weightless)
        add_box(path, 0, (0.0, 0.0, 0.0), p.size, p.color, tuple(float(v) for v in p.box_turn), opacity=p.opacity,
                name=p.box_name)
        for i, b in enumerate(p.boxes, start=1):
            centre, size, color, yaw, name = box_parts(b)
            opacity, ghost = box_look(b)
            add_box(path, i, centre, size, color, yaw, opacity, name=name, ghost=ghost)
        joint = UsdPhysics.PrismaticJoint.Define(stage, f"{path}/{joint_name(p)}")
        joint.CreateAxisAttr(p.axis)
        parent_path, anchor = joint_frame(p.parent, p.rest)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*anchor))
        joint.CreateLocalRot0Attr(Gf.Quatf(*(float(v) for v in p.frame)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(*(float(v) for v in p.frame)))
        joint.CreateLowerLimitAttr(p.lower)
        joint.CreateUpperLimitAttr(p.upper)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(p.rate)
        drive.CreateDampingAttr(p.damping)
        drive.CreateTargetPositionAttr(0.0)
    if spec.sliders or spec.hinges:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body))
        articulation = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(body))
        articulation.CreateEnabledSelfCollisionsAttr(bool(spec.self_collide))
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
            # Implicit actuators are the sliders' springs: their default targets are the rest stops.
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[joint_name(s)], stiffness=s.rate, damping=s.damping,
                                                     friction=s.friction)
                         for s in spec.sliders}
            for h in spec.hinges:  # SI here: rad, N m/rad, N m s/rad. Only USD counts in degrees
                actuators[h.name] = ImplicitActuatorCfg(joint_names_expr=[joint_name(h)], stiffness=h.rate,
                                                        damping=h.damping, friction=h.friction)
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            if any(h.init for h in spec.hinges):  # every joint named, so no pattern matches twice
                cfg.init_state.joint_pos = {**{joint_name(s): 0.0 for s in spec.sliders},
                                            **{joint_name(h): h.init for h in spec.hinges}}
            return cfg

    return ToolObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION if spec.sliders or spec.hinges else ObjectType.RIGID,
        spawner_cfg=ToolSpawnerCfg(tool_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_start_quat(spec)),
    )


def _start_quat(spec: Tool) -> tuple:
    """The tool's starting orientation (x, y, z, w): ``quat`` if it is turned, otherwise its yaw."""
    if tuple(spec.quat) != (0.0, 0.0, 0.0, 1.0):
        return tuple(float(v) for v in spec.quat)
    return (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))


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
    sliders, hinges = {}, {}
    if spec.sliders or spec.hinges:
        names = list(asset.joint_names)
        missing = [joint_name(p) for p in (*spec.sliders, *spec.hinges) if joint_name(p) not in names]
        if missing:
            raise RuntimeError(f"benchkit: {spec.name}'s articulation has joints {names}, missing {missing}")
        q = _np(data.joint_pos)[0]
        sliders = {s.name: float(q[names.index(joint_name(s))]) for s in spec.sliders}
        hinges = {h.name: float(q[names.index(joint_name(h))]) for h in spec.hinges}
    return tool_state(spec, pose[:3] - origin, pose[3:7], lin, ang, sliders, hinges)
