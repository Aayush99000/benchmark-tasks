"""Task-local native PhysX paper surface and read-only access snapshots.

The paper is a real Arena deformable. Its material top edge is attached to a
rigid roll before simulation starts. Arena owns nodal reset; control steps only
read live nodes, velocities and rigid transforms. Grasping uses the unchanged
robot's collision geometry, with no grasp weld or node targets.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from yam_arena import YAM_URDF
from yam_arena.task_file import Box, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix, rpy_matrix

import mechkit as mk
from surface_access_geometry import CollisionBox, Outlet, evaluate_surface_access, topology_hash
from release_clearance import validate_robot_coverage


@dataclass(frozen=True)
class PaperMarker(Box):
    """Loader-compatible marker converted into a native deformable at scene build."""


@dataclass(frozen=True)
class SurfaceSnapshot:
    nodes: np.ndarray
    velocities: np.ndarray
    stub_transform: np.ndarray
    robot_boxes: tuple[CollisionBox, ...]
    access: object
    tool_boxes: tuple[CollisionBox, ...] = ()
    env_origin: tuple[float, float, float] = (0., 0., 0.)
    expected_robot_names: frozenset[str] = frozenset()
    expected_tool_names: frozenset[str] = frozenset()


_PAPERS = {}
_ROBOT_BOUNDS = None


def _numpy(value):
    return getattr(value, "torch", value).detach().cpu().numpy().astype(float)


def _frozen(value):
    value = np.array(value, copy=True)
    value.setflags(write=False)
    return value


def material_labels(material):
    """Freeze top/free rows and a terminal material patch before deformation."""
    material = np.asarray(material, dtype=float)
    root = np.flatnonzero(np.isclose(material[:,1],material[:,1].max(),atol=1e-7))
    end = np.flatnonzero(np.isclose(material[:,1],material[:,1].min(),atol=1e-7))
    # Use complete rows within the last 20 mm. Rounding outward to a 21.25 mm
    # patch would incorrectly reject an end exactly 20 mm below the aperture.
    patch = np.flatnonzero(material[:,1]<=material[:,1].min()+.020+1e-7)
    return _frozen(root), _frozen(end), _frozen(patch)


def _robot_bounds():
    """Conservative bounds of every real URDF collision piece in its link frame."""
    global _ROBOT_BOUNDS
    if _ROBOT_BOUNDS is not None:
        return _ROBOT_BOUNDS
    _ROBOT_BOUNDS = {}
    for link in ET.parse(YAM_URDF).getroot().findall("link"):
        pieces = []
        for collision in link.findall("collision"):
            mesh_spec = collision.find("geometry/mesh")
            if mesh_spec is None:
                raise ValueError("The installed YAM collision geometry must be mesh-based")
            mesh = trimesh.load(Path(YAM_URDF).parent / mesh_spec.get("filename"), process=False)
            origin = collision.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            scale = np.fromstring(mesh_spec.get("scale", "1 1 1"), sep=" ")
            vertices = (mesh.vertices * scale) @ rpy_matrix(rpy).T + xyz
            low, high = vertices.min(0), vertices.max(0)
            pieces.append(((low + high) / 2, (high - low) / 2))
        _ROBOT_BOUNDS[link.get("name")] = tuple(pieces)
    return _ROBOT_BOUNDS


def _robot_boxes(base):
    boxes = []
    bounds = _robot_bounds()
    validate_robot_coverage({arm: tuple(base.scene[arm].body_names)
        for arm in ('left_robot', 'right_robot')}, {name: len(pieces) for name, pieces in bounds.items()})
    for arm_name in ("left_robot", "right_robot"):
        robot = base.scene[arm_name]
        positions = _numpy(robot.data.body_pos_w)[0]
        quaternions = _numpy(robot.data.body_quat_w)[0]  # current Isaac Lab is xyzw
        if (positions.shape != (len(robot.body_names), 3)
                or quaternions.shape != (len(robot.body_names), 4)
                or not np.isfinite(positions).all() or not np.isfinite(quaternions).all()
                or np.any(np.abs(np.linalg.norm(quaternions, axis=1)-1.) > 1e-3)):
            raise ValueError('Incomplete measured robot collision poses')
        for index, name in enumerate(robot.body_names):
            rotation = quat_xyzw_to_matrix(quaternions[index])
            for piece, (center, half) in enumerate(bounds[name]):
                boxes.append(CollisionBox(f"{arm_name}/{name}/{piece}",
                    center=rotation @ center + positions[index], half_extents=half,
                    quat_xyzw=quaternions[index]))
    return tuple(boxes)


class SurfacePaper:
    """One fixed-width paper strip with engineering material values, not calibrated tissue."""

    def __init__(self, name, parent, parent_pose, top_world, outlet: Outlet, minimum_z,
                 width=.090, length=.170, areal_mass=.028, thickness=.0001,
                 youngs_modulus=1e6, poissons_ratio=.25, dynamic_friction=.3):
        self.name, self.parent, self.parent_pose = name, parent, parent_pose
        self.tool_boxes_reader = None
        self.expected_tool_names = frozenset()
        self.width, self.length, self.thickness = float(width), float(length), float(thickness)
        self.areal_mass = float(areal_mass)
        self.mass = self.width * self.length * self.areal_mass
        self.youngs_modulus, self.poissons_ratio = float(youngs_modulus), float(poissons_ratio)
        # Omni Physics' documented surface-material field already includes this
        # thin-plate factor; the solver applies thickness cubed internally.
        self.bend_modulus = self.youngs_modulus / (12 * (1 - self.poissons_ratio**2))
        self.dynamic_friction = float(dynamic_friction)
        self.outlet, self.minimum_z = outlet, float(minimum_z)
        # Flat stress-free material starts tangent to the old roll, with its
        # real loose edge pre-threaded through the open target aperture.
        orientation = mk.axis_angle((1, 0, 0), 2 * np.pi / 3)
        world = mk._T(orientation,
                      np.asarray(top_world, float) - orientation[:, 1] * self.length / 2)
        self._in_parent = np.linalg.inv(np.asarray(parent_pose(), float)) @ world
        self.marker = PaperMarker(name=name, pos=tuple(float(v) for v in world[:3, 3]),
                                  size=(self.width, self.thickness, self.length), color=(.96, .95, .90))
        self.snapshot = None
        self.faces = self.root_ids = self.free_edge_ids = self.terminal_patch_ids = None
        self.root_local_anchors = None
        self.topology = None
        _PAPERS[name] = self

    def initial_pose(self):
        world = np.asarray(self.parent_pose(), float) @ self._in_parent
        # Arena builds torch tensors without an explicit dtype at reset. NumPy
        # float64 scalars would produce Double quaternions against Float nodes.
        return tuple(float(v) for v in world[:3, 3]), tuple(float(v) for v in mk.mat_to_quat(world[:3, :3]))

    def build(self, mod):
        from isaaclab_arena.assets.deformable_object import DeformableObject
        from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg
        from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg

        position, quaternion = self.initial_pose()
        return DeformableObject(name=self.name,
            initial_pose=mod.Pose(position_xyz=position, rotation_xyzw=quaternion),
            spawner_cfg=mod.sim_utils.MeshRectangleCfg(size=(self.width, self.length), edge_refinement=24,
                deformable_props=PhysxDeformableBodyPropertiesCfg(mass=self.mass,
                    solver_position_iteration_count=32, self_collision=True, linear_damping=.1,
                    max_linear_velocity=3., enable_speculative_c_c_d=True),
                collision_props=[PhysxCollisionCfg(rest_offset=.00005, contact_offset=.0005)],
                visual_material=mod.sim_utils.PreviewSurfaceCfg(diffuse_color=(.96, .95, .90), roughness=.95),
                physics_material=PhysxSurfaceDeformableBodyMaterialCfg(
                    density=self.areal_mass / self.thickness, surface_thickness=self.thickness,
                    youngs_modulus=self.youngs_modulus, poissons_ratio=self.poissons_ratio,
                    surface_bend_stiffness=self.bend_modulus, elasticity_damping=.005, bend_damping=.01,
                    dynamic_friction=self.dynamic_friction)))

    def author(self, stage, env_path):
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

        mesh = UsdGeom.Mesh.Get(stage, f"{env_path}/{self.name}/sim_mesh")
        material = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        self.faces = _frozen(np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int).reshape(-1, 3))
        self.root_ids, self.free_edge_ids, self.terminal_patch_ids = material_labels(material)
        self.topology = topology_hash(len(material), self.faces)
        cache = UsdGeom.XformCache()
        world = cache.GetLocalToWorldTransform(mesh.GetPrim())
        parent = stage.GetPrimAtPath(f"{env_path}/{self.parent}")
        inverse = cache.GetLocalToWorldTransform(parent).GetInverse()
        anchors = [inverse.Transform(world.Transform(Gf.Vec3d(*material[i]))) for i in self.root_ids]
        self.root_local_anchors = _frozen(anchors)
        self._rest_world = _frozen([world.Transform(Gf.Vec3d(*p)) for p in material])
        attachment = stage.DefinePrim(f"{env_path}/{self.name}_root", "OmniPhysicsVtxXformAttachment")
        attachment.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(f"{env_path}/{self.name}")])
        attachment.CreateRelationship("omniphysics:src1").SetTargets([parent.GetPath()])
        attachment.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set(self.root_ids.tolist())
        attachment.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray).Set(
            [Gf.Vec3f(*p) for p in anchors])
        for suffix in ("sim_mesh", "geometry/mesh"):
            surface = UsdGeom.Mesh.Get(stage, f"{env_path}/{self.name}/{suffix}")
            surface.CreateDoubleSidedAttr(True)

    def read(self, base, boxes):
        nodes = _numpy(base.arena_world.get_nodal_positions_w(self.name))[0]
        velocities = _numpy(base.arena_world.get_nodal_velocities_w(self.name))[0]
        body = base.scene.rigid_objects[self.parent]
        parent = mk._T(quat_xyzw_to_matrix(_numpy(body.data.root_quat_w)[0]), _numpy(body.data.root_pos_w)[0])
        origin = _numpy(base.scene.env_origins)[0]
        tool_boxes = tuple(self.tool_boxes_reader(base)) if self.tool_boxes_reader is not None else ()
        expected_robot = validate_robot_coverage({arm: tuple(base.scene[arm].body_names)
            for arm in ('left_robot', 'right_robot')}, {name: len(pieces) for name, pieces in _robot_bounds().items()})
        access = evaluate_surface_access(nodes=nodes, velocities=velocities, triangles=self.faces,
            root_ids=self.root_ids, free_edge_ids=self.free_edge_ids, terminal_patch_ids=self.terminal_patch_ids,
            stub_transform=parent, root_local_anchors=self.root_local_anchors, outlet=self.outlet,
            robot_boxes=tuple(boxes)+tool_boxes, env_origin=origin, expected_topology_hash=self.topology, minimum_z=self.minimum_z)
        self.snapshot = SurfaceSnapshot(_frozen(nodes), _frozen(velocities), _frozen(parent), boxes, access,
            tool_boxes, tuple(origin), expected_robot, self.expected_tool_names)


def _refresh(env):
    base = env.unwrapped
    boxes = _robot_boxes(base)
    for paper in _PAPERS.values():
        paper.read(base, boxes)


def _patch_scene(mod):
    if getattr(mod, "_surface_paper", False):
        return
    mod._surface_paper = True
    original, original_states = mod.task_object, mod.object_states

    def task_object(spec):
        return _PAPERS[spec.name].build(mod) if isinstance(spec, PaperMarker) else original(spec)

    def object_states(env, names):
        out = original_states(env, [name for name in names if name not in _PAPERS])
        base, origin = env.unwrapped, _numpy(env.unwrapped.scene.env_origins)[0]
        for name in names:
            if name in _PAPERS:
                points = _numpy(base.arena_world.get_nodal_positions_w(name))[0] - origin
                velocity = _numpy(base.arena_world.get_nodal_velocities_w(name))[0]
                # This aggregate is for generic object information only. Paper
                # settling uses max nodal speed through the full snapshot.
                out[name] = ObjectState(points.mean(0), np.array((0., 0., 0., 1.)),
                    velocity.mean(0), np.zeros(3), points.min(0), points.max(0))
        return out

    mod.task_object, mod.object_states = task_object, object_states
    from isaaclab.scene.interactive_scene import InteractiveScene
    init = InteractiveScene.__init__

    def scene_init(self, *a, **k):
        init(self, *a, **k)
        assert len(self.env_prim_paths) == 1, "This task supports one native surface environment"
        for paper in _PAPERS.values():
            paper.author(self.stage, self.env_prim_paths[0])

    InteractiveScene.__init__ = scene_init


def _patch_embodiment(mod):
    cls = mod.YamArenaEmbodiment
    if getattr(cls, "_surface_paper", False):
        return
    cls._surface_paper = True
    reset, task_step = cls.reset, cls._task_step

    def reset_wrap(self, *a, **k):
        for paper in _PAPERS.values():
            paper.snapshot = None
        result = reset(self, *a, **k)  # Arena's reset events restore nodes before settling/cameras.
        _refresh(self._env)
        for paper in _PAPERS.values():
            raw = self._env.unwrapped.scene.deformable_objects[paper.name].data.default_nodal_state_w
            default = _numpy(raw)[0, :, :3]
            spacing = min(paper.width, paper.length) / 32
            assert default.shape == paper._rest_world.shape
            assert np.max(np.linalg.norm(default - paper._rest_world, axis=1)) < spacing / 2, "Solver node labels changed"
        return result

    def step_wrap(self, *a, **k):
        _refresh(self._env)
        return task_step(self, *a, **k)

    cls.reset, cls._task_step = reset_wrap, step_wrap


def _after_mechanisms(name, patch):
    """Compose with the mechanism import callback instead of shadowing its loader."""
    if name in sys.modules:
        patch(sys.modules[name])
        return
    for finder in sys.meta_path:
        if isinstance(finder, mk._AfterImport) and finder.name == name:
            previous = finder.patch

            def combined(module, previous=previous, patch=patch):
                previous(module)
                patch(module)

            finder.patch = combined
            return
    mk._when_imported(name, patch)


_after_mechanisms("yam_arena.scene", _patch_scene)
_after_mechanisms("yam_arena.inspect_embodiment", _patch_embodiment)
