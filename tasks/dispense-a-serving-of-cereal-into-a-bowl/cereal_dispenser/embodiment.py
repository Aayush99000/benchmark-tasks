"""`yam_cereal`: the stock yam_arena embodiment plus the cereal dispenser, its paddle wheel, the cereal and the bowl.
Created 11_16_26 (America/New_York).

The dispenser and the bowl are spawned by `build`; the wheel is a fixed-base articulation and the pieces are free rigid
bodies that the harness does not manage. After every control step the scorer gets, besides `bowl` and each gripper's
fingertip midpoint (`left_tcp`, `right_tcp`):
- `cereal`: every piece's centre and velocity (world frame), from one batched rigid-body view;
- `wheel`: the paddle wheel's joint angle (rad) and rate, from an articulation view.
Nothing is credited by bookkeeping: a piece is in the bowl only if the physics engine put it there.
"""

import atexit
import math
import os
from dataclasses import dataclass

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import plan

ARMS = (("left", "left_robot"), ("right", "right_robot"))
SETTLE_S = 2.5  # extra settling at the first reset, after the harness's own 0.5 s, so the pieces come to rest
DEBUG = os.environ.get("CEREAL_DEBUG") == "1"

plan.install()


@dataclass
class Pieces:
    pos: np.ndarray  # (n, 3) world, m
    vel: np.ndarray  # (n, 3) m/s


@dataclass
class Wheel:
    angle: float  # rad, positive about the dispenser's +x (out of its front)
    rate: float  # rad/s


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


def _np(t):
    if hasattr(t, "detach"):
        return t.detach().cpu().numpy().astype(float)
    if hasattr(t, "numpy"):
        return np.asarray(t.numpy(), float)
    return np.asarray(t, float)


def _like(template, values, n):
    """(values, indices 0..n-1) as arrays of the tensor API's own kind (torch or warp), on its device."""
    if hasattr(template, "detach"):
        import torch

        dev = template.device
        return torch.tensor(values, device=dev, dtype=template.dtype), torch.arange(n, device=dev, dtype=torch.int32)
    import warp as wp

    dev = template.device
    return wp.array(np.asarray(values, np.float32), dtype=template.dtype, device=dev), wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32, device=dev)


class CerealEmbodiment(YamArenaEmbodiment):
    _views = None
    _episodes = 0

    def _ensure_sim(self, goal):
        from yam_arena import inspect_embodiment as base

        if base._APP is None:  # Kit must be up before Arena's scene module is imported
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            base._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene

        original = scene.task_object

        def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            from . import build

            stage = omni.usd.get_context().get_stage()
            _place(stage, prim_path, translation, orientation)
            name = prim_path.rstrip("/").rsplit("/", 1)[-1]
            if name == "dispenser":
                return build.build_dispenser(stage, prim_path)
            if name == "bowl":
                return build.build_bowl(stage, prim_path)
            raise ValueError(f"yam_cereal builds the dispenser and the bowl, not {name!r}")

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawn
            kind = ObjectType.RIGID if spec.free else ObjectType.BASE
            return Object(name=spec.name, object_type=kind, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                            rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def _bind(self):
        from isaaclab_physx.physics.physx_manager import PhysxManager

        view = PhysxManager.get_physics_sim_view()
        root = "/World/envs/env_0/dispenser"
        pieces = view.create_rigid_body_view(f"{root}/Cereal/p_*")
        wheel = view.create_articulation_view(f"{root}/Wheel")
        if pieces.count < 1:
            raise RuntimeError("no cereal pieces found")
        self._views = {"pieces": pieces, "wheel": wheel}
        if DEBUG:
            w = wheel
            for name in ("get_dof_positions", "get_dof_stiffnesses", "get_dof_dampings", "get_dof_max_forces", "get_dof_friction_properties", "get_dof_limits"):
                try:
                    print("CEREAL bind", name, np.round(_np(getattr(w, name)()), 4).tolist(), flush=True)
                except Exception as exc:  # noqa: BLE001
                    print("CEREAL bind", name, "n/a", repr(exc)[:120], flush=True)
            print("CEREAL bind dofs", getattr(w.shared_metatype, "dof_names", None), "links", getattr(w.shared_metatype, "link_names", None), "fixed", getattr(w.shared_metatype, "fixed_base", None), flush=True)
        motor = os.environ.get("CEREAL_MOTOR")  # dev only: drive the wheel at this many deg/s, no robot needed
        if motor:
            cur = wheel.get_dof_velocity_targets()
            tgt = _np(cur).reshape(1, -1).astype(np.float32)
            tgt[0, 0] = math.radians(float(motor))
            wheel.set_dof_velocity_targets(*_like(cur, tgt, 1))
            if DEBUG:
                print("CEREAL bind velocity targets", np.round(_np(wheel.get_dof_velocity_targets()), 3).tolist(), flush=True)
        if DEBUG:
            print("CEREAL bind array type", type(wheel.get_dof_positions()), flush=True)
        self._start = {"pieces": _np(pieces.get_transforms()).copy(), "dof": _np(wheel.get_dof_positions()).copy()}

    def reset(self, scene, *, seed=None):
        m = self.task.module
        m.reset_trial()
        out = super().reset(scene, seed=seed)
        base = self._base
        if self._views is None:
            self._bind()
        elif self._episodes:  # a later episode in this process: pieces back to where they started, wheel to zero
            v = self._views
            n = v["pieces"].count
            tf = v["pieces"].get_transforms()
            v["pieces"].set_transforms(*_like(tf, self._start["pieces"], n))
            v["pieces"].set_velocities(*_like(v["pieces"].get_velocities(), np.zeros((n, 6), np.float32), n))
            dof = v["wheel"].get_dof_positions()
            v["wheel"].set_dof_positions(*_like(dof, self._start["dof"], 1))
            v["wheel"].set_dof_velocities(*_like(dof, np.zeros_like(self._start["dof"]), 1))
        for _ in range(max(1, round(SETTLE_S / base.step_dt))):
            self._env.step(self._ready)
        base.sim.render()
        out = self._observation()
        self._episodes += 1
        m.reset_trial()
        m.note_start(self._states())
        return out

    def _states(self):
        from yam_arena import ik
        from yam_arena.station import BASES
        from yam_arena.task_file import ObjectState

        base = self._base
        origin = _np(base.scene.env_origins[0])
        zero, ident = np.zeros(3), np.array((0.0, 0.0, 0.0, 1.0))
        out = {}
        for arm, robot in ARMS:
            arm_ids, _ = self._ids[robot]
            q = _np(base.scene[robot].data.joint_pos.torch[0, arm_ids])
            p = np.asarray(ik.tcp_pose(q, np.array(BASES[arm]))[0], float)
            out[f"{arm}_tcp"] = ObjectState(pos=p, quat_xyzw=ident, lin_vel=zero, ang_vel=zero, low=p, high=p)
        world = base.arena_world
        for name in self.task.free_objects:
            pose = _np(world.get_pose_w(name)[0])
            p = pose[:3] - origin
            out[name] = ObjectState(pos=p, quat_xyzw=pose[3:7], lin_vel=_np(world.get_root_linear_velocity_w(name)[0]),
                                    ang_vel=_np(world.get_root_angular_velocity_w(name)[0]), low=p, high=p)
        v = self._views
        tf = _np(v["pieces"].get_transforms())
        vel = _np(v["pieces"].get_velocities())
        out["cereal"] = Pieces(pos=tf[:, :3] - origin, vel=vel[:, :3])
        q = _np(v["wheel"].get_dof_positions()).reshape(-1)
        dq = _np(v["wheel"].get_dof_velocities()).reshape(-1)
        names = list(v["wheel"].shared_metatype.dof_names) if hasattr(v["wheel"], "shared_metatype") else []
        k = names.index("turn") if "turn" in names else 0
        out["wheel"] = Wheel(angle=float(q[k]), rate=float(dq[k]))
        return out

    def _task_step(self):
        from yam_arena import scene

        original = scene.object_states
        m = self.task.module
        last = {}

        def object_states(env, names):
            last["s"] = self._states()
            m.note_step(last["s"], 1.0 / self.info.control_hz)
            return last["s"]

        scene.object_states = object_states
        try:
            result = super()._task_step()
        finally:
            scene.object_states = original
        if DEBUG and "s" in last and round(self._t * self.info.control_hz) % 10 == 0:
            print("CEREAL " + m.debug_line(last["s"]), flush=True)
            dump = os.environ.get("CEREAL_DUMP")  # dev only: the latest piece positions and bowl pose, as .npz
            if dump:
                st = last["s"]
                np.savez(os.path.expanduser(dump), t=self._t, pieces=st["cereal"].pos, vel=st["cereal"].vel, bowl_pos=st["bowl"].pos,
                         bowl_quat=st["bowl"].quat_xyzw, wheel=st["wheel"].angle)
        return result


def make_embodiment(**kwargs):
    return CerealEmbodiment(**kwargs)
