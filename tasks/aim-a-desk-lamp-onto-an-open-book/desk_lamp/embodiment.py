"""`yam_desk_lamp`: the stock yam_arena embodiment plus the Anglepoise Type 75 desk lamp (desk_lamp.mech), one
articulation with a free cast-iron base, and the open notebook, one loose rigid body. Created 11_16_26
(America/New_York).

Both are built by desk_lamp.mech in place of the task file's placeholder boxes "lamp" and "book". The scorer gets, besides
the stock objects: `left_tcp` and `right_tcp` (each gripper's fingertip midpoint); `lamp_base`, `lamp_lower`,
`lamp_upper` and `lamp_head` (each link frame's world pose: the base's centre on the desk, the shoulder, elbow and head
knuckle pivots; `lamp_head`'s orientation is the shade's); `lamp_q` (the four joint angles in the task's sign convention,
swivel, shoulder, elbow in pos and head in low[0]); and `book` (the notebook's centre on the desk and orientation). They are
read through the PhysX tensor views. The harness is unchanged."""

import math
import os

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

DEBUG = os.environ.get("LAMP_DEBUG") == "1"
LAMP, BOOK = "anglepoise", "book"
LINKS = ("base", "lower", "upper", "head")
SIGN = {"swivel": 1.0, "shoulder": -1.0, "elbow": -1.0, "head": -1.0}  # PhysX joint angle -> task angle (mech docstring)


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab spawners: xyzw
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


def _np(a):
    """A tensor-view getter's result as a NumPy copy (the view may hand back warp, torch or NumPy arrays)."""
    if hasattr(a, "numpy"):
        try:
            return np.array(a.numpy(), dtype=np.float32)
        except TypeError:
            pass
    return np.array(a.detach().cpu().numpy() if hasattr(a, "detach") else a, dtype=np.float32)


def _put(setter, data, like, n_idx=1):
    """Call a tensor-view setter with a NumPy array, in the kind of array and on the device the getter returned."""
    idx = np.arange(n_idx, dtype=np.int32)
    if type(like).__module__.startswith("warp"):
        import warp as wp

        setter(wp.from_numpy(data.astype(np.float32), dtype=wp.float32, device=like.device),
               wp.from_numpy(idx, dtype=wp.int32, device=like.device))
    elif hasattr(like, "detach"):
        import torch

        setter(torch.tensor(data, dtype=torch.float32, device=like.device), torch.tensor(idx, dtype=torch.int32, device=like.device))
    else:
        setter(data.astype(np.float32), idx)


class DeskLampEmbodiment(YamArenaEmbodiment):
    _sim_view = None
    _art_view = None
    _book_view = None
    _paths = None
    _dof = None
    _link = None

    def reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        self.task.module.reset_latch()
        self._start_pose()
        return super().reset(scene, seed=seed)

    def _ensure_sim(self, goal):
        import atexit

        from yam_arena import inspect_embodiment as base

        if self._env is not None:
            return super()._ensure_sim(goal)
        if base._APP is None:  # Kit must be up before Arena is imported
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            base._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene

        original = scene.task_object
        owner = self
        owner._paths = {}

        def spawner(name):
            def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
                import omni.usd

                from . import mech

                stage = omni.usd.get_context().get_stage()
                _place(stage, prim_path, translation, orientation)
                owner._paths[name] = prim_path
                return mech.build_lamp(stage, prim_path) if name == LAMP else mech.build_book(stage, prim_path)

            return spawn

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            if spec.name not in (LAMP, BOOK):
                return original(spec)
            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawner(spec.name)
            yaw = float(getattr(spec, "yaw", 0.0) or 0.0)
            return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                            rotation_xyzw=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))))

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    # -- PhysX tensor views ----------------------------------------------------------------------------------------------
    def _views(self):
        if self._art_view is None:
            import omni.physics.tensors as tensors

            from . import mech

            sim_view = None
            try:
                from isaaclab.sim import SimulationContext

                sim_view = SimulationContext.instance().physics_sim_view
            except Exception:  # noqa: BLE001
                sim_view = None
            if sim_view is None:
                sim_view = tensors.create_simulation_view("torch")
                sim_view.set_subspace_roots("/")
            self._sim_view = sim_view
            self._art_view = sim_view.create_articulation_view(self._paths[LAMP] + "/base")
            self._book_view = sim_view.create_rigid_body_view(self._paths[BOOK])
            meta = self._art_view.shared_metatype
            names = list(meta.dof_names)
            self._dof = {j: names.index(f"j_{j}") for j in mech.DOFS}
            links = list(meta.link_names)
            self._link = {k: links.index(k) for k in LINKS}
            v = self._art_view
            fr = _np(v.get_dof_friction_properties())
            for j, torque in mech.FRICTION.items():
                fr[0, self._dof[j], :] = (torque, torque, 0.0)
            _put(v.set_dof_friction_properties, fr, v.get_dof_friction_properties())
            if DEBUG:
                print("LAMP dofs", names, "links", links, "friction", _np(v.get_dof_friction_properties()).tolist(), flush=True)
        return self._art_view

    def _start_pose(self):
        v = self._views()
        m = self.task.module
        pos = _np(v.get_dof_positions())
        for j, a in m.START_Q.items():
            pos[0, self._dof[j]] = SIGN[j] * a
        _put(v.set_dof_positions, pos, v.get_dof_positions())
        _put(v.set_dof_velocities, pos * 0.0, v.get_dof_velocities())

    def _extra_states(self):
        from yam_arena import ik
        from yam_arena.station import BASES
        from yam_arena.task_file import ObjectState

        base = self._base
        arr = lambda t: t.detach().cpu().numpy().astype(float)  # noqa: E731
        zero, still = np.zeros(3), np.array((0.0, 0.0, 0.0, 1.0))
        out = {}
        for arm, robot in (("left", "left_robot"), ("right", "right_robot")):
            arm_ids, _ = self._ids[robot]
            q = arr(base.scene[robot].data.joint_pos.torch[0, arm_ids])
            p = np.asarray(ik.tcp_pose(q, np.array(BASES[arm]))[0], float)
            out[f"{arm}_tcp"] = ObjectState(pos=p, quat_xyzw=still, lin_vel=zero, ang_vel=zero, low=p, high=p)
        v = self._views()
        tf = _np(v.get_link_transforms())[0]
        vel = _np(v.get_link_velocities())[0]
        for k in LINKS:
            i = self._link[k]
            p = tf[i, :3].astype(float)
            out[f"lamp_{k}"] = ObjectState(pos=p, quat_xyzw=tf[i, 3:7].astype(float), lin_vel=vel[i, :3].astype(float),
                                           ang_vel=vel[i, 3:6].astype(float), low=p, high=p)
        qd = _np(v.get_dof_positions())[0]
        qv = _np(v.get_dof_velocities())[0]
        q = [SIGN[j] * float(qd[self._dof[j]]) for j in ("swivel", "shoulder", "elbow", "head")]
        w = [SIGN[j] * float(qv[self._dof[j]]) for j in ("swivel", "shoulder", "elbow", "head")]
        out["lamp_q"] = ObjectState(pos=np.array(q[:3]), quat_xyzw=still, lin_vel=np.array(w[:3]), ang_vel=np.array((w[3], 0.0, 0.0)),
                                    low=np.array((q[3], 0.0, 0.0)), high=np.array((q[3], 0.0, 0.0)))
        bt = _np(self._book_view.get_transforms())[0].astype(float)
        bv = _np(self._book_view.get_velocities())[0].astype(float)
        out[BOOK] = ObjectState(pos=bt[:3], quat_xyzw=bt[3:7], lin_vel=bv[:3], ang_vel=bv[3:6], low=bt[:3], high=bt[:3])
        return out

    def _task_step(self):
        from yam_arena import scene

        original = scene.object_states
        m = self.task.module
        extra = ("left_tcp", "right_tcp", "lamp_q", BOOK, LAMP, *(f"lamp_{k}" for k in LINKS))

        def object_states(env, names):
            names = [n for n in names if n not in extra]
            return {**(original(env, names) if names else {}), **self._extra_states()}

        scene.object_states = object_states
        try:
            st0 = object_states(self._env, [])
            m.note_step(st0, 1.0 / self.info.control_hz)
            result = super()._task_step()
        finally:
            scene.object_states = original
        if DEBUG:
            print(m.debug_line(st0, self._t), flush=True)
        return result


def make_embodiment(**kwargs):
    return DeskLampEmbodiment(**kwargs)
