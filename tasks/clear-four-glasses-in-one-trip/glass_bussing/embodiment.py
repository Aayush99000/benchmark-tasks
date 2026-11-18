"""`yam_bus`: the stock yam_arena embodiment plus the four tumblers, their ice and straws, and the bus tub
(glass_bussing.mech). The scorer gets every body in its own frame (a tumbler's origin is the centre of its underside, the
tub's the centre of its underside), and `left_tcp` and `right_tcp` (each gripper's fingertip midpoint and rotation). After
each control step the task's note_step runs. The robot, cameras, action and observations are the stock ones. The start is
one fixed layout: a seeded reset does not shift the objects."""

import os

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

DEBUG = os.environ.get("BUS_DEBUG") == "1"


def _place(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab spawners: xyzw
    x.AddOrientOp().Set(Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz))))
    return x.GetPrim()


def _builder(name):
    from . import mech

    if name == "tub":
        return mech.build_tub
    if name.endswith("_s"):
        return mech.build_straw
    if "_i" in name:
        return mech.build_ice
    return mech.build_glass


class BusEmbodiment(YamArenaEmbodiment):
    def __init__(self, **kwargs):
        kwargs["object_jitter"] = 0.0  # the ice and straws start inside the tumblers: one fixed layout
        super().__init__(**kwargs)

    def reset(self, scene, *, seed=None):
        self.task.module.reset_latch()
        return super().reset(scene, seed=seed)

    def _ensure_sim(self, goal):
        import atexit

        from yam_arena import inspect_embodiment as base

        if base._APP is None:  # Kit must be up before Arena is imported
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            base._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene

        original = scene.task_object
        start = self.task.module.START

        def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            _place(stage, prim_path, translation, orientation)
            name = prim_path.rstrip("/").rsplit("/", 1)[-1]
            return _builder(name)(stage, prim_path)

        def make_object(spec):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            if spec.name not in start:
                return original(spec)
            pos, quat = start[spec.name]
            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawn
            return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in pos), rotation_xyzw=tuple(float(v) for v in quat)))

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def _extra_states(self):
        from yam_arena import ik
        from yam_arena.station import BASES
        from yam_arena.task_file import ObjectState

        base = self._base
        origin = base.scene.env_origins[0].cpu().numpy().astype(float)
        arr = lambda t: t.detach().cpu().numpy().astype(float)  # noqa: E731
        zero = np.zeros(3)
        out = {}
        for arm, robot in (("left", "left_robot"), ("right", "right_robot")):
            arm_ids, finger_ids = self._ids[robot]
            q = base.scene[robot].data.joint_pos.torch[0]
            p, R = ik.tcp_pose(arr(q[arm_ids]), np.array(BASES[arm]))
            p = np.asarray(p, float)
            out[f"{arm}_tcp"] = ObjectState(pos=p, quat_xyzw=ik.matrix_to_quat_xyzw(R), lin_vel=zero, ang_vel=zero, low=p, high=p)
            if DEBUG:
                f = arr(q[finger_ids])
                out[f"{arm}_fingers"] = ObjectState(pos=np.array((f[0], f[1], 0.0)), quat_xyzw=np.array((0.0, 0.0, 0.0, 1.0)), lin_vel=zero, ang_vel=zero, low=zero, high=zero)
        world = base.arena_world
        for name in self.task.module.START:
            pose = arr(world.get_pose_w(name)[0])
            p = pose[:3] - origin
            out[name] = ObjectState(pos=p, quat_xyzw=pose[3:7], lin_vel=arr(world.get_root_linear_velocity_w(name)[0]),
                                    ang_vel=arr(world.get_root_angular_velocity_w(name)[0]), low=p, high=p)
        return out

    def _task_step(self):
        from yam_arena import scene

        original = scene.object_states
        last = {}
        m = self.task.module

        def object_states(env, names):
            names = [n for n in names if n not in m.START]  # these come from _extra_states
            last["s"] = {**original(env, names), **self._extra_states()}
            m.note_step(last["s"], 1.0 / self.info.control_hz)  # before the task's score() reads the trial's record
            return last["s"]

        scene.object_states = object_states
        try:
            result = super()._task_step()
        finally:
            scene.object_states = original
        if DEBUG and "s" in last:
            print(f"BUS t={self._t:6.2f} " + m.debug_line(last["s"]), flush=True)
        return result


def make_embodiment(**kwargs):
    return BusEmbodiment(**kwargs)
