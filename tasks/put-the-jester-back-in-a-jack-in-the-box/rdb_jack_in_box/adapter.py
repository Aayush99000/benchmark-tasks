"""Add the jack-in-the-box to the stock Inspect Robots YAM station, grade every step and log diagnostics."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as fixture
from . import grading
from . import layout as L
from .scripted import fingertips

if grading.CONTROL_HZ != fixture.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

BODIES = fixture.ACTORS
TOY = ("box", "lid", "coil", "head", "catch")
CURRENT = None                                       # the live embodiment, for the scripted demonstration


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def r1(value):
    return round(float(value), 1)


class JackArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_jack_in_box",
                            environment_id="isaaclab-arena-0.3.1-yam-jack-in-the-box",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a tin jack-in-the-box that has popped open: "
                                 "its lid stands open and the jester is up on its spring. Observations are overhead "
                                 "RGB, both wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.tick = 0
        self.diagnostics = []
        self.contact_trace = []
        self.final_grade = None
        CURRENT = self

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        from .recording import review_camera

        def make_env(embodiment, *, device, control_hz, **kwargs):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
            from isaaclab_arena.scene.scene import Scene

            objects = scenes.task_room("station", embodiment.rig.top_cam)
            objects.append(Object(name="jack_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_jack_in_box", scene=Scene(objects),
                                           embodiment=embodiment, env_cfg_callback=scenes.env_cfg_callback(control_hz))
            return ArenaEnvBuilder(env, ArenaEnvBuilderCfg(
                num_envs=1, solve_relations=False, device=device)).make_registered()

        with review_camera():
            from yam_arena import scene as scenes

            stock = scenes.make_env
            scenes.make_env = make_env
            try:
                super()._ensure_sim(None)
            finally:
                scenes.make_env = stock

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()
        actor_roots = [str(p.GetPath()) for p in stage.Traverse()
                       if p.GetName() == "Actors" and str(p.GetPath()).startswith(root + "/")]
        if len(actor_roots) != 1:
            raise RuntimeError(f"Expected one jack-in-the-box fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in BODIES}
        links = asset_link_paths()
        self.arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                          for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            self.arm_links["left"], [self.arm_links["right"]] * len(self.arm_links["left"]))
        # One contact view per pair, so that no reading is shared or mislabelled.
        self.arm_views = {}
        for arm, sensors in self.arm_links.items():
            for name in TOY:
                self.arm_views[arm, name] = self.physics.create_rigid_contact_view(
                    sensors, [[f"{actors}/{name}"]] * len(sensors))
        self.pair_views = {"lid_on_head": self.physics.create_rigid_contact_view([f"{actors}/lid"], [[f"{actors}/head"]]),
                           "catch_on_lid": self.physics.create_rigid_contact_view([f"{actors}/catch"], [[f"{actors}/lid"]]),
                           "head_on_box": self.physics.create_rigid_contact_view([f"{actors}/head"], [[f"{actors}/box"]])}

    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):
            raise RuntimeError("Nonfinite jack-in-the-box state")

    def pos(self, name):
        return self.poses[self.rows[name], :3].copy()

    def rot(self, name):
        return quat_xyzw_to_matrix(self.poses[self.rows[name], 3:])

    def toy_state(self):
        """Lid angle, head centre, spring extension and lean, catch travel, box pose: tabletop frame."""
        box_rot = self.rot("box")
        box_pos = self.pos("box")
        lid = L.lid_angle_from_rotation(box_rot.T @ self.rot("lid"))
        coil = box_rot.T @ self.rot("coil")
        lean = float(np.arctan2(coil[0, 2], coil[2, 2]))
        pivot = box_pos + box_rot @ np.array((0., 0., L.PIVOT_Z))
        head = self.pos("head")
        s = float((head - pivot) @ (self.rot("coil")[:, 2]))
        catch = float((box_rot.T @ (self.pos("catch") - box_pos))[0])
        return {"lid": lid, "head": head, "s": s, "lean": lean, "catch": catch, "box": box_pos, "box_rot": box_rot}

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        # Every body back where the scene file has it: lid open against its stop, jester sprung out, catch at rest.
        poses = self.body_view.get_transforms().clone()
        for name, ((x, y, z), quat) in fixture.WORLD_START.items():
            poses[self.rows[name]] = torch.tensor((x, y, z, *quat), device=poses.device, dtype=poses.dtype)
        ids = torch.arange(poses.shape[0], dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), ids)
        observation = super().reset(scene, seed=seed)
        self._read()
        state = self.toy_state()
        self.state_after_reset = self._state_row(state)
        if abs(np.degrees(state["lid"]) - np.degrees(L.LID_OPEN)) > 3. or abs(state["s"] - L.S_POPPED) > .006:
            raise RuntimeError(f"Toy is not at its start after reset: {self.state_after_reset}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        if not np.allclose([masses[self.rows[name]] for name in BODIES], L.MASSES, rtol=.01, atol=0):
            raise RuntimeError("A body's mass differs from the declared scene")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.grading = grading.JackGrade()
        self.tick = 0
        # Diagnostics only; none of these enters the score.
        self.peak_interarm_n = 0.
        self.peaks = {}
        self.final_grade = None
        self.diagnostics = []
        self.contact_trace = []
        self.demo_state = state
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _forces(self):
        """Contact force (N, last physics step) of each arm on each part of the toy, and between toy parts."""
        out = {}
        for (arm, name), view in self.arm_views.items():
            out[f"{arm}_on_{name}"] = float(np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum())
        for key, view in self.pair_views.items():
            out[key] = float(np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum())
        return out

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = self._forces()
        for key, value in forces.items():
            self.peaks[key] = max(self.peaks.get(key, 0.), value)
        touching = any(forces[f"{arm}_on_{name}"] > grading.TOUCH_N for arm in fixture.ARMS for name in TOY)
        tcp = np.array([ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0]
                        for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
        state = self.toy_state()
        grade = self.grading.update(np.degrees(state["lid"]), state["head"], state["box"], state["box_rot"], tcp, touching)
        self.final_grade = grade
        self.tick += 1
        self.demo_state = state
        # Every step: the key contacts and the toy's state, compact (for whoever reads the run afterwards).
        self.contact_trace.append([self.tick, r1(10 * np.degrees(state["lid"])) / 10, r1(1000 * (state["s"] - L.S_SHUT)),
                                   r1(10 * np.degrees(state["lean"])) / 10, round(1000 * state["catch"], 2),
                                   *(round(forces[k], 2) for k in ("left_on_head", "left_on_lid", "right_on_lid",
                                                                   "lid_on_head", "catch_on_lid", "left_on_box"))])
        if self.tick % 5 == 0 or grade["score"] == 1.:
            self.diagnostics.append(self._diagnose(joints, state, forces, grade))
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"jack_grade": grade})

    def _state_row(self, state):
        box_shift = state["box"] - (L.BOX_X, 0., 0.)
        return {"lid_deg": round(float(np.degrees(state["lid"])), 2),
                "head_top_above_rim_mm": r1(1000 * grading.head_state(state["head"], state["box"], state["box_rot"])[0]),
                "head_mm": [r1(1000 * v) for v in state["head"]], "spring_extension_mm": r1(1000 * state["s"]),
                "lean_deg": round(float(np.degrees(state["lean"])), 2), "catch_mm": round(1000 * state["catch"], 2),
                "box_shift_mm": [r1(1000 * v) for v in box_shift], "box_tilt_deg": round(grading.tilt_deg(state["box_rot"]), 2),
                "box_yaw_deg": round(grading.yaw_deg(state["box_rot"]), 2)}

    def _diagnose(self, joints, state, forces, grade):
        """One row for the run log, in millimetres, degrees and newtons. RESULT.json only; no policy receives it."""
        return {"tick": self.tick, **self._state_row(state),
                "fingertips_mm": [[r1(1000 * v) for v in fingertips(joints, arm)] for arm in fixture.ARMS],
                "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],
                "forces_n": {k: round(v, 2) for k, v in forces.items() if v > 0.},
                "score": grade["score"], "hands_clear": grade.get("hands_clear"),
                "gripper_clearance_mm": grade.get("gripper_clearance_mm"), "hold_ticks": grade["hold_ticks"]}

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


CONTACT_TRACE_COLUMNS = ["tick", "lid_deg", "spring_above_shut_mm", "lean_deg", "catch_mm", "left_on_head_n",
                         "left_on_lid_n", "right_on_lid_n", "lid_on_head_n", "catch_on_lid_n", "left_on_box_n"]


class JackScripted(PolicyBase):
    def __init__(self, variant="full"):
        self.variant = variant

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import JackReference

        self.reference = JackReference(self.variant)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (box pose, lid
        # angle, head position, catch travel), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return JackArena(**kwargs)


def make_scripted(variant=None):
    from yam_arena.scripted import YamScriptedPolicy

    policy = JackScripted(variant or os.environ.get("JACK_IN_BOX_DEMO", "full"))
    policy.info = replace(YamScriptedPolicy().info, name="jack_in_box_scripted")
    return policy
