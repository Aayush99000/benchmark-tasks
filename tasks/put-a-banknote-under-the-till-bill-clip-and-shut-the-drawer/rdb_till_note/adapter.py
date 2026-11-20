"""Add the cash drawer, its till and the banknote to the stock Inspect Robots YAM station and grade each step."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as fixture
from . import grading
from . import till_model as m

if grading.CONTROL_HZ != fixture.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

PHYSICS_DT = 1. / fixture.PHYSICS_HZ          # PhysX reports contact impulses of the last 240 Hz substep
CURRENT = None                                # the live embodiment, for the scripted demonstration
TOUCHED = ("hold_2", "drawer", "strip_0", "strip_1", "stack")       # what the fingertips' contact forces are logged against


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def plain(value):
    """JSON-safe copy: numpy scalars and arrays become Python numbers and lists."""
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def mm(values):
    return [round(1000 * float(v), 1) for v in values]


class TillArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_till_note",
                            environment_id="isaaclab-arena-0.3.1-yam-till-note",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with an open cash drawer, its till insert with "
                                 "five sprung bill hold-downs, and one banknote lying on top of the drawer case. "
                                 "Observations are overhead RGB, both wrist RGB views, and robot joint and jaw "
                                 "positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
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
            objects.append(Object(name="till_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_till_note", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one till fixture, found {actor_roots}")
        actors = actor_roots[0]
        till = actors.rsplit("/", 1)[0] + "/Till"
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        # Free bodies: the stack and the note's eight strips (links of the note's own floating articulation).
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in fixture.LOOSE}
        self.stack_ids = self._torch.as_tensor([self.rows["stack"]], dtype=self._torch.int32, device=self._base.device)
        # The drawer unit: case fixed to the world, drawer on a slide, five hold-downs on hinges.
        self.till = self.physics.create_articulation_view(till)
        names = list(self.till.shared_metatype.dof_names)
        self.till_dofs = {name: names.index(name) for name in fixture.TILL_JOINTS}
        self.note = self.physics.create_articulation_view(actors + "/strip_0")
        names = list(self.note.shared_metatype.dof_names)
        self.note_dofs = [names.index(name) for name in fixture.NOTE_JOINTS]
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        tips = [root + "/" + arm.title() + "Arm" + links[name] for arm in fixture.ARMS
                for name in ("tip_left", "tip_right")]
        where = {"hold_2": till, "drawer": till}
        targets = [f"{where.get(name, actors)}/{name}" for name in TOUCHED]
        self.fingertips = self.physics.create_rigid_contact_view(tips, [targets] * len(tips))

    # --- reading the simulator ------------------------------------------------------------------------
    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())
        dofs = array(self.till.get_dof_positions()).reshape(-1)
        self.note_folds = array(self.note.get_dof_positions()).reshape(-1)[self.note_dofs]
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all() and np.isfinite(dofs).all()
                and np.isfinite(self.note_folds).all()):
            raise RuntimeError("Nonfinite till or note state")
        self.drawer_open = float(-dofs[self.till_dofs["slide"]])
        self.hold = np.array([dofs[self.till_dofs[f"hinge_{i}"]] for i in range(5)])
        rows = [self.rows[name] for name in fixture.STRIPS]
        self.strips = self.poses[rows, :3].copy()
        self.strip_rot = np.array([quat_xyzw_to_matrix(q) for q in self.poses[rows, 3:]])
        self.stack = self.poses[self.rows["stack"], :3].copy()

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"strips": self.strips.copy(), "strip_rot": self.strip_rot.copy(), "hold": self.hold.copy(),
                           "drawer_open": self.drawer_open, "stack": self.stack.copy()}

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        device = self._base.device
        one = torch.arange(1, dtype=torch.int32, device=device)
        # Drawer open, hold-downs down (the middle one on the notes), their springs set as authored.
        positions = self.till.get_dof_positions().clone()
        targets = torch.zeros_like(positions)
        positions[0, self.till_dofs["slide"]] = -m.TRAVEL
        for i in range(5):
            rest = m.HOLD_ON_STACK + np.radians(.3) if i == m.STOCKED else m.HOLD_ON_FLOOR
            positions[0, self.till_dofs[f"hinge_{i}"]] = float(rest)
            targets[0, self.till_dofs[f"hinge_{i}"]] = float(np.radians(m.HOLD_SPRING_REST_DEG))
        self.till.set_dof_positions(positions, one)
        self.till.set_dof_velocities(torch.zeros_like(positions), one)
        self.till.set_dof_position_targets(targets, one)
        # The note flat on the case top: root strip at its start pose, every fold at zero.
        root = self.note.get_root_transforms().clone()
        x, y, z = m.NOTE_START[0]
        root[0] = torch.tensor((x, y, z + fixture.TOP, 0., 0., 0., 1.), device=device, dtype=root.dtype)
        self.note.set_root_transforms(root, one)
        self.note.set_root_velocities(torch.zeros_like(self.note.get_root_velocities()), one)
        folds = torch.zeros_like(self.note.get_dof_positions())
        self.note.set_dof_positions(folds, one)
        self.note.set_dof_velocities(folds, one)
        self.note.set_dof_position_targets(folds, one)
        # The notes already in the till.
        poses = self.body_view.get_transforms().clone()
        x, y, z = m.STACK_START
        poses[self.rows["stack"]] = torch.tensor((x, y, z + fixture.TOP, 0., 0., 0., 1.), device=device, dtype=poses.dtype)
        self.body_view.set_transforms(poses, self.stack_ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), self.stack_ids)
        observation = super().reset(scene, seed=seed)
        self._read()
        # Fail loudly, with the numbers, if the scene did not come up as declared.
        found = {"drawer_open_mm": round(1000 * self.drawer_open, 2), "hold_deg": np.degrees(self.hold).round(2).tolist(),
                 "note_strips_mm": (1000 * self.strips).round(1).tolist(), "stack_mm": mm(self.stack),
                 "note_folds_deg": np.degrees(self.note_folds).round(2).tolist()}
        if abs(self.drawer_open - m.TRAVEL) > .004:
            raise RuntimeError(f"Drawer is not open at reset: {found}")
        if abs(self.hold[m.STOCKED] - m.HOLD_ON_STACK) > np.radians(3.) or np.abs(self.hold - m.HOLD_ON_FLOOR)[
                [i for i in range(5) if i != m.STOCKED]].max() > np.radians(3.):
            raise RuntimeError(f"Hold-downs are not down at reset: {found}")
        if np.abs(self.strips - m.NOTE_START).max() > .006 or np.linalg.norm(self.stack - m.STACK_START) > .004:
            raise RuntimeError(f"Note or stack is off its start place at reset: {found}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        declared = {"stack": m.STACK_MASS, **dict.fromkeys(fixture.STRIPS, m.STRIP_MASS)}
        for name, mass in declared.items():
            if not np.isclose(masses[self.rows[name]], mass, rtol=.02, atol=1e-5):
                raise RuntimeError(f"{name} mass {masses[self.rows[name]]} differs from the declared {mass}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.reset_state = found
        self.grading = grading.TillNoteGrade()
        self.tick = 0
        self.peak_interarm_n = 0.       # a diagnostic; it does not enter the score
        self.peak = {"tab_n": 0., "note_end_n": 0., "drawer_n": 0., "stack_n": 0., "hold_deg": float(np.degrees(self.hold[m.STOCKED])),
                     "note_fold_deg": 0.}
        self.final_grade = None
        self.diagnostics = []
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        # rows: left tip_left, left tip_right, right tip_left, right tip_right; columns: TOUCHED
        forces = np.linalg.norm(array(self.fingertips.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
        tcp = [ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0] for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]
        grade = plain(self.grading.update(strips=self.strips, strip_rot=self.strip_rot, stack=self.stack,
                                          drawer_open=self.drawer_open, hold_angles=self.hold, tcp=tcp))
        self.final_grade = grade
        self.tick += 1
        self._publish_demo_state()
        column = {name: i for i, name in enumerate(TOUCHED)}
        self.peak["tab_n"] = max(self.peak["tab_n"], float(forces[:2, column["hold_2"]].min()))
        self.peak["note_end_n"] = max(self.peak["note_end_n"], float(forces[2:, column["strip_0"]].min()))
        self.peak["drawer_n"] = max(self.peak["drawer_n"], float(forces[:, column["drawer"]].sum()))
        self.peak["stack_n"] = max(self.peak["stack_n"], float(forces[:, column["stack"]].sum()))
        self.peak["hold_deg"] = max(self.peak["hold_deg"], float(np.degrees(self.hold[m.STOCKED])))
        self.peak["note_fold_deg"] = max(self.peak["note_fold_deg"], float(np.degrees(np.abs(self.note_folds).max())))
        if self.tick % 5 == 0:
            self.diagnostics.append(self._diagnose(action, joints, tcp, forces, column, grade))
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"till_note_grade": grade})

    def _diagnose(self, action, joints, tcp, forces, column, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        command = np.asarray(getattr(action, "data", joints), dtype=float).reshape(-1)
        wanted = [ik.tcp_pose(command[i:i + 6], fixture.BASES[arm])[0] for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]
        x0 = m.drawer_x(self.drawer_open)
        return {
            "tick": self.tick,
            "gripper_commanded_mm": [mm(p) for p in wanted],              # left, right; from the commanded joints
            "gripper_measured_mm": [mm(p) for p in tcp],                  # from the joint readings
            "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],     # 1 = 94 mm
            "gripper_opening_commanded": [round(float(command[6]), 3), round(float(command[13]), 3)],
            "hold_down_deg": np.degrees(self.hold).round(2).tolist(),
            "tab_from_left_gripper_mm": mm(m.tab_centre(self.hold[m.STOCKED], self.drawer_open) - tcp[0]),
            "drawer_open_mm": round(1000 * self.drawer_open, 2),
            "note_strips_mm": (1000 * self.strips).round(1).tolist(),
            "note_folds_deg": np.degrees(self.note_folds).round(1).tolist(),
            "note_strip_tilt_deg": np.degrees(np.arcsin(np.clip(self.strip_rot[:, 2, 0], -1., 1.))).round(1).tolist(),
            "note_end_from_right_gripper_mm": mm(self.strips[0] - tcp[1]),
            "note_side_in_compartment_mm": mm(self.strips[:, 1] - m.COMPARTMENT_Y[m.STOCKED]),
            "note_far_edge_in_drawer_mm": round(1000 * float(self.strips[-1][0] + m.STRIP_L / 2 - x0), 1),
            "stack_in_drawer_mm": mm(self.stack - (x0, 0., 0.)),
            "left_tips_on_tab_n": forces[:2, column["hold_2"]].round(2).tolist(),
            "right_tips_on_note_end_n": forces[2:, column["strip_0"]].round(2).tolist(),
            "right_tips_on_second_strip_n": forces[2:, column["strip_1"]].round(2).tolist(),
            "tips_on_drawer_n": forces[:, column["drawer"]].round(2).tolist(),
            "tips_on_stack_n": forces[:, column["stack"]].round(2).tolist(),
            "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class TillScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import TillReference

        # TILL_DEMO_VARIANT=no_lift runs the negative control (hold-down never lifted); unset runs the demonstration.
        self.reference = TillReference(os.environ.get("TILL_DEMO_VARIANT") or None)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the note's strips, the
        # hold-downs' angles, the drawer's opening), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return TillArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = TillScripted()
    policy.info = replace(YamScriptedPolicy().info, name="till_note_scripted")
    return policy
