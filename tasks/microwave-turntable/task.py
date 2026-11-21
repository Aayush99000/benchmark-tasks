"""RoboCurve microwave-turntable embodiment and continuous loaded-turn scorer."""

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MUJOCO_EGL_DEVICE_ID", "0")

import mujoco
import numpy as np
from inspect_robots import Observation
from inspect_robots.embodiment import (
    EmbodimentBase,
    EmbodimentInfo,
    RESETTABLE,
    SEEDABLE,
    PRIVILEGED_SUCCESS,
)
from inspect_robots.types import StepResult
from inspect_robots_yam.config import DEFAULT_CAMERAS, action_box, observation_space
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN

from scene import FLOOR, FRONT, HUB, LATCH_RADIUS, MOTOR_SPEED, OPEN, RING_R, TRAY_R, build
from scoring import HZ, LoadedTurn
from markers import PLAN_SIZE, PLAN_TICKS, PLAN_LAMP, PlanMarks, lamp_lit

HOME_POSE = [0.0, 0.65, 1.2, -0.3, 0.0, 0.0]
LIMIT = 1400
# Use the submitted instruction for both the embodiment and the recorded policies.
INSTRUCTION = json.loads(Path(__file__).with_name("task.json").read_text())["instruction"]


def mark_heights(body, radius, local_z=0):
    """Lowest and highest points of a circular painted line, as seen in elevation (mm)."""
    vertical = body.xmat.reshape(3, 3)[2]
    mid = body.xpos[2] - FLOOR + local_z * vertical[2]
    span = radius * np.hypot(vertical[0], vertical[1])
    return np.array((mid - span, mid + span)) * 1000


def in_band(heights, low, high):
    return bool(heights[0] >= low and heights[1] <= high)


class Microwave(EmbodimentBase):
    def __init__(self, cameras=True):
        self.model = mujoco.MjModel.from_xml_string(build())
        self.data = mujoco.MjData(self.model)
        self.cameras = cameras
        self.renderers = {}
        self.plan_marks = PlanMarks(PLAN_SIZE, PLAN_SIZE, PLAN_TICKS)
        self.steps_per_tick = round(1 / HZ / self.model.opt.timestep)
        self.arm_joints = np.array(
            [[self.model.joint(f"{s}_joint{i}").qposadr[0] for i in range(1, 9)] for s in ("left", "right")]
        )
        self.arm_drives = np.array(
            [[self.model.actuator(f"{s}_joint{i}").id for i in range(1, 9)] for s in ("left", "right")]
        )
        self.low = np.tile(np.r_[ik.LOWER, 0], 2)
        self.high = np.tile(np.r_[ik.UPPER, 1], 2)
        self.info = EmbodimentInfo(
            name="microwave_yam",
            action_space=action_box(self.low, self.high),
            observation_space=observation_space(176, 320, DEFAULT_CAMERAS),
            control_hz=HZ,
            is_simulated=True,
            capabilities=frozenset({RESETTABLE, SEEDABLE, PRIVILEGED_SUCCESS}),
            environment_id="microwave-turntable",
        )
        self.pads = [self.model.geom(f"{side}_tip_{finger}_pad").id
                     for side in ("left", "right") for finger in ("left", "right")]
        self.lamps = [self.model.geom(n).id for n in ("lamp_face", "lamp_plan")]
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[3] = 0
        self.opt.sitegroup[:] = 0

    def reset(self, scene, *, seed=None):
        mujoco.mj_resetData(self.model, self.data)
        self.instruction = scene.instruction or INSTRUCTION
        arm = np.r_[HOME_POSE, FINGER_OPEN, FINGER_OPEN]
        self.data.qpos[self.arm_joints] = arm
        self.data.ctrl[self.arm_drives] = arm
        self.data.joint("hinge").qpos[0] = OPEN
        self.data.ctrl[self.model.actuator("turntable_motor").id] = 0
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_step(self.model, self.data, nstep=200)
        self.steps, self.motor_ticks, self.previous_press = 0, 0, False
        self.turn = LoadedTurn()
        self.set_lamp(False)
        self.metrics = self.measure()
        return self.observe()

    def render(self, name, width=320, height=176):
        if (width, height) not in self.renderers:
            self.renderers[width, height] = mujoco.Renderer(self.model, height, width)
        renderer = self.renderers[width, height]
        renderer.update_scene(self.data, camera=name, scene_option=self.opt)
        return renderer.render().copy()

    def observe(self):
        joints = self.data.qpos[self.arm_joints]
        opening = np.clip(joints[:, 6:].mean(axis=1) / FINGER_OPEN, 0, 1)
        state = np.column_stack((joints[:, :6], opening)).ravel()
        images = {n: self.render(n) for n in DEFAULT_CAMERAS} if self.cameras else {}
        return Observation(
            images=images,
            state={"joint_pos": state},
            instruction=self.instruction,
            state_time=self.steps / HZ,
            image_times={n: self.steps / HZ for n in images},
        )

    def set_lamp(self, on):
        self.model.geom_rgba[self.lamps] = (0.20, 1.0, 0.22, 1) if on else (0.12, 0.15, 0.13, 1)

    def measure(self):
        d = self.data
        ring, tray, mug = (d.body(n) for n in ("roller_ring", "tray", "mug"))
        # All four pad centers must be on the operator side of the yellow sill in plan.
        clear = bool(np.all(d.geom_xpos[self.pads, 0] < FRONT))
        ring_error = float(np.linalg.norm(ring.xpos[:2] - HUB) * 1000)
        tray_error = float(np.linalg.norm(d.geom("tray_center").xpos[:2] - HUB) * 1000)
        ring_line = mark_heights(ring, RING_R)
        tray_line = mark_heights(tray, TRAY_R)
        foot_line = mark_heights(mug, .031, .006)
        lip_line = mark_heights(mug, .028, .067)
        mug_dot = d.geom("mug_dot").xpos[:2] - HUB
        radius = float(np.linalg.norm(mug_dot) * 1000)
        ring_ok = clear and ring_error <= 12 and in_band(ring_line, 5, 15)
        tray_ok = ring_ok and tray_error <= 6 and in_band(tray_line, 20, 28)
        mug_ok = tray_ok and 35 <= radius <= 78 and in_band(foot_line, 30, 38) and in_band(lip_line, 90, 101)
        latch_gap = float(np.linalg.norm(d.geom("latch_mark").xpos[:2] - d.body("latch_target").xpos[:2])*1000)
        on = self.motor_ticks > 0
        bearings = None
        if mug_ok and on and latch_gap <= 10:
            plan = self.render("plan", PLAN_SIZE, PLAN_SIZE)
            if lamp_lit(plan, PLAN_LAMP):
                bearings = self.plan_marks.bearings(plan)
        complete = self.turn.sample(*(bearings or (0, 0)), eligible=bearings is not None)
        score = 0.25 * sum((ring_ok, tray_ok, mug_ok, complete))
        return dict(
            score=score, turn_deg=max(0.0, float(self.turn.travel.min())),
            tray_mug_travel_deg=self.turn.travel.tolist(), tray_mug_bearing_deg=bearings,
            best_loaded_turn_deg=self.turn.best_deg, turn_resets=self.turn.resets,
            ring_seated=ring_ok, ring_error_mm=ring_error, ring_band_mm=ring_line.tolist(),
            tray_seated=tray_ok, tray_error_mm=tray_error, tray_band_mm=tray_line.tolist(),
            mug_loaded=mug_ok, mug_radius_mm=radius, foot_band_mm=foot_line.tolist(), lip_band_mm=lip_line.tolist(),
            latch_gap_mm=latch_gap, grippers_clear=clear, motor_on=on,
        )

    def step(self, action):
        target = np.clip(np.asarray(action.data, dtype=float), self.low, self.high).reshape(2, 7)
        self.data.ctrl[self.arm_drives[:, :6]] = target[:, :6]
        self.data.ctrl[self.arm_drives[:, 6:]] = target[:, 6, None] * FINGER_OPEN
        # Only the independent shaft actuator is powered. The ring, tray and mug are always free.
        for _ in range(self.steps_per_tick):
            closed = abs(self.data.joint("hinge").qpos[0]) <= 2 * math.asin(0.010 / (2 * LATCH_RADIUS))
            pressed = self.data.joint("button").qpos[0] >= 0.002
            if not closed:
                self.motor_ticks = 0
            elif pressed and not self.previous_press and self.motor_ticks == 0:
                self.motor_ticks = round(30 / self.model.opt.timestep)
            self.previous_press = pressed
            self.data.actuator("turntable_motor").ctrl[0] = MOTOR_SPEED if self.motor_ticks > 0 else 0.0
            self.motor_ticks = max(0, self.motor_ticks - 1)
            mujoco.mj_step(self.model, self.data)
        self.set_lamp(self.motor_ticks > 0)
        self.steps += 1
        self.metrics = self.measure()
        success = self.metrics["score"] == 1
        return StepResult(
            observation=self.observe(),
            terminated=success,
            termination_reason="success" if success else None,
            info={"success": success, **self.metrics},
        )

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()
