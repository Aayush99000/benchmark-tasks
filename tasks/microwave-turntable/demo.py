"""Open-loop Cartesian waypoints converted to YAM joint actions by robot-only kinematics."""

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
from inspect_robots import Action, Scene
from PIL import Image, ImageDraw, ImageFont

import scene as S
from task import HZ, LIMIT, INSTRUCTION, Microwave
from task import HOME_POSE as READY
import mujoco


def frame(approach, closing):
    z = np.array(approach, dtype=float)
    z /= np.linalg.norm(z)
    x = np.array(closing, dtype=float)
    x -= z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def xyz(xy, z):
    return np.r_[xy, z]


class Planner:
    """Only the robot's public kinematic model enters the planner."""

    def __init__(self, model):
        self.model = model
        self.kin = mujoco.MjData(model)
        self.q = np.tile(np.r_[READY, 1.0], (2, 1))
        self.joints = []
        self.dofs = []
        self.sites = []
        for arm, side in enumerate(("left", "right")):
            joints = [model.joint(f"{side}_joint{i}") for i in range(1, 7)]
            self.joints.append([j.qposadr[0] for j in joints])
            self.dofs.append([j.dofadr[0] for j in joints])
            self.sites.append(model.site(side + "_tcp").id)
            self.kin.qpos[self.joints[arm]] = READY
        mujoco.mj_kinematics(model, self.kin)

    def pose(self, arm):
        site = self.sites[arm]
        return self.kin.site_xpos[site].copy(), self.kin.site_xmat[site].reshape(3, 3).copy()

    def ik(self, arm, pos, rot):
        site = self.sites[arm]
        j = self.joints[arm]
        dof = self.dofs[arm]
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        bounds = self.model.jnt_range[[self.model.dof_jntid[i] for i in dof]]
        for _ in range(25):
            self.kin.qpos[j] = self.q[arm, :6]
            mujoco.mj_kinematics(self.model, self.kin)
            mujoco.mj_comPos(self.model, self.kin)
            now, orientation = self.pose(arm)
            err = np.r_[pos - now, 0.22 * Rotation.from_matrix(rot @ orientation.T).as_rotvec()]
            if np.linalg.norm(err) < 0.000035:
                break
            mujoco.mj_jacSite(self.model, self.kin, jp, jr, site)
            jac = np.vstack((jp[:, dof], 0.22 * jr[:, dof]))
            update = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 0.00008, err)
            self.q[arm, :6] = np.clip(
                self.q[arm, :6] + np.clip(update, -0.18, 0.18), bounds[:, 0], bounds[:, 1]
            )

    def move(self, arm, pos, rot, opening, steps):
        p0, r0 = self.pose(arm)
        g0 = self.q[arm, 6]
        relative = Rotation.from_matrix(rot @ r0.T).as_rotvec()
        for k in range(1, steps + 1):
            u = k / steps
            s = u * u * (3 - 2 * u)
            self.ik(arm, p0 + s * (pos - p0), Rotation.from_rotvec(relative * s).as_matrix() @ r0)
            self.q[arm, 6] = g0 + s * (opening - g0)
            yield self.q.ravel().copy()

    def park(self, arm, steps):
        q0 = self.q[arm].copy()
        for k in range(1, steps + 1):
            u = k / steps
            self.q[arm] = q0 + (np.r_[READY, 1.0] - q0) * u * u * (3 - 2 * u)
            yield self.q.ravel().copy()
        self.kin.qpos[self.joints[arm]] = self.q[arm, :6]
        mujoco.mj_kinematics(self.model, self.kin)


def script(p, miss_hub=False, stop_after=None, interrupt=False):
    L, R = 0, 1
    if stop_after == "reset":
        for _ in range(HZ):
            yield "inspect reset", p.q.ravel().copy()
        return

    def series(label, gen):
        for row in gen:
            yield label, row

    def move(tag, arm, xy, z, r, g, n):
        return series(tag, p.move(arm, xyz(xy, z), r, g, n))

    for name, arm, center, z0, z1, radius, dy, span in (
        ("ring", R, S.RING_START, S.RING_PICK_Z, S.FLOOR + 0.028, S.RING_R, 0.18, 0.44),
        ("tray", L, S.TRAY_START, S.TRAY_PICK_Z, S.FLOOR + 0.041, S.TRAY_R - 0.025, -0.18, 0.72),
    ):
        direction = np.array([math.sqrt(1 - dy * dy), dy])
        level = frame((*direction, -0.29 if name == "ring" else -0.10), (0, 0, 1 if arm == R else -1))
        offset = np.array([-radius, 0]) if name == "ring" else -radius * direction
        pick = center + offset
        dest = S.HUB + offset + (np.array([0.004, -0.004]) if name == "ring" else 0)
        if name == "tray" and miss_hub:
            dest = dest + np.array([0, 0.018])
        yield from move(
            "approach " + name,
            arm,
            pick - (0.033 if name == "ring" else 0.060) * direction,
            z0 + (0.022 if name == "ring" else 0),
            level,
            span,
            39,
        )
        yield from move("position at " + name, arm, pick, z0, level, span, 22)
        yield from move("grasp " + name, arm, pick, z0, level, 0.0, 16)
        carry_z = S.FLOOR + (0.110 if name == "tray" else 0.073)
        yield from move("lift " + name, arm, pick, carry_z, level, 0.0, 27)
        if name == "tray":
            yield from move(
                "line up with opening", arm, [pick[0], dest[1]], carry_z, level, 0.0, 32
            )
        yield from move("carry " + name, arm, dest, carry_z, level, 0.0, 45)
        yield from move("seat " + name, arm, dest, z1, level, 0.0, 29)
        yield from move("release " + name, arm, dest, z1, level, 0.56, 18)
        yield from move("withdraw " + name, arm, dest - 0.19 * direction, z1 + 0.015, level, 0.56, 32)
        yield from series("park " + name, p.park(arm, 32))
        if stop_after == name:
            for _ in range(HZ):
                yield "inspect placement", p.q.ravel().copy()
            return

    grip = frame((0.94, 0, -0.34), (0, -1, 0))
    pick = S.MUG_START
    dest = S.HUB + S.OFFSET
    yield from move("approach mug", R, pick, S.TABLE + 0.136, grip, 1.0, 34)
    yield from move("lower to mug", R, pick, S.TABLE + 0.049, grip, 1.0, 20)
    yield from move("grasp mug", R, pick, S.TABLE + 0.049, grip, 0.46, 15)
    yield from move("lift mug", R, pick, S.TABLE + 0.133, grip, 0.46, 24)
    yield from move("clear fascia", R, np.array([0.244, -0.056]), S.FLOOR + 0.110, grip, 0.46, 27)
    yield from move("carry mug", R, dest, S.FLOOR + 0.095, grip, 0.46, 35)
    yield from move("load mug", R, dest, S.FLOOR + 0.077, grip, 0.46, 19)
    yield from move("release mug", R, dest, S.FLOOR + 0.077, grip, 0.96, 13)
    yield from move("clear door", R, dest + [-0.155, 0], S.FLOOR + 0.122, grip, 0.96, 28)
    yield from series("park right", p.park(R, 30))
    if stop_after == "mug":
        for _ in range(HZ):
            yield "inspect placement", p.q.ravel().copy()
        return

    down = frame((0.77, 0, -0.64), (0, -1, 0))

    def door_point(a):
        rot = Rotation.from_euler("z", a).as_matrix()[:2, :2]
        return S.HINGE + rot @ [-0.015, -0.210]

    start = door_point(S.OPEN)
    yield from move("above door", L, start, S.FLOOR + 0.265, down, 0.02, 35)
    yield from move("contact door", L, start, S.FLOOR + 0.200, down, 0.02, 20)
    for a in np.linspace(S.OPEN, 0.10, 88):
        yield from move("swing door shut", L, door_point(a), S.FLOOR + 0.200, down, 0.02, 1)
    yield from move("clear upper rim", L, door_point(0.10) + [-0.03, 0], S.FLOOR + 0.265, down, 0.02, 18)
    yield from series("park left", p.park(L, 22))

    push = frame((1, 0, 0), (0, -1, 0))
    yield from series("approach start", p.move(R, S.BUTTON + [-0.075, 0, 0], push, 0.02, 29))
    yield from series("press start", p.move(R, S.BUTTON + [-0.008, 0, 0], push, 0.02, 20))
    yield from series("release start", p.move(R, S.BUTTON + [-0.085, 0, 0], push, 0.02, 15))
    yield from series("park right", p.park(R, 22))
    if interrupt:
        for _ in range(35):
            yield "wait during loaded turn", p.q.ravel().copy()
        handle = S.HINGE + [-0.035, -S.HANDLE_RADIUS]
        yield from move("approach handle", R, handle + [-0.06, 0], S.FLOOR + 0.112, push, 1.0, 18)
        yield from move("reach handle", R, handle, S.FLOOR + 0.112, push, 1.0, 15)
        yield from move("grasp handle", R, handle, S.FLOOR + 0.112, push, 0.02, 12)
        for a in np.linspace(0, -0.42, 30):
            xy = S.HINGE + Rotation.from_euler("z", a).as_matrix()[:2, :2] @ [-0.035, -S.HANDLE_RADIUS]
            yield from move("open door during turn", R, xy, S.FLOOR + 0.112, push, 0.02, 1)
        yield from move("release handle", R, xy, S.FLOOR + 0.112, push, 1.0, 12)
        yield from series("park after interruption", p.park(R, 24))
    while True:
        yield "verify driven turn", p.q.ravel().copy()


def evidence_frame(env, step, font):
    # Four synchronized views of physical marks only. No metric or score overlay.
    frame = Image.new("RGB", (1152, 678), "#18222c")
    draw = ImageDraw.Draw(frame)
    for name, title, x, y, width, height in (
        ("plan", "Plan through clear roof", 0, 0, 576, 576),
        ("overview", "Operator view", 576, 0, 576, 264),
        ("back", "Rear elevation", 576, 288, 576, 168),
        ("side", "Left elevation", 576, 480, 576, 168),
    ):
        draw.text((x+8, y+2), title, font=font, fill="white")
        frame.paste(Image.fromarray(env.render(name, width, height)), (x, y+24))
    for i, (ink, label) in enumerate((
        ("#ed75c4", "Roller ring"), ("#82a5ff", "Tray stripe"), ("#ffe651", "Mug dot"),
    )):
        x = 12 + i*158
        draw.rectangle((x, 619, x+12, 631), fill=ink)
        draw.text((x+20, 614), label, font=font, fill="white")
    draw.text((12, 644), f"Frame {step:04d}   {step/HZ:05.2f} s   |   20 fps", font=font, fill="white")
    return np.asarray(frame)


def run(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    env = Microwave(cameras=False)
    scene = Scene(id="microwave-turntable", instruction=INSTRUCTION)
    env.reset(scene, seed=0)
    planner = Planner(env.model)
    writer = None
    start = time.monotonic()
    font = ImageFont.truetype("DejaVuSans.ttf", 16) if args.record else None
    try:
        if args.record:
            import imageio.v2 as imageio

            writer = imageio.get_writer(
                str(out / "demo.mp4"),
                fps=HZ,
                codec="libx264",
                macro_block_size=1,
                pixelformat="yuv420p",
                output_params=["-movflags", "+faststart"],
            )
            setup = env.render("overview", 960, 540)
            Image.fromarray(setup).save(out / "setup.png")
            writer.append_data(evidence_frame(env, 0, font))
        with (out / "trace.jsonl").open("w") as trace:
            prev = None
            was_on = False
            door_cut_step = None
            for step, (stage, row) in enumerate(script(planner, args.miss_hub, args.stop_after, args.interrupt), 1):
                res = env.step(Action(data=row))
                if was_on and not env.metrics["motor_on"] and env.metrics["latch_gap_mm"] > 10:
                    door_cut_step = step
                was_on = env.metrics["motor_on"]
                # Physical poses are recorded for review after each action, outside the controller.
                poses = {
                    n: env.data.body(n).xpos.copy().round(5).tolist()
                    for n in ("roller_ring", "tray", "mug")
                }
                tcp = {
                    s: env.data.site(s + "_tcp").xpos.copy().round(5).tolist() for s in ("left", "right")
                }
                trace.write(
                    json.dumps(
                        dict(step=step, stage=stage, action=row.tolist(), **poses, tcp=tcp, **env.metrics)
                    )
                    + "\n"
                )
                if prev != stage:
                    print(step, stage, env.metrics["score"], flush=True)
                    prev = stage
                if writer:
                    writer.append_data(evidence_frame(env, step, font))
                if res.terminated or step >= args.max_steps:
                    break
        result = dict(
            policy="scripted",
            miss_hub=args.miss_hub, interrupted=args.interrupt, stop_after=args.stop_after,
            door_cut_step=door_cut_step,
            task_score=env.metrics["score"],
            steps=step,
            simulation_seconds=step / HZ,
            wall_seconds=time.monotonic() - start,
            seed=0,
            termination="success" if res.terminated else "max_steps" if step == args.max_steps else "placement_stop",
            **env.metrics,
        )
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
    finally:
        if writer:
            writer.close()
        env.close()
    if args.interrupt:
        return int(not (result["score"] == 0.75 and 0 < result["best_loaded_turn_deg"] < 360
                        and result["turn_deg"] == 0 and not result["motor_on"] and door_cut_step is not None))
    expected = 0.25 if args.miss_hub else {"reset": 0.0, "ring": 0.25, "tray": 0.5, "mug": 0.75, None: 1.0}[args.stop_after]
    return int(result["score"] != expected)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--record", action="store_true")
    controls = parser.add_mutually_exclusive_group()
    controls.add_argument("--stop-after", choices=("reset", "ring", "tray", "mug"))
    controls.add_argument("--interrupt", action="store_true", help="Pull the door open during a loaded turn")
    controls.add_argument(
        "--miss-hub", action="store_true", help="Place the tray 18 mm sideways as a scoring control"
    )
    parser.add_argument("--max-steps", type=int, default=LIMIT)
    args = parser.parse_args()
    if not 1 <= args.max_steps <= LIMIT:
        parser.error(f"--max-steps must be between 1 and {LIMIT}")
    raise SystemExit(run(args))
