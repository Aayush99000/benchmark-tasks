"""Scripted full-score demo for the lanyard task, run in the yam_arena embodiment.

    python demo.py task.py OUT_DIR [--max-steps 2400] [--no-record] [--cam-width 640 --cam-height 352]

It plans from the published start layout and, while it runs, reads only the hook's pose (what a camera sees) and the
arms' own joint angles. It moves the hook only with the grippers: nothing is teleported and the scene is not edited.

1. The right arm comes down over the hook's handle with its fingers across it, one on the lock lever on the back of
   the spine and one on the gate, and squeezes: the lock frees the gate, the gate opens, and the squeeze holds the
   hook. It lifts the hook, carries it to the middle of the deck, turns it to point left, lays it down and lets go.
2. The left arm takes the hook the same way, lifts it to the anchor's height and carries it left. It brings the open
   hook up beside the tip of the anchor's D-ring, slides it sideways so the ring's bar passes the open gate, draws it
   back so the bar sits in the bowl, and lets go: the gate shuts and locks round the ring.
3. Both arms move clear.

Variants (DEMO_VARIANT=...): nothing (both arms stay still); hold (hooked on the anchor but the left gripper keeps
squeezing, so the gate stays open); rail (the right arm clips the hook round the guardrail post it lies beside).

The layout of this file (an Arm class making smooth Cartesian moves by IK, generators stepped once per control
step) follows the demo published with this repository's luggage-cart-bungee task (MIT licence).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
OPEN_M = 0.094            # pads this far apart at gripper command 1.0


def grip_cmd(width_m: float) -> float:
    return float(np.clip(width_m / OPEN_M, 0.0, 1.0))


def hand(finger, pitch: float = 0.0, lean=(0.0, 1.0, 0.0)) -> np.ndarray:
    """Hand rotation: approach straight down, tilted ``pitch`` rad toward ``lean``; fingers close along ``finger``."""
    f = np.array(finger, float)
    f /= np.linalg.norm(f)
    le = np.array(lean, float)
    le = le - le.dot(f) * f
    le /= np.linalg.norm(le)
    z = -math.cos(pitch) * np.array([0.0, 0.0, 1.0]) + math.sin(pitch) * le
    return np.column_stack([f, np.cross(z, f), z])


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.offset = np.zeros(3)      # added to every target: makes up for the arm's sag (from its own joint readings)

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(np.asarray(pos, float) + self.offset, ik.matrix_to_quat_xyzw(rot), self.q,
                            base_pos=self.base, iters=80)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        if pe * 1000 > 3.0 and pe * 1000 > self.worst_mm:
            print(f"IK {self.name}: {pe * 1000:.1f} mm short of {np.round(pos, 3).tolist()}", flush=True)
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def trim(self, measured, rounds: int = 3, gain: float = 0.8):
        """Bring the hand to where it was told to be: compare the hand's position from the arm's measured joint
        angles with the target and shift the command by the difference, a few times."""
        from yam_arena import ik

        for _ in range(rounds):
            yield from wait(0.25)
            actual, _ = ik.tcp_pose(measured(self.name), self.base)
            err = self.pos - actual
            if np.linalg.norm(err) < 0.0015:
                break
            self.offset = np.clip(self.offset + gain * err, -0.04, 0.04)
            self._solve(self.pos, self.rot)

    def move(self, pos, rot=None, duration: float = 1.0, grip: float | None = None):
        from yam_arena import ik

        rot = self.rot if rot is None else rot
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def shift(self, delta, duration: float = 1.0):
        yield from self.move(self.pos + np.asarray(delta, float), self.rot, duration)

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)

    def joints_to(self, q, duration: float = 2.0, grip: float | None = None):
        """Joint-space move (used to park at the ready pose)."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (np.asarray(q, float) - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Where the fingers go on the hook (hook frame): across the handle, 45 mm ahead of the gate's pivot, centred between
# the lock lever's back and the gate's outer face.
GRASP = np.array([-0.105, 0.021, 0.0])
SQUEEZE_M = 0.030         # commanded pad gap while holding (the levers stop the pads at about 60 mm)
PAD_BELOW_TCP = 0.013


def script(T, ctx):
    variant = os.environ.get("DEMO_VARIANT", "")
    L, R = ctx.left, ctx.right
    from yam_arena.constants import READY

    if variant == "nothing":
        return
    zd = T.ZD
    deck_tcp = zd + PAD_BELOW_TCP + 0.004      # hand height that puts the pads round a hook lying on the deck

    def hook():
        p, r, gate, lock = ctx.hook()
        return p, r

    def grasp_point(p, r):
        return p + r @ GRASP

    # -- 1. right arm: squeeze the hook's handle, lift, carry to the middle, lay it down pointing left ------------
    p, r = hook()
    g = grasp_point(p, r)
    f = -r[:, 1]                                # right hand: fingers along the hook's -y (its lock side to the left pad)
    # to reach the guardrail post the hand has to lean 40 degrees forward along the hook; for the anchor it is upright
    rot = hand(f, 0.7, r[:, 0]) if variant == "rail" else hand(f)
    yield from R.move([g[0], g[1], zd + 0.12], rot, 2.0, grip=1.0)
    yield from R.move([g[0], g[1], zd + 0.05], rot, 1.0)
    yield from R.trim(ctx.joints)
    yield from R.move([g[0], g[1], deck_tcp], rot, 1.2)
    yield from R.trim(ctx.joints)
    ctx.pads("right before squeeze")
    yield from R.set_grip(grip_cmd(SQUEEZE_M), 1.0)
    yield from wait(0.3)
    ctx.pads("right after squeeze")
    ctx.note("right squeezed")
    yield from R.shift([0, 0, 0.10], 1.5)
    ctx.note("right lifted")

    if variant == "rail":
        yield from clip_on_post(T, ctx, R)
        return

    # the hook's frame as held: turn it to yaw 90 deg (nose to the left), bowl centre at HAND_OVER
    hand_over = np.array([0.321, 0.165])
    rot2 = hand([1.0, 0.0, 0.0])                # fingers along +x: the hook's y axis points to -x, its nose to +y
    tcp2 = np.array([hand_over[0], hand_over[1], 0.0]) + _rz(math.pi / 2) @ GRASP
    yield from R.move([tcp2[0], tcp2[1], zd + 0.08], rot2, 3.0)
    # trim: put the hook itself where it should be (it may have settled a little in the grip)
    p, r = hook()
    err = np.array([hand_over[0] - p[0], hand_over[1] - p[1], 0.0])
    yield from R.shift(err, 0.8)
    yield from R.move([R.pos[0], R.pos[1], deck_tcp + 0.002], rot2, 1.5)
    ctx.note("right laid down")
    yield from R.set_grip(1.0, 0.8)
    yield from R.shift([0, 0, 0.08], 1.2)
    yield from R.joints_to(READY, 2.0, grip=1.0)

    # -- 2. left arm: take the hook, lift to the anchor's height, hook the ring ----------------------------------
    yield from wait(0.3)
    p, r = hook()
    g = grasp_point(p, r)
    f = r[:, 1]                                 # left hand: fingers along the hook's +y
    rot = hand(f)
    yield from L.move([g[0], g[1], zd + 0.12], rot, 2.0, grip=1.0)
    yield from L.move([g[0], g[1], zd + 0.05], rot, 1.0)
    yield from L.trim(ctx.joints)
    yield from L.move([g[0], g[1], deck_tcp], rot, 1.2)
    yield from L.trim(ctx.joints)
    ctx.pads("left before squeeze")
    yield from L.set_grip(grip_cmd(SQUEEZE_M), 1.0)
    yield from wait(0.3)
    ctx.pads("left after squeeze")
    ctx.note("left squeezed")
    za = float(T.ANCHOR_Z)
    yield from L.move([L.pos[0], L.pos[1], za], rot, 2.0)
    ctx.note("left lifted")

    seat = np.asarray(T.ANCHOR_SEAT, float)     # the ring's tip bar

    def go(target_xy, z, duration):
        """Move so the hook's own origin (bowl centre) comes to ``target_xy`` at height ``z``, hook held level."""
        rot_l = hand([-1.0, 0.0, 0.0])          # fingers along -x: hook y axis = -x, nose toward +y
        tcp = np.array([target_xy[0], target_xy[1], 0.0]) + _rz(math.pi / 2) @ GRASP
        yield from L.move([tcp[0], tcp[1], z], rot_l, duration)
        for _ in range(2):                      # trim on the hook's measured pose
            p, _ = hook()
            e = np.array([target_xy[0] - p[0], target_xy[1] - p[1], z - p[2]])
            if np.linalg.norm(e) < 0.002:
                break
            yield from L.shift(np.clip(e, -0.03, 0.03), 0.5)

    # ring bar in the hook frame: first outside the nose, level with the gap behind its tip; then in past the open
    # gate; then forward into the bowl. With the hook's x axis along +y and its y axis along -x:
    #   bar at hook (x', y')  <=>  hook origin = (seat_x + y', seat_y - x')
    pre, inside, home = (-0.050, -0.078), (-0.050, -0.006), (0.018, -0.004)
    for name, (xh, yh), dur in (("pre", pre, 3.0), ("in", inside, 1.5), ("home", home, 1.5)):
        yield from go((seat[0] + yh, seat[1] - xh), za, dur)
        ctx.note(f"left {name}")
    if variant == "hold":
        yield from wait(4.0)
        return
    yield from L.set_grip(1.0, 0.8)
    ctx.note("left let go")
    yield from L.shift([-0.07, -0.02, 0.0], 1.2)
    yield from L.joints_to(READY, 2.5, grip=1.0)


def clip_on_post(T, ctx, R):
    """The cheap way: with the right arm, close the hook round the guardrail post it was lying beside."""
    from yam_arena.constants import READY

    post = np.array([T.POST_X, T.POST_Y[0]])
    z = T.ZD + 0.12
    yaw = float(T.HOOK_YAW)
    rh = _rz(yaw)

    def go(bar_in_hook, duration):
        xh, yh = bar_in_hook
        origin = post - (rh @ np.array([xh, yh, 0.0]))[:2]
        tcp = np.array([origin[0], origin[1], 0.0]) + rh @ GRASP
        yield from R.move([tcp[0], tcp[1], z], R.rot, duration)
        for _ in range(2):
            p, _, _, _ = ctx.hook()
            e = np.array([origin[0] - p[0], origin[1] - p[1], z - p[2]])
            if np.linalg.norm(e) < 0.002:
                break
            yield from R.shift(np.clip(e, -0.03, 0.03), 0.5)

    for name, bar, dur in (("pre", (-0.066, -0.092), 3.0), ("in", (-0.066, -0.004), 1.5), ("home", (0.008, 0.0), 1.5)):
        yield from go(bar, dur)
        ctx.note(f"rail {name}")
    yield from R.set_grip(1.0, 0.8)
    ctx.note("rail let go")
    yield from R.shift(-0.07 * np.array([rh[0, 0], rh[1, 0], 0.0]) + np.array([-0.03, 0, 0.0]), 1.2)
    yield from R.joints_to(READY, 2.5, grip=1.0)


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.steps = 0

    def hook(self):
        return self.emb.task.module.hook_state(self.emb._env)

    def joints(self, arm: str) -> np.ndarray:
        """The arm's six measured joint angles (what the policy observes)."""
        return np.asarray(self.emb._env.unwrapped.scene[f"{arm}_robot"].data.joint_pos[0].cpu(), float)[:6]

    def pads(self, what: str) -> None:
        """Measurement only: where each gripper's pads are in the hook's frame (mm)."""
        import rigkit

        p, r, gate, lock = self.hook()
        out = {}
        for arm, (pts, gap) in rigkit.gripper_points(self.emb._env).items():
            out[arm] = [np.round(1000 * (r.T @ (q - p)), 0).astype(int).tolist() for q in (pts[1], pts[2], pts[3])] + [round(1000 * gap)]
        print(f"PADS {what} step {self.steps}: (tcp, pad+, pad-, gap mm) {out}", flush=True)

    def note(self, what: str) -> None:
        """Measurement only (not used by the demo): what the scorer sees now."""
        T = self.emb.task.module
        p, r, gate, lock = self.hook()
        ev = T._S.get("last", {})
        print(f"NOTE {what} step {self.steps}: hook {np.round(p, 3).tolist()} nose-up "
              f"{math.degrees(math.asin(r[2, 0])):.0f} roll {math.degrees(math.asin(r[2, 1])):.0f} deg gate {gate:.1f} lock {lock:.1f} "
              f"on_anchor {ev.get('on_anchor')} on_rail {ev.get('on_rail')} score {T._S['score']}", flush=True)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 640,
        cam_height: int = 352) -> dict:
    from types import SimpleNamespace

    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scoring import task_score

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    ctx.note("start")
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, res, done = 0, None, False
    trace = []                                         # measurement only: the hook's pose and the webbing per step
    while steps < max_steps:
        if not done:
            try:
                next(gen)
            except StopIteration:
                done = True
        res = emb.step(ctx.action())
        steps = ctx.steps = steps + 1
        snap(res.observation)
        p, r, _, _ = ctx.hook()
        trace.append(np.concatenate([p, r.ravel(), task._S["hang"], task._S["strap"].p.ravel()]))
        if steps % 100 == 0:
            ctx.note("tick")
        if res.terminated:
            break
        if done and os.environ.get("DEMO_VARIANT") and steps > 0 and not hasattr(ctx, "tail"):
            ctx.tail = steps + 60                      # variants: watch 3 s more, then stop
        if hasattr(ctx, "tail") and steps >= ctx.tail:
            break
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    ctx.note("end")
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps,
              "variant": os.environ.get("DEMO_VARIANT", ""), "last": {k: (v if not isinstance(v, (np.bool_, np.floating))
                                                                        else v.item()) for k, v in task._S["last"].items()},
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import cv2
        import imageio

        def sheet(i):
            """One video frame: the top camera, with the two wrist cameras at half size below it."""
            top = frames["top_cam"][i]
            h, w = top.shape[:2]
            small = [cv2.resize(frames[c][i], (w // 2, h // 2), interpolation=cv2.INTER_AREA) for c in ("left_cam", "right_cam")]
            return np.vstack([top, np.hstack(small)])

        _write_mp4(frames["top_cam"], out / "demo_top.mp4")
        _write_mp4([sheet(i) for i in range(len(frames["top_cam"]))], out / "demo.mp4")
        imageio.imwrite(out / "setup.png", frames["top_cam"][0])
    np.save(out / "trace.npy", np.asarray(trace, np.float32))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, default=str), flush=True)
    emb.close()
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=352)
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=a.cam_width, cam_height=a.cam_height)
