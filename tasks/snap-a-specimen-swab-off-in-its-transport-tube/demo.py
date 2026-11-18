"""Demo for the swab task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1200] [--no-record] [--mode full|nohold|lifted|still]

1. The left hand comes in from the left, 15 degrees below level, its fingers open either side of the tube 2 cm below
   its rim. The right hand, pointing straight down, comes in from the near side with its fingers either side of the
   handle 3 cm above the rim.
2. The right hand closes on the handle first; then the left hand closes on the tube and holds it for the rest of the
   job. (A swab only leans on the rim: the jolt of a hand closing on the tube would flop a free swab over.)
3. The right hand pushes the handle 22 mm away from the arms. The shaft turns in the fingers and levers between the
   rim and the tube's wall until it snaps at the breakpoint, part-way through the push.
4. It lifts the handle clear, carries it over the waste cup and lets it go. The left hand lets go of the tube and
   draws back.

Controls: ``--mode nohold`` does steps 2 to 4 with the left hand kept away (the unheld tube is pushed over);
``--mode lifted`` lifts the swab 3 cm before bending it (the breakpoint is then well above the rim); ``--mode still``
keeps both arms still.

Every target comes from the layout constants in the task file and the sizes in ``swabkit.py``. The only feedback is
each hand's own measured position (the robot's proprioception), used to correct an arm's sag before a grasp. The demo
never reads the tube's or the swab's state.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.worst_deg = 0.0

    def _solve(self, pos, rot) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, duration: float, rot=None, grip: float | None = None):
        """Generator: go in a straight line to ``pos`` (world, tool point) and ``rot``; ``grip`` 1 open .. 0 closed."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        r1 = r0 if rot is None else rot
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, r1, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.5):
        """Generator: open or close to ``grip`` in place."""
        from yam_arena import ik

        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.grip = g0 + ik.smoothstep(k / n) * (grip - g0)
            self.cmd = np.r_[self.q, self.grip]
            yield

    def move_joints(self, q_goal, duration: float, grip: float | None = None):
        """Generator: swing to joint angles ``q_goal``, every joint moving together."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (np.asarray(q_goal, float) - q0), g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def chain(*gens):
    for g in gens:
        yield from g


def together(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of a few seeds (the hand's own, and its wrist turned either way)."""
    from yam_arena import ik

    best = None
    for q6 in (None, -1.0, 1.0):
        seed = hand.q.copy()
        if q6 is not None:
            seed[5] = q6
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=400)
        cost = pe + 0.05 * re
        if best is None or cost < best[1]:
            best = (q, cost)
    return best[0]


TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)
AIM_MAX = 0.015                          # the most a grasp is aimed off to make up for an arm's sag

# -- the left hand: holding the tube ----------------------------------------------------------------------------------
TILT = math.radians(15.0)                # it comes in from the left, 15 degrees below level
AP = np.array([0.0, -math.cos(TILT), -math.sin(TILT)])
HOLD = frame(AP, (-1.0, 0.0, 0.0))       # fingers closing along x, the line the handle is bent along; wrist camera uppermost
HOLD_Z = 0.080                           # on the tube, 2 cm below its rim
BACK = 0.10                              # it starts and ends this far back from the tube along its approach
TUBE_OPEN, TUBE_SHUT = 0.045, 0.0125     # finger gap round the 16 mm tube / commanded on it

# -- the right hand: the handle ---------------------------------------------------------------------------------------
DOWN = frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))    # straight down, fingers closing along y: the shaft can turn in them
                                                    # about that line as it is bent along x; wrist camera on the far side
PINCH_Z = 0.130                          # on the handle, 3 cm above the rim
SHAFT_OPEN, SHAFT_SHUT = 0.035, 0.0014   # finger gap round the 2.5 mm shaft / commanded on it
NEAR = 0.060                             # the open hand starts this far to the near side of the handle
SWING_TO = np.array([-0.020, -0.120, 0.040])   # where the first swing ends, from that near point
BEND = 0.022                             # the handle is pushed this far away from the arms. Both arms give under the
                                         # load (about 0.3 N a millimetre each), so the shaft snaps only part-way there
LIFT_Z = 0.155                           # the freed handle's lower end is then 1.7 cm above the rim
DROP_Z = 0.150


def aim(hand, target):
    """Correct ``target`` once for the hand's measured sag, by at most AIM_MAX."""
    target = np.asarray(target, float)
    off = target - TOOL[hand.name]
    n = float(np.linalg.norm(off))
    if n > AIM_MAX:
        off = off * AIM_MAX / n
    return target + off


def settle(hand, target, rounds: int = 2):
    for _ in range(rounds):
        yield from chain(hand.move(aim(hand, target), 0.5), wait(0.3))


def pinch_point(T, z: float = PINCH_Z) -> np.ndarray:
    """Where the resting swab's shaft is at height ``z`` above the table."""
    tip = T.tip_start()
    return tip + (T.TOP + z - tip[2]) / T.UP_SWAB[2] * T.UP_SWAB


def reach_tube(T, left):
    """Bring the open left hand in from the left until its fingers are either side of the tube."""
    at = np.array([T.TUBE_XY[0], T.TUBE_XY[1], T.TOP + HOLD_Z])
    back = at - AP * BACK
    yield from left.move_joints(best_ik(left, back, HOLD), 2.5, grip=gap(TUBE_OPEN))
    yield from settle(left, back, 1)
    yield from left.move(at, 2.0)
    yield from settle(left, at)


def close_on_tube(left):
    yield from chain(left.set_grip(gap(TUBE_SHUT), 0.8), wait(0.4))


def let_go(T, left):
    yield from chain(left.set_grip(gap(TUBE_OPEN), 0.8), wait(0.3))
    yield from left.move(left.pos - AP * BACK, 2.0)


def reach_handle(T, right):
    """Bring the open right hand in from the near side until its fingers are either side of the handle."""
    at = pinch_point(T)
    near = at - [NEAR, 0.0, 0.0]
    # The swing from the ready pose ends well to the right of the tube and above it: swung straight to the near point,
    # the hand's body passes through the top of the handle and knocks the swab round the rim.
    yield from right.move_joints(best_ik(right, near + SWING_TO, DOWN), 2.5, grip=gap(SHAFT_OPEN))
    yield from right.move(near, 1.5)
    yield from settle(right, near, 1)
    yield from right.move(at, 1.5)
    yield from settle(right, at, 1)


def pinch(right):
    """Close on the handle."""
    yield from chain(right.set_grip(gap(SHAFT_SHUT), 0.6), wait(0.3))


def snap(T, right, lift_first: float = 0.0, push: float = BEND):
    """Push the pinched handle away from the arms until the shaft snaps; then lift the handle clear."""
    if lift_first:
        yield from chain(right.move(right.pos + [0.0, 0.0, lift_first], 1.0), wait(0.2))
    yield from chain(right.move(right.pos + [push, 0.0, 0.0], 2.0), wait(0.5))
    yield from right.move([right.pos[0], right.pos[1], T.TOP + LIFT_Z], 1.0)


def bin_handle(T, right):
    """Carry the handle over the waste cup and let it go."""
    over = np.array([T.CUP_XY[0], T.CUP_XY[1], T.TOP + LIFT_Z])
    yield from right.move(over, 2.5)
    yield from right.move(over - [0.0, 0.0, LIFT_Z - DROP_Z], 0.5)
    yield from chain(right.set_grip(1.0, 0.5), wait(0.5))
    yield from right.move(over + [-0.08, 0.04, 0.0], 2.0)


def script(T, left, right, mode: str = "full", idle_s: float = 60.0):
    yield from wait(0.3)
    if mode == "still":
        yield from wait(idle_s)
        return
    if mode == "nohold":
        yield from reach_handle(T, right)
        yield from pinch(right)
        yield from snap(T, right, push=0.030)
        yield from bin_handle(T, right)
    else:
        # The handle is taken before the tube: the swab only leans on the rim, and the jolt of a hand closing on the
        # tube (a millimetre) is enough to flop a free swab over to the other side of it.
        yield from together(reach_tube(T, left), reach_handle(T, right))
        yield from pinch(right)
        yield from close_on_tube(left)
        yield from snap(T, right, lift_first=0.030 if mode == "lifted" else 0.0)
        yield from bin_handle(T, right)
        yield from let_go(T, left)
    yield from wait(2.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "full", every: int = 5, cam: tuple = (320, 176)) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam[0],
                               cam_height=cam[1], headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="swab-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []
    names = [T.TUBE, T.HEAD, T.HANDLE, T.CUP]

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, names)

    def measure() -> None:
        from swabkit import finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))

    def hands() -> dict:
        from swabkit import _np

        out_ = {}
        for h in (left, right):
            robot = emb._env.unwrapped.scene[f"{h.name}_robot"]
            q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out_[h.name] = {"tool_mm": [round(1000 * float(v), 1) for v in TOOL[h.name] - [0.0, 0.0, T.TOP]],
                            "want_mm": [round(1000 * float(v), 1) for v in h.pos - [0.0, 0.0, T.TOP]],
                            "gap_mm": round(1000 * sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")), 1)}
        return out_

    def places(st) -> dict:
        mm = lambda v: [round(1000 * float(x), 1) for x in np.asarray(v, float) - [0.0, 0.0, T.TOP]]   # noqa: E731
        return {n: mm(st[n].pos) for n in names}

    snap_frames(obs)
    measure()
    start = {"task": T.report(states()), "places": places(states())}
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        measure()
        info = res.info
        snap_frames(res.observation)
        if steps % every == 0:
            st = states()
            trace.append({"step": steps, "score": info.get("score"), "task": T.report(st), "hands": hands(),
                          "places": places(st)})
        if steps % 40 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {t['score']} {t['task']} {t['hands']} {t['places']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": {"task": T.report(states()), "places": places(states())}, "trace": trace}
    if record:
        import imageio

        for name, fs in frames.items():
            _write_mp4(fs, out / f"demo_{name}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=lambda o: o.item()))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "nohold", "lifted", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")))
