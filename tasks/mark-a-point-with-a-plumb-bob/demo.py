"""Demo for the plumb-bob task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1200] [--no-record] [--mode mark|steady|nosteady|light|still]

1. The left hand comes in from the left, open, level with the bob's body, its fingers either side of the swinging
   bob, and closes slowly until both fingers just touch it: the swing is stopped between them.
2. It opens again very slowly, so the bob is let down onto its own plumb line without a push, and draws back.
   Meanwhile the right hand has taken the pencil from its rest by its upper end.
3. The right hand brings the pencil in along its own slant, under the bob, until its point touches the worktop
   below the bob's point, presses it on (the hand goes on 3 mm along the pencil: about 0.6 N on the point), and draws
   it back the same way.

``--mode steady`` stops after step 2 (the bob steadied, no mark). ``--mode nosteady`` is a control: the right hand
makes the same mark with the bob left swinging. ``--mode light`` is a control: everything as in the demo, but the
point only touches the worktop, too lightly to mark. ``--mode still`` keeps both arms still (a scene check; it shows
how slowly the swing dies away).

Every target comes from the layout constants in the task file and the sizes in ``plumbkit.py``. The only feedback is
each hand's own measured position (the robot's proprioception), used to correct an arm's sag before a placement. The
demo never reads the bob's or the pencil's state.
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

    def set_grip(self, grip: float, duration: float = 0.5, even: bool = False):
        """Generator: open or close to ``grip`` in place; ``even`` moves the fingers at one steady speed."""
        from yam_arena import ik

        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = k / n if even else ik.smoothstep(k / n)
            self.grip = g0 + s * (grip - g0)
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
AIM_MAX = 0.015                          # the most a placement is aimed off to make up for an arm's sag

# -- the left hand: steadying the bob -----------------------------------------------------------------------------------
TILT = math.radians(15.0)                # the left hand comes in from the left, 15 degrees below level
AP = np.array([0.0, -math.cos(TILT), -math.sin(TILT)])
CAGE = frame(AP, (-1.0, 0.0, 0.0))       # fingers closing along x, the line of the swing; the wrist camera uppermost
BACK = 0.12                              # it starts and ends this far back from the bob along its approach
TOUCH_GAP = 0.030                        # commanded on the 32 mm body: both fingers just pressing on it
EASE_GAP, EASE_S = 0.044, 5.0            # then opened to here this slowly (each finger 1.4 mm/s): no push on the bob

# -- the right hand: the pencil -----------------------------------------------------------------------------------------
DOWN_X = frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))     # straight down, fingers closing along x on the pencil's edges
PINCH = 0.140                            # the pencil is taken this far up from its point
PENCIL_OPEN, PENCIL_SHUT = 0.045, 0.011  # finger gap round the 15 mm pencil / commanded on it
SHY = 0.060                              # the pencil's point waits this far back up its own slant from the mark
PRESS_ON = 0.0030                        # after it touches, the hand goes on this far along the pencil: the arm gives
                                         # and the lead's spring takes up the rest, about 0.5 N on the point
POINT_IN_HAND = np.array([-0.0006, 0.0011, -0.0006])   # the pencil's point sits this far off its nominal place in the hand (measured)
LIGHT_ON = 0.0008                        # the 'light' control goes on only this far after touching: under 0.3 N on the point


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


def steady(T, left):
    """Stop the swing between the fingers, then let the bob down onto its own line."""
    import plumbkit as K

    at = T.plumb_point() + [0.0, 0.0, T.POINT_GAP + K.BOB_COM]      # the bob's centre of mass, hanging plumb
    back = at - AP * BACK
    yield from left.move_joints(best_ik(left, back, CAGE), 2.5, grip=1.0)
    yield from settle(left, back, 1)
    yield from left.move(at, 2.5)                                    # fingers pass either side of the swinging bob
    yield from settle(left, at)
    yield from left.set_grip(gap(TOUCH_GAP), 3.0, even=True)         # close slowly: the swing is stopped between them
    yield from wait(1.5)
    yield from left.set_grip(gap(EASE_GAP), EASE_S, even=True)       # let it down onto its own line
    yield from left.set_grip(1.0, 1.5)
    yield from left.move(left.pos - AP * BACK, 2.5)


def fetch(T, right):
    """Take the pencil out of its rest by its upper end."""
    at = T.pencil_start() + PINCH * T.UP_PENCIL
    yield from right.move_joints(best_ik(right, at + [0.0, 0.0, 0.05], DOWN_X), 2.5, grip=gap(PENCIL_OPEN))
    yield from chain(right.move(at, 1.2), wait(0.2))
    yield from settle(right, at, 1)
    yield from chain(right.set_grip(gap(PENCIL_SHUT), 0.6), wait(0.3))
    yield from right.move(right.pos + [0.0, 0.0, 0.04], 1.2)


def mark(T, right, press_on: float = PRESS_ON):
    """Bring the pencil's point in along its own slant, under the bob, onto the worktop, and press it on by carrying
    the hand ``press_on`` further along the pencil; then back the same way."""
    up = T.UP_PENCIL

    def hand_at(shy: float) -> np.ndarray:                           # the tool point, with the pencil's point this far
        return T.plumb_point() + (shy + PINCH) * up - POINT_IN_HAND  # back up its slant from the mark

    yield from right.move(hand_at(SHY), 3.0)
    yield from settle(right, hand_at(SHY))
    yield from right.move(hand_at(0.012), 2.0)
    yield from settle(right, hand_at(0.012))
    yield from right.move(hand_at(0.004), 1.0)
    yield from settle(right, hand_at(0.004), 1)
    yield from right.move(hand_at(-press_on), 1.5)                   # down the slant: the point touches and is pressed on
    yield from wait(0.6)
    yield from right.move(hand_at(SHY), 2.0)
    yield from right.move(hand_at(SHY) + [-0.04, -0.06, 0.03], 2.0)


def script(T, left, right, mode: str = "mark", idle_s: float = 60.0):
    yield from wait(0.3)
    if mode == "still":
        yield from wait(idle_s)
        return
    if mode == "nosteady":
        yield from fetch(T, right)
        yield from mark(T, right)
    else:
        yield from together(steady(T, left), fetch(T, right))
        if mode == "mark":
            yield from mark(T, right)
        elif mode == "light":                                            # control: touch the worktop, too lightly to mark
            yield from mark(T, right, press_on=LIGHT_ON)
    yield from wait(2.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "mark", every: int = 5, cam: tuple = (320, 176)) -> dict:
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
    obs = emb.reset(Scene(id="plumb-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []

    def snap(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, [T.BOB, T.PENCIL])

    def measure() -> None:
        from plumbkit import finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))

    def hands() -> dict:
        from plumbkit import _np

        out_ = {}
        for h in (left, right):
            robot = emb._env.unwrapped.scene[f"{h.name}_robot"]
            q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out_[h.name] = {"tool_mm": [round(1000 * float(v), 1) for v in TOOL[h.name] - T.plumb_point()],
                            "want_mm": [round(1000 * float(v), 1) for v in h.pos - T.plumb_point()],
                            "gap_mm": round(1000 * sum(abs(float(q[names.index(j)])) for j in ("joint7", "joint8")), 1)}
        return out_

    snap(obs)
    measure()
    start = T.report(states())
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
        snap(res.observation)
        if steps % every == 0:
            st = states()
            entry = {"step": steps, "score": info.get("score"), "task": T.report(st), "hands": hands(),
                     "point_from_right_tool_mm": [round(1000 * float(v), 2) for v in np.asarray(st[T.PENCIL].pos) - TOOL["right"]]}
            trace.append(entry)
        if steps % 40 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {t['score']} {t['task']} {t['hands']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": T.report(states()), "trace": trace}
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
    ap.add_argument("--mode", default="mark", choices=("mark", "steady", "nosteady", "light", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")))
