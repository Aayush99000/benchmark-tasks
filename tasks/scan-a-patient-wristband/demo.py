"""Demo for the wristband-scanning task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1200] [--no-record]
                   [--mode scan|keep|drop|turn|nohold|letgo|still]

1. The left hand comes in level from the left, fingers one above the other, and takes the patient's hand by its
   fingers. It turns its wrist a quarter turn: the forearm rolls over until the thumb is up and the band's label
   faces the arms. It keeps holding.
2. The right hand comes straight down on the scanner's handle and closes on it, one finger on the trigger and one
   on the back of the handle: the trigger is pulled. It lifts the scanner to the height of the wrist and brings its
   window in line with the label, 9 cm from it, and holds still until the scanner reads (its light turns green).
3. The right hand takes the scanner back, sets it down on the table and lets go.
4. The left hand turns the forearm back palm down, lets go of the hand and backs away.

Controls: ``--mode keep`` stops after the read with everything still in hand; ``--mode drop`` lets go of the turned
forearm after the read (it rolls back by itself); ``--mode turn`` only turns the forearm, holds it and lays it back;
``--mode nohold`` aims the scanner at the band without turning the forearm; ``--mode letgo`` turns the forearm, lets
it go and then scans; ``--mode still`` keeps both arms still.

Open loop: every pose comes from the layout constants in the task file. Object states are read only to report
where things ended up. Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json.
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

    def joints_for(self, pos, rot, seed) -> np.ndarray:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(seed, float), base_pos=self.base, iters=400)
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        return q

    def set_grip(self, grip: float, duration: float = 0.5):
        yield from self.move(self.pos, duration, grip=grip)

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


# -- the plan's numbers ---------------------------------------------------------------------------------------------
TAKE_IN = 0.030                          # the left hand's grasp point, this far in from the patient's fingertips
TAKE_OPEN, TAKE_SHUT = 0.050, 0.008      # finger gap round the patient's fingers (18 mm thick): before, and commanded
TURN = math.radians(90.0)                # how far the left hand turns the forearm
TURN_S = 2.5                             # ... in this long (36 deg/s)
HANDLE_OPEN, HANDLE_SHUT = 0.075, 0.018  # finger gap round the scanner's handle and trigger (35 mm pulled): commanded
STANDOFF = 0.090                         # scan window to label
AIM_UP = 0.004                           # the loaded hand hangs a little low: aim this much high
READ_S = 1.5                             # hold the aim this long
FLAT = frame((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))       # level, pointing right, fingers one above the other
DOWN = frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))        # straight down, fingers closing along x
SEED_FLAT = (-0.34, 2.43, 1.37, 1.06, 1.23, 1.57)      # the left arm's joints near its level pose (wrist a quarter turn)
SEED_DOWN = (0.2, 1.3, 1.3, 0.0, 0.0, 0.0)


def rolled(angle: float) -> np.ndarray:
    """The left hand's level pose turned ``angle`` about the forearm's axis (+y), with the forearm."""
    import bandkit as K

    return K.turned(angle) @ FLAT


def points(T):
    """Where the plan's poses are, from the task file's layout."""
    import bandkit as K

    elbow = np.asarray(T.ELBOW, float)
    take = elbow + [0.0, K.TIP_Y - TAKE_IN, 0.0]                              # on the forearm's axis, in the fingers
    lying = K._rotation(T.LYING)
    rest = np.asarray(T.SCANNER_POS, float) + lying @ np.array(K.GRIP)       # the handle, where the hand takes it
    label = elbow + [-K.BAND_R, K.BAND_Y, 0.0]                                # the symbol, forearm a quarter turn over
    window = label + [-STANDOFF, 0.0, AIM_UP]
    aim = window - lying @ np.array(K.WINDOW) + lying @ np.array(K.GRIP)     # the handle, with the window there
    return take, rest, aim


def take_and_turn(T, left, angle=TURN):
    take, _, _ = points(T)
    back = take + [0.0, 0.080, 0.0]
    yield from left.move_joints(left.joints_for(back, FLAT, SEED_FLAT), 2.5, grip=gap(TAKE_OPEN))
    yield from left.move(take, 1.2)                                           # the patient's fingers between the pads
    yield from chain(left.set_grip(gap(TAKE_SHUT), 0.6), wait(0.2))
    yield from left.move(take, TURN_S, rot=rolled(angle))                     # roll the forearm over
    yield from wait(0.3)


def lay_back(T, left):
    from yam_arena.constants import READY

    take, _, _ = points(T)
    yield from left.move(take, TURN_S, rot=FLAT)                              # roll it back palm down
    yield from chain(wait(0.3), left.set_grip(gap(TAKE_OPEN), 0.6))
    yield from left.move(take + [0.0, 0.080, 0.0], 1.2)
    yield from left.move_joints(np.array(READY, float), 1.5)


def let_go(T, left):
    """Open the left hand wide round the turned hand, so that it rolls back by itself, and take the hand away."""
    take, _, _ = points(T)
    yield from chain(left.set_grip(1.0, 0.3), wait(1.0))
    yield from left.move(take + [0.0, 0.080, 0.0], 1.0)


def over_scanner(T, right):
    _, rest, _ = points(T)
    yield from right.move_joints(right.joints_for(rest + [0.0, 0.0, 0.090], DOWN, SEED_DOWN), 2.5, grip=gap(HANDLE_OPEN))


def scan(T, right):
    _, rest, aim = points(T)
    yield from right.move(rest, 1.0)
    yield from chain(right.set_grip(gap(HANDLE_SHUT), 0.5), wait(0.2))       # the grip pulls the trigger
    yield from right.move(np.array([rest[0], rest[1], aim[2]]), 1.0)          # up to the wrist's height
    yield from right.move(aim, 1.0)                                           # window in line with the label
    yield from wait(READ_S)


def put_down(T, right):
    from yam_arena.constants import READY

    _, rest, aim = points(T)
    yield from right.move(np.array([rest[0], rest[1], aim[2]]), 1.0)
    yield from right.move(rest + [0.0, 0.0, 0.002], 1.2)
    yield from chain(right.set_grip(gap(HANDLE_OPEN), 0.5), wait(0.2))
    yield from right.move(rest + [0.0, 0.0, 0.090], 1.0)
    yield from right.move_joints(np.array(READY, float), 1.5)


def script(T, left, right, mode: str = "scan"):
    yield from wait(0.3)
    if mode == "still":
        yield from wait(6.0)
        return
    if mode == "turn":
        yield from take_and_turn(T, left)
        yield from wait(2.0)
        yield from lay_back(T, left)
        return
    if mode == "nohold":                                                      # the scanner aimed with the label face down
        yield from over_scanner(T, right)
        yield from scan(T, right)
        yield from put_down(T, right)
        return
    yield from together(take_and_turn(T, left), over_scanner(T, right))
    if mode == "letgo":                                                       # let the turned forearm go, then scan
        yield from let_go(T, left)
        yield from scan(T, right)
        yield from put_down(T, right)
        return
    yield from scan(T, right)
    if mode == "keep":
        yield from wait(2.0)
        return
    if mode == "drop":                                                        # after the read: let the forearm go
        yield from let_go(T, left)
        yield from put_down(T, right)
        return
    yield from put_down(T, right)
    yield from lay_back(T, left)
    yield from wait(1.5)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, window: bool = False,
        hold_s: float = 2.0, mode: str = "scan", every: int = 5) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=320, cam_height=176,
                               headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="band-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode)
    frames: dict[str, list] = {}
    trace = []

    def snap(o) -> None:
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, ["training_arm", "scanner"])

    def hands() -> dict:
        import bandkit as K

        return {n: {"tool_cm": [round(100 * float(v), 2) for v in g["tcp"] - [0, 0, T.TOP]],
                    "gap_mm": round(1000 * g["gap"], 1)} for n, g in zip(("left", "right"), K.grippers(emb._env))}

    snap(obs)
    start = T.report()
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % every == 0:
            st = states()
            scan_st = st["scanner"]
            trace.append({"step": steps, "score": info.get("score"), "task": T.report(),
                          "scanner_cm": [round(100 * float(v), 2) for v in scan_st.pos - [0, 0, T.TOP]],
                          "scanner_quat": [round(float(v), 3) for v in scan_st.quat_xyzw],
                          "trigger_mm": round(1000 * scan_st.joints["trigger"], 2), "hands": hands(),
                          "want_cm": {h.name: [round(100 * float(v), 2) for v in h.pos - [0, 0, T.TOP]] for h in (left, right)}})
        if steps % 20 == 0 and trace:
            t = trace[-1]
            r = t["task"]
            print(f"step {steps} score {t['score']} turn {r['turn_deg']} rate_max {r['rate_max_deg_s']} "
                  f"trig {t['trigger_mm']} live {r['session_live']} why {r['why_not']} view {r['view']} read {r['read']} "
                  f"scanner {t['scanner_cm']} {t['scanner_quat']} hands {t['hands']} want {t['want_cm']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": T.report(), "trace": trace}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="scan", choices=("scan", "keep", "drop", "turn", "nohold", "letgo", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every)
