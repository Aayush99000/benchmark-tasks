"""Demo for the game-disc task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2400] [--no-record]
                                   [--mode load|open|nohold|nopress|partway|still]

1. The left hand, closed and leaning out to its own side, presses its fingertips on the tray's finger ledge, through
   the recess in the cover's left edge: it holds the case down.
2. The right hand, level and closed, slides its fingertips under the cover's lip at the tray's thumb cut-out and
   lifts: the catch lets go. The left hand lets go of the ledge; the right hand raises the cover halfway.
3. The left hand comes in under the raised cover, takes it from the right hand and pushes it up and over the spine:
   the cover falls open and lies flat beyond the case.
4. The left hand comes straight down on the case's hub, fingers closed, and presses its centre button: the hub's
   teeth draw in.
5. The right hand, level with its fingers one above the other, reaches through the thumb cut-out and pinches the
   disc's edge, then lifts the disc straight up, off the hub's teeth. The left hand lets go of the button.
6. The right hand lifts the disc clear of the hub and carries it, still level, in front of the console, and turns its
   wrist a quarter turn: the disc stands upright, square to the slot.
7. It slides the disc straight into the slot until the drive takes it, lets go and backs away.

Controls: ``--mode nohold`` lifts the cover's lip without holding the tray down (the whole case comes up, still
shut); ``--mode open`` stops once the case is open; ``--mode nopress`` opens the case and then picks at the disc
without pressing the hub; ``--mode partway`` does everything but stops with the disc 3.5 cm into the slot and holds
it there (short of where the drive takes it); ``--mode still`` keeps both arms still.

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
TIP = 0.013                        # the pads' tips reach this far past the tool point



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

    def follow(self, path, duration: float, rot=None):
        """Generator: follow ``path(s)`` (world tool point for s from 0 to 1) at a smooth pace."""
        from yam_arena import ik

        r = self.rot if rot is None else rot
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(path(ik.smoothstep(k / n)), r)
            yield

    def joints_for(self, pos, rot, seed=None) -> np.ndarray:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(self.q if seed is None else seed, float),
                             base_pos=self.base, iters=400)
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



def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of a few seeds (the hand's own, and its wrist turned either way)."""
    from yam_arena import ik

    best = None
    for q6 in (None, -1.57, 1.57):
        seed = hand.q.copy()
        if q6 is not None:
            seed[5] = q6
        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=400)
        if best is None or pe < best[1]:
            best = (q, pe)
    return best[0]



FINGERS = 0.008                          # the two closed fingertips together, across the closing direction
HOLD_LEAN = math.radians(40)             # the holding hand leans this far out to its own side
HOLD_DEPTH = 0.015                       # its fingertips are sent this far below the ledge's top
HOOK_IN = 0.005                          # fingertips this far past the tray's near edge, under the cover's lip
HOOK_Y = (0.014, -0.014)                 # ... this far left of the hub's line to open the catch (beside the hand that
                                         # holds the tray), then this far (right of it) to leave room for the left hand
POP_DEG, WAIT_DEG, HAND_OVER_DEG = 8.0, 15.0, 60.0   # the right hand turns the cover this far: to open the catch,
                                         # while the left hand comes round, and to hand the cover over
TAKE_S, TAKE_Y = 0.073, 0.070            # where the left hand takes the cover: from its hinge, and left of the hub's line
OVER = (-0.020, 0.070, 0.070)            # pushing the cover over: from this far before the hinge, this high, to this far past
PRESS_DEPTH = 0.009                      # fingertips sent this far below the button's top (the arm gives ~0.3 N/mm)
PINCH_IN = 0.018                         # the pads overlap the disc's edge by this much
PINCH_OPEN, PINCH_SHUT = 0.030, 0.0      # finger gap around the 1.2 mm disc / commanded on it
LIFT = (0.015, 0.025)                    # straight up: off the hub's teeth, as far as the fingers on its button let
                                         # the disc go; then, once they have let go, clear of the button for the carry
CARRY_Z = 0.100                          # tool height while carrying, above the table (the disc's centre in the slot)
SLOT_IN = 0.077                          # leading edge this far into the slot before letting go (the drive takes at 7)
PART_IN = 0.035                          # the 'partway' control stops with the leading edge this far in
DOWN = frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))        # straight down, fingers closing along x
LEVEL = frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))        # pointing forward, fingers one above the other
UPRIGHT = frame((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))     # pointing forward, fingers side by side: the disc upright
HOLD = frame((0.0, -math.sin(HOLD_LEAN), -math.cos(HOLD_LEAN)), (1.0, 0.0, 0.0))   # down, leaning out to the left


def open_case(T, left, right, hold: bool = True, swing: bool = True):
    """Generator: hold the tray down, lift the cover's lip until the catch lets go, and swing the cover open."""
    import disckit as K
    from yam_arena.constants import READY

    case = next(o for o in T.OBJECTS if o.name == "game_case")
    hub = np.array([T.HUB_XY[0], T.HUB_XY[1], T.TOP + T.STAND_H])
    x0, y1 = hub[0] - T.TRAY[0] / 2, hub[1] + T.TRAY[1] / 2
    hinge = np.array([hub[0] + T.TRAY[0] / 2 + K.RIM, hub[2] + T.CASE_T])     # the cover's hinge line (x, z)

    def under(s, deg, y, drop=0.0):
        """Tool point of a level, closed hand whose fingertips' top front edge is on the cover's underside ``s`` from
        the hinge, with the cover open ``deg`` degrees; ``drop`` lower than that."""
        th = math.radians(deg)
        c = hinge + s * np.array([-math.cos(th), math.sin(th)]) - case.plate * np.array([math.sin(th), math.cos(th)])
        return np.array([c[0] - TIP, y, c[1] - FINGERS / 2 - drop])

    ap = HOLD[:, 2]
    tip = np.array([hub[0] + T.RECESS_X, y1 - 0.018, hub[2] + case.wall + 0.005 - HOLD_DEPTH])
    press = tip - TIP * ap                                                    # fingertips on the tray's ledge
    y_r, y_w, s_r = hub[1] + HOOK_Y[0], hub[1] + HOOK_Y[1], hinge[0] - (x0 + HOOK_IN)
    hook = under(s_r, 0.0, y_r, drop=0.002)
    reach = right.move_joints(best_ik(right, hook + [-0.06, 0.0, 0.0], LEVEL), 2.5, grip=0.0)
    if hold:
        yield from together(reach, chain(left.move_joints(best_ik(left, press - 0.045 * ap, HOLD), 2.5, grip=0.0),
                                         left.move(press, 1.0)))              # hold the case down
    else:
        yield from reach
    yield from right.move(hook, 1.2)                                          # fingertips under the cover's lip
    yield from right.follow(lambda u: under(s_r, u * POP_DEG, y_r), 1.2)      # lift: the catch lets go
    yield from wait(0.3)
    if not swing:                                                             # control: put it back down
        yield from right.follow(lambda u: under(s_r, POP_DEG + u * 12.0, y_r), 1.5)
        yield from wait(1.0)
        yield from right.follow(lambda u: under(s_r, (1 - u) * (POP_DEG + 12.0), y_r, drop=0.002 * u), 2.0)
        yield from right.move(hook + [-0.06, 0.0, 0.0], 1.2)
        return
    y_l = hub[1] + TAKE_Y
    take = under(TAKE_S, HAND_OVER_DEG, y_l, drop=0.008)
    # the left hand lets go of the ledge and waits at its raised ready pose, clear of the cover's swing, while the
    # right hand raises the cover and moves along its lip to the right; then it comes in level, below the lip
    release = chain(left.move(press - 0.045 * ap, 0.8), left.move_joints(np.array(READY, float), 1.5)) if hold else wait(0.1)
    yield from together(release, chain(right.follow(lambda u: under(s_r, POP_DEG + u * (WAIT_DEG - POP_DEG), y_r), 1.5),
                                       right.move(under(s_r, WAIT_DEG, y_w), 1.2)))
    yield from right.follow(lambda u: under(s_r, WAIT_DEG + u * (HAND_OVER_DEG - WAIT_DEG), y_w), 2.5)
    yield from left.move_joints(best_ik(left, take + [-0.075, 0.0, 0.017], LEVEL), 2.0)
    yield from left.move(take, 1.2)                                           # in under the raised cover
    over = np.array([hinge[0] + OVER[0] - TIP, y_l, hinge[1] + OVER[1] - FINGERS / 2])
    yield from left.move(take + 0.4 * (over - take), 1.0)                     # the left hand takes the cover
    yield from together(left.move(over, 1.5), right.move(right.pos + [-0.09, 0.0, -0.04], 1.2))
    yield from left.move(over + [OVER[2] - OVER[0], 0.0, 0.0], 1.5)           # past upright: the cover falls open
    yield from wait(0.8)
    yield from left.move_joints(np.array(READY, float), 1.5)


def script(T, left, right, mode: str = "load"):
    import disckit as K

    yield from wait(0.3)
    if mode == "still":
        yield from wait(8.0)
        return
    if mode == "nohold":
        yield from open_case(T, left, right, hold=False, swing=False)
        return
    yield from open_case(T, left, right)
    if mode == "open":
        return
    hub = np.array([T.HUB_XY[0], T.HUB_XY[1], T.TOP + T.STAND_H])
    button_top = hub[2] + K.BUTTON_TOP
    press = np.array([hub[0], hub[1], button_top - PRESS_DEPTH + TIP])
    disc = T.disc_home()
    pinch = np.array([disc[0] - K.DISC_R + PINCH_IN - TIP, disc[1], disc[2]])

    ready = right.move_joints(best_ik(right, pinch + [-0.06, 0.0, 0.0], LEVEL), 2.5, grip=gap(PINCH_OPEN))
    if mode != "nopress":
        yield from together(ready, left.move_joints(best_ik(left, press + [0.0, 0.0, 0.03], DOWN), 2.5, grip=0.0))
        yield from wait(0.4)
        yield from left.move(press, 1.5)                                    # press the hub's button
        yield from wait(0.3)
    else:
        yield from together(ready, left.move(left.pos + [-0.10, 0.0, 0.02], 2.0))
    yield from right.move(pinch, 1.2)                                       # in through the thumb cut-out
    yield from chain(right.set_grip(gap(PINCH_SHUT), 0.5), wait(0.2))       # the disc's edge between the fingers
    yield from right.move(pinch + [0.0, 0.0, LIFT[0]], 1.0)                 # straight up off the hub's teeth
    if mode != "nopress":
        yield from left.move(press + [0.0, 0.0, 0.039], 0.8)                # let the button go
    high = pinch[2] + LIFT[1]
    yield from right.move(np.array([pinch[0], pinch[1], high]), 1.0)        # up over the hub's button
    reach_back = K.DISC_R * 2 - PINCH_IN + TIP                              # tool to the disc's leading edge
    front = np.array([T.FACE_X - reach_back - 0.02, T.SLOT_Y, T.TOP + CARRY_Z])
    yield from right.move(np.array([pinch[0] - 0.03, pinch[1], high]), 1.2)     # back, clear of the case
    yield from right.move(np.array([front[0], 0.5 * (pinch[1] + front[1]), high]), 2.5)
    yield from chain(right.move(front, 2.0), wait(0.3))                     # down to the slot's height, facing it
    q = right.joints_for(front, UPRIGHT, seed=np.r_[right.q[:5], 0.0])
    yield from chain(right.move_joints(q, 3.0), wait(0.5))                  # stand the disc upright
    if mode == "partway":                                                   # control: 3.5 cm in, held there
        yield from right.move(np.array([T.FACE_X + PART_IN - reach_back, T.SLOT_Y, front[2]]), 2.0)
        yield from wait(3.0)
        return
    reach = T.FACE_X + SLOT_IN - reach_back                                 # tool x with the leading edge SLOT_IN in
    yield from right.move(np.array([reach, T.SLOT_Y, front[2]]), 3.0)      # into the slot
    yield from chain(right.set_grip(1.0, 0.4), wait(0.2))                  # let go: the drive draws it in
    yield from right.move(right.pos + [-0.08, 0.0, 0.0], 1.0)
    yield from wait(1.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "load", every: int = 5) -> dict:
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
    obs = emb.reset(Scene(id="disc-demo", instruction=T.INSTRUCTION))
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

        return object_states(emb._env, ["game_case", "disc"])


    def hands() -> dict:
        from disckit import _np, finger_points

        out = {}
        for n, p in zip(("left", "right"), finger_points(emb._env)):
            robot = emb._env.unwrapped.scene[f"{n}_robot"]
            q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out[n] = {"tool_cm": [round(100 * float(v), 2) for v in p], "q6": round(float(q[names.index("joint6")]), 3),
                      "gap_mm": round(1000 * sum(abs(float(q[names.index(j)])) for j in ("joint7", "joint8")), 1)}
        return out

    def disc_cm():
        from disckit import body_pose

        p, q = body_pose(emb._env, "disc")
        return [round(100 * float(v), 2) for v in p] + [round(float(v), 3) for v in q]

    snap(obs)
    start = T.report(states())
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
            r = T.report(states())
            trace.append({"step": steps, "score": info.get("score"), "task": r, "disc_cm": disc_cm(), "case_cm": [round(100 * float(v), 2) for v in states()["game_case"].pos],
                          "button_mm": round(1000 * states()["game_case"].joints["button"], 2),
                          "cover_deg": [round(math.degrees(states()["game_case"].joints[j]), 1) for j in ("cover", "spine")],
                          "case_quat": [round(float(v), 4) for v in states()["game_case"].quat_xyzw], "hands": hands(), "want_q6": round(float(right.q[5]), 3),
                          "want_cm": {h.name: [round(100 * float(v), 2) for v in h.pos] for h in (left, right)}})
        if steps % (10 if mode.endswith("only") else 40) == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {t['score']} cover {t['cover_deg']} case {t['case_cm']} {t['case_quat']} "
                  f"disc {t['disc_cm']} {t['hands']} {t['task']}", flush=True)
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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="load", choices=("load", "open", "nohold", "nopress", "partway", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every)
