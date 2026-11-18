"""Demo for the lancing-device task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2400] [--no-record] [--mode full|nohold|pull|nocap|still]

1. The left hand comes straight down on the device's body, lifts the device 11 cm off the table and holds it level, its
   nose toward the right arm. It holds it for steps 2 to 5.
2. The right hand, leaning 30 degrees out to its own side, takes the end cap between its fingers, draws it straight
   off the nose and lays it in the kit tray's cradle.
3. It takes the new lancet by its protective tab, brings it level to the nose and pushes it into the holder until the
   holder grips it; then it lets go.
4. It goes round to a level pose pointing at the nose, takes the tab in its fingertips, turns its wrist 100 degrees
   (the tab's neck twists off) and drops the tab in the dish.
5. Leaning again, it takes the cap from the cradle, brings it over the bare needle and the nose and pushes it on until
   it snaps.
6. The left hand lays the device on the table and lets go. The right hand holds the body near the cap, the left takes
   the arming barrel and pulls it back until the device is cocked, and both let go.

Controls: ``--mode nohold`` pulls at the cap with the device lying loose (the device is dragged, the cap stays on);
``--mode pull`` pulls the seated lancet's tab straight back instead of twisting it (the tab stays on);
``--mode nocap`` leaves the cap off and cocks the device with its needle bare; ``--mode still`` keeps both arms still.

Every target comes from the layout constants in the task file and the sizes in ``lancetkit.py``. The only feedback is
each hand's own measured position (the robot's proprioception): to correct an arm's sag before a grasp, and to find
the device's nose from where the other hand is actually holding it. The demo never reads a part's state.
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

# -- hand poses ---------------------------------------------------------------------------------------------------------
LEAN = math.radians(30.0)                # a working hand leans this far out to its own side: two hands pointing straight
                                         # down this close together collide at the wrist
L_DOWN = frame((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0))                       # left, straight down, fingers closing along x
L_AP = np.array([0.0, -math.sin(LEAN), -math.cos(LEAN)])
L_LEAN = frame(L_AP, (-1.0, 0.0, 0.0))                                   # left, leaning toward +y
R_AP = np.array([0.0, math.sin(LEAN), -math.cos(LEAN)])
R_LEAN = frame(R_AP, (1.0, 0.0, 0.0))                                    # right, leaning toward -y
R_LEVEL = frame((0.0, 1.0, 0.0), (1.0, 0.0, 0.0))                        # right, level, pointing at the nose; fingers side by side

HOLD_Z = 0.110                           # the device's axis is held this high above the table
GRIP_AT = 0.066                          # the left hand takes the body this far back from the nose's tip
GRIP_UP = 0.004                          # its tool point this far above the device's axis
IN_PADS = 0.007                          # a leaning hand takes a part with its axis this far along the fingers from the tool
                                         # point, so that the part crosses a fingertip's side edge and not its tip
TABLE_IN = 0.005                         # ... and this far when the part lies on the table (the fingertips clear it by 1.6 mm)
PAD_TIP = 0.013                          # the fingertips reach this far past the tool point
BODY_OPEN, BODY_SHUT = 0.035, 0.0135     # finger gap round the 17 mm body, cap or barrel / commanded on it
TAB_OPEN, TAB_SHUT = 0.020, 0.0050       # finger gap round the 6.4 mm tab / commanded on it
SHY, OVER = 0.010, 0.002                 # the lancet waits this far in front of the mouth, and is pushed this far past home
CAP_SHY = 0.008                          # the cap's mouth waits this far in front of the needle's tip
TWIST = 100.0                            # degrees the wrist turns to twist the tab off
PULL = 0.009                             # the barrel is pulled this far past its travel: both arms give under the spring
HELD = {"on": False}                     # whether the left hand has the device in the air


def turn_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


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


def nose_now(T) -> np.ndarray:
    """The tip of the device's nose: from where the left hand measures itself while it holds the device, else where
    the layout puts it."""
    if HELD["on"]:
        return TOOL["left"] - GRIP_AT * T.ALONG - [0.0, 0.0, GRIP_UP]
    return T.device_start()


def lean_tool(axis_point, depth: float = IN_PADS, ap=R_AP) -> np.ndarray:
    """The tool point of a leaning hand that has a part's axis at ``axis_point`` between its fingers."""
    return np.asarray(axis_point, float) - depth * ap


def cradle(T) -> np.ndarray:
    """The middle of a cap lying in the tray's cradle."""
    return np.array([T.TRAY_XY[0] + T.CRADLE_X, T.TRAY_XY[1], T.cradle_axis_z()])


def lift_device(T, left):
    """Take the device's body from above, lift it and hold it level."""
    d0 = T.device_start()
    at = d0 + GRIP_AT * T.ALONG + [0.0, 0.0, GRIP_UP]
    yield from left.move_joints(best_ik(left, at + [0.0, 0.0, 0.07], L_DOWN), 2.5, grip=gap(BODY_OPEN))
    yield from chain(left.move(at, 1.5), wait(0.2))
    yield from settle(left, at, 1)
    yield from chain(left.set_grip(gap(BODY_SHUT), 0.6), wait(0.3))
    yield from chain(left.move(left.pos + [0.0, 0.0, HOLD_Z - (d0[2] - T.TOP)], 2.0), wait(0.4))
    HELD["on"] = True


def cap_off(T, right, depth: float = IN_PADS, park: bool = True):
    """Take the end cap, draw it straight off the nose and lay it in the tray's cradle."""
    import lancetkit as K

    at = lean_tool(nose_now(T) + (K.NOSE_L - K.CAP_L / 2) * T.ALONG, depth)
    yield from right.move_joints(best_ik(right, at - 0.06 * R_AP, R_LEAN), 2.5, grip=gap(BODY_OPEN))
    yield from chain(right.move(at, 1.2), wait(0.2))
    yield from settle(right, at, 1)
    yield from chain(right.set_grip(gap(BODY_SHUT), 0.6), wait(0.3))
    yield from chain(right.move(right.pos - 0.035 * T.ALONG, 1.5), wait(0.3))
    if not park:
        return
    rest = cradle(T)
    yield from right.move(lean_tool(rest + [0.0, 0.0, 0.05]), 2.5)
    yield from chain(right.move(lean_tool(rest + [0.0, 0.0, 0.0015]), 1.2), wait(0.2))
    yield from chain(right.set_grip(gap(BODY_OPEN), 0.5), wait(0.2))
    yield from right.move(right.pos - 0.05 * R_AP, 0.8)


def load_lancet(T, right, pull_back: bool = False):
    """Take the lancet by its tab, bring it level to the nose and push it into the holder."""
    import lancetkit as K

    grip_z = K.LANCET_L + K.TAB_L - 0.0015                                # the fingers' middle, from the insertion end

    def tool_for_end(end) -> np.ndarray:                                   # the tool point for the lancet's end at ``end``
        return lean_tool(np.asarray(end, float) - grip_z * T.ALONG)

    at = tool_for_end(T.lancet_start())
    yield from right.move_joints(best_ik(right, at - 0.05 * R_AP, R_LEAN), 2.0, grip=gap(TAB_OPEN))
    yield from chain(right.move(at, 1.0), wait(0.2))
    yield from settle(right, at, 1)
    yield from chain(right.set_grip(gap(TAB_SHUT), 0.5), wait(0.3))
    yield from right.move(right.pos + [0.0, 0.0, 0.04], 1.0)
    shy = tool_for_end(nose_now(T) - SHY * T.ALONG)
    yield from right.move(shy + [0.0, 0.0, 0.02], 2.5)
    yield from right.move(shy, 1.0)
    yield from settle(right, shy)
    off = right.pos - shy                                                  # the sag correction, kept through the push
    home = tool_for_end(nose_now(T) + (K.SOCKET_DEPTH + OVER) * T.ALONG)
    yield from chain(right.move(home + off, 2.0), wait(0.4))
    if pull_back:                                                          # control: pull the tab straight back
        yield from chain(right.move(right.pos - 0.030 * T.ALONG, 1.5), wait(0.5))
        return
    yield from chain(right.set_grip(gap(TAB_OPEN), 0.5), wait(0.2))
    yield from right.move(right.pos - 0.040 * T.ALONG + [0.0, 0.0, 0.010], 1.0)


def twist_tab(T, right):
    """From a level pose pointing at the nose, take the tab in the fingertips, twist it off and drop it in the dish."""
    import lancetkit as K
    from yam_arena.constants import READY

    yield from right.move_joints(np.array(READY, float), 2.0, grip=gap(TAB_OPEN))
    base = nose_now(T) - (K.LANCET_L - K.SOCKET_DEPTH) * T.ALONG          # the seated lancet's front, where the tab joins it
    at = base - (PAD_TIP - 0.0005) * T.ALONG                              # the fingertips reach just to the tab's base
    back = at - 0.060 * T.ALONG
    yield from right.move_joints(best_ik(right, back, R_LEVEL), 2.5)
    yield from settle(right, back, 1)
    yield from chain(right.move(at + (right.pos - back), 1.5), wait(0.2))
    yield from settle(right, at, 1)
    yield from chain(right.set_grip(gap(TAB_SHUT), 0.5), wait(0.3))
    yield from chain(right.move(right.pos, 2.0, rot=turn_y(TWIST) @ R_LEVEL), wait(0.3))
    yield from right.move(right.pos - 0.035 * T.ALONG, 1.0)
    over = np.array([T.DISH_XY[0], T.DISH_XY[1] - 0.009, T.TOP + 0.105])
    yield from right.move(over, 2.0)
    yield from chain(right.set_grip(gap(TAB_OPEN), 0.4), wait(0.4))
    yield from right.move(right.pos - 0.050 * T.ALONG, 1.0)
    yield from right.move_joints(np.array(READY, float), 2.0)


def cap_on(T, right):
    """Take the cap from the cradle, bring it over the needle and the nose and push it on until it snaps."""
    import lancetkit as K

    rest = lean_tool(cradle(T) + [0.0, 0.0, 0.0015])
    yield from right.move_joints(best_ik(right, rest - 0.06 * R_AP, R_LEAN), 2.5, grip=gap(BODY_OPEN))
    yield from chain(right.move(rest, 1.2), wait(0.2))
    yield from settle(right, rest, 1)
    yield from chain(right.set_grip(gap(BODY_SHUT), 0.6), wait(0.3))
    yield from right.move(right.pos + [0.0, 0.0, 0.05], 1.0)
    bare = K.LANCET_L - K.SOCKET_DEPTH + K.NEEDLE_L                        # how far the lancet and needle stand out of the nose

    def tool_for_mouth(mouth) -> np.ndarray:                               # the tool point for the cap's mouth at ``mouth``
        return lean_tool(np.asarray(mouth, float) - (K.CAP_L / 2) * T.ALONG)

    start = tool_for_mouth(nose_now(T) - (bare + CAP_SHY) * T.ALONG)
    yield from right.move(start + [0.0, 0.0, 0.02], 2.5)
    yield from right.move(start, 1.0)
    yield from settle(right, start)
    off = right.pos - start
    home = tool_for_mouth(nose_now(T) + (K.NOSE_L + 0.002) * T.ALONG)
    yield from chain(right.move(home + off, 2.5), wait(0.4))
    yield from chain(right.set_grip(gap(BODY_OPEN), 0.5), wait(0.2))
    yield from right.move(right.pos - 0.05 * R_AP, 0.8)


def lay_down(T, left):
    """Lower the device onto the table and let go. Returns nothing; the nose is then where the hand left it."""
    down = np.array([left.pos[0], left.pos[1], T.device_start()[2] + GRIP_UP + 0.0015])
    yield from chain(left.move(down, 2.0), wait(0.3))
    LAID["nose"] = TOOL["left"] - GRIP_AT * T.ALONG - [0.0, 0.0, GRIP_UP]
    HELD["on"] = False
    yield from chain(left.set_grip(gap(BODY_OPEN), 0.5), wait(0.2))
    yield from left.move(left.pos + [0.0, 0.0, 0.05], 0.8)


LAID: dict = {}                          # where the device's nose was when it was laid down


def cock(T, left, right):
    """With the device on the table: the right hand holds the body near the cap, the left pulls the barrel back."""
    import lancetkit as K

    nose = np.array(LAID.get("nose", T.device_start()), float)
    nose[2] = T.device_start()[2]
    r_at = lean_tool(nose + 0.034 * T.ALONG, TABLE_IN)
    l_at = lean_tool(nose + (K.BODY_END + K.BARREL_L / 2) * T.ALONG, TABLE_IN, L_AP)

    def right_in():
        yield from right.move_joints(best_ik(right, r_at - 0.06 * R_AP, R_LEAN), 2.5, grip=gap(BODY_OPEN))
        yield from chain(right.move(r_at, 1.2), wait(0.2))
        yield from settle(right, r_at, 1)

    def left_in():
        yield from left.move_joints(best_ik(left, l_at - 0.06 * L_AP, L_LEAN), 2.5, grip=gap(BODY_OPEN))
        yield from chain(left.move(l_at, 1.2), wait(0.2))
        yield from settle(left, l_at, 1)

    yield from together(right_in(), left_in())
    yield from chain(right.set_grip(gap(BODY_SHUT), 0.6), wait(0.2))
    yield from chain(left.set_grip(gap(BODY_SHUT), 0.6), wait(0.3))
    yield from chain(left.move(left.pos + (K.COCK_TRAVEL + PULL) * T.ALONG, 2.0), wait(0.4))
    yield from chain(left.move(left.pos - (K.COCK_TRAVEL + PULL) * T.ALONG, 1.0), wait(0.2))
    yield from together(chain(left.set_grip(gap(BODY_OPEN), 0.5), wait(0.2)), chain(right.set_grip(gap(BODY_OPEN), 0.5), wait(0.2)))
    yield from together(left.move(left.pos - 0.06 * L_AP, 1.0), right.move(right.pos - 0.06 * R_AP, 1.0))
    yield from together(left.move(left.pos + [0.0, 0.08, 0.0], 1.2), right.move(right.pos + [-0.04, -0.10, 0.0], 1.2))


def script(T, left, right, mode: str = "full", idle_s: float = 60.0):
    HELD["on"] = False
    LAID.clear()
    yield from wait(0.3)
    if mode == "still":
        yield from wait(idle_s)
        return
    if mode == "nohold":                                   # the device lies loose: a pull on the cap only drags it
        yield from cap_off(T, right, depth=TABLE_IN, park=False)
        yield from chain(right.set_grip(gap(BODY_OPEN), 0.5), wait(0.2))
        yield from right.move(right.pos - 0.06 * R_AP, 1.0)
        yield from wait(2.0)
        return
    yield from lift_device(T, left)
    yield from cap_off(T, right)
    yield from load_lancet(T, right, pull_back=(mode == "pull"))
    if mode == "pull":
        yield from wait(2.0)
        return
    yield from twist_tab(T, right)
    if mode != "nocap":
        yield from cap_on(T, right)
    yield from lay_down(T, left)
    yield from cock(T, left, right)
    yield from wait(2.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, window: bool = False,
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
    obs = emb.reset(Scene(id="lancet-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []
    names = [T.DEVICE, T.BARREL, T.CAP, T.LANCET, T.TAB]

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, names)

    def measure() -> None:
        from lancetkit import finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))

    def hands() -> dict:
        from lancetkit import _np

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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "nohold", "pull", "nocap", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")))
