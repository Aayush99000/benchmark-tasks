"""On the station table: the rear panel of a 1U network switch mock-up (a white 3D-printed block, 200 mm wide,
44 mm tall, fixed) stands across the middle of the table, its back face towards the robot. Four rectangular ports
(17.5 x 15.5 mm openings, 26 mm deep, open at table level, 45 mm apart) are in that face. A patch cord is plugged into
each port: a 6 mm round cord, 25 cm long, with a dark grey rigid plug body (55 x 16 x 14 mm) on its panel end; the
cords run straight back towards the robot and lie loose on the table (their far ends are free). Cord colours from
left to right (robot's view): blue, yellow, green, red. Three plugs are fully seated (25 mm inside the port); one
(by default the yellow one, port B) is only half inserted: 8 mm inside, so it sticks out 17 mm further than the
others. The robot must find the loose plug and push it fully home without unseating (even briefly) any of the other
three, then let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Variant: RESEAT_LOOSE=A|B|C|D (env var, default B) chooses which port holds the half-inserted plug.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("One of the four patch cords plugged into the back of the switch is not fully seated. Find the loose "
               "plug and push it fully into its port without pulling or unseating any of the other plugs, then let go.")

WHITE = (0.93, 0.93, 0.92)
PLUG_GREY = (0.16, 0.16, 0.17)
CORD_COLORS = {"A": (0.10, 0.30, 0.85), "B": (0.95, 0.80, 0.10), "C": (0.15, 0.65, 0.25), "D": (0.85, 0.15, 0.12)}

# -- geometry ------------------------------------------------------------------------------------------------------
PORTS = ("A", "B", "C", "D")  # left to right as seen from the robot (+y is left)
PORT_Y = {"A": 0.0675, "B": 0.0225, "C": -0.0225, "D": -0.0675}
FACE_X = 0.40  # the panel's back face (the plugs point +x into it)
PL, PW, PH = 0.055, 0.016, 0.014  # plug body: length along the cord, width, height
SEATED, LOOSE_DEPTH = 0.025, 0.008  # how far a plug's tip is inside its port at reset (m)
CAV_W, CAV_H, CAV_D = PW + 0.0015, PH + 0.0015, 0.026  # port opening and depth (a seated plug bottoms out at 26 mm)
PANEL_W, PANEL_D, PANEL_H = 0.20, 0.06, 0.044
R = 0.003  # 6 mm cord
CORD_LEN = 0.25
PLUG_Z = TOP + PH / 2 + 0.0005  # plug and cord centre height at spawn (the plug sits on the table)

LOOSE = os.environ.get("RESEAT_LOOSE", "B").strip().upper()
if LOOSE not in PORTS:
    raise ValueError(f"RESEAT_LOOSE must be one of {PORTS}, got {LOOSE!r}")
SEATED_PORTS = tuple(p for p in PORTS if p != LOOSE)


def _cord(port: str):
    depth = LOOSE_DEPTH if port == LOOSE else SEATED
    rear = FACE_X + depth - PL  # x of the plug's cord end
    return cable(f"cord_{port.lower()}", start=(rear - CORD_LEN, PORT_Y[port], PLUG_Z), yaw=0.0, length=CORD_LEN,
                 radius=R, anchor_start=False, end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=PLUG_GREY),
                 color=CORD_COLORS[port], bend_stiffness=0.01, bend_damping=0.001)


CORDS = {p: _cord(p) for p in PORTS}


def _panel() -> list[Box]:
    """Back block behind the ports, a roof over them and five walls between/beside them (all fixed, one part)."""
    zc = TOP + PANEL_H / 2
    back_x0 = FACE_X + CAV_D
    out = [
        Box("panel_back", pos=(back_x0 + (PANEL_D - CAV_D) / 2, 0.0, zc), size=(PANEL_D - CAV_D, PANEL_W, PANEL_H),
            color=WHITE),
        Box("panel_roof", pos=(FACE_X + CAV_D / 2, 0.0, (TOP + CAV_H + TOP + PANEL_H) / 2),
            size=(CAV_D, PANEL_W, PANEL_H - CAV_H), color=WHITE),
    ]
    ys = sorted(PORT_Y.values())
    edges = [PANEL_W / 2 * -1] + [(a + b) / 2 for a, b in zip(ys, ys[1:])] + [PANEL_W / 2]
    walls = []
    # wall k spans from the right edge of port k-1 opening to the left edge of port k opening
    bounds = [-PANEL_W / 2] + [v for y in ys for v in (y - CAV_W / 2, y + CAV_W / 2)] + [PANEL_W / 2]
    for k in range(0, len(bounds), 2):
        y0, y1 = bounds[k], bounds[k + 1]
        walls.append(Box(f"panel_wall_{k // 2}", pos=(FACE_X + CAV_D / 2, (y0 + y1) / 2, TOP + CAV_H / 2),
                         size=(CAV_D, y1 - y0, CAV_H), color=WHITE))
    assert edges  # (kept for readability: port pitch midlines)
    return out + walls


PANEL = _panel()
OBJECTS = [*CORDS.values(), *PANEL]

# -- trial history (a seated plug pulled out even briefly drops a live link) -------------------------------------
SEATED_MIN = 0.022  # a plug is seated when its tip is at least 22 mm inside its port
HALF_MIN = 0.015  # partial credit: loose plug pushed at least 15 mm in
RELEASE_DIST = 0.045  # m, both grippers' TCP at least this far from every plug centre = let go


class _Episode:
    def __init__(self):
        self.last = None
        self.unseated: set[str] = set()
        self.min_depth = {p: 1.0 for p in SEATED_PORTS}


_EP = _Episode()
_ENV: dict = {"env": None}


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _install_hooks() -> None:
    """Clear the trial history on every embodiment reset; keep a handle on the env so score() can see the grippers."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_reseat_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._reseat_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_reseat_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._reseat_hooked = True
        cls._task_step = _task_step


_install_hooks()


def depth(port: str, objects) -> float:
    """How far the plug's tip is inside its port (m); -1 if the plug is not in line with its port opening."""
    p = objects[CORDS[port].name].end_plug
    if p is None or not np.isfinite(p).all():
        return -1.0
    if abs(p[1] - PORT_Y[port]) > 0.008 or p[2] > TOP + CAV_H + 0.005:
        return -1.0
    return float(p[0] + PL / 2 - FACE_X)


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (13 mm above the finger pad tips); [] if unknown (offline)."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(objects) -> bool:
    plugs = [objects[c.name].end_plug for c in CORDS.values()]
    return all(float(np.linalg.norm(np.asarray(pl) - g)) >= RELEASE_DIST for g in grippers() for pl in plugs)


def _update(objects) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    for port in SEATED_PORTS:
        d = depth(port, objects)
        _EP.min_depth[port] = min(_EP.min_depth[port], d)
        if d < SEATED_MIN:
            _EP.unseated.add(port)


def score(objects) -> float:
    """0 if any of the three originally seated plugs was ever less than 22 mm inside its port during the trial, or if
    a gripper still holds/touches a plug at the end. Otherwise, by how far the loose plug's tip is inside its port at
    the end: >= 22 mm -> 1.0; 15-22 mm -> 0.5; less -> 0."""
    _update(objects)
    if _EP.unseated or not released(objects):
        return 0.0
    d = depth(LOOSE, objects)
    if d >= SEATED_MIN:
        return 1.0
    return 0.5 if d >= HALF_MIN else 0.0


def success(objects) -> bool:
    _update(objects)
    st = objects[CORDS[LOOSE].name]
    return bool(score(objects) == 1.0 and np.linalg.norm(st.lin_vel) < 0.02)


def distance(objects) -> float:
    """How far (m) the loose plug still is from fully seated."""
    return max(0.0, SEATED - depth(LOOSE, objects))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# The script inspects the panel (plug depths from the live states), picks the plug that sticks out, and the arm on
# that side pinches the exposed plug body (fingers opened only ~4 cm so they clear the neighbouring plugs), slides it
# straight into the port until it bottoms out (the grip slips once it does), lets go and moves clear. The two grippers
# cannot work side by side on ports 45 mm apart, so which arm acts depends on which plug is loose.

FINGER_PINCH = -0.020  # finger opening for the plug pinch (pads ~44 mm apart)


def _log(ctx, tag=""):
    st = ctx.states()
    ds = {p: round(depth(p, st) * 1000, 1) for p in PORTS}
    print(f"[t={ctx.t:5.2f}] {tag} depth_mm={ds} unseated={sorted(_EP.unseated)} "
          f"min={ {k: round(v * 1000, 1) for k, v in _EP.min_depth.items()} } rel={released(st)} "
          f"score={score(st)}", flush=True)


def _arm_for(ctx, port):
    return ctx.left if PORT_Y[port] > 0 else ctx.right


def _push(arm, st, port, *, push_to=0.029, back_off=0.004, hold=False):
    """Pinch ``port``'s plug 4 mm from its rear, slide it until its tip is ``push_to`` deep, release, lift."""
    from yam_arena.constants import FINGER_CLOSED

    p = st[CORDS[port].name].end_plug
    rear = p[0] - PL / 2
    g = np.array([rear + back_off, p[1], TOP + PH + 0.003])
    hi = TOP + 0.09
    yield from arm.move((g[0], g[1], hi), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
    yield from arm.move(g, yaw=0.0, duration=1.2)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    if os.environ.get("RESEAT_DBG"):
        print(f"[dbg] {port} cmd={np.round(g, 4)} actual={[np.round(x, 4) for x in grippers()]} "
              f"plug={np.round(p, 4)}", flush=True)
    tip = p[0] + PL / 2 - FACE_X
    dx = push_to - tip
    if dx > 0:
        yield from arm.move(g + np.array([dx, 0.0, 0.0]), yaw=0.0, duration=max(1.0, dx / 0.012))
    yield from arm.wait(0.6)
    if os.environ.get("RESEAT_DBG"):
        print(f"[dbg] pushed {port} actual={[np.round(x, 4) for x in grippers()]}", flush=True)
    if hold:
        return
    end = g + np.array([max(dx, 0.0), 0.0, 0.0])
    yield from arm.grip(FINGER_PINCH, 0.6)
    yield from arm.move((end[0] - 0.005, end[1], hi), yaw=0.0, duration=1.2)


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    mode = os.environ.get("RESEAT_DEMO", "correct")  # test only: correct | nothing | wrong | pull | hold | all
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(6):
            yield from ctx.left.wait(1.0)
            _log(ctx)
        return
    st = ctx.states()
    ds = {p: depth(p, st) for p in PORTS}
    loose = min(ds, key=ds.get)  # inspection: the plug sticking out farthest
    print(f"[demo] loose plug detected: {loose}", flush=True)
    arm = _arm_for(ctx, loose)
    same_side = [p for p in PORTS if p != loose and _arm_for(ctx, p) is arm]

    if mode == "wrong":  # cheat: push the wrong (already seated) plug on the same side
        yield from _push(arm, st, same_side[0])
        _log(ctx, "end")
        return
    if mode == "pull":  # cheat: tug a seated plug out ~1 cm to "test" it, push it back, then fix the loose one
        from yam_arena.constants import FINGER_CLOSED as FC
        q = same_side[0]
        p = st[CORDS[q].name].end_plug
        g = np.array([p[0] - PL / 2 + 0.010, p[1], TOP + PH + 0.003])
        yield from arm.move((g[0], g[1], TOP + 0.09), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
        yield from arm.move(g, yaw=0.0, duration=1.2)
        yield from arm.grip(FC, 0.6)
        yield from arm.move(g - np.array([0.012, 0.0, 0.0]), yaw=0.0, duration=1.0)
        _log(ctx, "tugged")
        yield from arm.move(g + np.array([0.002, 0.0, 0.0]), yaw=0.0, duration=1.0)
        yield from arm.grip(FINGER_PINCH, 0.6)
        yield from arm.move((g[0], g[1], TOP + 0.09), yaw=0.0, duration=1.0)
        yield from _push(arm, ctx.states(), loose)
        _log(ctx, "end")
        return
    if mode == "hold":  # cheat: push it home but keep holding it
        yield from _push(arm, st, loose, hold=True)
        yield from arm.wait(2.0)
        _log(ctx, "end")
        return
    if mode == "all":  # legitimate: push every plug home in turn (no inspection needed)
        def seq(a, ports):
            for q in ports:
                yield from _push(a, ctx.states(), q)
        yield from seq(ctx.left, ["A", "B"])
        yield from ctx.left.move((0.22, 0.20, TOP + 0.12), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        yield from seq(ctx.right, ["C", "D"])
        yield from ctx.right.move((0.22, -0.20, TOP + 0.12), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        _log(ctx, "end")
        return

    # correct: the arm on the loose plug's side reseats it, lets go and moves clear
    yield from _push(arm, st, loose)
    _log(ctx, "reseated")
    park = np.array([0.22, 0.20 if arm is ctx.left else -0.20, TOP + 0.12])
    yield from arm.move(park, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    _log(ctx, "end")
