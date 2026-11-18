"""On the station table stands a wooden side table: a 50 x 84 cm top, 20 cm above the main table, its near edge 22 cm
in front of the two arms. On it lie a white cloth napkin (23.5 x 28 cm, the size of a dinner napkin folded in
quarters), flat, its near edge at the side table's near edge; a table knife to the left of the napkin; and a dinner
fork to its right. Knife and fork lie parallel to the near edge, handles to the robot's left.

The job is to roll the knife and fork up together in the napkin. ``score`` gives 0 until the napkin goes all the way
round both, 0.3 when it is folded over them, 0.6 when it is rolled on, 0.8 when it is rolled up, and 1.0 when the roll
lies on the side table by itself with both grippers clear of it (see ``_credit``).
The napkin is true cloth: grippers cannot get under it where it lies flat, only take it where it hangs free.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # labkit sits next to this file

from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

import numpy as np  # noqa: E402

from labkit import Cloth, Marked, Model, Pose, finger_points, on_reset, ready_pose, trial_time  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- colours (sRGB 0..1) ------------------------------------------------------------------------------------------------
WOOD, LINEN = (0.62, 0.47, 0.28), (0.95, 0.95, 0.93)

# -- layout (m, world frame: x forward from the arm bases, y left, z up; arm bases at x 0, y +-0.305) -------------------
# Side table: a 2 cm top on two leg boards. Its top is smooth varnished wood, so a napkin slides on it.
BENCH_H = 0.20                                   # height of the side table's top above the main table
BENCH_TOP_Z = TOP + BENCH_H                      # the side table's top surface
BENCH_X, BENCH_Y, SLAB = (0.22, 0.72), 0.84, 0.02  # near and far edge (x), width (y), top thickness
LEG_SETBACK, LEG_T = 0.08, 0.02
TABLE_FRICTION = 0.2
BENCH_TOP = Marked("bench_top", pos=(sum(BENCH_X) / 2, 0.0, BENCH_TOP_Z - SLAB / 2),
                   size=(BENCH_X[1] - BENCH_X[0], BENCH_Y, SLAB), color=WOOD, surface_friction=TABLE_FRICTION)
LEG_H = BENCH_TOP_Z - SLAB - TOP
BENCH_LEG_NEAR = Box("bench_leg_near", pos=(BENCH_X[0] + LEG_SETBACK + LEG_T / 2, 0.0, TOP + LEG_H / 2),
                     size=(LEG_T, BENCH_Y, LEG_H), color=WOOD)
BENCH_LEG_FAR = Box("bench_leg_far", pos=(BENCH_X[1] - 0.02 - LEG_T / 2, 0.0, TOP + LEG_H / 2),
                    size=(LEG_T, BENCH_Y, LEG_H), color=WOOD)

# Napkin: a 47 x 56 cm dinner napkin folded in quarters, as ONE sheet of true cloth with the folded napkin's size,
# thickness (2.5 mm) and weight (68 g). It starts flat, wholly on the side table, its near edge 5 mm in from the
# table's near edge.
NAPKIN_DEPTH, NAPKIN_WIDTH, NAPKIN_THICK = 0.235, 0.28, 0.0025
START_GAP = 0.005
NAPKIN_NEAR_X = BENCH_X[0] + START_GAP
NAPKIN = Cloth("napkin", pos=(NAPKIN_NEAR_X + NAPKIN_DEPTH / 2, 0.0, BENCH_TOP_Z + NAPKIN_THICK / 2 + 0.001),
               size=(NAPKIN_DEPTH, NAPKIN_WIDTH, NAPKIN_THICK), color=LINEN, quarter=False, self_collide=True,
               thickness=NAPKIN_THICK, density=0.068 / (NAPKIN_DEPTH * NAPKIN_WIDTH * NAPKIN_THICK),
               rest_offset=NAPKIN_THICK / 2, contact_offset=NAPKIN_THICK / 2 + 0.004,
               bend=500.0, youngs=1e6, damping=1.0, bend_damping=5.0, edge=0.007, friction=1.5)

# Knife and fork: our own models, generated in code by models/make_cutlery.py (no third-party asset) and converted to
# USD with several convex pieces for collision. Each model's long axis is x, handle end first; ``pos`` is its box centre.
# They start parallel to the near edge and 4 cm in from it, the knife left of the napkin, the fork right of it,
# handles to the robot's left.
KNIFE_SIZE, FORK_SIZE = (0.240, 0.0205, 0.0070), (0.195, 0.0250, 0.0132)
START_IN = 0.04
KNIFE_START, FORK_START = (BENCH_X[0] + START_IN, 0.275), (BENCH_X[0] + START_IN, -0.2525)
KNIFE = Model("knife", pos=(*KNIFE_START, BENCH_TOP_Z + KNIFE_SIZE[2] / 2 + 0.001), size=KNIFE_SIZE, mass=0.085,
              usd=str(HERE / "models" / "knife.usd"), yaw=-math.pi / 2)
FORK = Model("fork", pos=(*FORK_START, BENCH_TOP_Z + FORK_SIZE[2] / 2 + 0.001), size=FORK_SIZE, mass=0.045,
             usd=str(HERE / "models" / "fork.usd"), yaw=-math.pi / 2)

OBJECTS = [BENCH_TOP, BENCH_LEG_NEAR, BENCH_LEG_FAR, NAPKIN, KNIFE, FORK]


# -- scoring ------------------------------------------------------------------------------------------------------------
TOGETHER_M, TOGETHER_DEG = 0.03, 20.0     # the fork's middle within 3 cm of the knife's centre line, and within 20 degrees of it
ROUND_SHARE = 0.75                         # napkin all the way round at least this share of the middle 80% of each piece
TURNS_FOLDED, TURNS_ROLLED, TURNS_DONE = 0.9, 1.15, 1.6   # times the napkin goes round the knife: folded over, rolled on, rolled up
ROLL_WIDTH = 0.11                          # rolled up, the napkin is at most this wide across the knife (m); it starts 23.5 cm
REST_HEIGHT = 0.04                         # lying on the side table: the knife's and the fork's centres at most this far above its top (m)
CLEAR_GAP, CLEAR_S = 0.02, 2.0             # let go: every finger pad at least 2 cm from the napkin, the knife and the fork, for the last 2 s
SUCCESS_HOLD_S = 2.0
STILL_M_S, STILL_RAD_S = 0.01, 0.5


def _axis_of(o) -> np.ndarray:
    """Unit vector along a knife's or fork's length, handle to tip, in the world."""
    x, y, z, w = (float(v) for v in o.quat_xyzw)
    return np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)])


def _measure(objects) -> dict:
    """What a judge would measure on the finished job, from the napkin's mesh nodes and the two pieces' poses:
    ``turns``   how many times the napkin goes round the knife's handle, counted at its middle (lying open under the
                knife it is half a turn; folded over once, about one; each half roll adds about a half);
    ``width``   the napkin's width across the knife (m): the roll and whatever is left flat beside it;
    ``knife_in``, ``fork_in``  the share of each piece's middle 80% that has napkin all the way round it;
    ``apart``, ``angle``  how far the fork's middle lies from the knife's centre line (m) and the angle between them (deg);
    ``on_table``  the knife, the fork and the napkin all rest on the side table."""
    k, f, n = objects["knife"], objects["fork"], objects["napkin"]
    nodes = getattr(n, "nodes", None)
    out = {"turns": 0.0, "width": NAPKIN_DEPTH, "knife_in": 0.0, "fork_in": 0.0, "apart": 1.0, "angle": 90.0, "on_table": False,
           "resting": False, "clear_s": 0.0}
    if nodes is None or len(nodes) != NAPKIN.node_count:
        return out
    a = _axis_of(k)
    u = np.array([-a[1], a[0], 0.0]); u = u / max(np.linalg.norm(u), 1e-9); v = np.cross(a, u)   # across the knife, level; and up

    def all_round(o, length) -> float:
        ax = _axis_of(o); side = np.array([-ax[1], ax[0], 0.0]); side = side / max(np.linalg.norm(side), 1e-9); up = np.cross(ax, side)
        rel = nodes - o.pos; along = rel @ ax; hits = 0; stations = np.linspace(-0.4, 0.4, 9) * length
        for s in stations:
            near = rel[np.abs(along - s) < 0.010]
            r = np.stack([near @ side, near @ up], 1); r = r[np.linalg.norm(r, axis=1) < 0.035]
            if len(r) < 4:
                continue
            ang = np.sort(np.arctan2(r[:, 1], r[:, 0])); gaps = np.diff(np.concatenate([ang, ang[:1] + 2 * math.pi]))
            hits += bool(gaps.max() < math.radians(120.0))          # napkin on every side of the piece here
        return hits / len(stations)

    grid = nodes.reshape(NAPKIN.cells[0] + 1, NAPKIN.cells[1] + 1, 3)     # node (i, j): i across the napkin's depth, j along its width
    rel = grid - (k.pos - 0.055 * a)                                       # counted at the middle of the knife's handle
    along = np.abs(np.median(rel @ a, axis=0))                             # each line of nodes that crosses the knife: how far along it
    turns = []
    for j in np.argsort(along)[:7]:
        line = rel[:, j, :]; ang = np.unwrap(np.arctan2(line @ v, line @ u))
        turns.append(abs(ang[-1] - ang[0]) / (2 * math.pi))
    across = (nodes - k.pos) @ u
    fa = _axis_of(f); d = f.pos - k.pos
    top_ok = lambda o: BENCH_X[0] - 0.02 < o.pos[0] < BENCH_X[1] and abs(o.pos[1]) < BENCH_Y / 2 and -0.005 < o.pos[2] - BENCH_TOP_Z < 0.08
    out.update(turns=float(np.median(turns)), width=float(np.percentile(across, 99) - np.percentile(across, 1)),
               knife_in=all_round(k, KNIFE_SIZE[0]), fork_in=all_round(f, FORK_SIZE[0]),
               apart=float(np.linalg.norm(d - (d @ a) * a)), angle=float(math.degrees(math.acos(min(1.0, abs(float(fa @ a)))))),
               on_table=bool(top_ok(k) and top_ok(f) and float(nodes[:, 2].min()) > BENCH_TOP_Z - 0.10
                             and BENCH_X[0] - 0.02 < float(nodes[:, 0].mean()) < BENCH_X[1] and abs(float(nodes[:, 1].mean())) < BENCH_Y / 2))
    out.update(resting=bool(out["on_table"] and max(k.pos[2], f.pos[2]) - BENCH_TOP_Z <= REST_HEIGHT), clear_s=_clear_for(objects))
    return out


_CLEAR = {"since": None, "at": 0.0}


@on_reset
def _new_trial() -> None:
    _CLEAR.update(since=None, at=0.0)


def _finger_gap(objects) -> float:
    """The smallest distance (m) from any finger pad of either gripper to the napkin, the knife or the fork; 0 when the
    finger pads are not known. The knife and fork are taken as points every centimetre along their centre lines."""
    pads = finger_points()
    nodes = getattr(objects["napkin"], "nodes", None)
    if pads is None or nodes is None or not len(nodes):
        return 0.0
    pts = [np.asarray(nodes, float)]
    for o, length in ((objects["knife"], KNIFE_SIZE[0]), (objects["fork"], FORK_SIZE[0])):
        pts.append(np.asarray(o.pos, float) + np.outer(np.linspace(-length / 2, length / 2, int(round(100 * length)) + 1), _axis_of(o)))
    pts = np.vstack(pts)
    return float(min(np.linalg.norm(pts - p, axis=1).min() for p in np.asarray(pads, float)))


def _clear_for(objects) -> float:
    """How long (s) both grippers have been clear of the napkin, the knife and the fork, up to now."""
    t = trial_time()
    if t < _CLEAR["at"]:                         # the clock went back: a new trial
        _CLEAR["since"] = None
    _CLEAR["at"] = t
    if _finger_gap(objects) < CLEAR_GAP:
        _CLEAR["since"] = None
        return 0.0
    if _CLEAR["since"] is None:
        _CLEAR["since"] = t
    return t - _CLEAR["since"]


def _credit(m: dict) -> float:
    """The score for one set of measurements (see ``_measure``).
    0    unless the knife, the fork and the napkin are on the side table, the fork lies along the knife (its middle
         within 3 cm of the knife's centre line and within 20 degrees of it), and the napkin goes all the way round at
         least three quarters of each one's middle 80%, at least 0.9 times round the knife;
    0.3  folded over them, as above;
    0.6  rolled on: the napkin goes at least 1.15 times round the knife;
    0.8  rolled up: at least 1.6 times round, and the napkin no wider than 11 cm across the knife;
    1.0  rolled up and let go: the roll lies on the side table (the knife's and the fork's centres no more than 4 cm
         above its top) and every finger pad of both grippers has been at least 2 cm from the napkin, the knife and
         the fork for the last 2 s."""
    if not m["on_table"] or m["apart"] > TOGETHER_M or m["angle"] > TOGETHER_DEG:
        return 0.0
    if min(m["knife_in"], m["fork_in"]) < ROUND_SHARE or m["turns"] < TURNS_FOLDED:
        return 0.0
    if m["turns"] >= TURNS_DONE and m["width"] <= ROLL_WIDTH:
        return 1.0 if m["resting"] and m["clear_s"] >= CLEAR_S else 0.8
    return 0.6 if m["turns"] >= TURNS_ROLLED else 0.3


_STEP = [0]


def score(objects) -> float:
    m = _measure(objects)
    s = _credit(m)
    _STEP[0] += 1
    if os.environ.get("ROLL_SILVERWARE_LOG") and _STEP[0] % 20 == 1:
        print(f"[roll] step {_STEP[0]:4d}: score {s:.1f}; napkin {m['turns']:.2f} times round the knife, {100 * m['width']:.1f} cm wide; all round the "
              f"knife {m['knife_in']:.2f}, the fork {m['fork_in']:.2f}; fork {100 * m['apart']:.1f} cm and {m['angle']:.0f} deg from the knife; "
              f"on the side table {m['on_table']}, lying on it {m['resting']}; grippers clear for {m['clear_s']:.1f} s{_also(objects)}", flush=True)
    return s


def _also(objects) -> str:
    """Two more numbers for the log, not used in scoring: how far the napkin hangs below the side table's top (its
    lowest point, and the lowest point of its left and right ends), and how far the knife is tipped from face up."""
    nodes = getattr(objects["napkin"], "nodes", None)
    if nodes is None or not len(nodes):
        return ""
    drop = lambda sel: 100 * max(0.0, BENCH_TOP_Z - float(nodes[sel][:, 2].min())) if sel.any() else 0.0
    x, y, _, _ = (float(v) for v in objects["knife"].quat_xyzw)
    tipped = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    return (f"; napkin hangs {drop(nodes[:, 1] > -9):.1f} cm below the table top (left end {drop(nodes[:, 1] > 0.10):.1f}, right end "
            f"{drop(nodes[:, 1] < -0.10):.1f}); knife tipped {tipped:.0f} deg from face up")


def success(objects) -> bool:
    """Rolled up and let go (score 1.0) with the knife and fork lying still; held for SUCCESS_HOLD_S it ends the trial."""
    if not objects or "napkin" not in objects:
        return False
    still = all(float(np.linalg.norm(objects[n].lin_vel)) < STILL_M_S and float(np.linalg.norm(objects[n].ang_vel)) < STILL_RAD_S for n in ("knife", "fork"))
    return still and _credit(_measure(objects)) == 1.0


# -- scripted demo ------------------------------------------------------------------------------------------------------
# Five pieces, played one after the other: pull the napkin, lay the knife and fork on it, fold the hanging part over
# them, turn the bundle a quarter turn, roll it. What the simulator's cloth and these grippers do, found by running it:
# a fingertip pinch holds cloth by friction that creeps under load, so cloth is always taken with a whole pad on it and
# never pulled hard; the fingers are wedges whose pads meet only over their last 13 mm; and a napkin lying flat cannot
# be picked up at all, only one that hangs free.

# A gripper leaning 30 degrees forward, fingers closed side by side: the fingertips press down on the table.
_TILT = math.radians(30.0)
_APPROACH = np.array([math.sin(_TILT), 0.0, -math.cos(_TILT)])
_CLOSING = np.array([0.0, -1.0, 0.0])
PRESS_ROT = np.column_stack([_CLOSING, np.cross(_APPROACH, _CLOSING), _APPROACH])  # gripper x, y, z axes in the world

# A gripper pointing straight forward, level, one finger above the other.
PINCH_ROT = np.column_stack([np.array([0.0, 0.0, 1.0]), np.array([0.0, -1.0, 0.0]), np.array([1.0, 0.0, 0.0])])


def _tip(x: float, y: float, z: float) -> tuple[float, float, float]:
    """The TCP position that puts the pad tips at (x, y, z) with the gripper leaning forward."""
    return tuple(np.array([x, y, z]) - PAD_TIP_PAST_TCP * _APPROACH)


def _spin(deg: float) -> np.ndarray:
    """The forward pinch turned ``deg`` about the forward axis (a turn of the wrist)."""
    a = math.radians(deg); c, sn = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -sn], [0.0, sn, c]]) @ PINCH_ROT


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg); c, sn = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]])


def _rz(deg: float) -> np.ndarray:
    a = math.radians(deg); c, sn = math.cos(a), math.sin(a)
    return np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])


def _then(first: dict, second: dict) -> dict:
    """Two plans one after the other; each arm's second part starts when the longer first part has ended."""
    end = max(sum(w.duration for w in wps) for wps in first.values())
    out = {}
    for arm in ("left", "right"):
        a = list(first.get(arm, [])); pad = end - sum(w.duration for w in a)
        b = list(second.get(arm, []))
        out[arm] = a + ([ready_pose(arm, pad, "wait")] if pad > 0.01 else []) + b
    return out


def pull_plan() -> dict:
    """1. Pull the napkin. Both grippers, closed and leaning forward, press their fingertips on the napkin 13 cm in
    from the table's near edge and draw it toward the robot until they reach the edge. The napkin's near part slides
    over the edge and hangs down: 6.1 cm of it, a quarter. (The fingertips creep on the cloth, so the napkin moves
    about half as far as they do.)"""
    edge, top = BENCH_X[0], BENCH_TOP_Z
    x_from, x_to, press_z, hover_z = 0.13, 0.005, top + NAPKIN_THICK - 0.014, top + 0.03
    plan = {}
    for arm, y in (("left", 0.08), ("right", -0.08)):
        plan[arm] = [
            Pose(2.0, _tip(edge + x_from, y, top + 0.05), FINGER_CLOSED, "over the napkin", rot=PRESS_ROT),
            Pose(1.0, _tip(edge + x_from, y, press_z), FINGER_CLOSED, "press the fingertips on the napkin", rot=PRESS_ROT),
            Pose(3.0, _tip(edge + x_to, y, press_z), FINGER_CLOSED, "draw the napkin toward the table's near edge", rot=PRESS_ROT),
            Pose(0.6, _tip(edge + x_to, y, hover_z), FINGER_CLOSED, "lift off", rot=PRESS_ROT),
            ready_pose(arm, 2.0),
        ]
    return plan


KNIFE_IN = 0.015                                 # the knife's and the fork's centre line ends up this far in from the near edge
FORK_Y = (KNIFE_SIZE[0] - FORK_SIZE[0]) / 2      # where the fork's middle ends up: its handle end level with the knife's
KNIFE_GRIP, FORK_GRIP = 0.085, 0.095             # where each is held, measured from its handle end: about its balance point
KNIFE_GRIP_W, FORK_GRIP_W = 0.0169, 0.0091       # how wide each is there: the knife's handle, the fork's neck


def lay_plan() -> dict:
    """2. Lay the knife and fork on the napkin. Each gripper points down, leaning 15 degrees toward the centre line
    (pointing straight down it cannot reach the middle of the table), one fingertip in front of its piece and one
    behind. The left gripper takes the knife by its handle, carries it sideways and lays it on the napkin along the
    near edge. The right gripper has meanwhile picked the fork up by its neck (the fork is arched, so its neck stands
    9 mm clear of the table); once the knife is down it lays the fork on the knife, handle ends together."""
    edge, top = BENCH_X[0], BENCH_TOP_Z
    x, lean, high, squeeze = edge + KNIFE_IN, math.radians(15.0), 0.03, 0.0015
    jobs = (   # arm, side, piece, (x, y) of the hold at the start, and on the napkin, width there, pad tips' height at the pick and at the lay, lift, wait
        ("left", 1.0, "knife", (KNIFE_START[0], KNIFE_START[1] + KNIFE_SIZE[0] / 2 - KNIFE_GRIP), (x, KNIFE_SIZE[0] / 2 - KNIFE_GRIP),
         KNIFE_GRIP_W, 0.004, 0.004 + NAPKIN_THICK + 0.001, 0.008, 0.0),
        ("right", -1.0, "fork", (FORK_START[0], FORK_START[1] + FORK_SIZE[0] / 2 - FORK_GRIP), (x, FORK_Y + FORK_SIZE[0] / 2 - FORK_GRIP),
         FORK_GRIP_W, 0.006, 0.006 + NAPKIN_THICK + KNIFE_SIZE[2] + 0.002, 0.020, 4.8),
    )
    plan = {}
    for arm, sy, what, pick, lay, width, tip_pick, tip_lay, lift, wait in jobs:
        a = np.array([0.0, -sy * math.sin(lean), -math.cos(lean)])           # pointing down, leaning toward the centre line
        c = np.array([-sy, 0.0, 0.0])                                         # the fingers close front to back, level
        rot = np.column_stack([c, np.cross(a, c), a])
        at = lambda xy, tip_z, a=a: tuple(np.array([xy[0], xy[1], top + tip_z]) - PAD_TIP_PAST_TCP * a)   # the TCP that puts the pad tips there
        open_, held = -(width / 2 + 0.007), -(width / 2 - squeeze)
        on = "on the napkin along the edge" if what == "knife" else "on the knife, handle ends together"
        # Closing on the knife's tapered handle skews it the same way every time (2.9 degrees, 3 mm along the edge), so
        # the wrist turns back by that much, about the vertical, on the way to the napkin and the knife lands square.
        lay_rot = _rz(-2.9 if what == "knife" else 0.0) @ rot
        if what == "knife":
            lay = (lay[0] - 0.0004, lay[1] - 0.003)
        at_lay = lambda xy, tip_z, a=lay_rot[:, 2]: tuple(np.array([xy[0], xy[1], top + tip_z]) - PAD_TIP_PAST_TCP * a)
        # Right in front of its own base an arm cannot raise a gripper that points down very far: it lifts just clear
        # at the pick, and rises further on the way to the napkin.
        plan[arm] = [
            Pose(2.5, at(pick, lift), open_, f"above the {what}", rot=rot),
            Pose(1.2, at(pick, tip_pick), open_, f"down: one fingertip in front of the {what}, one behind", rot=rot),
            Pose(0.8, at(pick, tip_pick), held, f"close on the {what}", rot=rot),
            Pose(1.0, at(pick, lift), held, f"lift the {what}", rot=rot),
        ] + ([Pose(wait, at(pick, lift), held, "wait until the knife is down", rot=rot)] if wait else []) + [
            Pose(3.0, at_lay(lay, high), held, f"carry the {what} sideways over the napkin", rot=lay_rot),
            Pose(1.2, at_lay(lay, tip_lay), held, f"lower the {what} {on}", rot=lay_rot),
            Pose(0.6, at_lay(lay, tip_lay), open_, f"let go of the {what}", rot=lay_rot),
            Pose(1.0, at_lay(lay, high), open_, "up", rot=lay_rot),
            ready_pose(arm, 2.5),
        ]
    return plan


FLAP = 0.061                                     # how much napkin hangs over the near edge after the pull
FOLD_Y = 0.125                                   # each gripper's distance from the centre line: its whole pad on the napkin, 1.5 cm in from the side edge


def fold_plan() -> dict:
    """3. Fold the hanging part over the knife and fork. Both arms, near the two ends of the hanging edge. Each gripper
    points forward, level, wide open, one finger above the other. It comes in with its lower fingertip under the
    hanging edge and its upper fingertip above the napkin on the table; lifts the edge 1 cm on the lower pad; draws
    back so the fingers are clear of the table; and closes downward, so the upper pad comes down onto the edge lying
    on the lower pad. Then straight up in front of the table edge, forward over the knife and fork with the fingers
    tipped down, to where the napkin is just taut over them. The upper finger lifts off and both grippers slide out
    sideways, away from each other."""
    edge, top = BENCH_X[0], BENCH_TOP_Z
    g = -FINGER_OPEN                             # each finger's travel from closed to wide open
    x_in, back, part = -0.005, 0.016, 0.012      # TCP x when the fingertips are 8 mm past the edge; how far it draws back; how far each finger opens to let go
    low = FLAP + 0.007                           # the lower pad starts this far below the table top: 7 mm under the hanging edge
    z0 = -low + g                                # the TCP height that puts the lower pad there
    lay = (0.021, 0.033)                         # where the napkin is let go, in from the edge and above the top: just taut over the knife and fork, the fingers clear of the fork's handle
    plan = {}
    for arm, sy in (("left", 1.0), ("right", -1.0)):
        y, out = sy * FOLD_Y, sy * 0.215
        stack = _spin(0.0 if arm == "left" else 180.0)         # pointing forward, one finger above the other (mirrored)
        down = _ry(15.0) @ stack
        plan[arm] = [
            Pose(2.5, (edge - 0.045, y, top + 0.04), FINGER_OPEN, "in front of the table edge, near one end of the hanging edge", rot=_spin(90.0)),
            Pose(1.3, (edge - 0.050, y, top + z0), FINGER_OPEN, "down in front of the hanging napkin", rot=_spin(90.0)),
            Pose(1.4, (edge - 0.050, y, top + z0), FINGER_OPEN, "turn the wrist: one finger above the other, the lower one below the hanging edge", rot=stack),
            Pose(1.2, (edge + x_in, y, top + z0), FINGER_OPEN, "forward: lower fingertip under the hanging edge", rot=stack),
            Pose(0.6, (edge + x_in, y, top + z0 + 0.010), FINGER_OPEN, "up 1 cm: the edge rides on the lower pad", rot=stack),
            Pose(0.7, (edge + x_in - back, y, top + z0 + 0.020), FINGER_OPEN, "back and up: clear of the table", rot=stack),
            Pose(1.4, (edge + x_in - back, y, top - low + 0.020), FINGER_CLOSED, "close downward onto the edge lying on the lower pad", rot=stack),
            Pose(0.3, (edge + x_in - back, y, top - low + 0.020), FINGER_CLOSED, "hold", rot=stack),
            Pose(1.0, (edge + x_in - back, y, top + 0.032), FINGER_CLOSED, "straight up in front of the table edge", rot=stack),
            Pose(1.4, (edge + lay[0], y, top + lay[1]), FINGER_CLOSED, "forward over the knife and fork, tipping down", rot=down),
            Pose(1.2, (edge + lay[0], y, top + lay[1]), FINGER_CLOSED, "hold the napkin there while it settles", rot=down),
            Pose(0.8, (edge + lay[0], y, top + lay[1] + part), -part, "let go: the upper finger lifts off", rot=down),
            Pose(1.5, (edge + lay[0], out, top + lay[1] + part), -part, "slide out sideways, away from the other gripper", rot=down),
            Pose(1.0, (edge + lay[0] - 0.03, out, top + 0.10), FINGER_OPEN, "up", rot=down),
            ready_pose(arm, 2.5),
        ]
    return plan


TURN_AT = (0.0135, 0.077)                        # where the turning gripper holds the knife and fork: in from the near edge, left of the centre line


def turn_plan(wait: float) -> dict:
    """4. Turn the bundle a quarter turn. Left arm; the right arm waits. The gripper points straight down over the folded
    napkin, one fingertip in front of the knife (just off the table edge) and the other behind the fork. It closes
    until the two fingertips hold the knife and fork between them through the cloth, 4 cm from their handle ends; then
    the wrist turns about the vertical, slowly, so the bundle slides round on the table with it. The knife and fork end
    up pointing away from the robot, handle ends 3 cm over the table edge, the rest of the napkin to the robot's left."""
    edge, top = BENCH_X[0], BENCH_TOP_Z
    x, y = edge + TURN_AT[0], TURN_AT[1]
    low = top + PAD_TIP_PAST_TCP + 0.004         # fingertips 4 mm above the table top
    hold, start = -0.027 / 2, -0.055 / 2         # the fingers stop 2.7 cm apart (knife, fork and cloth); they come down 5.5 cm apart
    a, c = np.array([0.0, 0.0, -1.0]), np.array([-1.0, 0.0, 0.0])
    v0 = np.column_stack([c, np.cross(a, c), a])   # pointing down, fingers one in front of the other
    full = 114.0                                 # 90 degrees plus the play the pair has between two narrow fingertips
    yaw = lambda deg: _rz(deg) @ v0
    t = 12.0                                     # slow: the cloth is damped and lags a quick turn, which strips it off the knife and fork
    # Pointing straight down, the arm cannot hold the gripper more than about 2 cm above the bundle here. It waits at
    # the nearest pose it can reach: 5.6 cm above the table, 3.7 cm further in and 3.7 cm to the right of the grip
    # point, tipped 13.8 degrees toward it; from there the fingers come down either side of the knife and fork.
    via = (x + 0.0377, y - 0.0368, top + 0.0696)
    k = np.array([math.cos(math.radians(45.7)), math.sin(math.radians(45.7)), 0.0]); ang = math.radians(-13.8)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    tip = np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)
    left = [
        ready_pose("left", wait, "wait"),
        Pose(2.0, via, start, "above the folded napkin, beyond it and to its right, the gripper tipped a little toward it", rot=tip @ yaw(0.0)),
        Pose(1.2, (x, y, low), start, "down: one fingertip behind the fork, on the napkin's folded-over end, one in front of the knife", rot=yaw(0.0)),
        Pose(1.2, (x, y, low), hold, "close on the knife and fork through the cloth", rot=yaw(0.0)),
        Pose(t / 2, (x, y, low), hold, "turn the wrist: the bundle slides round on the table", rot=yaw(full / 2)),
        Pose(t / 2, (x, y, low), hold, "turn the wrist: the bundle slides round on the table", rot=yaw(full)),
        Pose(0.8, (x, y, low), FINGER_OPEN, "let go", rot=yaw(full)),
        Pose(1.2, via, FINGER_OPEN, "lift off", rot=tip @ yaw(full)),
        ready_pose("left", 2.0),
    ]
    return {"left": left, "right": [ready_pose("right", sum(w.duration for w in left), "wait")]}


ROLL_AT = 0.077                                  # y of the knife's and fork's handles after the turn
HOLD_LEFT_OF_HANDLES = 0.009                     # the holding gripper presses this far to the left of where the handles come to rest


def roll_plan(wait: float) -> dict:
    """5. Roll the bundle up: two half turns, with a fresh hold for each. The left gripper points forward, level, and
    pinches the knife's and fork's handle ends together with the napkin where they stick out over the table edge. Its
    wrist turns half a turn while the hand moves left on a low arc, so the bundle tips over onto the rest of the napkin
    and the napkin winds on. The bundle is now upside down and would rock back on the fork's arched neck if it were let
    go, so the right gripper reaches over and holds it down with its fingertips while the left one opens a few
    millimetres, slides straight back out of the roll, turns its wrist back clear of the cloth and takes hold again
    from the bare side. The second half turn brings the bundle right side up, where it lies flat by itself."""
    edge, top = BENCH_X[0], BENCH_TOP_Z
    x_grip, x_back, z_high = edge - 0.018, edge - 0.050, top + 0.10
    travel = (0.064, 0.055)                      # the hand's move to the left in each half turn
    height = (top + 0.007, top + 0.015)          # the pinch's height in each half turn
    lift, step, side, ajar, rel = 0.007, 0.8, 0.06, 0.025, 0.020
    spin = lambda deg: _spin(180.0 - deg)        # 0: pads level at the start of a half turn; 180: at its end
    y = ROLL_AT
    left = [
        ready_pose("left", wait, "wait"),
        Pose(2.5, (edge - 0.045, y - side, top + 0.05), FINGER_OPEN, "in front of the table edge, to the right of the handles", rot=_spin(90.0)),
        Pose(1.2, (x_back, y - side, top + 0.05), FINGER_OPEN, "back", rot=_spin(90.0)),
        Pose(1.4, (x_back, y - side, top + 0.05), FINGER_OPEN, "turn the wrist: one finger above the other", rot=spin(0)),
        Pose(1.0, (x_back, y - side, height[0]), FINGER_OPEN, "down, level with the handles", rot=spin(0)),
    ]
    clock = lambda: sum(p.duration for p in left)
    marks = {}
    for k in range(2):
        zk, wk = height[k], travel[k]
        # The gripper comes in from the bundle's right, where no napkin hangs, its fingers only a hand's width apart, and
        # slides over and under the handles sideways: it takes hold of the cloth and the handles together from outside
        # the cloth. (Wide open and coming straight in, the lower finger catches the napkin hanging by the handles.)
        left += [
            Pose(0.5, (x_back, y - side, zk), -ajar, "close the fingers to a hand's width", rot=spin(0)),
            Pose(1.0, (x_grip, y - side, zk), -ajar, "forward, beside the bundle's end", rot=spin(0)),
            Pose(1.2, (x_grip, y, zk), -ajar, "sideways: one finger just over the handles, one just under", rot=spin(0)),
            Pose(1.0, (x_grip, y, zk), FINGER_CLOSED, "pinch the handles and the napkin together", rot=spin(0)),
        ]
        if k == 1:
            marks["held again"] = clock()
            left.append(Pose(0.8, (x_grip, y, zk), FINGER_CLOSED, "wait while the other gripper lifts off", rot=spin(0)))
        if k == 0:
            marks["roll starts"] = clock()
        left += [Pose(step, (x_grip, y + wk * d / 180.0, zk + lift * math.sin(math.radians(d))), FINGER_CLOSED,
                      "roll: turn the wrist while the hand moves left, as the bundle tips over onto the napkin", rot=spin(d)) for d in (45, 90, 135, 180)]
        y += wk
        if k == 0:
            marks["roll ends"], marks["y"] = clock(), y
        left += [
            Pose(1.6 if k == 0 else 1.0, (x_grip, y, zk), FINGER_CLOSED, "hold", rot=spin(180)),
            # The fingers are now inside the wound napkin. Opened wide in there they catch the cloth and drag the bundle
            # off with them: they open just enough to free the handles and slide straight back out, then move sideways
            # clear of the napkin's end (which sticks out 2 cm past the handles) before going up.
            Pose(0.7, (x_grip, y, zk), -rel, "let go: open the fingers a few millimetres", rot=spin(180)),
            Pose(1.2, (x_back, y, zk), -rel, "pull straight back, out of the roll", rot=spin(180)),
            Pose(0.9, (x_back, y - side, zk), -rel, "sideways to the right, clear of the bundle's end", rot=spin(180)),
        ]
        if k == 0:
            left += [
                Pose(0.9, (x_back, y - side, z_high), FINGER_OPEN, "up", rot=spin(180)),
                Pose(0.8, (x_back, y - side, z_high), FINGER_OPEN, "turn the wrist back", rot=spin(90)),
                Pose(0.8, (x_back, y - side, z_high), FINGER_OPEN, "turn the wrist back", rot=spin(0)),
                Pose(0.9, (x_back, y - side, height[1]), FINGER_OPEN, "down, level with the handles", rot=spin(0)),
            ]
    left += [Pose(1.2, (x_back, y - side, top + 0.10), FINGER_OPEN, "straight up, well clear of the roll", rot=spin(180)),
             Pose(0.9, (x_back, y - side, top + 0.10), FINGER_OPEN, "turn the wrist: fingers side by side", rot=spin(90)), ready_pose("left", 2.5)]
    total = clock()
    # The right gripper: closed, leaning 55 degrees forward (which is what lets it reach this far across), fingertips
    # down on the bundle 7 cm in from the table's edge, a little to the left of its middle so it cannot rock back.
    lean = math.radians(55.0)
    a, c = np.array([math.sin(lean), 0.0, -math.cos(lean)]), np.array([0.0, -1.0, 0.0])
    hold_rot = np.column_stack([c, np.cross(a, c), a])
    at = lambda z: tuple(np.array([edge + 0.07, marks["y"] + HOLD_LEFT_OF_HANDLES, top + z]) - PAD_TIP_PAST_TCP * a)
    hover, press = 0.055, 0.015
    right = [
        ready_pose("right", marks["roll starts"] - 2.0, "wait"),
        Pose(2.0, at(hover), FINGER_CLOSED, "over the napkin, above where the bundle will come to rest", rot=hold_rot),
        Pose(marks["roll ends"] - marks["roll starts"], at(hover), FINGER_CLOSED, "wait while the bundle is rolled under it", rot=hold_rot),
        Pose(1.0, at(press), FINGER_CLOSED, "hold the bundle down with the fingertips", rot=hold_rot),
        Pose(marks["held again"] - marks["roll ends"] - 1.0, at(press), FINGER_CLOSED, "keep holding while the other gripper takes a fresh hold", rot=hold_rot),
        Pose(0.6, at(hover), FINGER_CLOSED, "lift off", rot=hold_rot),
        ready_pose("right", 2.5),
    ]
    right.append(ready_pose("right", max(0.05, total - sum(p.duration for p in right)), "wait"))
    return {"left": left, "right": right}


def demo():
    """The whole job: pull the napkin, lay the knife and fork on it, fold, turn, roll. About 104 s."""
    return _then(_then(_then(_then(pull_plan(), lay_plan()), fold_plan()), turn_plan(0.5)), roll_plan(0.5))
