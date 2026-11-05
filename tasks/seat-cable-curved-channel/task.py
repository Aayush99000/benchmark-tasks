"""On the station table: a power cord (8 mm diameter, 50 cm, black PVC) comes out of the far (+x) face of a grey
device box fixed at the left rear of the table and lies straight across the table, angled 50 deg towards the
right arm. In front of the box is a white J-shaped cable channel (raceway) fixed to the table: 16 mm wide inside,
12 mm deep, 4 mm walls, 31 cm long along its centreline. Its entrance is 6 cm beyond the box face and 3.5 cm to
the right of the cord's exit; it runs 8 cm straight away from the robot, turns right through a quarter circle of
8 cm radius, then runs 10 cm straight to the right. The cord must be pressed into the channel along its whole
length, starting from the box end.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "Press the power cord into the white channel along its whole length, starting at the box."
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- cord and device box -----------------------------------------------------------------------------------
R = 0.004  # cord radius (8 mm cord)
CORD_LEN = 0.50
BOX = Box("device_box", pos=(0.13, 0.20, TOP + 0.02), size=(0.06, 0.06, 0.04), color=(0.45, 0.47, 0.50))
CORD_START = (0.165, 0.20, TOP + R + 0.0005)
CORD_YAW = math.radians(-50)
CORD = cable("cord", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=True,
             color=(0.06, 0.06, 0.06), bend_stiffness=0.003, bend_damping=0.0005)

# -- channel ------------------------------------------------------------------------------------------------
GAP = 0.016  # inner width (16 mm mini trunking for an 8 mm cord)
WALL_T = 0.004
WALL_H = 0.012
P0 = np.array([0.22, 0.165])  # channel entrance (centreline): where the cord, curving out of the box, meets it
STRAIGHT1 = 0.08  # away from the robot (+x)
ARC_R = 0.08  # quarter turn to the right (towards -y)
STRAIGHT2 = 0.10  # to the right (-y)
CH_LEN = STRAIGHT1 + math.pi / 2 * ARC_R + STRAIGHT2
N_SECTIONS = 8


def path_point(u: float) -> tuple[np.ndarray, float]:
    """Channel centreline (x, y) and heading (rad) at distance ``u`` from the entrance."""
    u = min(max(u, 0.0), CH_LEN)
    if u <= STRAIGHT1:
        return P0 + np.array([u, 0.0]), 0.0
    u2 = u - STRAIGHT1
    arc = math.pi / 2 * ARC_R
    centre = P0 + np.array([STRAIGHT1, -ARC_R])
    if u2 <= arc:
        th = u2 / ARC_R  # 0 -> pi/2
        return centre + ARC_R * np.array([math.sin(th), math.cos(th)]), -th
    u3 = u2 - arc
    return centre + np.array([ARC_R, -u3]), -math.pi / 2


def _walls() -> list[Box]:
    walls = []
    step = 0.02
    n = math.ceil(CH_LEN / step)
    for i in range(n):
        u0, u1 = i * CH_LEN / n, (i + 1) * CH_LEN / n
        (a, _), (b, _) = path_point(u0), path_point(u1)
        mid, d = (a + b) / 2, b - a
        yaw = math.atan2(d[1], d[0])
        normal = np.array([-math.sin(yaw), math.cos(yaw)])
        for side, sgn in (("l", 1), ("r", -1)):
            off = sgn * (GAP / 2 + WALL_T / 2)
            # outer wall of the turn gets a little extra length so the segments close up
            length = float(np.linalg.norm(d)) + (0.004 if (side == "l") else 0.001)
            c = mid + off * normal
            walls.append(Box(f"wall_{side}{i:02d}", pos=(float(c[0]), float(c[1]), TOP + WALL_H / 2),
                             size=(length, WALL_T, WALL_H), yaw=yaw, color=(0.93, 0.93, 0.92)))
    return walls


WALLS = _walls()
OBJECTS = [BOX, CORD, *WALLS]

# -- scoring --------------------------------------------------------------------------------------------------
SEATED_Z = TOP + WALL_H - R + 0.002  # cord centre at most this high: its top is no more than 2 mm above the lip
SAMPLES = np.linspace(0.005, CH_LEN - 0.005, 8 * N_SECTIONS)


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _seated_points(objects) -> np.ndarray:
    """Per check point (SAMPLES): True if the cord lies in the channel there, below the lip."""
    pts = _dense(objects["cord"].polyline())
    ok = np.zeros(len(SAMPLES), dtype=bool)
    for i, u in enumerate(SAMPLES):
        p, _ = path_point(u)
        d = np.linalg.norm(pts[:, :2] - p, axis=1)
        near = d < GAP / 2
        ok[i] = bool(near.any() and pts[near, 2].min() <= SEATED_Z)
    return ok


def seated_sections(objects) -> np.ndarray:
    """Per channel section: True if the cord lies in the channel, below the lip, all along that section."""
    return _seated_points(objects).reshape(N_SECTIONS, -1).all(axis=1)


def success(objects) -> bool:
    return bool(seated_sections(objects).all())


def score(objects) -> float:
    """1/8 per seated section (1.0 = success)."""
    return float(seated_sections(objects).mean())


def distance(objects) -> float:
    """Channel length (m) not yet seated."""
    return float((~seated_sections(objects)).mean() * CH_LEN)


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------


def _cord_at(objects, a: float) -> tuple[np.ndarray, float]:
    """Live cord point at arclength ``a`` from the box, and the cord's heading there."""
    c = objects["cord"]
    poly = c.polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def _grip_yaw(heading: float) -> float:
    """Grip yaw that closes the fingers across a cord running along ``heading``, kept within +-pi/2."""
    y = (heading + math.pi / 2) % math.pi - math.pi / 2
    return y


def _cord_arclength(objects) -> tuple[np.ndarray, np.ndarray]:
    """Dense live cord polyline and the arclength of each of its points from the box end."""
    pts = _dense(objects["cord"].polyline(), 0.002)
    return pts, np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def _seated_arclength(objects, u: float) -> float | None:
    """Arclength of the cord point seated at channel distance ``u`` (None if the cord isn't seated there)."""
    pts, s = _cord_arclength(objects)
    p, _ = path_point(u)
    d = np.linalg.norm(pts[:, :2] - p, axis=1)
    ok = (d < GAP / 2) & (pts[:, 2] <= SEATED_Z)
    if not ok.any():
        return None
    return float(s[np.where(ok)[0][d[ok].argmin()]])


def _near_yaw(yaw: float, ref: float) -> float:
    """``yaw`` + k*pi closest to ``ref`` (a straight-down grip is symmetric under a half turn)."""
    return yaw + math.pi * round((ref - yaw) / math.pi)


# Step between placement points along the channel (m): shorter round the bend.
DEMO_STEP, DEMO_STEP_ARC = 0.03, 0.025
DEMO_ENTRY_A = 0.068  # cord length from the box to the channel entrance along its natural curve (seg 2 cm + arc)
GRASP_OPEN = -0.014  # finger joint: pads 28 mm apart, enough round an 8 mm cord, clear of nearby walls
PAD_TIP = 0.013  # finger pad tips below the TCP
_CL = np.array([path_point(u)[0] for u in np.linspace(0.0, CH_LEN, 160)])


def _channel_clearance(xy: np.ndarray) -> float:
    """Horizontal distance (m) from points ``xy`` (k,2) to the channel's footprint (walls included)."""
    d = np.linalg.norm(xy[:, None, :] - _CL[None], axis=2).min(axis=1)
    return float(d.min() - (GAP / 2 + WALL_T))


def _pad_points(p: np.ndarray, heading: float, opening: float) -> np.ndarray:
    """Outline points of both finger pads (15 x 4 mm) around a cord point ``p`` running along ``heading``."""
    t = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-t[1], t[0]])
    out = []
    for side in (-1, 1):
        c = p[:2] + side * n * (abs(opening) + 0.002)
        out += [c + a * t * 0.0075 + b * n * 0.002 for a in (-1, 0, 1) for b in (-1, 1)]
    return np.array(out)


def _pick_grasp(objects, a: float) -> tuple[float, np.ndarray, float]:
    """A grasp arclength near ``a`` where the open fingers won't land on the channel walls."""
    best = None
    for da in (0.0, 0.008, 0.016, 0.024, 0.032, 0.04, 0.05, -0.008):
        p, h = _cord_at(objects, a + da)
        if p[2] + 0.010 - PAD_TIP > TOP + WALL_H + 0.002:
            return a + da, p, h  # cord up in the air: fingertips pass above the walls
        c = _channel_clearance(_pad_points(p, h, GRASP_OPEN))
        if c > 0.002:
            return a + da, p, h
        if best is None or c > best[0]:
            best = (c, a + da, p, h)
    return best[1], best[2], best[3]


def demo_script(ctx):
    """Right arm takes the cord near its free end and holds it up past the channel exit (slack, off the
    channel). Left arm seats the cord point by point from the box end: grasp the cord at the arclength that
    belongs at the next point (measured from the last seated point), lower it onto the slot, let go, and press it
    to the floor with the closed fingertips there and halfway back to the previous point.
    CHANNEL_CHEAT=nothing|taut|drag runs a cheat instead (each must score 0)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    over = 0.010  # TCP above the cord centre when grasping it
    hi = TOP + 0.06
    carry = TOP + 0.035
    press_z = TOP + 0.009 + PAD_TIP  # closed fingertips 9 mm above the table: 1 mm into a seated cord, 3 mm below the lip
    cheat = os.environ.get("CHANNEL_CHEAT", "")
    dbg = bool(os.environ.get("DEMO_DEBUG"))

    def log(tag):
        if dbg:
            st = ctx.states()
            print(f"[demo] {tag}: seated={seated_sections(st).astype(int).tolist()} score={score(st):.3f}", flush=True)

    if cheat == "nothing":
        yield from L.wait(3.0)
        log("end")
        return

    # 1) Right arm: take the cord 3 cm from its free end, hold it up beyond the channel exit.
    st = ctx.states()
    p, h = _cord_at(st, CORD_LEN - 0.03)
    gy = _grip_yaw(h)
    exit_pt, _ = path_point(CH_LEN)
    hold = np.array([exit_pt[0] + 0.02, exit_pt[1] - 0.05, TOP + 0.07])
    yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.2)
    yield from Rt.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((p[0], p[1], hi), yaw=gy, duration=0.6)

    if cheat == "taut":
        # pull the cord taut over the channel from the box towards the exit and lay it down there: it rests on
        # the wall tops where it crosses the channel and is never pressed in
        yield from Rt.move((0.43, -0.07, TOP + 0.03), yaw=0.0, duration=1.5)
        yield from Rt.move((0.43, -0.07, TOP + R + over), yaw=0.0, duration=0.8)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((0.43, -0.07, hi), yaw=0.0, duration=0.6)
        yield from L.wait(2.0)
        log("end")
        return

    if cheat == "drag":
        # drag the end through the channel region: pull it from the box side round to the exit, never pressing
        for q in ((0.33, 0.20), (0.40, 0.12), (0.39, -0.06)):
            yield from Rt.move((q[0], q[1], TOP + 0.012), yaw=0.0, duration=1.2)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((0.39, -0.06, hi), yaw=0.0, duration=0.6)
        yield from L.wait(2.0)
        log("end")
        return

    yield from Rt.move(tuple(hold), yaw=0.0, duration=1.0)
    log("end held")

    def press(u, yaw_ref):
        q, th = path_point(u)
        y = _near_yaw(th, yaw_ref)  # fingers close across the slot: the closed pair is 8 mm wide there
        yield from L.move((q[0], q[1], carry), yaw=y, finger=FINGER_CLOSED, duration=0.5)
        yield from L.move((q[0], q[1], press_z), yaw=y, duration=0.4)
        yield from L.move((q[0], q[1], carry), yaw=y, duration=0.3)

    u_prev, a_prev = 0.0, DEMO_ENTRY_A
    for _ in range(14):
        if u_prev >= CH_LEN - 0.012:
            break
        on_arc = STRAIGHT1 - 0.01 < u_prev < STRAIGHT1 + math.pi / 2 * ARC_R
        u = min(u_prev + (DEMO_STEP_ARC if on_arc else DEMO_STEP), CH_LEN - 0.008)
        st = ctx.states()
        a_seat = _seated_arclength(st, u_prev) if u_prev > 0 else None
        a_prev = a_seat if a_seat is not None else a_prev
        a_want = a_prev + (u - u_prev) + 0.002
        a, p, h = _pick_grasp(st, a_want)
        u_put = min(u + (a - a_want), CH_LEN - 0.004)
        tgt, th = path_point(u_put)
        gy = _near_yaw(h, th)  # keep the cord's direction: no half turn between grasp and place
        ty = _near_yaw(th, gy)
        yield from L.move((p[0], p[1], hi), yaw=gy, finger=GRASP_OPEN, duration=0.9)
        yield from L.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.6)
        yield from L.grip(FINGER_CLOSED, 0.4)
        yield from L.move((p[0], p[1], carry), yaw=gy, duration=0.4)
        yield from L.move((tgt[0], tgt[1], carry), yaw=ty, duration=0.6)
        yield from L.move((tgt[0], tgt[1], TOP + WALL_H + R + over + 0.001), yaw=ty, duration=0.4)
        yield from L.grip(-0.008, 0.3)  # let go: pads 16 mm apart, the cord drops onto/into the slot
        yield from L.move((tgt[0], tgt[1], carry), yaw=ty, duration=0.3)
        yield from press(u_put, ty)  # then press it down there and every <= 2 cm back to the last point
        k = max(1, math.ceil((u_put - u_prev) / 0.02))
        for uu in np.linspace(u_put, u_prev, k + 1)[1:-1][::-1] if k > 1 else [(u_prev + u_put) / 2]:
            yield from press(float(uu), ty)
        if dbg:
            st = ctx.states()
            pts, s = _cord_arclength(st)
            d = np.linalg.norm(pts[:, :2] - tgt, axis=1)
            j = int(d.argmin())
            print(f"DBG u={u:.3f} put={u_put:.3f} a={a:.3f} (want {a_want:.3f}) at {np.round(p, 3)} "
                  f"h={math.degrees(h):.0f} -> cord a={s[j]:.3f} lat={d[j]*1000:.1f}mm z={(pts[j, 2]-TOP)*1000:.1f}mm "
                  f"seated={seated_sections(st).astype(int).tolist()}", flush=True)
        u_prev = max(u_prev, u_put)
    # touch-up: press wherever a check point is still not seated (closed loop, a few presses at most)
    for _ in range(6):
        bad = [u for u, ok in zip(SAMPLES, _seated_points(ctx.states())) if not ok]
        if not bad:
            break
        yield from press(bad[len(bad) // 2], L._yaw)
    yield from L.move((0.25, 0.25, hi), yaw=0.0, finger=FINGER_OPEN, duration=0.8)
    log("pressed")
    # 3) Right arm lays the spare end down on the table beyond the exit and lets go.
    yield from Rt.move((hold[0], hold[1], TOP + R + over), yaw=0.0, duration=0.8)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((hold[0], hold[1], hi), yaw=0.0, duration=0.6)
    log("end")
