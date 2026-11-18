"""On the station table (a home testing kit laid out on a worktop): a lancing device, the pen that pricks a fingertip
for a blood-glucose test, lying with its end cap on. Beside it a kit tray holds one new twist-top lancet, and a dish
stands ready for the lancet's protective tab.

The job is what a person with diabetes does before a test with a fresh lancet: take the end cap off, push the lancet
into the holder until it is gripped, twist the lancet's protective tab off to bare the needle, put the cap back on
until it snaps, and cock the device by pulling its arming barrel back until it clicks; then lay it down.

The device weighs under 30 g and its cap snaps on with 2 N, so a pull on the cap only drags a loose device across
the table: one hand has to hold the device while the other works on it. The tab has to be twisted, not pulled: its
moulded neck stands a pull several times the holder's grip, so a pulled tab brings the lancet out with it. Once the
tab is off, the needle is bare until the cap is back on.

The device's place changes with ``LANCET_LOT`` (A or B).

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lancetkit as K  # noqa: E402
from lancetkit import Barrel, Cap, Device, Dish, Lancet, Tab, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Load the new lancet into the lancing device: take the device's end cap off, push the lancet into the "
               "holder, twist off the lancet's protective tab, put the end cap back on, then cock the device by pulling "
               "its arming barrel back until it clicks. Lay the device down on the table.")

# -- layout (m; x forward from the arms, y left, heights above the table) ------------------------------------------------
LOT = os.environ.get("LANCET_LOT", "A")
NOSE_XY = np.array({"A": (0.340, 0.050), "B": (0.325, 0.065)}[LOT])      # the tip of the device's nose
ALONG = np.array([0.0, 1.0, 0.0])        # the device lies along the table's y, its nose toward the right arm
DEVICE_QUAT = K.turn_x(-math.pi / 2)     # its own z (nose to barrel) along +y, its flat down
LANCET_QUAT = K.turn_x(math.pi / 2)      # a lancet's z (insertion end to needle) along -y: ready to go into the nose

TRAY_XY = np.array([0.240, -0.160])      # the kit tray's middle
TRAY = (0.070, 0.110, 0.020)             # x, y, height
RIDGE_X, RIDGE = -0.022, (0.0080, 0.019, 0.010)      # the lancet's ridge: its x in the tray, and its size
KERB = (0.0012, 0.0015)                  # the kerbs along the ridge that keep the lancet from rolling: width, height
KERB_PLAY = 0.0008                       # ... and the play they leave it on each side
CRADLE_X, CRADLE_GAP, CRADLE_H = 0.020, 0.0108, 0.0025   # the cap's cradle: its x in the tray, between its rails, their height
LANCET_END_Y = 0.012                     # the lancet's insertion end, this far up the tray from its middle
DISH_XY = np.array([0.440, -0.010])
DISH_IN, DISH_H = 0.070, 0.020

DEVICE, BARREL, CAP, LANCET, TAB, DISH = "device", "barrel", "cap", "lancet", "tab", "dish"

# -- scoring -----------------------------------------------------------------------------------------------------------
TAB_CLEAR = 0.010                        # the tab has been taken clear once it is this far from the lancet's front
NEEDLE_NEAR = 0.002                      # a gripper this close to the bare needle has touched it
LEVEL, DOWN = math.radians(5.0), 0.002   # laid down: the device within 5 degrees of level, its underside within 2 mm
STILL = 0.02                             # m/s
CLEAR = 0.030                            # both grippers this far from the device at the end
FELL = -0.05

TRAY_GREY, DISH_WHITE = (0.55, 0.57, 0.60), (0.93, 0.93, 0.91)


def device_start() -> np.ndarray:
    """The tip of the device's nose at the start, the device lying on its flat."""
    return np.array([NOSE_XY[0], NOSE_XY[1], TOP + K.BODY_R + 0.0003])


def lancet_start() -> np.ndarray:
    """The lancet's insertion end at the start, the lancet lying along the tray's ridge."""
    return np.array([TRAY_XY[0] + RIDGE_X, TRAY_XY[1] + LANCET_END_Y, TOP + TRAY[2] + RIDGE[2] + K.LANCET_R + 0.0003])


def cradle_axis_z() -> float:
    """How high a cap's axis is when it lies in the tray's cradle."""
    inner = CRADLE_GAP / 2
    return TOP + TRAY[2] + CRADLE_H + math.sqrt(K.CAP_R ** 2 - inner ** 2)


def tilt_from_level(quat) -> float:
    """How far a part's own z is from level (rad)."""
    return abs(math.asin(max(-1.0, min(1.0, float(K._rotation(quat)[2, 2])))))


def _tray() -> list:
    """The kit tray: a block, a narrow ridge with kerbs for the lancet, and two rails for the cap."""
    x, y = float(TRAY_XY[0]), float(TRAY_XY[1])
    top = TOP + TRAY[2]
    parts = [Box("tray", pos=(x, y, TOP + TRAY[2] / 2), size=TRAY, color=TRAY_GREY)]
    rx, ry = x + RIDGE_X, y + LANCET_END_Y - 0.002 - RIDGE[1] / 2          # under the lancet's body; the tab overhangs
    parts.append(Box("ridge", pos=(rx, ry, top + RIDGE[2] / 2), size=RIDGE, color=TRAY_GREY))
    for name, s in (("kerb_a", 1.0), ("kerb_b", -1.0)):
        parts.append(Box(name, pos=(rx + s * (K.LANCET_R + KERB_PLAY + KERB[0] / 2), ry, top + RIDGE[2] + KERB[1] / 2 - 0.0002),
                         size=(KERB[0], RIDGE[1], KERB[1]), color=TRAY_GREY))
    for name, s in (("rail_a", 1.0), ("rail_b", -1.0)):
        parts.append(Box(name, pos=(x + CRADLE_X + s * (CRADLE_GAP / 2 + 0.0015), y, top + CRADLE_H / 2),
                         size=(0.003, 0.060, CRADLE_H), color=TRAY_GREY))
    return parts


def _objects() -> list:
    d0, l0 = device_start(), lancet_start()
    size = (2 * K.BODY_R, 2 * K.BODY_R, K.BODY_END)
    device = Device(DEVICE, pos=tuple(float(v) for v in d0), quat=DEVICE_QUAT, size=size, barrel=BARREL, cap=CAP)
    barrel = Barrel(BARREL, pos=tuple(float(v) for v in d0 + K.BODY_END * ALONG), quat=DEVICE_QUAT,
                    size=(2 * K.BODY_R, 2 * K.BODY_R, K.BARREL_L))
    cap_at = d0 + K.NOSE_L * ALONG
    tab_at = l0 - K.LANCET_L * ALONG
    cap = Cap(CAP, pos=tuple(float(v) for v in cap_at), quat=DEVICE_QUAT, size=(2 * K.CAP_R, 2 * K.CAP_R, K.CAP_L))
    lancet = Lancet(LANCET, pos=tuple(float(v) for v in l0), quat=LANCET_QUAT, size=(2 * K.LANCET_R, 2 * K.LANCET_R, K.LANCET_L),
                    tab=TAB)
    tab = Tab(TAB, pos=tuple(float(v) for v in tab_at), quat=LANCET_QUAT, size=(2 * K.TAB_R, 2 * K.TAB_R, K.TAB_L))
    dish = Dish(DISH, pos=(float(DISH_XY[0]), float(DISH_XY[1]), TOP + 0.0005), size=(DISH_IN, DISH_IN, DISH_H), color=DISH_WHITE)
    return [*_tray(), device, barrel, cap, lancet, tab, dish]


OBJECTS = _objects()


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, seat=False, click=True, tab_off=False, tab_clear=False, pull=0.0, pull_max=0.0, cocked=False,
                  stuck=False, fell=False, laid=True, settled=False, depth=0.0, gap=0.0, give_mm=0.0, give_deg=0.0,
                  needle_mm=None, twist=0.0)


_clear()


def advance(s: dict, seat_fit: dict, cap_fit: dict, joined: bool, twisted: float, tab_gone: float, pull: float,
            needle_gap: float) -> list:
    """One control step of the device's own behaviour (pure). ``seat_fit``, ``cap_fit``: how the lancet and the cap sit;
    ``joined``: the tab is still in place on the lancet; ``twisted``: how far it has been turned on its neck (rad);
    ``tab_gone``: how far the tab is from the lancet's front (m); ``pull``: how far the barrel is pulled back (m);
    ``needle_gap``: the nearest gripper's distance from the needle (m). Returns what the kit has to do now: ``"seat"``
    or ``"click"`` (make that snap), ``"shear"`` (let the neck go)."""
    make = []
    if not s["seat"] and K.seated(seat_fit):
        s["seat"] = True                                   # the holder grips the lancet
        make.append("seat")
    elif s["seat"] and seat_fit["depth"] < K.SOCKET_DEPTH - 3 * K.SEAT_HOME:
        s["seat"] = False                                  # pulled out of the holder
    if not s["click"] and K.cap_on(cap_fit):
        s["click"] = True                                  # the cap snaps on
        make.append("click")
    elif s["click"] and abs(cap_fit["gap"]) > 3 * K.CAP_HOME:
        s["click"] = False                                 # pulled off
    if not s["tab_off"] and (not joined or abs(twisted) >= K.NECK_TWIST):
        s["tab_off"] = True                                # pulled off, or twisted until its neck sheared
        make.append("shear")
    if s["tab_off"] and tab_gone >= TAB_CLEAR:
        s["tab_clear"] = True
    s["pull"] = pull
    s["pull_max"] = max(s["pull_max"], pull)
    if pull >= K.COCK_AT:
        s["cocked"] = True
    if s["tab_clear"] and not s["click"] and needle_gap < NEEDLE_NEAR:
        s["stuck"] = True                                  # a gripper touched the bare needle
    return make


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    st = object_states(environment, [DEVICE, BARREL, CAP, LANCET, TAB])
    dev, bar, cap, lan, tab = (st[n] for n in (DEVICE, BARREL, CAP, LANCET, TAB))
    seat = K.seat_fit(dev.pos, dev.quat_xyzw, lan.pos, lan.quat_xyzw)
    capf = K.cap_fit(dev.pos, dev.quat_xyzw, cap.pos, cap.quat_xyzw)
    shift, angle = K.apart(lan.pos, lan.quat_xyzw, (0.0, 0.0, K.LANCET_L), tab.pos, tab.quat_xyzw)
    joined = shift <= K.PART_SHIFT and angle <= K.PART_ANGLE
    if joined and not s["tab_off"]:
        s["give_mm"], s["give_deg"] = max(s["give_mm"], 1000 * shift), max(s["give_deg"], math.degrees(angle))
    base, tip = K.needle(lan.pos, lan.quat_xyzw)
    hands = [p for side in K.gripper_points(environment) for p in side]
    needle_gap = min(K.segment_distance(p, base, tip) for p in hands)
    s["depth"], s["gap"], s["needle_mm"] = seat["depth"], capf["gap"], 1000 * needle_gap
    twisted = K.twist(lan.quat_xyzw, tab.quat_xyzw)
    s["twist"] = max(s["twist"], abs(math.degrees(twisted))) if not s["tab_off"] else s["twist"]
    for name in advance(s, seat, capf, joined, twisted, shift, K.barrel_pull(dev.pos, dev.quat_xyzw, bar.pos), needle_gap):
        if name == "shear":
            K.release(environment, "neck")
        else:
            K.snap(environment, name)

    s["fell"] = s["fell"] or float(dev.pos[2]) - TOP < FELL
    s["laid"] = bool(tilt_from_level(dev.quat_xyzw) <= LEVEL and float(dev.low[2]) - TOP <= DOWN)
    nose, tail = np.asarray(dev.pos, float), K.place(dev.pos, dev.quat_xyzw, (0.0, 0.0, K.BODY_END + K.BARREL_L))
    front = K.place(dev.pos, dev.quat_xyzw, (0.0, 0.0, K.NOSE_L - K.CAP_L))
    clear = all(K.segment_distance(p, front, tail) >= K.BODY_R + CLEAR for p in hands)
    still = all(float(np.linalg.norm(st[n].lin_vel)) < STILL for n in (DEVICE, BARREL, CAP, LANCET))
    s["settled"] = bool(clear and still)


# -- scoring --------------------------------------------------------------------------------------------------------
def score_from(seat: bool, tab_off: bool, click: bool, cocked: bool, stuck: bool, fell: bool, laid: bool, settled: bool) -> float:
    """Pure scoring rule. ``seat``: the lancet is home in the holder now. ``tab_off``: its protective tab has come off.
    ``click``: the end cap is snapped on now. ``cocked``: the arming barrel has been pulled back to its click.
    ``stuck``: a gripper touched the bare needle. ``fell``: the device went off the table. ``laid``: the device lies
    level on the table now. ``settled``: everything is at rest and both grippers are clear of the device."""
    if stuck or fell or not seat:
        return 0.0
    if not tab_off:
        return 0.25
    if not click:
        return 0.5
    return 1.0 if (cocked and laid and settled) else 0.75


def score(objects) -> float:
    """0 if the lancet is not home in the holder (its insertion end within 1 mm of the socket's floor), if a gripper
    came within 2 mm of the bare needle (after the tab was taken 10 mm clear and before the cap was back on), or if the
    device went off the table. Otherwise 0.25: the lancet is home with its tab still on. 0.5: and the tab is off.
    0.75: and the end cap is snapped back on (within 1 mm of fully home). 1.0: and the device has been cocked (its
    arming barrel pulled back 6.5 mm or more), it lies on the table within 5 degrees of level and 2 mm of the surface,
    the device, cap, barrel and lancet are at rest (under 2 cm/s), and both grippers are at least 3 cm from it."""
    s = _STATE
    return score_from(s["seat"], s["tab_off"], s["click"], s["cocked"], s["stuck"], s["fell"], s["laid"], s["settled"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    s = _STATE
    return {"score": score(objects), "lot": LOT, "seated": s["seat"], "depth_mm": round(1000 * s["depth"], 2),
            "tab_off": s["tab_off"], "tab_clear": s["tab_clear"], "cap_on": s["click"], "cap_gap_mm": round(1000 * s["gap"], 2),
            "pull_mm": round(1000 * s["pull"], 2), "pull_max_mm": round(1000 * s["pull_max"], 2), "cocked": s["cocked"],
            "needle_touched": s["stuck"], "needle_gap_mm": None if s["needle_mm"] is None else round(s["needle_mm"], 1),
            "neck_give_mm": round(s["give_mm"], 2), "neck_give_deg": round(s["give_deg"], 2), "twist_deg": round(s["twist"], 1),
            "fell": s["fell"], "laid": s["laid"], "settled": s["settled"]}


def demo(objects=None, seed=None):
    return {}
