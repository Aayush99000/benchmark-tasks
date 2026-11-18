"""On the station table: the forearm and hand of an adult training arm, lying palm down on a padded arm board with
its hand past the board's end, and a cordless handheld barcode scanner lying on its side near the right arm.

The training arm wears a patient identification band on its wrist. The band has turned, as bands do, so its label
(with the Data Matrix symbol the scanner reads) is on the palm side of the wrist, face down on the board: no scanner
can see it there. The forearm turns about its long axis, like a patient's, and is limp: a torsion spring at its
elbow pivot rolls it back palm down as soon as it is let go. So one hand has to take the patient's hand, turn the
forearm until the label faces out, and keep holding it there while the other hand picks up the scanner by its handle
(the grip pulls its trigger), points it at the label from reading distance and holds it steady until it reads. The
scanner reads by its published figures: range, field of view, tilt, motion, a clear line of sight, and a decode
session that times out 9.9 s after the trigger is pulled. Then the scanner goes back on the table and the patient's
arm is laid back down, not dropped.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable BAND_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bandkit as K  # noqa: E402
from bandkit import Arm, Fixed, Scanner, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20

INSTRUCTION = ("Scan the barcode on the training arm's wristband with the handheld scanner. Then put the scanner down "
               "on the table and lay the arm back palm down.")

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("BAND_LOT", "A")
LOTS = {"A": dict(axis_x=0.440, tips_y=0.020, scanner=(0.270, -0.400)),
        "B": dict(axis_x=0.455, tips_y=0.005, scanner=(0.255, -0.385))}
if LOT not in LOTS:
    raise ValueError(f"BAND_LOT must be one of {sorted(LOTS)}, got {LOT!r}")

# -- the training arm and its board -----------------------------------------------------------------------------------
AXIS_H = 0.108                          # the forearm's axis above the table
ARM_MASS = 1.2                          # forearm, wrist and hand (kg)
# The limp forearm: a torsion spring at the elbow pivot returns it palm down. Holding it a quarter turn over takes
# ROLL_BACK (N m), which is 1.3 N at the edge of the hand, 4 cm from the axis.
ROLL_BACK = 0.055
SPRING = ROLL_BACK / (math.pi / 2)      # N m/rad
ELBOW = np.array([LOTS[LOT]["axis_x"], LOTS[LOT]["tips_y"] - K.TIP_Y, TOP + AXIS_H])   # the axis's elbow end
BOARD_GAP = 0.003                       # between the forearm and the board's top
BOARD_HALF_W = 0.045
BOARD_Y = (ELBOW[1] + 0.004, ELBOW[1] + K.BAND_Y + 0.020)       # from the elbow to just past the band
BOARD_TOP = AXIS_H - K.FOREARM_R - BOARD_GAP

# -- the scanner: lying on its left side, reading toward +x, its head toward the middle of the table ------------------
SCANNER_MASS = 0.214
LYING = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))             # xyzw: its y axis down, its z (handle) axis along +y
REST_Z = -K.HEAD[1][0]                  # its origin's height when it lies on the side of its head
SCANNER_POS = np.array([LOTS[LOT]["scanner"][0], LOTS[LOT]["scanner"][1], TOP + REST_Z + 0.0005])

# -- the scanner's reading rule, from its published figures (see the README) ------------------------------------------
RANGE = (0.008, 0.157)                  # m from the scan window to the symbol: a 10 mil Data Matrix
FOV = (math.radians(32.8) / 2, math.radians(24.8) / 2)          # the imager's half field of view: horizontal, vertical
TILT = math.radians(65.0)               # skew and pitch tolerance
MOTION = 0.13                           # m/s of the symbol across the scanner's view
SESSION_S = 9.9                         # a pull of the trigger reads for this long, then needs a new pull
PULLED = -0.003                         # trigger travel (m) at which its switch closes
DWELL_S = 0.25                          # the symbol must stay readable this long for a decode
LIGHT_S = 2.0                           # the good-decode light stays lit this long

# -- how the trial is judged -------------------------------------------------------------------------------------------
FAST = math.radians(200.0)              # the forearm turned faster than this at any time: dropped or jerked
SHOWN = math.radians(60.0)              # the forearm turned this far ...
SHOWN_S = 1.0                           # ... and kept there this long: the label has been brought out
BACK = math.radians(10.0)               # the forearm within this of palm down: laid back
BACK_RATE = math.radians(10.0)          # ... and turning no faster than this
CLEAR = 0.030                           # a gripper's grasp point this far from a thing: it has let go of it
ON_TABLE = 0.005                        # the scanner's lowest point within this of the table: it lies on the table
STILL = (0.02, 0.2)                     # ... moving no faster than this (m/s, rad/s)


def board() -> Fixed:
    y0, y1 = BOARD_Y
    return Fixed("arm_board", pos=(float(ELBOW[0]), (y0 + y1) / 2, TOP + BOARD_TOP / 2),
                 size=(2 * BOARD_HALF_W, y1 - y0, BOARD_TOP), color=(0.16, 0.30, 0.52), roughness=0.7,
                 static_friction=0.6, dynamic_friction=0.5)


def board_box():
    y0, y1 = BOARD_Y
    return (np.array([ELBOW[0], (y0 + y1) / 2, TOP + BOARD_TOP / 2]), np.eye(3),
            np.array([BOARD_HALF_W, (y1 - y0) / 2, BOARD_TOP / 2]))


OBJECTS = [
    board(),
    Arm("training_arm", pos=tuple(float(v) for v in ELBOW), size=(0.116, K.TIP_Y, 0.116), mass=ARM_MASS,
        height=AXIS_H, spring=SPRING),
    Scanner("scanner", pos=tuple(float(v) for v in SCANNER_POS), quat=LYING, size=(0.090, 0.175, 0.066),
            mass=SCANNER_MASS),
]


# -- the reading rule ------------------------------------------------------------------------------------------------
def readable(scan_pos, scan_quat, turn: float, grips=()) -> tuple:
    """Whether the scanner, where it is, could read the band's symbol with the forearm turned ``turn``: every point of
    the symbol (its centre and corners) inside the imager's field of view and reading range, tilted no more than the
    scanner tolerates, with nothing between it and the scan window. Returns (bool, what failed, numbers)."""
    window, aim, across, up = K.scanner_optics(scan_pos, scan_quat)
    pts, normals = K.symbol_points(ELBOW, turn)
    blockers = [board_box(), *K.hand_boxes(ELBOW, turn)]
    for g in grips:
        blockers += K.gripper_boxes(g["tcp"], g["rot"], g["gap"])
    worst = {"range_cm": None, "off_axis_deg": 0.0, "tilt_deg": 0.0}
    failed = []
    for p, n in zip(pts, normals):
        ray = p - window
        dist = float(np.linalg.norm(ray))
        along = float(ray @ aim)
        if worst["range_cm"] is None:
            worst["range_cm"] = round(100 * dist, 2)                      # the symbol's centre
        if not RANGE[0] <= dist <= RANGE[1]:
            failed.append("range")
        h = math.atan2(abs(float(ray @ across)), along) if along > 0 else math.pi
        v = math.atan2(abs(float(ray @ up)), along) if along > 0 else math.pi
        worst["off_axis_deg"] = max(worst["off_axis_deg"], round(math.degrees(max(h, v)), 1))
        if h > FOV[0] or v > FOV[1]:
            failed.append("field of view")
        tilt = math.acos(max(-1.0, min(1.0, float(n @ -ray) / max(dist, 1e-9))))
        worst["tilt_deg"] = max(worst["tilt_deg"], round(math.degrees(tilt), 1))
        if tilt > TILT:
            failed.append("tilt")
        seen_from = p + 0.0005 * n                                        # just off the label's surface
        if any(K.segment_hits_box(window, seen_from, b) for b in blockers):
            failed.append("line of sight")
    return not failed, sorted(set(failed)), worst


def clear_of(boxes, grips, margin: float = CLEAR) -> bool:
    """Whether every gripper's grasp point is at least ``margin`` from every one of ``boxes``."""
    for g in grips:
        for centre, rot, half in boxes:
            local = rot.T @ (np.asarray(g["tcp"], float) - centre)
            if float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0))) < margin:
                return False
    return True


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(step=0, pulled=False, pulls=0, session_from=None, session_live=False, dwell=0, read=False,
                  read_step=None, timed_out=0, last_view=None, shown_steps=0, shown=False, fast=False, turn=0.0,
                  turn_max=0.0, rate_max=0.0, last_turn=None, laid=False, down=False, why=[], view={},
                  trigger_min=0.0, lit=False)


_clear()


def advance(s: dict, turn: float, rate: float, trigger: float, scan_pos, scan_quat, scan_low, scan_vel, grips,
            dt: float = 1.0 / CONTROL_HZ) -> None:
    """One control step of the trial's record. ``turn`` (rad) and ``rate`` (rad/s) are the forearm's, ``trigger`` the
    trigger's travel (m, negative pulled), ``scan_*`` the scanner's pose, lowest point and speeds, ``grips`` the two
    grippers (see :func:`bandkit.grippers`)."""
    s["step"] += 1
    # the forearm: how far and how fast it has been turned
    if s["last_turn"] is not None:
        rate = max(abs(rate), abs(turn - s["last_turn"]) / dt)
    s["last_turn"] = turn
    s["turn"], s["turn_max"], s["rate_max"] = turn, max(s["turn_max"], turn), max(s["rate_max"], abs(rate))
    if abs(rate) > FAST:
        s["fast"] = True
    s["shown_steps"] = s["shown_steps"] + 1 if turn >= SHOWN else 0
    if s["shown_steps"] * dt >= SHOWN_S:
        s["shown"] = True

    # the scanner: a pull of the trigger opens a decode session; a decode needs the symbol readable, and steady in
    # the scanner's view, for DWELL_S
    s["trigger_min"] = min(s["trigger_min"], trigger)
    pulled = trigger <= PULLED
    if pulled and not s["pulled"]:
        s["pulls"] += 1
        s["session_from"] = s["step"]
    s["pulled"] = pulled
    live = pulled and s["session_from"] is not None and (s["step"] - s["session_from"]) * dt <= SESSION_S
    if pulled and not live and s["session_live"]:
        s["timed_out"] += 1
    s["session_live"] = live
    ok, why, view = readable(scan_pos, scan_quat, turn, grips)
    window = K.scanner_optics(scan_pos, scan_quat)[0]
    centre = K.symbol_points(ELBOW, turn)[0][0]
    rel = K._rotation(scan_quat).T @ (centre - window)                    # the symbol in the scanner's own view
    speed = 0.0 if s["last_view"] is None else float(np.linalg.norm(rel - s["last_view"])) / dt
    s["last_view"] = rel
    view["speed_cm_s"] = round(100 * speed, 1)
    if speed > MOTION:
        ok, why = False, why + ["motion"]
    s["why"], s["view"] = why, view
    s["dwell"] = s["dwell"] + 1 if (live and ok and not s["read"]) else 0
    if s["dwell"] * dt >= DWELL_S:
        s["read"], s["read_step"] = True, s["step"]
    s["lit"] = s["read"] and (s["step"] - s["read_step"]) * dt <= LIGHT_S

    # afterwards: the scanner lying on the table, the forearm laid back palm down, both let go and at rest
    lying = scan_low[2] <= TOP + ON_TABLE and float(np.linalg.norm(scan_vel[:3])) <= STILL[0] and \
        float(np.linalg.norm(scan_vel[3:])) <= STILL[1]
    s["down"] = bool(lying and clear_of(K.scanner_boxes(scan_pos, scan_quat), grips))
    s["laid"] = bool(turn <= BACK and abs(rate) <= BACK_RATE and clear_of(K.hand_boxes(ELBOW, turn), grips))


def grade(s: dict) -> float:
    """1.0: the scanner read the band, the forearm was never turned faster than 200 deg/s (dropped or jerked), and
    afterwards the scanner lies on the table and the forearm is back within 10 deg of palm down, both let go and at
    rest. 0.75: read and never turned too fast, but not everything is back. 0.5: read, but the forearm was turned
    too fast at some time. 0.25: no read, but the forearm was turned at least 60 deg and kept there for 1 s without
    ever being turned too fast. 0 otherwise."""
    if s["read"]:
        if s["fast"]:
            return 0.5
        return 1.0 if (s["down"] and s["laid"]) else 0.75
    return 0.25 if (s["shown"] and not s["fast"]) else 0.0


@on_reset
def _reset(environment) -> None:
    _clear()
    K.write_joint(environment, "training_arm", "turn", 0.0)
    K.write_joint(environment, "scanner", "trigger", 0.0)
    K.show("scanner", "housing/led_on", False)
    K.show("scanner", "housing/led_off", True)


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    st = object_states(environment, ["training_arm", "scanner"])
    arm, scan = st["training_arm"], st["scanner"]
    dt = float(getattr(environment.unwrapped, "step_dt", 1.0 / CONTROL_HZ))
    advance(_STATE, arm.joints["turn"], arm.joint_vel["turn"], scan.joints["trigger"], scan.pos, scan.quat_xyzw,
            scan.low, np.r_[scan.lin_vel, scan.ang_vel], K.grippers(environment), dt)
    K.show("scanner", "housing/led_on", _STATE["lit"])
    K.show("scanner", "housing/led_off", not _STATE["lit"])


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    return grade(_STATE)


score.__doc__ = grade.__doc__


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects=None) -> dict:
    s = _STATE
    return {"score": grade(s), "lot": LOT, "turn_deg": round(math.degrees(s["turn"]), 1),
            "turn_max_deg": round(math.degrees(s["turn_max"]), 1),
            "rate_max_deg_s": round(math.degrees(s["rate_max"]), 1), "fast": s["fast"], "shown": s["shown"],
            "trigger_min_mm": round(1000 * s["trigger_min"], 2), "pulls": s["pulls"], "session_live": s["session_live"],
            "timed_out": s["timed_out"], "why_not": s["why"], "view": s["view"], "dwell": s["dwell"], "read": s["read"],
            "read_step": s["read_step"], "scanner_down": s["down"], "arm_laid": s["laid"]}


def demo(objects=None, seed=None):
    """No waypoints: ``--policy yam_scripted`` plays straight-down picks only, so with this task it keeps both arms
    still (a scene check). The scripted demo that reaches full score is ``demo.py``."""
    return {}
