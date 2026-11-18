"""On the station table (a clinic's specimen bench): an open 16 x 100 mm transport tube stands free on its skirted
base with 3 mL of medium in it, and the swab that has just been taken stands in it, its flocked tip in the medium and
its handle sticking 6 cm out of the tube. An open waste cup stands to the right.

The swab's shaft has a moulded breakpoint 80 mm from its tip, which sits just inside the tube's rim when the tip is on
the tube's floor. The job is the last step of every swab collection: bend the handle over the rim so the shaft snaps
at the breakpoint and the tip stays in the tube, then throw the handle away. The tube and its medium weigh 10 g and
stand on a 16 mm base, so a push on the handle that would snap the shaft tips an unheld tube over long before: one
hand has to hold the tube while the other bends the handle. Lifting the swab out, or knocking the tube over, loses the
specimen.

The tube's place changes with ``SWAB_LOT`` (A or B).

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

import swabkit as K  # noqa: E402
from swabkit import Cup, SwabHandle, SwabHead, Tube, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Snap the swab's handle off at its breakpoint, leaving the swab's tip in the transport tube, and drop the "
               "handle in the waste cup. Leave the tube standing.")

# -- layout (m; x forward from the arms, y left, heights above the table) ------------------------------------------------
LOT = os.environ.get("SWAB_LOT", "A")
TUBE_XY = np.array({"A": (0.260, -0.100), "B": (0.280, -0.080)}[LOT])    # the middle of the tube's base
# The swab rests with its tip in the middle of the tube's floor and its shaft against the far side of the rim.
LEAN = math.atan((K.TUBE_ID / 2 - K.SHAFT_R - 0.0002) / (K.TUBE_H - K.TUBE_FLOOR))
SWAB_QUAT = K.turn_y(LEAN)
UP_SWAB = np.array([math.sin(LEAN), 0.0, math.cos(LEAN)])                # from its tip along it to its end
CUP_XY = np.array([0.320, -0.300])
CUP_IN, CUP_H = 0.100, 0.080             # the waste cup's inside diameter and height

TUBE, HEAD, HANDLE, CUP = "tube", "swab_head", "swab_handle", "waste_cup"

# -- scoring -----------------------------------------------------------------------------------------------------------
SPILL_TILT = 60.0                        # deg: tipped this far at any time, the open tube has lost its medium
SNAP_TILT = 45.0                         # deg: the tube no further over than this when the shaft snaps
SEAT = 0.015                             # the tip within this of the tube's floor when the shaft snaps
STAND_TILT, BASE_UP = 5.0, 0.003         # standing: this upright, its base this close to the table
STILL = 0.02                             # m/s
CLEAR = 0.030                            # both grippers this far from the tube and from the handle at the end

TUBE_PLASTIC, CUP_GREY = (0.93, 0.95, 0.96), (0.90, 0.91, 0.92)


def tip_start() -> np.ndarray:
    """The swab's tip at the start, on the tube's floor."""
    return np.array([TUBE_XY[0], TUBE_XY[1], TOP + K.TUBE_FLOOR + 0.0004])


def breakpoint_start() -> np.ndarray:
    return tip_start() + K.BREAK_AT * UP_SWAB


def tube_tilt(quat) -> float:
    """How far the tube's axis is from upright (deg)."""
    return math.degrees(math.acos(float(np.clip(K._rotation(quat)[2, 2], -1.0, 1.0))))


def tip_in_tube(tip, tube_pos, tube_quat) -> tuple:
    """Whether the swab's tip is inside the tube's bore, and how far it is above the tube's floor."""
    p = K.to_local(tip, tube_pos, tube_quat)
    above = float(p[2]) - K.TUBE_FLOOR
    inside = bool(math.hypot(p[0], p[1]) < K.TUBE_ID / 2 and -0.002 <= above <= K.TUBE_H - K.TUBE_FLOOR)
    return inside, above


def in_cup(point, cup_pos, cup_quat) -> bool:
    """``point`` is inside the upright waste cup."""
    axis = K._rotation(cup_quat)[:, 2]
    v = np.asarray(point, float) - np.asarray(cup_pos, float)
    h = float(v @ axis)
    return bool(axis[2] > math.cos(math.radians(30.0)) and 0.0 < h < CUP_H and np.linalg.norm(v - h * axis) < CUP_IN / 2)


def handle_mid(pos, quat) -> np.ndarray:
    return np.asarray(pos, float) + K._rotation(quat) @ np.array([0.0, 0.0, K.HANDLE_L / 2])


def _objects() -> list:
    tube = Tube(TUBE, pos=(float(TUBE_XY[0]), float(TUBE_XY[1]), TOP + 0.0003), size=(K.TUBE_OD, K.TUBE_OD, K.TUBE_H),
                color=TUBE_PLASTIC)
    head = SwabHead(HEAD, pos=tuple(float(v) for v in tip_start()), quat=SWAB_QUAT, size=(2 * K.TIP_R, 2 * K.TIP_R, K.BREAK_AT),
                    handle=HANDLE)
    handle = SwabHandle(HANDLE, pos=tuple(float(v) for v in breakpoint_start()), quat=SWAB_QUAT,
                        size=(2 * K.SHAFT_R, 2 * K.SHAFT_R, K.HANDLE_L))
    cup = Cup(CUP, pos=(float(CUP_XY[0]), float(CUP_XY[1]), TOP + 0.0005), size=(CUP_IN, CUP_IN, CUP_H), color=CUP_GREY)
    return [tube, head, handle, cup]


OBJECTS = _objects()


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, tilt=0.0, tilt_max=0.0, spilled=False, exposed=False, seated=True, tip_up=0.0, snapped=False,
                  valid=False, snap_step=None, snap_tilt=None, snap_tip_up=None, binned=False, standing=True,
                  settled=False, give_mm=0.0, give_deg=0.0, seated_up=0.0)


_clear()


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    st = object_states(environment, [TUBE, HEAD, HANDLE, CUP])
    tube, head, handle, cup = st[TUBE], st[HEAD], st[HANDLE], st[CUP]

    s["tilt"] = tube_tilt(tube.quat_xyzw)
    s["tilt_max"] = max(s["tilt_max"], s["tilt"])
    if s["tilt"] > SPILL_TILT:
        s["spilled"] = True
    inside, above = tip_in_tube(head.pos, tube.pos, tube.quat_xyzw)
    s["tip_up"] = above
    if not inside:
        s["exposed"] = True                       # the swab's tip has been out of the tube

    # the snap: the first step on which the two lengths are no longer one piece; it counts only if, up to the step
    # before, the tip was on the tube's floor (the breakpoint at the rim) and the tube was upright enough
    if K.joined(head.pos, head.quat_xyzw, handle.pos, handle.quat_xyzw):
        if not s["snapped"]:
            s["seated"] = bool(inside and above <= SEAT and s["tilt"] <= SNAP_TILT)
            s["seated_up"] = above
            shift, angle = K.apart(head.pos, head.quat_xyzw, handle.pos, handle.quat_xyzw)
            s["give_mm"], s["give_deg"] = max(s["give_mm"], 1000 * shift), max(s["give_deg"], math.degrees(angle))
    elif not s["snapped"]:
        s.update(snapped=True, valid=bool(s["seated"]), snap_step=s["steps"], snap_tilt=round(s["tilt"], 1),
                 snap_tip_up=round(1000 * s.get("seated_up", 0.0), 1))

    # the end state: the handle in the waste cup, the tube standing with the tip in it, all at rest, hands clear
    s["binned"] = in_cup(handle_mid(handle.pos, handle.quat_xyzw), cup.pos, cup.quat_xyzw)
    s["standing"] = bool(s["tilt"] <= STAND_TILT and float(tube.pos[2]) - TOP <= BASE_UP and inside)
    still = all(float(np.linalg.norm(st[n].lin_vel)) < STILL for n in (TUBE, HEAD, HANDLE, CUP))
    rim = np.asarray(tube.pos, float) + K._rotation(tube.quat_xyzw) @ np.array([0.0, 0.0, K.TUBE_H])
    end = np.asarray(handle.pos, float) + K._rotation(handle.quat_xyzw) @ np.array([0.0, 0.0, K.HANDLE_L])
    clear = all(K.segment_distance(p, tube.pos, rim) >= K.TUBE_OD / 2 + CLEAR
                and K.segment_distance(p, handle.pos, end) >= CLEAR
                for side in K.gripper_points(environment) for p in side)
    s["settled"] = bool(still and clear)


# -- scoring --------------------------------------------------------------------------------------------------------
def score_from(snapped: bool, valid: bool, spilled: bool, exposed: bool, binned: bool, standing: bool, settled: bool) -> float:
    """Pure scoring rule. ``snapped``: the handle has come off the rest of the swab. ``valid``: when it did, the swab's
    tip was within SEAT of the tube's floor and the tube was within SNAP_TILT of upright. ``spilled``: the tube has been
    more than SPILL_TILT from upright at some time. ``exposed``: the swab's tip has been outside the tube at some time.
    ``binned``: the handle is in the waste cup now. ``standing``: the tube stands upright on the table with the tip in
    it now. ``settled``: everything is at rest and both grippers are clear."""
    if spilled or exposed or not snapped or not valid:
        return 0.0
    if not binned:
        return 0.5
    return 1.0 if (standing and settled) else 0.75


def score(objects) -> float:
    """0 if the shaft has not been snapped; if it was snapped with the tip more than 15 mm off the tube's floor or the
    tube more than 45 deg from upright; if the tube has at any time been more than 60 deg from upright (its medium is
    lost); or if the swab's tip has at any time been outside the tube. Otherwise 0.5: the handle is off. 0.75: and the
    handle's middle is inside the upright waste cup. 1.0: and the tube stands on the table within 5 deg of upright
    with the tip in it, the tube, both lengths of the swab and the cup are at rest (under 2 cm/s), and both grippers
    are at least 3 cm from the tube and from the handle."""
    s = _STATE
    return score_from(s["snapped"], s["valid"], s["spilled"], s["exposed"], s["binned"], s["standing"], s["settled"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    s = _STATE
    return {"score": score(objects), "lot": LOT, "snapped": s["snapped"], "valid": s["valid"], "snap_step": s["snap_step"],
            "snap_tilt_deg": s["snap_tilt"], "snap_tip_up_mm": s["snap_tip_up"],
            "joint_give_mm": round(s["give_mm"], 2), "joint_give_deg": round(s["give_deg"], 2), "tilt_deg": round(s["tilt"], 1), "tilt_max_deg": round(s["tilt_max"], 1),
            "spilled": s["spilled"], "tip_exposed": s["exposed"], "tip_above_floor_mm": round(1000 * s["tip_up"], 1),
            "handle_binned": s["binned"], "tube_standing": s["standing"], "settled": s["settled"]}


def demo(objects=None, seed=None):
    return {}
