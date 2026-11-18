"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
does any part of either hand (palm, fingers, the last two links) run into the flowmeter, its knob, the packet or the
tubing (other than where it is meant to hold them), how close do the two hands come, and where is the tubing's mouth
as it goes onto the nipple. The tubing is taken to move rigidly with the right hand while it is held, and to stay
seated where it was pushed once it is let go. No simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import oxygen_flowmeter as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link_6", "link_5"]
GRIP, LET_GO = "pinch it", "let it go"  # the right hand closes on the tubing, and opens off it
# the labels in which a hand is meant to touch a part: the right fingers round the tubing, the left round the knob
RIGHT_TOUCH = ("fingers either side of it", "pinch it", "lift it", "to the nipple", "in line with it", "onto its tip",
               "push it on", "hold it on", "let it go", "back off the nipple")
LEFT_TOUCH = ("fingers round the knob", "close on it", "settle the grip", "turn it to 2 L/min", "hold it there", "let go of the knob",
              "back off")


def hulls_at(q, base, links):
    qd = {n: float(v) for n, v in zip(ARM_JOINTS, q[:6])}
    finger = float(q[6]) * M.OPEN if len(q) > 6 else 0.0  # the plan's 7th value is the grip fraction, 1 = open
    qd.update(joint7=finger, joint8=finger)
    return [h for _, h in world_hulls(fk_link_poses_world(qd, base), links)]


def shape_boxes(spec, pos, rot):
    """Each collision shape of a body as an oriented box (centre, axes, half sizes); cylinders as their bounding box."""
    out = []
    for i, s in enumerate(spec.shapes):
        if i in spec.visual_only:
            continue
        if s.kind == "box":
            half = np.array(s.size) / 2
            c, sn = np.cos(s.yaw), np.sin(s.yaw)
            local_axes = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
        else:
            r, h = s.size
            half = {"X": np.array([h / 2, r, r]), "Y": np.array([r, h / 2, r]), "Z": np.array([r, r, h / 2])}[s.axis]
            local_axes = np.eye(3)
        out.append((pos + rot @ np.array(s.centre), rot @ local_axes, half))
    return out


def depth_into(points, part) -> float:
    """How deep (m) the deepest of ``points`` is inside the box (<= 0: none inside)."""
    c, axes, half = part
    local = (points - c) @ axes
    inside = half - np.abs(local)
    return float(np.max(np.min(inside, axis=1)))


def gap(a, b) -> float:
    best = np.inf
    for ha in a:
        for hb in b:
            s1 = (ha.vertices @ hb.planes[:, :3].T - hb.planes[:, 3]).max(axis=1).min()
            s2 = (hb.vertices @ ha.planes[:, :3].T - ha.planes[:, 3]).max(axis=1).min()
            best = min(best, s1, s2)
    return float(best)


def run(verbose=True) -> dict:
    d = M.demo()
    plans = {a: ArmPlan(d[a], station.BASES[a]) for a in ("left", "right")}
    ends, t = {}, 0.0
    for w in d["right"]:
        t += w.duration
        ends[w.label] = round(t, 2)
    total = t
    starts = {w.label: ends[w.label] - w.duration for w in d["right"]}
    times = sorted({round(x, 2) for e in ends.values() for x in (e - 0.25, e)} | set(np.round(np.arange(0, t, 0.25), 2)))
    seated = M.seated_pose(M.FULL_ON, M.R_TUBE_START)
    seated = (seated[0], M.fittingkit.quat_to_matrix(seated[1]))
    out = {"hits": [], "min_hand_gap_mm": np.inf}
    fixed = {
        "flowmeter": shape_boxes(M.FLOWMETER, np.array(M.FLOWMETER.pos), np.eye(3)),
        "knob": shape_boxes(M.KNOB, M.KNOB_AT, M.R_KNOB),
        "packet": shape_boxes(M.PACKET_BODY, np.array(M.PACKET_BODY.pos), np.eye(3)),
    }
    for tt in list(times) + [total]:
        qa = {a: p.action7(tt)[0] for a, p in plans.items()}
        p, r, _, lab = plans["right"].target(tt)
        if tt < starts[GRIP]:
            tube = (M.TUBE_AT, M.R_TUBE_START)
        elif tt < starts[LET_GO]:
            tube = M.tube_held(p, r)
        else:
            tube = seated
        parts = dict(fixed, tubing=shape_boxes(M.TUBE, *tube))
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            pts = np.vstack([h.vertices for h in hulls])
            for name, shapes in parts.items():
                if arm == "right" and name == "tubing" and lab in RIGHT_TOUCH:
                    continue
                if arm == "left" and name == "knob" and lab in LEFT_TOUCH:
                    continue
                deep = max(depth_into(pts, s) for s in shapes)
                if deep > 0:
                    out["hits"].append((float(tt), arm, name, lab, round(1000 * deep, 2)))
        g = 1000 * gap(hands["left"], hands["right"])
        if g < out["min_hand_gap_mm"]:
            out["min_hand_gap_mm"], out["closest_at"] = g, (float(tt), lab)
    # the tubing's mouth as the right hand brings it onto the nipple's tip and pushes it on
    out["mouth"] = {}
    for label in ("in line with it", "onto its tip", "push it on"):
        p, r, _, _ = plans["right"].target(ends[label] - 1e-6)
        depth, radial, angle = M.on_nipple(*M.tube_held(p, r))
        out["mouth"][label] = (round(1000 * depth, 2), round(1000 * radial, 3), round(angle, 3))
    # the knob's turn: the left hand's roll from closing on it to holding it turned
    _, r0, _, _ = plans["left"].target(ends["close on it"] - 1e-6)
    _, r1, _, _ = plans["left"].target(ends["hold it there"] - 1e-6)
    out["turn_deg"] = float(np.degrees(M.knob_angle(r1 @ r0.T @ M.R_KNOB)))
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1), "at", out.get("closest_at"))
        print("tubing mouth (on mm, off the axis mm, angle deg):", out["mouth"])
        print("knob turned (deg):", round(out["turn_deg"], 2), "->", round(M.flow_lpm(np.radians(out["turn_deg"])), 3),
              "L/min")
    return out


if __name__ == "__main__":
    run()
