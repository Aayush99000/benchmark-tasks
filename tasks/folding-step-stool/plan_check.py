"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
with the stool where the plan expects it (``folding_step_stool.PLAN``: the top's turn, each leg's opening), does any
part of either hand (palm, fingers, the last two links) run into the stool other than where it is meant to (the lifting
hand on the grip rib it pinches, the pushing hand on the foot rail it pushes), and how close do the two hands come.
No simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import folding_step_stool as M  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link_6", "link_5"]


def hulls_at(q, base, links):
    qd = {n: float(v) for n, v in zip(ARM_JOINTS, q[:6])}
    finger = float(q[6]) * M.OPEN if len(q) > 6 else 0.0  # the plan's 7th value is the grip fraction, 1 = open
    qd.update(joint7=finger, joint8=finger)
    return [h for _, h in world_hulls(fk_link_poses_world(qd, base), links)]


def shape_boxes(spec, pos, rot, only=None):
    """Each collision shape of a body as an oriented box (centre, axes, half sizes); cylinders as their bounding box."""
    out = []
    for i, s in enumerate(spec.shapes):
        if i in spec.visual_only or (only is not None and i not in only):
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


def depth_into_box(points, box) -> float:
    """How deep (m) the deepest of ``points`` is inside the oriented box (<= 0: none inside)."""
    c, axes, half = box
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


def state_at(d, t: float):
    """The stool's expected (stage, angle, leg_a, leg_b) at time ``t``, eased as the arms are."""
    prev = (1, 0.0, 0.0, 0.0)
    for w, st in zip(d["left"], M.PLAN):
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (st[0],) + tuple(p + s * (q - p) for p, q in zip(prev[1:], st[1:]))
        t -= w.duration
        prev = st
    return prev


LIFTING = {"left": ("left grips", "lift the +y end", "right over the +y foot", "right down beside it", "right to the rail",
                    "push the +y leg out", "click 2", "right back", "let the +y end down", "left lets go"),
           "right": ("right grips", "lift the -y end", "left over the -y foot", "left down beside it", "left to the rail",
                     "push the -y leg out", "click", "left back", "let the -y end down", "right lets go")}
# the push is aimed past the leg's stop, so the hand leaves the rail it pressed on as it backs off
PUSHING = {"left": ("left to the rail", "push the -y leg out", "click", "left back"),
           "right": ("right to the rail", "push the +y leg out", "click 2", "right back")}


def run(verbose=True) -> dict:
    d = M.demo()
    plans = {a: ArmPlan(d[a], station.BASES[a]) for a in ("left", "right")}
    total = sum(w.duration for w in d["left"])
    out = {"hits": [], "min_hand_gap_mm": np.inf, "allowed": []}
    for tt in np.round(np.arange(0.0, total, 0.1), 2):
        qa = {a: p.action7(tt)[0] for a, p in plans.items()}
        label = plans["left"].target(tt)[3]
        stage, angle, open_a, open_b = state_at(d, tt)
        tp, tr = M.top_pose(angle, stage)
        parts = {"top": shape_boxes(M.STOOL_TOP, tp, tr)}
        for leg, opening in (("leg_a", open_a), ("leg_b", open_b)):
            lp, lr = M.leg_pose(angle, stage, leg, opening)
            parts[leg] = shape_boxes(M.LEGS[leg], lp, lr)
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            pts = np.vstack([h.vertices for h in hulls])
            for name, boxes in parts.items():
                deep = max(depth_into_box(pts, b) for b in boxes)
                if deep <= 0:
                    continue
                lifting = label.startswith(LIFTING[arm]) and name == "top"
                pushed = "leg_a" if arm == "left" else "leg_b"
                pushing = label.startswith(PUSHING[arm]) and name == pushed
                rec = (float(tt), arm, name, label, round(1000 * deep, 2))
                (out["allowed"] if (lifting or pushing) else out["hits"]).append(rec)
        if round(tt * 10) % 5 == 0:  # the hands' gap (slow) every 0.5 s
            out["min_hand_gap_mm"] = min(out["min_hand_gap_mm"], 1000 * gap(hands["left"], hands["right"]))
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        worst = {}
        for t, arm, name, label, mm in out["allowed"]:
            worst[(arm, name, label)] = max(worst.get((arm, name, label), 0.0), mm)
        for k, v in worst.items():
            print("meant to touch", k, v, "mm")
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1))
    return out


if __name__ == "__main__":
    run()
