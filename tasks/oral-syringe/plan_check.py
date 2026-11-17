"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
does any part of either hand (palm, fingers, the last two links) run into the manikin, the tray or the syringe other
than where it holds or pushes it, how close do the two hands come, and is the tip in the cheek while the plunger is
pushed. The syringe is taken to move rigidly with the left hand once gripped. No simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import oral_syringe as M  # noqa: E402
from fittingkit import Frame  # noqa: E402
from yam_arena import station  # noqa: E402
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


def run(verbose=True) -> dict:
    d = M.demo()
    plans = {a: ArmPlan(d[a], station.BASES[a]) for a in ("left", "right")}
    ends, t = [], 0.0
    for w in d["right"]:
        t += w.duration
        ends.append((round(t, 2), w.label))
    times = sorted({round(x, 2) for e, _ in ends for x in (e - 0.25, e)} | set(np.round(np.arange(0, t, 0.25), 2)))
    grip_t = next(e for e, lab in ends if lab == "down to it")  # from when the fingers start closing on the barrel
    out = {"hits": [], "min_hand_gap_mm": np.inf}
    for tt in times:
        qa = {a: p.action7(tt)[0] for a, p in plans.items()}
        tgt = {a: p.target(tt) for a, p in plans.items()}
        held = tt >= grip_t
        if held:
            pl, rl = tgt["left"][0], tgt["left"][1]
            sy_pos, sy_rot = pl + rl @ M._P_TS, rl @ M._R_TS
        else:
            sy_pos, sy_rot = M.SYRINGE_AT, np.eye(3)
        parts = {
            "manikin": shape_boxes(M.MANIKIN, np.array(M.MANIKIN.pos), np.eye(3)),
            "tray": shape_boxes(M.TRAY, np.array(M.TRAY.pos), np.eye(3)),
            "syringe": shape_boxes(M.SYRINGE, sy_pos, sy_rot),
        }
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            pts = np.vstack([h.vertices for h in hulls])
            for name, boxes in parts.items():
                # where a hand holds a part, its fingers are meant to touch it
                if held and arm == "left" and name == "syringe":
                    continue
                deep = max(depth_into_box(pts, b) for b in boxes)
                if deep > 0:
                    out["hits"].append((float(tt), arm, name, round(1000 * deep, 2)))
        out["min_hand_gap_mm"] = min(out["min_hand_gap_mm"], 1000 * gap(hands["left"], hands["right"]))
        lab = tgt["right"][3]
        if lab.startswith("squirt"):
            s = Frame(pos=sy_pos, rot=sy_rot, vel=np.zeros(3), spin=np.zeros(3), com=sy_pos)
            out.setdefault("squirt_in_cheek", []).append(M.tip_in_cheek(s))
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1), " tip in the cheek while pushing:",
              all(out.get("squirt_in_cheek", [False])))
    return out


if __name__ == "__main__":
    run()
