"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
does any part of either hand (palm, fingers, the last two links) run into the camera, the lens cloth or a lens other
than the one it holds, how close do the two hands come, and how far is each lens turned on the mount when it goes on
and comes off. A lens is taken to move rigidly with the right hand while it is held. No simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import camera_lens as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link_6", "link_5"]
# (label it is gripped in, label it is let go in, where it is when let go) for each lens the right hand carries
HELD = {"old_lens": ("hold it", "let it go"), "new_lens": ("hold the new lens", "let the new lens go")}
PRESSING = ("press the release button", "hold the button in", "turn it off the lock",
            "let the button go, turn it to the stop")


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
        out.append((pos + rot @ np.array(s.centre), rot @ local_axes, half, s.kind == "cyl", s))
    return out


def depth_into(points, part) -> float:
    """How deep (m) the deepest of ``points`` is inside the shape (<= 0: none inside); a cylinder exactly."""
    c, axes, half, is_cyl, s = part
    local = (points - c) @ axes
    if is_cyl:
        r, h = s.size
        radial = r - np.hypot(local[:, 0], local[:, 1])
        axial = h / 2 - np.abs(local[:, 2])
        return float(np.max(np.minimum(radial, axial)))
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
    times = sorted({round(x, 2) for e in ends.values() for x in (e - 0.25, e)} | set(np.round(np.arange(0, t, 0.25), 2)))
    starts = {lab: ends[lab] - w.duration for w in d["right"] for lab in [w.label]}
    # each lens in the hand: its pose relative to the TCP where the fingers close on it
    rel = {}
    lens_at = {"old_lens": (M.MOUNT_AT, M.R_LOCKED), "new_lens": (M.NEW_AT, M.R_INSERT)}
    # in the hand, the new lens as the sim leaves it: turned by what it turns in the fingers as they close
    in_hand = {"old_lens": M.R_LOCKED, "new_lens": M.rot_z(M.HAND_TWIST) @ M.R_INSERT}
    for name, (grip_lab, _) in HELD.items():
        p, r, _, _ = plans["right"].target(ends[grip_lab] - 1e-6)
        lp, lr = lens_at[name][0], in_hand[name]
        rel[name] = (r.T @ (lp - p), r.T @ lr)
    out = {"hits": [], "min_hand_gap_mm": np.inf, "twist": {}}
    for tt in list(times) + [total]:
        qa = {a: p.action7(tt)[0] for a, p in plans.items()}
        p, r, _, lab = plans["right"].target(tt)
        lenses, held = {}, None
        for name, (grip_lab, go_lab) in HELD.items():
            if tt < starts[grip_lab]:
                lenses[name] = lens_at[name]
            elif tt < starts[go_lab]:
                lenses[name] = (p + r @ rel[name][0], r @ rel[name][1])
                held = name
            else:
                gp, gr, _, _ = plans["right"].target(starts[go_lab])
                lenses[name] = (gp + gr @ rel[name][0], gr @ rel[name][1])
                if tt <= ends[go_lab]:
                    held = name  # the fingers opening off it
        parts = {
            "camera": shape_boxes(M.CAMERA, np.array(M.CAMERA.pos), np.eye(3)),
            "button": shape_boxes(M.BUTTON, np.array(M.BUTTON.pos), np.eye(3)),
            "cloth": shape_boxes(M.LENS_CLOTH, np.array(M.LENS_CLOTH.pos), np.eye(3)),
        }
        for name, (lp, lr) in lenses.items():
            parts[name] = shape_boxes(M.OLD_LENS if name == "old_lens" else M.NEW_LENS, lp, lr)
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            pts = np.vstack([h.vertices for h in hulls])
            for name, shapes in parts.items():
                if arm == "right" and name == held:
                    continue  # its fingers are meant to touch the lens they hold
                if arm == "left" and name == "button" and lab in PRESSING:
                    continue  # and the left fingertips the button they press
                deep = max(depth_into(pts, s) for s in shapes)
                if deep > 0:
                    out["hits"].append((float(tt), arm, name, lab, round(1000 * deep, 2)))
        g = 1000 * gap(hands["left"], hands["right"])
        if g < out["min_hand_gap_mm"]:
            out["min_hand_gap_mm"], out["closest_at"] = g, (float(tt), lab)
    for lab in ("let the button go, turn it to the stop", "set it on the mount", "turn it until it clicks"):
        name = "old_lens" if lab.startswith("let the button") else "new_lens"
        _, r_end, _, _ = plans["right"].target(ends[lab] - 1e-6)
        out["twist"][lab] = M.twist_deg(r_end @ rel[name][1])  # at the end of the step, as the hand holds it
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1), "at", out.get("closest_at"))
        print("lens turned from the index at the end of (deg):", {k: round(v, 1) for k, v in out["twist"].items()})
    return out


if __name__ == "__main__":
    run()
