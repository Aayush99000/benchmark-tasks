"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
does any part of either arm (palm, fingers, the last three links) run into the nailer, the pusher or the strip other
than where it is meant to touch them, does the strip the left hand carries run into the nailer, and how close do the
two hands come. The strip is taken to move rigidly with the left hand while held, to be pushed by the fingertips
through the loading window, and to be fed by the pusher; the pusher to move with the right hand while held. No
simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nailer_strip as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link5", "link4", "link3"]
LEFT_ON_STRIP = ("round it", "hold it", "lift it", "turn it", "front to the driver, heads to the heads' wall",
                 "behind the magazine", "over the rear end", "in line with the track", "feed it into the magazine",
                 "let the strip go", "down behind it", "push it in past the nail stop",
                 "up out of the window")  # the last: the fingertips leaving the strip's back, aimed 3 mm past it
RIGHT_ON_KNOB = ("up out of the window", "left hand away; hold the knob", "pull the pusher back past the strip",
                 "hold it back", "ease it forward against the strip", "hold it there", "let the knob go")


def hulls_at(q, base, links):
    qd = {n: float(v) for n, v in zip(ARM_JOINTS, q[:6])}
    finger = float(q[6]) * M.OPEN if len(q) > 6 else 0.0  # the plan's 7th value is the grip fraction, 1 = open
    qd.update(joint7=finger, joint8=finger)
    return [h for _, h in world_hulls(fk_link_poses_world(qd, base), links)]


def shape_boxes(spec, pos, rot):
    """Each collision shape of a body as an oriented box (centre, axes, half sizes)."""
    out = []
    for i, s in enumerate(spec.shapes):
        if i in spec.visual_only:
            continue
        if s.kind == "box":
            half = np.array(s.size) / 2
            local_axes = s.rotation()
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
    return float(np.max(np.min(half - np.abs(local), axis=1)))


def box_corners(part) -> np.ndarray:
    c, axes, half = part
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    return c + (signs * half) @ axes.T


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
    for w in d["left"]:
        t += w.duration
        ends[w.label] = round(t, 3)
    total = t
    starts = {w.label: ends[w.label] - w.duration for w in d["left"]}
    times = sorted(set(np.round(np.arange(0, total, 0.25), 2)) | {round(e - 0.05, 2) for e in ends.values()})
    # the strip in the left hand, relative to its TCP, from where the fingers close on it
    back2 = -(M.FEED_SHORT + M.FINGER_HALF + M.GRIP_FROM_BACK)
    p, r, _, _ = plans["left"].target(ends["hold it"] - 1e-6)
    rel1 = (r.T @ (M.STRIP_AT - p), r.T @ M.R_STRIP_START)
    out = {"hits": [], "strip_hits": [], "min_hand_gap_mm": np.inf}
    nailer = shape_boxes(M.NAILER, np.array(M.NAILER.pos), np.eye(3))
    for tt in times:
        qa = {a: pl.action7(tt)[0] for a, pl in plans.items()}
        lp, lr, _, llab = plans["left"].target(tt)
        rp, rr, _, rlab = plans["right"].target(tt)
        # where the strip is
        if tt < ends["hold it"]:
            strip = (M.STRIP_AT, M.R_STRIP_START)
        elif tt < starts["let the strip go"]:
            strip = (lp + lr @ rel1[0], lr @ rel1[1])
        else:
            back = back2
            if tt >= starts["push it in past the nail stop"]:
                tip_a = M.R_MAG.T @ (lp - M.MAG_AT)
                back = max(back, min(tip_a[0] + M.TIP_T / 2, M.PUSH_TO))
            strip = (M.mag(back + M.STRIP_L / 2, 0.0, M.REST_N), M.R_STRIP_IN)
        # where the pusher is: held, it moves with the right TCP along the rail
        if starts["left hand away; hold the knob"] <= tt < starts["let the knob go"] + 0.5:
            fa = M.Y_REAR + M.KNOB_BACK - rp[1]
        else:
            fa = M.A_DRIVE if tt < starts["left hand away; hold the knob"] else M.A_DRIVE - M.STRIP_L
        fa = min(max(fa, M.FINGER_MIN), M.A_DRIVE)
        pusher_at = np.array([M.PUSHER_X, M.knob_y(fa), M.PUSHER_Z])
        parts = {"nailer": nailer, "offcut": shape_boxes(M.OFFCUT, np.array(M.OFFCUT.pos), np.eye(3)),
                 "pusher": shape_boxes(M.PUSHER, pusher_at, np.eye(3)),
                 "strip": shape_boxes(M.STRIP, strip[0], strip[1])}
        if ends["round it"] <= tt < ends["let the strip go"]:
            # the strip stops the left fingers at its width (offline they would close right through it)
            qa["left"] = np.array(qa["left"], float)
            qa["left"][6] = max(qa["left"][6], (M.NAIL_L / 2) / -M.OPEN)
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            lab = llab if arm == "left" else rlab
            pts = np.vstack([h.vertices for h in hulls])
            for name, shapes in parts.items():
                if arm == "left" and name == "strip" and lab in LEFT_ON_STRIP:
                    continue
                if arm == "right" and name == "pusher" and lab in RIGHT_ON_KNOB:
                    continue
                deep = max(depth_into(pts, s) for s in shapes)
                if deep > 0:
                    out["hits"].append((float(tt), arm, name, lab, round(1000 * deep, 2)))
        # the carried strip against the nailer (corner points of its shapes); none once it is in the track
        if ends["hold it"] <= tt < starts["feed it into the magazine"]:
            corners = np.vstack([box_corners(s) for s in parts["strip"]])
            deep = max(depth_into(corners, s) for s in nailer + parts["offcut"])
            if tt < ends["hold it"] + 0.5:
                deep = max(depth_into(corners, s) for s in nailer)  # it starts on the offcut
            if deep > 0:
                out["strip_hits"].append((float(tt), llab, round(1000 * deep, 2)))
        if np.linalg.norm(lp - rp) > 0.25:  # too far apart to matter; the hull gap is slow
            continue
        g = 1000 * gap(hands["left"], hands["right"])
        if g < out["min_hand_gap_mm"]:
            out["min_hand_gap_mm"], out["closest_at"] = g, (float(tt), llab, rlab)
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        for h in out["strip_hits"]:
            print("strip hits the nailer", h)
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1), "at", out.get("closest_at"))
    return out


if __name__ == "__main__":
    run()
