"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
does any part of either hand (palm, fingers, the last two links) run into the bowl, the seat or the knob (other than
where it is meant to touch them), and how close do the two hands come. The seat is taken to stay where it was set on
until the left hand has pushed it back, and to be fitted from then on. No simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import raised_toilet_seat as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link_6", "link_5"]
PUSHED = "push it back and straight"  # from the end of this label the seat is fitted


def right_on_knob(label: str) -> bool:
    return label == "onto the knob" or label.rstrip("0123456789 ").endswith(
        ("close on the knob", "turn it clockwise", "let it go", "turn back"))


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


def stack(parts):
    """A body's boxes as arrays (centres, axes, half sizes), to test many at once."""
    return (np.array([c for c, _, _ in parts]), np.array([a for _, a, _ in parts]), np.array([h for _, _, h in parts]))


def deepest(points, stacked) -> float:
    """How deep (m) the deepest of ``points`` is inside any of the stacked boxes (<= 0: none inside)."""
    c, axes, half = stacked
    lo, hi = points.min(axis=0), points.max(axis=0)
    reach = np.linalg.norm(half, axis=1)
    keep = np.all((c + reach[:, None] > lo) & (c - reach[:, None] < hi), axis=1)  # boxes near the points only
    if not keep.any():
        return -1.0
    c, axes, half = c[keep], axes[keep], half[keep]
    local = np.einsum("pj,bjk->bpk", points, axes) - np.einsum("bj,bjk->bk", c, axes)[:, None, :]
    inside = half[:, None, :] - np.abs(local)
    return float(inside.min(axis=2).max())


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
    times = sorted({round(x, 2) for e in ends.values() for x in (e - 0.2, e)} | set(np.round(np.arange(0, t, 0.2), 2)))
    bowl = stack(shape_boxes(M.BOWL_BODY, np.array(M.BOWL_BODY.pos), np.eye(3)))
    out = {"hits": [], "min_hand_gap_mm": np.inf, "left_into_seat_mm": 0.0}
    for tt in list(times) + [total]:
        qa = {a: p.action7(tt)[0] for a, p in plans.items()}
        _, _, _, lab = plans["right"].target(tt)
        if tt < ends[PUSHED]:
            seat_pos, seat_rot = M.SEAT_START, M.R_SEAT_START
        else:
            seat_pos, seat_rot = M.FIT_AT, np.eye(3)
        seat = stack(shape_boxes(M.SEAT_BODY, seat_pos, seat_rot))
        knob = stack(shape_boxes(M.KNOB_BODY, seat_pos + seat_rot @ M.KNOB_LOCAL, seat_rot @ M.R_KNOB_IN_SEAT))
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            pts = np.vstack([h.vertices for h in hulls])
            parts = {"bowl": bowl, "seat": seat, "knob": knob}
            for name, shapes in parts.items():
                deep = deepest(pts, shapes)
                if deep <= 0:
                    continue
                pressing = starts[PUSHED] <= tt < ends["hands clear"] - 1.0
                if arm == "left" and name == "seat" and pressing:
                    out["left_into_seat_mm"] = max(out["left_into_seat_mm"], 1000 * deep)
                    if deep < M.PUSH_PAST + 1.0e-3:
                        continue
                if arm == "right" and name == "knob" and right_on_knob(lab):
                    continue
                out["hits"].append((float(tt), arm, name, lab, round(1000 * deep, 2)))
        g = 1000 * gap(hands["left"], hands["right"])
        if g < out["min_hand_gap_mm"]:
            out["min_hand_gap_mm"], out["closest_at"] = g, (float(tt), lab)
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1), "at", out.get("closest_at"))
        print("the left fingers aimed into the seat's face (mm, at most):", round(out["left_into_seat_mm"], 1))
    return out


if __name__ == "__main__":
    run()
