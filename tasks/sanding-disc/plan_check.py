"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
does any part of either hand (palm, fingers, the last two links) run into the sander, the stand or the disc (other than
where it is meant to touch it), how close do the two hands come, and where are the disc's holes when it goes on. The
disc is taken to move rigidly with the right hand while it is held, and to lie flat on the pad, where it was planned to
land, once it is let go. No simulator.

    python plan_check.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sanding_disc as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link_6", "link_5"]
GRIP, LET_GO = "pinch it", "let it go"  # the right hand closes on the disc, and opens off it
# the labels in which a hand is meant to touch the disc: the right fingers round its edge, the left fingertips on it
RIGHT_TOUCH = ("fingers either side of it", "pinch it", "lift it out of the stand", "tip it forward",
               "level, grit side up", "over the pad, holes to its holes", "tip it, far edge down", "far edge on the pad",
               "finger over the far edge", "hold the far edge on", "let it go", "back away")
LEFT_TOUCH = ("hold the far edge on", "let it go", "back away", "lift off the edge", "press it on", "hold it pressed")
# pressing the disc on, the left fingertips are aimed past its face (the arm presses like a spring), so up to that far
# into the pad under it
PRESSING = ("press it on", "hold it pressed", "hands clear")
PRESS_INTO_PAD = M.PRESS_PAST - M.DISC_T + 0.5e-3


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
    landed = (M.PAD_AT + np.array([0.0, 0.0, M.DISC_T / 2]), M.rot_z(M.DISC_TURN + M.TURN))
    out = {"hits": [], "min_hand_gap_mm": np.inf}
    for tt in list(times) + [total]:
        qa = {a: p.action7(tt)[0] for a, p in plans.items()}
        p, r, _, lab = plans["right"].target(tt)
        if tt < starts[GRIP]:
            disc = (M.DISC_AT, M.R_DISC_START)
        elif tt < starts[LET_GO]:
            disc = M.disc_held(p, r)
        else:
            disc = landed
        parts = {
            "sander": shape_boxes(M.SANDER, np.array(M.SANDER.pos), np.eye(3)),
            "stand": shape_boxes(M.STAND_BODY, np.array(M.STAND_BODY.pos), np.eye(3)),
            "disc": shape_boxes(M.DISC, *disc),
        }
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            pts = np.vstack([h.vertices for h in hulls])
            for name, shapes in parts.items():
                if name == "disc" and lab in (RIGHT_TOUCH if arm == "right" else LEFT_TOUCH):
                    continue
                deep = max(depth_into(pts, s) for s in shapes)
                if arm == "left" and name == "sander" and lab in PRESSING:
                    deep -= PRESS_INTO_PAD
                if deep > 0:
                    out["hits"].append((float(tt), arm, name, lab, round(1000 * deep, 2)))
        g = 1000 * gap(hands["left"], hands["right"])
        if g < out["min_hand_gap_mm"]:
            out["min_hand_gap_mm"], out["closest_at"] = g, (float(tt), lab)
    # the disc as the right hand holds it touching on: its holes from the pad's, and its far edge from the pad's face
    p, r, _, _ = plans["right"].target(ends["far edge on the pad"] - 1e-6)
    c, rot = M.disc_held(p, r)
    flat = c - M.PAD_AT
    out["touch_hole_error_mm"] = M.hole_error_mm(flat[:2], M.disc_yaw(rot))
    out["touch_tilt_deg"] = math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1), "at", out.get("closest_at"))
        print("touching on: holes off (mm)", round(out["touch_hole_error_mm"], 2), "tilt (deg)",
              round(out["touch_tilt_deg"], 1))
    return out


if __name__ == "__main__":
    run()
