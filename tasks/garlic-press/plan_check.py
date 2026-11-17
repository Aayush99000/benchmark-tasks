"""Offline check of the scripted demo against the YAM's real gripper collision hulls (URDF forward kinematics), no
simulator: how close each hand comes to the press (its lower half, and the top handle where it stands at that point
of the demo), and how close the two arms come to each other.

    python plan_check.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import garlic_press as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link_6", "link_5", "link_4"]
DT = 0.2


class Hull:
    """A convex shape as its vertices and outward planes n . x = d."""

    def __init__(self, vertices: np.ndarray, planes: np.ndarray):
        self.vertices, self.planes = vertices, planes


def box_hull(shape, rot: np.ndarray, origin: np.ndarray) -> Hull:
    """A fittingkit box shape of a body at ``origin`` turned ``rot``, as a hull in the world."""
    c, s = math.cos(shape.yaw), math.sin(shape.yaw)
    r = rot @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    h = np.array(shape.size) / 2
    centre = origin + rot @ np.array(shape.centre)
    verts = np.array([centre + r @ (h * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    planes = []
    for i in range(3):
        for sign in (-1, 1):
            n = sign * r[:, i]
            planes.append(np.r_[n, n @ centre + h[i]])
    return Hull(verts, np.array(planes))


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def press_hulls(theta: float) -> tuple[list, list]:
    """The lower half's and the top handle's collision boxes, the handle open ``theta``."""
    lower = [box_hull(s, M.R_PRESS, M.PRESS_POS) for i, s in enumerate(M.PRESS.shapes)
             if s.kind == "box" and i not in M.PRESS.visual_only]
    hinge = M.PRESS_POS + np.array([0.0, 0.0, M.HINGE_Z])
    upper = [box_hull(s, M.R_PRESS @ rot_x(-theta), hinge) for i, s in enumerate(M.PRESS_TOP.shapes)
             if s.kind == "box" and i not in M.PRESS_TOP.visual_only]
    return lower, upper


def hulls_at(q, finger: float, base, links):
    qd = {n: float(v) for n, v in zip(ARM_JOINTS, q[:6])}
    qd.update(joint7=finger, joint8=finger)
    return [h for _, h in world_hulls(fk_link_poses_world(qd, base), links)]


FAR = 0.05  # hull pairs whose bounding boxes are farther apart than this are reported as this far


def gap(a, b) -> float:
    """Approximate separation (m) of two sets of convex hulls: the least signed distance of any vertex of one to a
    hull of the other (negative: they overlap). Capped at FAR: pairs whose bounding boxes are that far apart are
    skipped, which is what keeps the check to minutes."""
    best = FAR
    boxes_b = [(hb.vertices.min(axis=0), hb.vertices.max(axis=0)) for hb in b]
    for ha in a:
        lo_a, hi_a = ha.vertices.min(axis=0), ha.vertices.max(axis=0)
        for hb, (lo_b, hi_b) in zip(b, boxes_b):
            if np.linalg.norm(np.maximum(0.0, np.maximum(lo_a - hi_b, lo_b - hi_a))) >= best:
                continue
            s1 = (ha.vertices @ hb.planes[:, :3].T - hb.planes[:, 3]).max(axis=1).min()
            s2 = (hb.vertices @ ha.planes[:, :3].T - ha.planes[:, 3]).max(axis=1).min()
            best = min(best, s1, s2)
    return float(best)


def handle_angle(label_right: str) -> float:
    """Where the top handle stands at this point of the demo: open until the right hand tips it, then down on the
    clove (pushed shut later by the left hand, which is meant to touch it then)."""
    done = ("back and clear", "out of the way", "wait", "done")
    return M.THETA_FLAT_CONTACT if label_right in done else M.OPEN


def run(verbose: bool = True) -> dict:
    plans = {a: ArmPlan(M.demo()[a], station.BASES[a]) for a in ("left", "right")}
    times = np.arange(0.0, max(p.duration for p in plans.values()) + 1e-9, DT)
    out = {"worst_ik_mm": 0.0, "press": {"left": [], "right": []}, "arms_min_mm": np.inf}
    pressed = False
    for t in times:
        hands = {}
        labels = {}
        for a, p in plans.items():
            q, _ = p.action7(t)
            _, _, finger, label = p.target(t)
            labels[a] = label
            hands[a] = (hulls_at(q, finger, station.BASES[a], HAND), hulls_at(q, finger, station.BASES[a], ARM))
        pressed = pressed or labels["left"] == "press it down"
        theta = 0.0 if pressed else handle_angle(labels["right"])  # shut once the left hand has pressed it
        lower, upper = press_hulls(theta)
        for a in plans:
            # the left hand pressing the handle down is meant to touch it, and its aim below the shut handle (which
            # stops it) reaches down to the lower handle; the right hand tipping the handle is meant to touch it
            pressing = a == "left" and labels[a] in ("onto it", "press it down", "hold", "let go")
            touching = pressing or (a == "right" and labels[a] in ("tip it back over",))
            g_lower = 1.0 if pressing else gap(hands[a][0], lower)
            g_upper = None if touching else gap(hands[a][0], upper)
            out["press"][a].append((round(float(t), 2), labels[a], round(1000 * g_lower, 1),
                                    None if g_upper is None else round(1000 * g_upper, 1)))
        out["arms_min_mm"] = min(out["arms_min_mm"], 1000 * gap(hands["left"][1], hands["right"][1]))
    out["worst_ik_mm"] = max(p.worst_ik_mm for p in plans.values())
    for a in plans:
        rows = out["press"][a]
        out[f"{a}_lower_min_mm"] = min(r[2] for r in rows)
        ups = [r[3] for r in rows if r[3] is not None]
        out[f"{a}_upper_min_mm"] = min(ups)
    if verbose:
        for a in plans:
            worst = sorted(out["press"][a], key=lambda r: min(r[2], r[3] if r[3] is not None else 1e9))[:6]
            print(a, "closest to the press (t, label, lower mm, handle mm):", worst)
        print("worst IK (mm):", round(out["worst_ik_mm"], 2), " arms closest (mm):", round(out["arms_min_mm"], 1))
    return out


if __name__ == "__main__":
    run()
