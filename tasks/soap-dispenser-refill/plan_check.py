"""Offline check of the scripted demo against the YAM's real collision hulls (URDF forward kinematics): along the plan,
does any part of either hand (palm, fingers, the last two links) run into the dispenser, the wall, the refill case, the
recycling box, the cover or a refill it isn't holding; does a carried refill run into anything; and how close do the
two hands come. A refill is taken to move rigidly with the hand holding it, and the cover to follow the plan's swing.
The full refill is placed where the plan means it to be: the plan aims the hand 11 mm high (SAG) and rolled 5.7 degrees
(CARRY_R) against the arm's give under its weight, so here that aim is taken off again. Held rigidly at the aimed pose,
the roll alone would swing its collar 13 mm off the groove's line; in the simulator, pushed along the groove, it ran
within 0.2 mm of it (runs soap27, soap28). No simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import soap_dispenser_refill as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link5", "link4", "link3"]  # the YAM URDF's names (link_5 and link_6 matched nothing)
# (arm, label it is gripped in, label it is let go in) for each refill
HELD = {"old_refill": ("right", "hold the empty refill", "drop the empty refill in the bin"),
        "new_refill": ("right", "hold the new refill", "let the new refill go")}
PRESSING = ("finger to the latch", "push the release latch", "hold the latch in", "draw it off its seat",
            "let the latch go")
SWING = "swing the cover up to"
LIFTING = ("onto the cover", "push it shut", "hold it shut", "hand back from it")
SAGGED = ("lift it out of its case", "in front of the pump house", "pump to the groove", "push it into the pump house",
          "hold it home")  # the right hand aimed SAG high for the full refill's weight
ROLLED = SAGGED[1:]  # and rolled by CARRY_R
PRESS_CONTACT_MM = 2.5  # the most the pressing hand may meet the pump house's face or the refill above it
SEATED = ("hold the empty refill", "finger to the latch", "push the release latch", "hold the latch in",
          "draw it off its seat", "push it into the pump house", "hold it home", "let the new refill go")


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


def box_corners(part) -> np.ndarray:
    c, axes, half, _, _ = part
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    # the corners and the face middles, so a part that overlaps a box edge-on is found too
    pts = signs * half
    faces = np.vstack([np.eye(3) * half, -np.eye(3) * half])
    return c + np.vstack([pts, faces]) @ axes.T


def gap(a, b) -> float:
    best = np.inf
    for ha in a:
        for hb in b:
            s1 = (ha.vertices @ hb.planes[:, :3].T - hb.planes[:, 3]).max(axis=1).min()
            s2 = (hb.vertices @ ha.planes[:, :3].T - ha.planes[:, 3]).max(axis=1).min()
            best = min(best, s1, s2)
    return float(best)


def cover_deg_at(label: str) -> float:
    if label.startswith(SWING):
        return float(label.split()[-2])
    return 0.0


def intended(lab, tt, p, r, starts, ends):
    """The hand's pose with the plan's aim against the full refill's weight taken off: SAG up and CARRY_R's roll, each
    phased in over the step that first brings it in."""
    s = min(1.0, max(0.0, (tt - starts[lab]) / (ends[lab] - starts[lab])))
    s_sag = s if lab == SAGGED[0] else 1.0
    s_roll = 0.0 if lab not in ROLLED else (s if lab == ROLLED[0] else 1.0)
    roll = np.arcsin(0.10)  # CARRY_R's angle about x
    assert np.allclose(M.CARRY_R, M.axis_angle_matrix(M.X_AXIS, roll)) and not np.any(M.CARRY_DP)
    return p - np.array([0.0, 0.0, s_sag * M.SAG]), M.axis_angle_matrix(M.X_AXIS, -s_roll * roll) @ r


def run(verbose=True) -> dict:
    d = M.demo()
    plans = {a: ArmPlan(d[a], station.BASES[a]) for a in ("left", "right")}
    ends, starts, t = {}, {}, 0.0
    order = []
    for w in d["right"]:
        starts[w.label] = t
        t += w.duration
        ends[w.label] = round(t, 2)
        order.append(w.label)
    total = t
    times = sorted({round(x, 2) for e in ends.values() for x in (e - 0.3, e)} | set(np.round(np.arange(0, t, 0.3), 2)))
    seat = (np.array([M.XS, 0.0, M.SEAT_Z]), np.eye(3))
    start_at = {"old_refill": seat, "new_refill": (M.NEW_AT.copy(), np.eye(3))}
    lying = M.rot_y(np.pi / 2)
    in_box = (np.array([M.BOX_AT[0], M.BOX_AT[1], M.TOP + 0.05]) - lying @ np.array(M.OLD_REFILL.com), lying)
    rel = {}
    for name, (arm, grip_lab, _) in HELD.items():
        p, r, _, _ = plans[arm].target(ends[grip_lab] - 1e-6)
        # as held: pushed against the open finger, HELD_Y from the TCP
        grip_up = M.GRIP_UP if name == "old_refill" else M.GRIP_UP_NEW
        lp = p + np.array([M.BOTTLE_X / 2 - M.GRIP_IN, M.HELD_Y, -grip_up])
        rel[name] = (r.T @ (lp - p), r.T @ np.eye(3))
    out = {"hits": [], "carried_hits": [], "press_contacts": [], "min_hand_gap_mm": np.inf}
    fixed = {
        "dispenser": shape_boxes(M.DISPENSER, np.array(M.DISPENSER.pos), np.eye(3)),
        "wall": shape_boxes(M.WALL, np.array(M.WALL.pos), np.eye(3)),
        "case": shape_boxes(M.CASE, np.array(M.CASE.pos), np.eye(3)),
        "box": shape_boxes(M.RECYCLING, np.array(M.RECYCLING.pos), np.eye(3)),
        "latch": shape_boxes(M.LATCH, np.array(M.LATCH.pos), np.eye(3)),
    }
    cover_deg = M.OPEN_DEG
    for tt in list(times) + [total]:
        qa = {a: p.action7(tt)[0] for a, p in plans.items()}
        tcp = {a: plans[a].target(tt) for a in plans}
        lab = tcp["right"][3]
        if lab.startswith(SWING):
            # the cover where the hand has it: between this step's start and end angle
            i = order.index(lab)
            a0 = M.OPEN_DEG if i == 0 or not order[i - 1].startswith(SWING) else cover_deg_at(order[i - 1])
            b0 = cover_deg_at(lab)
            s = (tt - starts[lab]) / (ends[lab] - starts[lab])
            cover_deg = a0 + (b0 - a0) * min(1.0, max(0.0, s))
        elif lab in ("push it shut", "hold it shut", "hand back from it", "hands clear", "wait", "done"):
            cover_deg = 0.0
        refills, held = {}, {}
        for name, (arm, grip_lab, go_lab) in HELD.items():
            p, r = tcp[arm][0], tcp[arm][1]
            if tt < starts[grip_lab]:
                refills[name] = start_at[name]
            elif tt < starts[grip_lab] + M.GRIP_S:
                refills[name] = start_at[name]
                held[name] = arm  # the fingers closing on it
            elif tt < starts[go_lab]:
                if name == "new_refill" and lab in SAGGED:
                    p, r = intended(lab, tt, p, r, starts, ends)
                refills[name] = (p + r @ rel[name][0], r @ rel[name][1])
                held[name] = arm
            else:
                gp, gr, _, _ = plans[arm].target(starts[go_lab])
                refills[name] = (gp + gr @ rel[name][0], gr @ rel[name][1])
                if name == "new_refill":
                    refills[name] = seat  # pushed past the click, the groove's back stops it on its seat
                elif tt > ends[go_lab]:
                    refills[name] = in_box  # dropped: it falls into the box and lies there on its side
                if tt <= ends[go_lab] + 1.5:
                    held[name] = arm  # the fingers opening off it, and drawing back along it
        parts = dict(fixed)
        parts["cover"] = shape_boxes(M.COVER, M.HINGE_AT, M.rot_y(-np.radians(cover_deg)))
        for name, (lp, lr) in refills.items():
            parts[name] = shape_boxes(M.OLD_REFILL if name == "old_refill" else M.NEW_REFILL, lp, lr)
        hands = {a: hulls_at(qa[a], station.BASES[a], ARM) for a in plans}
        for arm, hulls in hands.items():
            pts = np.vstack([h.vertices for h in hulls])
            for name, shapes in parts.items():
                if held.get(name) == arm:
                    continue  # its fingers are meant to touch the refill they hold
                if arm == "left" and name == "latch" and lab in PRESSING:
                    continue  # and the left fingertips the latch they push
                if arm == "left" and name == "cover" and (lab.startswith(SWING) or lab in LIFTING):
                    continue  # and the cover they swing
                deep = max(depth_into(pts, s) for s in shapes)
                if deep > 0 and arm == "left" and lab in PRESSING and name in ("dispenser", "old_refill"):
                    # the left fingers pushing the latch to its stop, 0.5 mm behind the pump house's face: the leading
                    # corner of the hand, turned 70 degrees, meets the face round the latch's well and the bottom of the
                    # refill drawn out over it; reported apart, and held under PRESS_CONTACT_MM by the tests
                    out["press_contacts"].append((float(tt), arm, name, lab, round(1000 * deep, 2)))
                elif deep > 0:
                    out["hits"].append((float(tt), arm, name, lab, round(1000 * deep, 2)))
        for h in held:
            pts = np.vstack([box_corners(s) for s in parts[h]])
            for name, shapes in parts.items():
                if name == h or name == "latch":
                    continue
                if name in ("dispenser", "wall") and lab in SEATED:
                    continue  # seated in the groove, on its floor (or aimed past the click)
                deep = max(depth_into(pts, s) for s in shapes)
                if deep > 0.5 * M.MM:
                    out["carried_hits"].append((float(tt), h, name, lab, round(1000 * deep, 2)))
        g = 1000 * gap(hands["left"], hands["right"])
        if g < out["min_hand_gap_mm"]:
            out["min_hand_gap_mm"], out["closest_at"] = g, (float(tt), lab)
    if verbose:
        for h in out["hits"]:
            print("hit", h)
        for h in out["carried_hits"]:
            print("carried refill hits", h)
        for h in out["press_contacts"]:
            print("pressing the latch, touches", h)
        print("closest the hands come (mm):", round(out["min_hand_gap_mm"], 1), "at", out.get("closest_at"))
    return out


if __name__ == "__main__":
    run()
