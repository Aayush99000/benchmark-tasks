"""Offline check of the scripted demo against the real gripper collision hulls (URDF forward kinematics): at each press,
is the pressed key hidden from the onlooker's eye, how far apart are the two hands, how deep do the fingertips go and
do they touch a neighbouring key, and does the screening hand stay off the keys. No simulator.

    python plan_check.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import shielded_pin as M  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ["gripper", "tip_left", "tip_right"]
ARM = HAND + ["link_6", "link_5", "link_4"]


def hulls_at(q, base, links):
    qd = {n: float(v) for n, v in zip(ARM_JOINTS, q[:6])}
    qd.update(joint7=0.0, joint8=0.0)  # fingers closed
    return [h for _, h in world_hulls(fk_link_poses_world(qd, base), links)]


def gap(a, b) -> float:
    """Approximate separation (m) of two sets of convex hulls: the least signed distance of any vertex of one to a
    hull of the other (negative: they overlap)."""
    best = np.inf
    for ha in a:
        for hb in b:
            s1 = (ha.vertices @ hb.planes[:, :3].T - hb.planes[:, 3]).max(axis=1).min()
            s2 = (hb.vertices @ ha.planes[:, :3].T - ha.planes[:, 3]).max(axis=1).min()
            best = min(best, s1, s2)
    return float(best)


def run(verbose=True) -> dict:
    plans = {a: ArmPlan(M.demo()[a], station.BASES[a]) for a in ("left", "right")}
    labels = [w.label for w in plans["right"].waypoints]
    ends, t = {}, 0.0
    for w in plans["right"].waypoints:
        t += w.duration
        ends.setdefault(w.label, []).append(t)
    qs = {a: [] for a in plans}
    times = np.arange(0.0, plans["right"].duration + 1e-9, 0.05)
    for a, p in plans.items():
        for tt in times:
            qs[a].append(p.action7(tt)[0])
    out = {"worst_ik_mm": max(p.worst_ik_mm for p in plans.values()), "presses": [], "min_gap_mm": np.inf}
    check = sorted({int(round(e / 0.05)) for lab, es in ends.items() if lab.startswith(("press", "hold", "over", "let", "touch"))
                    for e in es for e in (e, e - 0.2) if 0 <= int(round(e / 0.05)) < len(times)})
    for i in check:  # the hands' gap at each press and just before it, at the end of each approach
        hl, hr = hulls_at(qs["left"][i], station.BASES["left"], ARM), hulls_at(qs["right"][i], station.BASES["right"], ARM)
        hl = [h for h in hl if h.vertices[:, 2].min() < M.KEY_TOP + 0.12]
        hr = [h for h in hr if h.vertices[:, 2].min() < M.KEY_TOP + 0.12]
        out["min_gap_mm"] = min(out["min_gap_mm"], 1000 * gap(hl, hr))
        out.setdefault("gaps", []).append((round(times[i], 2), round(1000 * gap(hl, hr), 1)))
    for name in M.EXPECTED:
        tt = ends[f"press {name}"][0]
        i = int(round(tt / 0.05))
        hl = hulls_at(qs["left"][i], station.BASES["left"], HAND)
        hr = hulls_at(qs["right"][i], station.BASES["right"], HAND)
        corners = M.key_corners(name, M.CLICK)
        each = [any(M.segment_hits_hull(M.EYE, c, h.planes) for h in hl + hr) for c in corners]
        by_shield = [any(M.segment_hits_hull(M.EYE, c, h.planes) for h in hl) for c in corners]
        rv = np.vstack([h.vertices for h in hr])
        lv = np.vstack([h.vertices for h in hl])
        # the key stops the fingertips at its travel: what of the hand is within 0.8 mm of the case top then
        low = rv[rv[:, 2] - rv[:, 2].min() + (M.KEY_TOP - M.KEY_TRAVEL) < M.CASE_TOP + 0.8 * M.MM]
        kx, ky = M.key_xy(name)
        on_key = np.all((np.abs(low[:, 0] - kx) < M.KEY_D / 2) & (np.abs(low[:, 1] - ky) < M.KEY_W / 2)) if len(low) else True
        rec = dict(key=name, hidden=all(each), corners=each, shield_alone=by_shield,
                   tip_below_top_mm=1000 * (M.KEY_TOP - rv[:, 2].min()), only_on_its_key=bool(on_key),
                   shield_above_keys_mm=1000 * (lv[:, 2].min() - M.KEY_TOP))
        out["presses"].append(rec)
        if verbose:
            print(rec)
    if verbose:
        print("worst IK (mm):", round(out["worst_ik_mm"], 2), " closest the hands come (mm):", round(out["min_gap_mm"], 1))
    return out


if __name__ == "__main__":
    run()
