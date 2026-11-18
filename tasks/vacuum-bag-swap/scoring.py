"""Stateful scorer for "Put a new bag in the vacuum." From the measured poses of the vacuum, both bags
and the lid, the robot's measured TCPs, and the lid's latch state reported by task.Dynamics."""
from __future__ import annotations

import math

import numpy as np

import vac as V

HZ = 20
HOLD_S = 1.0
CLEAR = 0.08                          # every finger pad this far from the lid's latch and the new collar's tab
REST_LIN = 0.01                       # m/s: vacuum, lid and new bag at rest through the final hold ...
REST_ANG = 0.10                       # ... rad/s
SHOVE_MM = 30.0                       # the vacuum pushed further than this over the bench: not steadied (a
                                      # canister shifts a little on its wheels while handled; 30 mm is a shove)
GUIDE_SLACK_MM = 1.0                  # a collar "in the holder": its bottom edge inside the channel (this slack) ...
GUIDE_Y_MM = 8.0                      # ... centred across within this ...
GUIDE_DEG = 10.0                      # ... square to the front wall within this ...
SEAT_MM = 3.0                         # seated: its bottom edge within this of the stop


def to_vac(vac_pose, p):
    """World point ``p`` in the vacuum's frame as it stood at the start (so at the start it is unchanged)."""
    vac_pose = np.asarray(vac_pose, float)
    r = V.quat_matrix(vac_pose[3:7])
    return r.T @ (np.asarray(p, float) - vac_pose[:3]) + np.asarray(V.VAC_POS)


def collar_state(vac_pose, bag_pose):
    """(bottom edge z, its bottom edge's x offset from the channel's middle, y offset, squareness deg) of a bag's collar in
    the vacuum's frame, and its tab's top point (world)."""
    vac_pose, bag_pose = np.asarray(vac_pose, float), np.asarray(bag_pose, float)
    rv, rb = V.quat_matrix(vac_pose[3:7]), V.quat_matrix(bag_pose[3:7])
    c = to_vac(vac_pose, bag_pose[:3])
    bag_x = rv.T @ rb[:, 0]           # the bag's own +x: from the collar toward the bag, +x when seated
    bag_z = rv.T @ rb[:, 2]
    bottom = c - bag_z * V.COLLAR[2] / 2
    square = math.degrees(math.acos(float(np.clip(bag_x[0], -1.0, 1.0))))
    tab = bag_pose[:3] + rb @ np.array([0.0, 0.0, V.COLLAR[2] / 2 + V.TAB[2]])
    return float(bottom[2]), float(bottom[0] - V.COLLAR_X), float(c[1] - V.YC), square, tab


def in_bin(p):
    return math.hypot(p[0] - V.BIN_C[0], p[1] - V.BIN_C[1]) <= V.BIN_R - 0.010 and p[2] <= V.TOP + V.BIN_H


def on_bench(p, vac_pose):
    """A bag lying on the bench, not in the bin, the carton or the vacuum: dropped."""
    if p[2] > V.TOP + 0.10 or in_bin(p):
        return False
    if abs(p[0] - V.CARTON_C[0]) <= V.CARTON[0] / 2 and abs(p[1] - V.CARTON_C[1]) <= V.CARTON[1] / 2:
        return False
    q = to_vac(vac_pose, p)
    return not (V.FRONT_X <= q[0] <= V.BACK_X and abs(q[1] - V.YC) <= V.HALF_W)


def _state(o, key):
    """A measured object's velocity, whether its state comes as an object or a dict."""
    if o is None:
        raise ValueError("object state missing for the rest check")
    v = o.get(key) if isinstance(o, dict) else getattr(o, key)
    return np.asarray(getattr(v, "tolist", lambda: v)(), float).reshape(-1)[:3]


class TrialScorer:
    def __init__(self):
        self.held = 0
        self.step = 0
        self.max_shove = 0.0
        self.dropped = False
        self.binned = False               # milestone: the old bag has been let go of in the wastebasket

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        if mechanics is None:
            raise ValueError("lid measurements missing")
        self.step = step
        vac = np.asarray(root_poses["vacuum"], float)
        old = np.asarray(root_poses["old_bag"], float)
        new = np.asarray(root_poses["new_bag"], float)
        shove = 1000 * float(np.linalg.norm(vac[:2] - np.asarray(V.VAC_POS[:2])))
        self.max_shove = max(self.max_shove, shove)
        if on_bench(old[:3], vac) or on_bench(new[:3], vac):
            self.dropped = True
        tcps = [np.asarray(robot[s]["tcp"], float)[:3] for s in ("left", "right")]
        pads = [np.asarray(p, float) for p in mechanics.get("pads", [])] or tcps
        hands = tcps + pads
        old_tab = collar_state(vac, old)[4]
        if in_bin(old[:3]) and all(float(np.linalg.norm(h - old_tab)) > 0.06 for h in hands):
            self.binned = True
        old_binned = self.binned and in_bin(old[:3])      # once let go in the bin, it has to stay there
        bottom, dx, dy, square, tab = collar_state(vac, new)
        guided = (abs(dx) <= (V.SLOT / 2 + GUIDE_SLACK_MM / 1000) and abs(dy) <= GUIDE_Y_MM / 1000 and square <= GUIDE_DEG
                  and bottom <= V.LIP_TOP)
        seated = guided and abs(bottom - V.SEAT_Z) <= SEAT_MM / 1000
        latched = bool(mechanics.get("latched"))
        latch = np.asarray(mechanics.get("latch_xyz", (0.0, 0.0, 0.0)), float)
        clear = all(float(np.linalg.norm(h - latch)) >= CLEAR and float(np.linalg.norm(h - tab)) >= CLEAR for h in hands)
        rest = True
        for name in ("vacuum", "lid", "new_bag"):
            o = objects.get(name) if isinstance(objects, dict) else None
            lin = float(np.linalg.norm(_state(o, "lin_vel")))
            ang = float(np.linalg.norm(_state(o, "ang_vel")))
            rest = rest and lin <= REST_LIN and ang <= REST_ANG
        score = 0.0
        if old_binned:
            score = 0.25
            if guided:
                score = 0.5
                if seated and self.max_shove <= SHOVE_MM:
                    score = 0.75
        done = score == 0.75 and latched and clear and rest
        self.held = self.held + 1 if done else 0
        if done and self.held >= HOLD_S * HZ:
            score = 1.0
        if self.max_shove > SHOVE_MM:
            score = min(score, 0.5)
        if self.dropped:
            score = min(score, 0.25)
        return {"score": score, "success": score == 1.0, "old_in_bin": bool(old_binned),
                "new_bottom_mm": round(1000 * (bottom - V.SEAT_Z), 1), "new_bottom_dx_mm": round(1000 * dx, 1),
                "new_dy_mm": round(1000 * dy, 1), "new_square_deg": round(square, 1), "guided": bool(guided),
                "seated": bool(seated), "lid_deg": mechanics.get("lid_deg"), "latched": latched,
                "vacuum_shove_mm": round(shove, 1), "max_shove_mm": round(self.max_shove, 1), "dropped": self.dropped,
                "hands_clear": bool(clear), "at_rest": bool(rest), "held_frames": self.held,
                "clear": bool(clear), "binned_milestone": self.binned}
