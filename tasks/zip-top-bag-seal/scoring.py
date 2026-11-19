"""Stateful scorer for "Seal the bag." From the bag's and its loose carrot coins' measured poses and
velocities, the robot's finger pads (oriented boxes placed from the measured joints and finger travel by
the URDF's forward kinematics at the runner's arm bases, bag.pad_boxes) and the track cells reported
closed by task.Dynamics."""
from __future__ import annotations

import numpy as np

import bag as B
from yam_arena.constants import PAD_TIP_PAST_TCP as PAD_TIP

HZ = 20
HOLD_S = 1.0
CLEAR = 0.08                          # every finger pad at least this far from the track at the end
REST_V = 0.02                         # m/s: the bag at rest ...
REST_W = 0.20                         # rad/s: ... and not turning


def _speed(o, key):
    v = o.get(key, (0, 0, 0)) if isinstance(o, dict) else getattr(o, key, (0, 0, 0))
    return float(np.linalg.norm(np.asarray(v, float)))


class TrialScorer:
    def __init__(self):
        self.held = 0
        self.step = 0
        self.best = 0                 # cells closed: a latched milestone (closed cells stay closed)
        self.spilled = False          # a coin out of the bag at any time: latched, in force after sealing too
        self.fell = False

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        if mechanics is None:
            raise ValueError("track measurements missing")
        self.step = step
        bag = np.asarray(root_poses["bag"], float)
        coins = [np.asarray(root_poses[f"coin{i + 1}"], float) for i in range(len(B.COINS))]
        out = [i + 1 for i, c in enumerate(coins) if not B.in_bag(bag, c)]
        if out:
            self.spilled = True
        if bag[2] < B.BOARD_TOP - 0.02:
            self.fell = True
        self.best = max(self.best, int(mechanics["closed"]))
        a, b = B.track_world(bag)
        boxes = B.pad_boxes(robot, PAD_TIP)
        far = min(B.segment_box_distance(a, b, box) for box in boxes)
        clear = far >= CLEAR
        still = _speed(objects["bag"], "lin_vel") < REST_V and _speed(objects["bag"], "ang_vel") < REST_W
        score = 0.0
        if self.best >= 4:
            score = 0.25
        if self.best >= 12:
            score = 0.5
        if self.best == B.N_CELLS:
            score = 0.75
        done = score == 0.75 and clear and still
        self.held = self.held + 1 if done else 0
        if done and self.held >= HOLD_S * HZ:
            score = 1.0
        if self.spilled or self.fell:
            score = 0.0
        return {"score": score, "success": score == 1.0, "closed_cells": self.best, "cells": mechanics["closed_cells"],
                "coins_out": out, "spilled": self.spilled, "fell": self.fell, "dropped": self.spilled or self.fell,
                "hands_clear": bool(clear), "clear": bool(clear), "clear_mm": round(1000 * far, 1),
                "bag_at_rest": bool(still), "held_frames": self.held}
