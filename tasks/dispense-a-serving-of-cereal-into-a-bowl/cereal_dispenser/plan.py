"""Open-loop joint plans for the scripted demo. NumPy and yam_arena's URDF kinematics only (no Isaac Sim).
Created 11_16_26 (America/New_York).

`JointPlan` holds one joint target per control step. The stock `yam_scripted` policy builds its plans with
`yam_arena.scripted.ArmPlan(waypoints, base)`; `install()` wraps that name so that a demo() returning a ready
`JointPlan` for an arm is played as it is, and any other demo still gets the stock planner.
"""

import math

import numpy as np

STEP_FRACTION = 0.05  # Inspect Robots' default guard: a joint target may change by 5% of its range per control step


class JointPlan:
    """Joint targets per control step: q (n, 6) rad and grip (n,) from 0 closed to 1 open, one row every dt seconds."""

    def __init__(self, q, grip, dt, labels=None):
        self.q, self.grip, self.dt = np.asarray(q, float), np.asarray(grip, float), float(dt)
        self.labels = list(labels) if labels is not None else [""] * len(self.q)

    @property
    def duration(self):
        return len(self.q) * self.dt

    def action7(self, t):
        k = int(min(max(round(t / self.dt) - 1, 0), len(self.q) - 1))
        return np.r_[self.q[k], self.grip[k]], self.labels[k]


def install():
    from yam_arena import scripted

    stock = scripted.ArmPlan
    if getattr(stock, "_cereal", False):
        return

    def arm_plan(waypoints, base_pos, *args, **kwargs):
        if len(waypoints) == 1 and isinstance(waypoints[0], JointPlan):
            return waypoints[0]
        return stock(waypoints, base_pos, *args, **kwargs)

    arm_plan._cereal = True
    scripted.ArmPlan = arm_plan


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def frame(approach, closing):
    """Gripper rotation whose approach (+z) is ``approach`` and whose finger closing axis (+x) is ``closing``
    (made orthogonal to the approach)."""
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.c_[x, np.cross(z, x), z]


class Arm:
    """IK for one arm, seeded from the previous solution so consecutive poses stay on one branch."""

    def __init__(self, base, q0=(0.0, 1.0, 1.0, -0.6, 0.0, 0.0)):
        from yam_arena import ik

        self.ik = ik
        self.base = np.asarray(base, float)
        self.q = np.asarray(q0, float)
        self.worst_mm = self.worst_deg = 0.0

    def solve(self, pos, rot, seed=None):
        ik = self.ik
        best = None
        seeds = [self.q] if seed is None else [np.asarray(seed, float), self.q]
        for s in seeds:
            q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), q_init=s, base_pos=self.base, iters=300)
            if best is None or pe + 0.05 * re < best[1] + 0.05 * best[2]:
                best = (q, pe, re)
        q, pe, re = best
        if pe > 0.002 or re > math.radians(1.5):
            raise ValueError(f"out of reach: {np.round(pos, 4).tolist()} ({pe * 1000:.1f} mm, {math.degrees(re):.1f} deg)")
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, math.degrees(re))
        self.q = q
        return q


class Builder:
    """Collects joint targets for one arm, one row per control step."""

    def __init__(self, arm, dt, grip=1.0):
        self.arm, self.dt = arm, float(dt)
        self.q, self.grip, self.labels = [], [], []
        self.cur = np.asarray(arm.q, float)
        self.cur_grip = float(grip)

    def _push(self, q, g, label):
        self.q.append(np.array(q, float))
        self.grip.append(float(g))
        self.labels.append(label)
        self.cur, self.cur_grip = np.array(q, float), float(g)

    def hold(self, seconds, label="hold"):
        for _ in range(max(1, round(seconds / self.dt))):
            self._push(self.cur, self.cur_grip, label)

    def joints(self, q_goal, seconds, label="move", grip=None):
        """Joint-space move with smoothstep timing."""
        q0, g0 = self.cur.copy(), self.cur_grip
        g1 = g0 if grip is None else float(grip)
        n = max(1, round(seconds / self.dt))
        for k in range(1, n + 1):
            s = smooth(k / n)
            self._push(q0 + s * (np.asarray(q_goal, float) - q0), g0 + s * (g1 - g0), label)
        self.arm.q = self.cur.copy()

    def line(self, p0, p1, rot, seconds, label="line"):
        """Straight-line move of the fingertip midpoint at a fixed gripper rotation, by IK at every step."""
        n = max(1, round(seconds / self.dt))
        self.arm.q = self.cur.copy()
        for k in range(1, n + 1):
            s = smooth(k / n)
            q = self.arm.solve(np.asarray(p0, float) + s * (np.asarray(p1, float) - np.asarray(p0, float)), rot)
            self._push(q, self.cur_grip, label)

    def gripper(self, g, seconds, label="grip"):
        n = max(1, round(seconds / self.dt))
        g0 = self.cur_grip
        for k in range(1, n + 1):
            self._push(self.cur, g0 + (g - g0) * smooth(k / n), label)

    def wrist_roll(self, delta, seconds, label="turn"):
        """Joint 6 only: the gripper turns about its own approach axis, the fingertip midpoint stays put."""
        q0 = self.cur.copy()
        n = max(1, round(seconds / self.dt))
        for k in range(1, n + 1):
            q = q0.copy()
            q[5] = q0[5] + delta * smooth(k / n)
            self._push(q, self.cur_grip, label)
        self.arm.q = self.cur.copy()

    def plan(self):
        return JointPlan(self.q, self.grip, self.dt, self.labels)
