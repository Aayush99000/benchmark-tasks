"""A kinematic stand-in for the simulator, for dry runs of the demonstration and of the scorer. No contact physics.

- Each arm follows its joint commands with a first-order lag; fingertip points come from the arm's kinematics.
- The head stands on its spring at its sprung-out height unless something is above it: the underside of the
  left hand's closed fingers (when that hand is over the head) or the lid's underside. It never goes below
  the solid coil. The spring does not lean here.
- The lid stays between the fork's two fingers when the right hand has it (21 mm apart round a 5 mm lid, so
  it has play), falls shut by its weight when nothing holds it and it is past upright, is pushed up by the
  head, and is pushed down by the left fingertips pressing on it. Shut to within 0.3 degrees, the catch
  snaps over the barb; after that the lid stays within 0.45 degrees.
- The box does not move.

This checks reach, joint limits, the order of the hand-offs, control flow and the scorer's rules. It says
nothing about forces, about whether the jester leans out of the box, or about whether the catch holds: those
need the simulator.
"""

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from rdb_jack_in_box import fixed_scene as fixture
from rdb_jack_in_box import layout as L
from rdb_jack_in_box.scripted import HOLD_BELLY, PRESS_BELLY, TIP

FINGER_T = .0063              # half the thickness of two closed fingers across their closing axis
FINGER_HALF_WIDTH = .0137


class World:
    def __init__(self, *, lag_s=.12, hold_offset=(0., 0., 0.), press_offset=(0., 0., 0.), lid_play_deg=None):
        self.blend = 1. - np.exp(-1. / fixture.CONTROL_HZ / lag_s)
        self.hold_offset = np.asarray(hold_offset, dtype=float)     # where the left hand really is vs its joints
        self.press_offset = np.asarray(press_offset, dtype=float)
        self.joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.lid = L.LID_OPEN
        self.s = L.S_POPPED
        self.caught = False
        self.catch = 0.
        self.touching = False
        self.lid_play = np.radians(lid_play_deg) if lid_play_deg is not None else None
        self.max_head_above_lid = 0.

    # -- geometry ---------------------------------------------------------------------------
    def hand(self, index):
        return ik.tcp_pose(self.joints[7 * index:7 * index + 6], fixture.BASES[fixture.ARMS[index]])

    def tip(self, index):
        point, rotation = self.hand(index)
        return point + TIP * rotation[:, 2], rotation

    def head_centre(self):
        return L.head_centre(self.s, 0.)

    def head_top(self):
        return float(self.head_centre()[2] + L.HEAD_R)

    def lid_bottom_at_head(self, angle):
        return L.lid_underside_z(angle, L.BOX_X)

    def angle_for_head(self, top):
        """Lid opening at which its underside at the head's axis is at ``top``."""
        hinge = L.hinge_world()
        dx = hinge[0] - L.BOX_X
        lo, hi = 0., np.radians(89.)
        for _ in range(40):
            mid = (lo + hi) / 2
            if L.lid_underside_z(mid, L.BOX_X) < top:
                lo = mid
            else:
                hi = mid
        return lo if top > hinge[2] - L.LID_T / 2 else 0.

    def state(self):
        return {"lid": self.lid, "head": self.head_centre(), "lean": 0., "box": np.array((L.BOX_X, 0., 0.)),
                "box_rot": np.eye(3), "catch": self.catch}

    def lid_plate(self):
        """(centre, rotation, half sizes) of the lid's plate as an oriented box, tabletop frame."""
        rotation = L.rot_y(self.lid)
        centre = L.hinge_world() + rotation @ np.array((-L.SIDE / 2, 0., 0.))
        return centre, rotation, np.array((L.SIDE / 2, L.HALF, L.LID_T / 2))

    # -- one control step -------------------------------------------------------------------
    def step(self, command):
        command = np.asarray(command, dtype=float)
        self.joints = self.joints + self.blend * (command - self.joints)
        hinge = L.hinge_world()
        self.touching = False
        # the head: what is above it
        free_top = L.PIVOT_Z + L.S_POPPED + L.HEAD_R
        left_tip, left_rot = self.tip(0)
        left_tip = left_tip + self.hold_offset
        closed = self.joints[6] < .05
        if closed:
            # the underside of the closed fingers over the head's axis
            a = left_rot[:, 2]
            horizontal = np.hypot(left_tip[0] - L.BOX_X, left_tip[1])
            back = horizontal / max(np.hypot(a[0], a[1]), 1e-6)
            if back < .070 and abs(left_rot[2, 2]) < .6:          # level fingers reach over the head's axis
                under = left_tip[2] - HOLD_BELLY - back * (-a[2])  # fingers rise toward the hand
                if under < free_top + .002:
                    free_top = min(free_top, under)
                    self.touching = True
        low_top = L.PIVOT_Z + L.S_LOW + L.HEAD_R
        free_top = max(free_top, low_top)
        # the lid: bounds from below (the head, the shut stop) and from above (fork, pressing fingertips)
        lower = max(0., self.angle_for_head(free_top)) if self.lid < np.radians(89.) else 0.
        upper = L.LID_OPEN
        fork_tip, fork_rot = self.tip(1)
        jaw = abs(self.joints[13] * FINGER_OPEN)
        rel = fork_tip - hinge
        fork_angle = float(np.arctan2(rel[2], -rel[0]))
        radius = np.hypot(rel[0], rel[2])
        fork_on = (fork_tip[1] > -L.HALF + .005 and abs(radius - .095) < .03 and jaw < .02
                   and abs(np.dot(fork_rot[:, 0], L.lid_normal(fork_angle))) > .9)
        if fork_on:
            play = self.lid_play if self.lid_play is not None else (jaw - L.LID_T / 2) / radius
            upper = min(upper, fork_angle + play)
            lower_fork = fork_angle - play
            self.touching = True
        else:
            lower_fork = 0.
        # left fingertips pressing on the lid's top near its front edge
        press_tip = left_tip + self.press_offset
        if not closed or abs(left_rot[2, 2]) >= .6:
            rel = press_tip - hinge
            if -rel[0] > .09 and abs(press_tip[1]) < L.HALF:
                corner_top = rel[2] - PRESS_BELLY
                angle_at_tip = np.arctan2(corner_top - L.LID_T / 2, -rel[0])
                if angle_at_tip < self.lid + np.radians(2.) or angle_at_tip < upper:
                    upper = min(upper, max(angle_at_tip, 0.))
                    self.touching = True
        if self.caught:
            upper = min(upper, np.radians(.45))
        if self.lid >= np.radians(90.) and not fork_on:
            target = L.LID_OPEN                                   # leaning back: it falls open
        else:
            target = max(lower, lower_fork)                       # leaning forward: it falls onto what is under it
        new = min(max(target, lower_fork), upper) if upper >= lower_fork else upper
        new = max(new, 0.)
        # the head yields to the lid; the lid cannot press the head below the solid coil
        min_angle = self.angle_for_head(low_top)
        new = max(new, min_angle) if self.lid < np.radians(89.) else new
        self.lid = float(new)
        top = min(free_top, self.lid_bottom_at_head(self.lid)) if self.lid < np.radians(89.) else free_top
        self.s = float(np.clip(top - L.HEAD_R - L.PIVOT_Z, L.S_LOW, L.S_POPPED))
        if self.lid <= np.radians(.3):
            self.caught = True
        self.catch = 0. if (self.caught or self.lid > np.radians(6.)) else -.003
        return self.joints.copy()

    def grade_inputs(self):
        tcp = [self.hand(index)[0] for index in range(2)]
        return np.degrees(self.lid), self.head_centre(), np.array((L.BOX_X, 0., 0.)), np.eye(3), tcp, self.touching
