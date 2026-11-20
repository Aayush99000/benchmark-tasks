"""Full-score demonstration: lift the hold-down, lay the note under it, let it down, shut the drawer.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a scripted
demonstration, simulator state: where the note's eight strips are, the hold-downs' angles, how far the drawer is
open. It changes nothing in the simulation: the note, the hold-down and the drawer move only because the grippers
move them.

Plan
  1. Left gripper: pinch the finger tab on the middle hold-down (16 mm across), swing the arm up 32 degrees above
     level about its hinge and keep it there.
  2. Right gripper, tilted 30 degrees below level and turned so that its pads close one above the other: pinch
     the 19.5 mm end of the note that hangs over the front edge of the drawer case, draw the note off the case.
  3. Carry it over the middle compartment, centre it between the walls, then move forward and down so that the
     note's far end lands on the notes in the compartment and slides to the back under the raised hold-down.
  4. Right gripper lets go of the note's end, which falls flat, and draws back; left gripper lets the hold-down
     down onto the note and lets go.
  5. Right gripper, closed, pushes the drawer front until the drawer is home. Both arms go back to ready.

Every grasp is checked by the gripper's measured opening (and the object's measured place) and tried again if it
missed; every approach is trimmed until the gripper's measured position is on target.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import till_model as m
from .fixed_scene import ARMS, BASES, CONTROL_HZ

OPEN = float(FINGER_OPEN)
GAP_PER_UNIT = -2. * OPEN                 # pad-to-pad opening for a gripper reading of 1


def grip_rotation(depression_deg, closing):
    """Gripper orientation: approach axis in the x-z plane, pointing away from the robot and ``depression_deg``
    below level; the pads close along ``closing``. Columns: closing axis, pad width axis, approach axis."""
    d = np.radians(depression_deg)
    a = np.array((np.cos(d), 0., -np.sin(d)))
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


NOTE_DEP = 30.
R_NOTE = grip_rotation(NOTE_DEP, (np.sin(np.radians(NOTE_DEP)), 0., np.cos(np.radians(NOTE_DEP))))   # pads one above the other
R_CLIP = grip_rotation(60., (0., -1., 0.))                                                           # pads side by side
# Joint angles near each grip (from the reach study); they seed the inverse kinematics.
SEED = {"note": (.51, 1.73, .89, .26, .43, -1.30), "clip": (-.86, 2.03, 1.49, -.67, -.39, -.79)}

TAB_OPEN = -.012                          # pads 24 mm apart round the 16 mm tab, inside the 70 mm compartment
TAB_CLOSED = -(m.ARM_W / 2 - .002)        # 2 mm inside the tab's faces: about 10 N a finger
TAB_RISE = .006                           # the gripper's centre point is this far above the tab's centre
CLIP_UP = np.radians(32.)
NOTE_OPEN = -.012                         # pads 24 mm apart round the overhanging end
NOTE_BACK = .003                          # centre point this far toward the free edge from the end strip's centre
STANDOFF = .060
CARRY_Z = .215
LAY_START = (.075, CARRY_Z)               # gripper centre point in the drawer frame (x from the drawer front, z)
LAY_END = (.152, .097)
FAR_GAP = .010                            # the note's far edge ends this far from the back wall
PUSH_Y, PUSH_Z = -.100, .060              # where the closed fingertips meet the drawer front
PUSH_PAST = .012
PUSH_SPEED = .07


class TillReference:
    """Emit one absolute 14-D joint action per control step."""

    VARIANTS = (None, "no_lift")

    def __init__(self, variant=None):
        """``variant="no_lift"`` is a negative control: the same demonstration with the left hand left out (the
        hold-down is never lifted). It is never the scored demonstration."""
        if variant not in self.VARIANTS:
            raise ValueError(f"unknown demonstration variant {variant!r}")
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.offset = {arm: np.zeros(3) for arm in ARMS}      # correction of each arm's commanded point
        self.pose = dict.fromkeys(ARMS)                       # (point, rotation) each arm was last sent to
        self.side = 0.                                        # sideways correction while the note is carried
        self.yaw = 0.                                         # and a turn about the vertical, to square it up
        self.clip_command = 0.
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.log = {"events": []}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state

    def actual_tcp(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])[0]

    def gap(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6]) * GAP_PER_UNIT

    def note(self, what, **values):
        row = {"tick": self.tick, "event": what}
        for key, value in values.items():
            row[key] = np.round(np.asarray(value, dtype=float), 4).tolist() if not isinstance(value, (bool, str)) else value
        self.log["events"].append(row)

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- motion ---------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def solve(self, arm, point, rotation, seed=None, iters=40):
        joints, error, _ = ik.solve(np.asarray(point) + self.offset[arm], ik.matrix_to_quat_xyzw(rotation),
                                    self.joints[arm] if seed is None else seed, BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        self.pose[arm] = (np.asarray(point, dtype=float).copy(), np.asarray(rotation))
        return joints

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, targets):
        """Move in joint space to the given joint angles (smooth start and stop)."""
        start = {arm: self.joints[arm].copy() for arm in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = ik.smoothstep(step / steps)
            for arm, goal in targets.items():
                self.joints[arm] = start[arm] + alpha * (np.asarray(goal) - start[arm])
            yield self.emit()

    def first(self, arm, point, rotation, seed, seconds):
        """From anywhere, in joint space, to a pose found from a seed."""
        goal = self.solve(arm, point, rotation, seed=np.asarray(seed, dtype=float), iters=300)
        yield from self.glide(seconds, {arm: goal})

    def move(self, seconds, targets):
        """Straight lines of the grippers' centre points, at fixed or smoothly turning orientation."""
        start = {arm: self.pose[arm] for arm in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = ik.smoothstep(step / steps)
            for arm, (point, rotation) in targets.items():
                p0, r0 = start[arm]
                turn = rotation if r0 is rotation or np.allclose(r0, rotation) else ik.slerp_matrix(r0, rotation, alpha)
                self.joints[arm] = self.solve(arm, p0 + alpha * (np.asarray(point) - p0), turn)
            yield self.emit()

    def trim(self, arm, repeats=3, limit=.025):
        """Shift the command by the measured error (the arms sag under their own weight and under load)."""
        for _ in range(repeats):
            point, rotation = self.pose[arm]
            error = point - self.actual_tcp(arm)
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(error, -.006, .006), -limit, limit)
            self.joints[arm] = self.solve(arm, point, rotation)
            yield from self.hold(.3)
        self.note(f"trim {arm}", offset=self.offset[arm], left_over=self.pose[arm][0] - self.actual_tcp(arm))

    def grip(self, arm, finger, seconds):
        start = self.aperture[arm]
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            self.aperture[arm] = start + step / steps * (finger - start)
            yield self.emit()

    def home(self, arms, seconds):
        yield from self.glide(seconds, {arm: np.asarray(READY, dtype=float) for arm in arms})
        for arm in arms:
            self.pose[arm] = None
            self.offset[arm] = np.zeros(3)

    # -- 1. the hold-down -------------------------------------------------------------------
    def tab_point(self, angle):
        return m.tab_centre(angle, float(self.state["drawer_open"])) + (0., 0., TAB_RISE)

    def swing_clip(self, start, end, seconds):
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            angle = start + ik.smoothstep(step / steps) * (end - start)
            self.joints["left"] = self.solve("left", self.tab_point(angle), R_CLIP)
            yield self.emit()
        self.clip_command = end

    def lift_clip(self):
        self.phase("reach hold-down")
        back = -R_CLIP[:, 2] * STANDOFF
        self.aperture["left"] = TAB_OPEN
        for attempt in range(3):
            angle = float(self.state["hold"][m.STOCKED])
            point = self.tab_point(angle)
            if attempt == 0:
                yield from self.first("left", point + back, R_CLIP, SEED["clip"], 2.2)
            else:
                yield from self.move(.8, {"left": (point + back, R_CLIP)})
            yield from self.trim("left")
            yield from self.move(1.2, {"left": (point, R_CLIP)})
            yield from self.trim("left", 2)
            yield from self.grip("left", TAB_CLOSED, .6)
            yield from self.hold(.4)
            held = self.gap("left") > m.ARM_W - .004
            self.note("hold-down tab grasp", attempt=attempt, opening_mm=1000 * self.gap("left"), held=bool(held),
                      tab_from_gripper_mm=1000 * (m.tab_centre(float(self.state["hold"][m.STOCKED]),
                                                               float(self.state["drawer_open"])) - self.actual_tcp("left")))
            if held:
                break
            yield from self.grip("left", TAB_OPEN, .4)
            yield from self.move(.8, {"left": (point + back, R_CLIP)})
        self.phase("lift hold-down")
        yield from self.swing_clip(angle, CLIP_UP, 1.8)
        yield from self.hold(.3)
        for _ in range(3):                                     # the loaded arm rides low: raise until measured
            short = CLIP_UP - float(self.state["hold"][m.STOCKED])
            if short < np.radians(3.):
                break
            yield from self.swing_clip(self.clip_command, self.clip_command + min(short, np.radians(8.)), .5)
            yield from self.hold(.3)
        self.note("hold-down raised", deg=np.degrees(float(self.state["hold"][m.STOCKED])),
                  commanded_deg=np.degrees(self.clip_command))

    # -- 2. the note ------------------------------------------------------------------------
    def take_note(self):
        self.phase("reach note")
        approach = R_NOTE[:, 2]
        self.aperture["right"] = NOTE_OPEN
        for attempt in range(3):
            centre, along = self.state["strips"][0], self.state["strip_rot"][0][:, 0]
            point = centre - along * NOTE_BACK
            if attempt == 0:
                yield from self.first("right", point - approach * STANDOFF, R_NOTE, SEED["note"], 2.2)
            else:
                yield from self.move(.8, {"right": (point - approach * STANDOFF, R_NOTE)})
            yield from self.trim("right")
            yield from self.move(1.4, {"right": (point, R_NOTE)})
            yield from self.trim("right", 2)
            yield from self.grip("right", 0., .8)
            yield from self.hold(.4)
            slip = self.state["strips"][0] - self.actual_tcp("right")
            held = self.gap("right") > .0008 and np.linalg.norm(slip) < .015
            self.note("note grasp", attempt=attempt, opening_mm=1000 * self.gap("right"), held=bool(held),
                      end_strip_from_gripper_mm=1000 * slip)
            if held:
                break
            yield from self.grip("right", NOTE_OPEN, .4)
            yield from self.move(.8, {"right": (point - approach * STANDOFF, R_NOTE)})
        self.phase("draw note off the case")
        # Up and back toward the robot, staying in the note's own lane, until the whole note hangs clear.
        x0 = m.drawer_x(float(self.state["drawer_open"]))
        yield from self.move(1.2, {"right": ((point[0] - .050, point[1], point[2] + .060), R_NOTE)})
        yield from self.move(2.4, {"right": ((x0 + LAY_START[0], point[1], CARRY_Z), R_NOTE)})
        yield from self.hold(.5)

    def note_rotation(self):
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return np.array(((c, -s, 0.), (s, c, 0.), (0., 0., 1.))) @ R_NOTE

    def centre_note(self, point):
        """Shift the gripper sideways until the hanging note is over the middle of the compartment, and turn it
        about the vertical until the note points straight along the compartment (it has 2 mm to spare a side)."""
        lane = m.COMPARTMENT_Y[m.STOCKED]
        error = float(np.mean(self.state["strips"][1:, 1])) - lane
        self.side = float(np.clip(self.side - .7 * error, -.012, .012))
        along = np.mean(self.state["strip_rot"][2:, :, 0], axis=0)
        skew = float(np.arctan2(along[1], along[0]))
        self.yaw = float(np.clip(self.yaw - .5 * skew, -np.radians(8.), np.radians(8.)))
        return np.array((point[0], lane + self.side, point[2])), error, skew

    def lay_note(self):
        self.phase("carry note over the till")
        x0 = m.drawer_x(float(self.state["drawer_open"]))
        lane = m.COMPARTMENT_Y[m.STOCKED]
        start = np.array((x0 + LAY_START[0], lane, LAY_START[1]))
        end = np.array((x0 + LAY_END[0], lane, LAY_END[1]))
        yield from self.move(2.4, {"right": (start, R_NOTE)})          # sideways only: the note swings little
        yield from self.hold(.8)
        yield from self.trim("right", 2)
        for _ in range(3):
            target, error, skew = self.centre_note(start)
            yield from self.move(.4, {"right": (target, self.note_rotation())})
            yield from self.hold(.3)
        self.note("note centred over the compartment", side_error_mm=1000 * error, shift_mm=1000 * self.side,
                  skew_deg=np.degrees(skew), turn_deg=np.degrees(self.yaw))
        self.phase("lay note under the hold-down")
        steps = round(4.5 * CONTROL_HZ)
        for step in range(1, steps + 1):
            point = start + ik.smoothstep(step / steps) * (end - start)
            if step % 8 == 0:
                self.centre_note(point)
            point[1] = lane + self.side
            self.joints["right"] = self.solve("right", point, self.note_rotation())
            yield self.emit()
        yield from self.hold(.4)
        # Slide the note until its far edge is where it belongs: FAR_GAP short of the compartment's back wall.
        wanted = x0 + m.BILL_X[1] - FAR_GAP
        for _ in range(round(2.5 * CONTROL_HZ)):
            short = wanted - (float(self.state["strips"][-1][0]) + m.STRIP_L / 2)
            if abs(short) < .0015:
                break
            point[0] = float(np.clip(point[0] + np.clip(.5 * short, -.002, .002), end[0] - .025, end[0] + .020))
            self.joints["right"] = self.solve("right", point, self.note_rotation())
            yield self.emit()
        yield from self.hold(.4)
        strips = self.state["strips"]
        self.note("note laid", far_end_in_drawer_mm=1000 * (strips[-1][0] + m.STRIP_L / 2 - x0),
                  strip_heights_mm=1000 * strips[:, 2], side_mm=1000 * (strips[:, 1] - lane),
                  turn_deg=np.degrees(self.yaw))

    # -- 4. let go ---------------------------------------------------------------------------
    def let_go_one_hand(self):
        self.phase("let go of note")
        yield from self.grip("right", NOTE_OPEN, .5)
        yield from self.hold(.5)
        yield from self.move(1.2, {"right": (self.pose["right"][0] - self.pose["right"][1][:, 2] * .07,
                                             self.pose["right"][1])})
        strips = self.state["strips"]
        self.note("note let go", strip_heights_mm=1000 * strips[:, 2],
                  hold_down_deg=np.degrees(float(self.state["hold"][m.STOCKED])))

    def let_go(self):
        # The right hand goes first: with both hands at the compartment there is no room to let the hold-down down.
        self.phase("let go of note")
        yield from self.grip("right", NOTE_OPEN, .5)
        yield from self.hold(.5)
        yield from self.move(1.2, {"right": (self.pose["right"][0] - self.pose["right"][1][:, 2] * .07,
                                             self.pose["right"][1])})
        strips = self.state["strips"]
        self.note("note let go", strip_heights_mm=1000 * strips[:, 2],
                  far_end_in_drawer_mm=1000 * (strips[-1][0] + m.STRIP_L / 2 - m.drawer_x(float(self.state["drawer_open"]))))
        self.phase("let hold-down down")
        yield from self.swing_clip(self.clip_command, m.HOLD_ON_NOTE - np.radians(2.), 1.6)
        yield from self.hold(.3)
        yield from self.grip("left", TAB_OPEN, .4)
        yield from self.hold(.3)
        yield from self.move(1.0, {"left": (self.pose["left"][0] + (.01, 0., .085), R_CLIP)})   # straight up
        self.note("hold-down let down", deg=np.degrees(float(self.state["hold"][m.STOCKED])),
                  strip_heights_mm=1000 * self.state["strips"][:, 2])

    # -- 5. the drawer -----------------------------------------------------------------------
    def shut_drawer(self):
        self.phase("reach drawer front")
        yield from self.grip("right", 0., .4)
        tip = R_NOTE[:, 2] * PAD_TIP_PAST_TCP                   # the closed fingertips, from the centre point
        front = m.drawer_x(float(self.state["drawer_open"]))
        start = np.array((front - .030, PUSH_Y, PUSH_Z)) - tip
        yield from self.move(1.8, {"right": (start + (0., 0., .07), R_NOTE)})
        if self.variant is None:
            yield from self.home(("left",), 1.6)               # the right arm is out of its way now
        yield from self.move(1.2, {"right": (start, R_NOTE)})
        yield from self.trim("right", 2)
        self.phase("push drawer shut")
        goal = m.FRONT_CLOSED_X + PUSH_PAST - tip[0]
        point = start.copy()
        for extra in range(3):
            while point[0] < goal and float(self.state["drawer_open"]) > .0005:
                point[0] = min(goal, point[0] + PUSH_SPEED / CONTROL_HZ)
                self.joints["right"] = self.solve("right", point, R_NOTE)
                yield self.emit()
            yield from self.hold(.4)
            if float(self.state["drawer_open"]) <= .0015:
                break
            goal += .008                                       # the arm gives under the push: lean further in
        self.note("drawer pushed", open_mm=1000 * float(self.state["drawer_open"]),
                  gripper_x=self.actual_tcp("right")[0], commanded_x=point[0])
        self.phase("away")
        yield from self.move(.8, {"right": (point + (-.06, 0., .02), R_NOTE)})
        yield from self.home(("right",), 2.0)
        yield from self.hold(3.0)

    def sequence(self):
        if self.variant == "no_lift":
            yield from self.take_note()
            yield from self.lay_note()
            yield from self.let_go_one_hand()
        else:
            yield from self.lift_clip()
            yield from self.take_note()
            yield from self.lay_note()
            yield from self.let_go()
        yield from self.shut_drawer()
        self.phase("done")
        while True:
            yield self.emit()
