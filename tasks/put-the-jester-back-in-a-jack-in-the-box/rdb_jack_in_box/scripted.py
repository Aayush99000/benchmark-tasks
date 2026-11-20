"""Full-score demonstration: the left hand holds the jester down, the right hand brings the lid down over it,
the left hand slips out as the lid takes the jester, then presses the lid's front edge shut while the right
hand lets go of the lid.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: the box's pose, the lid's angle, where the jester's head is and how
far the catch is pushed in. It changes nothing in the simulation: the head goes down only where fingers or
the lid press it, the lid moves only where fingers push it, and the catch holds the lid only by contact.

Plan.

1. The left hand, closed, comes in level with the head from the front left, pointing 35 degrees right of
   straight ahead and 15 degrees down, the flat of its fingers over the head. The right hand, open 21 mm,
   comes in from the right side, fingers pointing left, either side of the open lid 95 mm from its hinge: a
   fork, not a pinch.
2. The left hand pushes the head down until its top is 5 mm below the rim, against about 6 N of spring.
3. The right hand swings the lid down to 20 degrees from shut, turning its wrist with the lid, so the lid
   ends just above the left fingers.
4. Hand-off 1: the left hand slides out along its own fingers through the gap under the lid's front edge.
   The head comes up against the lid and the right hand holds the lid against it (the lid's angle is held by
   feedback on its measured angle).
5. Hand-off 2: the left hand comes over the lid and sets its fingertips on the lid 12 mm in from the front
   edge; it presses until it carries the lid, and the right hand slides out sideways.
6. The left hand presses the lid's front edge down along its arc until the lid is shut and the catch has
   snapped over the barb, then lifts off; if the lid rises, it presses once more.
7. Both arms move away and wait.

``variant`` gives two controls that must not reach full score: ``"lid_only"`` never presses the jester (the
lid is swung down onto it and pressed shut); ``"early_release"`` presses the jester down and lets it go
before the lid comes down.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import grading, layout as L
from .fixed_scene import ARMS, BASES, CONTROL_HZ

HOLD, LID = ARMS                        # the left hand holds the jester and later presses the lid; the right hand works the lid
OPEN = float(FINGER_OPEN)
TIP = float(PAD_TIP_PAST_TCP)           # the fingertips end this far beyond the gripper's centre point
DT = 1. / CONTROL_HZ

HOLD_YAW, HOLD_PITCH = np.radians(-15.), np.radians(30.)
PRESS_YAW, PRESS_PITCH = np.radians(-30.), np.radians(45.)
HOLD_BELLY = .0057                      # the closed fingers' lower edges are this far below the fingertips' middle
HOLD_PAST = .008                        # fingertips' middle this far past the head's axis, along the fingers
HEAD_BELOW = .005                       # press until the head's top is this far below the rim
HEAD_SLACK = .012                       # and ease off if it is further down than this
PRESS_SPEED = .040                      # m/s
FORK_R = .095                           # the fork takes the lid this far from its hinge line
FORK_IN = .022                          # fingertips this far in over the lid's side edge
FORK_OUT = .040                         # and this far back along the fingers before going in and after coming out
FORK_GAP = .021                         # pads this far apart (the lid's collider is 5 mm thick)
LID_HOLD = np.radians(18.)              # where the right hand holds the lid once the left hand is out
# How far down the lid may come (degrees) for each distance the left hand has slid back along its fingers (m):
# the lid's underside 3 mm clear of the left arm's collision hulls, plus 3 degrees (found with the hulls).
# The fingers leave the head's top at about 45 mm, with the lid at about 31 degrees.
LID_CLEAR = ((0., .01, .02, .03, .04, .05, .06, .08, .10), (39., 38., 38., 35., 31.5, 31.5, 28., 25.5, 18.))
LID_SPEED = np.radians(25.)             # rad/s
WITHDRAW = .125                         # the left hand slides this far back along its fingers
WITHDRAW_SPEED = .040
LID_PRESS_IN = .012                     # the left fingertips press the lid this far in from its front edge
PRESS_BELLY = .003                      # the pressing fingertips' lowest corner is this far below their middle
OVER_TRAVEL = .025                      # the shutting press may run this far past the lid's shut position
SHUT_DEG = .8                           # "shut" for the demonstration: lid within this
CAUGHT_M = -.0012                       # the catch is back out this far from its stop: it is over the barb
SEED = {"hold": (-.76, 1.41, .97, .18, -.14, -1.61), "press": (-.88, 1.54, 1.12, -.39, -.25, -1.47),
        "fork": (.09, 2.34, 2.26, -1.31, -1.37, -1.12)}


def frame(approach, closing):
    """Gripper orientation: fingers point along ``approach``; they open and close along ``closing``."""
    a = np.asarray(approach, dtype=float)
    a /= np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def heading(yaw, pitch):
    return np.array((np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), -np.sin(pitch)))


HOLD_ROT = frame(heading(HOLD_YAW, HOLD_PITCH), np.cross(heading(HOLD_YAW, HOLD_PITCH), (0., 0., 1.)))
PRESS_ROT = frame(heading(PRESS_YAW, PRESS_PITCH), (1., -1., 0.))


def fork_dir(angle):
    """Direction the right fingers point while they straddle the lid open ``angle``: to the left (+y) and, in
    the lid's plane, turned toward its hinge by 12 degrees with the lid upright and 30 degrees once it is
    below 75 degrees (where the right wrist keeps its joints well inside their limits)."""
    beta = np.radians(np.interp(np.degrees(angle), (0., 75., 95., 180.), (30., 30., 12., 12.)))
    toward_hinge = np.array((np.cos(angle), 0., -np.sin(angle)))
    return np.cos(beta) * np.array((0., 1., 0.)) + np.sin(beta) * toward_hinge


def fork_rot(angle):
    """Right hand straddling the lid open ``angle``: fingers in the lid's plane, closing across it."""
    return frame(fork_dir(angle), L.lid_normal(angle))


def fingertips(joint_pos, arm):
    """Middle of the two fingertips' ends, from joint readings (tabletop frame)."""
    start = 7 * ARMS.index(arm)
    point, rotation = ik.tcp_pose(np.asarray(joint_pos, dtype=float)[start:start + 6], BASES[arm])
    return point + TIP * rotation[:, 2]


class JackReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant="full"):
        if variant not in ("full", "lid_only", "early_release"):
            raise ValueError(variant)
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = {HOLD: OPEN, LID: OPEN}
        self.tip = {}                                   # commanded fingertip points (Cartesian moves)
        self.rot = {HOLD: HOLD_ROT, LID: fork_rot(L.LID_OPEN)}
        self.lid_cmd = L.LID_OPEN                       # commanded lid angle of the fork
        self.lid_trim = 0.
        self.worst_ik = 0.
        self.tick = 0
        self.phases = []
        self.log = {"variant": variant, "events": []}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def measured_tip(self, arm):
        return fingertips(self.measured, arm)

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def lid(self):
        return float(self.state["lid"])

    def head_top(self):
        """Head's top above the rim (box frame), m."""
        return grading.head_state(self.state["head"], self.state["box"], self.state["box_rot"])[0]

    def box_shift(self):
        return np.asarray(self.state["box"], dtype=float) - (L.BOX_X, 0., 0.)

    def lid_point(self, angle, x_rel, z_rel=0., y=0.):
        return L.lid_point(angle, x_rel, z_rel, y) + self.box_shift()

    def caught(self):
        return float(self.state["catch"]) >= CAUGHT_M and self.lid() <= np.radians(SHUT_DEG)

    # -- motion ---------------------------------------------------------------------------
    def solve(self, arm, point, rotation, seed=None, iters=40):
        target = np.asarray(point, dtype=float) - TIP * rotation[:, 2]
        joints, error, _ = ik.solve(target, ik.matrix_to_quat_xyzw(rotation),
                                    self.joints[arm] if seed is None else seed, BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        return joints

    def emit(self):
        for arm, point in self.tip.items():
            self.joints[arm] = self.solve(arm, point, self.rot[arm])
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def glide(self, seconds, targets, aperture=None, seeds=None):
        """Move in joint space to fingertip poses {arm: (point, rotation)} (smooth start and stop)."""
        aperture = aperture or {}
        seeds = seeds or {}
        goal = {arm: self.solve(arm, p, r, seed=np.asarray(SEED[seeds.get(arm, "hold")]), iters=400)
                for arm, (p, r) in targets.items()}
        start = {arm: self.joints[arm].copy() for arm in targets}
        jaws = {arm: self.aperture[arm] for arm in aperture}
        self.tip = {arm: point for arm, point in self.tip.items() if arm not in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm in targets:
                self.joints[arm] = start[arm] + alpha * (goal[arm] - start[arm])
            for arm, value in aperture.items():
                self.aperture[arm] = jaws[arm] + alpha * (value - jaws[arm])
            yield self.emit()
        for arm, (point, rotation) in targets.items():
            self.tip[arm] = np.asarray(point, dtype=float).copy()
            self.rot[arm] = rotation

    def line(self, seconds, targets, rotations=None):
        """Move fingertips in straight lines (smooth start and stop), turning to ``rotations`` on the way."""
        rotations = rotations or {}
        start = {arm: self.tip[arm].copy() for arm in targets}
        start_rot = {arm: self.rot[arm] for arm in rotations}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, goal in targets.items():
                self.tip[arm] = start[arm] + alpha * (np.asarray(goal, dtype=float) - start[arm])
            for arm, goal in rotations.items():
                self.rot[arm] = ik.slerp_matrix(start_rot[arm], goal, alpha)
            yield self.emit()

    def home(self, arm, seconds):
        """Open the fingers and swing the arm back to its ready pose in joint space."""
        self.tip.pop(arm, None)
        start, jaw = self.joints[arm].copy(), self.aperture[arm]
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            self.joints[arm] = start + alpha * (np.asarray(READY) - start)
            self.aperture[arm] = jaw + min(1., 3. * alpha) * (OPEN - jaw)
            yield self.emit()

    def wait(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    # -- the parts of the demonstration -----------------------------------------------------
    def hold_point(self, height):
        """Left fingertips' middle with the closed fingers' underside at ``height`` over the head's axis."""
        axis = np.asarray(self.state["head"], dtype=float) if self.state is not None else L.head_centre(L.S_POPPED, 0.)
        along = HOLD_ROT[:, 2]
        point = np.array((axis[0], axis[1], height + HOLD_BELLY)) + HOLD_PAST * along / np.linalg.norm(along[:2])
        return point

    def fork_point(self, angle, out=0.):
        """Right fingertips' middle on the lid's mid-plane, FORK_IN in over its side edge, or ``out`` back
        along the fingers."""
        return self.lid_point(angle, -FORK_R, 0., -L.HALF + FORK_IN) - out * fork_dir(angle)

    def press_point(self, angle, depth=0.):
        """Left fingertips' middle resting on the lid near its front edge (``depth`` pressed past the surface)."""
        return self.lid_point(angle, -(L.SIDE - LID_PRESS_IN), L.LID_T / 2 + PRESS_BELLY - depth)

    def press_head(self, seconds):
        """Servo the left hand down until the head's top is HEAD_BELOW under the rim."""
        floor = L.RIM_Z - HEAD_BELOW - .035                # never command the underside lower than this
        count = 0
        for _ in range(round(seconds * CONTROL_HZ)):
            top = self.head_top()
            tip = self.tip[HOLD]
            if top > -HEAD_BELOW:
                tip[2] = max(tip[2] - PRESS_SPEED * DT, floor + HOLD_BELLY)
            elif top < -HEAD_SLACK:
                tip[2] += .0005
            yield self.emit()
            count = count + 1 if top <= -HEAD_BELOW + .001 else 0
            if count >= 6:
                return True
        return False

    def hold_head(self):
        """One step of keeping the head where it is pressed (used while the lid moves)."""
        top = self.head_top()
        if top > -HEAD_BELOW + .002:
            self.tip[HOLD][2] -= .0005
        elif top < -HEAD_SLACK:
            self.tip[HOLD][2] += .0005

    def swing_lid(self, target, *, hold=False, settle=.5):
        """Move the fork along the lid's arc to ``target`` (its wrist turning with the lid), then hold the
        lid there by feedback on its measured angle."""
        steps = max(1, round(abs(self.lid_cmd - target) / LID_SPEED * CONTROL_HZ))
        start = self.lid_cmd
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            self.lid_cmd = start + alpha * (target - start)
            self._fork_to(self.lid_cmd)
            if hold:
                self.hold_head()
            yield self.emit()
        for _ in range(round(settle * CONTROL_HZ)):
            self._fork_servo(target)
            if hold:
                self.hold_head()
            yield self.emit()

    def _fork_to(self, angle):
        self.tip[LID] = self.fork_point(angle + self.lid_trim)
        self.rot[LID] = fork_rot(angle + self.lid_trim)

    def _fork_servo(self, target):
        """Trim the fork so the lid's measured angle comes to ``target`` (the arm gives under the spring)."""
        error = self.lid() - target
        self.lid_trim = float(np.clip(self.lid_trim - .15 * error, -np.radians(12.), np.radians(4.)))
        self._fork_to(target)

    def event(self, name, **extra):
        row = {"tick": self.tick, "event": name, "lid_deg": round(float(np.degrees(self.lid())), 2),
               "head_top_above_rim_mm": round(1000 * self.head_top(), 1),
               "lean_deg": round(float(np.degrees(self.state["lean"])), 2),
               "catch_mm": round(1000 * float(self.state["catch"]), 2),
               "box_shift_mm": [round(1000 * float(v), 1) for v in self.box_shift()[:2]],
               "lid_trim_deg": round(float(np.degrees(self.lid_trim)), 2),
               "tips_command_mm": {arm: [round(1000 * float(v), 1) for v in self.tip[arm]] for arm in self.tip},
               "tips_measured_mm": {arm: [round(1000 * float(v), 1) for v in self.measured_tip(arm)] for arm in ARMS},
               "gripper_opening": [round(self.opening(arm), 3) for arm in ARMS]}
        row.update(extra)
        self.log["events"].append(row)

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- the demonstration ------------------------------------------------------------------
    def sequence(self):
        self.phase("reach")
        popped_top = L.PIVOT_Z + L.S_POPPED + L.HEAD_R
        fork_jaw = -FORK_GAP / 2
        yield from self.glide(2.5, {HOLD: (self.hold_point(popped_top + .020), HOLD_ROT),
                                    LID: (self.fork_point(L.LID_OPEN, FORK_OUT), fork_rot(L.LID_OPEN))},
                              {HOLD: 0., LID: fork_jaw}, seeds={HOLD: "hold", LID: "fork"})
        yield from self.wait(.3)
        self.phase("fork in")
        yield from self.line(1.2, {LID: self.fork_point(L.LID_OPEN)})
        self.event("fork in")
        if self.variant != "lid_only":
            self.phase("press the jester down")
            yield from self.line(.6, {HOLD: self.hold_point(popped_top)})
            ok = yield from self.press_head(5.)
            yield from self.wait(.4)
            self.event("pressed", reached=ok)
            if self.variant == "early_release":
                self.phase("let the jester go")
                yield from self.line(1.5, {HOLD: self.tip[HOLD] - WITHDRAW * HOLD_ROT[:, 2]})
        self.phase("swing the lid down")
        holding = self.variant == "full"
        target = np.radians(LID_CLEAR[1][0]) if holding else np.radians(12.)
        yield from self.swing_lid(target, hold=holding)
        self.event("lid down")
        if holding:
            self.phase("hand-off 1: slide out under the lid")
            # The left hand slides back along its fingers; the lid follows it down as far as the hand allows,
            # so it is about 30 degrees from shut when the fingers come off the head and the head comes up
            # against it. Then the lid goes on down to LID_HOLD.
            start = self.tip[HOLD].copy()
            steps = round(WITHDRAW / WITHDRAW_SPEED * CONTROL_HZ)
            for step in range(1, steps + 1):
                alpha = step / steps
                alpha *= alpha * (3. - 2. * alpha)
                back = alpha * WITHDRAW
                self.tip[HOLD] = start - back * HOLD_ROT[:, 2]
                wanted = max(LID_HOLD, np.radians(np.interp(back, *LID_CLEAR)))
                self.lid_cmd = max(wanted, self.lid_cmd - LID_SPEED * DT)
                self._fork_servo(self.lid_cmd)
                yield self.emit()
            for _ in range(round(.8 * CONTROL_HZ)):
                self.lid_cmd = max(LID_HOLD, self.lid_cmd - LID_SPEED * DT)
                self._fork_servo(self.lid_cmd)
                yield self.emit()
            target = LID_HOLD
            self.event("left hand out", head_under_lid=bool(self.head_top() < .045))
        self.phase("hand-off 2: press the lid's front edge")
        angle = self.lid()
        above = self.press_point(angle) + .030 * L.lid_normal(angle)
        if HOLD not in self.tip:
            self.tip[HOLD] = self.measured_tip(HOLD)
        yield from self.glide(1.6, {HOLD: (above, PRESS_ROT)}, seeds={HOLD: "press"})
        for _ in range(round(.3 * CONTROL_HZ)):
            self._fork_servo(target)
            yield self.emit()
        yield from self.line(.8, {HOLD: self.press_point(self.lid(), .004)})
        for _ in range(round(.5 * CONTROL_HZ)):
            self._fork_servo(target)
            yield self.emit()
        self.event("left on the lid")
        self.phase("fork out")
        out = self.fork_point(self.lid_cmd + self.lid_trim, FORK_OUT)
        yield from self.line(1.2, {LID: out})
        yield from self.line(.6, {LID: out + (0., -.020, .040)})
        yield from self.home(LID, 2.)
        self.event("right hand away")
        for attempt in range(2):
            self.phase(f"shut the lid (attempt {attempt + 1})")
            shut = yield from self.shut()
            self.event("shut", attempt=attempt + 1, shut=shut)
            self.phase("lift off the lid")
            yield from self.line(.8, {HOLD: self.press_point(0., -.025)})
            yield from self.wait(.6)
            self.event("lifted off", lid_stays=bool(self.lid() <= np.radians(grading.SHUT_DEG)))
            if self.lid() <= np.radians(grading.SHUT_DEG - .5):
                break
        self.phase("hands away")
        yield from self.line(.8, {HOLD: self.tip[HOLD] + (-.060, .040, .040)})
        yield from self.home(HOLD, 2.)
        self.event("parked")
        self.phase("wait")
        while True:
            yield self.emit()

    def shut(self):
        """Press the lid's front edge down along its arc, past shut by up to OVER_TRAVEL, until the lid is shut
        and the catch is over the barb."""
        angle = max(self.lid(), 0.)
        depth = .004
        count = 0
        for _ in range(round(6. * CONTROL_HZ)):
            if angle > 0.:
                angle = max(0., angle - np.radians(12.) * DT)
            elif depth < OVER_TRAVEL:
                depth += .006 * DT
            self.tip[HOLD] = self.press_point(angle, depth)
            yield self.emit()
            count = count + 1 if self.caught() else 0
            if count >= 8:
                for _ in range(round(.3 * CONTROL_HZ)):
                    yield self.emit()
                return True
        return False
