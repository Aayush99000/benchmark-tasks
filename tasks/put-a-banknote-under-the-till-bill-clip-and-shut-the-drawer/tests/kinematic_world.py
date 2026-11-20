"""A stand-in for the simulator, for dry runs of the demonstration and of other strategies.

The arms follow their joint commands with a first-order lag (optionally with a drooping shoulder). There is no
contact physics:

* the hold-down follows the left gripper once that gripper has closed on its tab, and drops back onto whatever
  is under its free end when let go;
* the note is a chain of eight rigid strips in the vertical plane through the gripper, with the real hinge
  stiffness and weight; its shape is the one of least energy (springs + gravity) above the till's real surfaces,
  with no friction. Its end strip follows the right gripper once the pads have closed on it;
* the drawer front moves with the closed right fingertips that push it; the notes in the till ride with the drawer.

The score is the real grader's. This checks reach, joint limits, clearances, the order of work, the path of the
note and the scoring. It says nothing about PhysX contact: whether the pads really hold a 2 mm strip, whether
the note snags on a wall, how the strips settle.
"""

import numpy as np
from scipy.optimize import minimize
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from rdb_till_note import fixed_scene as fixture, grading, till_model as m

GAP_PER_UNIT = -2. * float(FINGER_OPEN)
G = 9.81
TAB_BEARING = np.arctan2(m.ARM_T / 2 + m.TAB_H / 2, sum(m.TAB_X) / 2)      # tab centre above the arm line, from the pivot
TAB_RADIUS = float(np.hypot(m.ARM_T / 2 + m.TAB_H / 2, sum(m.TAB_X) / 2))
SAMPLES = np.linspace(0., 1., 5)


class KinematicWorld:
    def __init__(self, *, arm_lag_s=.12, sag=0., clearance=None, note_y_error=0.):
        self.blend = 1 - np.exp(-1 / fixture.CONTROL_HZ / arm_lag_s)
        self.sag, self.clearance = sag, clearance
        self.joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.s = m.TRAVEL
        self.hold = np.array([m.HOLD_ON_FLOOR] * 5)
        self.hold[m.STOCKED] = m.HOLD_ON_STACK
        self.stack_local = np.array((m.STACK_X, m.COMPARTMENT_Y[m.STOCKED], m.BILL_FLOOR_Z + m.STACK_SIZE[2] / 2))
        # the note: direction of each strip (angle above +x) and the centre (x, z) of one pinned strip
        self.theta = np.zeros(m.STRIPS)
        self.pin, self.pin_centre = 0, m.NOTE_START[0][[0, 2]].copy()
        self.note_y = m.NOTE_START_Y
        self.note_y_error = note_y_error          # the note hangs this far to one side of the gripper
        self.note_mode = "resting"                # resting, held, free
        self.grip_along = 0.
        self.tab_held = False
        self.fed_under = False          # the note went in while the hold-down's free end was raised clear of it
        self.grade = grading.TillNoteGrade()
        self.result = None
        self.min_gap = {}
        self.min_arm_gap = np.inf
        self.arm_gaps = []
        self.peak = {"note_snag_mm": 0.}
        self.tick = 0
        self.measured = self.joints.copy()

    # --- the note -------------------------------------------------------------------------------------
    def support(self, x):
        best = 0.
        for dx in (-.0025, 0., .0025):                       # thin walls are widened so that no sample steps over one
            h = m.support_height(x + dx, self.note_y, self.s)
            local = x + dx - m.drawer_x(self.s) - self.stack_local[0]
            if abs(local) <= m.STACK_SIZE[0] / 2 and abs(self.note_y - self.stack_local[1]) <= m.STACK_SIZE[1] / 2:
                h = max(h, m.STACK_TOP_Z)
            best = max(best, h)
        # The middle hold-down is solid: where the note is not under it (it has not been fed in below the raised
        # arm), the arm's upper surface is a surface the note lies on.
        if abs(self.note_y - m.COMPARTMENT_Y[m.STOCKED]) < m.NOTE_W / 2 + m.ARM_W / 2 and not self.fed_under:
            u, n = m.hold_axes(self.hold[m.STOCKED])
            pivot_x = m.drawer_x(self.s) + m.PIVOT_X
            along = (x - pivot_x) / u[0]
            if 0. <= along <= m.ARM_L:
                best = max(best, m.PIVOT_Z + along * u[2] + m.ARM_T / 2 / n[2])
        return best

    def nodes(self, theta):
        step = m.STRIP_L * np.column_stack((np.cos(theta), np.sin(theta)))
        points = np.zeros((m.STRIPS + 1, 2))
        points[self.pin] = self.pin_centre - step[self.pin] / 2
        for i in range(self.pin, m.STRIPS):
            points[i + 1] = points[i] + step[i]
        for i in range(self.pin - 1, -1, -1):
            points[i] = points[i + 1] - step[i]
        return points

    def energy(self, free, theta, index):
        theta = theta.copy()
        theta[index] = free
        points = self.nodes(theta)
        spring = .5 * m.HINGE_K * float(np.sum(np.diff(theta) ** 2))
        centres = (points[:-1] + points[1:]) / 2
        weight = m.STRIP_MASS * G * float(centres[:, 1].sum())
        sample = (points[:-1, None, :] + SAMPLES[None, :, None] * (points[1:] - points[:-1])[:, None, :]).reshape(-1, 2)
        floor = np.array([self.support(x) for x in sample[:, 0]])
        dip = np.maximum(floor + m.NOTE_T / 2 - sample[:, 1], 0.)
        return 1e4 * (spring + weight + .5 * 400. * float(np.sum(dip ** 2)))

    def settle(self, rounds=1):
        index = np.array([i for i in range(m.STRIPS) if i != self.pin])
        for _ in range(rounds):
            found = minimize(self.energy, self.theta[index], args=(self.theta, index), method="L-BFGS-B",
                             options={"maxiter": 40, "eps": 1e-4})
            self.theta[index] = np.clip(found.x, -np.pi, np.pi)

    def strips(self):
        points = self.nodes(self.theta)
        centres = (points[:-1] + points[1:]) / 2
        out, rotations = [], []
        for (x, z), t in zip(centres, self.theta):
            out.append((x, self.note_y, z))
            rotations.append(np.array(((np.cos(t), 0., -np.sin(t)), (0., 1., 0.), (np.sin(t), 0., np.cos(t)))))
        return np.array(out), np.array(rotations)

    # --- what the demonstration is given ----------------------------------------------------------------
    def stack(self):
        return self.stack_local + (m.drawer_x(self.s), 0., 0.)

    def state(self):
        strips, rotations = self.strips()
        return {"strips": strips, "strip_rot": rotations, "hold": self.hold.copy(), "drawer_open": self.s,
                "stack": self.stack()}

    # --- one control step --------------------------------------------------------------------------------
    def step(self, values):
        values = np.asarray(values, dtype=float)
        self.joints = self.joints + self.blend * (values - self.joints)
        measured = self.joints.copy()
        measured[[1, 8]] += self.sag                              # the shoulders really droop, and report it
        tcp, rot = {}, {}
        for i, arm in enumerate(fixture.ARMS):
            q = measured[7 * i:7 * i + 6]
            if np.any(values[7 * i:7 * i + 6] < ik.LOWER - 1e-6) or np.any(values[7 * i:7 * i + 6] > ik.UPPER + 1e-6):
                raise RuntimeError(f"{arm} arm commanded past a joint limit at tick {self.tick}")
            tcp[arm], rot[arm] = ik.tcp_pose(q, fixture.BASES[arm])
        jaw = {"left": self.joints[6], "right": self.joints[13]}
        wanted = {"left": values[6], "right": values[13]}

        # the hold-down and the left gripper
        pivot = np.array((m.drawer_x(self.s) + m.PIVOT_X, m.COMPARTMENT_Y[m.STOCKED], m.PIVOT_Z))
        strips, _ = self.strips()
        under = m.STACK_TOP_Z
        tip = m.tip_underside(self.hold[m.STOCKED], self.s)
        lying = [c for c in strips if abs(c[0] - tip[0]) <= m.STRIP_L / 2 + .002 and abs(c[1] - tip[1]) <= m.NOTE_W / 2
                 and c[2] <= m.STACK_TOP_Z + m.NOTE_T + .004]
        if lying and self.note_mode != "resting":
            under += m.NOTE_T
        rest = m.rest_angle(under)
        tab = m.tab_centre(self.hold[m.STOCKED], self.s)
        if not self.tab_held and jaw["left"] * GAP_PER_UNIT < m.ARM_W + .001 and wanted["left"] < .2:
            local = rot["left"].T @ (tab - tcp["left"])
            if abs(local[0]) < .006 and abs(local[1]) < .010 and -.014 < local[2] < PAD_TIP_PAST_TCP:
                self.tab_held = True
                self.tab_offset = tab - tcp["left"]
        elif self.tab_held and wanted["left"] * GAP_PER_UNIT > m.ARM_W + .004:
            self.tab_held = False
        if self.tab_held:
            reach = tcp["left"] + self.tab_offset - pivot
            angle = np.arctan2(reach[2], -reach[0]) - TAB_BEARING
            self.hold[m.STOCKED] = float(np.clip(angle, rest, np.radians(m.HOLD_LIMITS_DEG[1])))
        else:
            self.hold[m.STOCKED] = rest

        # the note and the right gripper
        approach = rot["right"][:, 2]
        if self.note_mode != "held" and jaw["right"] * GAP_PER_UNIT < m.NOTE_T + .003 and wanted["right"] < .05:
            centre = strips[0]
            local = rot["right"].T @ (centre - tcp["right"])
            if abs(local[0]) < .013 and abs(local[1]) < m.NOTE_W / 2 and -.012 < local[2] < PAD_TIP_PAST_TCP - .002:
                self.note_mode, self.pin, self.grip_along = "held", 0, float(local[2])
        elif self.note_mode == "held" and wanted["right"] * GAP_PER_UNIT > m.NOTE_T + .004:
            self.note_mode = "free"
            centres, _ = self.strips()
            tip = m.tip_underside(self.hold[m.STOCKED], self.s)
            pinned = [i for i, c in enumerate(centres) if abs(c[0] - tip[0]) <= m.STRIP_L / 2
                      and abs(tip[2] - c[2] - m.NOTE_T / 2) < .003]
            self.pin = pinned[0] if pinned else int(np.argmin(centres[:, 2]))
            self.pin_centre = centres[self.pin][[0, 2]].copy()
            self.settle(6)
        if self.note_mode == "held":
            centre = tcp["right"] + approach * self.grip_along
            self.pin_centre = centre[[0, 2]]
            self.theta[0] = np.arctan2(approach[2], approach[0])
            self.note_y = float(centre[1]) + self.note_y_error
            self.settle()

        if (self.note_mode == "held" and self.hold[m.STOCKED] > np.radians(25.)
                and abs(self.note_y - m.COMPARTMENT_Y[m.STOCKED]) < m.NOTE_W / 2 + m.ARM_W / 2 + .004):     # how close the carried note comes to the raised hold-down
            points = self.nodes(self.theta)
            line = m.hold_point(self.hold[m.STOCKED], np.linspace(0., m.ARM_L, 21)[:, None], -m.ARM_T / 2, self.s)
            gaps = np.linalg.norm(points[:, None, :] - line[None][:, :, [0, 2]], axis=2)
            self.peak["note_to_hold_down_mm"] = min(self.peak.get("note_to_hold_down_mm", np.inf),
                                                    round(1000 * float(gaps.min()), 1))

        if self.note_mode == "held" and not self.fed_under:
            tip_z = m.tip_underside(self.hold[m.STOCKED], self.s)[2]
            if tip_z > m.STACK_TOP_Z + .015:                      # room under the free end: the note can go in
                self.fed_under = True

        # the drawer and the closed right fingertips
        fingertip = tcp["right"] + approach * PAD_TIP_PAST_TCP
        front = m.drawer_x(self.s)
        if (self.note_mode != "held" and jaw["right"] < .06 and abs(fingertip[1]) <= m.PANEL_W / 2
                and m.PANEL_Z[0] <= fingertip[2] <= m.PANEL_Z[1] and front < fingertip[0] < front + .04):
            moved = min(fingertip[0] - front, self.s)
            self.s -= moved
            if self.note_mode == "free" and self.note_in_drawer():
                self.pin_centre = self.pin_centre + (moved, 0.)

        # what the robot reads
        measured[6] = max(measured[6], m.ARM_W / GAP_PER_UNIT) if self.tab_held else measured[6]
        measured[13] = max(measured[13], m.NOTE_T / GAP_PER_UNIT) if self.note_mode == "held" else measured[13]
        self.measured = measured
        self.check_clearance(measured)
        strips, rotations = self.strips()
        self.result = self.grade.update(strips=strips, strip_rot=rotations, stack=self.stack(), drawer_open=self.s,
                                        hold_angles=self.hold, tcp=[tcp[arm] for arm in fixture.ARMS])
        self.tick += 1
        return self.result

    def note_in_drawer(self):
        centres, _ = self.strips()
        d = centres[:, 0] - m.drawer_x(self.s)
        return bool(np.all((d > m.PANEL_T) & (d < m.DRAWER_FLOOR_X[1])) and centres[:, 2].max() < m.PANEL_Z[1])

    def check_clearance(self, measured):
        if self.clearance is None or self.tick % 2:
            return
        clouds = []
        for i, arm in enumerate(fixture.ARMS):
            cloud = self.clearance.arm_cloud(measured[7 * i:7 * i + 6], measured[7 * i + 6], fixture.BASES[arm],
                                             links=("gripper", "tip_left", "tip_right", "link5", "link4", "link3"))
            clouds.append(cloud)
            boxes = m.solid_boxes(self.s) + [("table", (-1., -1., -1.), (2., 1., 0.))]
            stack = self.stack()
            half = np.asarray(m.STACK_SIZE) / 2
            boxes.append(("stack", stack - half, stack + half))
            for name, low, high in boxes:
                low, high = np.asarray(low), np.asarray(high)
                gap = self.clearance.box_gap(cloud, (low + high) / 2, (high - low) / 2)
                key = (arm, name)
                if gap < self.min_gap.get(key, (np.inf, 0))[0]:
                    self.min_gap[key] = (gap, self.tick)
        gap = self.clearance.arm_gap(*clouds)
        self.arm_gaps.append((self.tick, gap))
        self.min_arm_gap = min(self.min_arm_gap, gap)


def play(reference, world, max_steps, *, stop_at_full=True, watch=None):
    """Run the demonstration in the stand-in world. Returns (final grade, commanded joint rows)."""
    actions = reference.sequence()
    trace = []
    for _ in range(max_steps):
        reference.observe(world.measured, world.state())
        values = np.asarray(next(actions).actions[0].data, dtype=float)
        trace.append(values)
        result = world.step(values)
        if watch is not None:
            watch(world, reference, result)
        if stop_at_full and result["score"] == 1.:
            break
    return world.result, np.array(trace)
