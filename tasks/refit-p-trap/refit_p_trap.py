"""Under a bathroom vanity's basin (its doors open; the station table is the cabinet's floor): the basin's chrome
drain tailpiece hangs down from the drain, and a white PVC trap arm comes out of the back wall and turns down beside
it. The 1-1/2 in J-bend that joins them has been taken off, cleared and rinsed, and lies on the cabinet floor on its
side. Each pipe still carries its white slip nut, its beveled washer inside it, slid up out of the way.

The robot must refit the J-bend the way a plumber or maintenance worker does after clearing a clogged trap: offer it
up onto both pipes at once (the tailpiece into one leg, the trap arm's end into the other), hold it there, run each
slip nut down onto its leg's thread, and turn both hand-tight, so that the trap hangs from its nuts by itself.

Run it:  inspect-robots run --embodiment yam_arena -E task=refit_p_trap.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "..." --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ptrapkit  # noqa: E402
from ptrapkit import body, box, cyl, prism12, ring  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

ptrapkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]  # the cabinet's floor
MM = 0.001
PVC = (0.93, 0.93, 0.91)
CHROME = (0.80, 0.81, 0.83)
PORCELAIN = (0.96, 0.96, 0.95)
WALL = (0.70, 0.74, 0.78)  # painted drywall
CABINET = (0.62, 0.50, 0.36)  # the vanity's carcass
COUNTER = (0.42, 0.40, 0.38)
DARK = (0.12, 0.12, 0.13)
ptrapkit.METALLIC.update({CHROME: 0.12})


def quat_about(axis, angle):
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


# -- dimensions (a standard 1-1/2 in tubular PVC P-trap, e.g. Oatey HDC9682) -----------------------------------------
PIPE_R = 0.75 * 25.4 * MM  # 1-1/2 in tubular: 38.1 mm outside
LEG_C = 3.45 * 25.4 * MM  # the J-bend's legs, 87.6 mm apart on centre
LEG_RI, LEG_RO = 20.7 * MM, 23.5 * MM  # a leg's bore (the pipe slips in) and outside
THREAD_RO = 24.4 * MM  # its thread band
TUBE_R = 21.0 * MM  # the bend's tube
PITCH = 25.4 * MM / 11.5  # tubular slip-joint thread, 11-1/2 per inch
SNUG_TURNS = 4.0  # turns from the thread's start until the nut squeezes its washer home (the thread band's length)
THREAD_LEN = PITCH * SNUG_TURNS
NUT_H, NUT_RO, NUT_LIP_RI, NUT_SKIRT_RI = 25.0 * MM, 31.0 * MM, 20.3 * MM, 26.2 * MM
NUT_LIP = 4.0 * MM
WING_R = 34.0 * MM  # out to its four wings' tips (68 mm across)

# the J-bend, in its own frame: origin on the inlet leg's axis at the bend's outside bottom, z up its legs, the outlet
# leg on +y
ZU = TUBE_R + LEG_C / 2  # the bend's centre height: its straight legs start here
IN_TOP, OUT_TOP = 125.0 * MM, 100.0 * MM  # the legs' mouths (their threads' tops)
J_MASS = 0.14

# the layout (world), with the J-bend refitted
XI, YI = 0.36, -0.045  # the tailpiece's axis: the J-bend's inlet leg
XO, YO = XI, YI + LEG_C  # the trap arm's down leg: the outlet leg
Z_JB = TOP + 0.235  # the refitted J-bend's bottom
IN_INSERT, OUT_INSERT = 30.0 * MM, 25.0 * MM  # how far each pipe reaches into its leg, refitted
TAIL_END = Z_JB + IN_TOP - IN_INSERT
OUT_END = Z_JB + OUT_TOP - OUT_INSERT
DRAIN_Z = TOP + 0.56  # the basin's drain (its locknut's underside)
ELBOW_Z0 = TOP + 0.395  # the trap arm's down leg turns toward the wall here
ELBOW_R = 45.0 * MM
Z_ARM = ELBOW_Z0 + ELBOW_R  # the trap arm's horizontal run, into the wall
X_WALL = 0.58
# where the nuts are slid up to, out of the way (their bottoms)
IN_NUT_PARK = Z_JB + IN_TOP + 40.0 * MM
OUT_NUT_PARK = Z_JB + OUT_TOP + 30.0 * MM
# the J-bend lying on the cabinet floor on its side, legs toward the back wall (+x), outlet leg to the left
J0_XY = (0.185, 0.135)


def _j_shapes():
    solid, seen = [], []
    for y, top in ((0.0, IN_TOP), (LEG_C, OUT_TOP)):
        for s in ring(ZU, top, LEG_RI, LEG_RO, PVC, segments=16):
            solid.append(box((s.centre[0], s.centre[1] + y, s.centre[2]), s.size, PVC, yaw=s.yaw))
        seen.append(cyl((0.0, y, (ZU + top - THREAD_LEN) / 2), LEG_RO + 0.2 * MM, top - THREAD_LEN - ZU, PVC))
        for k in range(4):  # the thread band's crests
            z = top - THREAD_LEN + (k + 0.5) * PITCH
            seen.append(cyl((0.0, y, z), THREAD_RO, PITCH * 0.55, PVC))
        seen.append(cyl((0.0, y, top - THREAD_LEN / 2), LEG_RO + 0.3 * MM, THREAD_LEN, PVC))
    # the bend: a half circle of tube in the y-z plane below the legs
    n = 8
    rb = LEG_C / 2
    for k in range(n):
        a0, a1 = math.pi + math.pi * k / n, math.pi + math.pi * (k + 1) / n
        am = (a0 + a1) / 2
        c = (0.0, rb + rb * math.cos(am), ZU + rb * math.sin(am))
        length = 2 * rb * math.sin(math.pi / (2 * n)) + 6.0 * MM
        q = quat_about((1.0, 0.0, 0.0), am)
        solid.append(box(c, (2 * TUBE_R * 0.9, 2 * TUBE_R * 0.9, length), PVC, quat=q))
        seen.append(cyl(c, TUBE_R, length, PVC, quat=q))
    return solid, seen


def _nut_shapes():
    """In the nut's frame: origin at its middle, z up its bore. Its top lip rides on the pipe; its skirt (the female
    thread) goes over a leg's thread band."""
    h = NUT_H / 2
    solid = [*ring(h - NUT_LIP, h, NUT_LIP_RI, NUT_RO, PVC, segments=16),
             *ring(-h, h - NUT_LIP, NUT_SKIRT_RI, NUT_RO, PVC, segments=16)]
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        r = (NUT_RO + WING_R) / 2 - 0.5 * MM
        solid.append(box((r * math.cos(a), r * math.sin(a), -1.0 * MM), (WING_R - NUT_RO + 1.0 * MM, 8.0 * MM, 20.0 * MM),
                         PVC, yaw=a))
    seen = [cyl((0.0, 0.0, 0.0), NUT_RO - 0.2 * MM, NUT_H - 0.4 * MM, PVC),
            cyl((0.0, 0.0, h + 0.05 * MM), NUT_LIP_RI + 0.6 * MM, 0.1 * MM, DARK)]  # the washer's edge, seen in its bore
    return solid, seen


def _split(shapes):
    solid, seen = shapes
    return solid + seen, tuple(range(len(solid), len(solid) + len(seen)))


_JS, _JV = _split(_j_shapes())
JBEND = body("jbend", pos=(J0_XY[0], J0_XY[1], TOP + LEG_RO + 0.3 * MM), mass=J_MASS, friction=0.9, shapes=_JS,
             visual_only=_JV, quat=quat_about((0.0, 1.0, 0.0), math.pi / 2), com=(0.0, LEG_C / 2, 0.065),
             solver_iterations=32)
_NS, _NV = _split(_nut_shapes())
NUT_IN = body("nut_in", pos=(XI, YI, IN_NUT_PARK + NUT_H / 2), mass=0.03, friction=0.9, shapes=_NS, visual_only=_NV,
              solver_iterations=32)
NUT_OUT = body("nut_out", pos=(XO, YO, OUT_NUT_PARK + NUT_H / 2), mass=0.03, friction=0.9, shapes=_NS,
               visual_only=_NV, solver_iterations=32)


def _tailpiece_shapes():
    length = DRAIN_Z - TAIL_END
    solid = prism12((XI, YI, TAIL_END + length / 2), PIPE_R, length, CHROME)
    seen = [cyl((XI, YI, TAIL_END + length / 2), PIPE_R + 0.2 * MM, length, CHROME),
            cyl((XI, YI, TAIL_END + 0.1 * MM), PIPE_R - 1.5 * MM, 0.2 * MM, DARK),  # its open end
            cyl((XI, YI, DRAIN_Z + 6.0 * MM), 38.0 * MM, 12.0 * MM, CHROME),  # the drain's locknut, under the basin
            cyl((XI, YI, DRAIN_Z + 30.0 * MM), 24.0 * MM, 36.0 * MM, CHROME)]  # the drain body
    return solid, seen


def _trap_arm_shapes():
    solid, seen = [], []
    down = ELBOW_Z0 - OUT_END
    solid += prism12((XO, YO, OUT_END + down / 2), PIPE_R, down, PVC)
    seen += [cyl((XO, YO, OUT_END + down / 2), PIPE_R + 0.2 * MM, down, PVC),
             cyl((XO, YO, OUT_END + 0.1 * MM), PIPE_R - 1.5 * MM, 0.2 * MM, DARK)]
    n = 6
    for k in range(n):  # the bend toward the wall, in the x-z plane
        g = (k + 0.5) * (math.pi / 2) / n
        c = (XO + ELBOW_R - ELBOW_R * math.cos(g), YO, ELBOW_Z0 + ELBOW_R * math.sin(g))
        length = 2 * ELBOW_R * math.sin(math.pi / 4 / n) + 6.0 * MM
        q = quat_about((0.0, 1.0, 0.0), g)
        solid.append(box(c, (2 * PIPE_R * 0.9, 2 * PIPE_R * 0.9, length), PVC, quat=q))
        seen.append(cyl(c, PIPE_R, length, PVC, quat=q))
    run = X_WALL - (XO + ELBOW_R)
    cx = XO + ELBOW_R + run / 2
    solid.append(box((cx, YO, Z_ARM), (run, 2 * PIPE_R * 0.9, 2 * PIPE_R * 0.9), PVC))
    seen.append(cyl((cx, YO, Z_ARM), PIPE_R, run, PVC, axis="X"))
    # the trap adapter in the wall's drain, its slip nut and the escutcheon on the wall
    seen += [cyl((X_WALL - 12.0 * MM, YO, Z_ARM), NUT_RO, 22.0 * MM, PVC, axis="X"),
             cyl((X_WALL - 1.0 * MM, YO, Z_ARM), 45.0 * MM, 2.0 * MM, CHROME, axis="X")]
    return solid, seen


def _cabinet_shapes():
    solid = [
        box((X_WALL + 10.0 * MM, 0.0, TOP + 0.365), (20.0 * MM, 0.92, 0.73), WALL),  # the back wall
        box((0.36, 0.47, TOP + 0.365), (0.50, 18.0 * MM, 0.73), CABINET),  # the cabinet's sides
        box((0.36, -0.47, TOP + 0.365), (0.50, 18.0 * MM, 0.73), CABINET),
        box((0.36, 0.0, TOP + 0.73 + 15.0 * MM), (0.52, 0.96, 30.0 * MM), COUNTER),  # the vanity top
        cyl((XI, YI, DRAIN_Z + 12.0 * MM + 0.075), 0.19, 0.15, PORCELAIN),  # the basin's bowl, under the top
    ]
    return solid, []


_TS, _TV = _split(_tailpiece_shapes())
TAILPIECE = body("tailpiece", pos=(0.0, 0.0, 0.0), friction=0.6, shapes=_TS, visual_only=_TV)
_AS, _AV = _split(_trap_arm_shapes())
TRAP_ARM = body("trap_arm", pos=(0.0, 0.0, 0.0), friction=0.6, shapes=_AS, visual_only=_AV)
_CS, _CV = _split(_cabinet_shapes())
CABINET_BODY = body("vanity", pos=(0.0, 0.0, 0.0), friction=0.6, shapes=_CS, visual_only=_CV)
# a plumber's work light under the basin, in front of the trap and above the arms' reach (no collider)
CABINET_BODY.meta["lights"] = [((0.12, 0.0, TOP + 0.52), 0.03, float(os.environ.get("TRAP_LIGHT", "60000")))]

OBJECTS = [CABINET_BODY, TAILPIECE, TRAP_ARM, JBEND, NUT_IN, NUT_OUT]
ptrapkit.register(*OBJECTS)


# -- the slip-joint model ------------------------------------------------------------------------------------------
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SlipJoint:
    """One slip joint of the trap: a fixed pipe (axis vertical at ``axis``, its open end at height ``end``) going into
    a leg of the J-bend (the leg's mouth at ``mouth`` in the J-bend's frame), with its slip nut riding on the pipe.

    Our model, run before every physics substep; everything the robot does is physical (PhysX contacts):
    - The pipe goes into the leg's bore physically (both are solids, 1 mm apart). Pushed up onto a pipe's end within
      CAPTURE of the bore's axis, the leg's chamfered mouth centres it (with the play real tailpieces and trap arms have
      in their drain and adapter): a centring force on the J-bend at the mouth, at most 15 N, only while the pipe's end
      is on the mouth.
    - The nut is a free body. Off the thread, it rides the pipe square and centred (as its lip and washer do), stays
      where it is put (its washer's drag holds its weight) and slides and spins freely when pushed or turned.
    - It reaches the leg's thread when its skirt's mouth comes down to the leg's mouth: there it stops (it cannot be
      pushed past the thread's start, and pushing it pushes the J-bend down). It goes onto the thread only if the leg
      is on the pipe (the pipe at least ENGAGE_IN inside it), coaxial with the pipe within ALIGN and upright within
      TILT_ON degrees (a misaligned nut does not start: it stays on the thread's start).
    - On the thread, we pose the nut: it turns only with a hand that grips it (both pads closed on it, the hand's TCP
      on its axis at its height) and swings round its axis; turned clockwise seen from above (tightening) it runs down
      PITCH a turn, and stops SNUG_TURNS turns on, its washer squeezed home: hand-tight. Turned back past its start, it
      comes off the thread.
    - From a quarter turn on, the thread holds the leg on the pipe: the J-bend's leg is held on the pipe's axis, at
      its height then, and the J-bend can hang from the nut.
    """

    CAPTURE = 4.0 * MM
    ENGAGE_IN = 10.0 * MM
    ALIGN = 2.0 * MM
    TILT_ON = 3.0
    CATCH_TURNS = 0.25
    GRIP_AXIS = 8.0 * MM  # the hand's TCP within this of the nut's axis (horizontally)
    GRIP_HEIGHT = 10.0 * MM  # and of its middle's height
    GRIP_GAP = (55.0 * MM, 2 * WING_R + 6.0 * MM)  # the pads closed on the nut: on its body or its wings
    GRIP_LEVEL = math.sin(math.radians(20.0))  # the closing axis within 20 degrees of level

    def __init__(self, label, nut, axis, end, mouth):
        self.label, self.nut = label, nut
        self.axis = np.asarray(axis, float)
        self.end = float(end)
        self.mouth = np.asarray(mouth, float)  # in the J-bend's frame
        self.origin = np.zeros(3)
        self.clear()

    def clear(self):
        self.threaded = False
        self.theta = 0.0
        self.caught = False
        self.catch_z = 0.0
        self.hand = None
        self.heading = 0.0
        self.inside = -1.0
        self.radial = 1.0
        self.tilt = 90.0
        self.nut_z = 1.0
        self.leg_top = 0.0
        self.peak_turns = 0.0
        self.offset = np.zeros(2)
        self.catch_xy = np.zeros(2)

    def turns(self):
        return self.theta / (2 * math.pi) if self.threaded else 0.0

    def on_pipe(self, depth=None):
        """The pipe at least ``depth`` inside the leg, coaxial within 3 mm, the J-bend upright within 5 degrees."""
        return self.inside >= (self.ENGAGE_IN if depth is None else depth) and self.radial <= 3.0 * MM and self.tilt <= 5.0

    def snug(self):
        return self.threaded and self.turns() >= SNUG_TURNS - 0.25

    def _nut_pose_z(self):
        return self.leg_top + NUT_H / 2 - PITCH * self.turns()

    def _gripping(self, hand, nut_z):
        d = hand.tcp - np.array([*self.axis, nut_z])
        return (math.hypot(d[0], d[1]) <= self.GRIP_AXIS and abs(d[2]) <= self.GRIP_HEIGHT
                and abs(float(hand.closing[2])) <= self.GRIP_LEVEL and self.GRIP_GAP[0] <= hand.gap <= self.GRIP_GAP[1])

    def update(self, frames, acc, model):
        fj, fn = frames.get("jbend"), frames.get(self.nut)
        if fj is None or fn is None:
            return
        o = self.origin
        # the leg on the pipe
        up = fj.rot[:, 2]
        self.tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
        mouth_w = fj.world(self.mouth) - o
        self.leg_top = float(mouth_w[2])
        self.inside = self.leg_top - self.end
        # the pipe's axis seen from the leg: its offset at the pipe's end (or the mouth, whichever is higher)
        at_z = max(self.end, self.leg_top - max(self.inside, 0.0))
        leg_at = mouth_w + up * ((at_z - self.leg_top) / max(up[2], 1e-3))
        self.radial = float(np.hypot(*(leg_at[:2] - self.axis)))
        self.offset = mouth_w[:2] - self.axis
        if -3.0 * MM < self.inside < 6.0 * MM:  # the pipe's end on the mouth's chamfer: pushed up, it centres the leg
            off = self.offset
            r = float(np.hypot(*off))
            if r <= self.CAPTURE:
                v = fj.point_vel(mouth_w + o)
                f = -6000.0 * off - 20.0 * v[:2]
                n = float(np.hypot(*f))
                if n > 15.0:
                    f *= 15.0 / n
                ptrapkit._wrench(acc, "jbend", fj, np.array([f[0], f[1], 0.0]), mouth_w + o)
        # the nut
        p = fn.pos - o
        self.nut_z = float(p[2]) if not self.threaded else self._nut_pose_z()
        if not self.threaded:
            skirt = float(p[2]) - NUT_H / 2
            if float(p[2]) + NUT_H / 2 > self.end + 3.0 * MM:  # its lip still round the pipe: it rides it
                f = -300.0 * (p[:2] - self.axis) - 0.5 * np.asarray(fn.vel[:2])
                fz = 0.03 * 9.81 - 1.5 * float(fn.vel[2])  # its washer's drag holds it where it is put
                ptrapkit._wrench(acc, self.nut, fn, np.array([f[0], f[1], fz]), fn.pos)
                upn = fn.rot[:, 2]
                ptrapkit._torque(acc, self.nut, 0.02 * np.cross(upn, [0.0, 0.0, 1.0]) - 1.0e-4 * np.asarray(fn.spin))
            # the leg's thread's start: the nut stops on it
            nut_leg = float(np.hypot(*(p[:2] - mouth_w[:2])))
            if nut_leg <= 6.0 * MM and skirt < self.leg_top + 1.0 * MM and skirt > self.leg_top - 12.0 * MM:
                push = min(20.0, 3000.0 * (self.leg_top + 0.5 * MM - skirt) - 2.0 * float(fn.vel[2]))
                if push > 0.0:
                    ptrapkit._wrench(acc, self.nut, fn, np.array([0.0, 0.0, push]), fn.pos)
                    ptrapkit._wrench(acc, "jbend", fj, np.array([0.0, 0.0, -push]), mouth_w + o)
                if (skirt <= self.leg_top + 1.0 * MM and self.on_pipe() and self.radial <= self.ALIGN
                        and nut_leg <= self.ALIGN + 1.0 * MM and self.tilt <= self.TILT_ON):
                    self.threaded, self.theta, self.hand = True, 0.0, None
        if self.threaded:
            # the hand that grips it turns it
            z = self._nut_pose_z()
            gripping = None
            for side in ("left", "right"):
                h = frames.get(f"hand_{side}")
                if h is not None and self._gripping(_shift(h, o), z):
                    gripping = side
                    break
            if gripping is None:
                self.hand = None
            else:
                h = frames[f"hand_{gripping}"]
                heading = math.atan2(float(h.closing[1]), float(h.closing[0]))
                if self.hand == gripping:
                    turn = -_wrap(heading - self.heading)  # clockwise from above tightens
                    self.theta = min(self.theta + turn, SNUG_TURNS * 2 * math.pi)
                self.hand, self.heading = gripping, heading
            if self.theta < -0.1:
                self.threaded, self.caught = False, False  # backed off the thread
            self.peak_turns = max(self.peak_turns, self.turns())
            if self.threaded and not self.caught and self.turns() >= self.CATCH_TURNS:
                self.caught, self.catch_z = True, self.leg_top
                self.catch_xy = mouth_w[:2].copy()  # held where it is (on the pipe), not pulled onto the exact axis
            if self.threaded:
                self.nut_z = self._nut_pose_z()
                model.poses[self.nut] = (np.array([*self.axis, self.nut_z]) + o, (0.0, 0.0, math.sin(-self.theta / 2),
                                                                                  math.cos(-self.theta / 2)))
        if self.caught:
            # the thread holds the leg on the pipe
            v = fj.point_vel(mouth_w + o)
            f = np.array([*(-2000.0 * (mouth_w[:2] - self.catch_xy) - 20.0 * v[:2]),
                          -2000.0 * (self.leg_top - self.catch_z) - 20.0 * float(v[2])])
            n = float(np.linalg.norm(f))
            if n > 30.0:
                f *= 30.0 / n
            ptrapkit._wrench(acc, "jbend", fj, f, mouth_w + o)
            # and square on it (a started nut holds a 140 g J-bend within about a degree)
            # (explicit damping: kept under 2 I/dt, about 0.17 N m s here, or it diverges)
            ptrapkit._torque(acc, "jbend", 8.0 * np.cross(up, [0.0, 0.0, 1.0]) - 0.08 * np.asarray(fj.spin))


def _shift(h, o):
    return ptrapkit.Hand(tcp=np.asarray(h.tcp) - o, approach=h.approach, closing=h.closing, gap=h.gap)


class TrapModel:
    def __init__(self):
        self.joints = [SlipJoint("inlet", "nut_in", (XI, YI), TAIL_END, (0.0, 0.0, IN_TOP)),
                       SlipJoint("outlet", "nut_out", (XO, YO), OUT_END, (0.0, LEG_C, OUT_TOP))]
        self.origin = np.zeros(3)
        self.dt = 1.0 / 240.0
        self.poses = {}

    @property
    def bodies(self):
        return ["jbend", "nut_in", "nut_out"]

    def clear(self):
        for j in self.joints:
            j.clear()
        self.poses = {}
        self.n = 0
        self._last = None

    def update(self, frames, acc):
        self.poses = {}
        for j in self.joints:
            j.origin = self.origin
            j.update(frames, acc, self)
        self.n = getattr(self, "n", 0) + 1
        if TRACE and self.n % 60 == 0:
            parts = []
            for side in ("left", "right"):
                h = frames.get(f"hand_{side}")
                if h is not None:
                    t = (np.asarray(h.tcp) - self.origin) * 1e3
                    parts.append("%s tcp(%.0f,%.0f,%.0f) gap%.0f head%.0f" % (
                        side, t[0], t[1], t[2] - TOP * 1e3, h.gap * 1e3,
                        math.degrees(math.atan2(float(h.closing[1]), float(h.closing[0])))))
            fo = frames.get("nut_out")
            if fo is not None:
                q = (fo.pos - self.origin) * 1e3
                parts.append("nut_out(%.0f,%.0f,%.0f)" % (q[0], q[1], q[2] - TOP * 1e3))
            print("HAND t=%.2f " % (self.n / 240.0) + " | ".join(parts), flush=True)
        if TRACE:
            state = tuple((j.threaded, j.caught, j.snug(), j.on_pipe()) for j in self.joints)
            if state != getattr(self, "_last", None) or self.n % 240 == 0:
                self._last = state
                print("TRAP t=%.2f " % (self.n / 240.0) + " | ".join(
                    "%s in%.1f r%.1f tilt%.1f nut%.1f thr%d turns%.2f caught%d hand=%s" % (
                        j.label, j.inside * 1e3, j.radial * 1e3, j.tilt, (j.nut_z - j.leg_top) * 1e3, j.threaded,
                        j.turns(), j.caught, j.hand) for j in self.joints), flush=True)


TRACE = os.environ.get("TRAP_TRACE", "0") == "1"
MODEL = ptrapkit.add_model(TrapModel())
INLET, OUTLET = MODEL.joints

# -- scoring -------------------------------------------------------------------------------------------------------
SCORE_OVER = "end"
RUN_LIMIT_STEPS = 3600
CLEAR = 10.0 * MM
STILL_STEPS = 20  # 1 s at 20 Hz
_STATE = {}


@ptrapkit.on_reset
def _reset_state():
    _STATE.update(still=0, hist=[])


_reset_state()


def fingers_to(state, spec):
    """Nearest distance (m) from either gripper's hulls to a body's bounding box (in its own frame). inf offline."""
    pts = ptrapkit.finger_points()
    if not len(pts):
        return math.inf
    lo, hi = spec.bounds_local()
    local = (pts - state.pos) @ state.rotation()
    d = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


def tiers(objects):
    """The score's parts, as in the README: (seated on both pipes, per nut started a turn, per nut snug, hands off)."""
    seated = INLET.on_pipe() and OUTLET.on_pipe()
    started = [j.threaded and j.turns() >= 1.0 for j in (INLET, OUTLET)]
    snug = [j.snug() for j in (INLET, OUTLET)]
    done = seated and all(snug)
    hands_off = False
    if done:
        clear = min(fingers_to(objects[n], spec) for n, spec in (("jbend", JBEND), ("nut_in", NUT_IN),
                                                                 ("nut_out", NUT_OUT)))
        hands_off = clear >= CLEAR and _still()
    return seated, started, snug, hands_off


def _still():
    """The J-bend has not moved: its origin within 2 mm over the last second (as a person judges it by eye; with
    contacts held under load PhysX reports a small steady velocity on a body that does not move)."""
    h = _STATE.get("hist", [])
    if len(h) < STILL_STEPS:
        return False
    pts = np.array(h[-STILL_STEPS:])
    return bool(np.linalg.norm(pts - pts[-1], axis=1).max() < 2.0 * MM)


def _score(objects):
    seated, started, snug, hands_off = tiers(objects)
    s = 0.2 * seated + sum(0.15 * a + 0.15 * b for a, b in zip(started, snug))
    if TRACE and seated and all(snug):
        _STATE["dbg"] = _STATE.get("dbg", 0) + 1
        if _STATE["dbg"] % 20 == 1:
            j = objects["jbend"]
            clear = min(fingers_to(objects[n], spec) for n, spec in (("jbend", JBEND), ("nut_in", NUT_IN),
                                                                     ("nut_out", NUT_OUT)))
            print("DONE clear %.1f mm jbend still %s hands_off %s held %d" % (
                clear * 1e3, _still(), hands_off, _STATE.get("still", 0)), flush=True)
    return s, hands_off


def _remember(objects):
    h = _STATE.setdefault("hist", [])
    h.append(np.asarray(objects["jbend"].pos, float).copy())
    del h[:-STILL_STEPS - 5]


def success(objects):
    """Refitted: the J-bend on both pipes, both slip nuts hand-tight on its threads, nothing holding it but its nuts
    (no gripper within 10 mm of the J-bend or a nut, the J-bend still), held 1 s by the run. It also counts the
    steps that state has held, for score()."""
    _remember(objects)
    ok = _score(objects)[1]
    _STATE["still"] = _STATE["still"] + 1 if ok else 0
    return ok


def score(objects):
    """0.2 seated on both pipes + per nut 0.15 started (a full turn onto the thread) + 0.15 snug, + 0.2 once it hangs
    from its nuts alone (held 1 s, counted by success(), which the run calls first every step)."""
    s, hands_off = _score(objects)
    if hands_off and _STATE["still"] >= STILL_STEPS:
        s += 0.2
    return round(min(1.0, s), 4)


# -- scripted demo -------------------------------------------------------------------------------------------------
class SideGrip(float):
    """The gripper's approach level, pointing ``yaw`` rad from +x about the vertical, its pads closing level (across
    it); ``pitch`` tips the approach down. Passed as a Waypoint's yaw (a float), it turns the straight-down grip."""

    def __new__(cls, yaw=0.0, pitch=0.0):
        out = super().__new__(cls, 0.0)
        out.side_yaw, out.pitch = float(yaw), float(pitch)
        return out

    def matrix(self, down):
        def about(axis, angle):
            c, s = math.cos(angle), math.sin(angle)
            i, j = [(1, 2), (2, 0), (0, 1)][axis]
            m = np.eye(3)
            m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
            return m

        level = about(1, -math.pi / 2) @ down  # approach turned from straight down to +x; pads still closing along y
        return about(2, self.side_yaw) @ about(1, self.pitch) @ level


def _install_side_grips():
    original = _demo.grip_rotation
    if getattr(original, "side_grips", False):
        return

    def grip_rotation(yaw=0.0):
        if isinstance(yaw, SideGrip):
            return yaw.matrix(original(0.0))
        return original(yaw)

    grip_rotation.side_grips = True
    _demo.grip_rotation = grip_rotation


_install_side_grips()

TIP = 13.0 * MM  # the pad tips past the TCP
DEG = math.pi / 180
LHOME, RHOME = (0.20, 0.20, TOP + 0.30), (0.20, -0.20, TOP + 0.30)  # clear of the trap, still side grips
LHOME_YAW, RHOME_YAW = -40.0, 40.0
HOLD_LEG = FINGER_OPEN * 38.0 / 94.0  # across a 47 mm leg
HOLD_NUT = FINGER_OPEN * 54.0 / 94.0  # across a 62 mm nut
OPEN_NUT = FINGER_OPEN * 84.0 / 94.0  # open round it between strokes: clear of its 68 mm wings, and quick to close
# (the run's guard lets the gripper move 5% of its travel a step)
SAG = float(os.environ.get("TRAP_SAG", "0.004"))  # the left arm sags with the J-bend held out: aim this much higher
SEAT_EXTRA = float(os.environ.get("TRAP_SEAT_EXTRA", "0.009"))  # aim this much past its nominal seat: the arm sags
# ~6 mm pushing it up (probe 3: the pipes 24 and 22 mm in, not 30 and 25)
PRESS = float(os.environ.get("TRAP_PRESS", "0.006"))  # run each nut down this far past the thread's start (it stops)
STROKE = (float(os.environ.get("TRAP_Y0", "10")), float(os.environ.get("TRAP_Y1", "-80")))  # left hand's swing (deg)
R_STROKE = (float(os.environ.get("TRAP_RY0", "70")), float(os.environ.get("TRAP_RY1", "-20")))
STROKES = int(os.environ.get("TRAP_STROKES", "17"))  # 90 degrees each, of which the nut gets ~70-80 (the arm lags)
OVER = float(os.environ.get("TRAP_OVER", "8"))  # degrees past each stroke's end: the arm lags
T_CLOSE, T_SWING, T_HOLD, T_OPEN, T_OFF, T_BACK = 0.7, 1.4, 0.2, 0.6, 0.5, 1.6  # within the run's joint guards
R_OFF = 0.05  # the hand draws back this far off the nut to swing back
PERIOD = T_CLOSE + T_SWING + T_HOLD + T_OPEN + 2 * T_OFF + T_BACK


def _w(dur, pos, finger, label, yaw=0.0):
    return Waypoint(float(dur), tuple(float(v) for v in pos), float(finger), label, yaw)


def _t(ws):
    return sum(w.duration for w in ws)


def _back(pos, yaw_deg, d):
    a = yaw_deg * DEG
    return (pos[0] - d * math.cos(a), pos[1] - d * math.sin(a), pos[2])


def _strokes(axis_xy, z0, y0, y1, n, label, first_turns=0.0):
    """``n`` strokes on a nut whose middle starts at ``z0`` on the thread: grip it with the hand pointing ``y0``,
    swing it clockwise to ``y1``, let go, draw the hand back off it, swing back round it clear of it (the arm lags on a
    free swing, so the open fingers must not be near the nut), and come in again. The TCP follows the nut down."""
    out = []
    n0 = round(first_turns / 0.25)
    ya, yb = y0 * DEG, (y1 - OVER) * DEG
    ym = (ya + yb) / 2
    for k in range(n):
        t0 = first_turns + k * 0.25
        za, zb = z0 - PITCH * min(t0, SNUG_TURNS), z0 - PITCH * min(t0 + 0.25, SNUG_TURNS)
        at_a, at_b = (*axis_xy, za), (*axis_xy, zb)
        i = n0 + k + 1
        out += [
            _w(T_CLOSE, at_a, HOLD_NUT, f"{label}: grip {i}", SideGrip(ya)),
            _w(T_SWING, at_b, HOLD_NUT, f"{label}: turn {i}", SideGrip(yb)),
            _w(T_HOLD, at_b, HOLD_NUT, f"{label}: settle {i}", SideGrip(yb)),
            _w(T_OPEN, at_b, OPEN_NUT, f"{label}: let go {i}", SideGrip(yb)),
            _w(T_OFF, _back(at_b, yb / DEG, R_OFF), OPEN_NUT, f"{label}: off {i}", SideGrip(yb)),
            _w(T_BACK / 2, _back(at_b, ym / DEG, R_OFF), OPEN_NUT, f"{label}: swing back {i}", SideGrip(ym)),
            _w(T_BACK / 2, _back(at_b, ya / DEG, R_OFF), OPEN_NUT, f"{label}: swing back {i}", SideGrip(ya)),
            _w(T_OFF, at_b, OPEN_NUT, f"{label}: in {i}", SideGrip(ya)),
        ]
    return out


def demo():
    left, right = [], []
    # -- the left hand picks the J-bend up by its outlet leg and offers it up onto both pipes ---------------------
    grip_body_z = 0.072  # up the outlet leg from the bend's bottom
    gx, gy = J0_XY[0] + grip_body_z, J0_XY[1] + LEG_C  # lying: legs along +x, the outlet leg at +y
    gz = TOP + LEG_RO + 0.3 * MM  # the TCP on the leg's axis (the pad tips 11 mm above the floor), so that turned
    # upright the leg's axis is the TCP's (gripped with the tips at the axis, the axis ends up a tip's length ahead)
    left += [
        _w(2.0, (gx, gy, gz + 0.06), FINGER_OPEN, "over the J-bend's outlet leg"),
        _w(1.3, (gx, gy, gz), FINGER_OPEN, "round the leg"),
        _w(1.2, (gx, gy, gz), HOLD_LEG, "grip the leg"),
        _w(1.4, (gx, gy, TOP + 0.20), HOLD_LEG, "lift it"),
    ]
    # turned upright: the TCP holds the outlet leg grip_body_z up from the bend's bottom; the legs below the pipes
    hold_z = Z_JB + grip_body_z + SAG
    below = 0.040  # the legs' mouths this far below their seats, under the pipes' ends
    left += [
        _w(2.2, (XO - 0.12, YO, hold_z - below + 0.03), HOLD_LEG, "stand it upright", SideGrip(0.0)),
        _w(1.4, (XO - 0.12, YO, hold_z - below), HOLD_LEG, "down to below the pipes", SideGrip(0.0)),
        _w(1.8, (XO, YO, hold_z - below), HOLD_LEG, "in under the pipes", SideGrip(0.0)),
        _w(1.0, (XO, YO, hold_z - below), HOLD_LEG, "steady", SideGrip(0.0)),
        _w(2.2, (XO, YO, hold_z + SEAT_EXTRA), HOLD_LEG, "push it up onto both pipes", SideGrip(0.0)),
    ]
    t_seated = _t(left)
    # -- the right hand runs the inlet nut down and starts it while the left hand holds the J-bend ------------------
    r0, r1 = R_STROKE
    in_park = IN_NUT_PARK + NUT_H / 2
    in_start = Z_JB + IN_TOP + NUT_H / 2  # the nut's middle with its skirt's mouth on the thread's start
    right += [
        _w(3.0, _back((XI, YI, in_park + 0.03), r0, 0.10), FINGER_OPEN, "toward the inlet nut", SideGrip(r0 * DEG)),
        _w(max(0.5, t_seated - 3.0 - 1.8), _back((XI, YI, in_park + 0.03), r0, 0.10), FINGER_OPEN, "wait",
           SideGrip(r0 * DEG)),
        _w(1.8, _back((XI, YI, in_park), r0, 0.07), FINGER_OPEN, "beside the inlet nut", SideGrip(r0 * DEG)),
        _w(1.4, (XI, YI, in_park), FINGER_OPEN, "round the nut", SideGrip(r0 * DEG)),
        _w(1.0, (XI, YI, in_park), HOLD_NUT, "grip the nut", SideGrip(r0 * DEG)),
        _w(1.8, (XI, YI, in_start - PRESS), HOLD_NUT, "run it down onto the thread", SideGrip(r0 * DEG)),
        _w(0.6, (XI, YI, in_start - PRESS), HOLD_NUT, "press it on", SideGrip(r0 * DEG)),
    ]
    t_r_strokes = _t(right)
    first = 2  # two strokes: the inlet nut half a turn on, the J-bend hangs from it
    right += _strokes((XI, YI), in_start, r0, r1, first, "inlet nut")
    # -- the left hand lets go once the inlet nut holds the J-bend, and goes to the outlet nut while the right hand
    # waits at the start of its swing (its palm well to the right), so the hands never cross ------------------------
    l0, l1 = STROKE
    out_park = OUT_NUT_PARK + NUT_H / 2
    out_start = Z_JB + OUT_TOP + NUT_H / 2
    t_free = _t(right)
    left.append(_w(max(0.1, t_free - _t(left)), (XO, YO, hold_z + SEAT_EXTRA), HOLD_LEG, "hold it up", SideGrip(0.0)))
    left += [
        _w(1.2, (XO, YO, hold_z + SEAT_EXTRA), FINGER_OPEN, "let go of the J-bend", SideGrip(0.0)),
        _w(1.2, _back((XO, YO, hold_z + SEAT_EXTRA), 0.0, 0.08), FINGER_OPEN, "back out", SideGrip(0.0)),
        _w(1.4, _back((XO, YO, out_park), l0, 0.10), FINGER_OPEN, "up beside the outlet nut", SideGrip(l0 * DEG)),
        _w(0.8, _back((XO, YO, out_park), l0, 0.07), FINGER_OPEN, "lined up with it", SideGrip(l0 * DEG)),
        _w(1.2, (XO, YO, out_park), FINGER_OPEN, "round the nut", SideGrip(l0 * DEG)),
        _w(1.0, (XO, YO, out_park), HOLD_NUT, "grip the nut", SideGrip(l0 * DEG)),
        _w(1.4, (XO, YO, out_start - PRESS), HOLD_NUT, "run it down onto the thread", SideGrip(l0 * DEG)),
        _w(0.6, (XO, YO, out_start - PRESS), HOLD_NUT, "press it on", SideGrip(l0 * DEG)),
    ]
    z_r = right[-1].pos  # at the start of its swing, open: it backs off its nut while the left hand comes in
    right += [_w(0.8, _back(z_r, r0, 0.04), FINGER_OPEN, "back off while the left hand comes in", SideGrip(r0 * DEG)),
              _w(max(0.1, _t(left) - _t(right) - 0.8 - 0.8), _back(z_r, r0, 0.04), FINGER_OPEN, "wait for the left hand",
                 SideGrip(r0 * DEG)),
              _w(0.8, z_r, FINGER_OPEN, "back round its nut", SideGrip(r0 * DEG))]
    # then both swing in step: the two palms swing apart together
    right += _strokes((XI, YI), in_start, r0, r1, STROKES - first, "inlet nut", first_turns=first * 0.25)
    left += _strokes((XO, YO), out_start, l0, l1, STROKES, "outlet nut")
    # -- each lets go, backs out the way it came in and goes clear ------------------------------------------------------
    for ws, home, hy, xy, y0 in ((left, LHOME, LHOME_YAW, (XO, YO), l0), (right, RHOME, RHOME_YAW, (XI, YI), r0)):
        z = ws[-1].pos[2]
        ws += [_w(1.2, _back((*xy, z), y0, 0.08), FINGER_OPEN, "back out", SideGrip(y0 * DEG)),
               _w(2.0, home, FINGER_OPEN, "clear of the trap", SideGrip(hy * DEG))]
    end = max(_t(left), _t(right)) + 2.0
    left.append(_w(end - _t(left), LHOME, FINGER_OPEN, "done", SideGrip(LHOME_YAW * DEG)))
    right.append(_w(end - _t(right), RHOME, FINGER_OPEN, "done", SideGrip(RHOME_YAW * DEG)))
    return {"left": left, "right": right}
