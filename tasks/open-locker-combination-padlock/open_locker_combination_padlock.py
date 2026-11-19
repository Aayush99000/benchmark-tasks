"""A combination padlock hanging on a locker's hasp. The lock-area section of a single-tier steel locker door stands
upright on the table, facing the robot; a 3 mm steel hasp tab sticks out of it. A real-size dial padlock of the Master Lock
No. 1500 type (round 48 mm body, 37 mm black dial printed with 40 marks and the numerals 0-35 under a red index, 18 mm
knurled knob, 7 mm arched shackle, 163 g) hangs free on the tab with its dial facing the robot. Inside it is a working three-gate wheel pack (two free wheels driven through pins by the dial's
drive cam, the cam carrying the third gate) and a spring-loaded shackle held by a ramped catch whose bolt can only
move when all three gates sit under its fence. The robot must dial the given combination by the maker's rule (right
past two full turns to the first number, left one full turn past it to the second, right to the third) and pull the
body down so the shackle springs open, leaving the lock hanging open on the hasp. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl
from yam_joints import Hinge, Link, Slide
from yam_mesh import Mesh, MeshMechanism
from yam_arena import demo as _motion
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"

# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ------------------------------------------------------------------ helpers
MM = 0.001


def _prism_mesh(name, profile_yz, x0, x1, color=None, proxy=()):
    """Closed convex prism: a convex (y, z) polygon in mm, extruded along x from x0 to x1 (mm), in the lock frame."""
    prof = [(float(y) * MM, float(z) * MM) for y, z in profile_yz]
    n = len(prof)
    pts = np.array([(x * MM, y, z) for x in (x0, x1) for (y, z) in prof], float)
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    for k in range(1, n - 1):
        faces += [(0, k + 1, k), (n, n + k, n + k + 1)]
    faces = np.array(faces, int)
    tri = pts[faces]
    if np.einsum('ij,ij->i', tri[:, 0] - pts.mean(0), np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, pts, faces, collision="convex", color=color, proxy=proxy)


def _wrap(deg):
    return (float(deg) + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

COMBINATION = (24, 8, 31)
MARK_DEG = 9.0                      # 40 marks
TOL_DEG = 4.5                       # a gate counts as set within half a mark: the number has to be the one on the index
OPEN_WINDOW_DEG = 4.0               # the fence reaches release depth only with every gate within 4 degrees (see GATE_DEG)

# Lock frame: origin on the dial axis at the body's mid-thickness, axes = world axes (x into the door, y left, z up);
# the dial faces -x. The maker's photos show a round 48 mm body with the dial set about 3 mm below the body's centre.
BODY_R, BODY_T, BODY_CZ = 24.0, 19.0, 3.0          # mm: 48 mm round body, 19 mm thick, centre 3 mm above the dial axis
BODY_W, BODY_H = 2 * BODY_R, 2 * BODY_R
BODY_TOP = BODY_CZ + BODY_R                        # 27 mm above the dial axis
DISC_R, DISC_PROUD = 18.5, 1.0                     # 37 mm black numbered dial disc, 1 mm proud of the body face
KNOB_R, KNOB_H, CAP_R, CAP_H = 9.0, 6.0, 7.5, 1.0  # 18 mm knurled knob on the disc, capped 1 mm
DIAL_PROUD = DISC_PROUD + KNOB_H + CAP_H           # 8 mm from the body face to the knob's front
DIAL_R = KNOB_R                                    # the knob is what a hand (or the gripper) turns
SHACKLE_R, SHACKLE_HALF_SPAN = 3.5, 14.0           # 7 mm rod; leg centres 28 mm apart (21 mm inside width)
SHACKLE_CLEAR = 19.0                               # inside vertical clearance, from the body's top to the arch
INSIDE_TOP = BODY_TOP + SHACKLE_CLEAR              # 46 mm: the arch's inside top
ARCH_ZC = INSIDE_TOP - (SHACKLE_HALF_SPAN - SHACKLE_R)   # centre of the semicircular arch (centre-line radius 14)
BAR_Z = INSIDE_TOP + SHACKLE_R                     # the arch's top centre-line point, 49.5 mm above the dial axis
SHORT_LEG_END = 18.0                               # short leg's end (with its catch), 4.5 mm inside the body's curve
SHACKLE_TRAVEL = 0.012                             # shackle springs up 12 mm when unlocked
OPEN_Q = 0.008                                     # open: shackle up 8 mm (short leg end out of the body)

LOCK_X, LOCK_Y = 0.2395, -0.060
LOCK_Z = TOP + 0.145
LOCK_POS = (LOCK_X, LOCK_Y, LOCK_Z)
BODY_FACE_X = LOCK_X - BODY_T / 2 * MM             # 0.230
DOOR_X = LOCK_X + 0.015                            # hasp hole centre 15 mm out from the door face
HOLE_BOTTOM = LOCK_Z + INSIDE_TOP * MM             # the shackle's inside top rests on the hole's bottom
HOLE = 0.010                                       # 10 mm hole
TAB_T = 0.003
TAB_OUT = 0.025

# Locker door section (fixed): 1 mm painted steel face with 25 mm returns, on a steel foot behind it.
DOOR_Y0, DOOR_Y1 = LOCK_Y - 0.150, LOCK_Y + 0.150
DOOR_H = 0.250
_hx0 = DOOR_X - TAB_OUT
_hz0 = HOLE_BOTTOM - TOP
DOOR_SHAPES = (
    box((DOOR_X + 0.0005, LOCK_Y, DOOR_H / 2), (0.001, DOOR_Y1 - DOOR_Y0, DOOR_H)),                  # face sheet
    box((DOOR_X + 0.0130, DOOR_Y0 + 0.0005, DOOR_H / 2), (0.025, 0.001, DOOR_H)),                     # side returns
    box((DOOR_X + 0.0130, DOOR_Y1 - 0.0005, DOOR_H / 2), (0.025, 0.001, DOOR_H)),
    box((DOOR_X + 0.060, LOCK_Y, 0.004), (0.090, DOOR_Y1 - DOOR_Y0, 0.008)),                         # foot
    # Hasp tab: 3 mm steel, 25 mm out from the face, 10 mm hole (axis along y) centred 15 mm out.
    box(((_hx0 + DOOR_X) / 2, LOCK_Y, _hz0 - 0.003), (TAB_OUT, TAB_T, 0.006)),                       # below the hole
    box(((_hx0 + DOOR_X) / 2, LOCK_Y, _hz0 + HOLE + 0.003), (TAB_OUT, TAB_T, 0.006)),                # above
    box((_hx0 + 0.0025, LOCK_Y, _hz0 + HOLE / 2), (0.005, TAB_T, HOLE)),                              # front post
    box((DOOR_X - 0.005, LOCK_Y, _hz0 + HOLE / 2), (0.010, TAB_T, HOLE)),                             # back post
)
DOOR_POS = (DOOR_X, LOCK_Y, TOP)
DOOR = Part("locker_door", DOOR_POS, tuple(box(np.asarray(s.center) - [DOOR_X, LOCK_Y, 0.0], s.size)
                                           for s in DOOR_SHAPES),
            color=(0.42, 0.50, 0.58), friction=(0.3, 0.25), contact_offset=0.001)

# ------------------------------------------------------------------ the padlock (free MeshMechanism)
# Wheel pack along the dial axis (x, mm): drive cam disc -8..-6, its pin -6..-2.6; wheel 2 front pin -5.4..-2,
# disc -2..0, back pin 0..3.4; wheel 1 front pin 0.6..4, disc 4..6. Pins 2 mm wide at radius 5.5..8.5 mm.
# Rims radius 12 mm, gates 3 mm deep with radial sides. The fence sits at +y (angle 0, measured from +y toward +z,
# the right-hand sense about +x). Turning RIGHT (clockwise seen from the robot) is +angle about +x and brings lower
# numbers to the index at the top: number = (-angle / 9) mod 40.
# Gate width: the 1.2 mm fence (inner face at 12.8 mm radius when idle) must go 2.6 mm deep, to 10.2 mm radius, before
# the bolt's nose clears the catch (nose edge at 9.6 mm, catch face at 7.0 mm). A fence corner there sits
# atan(0.6 / 10.2) = 3.4 degrees off the fence line, so a gate whose sides are OPEN_WINDOW_DEG + 3.4 degrees either side
# of its centre lets the fence reach release depth only while that gate is within OPEN_WINDOW_DEG of the fence.
FENCE_HALF_T, FENCE_R_IDLE, RELEASE_DEPTH = 0.6, 12.8, 2.6       # mm
FENCE_R_RELEASE = FENCE_R_IDLE - RELEASE_DEPTH                     # 10.2 mm
RIM_R, HUB_R = 12.0, 9.0
GATE_DEG = 2 * (OPEN_WINDOW_DEG + math.degrees(math.atan(FENCE_HALF_T / FENCE_R_RELEASE)))   # 14.7 degrees
PIN_R0, PIN_R1, PIN_W = 5.5, 8.5, 2.0
PIN_DEG = 20.6                                    # pin-centre angle at contact (inner corners, computed below)
DISCS = {"dial": (-8.0, -6.0), "wheel2": (-2.0, 0.0), "wheel1": (4.0, 6.0)}


def _pin_contact_deg():
    # Two 2 x 3 mm pins at radius 5.5..8.5 touch when 5.5 sin(d) - cos(d) = 1 (trailing inner corner on the face).
    lo, hi = 0.0, 60.0
    for _ in range(60):
        mid = (lo + hi) / 2
        f = PIN_R0 * math.sin(math.radians(mid)) - (PIN_W / 2) * math.cos(math.radians(mid)) - PIN_W / 2
        lo, hi = (mid, hi) if f < 0 else (lo, mid)
    return lo


A_PIN = _pin_contact_deg()


def set_angles(comb=COMBINATION, a=None):
    """Joint angles (deg, mod 360) of wheel 1, wheel 2 and the cam when the combination has been dialled by the rule."""
    a = A_PIN if a is None else a
    n1, n2, n3 = comb
    return {"wheel1": (-MARK_DEG * n1 + 2 * a) % 360.0, "wheel2": (-MARK_DEG * n2 - a) % 360.0,
            "dial": (-MARK_DEG * n3) % 360.0}


SET = set_angles()
GATE_PSI = {k: (-v) % 360.0 for k, v in SET.items()}   # gate angle in each link's own frame (at q = 0)
# Start = the reset: dial spun LEFT at least three turns and stopped at 0, so every pin is engaged on the left side
# (wheel 2 one pin-contact angle behind the cam, wheel 1 two), with 0.9 deg of spawn clearance at each pin pair. From
# here the first number needs the full right turns: the cam picks up wheel 2 after ~318 deg and wheel 2 picks up wheel 1
# after ~318 deg more.
START_Q = {"dial": 0.0, "wheel2": -(A_PIN + 0.9), "wheel1": -2 * (A_PIN + 0.9)}


def rim_profiles(link):
    """The rim's convex (y, z) pieces in mm, in the link's own frame: the whole rim but the gate."""
    g = GATE_PSI[link]
    profiles = []
    span = 360.0 - GATE_DEG
    n = 8
    for k in range(n):
        a0 = g + GATE_DEG / 2 + span * k / n
        a1 = g + GATE_DEG / 2 + span * (k + 1) / n
        arc = [(RIM_R * math.cos(math.radians(a)), RIM_R * math.sin(math.radians(a)))
               for a in np.linspace(a0, a1, 7)]
        inner = [(HUB_R * 0.8 * math.cos(math.radians(a)), HUB_R * 0.8 * math.sin(math.radians(a))) for a in (a1, a0)]
        profiles.append(arc + inner)
    return profiles


def _rim_meshes(link, color):
    x0, x1 = DISCS[link]
    return [_prism_mesh(f"{link}_rim{k}", prof, x0, x1, color=color) for k, prof in enumerate(rim_profiles(link))]


def _pin(x0, x1, color=None):
    return box(((x0 + x1) / 2 * MM, 0.0, (PIN_R0 + PIN_R1) / 2 * MM), ((x1 - x0) * MM, PIN_W * MM, (PIN_R1 - PIN_R0) * MM),
               color)


BLACK = (0.05, 0.05, 0.06)
STEEL = (0.72, 0.73, 0.75)
BRASS = (0.70, 0.55, 0.25)
WHITE = (0.95, 0.95, 0.95)
_c = lambda x, y, z: (x * MM, y * MM, z * MM)

RED = (0.80, 0.08, 0.06)
CHROME = (0.62, 0.63, 0.66)
FACE_X = -BODY_T / 2 - DISC_PROUD                  # the numbered disc's face, mm (lock frame)
KNOB_FRONT_X = FACE_X - KNOB_H - CAP_H
def _round_mesh(name, r, cz, x0, x1, color, sides):
    """A smooth round disc (convex prism, `sides` facets) on an axis along x through (y 0, z cz), x0..x1 in mm; its
    cylinder primitive is the offline proxy. Primitive cylinders render with only ten facets."""
    prof = [(r * math.cos(a), cz + r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, sides, endpoint=False)]
    return _prism_mesh(name, prof, x0, x1, color=color,
                       proxy=(cyl(_c((x0 + x1) / 2, 0, cz), r * MM, (x1 - x0) * MM, axis="x", color=color),))


BODY_MESH = _round_mesh("body", BODY_R, BODY_CZ, -BODY_T / 2, BODY_T / 2, STEEL, 72)   # round stainless-covered body
DIAL_MESHES = (
    _round_mesh("disc", DISC_R, 0.0, FACE_X, -BODY_T / 2, BLACK, 64),                      # numbered disc
    _round_mesh("cap", CAP_R, 0.0, KNOB_FRONT_X, KNOB_FRONT_X + CAP_H, BLACK, 40),         # its black cap
)
BODY_SHAPES = (
    box(_c(0, 0, -15.0), _c(4.0, 4.0, 4.0), STEEL),     # the body's inner frame below the wheel pack (out of sight)
)
DIAL_SHAPES = (
    cyl(_c(FACE_X - KNOB_H / 2, 0, 0), KNOB_R * MM, KNOB_H * MM, axis="x", color=CHROME),   # knurled knob (the pads
    #     pinch a true cylinder; a faceted mesh knob made the pinch slip on a GPU run)
    box(_c(KNOB_FRONT_X - 0.2, 0, CAP_R - 1.6), _c(0.4, 1.0, 2.4), WHITE),     # the knob's zero mark
    cyl(_c(-7.0, 0, 0), HUB_R * MM, 2.0 * MM, axis="x", color=BRASS),       # drive-cam hub
    _pin(-6.0, -2.6, BRASS),
)
WHEEL2_SHAPES = (cyl(_c(-1.0, 0, 0), HUB_R * MM, 2.0 * MM, axis="x", color=BRASS), _pin(-5.4, -2.0, BRASS),
                 _pin(0.0, 3.4, BRASS))
WHEEL1_SHAPES = (cyl(_c(5.0, 0, 0), HUB_R * MM, 2.0 * MM, axis="x", color=BRASS), _pin(0.6, 4.0, BRASS))

# Shackle: two straight legs and a semicircular arch (centre-line radius 14 mm), as on the real lock.
_long_leg_z0, _short_leg_z0 = 9.0, SHORT_LEG_END
SHACKLE_SHAPES = (
    cyl(_c(0, -SHACKLE_HALF_SPAN, (_long_leg_z0 + ARCH_ZC) / 2), SHACKLE_R * MM, (ARCH_ZC - _long_leg_z0) * MM,
        axis="z", color=STEEL),
    cyl(_c(0, SHACKLE_HALF_SPAN, (_short_leg_z0 + ARCH_ZC) / 2), SHACKLE_R * MM, (ARCH_ZC - _short_leg_z0) * MM,
        axis="z", color=STEEL),
)


def _closed_mesh(name, shells, color, proxy=(), collision="convex"):
    """One mesh from several closed shells, each given as (bottom ring, top ring) of equal length; every shell is
    oriented outward on its own. Points in mm, lock frame."""
    pts, faces = [], []
    for lo, hi in shells:
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        n, base = len(lo), len(pts)
        f = []
        for i in range(n):
            j = (i + 1) % n
            f += [(i, j, n + j), (i, n + j, n + i)]
        for k in range(1, n - 1):
            f += [(0, k + 1, k), (n, n + k, n + k + 1)]
        f = np.array(f, int)
        p = np.vstack([lo, hi])
        tri = p[f]
        if np.einsum('ij,ij->i', tri[:, 0] - p.mean(0), np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])).sum() < 0:
            f = f[:, ::-1]
        pts += list(p)
        faces += list(f + base)
    return Mesh(name, np.array(pts) * MM, np.array(faces, int), collision=collision, color=color, proxy=proxy)


def _arch_shells(segments=11, sides=12):
    # An odd count puts a 16-degree facet at the arch's top: it rests on the hasp hole's bottom as a short flat (3 mm),
    # so a released lock rocks to rest instead of rolling on a frictionless circle.
    shells = []
    ring = lambda phi: [(SHACKLE_R * math.cos(b),
                         SHACKLE_HALF_SPAN * math.cos(phi) + SHACKLE_R * math.sin(b) * math.cos(phi),
                         ARCH_ZC + SHACKLE_HALF_SPAN * math.sin(phi) + SHACKLE_R * math.sin(b) * math.sin(phi))
                        for b in np.linspace(0, 2 * math.pi, sides, endpoint=False)]
    for k in range(segments):
        shells.append((ring(math.pi * k / segments), ring(math.pi * (k + 1) / segments)))
    return shells


ARCH = [_closed_mesh(f"shackle_arch{k}", [s], STEEL,
                     proxy=(box(_c(0, 0, ARCH_ZC + (SHACKLE_HALF_SPAN + SHACKLE_R) / 2),
                                _c(2 * SHACKLE_R, 2 * (SHACKLE_HALF_SPAN + SHACKLE_R), SHACKLE_HALF_SPAN + SHACKLE_R)),)
                     if k == 0 else ())
        for k, s in enumerate(_arch_shells())]

# ------------------------------------------------------------------ the printed dial face (Master Lock No. 1500 layout)
# Seen from the robot the numbers run clockwise: 0 at the top, 10 at three o'clock, 20 at the bottom, 30 at nine
# o'clock. A tick at every number (longer at every fifth), white numerals 0, 5 ... 35 standing on the radius with their
# tops toward the rim (20 reads upside down), and a red index on the body above the dial. Marks are 0.15 mm thick
# white print on the black disc; they are drawn only (no collision) and lie inside the disc, clear of the knob.
PRINT_T = 0.15
TICK_R0_LONG, TICK_R0, TICK_R1, TICK_W = 15.0, 16.3, 18.1, 0.7
NUM_R, NUM_H, NUM_STROKE = 12.6, 3.6, 0.75         # numeral centre radius, height and stroke (mm)
_GLYPHS = {   # stroke font in a unit-height box, x from 0 to 0.62
    "0": [[(0.31, 1), (0.53, 0.92), (0.62, 0.68), (0.62, 0.32), (0.53, 0.08), (0.31, 0), (0.09, 0.08), (0, 0.32),
           (0, 0.68), (0.09, 0.92), (0.31, 1)]],
    "1": [[(0.12, 0.78), (0.36, 1), (0.36, 0)]],
    "2": [[(0.02, 0.8), (0.14, 0.96), (0.31, 1), (0.5, 0.95), (0.6, 0.78), (0.56, 0.6), (0, 0), (0.62, 0)]],
    "3": [[(0.02, 0.9), (0.2, 1), (0.45, 1), (0.6, 0.86), (0.58, 0.66), (0.42, 0.54), (0.22, 0.54)],
          [(0.42, 0.54), (0.6, 0.42), (0.62, 0.18), (0.48, 0.02), (0.22, 0), (0.0, 0.1)]],
    "5": [[(0.58, 1), (0.08, 1), (0.04, 0.56), (0.32, 0.62), (0.54, 0.54), (0.62, 0.3), (0.52, 0.06), (0.3, 0),
           (0.0, 0.1)]],
}


def _screen_to_lock(sx, sy):
    """Robot's view of the dial face (screen right, screen up) in mm -> lock-frame (y, z)."""
    return -sx, sy


def _strip_shell(p, q, width, x_face):
    """A flat print stroke from p to q (screen mm), square-capped, lying on the face at x_face."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    d = d / max(np.linalg.norm(d), 1e-9)
    nrm = np.array([-d[1], d[0]]) * width / 2
    a, b = p - d * width / 2, q + d * width / 2
    quad = [a - nrm, b - nrm, b + nrm, a + nrm]
    lo = [(x_face + 0.05, *_screen_to_lock(*c)) for c in quad]
    hi = [(x_face - PRINT_T, *_screen_to_lock(*c)) for c in quad]
    return lo, hi


def dial_marks():
    """Shells of the printed face (lock frame at dial angle 0) plus, per number, where its tick and label sit."""
    shells, where = [], {}
    for n in range(40):
        th = math.radians(MARK_DEG * n)                    # clockwise from the top, seen from the robot
        u = np.array([math.sin(th), math.cos(th)])         # outward, screen coordinates
        r0 = TICK_R0_LONG if n % 5 == 0 else TICK_R0
        shells.append(_strip_shell(u * r0, u * TICK_R1, TICK_W, FACE_X))
        where[n] = {"tick": tuple(_screen_to_lock(*(u * (r0 + TICK_R1) / 2)))}
        if n % 5:
            continue
        right = np.array([math.cos(th), -math.sin(th)])    # the numeral's own "right", tops toward the rim
        text = str(n)
        w = 0.62 * NUM_H
        gap = 0.18 * NUM_H
        total = len(text) * w + (len(text) - 1) * gap
        centre = u * NUM_R
        for i, ch in enumerate(text):
            x0 = -total / 2 + i * (w + gap)
            for line in _GLYPHS[ch]:
                pts = [centre + right * (x0 + gx * NUM_H) + u * (gy - 0.5) * NUM_H for gx, gy in line]
                for a, b in zip(pts[:-1], pts[1:]):
                    shells.append(_strip_shell(a, b, NUM_STROKE, FACE_X))
        where[n]["label"] = tuple(_screen_to_lock(*centre))
    return shells, where


_FACE_SHELLS, DIAL_MARKS = dial_marks()
PRINT_FACE = _closed_mesh("print_face", _FACE_SHELLS, WHITE)
_ix = -BODY_T / 2
PRINT_INDEX = _closed_mesh("print_index", [(
    [(_ix + 0.05, *_screen_to_lock(*c)) for c in ((0.0, DISC_R + 0.6), (1.3, DISC_R + 3.0), (-1.3, DISC_R + 3.0))],
    [(_ix - PRINT_T, *_screen_to_lock(*c)) for c in ((0.0, DISC_R + 0.6), (1.3, DISC_R + 3.0), (-1.3, DISC_R + 3.0))])],
    RED)


def number_at_index(dial_deg):
    """The dial reading under the index for a dial joint angle (degrees): numbers rise clockwise, turning right is +."""
    return (-dial_deg / MARK_DEG) % 40.0


# Print is drawn only: the task gives its "print_" meshes (and the black painted disc and cap) a matte paint look and
# switches the print's collision off when the simulator authors them (the kit files are untouched; a failure here leaves 0.15 mm print that nothing reaches).
_PAINTED = ("disc", "cap")          # the black painted dial and knob cap: matte, not polished metal
_ym = sys.modules[MeshMechanism.__module__]
_kit_author_mesh = getattr(_ym._author_mesh, "_padlock_original", _ym._author_mesh)


def _author_mesh(stage, parent, mesh, anchor, color, contact_offset):
    _kit_author_mesh(stage, parent, mesh, anchor, color, contact_offset)
    if not (mesh.name.startswith("print_") or mesh.name in _PAINTED):
        return
    try:
        import isaaclab.sim as sim_utils
        from pxr import UsdPhysics
        from yam_arena.station import linear_rgb
        path = f"{parent}/mesh_{mesh.name}"
        prim = stage.GetPrimAtPath(path)
        if mesh.name.startswith("print_"):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(mesh.color), roughness=0.55, metallic=0.0)
        look.func(f"{path}_paint", look)
        sim_utils.bind_visual_material(path, f"{path}_paint", stage=stage)
    except Exception as exc:  # pragma: no cover - simulator only
        print(f"padlock: print look not applied to {mesh.name}: {exc}", flush=True)


_author_mesh._padlock_original = _kit_author_mesh
_ym._author_mesh = _author_mesh
# Ramped catch at the short leg's end (45 degrees) and the bolt's nose that sits on it; the bolt's fence (+y side
# of the wheel pack) can move in only when all three gates are under it.
CATCH = _prism_mesh("catch", ((7, 12), (14, 12), (14, 18), (11, 18), (7, 14)), -3.5, 3.5, color=STEEL)
NOSE = _prism_mesh("nose", ((3, 14), (6.6, 14), (9.6, 17), (9.6, 20), (3, 20)), -3.0, 9.0, color=BRASS)
CONNECTOR = _prism_mesh("bolt_arm", ((12.8, -0.6), (15.5, -0.6), (15.5, 1.0), (9.6, 20), (3, 20), (3, 14)), 7.0, 9.0,
                        color=BRASS)
FENCE = box(_c(-0.5, (FENCE_R_IDLE + 15.5) / 2, 0.0), _c(16.0, 15.5 - FENCE_R_IDLE, 2 * FENCE_HALF_T), BRASS)  # y 12.8..15.5

BOLT_PRELOAD = 4.9            # N, bolt spring at its idle stop
BOLT_K = 300.0
SHACKLE_PRELOAD = 1.5         # N, shackle spring with the shackle closed
SHACKLE_K = 50.0
DIAL_DRAG = 0.008             # N m, static (dynamic 75 %)
WHEEL_DRAG = 0.002            # N m each (wheel-pack friction washer)
TURN_LIMIT = 3600.0           # degrees either way (10 turns)

LOCK = MeshMechanism(
    "padlock", LOCK_POS, BODY_SHAPES,
    [Link("dial", DIAL_SHAPES,
          Hinge("x", (0.0, 0.0, 0.0), -TURN_LIMIT, TURN_LIMIT, damping=1e-4, initial=START_Q["dial"], armature=2e-6,
                static_friction=DIAL_DRAG, dynamic_friction=0.75 * DIAL_DRAG),
          mass=0.012, color=BLACK, friction=(1.0, 0.9)),
     Link("wheel2", WHEEL2_SHAPES,
          Hinge("x", (0.0, 0.0, 0.0), -TURN_LIMIT, TURN_LIMIT, damping=1e-5, initial=START_Q["wheel2"], armature=1e-6,
                static_friction=WHEEL_DRAG, dynamic_friction=0.75 * WHEEL_DRAG),
          mass=0.003, color=BRASS, friction=(0.2, 0.15)),
     Link("wheel1", WHEEL1_SHAPES,
          Hinge("x", (0.0, 0.0, 0.0), -TURN_LIMIT, TURN_LIMIT, damping=1e-5, initial=START_Q["wheel1"], armature=1e-6,
                static_friction=WHEEL_DRAG, dynamic_friction=0.75 * WHEEL_DRAG),
          mass=0.003, color=BRASS, friction=(0.2, 0.15)),
     Link("shackle", SHACKLE_SHAPES,
          Slide("z", (0.0, 0.0, 0.0), 0.0, SHACKLE_TRAVEL, damping=0.5, spring=SHACKLE_K,
                rest=SHACKLE_PRELOAD / SHACKLE_K, armature=1e-4),
          mass=0.030, color=STEEL, friction=(0.1, 0.1)),
     Link("bolt", (FENCE,),
          Slide("y", (0.0, 0.0, 0.0), -0.0035, 0.0, damping=0.5, spring=BOLT_K, rest=BOLT_PRELOAD / BOLT_K,
                armature=1e-4),
          mass=0.003, color=BRASS, friction=(0.1, 0.1))],
    mass=0.112, color=STEEL, friction=(0.3, 0.25), contact_offset=0.0005, self_collide=True, grip=None,
    meshes={"base": (BODY_MESH, PRINT_INDEX), "dial": DIAL_MESHES + tuple(_rim_meshes("dial", BRASS)) + (PRINT_FACE,),
            "wheel2": tuple(_rim_meshes("wheel2", BRASS)), "wheel1": tuple(_rim_meshes("wheel1", BRASS)),
            "shackle": (CATCH, *ARCH), "bolt": (NOSE, CONNECTOR)})
LOCK_MASS = 0.163
PRESSED_PARTS = ("padlock",)
OBJECTS = [DOOR, LOCK]

# ------------------------------------------------------------------ scoring
FELL_DROP = 0.035             # sticky zero: the lock body more than 35 mm below where it hung (off the hasp)
ON_HASP_XY = 0.012            # on the hasp: the shackle's top bar still through the hole
SETTLE_M, SETTLE_DEG = 0.005, 5.0   # at rest: body centre within 5 mm and its tilt within 5 deg over the 1 s window
FORCED_DEG = 9.0              # sticky zero: shackle open while a gate is more than one mark off (forced open)
TEST_STICKY = ()


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def gate_offsets(lock):
    """Each gate's offset from the fence in degrees (wrapped to -180..180)."""
    return {k: _wrap(GATE_PSI[k] + lock.angle_deg(k)) for k in ("wheel1", "wheel2", "dial")}


def gates_set(lock):
    off = gate_offsets(lock)
    return [abs(off[k]) <= TOL_DEG for k in ("wheel1", "wheel2", "dial")]


def shackle_open(lock):
    return lock.joints["shackle"] >= OPEN_Q


def on_hasp(lock):
    bar = np.asarray(lock.link_to_world("shackle", _c(0, 0, BAR_Z)), float)
    hole = np.array([DOOR_X - 0.015, LOCK_Y, HOLE_BOTTOM + SHACKLE_R * MM])
    return bool(abs(bar[0] - hole[0]) <= ON_HASP_XY and abs(bar[1] - hole[1]) <= ON_HASP_XY
                and abs(bar[2] - hole[2]) <= ON_HASP_XY)


_HIST = {"episode": None, "poses": []}


def _tilt_deg(lock):
    """Angle between the body's up axis and the world vertical, signed pair (about x, about y)."""
    up = lock.rotation() @ np.array([0.0, 0.0, 1.0])
    return np.degrees([math.atan2(-up[1], up[2]), math.atan2(up[0], up[2])])


def lock_settled(lock, seconds):
    """A hanging lock is at rest when, over the last STILL_S of `seconds`, its body centre stayed within SETTLE_M
    and its tilt within SETTLE_DEG (a small residual sway on the hasp is allowed; a lock being moved is not)."""
    if _RUN["t"] is None:
        return lock.resting()
    if _HIST["episode"] != _RUN["episode"]:
        _HIST.update(episode=_RUN["episode"], poses=[])
    t = float(_RUN["t"])
    poses = _HIST["poses"]
    if not poses or poses[-1][0] != t:
        poses.append((t, np.asarray(lock.origin, float), _tilt_deg(lock)))
    window = [p for p in poses if p[0] >= t - min(seconds, STILL_S) - 1e-9]
    del poses[:max(0, len(poses) - 200)]
    if seconds + 1e-9 < STILL_S:
        return False
    xyz = np.array([p[1] for p in window])
    ang = np.array([p[2] for p in window])
    return bool(np.all(np.ptp(xyz, axis=0) <= SETTLE_M) and np.all(np.ptp(ang, axis=0) <= SETTLE_DEG))


# The latest dial angle, kept in the run context shared by every copy of this module (the scorer's and the scripted
# policy's), for the closed-loop demo grips.
_DIAL = _RUN.setdefault("padlock_dial", {"step": None, "deg": None})


def score(objects):
    lock = objects["padlock"]
    _DIAL.update(step=_RUN["step"], deg=float(lock.angle_deg("dial")), off=gate_offsets(lock))   # for the demo grips
    if _RUN.get("padlock_log") and _RUN["t"] is not None and _DIAL.get("logged") != _RUN["step"]:
        _DIAL["logged"] = _RUN["step"]          # scripted demo only: one line of angles per control step
        off = _DIAL["off"]
        print(f"PADLOG {_RUN['step']} {float(_RUN['t']):.2f} dial={_DIAL['deg']:.2f} w1={off['wheel1']:.2f} "
              f"w2={off['wheel2']:.2f} cam={off['dial']:.2f} shackle={lock.joints['shackle'] * 1000:.2f}", flush=True)
    fell = sticky("lock off the hasp", lock.origin[2] < LOCK_Z - FELL_DROP)
    w1, w2, cam = gates_set(lock)
    opened = shackle_open(lock)
    # Forced: the shackle comes open while a gate is more than one mark off. Judged as it opens (the first 0.25 s);
    # once open, the fence is out of the gates and the dial turns freely, as on the real lock.
    was_open = ever("shackle was open", False)                           # before this step (offline: never)
    just_opened = opened and not was_open
    ever("shackle was open", opened)
    forced = sticky("shackle forced", just_opened and max(abs(v) for v in gate_offsets(lock).values()) > FORCED_DEG)
    hanging = on_hasp(lock)
    done = opened and hanging and released(lock)
    seconds = held_for("open, hanging, released", done)
    settled = done and lock_settled(lock, seconds)
    if fell or forced:
        return 0.0
    if settled:
        return 1.0
    if opened or (w1 and w2 and cam):
        return 0.75
    if w1 and w2:
        return 0.5
    if w1:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo
# Tilted-wrist waypoints: a float yaw (for the offline checker's coarse replay) carrying the full grip rotation.
class GripOrientation(float):
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_lock_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


_grip_rotation._lock_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation


def roll_rotation(roll_deg):
    """Gripper pointing +x (into the dial), pads closing along (0, cos r, sin r): +roll turns the dial RIGHT."""
    r = math.radians(roll_deg)
    a = np.array([1.0, 0.0, 0.0])
    c = np.array([0.0, math.cos(r), math.sin(r)])
    return np.column_stack([c, np.cross(a, c), a])


PAD_TIP = 0.013                                    # pad tips 13 mm past the TCP
TCP_DIAL = np.array([BODY_FACE_X - (DISC_PROUD + 1.5) * MM - PAD_TIP, LOCK_Y, LOCK_Z - 0.002])   # pads 1.5 mm off the disc
BACK_OFF = 0.008
PULL_DOWN = 0.024
PINCH = -(2 * KNOB_R * MM - 0.008) / 2             # 4 mm past contact per pad on the 18 mm knob
OPEN_W = -0.018                                    # 36 mm opening around the knob (fingers stay over the disc)
LEFT_PARK = np.array([0.12, 0.25, TOP + 0.17])
ROLL_STEP = 60.0
DEMO_PACE = 1.0
# (start roll, end roll) per grip: five right grips (864 deg: two turns + to 24), three left grips (576 deg: one turn
# past 24 + to 8), one right grip (153 deg to 31). Wrist joint 6 spans 240 degrees, so the dial is re-gripped.
# Each grip turns the dial a few degrees less than the wrist rolls (pad compliance and wrist lag: GPU runs measured
# 2-17 deg per grip, varying), so the planned grips stop short and the last grip of each number is closed-loop on the
# angles read from the simulator (as scripted demos may), never reversing: a fast approach in 0.1 s steps that stops
# LIVE_FAST["creep"] degrees short (the pinched dial creeps on 2-3 deg after the wrist stops), a pause, then a slow
# trim in 0.25 s steps of at most 0.5 deg onto the number. While a wheel is being carried, the loop steers on that
# wheel's own gate (so the pins' play is included). Targets are the dial angles of the maker's rule: 864 (24 after two
# turns right), 288 (8 after one turn left past 24), 441 (31, right).
DIAL_TARGETS = (864.0, 288.0, 441.0)
SET_BY = ("wheel1", "wheel2", "dial")                               # the gate each number sets
PLANNED = ((68.0, 200.0, 4), (292.0, -205.0, 2), (68.0, 0.0, 0))   # (start roll, roll per grip, grips) per number
LIVE_START = (68.0, 292.0, 68.0)
LIVE_CAP = 220.0
LIVE_FAST = {"dt": 0.1, "n": 70, "gain": 0.3, "max": 4.0, "creep": 4.0, "tol": 0.3}
LIVE_TRIM = {"dt": 0.25, "n": 14, "gain": 0.5, "max": 0.5, "creep": 0.3, "tol": 0.15}
UNWIND = 8.0                  # before letting go of numbers 1 and 2 the wrist eases back 8 deg, so the pads opening
                              # cannot flick the dial (and the wheels it carries) past the number


def _remaining(c):
    """Degrees still to turn for this number, in the turning direction (negative = past it)."""
    rem = c["dir"] * (c["target"] - _DIAL["deg"])
    off = (_DIAL.get("off") or {}).get(c["link"])
    if c["link"] != "dial" and off is not None and abs(-c["dir"] * off - rem) < 6.0:
        rem = -c["dir"] * off           # the wheel is being carried: steer on its own gate
    return rem


class LiveRoll(float):
    """One step of a closed-loop grip: its roll is fixed the first time the plan reaches it, from the previous
    step's roll and the latest angles. Offline (no simulator) it follows a planned ramp to the cap."""
    def __new__(cls, chain, idx, phase):
        obj = super().__new__(cls, 0.0)
        obj.chain, obj.idx, obj.phase = chain, idx, phase
        return obj

    def roll(self):
        c, p = self.chain, self.phase
        if self.idx in c["values"]:
            return c["values"][self.idx]
        planned = c["start"] + c["dir"] * LIVE_CAP * min(self.idx + 1, LIVE_FAST["n"]) / LIVE_FAST["n"]
        if _RUN["t"] is None or _DIAL["deg"] is None:
            return planned
        prev = c["values"].get(self.idx - 1, c["start"])
        rem = _remaining(c)
        err = rem - p["creep"]
        inc = min(p["max"], p["gain"] * err) if err > p["tol"] else 0.0
        value = c["start"] + c["dir"] * min(LIVE_CAP, c["dir"] * (prev - c["start"]) + inc)
        c["values"][self.idx] = value
        if self.idx == c["last"]:
            print(f"PADSTOP number={c['number']} dial={_DIAL['deg']:.2f} target={c['target']:.1f} "
                  f"remaining={rem:.2f} wrist_roll={value:.1f}", flush=True)
        return value

    @property
    def matrix(self):
        return roll_rotation(self.roll())


class LiveLerp(float):
    """A wrist step after a closed-loop grip: a fraction of the way from that grip's final roll to `target`."""
    def __new__(cls, end, target, frac):
        obj = super().__new__(cls, 0.0)
        obj.end, obj.target, obj.frac = end, float(target), float(frac)
        return obj

    def roll(self):
        r = self.end.roll()
        return r + (self.target - r) * self.frac

    @property
    def matrix(self):
        return roll_rotation(self.roll())


class LiveOffset(LiveLerp):
    """The final roll of a closed-loop grip plus a fixed offset."""
    def __new__(cls, end, delta):
        obj = super().__new__(cls, end, 0.0, 0.0)
        obj.delta = float(delta)
        return obj

    def roll(self):
        return self.end.roll() + self.delta


def demo():
    _RUN["padlock_log"] = True        # the scorer prints the dial and gate angles once per control step for this demo
    G = GripOrientation
    plan = {"left": [], "right": []}
    st = {}

    def add(dt, label, right=None, rf=None, rr=None):
        r = st["r"] if right is None else np.asarray(right, float)
        rf = st["rf"] if rf is None else rf
        rr = st["rr"] if rr is None else rr
        st.update(r=r, rf=rf, rr=rr)
        yaw = rr if isinstance(rr, (LiveRoll, LiveLerp)) else G(roll_rotation(rr))
        plan["right"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in r), rf, label, yaw))
        plan["left"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in LEFT_PARK), FINGER_OPEN, label, 0.0))

    def roll_to(target, per60, label):
        if isinstance(st["rr"], (LiveRoll, LiveLerp)):
            end = st["rr"]
            for k in range(1, 5):
                add(per60 * LIVE_CAP / 4 / ROLL_STEP, label, rr=LiveLerp(end, target, k / 4) if k < 4 else float(target))
            return
        cur = st["rr"]
        n = max(1, int(math.ceil(abs(target - cur) / ROLL_STEP - 1e-9)))
        for k in range(1, n + 1):
            add(per60 * abs(target - cur) / n / ROLL_STEP, label, rr=cur + (target - cur) * k / n)

    def regrip(k, label):
        add(0.5, f"grip {k}: let go", rf=OPEN_W)
        add(0.4, f"grip {k}: back off", right=back)

    back = TCP_DIAL - [BACK_OFF, 0, 0]
    st.update(r=TCP_DIAL - [0.025, 0, -0.020], rf=OPEN_W, rr=PLANNED[0][0])
    add(4.0, "approach in front of the dial")
    add(1.0, "line up", right=back)
    k = 0
    for number, ((r0, per, n), target, live0) in enumerate(zip(PLANNED, DIAL_TARGETS, LIVE_START)):
        side = "right" if number != 1 else "left"
        for _ in range(n):
            k += 1
            roll_to(r0, 0.5, f"grip {k}: wrist back")
            add(0.6, f"grip {k}: pads around the dial", right=TCP_DIAL)
            add(0.6, f"grip {k}: pinch", rf=PINCH)
            roll_to(r0 + per, 0.9, f"grip {k}: turn {side}")
            add(0.3, f"grip {k}: settle")
            regrip(k, "")
        k += 1
        roll_to(live0, 0.5, f"grip {k}: wrist back")
        add(0.6, f"grip {k}: pads around the dial", right=TCP_DIAL)
        add(0.6, f"grip {k}: pinch", rf=PINCH)
        chain = {"start": live0, "dir": 1.0 if side == "right" else -1.0, "target": target, "link": SET_BY[number],
                 "values": {}, "number": number + 1, "last": LIVE_FAST["n"] + LIVE_TRIM["n"] - 1}
        for i in range(LIVE_FAST["n"]):
            add(LIVE_FAST["dt"], f"grip {k}: turn {side} to the number", rr=LiveRoll(chain, i, LIVE_FAST))
        add(0.6, f"grip {k}: pause")
        for i in range(LIVE_TRIM["n"]):
            add(LIVE_TRIM["dt"], f"grip {k}: trim onto the number", rr=LiveRoll(chain, LIVE_FAST["n"] + i, LIVE_TRIM))
        add(0.4, f"grip {k}: settle")
        if number < 2:
            add(0.4, f"grip {k}: ease the wrist back", rr=LiveOffset(st["rr"], -chain["dir"] * UNWIND))
            regrip(k, "")
    add(1.5, "pull the body down", right=TCP_DIAL - [0, 0, PULL_DOWN])
    add(1.0, "hold the pull")
    add(0.8, "ease off", right=TCP_DIAL - [0, 0, SHACKLE_TRAVEL])
    add(0.8, "let go", rf=OPEN_W)
    add(1.0, "back away", right=TCP_DIAL - [0.025, 0, 0.008])
    add(1.2, "clear", right=TCP_DIAL - [0.045, 0, 0.0])
    add(4.0, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
