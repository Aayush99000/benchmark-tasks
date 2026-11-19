"""On the station table (standing in for a bathroom floor): a backless aluminium shower chair, turned so that one leg
points at the arms. Its white blow-moulded seat is 50 x 30 cm; four splayed aluminium legs end in grey rubber tips,
46 x 34 cm apart. Each leg telescopes: a 22 mm inner tube with the rubber tip slides in a 25 mm outer tube that has a
row of eight holes, 1 inch (25.4 mm) apart, for seat heights of 14 to 21 inches. A spring push-button on the inner
tube pokes 4 mm out through one hole and locks the leg there; pressed in, it frees the leg to slide. Three legs are
set at 17 in. The near leg, the one pointing at the arms, is set one hole short, at 16 in, so the chair rocks: it
rests tilted towards the arms on that leg and its two neighbours, and the far leg's tip is about 2.5 cm off the
table. The near leg's button faces the arms, about 23 cm above the table. A pump bottle of soap and a bath sponge
stand beyond the chair.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena import demo as _demo
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix
from yam_arena import station
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Fixed camera mount for the full-height chair; the station constructs the matching
# camera body and stand from this same rig. No rendered-only pose changes.
CAMERA_POS = (-0.30, -0.60, 1.50)
CAMERA_TARGET = (0.45, 0.0, 0.98)
_base_rig = getattr(station.rig, "_chair_base", station.rig)
def _chair_rig(top_cam="calibrated"):
    rig = _base_rig(top_cam)
    forward = np.asarray(CAMERA_TARGET) - np.asarray(CAMERA_POS)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rot = np.column_stack((right, up, -forward))
    camera = replace(rig.top_cam, pos=CAMERA_POS,
                     rot_xyzw=tuple(station.matrix_to_quat_xyzw(rot)), convention="opengl")
    return replace(rig, name="station-shower-chair", top_cam=camera)
_chair_rig._chair_base = _base_rig
station.rig = _chair_rig

# -- the chair, in its own frame: origin on the floor under the seat's centre, x along the seat's 50 cm, level, all
# four legs at 17 in -------------------------------------------------------------------------------------------------

PITCH = 0.0254  # hole spacing: 1 inch
SPLAY = math.radians(10.0)  # each leg leans out this far from vertical, away from the seat's centre
FOOT_X, FOOT_Y = 0.23, 0.17  # rubber tips' centres, 46 x 34 cm apart
TOP_Z = 0.375  # where the legs meet the seat frame
LEG_LEN = TOP_Z / math.cos(SPLAY)
SEAT = (0.50, 0.30, 0.04)
SEAT_TOP = 0.432  # 17 in
R_OUT, R_IN = 0.0125, 0.011  # outer and inner tubes' radii
S_BUTTON = 0.235  # the button's centre, along the leg from the tip's bottom
OUTER_LOW = 0.118  # the outer tube's bottom end, along the leg (for a leg at 17 in)
INNER_TOP = 0.28  # the inner tube's top end, along the leg
HOLES = [S_BUTTON + (3 - j) * PITCH for j in range(8)]  # along a 17 in leg: j = 0 is 14 in (top hole) .. 7 is 21 in
BTN_R, BTN_OUT, BTN_TRAVEL = 0.003, 0.004, 0.0055  # 6 mm button, 4 mm proud of the outer tube, 5.5 mm of travel
UNLATCH = 0.003  # the button frees the leg once pressed this far in (1 mm short of flush)
CAPTURE = 0.0015  # a free leg's button snaps into a hole it comes within this of
SPRING_BTN = 2000.0  # N/m: 8 N to press it flush
LEG_MASS, BTN_MASS, FRAME_MASS = 0.10, 0.01, 1.828  # total 2.268 kg (5 lb), manufacturer-scale bench
HELD_K, HELD_C, FREE_C = 50000.0, 150.0, 8.0  # a latched leg's drive, and a free leg's damping (N s/m)

LEGS = {"near": (-1, -1), "left": (-1, 1), "right": (1, -1), "far": (1, 1)}
START_SETTING = {"near": 16, "left": 17, "right": 17, "far": 17}  # seat height in inches each leg is set for
YAW = -math.degrees(math.atan2(FOOT_Y, FOOT_X))  # turns the near leg's tip to point straight at the arms (-x)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _leg_axes(sx, sy):
    """For one leg (17 in, level chair): its tip's bottom point F, the unit vector up the leg, and the button's
    outward normal."""
    radial = _unit((sx * FOOT_X, sy * FOOT_Y))
    foot = np.array([sx * FOOT_X, sy * FOOT_Y, 0.0])
    up = np.array([-math.sin(SPLAY) * radial[0], -math.sin(SPLAY) * radial[1], math.cos(SPLAY)])
    out = np.array([math.cos(SPLAY) * radial[0], math.cos(SPLAY) * radial[1], math.sin(SPLAY)])
    return foot, up, out


def _frame_to(z_dir, x_hint):
    """A rotation matrix whose z column is z_dir and x column is x_hint made perpendicular to it."""
    z = _unit(z_dir)
    x = _unit(np.asarray(x_hint, float) - np.dot(x_hint, z) * z)
    return np.column_stack([x, np.cross(z, x), z])


def _quat_wxyz(r):
    """Rotation matrix to (w, x, y, z)."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    q = np.array(q)
    return tuple(float(v) for v in q / np.linalg.norm(q))


def _along(mesh, base, up, out):
    """A mesh built about +z (its x towards the button side) placed along a leg from `base`."""
    r = np.column_stack([out, np.cross(up, out), up])
    return m.Mesh(mesh.v @ r.T + base, mesh.f)


TIP_PROFILE = [(0.0, 0.0), (0.009, 0.0004), (0.0145, 0.0035), (0.0165, 0.009), (0.0165, 0.030), (0.0125, 0.032), (0.0, 0.032)]

LOOKS = {
    "seat": m.Look(m.rgb(0.93, 0.93, 0.91), roughness=0.45),
    "drain": m.Look(m.rgb(0.55, 0.56, 0.57), roughness=0.6),
    "alu": m.Look(m.rgb(0.80, 0.81, 0.83), metallic=0.8, roughness=0.32),
    "inner": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.38),
    "hole": m.Look(m.rgb(0.06, 0.06, 0.07), roughness=0.9),
    "button": m.Look(m.rgb(0.86, 0.87, 0.89), metallic=0.9, roughness=0.2),
    "rubber": m.Look(m.rgb(0.30, 0.31, 0.33), roughness=0.85),
    "bracket": m.Look(m.rgb(0.62, 0.63, 0.66), roughness=0.5),
}


def _rounded_rect(w, d, r, n=6):
    pts = []
    for cx, cy, a0 in ((w / 2 - r, d / 2 - r, 0), (-w / 2 + r, d / 2 - r, 90), (-w / 2 + r, -d / 2 + r, 180), (w / 2 - r, -d / 2 + r, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _chair():
    frame_parts, frame_cols, links = [], [], []
    seat = m.prism(_rounded_rect(SEAT[0], SEAT[1], 0.04), SEAT_TOP - SEAT[2], SEAT_TOP)
    frame_parts.append(("seat", seat, "seat"))
    frame_cols.append(m.col_hull("seat_c", seat))
    k = 0
    for gx in np.linspace(-0.15, 0.15, 7):  # drain holes
        for gy in (-0.06, 0.0, 0.06):
            frame_parts.append((f"drain{k}", m.cylinder(0.0045, 0.001, (gx, gy, SEAT_TOP + 0.0003)), "drain"))
            k += 1
    for sx in (-1, 1):  # moulded hand-holds near the seat's ends
        frame_parts.append((f"handhold{sx + 1}", m.box((0.025, 0.12, 0.001), (sx * 0.215, 0.0, SEAT_TOP + 0.0003)), "drain"))
    tops = {}
    for name, (sx, sy) in LEGS.items():
        foot, up, out = _leg_axes(sx, sy)
        tops[name] = foot + LEG_LEN * up
    # the seat frame: aluminium tubes under the seat joining the legs' tops
    for a, b in (("near", "right"), ("left", "far"), ("near", "left"), ("right", "far")):
        frame_parts.append((f"rail_{a}_{b}", m.tube([tops[a] + (0, 0, 0.005), tops[b] + (0, 0, 0.005)], 0.0095), "alu"))
    for name, (sx, sy) in LEGS.items():
        foot, up, out = _leg_axes(sx, sy)
        outer = m.cylinder(R_OUT, LEG_LEN - OUTER_LOW, (0, 0, (LEG_LEN + OUTER_LOW) / 2), segments=24)
        frame_parts.append((f"outer_{name}", _along(outer, foot, up, out), "alu"))
        frame_parts.append((f"bracket_{name}", _along(m.cylinder(0.017, 0.03, (0, 0, LEG_LEN - 0.012), segments=20), foot, up, out), "bracket"))
        frame_cols.append(m.col_hull(f"outer_{name}_c", _along(m.cylinder(R_OUT, LEG_LEN - OUTER_LOW, (0, 0, (LEG_LEN + OUTER_LOW) / 2), segments=16), foot, up, out)))
        for j, s in enumerate(HOLES):
            disc = m.cylinder(0.0033, 0.0006, (0, 0, R_OUT - 0.0001)).aimed("+X").moved((0, 0, s))
            frame_parts.append((f"hole_{name}_{j}", _along(disc, foot, up, out), "hole"))
        # the inner tube, with the rubber tip and the button, authored at the leg's starting setting
        shift = (START_SETTING[name] - 17) * PITCH  # negative: the tube sits further up the leg
        base = foot - shift * up
        tip = _along(m.lathe(TIP_PROFILE, 24), base, up, out)
        inner = _along(m.cylinder(R_IN, INNER_TOP - 0.028, (0, 0, (INNER_TOP + 0.028) / 2), segments=20), base, up, out)
        leg_rot = _frame_to(-up, out)  # joint +z: down the leg, so a positive value lengthens it
        links.append(m.Link(f"leg_{name}", "slide", "Z", tuple(float(v) for v in base),
                            (14 - START_SETTING[name]) * PITCH, (21 - START_SETTING[name]) * PITCH,
                            parts=(("tip", tip, "rubber"), ("tube", inner, "inner")),
                            colliders=(m.col_hull(f"tip_{name}_c", tip), m.col_hull(f"inner_{name}_c", inner)),
                            mass=LEG_MASS, spring=HELD_K, damping=HELD_C, effort=2000.0, friction=0.02, gravity=True,
                            com=tuple(float(v) for v in base + 0.12 * up), rot=_quat_wxyz(leg_rot)))
        cap = m.cylinder(BTN_R, R_OUT + BTN_OUT - 0.0115, (0, 0, (R_OUT + BTN_OUT + 0.0115) / 2), segments=16).aimed("+X").moved((0, 0, S_BUTTON))
        cap_c = m.cylinder(BTN_R, R_OUT + BTN_OUT - 0.012, (0, 0, (R_OUT + BTN_OUT + 0.012) / 2), segments=12).aimed("+X").moved((0, 0, S_BUTTON))
        btn_at = base + S_BUTTON * up + R_OUT * out
        links.append(m.Link(f"btn_{name}", "slide", "Z", tuple(float(v) for v in btn_at), -BTN_TRAVEL, 0.0,
                            parts=(("cap", _along(cap, base, up, out), "button"),), colliders=(m.col_hull(f"btn_{name}_c", _along(cap_c, base, up, out)),),
                            mass=BTN_MASS, spring=SPRING_BTN, damping=2.0, effort=100.0, friction=0.0, parent=f"leg_{name}",
                            rot=_quat_wxyz(_frame_to(out, up))))
    return frame_parts, frame_cols, links


FRAME_PARTS, FRAME_COLS, LINKS = _chair()
FRAME_COM = (0.0, 0.0, 0.368)
FRAME_INERTIA = (0.027, 0.048, 0.057)


def _tip_points(name):
    """The rubber tip's collider points in the chair frame, at the leg's starting setting (joint zero)."""
    sx, sy = LEGS[name]
    foot, up, out = _leg_axes(sx, sy)
    base = foot - (START_SETTING[name] - 17) * PITCH * up
    return _along(m.lathe(TIP_PROFILE, 24), base, up, out).v


TIP_POINTS = {n: _tip_points(n) for n in LEGS}
LEG_DOWN = {n: -_leg_axes(*LEGS[n])[1] for n in LEGS}  # a leg's extension direction, chair frame


def _rot(axis, deg):
    return axis_angle_matrix(_unit(axis), math.radians(deg))


def _start_pose():
    """The chair as it settles with its near leg short: on the near and both side tips, tilted towards the arms.
    Returns (rotation, origin) in the world."""
    yaw = _rot((0, 0, 1), YAW)
    low = {n: TIP_POINTS[n] for n in LEGS}

    def lowest(r, n):
        return float((low[n] @ r.T)[:, 2].min())

    # turn about the line through the two side tips' lowest points until the near tip's lowest point is as low
    pl = low["left"][np.argmin(low["left"][:, 2])]
    pr = low["right"][np.argmin(low["right"][:, 2])]
    axis = _unit(pr - pl)  # turning positively about left->right tips the near corner down
    lo, hi = 0.0, 15.0
    for _ in range(60):
        mid = (lo + hi) / 2
        r = _rot(axis, mid)
        side = min(lowest(r, "left"), lowest(r, "right"))
        if lowest(r, "near") > side:
            lo = mid
        else:
            hi = mid
    r = _rot(axis, lo)
    origin = pl - r @ pl  # the side tips stay where they were
    world_r = yaw @ r
    floor = min(float((low[n] @ world_r.T)[:, 2].min()) for n in LEGS)
    return world_r, yaw @ origin - np.array([0, 0, floor]), lo


CX = 0.476  # the chair's centre (seat centre over the floor) in front of the arms
START_R, _O, START_TILT = _start_pose()
START_POS = np.array([CX, 0.0, TOP]) + _O + np.array([0.0, 0.0, 0.001])


def _xyzw(r):
    w, x, y, z = _quat_wxyz(r)
    return (x, y, z, w)


CHAIR = m.mechanism("chair", tuple(START_POS), LINKS, LOOKS, frame_parts=FRAME_PARTS, frame_colliders=FRAME_COLS,
                    friction=(1.2, 1.0), solver_iters=48, free_mass=FRAME_MASS, start_quat=_xyzw(START_R),
                    frame_inertia=FRAME_INERTIA, frame_com=FRAME_COM)

OBJECTS = [
    CHAIR,
    # Distractors beyond the chair (SimReady props), which scoring ignores. pos is each one's origin, on the table top.
    SimReady("soap", pos=(0.46, 0.43, TOP), size=(0.075, 0.070, 0.182), mass=0.45, yaw=0.4,
             package="cleaning_soapDispenser_a01", usd="sm_cleaning_soapDispenser_a01_01.usd"),
    SimReady("sponge", pos=(0.70, -0.25, TOP), size=(0.071, 0.101, 0.033), mass=0.03, yaw=0.5,
             package="cleaning_sponge_b01", usd="sm_cleaning_sponge_b01_01.usd"),
]

# -- the push-button latches --------------------------------------------------------------------------------------------

_latched = {n: True for n in LEGS}
_hole = {n: 0 for n in LEGS}  # holes from the leg's starting one: +1 is one inch longer
_prev = {n: 0.0 for n in LEGS}
_tips = None


def _button_face(st, name, q_leg):
    """World point at the middle of a button's face (as if out), with the leg at extension q_leg."""
    sx, sy = LEGS[name]
    foot, up, out = _leg_axes(sx, sy)
    base = foot - (START_SETTING[name] - 17) * PITCH * up
    p = base + S_BUTTON * up + (R_OUT + BTN_OUT) * out - q_leg * up
    return st.to_world(p)[0]


def _fingers():
    """Each finger of both grippers as (tip point, unit vector up the finger, unit vector across it, unit normal),
    from mechkit's fingertip points (per arm: 6 per finger, across -18/0/+18 mm, at the tip and 3 cm up)."""
    out = []
    for f in range(4):
        p = _tips[6 * f:6 * f + 6]
        o = p[2]
        a = _unit(p[3] - o)
        w = _unit(p[4] - p[0])
        out.append((o, a, w, np.cross(a, w)))
    return out


def _covered(st, name, hole):
    """True if a finger lies over this leg's hole (where the button would come out): a finger pressing the button,
    or a pad clamped round the tube over that hole, keeps the button from coming out there."""
    if _tips is None:
        return False
    p = _button_face(st, name, hole * PITCH)
    for o, a, w, nrm in _fingers():
        d = p - o
        u, v, t = float(d @ a), float(d @ w), float(d @ nrm)
        # the finger: 15 mm wide at its tip, widening to 68 mm 3 cm up, 9 cm long; the button's face within 1 cm
        # of the finger's middle plane (the pad's face is 4.5 mm from it; the button stands 4 mm proud)
        if -0.004 <= u <= 0.09 and abs(v) <= min(0.0077 + 0.9 * max(u, 0.0), 0.034) + BTN_R and abs(t) <= 0.010:
            return True
    return False


@m.on_reset
def _reset_latches(environment=None):
    global _tips
    for n in LEGS:
        _latched[n], _hole[n], _prev[n] = True, 0, 0.0
    _tips = None
    _new_trial()


@m.on_step
def _latches(environment):
    """A latched leg stays at its hole until its button is pressed 3 mm in; then it slides free, its button riding in
    the bore, until the button meets a hole that no finger covers and snaps out into it."""
    global _tips
    _tips = m.fingertips(environment)
    st = m.state(environment, "chair")
    for n in LEGS:
        q = st.joint(f"leg_{n}")
        b = st.joint(f"btn_{n}")
        if _latched[n]:
            if -b >= UNLATCH:
                _latched[n] = False
                m.slacken(environment, "chair", f"leg_{n}", FREE_C)
                m.set_target(environment, "chair", f"btn_{n}", -BTN_TRAVEL)
        else:
            lo, hi = sorted((_prev[n], q))
            near = round(q / PITCH)
            crossed = [k for k in range(math.floor(lo / PITCH) - 1, math.ceil(hi / PITCH) + 2)
                       if lo - CAPTURE <= k * PITCH <= hi + CAPTURE]
            crossed.sort(key=lambda k: abs(k * PITCH - _prev[n]))  # the first hole met, in the direction of travel
            for k in crossed + ([near] if abs(q - near * PITCH) < CAPTURE else []):
                if _covered(st, n, k):
                    continue
                _latched[n], _hole[n] = True, k
                m.hold_at(environment, "chair", f"leg_{n}", k * PITCH, HELD_K, HELD_C)
                m.set_target(environment, "chair", f"btn_{n}", 0.0)
                break
        _prev[n] = q


# -- scoring ------------------------------------------------------------------------------------------------------------

SUCCESS_HOLD_S = 0.0  # Trial enforces the complete three-second condition itself.
SCORE_OVER = "end"
LEVEL_DEG = 1.0
FOOT_GAP = 0.001
PENETRATION_TOL = 0.002
CLEAR = 0.03
HOLD_SECONDS = 3.0

def _new_trial():
    TRIAL.reset()


def setting(st, name):
    """The leg's setting in inches of seat height, if its button is out in a hole; else None."""
    q, b = st.joint(f"leg_{name}"), st.joint(f"btn_{name}")
    k = round(q / PITCH)
    if -b < 0.001 and abs(q - k * PITCH) < 0.002:
        return START_SETTING[name] + k
    return None


def tip_gap(st, name):
    """Height of the leg's rubber tip's lowest point above the table (m)."""
    pts = TIP_POINTS[name] + st.joint(f"leg_{name}") * LEG_DOWN[name]
    return float(st.to_world(pts)[:, 2].min() - TOP)


def seat_tilt(st):
    return st.tilt_deg()


def _hands_off(st):
    """No fingertip point within CLEAR of the seat or any leg (taken as a 2 cm thick rod from tip to top)."""
    if _tips is None:
        return False
    local = st.to_local(_tips)
    half = np.array(SEAT) / 2
    centre = np.array([0, 0, SEAT_TOP - SEAT[2] / 2])
    d_seat = np.linalg.norm(np.maximum(np.abs(local - centre) - half, 0.0), axis=1)
    d = d_seat
    for n, (sx, sy) in LEGS.items():
        foot, up, out = _leg_axes(sx, sy)
        a = foot + st.joint(f"leg_{n}") * LEG_DOWN[n] - (START_SETTING[n] - 17) * PITCH * up
        b = foot + LEG_LEN * up
        t = np.clip((local - a) @ (b - a) / np.dot(b - a, b - a), 0, 1)
        d = np.minimum(d, np.linalg.norm(local - (a + t[:, None] * (b - a)), axis=1) - 0.0165)
    return bool(d.min() > CLEAR)


class Trial:
    """Updated once per control step; score and success never advance time."""
    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.hold = 0.0
        self.value = 0.0
        self.full = False

    def update(self, st, dt):
        settings = [setting(st, name) for name in LEGS]
        matched = (all(v is not None and 14 <= v <= 21 for v in settings)
                   and len(set(settings)) == 1)
        # Keep the chair on the station work surface. Tilting an empty chair or
        # choosing another equal leg height is a valid recovery route.
        tips = np.vstack([st.to_world(TIP_POINTS[name] + st.joint(f"leg_{name}") * LEG_DOWN[name])
                          for name in LEGS])
        table_lo = np.asarray(station.TABLE_CENTER[:2]) - np.asarray(station.TABLE_SIZE[:2]) / 2
        table_hi = np.asarray(station.TABLE_CENTER[:2]) + np.asarray(station.TABLE_SIZE[:2]) / 2
        contained = (np.all(tips[:, :2] >= table_lo) and np.all(tips[:, :2] <= table_hi)
                     and float(tips[:, 2].min()) >= TOP - PENETRATION_TOL)
        steady = all(-PENETRATION_TOL <= tip_gap(st, n) <= FOOT_GAP for n in LEGS)
        still = np.linalg.norm(st.lin_vel) < 0.005 and np.linalg.norm(st.ang_vel) < 0.05
        ready = bool(matched and contained and seat_tilt(st) <= LEVEL_DEG
                     and steady and still and _hands_off(st))
        self.hold = self.hold + float(dt) if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_SECONDS - 1e-9)
        self.value = 1.0 if self.full else (0.5 if matched and contained else 0.0)

    def step(self, environment):
        self.update(m.state(environment, "chair"), environment.unwrapped.step_dt)


TRIAL = Trial()
m.on_step(TRIAL.step)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo --------------------------------------------------------------------------------------------------


class Orient(float):
    """A waypoint 'yaw' that carries a whole TCP rotation."""

    rot = np.eye(3)


def _orient(r):
    o = Orient(0.0)
    o.rot = r
    return o


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def tilted(theta_deg, phi_deg):
    """Tilted theta from straight down towards phi (from +x), fingers closing across that direction."""
    phi = math.radians(phi_deg)
    r = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return _orient(r)


def start_world(p_local):
    """A point of the chair (its own frame) where it stands at the start."""
    return START_R @ np.asarray(p_local, float) + START_POS


def _level_pose():
    """The chair turned level about its side tips, as when its near corner is lifted until the far tip touches."""
    r = _rot((0, 0, 1), YAW)
    low = min(float((TIP_POINTS[n] @ r.T)[:, 2].min()) for n in ("left", "right", "far"))
    return r, np.array([CX, 0.0, TOP - low])


LEVEL_R, LEVEL_POS = _level_pose()


def level_world(p_local):
    return LEVEL_R @ np.asarray(p_local, float) + LEVEL_POS


GRIP_S = 0.16  # where the left gripper takes the near leg's outer tube, along the leg from a 17 in tip's bottom
LIFT_OVER = 0.020  # lift this far past level: the arm sags under the chair
SET_DOWN = 0.008  # then lower this far below level, to put the lengthened leg's tip on the table
LIFT_GRIP = tilted(90.0, -90.0)  # level, pointing -y, the fingers closing along x across the tube


def demo():
    """The left arm takes the near leg's outer tube from the side and lifts that corner until the chair stands level
    on its other three legs, the near leg now carrying nothing. The right arm then closes its fingers round the same
    tube at the button, from the other side, the near finger's pad pushing the button in as a thumb would: the inner
    tube drops out onto the table. The right hand opens, and the button snaps out into the hole it now faces, one
    inch lower. The right arm backs off, the left sets the corner down on its lengthened leg, lets go and leaves."""
    foot, up, out = _leg_axes(*LEGS["near"])
    g = foot + GRIP_S * up
    g0, g1 = start_world(g), level_world(g)
    button_axis = level_world(foot + (S_BUTTON - (START_SETTING["near"] - 17) * PITCH) * up)  # on the leg's axis
    tilt = math.degrees(math.asin((LEVEL_R @ out)[2]))  # the leg leans this far from vertical
    # level, pointing +y, the fingers closing along the button's axis (turned with the leg's lean)
    pinch = _orient(_rot((0, 1, 0), tilt) @ tilted(90.0, 90.0).rot)
    side = np.array([0.0, 1.0, 0.0])
    up_w = np.array([0.0, 0.0, 1.0])
    left = [
        Waypoint(2.5, tuple(g0 + 0.11 * side + 0.02 * up_w), FINGER_OPEN, "beside the near leg", LIFT_GRIP),
        Waypoint(1.5, tuple(g0 + 0.003 * side), FINGER_OPEN, "round the outer tube", LIFT_GRIP),
        Waypoint(0.8, tuple(g0 + 0.003 * side), FINGER_CLOSED, "grip", LIFT_GRIP),
        Waypoint(2.0, tuple(g1 + 0.003 * side + LIFT_OVER * up_w), FINGER_CLOSED, "lift the corner level", LIFT_GRIP),
        Waypoint(5.8, tuple(g1 + 0.003 * side + LIFT_OVER * up_w), FINGER_CLOSED, "hold it level", LIFT_GRIP),
        Waypoint(1.2, tuple(g1 + 0.003 * side - SET_DOWN * up_w), FINGER_CLOSED, "set it down", LIFT_GRIP),
        Waypoint(0.8, tuple(g1 + 0.003 * side - SET_DOWN * up_w), FINGER_OPEN, "let go", LIFT_GRIP),
        Waypoint(1.5, tuple(g1 + 0.11 * side), FINGER_OPEN, "out to the side", LIFT_GRIP),
        Waypoint(1.5, tuple(g1 + 0.11 * side + 0.08 * up_w - 0.05 * np.array([1.0, 0, 0])), FINGER_OPEN, "away", LIFT_GRIP),
    ]
    right = [
        Waypoint(2.5, tuple(button_axis - 0.10 * side), FINGER_OPEN, "beside the button", pinch),
        Waypoint(4.8, tuple(button_axis - 0.10 * side), FINGER_OPEN, "wait for the lift", pinch),
        Waypoint(1.5, tuple(button_axis), FINGER_OPEN, "fingers round the tube at the button", pinch),
        Waypoint(0.8, tuple(button_axis), FINGER_CLOSED, "squeeze: the pad pushes the button in", pinch),
        Waypoint(1.0, tuple(button_axis), FINGER_CLOSED, "the inner tube drops out", pinch),
        Waypoint(0.6, tuple(button_axis), FINGER_OPEN, "open: the button snaps into the next hole", pinch),
        Waypoint(1.2, tuple(button_axis - 0.10 * side), FINGER_OPEN, "back out", pinch),
        Waypoint(1.5, (0.15, -0.15, TOP + 0.20), FINGER_OPEN, "away", tilted(60.0, 30.0)),
    ]
    return {"left": left, "right": right}
