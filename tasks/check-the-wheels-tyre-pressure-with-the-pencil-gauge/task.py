"""On the station table: a car wheel (steel 14 inch rim, 165/70 R14 tyre) lies flat, face up, to the robot's right
and away from it. Its rubber valve stem sticks out of the rim's well on the side nearest the robot, leaning in toward
the hub, with no dust cap. Near the robot's right arm a grey cradle holds a straight-headed pencil tyre gauge lying
level, its chuck toward the robot. The gauge's white reading stick is pushed in.
"""

from __future__ import annotations

import math
import pathlib
import sys
from functools import lru_cache

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

TYRE_BLACK, RUBBER = (0.06, 0.06, 0.065), (0.09, 0.09, 0.09)
STEEL, DISC_GREY, DARK = (0.66, 0.67, 0.69), (0.55, 0.56, 0.58), (0.15, 0.15, 0.16)
BRASS, CHROME, WHITE = (0.80, 0.62, 0.25), (0.74, 0.75, 0.77), (0.95, 0.95, 0.93)
MARK, CRADLE_GREY = (0.12, 0.12, 0.12), (0.42, 0.44, 0.47)

# --- the wheel (fixed): a 14 x 5 in steel wheel with a 165/70 R14 tyre, lying face up. Wheel frame: origin at the
# hub's centre on the table, z up, axes as the world's. ----------------------------------------------------------------
HUB = (0.520, -0.160)
TYRE_R = (0.183, 0.2935)  # tyre ring: 587 mm across the tread, 366 mm inside
TYRE_H = 0.165  # 165 mm section width: the tyre's height lying flat
RIM_R = 0.168  # inside face of the rim's barrel
RIM_T = 0.004
FLANGE_R = 0.192
DISC_Z = (0.084, 0.090)

# Valve: a snap-in rubber stem (TR413 size) in the well, on the side nearest the robot, leaning toward the hub
VALVE_LEAN = 30.0  # degrees from the wheel's axis (vertical) toward the hub
_E = math.radians(90.0 - VALVE_LEAN)
D = np.array([math.cos(_E), 0.0, math.sin(_E)])  # stem axis, wheel frame, pointing out of the stem (in and up)
STEM_BASE = np.array([-RIM_R, 0.0, 0.110])  # where the stem leaves the rim, wheel frame
V_YAW, V_ROLL = -math.pi / 2, math.radians(90.0 - VALVE_LEAN)  # turns the valve's own y axis onto D
STEM_LEN = 0.030  # base to the barrel's tip
RUBBER_LEN = 0.019
BARREL_IN, BARREL_OUT = 0.0026, 0.00385  # brass barrel: 5.2 mm bore, 7.7 mm across the thread
CORE_W = 0.0016  # core pin, 1.6 mm square
CORE_DOWN = 0.0008  # its tip 0.8 mm below the barrel's tip at rest
CORE_TRAVEL = 0.003
CORE_SPRING, CORE_PRELOAD = 250.0, 0.006  # N/m; rest point 6 mm out: 1.5 N at rest, 1.8 N pushed 1.2 mm (spring and air on the seal)
OPENS = 0.0006  # the core opens the valve once pushed 0.6 mm


def _ring_z(r_in, r_out, z0, z1, n, color, a0=0.0, collide=True):
    """n slabs round the wheel's axis: an n-sided ring from r_in to r_out, z0 to z1 (wheel frame)."""
    w = 2 * math.tan(math.pi / n) * r_out
    out = []
    for i in range(n):
        a = a0 + i * 2 * math.pi / n
        rm = (r_in + r_out) / 2
        out.append(parts.box((r_out - r_in, w, z1 - z0), at=(rm * math.cos(a), rm * math.sin(a), (z0 + z1) / 2),
                             color=color, yaw=a, collide=collide))
    return out


_V_ROT = parts._rot((0, 0, 1), V_YAW) @ parts._rot((1, 0, 0), V_ROLL)  # valve frame -> wheel frame (valve y = D)
assert np.allclose(_V_ROT @ np.array([0.0, 1.0, 0.0]), D)


def _valve_at(local) -> tuple[float, float, float]:
    return tuple(float(v) for v in STEM_BASE + _V_ROT @ np.asarray(local, float))


def _ring_v(r_in, r_out, s0, s1, n, color):
    """n slabs round the stem's axis (an n-sided tube), from s0 to s1 along it."""
    w = 2 * math.tan(math.pi / n) * r_out
    rm = (r_in + r_out) / 2
    out = []
    for i in range(n):
        a = i * 2 * math.pi / n
        c = (rm * math.sin(a), (s0 + s1) / 2, rm * math.cos(a))
        out.append(parts.box((w, s1 - s0, r_out - r_in), at=_valve_at(c), color=color, yaw=V_YAW, roll=V_ROLL,
                             pitch=a))
    return out


def _wheel_shapes():
    out = _ring_z(*TYRE_R, 0.0, TYRE_H, 24, TYRE_BLACK)
    out += _ring_z(RIM_R, RIM_R + RIM_T, 0.012, 0.152, 24, STEEL, a0=math.pi)  # barrel (a flat face at the valve)
    out += _ring_z(RIM_R, FLANGE_R, 0.150, 0.158, 24, STEEL, a0=math.pi)  # outer flange
    out += _ring_z(RIM_R, FLANGE_R, 0.006, 0.014, 24, STEEL, a0=math.pi)  # inner flange
    out.append(parts.cylinder(RIM_R + 0.001, DISC_Z[1] - DISC_Z[0], at=(0.0, 0.0, sum(DISC_Z) / 2), color=DISC_GREY))
    out.append(parts.cylinder(0.029, 0.004, at=(0.0, 0.0, DISC_Z[1] + 0.002), color=DARK, collide=False))  # hub bore
    for k in range(4):  # wheel nuts on a 100 mm circle
        a = math.pi / 4 + k * math.pi / 2
        out.append(parts.cylinder(0.0095, 0.012, at=(0.05 * math.cos(a), 0.05 * math.sin(a), DISC_Z[1] + 0.006),
                                  color=CHROME, collide=False))
    # the valve stem: rubber body, then the brass barrel (open bore) up to the tip
    out += _ring_v(0.0018, 0.0058, -0.004, RUBBER_LEN, 12, RUBBER)
    out += _ring_v(BARREL_IN, BARREL_OUT, RUBBER_LEN, STEM_LEN, 12, BRASS)
    return out


def _core():
    s1 = STEM_LEN - CORE_DOWN
    s0 = s1 - 0.015
    pin = parts.box((CORE_W, s1 - s0, CORE_W), at=_valve_at((0.0, (s0 + s1) / 2, 0.0)), color=BRASS, yaw=V_YAW,
                    roll=V_ROLL)
    return parts.Slider("core", shapes=[pin], axis=tuple(float(v) for v in -D), travel=CORE_TRAVEL, mass=0.008,
                        spring=CORE_SPRING, damping=0.3, max_effort=20)


WHEEL = parts.Part("wheel", pos=(*HUB, TOP), shapes=_wheel_shapes(), joints=[_core()], friction=(0.9, 0.8),
                   contact_offset=0.001)

# Valve in the world (the wheel is fixed)
_W_POS = np.array([*HUB, TOP])
TIP_W = _W_POS + STEM_BASE + STEM_LEN * D  # centre of the barrel's tip

# --- the pencil gauge (loose). Gauge frame: origin at the centre of the chuck's mouth, +x along the gauge toward its
# top end (where the stick comes out). ----------------------------------------------------------------------------------
SOCKET_IN, HEAD_R = 0.0044, 0.0080  # chuck: 8.8 mm socket (0.55 mm round the 7.7 mm barrel), 16 mm head
FLARE_R, FLARE_RUN = 0.0064, 0.0020  # its mouth flares at 45 degrees from 12.8 mm across down to the socket
GASKET_X = 0.010  # the rubber gasket's face, 10 mm inside the mouth
PIN_OUT = 0.0020  # depressor pin stands 2.0 mm proud of the gasket
PIN_R = 0.0010
HEAD_X = 0.024
BODY_R, BODY_END = 0.0070, 0.128  # barrel 14 mm across, ends 128 mm from the mouth
GRIP_X, GRIP_FLATS = (0.045, 0.085), 0.012  # a flat-sided grip section, 12 mm across its flats (14 mm deep)
STICK_R, STICK_X = 0.0030, (0.058, 0.130)  # 6 mm stick, 2 mm out of the end when pushed in
STICK_TRAVEL = 0.066
MM_PER_PSI = 0.0015  # 1.5 mm of stick per psi; pushed in it reads 10 psi (the bottom of the scale)
PSI_ZERO = 10.0
TYRE_PSI = 32.0
LEAK_PSI = 12.0  # pressed off square or not home: air escapes round the gasket and the stick barely moves
STICK_RATE = 0.004  # m per control step the stick is pushed out while air reaches the piston
GAUGE_MID = 0.0725  # the gauge's middle, where it sits in the cradle


def _ext(psi):
    return (psi - PSI_ZERO) * MM_PER_PSI


def _ring_x(r_in, r_out, x0, x1, n, color):
    w = 2 * math.tan(math.pi / n) * r_out
    rm = (r_in + r_out) / 2
    out = []
    for i in range(n):
        a = i * 2 * math.pi / n
        c = parts._rot((1, 0, 0), a) @ np.array([(x0 + x1) / 2, 0.0, rm])
        out.append(parts.box((x1 - x0, w, r_out - r_in), at=tuple(float(v) for v in c), color=color, roll=a))
    return out


def _gauge_shapes():
    def cyl(r, x0, x1, color, collide=True):
        return parts.cylinder(r, x1 - x0, at=((x0 + x1) / 2, 0.0, 0.0), color=color, axis="x", collide=collide)

    out = _ring_x(SOCKET_IN, HEAD_R, FLARE_RUN, GASKET_X, 12, CHROME)  # the socket
    t = 0.0015  # the flared mouth: twelve plates sloping 45 degrees from FLARE_R at the mouth to the socket
    length = FLARE_RUN * math.sqrt(2)
    centre = np.array([FLARE_RUN / 2, 0.0, (FLARE_R + SOCKET_IN) / 2]) + np.array([math.sqrt(0.5), 0.0, math.sqrt(0.5)]) * (t / 2)
    w = 2 * math.tan(math.pi / 12) * (FLARE_R + t)
    for i in range(12):
        a = i * math.pi / 6
        c = parts._rot((1, 0, 0), a) @ centre
        out.append(parts.box((length, w, t), at=tuple(float(v) for v in c), color=CHROME, roll=a, pitch=math.pi / 4))
    out.append(cyl(0.0060, GASKET_X, GASKET_X + 0.0015, RUBBER))  # gasket
    out.append(cyl(PIN_R, GASKET_X - PIN_OUT, GASKET_X, BRASS))  # depressor pin
    out.append(cyl(HEAD_R, GASKET_X + 0.0015, HEAD_X, CHROME))  # rest of the head
    out.append(cyl(BODY_R, HEAD_X, GRIP_X[0], CHROME))  # barrel
    out.append(parts.box((GRIP_X[1] - GRIP_X[0], GRIP_FLATS, 2 * BODY_R), at=(sum(GRIP_X) / 2, 0.0, 0.0), color=DARK))
    out.append(cyl(BODY_R, GRIP_X[1], BODY_END, CHROME))
    out.append(parts.box((0.050, 0.004, 0.0015), at=(0.095, 0.0, BODY_R + 0.0008), color=CHROME, collide=False))  # clip
    return out


def _stick():
    x0, x1 = STICK_X
    shapes = [parts.cylinder(STICK_R, x1 - x0, at=((x0 + x1) / 2, 0.0, 0.0), color=WHITE, axis="x")]
    for psi in (20.0, 30.0, 40.0):  # printed bands, read against the barrel's end
        xm = BODY_END - _ext(psi)
        shapes.append(parts.cylinder(STICK_R + 0.0002, 0.0012, at=(xm, 0.0, 0.0), color=MARK, axis="x",
                                     collide=False))
    return parts.Slider("stick", shapes=shapes, axis=(1.0, 0.0, 0.0), travel=STICK_TRAVEL, mass=0.006, spring=150.0,
                        damping=0.5, max_effort=20)


# --- the cradle (loose, weighted): two U-shaped saddles on a base plate. Cradle frame: centre of the plate on the
# table, x along the saddles. ----------------------------------------------------------------------------------------------
PLATE = (0.170, 0.050, 0.006)
SADDLE_X = 0.0375
FLOOR = 0.012
WALL_GAP = 0.011  # inner faces of the saddle walls 11 mm from the line: 22 mm across for the 14 mm barrel
WALL_T = 0.005
WALL_TOP = PLATE[2] + FLOOR + 0.009
REST_Z = PLATE[2] + FLOOR + BODY_R  # the gauge's axis above the table when it lies in the cradle


def _cradle_shapes():
    out = [parts.box(PLATE, at=(0.0, 0.0, PLATE[2] / 2), color=CRADLE_GREY)]
    for sx in (-SADDLE_X, SADDLE_X):
        out.append(parts.box((0.012, 2 * WALL_GAP + 2 * WALL_T, FLOOR), at=(sx, 0.0, PLATE[2] + FLOOR / 2),
                             color=CRADLE_GREY))
        for sy in (-1.0, 1.0):
            out.append(parts.box((0.012, WALL_T, WALL_TOP - PLATE[2] - FLOOR),
                                 at=(sx, sy * (WALL_GAP + WALL_T / 2), (PLATE[2] + FLOOR + WALL_TOP) / 2),
                                 color=CRADLE_GREY))
    return out


CRADLE_A = (0.200, -0.400)
CRADLE_B = (0.170, -0.330)


def _gauge_pos(c):
    return (c[0] - GAUGE_MID, c[1], TOP + REST_Z)


CRADLE = parts.Part("cradle", pos=(*CRADLE_A, TOP), shapes=_cradle_shapes(), mass=0.35, friction=(0.8, 0.7))
GAUGE = parts.Part("gauge", pos=_gauge_pos(CRADLE_A), shapes=_gauge_shapes(), joints=[_stick()], mass=0.045,
                   friction=(0.9, 0.8), contact_offset=0.001)

OBJECTS = [WHEEL, CRADLE, GAUGE]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"cradle": (*CRADLE_A, TOP), "gauge": _gauge_pos(CRADLE_A)}),
    lots.Lot("B", place={"cradle": (*CRADLE_B, TOP), "gauge": _gauge_pos(CRADLE_B)}),
)
DEMO_TOUCHES = ("wheel",)  # the pressed gauge's chuck closes round the valve; check.py treats the fingers as closed

# --- scoring ----------------------------------------------------------------------------------------------------------
EPS = 1e-6
SEAT_ALONG = 0.0006  # gasket face within 0.6 mm of the barrel's tip, along the stem
SEAT_SIDE = 0.0012  # and the chuck within 1.2 mm of the stem's axis
SEAT_TILT = 4.0  # and the gauge within 4 degrees of the stem's axis
READ_LO, READ_HI = 30.0, 34.0
IN_ALONG, IN_SIDE, IN_UP, IN_TILT = 0.012, 0.0045, 0.0025, 6.0
CLEAR = 0.005
ARM_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")
STILL_STEPS = 20
DROPPED = 0.02

# Kept in the parts module so near_misses.py, which imports this file separately, sees and can set the same state.
_grip = parts.__dict__.setdefault("_gauge_task_grip", {"clear": None})
_stick_air = parts.__dict__.setdefault("_gauge_task_air", {"target": 0.0})
_record = parts.__dict__.setdefault("_gauge_task_record", {"still": 0, "still_pose": None})


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def chuck_on_valve(gauge):
    """(along, side, tilt in degrees) of the gauge's gasket face against the valve's barrel tip: along is how far the
    gasket is out from the tip along the stem (0 seated), side its offset from the stem's axis."""
    r = _rot(gauge.quat_xyzw)
    ax = r @ np.array([1.0, 0.0, 0.0])
    g = np.asarray(gauge.pos, float) + ax * GASKET_X
    v = g - TIP_W
    along = float(v @ D)
    side = float(np.linalg.norm(v - along * D))
    t = math.degrees(math.acos(float(np.clip(ax @ D, -1.0, 1.0))))
    return along, side, t


def sealed(gauge) -> bool:
    along, side, t = chuck_on_valve(gauge)
    return bool(abs(along) <= SEAT_ALONG + EPS and side <= SEAT_SIDE + EPS and t <= SEAT_TILT + EPS)


def valve_open(wheel) -> bool:
    return bool(float(wheel.joints.get("core", 0.0)) >= OPENS - EPS)


def air_step(wheel, gauge) -> float:
    """Once per control step: where the gauge's piston pushes the stick. With the core pushed open and the gasket
    seated on the barrel's tip, tyre air drives the stick out toward the tyre's pressure; with the core open but the
    gasket not seated, air leaks round it and the stick goes no further than 12 psi. The stick is only ever pushed
    out (it stays where the air left it, as on a real pencil gauge). Returns the stick's target (m)."""
    if valve_open(wheel):
        cap = _ext(TYRE_PSI) if sealed(gauge) else _ext(LEAK_PSI)
        if _stick_air["target"] < cap:
            _stick_air["target"] = min(cap, _stick_air["target"] + STICK_RATE)
    return _stick_air["target"]


def reading(gauge) -> float:
    """What the stick shows at the barrel's end, psi."""
    return PSI_ZERO + float(gauge.joints.get("stick", 0.0)) / MM_PER_PSI


def reads_right(gauge) -> bool:
    p = reading(gauge)
    return bool(READ_LO - EPS <= p <= READ_HI + EPS)


def in_cradle(gauge, cradle) -> bool:
    """The gauge lying in the cradle's saddles: its middle within 12 mm of the cradle's middle along it, 4.5 mm across,
    its axis within 2.5 mm of its resting height and within 6 degrees of the saddles' line (either way round)."""
    rc = _rot(cradle.quat_xyzw)
    rg = _rot(gauge.quat_xyzw)
    ax = rg @ np.array([1.0, 0.0, 0.0])
    mid = np.asarray(gauge.pos, float) + ax * GAUGE_MID
    loc = rc.T @ (mid - np.asarray(cradle.pos, float))
    t = math.degrees(math.acos(float(np.clip(abs(ax @ (rc @ np.array([1.0, 0.0, 0.0]))), -1.0, 1.0))))
    return bool(abs(loc[0]) <= IN_ALONG + EPS and abs(loc[1]) <= IN_SIDE + EPS and abs(loc[2] - REST_Z) <= IN_UP + EPS
                and t <= IN_TILT + EPS)


def _dropped(objects) -> bool:
    return bool(float(objects["gauge"].pos[2]) < TOP - DROPPED)


def _same_pose(a, b) -> bool:
    """The gauge's middle within 0.5 mm and its axis within 0.5 degrees. Turning about its own axis is not counted:
    a round barrel rocks a degree or so in place in the saddles."""
    (pa, qa), (pb, qb) = a, b
    xa, xb = _rot(qa) @ np.array([1.0, 0.0, 0.0]), _rot(qb) @ np.array([1.0, 0.0, 0.0])
    ma, mb = pa + xa * GAUGE_MID, pb + xb * GAUGE_MID
    return bool(np.linalg.norm(ma - mb) <= 0.0005 + EPS and float(xa @ xb) >= math.cos(math.radians(0.5)) - 1e-9)


def _still_now(objects):
    """(how many consecutive control steps, this one included, the gauge has stayed within 0.5 mm and its axis within 0.5 degrees
    of where they were when the count began; the pose the count is anchored to). Pose only, as a person judges stillness
    from the video: a body resting in contact jitters in velocity without moving. Read by success() and score() alike;
    score() keeps it."""
    g = objects["gauge"]
    now = (np.asarray(g.pos, float), np.asarray(g.quat_xyzw, float))
    anchor = _record["still_pose"]
    if anchor is not None and _same_pose(now, anchor):
        return _record["still"] + 1, anchor
    return 0, now


def _count_still(objects) -> None:
    _record["still"], _record["still_pose"] = _still_now(objects)


def _still_count(objects) -> int:
    """In the simulator the count is kept once per control step by an on_step callback (below); offline,
    near_misses.py's cases are the steps and score() keeps it."""
    return _record["still"] if _record.get("sim") else _still_now(objects)[0]


def _clear(objects):
    """The arms' measured clearance from the gauge (None until measured). near_misses.py stands in offline."""
    if "offline_clear" in _grip:
        _grip["clear"] = _grip["offline_clear"](objects)
    return _grip["clear"]


def _value(objects) -> float:
    g = objects["gauge"]
    if (_dropped(objects) or not reads_right(g) or _grip["clear"] is not True
            or _stick_air.get("target", 0.0) < _ext(READ_LO) - EPS):
        return 0.0
    if in_cradle(g, objects["cradle"]) and _still_count(objects) >= STILL_STEPS:
        return 1.0
    return 0.5


def success(objects):
    """The gauge shows the tyre's pressure (30 to 34 psi), lies in its cradle, let go and still."""
    _clear(objects)
    return bool(_value(objects) == 1.0)


def score(objects):
    _clear(objects)
    v = _value(objects)
    if not _record.get("sim"):
        _count_still(objects)
    return v


# --- release check: every arm link's collision hull, its faces sampled every 3 mm ---------------------------------------
def _face_points(vertices, planes, step=0.003):
    """Points over every face of a convex hull (its vertices, plus a grid at most `step` apart on each face)."""
    v = np.asarray(vertices, float)
    pts = [v]
    for n in np.unique(np.round(np.asarray(planes, float), 7), axis=0):
        face = v[np.abs(v @ n[:3] - n[3]) < 2e-6]
        if len(face) < 3:
            continue
        c = face.mean(0)
        u = face[0] - c
        u /= np.linalg.norm(u) or 1.0
        w = np.cross(n[:3], u)
        face = face[np.argsort(np.arctan2((face - c) @ w, (face - c) @ u))]
        for a, b in zip(face, np.roll(face, -1, axis=0)):
            k = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step))
            for i in range(k + 1):
                for j in range(k - i + 1):
                    pts.append((c + (a - c) * (i / k) + (b - c) * (j / k))[None])
    return np.concatenate(pts)


@lru_cache(maxsize=1)
def _link_points():
    from yam_arena import analysis

    out = {}
    for name in ARM_LINKS:
        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        cloud = np.concatenate([_face_points(h.vertices, h.planes) for h in hs])
        _, keep = np.unique(np.floor(cloud / 0.002).astype(int), axis=0, return_index=True)  # one per 2 mm cell
        out[name] = np.concatenate([cloud[keep], np.concatenate([h.vertices for h in hs])])  # and every vertex
    return out


def _gauge_boxes(gauge):
    poses = {"base": (np.asarray(gauge.pos, float), _rot(gauge.quat_xyzw))}
    poses.update({k: (np.asarray(p, float), _rot(q)) for k, (p, q) in gauge.links.items()})
    return [(c, r, s) for _, c, r, s in GAUGE.world_boxes(poses)]


def arms_clear(points, boxes) -> bool:
    """True when no sampled arm point is within 5 mm of any of the gauge's shapes (cylinders as their boxes)."""
    from yam_arena import analysis

    if not points or any(len(p) == 0 for p in points):
        return False
    corners = np.concatenate([c + (np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
                                   * np.asarray(s) / 2) @ np.asarray(r).T for c, r, s in boxes])
    lo, hi = corners.min(0) - 0.05, corners.max(0) + 0.05  # links whose points are all over 5 cm from the gauge's box
    near = [p for p in points if not (np.any(p.max(0) < lo) or np.any(p.min(0) > hi))]
    if not near:
        return True
    pts = np.concatenate(near)
    return bool(all(float(analysis.box_signed_distance(pts, c, r, s).min()) > CLEAR for c, r, s in boxes))


@parts.on_step
def _measure_grip(sim) -> None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    local = _link_points()
    pts = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in local:
                pts.append(local[n] @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
    _grip["clear"] = len(pts) == 2 * len(ARM_LINKS) and arms_clear(pts, _gauge_boxes(st["gauge"]))


@parts.on_step
def _track_still(sim) -> None:
    """Every control step: the gauge's stillness count."""
    _record["sim"] = True
    _count_still(sim.states())


@parts.on_step
def _drive_stick(sim) -> None:
    st = sim.states()
    if not _record.get("preloaded"):
        sim.set_target("wheel", "core", -CORE_PRELOAD)
        _record["preloaded"] = True
    sim.set_target("gauge", "stick", air_step(st["wheel"], st["gauge"]))


@parts.on_reset
def _reset(sim) -> None:
    _grip["clear"] = None
    _stick_air["target"] = 0.0
    _record.update(still=0, still_pose=None, preloaded=False, sim=True)
    sim.set_target("wheel", "core", -CORE_PRELOAD)
    sim.set_target("gauge", "stick", 0.0)
    _record["preloaded"] = True


# --- demo: open loop from the published start layout of the trial's lot -------------------------------------------------
Waypoint = tilt.Waypoint
GRIP_AT = 0.065  # grip the barrel 65 mm from the mouth, between the saddles
PINCH = -(GRIP_FLATS / 2 - 0.003)  # fingers 3 mm past the grip's flats: flat pads stop it turning in the hand
OPEN = -0.030
LEAN = math.degrees(_E)  # 60: the level gauge turned about the fingers' line until it lies along the stem
PRESS = 0.025  # commanded past seated: the arm is soft (about 190 N/m), so about 4 to 5 N on the valve
APPROACH = 0.040  # line up 40 mm out along the stem


SETTLE = np.array([0.0035, -0.0015, -0.0020])  # the loaded right arm settles this far off its target here (measured)


def _on_stem(back):
    """TCP to command for the gauge to lie on the stem's axis with its gasket `back` metres out from the barrel's tip
    (allowing for how the loaded arm settles there)."""
    return tuple(float(v) for v in TIP_W + D * (GRIP_AT - GASKET_X + back) - SETTLE)


def demo():
    lot = lots.current()
    g = np.asarray(lot.pos("gauge"), float)
    pick = (float(g[0] + GRIP_AT), float(g[1]))
    z_pick = float(g[2]) + 0.004
    hover = TOP + 0.110
    lean = dict(tilt=LEAN, heading=0.0)  # finger tips toward the hub (+x): the gauge turns about the fingers' line
    a = _on_stem(APPROACH)
    p = _on_stem(-PRESS)
    high = TOP + 0.270  # crossing height: the leaning gauge's chuck end hangs about 60 mm below the hand, the tyre is 165 mm
    over = (a[0], a[1], high)
    put = (pick[0], pick[1], z_pick + 0.004)
    right = [
        Waypoint(2.0, (*pick, hover), OPEN, "over the gauge"),
        Waypoint(1.5, (*pick, z_pick), OPEN, "down round its grip"),
        Waypoint(0.8, (*pick, z_pick), PINCH, "grip"),
        Waypoint(1.5, (*pick, hover), PINCH, "lift it out"),
        Waypoint(2.0, (*pick, high), PINCH, "up, turning it onto the stem's angle", **lean),
        Waypoint(2.5, over, PINCH, "across, high over the tyre", **lean),
        Waypoint(2.0, a, PINCH, "down onto the stem's line, 40 mm out", **lean),
        Waypoint(0.8, a, PINCH, "settle", **lean),
        Waypoint(1.5, p, PINCH, "press the chuck onto the valve", **lean),
        Waypoint(1.5, p, PINCH, "hold it pressed", **lean),
        Waypoint(1.2, a, PINCH, "straight back off the valve", **lean),
        Waypoint(1.5, over, PINCH, "up", **lean),
        Waypoint(2.5, (*pick, high), PINCH, "back over the cradle", **lean),
        Waypoint(2.0, (*pick, hover), PINCH, "down, turning it level"),
        Waypoint(1.2, put, PINCH, "into the saddles"),
        Waypoint(0.6, put, OPEN, "let go"),
        Waypoint(1.0, (*pick, hover), OPEN, "up"),
        Waypoint(1.5, (pick[0] - 0.05, pick[1] + 0.05, hover + 0.05), OPEN, "away"),
        Waypoint(10.0, (pick[0] - 0.05, pick[1] + 0.05, hover + 0.05), OPEN, "wait clear while the gauge settles"),
    ]
    return {"right": right}
