"""On the table, 36 cm in front of the arms and centred between them, stands a manual Rockwell hardness tester (grey
cast body 16 cm wide, 52 cm deep, 70 cm tall; its column at the back, its head above). A steel test block (64 mm
across, 10 mm thick) lies on the tester's flat anvil (60 mm across, 20 cm above the table), 1.2 mm below the point of
the diamond indenter that hangs from the head. The anvil rides on an elevating screw turned by a horizontal handwheel
just below it (15 cm above the table): three spokes, each ending in an upright knob that spins freely, 8 cm from the
wheel's centre. Turning the handwheel clockwise (seen from above) raises the anvil, 4 mm a turn.

On the front of the head, facing the robot, is the dial (9 cm across): a long pointer that turns once for every
0.2 mm the indenter is pushed up (100 divisions of 0.002 mm), with C at the top, and a short pointer that moves from
a black dot to a red dot while the long pointer makes three turns. On the tester's right side is the main-load handle,
a rod with a knob on its end; it rests tilted forward. Pushed back, it lets the weights load the indenter over about
5 s; pulled forward again, it takes that load off.

Take one Rockwell C reading on the block: raise it onto the indenter until the preliminary load is set (the long
pointer at C after three turns), push the handle back, hold the full load 5 s, pull the handle forward, then lower
the block.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import hardness_scoring as S  # noqa: E402
import mechanism as mech  # noqa: E402
from mechanism import Joint, Link, Shape  # noqa: E402
from yam_arena import demo as yam_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tester, in the world frame (the mechanism's frame is the world's) [sizes: spec; estimates marked there] -------
AX = 0.36  # the indenter's axis, x (y = 0)
HALF_W = 0.08  # body 160 mm wide (Insize ISH-R150: 6.30 in)
BACK = AX + 0.30  # the body's back (overall depth 520 mm from the base's front at AX - 0.22)
BASE_FRONT = AX - 0.12
BASE_TOP = TOP + 0.115  # 4 cm under the knobs: room for the gripper below a knob held at the front
WHEEL_Z = TOP + 0.150  # the spokes' mid-plane
WHEEL_KNOB_R = 0.080  # knob axes from the wheel's centre
KNOB_R, KNOB_LO, KNOB_HI = 0.011, WHEEL_Z + 0.004, WHEEL_Z + 0.040
GRIP_Z = (KNOB_LO + KNOB_HI) / 2
KNOB_ANG0 = (math.radians(-60.0), math.radians(60.0), math.radians(180.0))  # from the front (-x), towards +y
ANVIL_R, ANVIL_TOP = 0.030, TOP + 0.200
BLOCK_H = 0.010
BLOCK_TOP = ANVIL_TOP + BLOCK_H
TIP_Z = BLOCK_TOP + S.START_GAP  # the indenter's point
TIP_CLEAR = 0.0  # the indenter's collider reaches its point: the block meets it and is pressed against it
NOSE_LO = TOP + 0.228
HEAD_LO, HEAD_HI, HEAD_FRONT = TOP + 0.255, TOP + 0.700, AX - 0.045
COLUMN_FRONT = AX + 0.165  # throat depth 6.5 in (ISH-R150 work surface depth)
DIAL_C = np.array([HEAD_FRONT - 0.004, 0.0, TOP + 0.330])
DIAL_R = 0.045
SHORT_C = DIAL_C + (0.0, 0.0, 0.020)
PIVOT = np.array([AX + 0.06, -(HALF_W + 0.015), TOP + 0.300])  # the main-load handle's pivot, on the right side
HANDLE_L = 0.100
HANDLE_KNOB_Y = PIVOT[1] - 0.020  # the knob stands out on the handle's end, towards -y

GREY, DARK, STEEL, WHITE, BLACK, RED = ((0.45, 0.47, 0.50), (0.18, 0.19, 0.21), (0.72, 0.73, 0.75), (0.95, 0.95, 0.93),
                                        (0.05, 0.05, 0.05), (0.85, 0.10, 0.08))
MATERIALS = {"cast": (0.5, 0.4), "steel": (0.4, 0.3), "rubber": (0.9, 0.8)}


def _box(lo, hi, color, material="cast") -> Shape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple((lo + hi) / 2), tuple(hi - lo), color, material=material)


def _cyl(c, r, length, color, axis="Z", material="steel") -> Shape:
    return Shape("cylinder", tuple(float(v) for v in c), (r, length), color, axis=axis, material=material)


def knob_xy(ang: float) -> np.ndarray:
    """A handwheel knob's axis at angle ``ang`` (rad from the front, towards +y)."""
    return np.array([AX - WHEEL_KNOB_R * math.cos(ang), WHEEL_KNOB_R * math.sin(ang)])


def knob_angles(wheel_cw: float) -> tuple[float, ...]:
    """The three knobs' angles with the handwheel turned ``wheel_cw`` rad clockwise from above."""
    return tuple(p + wheel_cw for p in KNOB_ANG0)


def handle_knob(theta: float) -> np.ndarray:
    """The handle knob's centre with the handle at ``theta`` (rad from vertical, positive back)."""
    return np.array([PIVOT[0] + HANDLE_L * math.sin(theta), HANDLE_KNOB_Y, PIVOT[2] + HANDLE_L * math.cos(theta)])


def _base_shapes() -> tuple:
    # Division dots round the dial every 5 divisions (18 degrees), larger every 10, so a camera can judge the long
    # pointer's distance from C: the manual's tolerance is 5 divisions.
    # (on the face's front, which is 4 mm in front of DIAL_C; the long pointer turns 1 mm in front of them)
    dots = tuple(_cyl(DIAL_C + (-0.0045, (DIAL_R - 0.005) * math.sin(math.radians(18 * k)),
                                (DIAL_R - 0.005) * math.cos(math.radians(18 * k))),
                      0.0020 if k % 2 == 0 else 0.0012, 0.001, BLACK, axis="X", material="cast")
                 for k in range(1, 20))
    marks = (
        *dots,
        _box(DIAL_C + (-0.005, -0.003, DIAL_R - 0.009), DIAL_C + (-0.004, 0.003, DIAL_R - 0.001), BLACK),  # C, at the top
        _box(SHORT_C + (-0.005, -0.0025, -0.0025), SHORT_C + (-0.004, 0.0025, 0.0025), BLACK),  # short dial: black dot
        _box(SHORT_C + (-0.005, -0.0125, -0.0025), SHORT_C + (-0.004, -0.0075, 0.0025), RED),  # red dot, a quarter turn on
    )
    return (
        _box((BASE_FRONT, -HALF_W, TOP), (BACK, HALF_W, BASE_TOP), GREY),  # base casting
        _cyl((AX, 0.0, (BASE_TOP + WHEEL_Z - 0.009) / 2), 0.030, WHEEL_Z - 0.009 - BASE_TOP, GREY, material="cast"),
        _box((COLUMN_FRONT, -HALF_W, BASE_TOP), (BACK, HALF_W, HEAD_LO), GREY),  # column
        _box((HEAD_FRONT, -HALF_W, HEAD_LO), (BACK, HALF_W, HEAD_HI), GREY),  # head
        _cyl((AX, 0.0, (NOSE_LO + HEAD_LO) / 2), 0.012, HEAD_LO - NOSE_LO, STEEL),  # indenter holder
        _cyl((AX, 0.0, (TIP_Z + TIP_CLEAR + NOSE_LO) / 2), 0.0035, NOSE_LO - TIP_Z - TIP_CLEAR, DARK),  # indenter
        _cyl(DIAL_C + (-0.002, 0.0, 0.0), DIAL_R, 0.004, WHITE, axis="X", material="cast"),  # dial face
        *marks,
        _cyl((PIVOT[0], PIVOT[1] + 0.0075, PIVOT[2]), 0.012, 0.015, DARK, axis="Y"),  # handle boss
    )


BASE = Link("tester", 60.0, _base_shapes())  # 132 lb
WHEEL = Link("wheel", 0.8, (
    _cyl((AX, 0.0, WHEEL_Z), 0.022, 0.018, DARK),  # hub
    *(Shape("box", (AX - 0.5 * WHEEL_KNOB_R * math.cos(p), 0.5 * WHEEL_KNOB_R * math.sin(p), WHEEL_Z),
            (WHEEL_KNOB_R, 0.012, 0.008), DARK, material="steel", yaw=math.pi - p) for p in KNOB_ANG0),  # spokes
))
KNOBS = tuple(Link(f"knob{i}", 0.02, (
    _cyl((*knob_xy(p), (KNOB_LO + KNOB_HI) / 2), KNOB_R, KNOB_HI - KNOB_LO, BLACK, material="rubber"),
)) for i, p in enumerate(KNOB_ANG0))
ANVIL = Link("anvil", 0.6, (
    _cyl((AX, 0.0, ANVIL_TOP - 0.0125), ANVIL_R, 0.025, STEEL),  # flat anvil
    _cyl((AX, 0.0, (WHEEL_Z + 0.009 + ANVIL_TOP - 0.025) / 2), 0.012, ANVIL_TOP - 0.025 - WHEEL_Z - 0.009, STEEL),
))
HANDLE = Link("handle", 0.15, (
    Shape("capsule", (PIVOT[0], PIVOT[1], PIVOT[2] + HANDLE_L / 2), (0.006, HANDLE_L), DARK, material="steel"),
))
HANDLE_KNOB = Link("handle_knob", 0.03, (
    _cyl((PIVOT[0], HANDLE_KNOB_Y, PIVOT[2] + HANDLE_L), 0.012, 0.030, BLACK, axis="Y", material="rubber"),
))
LONG = Link("long_pointer", 0.003, (_box(DIAL_C + (-0.006, -0.0015, -0.006), DIAL_C + (-0.005, 0.0015, 0.040), BLACK),))
SHORT = Link("short_pointer", 0.002, (
    _box(SHORT_C + (-0.0075, -0.001, 0.0), SHORT_C + (-0.0065, 0.001, 0.010), RED),
))

JOINTS = (
    # The wheel's joint turns about +z (anticlockwise from above): the scorer's clockwise angle is its negative.
    # Damping 0.2 N m s/rad: under 0.01 N m at the demo's creep, but it stops a torque left on the wheel from spinning it.
    Joint("wheel", "revolute", "wheel", (AX, 0.0, WHEEL_Z), "Z", -S.wheel_for(S.TRAVEL_STOP),
          -S.wheel_for(-0.0048), damping=0.2, friction=0.05),
    *(Joint(f"knob{i}", "revolute", f"knob{i}", (*knob_xy(p), WHEEL_Z), "Z", -6.2, 6.2, damping=0.0005,
            parent="wheel") for i, p in enumerate(KNOB_ANG0)),
    # The anvil follows the screw: its spring target is set from the wheel before every control step. Once the block
    # meets the indenter's point the spring is what presses it there: its rate puts the 98.07 N preliminary force on the
    # block at the three-turn travel (S.ANVIL_RATE).
    Joint("anvil", "prismatic", "anvil", (AX, 0.0, ANVIL_TOP), "Z", -0.0050, S.TRAVEL_STOP + S.START_GAP + 0.0005,
          stiffness=S.ANVIL_RATE, damping=2.0e3),
    Joint("handle", "revolute", "handle", tuple(PIVOT), "Y", S.HANDLE_FORWARD, S.HANDLE_BACK, damping=0.5,
          friction=0.2, start=S.HANDLE_FORWARD),
    Joint("handle_knob", "revolute", "handle_knob", (PIVOT[0], HANDLE_KNOB_Y, PIVOT[2] + HANDLE_L), "Y", -1.5, 1.5,
          damping=0.0005, parent="handle"),
    Joint("long_pointer", "revolute", "long_pointer", tuple(DIAL_C), "X", -math.pi - 0.2, math.pi + 0.2,
          stiffness=0.05, damping=0.001),
    Joint("short_pointer", "revolute", "short_pointer", tuple(SHORT_C), "X", -0.2, S.SHORT_RED + 0.5,
          stiffness=0.02, damping=0.0005),
)
TESTER = mech.mechanism("tester", (0.0, 0.0, 0.0), BASE, (WHEEL, *KNOBS, ANVIL, HANDLE, HANDLE_KNOB, LONG, SHORT),
                        JOINTS, MATERIALS, color=GREY)
BLOCK_XY = (AX + 0.002, 0.006)  # the indenter meets it 6 mm from its centre: a fresh spot
BLOCK = Cylinder("block", pos=(*BLOCK_XY, ANVIL_TOP + BLOCK_H / 2), radius=S.BLOCK_RADIUS, height=BLOCK_H,
                 mass=0.25, color=STEEL)  # 64 x 10 mm steel: 0.25 kg
OBJECTS = [TESTER, BLOCK]


# -- reading the state ----------------------------------------------------------------------------------------------------
def wheel_cw(state) -> float:
    return -float(state.joints.get("wheel", 0.0))


def handle_angle(state) -> float:
    return float(state.joints.get("handle", S.HANDLE_FORWARD))


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(quat_xyzw, float) / np.linalg.norm(quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def seated(block, tester) -> bool:
    """The block lies flat on the anvil: its bottom within 1.5 mm of the anvil's top, tilted under 3 degrees, its
    centre over the anvil."""
    r = _rot(block.quat_xyzw)
    anvil_top = ANVIL_TOP + float(tester.joints.get("anvil", 0.0))
    bottom = float(block.pos[2]) - BLOCK_H / 2
    over = float(np.hypot(block.pos[0] - AX, block.pos[1])) < ANVIL_R
    return bool(abs(bottom - anvil_top) < 0.0015 and r[2, 2] > math.cos(math.radians(3.0)) and over)


def spot(block) -> np.ndarray:
    """The indenter's axis in the block's frame (m, xy)."""
    r = _rot(block.quat_xyzw)
    return (r.T @ (np.array([AX, 0.0, block.pos[2]]) - np.asarray(block.pos, float)))[:2]


def control_points(tester) -> list:
    """The handwheel's knob centres and the handle's knob, where grippers must not be at the end."""
    pts = [np.array([*knob_xy(p), GRIP_Z]) for p in knob_angles(wheel_cw(tester))]
    return pts + [handle_knob(handle_angle(tester))]


def hands_clear(tcps, tester) -> bool:
    return all(np.linalg.norm(t - p) >= S.CLEAR for t in tcps for p in control_points(tester))


# -- the record and the coupling, once per control step ----------------------------------------------------------------
RECORD = S.HardnessRecord()
_LAST = {"t": None}


_WHEEL = {"cw": None, "torque": 0.0, "stale": True}  # the wheel's last angle, the torque held on it, and whether a
# torque from before a reset may still be held (the next step writes one either way)


def _clear_record() -> None:
    RECORD.reset()
    _LAST["t"] = None
    _WHEEL["cw"], _WHEEL["torque"], _WHEEL["stale"] = None, 0.0, True


mech.on_reset(_clear_record)


def update_record(states, t: float | None) -> S.HardnessRecord:
    """Add a control step to the record, once per simulated time ``t`` (states are read more than once a step)."""
    if t is not None and _LAST["t"] is not None and t <= _LAST["t"] + 1e-9:
        return RECORD
    dt = S.DT if t is None or _LAST["t"] is None else t - _LAST["t"]
    _LAST["t"] = t
    tester, block = states["tester"], states["block"]
    RECORD.step(wheel_cw(tester), handle_angle(tester), seated(block, tester), spot(block), dt=dt)
    return RECORD


def _twist_wheel(environment, torque_z: float) -> None:
    """Hold ``torque_z`` (N m about the world's +z: anticlockwise from above) on the handwheel at every physics step
    until changed."""
    import torch

    asset = environment.unwrapped.scene[TESTER.name]
    body = list(asset.body_names).index("wheel")
    t = torch.tensor([[[0.0, 0.0, float(torque_z)]]], device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.zeros_like(t), torques=t,
                                                                 body_ids=[body], is_global=True)


def couple(environment) -> None:
    """Before each control step: the anvil to where the screw puts it, the screw's torque against the wheel's turn,
    and the pointers to what the dial would show."""
    j = mech.joint_positions(environment, TESTER)
    w = -float(j["wheel"])
    rise = S.travel(w)
    asset = environment.unwrapped.scene[TESTER.name]
    speed_cw = -float(mech._numpy(asset.data.joint_vel)[0][list(asset.joint_names).index("wheel")])
    _WHEEL["cw"] = w
    torque = S.wheel_torque(rise, speed_cw)
    if torque or _WHEEL["torque"] or _WHEEL["stale"]:
        # against the raising turn: clockwise from above is the joint's negative, so it is held anticlockwise (+z)
        _twist_wheel(environment, torque)
        _WHEEL["torque"], _WHEEL["stale"] = torque, False
    mech.set_targets(environment, TESTER, {
        "anvil": w * S.PITCH / (2 * math.pi),
        "long_pointer": S.long_pointer(rise, RECORD.depth()),
        "short_pointer": S.short_pointer(rise),
    })


mech.on_step(couple)


def _sim_time(env) -> float | None:
    base = env.unwrapped
    try:
        return float(base.episode_length_buf[0]) * float(base.step_dt)
    except (AttributeError, TypeError, IndexError):
        sim = getattr(base, "sim", None)
        return float(sim.current_time) if sim is not None and hasattr(sim, "current_time") else None


def _tcps(env) -> list:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    return [runtime.tcp_world(base.scene[f"{side}_robot"]) - origin for side in ("left", "right")]


def _patch_scene(scene) -> None:
    if getattr(scene, "_rockwell_patched", False):
        return
    stock_states = scene.object_states

    def object_states(env, names):
        states = stock_states(env, names)
        return {**states, "tcps": _tcps(env), "hardness_record": update_record(states, _sim_time(env))}

    scene.object_states = object_states
    scene._rockwell_patched = True


# WHY through mechanism's own import hook: two hooks that each look the module up through the other finders called
# each other without end (copy-id-card, official sim, 2026-11-20).
_mech_scene_patch = mech._PATCHES["yam_arena.scene"]


def _scene_patches(scene) -> None:
    _mech_scene_patch(scene)
    _patch_scene(scene)


mech._PATCHES["yam_arena.scene"] = _scene_patches
if "yam_arena.scene" in sys.modules:
    _scene_patches(sys.modules["yam_arena.scene"])


# -- scoring --------------------------------------------------------------------------------------------------------------
def _args(objects):
    rec = objects.get("hardness_record") or RECORD
    tester, block = objects["tester"], objects["block"]
    tcps = objects.get("tcps") or []
    return rec, wheel_cw(tester), handle_angle(tester), seated(block, tester), hands_clear(tcps, tester)


def _debug(objects, g) -> None:
    path = os.environ.get("ROCKWELL_DEBUG")
    if not path:
        return
    rec, w, h, on, clear = _args(objects)
    a = rec.attempts[-1] if rec.attempts else None
    with open(path, "a") as f:
        f.write(f"t={rec.t:.2f} grade={g} wheel_deg={math.degrees(w):.2f} rise_um={S.travel(w) * 1e6:.1f} "
                f"wheel_torque={_WHEEL['torque']:.3f} anvil_um={objects['tester'].joints.get('anvil', 0.0) * 1e6:.1f} "
                f"div_from_c={S.divisions_from_c(S.travel(w)):.1f} handle_deg={math.degrees(h):.1f} seated={on} "
                f"clear={clear} attempt={a}\n")


def score(objects) -> float:
    g = S.grade(*_args(objects))
    _debug(objects, g)
    return min(g, S.PARTIAL_CAP)


def success(objects) -> bool:
    return S.grade(*_args(objects)) == 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------------
def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle (as in take-throat-swab)."""
    if getattr(yam_demo, "_matrix_grips", False):
        return
    plain = yam_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    yam_demo.grip_rotation = grip_rotation
    yam_demo._matrix_grips = True


_allow_matrix_grips()


def _frame(z, x) -> np.ndarray:
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - np.dot(x, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# Both grips point horizontally with the fingers closing across a knob: the handwheel's from the front (+x), the
# handle's from the right (+y). The knobs spin, so the grips keep their orientation along each arc
# (tools/scan_reach.py: the best reachable orientations).
WHEEL_GRIP = _frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
HANDLE_GRIP = _frame((0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
BACK_OFF = 0.06
PARK = {"left": (0.18, 0.26, TOP + 0.15), "right": (0.18, -0.26, TOP + 0.15)}
# While the left hand raises the block, the right hand holds its wrist camera 17 cm from the dial, looking at it 20 deg
# from the right and 20 deg down: the dial is about 100 px across in the policy's 320 x 176 image, against about 20 in
# the top camera (tools/scan_dial_view.py: 37 deg of joint margin).
DIAL_VIEW = dict(dist=0.17, yaw=math.radians(20.0), pitch=math.radians(20.0), roll=0.0)
# In the official sim the left hand leaves the handwheel short of its last waypoint: 1.6 degrees with a free wheel
# (9 divisions; run rockwell-demo1, 2026-11-22), 2.5 degrees against the screw's torque (aimed 4.4 past, it settled
# 2.5 short, 10.9 divisions past C; the overshoot probe the same: rockwell-rev runs, 2026-11-23 00:57). So the demo
# aims 2.5 degrees past the set point. ROCKWELL_DEMO_TRIM_DEG overrides it (the offline
# replay, whose hand reaches its IK pose exactly, uses 0).
SETTLE_TRIM_DEG = 2.5
SET_CW = S.wheel_for(S.PRE_TRAVEL) + math.radians(float(os.environ.get("ROCKWELL_DEMO_TRIM_DEG", SETTLE_TRIM_DEG)))
# Probe settings (the submitted demo uses the defaults): ROCKWELL_DEMO_OVERSHOOT_DIV raises the block that many
# divisions past C before the load; ROCKWELL_DEMO_HOLD_S changes how long the full load is held.
OVERSHOOT = float(os.environ.get("ROCKWELL_DEMO_OVERSHOOT_DIV", "0")) * S.DIVISION * 2 * math.pi / S.PITCH
# 9.9 s at the back stop: the load goes on over 5 s from the 30 degree release, then dwells about 5 s (the manual's
# hold; run rockwell-demo4 at 8.65 s dwelt 3.75 s).
HOLD_S = float(os.environ.get("ROCKWELL_DEMO_HOLD_S", "9.9"))


def _pt(v) -> tuple:
    return tuple(float(a) for a in v)


def _knob_point(ang: float) -> np.ndarray:
    return np.array([*knob_xy(ang), GRIP_Z])


def _stroke(ang_from: float, ang_to: float, label: str, fine_last_deg: float = 0.0) -> list:
    """Take the knob standing at ``ang_from``, carry it round to ``ang_to`` (rad), let go and back off. The last
    ``fine_last_deg`` degrees go in half-degree steps, slowly, as an operator creeps up on the set point."""
    p0 = _knob_point(ang_from)
    out = [
        Waypoint(1.5, _pt(p0 - (BACK_OFF, 0, 0)), FINGER_OPEN, f"{label}: in front of the knob", WHEEL_GRIP),
        Waypoint(1.0, _pt(p0), FINGER_OPEN, f"{label}: round the knob", WHEEL_GRIP),
        Waypoint(1.0, _pt(p0), FINGER_CLOSED, f"{label}: grip it", WHEEL_GRIP),
    ]
    span = math.degrees(ang_to - ang_from)
    coarse = span - math.copysign(fine_last_deg, span) if fine_last_deg else span
    steps = max(1, int(abs(coarse) // 5))
    for k in range(1, steps + 1):
        ang = ang_from + math.radians(coarse) * k / steps
        out.append(Waypoint(0.3, _pt(_knob_point(ang)), FINGER_CLOSED, f"{label}: turn", WHEEL_GRIP))
    if fine_last_deg:
        n = int(round(fine_last_deg / 0.5))
        start = ang_from + math.radians(coarse)
        for k in range(1, n + 1):
            ang = start + (ang_to - start) * k / n
            out.append(Waypoint(0.4, _pt(_knob_point(ang)), FINGER_CLOSED, f"{label}: creep", WHEEL_GRIP))
    p1 = _knob_point(ang_to)
    out += [
        Waypoint(0.8, _pt(p1), FINGER_CLOSED, f"{label}: settle", WHEEL_GRIP),
        Waypoint(1.0, _pt(p1), FINGER_OPEN, f"{label}: let go", WHEEL_GRIP),
        Waypoint(1.0, _pt(p1 - (BACK_OFF, 0, 0)), FINGER_OPEN, f"{label}: back off", WHEEL_GRIP),
    ]
    return out


def raise_strokes(target_cw: float) -> list:
    """Clockwise strokes from the front-right (-60 deg) to at most the front-left (+60 deg), knob after knob, until
    the wheel stands at ``target_cw``."""
    out, w = [], 0.0
    lo, hi = math.radians(-60.0), math.radians(60.0)
    n = 1
    while w < target_cw - 1e-9:
        step = min(hi - lo, target_cw - w)
        last = w + step >= target_cw - 1e-9
        out += _stroke(lo, lo + step, f"raise {n}", fine_last_deg=6.0 if last else 0.0)
        w += step
        n += 1
    return out


def lower_strokes(from_cw: float) -> list:
    """Anticlockwise strokes back to the start: the knob nearest the front-right side first, then knob by knob from
    the front-left (+60 deg) to the front-right (-60 deg)."""
    out, w = [], from_cw
    lo, hi = math.radians(-60.0), math.radians(60.0)
    n = 1
    while w > 1e-9:
        # the knob now in the front sector nearest the right end: the one we can carry furthest anticlockwise
        angs = [((p + w + math.pi) % (2 * math.pi)) - math.pi for p in KNOB_ANG0]
        reach = [p for p in angs if lo - 1e-9 <= p <= hi + 1e-9 and p > lo + 1e-6]
        start = max(reach)
        step = min(start - lo, w)
        out += _stroke(start, start - step, f"lower {n}")
        w -= step
        n += 1
    return out


def _left() -> list:
    down = yam_demo.grip_rotation(0.0)
    out = [*raise_strokes(SET_CW + OVERSHOOT), Waypoint(1.5, PARK["left"], FINGER_OPEN, "park", down)]
    stay = max(1.0, _right_duration() - sum(w.duration for w in out))
    return [*out, Waypoint(stay, PARK["left"], FINGER_OPEN, "stay", down)]


# The open fingers straddle the handle's knob front and back, so the hand comes up to it from below: drawing back
# along -y from it runs out of reach 3 cm out (tools/scan_reach.py).
BELOW_KNOB = np.array([0.0, -0.010, -0.050])


def _handle() -> list:
    """The right hand takes the handle's knob, pushes it back, holds the full load, pulls it forward, lets go."""
    k0, k1 = handle_knob(S.HANDLE_FORWARD), handle_knob(S.HANDLE_BACK)
    out = [Waypoint(1.5, _pt(k0 + BELOW_KNOB), FINGER_OPEN, "below the handle's knob", HANDLE_GRIP),
           Waypoint(1.0, _pt(k0), FINGER_OPEN, "round the knob", HANDLE_GRIP),
           Waypoint(1.0, _pt(k0), FINGER_CLOSED, "grip it", HANDLE_GRIP)]
    for d in range(-15, 41, 5):
        out.append(Waypoint(2.0 / 12, _pt(handle_knob(math.radians(d))), FINGER_CLOSED, "push the handle back",
                            HANDLE_GRIP))
    out.append(Waypoint(HOLD_S, _pt(k1), FINGER_CLOSED, "hold: the load goes on, then dwells", HANDLE_GRIP))
    for d in range(35, -21, -5):
        out.append(Waypoint(2.5 / 12, _pt(handle_knob(math.radians(d))), FINGER_CLOSED, "pull it forward",
                            HANDLE_GRIP))
    out += [Waypoint(1.0, _pt(k0), FINGER_OPEN, "let go", HANDLE_GRIP),
            Waypoint(1.0, _pt(k0 + BELOW_KNOB), FINGER_OPEN, "drop away below it", HANDLE_GRIP)]
    return out


def dial_view_pose(dist: float, yaw: float, pitch: float, roll: float) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP's position and rotation that put its wrist camera ``dist`` from the dial's centre, looking at it
    from ``yaw`` right of straight on and ``pitch`` down, rolled ``roll`` about the line of sight."""
    from yam_arena import station
    from yam_arena.constants import TCP_OFFSET
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.wrist_camera("right")
    cam_rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))  # opengl: it looks along its -z, up is its +y
    look = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
    z = -look
    up = np.array([0.0, 0.0, 1.0]) - z[2] * z
    up /= np.linalg.norm(up)
    side = np.cross(up, z)
    up = math.cos(roll) * up + math.sin(roll) * side
    r_grip = np.column_stack([np.cross(up, z), up, z]) @ cam_rot.T
    grip = DIAL_C - dist * look - r_grip @ np.array(cam.pos)
    return grip + r_grip @ np.array(TCP_OFFSET), r_grip


def _right() -> list:
    """The right hand watches the dial while the left raises the block, then works the handle, then lowers the block.
    Lowering is the mirror image of the left hand's raising strokes; the left hand pulling the knob back from the
    front-left lost it in the official sim (the knob slid out of the open end of its fingers, 2026-11-22 runs
    rockwell-demo1/2)."""
    down = yam_demo.grip_rotation(0.0)
    t_raise = sum(w.duration for w in raise_strokes(SET_CW + OVERSHOOT)) + 1.5
    view_p, view_r = dial_view_pose(**DIAL_VIEW)
    return [Waypoint(2.0, _pt(view_p), FINGER_OPEN, "the wrist camera on the dial", view_r),
            Waypoint(t_raise - 3.5, _pt(view_p), FINGER_OPEN, "watch the dial: the preliminary load", view_r),
            Waypoint(1.5, PARK["right"], FINGER_OPEN, "out of the way", down),
            *_handle(),
            *lower_strokes(SET_CW + OVERSHOOT),
            Waypoint(1.5, PARK["right"], FINGER_OPEN, "park", down),
            Waypoint(4.0, PARK["right"], FINGER_OPEN, "stay", down)]


def _right_duration() -> float:
    return sum(w.duration for w in _right())


def demo() -> dict:
    return {"left": _left(), "right": _right()}
