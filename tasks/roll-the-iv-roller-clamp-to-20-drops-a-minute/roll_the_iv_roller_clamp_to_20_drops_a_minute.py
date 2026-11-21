"""On the station table, straight ahead of the robot: the roller clamp of a primed IV line, lying across the table.
The clamp is a white plastic housing 6.4 cm long with a blue wheel in it, rolled fully shut. Its tubing is taped to
the table 4 cm beyond each end of the clamp. To the robot's left, the line runs along the table to an IV pole standing
beside the table's left edge, where a 1,000 mL saline bag hangs with its drip chamber 30 cm above the table, in view
of the top camera, in front of a matte black board that makes the clear drops show. To the robot's right, the line
runs off the table's right edge and ends inside a beaker on the floor.

- Rolling the wheel toward the robot's left (toward the bag) opens the clamp; toward the right shuts it. The wheel
  stays where it is left.
- The drip chamber shows the rate: 10 drops make 1 mL. Fully shut, nothing drips. Opened a little, a drop grows at the
  top of the chamber and falls every few seconds; the rate climbs steeply as the wheel goes on; opened too far, the
  drops run together into a stream.
- The clamp lies loose on the table, so pushing the wheel pushes the whole clamp unless the clamp is held. Hold the
  housing still with one hand and roll the wheel with the other.
- The trial ends when both arms are back in their start pose, or at the run limit; then the drops are counted.

Run it:  inspect-robots run --embodiment yam_arena -E task=roll_the_iv_roller_clamp_to_20_drops_a_minute.py ...
"""

from __future__ import annotations

import math
import pathlib
import sys
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import run_record  # noqa: E402
from benchkit import _ENV, Part, Slider, ToolState, _np, on_reset, on_step, recolor, tool  # noqa: E402
from yam_arena import analysis, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN, READY  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the trial ends at the start pose (grade() freezes the outcome) or the run limit
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0  # the run commands set -E control_hz=20; times below are counted in its steps
DT = 1.0 / CONTROL_HZ

INSTRUCTION = "Set the IV to drip at 20 drops a minute with the roller clamp, then let go and return both arms to their start pose."

# -- the scoring thresholds -------------------------------------------------------------------------------------------
TARGET = 20  # drops a minute
BANDS = ((18, 22, 1.0), (16, 24, 0.75), (10, 30, 0.5))  # drops counted in the minute: inclusive range, score
DRIPPING = 0.25  # any other count of 1 up to OVER, or a gripper still on or near the clamp or the line at the run limit
OVER = 60  # drops counted in the minute: more than this (three times the order) is a dangerous over-rate, score 0
CLEAR_GAP = 0.050  # m: clear means no gripper this close to the clamp or its free tubing ...
LINE_GAP = 0.015  # m: ... and none this close to the rest of the line, the bag or its chamber (touching, in the sim)
START_GAP = 0.050  # m: each gripper this close to where it started, after one has come to the clamp, ends the trial
COUNT_S = 60.0  # the count runs for one minute
STREAM_S = 3.0  # a continuous stream for longer than this: score 0
STREAM_STEPS = int(round(STREAM_S * CONTROL_HZ))
PULL_GAP = 0.030  # m: the tubing at a tape strip moved further than this has been pulled out from under it: score 0
# a kink or squeeze in the line outside the clamp at the end scores 0 (sim: a gripper reaching the tube's centreline)

# -- how the drip rate follows the wheel (README: fidelity) -----------------------------------------------------------
# The wheel pinches the tube against the floor of the housing. Rolled from shut, the tube stays closed for the first
# T_OPEN of travel; then the gap opens in proportion to travel. Flow through a thin slit goes as the head times the
# cube of its gap, so the rate is R_PER_M x HEAD x ((travel - T_OPEN) / SPAN)^3, until the drops run together into a
# stream at R_STREAM. T_OPEN, SPAN and R_PER_M are a model, to be calibrated on the real clamp.
CHAMBER_MID = 0.30  # m above the table: the drip chamber's middle, where its saline stands (half full)
TABLE_H = TOP - station.FLOOR_Z  # m: the table top above the floor
LINE_END = 0.003 + 0.060  # m above the floor: the line's end, 6 cm above the floor of the beaker on the floor
HEAD = CHAMBER_MID + TABLE_H - LINE_END  # m: the chamber's saline above the line's end, about 1 m as at a bedside
TRAVEL = 0.020  # m: the wheel's whole run along the housing
T_OPEN = 0.006  # m of travel before the first drop
SPAN = 0.010  # m of travel past T_OPEN at which ...
R_PER_M = 230.0  # ... the rate is this many drops a minute per metre of head
R_STREAM = 180.0  # drops a minute (3 a second) at which the drops run together
DROPS_PER_ML = 10  # the set's printed drop factor


def travel_for(rate: float) -> float:
    """The wheel travel (m) that gives ``rate`` drops a minute."""
    return T_OPEN + SPAN * (rate / (R_PER_M * HEAD)) ** (1.0 / 3.0)


T_STREAM = travel_for(R_STREAM)  # m of travel at which the drops run together: about 15.2 mm


def drip_rate(travel: float) -> float:
    """Drops a minute with the wheel ``travel`` m from shut (inf: a stream)."""
    if travel <= T_OPEN:
        return 0.0
    if travel >= T_STREAM:
        return math.inf
    return R_PER_M * HEAD * ((travel - T_OPEN) / SPAN) ** 3


def counted(travel: float) -> int:
    """Drops in the counted minute with the wheel left at ``travel``: the rate is steady once nothing touches it."""
    rate = drip_rate(travel)
    return 10**6 if math.isinf(rate) else int(round(rate * COUNT_S / 60.0))


def _fresh() -> SimpleNamespace:
    return SimpleNamespace(step=0, stream_steps=0, streamed=False, pulled=False, clear=True, kinked=False, touched=False,
                           ended=False, end_travel=0.0, phase=0.0, drops=0, travel=0.0, fall=0, shown={})


LOG = _fresh()
LATEST: dict = {"states": None}  # the last object states, which the scripted demo reads to plan


def zeroed() -> bool:
    """The latched zeros: a stream for more than 3 s, or the tubing pulled out from under a tape strip."""
    return LOG.streamed or LOG.pulled


def grade(objects) -> float:
    """The score as the outcome table states it: frozen when both arms are back in their start pose, or, before that,
    what the trial would score if the run limit came now."""
    if zeroed():
        return 0.0
    n = counted(LOG.end_travel if LOG.ended else objects["roller_clamp"].sliders["wheel"])
    if n < 1 or n > OVER or (not LOG.ended and LOG.kinked):
        return 0.0
    if not LOG.ended and not LOG.clear:
        return DRIPPING  # a gripper still on or near the clamp or the line at the run limit: the operator lifts it off
    for lo, hi, value in BANDS:
        if lo <= n <= hi:
            return value
    return DRIPPING


def score(objects) -> float:
    return grade(objects)


def success(objects) -> bool:
    return LOG.ended and grade(objects) >= 1.0


def distance(objects) -> float:
    """Metres of wheel travel from where it drips at 20 drops a minute."""
    return float(abs(objects["roller_clamp"].sliders["wheel"] - travel_for(TARGET)))


@on_reset
def _reset(states) -> None:
    global LOG
    LOG = _fresh()
    LATEST["states"] = states or None
    for name in ("drop", "drop_fall", "stream"):
        recolor(name, CHAMBER)
    run_record.reset(globals())


@on_step
def _watch(states, grips=None, at=None) -> None:
    """Follow the trial: the drip, a stream, the tubing at its strips and the grippers (``grips`` their hulls, ``at``
    their positions; read from the arms if not given). It ends when both are back at the start, after one has come to
    the clamp; then only the drip is drawn."""
    LATEST["states"] = states
    clamp = states["roller_clamp"]
    LOG.step += 1
    LOG.travel = clamp.sliders["wheel"]
    rate = drip_rate(LOG.travel)
    _show_drip(rate)
    run_record.tick(globals(), states, rate)
    if LOG.ended:
        return
    LOG.stream_steps = LOG.stream_steps + 1 if math.isinf(rate) else 0
    if LOG.stream_steps > STREAM_STEPS:
        LOG.streamed = True
    if pulled_loose(clamp):
        LOG.pulled = True
    if grips is None:
        grips, at = grippers()
    near = not clear_of(clamp, grips)
    LOG.touched = LOG.touched or near
    LOG.clear = not near and off_line(grips)
    LOG.kinked = kinked(grips)
    if LOG.touched and LOG.clear and at_start(at or []):
        LOG.ended, LOG.end_travel = True, LOG.travel
        run_record.ended(globals(), states)


def pulled_loose(clamp: ToolState) -> bool:
    """Either end of the free tubing (at its tape strip) moved more than 30 mm from where it is taped."""
    return any(float(np.linalg.norm(clamp.point(n) - np.asarray(TAPED[n]))) > PULL_GAP for n in TAPED)


# -- what the scorer reads: the grippers ------------------------------------------------------------------------------

GRIPPER_LINKS = ["gripper", "tip_left", "tip_right"]


def gripper(q6, finger: float, base) -> tuple:
    """One gripper's hulls in the world and its position, for arm joints ``q6`` and finger joint ``finger`` (m)."""
    q = {n: float(v) for n, v in zip(ARM_JOINTS, q6, strict=True)} | {j: float(finger) for j in FINGER_JOINTS}
    poses = analysis.fk_link_poses_world(q, base)
    return analysis.world_hulls(poses, GRIPPER_LINKS), poses["gripper"][:3, 3]


START = {arm: gripper(READY, FINGER_OPEN, station.BASES[arm])[1] for arm in ("left", "right")}  # where each starts


def grippers() -> tuple:
    """Both grippers' hulls and positions, from the arms' joints. The object states don't include the arms."""
    env = _ENV["env"]
    if env is None:
        return [], []
    hulls, at = [], []
    for arm in ("left", "right"):
        robot = env.unwrapped.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        joints = _np(robot.data.joint_pos)[0]
        finger = float(np.mean([joints[names.index(j)] for j in FINGER_JOINTS]))
        h, p = gripper(joints[[names.index(j) for j in ARM_JOINTS]], finger, station.BASES[arm])
        hulls.append(h)
        at.append(p)
    return hulls, at


def at_start(at) -> bool:
    """Both grippers (left, right) within 50 mm of where they started."""
    return len(at) == 2 and all(float(np.linalg.norm(np.asarray(p) - START[a])) <= START_GAP
                                for a, p in zip(("left", "right"), at))


def points_to_grips(points, grips, beyond: float = math.inf) -> float:
    """Metres from the nearest point to the nearest gripper hull (negative inside). Hulls whose bounding box is
    ``beyond`` m or more from the points' are skipped: they can't be nearer than that."""
    best = math.inf
    lo, hi = points.min(axis=0), points.max(axis=0)
    for hulls in grips:
        for _, h in hulls:
            v = getattr(h, "vertices", None)
            if v is not None and np.maximum(v.min(axis=0) - hi, lo - v.max(axis=0)).max() >= beyond:
                best = min(best, beyond)
                continue
            best = min(best, float((points @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1).min()))
    return best


def clamp_points(clamp: ToolState) -> np.ndarray:
    """A 3 x 3 x 3 grid over the housing, the free tubing and the wheel where it is, in the world."""
    g = np.array([[a, b, c] for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)], dtype=float) / 2
    boxes = [(np.asarray(b[0]), np.asarray(b[1])) for b in CLAMP.boxes if not (len(b) > 3 and b[3].ghost)]
    boxes.append((np.array((WHEEL_X0 + clamp.sliders["wheel"], 0.0, WHEEL_Z)), np.array(WHEEL)))
    return np.array([clamp.to_world(c + s * k) for c, s in boxes for k in g])


def clear_of(clamp: ToolState, grips) -> bool:
    """No gripper within 50 mm of the clamp or its free tubing."""
    return bool(not grips or points_to_grips(clamp_points(clamp), grips, beyond=CLEAR_GAP) >= CLEAR_GAP)


def box_points(box, step: float = 0.02) -> np.ndarray:
    """A grid at most ``step`` m apart through a fixed box, in the world."""
    axes = [np.linspace(-s / 2, s / 2, max(2, math.ceil(s / step) + 1)) for s in box.size]
    g = np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    return np.asarray(box.pos) + g @ np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])


def off_line(grips) -> bool:
    """No gripper touching (within 15 mm of) the rest of the line, the slide clamp, the chamber or the bag."""
    return all(not grips or points_to_grips(p, grips, beyond=LINE_GAP) >= LINE_GAP for p in LINE_POINTS)


def axis_points(box, step: float = 0.005) -> np.ndarray:
    """Points ``step`` m apart along a fixed tube's centreline, in the world."""
    k = int(np.argmax(box.size))
    local = np.zeros((max(2, math.ceil(box.size[k] / step) + 1), 3))
    local[:, k] = np.linspace(-box.size[k] / 2, box.size[k] / 2, len(local))
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    return np.asarray(box.pos) + local @ np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])


def kinked(grips) -> bool:
    """A gripper reaching the centreline of the line's tubing outside the clamp: squeezing it at least half shut."""
    return bool(grips) and any(points_to_grips(p, grips, beyond=1e-6) < 0.0 for p in LINE_AXES)


# -- the props, at the real items' sizes ------------------------------------------------------------------------------

CATALOGUE = {"bag": "2B1324X", "set": "2C8537", "pole": "13033", "beaker": "1201-2000"}

CLAMP_WHITE = (0.93, 0.94, 0.95)
WHEEL_BLUE = (0.20, 0.42, 0.80)
TUBE = (0.80, 0.86, 0.90)  # clear PVC full of saline
TAPE = (0.78, 0.76, 0.70)
STEEL = (0.72, 0.74, 0.77)
POLE_GREY = (0.62, 0.64, 0.67)
BAG = (0.86, 0.90, 0.94)
# the chamber is clear: in front of the matte black board it looks dark, and a clear drop shows only as a grey glint
BOARD_BLACK = (0.04, 0.04, 0.05)
CHAMBER = (0.18, 0.19, 0.21)  # clear PVC seen against the board
SALINE = (0.25, 0.27, 0.29)  # the saline in its lower half, a little lighter at its surface
DROP = (0.55, 0.56, 0.57)  # a clear drop: its highlights against the board, averaged over its 1-2 px; no colour
BEAKER = (0.90, 0.90, 0.86)  # translucent polypropylene
PORT_BLUE = (0.30, 0.50, 0.85)

# -- the roller clamp of the 2C8537 set (tool frame: x along the clamp toward the bag, z up, origin on the table under
#    the middle of the housing). It lies across the table: tool x is world +y. ------------------------------------
CLAMP_XY = (0.28, 0.01)
CLAMP_YAW = math.pi / 2
LENGTH, WIDTH = 0.064, 0.020  # the housing
BASE_T = 0.003  # its floor
WALL_T, WALL_TOP = 0.003, 0.013  # side walls, up to just above the wheel's axle
TUBE_OD = 0.0041  # standard-bore PVC IV tubing
STUB = 0.040  # free tubing either side, from the housing to its tape strip
WHEEL_D, WHEEL_W = 0.014, 0.012  # the wheel's diameter (across its flats) and width
WHEEL = (WHEEL_D, WHEEL_W, WHEEL_D)
DIAG = 0.006  # m: its 45-degree faces from the axle (an octagon of four bars: those faces 7.8 mm wide, the rest 3)
SIDE = DIAG * math.sqrt(2) - WHEEL_D / 2
BARS = [(WHEEL_D, WHEEL_W, 2 * SIDE), (2 * DIAG, WHEEL_W, math.sqrt(2) * (WHEEL_D / 2 - SIDE))]
WHEEL_Z = 0.012  # its axle above the table
WHEEL_X0 = -LENGTH / 2 + 0.004 + WHEEL_D / 2  # its axle when rolled fully shut, 4 mm from the housing's end
WHEEL_BREAKAWAY = 1.5  # N along the housing: the wheel stays put until pushed harder (an estimate: to be measured)
WHEEL_DAMPING = 0.0  # N per m/s: no viscous drag; the breakaway holds it
SPAWN_GAP = 0.0005  # m: the clamp starts this far above the table and settles onto it
CLAMP_MASS = 0.010  # housing and its free tubing full of saline
WHEEL_MASS = 0.010  # a real wheel is about 2 g; 10 g keeps PhysX contacts steady (README, fidelity)
WHEEL_WEIGHTLESS = True  # its weight never loads its slide (README, fidelity)
FACET = DIAG * math.cos(math.pi / 4)  # the wheel's upper face toward shut, 45 degrees: its centre's offset


def _clamp_boxes() -> list:
    side = WIDTH / 2 - WALL_T / 2
    wall_h = WALL_TOP - BASE_T
    stub_x = LENGTH / 2 + STUB / 2
    return [
        ((0.0, 0.0, BASE_T / 2), (LENGTH, WIDTH, BASE_T), CLAMP_WHITE),
        ((0.0, side, BASE_T + wall_h / 2), (LENGTH, WALL_T, wall_h), CLAMP_WHITE),
        ((0.0, -side, BASE_T + wall_h / 2), (LENGTH, WALL_T, wall_h), CLAMP_WHITE),
        # the tube through the housing, squeezed under the wheel: drawn only
        ((0.0, 0.0, BASE_T + TUBE_OD / 2), (LENGTH, TUBE_OD, TUBE_OD), TUBE, Part(ghost=True)),
        ((stub_x, 0.0, TUBE_OD / 2), (STUB, TUBE_OD, TUBE_OD), TUBE),
        ((-stub_x, 0.0, TUBE_OD / 2), (STUB, TUBE_OD, TUBE_OD), TUBE),
    ]


def _taped_points() -> dict:
    c, s = math.cos(CLAMP_YAW), math.sin(CLAMP_YAW)
    out = {}
    for name, sign in (("bag_side", 1.0), ("beaker_side", -1.0)):
        lx = sign * (LENGTH / 2 + STUB)
        out[name] = (CLAMP_XY[0] + c * lx, CLAMP_XY[1] + s * lx, TOP + SPAWN_GAP + TUBE_OD / 2)
    return out


TAPED = _taped_points()

CLAMP = tool(
    "roller_clamp",
    pos=(*CLAMP_XY, TOP + SPAWN_GAP),
    boxes=_clamp_boxes(),
    sliders=[Slider("wheel", rest=(WHEEL_X0, 0.0, WHEEL_Z), size=BARS[0], axis="X", lower=0.0, upper=TRAVEL,
                    color=WHEEL_BLUE, rate=0.0, damping=WHEEL_DAMPING, mass=WHEEL_MASS, friction=WHEEL_BREAKAWAY,
                    weightless=WHEEL_WEIGHTLESS,
                    boxes=tuple(((0.0, 0.0, 0.0), BARS[k % 2], WHEEL_BLUE, Part(turn=(0.0, k * math.pi / 4, 0.0)))
                                for k in (1, 2, 3)))],
    mass=CLAMP_MASS,
    yaw=CLAMP_YAW,
    friction=1.0,
    points=[("bag_side", (LENGTH / 2 + STUB, 0.0, TUBE_OD / 2)), ("beaker_side", (-LENGTH / 2 - STUB, 0.0, TUBE_OD / 2))],
)


def facet_point(clamp: ToolState, travel: float, press: float = 0.0) -> np.ndarray:
    """World point at the middle of the wheel's upper face toward shut, with the wheel ``travel`` m from shut, moved
    ``press`` m into it along the face's normal: where a fingertip rolls the wheel open."""
    normal = np.array((-1.0, 0.0, 1.0)) / math.sqrt(2.0)
    local = np.array((WHEEL_X0 + travel - FACET, 0.0, WHEEL_Z + FACET)) - press * normal
    return clamp.to_world(local)


# -- the fixed line: taped tubing, the IV pole with its bag and drip chamber, the beaker on the floor -------------------
FLOOR = station.FLOOR_Z
EDGE_Y = station.TABLE_CENTER[1] + station.TABLE_SIZE[1] / 2  # the table's left edge (y); its right edge is -EDGE_Y
CAMERA = station.top_camera().pos  # the top camera, which the drip chamber's face is turned to
POLE_XY = (0.45, 0.67)  # on the floor, 2 cm beyond the table's left edge
HOOK_XY = (0.45, 0.55)  # the hook, on a 12 cm arm over the table: the bag, chamber and hanging tubing hang below it
CHAMBER_LEN, CHAMBER_D = 0.060, 0.018
CHAMBER_TOP = CHAMBER_MID + CHAMBER_LEN / 2  # m above the table
FACE = math.atan2(CAMERA[1] - HOOK_XY[1], CAMERA[0] - HOOK_XY[0])  # yaw of the chamber's face toward the camera
BAG_SIZE = (0.040, 0.140, 0.280)  # 1,000 mL VIAFLEX bag, filled: thick, wide, tall
SPIKE_LEN = 0.045
BAG_BOTTOM = CHAMBER_TOP + SPIKE_LEN
HOOK_Z = BAG_BOTTOM + BAG_SIZE[2] + 0.030
SLIDE_Z = 0.15  # m above the table: the slide clamp, open, on the hanging tubing 12 cm below the chamber
SLIDE = (0.035, 0.013, 0.005)  # its plate, flat round the tube
BEAKER_XY = (0.45, -0.74)  # on the floor, beside the table's right edge
BEAKER_D, BEAKER_H = 0.134, 0.185  # outside diameter and height (to check on the catalogue item)
BEAKER_WALL = 0.003
TAPE_GAP = 0.003  # m: the fixed tubing starts this far past the free tubing's end, so the two never overlap at the start
OFF_EDGE = (BEAKER_XY[0], -EDGE_Y - 0.003)  # where the line leaves the table's right edge
UP_ROUTE = ((TAPED["bag_side"][0], TAPED["bag_side"][1] + TAPE_GAP), (CLAMP_XY[0], 0.20), HOOK_XY)
DOWN_ROUTE = ((TAPED["beaker_side"][0], TAPED["beaker_side"][1] - TAPE_GAP), (CLAMP_XY[0], -0.16), OFF_EDGE)


def _run(name: str, a, b, z: float) -> Box:
    """Tubing lying on the table from ``a`` to ``b`` (x, y), at height ``z`` (its centre)."""
    (ax, ay), (bx, by) = a, b
    length = math.hypot(bx - ax, by - ay)
    return Box(name, pos=((ax + bx) / 2, (ay + by) / 2, z), size=(length + TUBE_OD, TUBE_OD, TUBE_OD), color=TUBE,
               yaw=math.atan2(by - ay, bx - ax))


def _upright(name: str, xy, z0: float, z1: float, color=TUBE, w: float = TUBE_OD, yaw: float = 0.0) -> Box:
    return Box(name, pos=(*xy, (z0 + z1) / 2), size=(w, w, z1 - z0), color=color, yaw=yaw)


def _line() -> list:
    z = TOP + TUBE_OD / 2
    out = [_run(f"line_up_{i}", a, b, z) for i, (a, b) in enumerate(zip(UP_ROUTE, UP_ROUTE[1:]))]
    out += [_run(f"line_down_{i}", a, b, z) for i, (a, b) in enumerate(zip(DOWN_ROUTE, DOWN_ROUTE[1:]))]
    # straight up from the table to the drip chamber, through the slide clamp
    out.append(_upright("line_drop", HOOK_XY, TOP + TUBE_OD, TOP + CHAMBER_TOP - CHAMBER_LEN))
    out.append(Box("slide_clamp", pos=(*HOOK_XY, TOP + SLIDE_Z), size=SLIDE, color=CLAMP_WHITE))
    # off the table's right edge, down to the beaker on the floor, over its rim and down inside
    rim = FLOOR + BEAKER_H + 0.006
    inside = BEAKER_XY[1] + BEAKER_D / 2 - 0.016
    out.append(_upright("line_off_edge", OFF_EDGE, rim, TOP + TUBE_OD))
    out.append(_run("line_beaker_rim", OFF_EDGE, (OFF_EDGE[0], inside), rim))
    out.append(_upright("line_beaker_in", (OFF_EDGE[0], inside), FLOOR + LINE_END, rim))
    # tape strips, across the tubing, 2 mm thick on top of it: by the clamp, then along both runs (the last at the edge)
    for name, p in TAPED.items():
        out.append(Box(f"tape_{name}", pos=(p[0], p[1] + (0.013 if name == "bag_side" else -0.013), TOP + TUBE_OD + 0.001),
                       size=(0.025, 0.020, 0.002), color=TAPE))
    for name, (x, y, yaw) in TAPE_STRIPS:
        out.append(Box(name, pos=(x, y, TOP + TUBE_OD + 0.001), size=(0.020, 0.025, 0.002), color=TAPE, yaw=yaw))
    return out


TAPE_EVERY = 0.10  # m: along the table the line is taped at each bend and at most this far between strips
FIRST_STRIP = 0.010  # m along each run from its start to the middle of the strip by the clamp


def _along(route, s: float) -> tuple:
    """(x, y, yaw) ``s`` m along a route of (x, y) points."""
    for a, b in zip(route, route[1:]):
        n = math.dist(a, b)
        if s <= n:
            return a[0] + (b[0] - a[0]) * s / n, a[1] + (b[1] - a[1]) * s / n, math.atan2(b[1] - a[1], b[0] - a[0])
        s -= n
    raise ValueError(s)


def _strips() -> list:
    """Strips along both runs: at each bend, 3 cm before the hanging tubing's foot, at the table's edge, and evenly
    between."""
    out = []
    for tag, route, end in (("up", UP_ROUTE, 0.030), ("down", DOWN_ROUTE, 0.016)):
        arcs = np.cumsum([0.0] + [math.dist(a, b) for a, b in zip(route, route[1:])])
        spots, at = [FIRST_STRIP, *arcs[1:-1], arcs[-1] - end], []
        for a, b in zip(spots, spots[1:]):
            k = math.ceil((b - a) / TAPE_EVERY - 1e-9)
            at += [a + (b - a) * i / k for i in range(1, k + 1)]
        out += [(f"tape_{tag}_{i}", _along(route, s)) for i, s in enumerate(at)]
    out[-1] = ("tape_edge", out[-1][1])  # the last, where the line leaves the table's edge
    return out


TAPE_STRIPS = _strips()


def _on_face(name: str, below: float, w: float, h: float, color, out: float = 0.0012) -> Box:
    """A thin plate on the chamber's face toward the top camera, centred ``below`` m under the chamber's top."""
    r = CHAMBER_D / 2 + out
    return Box(name, pos=(HOOK_XY[0] + r * math.cos(FACE), HOOK_XY[1] + r * math.sin(FACE), TOP + CHAMBER_TOP - below),
               size=(0.002, w, h), color=color, yaw=FACE)


def _pole() -> list:
    px, py = POLE_XY
    top = TOP + HOOK_Z
    out = [_upright("pole", POLE_XY, FLOOR + 0.06, top + 0.02, POLE_GREY, 0.025)]
    for i, yaw in enumerate((0.0, math.pi / 2)):  # the four-legged base, two crossed bars on casters
        out.append(Box(f"pole_base_{i}", pos=(px, py, FLOOR + 0.045), size=(0.56, 0.035, 0.025), color=POLE_GREY,
                       yaw=yaw))
    out.append(Box("pole_arm", pos=(px, (py + HOOK_XY[1]) / 2, top), size=(0.010, py - HOOK_XY[1] + 0.01, 0.010),
                   color=STEEL))
    out.append(_upright("hook", HOOK_XY, top - 0.030, top, STEEL, 0.006))
    bag_mid = TOP + BAG_BOTTOM + BAG_SIZE[2] / 2
    out.append(Box("saline_bag", pos=(HOOK_XY[0], HOOK_XY[1], bag_mid), size=BAG_SIZE, color=BAG, yaw=FACE))
    out.append(_upright("bag_port", HOOK_XY, TOP + BAG_BOTTOM - 0.020, TOP + BAG_BOTTOM, PORT_BLUE, 0.010))
    out.append(_upright("spike", HOOK_XY, TOP + CHAMBER_TOP, TOP + BAG_BOTTOM - 0.020, CLAMP_WHITE, 0.008))
    out.append(_upright("drip_chamber", HOOK_XY, TOP + CHAMBER_TOP - CHAMBER_LEN, TOP + CHAMBER_TOP, CHAMBER,
                        CHAMBER_D, FACE))
    # half full of saline; the drop growing at the drip tube's end, the falling drop and the stream (drawn in CHAMBER
    # until shown: _show_drip)
    out.append(_on_face("chamber_saline", 0.75 * CHAMBER_LEN, CHAMBER_D - 0.002, CHAMBER_LEN / 2, SALINE, 0.0011))
    out.append(_on_face("drop", DROP_AT, DROP_D, DROP_D, CHAMBER))
    out.append(_on_face("drop_fall", FALL_AT, DROP_D, FALL_H, CHAMBER))
    out.append(_on_face("stream", 0.021, 0.003, 0.018, CHAMBER))
    return out


# a matte black foam board standing on the table behind the chamber, as the top camera sees it, slotted upright into a
# wooden block taped to the table: against it the clear drops show
BOARD_BACK = 0.030  # m from the chamber's axis to the board's middle, away from the camera
BOARD = (0.005, 0.10, 0.36)  # thick, wide, tall
BLOCK = (0.04, 0.10, 0.02)
BOARD_XY = (HOOK_XY[0] - BOARD_BACK * math.cos(FACE), HOOK_XY[1] - BOARD_BACK * math.sin(FACE))


def _board() -> list:
    return [Box("backing_board", pos=(*BOARD_XY, TOP + BOARD[2] / 2), size=BOARD, color=BOARD_BLACK, yaw=FACE),
            Box("board_block", pos=(*BOARD_XY, TOP + BLOCK[2] / 2), size=BLOCK, color=TAPE, yaw=FACE)]


def _beaker() -> list:
    """Nalgene 1201-2000 low-form beaker on the floor: twelve wall panels round a 3 mm floor, a little saline in it."""
    bx, by = BEAKER_XY
    r = BEAKER_D / 2 - BEAKER_WALL / 2
    panel = 2 * r * math.tan(math.pi / 12) + 0.001
    out = [Box("beaker_floor", pos=(bx, by, FLOOR + 0.0015), size=(BEAKER_D * 0.96, BEAKER_D * 0.96, 0.003),
               color=BEAKER)]
    for k in range(12):
        a = 2 * math.pi * k / 12
        out.append(Box(f"beaker_wall_{k}", pos=(bx + r * math.cos(a), by + r * math.sin(a), FLOOR + BEAKER_H / 2),
                       size=(BEAKER_WALL, panel, BEAKER_H), color=BEAKER, yaw=a))
    out.append(Box("beaker_saline", pos=(bx, by, FLOOR + 0.004), size=(BEAKER_D * 0.9, BEAKER_D * 0.9, 0.002),
                   color=SALINE))
    return out


# the drops as the top camera sees them: a 0.1 mL drop (10 a mL) is 5.8 mm across
DROP_D = 0.006
DROP_AT = 0.013  # m below the chamber's top: the drop hanging from the drip tube's end
HANG_FROM = 0.3  # the hanging drop shows once it has grown to this share of its volume
FALL_AT, FALL_H = 0.023, 0.014  # the falling drop, smeared over a frame as a streak from the tube's end to the saline
FALL_STEPS = 1  # control step (0.05 s) the falling drop shows: a real drop falls the 2 cm in about 0.06 s
OBJECTS = [CLAMP, *_line(), *_pole(), *_board(), *_beaker()]
LINE_POINTS = [box_points(o) for o in OBJECTS if o.name.startswith(("line_", "slide_clamp", "drip_chamber", "spike",
                                                                    "saline_bag", "bag_port"))]
LINE_AXES = [axis_points(o) for o in OBJECTS if o.name.startswith("line_")]


def _show(name: str, on: bool) -> None:
    if LOG.shown.get(name) != on:
        LOG.shown[name] = on
        recolor(name, DROP if on else CHAMBER)


def _show_drip(rate: float) -> None:
    """A drop grows at the top of the chamber and falls as a streak shown for one step; a stream shows as a line."""
    stream = math.isinf(rate)
    if not stream:
        LOG.phase += rate / 60.0 * DT
        if int(LOG.phase) > LOG.drops:
            LOG.drops, LOG.fall = int(LOG.phase), FALL_STEPS
        elif LOG.fall:
            LOG.fall -= 1
    _show("stream", stream)
    _show("drop_fall", not stream and LOG.fall > 0)
    _show("drop", not stream and rate > 0 and LOG.phase % 1.0 >= HANG_FROM and not LOG.fall)


# -- the scripted demo (clamp_demo.py) -------------------------------------------------------------------------------
REQUIRED_FORCES = {"roll the wheel": 2.12}  # N: the wheel's 1.5 N breakaway, taken along its 45-degree face

from clamp_demo import plan, steps, total_s  # noqa: E402,F401


def demo():
    """Left arm pinches the housing; right fingertip rolls the wheel open, corrects on or back; both return."""
    return plan(SimpleNamespace(**globals()))
