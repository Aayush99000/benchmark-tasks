"""On the station table: a hospital wall oxygen flowmeter (0-15 L/min, a float ball in a clear flow tube, a green
flow-control knob on its front and a tubing nipple at its outlet), plugged into an oxygen wall outlet on a headwall
panel and turned off. The end of a nasal cannula's oxygen tubing lies on the cannula's packet beside it. Set the
patient's oxygen up: push the tubing's end fully onto the outlet nipple, then turn the knob until the middle of the
float ball is on the 2 L/min line, and let go of both.

Two models run here. The nipple's barbs: tubing pushed onto the nipple until its end is past the last barb is
seated (held where it is); short of that it isn't held. The flowmeter: the knob opens the needle valve as it is
turned anticlockwise, seen from the front; the flow follows the knob's angle and the float ball rides on the flow,
its middle at the flow's line on the tube's scale.

Run it:  inspect-robots run --embodiment yam_arena -E task=oxygen_flowmeter.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import Hinge, add_hinge, body, box, cyl, prism12, ring  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

ALUMINIUM = (0.74, 0.75, 0.77)
O2_GREEN = (0.05, 0.50, 0.25)
DARK = (0.08, 0.08, 0.09)
WHITE = (0.93, 0.93, 0.91)
CLEAR = (0.85, 0.92, 0.95, 0.25)  # see-through acrylic (alpha under 0.4)
TUBING = (0.70, 0.88, 0.78, 0.6)  # clear green-tinted PVC tubing
PANEL = (0.86, 0.83, 0.76)
PACKET = (0.90, 0.91, 0.93)
fittingkit.METALLIC[ALUMINIUM] = 0.35


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the flowmeter's scale ------------------------------------------------------------------------------------------
# A 0-15 L/min oxygen flowmeter with the low range spread out, like Ohio Medical's 6700 series (spec sheet 550038:
# "1/2 L/min increments from 1 to 5", "1 L/min increments from 5 to 15", "expanded 1-5 L/min range"). Read at the
# middle of the ball (Ohio Medical IFU 7700-0631-000: "align the center of the Float Ball to the indicator line").
# [estimate: the lines' spacing: 10 mm per L/min from 0 to 5, 2.6 mm per L/min from 5 to 15]
FLOW_MAX = 15.0  # L/min, the top line


def ball_rise(q: float) -> float:
    """How far (m) the ball's middle rides above its rest, at ``q`` L/min (its line on the scale)."""
    q = max(0.0, q)
    if q <= 5.0:
        return 0.010 * q
    return 0.050 + 0.0026 * (min(q, FLOW_MAX + 0.5) - 5.0)


# -- the flowmeter (fixed): front face towards the arms -------------------------------------------------------------
# 152 mm tall, 56 mm wide, 58 mm deep with its flow tube's shroud (Ohio Medical 6700, spec sheet 550038).
FX = 0.31  # the body's front face (x); the shroud stands 12 mm in front of it
FM_W, FM_D = 0.056, 0.046  # the body behind the shroud
FM_BOT, FM_TOP = 0.070, 0.222  # above the table
SHROUD_D = 0.012
SHROUD_W = 0.026
# WHY its top at 18.6 cm: the left hand's lower finger, rolled to turn the knob above it, met its top cap as it
# opened (plan_check, and run oxy8); 8 mm under the knob now
SHROUD_BOT, SHROUD_TOP = 0.096, 0.186
BALL_R = 4.0 * MM
BALL_X = -0.006  # the ball's middle, from the front face
REST_Z = SHROUD_BOT + 0.002 + BALL_R + 0.002  # the ball's middle resting on its stop (0 L/min)
# WHY 21.2 cm: at 20.5 the left hand's lower finger, rolled 34 degrees, met the shroud's top cap (plan_check); at
# 20.9 the knob's corners turned 0.5 mm over the cap and dragged on it (run oxy1: it lagged the hand by 12 degrees,
# then sprang back), so it clears the cap by 4 mm
KNOB_Z = 0.212  # the knob's axis, above the table
KNOB_R, KNOB_L = 13.5 * MM, 18.0 * MM  # a fluted knob 27 mm across
KNOB_GAP = 2.0 * MM  # between its back and the front face
KNOB_X = -(KNOB_GAP + KNOB_L / 2)  # its middle, from the front face
OUT_Z = 0.084  # the outlet's axis, above the table
NUT_R, NUT_L = 9.5 * MM, 15.0 * MM  # the DISS outlet nut
NIP_R, NIP_L = 2.0 * MM, 20.0 * MM  # the tubing nipple (its core; the barbs stand 1.2 mm proud of it)
TIP_TAPER = 6.0 * MM  # its last 6 mm taper to its tip
NIP_TIP = np.array([FX - NUT_L - NIP_L, 0.0, TOP + OUT_Z])  # the nipple's tip, world
NIP_AXIS = X_AXIS  # into the flowmeter


def _scale_marks() -> list:
    """The lines and numbers on the shroud's front: long lines at 1-5, 10 and 15 L/min, short ones at the halves
    from 1 to 5, middle-length ones at the other whole numbers to 15; seven-segment numbers 1-5, 10, 15 beside the
    long lines, to the right as seen from the front (-y)."""
    x = -SHROUD_D - 0.0004
    marks = []
    q = 1.0
    while q <= FLOW_MAX + 1e-9:
        long_ = q in (1, 2, 3, 4, 5, 10, 15)
        half = q < 5 and q != int(q)
        w = 0.010 if long_ else (0.005 if half else 0.007)
        marks.append(box((x, 0.0, REST_Z + ball_rise(q)), (0.0008, w, 0.0007), DARK))
        q += 0.5 if q < 5 else 1.0
    for value in (1, 2, 3, 4, 5, 10, 15):
        marks += _number(value, x, -0.008, REST_Z + ball_rise(value))
    return marks


_SEGMENTS = {  # seven segments: a top, b top right, c bottom right, d bottom, e bottom left, f top left, g middle
    0: "abcdef", 1: "bc", 2: "abdeg", 3: "abcdg", 4: "bcfg", 5: "acdfg"}


def _number(value: int, x: float, y_left: float, z_mid: float) -> list:
    """``value`` in 2.4 x 4 mm seven-segment digits, its left edge at ``y_left`` (reading from the front, y falls to
    the right)."""
    out, w, h, t = [], 0.0024, 0.004, 0.0004
    for k, digit in enumerate(str(value)):
        y0 = y_left - k * (w + 0.0012)  # the digit's left edge
        yc = y0 - w / 2
        seg = {"a": (yc, z_mid + h / 2, w, t), "d": (yc, z_mid - h / 2, w, t), "g": (yc, z_mid, w, t),
               "b": (y0 - w, z_mid + h / 4, t, h / 2), "c": (y0 - w, z_mid - h / 4, t, h / 2),
               "e": (y0, z_mid - h / 4, t, h / 2), "f": (y0, z_mid + h / 4, t, h / 2)}
        for s in _SEGMENTS[int(digit)]:
            yy, zz, sw, sh = seg[s]
            out.append(box((x, yy, zz), (0.0008, sw, sh), DARK))
    return out


def _flowmeter_shapes() -> tuple[list, list]:
    """In the flowmeter's frame (origin on the table under the middle of its front face; the world's axes)."""
    zc = (SHROUD_BOT + SHROUD_TOP) / 2
    sh = SHROUD_TOP - SHROUD_BOT
    solid = [
        box((FM_D / 2, 0.0, (FM_BOT + FM_TOP) / 2), (FM_D, FM_W, FM_TOP - FM_BOT), ALUMINIUM),  # the body
        # the shroud, hollow so the ball inside it touches nothing: its front and two sides
        box((-SHROUD_D + 0.0005, 0.0, zc), (0.001, SHROUD_W, sh), CLEAR),
        box((-SHROUD_D / 2, SHROUD_W / 2 - 0.0005, zc), (SHROUD_D, 0.001, sh), CLEAR),
        box((-SHROUD_D / 2, -SHROUD_W / 2 + 0.0005, zc), (SHROUD_D, 0.001, sh), CLEAR),
        box((-SHROUD_D / 2, 0.0, SHROUD_TOP + 0.002), (SHROUD_D, SHROUD_W, 0.004), ALUMINIUM),  # its top cap
        box((-SHROUD_D / 2, 0.0, SHROUD_BOT - 0.002), (SHROUD_D, SHROUD_W, 0.004), ALUMINIUM),  # its base
        cyl((-NUT_L / 2, 0.0, OUT_Z), NUT_R, NUT_L, ALUMINIUM, axis="X"),  # the DISS outlet nut
        # the nipple: its 4 mm core, tapering over its last 6 mm in three steps to 2.2 mm at the tip (a lead-in)
        cyl((-NUT_L - (NIP_L - TIP_TAPER) / 2, 0.0, OUT_Z), NIP_R, NIP_L - TIP_TAPER, O2_GREEN, axis="X"),
        *[cyl((-NUT_L - NIP_L + TIP_TAPER - (k + 0.5) * TIP_TAPER / 3, 0.0, OUT_Z), r, TIP_TAPER / 3, O2_GREEN,
              axis="X") for k, r in enumerate((1.7 * MM, 1.4 * MM, 1.1 * MM))],
        cyl((FM_D + 0.010, 0.0, 0.150), 10.0 * MM, 0.020, ALUMINIUM, axis="X"),  # the inlet probe, into the outlet
        box((FM_D + 0.025, 0.0, 0.150), (0.010, 0.070, 0.070), O2_GREEN),  # the oxygen wall outlet
        box((FM_D + 0.036, 0.0, 0.150), (0.012, 0.240, 0.300), PANEL),  # the headwall panel
        box((FM_D + 0.036, 0.0, 0.0025), (0.080, 0.240, 0.005), PANEL),  # its foot on the table
    ]
    seen = [
        cyl((BALL_X, 0.0, zc), 6.0 * MM, sh, CLEAR),  # the flow tube in the shroud
        box((-SHROUD_D / 2, 0.0, OUT_Z + NUT_R + 0.004), (SHROUD_D, 0.040, 0.008), O2_GREEN),  # the oxygen band
        cyl((-KNOB_GAP / 2, 0.0, KNOB_Z), 4.0 * MM, KNOB_GAP, ALUMINIUM, axis="X"),  # the knob's stem
        box((FM_D + 0.019, 0.0, 0.150 + 0.026), (0.0005, 0.050, 0.006), WHITE),  # the outlet's label
    ]
    # the nipple's barbs: three rings 1.2 mm proud, its hex base
    seen += [cyl((-NUT_L - d, 0.0, OUT_Z), NIP_R + 1.2 * MM, 0.003, O2_GREEN, axis="X") for d in (0.006, 0.011, 0.016)]
    seen.append(cyl((-NUT_L - 0.002, 0.0, OUT_Z), 6.0 * MM, 0.004, O2_GREEN, axis="X"))
    seen += _scale_marks()
    return solid, seen


_FS, _FV = _flowmeter_shapes()
# WHY friction 0.3 (and 0.4 on the tubing): plastic on PVC; at the 1.0 to 1.2 the grippers need, the bore, pressed
# onto the nipple's core by the hand's tilt under the barbs' push, jammed 9 mm on (run oxy4)
FLOWMETER = body("flowmeter", pos=(FX, 0.0, TOP), friction=0.3, contact_offset=0.0005, shapes=_FS + _FV,
                 visual_only=tuple(range(len(_FS), len(_FS) + len(_FV))))

# -- the knob: a body on a hinge in the flowmeter -------------------------------------------------------------------
# Its frame: z out of the flowmeter's face (towards the arms), x up at the start. Turned anticlockwise, seen from the
# front (about -x), it opens the valve (Ohio Medical IFU: "To INCREASE Flow: Turn the know counter clockwise"); fully
# clockwise it is off, against its stop.
R_KNOB = np.column_stack([Z_AXIS, Y_AXIS, -X_AXIS])
KNOB_AT = np.array([FX + KNOB_X, 0.0, TOP + KNOB_Z])


def _knob_shapes() -> tuple[list, list]:
    # (a round collider pushed the closing fingers 12 mm back off the knob: run oxy8)
    solid = prism12((0.0, 0.0, 0.0), KNOB_R, KNOB_L, O2_GREEN)
    seen = [box((KNOB_R * math.cos(a), KNOB_R * math.sin(a), 0.0), (0.0015, 0.0015, KNOB_L), O2_GREEN, yaw=a)
            for a in (2 * math.pi * k / 18 for k in range(18))]  # the flutes
    seen.append(box((0.0075, 0.0, KNOB_L / 2 + 0.0002), (0.007, 0.0016, 0.0004), WHITE))  # its pointer line
    return solid, seen


_KS, _KV = _knob_shapes()
# WHY 50 g and 32 solver iterations: at 20 g, pinched between the fingers, its hinge jolted (runs oxy7 to oxy11)
KNOB = body("knob", pos=tuple(KNOB_AT), mass=0.050, friction=1.2, shapes=_KS + _KV,
            visual_only=tuple(range(len(_KS), len(_KS) + len(_KV))), quat=quat_xyzw(R_KNOB), solver_iterations=32)
# WHY 300 degrees: 15 L/min at 270 (the top line), flush above it to its stop [estimate: the needle valve's travel]
KNOB_TRAVEL = math.radians(300.0)
THETA_FULL = math.radians(270.0)  # 15 L/min
# WHY 0.03: at 0.3 it wouldn't give as the fingers closed on its corners, and threw the hand 23 mm off it (run oxy7)
# [estimate: a needle valve's packing]
KNOB_DAMP = 0.03  # N m s/rad
add_hinge(Hinge(parent="flowmeter", child="knob", anchor=(KNOB_X, 0.0, KNOB_Z), axis=(1.0, 0.0, 0.0),
                damping=KNOB_DAMP, lower=-KNOB_TRAVEL, upper=0.0))


def flow_lpm(theta: float) -> float:
    """The flow (L/min) with the knob ``theta`` (rad) open from off: proportional up to 15 L/min at 270 degrees,
    then flush (65 L/min, Ohio Medical: "Flush setting delivers a minimum of 65 L/min")."""
    if theta <= 0.0:
        return 0.0
    if theta <= THETA_FULL:
        return FLOW_MAX * theta / THETA_FULL
    return 65.0


def theta_for(q: float) -> float:
    return THETA_FULL * q / FLOW_MAX


# -- the float ball: moved by the flowmeter model, inside the hollow shroud -----------------------------------------
BALL_AT = np.array([FX + BALL_X, 0.0, TOP + REST_Z])


def _ball_shapes() -> tuple[list, list]:
    solid = [box((0.0, 0.0, 0.0), (0.002, 0.002, 0.002), DARK)]  # a token collider; it touches nothing
    seen = [cyl((0.0, 0.0, 0.0), BALL_R, 2 * BALL_R, DARK, axis=a) for a in ("X", "Y", "Z")]
    return solid, seen


_BS, _BV = _ball_shapes()
BALL = body("ball", pos=tuple(BALL_AT), mass=0.005, friction=0.5, shapes=_BS + _BV,
            visual_only=tuple(range(len(_BS), len(_BS) + len(_BV))))
fittingkit.no_gravity("ball")

# -- the tubing's end, on the cannula's packet ----------------------------------------------------------------------
# Oxygen supply tubing (about 7 mm across, a 4.8 mm bore): its last 15 cm, rigid. Its frame: z along it, the open end
# (the mouth) at +z. It lies along x on the packet, its mouth towards the flowmeter, its last 55 mm over the
# packet's edge.
TUBE_L = 0.150
TUBE_R = 4.0 * MM  # [estimate: 8 mm across, a little thicker than real tubing, for a firm pinch]
# WHY 1.4 mm round the nipple's core: with 1 mm, the tubing pitched 3.7 degrees in the pinch as it met the taper and
# the bore bound on the core 7 mm on (run oxy2)
BORE_R = 3.4 * MM  # the bore over its first 24 mm
BORE_D = 0.024
MOUTH_CHAMFER = 3.0 * MM
TUBE_MASS = 0.040  # WHY 40 g: real tubing is a few grams; a part much under 30 g squeezes out of the YAM's grip
TUBE_COM = 0.070  # its centre of mass this far back from the mouth: the rest of the tubing pulls back on it
R_TUBE_START = np.column_stack([Z_AXIS, -Y_AXIS, X_AXIS])
PACKET_AT = np.array([0.095, -0.10, TOP])
PACKET_SIZE = (0.100, 0.070, 0.025)
OVERHANG = 0.055  # the mouth this far past the packet's front edge
MOUTH_START = np.array([PACKET_AT[0] + PACKET_SIZE[0] / 2 + OVERHANG, PACKET_AT[1],
                        TOP + PACKET_SIZE[2] + TUBE_R + 0.0002])
TUBE_AT = MOUTH_START - (TUBE_L / 2) * X_AXIS


def _tube_shapes() -> tuple[list, list]:
    z_mouth = TUBE_L / 2
    # the bore, its last 3 mm chamfered out to 4.0 mm (the soft end gives as it meets the nipple)
    solid = ring(z_mouth - BORE_D, z_mouth - MOUTH_CHAMFER, BORE_R, TUBE_R, TUBING, segments=12)
    solid += ring(z_mouth - MOUTH_CHAMFER, z_mouth, BORE_R + 0.6 * MM, TUBE_R + 0.6 * MM, TUBING, segments=12)
    rest = TUBE_L - BORE_D
    solid += prism12((0.0, 0.0, -TUBE_L / 2 + rest / 2), TUBE_R - 0.0002, rest, TUBING)
    seen = [cyl((0.0, 0.0, 0.0), TUBE_R + 0.0002, TUBE_L, TUBING)]
    return solid, seen


_TS, _TV = _tube_shapes()
TUBE = body("tubing", pos=tuple(TUBE_AT), mass=TUBE_MASS, friction=0.4, contact_offset=0.0005, shapes=_TS + _TV,
            visual_only=tuple(range(len(_TS), len(_TS) + len(_TV))), quat=quat_xyzw(R_TUBE_START),
            com=(0.0, 0.0, TUBE_L / 2 - TUBE_COM))
PACKET_BODY = body("packet", pos=tuple(PACKET_AT), friction=0.8, shapes=[
    box((0.0, 0.0, PACKET_SIZE[2] / 2), PACKET_SIZE, PACKET),
    box((0.0, 0.0, PACKET_SIZE[2] + 0.0002), (0.040, 0.030, 0.0004), O2_GREEN),  # its label
], visual_only=(1,))

OBJECTS = [FLOWMETER, KNOB, BALL, TUBE, PACKET_BODY]

# -- the two models ---------------------------------------------------------------------------------------------------
ON_RADIAL = 2.0 * MM  # the mouth within this of the nipple's axis,
ON_DEG = 10.0  # the tubing within this of its line: it is on the nipple
# WHY no barb force: a force resisting the push (6, 3, then 1.5 N) pivoted the tubing in the YAM's two-finger pinch
# every time it came on, dropping its mouth 1.5 mm onto the nipple's core, where it jammed (runs oxy1 to oxy6); the
# simulated pinch barely resists turning about the line between the fingertips. So the barbs are only the seat
SEAT = 12.0 * MM  # past the last barb: seated
FULL_ON = NIP_L  # the mouth against the nut
SET_LPM, SET_TOL = 2.0, 0.25  # the prescribed flow and how close counts as on its line (half a line's spacing)
OVER_LPM = 4.0  # over this at the end: far over the prescription
CLEAR_M = 0.02  # both grippers this far from the tubing and the knob at the end
TRACE_EVERY = 24  # substeps (0.1 s)


def tube_mouth(pos, rot) -> np.ndarray:
    return np.asarray(pos, float) + np.asarray(rot, float) @ np.array([0.0, 0.0, TUBE_L / 2])


def on_nipple(pos, rot, origin=np.zeros(3)) -> tuple[float, float, float]:
    """(how far (m) the mouth is past the nipple's tip along its axis, its distance (m) from the axis, the tubing's
    angle (deg) from the axis)."""
    rel = tube_mouth(pos, rot) - origin - NIP_TIP
    depth = float(rel @ NIP_AXIS)
    radial = float(np.linalg.norm(rel - depth * NIP_AXIS))
    along = float(np.clip(np.asarray(rot, float)[:, 2] @ NIP_AXIS, -1.0, 1.0))
    return depth, radial, math.degrees(math.acos(along))


def knob_angle(rot) -> float:
    """How far (rad) the knob is turned anticlockwise, seen from the front, from its start (the pointer up); in
    (-pi, pi]."""
    up = np.asarray(rot, float)[:, 0]
    return math.atan2(float(up[1]), float(up[2]))


def seated_pose(depth: float, rot) -> tuple[np.ndarray, tuple]:
    """The tubing's pose seated ``depth`` onto the nipple, on its axis, keeping its roll."""
    r = np.asarray(rot, float)
    x_dir = r[:, 0] - (r[:, 0] @ NIP_AXIS) * NIP_AXIS
    x_dir /= np.linalg.norm(x_dir)
    r_on = np.column_stack([x_dir, np.cross(NIP_AXIS, x_dir), NIP_AXIS])
    mouth = NIP_TIP + depth * NIP_AXIS
    return mouth - (TUBE_L / 2) * NIP_AXIS, quat_xyzw(r_on)


class Oxygen:
    """Runs before every physics substep (240 Hz).

    The nipple: once the tubing's mouth is on the nipple (within 2 mm of its axis, the tubing within 10 degrees of
    its line, past the tip) and its mouth is 12 mm on,
    past the last barb, it is seated and held where it is (its pose is written back every substep).

    The flowmeter: the knob's angle (unwrapped) sets the flow; the ball's pose is written every substep at the
    flow's line."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.dt = 1.0 / 240.0
        self.log = os.environ.get("OXY_LOG")
        self.trace = os.environ.get("OXY_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["tubing", "knob", "ball"]

    def clear(self) -> None:
        self._n = 0
        self.seated = False
        self.seated_depth = 0.0
        self.depth = -1.0
        self.max_depth = -1.0
        self.theta = 0.0
        self._raw = None
        self.max_flow = 0.0
        self.tube = None
        self.knob = None
        self.poses = {}

    @property
    def flow(self) -> float:
        return flow_lpm(self.theta)

    def update(self, frames: dict, acc: dict) -> None:
        self._n += 1
        k = frames.get("knob")
        if k is not None:
            self.knob = k
            raw = knob_angle(k.rot)
            if self._raw is None:
                self.theta = raw
            else:
                self.theta += (raw - self._raw + math.pi) % (2 * math.pi) - math.pi
            self._raw = raw
            self.max_flow = max(self.max_flow, self.flow)
        poses = {"ball": (BALL_AT + self.origin + np.array([0.0, 0.0, ball_rise(self.flow)]), (0.0, 0.0, 0.0, 1.0))}
        t = frames.get("tubing")
        if t is not None:
            self.tube = t
            if self.seated:
                poses["tubing"] = seated_pose(self.seated_depth, t.rot)
                poses["tubing"] = (poses["tubing"][0] + self.origin, poses["tubing"][1])
            else:
                depth, radial, angle = on_nipple(t.pos, t.rot, self.origin)
                on = radial < ON_RADIAL and angle < ON_DEG and depth > 0.0
                self.depth = depth if on else -1.0
                if on:
                    self.max_depth = max(self.max_depth, depth)
                if on and depth >= SEAT:
                    self.seated = True
                    self.seated_depth = min(depth, FULL_ON)
                    self._note(f"{self._n / 240:7.2f}s seated: {1000 * depth:5.1f} mm on, {1000 * radial:4.2f} mm off "
                               f"its axis, {angle:4.1f} deg")
        self.poses = poses
        if self._n % TRACE_EVERY == 0 and self.trace:
            self._write_trace()

    def _write_trace(self) -> None:
        """Debug only (OXY_TRACE=<file>): every 0.1 s, the tubing's mouth, the knob, the flow and both TCPs."""
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        mouth = ""
        if self.tube is not None:
            m = 1000 * (tube_mouth(self.tube.pos, self.tube.rot) - self.origin)
            d, r, a = on_nipple(self.tube.pos, self.tube.rot, self.origin)
            mouth = (f"mouth ({m[0]:6.1f},{m[1]:6.1f},{m[2] - 1000 * TOP:6.1f}) on {1000 * d:6.1f} off "
                     f"{1000 * r:5.2f} ang {a:5.1f} s{int(self.seated)}")
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s {mouth} knob {math.degrees(self.theta):6.1f} flow {self.flow:5.2f} |"
                     f"{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


MODEL = fittingkit.add_model(Oxygen())

# -- scoring ----------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def flow_set() -> bool:
    return abs(MODEL.flow - SET_LPM) <= SET_TOL


def tubing_on_nipple() -> bool:
    """The tubing's mouth on the nipple but short of its seat (pushed on part way)."""
    return not MODEL.seated and MODEL.depth > 0.0


def _gap(points: np.ndarray, frame, half: np.ndarray) -> float:
    """The least distance (m) from ``points`` to a box of half sizes ``half`` in ``frame`` (centred on its origin)."""
    rel = (points - frame.pos) @ frame.rot
    out = np.maximum(np.abs(rel) - half, 0.0)
    return float(np.linalg.norm(out, axis=1).min())


def hands_clear() -> bool:
    """No gripper point within 2 cm of the tubing or the knob; unknown offline counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0 or MODEL.tube is None or MODEL.knob is None:
        return True
    tube_half = np.array([TUBE_R, TUBE_R, TUBE_L / 2])
    knob_half = np.array([KNOB_R, KNOB_R, KNOB_L / 2])
    return _gap(pts, MODEL.tube, tube_half) > CLEAR_M and _gap(pts, MODEL.knob, knob_half) > CLEAR_M


def success(objects) -> bool:
    """The tubing seated on the nipple, the flow on the 2 L/min line (within 0.25 L/min), and both grippers clear of
    the tubing and the knob. Must hold 1 s."""
    return MODEL.seated and flow_set() and hands_clear()


def score(objects) -> float:
    """At the end: 0 if the flow is over 4 L/min; 0.9 for the tubing seated and the flow on the 2 L/min line (1.0
    once success() has held 1 s, the hands clear); 0.5 for the tubing seated with the flow on but off its line; 0.25
    for the tubing seated with the flow off, or the flow on its line with the tubing not seated; 0 otherwise."""
    q = MODEL.flow
    if q > OVER_LPM:
        return 0.0
    if MODEL.seated and flow_set():
        return COMPLETE_SCORE
    if MODEL.seated and q >= 0.1:
        return 0.5
    if MODEL.seated or flow_set():
        return 0.25
    return 0.0


# -- the scripted demo ------------------------------------------------------------------------------------------------
# The right hand, pointing down, its fingers closing across the tubing, pinches it 38 mm behind its mouth where it
# hangs over the packet's edge, lifts it, carries it level to the nipple's line and pushes it on to the nut; it lets
# go and moves away. Then the left hand, level and pointing at the knob, rolled 90 degrees clockwise so the turn
# stays within its wrist's reach, closes on the knob, turns it 36 degrees anticlockwise (2 L/min), lets go and moves
# clear.
class RotWaypoint(Waypoint):
    def __init__(self, duration, pos, finger, label="", rot=None):
        super().__init__(duration, pos, finger, label, 0.0)
        self.rot = None if rot is None else np.asarray(rot, float)


def tcp_rotation(w) -> np.ndarray:
    rot = getattr(w, "rot", None)
    return grip_rotation(w.yaw) if rot is None else rot


def _target(self, t: float):
    """yam_arena.demo.ArmPlan.target, honouring a RotWaypoint's orientation."""
    from yam_arena import ik

    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal, rot = np.asarray(w.pos, dtype=float), tcp_rotation(w)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_demo.ArmPlan.target = _target

OPEN, CLOSED = -0.04695, 0.0
PAD_TIP = 13.0 * MM  # the closed fingertips this far past the TCP

# The right hand: straight down, its fingers closing along y (across the tubing).
R_PINCH = grip_rotation(0.0)
# WHY 38 mm: nearer the mouth, the fingers' far edges reach the flow tube's shroud at the end of the push (plan_check:
# 0.5 mm into it at 30 to 34 mm)
GRIP_BACK = 0.038  # the pinch this far behind the mouth (the packet's edge 55 mm behind it)
TIP_BELOW = 4.0 * MM  # the fingertips this far below the tubing's middle
PICK_TCP = MOUTH_START - GRIP_BACK * X_AXIS + np.array([0.0, 0.0, PAD_TIP - TIP_BELOW])
_R_TT = R_PINCH.T @ R_TUBE_START  # the tubing in the right TCP's frame, as the fingers close on it
_P_TT = R_PINCH.T @ (TUBE_AT - PICK_TCP)
PUSH_PAST = 4.0 * MM  # the mouth aimed this far past the nut, so the arm presses it home on the nut
# WHY 2 mm up: held, the tubing's mouth rode 1.9 mm below the plan (run oxy1: the hand 1 mm low, the tubing 0.7 mm
# lower in the grip), and its bore jammed on the nipple's core 5 mm on
# (run oxy2, aimed 2 mm up: the mouth 0.7 mm over the axis before it touched the nipple)
# (runs oxy3 and oxy4, aimed 1.4 and 1.0 mm up: the mouth 0.4 to 0.7 mm over the axis, and 0.4 mm under it, before it
# touched the nipple)
AIM_UP = 1.4 * MM


def tube_held(tcp, r=R_PINCH) -> tuple[np.ndarray, np.ndarray]:
    """The tubing's origin and rotation held by the right TCP at ``tcp``, ``r``."""
    return np.asarray(tcp, float) + r @ _P_TT, r @ _R_TT


def tcp_for_mouth(mouth, r=R_PINCH) -> np.ndarray:
    """The right TCP's position holding the tubing with its mouth at ``mouth``."""
    pos = np.asarray(mouth, float) - (r @ _R_TT) @ np.array([0.0, 0.0, TUBE_L / 2])
    return pos - r @ _P_TT


# The left hand: level, pointing along x at the knob, its fingers closing across it.
def r_level(roll_deg: float) -> np.ndarray:
    """Level, pointing along x, the fingers closing up and down at roll 0, rolled ``roll_deg`` about x (positive:
    clockwise seen from behind the hand, i.e. from the arms)."""
    r = np.column_stack([Z_AXIS, np.cross(X_AXIS, Z_AXIS), X_AXIS])
    return axis_angle_matrix(X_AXIS, math.radians(roll_deg)) @ r


# WHY 90: rolled 70 and turned to 34, the lower finger, opening, met the flow tube's top cap (plan_check); at 100 the
# pads were 10 degrees off the knob's flats (they face every 30 degrees from its pointer) and closing turned the knob
# 7.7 degrees back to square them (run oxy10); at 90 they close square on two flats
ROLL_START = 90.0  # degrees: the left hand closes on the knob rolled this far
TURN_DEG = math.degrees(theta_for(SET_LPM))  # 36 degrees anticlockwise from the front: 2 L/min
# WHY 3 degrees less: turned 36 degrees by the plan, the knob ended at 39.3 (run oxy2: it went on past the hand's turn
# and settled when the fingers opened)
# WHY 1.5 degrees more: with the grip settled, turned 35 degrees by the plan the knob went 32.7 with the hand and 0.7
# more as the fingers opened ajar (run oxy10, fingers closed hard); with the snug grip it follows the hand: 37.4
# degrees for the plan's 37.5, 2.08 L/min (run oxy12), inside the 0.25 L/min tolerance either way
TURN_LAG = 1.5  # degrees more than the plan's turn, for what the sim shows
# WHY 5 mm: at 8 the pads held 10 mm of the knob and the grip sometimes failed as the turn began (runs oxy7, oxy9)
KNOB_TIPS = FX - 5.0 * MM  # the fingertips 5 mm short of the face, the pads round the knob
KNOB_TCP = np.array([KNOB_TIPS - PAD_TIP, 0.0, TOP + KNOB_Z])


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    up = lambda h: np.array([0.0, 0.0, h])  # noqa: E731
    l_wait = (np.array([0.20, 0.16, TOP + 0.20]), r_level(ROLL_START))
    r_wait = (np.array([0.18, -0.20, TOP + 0.16]), R_PINCH)
    # the right hand connects the tubing
    both(2.5, (PICK_TCP + up(0.04), R_PINCH), OPEN, l_wait, CLOSED, "over the tubing")
    both(1.2, (PICK_TCP, R_PINCH), OPEN, l_wait, CLOSED, "fingers either side of it")
    both(2.0, (PICK_TCP, R_PINCH), CLOSED, l_wait, CLOSED, "pinch it")
    both(1.2, (PICK_TCP + up(0.03), R_PINCH), CLOSED, l_wait, CLOSED, "lift it")
    line_up = NIP_TIP - 0.030 * NIP_AXIS + up(AIM_UP)
    both(2.5, (tcp_for_mouth(line_up + up(0.02)), R_PINCH), CLOSED, l_wait, CLOSED, "to the nipple")
    both(1.2, (tcp_for_mouth(line_up), R_PINCH), CLOSED, l_wait, CLOSED, "in line with it")
    both(1.2, (tcp_for_mouth(NIP_TIP + 0.003 * NIP_AXIS + up(AIM_UP)), R_PINCH), CLOSED, l_wait, CLOSED,
         "onto its tip")
    pushed = (tcp_for_mouth(NIP_TIP + (FULL_ON + PUSH_PAST) * NIP_AXIS + up(AIM_UP)), R_PINCH)
    both(2.0, pushed, CLOSED, l_wait, CLOSED, "push it on")
    both(0.6, pushed, CLOSED, l_wait, CLOSED, "hold it on")
    both(2.0, pushed, OPEN, l_wait, CLOSED, "let it go")
    # WHY back first: lifted straight up, its open fingers swept through the knob (plan_check)
    both(1.2, (pushed[0] - 0.04 * X_AXIS + up(0.01), R_PINCH), OPEN, l_wait, CLOSED, "back off the nipple")
    # the left hand sets the flow
    near = KNOB_TCP - 0.04 * X_AXIS
    both(2.0, r_wait, OPEN, (near, r_level(ROLL_START)), OPEN, "in front of the knob")
    both(1.2, r_wait, OPEN, (KNOB_TCP, r_level(ROLL_START)), OPEN, "fingers round the knob")
    # WHY snug, not closed: closed, each finger squeezes 50 N; the jolt turned the knob as they closed, and the arm,
    # pushed 6 mm aside, sprang back as they opened and dragged the knob round (runs oxy9 to oxy11)
    snug = -(KNOB_R - 2.5 * MM)  # each finger 2.5 mm into the knob's flats: about 12 N
    both(2.0, r_wait, OPEN, (KNOB_TCP, r_level(ROLL_START)), snug, "close on it")
    turned = (KNOB_TCP, r_level(ROLL_START - TURN_DEG - TURN_LAG))
    both(0.8, r_wait, OPEN, (KNOB_TCP, r_level(ROLL_START)), snug, "settle the grip")
    both(4.0, r_wait, OPEN, turned, snug, "turn it to 2 L/min")
    both(1.0, r_wait, OPEN, turned, snug, "hold it there")
    # (opened in place with the knob at 20.5 cm the lower finger met the shroud's top; the knob now clears it)
    # WHY only ajar, then back off: opened wide, the fingers dragged on the knob and turned it 1 to 8 degrees (runs
    # oxy2 to oxy6), or caught the shroud and spun it (run oxy8)
    ajar = -0.020  # each finger 20 mm from the middle: 6 mm clear of the knob
    both(2.5, r_wait, OPEN, turned, ajar, "let go of the knob")
    both(1.2, r_wait, OPEN, (KNOB_TCP - 0.05 * X_AXIS, turned[1]), ajar, "back off")
    both(1.5, r_wait, OPEN, (l_wait[0], turned[1]), OPEN, "hands clear")
    both(1.5, r_wait, OPEN, (l_wait[0], turned[1]), OPEN, "wait")
    return {"left": left, "right": right}
