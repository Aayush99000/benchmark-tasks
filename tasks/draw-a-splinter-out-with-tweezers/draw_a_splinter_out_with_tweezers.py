"""On the station table, in front of the right arm: a life-size silicone hand lies palm up on a pine block, its fingers
pointing at the robot and its index finger straight and level. A 15 mm wooden splinter (a piece of a round toothpick)
is stuck in the index fingertip's pad: it went in away from the robot at 25 degrees to the skin, 11 mm is under the
skin and 4 mm stands out, its outer end toward the robot. A thin ink line on the pad marks where it went in. To the
right, a small translucent medicine cup holds a pair of slant-tip stainless tweezers, tips down, leaning back toward
the robot about 40 degrees against the cup's near rim, their legs to the left and right.

- The tweezers' tips open about 6 mm and spring back open; squeezing the legs a third of the way down closes them.
  Their slanted tips lie flat on the skin when the tweezers point along the splinter.
- The splinter slides back out along the line it went in. Levered sideways or up, it tears the skin at the entry.

Run it:  inspect-robots run --embodiment yam_arena -E task=draw_a_splinter_out_with_tweezers.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import tiltgrip  # noqa: E402,F401 - lets a Waypoint's yaw carry a tilted grip
from benchkit import Hinge, Slider, ToolState, finger_frames, on_reset, on_step, tool  # noqa: E402
from splinter_geometry import *  # noqa: E402,F403 - the props' numbers and frames
from splinter_geometry import (  # noqa: E402
    BURIED, ENTRY, FINGERS, IN, LEG_T, OUT, PROUD, PROXY, SPLINTER_LONG, TOP, TRACK, TRACK_Q, jaw_corners, q, rot_x,
    rot_y,
)
from tiltgrip import Grip  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
REQUIRED_FORCES: dict = {}  # nothing is pressed into place; the pull's force budget is in the demo notes below

INSTRUCTION = (
    "Take the tweezers from the cup and pull the splinter out of the fingertip, gripping it by its end and "
    "drawing it out at the same angle it went in. Use only the tweezers on the splinter and don't pinch the skin."
)

# == scoring ==========================================================================================================
# Real bench: side video level with the fingertip and the top video; after the trial, calipers and a steel rule.
# Drawn out: how far the splinter has come out along its track, from the length still standing out at the ink line
# (4 mm at reset). Out: no part under the skin (11 mm drawn). Torn: the entry slit, measured with the track empty
# before and after the trial, grew by more than SLIT_GROWTH (sim: the load across the stub at the entry over F_TEAR
# at any time while some of it is still in; see torn_on_bench and the fidelity note).
DRAWN_QUARTER = 0.003  # m drawn out: 7 mm or more standing out
DRAWN_HALF = 0.006  # more than half its buried 11 mm
DRAWN_OUT = BURIED  # 11 mm: fully out
PUSHED_IN = 0.002  # pushed in this far (under 2 mm left standing out): zero
SLIT_GROWTH = 0.001  # real bench: the slit may grow at most 1 mm over the trial, against its own start
SLIT_RESET_MAX = 0.002  # real bench: a slit over 2 mm at reset means a fresh track
SNAPPED_CAP = 0.5  # real bench: a snapped splinter scores its remaining piece's draw, at most this
# Sim-only model values. The real-bench score never uses them: it reads the slit, the length drawn and the dent
# directly. F_TEAR with PLAY_RATE puts the tear at a pull about 6 to 13 degrees off the splinter's line (lever 2-4 mm).
F_TEAR = 1.5  # N across the stub at the entry: the sim's tear
PLAY_RATE = 0.027  # N m/rad: the silicone's give around the splinter, in the sim's two play joints
LEVER_MIN = 0.002  # m: the shortest lever counted, from the entry to where the stub is held
FINGER_MOVED = 0.005  # the fingertip may move at most 5 mm in the trial for full credit
HAND_OFF = 0.020  # moved 2 cm, or tipped 10 degrees: the hand is off its block
HAND_TIP_DEG = 10.0
# A gripper finger touching the splinter: each finger is a box (its last 40 mm, 15.5 mm wide, 4 mm thick, from its pad
# face out); it touches when it comes within TOUCH of the splinter's collider (the 2.4 mm stick) anywhere along the
# part that is out of the skin. Real bench: a frame of the side video with no gap between finger and splinter.
FINGER_LONG, FINGER_WIDE, FINGER_THICK = 0.040, 0.0155, 0.004
TOUCH = 0.0005
PINCH_DENT = 0.001  # the tips dent the fingertip's pad more than 1 mm while squeezed: they catch the skin
ON_PAD = 0.0005  # a tip this close to the pad's surface is on it
SQUEEZED = TIP_GAP - 0.001  # the tips are being squeezed once they are 1 mm closer than at rest
CLEAR_OF_SKIN = 0.005  # full credit: the tweezer tips at least 5 mm off the finger at the end

STATE: dict = {}


def reset_state(hand: ToolState | None = None) -> None:
    STATE.clear()
    STATE.update(fingered=False, pushed=False, pinched=False, off=False, torn=False, moved=0.0,
                 start=None if hand is None else hand.to_world((0.0, 0.0, 0.0)).copy())


reset_state()


def drawn(hand: ToolState) -> float:
    """How far the splinter has come out along its track (m); negative if pushed in."""
    return -float(hand.sliders["draw"])


def play_rad(hand: ToolState) -> float:
    """How far the splinter is levered off its track (rad), either way."""
    return math.hypot(hand.hinges["tilt"], hand.hinges["sway"])


def splinter_ends(hand: ToolState) -> tuple[np.ndarray, np.ndarray]:
    """World (outer end, inner end) of the splinter."""
    return hand.point("splinter_out"), hand.point("splinter_in")


def jaws_world(tw: ToolState) -> np.ndarray:
    """The 16 corners of the two slanted tips, in the world."""
    pts = [jaw_corners(side, tw.sliders[name]) for side, name in ((1, "leg_l"), (-1, "leg_r"))]
    return np.array([tw.to_world(p) for p in np.vstack(pts)])


def tip_gap(tw: ToolState) -> float:
    """Between the tips' inner faces (m)."""
    left = jaw_corners(1, tw.sliders["leg_l"])[:, 1].mean()
    right = jaw_corners(-1, tw.sliders["leg_r"])[:, 1].mean()
    return float(left - right - LEG_T)


def dent(hand: ToolState) -> float:
    """How far the fingertip's pad is pressed in (m)."""
    return -float(hand.sliders["pad"])


def over_index(hand: ToolState, pts: np.ndarray) -> np.ndarray:
    """Each point's height above the index finger's skin (m, the pad as dented), or +inf if it isn't over the
    finger."""
    _, _, x1, yc, width, _ = FINGERS[0]
    local = np.array([hand.to_local(p) for p in pts])
    inside = (local[:, 0] >= TIP_X[0]) & (local[:, 0] <= x1) & (np.abs(local[:, 1] - yc) <= width / 2)
    on_tip = local[:, 0] <= TIP_X[1]
    skin = np.where(on_tip, -dent(hand), 0.0)
    return np.where(inside, local[:, 2] - skin, np.inf)


def lever(hand: ToolState, tw: ToolState) -> float:
    """From the entry out along the splinter to the middle of the tweezer tips, at most to the stub's end: where a
    sideways pull acts."""
    mid = jaws_world(tw).mean(axis=0)
    out = hand.rotation() @ OUT
    along = float((mid - hand.to_world((0.0, 0.0, 0.0))) @ out)
    return float(np.clip(along, LEVER_MIN, max(LEVER_MIN, PROUD + drawn(hand))))


def side_load(hand: ToolState, tw: ToolState) -> float:
    """The sideways load on the stub at the entry (N), from the silicone's give and the lever to the tips."""
    return PLAY_RATE * play_rad(hand) / lever(hand, tw)


def exposed(hand: ToolState) -> np.ndarray:
    """Points every 0.5 mm along the splinter's axis, over the part out of the skin (all of it once out)."""
    outer, _ = splinter_ends(hand)
    long = min(PROUD + max(drawn(hand), 0.0), SPLINTER_LONG)
    return outer + np.linspace(0.0, long, int(long / 0.0005) + 1)[:, None] * (hand.rotation() @ IN)


def finger_gap(frame, pts: np.ndarray) -> float:
    """Closest distance (m) from ``pts`` to either finger of a gripper ``frame``: (TCP position, TCP rotation with
    columns closing, palm normal, approach, (each pad face's offset from the TCP along the closing axis))."""
    tcp, rot, faces = frame
    local = (np.asarray(pts) - np.asarray(tcp)) @ np.asarray(rot)
    half = np.array([FINGER_THICK / 2, FINGER_WIDE / 2, FINGER_LONG / 2])
    best = np.inf
    for face in faces:
        centre = np.array([face + math.copysign(FINGER_THICK / 2, face), 0.0, PAD_TIP_PAST_TCP - FINGER_LONG / 2])
        out = np.maximum(np.abs(local - centre) - half, 0.0)
        best = min(best, float(np.min(np.linalg.norm(out, axis=1))))
    return best


def fingered(hand: ToolState, frames) -> bool:
    """A gripper finger touches the splinter (its 2.4 mm collider, plus TOUCH)."""
    pts = exposed(hand)
    return any(finger_gap(f, pts) <= PROXY / 2 + TOUCH for f in frames)


def hand_moved(hand: ToolState) -> float:
    return float(np.linalg.norm(hand.to_world((0.0, 0.0, 0.0)) - STATE["start"]))


@on_reset
def _reset(states) -> None:
    reset_state(states["hand"])


@on_step
def _track(states, frames=None) -> None:
    hand, tw = states["hand"], states["tweezers"]
    if STATE["start"] is None:
        STATE["start"] = hand.to_world((0.0, 0.0, 0.0)).copy()
    frames = list(finger_frames().values()) if frames is None else frames
    if fingered(hand, frames):
        STATE["fingered"] = True
    if drawn(hand) <= -PUSHED_IN:
        STATE["pushed"] = True
    if dent(hand) > PINCH_DENT and tip_gap(tw) < SQUEEZED and np.min(over_index(hand, jaws_world(tw))) < ON_PAD:
        STATE["pinched"] = True
    if drawn(hand) < DRAWN_OUT and side_load(hand, tw) > F_TEAR:
        STATE["torn"] = True
    STATE["moved"] = max(STATE["moved"], hand_moved(hand))
    if STATE["moved"] > HAND_OFF or hand.tilt_deg() > HAND_TIP_DEG:
        STATE["off"] = True


def zeroed() -> bool:
    """A latched zero: a gripper finger on the splinter, the splinter pushed in, the skin pinched, the hand off."""
    return STATE["fingered"] or STATE["pushed"] or STATE["pinched"] or STATE["off"]


def tips_clear(hand: ToolState, tw: ToolState) -> bool:
    return bool(np.min(over_index(hand, jaws_world(tw))) >= CLEAR_OF_SKIN)


def ladder(out: float) -> float:
    """Credit for a splinter drawn ``out`` m and still in: 0, 0.25 or 0.5."""
    return 0.0 if out < DRAWN_QUARTER else 0.25 if out < DRAWN_HALF else 0.5


def grade(hand: ToolState, tw: ToolState) -> float:
    """The score as the outcome table states it."""
    if zeroed():
        return 0.0
    out = drawn(hand)
    if out < DRAWN_OUT:
        return ladder(out)
    whole = (not STATE["torn"] and STATE["moved"] <= FINGER_MOVED and tips_clear(hand, tw)
             and hand.resting() and tw.resting())
    return 1.0 if whole else 0.75


# Real-bench readings the sim can't make (its splinter has no slit and can't snap), as the scoring text states them.
def torn_on_bench(slit_before: float, slit_after: float) -> bool:
    """The slit (m), track empty, measured at reset before the trial and again after it: torn if it grew over 1 mm."""
    return slit_after - slit_before > SLIT_GROWTH + 1e-9


def snapped_score(piece_long: float, standing_out: float) -> float:
    """A snapped splinter: the piece left in the finger, ``piece_long`` m (0 if none), ``standing_out`` m of it out at
    the ink line at the end. Drawn out is 11 mm less what was still under the skin; at most 0.5."""
    if piece_long <= 0.0:
        return SNAPPED_CAP
    return min(ladder(round(BURIED - (piece_long - max(standing_out, 0.0)), 5)), SNAPPED_CAP)  # to 0.01 mm


def score(objects) -> float:
    return grade(objects["hand"], objects["tweezers"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres from the middle of the tweezer tips to the middle of the splinter's stub (0 once it is out)."""
    hand, tw = objects["hand"], objects["tweezers"]
    if drawn(hand) >= DRAWN_OUT:
        return 0.0
    outer, _ = splinter_ends(hand)
    stub = (outer + hand.to_world((0.0, 0.0, 0.0))) / 2
    return float(np.linalg.norm(jaws_world(tw).mean(axis=0) - stub))


# == props ============================================================================================================
SKIN = (0.91, 0.76, 0.66)  # pale skin-tone silicone
CREASE = (0.74, 0.56, 0.47)  # the skin's flexion creases, drawn on
PINE = (0.80, 0.64, 0.42)
STAINED = (0.42, 0.26, 0.13)  # the toothpick, stained dark brown
INK = (0.10, 0.10, 0.45)
STEEL = (0.78, 0.79, 0.81)
CUP_WHITE = (0.93, 0.94, 0.95)  # translucent white polypropylene
LABEL = (0.97, 0.97, 0.94)
HIDDEN = (0.5, 0.5, 0.5)
SOFT = 0.0025  # m: the drawn hand's long edges are softened this much
NAIL_R = 0.007  # m: and its fingertips rounded, in plan view, about this radius


def rounded(name, x, y, z, color=SKIN, tip=True, top=True, bottom=True, r=NAIL_R) -> list:
    """Drawn-only boxes inside the box spanning ``x``, ``y``, ``z`` (each (from, to), m): its -x end rounded in plan
    view (a fingertip) when ``tip``, and its long top and bottom edges softened. The colliders stay the plain boxes."""
    out = []
    for i, th in enumerate((0.16, 0.48, 0.79, 1.1, 1.41)):
        dw, dl = r * (1 - math.cos(th)), (r * (1 - math.sin(th)) if tip else 0.0)
        for j, (ey, ez0, ez1) in enumerate(((0.0, SOFT * bottom, SOFT * top), (SOFT, 0.0, 0.0))):
            x0, y0, y1, z0, z1 = x[0] + dl, y[0] + dw + ey, y[1] - dw - ey, z[0] + ez0, z[1] - ez1
            out.append((((x0 + x[1]) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x[1] - x0, y1 - y0, z1 - z0), color, 0.0,
                        f"{name}_look{i}{j}", 1.0, True))
    return out


def _crease(name, x, y, z, yaw=0.0) -> tuple:
    return ((x, y, z + 0.0001), (0.0008, 0.011, 0.0002), CREASE, yaw, name, 1.0, True)


def _hand_boxes() -> list:
    """The colliders are plain boxes, not drawn; what is drawn is the same shapes with rounded fingertips, softened
    edges and the palm's creases, so the hand reads as a hand."""
    boxes = []
    bottom = HAND_BOTTOM
    for name, x0, x1, yc, width, top in FINGERS:
        boxes.append((((x0 + x1) / 2, yc, (bottom + top) / 2), (x1 - x0, width, top - bottom), SKIN, 0.0, f"{name}",
                      0.0))
        index = name == "index"
        span = (x0, x1)
        boxes += rounded(name, span, (yc - width / 2, yc + width / 2), (bottom, top), tip=not index)
        long = x1 - (TIP_X[0] if index else x0)
        for k, f in enumerate((0.36, 0.66)):  # the two finger creases, palm side
            boxes.append(_crease(f"{name}_crease{k}", x1 - long + f * long + (0.002 if index else 0.0), yc, top))
    for name, ((x0, x1), (y0, y1), top) in (("palm", PALM), ("thumb", THUMB), ("wrist", WRIST)):
        boxes.append((((x0 + x1) / 2, (y0 + y1) / 2, (bottom + top) / 2), (x1 - x0, y1 - y0, top - bottom), SKIN, 0.0,
                      name, 0.0))
        boxes += rounded(name, (x0, x1), (y0, y1), (bottom, top), tip=name == "thumb",
                         r=NAIL_R if name == "thumb" else 0.004)
    px, py, _ = PALM
    for k, (fx, fy, yaw) in enumerate(((0.25, 0.55, 0.35), (0.45, 0.45, -0.25), (0.70, 0.30, 0.6))):  # palm creases
        c = _crease(f"palm_crease{k}", px[0] + fx * (px[1] - px[0]), py[0] + fy * (py[1] - py[0]), PALM[2], yaw)
        boxes.append((c[0], (0.0008, 0.040, 0.0002), *c[2:]))
    # the index fingertip under its pad
    tip_y = (-FINGERS[0][4] / 2, FINGERS[0][4] / 2)
    boxes.append((((TIP_X[0] + TIP_X[1]) / 2, 0.0, (bottom - PAD_DEPTH) / 2),
                   (TIP_X[1] - TIP_X[0], FINGERS[0][4], -PAD_DEPTH - bottom), SKIN, 0.0, "index_tip", 0.0))
    boxes += rounded("index_tip", TIP_X, tip_y, (bottom, -PAD_DEPTH), top=False)
    # the mm scale label on the fingertip's right side: 25 x 7 mm, its top edge 5 mm under the pad's surface
    face = tip_y[0] - 0.0001
    boxes.append(((LABEL_X[0] + 0.0125, face, -0.0085), (0.025, 0.0002, 0.007), LABEL, 0.0, "scale_label", 1.0, True))
    for mm in range(26):
        tall = 0.003 if mm % 5 == 0 else 0.0015
        boxes.append(((LABEL_X[0] + mm * 0.001, face - 0.0001, -0.005 - tall / 2), (0.0002, 0.0002, tall),
                      INK, 0.0, f"scale_tick{mm}", 1.0, True))
    return boxes


LABEL_X = (-0.002, 0.023)  # hand frame: the label runs from 2 mm before the ink line to 23 mm behind it


def _play(name: str, axis: str, parent: str = "") -> Hinge:
    """The silicone's give around the splinter: a sprung turn about the entry, inside the finger."""
    return Hinge(name, rest=(0.0, 0.0, 0.0), parent=parent, axis=axis, frame=TRACK_Q, lower=-0.6, upper=0.6,
                 rate=PLAY_RATE, damping=0.0004, friction=0.0, mass=0.010, weightless=True,
                 boxes=(((0.003, 0.0, -0.008), (0.003, 0.003, 0.003), HIDDEN, 0.0, f"{name}_core", 0.0),))


SPLINTER_MID = IN * (BURIED - SPLINTER_LONG / 2)  # the splinter's centre at reset, hand frame
HOLD_FORCE = 1.0  # N to slide it along its track: a sim model value (reset checks the real one is at least 0.5 N)
TRACK_END = 0.016  # m: the track runs on 5 mm past fully out
DRAW = Slider("draw", rest=tuple(float(v) for v in SPLINTER_MID), size=(SPLINTER_LONG, PROXY, PROXY), axis="X",
              frame=TRACK_Q, box_turn=TRACK_Q, parent="sway", lower=-TRACK_END, upper=0.004, rate=0.0, damping=20.0,
              friction=HOLD_FORCE, mass=0.010, weightless=True, opacity=0.0, box_name="splinter_proxy",
              # drawn round: two 1.8 mm square sticks, one turned 45 degrees about its axis
              boxes=tuple(((0.0, 0.0, 0.0), (SPLINTER_LONG, SPLINTER_DIA, SPLINTER_DIA), STAINED,
                           q(TRACK @ rot_x(a)), f"splinter{'_45' if a else ''}", 1.0, True) for a in (0.0, math.pi / 4)))
_PAD_MID = ((TIP_X[0] + TIP_X[1]) / 2, 0.0, -PAD_DEPTH / 2)
PAD = Slider("pad", rest=_PAD_MID, size=(TIP_X[1] - TIP_X[0], FINGERS[0][4], PAD_DEPTH), axis="Z", lower=-PAD_GIVE,
             upper=0.0, color=SKIN, rate=PAD_RATE, damping=3.0, mass=0.010, weightless=True, box_name="index_pad",
             opacity=0.0,
             # drawn: the pad with its rounded tip, and the ink line across it where the splinter went in
             boxes=tuple((tuple(c - m for c, m in zip(b[0], _PAD_MID)), *b[1:]) for b in rounded(
                 "index_pad", TIP_X, (-FINGERS[0][4] / 2, FINGERS[0][4] / 2), (-PAD_DEPTH, 0.0), bottom=False))
             + (((-(TIP_X[0] + TIP_X[1]) / 2, 0.0, PAD_DEPTH / 2 + 0.0001), (0.001, 0.012, 0.0002), INK, 0.0,
                 "ink_line", 1.0, True),))
HAND = tool("hand", pos=tuple(float(v) for v in ENTRY), boxes=_hand_boxes(), mass=HAND_MASS, friction=1.0,
            hinges=[_play("tilt", "Y"), _play("sway", "Z", "tilt")], sliders=[PAD, DRAW],
            link_points=[("splinter_out", "draw", tuple(float(v) for v in OUT * SPLINTER_LONG / 2)),
                         ("splinter_in", "draw", tuple(float(v) for v in IN * SPLINTER_LONG / 2))])


def _leg(name: str, side: int) -> Slider:
    # the left leg (+y) closes sliding -y, the right one +y; each stops just short of its tip meeting the other's
    lo, hi = (-CLOSE_TRAVEL, 0.0) if side > 0 else (0.0, CLOSE_TRAVEL)
    return Slider(name, rest=tuple(float(v) for v in leg_rest(side)), size=GRIP_BUMP,
                  axis="Y", lower=lo, upper=hi, color=STEEL, rate=LEG_RATE, damping=LEG_DAMPING, mass=LEG_MASS,
                  box_name=f"grip_{'l' if side > 0 else 'r'}", boxes=leg_boxes(side, STEEL))


TWEEZERS = tool("tweezers", pos=TWEEZER_POS, mass=TW_MASS - 2 * LEG_MASS, friction=1.0,
                quat=xyzw(TWEEZER_TURN),
                boxes=[((0.0, 0.0, -ROOT_LONG / 2), (TW_WIDE, 2 * LEG_T, ROOT_LONG), STEEL, 0.0, "joined_end")],
                sliders=[_leg("leg_l", 1), _leg("leg_r", -1)])


def _cup() -> list:
    cx, cy = CUP_XY
    parts = []
    for k, (lo, hi, radius) in enumerate(cup_rings()):
        side = 2 * (radius + SIM_WALL) * math.tan(math.pi / CUP_SIDES) + 0.0004
        r = radius + SIM_WALL / 2
        for i in range(CUP_SIDES):  # twelve flat sides whose inner faces touch the ring's circle
            a = 2 * math.pi * i / CUP_SIDES
            parts.append(Box(f"cup_{k}_{i}", pos=(cx + r * math.cos(a), cy + r * math.sin(a), TOP + (lo + hi) / 2),
                             size=(SIM_WALL, side, hi - lo), yaw=a, color=CUP_WHITE))
    # the floor is 3 mm thick in the sim, its top at the real 1 mm (thinner colliders tunnel)
    floor = 2 * cup_rings()[0][2]
    parts.append(Box("cup_floor", pos=(cx, cy, TOP + CUP_FLOOR - 0.0015), size=(floor, floor, 0.003), color=CUP_WHITE))
    return parts


OBJECTS = [
    Box("block", pos=BLOCK_POS, size=BLOCK, color=PINE),
    *_cup(),
    HAND,
    TWEEZERS,
]

# == the scripted demo: the right arm takes the tweezers and draws the splinter out; the left arm stays parked =========
# Taking them: the fingers come down the tweezers' own line, pitched forward as far as they lean, and close on the
# grip section, their tips at its lower end; the fingertips stay outside the cup's rim.
# Holding them lightly (each leg in 0.8 mm at 2.4 N) leaves the tips 4.4 mm apart, wide enough to pass either side of
# the 2.4 mm stub. Using them: the wrist pitches on to 65 degrees, so the tweezers lie along the splinter. The open tips
# are set down on the pad either side of the stub, sent 1.5 mm below its surface so they come to rest on it whatever
# the arm's sag (the pad dents about 0.3 mm). Shut, each finger squeezes about 7 N, each tip about 2 N on the
# stub, which holds far more than its 1.0 N in the track. The pull is commanded 20 mm along the splinter, past the
# 16 mm end of its track, since the arm gives about 5 mm per newton along a pull.
GRIP_AT = GRIP_TO - PAD_TIP_PAST_TCP  # m below the joined end: the pad tips at the grip section's lower end
HOLD_WIDTH = 0.00232  # finger target (each pad, from the middle): each leg in 0.8 mm at 2.4 N, the tips 4.4 mm apart
APPROACH = 0.050  # m above the grip before coming down
DRAW_OUT = 0.035  # m back along the tweezers' own line, so the tips leave the cup's far wall
LIFT = 0.040  # m then straight up
SET_DOWN = 0.0015
TIP_SHORT = -0.0015  # the tips' front end this far short of the ink line (along the table): 1.5 mm past it, so they
                     # straddle the whole stub even if the tweezers sit a few millimetres back in the grip
ABOVE = 0.010  # m above the stub before setting the open tips down either side of it
EASE_BACK = 0.003  # m back along the splinter while the tips close, so squeezing never pushes the stub in
CLAMP_WIDTH = 0.0005  # finger target while the tips are shut on the stub: about 7 N a pad
PULL = 0.020
PITCH_USE = 90.0 - math.degrees(ENTRY_ANGLE)  # the tweezers along the splinter: 65 degrees from straight down


def tool_turn(pitch_deg: float) -> np.ndarray:
    """The tweezers' rotation when they lie ``pitch_deg`` from straight down, tips forward."""
    return rot_y(-math.radians(pitch_deg))


def tcp_for(turn: np.ndarray, origin) -> tuple:
    """The TCP for tweezers whose joined end is at ``origin`` with rotation ``turn``."""
    return tuple(float(v) for v in np.asarray(origin) + turn @ np.array([0.0, 0.0, -GRIP_AT]))


def use_origin() -> np.ndarray:
    """Where the joined end goes so the slanted tips lie on the pad either side of the stub: their lowest corner on the
    skin, their front end TIP_SHORT short of the ink line (past it if negative), centred on the splinter."""
    turn = tool_turn(PITCH_USE)
    pts = np.array([turn @ p for p in np.vstack([jaw_corners(1, 0.0), jaw_corners(-1, 0.0)])])
    return np.array([ENTRY[0] - TIP_SHORT - pts[:, 0].max(), ENTRY[1], ENTRY[2] - pts[:, 2].min()])


def demo():
    lean = math.degrees(LEAN)
    take, use = Grip.deg(pitch=lean), Grip.deg(pitch=PITCH_USE)
    t0, t1 = tool_turn(lean), tool_turn(PITCH_USE)
    up = np.array([0.0, 0.0, 1.0])
    grip = np.array(tcp_for(t0, TWEEZER_POS))
    along = t0 @ up  # up the tweezers' line
    origin = use_origin()
    on_stub = tcp_for(t1, origin - SET_DOWN * up)
    closed = tcp_for(t1, origin - SET_DOWN * up + EASE_BACK * OUT)
    pre = tcp_for(t1, origin + ABOVE * up)
    pulled = tcp_for(t1, origin - SET_DOWN * up + PULL * OUT)
    high = np.array(tcp_for(t1, origin + np.array([-0.02, 0.0, 0.06])))
    drawn_out = grip + DRAW_OUT * along
    lifted = drawn_out + LIFT * up
    mid = (lifted + high) / 2 + 0.02 * up
    hold = -HOLD_WIDTH
    return {"right": [
        Waypoint(2.0, tuple(grip + APPROACH * along), FINGER_OPEN, "in line above the tweezers", take),
        Waypoint(1.5, tuple(grip), FINGER_OPEN, "down to the grip section", take),
        Waypoint(1.0, tuple(grip), hold, "grip the tweezers", take),
        Waypoint(1.5, tuple(drawn_out), hold, "draw them up out of the cup", take),
        Waypoint(1.5, tuple(lifted), hold, "lift", take),
        Waypoint(2.5, tuple(mid), hold, "carry", Grip.deg(pitch=(PITCH_USE + lean) / 2)),
        Waypoint(2.5, tuple(high), hold, "above the finger", use),
        Waypoint(1.5, pre, hold, "over the stub", use),
        Waypoint(0.8, pre, hold, "settle", use),
        Waypoint(1.5, on_stub, hold, "set the tips down either side of the stub", use),
        Waypoint(0.8, on_stub, hold, "settle", use),
        Waypoint(2.0, closed, -CLAMP_WIDTH, "close the tips, easing back", use),
        Waypoint(5.0, pulled, -CLAMP_WIDTH, "draw the splinter out", use),
        Waypoint(3.0, pulled, -CLAMP_WIDTH, "keep still", use),
    ]}
