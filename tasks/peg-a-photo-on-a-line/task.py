"""On the station table: a photo-display wire stretched taut between two posts, running across in front of the arms,
with a photo print hanging from it by one soft-grip wooden clothes peg at the print's left top corner, and more pegs
clipped on the rim of a peg basket to the right.

Held by one corner, the print has swung down: its free top corner hangs 10 cm below the wire. It has to be pegged at
that corner too. A clothes peg is two wooden legs on a pivot with a torsion spring: squeezing its handles together
opens its jaws, and letting go closes them on whatever is between them. So one hand has to lift the print's hanging
end until its top edge is back at the wire and keep it there, while the other squeezes a peg off the basket's rim,
carries it squeezed open, brings it down so that its open jaws pass either side of the wire and the print's edge
together (the wire 1.2 mm, the print 0.3 mm, in front of it), and lets go there.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import pegkit as K  # noqa: E402
from pegkit import Fixed, Joint, Peg, Photo, finger_points, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "The photo hangs from the wire by one peg. Peg its other top corner to the wire too."

# -- layout (m; x forward from the arms, y left, heights above the table) ------------------------------------------------
LOT = os.environ.get("PEG_LOT", "A")
LOTS = {"A": dict(pivot_y=0.030), "B": dict(pivot_y=0.015)}
if LOT not in LOTS:
    raise ValueError(f"PEG_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
PIVOT_Y = LOTS[LOT]["pivot_y"]          # where along the wire the first peg holds the photo's left top corner

# -- the wire: a 1.2 mm steel photo-display wire, stretched taut between two posts, running across the table (y) --------
LINE_X, LINE_Z, LINE_R = 0.300, 0.160, 0.0006
LINE_Y = (-0.150, 0.150)
POST = 0.020

# -- the photo: a 5 x 3.5 in (127 x 89 mm) print on 250 gsm photo paper, in front of the wire ----------------------------
PHOTO = (0.127, 0.0003, 0.089)          # wide, thick, tall
PHOTO_MASS = 0.0028
LEVEL_ROT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # its width along -y, its picture facing the arms
ABOVE = 0.012                           # hanging level, its top edge is this far above the wire's axis
PEG_IN = 0.005                          # a corner's peg sits this far in from the photo's end
IN_FRONT = 0.0004                       # hanging by one corner, it is this far in front of the wire
PHOTO_HOLD = 2.0                        # N: a pull this hard takes the photo out of its first peg's jaws
PEG_HOLD = 3.0                          # N: ... and a push this hard takes the first peg off the wire
PEG_MOVED = 0.005                       # the first peg this far from its place has let the photo go
FLEX, FLEX_MAX = 0.005, 20.0            # the print leans out of its plane against this spring (N m/deg), up to this (deg)

# -- the pegs and their basket -------------------------------------------------------------------------------------------
PEG_MASS = 0.006
PEG_QUAT = (0.0, 0.0, 1.0, 0.0)         # the legs part across the wire (x): the fixed leg behind it, the sprung leg in front
BASKET_X = 0.240                        # the basket's near wall, along y, where the spare pegs are clipped
BASKET_Y = (-0.300, -0.180)
PEGS_Y = (-0.220, -0.260)
RIM_Z = 0.090
RIM_T = 0.003
RIM_DEPTH = 0.005                       # the rim sits this far up from the jaw tips of a peg clipped on it
CLIP_DEPTH = K.NOTCH_Z[1] - LINE_R - 0.0001   # a peg hangs on the wire by the top of its jaws' groove: the wire's
                                        # middle is this far up from the tips

# -- scoring -----------------------------------------------------------------------------------------------------------
HANG_STEPS = 40                         # hanging by both pegs this many control steps (2 s) for full marks
TAKEN_STEPS = 5                         # a spare peg off the rim this many control steps (0.25 s) has been taken
RAISED = math.radians(10.0)             # the photo within this of level: its hanging end has been raised to the wire
RAISED_STEPS = 20                       # ... for this many control steps (1 s)
LEVEL = math.radians(5.0)               # pegged at both corners it hangs within this of level
STILL_MOVE, STILL_STEPS = 0.001, 10     # at rest: moved less than 1 mm in the last 10 control steps (0.5 s)
CLEAR = 0.040                           # both grippers' tool points this far from the new peg and from the photo: let go
CLOSED = math.radians(-4.0)             # a peg's spring joint above this: jaws closed by the spring, not squeezed open
GROOVE = 0.0015                         # m: the jaws' groove for the line, cut this deep into each face
PAPER_DOWN = 0.002                      # m: the photo is checked this far down from its top edge
TIP_UP = 0.0025                         # m: ... and at the jaws' tips, this far up from their ends
JAW_TOL = 0.0006                        # m: slack allowed outside the jaws' faces for the wire and the photo
PINCH_TOL = 0.0002                      # m: slack allowed between the photo's sheet and the gap between the jaws' tips
EDGE_IN = 0.002                         # m: the tips' pinch is this far inside the photo's edges
CORNER = 0.020                          # a peg whose tips pinch the photo within this of its free end holds that corner
FALLEN = 0.010                          # the photo's pegged corner this far from where it hung: off its first peg
SPARES = tuple(f"peg_{i + 1}" for i in range(len(PEGS_Y)))
FIRST = "peg_far"


def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def beside() -> float:
    """The photo's middle plane, hanging by one corner: just in front of the wire (toward the arms)."""
    return LINE_X - LINE_R - IN_FRONT - PHOTO[1] / 2


def pivot_local() -> np.ndarray:
    """Where the first peg's jaws hold the photo, in the photo's own frame (x along its width toward its free end,
    z up its height): PEG_IN from its left end, at the wire."""
    return np.array([-(PHOTO[0] / 2 - PEG_IN), 0.0, PHOTO[2] / 2 - ABOVE])


def pivot_point() -> np.ndarray:
    return np.array([beside(), PIVOT_Y, TOP + LINE_Z])


def hang_angle() -> float:
    """How far the photo swings down, held by that one corner: until its middle is straight below the peg."""
    return math.atan2(PHOTO[0] / 2 - PEG_IN, PHOTO[2] / 2 - ABOVE)


def photo_pose(angle: float) -> tuple:
    """The photo's centre and quaternion when it has swung ``angle`` down from level about its first peg."""
    rot = rot_x(angle) @ LEVEL_ROT
    return pivot_point() - rot @ pivot_local(), K._quat(rot)


def photo_angle(quat) -> float:
    """How far the photo's top edge is from level (rad, positive: its free end low)."""
    along = K._rotation(quat)[:, 0]                             # along its width, toward its free end
    return math.atan2(-float(along[2]), -float(along[1]))


def photo_point(pos, quat, local) -> np.ndarray:
    return np.asarray(pos, float) + K._rotation(quat) @ np.asarray(local, float)


def peg_place(end: int) -> np.ndarray:
    """Where a peg clipped on the wire and the level photo at its left (+1) or free (-1) top corner has its pivot's
    middle: hanging on the wire by its jaws' groove, the fixed leg behind the wire, the sprung leg's tip pinching the
    photo's front."""
    y = PIVOT_Y if end > 0 else PIVOT_Y - (PHOTO[0] - 2 * PEG_IN)
    return np.array([LINE_X - K.GAP / 4, y, TOP + LINE_Z - CLIP_DEPTH + K.PIVOT_Z])   # the wire midway under both jaws' shoulders


def spare_start(i: int) -> np.ndarray:
    """A spare peg clipped on the basket's rim: its pivot's middle."""
    return np.array([BASKET_X, PEGS_Y[i], TOP + RIM_Z - RIM_DEPTH + K.PIVOT_Z])


def _line() -> list:
    wire = (0.80, 0.81, 0.84)
    out = [Fixed("line", pos=(LINE_X, sum(LINE_Y) / 2, TOP + LINE_Z), size=(2 * LINE_R, LINE_Y[1] - LINE_Y[0], 2 * LINE_R),
                 color=wire, roughness=0.3, metallic=0.8, static_friction=0.5, dynamic_friction=0.4)]
    for k, y in enumerate(LINE_Y):
        side = -1 if k == 0 else 1
        out.append(Fixed(f"post_{k + 1}", pos=(LINE_X, y + side * POST / 2, TOP + (LINE_Z + 0.012) / 2),
                         size=(POST, POST, LINE_Z + 0.012), color=(0.35, 0.25, 0.18), roughness=0.8))
    return out


def _basket_box() -> tuple:
    """The basket's footprint: (x0, x1), (y0, y1). Its near wall, where the pegs are clipped, is at x0."""
    x0 = BASKET_X - RIM_T / 2
    return (x0, x0 + 0.12), BASKET_Y


def _basket() -> list:
    blue = (0.20, 0.45, 0.75)
    (x0, x1), (y0, y1) = _basket_box()
    parts = [("basket_near", (x0, x0 + RIM_T), (y0, y1), RIM_Z), ("basket_far", (x1 - RIM_T, x1), (y0, y1), RIM_Z),
             ("basket_left", (x0, x1), (y1 - RIM_T, y1), RIM_Z), ("basket_right", (x0, x1), (y0, y0 + RIM_T), RIM_Z),
             ("basket_floor", (x0, x1), (y0, y1), 0.004)]
    return [Fixed(n, pos=((xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2, TOP + h / 2), size=(xs[1] - xs[0], ys[1] - ys[0], h),
                  color=blue, roughness=0.6) for n, xs, ys, h in parts]


def _objects() -> list:
    pos, quat = photo_pose(hang_angle())
    first = peg_place(+1)                                       # on the wire; the photo's corner between its jaws
    return [
        *_line(), *_basket(),
        Photo("photo", pos=tuple(float(v) for v in pos), quat=quat, size=PHOTO, mass=PHOTO_MASS, no_touch=(FIRST,)),
        Peg(FIRST, pos=tuple(float(v) for v in first), quat=PEG_QUAT, size=(0.024, K.PEG_W, K.PEG_LEN), mass=PEG_MASS,
            open0=K.SHUT - math.radians(0.2)),
        *(Peg(SPARES[i], pos=tuple(float(v) for v in spare_start(i)), quat=PEG_QUAT, size=(0.024, K.PEG_W, K.PEG_LEN),
              mass=PEG_MASS, open0=math.radians(-5.0)) for i in range(len(SPARES))),
    ]


OBJECTS = _objects()

# The first peg's hold. On the wire it stays where it was clipped, up to a 3 N push. Its jaws hold the photo's corner
# as a pivot: the print turns in them under its own weight, bends a little out of its plane, and comes out at a 2 N
# pull (or when the peg itself is knocked 5 mm from its place).
K.joints(Joint("hold", None, (FIRST, "leg_a"), kind="fixed", force=PEG_HOLD, torque=0.05),
         Joint("pivot", None, ("photo", None), kind="pivot", anchor1=tuple(float(v) for v in pivot_local()),
               force=PHOTO_HOLD, torque=0.5, cone=FLEX_MAX, flex=FLEX))


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(taken=False, off_steps=0, raised=False, raised_steps=0, held_steps=0, held=False, fallen=False, corner0=None, steps=0,
                  angle=0.0, first0=None, let_go=False, past=[])


_clear()


def wire_in_peg(pos, quat):
    """Where the wire crosses a peg's middle plane (the plane across its width), in the peg's frame; None if it does
    not cross it between the posts. A peg hanging crooked on the wire is judged where the wire actually passes it."""
    pos = np.asarray(pos, float)
    rot = K._rotation(quat)
    across = rot[:, 1]                                          # the peg's width, in the world
    if abs(across[1]) < 0.3:
        return None
    y = pos[1] - (across[0] * (LINE_X - pos[0]) + across[2] * (TOP + LINE_Z - pos[2])) / across[1]
    if not LINE_Y[0] <= y <= LINE_Y[1]:
        return None
    return rot.T @ (np.array([LINE_X, y, TOP + LINE_Z]) - pos)


def jaw_faces(z: float, spring: float) -> tuple:
    """In a peg's frame, the x of its two jaws' faces at height z (negative: below the pivot) for spring angle
    ``spring`` (rad, negative open): leg a is fixed, leg b turns about the pivot's y axis."""
    return -K.GAP / 2, K.GAP / 2 * math.cos(spring) + z * math.sin(spring)


def between_jaws(local, spring: float, half: float = 0.0, slack: float = JAW_TOL) -> bool:
    """Whether a point given in a peg's frame (the middle of something of half-thickness ``half`` across the jaws)
    lies between its jaws' faces (give or take ``slack``), in the jaws (between the tips and the pivot) and across the
    peg's width."""
    lo, hi = jaw_faces(float(local[2]), spring)
    return (lo - slack < local[0] - half and local[0] + half < hi + slack
            and -K.PIVOT_Z < local[2] < 0.0 and abs(local[1]) < K.PEG_W / 2)


def rim_in_peg(pos, quat):
    """Where the top edge of the basket's near rim crosses a peg's middle plane (the plane across its width), in the
    peg's frame; None if it does not cross it within the basket."""
    pos = np.asarray(pos, float)
    rot = K._rotation(quat)
    across = rot[:, 1]
    if abs(across[1]) < 0.3:
        return None
    y = pos[1] - (across[0] * (BASKET_X - pos[0]) + across[2] * (TOP + RIM_Z - pos[2])) / across[1]
    if not BASKET_Y[0] <= y <= BASKET_Y[1]:
        return None
    return rot.T @ (np.array([BASKET_X, y, TOP + RIM_Z]) - pos)


def peg_on_rim(pos, quat, spring: float) -> bool:
    """Whether a peg is still on the basket's near rim: the rim's top edge passes between its two legs (squeezed open
    or not), no more than 2 mm below their tips and no more than 20 mm up from them. Sliding a peg along the rim, or
    squeezing it open without lifting it, keeps it on the rim."""
    rim = rim_in_peg(pos, quat)
    if rim is None:
        return False
    lo, hi = jaw_faces(float(rim[2]), spring)
    return bool(lo - K.LEG_T < rim[0] < hi + K.LEG_T and -K.PIVOT_Z - 0.002 < rim[2] < -K.PIVOT_Z + 0.020)


def in_basket(pos) -> bool:
    """Whether a peg has dropped into the basket (inside its walls, below the rim)."""
    (x0, x1), (y0, y1) = _basket_box()
    return x0 < pos[0] < x1 and y0 < pos[1] < y1 and pos[2] < TOP + RIM_Z


def taken_off(pos, quat, spring: float) -> bool:
    """A spare peg is off the rim and not lying in the basket."""
    return not peg_on_rim(pos, quat, spring) and not in_basket(pos)


def photo_top_edge(photo_pos, photo_quat) -> np.ndarray:
    """The middle of the photo's top edge."""
    return np.asarray(photo_pos) + K._rotation(photo_quat)[:, 2] * PHOTO[2] / 2


def grip_check(pos, quat, spring: float, photo_pos, photo_quat) -> dict:
    """What a peg's jaws hold. ``closed``: they have been let shut by the spring (not squeezed open). ``wire``: the
    wire lies between the jaws, in their groove. ``photo``: the photo's paper lies between the jaws just under its top
    edge (2 mm down, under the peg), and the jaws' tips pinch it: the gap between the tips' faces lies on the photo's
    sheet, 2 mm or more inside its edges. (A peg let shut beside the photo has its tips shut on nothing, off the
    sheet.) ``along``: the peg's place along the photo's top edge, from its middle toward its free end."""
    pos = np.asarray(pos, float)
    rot = K._rotation(quat)
    wire = wire_in_peg(pos, quat)
    prot = K._rotation(photo_quat)
    top = photo_top_edge(photo_pos, photo_quat)
    along = float((pos - top) @ prot[:, 0])
    paper = rot.T @ (top + prot[:, 0] * along - prot[:, 2] * PAPER_DOWN - pos)
    # the gap between the jaws' tips: its middle, seen from the photo (across its width, through its thickness, up it)
    z = -K.PIVOT_Z + TIP_UP
    lo, hi = jaw_faces(z, spring)
    tip = prot.T @ (pos + rot @ np.array([(lo + hi) / 2, 0.0, z]) - np.asarray(photo_pos, float))
    pinched = bool(abs(tip[0]) < PHOTO[0] / 2 - EDGE_IN and abs(tip[2]) < PHOTO[2] / 2 - EDGE_IN
                   and abs(tip[1]) <= max(hi - lo, 0.0) / 2 + PHOTO[1] / 2 + PINCH_TOL)
    return {"closed": bool(spring > CLOSED),
            "wire": bool(wire is not None and between_jaws(wire, spring, LINE_R, GROOVE + JAW_TOL)),  # in the jaws' groove
            "photo": bool(between_jaws(paper, spring, PHOTO[1] / 2) and pinched),
            "along_mm": round(1000 * along, 1),
            "wire_mm": None if wire is None else [round(1000 * float(v), 2) for v in wire],
            "paper_mm": [round(1000 * float(v), 2) for v in paper],
            "tip_mm": [round(1000 * float(v), 2) for v in tip]}


def grips_wire_and_photo(pos, quat, spring: float, photo_pos, photo_quat) -> bool:
    g = grip_check(pos, quat, spring, photo_pos, photo_quat)
    return g["closed"] and g["wire"] and g["photo"]


def holds_free_corner(pos, quat, spring: float, photo_pos, photo_quat) -> bool:
    """Whether a peg holds the photo's free top corner on the wire: its jaws closed on both the wire and the photo,
    its tips pinching the photo within 2 cm of the photo's free end."""
    g = grip_check(pos, quat, spring, photo_pos, photo_quat)
    return bool(g["closed"] and g["wire"] and g["photo"] and g["tip_mm"][0] / 1000 > PHOTO[0] / 2 - CORNER)


def gap_to_photo(point, photo_pos, photo_quat) -> float:
    """How far a point is from the photo (m)."""
    local = K._rotation(photo_quat).T @ (np.asarray(point, float) - np.asarray(photo_pos, float))
    half = np.array([PHOTO[0], PHOTO[1], PHOTO[2]]) / 2
    return float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0)))


def advance(s: dict, taken: bool, angle: float, corner_off: float, pegged: bool, let_go: bool, still: bool) -> None:
    """One control step of the trial's record (pure). ``taken``: a spare peg is off the basket's rim now (it counts
    as taken after 0.25 s off it); ``angle``: the
    photo's top edge from level (rad); ``corner_off``: how far its pegged corner is from where it hung (m);
    ``pegged``: a second peg, let go and at rest, holds the wire and the photo's free corner; ``let_go``: both grippers
    are clear of the photo; ``still``: the photo is at rest."""
    s["steps"] += 1
    s["angle"] = angle
    s["off_steps"] = s["off_steps"] + 1 if taken else 0
    s["taken"] = s["taken"] or s["off_steps"] >= TAKEN_STEPS
    if corner_off >= FALLEN:
        s["fallen"] = True
    s["raised_steps"] = s["raised_steps"] + 1 if abs(angle) <= RAISED and not s["fallen"] else 0
    s["raised"] = s["raised"] or s["raised_steps"] >= RAISED_STEPS
    good = pegged and let_go and still and abs(angle) <= LEVEL and not s["fallen"]
    s["held_steps"] = s["held_steps"] + 1 if good else 0
    s["held"] = s["held"] or s["held_steps"] >= HANG_STEPS


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    states = object_states(environment, ["photo", FIRST, *SPARES])
    hands = finger_points(environment)
    ph = states["photo"]
    corner = photo_point(ph.pos, ph.quat_xyzw, pivot_local())
    first = np.asarray(states[FIRST].pos, float)
    if s["corner0"] is None:
        s["corner0"], s["first0"] = corner, first
    if not s["let_go"] and float(np.linalg.norm(first - s["first0"])) >= PEG_MOVED:
        s["let_go"] = True                                      # the first peg was knocked off: the photo drops
        K.release("pivot")
    # At rest: where each thing is now against where it was half a second ago (a pinched print's reported speed
    # chatters by millimetres a second while it does not move).
    now = {n: np.asarray(states[n].pos, float) for n in ("photo", *SPARES)}
    now["photo_free"] = photo_point(ph.pos, ph.quat_xyzw, (PHOTO[0] / 2, 0.0, 0.0))
    s["past"].append(now)
    before = s["past"].pop(0) if len(s["past"]) > STILL_STEPS else None

    def rest(*names) -> bool:
        return before is not None and all(float(np.linalg.norm(now[n] - before[n])) < STILL_MOVE for n in names)

    taken, pegged = False, False
    for p in SPARES:
        st = states[p]
        pos = np.asarray(st.pos, float)
        taken = taken or taken_off(pos, st.quat_xyzw, st.joints["spring"])
        free = all(float(np.linalg.norm(pos - h)) > CLEAR for h in hands)
        if free and rest(p) and holds_free_corner(pos, st.quat_xyzw, st.joints["spring"], np.asarray(ph.pos), ph.quat_xyzw):
            pegged = True
    let_go = all(gap_to_photo(h, ph.pos, ph.quat_xyzw) > CLEAR for h in hands)
    advance(s, taken, photo_angle(ph.quat_xyzw), float(np.linalg.norm(corner - s["corner0"])), pegged, let_go,
            rest("photo", "photo_free"))


# -- scoring --------------------------------------------------------------------------------------------------------
def score_from(taken: bool, raised: bool, held: bool, fallen: bool) -> float:
    """Pure scoring rule. ``taken``: a spare peg was taken off the basket's rim. ``raised``: the photo's hanging end
    was lifted until the photo was within 10 degrees of level, for 1 s. ``held``: a second peg held the wire and the
    photo's free corner, everything let go, the photo within 5 degrees of level and at rest, for 2 s. ``fallen``: the
    photo came off its first peg."""
    if fallen:
        return 0.0
    if held:
        return 1.0
    return 0.25 * (int(taken) + int(raised))


def score(objects) -> float:
    """0 if the photo ever came off its first peg (its pegged corner 1 cm from where it hung). Otherwise 1.0 once a
    second peg hangs on the wire with the wire in its jaws and its tips pinching the photo within 2 cm of the photo's
    free end, the peg and the photo let go (both grippers over 4 cm from each), with the photo within 5 degrees of
    level and both at rest (moving under 1 mm in 0.5 s), for 2 s. Otherwise 0.25 for each of: a spare peg taken off
    the basket's rim (not slid along it, not dropped into the basket); the photo's hanging end raised until the photo
    was within 10 degrees of level, for 1 s."""
    s = _STATE
    return score_from(s["taken"], s["raised"], s["held"], s["fallen"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    out = {k: _STATE[k] for k in ("taken", "raised", "raised_steps", "held_steps", "held", "fallen")}
    out["score"] = score(objects)
    ph = objects["photo"]
    out["photo_deg"] = round(math.degrees(photo_angle(ph.quat_xyzw)), 2)
    normal = K._rotation(ph.quat_xyzw)[:, 1]
    out["lean_deg"] = round(math.degrees(math.acos(max(-1.0, min(1.0, float(normal[0]))))), 2)
    if _STATE["corner0"] is not None:
        out["corner_off_mm"] = round(1000 * float(np.linalg.norm(photo_point(ph.pos, ph.quat_xyzw, pivot_local())
                                                                 - _STATE["corner0"])), 2)
    for n in ("photo", FIRST, *SPARES):
        st = objects[n]
        out[n] = [round(100 * float(v), 3) for v in (np.asarray(st.pos, float) - [0.0, 0.0, TOP])]
        if n != "photo":
            sp = st.joints.get("spring", 0.0)
            out[n + "_open_deg"] = round(math.degrees(sp), 2)
            out[n + "_on_rim"] = peg_on_rim(np.asarray(st.pos), st.quat_xyzw, sp)
            out[n + "_grip"] = grip_check(np.asarray(st.pos), st.quat_xyzw, sp, np.asarray(ph.pos), ph.quat_xyzw)
    return out


def demo(objects=None, seed=None):
    return {}
