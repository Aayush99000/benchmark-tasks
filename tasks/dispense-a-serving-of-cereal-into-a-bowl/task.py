"""A twist-knob cereal dispenser of the kind found at hotel breakfast buffets and in dining halls is screwed to an
upright board standing on the counter, its spout 80 mm above the counter and its knob facing the right arm. Its clear
canister holds about 78 g of toasted oat rings (760 true-size pieces) above a six-vane paddle wheel; turning the knob
turns the wheel, and each compartment that comes round empties down the spout. An empty 6 1/2 in plastic bowl stands on
the counter beside the spout, not under it. The job: one serving (39 g, the box's 1 1/2 cups) in the bowl, none on the
counter.

Run it:  inspect-robots run --embodiment yam_cereal -E task=task.py -E object_jitter=0 --policy yam_scripted --no-prompt \\
           --instruction "<INSTRUCTION>" --scorer task_score

`yam_cereal` (cereal_dispenser/embodiment.py) spawns the dispenser, its wheel and the cereal, and the bowl. The wheel,
its soft vane tips, every piece and the bowl are rigid bodies in real contact; the only way cereal leaves the canister
is through the wheel as the knob turns it.

Scored at the end of the trial, from the pieces' simulated positions:
- in the bowl: a piece whose centre lies inside the bowl (within its inner wall, above its floor, below its rim);
- spilled: a piece neither in the bowl nor inside the dispenser (canister, funnel, chamber or spout).
Mass in the bowl is the count times the piece mass (0.103 g).
"""

import math
import os

import numpy as np

from cereal_dispenser import geometry as G

try:
    from yam_arena.task_file import TABLE_TOP, Box
except ImportError:  # the scoring criteria and their tests need no simulator packages
    from dataclasses import dataclass

    TABLE_TOP = {"station": 0.7475}

    @dataclass(frozen=True)
    class Box:
        name: str
        pos: tuple
        size: tuple = (0.04, 0.04, 0.04)
        mass: float | None = None
        yaw: float = 0.0


SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Get one serving of cereal from the dispenser into the bowl, that is 1 1/2 cups or 39 grams, without spilling any on the counter."
VARIANT = os.environ.get("CEREAL_DEMO", "")  # "", "none", "nobowl", "under", "over": the demo and its shortcut variants

# -- layout (world: x forward from the arms, y left, z up) -------------------------------------------------------------
RIGHT_BASE = np.array((0.0, -0.305, 0.75))
LEFT_BASE = np.array((0.0, 0.305, 0.75))
HEADING = math.radians(50.0)  # the knob's axis, from the right arm's base into the dispenser
APPROACH = np.array((math.cos(HEADING), math.sin(HEADING), 0.0))
KNOB_REACH = 0.40  # the grasp point on the knob's grip bar, horizontally from the right arm's base axis
GRASP_X = 0.072  # the grasp point on the bar, in the dispenser frame: finger pads clear of the knob's face
GRASP = np.array((*(RIGHT_BASE[:2] + KNOB_REACH * APPROACH[:2]), TOP + G.WHEEL_Z))
D_XY = GRASP[:2] + GRASP_X * APPROACH[:2]
D_YAW = HEADING + math.pi  # the dispenser's +x (its front) points back at the right arm's base
X_D = -APPROACH
Y_D = np.array((-X_D[1], X_D[0], 0.0))
BOWL_SPOT = 0.022  # the bowl's centre under the spout, along the dispenser's +x from the wheel axis
BOWL_TARGET = np.array((*(D_XY + BOWL_SPOT * X_D[:2]), TOP))
PUSH = 0.10  # the bowl starts this far to the dispenser's right (seen from its front), beside the spout
BOWL_START = BOWL_TARGET - PUSH * Y_D
if os.environ.get("CEREAL_BOWL_START") == "spout":  # dev only: the bowl starts under the spout
    BOWL_START = BOWL_TARGET.copy()

OBJECTS = [
    Box("dispenser", pos=(*D_XY, TOP), size=(0.01, 0.01, 0.01), yaw=D_YAW),
    Box("bowl", pos=tuple(BOWL_START + (0.0, 0.0, 0.0005)), size=(0.01, 0.01, 0.01), mass=G.BOWL["mass"]),
]

# -- criteria -------------------------------------------------------------------------------------------------------------
SERVING_LO, SERVING_HI = 31.0, 47.0  # g: one labelled serving (39 g) within 20 %
HALF_G = 20.0  # g: about half a serving
SPILL_FULL = 5  # pieces on the counter allowed at full score (about 0.5 g)
SPILL_CAP = 15  # more than this caps the score at 0.25
TILT_MAX = math.radians(20.0)  # bowl upright
CLEAR = 0.05  # m: each fingertip midpoint outside the bowl's rim cylinder by this much
LIN, STILL_S = 0.02, 1.0  # the bowl under 2 cm/s for the last second ...
PIECE_LIN, PIECE_MOVING_MAX = 0.05, 2  # ... and no more than 2 pieces in it moving faster than 5 cm/s
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _fresh():
    return {"still": 0.0, "start_in_dispenser": None}


_trial = _fresh()


def trial():
    return _trial


def reset_trial():
    """Forget the trial; the embodiment calls this at every reset."""
    _trial.clear()
    _trial.update(_fresh())


def _rot(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def to_dispenser(points):
    """World points (n, 3) in the dispenser frame."""
    p = np.atleast_2d(np.asarray(points, float)) - np.array((*D_XY, TOP))
    return np.c_[p @ X_D, p @ Y_D, p[:, 2]]


def in_dispenser(points):
    """Pieces still inside the dispenser: above the spout's mouth and within its outline."""
    d = to_dispenser(points)
    return (d[:, 2] > G.SPOUT_Z - 0.002) & (np.abs(d[:, 0]) < 0.08) & (np.abs(d[:, 1]) < 0.08)


def bowl_tilt(bowl):
    return math.acos(max(-1.0, min(1.0, _rot(bowl.quat_xyzw)[2, 2])))


def in_bowl(objects, points):
    """Pieces whose centre lies inside the bowl: within its inner wall, above its floor, no higher than its rim."""
    bowl = objects["bowl"]
    p = (np.atleast_2d(np.asarray(points, float)) - np.asarray(bowl.pos, float)) @ _rot(bowl.quat_xyzw)
    r = np.hypot(p[:, 0], p[:, 1])
    z = p[:, 2]
    rin = np.interp(z, G.BOWL_PROFILE[:, 1], G.BOWL_PROFILE[:, 0])
    return (z > G.BOWL_FLOOR_Z - 0.001) & (z < G.BOWL["h"] + 0.004) & (r < rin + 0.0015)


def counts(objects):
    pos = objects["cereal"].pos
    inside = in_bowl(objects, pos)
    disp = in_dispenser(pos) & ~inside
    spilled = ~inside & ~disp
    return {"bowl": int(inside.sum()), "grams": float(inside.sum() * G.PIECE_MASS * 1000), "spilled": int(spilled.sum()),
            "dispenser": int(disp.sum())}


def hands_clear(objects):
    bowl = objects["bowl"]
    for arm in ("left", "right"):
        p = np.asarray(objects[f"{arm}_tcp"].pos, float) - np.asarray(bowl.pos, float)
        if math.hypot(p[0], p[1]) < G.BOWL["r_rim"] + CLEAR and p[2] < G.BOWL["h"] + 0.10:
            return False
    return True


def note_start(objects):
    _trial["start_in_dispenser"] = int(in_dispenser(objects["cereal"].pos).sum())


def note_step(objects, dt):
    """Accumulate how long the bowl and the cereal in it have been at rest."""
    bowl = objects["bowl"]
    pos, vel = objects["cereal"].pos, objects["cereal"].vel
    inside = in_bowl(objects, pos)
    fast = int((np.linalg.norm(vel[inside], axis=1) > PIECE_LIN).sum()) if inside.any() else 0
    moving = np.linalg.norm(bowl.lin_vel) > LIN or fast > PIECE_MOVING_MAX
    _trial["still"] = 0.0 if moving else _trial["still"] + dt


def score(objects):
    c = counts(objects)
    if c["bowl"] == 0:
        return 0.0
    if c["spilled"] > SPILL_CAP or bowl_tilt(objects["bowl"]) > TILT_MAX:
        return 0.25
    if c["grams"] < HALF_G:
        return 0.25
    if not (SERVING_LO <= c["grams"] <= SERVING_HI):
        return 0.5  # at least half a serving, but short of or over the serving
    finished = c["spilled"] <= SPILL_FULL and hands_clear(objects) and _trial["still"] >= STILL_S
    return 1.0 if finished else 0.75


def success(objects):
    return score(objects) >= 1.0


def debug_line(objects):
    c = counts(objects)
    w = objects["wheel"]
    return (f"wheel {math.degrees(w.angle):7.1f} deg {math.degrees(w.rate):6.1f} deg/s  bowl {c['bowl']:3d} ({c['grams']:5.1f} g)  spilled {c['spilled']:3d}  "
            f"in dispenser {c['dispenser']:3d}  tilt {math.degrees(bowl_tilt(objects['bowl'])):4.1f}  still {_trial['still']:.2f}  "
            f"clear {int(hands_clear(objects))}  score {score(objects):.2f}")


# -- scripted demo (open loop, from the fixed layout only) ---------------------------------------------------------------
DT = 0.05
PUSH_Z = 0.030  # m above the bowl's base where the fingertips push
PUSH_SPREAD = 0.050  # m between the two fingertips
PUSH_OPEN = 0.55  # gripper opening for that spread
PAD_HALF = 0.008  # m from the fingertip midpoint to a pad's outer face
PUSH_LEAN_DEG = 35.0
TURN_DEG = {"under": 200.0, "over": 760.0}.get(VARIANT, 485.0)  # the knob turn the demo makes (shortcuts: too little, too much)
TURN_TOTAL = math.radians(float(os.environ.get("CEREAL_TURN_DEG", TURN_DEG)))


def demo():
    from cereal_dispenser.plan import Arm, Builder, frame

    hold_s = float(os.environ.get("CEREAL_HOLD_S", "0"))
    if VARIANT == "none" or hold_s:
        out = {}
        for arm, base in (("left", LEFT_BASE), ("right", RIGHT_BASE)):
            b = Builder(Arm(base), DT)
            b.hold(hold_s or 20.0)
            out[arm] = [b.plan()]
        return out
    return {"left": [_left_plan()], "right": [_right_plan()]}


def _left_plan():
    """Slide the bowl along the counter, with both fingertips against its side, until it stands under the spout;
    then lift the hand clear and go home."""
    from cereal_dispenser.plan import Arm, Builder, frame

    arm = Arm(LEFT_BASE)
    b = Builder(arm, DT)
    if VARIANT == "nobowl":
        b.hold(30.0)
        return b.plan()
    lean = math.radians(PUSH_LEAN_DEG)  # the hand leans back, so the wrist stays clear of the canister
    approach = -math.cos(lean) * np.array((0.0, 0.0, 1.0)) + math.sin(lean) * Y_D
    rot = frame(approach, X_D)  # the fingers spread across the push
    contact = math.sqrt((G.bowl_inner_radius(PUSH_Z) + G.BOWL["t"]) ** 2 - (PUSH_SPREAD / 2) ** 2) + PAD_HALF
    tip_to_tcp = -0.013 * approach  # the pads' tips are 13 mm past the fingertip midpoint along the approach
    start = BOWL_START - (contact + 0.025) * Y_D + (0.0, 0.0, PUSH_Z) + tip_to_tcp
    end = BOWL_TARGET - contact * Y_D + (0.0, 0.0, PUSH_Z) + tip_to_tcp
    above = start + (0.0, 0.0, 0.08)
    b.gripper(PUSH_OPEN, 0.3)
    b.joints(arm.solve(above, rot), 2.5, "reach")
    b.line(above, start, rot, 1.2, "down")
    b.line(start, end, rot, 3.5, "slide")
    b.hold(0.4)
    b.line(end, end - 0.03 * Y_D, rot, 0.6, "back off")
    b.line(end - 0.03 * Y_D, end - 0.03 * Y_D + (0.0, 0.0, 0.10), rot, 1.0, "up")
    b.line(end - 0.03 * Y_D + (0.0, 0.0, 0.10), end - 0.13 * Y_D + (0.0, 0.0, 0.12), rot, 1.2, "away")
    b.joints(np.array((0.0, 1.0, 1.0, -0.6, 0.0, 0.0)), 2.0, "home")
    b.hold(1.0)
    return b.plan()


def _right_plan():
    """Wait for the bowl, then take the knob's grip bar and turn it TURN_TOTAL in strokes of the wrist, regripping
    between strokes, and back away."""
    from cereal_dispenser.plan import Arm, Builder, frame

    arm = Arm(RIGHT_BASE)
    b = Builder(arm, DT)
    b.hold(0.5 if VARIANT == "nobowl" else 9.0)
    rot = frame(APPROACH, (0.0, 0.0, 1.0))  # fingers above and below the bar, which lies level at the start
    back = GRASP - 0.035 * APPROACH
    seed = np.array((HEADING + 0.0, 1.7, 0.25, 1.4, 0.0, 0.0))
    q = arm.solve(back, rot, seed=seed)
    # use the equivalent grip (fingers swapped) that leaves the most wrist travel for the first stroke
    lo, hi = -2.0944 + 0.06, 2.0944 - 0.06
    if q[5] > 0:
        q2 = arm.solve(back, rot @ np.diag((-1.0, -1.0, 1.0)), seed=np.r_[q[:5], q[5] - math.pi])
        q = q2
        rot = rot @ np.diag((-1.0, -1.0, 1.0))
    b.gripper(0.45, 0.3)
    b.joints(q, 3.0, "reach")
    remaining = TURN_TOTAL
    while remaining > 1e-6:
        b.line(back, GRASP, rot, 0.8, "in")
        b.gripper(0.0, 0.5, "grip")
        b.hold(0.2)
        stroke = min(remaining, hi - b.cur[5])
        b.wrist_roll(stroke, max(1.0, stroke / math.radians(80.0)), "turn")
        rot = _roll(rot, stroke)
        remaining -= stroke
        b.hold(0.3)
        b.gripper(0.45, 0.4, "open")
        b.line(GRASP, back, rot, 0.6, "out")
        if remaining > 1e-6:  # the bar looks the same every half turn: roll back half a turn and take it again
            b.wrist_roll(-math.pi, 1.6, "regrip")
            rot = _roll(rot, -math.pi)
    b.joints(np.array((0.0, 1.0, 1.0, -0.6, 0.0, 0.0)), 2.5, "home")
    b.hold(3.0)
    return b.plan()


def _roll(rot, angle):
    """The gripper rotation after turning joint 6 by ``angle`` (about the approach axis)."""
    c, s = math.cos(angle), math.sin(angle)
    return rot @ np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
