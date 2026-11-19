"""On the station table, 26 cm out on the midline: a closed 12 oz drink can (6.6 cm across, 12.3 cm tall), standing
upright in a holder that keeps it from moving. Its lid panel is 4.6 mm below the rim. On the lid lies the stay-on
tab, flat, at its real size (25 x 15.7 mm, its curled edge 1.5 mm thick, 1.3 mm off the lid), its rivet in the middle of
the lid; the tab's tail points away from the arms and its nose lies over the scored opening, which is on the side
nearer the arms. Open the can.
"""

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys
from dataclasses import dataclass

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import mechanism as mech  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from yam_arena import demo as yam_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SILVER, LID_GREY, TAB_GREY, RED, DARK = (0.80, 0.81, 0.83), (0.70, 0.71, 0.73), (0.62, 0.63, 0.66), (0.75, 0.10, 0.12), \
    (0.15, 0.15, 0.16)
MATERIALS = {"default": (1.2, 1.0), "metal": (0.3, 0.2), "tab": (0.6, 0.5)}

# -- sizes, mm, in the can's frame: origin at the can's bottom centre on the table, x away from the arms -----------
# A US 12 fl oz (355 ml) can with a 202 end. Sources (all opened 2026-11-18):
# - body 2.600 in (66.04 mm) across, flanged height 4.812 in (122.2 mm): Ball's "202/211 x 413 12 oz industry standard"
#   drawing, https://www.cansource.com/assets/Uploads/Products/Documents/Ball-12oz-std-can-specs.pdf
# - 202 end: countersink depth 0.270 in, panel height 0.090 in, so the centre panel lies 0.180 in (4.6 mm) below the
#   end's top: Can Manufacturers Institute, https://www.cancentral.com/standards/ ; seamed, the end is 2.125 in (54 mm)
#   across (US 7,909,196 B2).
BODY_R, BODY_H = 33.0, 100.0
NECK_R = 28.6  # the neck, as a ring of posts so the can is hollow under the lid
RIM_IN, RIM_OUT, RIM_TOP = 25.5, 27.0, 122.7
PANEL_DEPTH = 4.6  # the lid's centre panel below the rim's top
LID_Z = RIM_TOP - PANEL_DEPTH  # the lid's top surface: 118.1
LID_T = 1.0
# The opening (the scored panel), on the -x side (towards the arms), hinged near the rivet.
# WHY the hinge 4 mm from the rivet: with it at 1.5 mm, the tab's underside pressed on the panel right by its hinge,
# where the panel can't go down, and the tab jammed at 4 degrees (official sim, 2026-11-13).
PANEL_X0, PANEL_X1, PANEL_HALF_W = -21.0, -4.0, 7.0  # panel from x -21 to -4, 14 mm wide
OPEN_GAP = 0.4  # the opening is this much bigger than the panel all round
# The tab, at the size of a real one: rivet at the lid's centre; nose over the panel (-x), tail at +x.
# - 0.985 in long and 0.620 in wide (25.0 x 15.7 mm): "a conventional tab" on a 202 end, US 2021/0347526 A1 (Stolle);
#   0.990 x 0.625 in in US 5,715,964.
# - its nose 0.305 to 0.317 in (7.7 to 8.1 mm) from the rivet's centre: US 8,646,643 B2 (Rexam), table 1. So the tail
#   ends 17 mm from the rivet.
# - its edge is the stock (0.28 mm) curled over, 0.058 to 0.065 in (about 1.5 mm) thick: from US 5,715,964 (the nose
#   curl 0.070 in, "8 to 20 % thicker than the thickest region of the pull ring"). No direct figure was found.
# - the tail's underside off the lid panel: no measured figure for a production end was found. US 7,909,196 B2 (Rexam)
#   gives the height of the lift end above the panel as "typically greater than 0.050 inches", "preferably 0.050 to
#   0.100 inches" (1.3 to 2.5 mm), without saying whether to its underside or its top; the low end, 1.3 mm (0.050 in),
#   is used here as the gap under the tail.
TAB_GAP, TAB_T = 1.3, 1.5  # the tab's underside above the lid, and its curled edge's thickness
CURL_R = TAB_T / 2  # the curled edge is round: tail and sides
TAB_NOSE_X, TAB_TAIL_X, TAB_HALF_W = -8.0, 17.0, 7.85
# The tab turns about the rivet 1.7 mm above the lid (as in the earlier, accepted mechanism, so its nose moves the same).
RIVET = (0.0, 0.0, LID_Z + 1.7)
BODY_X0 = PANEL_X1  # the flat body stops where the panel starts
# WHY the nose raised 3 mm over the panel, pressing only through its nub: a real tab's nose presses the panel at one
# point; flat over the panel, the body met the panel by its hinge as both swung down and stopped the tab at 18.6
# degrees (official sim, 2026-11-13).
NOSE_LIFT, NOSE_T = 3.0, 1.0  # the nose's underside above the lid, and its thickness
NUB_X0, NUB_X1 = -7.0, -5.8  # the nose's nub, down to 0.1 mm over the panel
TAIL_CORNER = (TAB_TAIL_X - CURL_R, TAB_HALF_W - CURL_R, LID_Z + TAB_GAP + CURL_R)  # the centre of the tail's +y corner
# The seal: a sprung catch under the panel's +y edge, beside where the tab's nose presses, overlapping the edge by
# 0.4 mm. Pushed down, the panel cams it aside and swings in; once past, it is free. The catch's spring sets the force
# the seal takes to break.
# WHY beside the nose, 0.4 mm over: under the panel's far end, 0.8 mm over, the nose (2-3 mm from the panel's hinge)
# had too little leverage and the round catch wedged: the arm could not lift the tail at all (official sim, 2026-11-13).
CATCH_X, CATCH_R, CATCH_LEN = -7.0, 1.5, 4.0
CATCH_OVERLAP = 0.4
CATCH_Y = PANEL_HALF_W + CATCH_R - CATCH_OVERLAP
CATCH_Z = LID_Z - LID_T - CATCH_R
CATCH_K, CATCH_PRELOAD, CATCH_TRAVEL = 100.0, 0.002, 0.004  # N/m, m, m [tuned in sim]

CAN_XY = (0.26, 0.0)
TAB_KG, PANEL_KG, CATCH_KG = 0.003, 0.003, 0.002  # real tab and panel are about 0.5 g; heavier here for stability


def _m(v):
    return tuple(x / 1000 for x in v)


def _box(c, s, color, material="metal"):
    return Shape("box", _m(c), _m(s), color, material=material, contact_offset=0.001)


def _cyl(c, r, length, axis, color, material="metal"):
    return Shape("cylinder", _m(c), (r / 1000, length / 1000), color, axis=axis, material=material,
                 contact_offset=0.001)


def _lid_strips():
    """The lid as 2 mm strips across x, each as wide as the lid's circle there, with the opening left out."""
    shapes, step, z = [], 2.0, LID_Z - LID_T / 2
    x = -RIM_IN
    while x < RIM_IN - 1e-9:
        cx = x + step / 2
        half = math.sqrt(max(RIM_IN ** 2 - max(abs(x), abs(x + step)) ** 2, 0.0))
        lo, hi = PANEL_X0 - OPEN_GAP, PANEL_X1
        if half > 0.5:
            if lo <= cx <= hi:  # beside the opening: two pieces
                w = PANEL_HALF_W + OPEN_GAP
                if half > w:
                    for sy in (1, -1):
                        shapes.append(_box((cx, sy * (w + half) / 2, z), (step, half - w, LID_T), LID_GREY))
            else:
                shapes.append(_box((cx, 0.0, z), (step, 2 * half, LID_T), LID_GREY))
        x += step
    return shapes


def _ring(r, post_r, z0, z1, n, color):
    """A wall of upright posts round a circle (boxes can't be turned within a link, so a round wall is posts)."""
    return [_cyl((r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), (z0 + z1) / 2), post_r, z1 - z0,
                 "Z", color) for i in range(n)]


def _can_base():
    lid_bottom = LID_Z - LID_T
    return tuple([
        _cyl((0, 0, BODY_H / 2), BODY_R, BODY_H, "Z", RED),
        _cyl((0, 0, BODY_H + 0.25), BODY_R - 1.0, 0.5, "Z", DARK),  # seen through the opening, inside the can
        # posts touching one another, so the neck and rim read as solid walls
        *_ring(NECK_R, 0.95, BODY_H, lid_bottom, 96, SILVER),
        *_ring((RIM_IN + RIM_OUT) / 2, 0.75, lid_bottom, RIM_TOP, 128, SILVER),
        *_lid_strips(),
    ])


def _panel():
    length = PANEL_X1 - PANEL_X0
    return (_box(((PANEL_X0 + PANEL_X1) / 2, 0, LID_Z - LID_T / 2), (length, 2 * PANEL_HALF_W, LID_T), LID_GREY),)


def _capsule(c, r, straight, axis, color, material):
    return Shape("capsule", _m(c), (r / 1000, straight / 1000), color, axis=axis, material=material,
                 contact_offset=0.001)


def _tab():
    x0, x1, y1, zc = BODY_X0, TAB_TAIL_X - CURL_R, TAB_HALF_W - CURL_R, LID_Z + TAB_GAP + TAB_T / 2
    nose_top = NOSE_LIFT + NOSE_T
    return (
        _box(((x0 + x1) / 2, 0, zc), (x1 - x0, 2 * y1, TAB_T), TAB_GREY, "tab"),
        # the nose, bent up over the panel, and its nub pressing down onto it
        _box(((TAB_NOSE_X + BODY_X0) / 2, 0, LID_Z + NOSE_LIFT + NOSE_T / 2), (BODY_X0 - TAB_NOSE_X, 8.0, NOSE_T), TAB_GREY),
        _box((BODY_X0 - 0.25, 0, LID_Z + (TAB_GAP + nose_top) / 2), (0.5, 8.0, nose_top - TAB_GAP), TAB_GREY),
        _box(((NUB_X0 + NUB_X1) / 2, 0, LID_Z + (0.1 + NOSE_LIFT) / 2), (NUB_X1 - NUB_X0, 6.0, NOSE_LIFT - 0.1), TAB_GREY),
        # the curled edge, round, 1.5 mm thick: along the tail's end and down both sides
        _capsule((x1, 0, zc), CURL_R, 2 * y1, "Y", TAB_GREY, "tab"),
        *[_capsule(((x0 + CURL_R + x1) / 2, sy * y1, zc), CURL_R, x1 - x0 - CURL_R, "X", TAB_GREY, "tab") for sy in (1, -1)],
    )


def _catch():
    return (_cyl((CATCH_X, CATCH_Y, CATCH_Z), CATCH_R, CATCH_LEN, "X", LID_GREY),)


TAB_MAX = math.radians(120)
PANEL_MAX = math.radians(100)
CAN = mechanism(
    "can", (*CAN_XY, TOP), base=Link("can_body", 0.40, _can_base()),
    links=(Link("tab", TAB_KG, _tab()), Link("panel", PANEL_KG, _panel()), Link("catch", CATCH_KG, _catch())),
    joints=(
        # - lifts the tab's tail (+x) about the rivet; its nose (-x) then goes down onto the panel. No spring: a
        # real tab stays where it is bent to; friction holds it there.
        Joint("tab_hinge", "revolute", "tab", _m(RIVET), "Y", -TAB_MAX, 0.0, damping=0.0005, friction=0.002),
        # - swings the panel down into the can about its hinge by the rivet. Friction keeps it where it is pushed,
        # as the real panel stays folded in.
        Joint("panel_hinge", "revolute", "panel", _m((PANEL_X1, 0, LID_Z - LID_T / 2)), "Y", -PANEL_MAX, 0.0,
              damping=0.0005, friction=0.002),
        # + moves the catch out from under the panel; preloaded against its stop at 0
        Joint("catch_slide", "prismatic", "catch", _m((CATCH_X, CATCH_Y, CATCH_Z)), "Y", 0.0, CATCH_TRAVEL,
              stiffness=CATCH_K, damping=0.5, target=-CATCH_PRELOAD),
    ),
    materials=MATERIALS)
OBJECTS = [CAN]


# -- the grippers, for scoring: full credit needs the hands off the can ----------------------------------------------
@dataclass(frozen=True)
class Pads:
    """One gripper after a control step (world frame, m): the span along its pointing axis from the pads' back to
    their tips, and points across the end of each of its two fingertips."""

    back: np.ndarray
    tips: np.ndarray
    fingertips: tuple = ()


PAD_BACK = 0.025  # the pads reach this far behind the TCP
GRIPPERS = ("left_gripper", "right_gripper")
# The end of each fingertip in its own link's frame (m), at its middle and 6 mm either side, from the gripper's meshes.
FINGERTIP_ENDS = {
    "tip_left": [(x, -0.04454, -0.09016) for x in (0.01799, 0.02399, 0.02999)],
    "tip_right": [(x, 0.04454, -0.09016) for x in (-0.02999, -0.02399, -0.01799)],
}


def _gripper_states(env) -> dict:
    """Each gripper's ``Pads``, keyed ``left_gripper`` and ``right_gripper`` (as in the accepted flashlight tasks)."""
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = runtime.link_poses(robot)
        axis = poses["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        ends = tuple(poses[link][:3, :3] @ np.array(p) + poses[link][:3, 3] - origin
                     for link, points in FINGERTIP_ENDS.items() for p in points)
        out[f"{side}_gripper"] = Pads(back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, fingertips=ends)
    return out


def _patch_scene(scene) -> None:
    if getattr(scene.object_states, "_with_grippers", False):
        return
    stock = scene.object_states

    def object_states(env, names):
        """The stock object states (the can's from mechanism.py), plus the two grippers' pads for this file's scoring."""
        return {**stock(env, names), **_gripper_states(env)}

    object_states._with_grippers = True
    scene.object_states = object_states


class _AfterSceneImport(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` right after it is first imported (it needs Isaac Sim, which starts later). Put ahead
    of mechanism.py's finder, so this patch wraps that one and the grippers aren't filtered out."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        run = spec.loader.exec_module

        def exec_module(module):
            run(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _AfterSceneImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _AfterSceneImport())

# -- scoring ----------------------------------------------------------------------------------------------------------
OPEN_DEG = 45.0  # the panel pushed in at least this far: the can is open
# WHY the tab only 10 degrees: it shows the tab opened the can (a panel poked in with a finger leaves the tab flat at
# 0). Lifting the tail turns the panel about 4 degrees per degree of tab, so the panel is fully in with the tab at
# about 20 degrees, and its nose then meets the panel by the hinge (official sim, 2026-11-13: tab 22, panel 82).
TAB_UP_DEG = 10.0
SEAL_BROKEN_DEG = 10.0  # the panel in this far: the seal has gone
TAB_STARTED_DEG = 3.0
TAB_LIFTED_DEG = 10.0


def angles(objects) -> tuple[float, float]:
    """How far the tab's tail is raised and the panel pushed in (degrees, both positive)."""
    joints = objects["can"].joints
    return -math.degrees(joints["tab_hinge"]), -math.degrees(joints["panel_hinge"])


# Hands off: no point along either gripper's pads, and neither fingertip's end, within 1.5 cm of the can's bounding
# box (rim included).
CLEAR = 0.015
COMPLETE_SCORE = 0.9  # score() for the open can: 1.0 comes only from success() holding for 1 s, which ends the trial
# WHY the tab's highest angle this trial, not its angle now: people often fold the tab back down once the can is
# open, and the demo does, so the opening is uncovered; the tab still has to have been raised to open it. Each reset
# (a fresh can) clears it.
# WHY a fingertip on the opening's side before the seal breaks scores 0: pressing the tab's nose (or the panel) down
# from above also turns the tab and pushes the panel in, without the tail ever being lifted by a finger. The can has to
# be opened from the tab's tail.
_RECORD = {"tab_max": 0.0, "pressed": False}


@mech.on_reset
def _new_trial():
    _RECORD["tab_max"] = 0.0
    _RECORD["pressed"] = False


def on_opening_side(objects) -> bool:
    """A fingertip's end inside the rim, below the rim's top, on the opening's side of the rivet (the nose and panel)."""
    can = np.asarray(objects["can"].pos, float)
    for g in GRIPPERS:
        for p in getattr(objects.get(g), "fingertips", ()):
            d = np.asarray(p, float) - can
            if d[0] < 0.0 and math.hypot(d[0], d[1]) < RIM_IN / 1000 and d[2] < RIM_TOP / 1000:
                return True
    return False


def _touches(pads: Pads, part) -> bool:
    lo, hi = np.asarray(part.low, float) - CLEAR, np.asarray(part.high, float) + CLEAR
    along = [pads.back + s * (pads.tips - pads.back) for s in np.linspace(0.0, 1.0, 11)]
    return any(bool(np.all(p >= lo) and np.all(p <= hi)) for p in (*along, *(np.asarray(q, float) for q in pads.fingertips)))


def hands_off(objects) -> bool:
    """Both grippers clear of the can. Without the grippers' state it can't be shown, so it doesn't count."""
    pads = [objects.get(g) for g in GRIPPERS]
    if any(p is None for p in pads):
        return False
    return not any(_touches(p, objects["can"]) for p in pads)


def _grade(objects) -> float:
    """1.0 for the panel pushed in at least 45 degrees, the tab having been raised at least 10 degrees this trial (it
    was the tab that opened it), and the hands off the can; 0.75 for that with a hand still on the can; 0.5 if the
    seal is broken (the panel in at least 10 degrees) with the tab raised at least 3 degrees; 0.25 if the tab has been
    raised at least 10 degrees but the seal holds; 0 otherwise, including a panel pushed in with the tab never raised,
    and a trial in which a fingertip went below the rim on the opening's side of the rivet while the seal held."""
    tab, panel = angles(objects)
    _RECORD["tab_max"] = max(_RECORD["tab_max"], tab)
    raised = _RECORD["tab_max"]
    if panel < SEAL_BROKEN_DEG and on_opening_side(objects):
        _RECORD["pressed"] = True
    if _RECORD["pressed"]:
        return 0.0
    if panel >= OPEN_DEG and raised >= TAB_UP_DEG:
        return 1.0 if hands_off(objects) else 0.75
    if panel >= SEAL_BROKEN_DEG and raised >= TAB_STARTED_DEG:
        return 0.5
    if raised >= TAB_LIFTED_DEG and panel < SEAL_BROKEN_DEG:
        return 0.25
    return 0.0


def _debug(objects, grade) -> None:
    """With CAN_DEBUG=path set, one line per control step: the angles, the grade, and the left hand's two fingertips."""
    path = os.environ.get("CAN_DEBUG")
    if not path:
        return
    tab, panel = angles(objects)
    can = np.asarray(objects["can"].pos, float) + np.array([0.0, 0.0, LID_Z / 1000])
    tips = getattr(objects.get("left_gripper"), "fingertips", ())[1::3]
    where = " ".join("(%.1f,%.1f,%.1f)" % tuple((np.asarray(p, float) - can) * 1000) for p in tips)
    with open(path, "a") as f:
        f.write(f"tab={tab:.2f} panel={panel:.2f} raised={_RECORD['tab_max']:.2f} pressed={int(_RECORD['pressed'])} "
                f"grade={grade} catch_mm={objects['can'].joints.get('catch_slide', 0.0) * 1000:.2f} "
                f"left fingertips, mm from the lid's centre={where}\n")


def score(objects) -> float:
    """0.9 for the can opened with its tab, hands off it (1.0 once success() has held 1 s); 0.75 opened with a hand
    still on the can; 0.5 seal broken with the tab started; 0.25 tab raised but the seal holding; 0 otherwise."""
    grade = _grade(objects)
    _debug(objects, grade)
    return min(grade, COMPLETE_SCORE)


def success(objects) -> bool:
    """The panel pushed in at least 45 degrees, the tab raised at least 10 degrees at some point this trial, and both
    hands off the can. Must hold for 1 s."""
    return _grade(objects) == 1.0


# -- the scripted demo (left arm) -------------------------------------------------------------------------------------
# A real tab's tail is too thin and too close to the lid to pinch: a person slides a fingertip's edge under it and
# levers it up, and so does the demo. Measured on the gripper's meshes: each fingertip ends in a tongue 15.5 mm wide
# and 3.95 mm thick, its end rounded on the outside and square (about 0.25 mm) where it meets the flat pad. Leaning
# the hand 15 degrees puts that square pad edge 0.55 mm above the lid when the tongue's end rests on it, under the
# tail's round edge (its underside 1.3 mm up), with the pad facing up at the tail like a ramp.
CX, CY = CAN_XY
CAN_W = np.array([CX, CY, TOP])
LID_W = TOP + LID_Z / 1000
RIVET_W = CAN_W + np.array(RIVET) / 1000
ACROSS_X = -math.pi / 2  # straight down, fingers closing along x (turned the left arm's easy way)
LEAN, TURN = math.radians(15.0), math.radians(-15.0)
_Z = np.array([0.0, 0.0, 1.0])
_FACING = np.array([math.sin(TURN), -math.cos(TURN), 0.0])  # where the lever finger's pad looks (level part): at the tail's +y corner
_EDGE = np.array([math.cos(TURN), math.sin(TURN), 0.0])  # along the pad's end edge
_PAD = _FACING * math.cos(LEAN) + _Z * math.sin(LEAN)  # the pad's normal: at the tail and 15 degrees up
_FINGER = -_FACING * math.sin(LEAN) + _Z * math.cos(LEAN)  # up the finger, away from its end
# The hand leaning 15 degrees (its tips towards -y, the left arm's easy way) and turned 15 degrees, fingers wide open:
# the lower finger is the one on the +y side, and only it goes inside the rim; the other stays 2 cm above the can.
# WHY turned: square to the tab's side, the pad would meet the side edge near the rivet first, where there is no
# leverage; turned, it meets the tail's corner, 17 mm from the rivet.
LEVER_GRIP = axis_angle_matrix(_Z, TURN) @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -LEAN) @ yam_demo.grip_rotation(0.0)
TIP_DROP = 0.00055  # the tongue's lowest point below its pad edge at this lean (from the mesh)
ALONG = 0.0055  # the tail's corner meets the pad this far from the tongue's middle, towards its +x end (2 mm from that end)
BACK = 0.0012  # the fingertip comes down this far short of the tail (and 1 mm inside the rim)
PRESS = 0.0005  # aimed this far into the lid, so the fingertip rests on it
# WHY 8 mm: aimed 3 mm past, the arm (which gives) pressed too lightly: the pad slid up off the tail's corner with the
# tab at 1.8 degrees and the seal holding (official sim, 2026-11-18).
PUSH = 0.008  # and this far past first touching the tail, so the pad stays against it
LIFT_MM = (3, 6, 9)  # then up: the tail's corner rises 6 mm for the tab's 22 degrees
HOLD_S = 1.0
DEMO_AIM_SHIFT_MM = 0.0  # the lever aimed this far off (+x and +y), for margin runs
HOVER_ABOVE_RIM = 0.0437  # TCP this far over the rim: the pad tips 3.1 cm over it, clear of the can (2 cm more is out of reach)
FOLD_X = 0.0125  # the closed fingertips press the tail down here, from the rivet


def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle, for the leaning hand (yam_arena.demo
    builds each waypoint's rotation with grip_rotation(yaw), which this wraps)."""
    if getattr(yam_demo, "_matrix_grips", False):
        return
    plain = yam_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    yam_demo.grip_rotation = grip_rotation
    yam_demo._matrix_grips = True


_allow_matrix_grips()


def pad_edge(back=0.0, lift=0.0) -> np.ndarray:
    """The middle of the lever finger's pad edge (world, m), its tongue resting on the lid and its pad ``back`` short
    of touching the tail's +y corner, then raised by ``lift``."""
    corner = CAN_W + np.array(TAIL_CORNER) / 1000
    touching = corner - _PAD * CURL_R / 1000 - _EDGE * ALONG
    edge = touching + _FINGER * ((LID_W + TIP_DROP - PRESS - touching[2]) / _FINGER[2])  # down the pad's own plane
    shift = DEMO_AIM_SHIFT_MM / 1000
    return edge - _FACING * back + np.array([shift, shift, lift])


def lever_tcp(back=0.0, lift=0.0):
    """TCP for that: the lever finger is the open hand's lower one, 47 mm from the TCP's axis."""
    return tuple(pad_edge(back, lift) + PAD_TIP_PAST_TCP * _FINGER + abs(FINGER_OPEN) * _PAD)


def fold_tcp(above=0.0):
    """TCP with the closed fingertips flat on the tab's tail, the tab lying flat (aimed 1 mm lower, to press)."""
    return (CX + FOLD_X, CY, LID_W + (TAB_GAP + TAB_T) / 1000 + PAD_TIP_PAST_TCP - 0.001 + above)


def hover_tcp():
    return (CX, CY, TOP + RIM_TOP / 1000 + HOVER_ABOVE_RIM)


def _poke_plan():
    """A wrong way, for checking the score (CAN_DEMO=poke): the closed fingertips (15.5 mm long along x) push straight
    down on the panel and on the tab's nose over it, and the tail is never lifted by a finger."""
    over = (CX - 0.0132, CY, LID_W + PAD_TIP_PAST_TCP)
    return [
        Waypoint(2.0, (over[0], over[1], over[2] + 0.03), FINGER_CLOSED, "over the panel"),
        Waypoint(2.0, (over[0], over[1], over[2] - 0.008), FINGER_CLOSED, "push the panel and the nose in"),
        Waypoint(1.0, (over[0], over[1], over[2] + 0.03), FINGER_CLOSED, "up"),
        Waypoint(1.5, hover_tcp(), FINGER_OPEN, "over the lid", ACROSS_X),
        Waypoint(3.0, hover_tcp(), FINGER_OPEN, "stay", ACROSS_X),
    ]


def demo():
    right = [Waypoint(1.0, (0.22, -0.30, TOP + 0.15), FINGER_OPEN, "stay clear")]
    if os.environ.get("CAN_DEMO") == "poke":
        return {"left": _poke_plan(), "right": right}
    left = [
        Waypoint(2.0, lever_tcp(BACK, 0.025), FINGER_OPEN, "over the tail's corner, the hand leaning", LEVER_GRIP),
        Waypoint(1.5, lever_tcp(BACK), FINGER_OPEN, "one fingertip down onto the lid beside the tail", LEVER_GRIP),
        Waypoint(1.0, lever_tcp(-PUSH), FINGER_OPEN, "slide its edge under the tail", LEVER_GRIP),
        *[Waypoint(0.7, lever_tcp(-PUSH, mm / 1000), FINGER_OPEN, f"lever the tail up {mm} mm", LEVER_GRIP)
          for mm in LIFT_MM],
        Waypoint(HOLD_S, lever_tcp(-PUSH, LIFT_MM[-1] / 1000), FINGER_OPEN, "keep levering", LEVER_GRIP),
        Waypoint(0.6, lever_tcp(0.003, 0.012), FINGER_OPEN, "off the tail", LEVER_GRIP),
        # the tab is folded back down, as people often do, so the open panel is uncovered for the cameras
        Waypoint(1.5, fold_tcp(0.012), FINGER_CLOSED, "over the raised tail, fingers closed", TURN),
        Waypoint(1.2, fold_tcp(), FINGER_CLOSED, "fold the tab back down", TURN),
        Waypoint(0.6, fold_tcp(0.012), FINGER_CLOSED, "up", TURN),
        Waypoint(1.5, hover_tcp(), FINGER_OPEN, "over the lid, looking at the opening", ACROSS_X),
        Waypoint(3.0, hover_tcp(), FINGER_OPEN, "stay", ACROSS_X),
    ]
    return {"left": left, "right": right}
