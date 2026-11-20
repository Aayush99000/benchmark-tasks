"""On the station table, between the arms: an ATX motherboard (305 x 244 mm) lying flat, its CPU socket straight
ahead between the arms, a processor already in it. The socket's retention frame has a lug at each end (its left and
right ends, each towards one arm). On the left, an AMD Wraith Prism cooler (105 x 105 x 93 mm, 0.545 kg: aluminium fins,
four copper heat pipes, a 92 mm fan in a black shroud) stands on a holed foam support, its pasted base clear. Its steel clip ends in a strap with a
window at each end: the left one fixed, the right one loose, hanging from a black cam lever that lies along the
shroud's side, unlocked. On the right, a precision flat-head screwdriver stands tip down in a rack.

Scene, scoring and scripted demo (see README.md).
Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402

import clip  # noqa: E402
import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Install the CPU cooler on the processor. Once its thermal paste touches the processor, don't lift it "
               "off again or slide it more than 10 mm. Hook the fixed end of its clip over the lug first, then use "
               "the screwdriver to press the lever down until the loose end hooks over its lug, and swing the lever "
               "over the top until it locks.")

MM = 0.001
TABLE = 0.7475
GREY = (0.45, 0.47, 0.50)
FOAM = (0.20, 0.21, 0.22)

# -- the board (assets/board.usda: origin at the board's centre, bottom face) -----------------------------------
BOARD_YAW = 0.0  # board +y along world +y: the clip runs left-right, each tab towards one arm
SOCKET = (8 * MM, 64 * MM)  # the socket's centre, board frame
CPU_TOP = 9.7 * MM  # the processor's top above the board's bottom face
BOARD_POS = np.array([0.290 - SOCKET[0], -SOCKET[1], TABLE])  # the socket at world (0.29, 0)


def yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def board_to_world(p) -> np.ndarray:
    c, s = math.cos(BOARD_YAW), math.sin(BOARD_YAW)
    x, y, z = p
    return BOARD_POS + np.array([c * x - s * y, s * x + c * y, z])


# -- the cooler (assets/cooler.usda, origin at the base's bottom centre, z up, its clip across its y, the lever on its
# +y side; work/cooler/make_cooler.py). On the CPU it is turned 180 degrees from the board, its lever towards the right
# arm (world -y).
COOLER_YAW = BOARD_YAW + math.pi
SEATED_POS = board_to_world((*SOCKET, CPU_TOP))
COOLER_MASS = 0.545
COVER = (105 * MM, 105 * MM, (58 * MM, 93 * MM))  # fan shroud: x, y, z range
FAN_R = 46 * MM  # the shroud's round opening
FAN_TOP = 10 * MM  # the fan stays this far below the shroud's top
CLIP_Y = 50.5 * MM  # each leg's plane, from the cooler's centre
CLIP_T = 1.2 * MM
LEVER_PIVOT = np.array([0.0, 57.0 * MM, 66.0 * MM])  # on the loose strap's riser, beside the shroud's +y side
LEVER_LEN = 48 * MM
LEVER_W = 8 * MM  # its width in its plane
LEVER_UP = math.radians(20.0)  # unlocked it points along the cooler's -x, its end raised this much; it swings about the
# clip's line, over the top, to point along +x, its end as far down, locked
HOOK_DROP = clip.HOOK_DROP
FIX_SHIFT = 5.5 * MM  # set the cooler down this far towards its fixed end (-y), the fixed strap outside its lug's tip,
# then slide it back: the lug goes into the strap's window


def cooler_to_world(p, origin=SEATED_POS) -> np.ndarray:
    c, s = math.cos(COOLER_YAW), math.sin(COOLER_YAW)
    x, y, z = p
    return np.asarray(origin) + np.array([c * x - s * y, s * x + c * y, z])


# -- the foam support it starts on (left of the board) ------------------------------------------------------------
# A foam support with a square hole through it: the fin block rests on its rim, so the pasted base hangs in the hole
# touching nothing; the rim stays inside the fin block's 92 x 84 mm (clear of the heat pipes' outer runs and the
# clip's straps, which hang beyond it)
FOAM_SIZE = (0.080, 0.076, 0.015)  # x, y, height
FOAM_HOLE = 0.050  # square, through
FOAM_XY = np.array([0.290, 0.200])
FIN_BOTTOM = 1.5 * MM  # the fin block's bottom above the base's bottom
START_POS = np.array([*FOAM_XY, TABLE + FOAM_SIZE[2] - FIN_BOTTOM + 0.0003])
_fx, _fy, _fh = FOAM_SIZE
_sx, _sy = (_fx - FOAM_HOLE) / 2, (_fy - FOAM_HOLE) / 2
FOAM_BLOCK = [
    *(Shape(f"foam_x{i}", pos=(FOAM_XY[0] + s * (FOAM_HOLE + _sx) / 2, FOAM_XY[1], TABLE + _fh / 2),
            size=(_sx, _fy, _fh), color=FOAM) for i, s in enumerate((-1, 1))),
    *(Shape(f"foam_y{i}", pos=(FOAM_XY[0], FOAM_XY[1] + s * (FOAM_HOLE + _sy) / 2, TABLE + _fh / 2),
            size=(FOAM_HOLE, _sy, _fh), color=FOAM) for i, s in enumerate((-1, 1))),
]

BOARD = UsdBody("board", usd="assets/board.usda", pos=tuple(BOARD_POS), rot_xyzw=yaw_xyzw(BOARD_YAW),
                size=(0.244, 0.305, 0.04), mass=None, material=False)
COOLER = UsdBody("cooler", usd="assets/cooler.usda", pos=tuple(START_POS), rot_xyzw=yaw_xyzw(COOLER_YAW),
                 size=(0.105, 0.105, 0.093), articulation=True, material=False, solver_iters=32,
                 joint_pos=tuple((n, 0.0) for n in ("leg_pos_slide_joint", "leg_pos_flex_joint", "lever_joint")))

# -- the screwdriver (assets/screwdriver.usda, work/tool/make_screwdriver.py: Wiha PicoFinish 260P 3.0 x 50 mm, origin
# at its tip, z along it: a 3 x 0.5 mm tip, a 3 mm blade to 50 mm, an 18 mm handle to 150 mm), standing tip down in a
# rack right of the board (a 30 mm block with a 4 mm square hole), and an empty cup beside it to drop it in after ----
TOOL_MASS = 0.03
RACK_FLOOR, RACK_H, RACK_HOLE, RACK_W = 0.003, 0.025, 0.004, 0.030
RACK_XY = np.array([0.22, -0.25])
TOOL_START = np.array([*RACK_XY, TABLE + RACK_FLOOR + 0.0003])  # the tip, resting on the rack's floor
TOOL_YAW = 0.0  # the tip's 3 mm edge along world x
_side = (RACK_W - RACK_HOLE) / 2
RACK = [
    Shape("rack_floor", pos=(*RACK_XY, TABLE + RACK_FLOOR / 2), size=(RACK_W, RACK_W, RACK_FLOOR), color=GREY),
    *(Shape(f"rack_wall{i}", pos=(RACK_XY[0] + sx * (RACK_HOLE + _side) / 2, RACK_XY[1] + sy * (RACK_HOLE + _side) / 2,
                                  TABLE + RACK_FLOOR + RACK_H / 2),
            size=(_side if sx else RACK_W, _side if sy else RACK_HOLE, RACK_H), color=GREY)
      for i, (sx, sy) in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1)))),
]
CUP_FLOOR, CUP_H, CUP_IN, CUP_T = 0.003, 0.050, 0.030, 0.008
CUP_XY = np.array([0.31, -0.25])
_cw = CUP_IN + 2 * CUP_T
CUP = [
    Shape("cup_floor", pos=(*CUP_XY, TABLE + CUP_FLOOR / 2), size=(_cw, _cw, CUP_FLOOR), color=GREY),
    *(Shape(f"cup_wall{i}", pos=(CUP_XY[0] + sx * (CUP_IN + CUP_T) / 2, CUP_XY[1] + sy * (CUP_IN + CUP_T) / 2,
                                 TABLE + CUP_FLOOR + CUP_H / 2),
            size=(CUP_T if sx else _cw, CUP_T if sy else CUP_IN, CUP_H), color=GREY)
      for i, (sx, sy) in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1)))),
]
TOOL = UsdBody("screwdriver", usd="assets/screwdriver.usda", pos=tuple(TOOL_START), rot_xyzw=yaw_xyzw(TOOL_YAW),
               size=(0.018, 0.018, 0.15), mass=TOOL_MASS, material=False, torsional_patch=0.009)
OBJECTS = [BOARD, COOLER, *FOAM_BLOCK, TOOL, *RACK, *CUP]

# -- the clip springs ---------------------------------------------------------------------------------------------
_STATE: dict = {"spring": None, "free": True, "anchor": None, "slid": 0.0, "lifted": False}


@on_reset
def _reset(environment) -> None:
    cooler = environment.unwrapped.scene["cooler"]
    if _STATE["spring"] is None or _STATE["spring"].cooler is not cooler:
        _STATE["spring"] = clip.ClipSpring(cooler)
    else:
        _STATE["spring"].reset()
    touch.reset()
    _STATE.update(free=True, anchor=None, slid=0.0, lifted=False)


@on_step
def _watch(environment) -> None:
    """Whether the cooler is released (no robot link touches its body, from PhysX contacts: touch.py), and the paste
    rule: from the moment the base first touches the processor, how far its centre has moved from there while touching,
    and whether it was lifted off again."""
    from yam_arena.scene import object_states

    _STATE["free"] = not touch.touching(environment, ["cooler"])["cooler"]
    paste_watch(object_states(environment, ["cooler"])["cooler"])


# -- the paste rule ---------------------------------------------------------------------------------------------
# The cooler's base (40 x 40 mm) carries pre-applied thermal paste, as AMD's boxed coolers do. AMD's instructions:
# once the heatsink has been installed it may not be lifted off and put back without cleaning both surfaces and
# applying new material, and the material must not be scratched or have gaps (FM2+ installation guide; Builder's Guide
# 30925, p. 23). So after the base touches the processor (40 x 40 mm, AMD's AM4/AM5 lid) it must not lift off, and it
# may not slide so far that the paste is wiped off part of the processor (a small shuffle into place is normal).
BASE_HALF = 20 * MM
CPU_HALF = 20 * MM
PASTE_TOUCH = 0.7 * MM  # the base's lowest corner within this of the processor's top, over it: touching
PASTE_LIFT = 3.0 * MM  # after touching, the base's lowest corner this far above the processor's top: lifted off
SLIDE_MAX = 10.0 * MM  # the base's centre may move this far (sideways) from where it first touched, ever after


def base_low(c) -> float:
    """The base's lowest corner above the processor's top (m)."""
    rot = c.rotation()
    corners = [c.pos + rot @ np.array([sx * BASE_HALF, sy * BASE_HALF, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    return min(p[2] for p in corners) - SEATED_POS[2]


def paste_watch(c) -> None:
    """Called every control step. Before the first touch nothing is recorded; from the first touch on, the base's
    sideways movement from where it touched is tracked on every step, whatever its height (so sliding while hovering
    0.7-3 mm up is caught too), and rising more than 3 mm or leaving the processor counts as lifted off."""
    off = c.pos[:2] - SEATED_POS[:2]
    over = abs(off[0]) < BASE_HALF + CPU_HALF and abs(off[1]) < BASE_HALF + CPU_HALF  # the base overlaps the processor
    low = base_low(c)
    if _STATE["anchor"] is None:
        if not (over and low < PASTE_TOUCH):
            return
        _STATE["anchor"] = c.pos[:2].copy()
    _STATE["slid"] = max(_STATE["slid"], float(np.hypot(*(c.pos[:2] - _STATE["anchor"]))))
    if not over or low > PASTE_LIFT:
        _STATE["lifted"] = True


def paste_broken() -> bool:
    return _STATE.get("lifted", False) or _STATE.get("slid", 0.0) > SLIDE_MAX


# -- scoring ----------------------------------------------------------------------------------------------------
ON_XY = 3.0 * MM  # the base's centre within this of the processor's centre
ON_Z = 1.5 * MM  # and its bottom within this of the processor's top
ON_YAW = math.radians(5.0)  # turned within this of square with the socket (lever towards the robot)
ON_TILT = math.radians(3.0)


TOOL_CLEAR = 0.080  # the screwdriver's tip at least this far (sideways) from the cooler's centre: not resting on it


def measure(objects) -> dict:
    """The cooler against its seated pose, the loose strap's travel, flex and whether it is hooked, the lever, and
    whether the screwdriver is clear of the cooler."""
    c = objects["cooler"]
    rot = c.rotation()
    off = c.pos - SEATED_POS
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    j = c.joints
    drop = -j.get("leg_pos_slide_joint", 0.0)
    flex = j.get("leg_pos_flex_joint", 0.0)
    lever = j.get(clip.LEVER, 0.0)
    tool = objects.get("screwdriver")
    return {
        "xy": float(np.hypot(off[0], off[1])),
        "z": float(off[2]),
        "yaw_err": abs((yaw - COOLER_YAW + math.pi) % (2 * math.pi) - math.pi),
        "tilt": math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))),
        "loose_hooked": clip.HOOK_DROP - clip.HOOK_TOL[1] < drop < clip.HOOK_DROP + clip.HOOK_TOL[0] and flex < clip.FLEX_TOL,
        "locked": lever > clip.LOCKED_AT,
        "tool_clear": tool is None or float(np.hypot(*(tool.pos[:2] - c.pos[:2]))) > TOOL_CLEAR,
    }


def score(objects) -> float:
    """0 if the paste was torn (lifted off after touching) or wiped (the base slid more than 10 mm on the processor),
    or unless the cooler sits square on the processor (which it can only do with its fixed strap hooked: lowered
    straight, the strap lands on the lug and holds that end up), released by the robot and clear of the screwdriver.
    0.25: it does. 0.5: and the loose strap hooks its lug. 1.0: and the lever is over, locked."""
    m = measure(objects)
    on = m["xy"] < ON_XY and abs(m["z"]) < ON_Z and m["yaw_err"] < ON_YAW and m["tilt"] < ON_TILT
    if paste_broken() or not (on and _STATE["free"] and m["tool_clear"]):
        return 0.0
    if not m["loose_hooked"]:
        return 0.25
    return 1.0 if m["locked"] else 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo ----------------------------------------------------------------------------------------
# The left arm holds the cooler from inside its fan opening: closed fingertips go down into the opening (along world
# x, across the opening's middle), and the fingers open until the pads' backs press the shroud's inner wall.
GRIP_Z = COVER[2][1] - 8 * MM  # the pads' tips this high: 8 mm into the opening, 2 mm over the fan
PAD_T = 3.15 * MM  # each closed pad's thickness along the closing direction (6.3 mm for the pair)
GRIP_SQUEEZE = 3.0 * MM  # each pad's back commanded this far past the opening's wall
GRIP_OPEN = -(FAN_R - PAD_T + GRIP_SQUEEZE)  # finger joint: the pads' inner faces this far out from the TCP
CARRY = 0.05  # the base this high over the table while carrying (the TCP stays under the arm's reach)
SET_ABOVE = 0.004  # lower to this over the processor, then set it down slowly
SLIDE_OVER = 0.004  # command the slide this far further: the arm lags behind it sideways
SLIDE_EASE = 0.001  # slide it with the hand this much higher than set down (pressed down, it sticks)
SET_CORRECTION = (0.0059, -0.0065)  # aim the base this far past the processor (world x, y): the arm sags with the cooler and
# it hangs in the hand (measured)
# The right arm holds the screwdriver upright by its handle, tip down, the fingers closing along world y. The blade
# works in the lever's plane beside the shroud's side (the lever, unlocked, lies along world +x, away from the robot).
TOOL_GRIP = 68.0 * MM  # the pads' tips this far above the tool's tip, on the handle (its top then just under the palm)
TOOL_CLOSE = -(9.0 * MM - 4.0 * MM)  # finger joint on the 18 mm handle: 4 mm of squeeze each side
TOOL_CORRECTION = (0.0, 0.0015, 0.0)  # aim the tool's tip this far past where it should go (world; measured: the hand
# holds it about 2 mm out from the cooler)
TOOL_LIFT = 0.045  # lift it this far out of its rack
LEVER_PRESS = 35.0 * MM  # the tip commanded this far below the lever's top (the strap travels 7.6 mm, then stops)
PRESS_AT = 12.0 * MM  # the tip presses the lever this far out from its pivot
PUSH_UP = 8.0 * MM  # the tip this far above the pivot (hooked) while it pushes the lever over: clear of its 6 mm boss,
# under its end's lower corner (12.6 mm up, unlocked): the shaft's side pushes the end towards the pivot, the end
# riding up the shaft, and the hand stays low
PUSH_HOLD_DOWN = 6.0 * MM  # command the hand this much lower while it pushes the end (it is lifted about 10 mm)
PUSH_UP_LATE = 16.0 * MM  # the tip this far above the pivot once the shaft is within 12 mm of it
PUSH_PAST = 10.0 * MM  # push until the shaft's side is this far past the pivot: well over straight up


def demo():
    """The left arm takes the cooler off its foam support from inside its fan opening, carries it over the processor,
    sets it down 5.5 mm towards its fixed end (the fixed strap outside its lug) and slides it back, so the lug goes
    into the strap's window; it lets go and leaves. The right arm then takes the screwdriver upright out of its rack,
    presses the lever down with its tip until the loose strap rides over its lug and hooks, lifts the lever's end with
    the tip until the spring takes it over, locked, and drops the screwdriver in the cup."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    # -- left arm: the cooler
    YAW_L = -math.pi / 2  # the fingers open along world x, across the opening, away from the clip and the lever
    C = GRIP_OPEN
    up = GRIP_Z + TIP  # the TCP above the cooler's origin
    sx, sy = START_POS[:2]
    px, py = SEATED_POS[0] + SET_CORRECTION[0], SEATED_POS[1] + SET_CORRECTION[1]
    fix = cooler_to_world((0.0, -FIX_SHIFT, 0.0)) - SEATED_POS  # towards the fixed end (world)
    carry = TABLE + CARRY + up
    hover = TABLE + 0.15
    cover_top = SEATED_POS[2] + COVER[2][1]
    seat = SEATED_POS[2] + up - 0.001
    # the TCP's y when the closed fingers' side has pushed the opening's far wall until the cooler is centred, and 3 mm
    # on (the frame stops it). The fingers taper: 6 mm up from their tips, where they meet the wall, they are 18 mm wide.
    PUSH_TO = SEATED_POS[1] - FAN_R + 9.0 * MM - 0.006
    push_x = SEATED_POS[0]  # on the cooler's centre line, so the push doesn't turn it
    left = [
        W(2.0, (sx, sy, START_POS[2] + up + 0.03), 0.0, "over the cooler's opening, fingers closed", YAW_L),
        W(2.0, (sx, sy, START_POS[2] + up), 0.0, "down into the opening", YAW_L),
        W(1.5, (sx, sy, START_POS[2] + up), C, "open against the shroud's inner wall", YAW_L),
        W(1.0, (sx, sy, START_POS[2] + up), C, "settle", YAW_L),
        W(1.5, (sx, sy, carry), C, "lift", YAW_L),
        W(3.0, (px + fix[0], py + fix[1], carry), C, "over the processor, towards the fixed end", YAW_L),
        W(2.0, (px + fix[0], py + fix[1], SEATED_POS[2] + SET_ABOVE + up), C, "down to it", YAW_L),
        W(1.5, (px + fix[0], py + fix[1], seat), C, "set it on, the fixed strap outside its lug", YAW_L),
        W(1.5, (px + fix[0], py + fix[1], seat), 0.0, "let go: fingers closed, off the wall; it settles flat", YAW_L),
        W(1.0, (px + fix[0], py + fix[1], seat + 0.003), 0.0, "fingertips up a little", YAW_L),
        # Held while it is pushed, the cooler tips (the wrist gives under it) and the fixed strap's lower bar meets the
        # lug's tip. Let go, it sits flat, and the closed fingers' side pushes the opening's wall low enough that it
        # slides on its paste without tipping; the retention frame's end stops the strap once the lug is in.
        W(1.0, (push_x, py + fix[1], seat + 0.003), 0.0, "over to its centre line", YAW_L),
        W(3.0, (push_x, PUSH_TO, seat + 0.003), 0.0, "push it back by the opening's wall: the lug into the fixed strap's window", YAW_L),
        W(1.0, (push_x, PUSH_TO + 0.006, seat + 0.003), 0.0, "off the wall", YAW_L),
        W(1.0, (push_x, PUSH_TO + 0.006, cover_top + 0.025 + TIP), 0.0, "up out of the opening", YAW_L),
        W(1.5, (0.25, 0.25, TABLE + 0.20), O, "away, clear of the right arm", 0.0),
    ]

    # -- right arm: the screwdriver
    def tool_at(tip_x, tip_y, tip_z):
        """The TCP holding the tool upright with its tip at world (tip_x, tip_y, tip_z)."""
        return (tip_x + TOOL_CORRECTION[0], tip_y + TOOL_CORRECTION[1], tip_z + TOOL_GRIP + TIP + TOOL_CORRECTION[2])

    t0 = TOOL_START
    g = (t0[0], t0[1], t0[2] + TOOL_GRIP + TIP)  # picking it up: no correction
    T = TOOL_CLOSE
    high = TABLE + 0.105  # the tip this high while it travels: over the shroud's top
    # it moves only once the left arm has set the cooler down and gone (the two would meet over the cooler)
    right = [
        W(1.5, (0.25, -0.20, hover), O, "to its waiting place", 0.0),
        W(sum(w.duration for w in left) - 1.5, (0.25, -0.20, hover), O, "wait for the left arm to finish", 0.0),
        W(2.0, (g[0], g[1], g[2] + 0.05), O, "over the screwdriver's handle", 0.0),
        W(1.5, g, O, "down round it", 0.0),
        W(1.5, g, T, "grip", 0.0),
        W(1.0, g, T, "settle", 0.0),
        W(1.5, (g[0], g[1], g[2] + TOOL_LIFT), T, "lift it out of the rack", 0.0),
    ]
    pivot = cooler_to_world(LEVER_PIVOT)  # loose
    ly = pivot[1]
    ca, sa = math.cos(LEVER_UP), math.sin(LEVER_UP)  # the lever points along world +x from its pivot, raised LEVER_UP
    # press it down on its top 12 mm out from the pivot
    px = pivot[0] + PRESS_AT * ca - (LEVER_W / 2) * sa
    top = pivot[2] + PRESS_AT * sa + (LEVER_W / 2) * ca
    right += [
        W(3.0, tool_at(px, ly, high), T, "over the lever, beside the shroud", 0.0),
        W(1.5, tool_at(px, ly, top + 0.003), T, "tip down onto the lever", 0.0),
        W(2.0, tool_at(px, ly, top - LEVER_PRESS), T, "press the lever down: the loose strap rides over its lug", 0.0),
        W(1.0, tool_at(px, ly, top - LEVER_PRESS), T, "hold", 0.0),
        W(1.0, tool_at(px, ly, top + 0.008), T, "up: the strap springs back up under its lug, hooked", 0.0),
    ]
    # push it over: the shaft's side against its end's lower corner, towards the pivot
    hooked = pivot - (0.0, 0.0, HOOK_DROP)
    corner_x = hooked[0] + LEVER_LEN * ca + (LEVER_W / 2) * sa
    half = 1.5 * MM  # the blade's half width
    x_start = corner_x + half + 0.002
    z_push = hooked[2] + PUSH_UP
    x_end = hooked[0] - PUSH_PAST - half
    right += [
        W(1.0, tool_at(x_start, ly, top + 0.008), T, "out past the lever's end", 0.0),
        W(1.5, tool_at(x_start, ly, z_push), T, "down beside its end, under its corner", 0.0),
    ]
    steps = 8
    for k in range(1, steps + 1):
        x = x_start + (x_end - x_start) * k / steps
        # the lever's end, riding up the shaft, lifts the hand: command it lower while the shaft is clear of the boss
        # near the pivot the tip rides higher, pushing the steep lever further out along it (more leverage)
        z = z_push - PUSH_HOLD_DOWN if x > hooked[0] + 0.012 else hooked[2] + PUSH_UP_LATE
        right.append(W(0.8, tool_at(x, ly, z), T, f"push its end towards the pivot ({k}/{steps})", 0.0))
    cx, cz = x_end, z_push
    ux, uy = CUP_XY
    cup_z = TABLE + CUP_FLOOR + 0.004
    right += [
        W(1.0, tool_at(cx, ly - 0.006, cz), T, "out of the lever's plane: the spring takes it over, locked", 0.0),
        W(1.5, tool_at(cx, ly - 0.06, cz), T, "out away from the cooler, low", 0.0),
        W(1.5, tool_at(cx, ly - 0.06, TABLE + 0.07), T, "up", 0.0),
        W(2.5, tool_at(ux, uy, TABLE + 0.07), T, "over the cup", 0.0),
        W(1.5, tool_at(ux, uy, cup_z), T, "down into the cup", 0.0),
        W(1.0, tool_at(ux, uy, cup_z), O, "let go", 0.0),
        W(1.5, (ux, uy, cup_z + TOOL_GRIP + TIP + 0.10), O, "straight up off the handle", 0.0),
        W(1.5, (0.25, -0.25, hover), O, "away", 0.0),
    ]

    def sync():
        """Pad the arm that is ahead with a wait, so both end together."""
        tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
        if tl < tr:
            left.append(W(tr - tl, left[-1].pos, left[-1].finger, "wait", left[-1].yaw))
        elif tr < tl:
            right.append(W(tl - tr, right[-1].pos, right[-1].finger, "wait", right[-1].yaw))

    sync()
    return {"left": left, "right": right}
