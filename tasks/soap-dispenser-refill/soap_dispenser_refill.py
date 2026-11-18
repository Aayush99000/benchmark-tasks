"""On the station table: a wall soap dispenser (a GOJO ADX-7 push-style dispenser, 700 mL) on a wall panel standing on
the table, its front towards the arms. Its front cover, hinged at its bottom, hangs open; in it the empty refill
stands pump down in the pump house. Beside it, a refill case holds the new refill, pump down, and an empty slot.
Change the refill: hold the empty refill while pushing the release latch on the front of the pump house, take it out
and put it in the case's empty slot; push the new refill's pump back into the pump house until it clicks; then swing
the cover up until it latches.

The pump house is a model of the ADX-7's: a refill's collar slides back into a groove open at the front; the last
3 mm take a push against a detent (up to 6 N), and at the back of the groove the collar clicks and is locked; a
locked refill comes out only while the release latch is pushed in. The cover latches when swung within 1.5 degrees of
shut; a cover short of that falls open again.

Run it:  inspect-robots run --embodiment yam_arena -E task=soap_dispenser_refill.py -E object_jitter=0 \\
           -E control_hz=20 --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import Hinge, Slider, body, box, cyl  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

WHITE = (0.93, 0.93, 0.91)
GREY = (0.55, 0.57, 0.60)
DARK = (0.20, 0.21, 0.23)
GREEN = (0.10, 0.62, 0.30)
LATCH_BLUE = (0.10, 0.35, 0.85)
TILE = (0.80, 0.84, 0.86)
GROUT = (0.62, 0.66, 0.69)
# WHY opaque: drawn see-through (alpha 0.45), the bottle rendered black in the cameras (run soap3)
BOTTLE = (0.84, 0.89, 0.92)  # clear plastic, empty
SOAP = (0.95, 0.78, 0.15)
CARD = (0.62, 0.45, 0.26)


def rot_y(angle: float) -> np.ndarray:
    return axis_angle_matrix(Y_AXIS, angle)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in fittingkit.matrix_to_quat_rel(r))


# -- the dispenser (fixed) and its wall panel ---------------------------------------------------------------------------
# GOJO ADX-7 (8780-06): 3.94 x 3.71 x 9.79 in (100 x 94 x 249 mm) [retailer listing]. Its frame: origin on the table
# under the middle of its front (the line of the refill's front), the world's axes; its back on the wall panel.
XF = 0.339  # the front of the pump house, the top cap and a seated refill
DEPTH = 0.094  # from that front to the wall: a seated refill's back 3 mm in front of the 3 mm back plate
# WHY 94 mm (it was 91): a seated refill's back met the back plate, and with a degree of tilt the new one jammed on it
# 1.2 mm before its click (runs soap23 and soap24); the dispenser is now 102.5 mm deep, against GOJO's 94 mm
XW = XF + DEPTH  # the wall panel's face
WIDTH = 0.100
Z0 = 0.100  # the dispenser's bottom, above the table: GOJO asks for at least 3 in (7.6 cm) above a counter
# WHY 10 cm and no higher: a refill is gripped 11 cm above its collar, 24.5 cm up in the pump house (27 cm for the full
# one, the arm's sag allowed for), and both arms reach a forward grip up to 30 cm (grid.py)
HEIGHT = 0.249
PH_TOP = 0.050  # the pump house: from the bottom to 5 cm up [estimate]
FLOOR = 0.040  # the groove's floor, 1 cm under the pump house's top
CAP_H = 0.014  # the top cap
GROOVE_HALF = 27.5 * MM  # the groove's half width round a 50 mm collar (2.5 mm either side)
COLLAR_R, COLLAR_H = 25.0 * MM, 20.0 * MM
# GOJO 8711-04 refill: 9.3 x 3.33 x 3.48 in (236 x 85 x 88 mm) [GOJO listing], its pump and collar included: here the
# bottle 165 mm and the collar 20 mm; the pump's nozzle, inside the pump house, isn't modelled [estimate]
# WHY 165 mm: 170 mm left 5 mm under the top cap, and a refill coming in 7 degrees over caught the cap's edge (run soap22)
BOTTLE_X, BOTTLE_Y, BOTTLE_H = 0.088, 0.085, 0.165
CORNER_R = 0.006  # its upright edges rounded [estimate: the ADX refill bottle's corners are rounded]
XS = XF + BOTTLE_X / 2  # a seated refill's axis (its collar's middle)
GROOVE_BACK = XS + COLLAR_R + 1.5 * MM
SEAT_Z = TOP + Z0 + FLOOR  # a seated refill's collar bottom
DISPENSER_AT = np.array([XF, 0.0, TOP])
LATCH_Z = Z0 + 0.0195  # the release latch's middle, on the pump house's front under the groove
LATCH_W, LATCH_H, LATCH_D = 20.0 * MM, 24.0 * MM, 8.0 * MM
LATCH_OUT, LATCH_TRAVEL, UNLOCK = 4.5 * MM, 5.0 * MM, 2.5 * MM  # it stands 4.5 mm out of its well, under the cover
# WHY 4.5 mm out: standing 1.5 mm out, it unlocks only once the fingertip is past the pump house's front, where the
# finger's flank meets the face beside the well (plan_check); 3.5 mm out, the hand stopped with it 2.0 mm in (run soap7)
LATCH_K = 1000.0  # N/m: 2.5 N at the unlock point [estimate: a thumb push]
# WHY a 60 x 34 mm well: the closed fingertips, coming in 70 degrees across and 20 degrees down, cut the well's mouth
# about 25 mm tall and much wider than they are thick when they are 6 mm into the latch, and a 30 mm well's top edge
# rubbed them (their hulls, plan_check)
WELL_W, WELL_H, WELL_D = 60.0 * MM, 34.0 * MM, 12.0 * MM
LATCH_AT = np.array([XF - LATCH_OUT + LATCH_D / 2, 0.0, TOP + LATCH_Z])


def _block_with_well(x0, x1, y0, y1, z0, z1, wz, ww, wh, wd) -> list:
    """The pump house's lower block, x0..x1 (front to back), y0..y1, z0..z1, with a well ``ww`` wide, ``wh`` tall and
    ``wd`` deep in its front face round height ``wz``; in the dispenser's frame."""
    a, b = wz - wh / 2, wz + wh / 2
    xm = x0 + wd
    out = [
        box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + a) / 2), (x1 - x0, y1 - y0, a - z0), GREY),  # under the well
        box(((x0 + x1) / 2, (y0 + y1) / 2, (b + z1) / 2), (x1 - x0, y1 - y0, z1 - b), GREY),  # over it
        box(((xm + x1) / 2, (y0 + y1) / 2, wz), (x1 - xm, y1 - y0, wh), GREY),  # behind it
        box(((x0 + xm) / 2, (y0 - ww / 2) / 2, wz), (wd, -ww / 2 - y0, wh), GREY),  # beside it
        box(((x0 + xm) / 2, (ww / 2 + y1) / 2, wz), (wd, y1 - ww / 2, wh), GREY),
    ]
    return out


def _dispenser_shapes() -> list:
    w2 = WIDTH / 2
    back_t = 0.003
    gz0, gz1 = Z0 + FLOOR, Z0 + PH_TOP
    gb = GROOVE_BACK - XF
    return [
        box((DEPTH - back_t / 2, 0.0, Z0 + HEIGHT / 2), (back_t, WIDTH, HEIGHT), WHITE),  # the back plate
        # the pump house: a block with the latch's well in its front, and above it the groove's two sides and back
        *_block_with_well(0.0, DEPTH - back_t, -w2, w2, Z0, gz0, LATCH_Z, WELL_W, WELL_H, WELL_D),
        box(((DEPTH - back_t) / 2, -(w2 + GROOVE_HALF) / 2, (gz0 + gz1) / 2),
            (DEPTH - back_t, w2 - GROOVE_HALF, gz1 - gz0), GREY),
        box(((DEPTH - back_t) / 2, (w2 + GROOVE_HALF) / 2, (gz0 + gz1) / 2),
            (DEPTH - back_t, w2 - GROOVE_HALF, gz1 - gz0), GREY),
        box(((gb + DEPTH - back_t) / 2, 0.0, (gz0 + gz1) / 2), (DEPTH - back_t - gb, 2 * GROOVE_HALF, gz1 - gz0), GREY),
        box(((DEPTH - back_t) / 2, 0.0, Z0 + HEIGHT - CAP_H / 2), (DEPTH - back_t, WIDTH, CAP_H), WHITE),  # top cap
    ]


def _dispenser_looks() -> list:
    """Drawn only: the green ring round the groove, the back shell's side rails, the cover's hinge knuckles and the
    wall's tiles."""
    w2 = WIDTH / 2
    gz1 = Z0 + PH_TOP
    out = [
        box((XS - XF, 0.0, gz1 + 0.0005), (2 * COLLAR_R + 8 * MM, 2 * COLLAR_R + 8 * MM, 0.001), GREEN),
        box((DEPTH - 0.010, -w2 + 0.0015, Z0 + HEIGHT / 2), (0.014, 0.003, HEIGHT - 0.02), WHITE),
        box((DEPTH - 0.010, w2 - 0.0015, Z0 + HEIGHT / 2), (0.014, 0.003, HEIGHT - 0.02), WHITE),
        box((-0.004, -w2 + 0.006, Z0 - 0.002), (0.006, 0.012, 0.006), GREY),
        box((-0.004, w2 - 0.006, Z0 - 0.002), (0.006, 0.012, 0.006), GREY),
    ]
    return out


def _wall_shapes() -> tuple[list, list]:
    """The wall panel the dispenser is screwed to: 1.5 cm thick, 40 cm wide, 40 cm tall, standing on the table; its
    tiles drawn only. In the panel's frame (origin on the table under its face's middle)."""
    solid = [box((0.0075, 0.0, 0.20), (0.015, 0.40, 0.40), TILE)]
    seen = [box((0.0003, y, 0.20), (0.001, 0.003, 0.40), GROUT) for y in (-0.15, -0.05, 0.05, 0.15)]
    seen += [box((0.0003, 0.0, z), (0.001, 0.40, 0.003), GROUT) for z in (0.10, 0.20, 0.30)]
    return solid, seen


_DS, _DV = _dispenser_shapes(), _dispenser_looks()
DISPENSER = body("dispenser", pos=tuple(DISPENSER_AT), friction=0.8, shapes=_DS + _DV,
                 visual_only=tuple(range(len(_DS), len(_DS) + len(_DV))))
_WS, _WV = _wall_shapes()
WALL = body("wall_panel", pos=(XW, 0.0, TOP), friction=0.8, shapes=_WS + _WV,
            visual_only=tuple(range(len(_WS), len(_WS) + len(_WV))))
LATCH = body("release_latch", pos=tuple(LATCH_AT), mass=0.004, friction=0.8,
             shapes=[box((0.0, 0.0, 0.0), (LATCH_D, LATCH_W, LATCH_H), LATCH_BLUE)])
fittingkit.add_slider(Slider("dispenser", "release_latch", anchor=tuple(LATCH_AT - DISPENSER_AT), axis=(1, 0, 0),
                             lower=0.0, upper=LATCH_TRAVEL, stiffness=LATCH_K, damping=2.0, target=0.0))

# -- the cover: a shell hinged at its bottom front edge -------------------------------------------------------------
# Its frame: origin on the hinge's axis (the bottom of its inner face), x towards the dispenser, z up its face when
# shut. Shut, its inner face is 5.5 mm in front of the pump house (the latch stands 4.5 mm out), so the dispenser is
# 99.5 mm deep where a real one is 94.
# WHY the hinge 1 cm under the pump house, the shell wrapping its bottom: the latch, 1.75 cm above a hinge at the pump
# house's bottom, left the pushing hand no room between the open cover and the refill above (plan_check)
HINGE_DROP = 0.010
COVER_H, COVER_W, COVER_T = 0.240 + HINGE_DROP, 0.106, 0.003
COVER_GAP = LATCH_OUT + 1.0 * MM  # the shut cover's inside this far in front of the pump house, clear of the latch
HINGE_AT = np.array([XF - COVER_GAP, 0.0, TOP + Z0 - HINGE_DROP])
OPEN_DEG = 70.0  # the hinge's stop
# WHY a stop 70 degrees open: a real ADX-7 cover hangs straight down the wall; 10 cm above the table it would hang
# onto the table, and a hand reaches its edge to push it shut only from 70 degrees open or less (grid.py) [the stop is
# ours]
LATCH_DEG = 1.5  # the cover latches within this of shut
COVER_MASS = 0.15  # [estimate]


def _cover_shapes() -> tuple[list, list]:
    solid = [box((-COVER_T / 2, 0.0, COVER_H / 2), (COVER_T, COVER_W, COVER_H), WHITE)]
    seen = [
        box((-COVER_T - 0.0005, 0.0, 0.135), (0.001, 0.030, 0.080), (0.70, 0.78, 0.82)),  # the sight window
        box((-COVER_T - 0.0005, 0.0, 0.215), (0.001, 0.070, 0.030), DARK),  # the push bar
        box((0.006, -COVER_W / 2 + 0.0015, COVER_H / 2), (0.015, 0.003, COVER_H), WHITE),  # the shell's sides
        box((0.006, COVER_W / 2 - 0.0015, COVER_H / 2), (0.015, 0.003, COVER_H), WHITE),
    ]
    return solid, seen


R_COVER_OPEN = rot_y(-math.radians(OPEN_DEG))  # its top swung down towards the arms
_CS, _CV = _cover_shapes()
COVER = body("cover", pos=tuple(HINGE_AT), mass=COVER_MASS, friction=0.8, shapes=_CS + _CV,
             visual_only=tuple(range(len(_CS), len(_CS) + len(_CV))), quat=quat_xyzw(R_COVER_OPEN),
             com=(-COVER_T / 2, 0.0, COVER_H / 2))
# a PhysX revolute angle is the child's turn about the axis (+y) from where it is spawned: open, so shutting it is +70
fittingkit.add_hinge(Hinge("dispenser", "cover", anchor=tuple(HINGE_AT - DISPENSER_AT), axis=(0.0, 1.0, 0.0),
                           damping=0.004, lower=0.0, upper=math.radians(OPEN_DEG)))


def cover_open_deg(rot: np.ndarray) -> float:
    """How far (degrees) the cover is open from shut: 0 shut, 70 on its stop."""
    z = rot[:, 2]
    return math.degrees(math.atan2(-z[0], z[2]))


# -- the refills -----------------------------------------------------------------------------------------------------
# A refill's frame: origin in the middle of its pump collar's bottom, z up through the bottle (it stands pump down).
def _refill_shapes(soap_h: float) -> tuple[list, list]:
    """A refill's shapes; ``soap_h`` of soap drawn on its outside from the pump end (the bottle is opaque here)."""
    zc, r = COLLAR_H + BOTTLE_H / 2, CORNER_R
    # the bottle with its four upright edges rounded: two crossed boxes and a cylinder in each corner
    # WHY rounded: the open fingers leave 4.5 mm either side of it, and an arm about 5 mm off caught a square edge
    # with a fingertip and stopped on the bottle's front (runs soap4, soap5); a rounded edge, as a real bottle's,
    # steers the finger round it. WHY 6 mm, not 12: squeezed on 12 mm roundings, the closing fingers pushed the hand
    # 3.5 cm back off the bottle (run soap7)
    solid = [cyl((0.0, 0.0, COLLAR_H / 2), COLLAR_R, COLLAR_H, WHITE),
             box((0.0, 0.0, zc), (BOTTLE_X, BOTTLE_Y - 2 * r, BOTTLE_H), BOTTLE),
             box((0.0, 0.0, zc), (BOTTLE_X - 2 * r, BOTTLE_Y, BOTTLE_H), BOTTLE)]
    solid += [cyl((sx * (BOTTLE_X / 2 - r), sy * (BOTTLE_Y / 2 - r), zc), r, BOTTLE_H, BOTTLE)
              for sx in (-1, 1) for sy in (-1, 1)]
    seen = [cyl((0.0, 0.0, COLLAR_H - 0.003), COLLAR_R + 0.5 * MM, 0.004, GREEN),  # the collar's key ring
            box((-BOTTLE_X / 2 - 0.0015, 0.0, COLLAR_H + 0.10), (0.001, 0.060, 0.045), WHITE)]  # the label
    if soap_h > 0:  # the soap, at the pump end (the bottle stands on its pump), drawn just outside the bottle
        seen.append(box((0.0, 0.0, COLLAR_H + 0.001 + soap_h / 2), (BOTTLE_X + 0.001, BOTTLE_Y + 0.001, soap_h), SOAP))
    return solid, seen


_OS, _OV = _refill_shapes(0.012)
# WHY its inertia 10 times a 60 g bottle's: held by the lock's turning spring (30 N m/rad, see PumpHouse), the light
# bottle's own inertia would make that spring unstable at 240 Hz [model]
OLD_INERTIA = (2.2e-3, 2.2e-3, 0.8e-3)
OLD_REFILL = body("old_refill", pos=(XS, 0.0, SEAT_Z), mass=0.06, friction=1.0, shapes=_OS + _OV,
                  visual_only=tuple(range(len(_OS), len(_OS) + len(_OV))), com=(0.0, 0.0, 0.08), inertia=OLD_INERTIA)
# A full 700 mL refill weighs about 0.75 kg (foam soap about 1 kg/L, and the bottle and pump) [estimate]; here 0.30 kg.
# WHY lighter than a real one: holding 0.75 kg out in front, the official sim's YAM arm gave 6 cm and 25 degrees (its
# wrist joints are soft: 10 N m/rad), and aimed against that give it reached its joints' force limits and stopped
# 9 to 11 cm short (runs soap18, soap19); a real YAM's wrist (7 N m) holds it [fidelity: stated in task.json]
FULL_MASS = 0.30

# The refill case and the recycling box are both on the robot's right, where the right hand, which handles both refills,
# reaches them.
# WHY not the new refill on the left for the left hand: pushing the latch, the left wrist must stay out on its own side
# (the two forearms met in front of the dispenser, runs soap2 to soap4), and there it ran into a case on the left
# The refill case: an open carton the new refill stands in (3 mm wider than it front to back, 20 mm across), on a
# closed case.
# WHY on a closed case 5 cm tall: on the table, an arm can't reach a refill's grip beside its own base with the hand
# level (grid.py: 13 to 76 mm off at y -0.15 to -0.25 under 14 cm up)
CASE_BASE = 0.050
CASE_AT = np.array([0.34, -0.135, TOP + CASE_BASE])
CASE_FLOOR, CASE_WALL_H, CASE_T = 0.004, 0.060, 0.004
# The recycling bin: an open bin for the empty refill (GOJO: the bottle is recyclable), beyond the case and nearer the
# arms, so neither refill is carried over the other's place.
# WHY 15 cm tall: dropped in a 7 cm box, the empty refill leaned out over its side and the hand, coming for the new
# refill, knocked it (run soap14)
BOX_AT = np.array([0.22, -0.285, TOP])
BOX_IN, BOX_H, BOX_T = 0.150, 0.150, 0.004
RECYCLE_BLUE = (0.12, 0.40, 0.75)


CASE_SIDE_GAP = 0.020  # the slot's room either side of the refill, across


def _case_shapes() -> list:
    # WHY 20 mm either side: in a slot 3 mm wider than the refill, the closing finger shoved it into the slot's side,
    # it tipped 13 degrees and jammed there as the hand lifted it (run soap15)
    ix, iy = BOTTLE_X + 0.006, BOTTLE_Y + 2 * CASE_SIDE_GAP
    ox, oy = ix + 2 * CASE_T, iy + 2 * CASE_T
    h = CASE_WALL_H
    out = [box((0.0, 0.0, -CASE_BASE / 2), (ox, oy, CASE_BASE), (0.55, 0.40, 0.23)),  # the closed case under it
           box((0.0, 0.0, CASE_FLOOR / 2), (ox, oy, CASE_FLOOR), CARD)]
    for sx in (-1, 1):  # front and back
        out.append(box((sx * (ix + CASE_T) / 2, 0.0, h / 2), (CASE_T, oy, h), CARD))
    for sy in (-1, 1):  # the two ends
        out.append(box((0.0, sy * (iy + CASE_T) / 2, h / 2), (ix, CASE_T, h), CARD))
    return out


def _box_shapes() -> list:
    o = BOX_IN + 2 * BOX_T
    out = [box((0.0, 0.0, BOX_T / 2), (o, o, BOX_T), RECYCLE_BLUE)]
    for s in (-1, 1):
        out.append(box((s * (BOX_IN + BOX_T) / 2, 0.0, BOX_H / 2), (BOX_T, o, BOX_H), RECYCLE_BLUE))
        out.append(box((0.0, s * (BOX_IN + BOX_T) / 2, BOX_H / 2), (BOX_IN, BOX_T, BOX_H), RECYCLE_BLUE))
    return out


CASE = body("refill_case", pos=tuple(CASE_AT), friction=0.8, shapes=_case_shapes())
RECYCLING = body("recycling_box", pos=tuple(BOX_AT), friction=0.8, shapes=_box_shapes())
NEW_AT = np.array([CASE_AT[0], CASE_AT[1], CASE_AT[2] + CASE_FLOOR])
_NS, _NV = _refill_shapes(0.150)
NEW_INERTIA = (1.05e-3, 1.05e-3, 0.37e-3)  # a solid 0.30 kg block 88 x 85 x 185 mm, about its centre of mass
NEW_REFILL = body("new_refill", pos=tuple(NEW_AT), mass=FULL_MASS, friction=1.0, shapes=_NS + _NV,
                  visual_only=tuple(range(len(_NS), len(_NS) + len(_NV))), com=(0.0, 0.0, 0.10),
                  inertia=NEW_INERTIA)

OBJECTS = [WALL, DISPENSER, LATCH, COVER, OLD_REFILL, CASE, NEW_REFILL, RECYCLING]
REFILLS = ("old_refill", "new_refill")


# -- the pump house and the cover's latch ------------------------------------------------------------------------------
SEAT_TOL = 2.0 * MM  # within this of the seat, a collar clicks
DETENT_FROM = 3.0 * MM  # the last 3 mm before the seat take a push
DETENT_K, DETENT_MAX = 3000.0, 3.0  # N/m, N
# WHY a 3 N click 2 mm short of the seat (it was 6 N, 1 mm short): holding the 0.30 kg refill out in front, the sim's
# arm pushed it no further than 42 mm in, against about 3 N, however far past the seat it was aimed (10, 12 and 22 mm;
# runs soap23, soap24, soap26); a real refill takes a firmer push [model]
IN_Y, IN_Z, IN_TILT = 4.0 * MM, 4.0 * MM, 8.0  # in the groove: within these of its middle line and floor
LEAD_Y, LEAD_K, LEAD_MAX = 12.0 * MM, 1500.0, 6.0  # the groove's chamfered mouth steers a collar within 12 mm in
PUSHED_IN = 25.0 * MM  # a collar this far in (its middle past the pump house's front) is in the pump house
# A locked refill is held on its seat by a stiff spring and damper on its pose, capped at 40 N, not by writing its pose
# back each substep.
# WHY a spring, not its pose written back: written back every substep, a gripped refill let the YAM's closing finger
# pass 35 mm into it (its joint went from -47 to -12 mm; run soapdiag1), and the hand slid off it once it was let go
HOLD_K, HOLD_MAX = 4000.0, 40.0  # N/m, N
HOLD_TURN_K, HOLD_TURN_C = 30.0, 0.5  # N m/rad, N m s/rad (about critical for 2.2e-3 kg m2)
HOLD_DT = 1.0 / 240.0  # the substep the hold is applied at
# WHY each gain capped by its axis's mass or inertia (k <= I / (4 dt^2), then critical damping, so c dt / I <= 1): the
# hold is an explicit force, and at 0.5 N m s/rad about the empty bottle's upright (0.8e-3 kg m2) its damping
# overshot each substep; once the back plate no longer touched the seated bottle, the closing finger set it turning
# and it spun up from 5 to 154 degrees in 0.2 s and flew off (run soap25)
# WHY 30 N m/rad: at 2, the closing finger, pushing near the bottle's front edge, turned the held refill about its
# upright like a door, closed 32 mm and dragged the hand 23 mm (run soap13); a refill seated against the back plate
# can't turn that way
CLEAR = 0.02  # both grippers this far from the refills and the cover at the end


def collar_offset(f, origin=np.zeros(3)) -> tuple[float, float, float, float]:
    """(how far the collar's middle is past the pump house's front, how far off the groove's middle line, how far its
    bottom is above the groove's floor, how far the refill is tilted from upright)."""
    p = f.pos - origin
    tilt = math.degrees(math.acos(float(np.clip(f.rot[2, 2], -1.0, 1.0))))
    return float(p[0] - XF), float(p[1]), float(p[2] - SEAT_Z), tilt


class PumpHouse:
    """Runs before every physics substep (240 Hz): the pump house's lock and the cover's latch, from the refills', the
    release latch's and the cover's poses.

    A collar in the groove (on its middle line and floor, upright) is steered to its middle at the mouth, pushed back
    by the detent in its last 3 mm, and clicks at its back: from then it is locked (held where it clicked). A locked
    refill is let go while the release latch is pushed in 2.5 mm, and clicks again if it is still at the back when the
    latch comes out. The cover, swung within 1.5 degrees of shut, latches and is held shut."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("SOAP_LOG")
        self.trace = os.environ.get("SOAP_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["old_refill", "new_refill", "release_latch", "cover"]

    def clear(self) -> None:
        self._n = 0
        self.locked = {"old_refill": True, "new_refill": False}
        self.clicks = {"old_refill": 0, "new_refill": 0}
        self.offset = {"old_refill": (XS - XF, 0.0, 0.0, 0.0), "new_refill": (-1.0, 0.0, 0.0, 0.0)}
        self.latch_in = 0.0
        self.cover_deg = OPEN_DEG
        self.cover_latched = False
        self.frames = {}
        self.poses = {}

    def pressed(self) -> bool:
        return self.latch_in >= UNLOCK

    def update(self, frames: dict, acc: dict) -> None:
        latch, cover = frames.get("release_latch"), frames.get("cover")
        if latch is None or cover is None or any(frames.get(n) is None for n in REFILLS):
            return
        self._n += 1
        self.latch_in = float(latch.pos[0] - self.origin[0] - LATCH_AT[0])
        self.poses = {}
        for name in REFILLS:
            self._refill(name, frames[name], acc)
        self._cover(cover)
        self.frames = {n: frames[n] for n in (*REFILLS, "cover")}
        if self.trace and self._n % 24 == 0:
            self._write_trace()

    def _refill(self, name: str, f, acc: dict) -> None:
        depth, off, up, tilt = collar_offset(f, self.origin)
        self.offset[name] = (depth, off, up, tilt)
        seat = XS - XF
        if self.locked[name]:
            if self.pressed():
                self.locked[name] = False
                self._note(f"{self._n / 240:7.2f}s {name} let go (latch in {1000 * self.latch_in:3.1f} mm)")
            else:
                self._hold(name, f, acc)
                return
        in_line = abs(off) < IN_Y and -2.0 * MM < up < IN_Z and tilt < IN_TILT
        bottom = f.world((0.0, 0.0, 0.0))
        if -COLLAR_R < depth < seat and abs(off) < LEAD_Y and -2.0 * MM < up < 10.0 * MM and tilt < 15.0:
            # WHY a lead-in: with 1.5 mm between the collar and the groove's sides, a refill brought in a few mm off
            # the line sat on the groove's edge; a real pump house's mouth is chamfered and steers it in
            if abs(off) > 0.2 * MM:
                push = -LEAD_K * off
                fittingkit._wrench(acc, name, f, Y_AXIS * float(np.clip(push, -LEAD_MAX, LEAD_MAX)), bottom)
        if in_line and depth > seat - DETENT_FROM and not self.pressed():
            if depth >= seat - SEAT_TOL:
                self.locked[name] = True
                self.clicks[name] += 1
                self._note(f"{self._n / 240:7.2f}s {name} clicks into the pump house, locked")
                self._hold(name, f, acc)
                return
            back = min(DETENT_MAX, DETENT_K * (depth - (seat - DETENT_FROM)))
            fittingkit._wrench(acc, name, f, -X_AXIS * back, bottom)

    def _hold(self, name: str, f, acc: dict) -> None:
        """Hold a locked refill on its seat: a stiff spring and damper on its centre of mass's place and on its turn."""
        mass = OLD_REFILL.mass if name == "old_refill" else NEW_REFILL.mass
        spec = OLD_REFILL if name == "old_refill" else NEW_REFILL
        com_local = np.asarray(spec.com, float)
        target = np.array([XS, 0.0, SEAT_Z]) + self.origin + com_local
        com = f.world(com_local)
        com_vel = f.vel + np.cross(f.spin, com - f.pos)
        k = min(HOLD_K, mass / (4.0 * HOLD_DT ** 2))
        force = -k * (com - target) - 2.0 * math.sqrt(k * mass) * com_vel
        n = np.linalg.norm(force)
        if n > HOLD_MAX:
            force *= HOLD_MAX / n
        fittingkit._wrench(acc, name, f, force, com)
        # the turn back to upright and square: the rotation vector of its rotation from the seat's, per body axis
        r = np.asarray(f.rot)
        angle = math.acos(float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)))
        turn = np.zeros(3)
        if angle > 1e-6:
            turn = angle * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2.0 * math.sin(angle))
        inertia = np.asarray(spec.inertia, float)
        k_turn = np.minimum(HOLD_TURN_K, inertia / (4.0 * HOLD_DT ** 2))
        c_turn = np.minimum(HOLD_TURN_C, 2.0 * np.sqrt(k_turn * inertia))
        local = -k_turn * (r.T @ turn) - c_turn * (r.T @ np.asarray(f.spin, float))
        fittingkit._torque(acc, name, r @ local)

    def _cover(self, f) -> None:
        self.cover_deg = cover_open_deg(f.rot)
        if not self.cover_latched and self.cover_deg < LATCH_DEG:
            self.cover_latched = True
            self._note(f"{self._n / 240:7.2f}s the cover latches")
        if self.cover_latched:
            self.poses["cover"] = (HINGE_AT + self.origin, (0.0, 0.0, 0.0, 1.0))

    def _write_trace(self) -> None:
        """Debug only (SOAP_TRACE=<file>): every 0.1 s, the latch, each refill, the cover and both TCPs (mm)."""
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import link_poses, tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                fingers = 1000 * np.asarray(scene[name].data.joint_pos[0, -2:].cpu())
                g = link_poses(scene[name])["gripper"][:3, :3]
                pt, fl = g[:, 2], g[:, 0]  # the hand's pointing and its fingers' line
                tcps += (f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
                         f" f ({fingers[0]:5.1f},{fingers[1]:5.1f}) pt ({pt[0]:5.2f},{pt[1]:5.2f},{pt[2]:5.2f})"
                         f" fl ({fl[0]:5.2f},{fl[1]:5.2f},{fl[2]:5.2f})")
        parts = []
        for n in REFILLS:
            p = 1000 * (self.frames[n].pos - self.origin)
            d, o, u, t = self.offset[n]
            yaw = math.degrees(math.atan2(self.frames[n].rot[1, 0], self.frames[n].rot[0, 0]))
            parts.append(f"{n[:3]} ({p[0]:6.1f},{p[1]:6.1f},{p[2] - 1000 * TOP:6.1f}) in {1000 * d:5.1f} off "
                         f"{1000 * o:5.1f} up {1000 * u:5.1f} tilt {t:4.1f} yaw {yaw:5.1f} l{int(self.locked[n])}")
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s latch {1000 * self.latch_in:4.1f} cover {self.cover_deg:5.1f} "
                     f"c{int(self.cover_latched)} | " + " | ".join(parts) + f" |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


MODEL = fittingkit.add_model(PumpHouse())

# -- scoring ---------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9
REST_SPEED = 0.03  # m/s


def old_refill_out() -> bool:
    """The empty refill let go from the pump house and out of it: its collar in front of the pump house, or off the
    groove's line, or lifted clear of the groove."""
    if MODEL.locked["old_refill"]:
        return False
    depth, off, up, _ = MODEL.offset["old_refill"]
    return depth < -COLLAR_R - 5.0 * MM or abs(off) > 0.06 or up > 0.05 or up < -0.03


def old_refill_in_box() -> bool:
    """Out of the dispenser, in the recycling box (its centre of mass over the box's inside and under 5 cm over its
    rim) and at rest."""
    f = MODEL.frames.get("old_refill")
    if f is None or not old_refill_out():
        return False
    com = f.world(OLD_REFILL.com) - MODEL.origin
    inside = np.all(np.abs(com[:2] - BOX_AT[:2]) < BOX_IN / 2) and com[2] < BOX_AT[2] + BOX_H + 0.05
    return bool(inside and np.linalg.norm(f.vel) < REST_SPEED)


def new_refill_pushed_in() -> bool:
    """The new refill's collar at least 25 mm into the pump house, on the groove's line and floor and upright."""
    depth, off, up, tilt = MODEL.offset["new_refill"]
    return MODEL.locked["new_refill"] or (depth >= PUSHED_IN and abs(off) < IN_Y and -2.0 * MM < up < IN_Z
                                          and tilt < IN_TILT)


def _box_gap(points: np.ndarray, frame, centre, size) -> float:
    """The smallest distance (m) from ``points`` to a box (in a body's frame); negative inside."""
    local = (points - frame.world(centre)) @ frame.rot
    q = np.abs(local) - np.asarray(size, float) / 2
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(np.max(q, axis=1), 0.0)
    return float((outside + inside).min())


def hands_clear() -> bool:
    """No gripper point within 2 cm of either refill or the cover; unknown offline counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0:
        return True
    for name in REFILLS:
        f = MODEL.frames.get(name)
        if f is not None and _box_gap(pts, f, (0.0, 0.0, (COLLAR_H + BOTTLE_H) / 2),
                                      (BOTTLE_X, BOTTLE_Y, COLLAR_H + BOTTLE_H)) < CLEAR:
            return False
    f = MODEL.frames.get("cover")
    return f is None or _box_gap(pts, f, (-COVER_T / 2, 0.0, COVER_H / 2), (COVER_T, COVER_W, COVER_H)) >= CLEAR


def done_but_hands() -> bool:
    return MODEL.locked["new_refill"] and old_refill_in_box() and MODEL.cover_latched


def success(objects) -> bool:
    """The new refill clicked into the pump house, the empty one in the recycling box, the cover latched shut, and both
    grippers clear. Must hold for 1 s."""
    return done_but_hands() and hands_clear()


def score(objects) -> float:
    """0.9 for the new refill locked, the empty one in the recycling box and the cover latched (1.0 once success() has
    held 1 s, the hands clear); 0.6 for the new refill locked with the empty one out (the cover not latched, or the empty
    one not in the box); 0.35 for the new refill pushed into the pump house but not clicked, with the empty one out;
    0.15 for the empty one out; 0 otherwise."""
    if done_but_hands():
        return COMPLETE_SCORE
    if not old_refill_out():
        return 0.0
    if MODEL.locked["new_refill"]:
        return 0.6
    return 0.35 if new_refill_pushed_in() else 0.15


# -- the scripted demo ---------------------------------------------------------------------------------------------
# The right hand, pointing at the dispenser with its fingers closing across the refill (along y), holds the empty
# refill; the left hand's closed fingertips push the release latch in; the right draws the refill out and drops it in
# the recycling box. The left takes the new refill out of its case and pushes its collar back into the pump house until
# it clicks, then, from beside the cover, pushes its underside up with the flat of its fingers until it latches. Both
# hands move clear.
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
PAD_TIP = 13.0 * MM


def _pointing(direction, finger_line) -> np.ndarray:
    """A TCP orientation: the fingers pointing along ``direction``, closing along ``finger_line`` (the TCP's x)."""
    z = np.asarray(direction, float)
    z /= np.linalg.norm(z)
    x = np.asarray(finger_line, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# WHY the finger line along -y: built from the straight-down grip turned to point forward, both arms reach a refill's
# grip anywhere from 16 to 30 cm above the table, from y -0.25 to 0.20 (grid.py)
R_HOLD = _pointing(X_AXIS, -Y_AXIS)
GRIP_UP = 0.130  # the TCP this far above the empty refill's collar bottom (the fingers on the bottle's middle)
GRIP_UP_NEW = 0.130  # and the full one's: 3 cm above its centre of mass, so it hangs plumb in the fingertips
# WHY above it: held level with its centre of mass the fingertips' small contact gives it nothing to right it, and the
# refill, tipped 7 degrees lifting it out of its case, reached the groove 14 degrees over and stuck (run soap21); at
# 0.75 kg it swung 24 to 31 degrees held 3 cm above, but that was the arm giving under it (runs soap16, soap18)
# WHY 13 cm, not 11: the left hand pushing the latch under the refill came up against the right hand holding it 11 cm up
# (run soap2: both stalled 2 to 9 cm off the plan from 4.5 s)
GRIP_IN = 0.038  # and this far behind its front: the fingertips 51 mm along its sides, over its middle
# WHY 51 mm deep: only the fingertips squeeze (the fingers' inner faces open out towards their base); 60 mm deep, they
# held the full refill 1.6 cm behind its centre of mass and it swung 38 degrees in the grip (run soap11)
# WHY the TCP off the refill's middle: only one YAM finger closes; the other stays at its open stop, 47 mm from the TCP,
# and the part is pushed against it, so a held refill's middle ends 4.5 mm (47 less half its 85 mm) towards that
# finger. For this grip that finger is on the robot's right (-y): squeezing, the hand moved 10 mm to +y (runs soap8,
# soap9: planned the other way round, the squeeze fought the arm and pushed the hand 3 cm back off the bottle)
IN_HAND = 47.0 * MM - BOTTLE_Y / 2
HELD_Y = -IN_HAND  # a held refill's middle from the TCP, along y
APPROACH_Y = -IN_HAND + 2.0 * MM  # coming round it: the open finger 2 mm off its side, the closing one 6.5 mm off
# WHY so, not centred: closing with the open finger off the bottle's side, the closing finger shoved a free refill into
# its case's wall and tipped it (runs soap10 to soap12)
SAG = 0.011  # an arm sags this much holding the full refill [estimate: 11 mm at about 3 N, step-stool runs]
PUSH_PAST = 12.0 * MM  # the new refill's collar aimed this far past the click (the detent and the arm's give)
# WHY 12 mm: aimed 10, 12 or 22 mm past, the collar stopped at 42 mm in either way, short of a 6 N click 1 mm from the
# seat (runs soap23, soap24, soap26; see DETENT_MAX); with the click at 3 N, 2 mm short, it clicked (run soap27)

# The left hand pushes the latch with its closed fingertips, pointing 20 degrees down and 70 degrees in from its own
# side, its fingers closing across.
# WHY down and closing across: the hand flares behind its fingertips (46 x 67 mm 6 cm back, the gripper
# 100 mm along the finger line 8 cm back; hull_slices.py); pointing 20 degrees down with the fingers closing up and
# down, the gripper went through the open cover (run sd1: the hand stopped 4.5 cm short, on the cover)
PRESS_DOWN, PRESS_ACROSS = math.radians(20.0), math.radians(70.0)
_d = np.array([math.cos(PRESS_DOWN) * math.cos(PRESS_ACROSS), -math.cos(PRESS_DOWN) * math.sin(PRESS_ACROSS),
               -math.sin(PRESS_DOWN)])
# WHY 70 degrees across, from the robot's left: pointing straight at the latch, the left forearm met the right one,
# holding the refill above it, by up to 25 mm (all links' hulls; runs soap2 to soap4 stalled there); turned 70 degrees
# the arms are over 20 mm apart
# WHY 20 degrees down: pointing 35 degrees down, the hand rose into the seated refill's bottom corner and stopped 0.8 mm
# into the latch (run soap6)
R_PRESS = _pointing(_d, np.cross(_d, Z_AXIS))
LATCH_FACE = np.array([XF - LATCH_OUT, 0.0, TOP + LATCH_Z])
PRESS_PAST = 7.0 * MM  # the fingertips aimed this far into the latch (it unlocks at 2.5, stops at 5)

# The left hand swings the cover shut from beside it: pointing across (-y) from outside its edge, its fingers closed
# and closing along the cover's normal, the flat of its fingers pushes on the cover's outside, 21 cm from the hinge,
# along the last 2 cm of its edge.
# WHY from beside it: under the open cover the hand's flare (it widens about 22 degrees behind the fingertips) runs into
# the cover wherever the fingertips touch it; from beside it, only the fingers are under the cover
EDGE_R = 0.21  # WHY 21 cm: nearer the hinge, the left arm cannot reach 2 cm under the open cover (reach.py)
EDGE_Y = COVER_W / 2 - 0.008  # the TCP: the fingers under the cover's last 21 mm (13 mm of them past the TCP)
FINGER_FACE = 5.0 * MM  # the closed fingers' half thickness at the TCP (hull_slices.py)
EDGE_INTO = 4.0 * MM  # the fingers aimed this far into the cover as they swing it
SHUT_PAST = 8.0 * MM  # and this far past shut at the end (6 mm left it 2 degrees open, short of its latch: run soap9)


def press_pose(depth: float, back: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose with its closed fingertips ``depth`` into the latch's face (or ``back`` away from it)."""
    tip = LATCH_FACE + X_AXIS * depth - _d * back
    return tip - _d * PAD_TIP, R_PRESS


def hold_pose(collar_at, reach: float = 0.0, extra_up: float = 0.0, grip_up: float = GRIP_UP
              ) -> tuple[np.ndarray, np.ndarray]:
    """A TCP's pose holding a refill whose collar bottom is at ``collar_at``: ``reach`` = 0 as held (the refill against
    the open finger), IN_HAND coming round it (the open finger 2 mm off its side), IN_HAND / 2 letting it go (2.25 mm
    off)."""
    side = -HELD_Y if reach == 0.0 else (-APPROACH_Y if reach == IN_HAND else -HELD_Y * (1.0 - reach / IN_HAND))
    p = np.asarray(collar_at, float) + np.array([-BOTTLE_X / 2 + GRIP_IN, side, grip_up + extra_up])
    return p, R_HOLD


def cover_point(open_deg: float, r: float) -> tuple[np.ndarray, np.ndarray]:
    """A point on the cover's outside ``r`` up from the hinge, and its outward normal, with the cover ``open_deg``
    open."""
    rot = rot_y(-math.radians(open_deg))
    p = HINGE_AT + rot @ np.array([-COVER_T, 0.0, r])
    return p, rot @ -X_AXIS


def edge_pose(open_deg: float, into: float = EDGE_INTO, out: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The left TCP's pose with the flat of its closed fingers on the cover's outside (``into`` it), ``out`` further
    out from its edge."""
    c, n = cover_point(open_deg, EDGE_R)
    p = c + (FINGER_FACE - into) * n
    p[1] = EDGE_Y + out
    return p, _pointing(-Y_AXIS, n)


def _measured_hand(pt, fl) -> np.ndarray:
    """A hand orientation from its pointing and its fingers' line, as the trace logs them."""
    z = np.asarray(pt, float) / np.linalg.norm(pt)
    x = np.asarray(fl, float) - z * (np.asarray(fl, float) @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# Holding a 0.75 kg refill out in front, the right arm gave 6 cm and about 25 degrees (run soap18); aimed against
# that give it stopped 9 to 11 cm short at its joints' force limits (run soap19). Kept so a heavier refill could be
# corrected the same way:
# With the refill at 0.30 kg (see FULL_MASS) the carry is only turned, not moved: carried, the hand rolled about 6
# degrees about its pointing (its fingers' line (0.0, -1.0, 0.10)), the refill leaned with it, and its collar jammed in
# the groove's mouth and levered the bottle 16 degrees over (run soap22); the carry's turn is rolled back by as much.
CARRY_DP = np.zeros(3)
CARRY_R = axis_angle_matrix(X_AXIS, math.asin(0.10))
LIFT_DP = np.zeros(3)


def carried(pose) -> tuple[np.ndarray, np.ndarray]:
    """A target for the right hand carrying the full refill, moved and turned against the arm's give."""
    return pose[0] + CARRY_DP, CARRY_R @ pose[1]


GRIP_S = 2.0  # s to open or close the fingers: at 1.5 s the gripper's step is 99.9% of the step guard


def demo():
    right, left = [], []

    def both(dur, rpose, rf, lpose, lf, label):
        right.append(RotWaypoint(dur, tuple(rpose[0]), rf, label, rpose[1]))
        left.append(RotWaypoint(dur, tuple(lpose[0]), lf, label, lpose[1]))

    seat = np.array([XS, 0.0, SEAT_Z])
    fwd = lambda d: np.array([-d, 0.0, 0.0])  # noqa: E731
    up = lambda h: np.array([0.0, 0.0, h])  # noqa: E731
    l_wait = press_pose(0.0, back=0.10)
    l_side = (np.array([0.22, 0.20, TOP + 0.22]), R_PRESS)  # out on the left, clear of the right arm
    # WHY 13 cm to the right: at 10 cm, the cover's top corner swung through the right hand's fingers (plan_check.py)
    r_park = (np.array([0.18, -0.13, TOP + 0.30]), R_HOLD)

    # 1. the right hand holds the empty refill; the left pushes the latch; the right draws the refill out
    # WHY high first: straight from the ready pose, a hand's open fingers sweep through the refill standing in its case
    # (run sd1 tipped one 12 degrees)
    both(1.8, (np.array([0.30, -0.12, TOP + 0.30]), R_HOLD), OPEN, l_side, CLOSED, "turn to point forward")
    both(1.6, hold_pose(seat + fwd(0.075), IN_HAND), OPEN, l_wait, CLOSED, "in front of the empty refill")
    # WHY stop and settle short of it, then slide in slowly: the open fingers leave 4.5 mm either side of the refill,
    # and sliding straight in the right hand lagged 6 mm sideways, caught the bottle and rode up over it (run soap3)
    both(1.0, hold_pose(seat + fwd(0.062), IN_HAND), OPEN, l_wait, CLOSED, "just short of it")
    both(2.5, hold_pose(seat, IN_HAND), OPEN, l_wait, CLOSED, "round the empty refill")
    both(GRIP_S, hold_pose(seat), CLOSED, l_wait, CLOSED, "hold the empty refill")
    both(1.2, hold_pose(seat), CLOSED, press_pose(0.0, back=0.015), CLOSED, "finger to the latch")
    both(1.0, hold_pose(seat), CLOSED, press_pose(PRESS_PAST), CLOSED, "push the release latch")
    both(0.5, hold_pose(seat), CLOSED, press_pose(PRESS_PAST), CLOSED, "hold the latch in")
    both(1.0, hold_pose(seat + fwd(0.012)), CLOSED, press_pose(PRESS_PAST), CLOSED, "draw it off its seat")
    both(1.0, hold_pose(seat + fwd(0.012)), CLOSED, press_pose(0.0, back=0.04), CLOSED, "let the latch go")
    both(1.5, hold_pose(seat + fwd(0.012)), CLOSED, l_side, CLOSED, "left hand out to its side")
    # 2. the right drops the empty refill in the recycling box
    # WHY 9.5 cm out before lifting: the top cap is over the refill until its back is past the dispenser's front
    both(1.5, hold_pose(seat + fwd(0.095)), CLOSED, l_side, CLOSED, "draw it out")
    lifted = seat + fwd(0.095) + up(0.03)
    both(0.8, hold_pose(lifted), CLOSED, l_side, CLOSED, "lift it clear")
    # in front of the new refill's case (towards the arms), then out over the bin beyond it
    both(1.2, hold_pose(np.array([0.21, -0.05, lifted[2]])), CLOSED, l_side, CLOSED, "past the front of the case")
    over_box = np.array([BOX_AT[0], BOX_AT[1], lifted[2]])
    both(1.4, hold_pose(over_box), CLOSED, l_side, CLOSED, "over the recycling bin")
    both(GRIP_S, hold_pose(over_box, IN_HAND / 2), OPEN, l_side, CLOSED, "drop the empty refill in the bin")
    # 3. the right takes the new refill out of its case and pushes it into the pump house
    new_in = NEW_AT.copy()
    # WHY from the front, as the empty refill: coming down round it from above, the hand was 4 mm off, a finger landed
    # on the bottle's top edge and tipped it in its case (run soap10); from the front its rounded edges steer the fingers
    both(1.2, hold_pose(new_in + fwd(0.075) + up(0.12), IN_HAND, grip_up=GRIP_UP_NEW), OPEN, l_side, CLOSED, "up over the gap to the case")
    both(1.2, hold_pose(new_in + fwd(0.075), IN_HAND, grip_up=GRIP_UP_NEW), OPEN, l_side, CLOSED, "in front of the new refill")
    both(1.0, hold_pose(new_in + fwd(0.062), IN_HAND, grip_up=GRIP_UP_NEW), OPEN, l_side, CLOSED, "just short of the new refill")
    both(1.6, hold_pose(new_in, IN_HAND, grip_up=GRIP_UP_NEW), OPEN, l_side, CLOSED, "round the new refill")
    both(GRIP_S, hold_pose(new_in, grip_up=GRIP_UP_NEW), CLOSED, l_side, CLOSED, "hold the new refill")
    out_of_case = new_in + up(CASE_WALL_H + 0.015)
    lift_pose = hold_pose(out_of_case, extra_up=SAG, grip_up=GRIP_UP_NEW)
    both(2.2, (lift_pose[0] + LIFT_DP, lift_pose[1]), CLOSED, l_side, CLOSED, "lift it out of its case")
    mouth = seat + fwd(XS - XF + 0.030) + up(3.0 * MM)
    # WHY 3 cm further out and 1.5 cm up before it comes in line: brought sideways to the groove's mouth, the collar
    # caught the groove's side wall and the bottle's back the pump house's corner, and the hand stopped 6 cm short
    # (run soap20); from here it goes straight back into the groove
    high = mouth + fwd(0.030) + up(0.015)
    both(2.6, carried(hold_pose(high, extra_up=SAG, grip_up=GRIP_UP_NEW)), CLOSED, l_side, CLOSED,
         "in front of the pump house")
    both(1.2, carried(hold_pose(mouth, extra_up=SAG, grip_up=GRIP_UP_NEW)), CLOSED, l_side, CLOSED, "pump to the groove")
    home = seat + fwd(-PUSH_PAST) + up(3.0 * MM)
    both(1.6, carried(hold_pose(home, extra_up=SAG, grip_up=GRIP_UP_NEW)), CLOSED, l_side, CLOSED,
         "push it into the pump house")
    both(0.5, carried(hold_pose(home, extra_up=SAG, grip_up=GRIP_UP_NEW)), CLOSED, l_side, CLOSED, "hold it home")
    both(GRIP_S, hold_pose(seat, IN_HAND / 2, grip_up=GRIP_UP_NEW), OPEN, l_side, CLOSED, "let the new refill go")
    beside = edge_pose(OPEN_DEG, into=-0.015, out=0.05)
    # WHY a turn on the way: turning from one pose to the other while it moved, the left arm lost the path (reach.py)
    both(1.5, hold_pose(seat + fwd(0.09), IN_HAND / 2, grip_up=GRIP_UP_NEW), OPEN, (np.array([0.20, 0.12, TOP + 0.24]), beside[1]), CLOSED,
         "fingers off it, left hand turns to point across")
    # 4. the left swings the cover shut from beside it
    both(2.0, r_park, OPEN, (beside[0] + up(0.04), beside[1]), CLOSED, "right hand away, left beside the cover")
    both(1.2, r_park, OPEN, edge_pose(OPEN_DEG, into=-0.015), CLOSED, "fingers under the cover's edge")
    both(1.0, r_park, OPEN, edge_pose(OPEN_DEG), CLOSED, "onto the cover")
    for a, b in ((OPEN_DEG, 50.0), (50.0, 30.0), (30.0, 12.0), (12.0, 0.0)):
        both(1.0 + (a - b) / 30.0, r_park, OPEN, edge_pose(b), CLOSED, f"swing the cover up to {b:.0f} degrees")
    both(1.0, r_park, OPEN, edge_pose(0.0, into=SHUT_PAST), CLOSED, "push it shut")
    both(0.5, r_park, OPEN, edge_pose(0.0, into=SHUT_PAST), CLOSED, "hold it shut")
    both(1.2, r_park, OPEN, edge_pose(0.0, into=-0.03, out=0.04), CLOSED, "hand back from it")
    l_clear = (np.array([0.24, 0.22, TOP + 0.26]), R_HOLD)
    both(1.8, r_park, OPEN, l_clear, CLOSED, "hands clear")
    both(0.6, r_park, OPEN, l_clear, CLOSED, "wait")
    return {"left": left, "right": right}
