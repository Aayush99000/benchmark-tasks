"""The buildable stool's dimensions and its latch's spring, in mm, N and degrees. Standard library only.

One source for the printed parts (``build_parts.py``), the drawing, and the tests that hold the simulated stool
(``folding_step_stool.py``) to the same numbers.

Frames. The top's: origin at the top's middle, x across the stool (its depth, 200 mm; +x is the side away from the
arms), y along it (246 mm; the legs at the -y and +y ends), z up. The +y leg's: origin on its hinge axis, axes those of
the top with the leg folded (pointing -y, in under the top); the leg opens turning about +x. Everything at the -y end
is the +y end mirrored in y. Angles about the hinge axis are measured in the y-z plane from +y towards +z.
"""

from __future__ import annotations

import math

# -- the top (printed) ------------------------------------------------------------------------------------------------
TOP_D, TOP_L, TOP_T = 200.0, 246.0, 18.0  # across (x), along (y), thick (z)
RIB_W, RIB_T, RIB_H = 70.0, 14.0, 20.0  # a grip rib (printed separately): along x, along y, high
RIB_Y = TOP_L / 2 - RIB_T / 2 - 1.0  # 115: the ribs' middles
RIB_SCREW_X = 20.0  # two M3 screws per rib, at x = +-20
BALLAST_Y, BALLAST_D, BALLAST_DEPTH = 50.0, 32.0, 6.0  # two recesses under the top for steel washers

# -- the hinges ---------------------------------------------------------------------------------------------------------
HINGE_Y = 105.0  # each leg's hinge axis, from the top's middle
KNUCKLE_R = 8.0  # the leg's knuckle round the axis; the axis is this far under the top
HINGE_Z = -(TOP_T / 2 + KNUCKLE_R)  # -17: the axis in the top's frame
ROD_D, ROD_HOLE_D = 5.0, 5.3  # M5 threaded rod; its clearance hole in the knuckle and the ears
EAR_X = ((-86.0, -76.0), (86.0, 95.0))  # the top's two ears per hinge, either side of the knuckle (1 mm washer gaps)
KNUCKLE_X = (-75.0, 85.0)  # the leg's knuckle; past the panel (x 75) it carries the latch's wedge

# -- a leg (printed panel and knuckle; wooden foot rail) -----------------------------------------------------------------
LEG_L, LEG_W, LEG_T = 100.0, 150.0, 10.0  # hinge axis to the foot's far face; wide (x); the panel's thickness
LEG_Z = 1.0  # the panel's middle, above the axis (leg frame)
FOOT_L, FOOT_T = 340.0, 16.0  # the foot rail: a 16 mm square pine strip, 340 mm long
OPEN_DEG = 100.0  # the leg's stop: 10 degrees past straight down

# -- the latch (one per leg, at the +x side of the stool) ---------------------------------------------------------------
# A printed bolt slides along x in a channel in the top, pushed towards -x by a compression spring. Its tip is a chisel:
# a knife edge on the side the wedge leaves from, a 45 degree bevel on the side it comes from. The leg carries a wedge
# whose face rises 3 mm (towards +x) over 18 degrees and then drops: opening the leg, the wedge pushes the bolt back
# (and the spring pushes the leg back), and once the wedge's high end passes the knife edge the bolt drops behind it,
# 3.5 mm deep. Folding the leg back then meets the bolt's flat side: locked. Pulling the knob 4.5 mm releases it.
BOLT_W, BOLT_H, BOLT_LEN = 5.0, 6.0, 24.0  # across the motion (y), radial (z), along x
BOLT_R = (15.0, 21.0)  # the bolt's span from the hinge axis, above it
KNIFE_Y = 0.0  # the knife edge's y from the hinge axis: straight above it, on a radius, so the whole edge meets the
# wedge's face at the same height (2.5 mm off the axis it rode the wedge's high end for the last 2.5 degrees: CAD check)
BOLT_Y = KNIFE_Y + 5.0 / 2  # the bolt's middle, from the hinge axis (the bolt is 5 mm wide, on the knife edge's +y side)
BOLT_BEVEL = 5.0  # the bevel rises 5 mm across the bolt's 5 mm width: 45 degrees
BOLT_TIP_X = 81.5  # the knife edge's x with the bolt out (rest, s = 0)
CHANNEL_CLEAR = 0.15  # each side of the bolt in its channel
POCKET_X = (76.0, 86.0)  # the pocket under the top the wedge turns in; the channel opens into it at x 86
POCKET_R = 22.5
WEDGE_X0 = 77.0  # the wedge's back face
WEDGE_R = (5.0, 21.5)
WEDGE_LOW_X, WEDGE_HIGH_X = 82.0, 85.0  # its face's x at its leading (low) and trailing (high) ends
WEDGE_SPAN_DEG = 18.0
WEDGE_TRAIL_DEG = 90.0 - 98.0  # -8: the high end's radial face, in the leg's frame: it passes the knife edge at 98
WEDGE_LEAD_DEG = WEDGE_TRAIL_DEG + WEDGE_SPAN_DEG  # 10: its low end's face, which meets the pocket's end wall at 100
POCKET_DEG = (-12.0, WEDGE_LEAD_DEG + OPEN_DEG)  # the pocket's sector; its end wall (110) is the leg's stop
CHANNEL_X = (POCKET_X[1], 105.5)  # the bolt's square channel
BORE_D, BORE_X = 8.0, (105.5, 117.5)  # the spring's bore behind the bolt
WALL_X = (117.5, 119.5)  # the bore's end wall, with the screw's 3.4 mm hole; the knob sits outside it
BOSS_X, BOSS_Y = (100.0, 119.5), (-7.0, 9.5)  # the boss on the top's +x side that holds the channel and the bore
# (its y span from the hinge axis)
KNOB_D, KNOB_X = 14.0, (119.5, 126.5)  # a printed cap on the M3 screw's head: the release knob
RELEASE_MM = 4.5  # how far the knob is pulled to release
BOLT_AXIS_Z = HINGE_Z + sum(BOLT_R) / 2  # +1: the bolt's middle in the top's frame

# The spring: Lee Spring LC 026C 05 M, music wire, outside diameter 6.1 mm, free length 15.88 mm, rate 2.189 N/mm,
# solid height 5.16 mm (leespring.com product page, read 2026-11-19).
SPRING_FREE, SPRING_RATE, SPRING_SOLID = 15.88, 2.189, 5.16
WASHER_T = 0.5  # an M3 washer between the bolt and the spring
SPRING_INSTALLED = BORE_X[1] - BORE_X[0] - WASHER_T  # 11.5 mm with the bolt out
SPRING_PRELOAD_N = SPRING_RATE * (SPRING_FREE - SPRING_INSTALLED)  # 9.59 N

# The latch as the leg turns: the bolt's push back (s, mm) is the wedge's face under the knife edge, from 0.5 mm at 80
# degrees to 3.5 mm at 98, where it drops. (Just before 80 the wedge's low end meets the bolt's bevel, 0.5 mm from the
# knife edge, and pushes it the first 0.5 mm; build_parts.py checks the whole travel against the CAD.)
SPRING_FROM_DEG, LATCH_DEG = 80.0, 98.0
S_AT_FROM, S_AT_LATCH = WEDGE_LOW_X - BOLT_TIP_X, WEDGE_HIGH_X - BOLT_TIP_X  # 0.5 and 3.5 mm


def bolt_push_mm(opening_deg: float) -> float:
    """How far (mm) the wedge holds the bolt back with the leg open ``opening_deg``; 0 short of the wedge and once the
    bolt has dropped behind it."""
    if not SPRING_FROM_DEG <= opening_deg < LATCH_DEG:
        return 0.0
    frac = (opening_deg - SPRING_FROM_DEG) / (LATCH_DEG - SPRING_FROM_DEG)
    return S_AT_FROM + frac * (S_AT_LATCH - S_AT_FROM)


def spring_force_n(push_mm: float) -> float:
    return SPRING_PRELOAD_N + SPRING_RATE * push_mm


def fold_back_torque_nm(opening_deg: float) -> float:
    """The torque (N m) folding the leg back while the bolt rides the wedge, from the spring's force and the wedge's
    rise per radian (virtual work; friction left out). 0 outside 80-98 degrees."""
    if not SPRING_FROM_DEG <= opening_deg < LATCH_DEG:
        return 0.0
    rise_per_rad = (S_AT_LATCH - S_AT_FROM) / math.radians(LATCH_DEG - SPRING_FROM_DEG) / 1000.0  # m per rad
    return spring_force_n(bolt_push_mm(opening_deg)) * rise_per_rad


# -- real trials --------------------------------------------------------------------------------------------------------
LOAD_N = 40.0
LOAD_KG = LOAD_N / 9.81  # 4.08 kg set on the top's middle
LOCK_CHECK_N = 5.0  # the push inwards on a foot rail's middle that a locked leg holds and an unlocked one doesn't
