"""Geometry and measurements for changing the dust bag in a canister vacuum.

On the bench: a bagged canister vacuum, its front toward the arms and its dust-compartment lid swung
open on its rear hinge. In the compartment the full paper bag hangs from its cardboard collar, which
stands in the collar holder on the inside of the front wall: two upright channels and a stop, so the
collar slides straight down over the inlet and its rubber gasket seals round the inlet's mouth. To the
left stands the open carton of new bags, one standing in it with its collar's pull tab up; to the
right an empty wastebasket. The old bag comes up out of its holder by its tab and goes in the
wastebasket; the new one goes down the channels to the stop.
Frame: metres, x away from the arm bases, y left, z up; TOP is the bench.
"""
from __future__ import annotations

import math

import numpy as np

from simkit import Hinge, Part, Piece

TOP = 0.7475

BODY = (0.62, 0.64, 0.66)            # silver-grey ABS shell
BODY_DARK = (0.16, 0.17, 0.18)
LID_RED = (0.66, 0.10, 0.12)         # the lid's red ABS
RUBBER = (0.06, 0.06, 0.06)
GREY_PLASTIC = (0.55, 0.57, 0.60)
CARD = (0.86, 0.80, 0.66)            # the bag collar's cardboard
FLEECE = (0.93, 0.93, 0.90)          # the bag's white synthetic fleece
FLEECE_DUST = (0.62, 0.60, 0.56)     # dust showing through a full bag
BOX_WHITE = (0.96, 0.96, 0.95)
BOX_BLUE = (0.10, 0.30, 0.62)
BIN_GREY = (0.42, 0.44, 0.47)

# -- the canister vacuum: 470 L x 290 W x 255 H mm (free on its wheels) -------------------------------------------
YC = 0.0                             # its centreline
FRONT_X = 0.240                      # the front bumper (toward the arms)
BACK_X = 0.710
HALF_W = 0.145
WALL_T = 0.015                       # the compartment's front wall
FRONT_IN = FRONT_X + 0.010 + WALL_T  # inner face of the front wall, x 0.265
PART_X = 0.440                       # the partition (motor filter) at the compartment's back
IN_HALF_W = 0.125                    # the compartment inside, y +-125 mm
FLOOR_Z = TOP + 0.030
RIM_Z = TOP + 0.205                  # top edge of the compartment walls
HINGE_X = PART_X + 0.012             # the lid's hinge line, over the partition
LID_OPEN_DEG = 100.0                 # as left: swung open past upright, resting on its stop
LID_COM_CLOSED = (-0.130, 0.0, 0.010)  # the lid's centre of mass from its hinge, laid out shut
LID_DAMP = 0.008                     # N m s / deg on its hinge: the lid swings shut steadily, not slammed
LID_LATCH_DAMP = 1.0e4               # latched: the hinge held by damping alone
INLET_Z = TOP + 0.130                # the inlet's axis through the front wall
INLET_R = 0.019                      # 38 mm inlet mouth

# the collar holder: two upright channels on the front wall's inner face and a stop under them
COLLAR = (0.002, 0.120, 0.125)       # x thick, y wide, z tall: the collar plate
SLOT = 0.012                         # the channel's gap for the 2 mm collar (5 mm play either way)
STOP_D = SLOT                        # the stop ledge spans the channel: the collar and its pack land on it
LIP = (0.004, 0.012)                 # each channel's front lip: x thick, y wide (covers 4-6 mm of the collar's edge)
LIP_Y = 0.062                        # the lips' centres from the centreline (they cover |y| 0.056-0.068)
COLLAR_X = FRONT_IN + SLOT / 2       # the collar's mid-plane in its channel, x 0.267
SEAT_Z = TOP + 0.070                 # the seated collar's bottom edge, on the stop
LIP_TOP = SEAT_Z + 0.030             # the channels are short guides: their lips run 30 mm up from the stop
TAB = (0.002, 0.040, 0.075)          # the tall pull tab (grip handle) above the collar's top edge
HOLE_Z = INLET_Z - SEAT_Z            # the gasket's centre above the collar's bottom edge (60 mm)
CARD_ON_PLASTIC = (0.60, 0.50)       # the collar card (on the holder's plastic and on the fingers' pads)
TAB_FRICTION = (1.0, 0.9)            # the grip tab's card on the fingers' rubber pads

SHELL_DARK = (0.22, 0.23, 0.25)      # the graphite lower shell and bumper
SEAM = (0.08, 0.08, 0.09)            # panel lines
ICON = (0.92, 0.92, 0.92)
HOSE = (0.10, 0.10, 0.11)            # the corrugated hose: dark valleys ...
HOSE_RIB = (0.40, 0.41, 0.43)        # ... and ribs catching the light
WAND = (0.80, 0.81, 0.83)            # the telescopic wand: brushed aluminium
WHEEL_SLIP = (0.15, 0.12)            # the wheels' contact: rolling resistance of a canister on a hard floor

VAC_ROOT_Z = TOP + 0.018             # the chassis plate's centre
VAC_POS = ((FRONT_X + BACK_X) / 2, YC, VAC_ROOT_Z)
VAC_MASS = 5.5


def _seg(x0, z0, x1, z1, width, thick, y=0.0, color=None, collide=True, inset=0.0):
    """A plank from (x0, z0) to (x1, z1) in the x-z plane, ``width`` across y: one facet of a curved shell."""
    a = math.atan2(z1 - z0, x1 - x0)
    L = math.hypot(x1 - x0, z1 - z0) + 0.002
    cx, cz = (x0 + x1) / 2 + inset * math.sin(a), (z0 + z1) / 2 - inset * math.cos(a)
    return (L, width, thick), (cx, y, cz), (0.0, -a, 0.0), color, collide


def _dir_rot(d):
    """(roll, pitch, yaw) turning a piece's x axis onto the direction ``d``."""
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    return (0.0, -math.asin(float(np.clip(d[2], -1, 1))), math.atan2(float(d[1]), float(d[0])))


def _spline(points, step):
    """Points every ``step`` metres along a Catmull-Rom curve through ``points``."""
    P = [np.asarray(p, float) for p in points]
    dense = []
    for i in range(len(P) - 1):
        p0 = P[i - 1] if i > 0 else P[i]
        p1, p2 = P[i], P[i + 1]
        p3 = P[i + 2] if i + 2 < len(P) else P[i + 1]
        for u in np.linspace(0, 1, 40, endpoint=False):
            dense.append(0.5 * (2 * p1 + (-p0 + p2) * u + (2 * p0 - 5 * p1 + 4 * p2 - p3) * u * u
                                + (-p0 + 3 * p1 - 3 * p2 + p3) * u ** 3))
    dense.append(P[-1])
    out, acc = [dense[0]], 0.0
    for a, b in zip(dense, dense[1:]):
        acc += float(np.linalg.norm(b - a))
        if acc >= step:
            out.append(b)
            acc = 0.0
    return out


def _corrugated(points, r, color, collide=False):
    """A corrugated hose along ``points``: alternate ribs and valleys, 7 mm pitch."""
    pts = _spline(points, 0.0035)
    out = []
    for k in range(1, len(pts)):
        d = pts[k] - pts[k - 1]
        c = (pts[k] + pts[k - 1]) / 2
        rib = k % 2 == 1
        rr = r if rib else r * 0.86
        out.append(Piece("cylinder", (rr, 0.0040, rr), tuple(float(v) for v in c), rot=_dir_rot(d), axis="X",
                         color=HOSE_RIB if rib else color, collide=collide))
    return out


# hood profile (x, z above the bench) from the lid's hinge over the motor to the back
HOOD = ((HINGE_X, 0.196), (0.490, 0.218), (0.530, 0.236), (0.575, 0.246), (0.620, 0.246), (0.660, 0.232),
        (0.690, 0.205), (0.706, 0.170), (0.712, 0.130))


def vacuum_object() -> Part:
    """The canister vacuum, free on its two rear wheels and two front castors (5.5 kg).

    Lower shell and bumper in graphite, the upper shell silver: the open dust compartment at the front
    with the collar holder on the inside of its front wall and the inlet socket through it, the motor
    hood behind it with the carry handle, the two foot pedals (power, cord rewind) and the exhaust
    grille on the back. Its root is the chassis plate. The wheels and castors are welded to it but meet
    the bench with almost no friction, standing in for wheels that roll at a touch.
    """
    rel = lambda x, y, z: (x - VAC_POS[0], y - YC, z - VAC_ROOT_Z)
    ps: list[Piece] = []
    add = ps.append
    W = 2 * HALF_W
    # -- lower shell: a graphite tub with rounded edges, the bumper round its front ---------------------
    add(Piece("box", (BACK_X - HINGE_X - 0.010, W - 0.030, 0.080), rel((HINGE_X + BACK_X) / 2, YC, TOP + 0.064),
              color=SHELL_DARK, collide=False))
    for s in (-1, 1):
        add(Piece("capsule", (0.015, BACK_X - FRONT_X - 0.060, 0.015), rel((FRONT_X + BACK_X) / 2 + 0.005, YC + s * (HALF_W - 0.015),
                                                                          TOP + 0.040), axis="X", color=SHELL_DARK))
    add(Piece("capsule", (0.018, W - 0.040, 0.018), rel(FRONT_X + 0.007, YC, TOP + 0.048), axis="Y", color=RUBBER))      # bumper
    for s in (-1, 1):
        add(Piece("sphere", (0.018, 0.018, 0.018), rel(FRONT_X + 0.007, YC + s * (HALF_W - 0.020), TOP + 0.048), color=RUBBER))
    add(Piece("box", (0.004, W - 0.06, 0.004), rel(FRONT_X + 0.006, YC, TOP + 0.078), color=SEAM, collide=False))
    # -- the dust compartment: floor, front wall, side walls, partition (silver upper shell) -------------
    add(Piece("box", (PART_X - FRONT_IN, 2 * IN_HALF_W, 0.006), rel((FRONT_IN + PART_X) / 2, YC, FLOOR_Z - 0.003), color=SHELL_DARK))
    fw_x0 = FRONT_X + 0.010
    add(Piece("box", (WALL_T, W - 0.016, RIM_Z - (TOP + 0.074)), rel(fw_x0 + WALL_T / 2, YC, (RIM_Z + TOP + 0.074) / 2), color=BODY))
    add(Piece("box", (WALL_T, W - 0.016, 0.052), rel(fw_x0 + WALL_T / 2, YC, TOP + 0.050), color=SHELL_DARK))
    add(Piece("capsule", (0.008, W - 0.040, 0.008), rel(fw_x0 + WALL_T / 2, YC, RIM_Z - 0.002), axis="Y", color=BODY, collide=False))
    for s in (-1, 1):
        yw = YC + s * (IN_HALF_W + HALF_W) / 2
        add(Piece("box", (PART_X - fw_x0 + 0.012, HALF_W - IN_HALF_W, RIM_Z - (TOP + 0.074)),
                  rel((fw_x0 + PART_X + 0.012) / 2, yw, (RIM_Z + TOP + 0.074) / 2), color=BODY))
        add(Piece("capsule", (0.010, PART_X - fw_x0, 0.010), rel((fw_x0 + PART_X) / 2, YC + s * (HALF_W - 0.010), RIM_Z - 0.006),
                  axis="X", color=BODY, collide=False))
        add(Piece("box", (PART_X - fw_x0 + 0.012, HALF_W - IN_HALF_W, 0.052), rel((fw_x0 + PART_X + 0.012) / 2, yw, TOP + 0.050),
                  color=SHELL_DARK))
        # panel line between the silver upper shell and the graphite lower shell, and a moulded side recess
        add(Piece("box", (BACK_X - FRONT_X - 0.050, 0.002, 0.004), rel((FRONT_X + BACK_X) / 2, YC + s * (HALF_W + 0.0005), TOP + 0.104),
                  color=SEAM, collide=False))
        add(Piece("box", (0.120, 0.002, 0.040), rel(0.345, YC + s * (HALF_W + 0.0005), TOP + 0.150), color=(0.56, 0.58, 0.60),
                  collide=False))
    add(Piece("box", (0.012, 2 * IN_HALF_W, RIM_Z - FLOOR_Z), rel(PART_X + 0.006, YC, (RIM_Z + FLOOR_Z) / 2), color=SHELL_DARK))
    # the motor filter behind its grille on the partition
    add(Piece("box", (0.002, 0.190, 0.120), rel(PART_X - 0.0005, YC, FLOOR_Z + 0.085), color=(0.85, 0.85, 0.83), collide=False))
    for k in range(9):
        add(Piece("box", (0.003, 0.196, 0.004), rel(PART_X - 0.0015, YC, FLOOR_Z + 0.030 + 0.012 * k), color=SHELL_DARK, collide=False))
    # -- the motor hood: a smooth curved silver shell over the motor, faceted along its profile ---------
    add(Piece("box", (BACK_X - HINGE_X - 0.010, W - 0.010, 0.120), rel((HINGE_X + BACK_X) / 2, YC, TOP + 0.130), color=BODY))
    for (x0, z0), (x1, z1) in zip(HOOD, HOOD[1:]):
        size, c, rot, _, _ = _seg(x0, TOP + z0, x1, TOP + z1, W - 0.030, 0.008, inset=0.004)
        add(Piece("box", size, rel(*c), rot=rot, color=BODY))
        for s in (-1, 1):    # rounded shoulders down each side
            sz, cc, rr, _, _ = _seg(x0, TOP + z0 - 0.012, x1, TOP + z1 - 0.012, 0.024, 0.024, inset=0.0)
            add(Piece("capsule", (0.012, sz[0], 0.012), rel(cc[0], YC + s * (HALF_W - 0.013), cc[2]), rot=(rr[0], rr[1], rr[2]),
                      axis="X", color=BODY, collide=False))
    for s in (-1, 1):        # the hood's sides
        add(Piece("box", (BACK_X - HINGE_X - 0.020, 0.004, 0.110), rel((HINGE_X + BACK_X) / 2, YC + s * (HALF_W - 0.002), TOP + 0.165),
                  color=BODY, collide=False))
    # panel line where the hood meets the compartment, and the seam round the hood's crown
    add(Piece("box", (0.004, W - 0.020, 0.004), rel(HINGE_X + 0.004, YC, TOP + 0.200), color=SEAM, collide=False))
    # the body's hinge knuckles between the lid's, and the latch's keeper on the front wall's top
    for yk in (-0.070, 0.010, 0.090):
        add(Piece("cylinder", (0.0085, 0.040, 0.0085), rel(HINGE_X, YC + yk, RIM_Z), axis="Y", color=SHELL_DARK, collide=False))
    add(Piece("box", (0.008, 0.070, 0.010), rel(FRONT_X + 0.006, YC, RIM_Z - 0.010), color=SHELL_DARK, collide=False))
    # carry handle on the crown: two posts and a grip bar along x
    for xh in (0.548, 0.648):
        add(Piece("capsule", (0.011, 0.040, 0.011), rel(xh, YC, TOP + 0.268), axis="Z", color=SHELL_DARK))
    add(Piece("capsule", (0.013, 0.100, 0.013), rel(0.598, YC, TOP + 0.292), axis="X", color=SHELL_DARK))
    # foot pedals on the hood's rear slope: power (left) and cord rewind (right), with their icons
    for s, icon in ((1, "power"), (-1, "cord")):
        pc = (0.683, YC + s * 0.085, TOP + 0.214)
        add(Piece("cylinder", (0.024, 0.010, 0.024), rel(*pc), rot=(0.0, 0.55, 0.0), color=SHELL_DARK, collide=False))
        top = (pc[0] + 0.004, pc[1], pc[2] + 0.006)
        if icon == "power":
            for k in range(10):
                a = 2 * math.pi * k / 10
                if abs(a - math.pi / 2) < 0.5:
                    continue
                add(Piece("box", (0.0015, 0.004, 0.0015), rel(top[0] + 0.009 * math.sin(a) * 0.85, top[1] + 0.009 * math.cos(a), top[2]
                                                               + 0.009 * math.sin(a) * 0.5), rot=(0.0, 0.55, 0.0), color=ICON, collide=False))
            add(Piece("box", (0.010, 0.002, 0.0015), rel(top[0] - 0.004, top[1], top[2] + 0.003), rot=(0.0, 0.55, 0.0), color=ICON,
                      collide=False))
        else:
            add(Piece("box", (0.010, 0.006, 0.0015), rel(*top), rot=(0.0, 0.55, 0.0), color=ICON, collide=False))
            for dy in (-0.002, 0.002):
                add(Piece("box", (0.005, 0.0012, 0.0015), rel(top[0] - 0.006, top[1] + dy, top[2] + 0.004), rot=(0.0, 0.55, 0.0),
                          color=ICON, collide=False))
    # exhaust grille across the back: slots in a recessed panel
    add(Piece("box", (0.003, 0.200, 0.075), rel(BACK_X + 0.0005, YC, TOP + 0.095), color=SHELL_DARK, collide=False))
    for k in range(8):
        add(Piece("box", (0.004, 0.186, 0.003), rel(BACK_X + 0.0015, YC, TOP + 0.064 + 0.009 * k), color=(0.05, 0.05, 0.05), collide=False))
    # the cord's exit under the grille
    add(Piece("cylinder", (0.008, 0.012, 0.008), rel(BACK_X + 0.004, YC - 0.070, TOP + 0.045), axis="X", color=RUBBER, collide=False))
    # -- wheels: two 150 mm rear wheels with tread and hub caps, two front castors ----------------------
    for s in (-1, 1):
        wc = (0.618, YC + s * (HALF_W + 0.010), TOP + 0.075)
        add(Piece("cylinder", (0.075, 0.028, 0.075), rel(*wc), axis="Y", color=RUBBER, friction=WHEEL_SLIP))
        for k in range(28):
            a = 2 * math.pi * k / 28
            add(Piece("box", (0.010, 0.026, 0.004), rel(wc[0] + 0.0755 * math.cos(a), wc[1], wc[2] + 0.0755 * math.sin(a)),
                      rot=(0.0, -a + math.pi / 2, 0.0), color=(0.03, 0.03, 0.03), collide=False))
        add(Piece("cylinder", (0.050, 0.030, 0.050), rel(wc[0], wc[1] + s * 0.001, wc[2]), axis="Y", color=SHELL_DARK, collide=False))
        add(Piece("cylinder", (0.034, 0.032, 0.034), rel(wc[0], wc[1] + s * 0.002, wc[2]), axis="Y", color=BODY, collide=False))
        for k in range(5):
            a = 2 * math.pi * k / 5
            add(Piece("box", (0.030, 0.0015, 0.006), rel(wc[0] + 0.020 * math.cos(a), wc[1] + s * 0.0165, wc[2] + 0.020 * math.sin(a)),
                      rot=(0.0, -a, 0.0), color=SHELL_DARK, collide=False))
        add(Piece("cylinder", (0.012, 0.040, 0.012), rel(wc[0], wc[1] + s * 0.004, wc[2]), axis="Y", color=(0.08, 0.08, 0.09),
                  collide=False))                                                                       # axle cap
        add(Piece("cylinder", (0.0045, 0.042, 0.0045), rel(wc[0], wc[1] + s * 0.004, wc[2]), axis="Y", color=GREY_PLASTIC,
                  collide=False))
        cc = (0.300, YC + s * 0.090, TOP + 0.017)
        add(Piece("sphere", (0.017, 0.017, 0.017), rel(*cc), color=RUBBER, friction=WHEEL_SLIP))
        add(Piece("cylinder", (0.020, 0.012, 0.020), rel(cc[0], cc[1], cc[2] + 0.018), color=SHELL_DARK, collide=False))
    # -- the inlet: the hose's socket on the front wall, the inlet's mouth through to the holder --------
    add(Piece("cylinder", (0.030, 0.026, 0.030), rel(FRONT_X - 0.003, YC, INLET_Z), axis="X", color=SHELL_DARK))
    add(Piece("box", (0.010, 0.016, 0.010), rel(FRONT_X - 0.004, YC + 0.031, INLET_Z), color=GREY_PLASTIC, collide=False))  # release catch
    add(Piece("cylinder", (INLET_R, 0.0008, INLET_R), rel(FRONT_IN + 0.0003, YC, INLET_Z), axis="X", color=RUBBER, collide=False))
    add(Piece("cylinder", (0.023, 0.030, 0.023), rel(FRONT_X - 0.028, YC, INLET_Z), axis="X", color=SHELL_DARK, collide=False))  # hose cuff
    # -- the collar holder: two upright channel lips and the stop ---------------------------------------
    lip_x = FRONT_IN + SLOT + LIP[0] / 2
    lip_z0, lip_z1 = TOP + 0.045, LIP_TOP
    for s in (-1, 1):
        add(Piece("box", (LIP[0], LIP[1], lip_z1 - lip_z0), rel(lip_x, YC + s * LIP_Y, (lip_z0 + lip_z1) / 2), color=GREY_PLASTIC))
        add(Piece("box", (SLOT + LIP[0], 0.006, lip_z1 - lip_z0), rel(FRONT_IN + (SLOT + LIP[0]) / 2, YC + s * (LIP_Y + 0.009),
                                                                     (lip_z0 + lip_z1) / 2), color=GREY_PLASTIC))
    add(Piece("box", (STOP_D, 0.110, 0.006), rel(FRONT_IN + STOP_D / 2, YC, SEAT_Z - 0.003), color=GREY_PLASTIC))
    # a chamfer along the front wall's inner top edge over the holder: a collar let down on the wall's
    # top slides off it into the channels
    add(Piece("box", (0.020, 0.150, 0.003), rel(fw_x0 + WALL_T - 0.004, YC, RIM_Z + 0.006), rot=(0.0, 0.70, 0.0), color=GREY_PLASTIC))
    for s in (-1, 1):      # the channels' lead-in: chamfered lips flaring back at the top
        add(Piece("box", (0.003, LIP[1], 0.014), rel(lip_x + 0.004, YC + s * LIP_Y, lip_z1 + 0.004), rot=(0.0, 0.6, 0.0),
                  color=GREY_PLASTIC))
    size = (BACK_X - FRONT_X - 0.060, W - 0.040, 0.012)
    return Part("vacuum", pos=VAC_POS, size=size, exact_z=True, mass=VAC_MASS, color=SHELL_DARK, friction=(0.5, 0.4),
                contact_offset=0.002, com=(0.04, 0.0, 0.050), pieces=tuple(ps))


HOSE_PATH = ((FRONT_X - 0.040, YC, INLET_Z), (0.195, -0.006, TOP + 0.118), (0.172, -0.055, TOP + 0.075),
             (0.176, -0.122, TOP + 0.030), (0.215, -0.180, TOP + 0.019), (0.300, -0.197, TOP + 0.019),
             (0.450, -0.198, TOP + 0.019), (0.600, -0.199, TOP + 0.019), (0.700, -0.212, TOP + 0.019),
             (0.755, -0.268, TOP + 0.019), (0.785, -0.282, TOP + 0.019), (0.785, -0.262, TOP + 0.019))
HANDLE_X = 0.785


def hose_object() -> Part:
    """The vacuum's hose, plugged into the inlet socket, curving down and lying along the vacuum's right
    side on the bench to its handle behind the vacuum; the telescopic wand and floor nozzle lie across the
    bench behind it, out of the arms' way (fixed set dressing; drawn, so it catches nothing)."""
    ps = list(_corrugated([np.array(p) for p in HOSE_PATH], 0.019, HOSE))
    y0 = -0.248
    ps.append(Piece("cylinder", (0.022, 0.024, 0.022), (HANDLE_X, y0, TOP + 0.020), axis="Y", color=SHELL_DARK))          # hose cuff
    ps.append(Piece("capsule", (0.018, 0.085, 0.018), (HANDLE_X, y0 + 0.062, TOP + 0.020), axis="Y", color=SHELL_DARK))   # handle body
    ps.append(Piece("capsule", (0.012, 0.075, 0.012), (HANDLE_X, y0 + 0.066, TOP + 0.064), axis="Y", color=SHELL_DARK))   # grip loop
    for yh in (y0 + 0.028, y0 + 0.104):
        ps.append(Piece("capsule", (0.010, 0.030, 0.010), (HANDLE_X, yh, TOP + 0.043), axis="Z", color=SHELL_DARK))
    ps.append(Piece("box", (0.012, 0.022, 0.004), (HANDLE_X, y0 + 0.050, TOP + 0.039), color=(0.80, 0.80, 0.80)))         # air slider
    # the brushed-aluminium telescopic wand off the handle, lying across the bench behind the vacuum, and
    # the floor nozzle on its end
    yw0, yw1 = y0 + 0.125, 0.300
    ps.append(Piece("cylinder", (0.016, yw1 - yw0, 0.016), (HANDLE_X, (yw0 + yw1) / 2, TOP + 0.016), axis="Y", color=WAND))
    ps.append(Piece("cylinder", (0.0185, 0.036, 0.0185), (HANDLE_X, yw0 + 0.200, TOP + 0.017), axis="Y", color=SHELL_DARK))  # telescope lock
    ps.append(Piece("box", (0.010, 0.016, 0.004), (HANDLE_X, yw0 + 0.200, TOP + 0.035), color=(0.80, 0.80, 0.80)))
    ny = yw1 + 0.040
    ps.append(Piece("cylinder", (0.020, 0.045, 0.020), (HANDLE_X, yw1 + 0.016, TOP + 0.024), axis="Y", rot=(0.5, 0.0, 0.0),
                    color=SHELL_DARK))                                                                                   # nozzle neck
    ps.append(Piece("box", (0.240, 0.058, 0.026), (HANDLE_X - 0.010, ny, TOP + 0.013), color=SHELL_DARK))              # floor nozzle
    ps.append(Piece("box", (0.232, 0.008, 0.008), (HANDLE_X - 0.010, ny - 0.030, TOP + 0.005), color=(0.55, 0.45, 0.30)))  # brush strip
    ps.append(Piece("box", (0.200, 0.040, 0.003), (HANDLE_X - 0.010, ny + 0.002, TOP + 0.027), color=(0.30, 0.31, 0.33)))
    ps.append(Piece("box", (0.040, 0.020, 0.005), (HANDLE_X + 0.070, ny + 0.002, TOP + 0.029), color=BODY))              # carpet pedal
    c = (0.50, -0.10, TOP + 0.003)
    ps = [Piece(p.shape, p.size, (p.offset[0] - c[0], p.offset[1] - c[1], p.offset[2] - c[2]), rot=p.rot, axis=p.axis,
                color=p.color, material=p.material, collide=False) for p in ps]
    return Part("hose", pos=c, size=(0.010, 0.010, 0.006), exact_z=True, color=SHELL_DARK, pieces=tuple(ps))


# -- the lid: a domed red ABS shell, 200 x 290 x 82 mm, hinged at its rear edge to the body ----------
LID = (0.200, 0.290, 0.035)
LID_PROFILE = ((0.0, 0.050), (-0.040, 0.070), (-0.085, 0.080), (-0.130, 0.082), (-0.170, 0.080), (-0.200, 0.074))


def _rot_y(t):
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def lid_pivot():
    return (HINGE_X, YC, RIM_Z)


def lid_object() -> Part:
    """The compartment lid as left: swung open 100 degrees on its rear hinge, past upright, so it rests
    back on its stop. Laid out closed (a crowned shell with a skirt, the latch button at its front, the
    bag-full indicator window on top and the inspection grille on its underside) and turned open."""
    t = math.radians(LID_OPEN_DEG)
    r = _rot_y(t)
    W = LID[1]
    th = 0.004
    closed = []        # (shape, size, centre relative to the pivot when closed, local rot, colour, collide)
    for k, ((x0, z0), (x1, z1)) in enumerate(zip(LID_PROFILE, LID_PROFILE[1:])):
        size, c, rot, _, _ = _seg(x0, z0, x1, z1, W - 0.016, th)
        shade = (1.00, 1.06, 1.10, 1.04, 0.94)[k]        # the crown catches the light, the edges fall away
        closed.append(("box", size, c, rot, tuple(min(1.0, v * shade) for v in LID_RED), True))
        for s in (-1, 1):
            sz, cc, rr, _, _ = _seg(x0, z0 - 0.008, x1, z1 - 0.008, 0.016, 0.016)
            closed.append(("capsule", (0.008, sz[0], 0.008), (cc[0], s * (W / 2 - 0.008), cc[2]), rr, LID_RED, False))
    closed.append(("box", (th, W - 0.010, 0.072), (-0.200 + th / 2, 0.0, 0.036), (0.0, 0.0, 0.0), LID_RED, True))   # front skirt
    closed.append(("box", (th, W - 0.010, 0.048), (-th / 2, 0.0, 0.024), (0.0, 0.0, 0.0), LID_RED, True))           # rear skirt
    for s in (-1, 1):
        closed.append(("box", (0.196, th, 0.050), (-0.100, s * (W / 2 - th / 2), 0.025), (0.0, 0.0, 0.0), LID_RED, True))
    # the latch: a graphite catch and its thumb button at the front edge
    closed.append(("box", (0.016, 0.080, 0.024), (-0.206, 0.0, 0.040), (0.0, 0.0, 0.0), SHELL_DARK, True))
    closed.append(("box", (0.010, 0.050, 0.006), (-0.207, 0.0, 0.054), (0.0, 0.0, 0.0), GREY_PLASTIC, False))
    # a moulded finger recess behind the latch and two stiffening ribs along the crown
    closed.append(("box", (0.030, 0.090, 0.002), (-0.183, 0.0, 0.078), (0.0, -0.20, 0.0), (0.40, 0.05, 0.07), False))
    for yr in (-0.080, 0.080):
        closed.append(("box", (0.150, 0.006, 0.004), (-0.100, yr, 0.084), (0.0, 0.0, 0.0), tuple(v * 0.85 for v in LID_RED), False))
    # the bag-full indicator: a round window on the crown
    closed.append(("cylinder", (0.020, 0.003, 0.020), (-0.085, 0.040, 0.081), (0.0, 0.0, 0.0), (0.12, 0.14, 0.16), False))
    closed.append(("box", (0.024, 0.006, 0.0008), (-0.085, 0.040, 0.0828), (0.0, 0.0, 0.0), (0.95, 0.80, 0.10), False))
    # the inspection grille on the underside, over the bag
    closed.append(("box", (0.150, 0.220, 0.002), (-0.100, 0.0, 0.020), (0.0, 0.0, 0.0), (0.30, 0.06, 0.08), False))
    for k in range(10):
        closed.append(("box", (0.004, 0.214, 0.003), (-0.035 - 0.014 * k, 0.0, 0.0185), (0.0, 0.0, 0.0), SHELL_DARK, False))
    closed.append(("box", (0.150, 0.004, 0.003), (-0.100, 0.0, 0.0185), (0.0, 0.0, 0.0), SHELL_DARK, False))
    pieces = []
    for shape, size, c, rot, col, coll in closed:
        # turned by its own pitch, then with the whole lid open about the hinge (both about y)
        axis = {"capsule": "X", "cylinder": "Z"}.get(shape, "Z")
        pieces.append(Piece(shape, size, tuple(float(v) for v in r @ np.array(c)), rot=(0.0, t + rot[1], 0.0), axis=axis,
                            color=col, collide=coll))
    # the lid's hinge knuckles on its rear edge (the body's knuckles sit between them)
    for yk in (-0.110, -0.030, 0.050, 0.130):
        pieces.append(Piece("cylinder", (0.008, 0.034, 0.008), (0.0, yk, 0.0), axis="Y", color=SHELL_DARK, collide=False))
    pv = lid_pivot()
    root_size = (0.010, W - 0.030, 0.010)
    # its weight sits toward its front (the latch and the thick front moulding), low in the shell: open
    # past upright it rests back on its stop, swung forward past ~85 degrees it falls shut by itself
    com = tuple(float(v) for v in r @ np.array(LID_COM_CLOSED))
    return Part("lid", pos=pv, size=root_size, shape="box", exact_z=True, mass=0.30, color=SHELL_DARK, friction=(0.5, 0.4),
                contact_offset=0.002, pieces=tuple(pieces), com=com,
                hinge=Hinge(pivot=pv, axis="Y", damping=LID_DAMP, lower_deg=-LID_OPEN_DEG, upper_deg=2.0,
                            parent="vacuum", parent_pos=VAC_POS))


# -- bags: a vacuum bag's collar is a 120 x 125 x 2 mm card with a 38 mm hole sealed by a rubber
# gasket ring, a 40 x 30 mm pull tab above; the old one carries a full fleece bag, the new one its pack
def _collar_pieces(rel, x_back):
    """The collar's tab, gasket ring and print, relative to the collar plate's centre; ``x_back`` is +1
    where the bag lies (+x when seated)."""
    f = -x_back            # the printed front face is on -x
    gz = HOLE_Z - COLLAR[2] / 2
    ps = [Piece("box", TAB, rel(0.0, 0.0, COLLAR[2] / 2 + TAB[2] / 2), color=CARD, friction=TAB_FRICTION)]
    # the rubber gasket ring round the hole, standing proud of the card's front, and the hole
    for k in range(16):
        a = 2 * math.pi * k / 16
        ps.append(Piece("box", (0.004, 0.009, 0.006), rel(f * 0.002, 0.024 * math.cos(a), gz + 0.024 * math.sin(a)),
                        rot=(a + math.pi / 2, 0.0, 0.0), color=RUBBER, collide=False))
    ps.append(Piece("cylinder", (0.019, 0.0024, 0.019), rel(f * 0.0003, 0.0, gz), axis="X", color=(0.10, 0.10, 0.10), collide=False))
    # print: a coloured band across the top, an arrow pointing down (slide this way), a size code
    ps.append(Piece("box", (0.0004, 0.116, 0.018), rel(f * 0.0012, 0.0, COLLAR[2] / 2 - 0.013), color=BOX_BLUE, collide=False))
    for k, w in enumerate((0.024, 0.016, 0.008)):
        ps.append(Piece("box", (0.0004, w, 0.005), rel(f * 0.0012, -0.040, gz + 0.020 - 0.006 * k), color=BOX_BLUE, collide=False))
    ps.append(Piece("box", (0.0004, 0.006, 0.014), rel(f * 0.0012, -0.040, gz + 0.032), color=BOX_BLUE, collide=False))
    ps.append(Piece("box", (0.0004, 0.020, 0.008), rel(f * 0.0012, 0.042, gz - 0.040), color=(0.85, 0.15, 0.15), collide=False))
    ps.append(Piece("box", (0.0004, 0.026, 0.006), rel(f * 0.0012, 0.0, COLLAR[2] / 2 + TAB[2] - 0.010), color=BOX_BLUE, collide=False))
    # the back is printed too: the band, an arrow and "pull" on the tab
    ps.append(Piece("box", (0.0004, 0.116, 0.018), rel(-f * 0.0012, 0.0, COLLAR[2] / 2 - 0.013), color=BOX_BLUE, collide=False))
    ps.append(Piece("box", (0.0004, 0.026, 0.006), rel(-f * 0.0012, 0.0, COLLAR[2] / 2 + TAB[2] - 0.010), color=BOX_BLUE,
                    collide=False))
    for k, w in enumerate((0.008, 0.016, 0.024)):          # an arrow pointing up the tab: pull here
        ps.append(Piece("box", (0.0004, w, 0.004), rel(-f * 0.0012, 0.0, COLLAR[2] / 2 + 0.003 + 0.005 * (2 - k) + 0.004),
                        color=(0.85, 0.15, 0.15), collide=False))
    return ps


OLD_C = (COLLAR_X, YC, SEAT_Z + COLLAR[2] / 2)          # the old collar's centre, seated
BAG_BODY_X = (FRONT_IN + SLOT + LIP[0] + 0.010, PART_X - 0.022)   # clear of the lips, lead-in and partition as it comes up
BAG_BODY_Z = (FLOOR_Z + 0.006, SEAT_Z + COLLAR[2] - 0.042)   # its top 42 mm under the collar's, the card showing above
LABEL = (0.95, 0.95, 0.93)
OLD_BAG_MASS = 0.18                  # a bag about two-thirds full of fine dust (180 g)


def old_bag_object() -> Part:
    """The full bag, seated: collar in its channels down on the stop, the fleece bag swollen with dust
    filling the compartment behind it, its side gussets folded in (one rigid body, 0.18 kg)."""
    rel = lambda x, y, z: (x, y, z)
    ps = _collar_pieces(rel, +1)
    x0, x1 = BAG_BODY_X
    z0, z1 = BAG_BODY_Z
    bx = (x0 + x1) / 2 - OLD_C[0]
    bz = (z0 + z1) / 2 - OLD_C[2]
    lx, lz, ly = x1 - x0, z1 - z0, 0.200
    r = 0.026
    # the neck: fleece gathered from the collar's back, between the channel lips
    ps.append(Piece("box", (x0 - (COLLAR_X + COLLAR[0] / 2) + 0.004, 0.080, 0.095),
                    (((COLLAR_X + COLLAR[0] / 2) + x0) / 2 - OLD_C[0] - 0.001, 0.0, HOLE_Z - COLLAR[2] / 2 + 0.004), color=FLEECE))
    # the swollen bag: a pillow of fleece, rounded all round and crowned on top
    ps.append(Piece("box", (lx - 2 * r, ly - 2 * r, lz), (bx, 0.0, bz), color=FLEECE))
    ps.append(Piece("box", (lx, ly - 2 * r, lz - 2 * r), (bx, 0.0, bz), color=FLEECE))
    ps.append(Piece("box", (lx - 2 * r, ly, lz - 2 * r), (bx, 0.0, bz), color=FLEECE))
    for sy in (-1, 1):
        for sx in (-1, 1):
            ps.append(Piece("capsule", (r, lz - 2 * r, r), (bx + sx * (lx / 2 - r), sy * (ly / 2 - r), bz), axis="Z", color=FLEECE))
        for sz in (-1, 1):
            ps.append(Piece("capsule", (r, lx - 2 * r, r), (bx, sy * (ly / 2 - r), bz + sz * (lz / 2 - r)), axis="X", color=FLEECE))
    for sx in (-1, 1):
        for sz in (-1, 1):
            ps.append(Piece("capsule", (r, ly - 2 * r, r), (bx + sx * (lx / 2 - r), 0.0, bz + sz * (lz / 2 - r)), axis="Y", color=FLEECE))
    # quilted top: three puffed pillows across the bag with stitched seams between them
    for k, dxq in enumerate((-0.045, 0.0, 0.045)):
        ps.append(Piece("capsule", (0.030, ly - 0.070, 0.030), (bx + dxq, 0.0, bz + lz / 2 - 0.024), axis="Y",
                        color=FLEECE if k != 1 else (0.96, 0.96, 0.94)))
    for dxq in (-0.025, 0.025):
        ps.append(Piece("box", (0.003, ly - 0.060, 0.002), (bx + dxq, 0.0, bz + lz / 2 - 0.004), color=(0.70, 0.70, 0.68),
                        collide=False))
    top = bz + lz / 2 + 0.0062
    # the printed label on top: white panel, a coloured band, text lines and an arrow
    ps.append(Piece("box", (0.090, 0.120, 0.001), (bx + 0.005, 0.0, top + 0.0002), color=LABEL, collide=False))
    ps.append(Piece("box", (0.090, 0.022, 0.0012), (bx + 0.005, -0.049, top + 0.0004), color=BOX_BLUE, collide=False))
    for k in range(4):
        ps.append(Piece("box", (0.004 if k else 0.006, 0.070 - 0.012 * k, 0.0012), (bx - 0.025 + 0.014 * k, 0.010, top + 0.0004),
                        color=(0.35, 0.35, 0.38), collide=False))
    ps.append(Piece("box", (0.030, 0.006, 0.0012), (bx + 0.035, 0.040, top + 0.0004), color=(0.85, 0.15, 0.15), collide=False))
    # dust showing grey through the fleece low down, and the welded seams and gusset folds
    ps.append(Piece("box", (lx - 0.050, 0.0015, lz - 0.070), (bx + 0.010, ly / 2 + 0.0004, bz - 0.020), color=FLEECE_DUST,
                    collide=False))
    ps.append(Piece("box", (lx - 0.050, 0.0015, lz - 0.070), (bx + 0.010, -ly / 2 - 0.0004, bz - 0.020), color=FLEECE_DUST,
                    collide=False))
    for sy in (-1, 1):
        for a in (0.55, -0.55):          # gusset folds: a V on each side
            ps.append(Piece("box", (0.090, 0.0015, 0.003), (bx + 0.020, sy * (ly / 2 + 0.0006), bz + 0.010), rot=(0.0, a, 0.0),
                            color=(0.78, 0.78, 0.75), collide=False))
    ps.append(Piece("box", (0.003, ly - 0.040, 0.0015), (bx + lx / 2 - 0.010, 0.0, top - 0.004), color=(0.80, 0.80, 0.77),
                    collide=False))
    # a full fleece bag lifted by its collar sags and hangs straight under it: the rigid stand-in carries
    # its weight under the collar, so held by the tab it hangs as the real one does
    com = (0.0, 0.0, bz)
    return Part("old_bag", pos=OLD_C, size=COLLAR, exact_z=True, mass=OLD_BAG_MASS, color=CARD, friction=CARD_ON_PLASTIC,
                contact_offset=0.001, com=com, pieces=tuple(ps))


# -- the carton of new bags (fixed) and the new bag standing in it (free) ------------------------------
CARTON = (0.140, 0.075, 0.075)       # x long, y deep, z tall: a carton of four bags and two filters, opened on top
CARTON_C = (0.300, -0.300)
PACK = (0.014, 0.096, 0.125)         # the new bag folded: x thick (behind its collar), y wide, z tall
PACK_DROP = 0.0                      # the pack's bottom is flush with the collar's bottom edge
PACK_GAP = 0.0045                    # the pack hangs this far behind the collar's back (clear of the lips)


def carton_object() -> Part:
    """The opened carton of new bags (printed paperboard: a picture of the bag, the pack count, text),
    standing at the left with its top flaps folded out (fixed: the bags are drawn out of it)."""
    L, D, H = CARTON
    t = 0.002
    c = (CARTON_C[0], CARTON_C[1], TOP + t / 2)
    ps = [Piece("box", (L, t, H), (0.0, -D / 2 + t / 2, H / 2), color=BOX_WHITE),
          Piece("box", (L, t, H), (0.0, D / 2 - t / 2, H / 2), color=BOX_WHITE),
          Piece("box", (t, D, H), (-L / 2 + t / 2, 0.0, H / 2), color=BOX_WHITE),
          Piece("box", (t, D, H), (L / 2 - t / 2, 0.0, H / 2), color=BOX_WHITE)]
    for fy in (-D / 2 - 0.0005, D / 2 + 0.0005):
        ps += [Piece("box", (L, 0.001, 0.016), (0.0, fy, H - 0.010), color=BOX_BLUE, collide=False),                 # top band
               Piece("box", (L, 0.001, 0.006), (0.0, fy, 0.005), color=(0.85, 0.15, 0.15), collide=False),            # foot band
               # the bag's picture: a white pouch with its card collar and the gasket's ring
               Piece("box", (0.042, 0.0012, 0.040), (-0.035, fy, 0.033), color=(0.90, 0.90, 0.88), collide=False),
               Piece("box", (0.022, 0.0014, 0.024), (-0.047, fy, 0.037), color=CARD, collide=False),
               Piece("cylinder", (0.006, 0.0016, 0.006), (-0.047, fy, 0.035), axis="Y", color=RUBBER, collide=False),
               # pack count and text lines
               Piece("box", (0.026, 0.0012, 0.020), (0.045, fy, 0.040), color=BOX_BLUE, collide=False),
               Piece("box", (0.008, 0.0014, 0.010), (0.045, fy, 0.040), color=BOX_WHITE, collide=False)]
        for k in range(3):
            ps.append(Piece("box", (0.040 - 0.010 * k, 0.0012, 0.003), (0.005, fy, 0.044 - 0.008 * k), color=(0.35, 0.35, 0.38),
                            collide=False))
    ps += [Piece("box", (0.001, D - 0.010, 0.016), (L / 2 + 0.0005, 0.0, H - 0.010), color=BOX_BLUE, collide=False),
           Piece("box", (0.001, D - 0.010, 0.016), (-L / 2 - 0.0005, 0.0, H - 0.010), color=BOX_BLUE, collide=False),
           # the lid flap folded back and the side flaps folded out
           Piece("box", (L, 0.060, t), (0.0, D / 2 + 0.026, H + 0.012), rot=(0.75, 0.0, 0.0), color=BOX_WHITE, collide=False),
           Piece("box", (L - 0.010, 0.040, 0.0012), (0.0, D / 2 + 0.026, H + 0.013), rot=(0.75, 0.0, 0.0), color=BOX_BLUE,
                 collide=False),
           Piece("box", (0.030, D, t), (-L / 2 - 0.014, 0.0, H + 0.004), rot=(0.0, 0.5, 0.0), color=BOX_WHITE, collide=False),
           Piece("box", (0.030, D, t), (L / 2 + 0.014, 0.0, H + 0.004), rot=(0.0, -0.5, 0.0), color=BOX_WHITE, collide=False),
           # the three bags still packed behind the loose one, their collars and tabs showing
           Piece("box", (L - 0.010, 0.016, H - 0.010), (0.0, -D / 2 + 0.012, H / 2), color=(0.90, 0.90, 0.88), collide=False),
           Piece("box", (0.120, 0.004, 0.030), (0.0, -D / 2 + 0.018, H + 0.002), color=CARD, collide=False),
           Piece("box", (0.040, 0.004, 0.018), (0.0, -D / 2 + 0.018, H + 0.026), color=CARD, collide=False),
           Piece("box", (0.120, 0.004, 0.026), (0.0, -D / 2 + 0.008, H), color=CARD, collide=False)]
    return Part("carton", pos=c, size=(L, D, t), exact_z=True, color=BOX_WHITE, pieces=tuple(ps))


NEW_YAW = -math.pi / 2               # the new bag's pack on the -y side, its collar facing +y, across the carton


def new_bag_pos():
    """The new collar's centre as it stands in the carton: pack bottom on the carton floor, the collar's
    printed face against the carton's +y side (the packed bags behind it, at -y, keep it from leaning)."""
    z = TOP + 0.002 + PACK_DROP + COLLAR[2] / 2 + 0.002
    return (CARTON_C[0], CARTON_C[1] + CARTON[1] / 2 - 0.002 - 0.0035, z)


def new_bag_object() -> Part:
    """A new bag standing in its carton: the collar card with its tab up, the folded fleece pack behind
    it (0.03 kg). Laid out as seated (pack on +x), then turned 90 degrees about the vertical so its
    collar faces -y across the carton."""
    rel = lambda x, y, z: (x, y, z)
    ps = _collar_pieces(rel, +1)
    px = COLLAR[0] / 2 + PACK_GAP + PACK[0] / 2     # the pack clears the channel lips and stop behind the collar
    pz = -COLLAR[2] / 2 - PACK_DROP + PACK[2] / 2   # the pack from the collar's bottom edge up
    ps.append(Piece("box", PACK, (px, 0.0, pz), color=FLEECE))
    ps.append(Piece("box", (PACK_GAP, 0.050, 0.040), (COLLAR[0] / 2 + PACK_GAP / 2, 0.0, HOLE_Z - COLLAR[2] / 2), color=FLEECE))
    for k in range(4):   # the pack's folds
        ps.append(Piece("box", (0.0008, PACK[1] - 0.006, 0.0015), (px + PACK[0] / 2 + 0.0004, 0.0, pz - PACK[2] / 2 + 0.030 * (k + 1)),
                        color=(0.80, 0.80, 0.77), collide=False))
    return Part("new_bag", pos=new_bag_pos(), size=COLLAR, exact_z=True, mass=0.03, yaw=NEW_YAW, color=CARD,
                friction=CARD_ON_PLASTIC, contact_offset=0.001, pieces=tuple(ps))


# -- the wastebasket (fixed): 6 L polypropylene, 200 mm across, 250 mm tall, a rolled rim, empty -------
BIN_C = (0.300, 0.430)
BIN_R = 0.130
BIN_H = 0.230


def bin_object() -> Part:
    n = 24
    ps = []
    taper = 0.016                     # narrower at the foot
    for k in range(n):
        a = 2 * math.pi * k / n
        rm = BIN_R - 0.004 - taper / 2
        ps.append(Piece("box", (0.003, 2 * BIN_R * math.tan(math.pi / n) + 0.002, BIN_H - 0.006),
                        (rm * math.cos(a), rm * math.sin(a), (BIN_H - 0.006) / 2), rot=(0.0, math.atan2(taper, BIN_H), a),
                        color=BIN_GREY))
        # the rolled rim: a ring of short tubes laid round the top
        ps.append(Piece("capsule", (0.006, 2 * BIN_R * math.tan(math.pi / n) - 0.002, 0.006),
                        ((BIN_R - 0.001) * math.cos(a), (BIN_R - 0.001) * math.sin(a), BIN_H - 0.006),
                        rot=(0.0, 0.0, a + math.pi / 2), axis="X", color=(0.36, 0.38, 0.40), collide=False))
        if k % 2 == 0:     # moulded ribs down the side
            ps.append(Piece("box", (0.002, 0.004, BIN_H - 0.060), ((rm + 0.0025) * math.cos(a), (rm + 0.0025) * math.sin(a),
                                                                  (BIN_H - 0.006) / 2), rot=(0.0, math.atan2(taper, BIN_H), a),
                            color=(0.38, 0.40, 0.43), collide=False))
    c = (BIN_C[0], BIN_C[1], TOP + 0.003)
    return Part("wastebasket", pos=c, size=(BIN_R - 0.016, 0.006, BIN_R - 0.016), shape="cylinder", exact_z=True,
                color=BIN_GREY, pieces=tuple(ps))


# -- measurements ---------------------------------------------------------------------------------------

def quat_matrix(q):
    """Rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
