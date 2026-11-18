"""Geometry and mass of the cereal dispenser, the paddle wheel, the cereal pieces and the bowl. NumPy only, so the task
file and the tests use it without Isaac Sim. Created 11_16_26 (America/New_York).

Dispenser frame D (m): origin on the counter top under the paddle wheel's centre, +x out of the dispenser's front
(the knob side, facing the robot), z up. The wheel turns about the x axis.

The dispenser is a true-size single-canister twist-knob wall-mount unit of the Zevro/Honey-Can-Do SmartSpace type
(listed overall size 4.8 in wide, 5.8 in deep, 13 in tall; a 13 oz cylindrical canister; about 1 oz of cereal per
twist of the knob), screwed to an upright mounting board with its spout 80 mm above the counter, so that a 2 1/2 in
bowl just fits under it. Its internal proportions (wheel diameter and length, inlet and spout openings) are not
published; they are chosen to fit the listed outer size.

Cereal: toasted oat rings at true size and mass. A 39 g serving is 1 1/2 cups on the box; one cup holds about 252
pieces, so a piece weighs about 0.10 g, and an 11.5 mm x 5 mm ring fills a cup at that count.
"""

import math
import os

import numpy as np

MM = 1e-3

# -- dispenser --------------------------------------------------------------------------------------------------------
OVERALL = dict(w=0.122, d=0.147, h=0.330)  # listed 4.8 x 5.8 x 13 in
BACK_PLATE = dict(x0=-0.0725, x1=-0.0665, w=0.090, z0=0.100)  # the mounting plate screwed to the board
BOARD = dict(x0=-0.0925, x1=-0.0725, w=0.240, h=0.500)  # the upright mounting board (stands in for a wall)
WHEEL_Z = 0.135  # wheel axis height above the counter
R_BORE = 0.041  # the wheel chamber's inner radius
L_WHEEL = 0.068  # wheel length along its axis
END_GAP = 0.0015  # vane ends to the chamber's end plates
X_END = L_WHEEL / 2 + END_GAP  # inner faces of the end plates
T_SHELL = 0.004
N_BORE = 36  # 10 degree collider pieces round the chamber
INLET = (50.0, 130.0)  # degrees in the D frame's y-z plane (0 = +y, 90 = up): the opening into the canister
OUTLET = (240.0, 300.0)  # the opening down into the spout
SPOUT_Z = 0.080  # the spout's mouth above the counter
FUNNEL_H = 0.045
FUNNEL_TOP_HALF = 0.058
CANISTER_R = 0.056  # inner radius of the clear canister
CANISTER_TOP = 0.400
LID_T = 0.010
N_CAN = 32

# -- paddle wheel -----------------------------------------------------------------------------------------------------
N_VANES = 6
VANE_ANGLES = tuple(30.0 + 60.0 * k for k in range(N_VANES))  # degrees, wheel at zero: a vane splits each opening
R_HUB = 0.009
R_HINGE = 0.027  # where each soft vane tip is hinged to the rigid inner vane
R_TIP = 0.0395  # vane tip radius: 1.5 mm short of the chamber wall
T_VANE = 0.003
TIP_T = 0.0025
WHEEL_MASS = 0.030  # hub, inner vanes, shaft (polypropylene)
TIP_MASS = 0.0015  # each soft vane tip (silicone)
TIP_STIFFNESS = 0.05  # N m/rad: a tip bends back about 15 degrees under 1 N at its edge, so it never crushes a piece
TIP_DAMPING = 2e-4
TIP_LIMIT = 50.0  # degrees either way
WHEEL_FRICTION = 0.10  # N m: the static drag of the wheel's seals; the knob needs a light twist to start
WHEEL_HOLD_DAMPING = 60.0  # N m s/rad: the friction drive saturates above about 0.1 deg/s of slip

# knob on the shaft's front end: a round knob with a raised grip bar across its face
KNOB = dict(r=0.028, t=0.014, gap=0.003)
BAR = dict(len=0.050, w=0.012, h=0.018)
KNOB_MASS = 0.020
KNOB_X0 = X_END + T_SHELL + KNOB["gap"]  # the knob's back face
BAR_X = KNOB_X0 + KNOB["t"] + BAR["h"] / 2  # the grip bar's centre along the axis
BAR_ANGLE0 = 0.0  # degrees: the bar lies along y with the wheel at zero

# -- cereal -----------------------------------------------------------------------------------------------------------
PIECE_D = 0.0115
PIECE_T = 0.0050
PIECE_HOLE = 0.0038
PIECE_MASS = 0.000103  # 39 g / (1.5 cups x 252 pieces)
SERVING_G = 39.0  # the box's labelled serving: 1 1/2 cups (39 g)
N_PIECES = int(os.environ.get("CEREAL_N", "760"))  # about 78 g in the canister at the start (CEREAL_N: dev only)
CUP_COUNT = 252


def piece_hull():
    """The ring's collider: the convex hull of a torus (a rounded disc), 14 sides, as vertices in the piece frame."""
    pts = []
    for r, z in ((PIECE_D / 2, 0.30 * PIECE_T), (PIECE_D / 2, -0.30 * PIECE_T), (PIECE_D / 2 - 0.0012, 0.5 * PIECE_T), (PIECE_D / 2 - 0.0012, -0.5 * PIECE_T)):
        for k in range(14):
            a = 2 * math.pi * k / 14
            pts.append((r * math.cos(a), r * math.sin(a), z))
    return np.array(pts)


def initial_pieces(n=N_PIECES, seed=0):
    """Starting centres (D frame) of the pieces: a loose lattice filling the funnel and canister from the bottom up,
    from which they fall and settle before the trial starts. Deterministic."""
    rng = np.random.default_rng(seed)
    out = []
    pitch = 0.0125
    z = WHEEL_Z + R_BORE * math.cos(math.radians(40)) + 0.008
    while len(out) < n:
        frac = min(max((z - (WHEEL_Z + 0.030)) / FUNNEL_H, 0.0), 1.0)
        half = 0.026 + frac * (FUNNEL_TOP_HALF - 0.026) - 0.009  # inside the funnel walls at this height
        lim = min(half, CANISTER_R - 0.009) if z > WHEEL_Z + 0.030 + FUNNEL_H else half
        g = np.arange(-lim, lim + 1e-9, pitch)
        for x in g:
            for y in g:
                if z > WHEEL_Z + 0.030 + FUNNEL_H and x * x + y * y > (CANISTER_R - 0.009) ** 2:
                    continue
                out.append((x + rng.uniform(-0.002, 0.002), y + rng.uniform(-0.002, 0.002), z))
                if len(out) == n:
                    break
            if len(out) == n:
                break
        z += 0.0072
    out = np.array(out)
    if out[:, 2].max() > CANISTER_TOP - 0.010:
        raise ValueError("the starting lattice does not fit under the lid")
    return out


# -- bowl -------------------------------------------------------------------------------------------------------------
# A 6 1/2 in polypropylene everyday bowl (listed: 6 1/2 in across, 2 1/2 in tall, 3 oz with packaging).
BOWL = dict(r_rim=0.0825, r_foot=0.045, h=0.0635, t=0.0022, foot_h=0.005, mass=0.080)
BOWL_SEG = 32


def bowl_profile(n=6):
    """The bowl's inner surface as (r, z) from the floor's edge to the rim (bowl frame: origin at the centre of the
    underside, z up). The wall is a quarter-ellipse from the floor to the rim."""
    r_floor = BOWL["r_foot"]
    z_floor = BOWL["foot_h"] + BOWL["t"]
    pts = []
    for k in range(n + 1):
        th = 0.5 * math.pi * k / n
        pts.append((r_floor + (BOWL["r_rim"] - BOWL["t"] - r_floor) * math.sin(th), z_floor + (BOWL["h"] - z_floor) * (1 - math.cos(th))))
    return np.array(pts)


BOWL_PROFILE = bowl_profile()
BOWL_FLOOR_Z = BOWL["foot_h"] + BOWL["t"]


def bowl_inner_radius(z):
    """Inner radius of the bowl at height z in its frame (clamped)."""
    return float(np.interp(z, BOWL_PROFILE[:, 1], BOWL_PROFILE[:, 0]))


def bowl_wall_pieces():
    """Convex collider pieces of the bowl's wall: (8, 3) vertex arrays in the bowl frame."""
    inner = BOWL_PROFILE
    nrm = []
    for k in range(len(inner)):
        a = inner[min(k + 1, len(inner) - 1)] - inner[max(k - 1, 0)]
        n = np.array((a[1], -a[0]))
        nrm.append(n / np.linalg.norm(n))
    outer = inner + BOWL["t"] * np.array(nrm)
    out = []
    for k in range(len(inner) - 1):
        quad = (inner[k], inner[k + 1], outer[k + 1], outer[k])
        for j in range(BOWL_SEG):
            a0, a1 = 2 * math.pi * j / BOWL_SEG, 2 * math.pi * (j + 1) / BOWL_SEG
            out.append(np.array([(r * math.cos(a), r * math.sin(a), z) for a in (a0, a1) for r, z in quad]))
    return out


def ring_pieces(r_in, r_out, x0, x1, angles_deg, skip=()):
    """Convex pieces of a cylindrical shell about the x axis (centre y = 0, z = WHEEL_Z): one per angular step,
    leaving out those whose middle falls in any (lo, hi) degree range in ``skip``."""
    out = []
    for a0, a1 in zip(angles_deg[:-1], angles_deg[1:]):
        mid = 0.5 * (a0 + a1)
        if any(lo < mid < hi for lo, hi in skip):
            continue
        v = []
        for a in (a0, a1):
            c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
            for r in (r_in, r_out):
                for x in (x0, x1):
                    v.append((x, r * c, WHEEL_Z + r * s))
        out.append(np.array(v))
    return out


def chamber_pieces():
    angles = np.linspace(0.0, 360.0, N_BORE + 1)
    return ring_pieces(R_BORE, R_BORE + T_SHELL, -X_END, X_END, angles, skip=(INLET, OUTLET))


def funnel_pieces():
    """Four sloped plates from the chamber's inlet up to the canister's bottom, as convex hulls."""
    z0 = WHEEL_Z + R_BORE * math.sin(math.radians(INLET[0]))
    y0 = R_BORE * math.cos(math.radians(INLET[0]))
    x0 = X_END  # the front and back plates start at the chamber's end plates, clear of the vanes
    z1 = z0 + FUNNEL_H
    h1 = FUNNEL_TOP_HALF
    t = 0.003
    out = []
    for sx in (1, -1):  # front and back plates (normal along x)
        inner = [(sx * x0, -y0 - 0.003, z0), (sx * x0, y0 + 0.003, z0), (sx * h1, -h1, z1), (sx * h1, h1, z1)]
        out.append(np.array(inner + [(px + sx * t, py, pz) for px, py, pz in inner]))
    for sy in (1, -1):  # side plates (normal along y)
        inner = [(-x0, sy * y0, z0), (x0, sy * y0, z0), (-h1, sy * h1, z1), (h1, sy * h1, z1)]
        out.append(np.array(inner + [(px, py + sy * t, pz) for px, py, pz in inner]))
    return out, z0, z1


FUNNEL_Z0 = WHEEL_Z + R_BORE * math.sin(math.radians(INLET[0]))
FUNNEL_Z1 = FUNNEL_Z0 + FUNNEL_H
OUTLET_Y = R_BORE * math.cos(math.radians(OUTLET[1]))  # half-width of the spout across the axis
OUTLET_Z = WHEEL_Z + R_BORE * math.sin(math.radians(OUTLET[0]))  # where the chamber opens into the spout


def canister_pieces():
    """The clear canister's wall: convex pieces of a vertical cylinder from the funnel top to the lid."""
    out = []
    for j in range(N_CAN):
        a0, a1 = 2 * math.pi * j / N_CAN, 2 * math.pi * (j + 1) / N_CAN
        v = [(r * math.cos(a), r * math.sin(a), z) for a in (a0, a1) for r in (CANISTER_R, CANISTER_R + 0.003) for z in (FUNNEL_Z1 - 0.004, CANISTER_TOP)]
        out.append(np.array(v))
    return out


def vane_boxes():
    """Rigid inner vanes in the wheel frame (wheel at zero): (centre, size, angle_deg) of each box, x along the axis."""
    out = []
    for a in VANE_ANGLES:
        rm = 0.5 * (R_HUB + R_HINGE)
        c = (0.0, rm * math.cos(math.radians(a)), WHEEL_Z + rm * math.sin(math.radians(a)))
        out.append((c, (L_WHEEL, R_HINGE - R_HUB + 0.002, T_VANE), a))
    return out


def tip_box(a):
    """A soft vane tip in the D frame (wheel at zero): (hinge point, centre, size, angle_deg)."""
    hinge = (0.0, R_HINGE * math.cos(math.radians(a)), WHEEL_Z + R_HINGE * math.sin(math.radians(a)))
    rm = 0.5 * (R_HINGE + R_TIP)
    c = (0.0, rm * math.cos(math.radians(a)), WHEEL_Z + rm * math.sin(math.radians(a)))
    return hinge, c, (L_WHEEL, R_TIP - R_HINGE, TIP_T), a


def wheel_swept_volume():
    """Volume (m^3) the vanes sweep per turn, less the vanes: the most one turn can carry."""
    return math.pi * (R_TIP**2 - R_HUB**2) * L_WHEEL - N_VANES * (R_TIP - R_HUB) * L_WHEEL * T_VANE


BULK_DENSITY = SERVING_G / 1000 / (1.5 * 236.6e-6)  # kg/m^3 of loose rings: 39 g per 1 1/2 US cups
