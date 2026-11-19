"""Geometry and measurements for sealing a filled zip-top bag by pinching its track shut along its length.

On a kitchen prep counter: an end-grain butcher block, and on it a filled 1-quart zip-top freezer bag of
chopped vegetables lying flat, its press-to-seal track toward the arms and standing 30 mm out past the
block's near edge, so a hand's fingers can close on it from above and below. A few
vegetable ends lie on the block; a bowl of more chopped vegetables and the carton of bags stand behind.

The track is two interlocking profiles, one on each film: a rib on the top film over a groove on the
bottom film (the blue and yellow strips of a colour-change track, which read green where the rib is
pressed home). As left the mouth stands open, the rib lying 3 mm above its groove. A press-to-seal
track has no slider: it closes only where it is squeezed between finger and thumb, so one hand pinches
it at one corner and slides along to the other while the other hand holds the bag down.
Frame: metres, x away from the arm bases, y left, z up; TOP is the counter.
"""
from __future__ import annotations

import math

import numpy as np

from simkit import Part, Piece

TOP = 0.7475

WOOD = (0.66, 0.48, 0.30)            # maple butcher block
FILM = (0.88, 0.92, 0.95)            # clear LDPE film, drawn by its puffed edges and seams
FILM_EDGE = (0.78, 0.84, 0.88)
SHEEN = (0.97, 0.98, 1.00)
INSIDE = (0.20, 0.22, 0.24)          # the dark inside of the open mouth
YELLOW = (0.96, 0.80, 0.10)          # the groove profile (bottom film)
BLUE = (0.12, 0.34, 0.86)            # the rib profile (top film)
GREEN = (0.10, 0.62, 0.28)           # rib pressed into groove: the colour-change track reads green
WHITE = (0.97, 0.97, 0.97)           # the write-on label panel
CARROT = (0.93, 0.46, 0.10)
PEPPER = (0.80, 0.12, 0.10)
PEPPER_G = (0.25, 0.55, 0.15)
CELERY = (0.62, 0.78, 0.38)
STEEL = (0.74, 0.75, 0.77)
BLACK = (0.08, 0.08, 0.09)

# -- butcher block (fixed) -------------------------------------------------------------------------
BOARD = (0.305, 0.457, 0.045)        # x deep, y wide, thick: a 12 x 18 x 1 3/4 in end-grain block
BOARD_C = (0.3875, 0.020)
BOARD_TOP = TOP + BOARD[2]
BOARD_NEAR = BOARD_C[0] - BOARD[0] / 2          # its near edge (toward the arms), x 0.235

# -- the bag: a 1-quart zip-top freezer bag, 177 x 188 mm outside (6 15/16 x 7 3/8 in) --------------
BAG_W = 0.177                        # along y
BAG_L = 0.188                        # along x, mouth edge to bottom fold
BAG_Y = 0.000                        # its centreline
# the press-to-seal track, 10 mm in from the mouth's edge (the flanges past it)
TRACK_W = 0.004                      # each profile 4 mm wide (x) ...
PROFILE_T = 0.0016                   # ... and 1.6 mm thick; pressed home, the pair is 2.6 mm
CLOSED_T = 0.0026
TRACK_X = BOARD_NEAR - 0.030         # the track 30 mm out past the block's edge, x 0.205
MOUTH_X = TRACK_X - 0.010            # the flanges' free edge (toward the arms)
BOTTOM_X = MOUTH_X + BAG_L           # the bag's bottom fold, far from the arms
GAPE = 0.0030                        # as left: the rib lies 3 mm clear above its groove (mouth open)
N_CELLS = 16                         # the track's length scored in 16 cells of 11 mm
CELL = BAG_W / N_CELLS
# the filled pouch: chopped vegetables (about 300 g) lie in it 30 mm deep, from 30 mm behind the track
POUCH_X = (BOARD_NEAR + 0.004, BOTTOM_X - 0.004)
POUCH_H = 0.030
COVER_Z = POUCH_H + 0.010              # the top film over the load's loose top layer (above the block)
LOAD_H = 0.022                       # the packed load's solid bulk under its loose top layer
BAG_MASS = 0.32
ROOT_X = (POUCH_X[0] + POUCH_X[1]) / 2
ROOT_Z = BOARD_TOP + 0.0005          # the root (the pouch's base film) half a millimetre thick
GROOVE_Z = BOARD_TOP + PROFILE_T / 2     # groove profile centre (world, as laid)
TRACK_FRICTION = (0.15, 0.12)        # LDPE profile on the fingers' pads (slippery polyethylene)


def cell_y(k: int) -> float:
    return BAG_Y - BAG_W / 2 + (k + 0.5) * CELL


def board_object() -> Part:
    pos = (BOARD_C[0], BOARD_C[1], TOP + BOARD[2] / 2)
    groove = Piece("box", (BOARD[0] - 0.024, 0.004, 0.0012), (0.0, BOARD[1] / 2 - 0.016, BOARD[2] / 2 - 0.0005),
                   color=(0.55, 0.39, 0.24), collide=False)                # juice groove round the edge
    groove2 = Piece("box", (BOARD[0] - 0.024, 0.004, 0.0012), (0.0, -BOARD[1] / 2 + 0.016, BOARD[2] / 2 - 0.0005),
                    color=(0.55, 0.39, 0.24), collide=False)
    return Part("board", pos=pos, size=BOARD, exact_z=True, color=WOOD, material="oak", friction=(0.45, 0.38),
                pieces=(groove, groove2))


def _veg(rng, n, x0, x1, y0, y1, z0, z1):
    """Chopped vegetables: carrot coins, pepper and celery pieces."""
    out = []
    kinds = ((CARROT, "cylinder", (0.0085, 0.005, 0.0085)), (PEPPER, "box", (0.018, 0.010, 0.005)),
             (PEPPER_G, "box", (0.016, 0.011, 0.005)), (CELERY, "box", (0.020, 0.008, 0.006)))
    for i in range(n):
        col, shape, size = kinds[i % len(kinds)]
        at = (rng.uniform(x0, x1), rng.uniform(y0, y1), rng.uniform(z0, z1))
        out.append(((col, shape, size), at, (*rng.uniform(-0.3, 0.3, 2), rng.uniform(-3.1, 3.1))))
    return out


TRACK_PIECES: dict[str, list[int]] = {}     # piece indices of each cell's groove, rib and sealed strip


def bag_object() -> Part:
    """The filled bag as one rigid body (its root: the pouch's base film on the block).

    The renderer draws no see-through film, so the clear film is drawn by what shows of it: the puffed
    rolls of film round the load's edges, the sheen across its top, the flat seams, the open mouth's
    films and flanges. The vegetables show through between, and carry the pouch's collision.
    """
    z0 = BOARD_TOP
    rel = lambda x, y, z: (x - ROOT_X, y - BAG_Y, z - ROOT_Z)
    pieces: list[Piece] = []
    add = pieces.append
    x0, x1 = POUCH_X
    plen = x1 - x0
    r = 0.011                                          # the puffed film rolls round the load
    ye = BAG_W / 2 - 0.010
    # the load: vegetables packed 30 mm deep (they collide: a hand pressing the bag presses them)
    rng = np.random.default_rng(7)
    for (col, shape, size), at, rot in _veg(rng, 110, x0 + 0.014, x1 - 0.014, BAG_Y - ye + 0.008, BAG_Y + ye - 0.008,
                                            z0 + 0.012, z0 + POUCH_H - 0.005):
        add(Piece(shape, size, rel(*at), rot=rot, axis="Z", color=col))
    # the load's bulk: the packed vegetables as one solid slab under the loose top layer, so a hand
    # pressing the bag bears on the whole load (its colour is the dark between the pieces)
    add(Piece("box", (plen - 0.020, BAG_W - 0.036, LOAD_H), rel((x0 + x1) / 2, BAG_Y, z0 + 0.0010 + LOAD_H / 2),
              color=(0.28, 0.36, 0.18)))
    # film rolls round the load (rounded corners), lying a little proud at the sides
    add(Piece("capsule", (r, BAG_W - 2 * 0.010 - 2 * r, r), rel(x0 + r, BAG_Y, z0 + r + 0.002), axis="Y", color=FILM, collide=False))
    add(Piece("capsule", (r, BAG_W - 2 * 0.010 - 2 * r, r), rel(x1 - r, BAG_Y, z0 + r + 0.002), axis="Y", color=FILM, collide=False))
    for s in (-1, 1):
        add(Piece("capsule", (r, plen - 2 * r, r), rel((x0 + x1) / 2, BAG_Y + s * (ye - r), z0 + r + 0.002), axis="X",
                  color=FILM, collide=False))
        for xe in (x0 + r, x1 - r):
            add(Piece("sphere", (r, r, r), rel(xe, BAG_Y + s * (ye - r), z0 + r + 0.002), color=FILM, collide=False))
    # the side seams: flat welded film the full length of the bag, and the bottom fold
    for s in (-1, 1):
        add(Piece("box", (BAG_L - 0.002, 0.009, 0.0012), rel(MOUTH_X + BAG_L / 2, BAG_Y + s * (BAG_W / 2 - 0.0045), z0 + 0.0006),
                  color=FILM_EDGE, collide=False))
    add(Piece("capsule", (0.0025, BAG_W - 0.010, 0.0025), rel(BOTTOM_X - 0.0025, BAG_Y, z0 + 0.0025), axis="Y",
              color=FILM_EDGE, collide=False))
    # sheen: light catching the top film stretched over the load (drawn only)
    for k, (dx, dy, a) in enumerate(((0.020, -0.030, 0.55), (0.055, 0.010, 0.55), (0.085, 0.045, 0.55))):
        add(Piece("box", (0.075 - 0.015 * k, 0.003, 0.0004), rel(x0 + 0.030 + dx, BAG_Y + dy, z0 + POUCH_H + 0.0015),
                  rot=(0.0, 0.0, a), color=SHEEN, collide=False))
    # the write-on label printed on the top film, near the bottom
    add(Piece("box", (0.034, 0.080, 0.0005), rel(x1 - 0.030, BAG_Y - 0.030, z0 + POUCH_H + 0.0012), color=WHITE, collide=False))
    for j in range(3):
        add(Piece("box", (0.0008, 0.064, 0.0003), rel(x1 - 0.040 + 0.009 * j, BAG_Y - 0.030, z0 + POUCH_H + 0.0016),
                  color=(0.55, 0.62, 0.75), collide=False))
    # the neck: the bottom film flat from the track to the load; the top film rising from the open rib
    # to the top of the load, the dark inside showing between them
    n0, n1 = TRACK_X + TRACK_W / 2, x0 + r
    add(Piece("box", (n1 - n0, BAG_W - 0.004, 0.0010), rel((n0 + n1) / 2, BAG_Y, z0 + 0.0005), color=FILM_EDGE))
    add(Piece("box", (n1 - n0 - 0.004, BAG_W - 0.022, 0.0004), rel((n0 + n1) / 2 + 0.002, BAG_Y, z0 + 0.0012),
              color=INSIDE, collide=False))
    rise = COVER_Z - (PROFILE_T + GAPE)
    tilt = math.atan2(rise, n1 - n0)
    # (drawn whole; it collides only from 20 mm behind the track, clear of the fingers pinching the track)
    add(Piece("box", (math.hypot(n1 - n0, rise), BAG_W - 0.004, 0.0008),
              rel((n0 + n1) / 2, BAG_Y, z0 + PROFILE_T + GAPE + rise / 2 + 0.0008), rot=(0.0, -tilt, 0.0),
              color=FILM, collide=False))
    f = 0.020 / (n1 - n0)
    xa = n0 + f * (n1 - n0)
    add(Piece("box", (math.hypot(n1 - xa, rise * (1 - f)), BAG_W - 0.016, 0.0008),
              rel((xa + n1) / 2, BAG_Y, z0 + PROFILE_T + GAPE + rise * (1 + f) / 2 + 0.0004), rot=(0.0, -tilt, 0.0),
              color=FILM))
    # the top film over the load: it collides, drawn as what shows of clear film (the light catching it in
    # bands across, 16 mm apart), and the side and bottom walls of film round the load's loose top layer
    nb = int(round((x1 - n1) / 0.016))
    for i in range(nb + 1):
        xb = n1 + (x1 - 0.002 - n1) * i / nb
        add(Piece("box", (0.0025, BAG_W - 0.016, 0.0008), rel(xb, BAG_Y, z0 + COVER_Z), color=SHEEN))
    for sgn in (-1, 1):
        add(Piece("box", (x1 - n1, 0.0008, COVER_Z - 0.018), rel((n1 + x1) / 2, BAG_Y + sgn * (ye - 0.002), z0 + (COVER_Z + 0.018) / 2),
                  color=FILM))
    add(Piece("box", (0.0008, BAG_W - 0.016, COVER_Z - 0.018), rel(x1 - 0.002, BAG_Y, z0 + (COVER_Z + 0.018) / 2), color=FILM))
    # the track: groove profile (bottom film), rib profile (top film) above it, and the sealed strip that
    # replaces both where it is pressed home (drawn hidden until then)
    TRACK_PIECES.clear()
    for name in ("groove", "rib", "sealed"):
        TRACK_PIECES[name] = []
    for k in range(N_CELLS):
        TRACK_PIECES["groove"].append(len(pieces))
        add(Piece("box", (TRACK_W, CELL - 0.0003, PROFILE_T), rel(TRACK_X, cell_y(k), z0 + PROFILE_T / 2),
                  color=YELLOW, collide=False))
    for k in range(N_CELLS):
        TRACK_PIECES["rib"].append(len(pieces))
        add(Piece("box", (TRACK_W, CELL - 0.0003, PROFILE_T), rel(TRACK_X, cell_y(k), z0 + PROFILE_T + GAPE + PROFILE_T / 2),
                  color=BLUE, collide=False))
    for k in range(N_CELLS):
        TRACK_PIECES["sealed"].append(len(pieces))
        add(Piece("box", (TRACK_W + 0.0006, CELL - 0.0003, 2 * PROFILE_T + GAPE + 0.0004), rel(TRACK_X, cell_y(k), z0 + (2 * PROFILE_T + GAPE) / 2),
                  color=GREEN, collide=False))         # drawn over the rib and groove where pressed home
    # the track's colliders: the groove and the rib profiles, each as overlapping 13 mm segments (2 mm
    # overlaps, coplanar, so a pad slides along them without catching on seams; one 177 mm box would be
    # too long and thin for PhysX's convex cooking). Each sits 0.1 mm inside the cells drawn over it, and a
    # closed cell's green strip encloses both; the lower pad presses the groove, the upper pad the rib.
    TRACK_PIECES["strips"] = []
    for z in (z0 + PROFILE_T / 2, z0 + PROFILE_T + GAPE + PROFILE_T / 2):
        for k in range(N_CELLS):
            TRACK_PIECES["strips"].append(len(pieces))
            add(Piece("box", (TRACK_W - 0.0002, CELL + 0.002, PROFILE_T - 0.0002), rel(TRACK_X, cell_y(k), z),
                      color=YELLOW if z < z0 + PROFILE_T else BLUE, friction=TRACK_FRICTION))
    # the flanges: the films' 8 mm free edges past the track, the top one turned up at the open mouth
    fl = TRACK_X - TRACK_W / 2 - MOUTH_X
    add(Piece("box", (fl, BAG_W - 0.004, 0.0010), rel(MOUTH_X + fl / 2, BAG_Y, z0 + 0.0005), color=FILM_EDGE))
    add(Piece("box", (fl, BAG_W - 0.004, 0.0006), rel(MOUTH_X + fl / 2, BAG_Y, z0 + 2 * PROFILE_T + GAPE + 0.0012),
              rot=(0.0, 0.30, 0.0), color=FILM, collide=False))
    return Part("bag", pos=(ROOT_X, BAG_Y, ROOT_Z), size=(plen, BAG_W - 0.010, 0.001), exact_z=True, mass=BAG_MASS,
                color=FILM_EDGE, friction=(0.40, 0.32), restitution=0.05, contact_offset=0.002, pieces=tuple(pieces),
                contact_report=True)


def scraps_object() -> Part:
    """Vegetable ends left on the block (stand-ins): a carrot top, pepper stem and core, a celery end (fixed: set dressing)."""
    c = (BOARD_C[0] + 0.090, BOARD_C[1] + 0.155, BOARD_TOP + 0.006)
    ps = (Piece("cone", (0.012, 0.030, 0.012), (0.020, 0.010, 0.006), rot=(0.0, 1.5708, 0.3), color=CARROT),
          Piece("cylinder", (0.004, 0.012, 0.004), (0.002, 0.016, 0.008), rot=(0.0, 1.5708, 0.3), color=PEPPER_G),
          Piece("box", (0.030, 0.020, 0.012), (-0.030, -0.012, 0.0), rot=(0.0, 0.0, 0.5), color=PEPPER),
          Piece("box", (0.024, 0.012, 0.010), (0.010, -0.030, 0.0), rot=(0.0, 0.0, -0.4), color=CELERY))
    return Part("scraps", pos=c, size=(0.020, 0.012, 0.010), exact_z=True, color=CELERY, pieces=ps)


def bowl_object() -> Part:
    """A 1.5-qt stainless prep bowl (200 mm across the rim, 85 mm deep) of more chopped vegetables behind
    the block (fixed: set dressing)."""
    rim_r, h = 0.100, 0.085
    c = (0.645, 0.200, TOP + 0.003)
    n = 16
    walls = []
    for k in range(n):
        a = 2 * np.pi * k / n
        rm = rim_r - 0.017
        walls.append(Piece("box", (0.004, 2 * rim_r * np.tan(np.pi / n) + 0.002, h),
                           (rm * np.cos(a), rm * np.sin(a), h / 2), rot=(0.0, 0.35, a), color=STEEL))
    rng = np.random.default_rng(3)
    veg = [Piece(shape, size, (*rng.uniform(-0.050, 0.050, 2), rng.uniform(0.015, 0.045)), rot=rot, color=col, collide=False)
           for (col, shape, size), _, rot in _veg(rng, 60, 0, 1, 0, 1, 0, 1)]
    return Part("bowl", pos=c, size=(0.060, 0.006, 0.060), shape="cylinder", exact_z=True, color=STEEL,
                pieces=tuple(walls) + tuple(veg))


def bag_box_object() -> Part:
    """The open carton of quart freezer bags (190 x 75 x 95 mm: the folded 177 mm bags lie in it across
    its length) behind the block (fixed: set dressing)."""
    size = (0.075, 0.190, 0.095)
    c = (0.660, -0.200, TOP + size[2] / 2)
    ps = (Piece("box", (0.076, 0.170, 0.028), (0.0, 0.0, 0.018), color=BLUE, collide=False),
          Piece("box", (0.076, 0.170, 0.008), (0.0, 0.0, -0.008), color=YELLOW, collide=False),
          Piece("box", (0.050, 0.150, 0.002), (0.0, 0.0, size[2] / 2 + 0.001), color=(0.30, 0.30, 0.32), collide=False),
          Piece("box", (0.004, 0.120, 0.030), (0.0, 0.0, size[2] / 2 + 0.010), rot=(0.0, 0.2, 0.0), color=FILM, collide=False))
    return Part("bag_box", pos=c, size=size, exact_z=True, color=(0.95, 0.95, 0.96), pieces=ps)


# -- measurements -----------------------------------------------------------------------------------

def quat_matrix(q):
    """Rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_bag(pose, p):
    """World point ``p`` in the bag's frame as laid (origin the track's middle at the groove's centre)."""
    pose = np.asarray(pose, float)
    r = quat_matrix(pose[3:7])
    local = r.T @ (np.asarray(p, float) - pose[:3])
    return local + np.array([ROOT_X - TRACK_X, 0.0, ROOT_Z - GROOVE_Z])


def track_world(pose):
    """The track's two end points (world) for the bag's measured pose."""
    pose = np.asarray(pose, float)
    r = quat_matrix(pose[3:7])
    ends = []
    for y in (BAG_Y - BAG_W / 2, BAG_Y + BAG_W / 2):
        ends.append(pose[:3] + r @ np.array([TRACK_X - ROOT_X, y - BAG_Y, GROOVE_Z - ROOT_Z]))
    return ends


def yaw_tilt(pose):
    r = quat_matrix(np.asarray(pose, float)[3:7])
    return math.degrees(math.atan2(r[1, 0], r[0, 0])), math.degrees(math.acos(float(np.clip(r[2, 2], -1, 1))))


# -- the load's loose top layer: six carrot coins free inside the bag, the ones that would spill ----------
COIN_R, COIN_H, COIN_MASS = 0.0085, 0.005, 0.0015
COINS = tuple((POUCH_X[0] + 0.035 + 0.040 * (i % 3), BAG_Y + (-0.035 if i < 3 else 0.035)) for i in range(6))


def coin_object(i: int) -> Part:
    x, y = COINS[i]
    return Part(f"coin{i + 1}", pos=(x, y, BOARD_TOP + POUCH_H + COIN_H / 2 + 0.0015), size=(COIN_R, COIN_H, COIN_R),
                shape="cylinder", axis="Z", exact_z=True, mass=COIN_MASS, color=CARROT, friction=(0.5, 0.4),
                restitution=0.05, contact_offset=0.002)


def in_bag(bag_pose, p):
    """A point inside the bag (in its frame as laid): between the track and the bottom fold, inside the
    side seams and under the top film."""
    bag_pose = np.asarray(bag_pose, float)
    r = quat_matrix(bag_pose[3:7])
    q = r.T @ (np.asarray(p, float)[:3] - bag_pose[:3]) + np.array([ROOT_X, BAG_Y, ROOT_Z])
    return bool(TRACK_X <= q[0] <= BOTTOM_X and abs(q[1] - BAG_Y) <= BAG_W / 2 and q[2] - BOARD_TOP <= COVER_Z + 0.004)


# -- the hands' finger pads, for clearance ------------------------------------------------------------
PAD_T, PAD_W, PAD_L = 0.008, 0.024, 0.045      # a fingertip pad box: thickness, width across the jaw, length


def pad_boxes(robot, pad_tip):
    """Both hands' finger pads as oriented boxes (centre, axes, half-sizes), from the measured joints and
    finger travel through the URDF's forward kinematics at the runner's arm bases (station.BASES)."""
    from yam_arena import ik, station
    boxes = []
    for side in ("left", "right"):
        q = np.asarray(robot[side]["joints"], float)
        pos, R = ik.tcp_pose(q, np.asarray(station.BASES[side], float))
        half = abs(float(robot[side]["fingers"][0]))
        tip = pos + R[:, 2] * pad_tip
        for sgn in (-1.0, 1.0):
            c = tip + R[:, 0] * sgn * (half + PAD_T / 2) - R[:, 2] * (PAD_L / 2)
            boxes.append((c, R.copy(), np.array([PAD_T / 2, PAD_W / 2, PAD_L / 2])))
    return boxes


def segment_box_distance(a, b, box, n=81):
    c, R, h = box
    t = np.linspace(0.0, 1.0, n)[:, None]
    local = (a + t * (b - a) - c) @ R
    return float(np.linalg.norm(np.maximum(np.abs(local) - h, 0.0), axis=1).min())
