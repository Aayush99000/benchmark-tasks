"""Every dimension of the jack-in-the-box, in metres, and the geometry the scorer and the demo share. NumPy only.

Tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from the
table top. The box stands with its front (the side with the catch) toward the robot and its lid hinge at the
back. "Box frame": the same axes with the origin at the middle of the box's footprint on the table.

Each number is marked PUBLISHED (read on a retailer's page, see README.md), DERIVED from published numbers,
or ESTIMATE.

Angles: the lid's opening angle is 0 when shut and grows as its front edge lifts (a rotation about +y at the
hinge). The spring's lean is a rotation about +y at its foot: positive leans the jester toward the hinge
(back), negative toward the front.
"""

import numpy as np

INCH = .0254

# -- the box: Schylling Jester Jack in the Box (tin) ----------------------------------------------------------
BOX_PUBLISHED = (5.25 * INCH, 5.25 * INCH, 5.5 * INCH)   # PUBLISHED 5.25 W x 5.25 D x 5.5 H in, lid shut
TOY_MASS = .454                          # PUBLISHED 1.0 lb, the whole toy
BOX_X = .360                             # where the box stands: middle of its footprint, centred on y = 0
SIDE = BOX_PUBLISHED[0]                  # 133.35 mm square
HALF = SIDE / 2
LID_T = .005                             # modelled lid thickness (real tin about 0.3 mm with a rolled edge)
RIM_Z = BOX_PUBLISHED[2] - LID_T         # DERIVED top of the walls, where the shut lid rests: 134.7 mm
WALL = .006                              # modelled wall (real tin about 0.3 mm; the outside sizes are true)
FLOOR_T = .008                           # modelled base plate
INNER = HALF - WALL                      # half the inside width
HINGE = (HALF, RIM_Z + LID_T / 2)        # box frame (x, z) of the lid's hinge line, along y at the back top edge
LID_OPEN = np.radians(105.)              # ESTIMATE the lid falls back to rest against its hinge stop here
LID_MASS = .050                          # ESTIMATE lid tin with its rolled edge
LID_DAMPING = .002                       # ESTIMATE N m s/rad, a loose tin hinge

# -- the jester ----------------------------------------------------------------------------------------------
HEAD_R = .032                            # ESTIMATE the plush head with its cap, as one 64 mm ball
POPPED_TOP = 10. * INCH                  # PUBLISHED for the same maker's tin jack-in-the-box (KRJB2): "10 inches
                                         # tall when the puppet is out" on a 5.5 in box; the jester's is assumed equal
PIVOT_Z = .020                           # ESTIMATE where the coil spring stands on the box's inner floor
S_POPPED = POPPED_TOP - HEAD_R - PIVOT_Z         # head centre above the spring's foot, sprung right out: 202 mm
S_SHUT = RIM_Z - HEAD_R - PIVOT_Z                # head top level with the rim: 82.7 mm
S_LOW = .074                             # fully pressed: the coil is solid (head top 8.7 mm below the rim)
SPRING_RATE = 44.                        # ESTIMATE N/m
SPRING_PRELOAD = .8                      # ESTIMATE N, sprung right out (the cloth sleeve is taut)
S_FREE = S_POPPED + SPRING_PRELOAD / SPRING_RATE
SPRING_DAMPING = .4                      # ESTIMATE N s/m: springs up in about 0.2 s and bobs
HEAD_MASS = .035                         # ESTIMATE head, cap, bells and the top of the sleeve
LINK_MASS = .015                         # ESTIMATE the coil's lower part
LEAN_LIMIT = np.radians(30.)             # how far the coil can bend over, either way
LEAN_RATE = 2.0                          # ESTIMATE N m/rad, the coil's bending stiffness
LEAN_DAMPING = .02                       # N m s/rad
HEAD_COLUMN = (.020, .030, .084)         # sleeve collider under the head: radius, from and to below the head centre
LINK_COLUMN = (.016, .005, .100)         # the coil's lower collider: radius, from and to above the foot

# -- the catch: a sprung tin hook inside the front wall catches a barb under the lid's front edge ---------------
FRONT_IN = -INNER                        # box-frame x of the front wall's inner face
CATCH_TRAVEL = .005                      # the hook slides this far forward into the wall when pushed
CATCH_RATE = 400.                        # ESTIMATE N/m
CATCH_PRELOAD = .0015                    # m: 0.6 N against its stop at rest
CATCH_MASS = .010
CATCH_X = (FRONT_IN - .004, FRONT_IN + .004)     # the hook's back face stands 4 mm proud of the wall at rest
CATCH_Z = (RIM_Z - .0095, RIM_Z - .0030)         # flat underside, sloped top
CATCH_Y = .012                           # half width
BARB_X = (FRONT_IN + .001, FRONT_IN + .006)      # lid shut: the barb's front face 1 mm behind the wall
BARB_Z = (RIM_Z - .0170, RIM_Z - .0105)          # lid shut: its top 1 mm under the hook's underside
TONGUE_X = (FRONT_IN + .006, FRONT_IN + .011)
TONGUE_Z = (RIM_Z - .0170, RIM_Z)
CATCH_FRICTION = .3

# -- friction (ESTIMATE) -----------------------------------------------------------------------------------------
FRICTION = {"tin": .30, "plush": .80, "base": .50}

# -- crank (display and collider; not used) ------------------------------------------------------------------
CRANK_Z = .070

BOX_MASS = TOY_MASS - LID_MASS - HEAD_MASS - LINK_MASS - CATCH_MASS
ACTORS = ("box", "lid", "coil", "head", "catch")
MASSES = (BOX_MASS, LID_MASS, LINK_MASS, HEAD_MASS, CATCH_MASS)


def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))


def quat_y(angle):
    """(x, y, z, w) of a rotation about +y."""
    return (0., float(np.sin(angle / 2)), 0., float(np.cos(angle / 2)))


def hinge_world():
    return np.array((BOX_X + HINGE[0], 0., HINGE[1]))


def lid_point(angle, x_rel, z_rel=0., y=0.):
    """Tabletop point of the lid at ``x_rel`` (<= 0, measured from the hinge line toward the front edge) and
    ``z_rel`` above the lid's mid-plane, with the lid open ``angle``."""
    return hinge_world() + rot_y(angle) @ np.array((x_rel, y, z_rel))


def lid_normal(angle):
    """Outward normal of the lid's top face."""
    return rot_y(angle) @ np.array((0., 0., 1.))


def lid_angle_from_rotation(rotation):
    """Opening angle from the lid body's rotation matrix (the box upright)."""
    x_axis = np.asarray(rotation)[:, 0]
    return float(np.arctan2(-x_axis[2], x_axis[0]))


def lid_underside_z(angle, x):
    """Height of the lid's underside above the tabletop x (lid open ``angle``, box at its start)."""
    hinge = hinge_world()
    dx = hinge[0] - x                              # distance in front of the hinge line
    c, s = np.cos(angle), np.sin(angle)
    if c < 1e-6:
        return np.inf
    return hinge[2] + dx * np.tan(angle) - (LID_T / 2) / c


def head_centre(s, lean, box_x=BOX_X):
    """Tabletop centre of the head for spring extension ``s`` and lean."""
    return np.array((box_x + s * np.sin(lean), 0., PIVOT_Z + s * np.cos(lean)))


def spring_force(s):
    return SPRING_RATE * (S_FREE - s)


def start_poses():
    """Tabletop start pose (position, xyzw quaternion) of every body's origin. Box: middle of its footprint on
    the table. Lid: on the hinge line, open LID_OPEN. Coil: its foot. Head: the head's centre, sprung out.
    Catch: its rest position."""
    hinge = hinge_world()
    return {"box": ((BOX_X, 0., .0005), (0., 0., 0., 1.)),
            "lid": (tuple(hinge + (0., 0., .0005)), quat_y(LID_OPEN)),
            "coil": ((BOX_X, 0., PIVOT_Z + .0005), (0., 0., 0., 1.)),
            "head": ((BOX_X, 0., PIVOT_Z + S_POPPED + .0005), (0., 0., 0., 1.)),
            "catch": ((BOX_X, 0., .0005), (0., 0., 0., 1.))}


def box_hull(box_pos=(BOX_X, 0., 0.)):
    """(low, high) of the box with its lid shut, for the hands-clear rule (tabletop frame)."""
    centre = np.asarray(box_pos, dtype=float)
    low = centre + (-HALF - .003, -HALF - .030, 0.)            # -y: the crank stands 30 mm off the right side
    high = centre + (HALF + .006, HALF, BOX_PUBLISHED[2])
    low[2], high[2] = 0., BOX_PUBLISHED[2]
    return low, high


assert abs(S_POPPED - .202) < .0005 and abs(S_SHUT - .0827) < .0005
assert PIVOT_Z + S_LOW - HEAD_COLUMN[2] > FLOOR_T + .001
assert S_LOW < S_SHUT < S_POPPED
assert BARB_Z[1] < CATCH_Z[0] and BARB_X[0] < CATCH_X[1] < BARB_X[1] and TONGUE_X[0] > CATCH_X[1]
