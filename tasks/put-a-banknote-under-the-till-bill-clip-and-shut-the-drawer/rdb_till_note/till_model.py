"""Every dimension, mass and spring of the cash drawer, its till, the bill hold-downs and the banknote.

NumPy only. Metres, kilograms, newtons. Two frames are used:

* tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from the table top;
* drawer frame: the tabletop frame moved along x so that its origin is under the outer face of the drawer front.
  With the drawer open by ``s`` metres the origin is at tabletop x = FRONT_CLOSED_X - s.

Status of each number: PUBLISHED (read on a maker or retailer page, see README), DERIVED (follows from published
numbers) or ESTIMATE (to be measured on the bought item; the README lists them).
"""

import numpy as np

# --- cash drawer: APG Vasario VB320-BL1616 -----------------------------------------------------------------
CASE_W, CASE_D, CASE_H = .411, .414, .109        # PUBLISHED: 41.1 x 41.4 x 10.9 cm (16.2 x 16.3 x 4.3 in)
DRAWER_TOTAL_MASS = 7.6                          # PUBLISHED: 16.8 lb, drawer and till together
CASE_FRONT_X = .470                              # where the case's front edge is put on the table
CASE_WALL, CASE_TOP_T, CASE_BOTTOM_T = .008, .009, .010      # collider thicknesses (sheet steel is about 1 mm)
TRAVEL = .310                                    # ESTIMATE: how far the drawer comes out
PANEL_T, PANEL_W, PANEL_Z = .020, .407, (.004, .105)         # ESTIMATE: drawer front panel
FRONT_CLOSED_X = CASE_FRONT_X - PANEL_T          # tabletop x of the front face with the drawer shut
FRONT_OPEN_X = FRONT_CLOSED_X - TRAVEL
DRAWER_FLOOR_X, DRAWER_FLOOR_HALF_Y, DRAWER_FLOOR_Z = (.020, .345), .190, (.012, .020)
DRAWER_SIDE_T, DRAWER_SIDE_Z = .004, (.020, .070)
DRAWER_MASS = 3.4                                # ESTIMATE: sliding part (drawer 2.3 kg + till 1.1 kg); case 4.2 kg
SLIDE_FRICTION_N = 2.0                           # ESTIMATE: roller slide

# --- till insert: APG VPK-15B-2A-BX, 5 note and 5 coin compartments ---------------------------------------
TILL_W, TILL_D, TILL_H = .361, .307, .060        # PUBLISHED: 361 x 307 x 60 mm (14.2 x 12.05 x 2.35 in)
TILL_MASS = 1.13                                 # PUBLISHED by a retailer: 2.5 lb
TILL_X0, TILL_Z0 = .026, DRAWER_FLOOR_Z[1]       # drawer frame: front of the till, its underside
RIM_Z = TILL_Z0 + TILL_H                         # 80 mm above the table
WALL = .002                                      # ESTIMATE: moulded wall thickness
PITCH = TILL_W / 5                               # DERIVED: 72.2 mm per compartment
COMPARTMENT_Y = tuple((i - 2) * PITCH for i in range(5))     # centres, right-hand one first (y = -144 mm)
COMPARTMENT_W = PITCH - WALL                     # DERIVED from the estimate: 70.2 mm inside
COIN_X = (TILL_X0 + WALL, .130)                  # ESTIMATE: coin cups at the front
COIN_FLOOR_Z = .030
DIVIDER_X = (.130, .132)                         # wall between coins and notes, as high as the rim
BILL_X = (.132, .304)                            # ESTIMATE: note compartments, 172 mm long inside
BILL_FLOOR_Z = .040                              # ESTIMATE: 40 mm deep
REAR_X = (.304, TILL_X0 + TILL_D)                # rear wall and the ledge the hold-downs are hinged on
HINGE_BLOCK_Z = .090
STOCKED = 2                                      # the middle compartment holds the notes

# --- bill hold-downs: spring-loaded plastic arms that flip up (PUBLISHED description; sizes ESTIMATES) ----
PIVOT_X, PIVOT_Z = .300, .084                    # drawer frame
ARM_L, ARM_W, ARM_T = .100, .016, .004
TAB_X, TAB_H = (.088, .100), .016                # finger tab standing on the arm's free end
HOLD_MASS = .006
HOLD_LIMITS_DEG = (-24.5, 100.)             # the lower stop holds an arm just clear of an empty floor                   # angle above level
HOLD_SPRING = .06                                # N m/rad
HOLD_SPRING_REST_DEG = -165.                     # spring's unloaded angle: about 0.15 N m (1.5 N at the tip) when down
HOLD_DAMPING = .002                              # N m s/rad
HOLD_ARMATURE = 1e-5

# --- the notes -------------------------------------------------------------------------------------------
NOTE_L, NOTE_W = .156, .0663                     # PUBLISHED: US note 6.14 x 2.61 in
NOTE_REAL_T, NOTE_REAL_MASS = .00011, .001       # PUBLISHED: 0.0043 in, about 1 g
STRIPS = 8
STRIP_L = NOTE_L / STRIPS                        # 19.5 mm
NOTE_T = .002                                    # collider thickness (real 0.11 mm), so that pads can hold it
STRIP_MASS = .001                                # 8 g in all (real 1 g), for steady contact
NOTE_EI_REAL = 2.2e-5                            # N m^2: E 3 GPa x 66.3 mm x (0.11 mm)^3 / 12  (ESTIMATE of E)
# One hinge stands for 19.5 mm of paper; stiffness scaled with the mass so that the note droops as a real one.
HINGE_K = NOTE_EI_REAL / STRIP_L * (STRIPS * STRIP_MASS / NOTE_REAL_MASS)      # 9.0e-3 N m/rad
HINGE_DAMPING = 2e-4                             # N m s/rad
HINGE_ARMATURE = 2e-6
HINGE_LIMIT_DEG = 120.
NOTE_FRICTION = .5                               # ESTIMATE: paper on paper and on painted steel
STACK_NOTES = 50
STACK_SIZE = (NOTE_L, NOTE_W, .006)              # 50 used notes: 5.5 mm, drawn and collided as 6 mm
STACK_MASS = STACK_NOTES * NOTE_REAL_MASS
STACK_X = (BILL_X[0] + BILL_X[1]) / 2            # drawer frame: centred in its compartment
STACK_TOP_Z = BILL_FLOOR_Z + STACK_SIZE[2]

# The customer's note lies on top of the drawer case, over its front edge, above the right-hand compartment.
OVERHANG = .050
NOTE_START_Y = COMPARTMENT_Y[0]
NOTE_START_X = (CASE_FRONT_X - OVERHANG, CASE_FRONT_X - OVERHANG + NOTE_L)
NOTE_START_Z = CASE_H + NOTE_T / 2


def drawer_x(s):
    """Tabletop x of the drawer frame's origin with the drawer open by ``s``."""
    return FRONT_CLOSED_X - s


def hold_axes(angle):
    """Unit vectors along the hold-down arm (pivot to free end) and across it (its upper side), in x-z."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array((-c, 0., s)), np.array((s, 0., c))


def hold_point(angle, along, above, s=TRAVEL, compartment=STOCKED):
    """A point of a hold-down, tabletop frame: ``along`` the arm from the pivot, ``above`` its mid-plane."""
    u, n = hold_axes(angle)
    return np.array((drawer_x(s) + PIVOT_X, COMPARTMENT_Y[compartment], PIVOT_Z)) + along * u + above * n


def tab_centre(angle, s=TRAVEL, compartment=STOCKED):
    return hold_point(angle, sum(TAB_X) / 2, ARM_T / 2 + TAB_H / 2, s, compartment)


def tip_underside(angle, s=TRAVEL, compartment=STOCKED):
    """The lowest point of the arm's free end: what presses on the notes."""
    return hold_point(angle, ARM_L, -ARM_T / 2, s, compartment)


def rest_angle(surface_z):
    """Angle at which the free end's underside rests on a level surface at height ``surface_z``."""
    lo, hi = np.radians(HOLD_LIMITS_DEG[0]), 0.
    for _ in range(60):
        mid = (lo + hi) / 2
        if tip_underside(mid)[2] > surface_z:
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


HOLD_ON_STACK = rest_angle(STACK_TOP_Z)
HOLD_ON_NOTE = rest_angle(STACK_TOP_Z + NOTE_T)
HOLD_ON_FLOOR = np.radians(HOLD_LIMITS_DEG[0])     # in an empty compartment the arm sits on its lower stop


def hold_torque(angle):
    """Spring torque (N m) pressing the arm down at ``angle``."""
    return HOLD_SPRING * (angle - np.radians(HOLD_SPRING_REST_DEG))


def strip_centres_flat(x0, y, z):
    """Centres of the eight strips of a flat note whose near edge is at x0 (strip 0 is nearest the robot)."""
    return np.array([(x0 + (i + .5) * STRIP_L, y, z) for i in range(STRIPS)])


NOTE_START = strip_centres_flat(NOTE_START_X[0], NOTE_START_Y, NOTE_START_Z)
STACK_START = np.array((drawer_x(TRAVEL) + STACK_X, COMPARTMENT_Y[STOCKED], BILL_FLOOR_Z + STACK_SIZE[2] / 2))


def support_height(x, y, s):
    """Height of the highest fixed surface under tabletop point (x, y): case top, till walls and floors, drawer
    front, table. The loose stack of notes is not included. Used by dry runs and by the grader's tests."""
    if x >= CASE_FRONT_X and abs(y) <= CASE_W / 2 and x <= CASE_FRONT_X + CASE_D:
        return CASE_H
    d = x - drawer_x(s)
    if d < 0 or abs(y) > PANEL_W / 2:
        return 0.
    if d <= PANEL_T:
        return PANEL_Z[1]
    if d < TILL_X0 or d > REAR_X[1] or abs(y) > TILL_W / 2:
        return DRAWER_FLOOR_Z[1] if abs(y) <= DRAWER_FLOOR_HALF_Y and d <= DRAWER_FLOOR_X[1] else 0.
    lane = (y + TILL_W / 2) % PITCH
    if lane < WALL / 2 or lane > PITCH - WALL / 2:
        return RIM_Z
    if d < COIN_X[0] or DIVIDER_X[0] <= d <= DIVIDER_X[1]:
        return RIM_Z
    if d < DIVIDER_X[0]:
        return COIN_FLOOR_Z
    if d < BILL_X[1]:
        return BILL_FLOOR_Z
    return HINGE_BLOCK_Z if d >= REAR_X[0] + WALL else RIM_Z


def compartment_of(y):
    """Index of the note compartment whose inside contains lateral position ``y``, or None (on a wall, outside)."""
    for i, centre in enumerate(COMPARTMENT_Y):
        if abs(y - centre) <= COMPARTMENT_W / 2:
            return i
    return None


def solid_boxes(s=TRAVEL):
    """The fixed and sliding solids as (name, low corner, high corner) in the tabletop frame, drawer open by ``s``.
    The same boxes build_scene.py writes as colliders; used for clearance checks in dry runs."""
    hw, x0 = CASE_W / 2, drawer_x(s)
    c0, c1 = CASE_FRONT_X, CASE_FRONT_X + CASE_D
    boxes = [("case_top", (c0, -hw, CASE_H - CASE_TOP_T), (c1, hw, CASE_H)),
             ("case_bottom", (c0, -hw, 0.), (c1, hw, CASE_BOTTOM_T)),
             ("case_side_right", (c0, -hw, 0.), (c1, -hw + CASE_WALL, CASE_H)),
             ("case_side_left", (c0, hw - CASE_WALL, 0.), (c1, hw, CASE_H))]
    tw = TILL_W / 2

    def add(name, x, y, z):
        boxes.append((name, (x0 + x[0], y[0], z[0]), (x0 + x[1], y[1], z[1])))

    add("front_panel", (0., PANEL_T), (-PANEL_W / 2, PANEL_W / 2), PANEL_Z)
    add("drawer_floor", DRAWER_FLOOR_X, (-DRAWER_FLOOR_HALF_Y, DRAWER_FLOOR_HALF_Y), DRAWER_FLOOR_Z)
    add("till_coin_floor", (TILL_X0, DIVIDER_X[0]), (-tw, tw), (TILL_Z0, COIN_FLOOR_Z))
    add("till_bill_floor", (DIVIDER_X[0], REAR_X[0]), (-tw, tw), (TILL_Z0, BILL_FLOOR_Z))
    add("till_front", (TILL_X0, COIN_X[0]), (-tw, tw), (TILL_Z0, RIM_Z))
    add("till_divider", DIVIDER_X, (-tw, tw), (TILL_Z0, RIM_Z))
    add("till_rear_wall", (REAR_X[0], REAR_X[0] + WALL), (-tw, tw), (TILL_Z0, RIM_Z))
    add("till_hinge_block", (REAR_X[0] + WALL, REAR_X[1]), (-tw, tw), (TILL_Z0, HINGE_BLOCK_Z))
    for k in range(6):
        y = -tw + k * PITCH
        add(f"till_wall_{k}", (TILL_X0, REAR_X[0]), (y - WALL / 2, y + WALL / 2), (TILL_Z0, RIM_Z))
    return boxes
