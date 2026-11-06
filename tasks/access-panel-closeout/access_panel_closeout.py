"""On the station table, on the midline, a 1 g bench analogue of access-panel close-out in aircraft and spacecraft
integration: the cover goes into its recess flush, then retainers are set across the joint. A fixed light grey frame
holds a 5 mm pocket ring with 3 mm of clearance per side around a dark grey 160 x 80 x 16 mm panel, flanked by two
16 mm frame plates whose tops are flush with a seated panel. Two orange retainer bars lie stowed along x in keyways on
the plates. A bar is set by lifting it out and laying it across the joint, turned 90 degrees, in a slot between two
ribs on the plate, its outer end within 4 mm of the ribs' outer faces and at least 20 mm of it on the panel. Each bar
bridges the panel and the plate, so its seat exists only once the panel is flush: a bar laid first tips into the
empty pocket, and a panel left proud on the ring tilts the bar. The score reads the panel from its centre and every
corner, and each bar from its centre, its two ends and its quaternion.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/access-panel-closeout/access_panel_closeout.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Seat the grey panel flush in its pocket, then lay each orange retainer bar flat across the panel edge into its rib slot, at least 20 mm onto the panel." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
F = (0.250, 0.000)  # the pocket centre, on the midline
PANEL = (0.160, 0.080, 0.016)
PANEL_MASS = 0.12
BAR = (0.060, 0.016, 0.016)
BAR_MASS = 0.010
CLEAR = 0.003  # panel to ring, per side
RING_H = 0.005
RING_T = 0.010
PLATE = (0.220, 0.080, 0.016)  # its top is flush with a seated panel's top
PLATE_Y = PANEL[1] / 2 + CLEAR + RING_T + PLATE[1] / 2  # 93 mm, plate centre from the midline
PLATE_TOP = TOP + PLATE[2]
RIB_H = 0.010  # every rib stands on a plate
STOW_Y = 0.113  # a stowed bar's centre, from the midline
SLOT_Y = 0.046  # a set bar's centre, from the midline: it spans 16 to 76 mm, 24 mm on a centred panel
SLOT_RIB_Y = 0.071  # slot rib centres, spanning 66 to 76 mm from the midline
SLOT_OUTER = SLOT_RIB_Y + 0.005  # 76 mm: the ribs' outer faces, where a set bar's outer end lies
SLOT_GAP = BAR[1] + 2 * 0.003  # 22 mm between the slot ribs' inner faces, 3 mm per side
KEY_GAP = BAR[1] + 2 * 0.002  # 20 mm between the keyway ribs' inner faces, 2 mm per side
PANEL_START = (0.220, 0.330)  # flat, long axis along y

DARK = (0.20, 0.20, 0.22)
ORANGE = (0.95, 0.55, 0.10)
LIGHT = (0.80, 0.80, 0.82)

_ring_y = PANEL[1] / 2 + CLEAR + RING_T / 2  # 48 mm
_ring_x = PANEL[0] / 2 + CLEAR + RING_T / 2  # 88 mm
_rib_z = PLATE_TOP + RIB_H / 2
RING = [
    Box("ring_left", pos=(F[0], F[1] + _ring_y, TOP + RING_H / 2), size=(PANEL[0] + 2 * (CLEAR + RING_T), RING_T, RING_H), color=LIGHT),
    Box("ring_right", pos=(F[0], F[1] - _ring_y, TOP + RING_H / 2), size=(PANEL[0] + 2 * (CLEAR + RING_T), RING_T, RING_H), color=LIGHT),
    Box("ring_far", pos=(F[0] + _ring_x, F[1], TOP + RING_H / 2), size=(RING_T, PANEL[1] + 2 * CLEAR, RING_H), color=LIGHT),
    Box("ring_near", pos=(F[0] - _ring_x, F[1], TOP + RING_H / 2), size=(RING_T, PANEL[1] + 2 * CLEAR, RING_H), color=LIGHT),
]
PLATES = [Box(f"plate_{side}", pos=(F[0], s * PLATE_Y, TOP + PLATE[2] / 2), size=PLATE, color=LIGHT) for side, s in (("left", 1), ("right", -1))]
KEYWAYS = [
    Box(f"keyway_{side}_{k}", pos=(F[0] + dx, s * (STOW_Y + dy), _rib_z), size=(0.010, 0.004, RIB_H), color=LIGHT)
    for side, s in (("left", 1), ("right", -1))
    for k, (dx, dy) in enumerate(((-0.027, -(KEY_GAP / 2 + 0.002)), (-0.027, KEY_GAP / 2 + 0.002), (0.027, -(KEY_GAP / 2 + 0.002)), (0.027, KEY_GAP / 2 + 0.002)))
]
SLOT_RIBS = [
    Box(f"slot_{side}_{k}", pos=(F[0] + dx, s * SLOT_RIB_Y, _rib_z), size=(0.004, 0.010, RIB_H), color=LIGHT)
    for side, s in (("left", 1), ("right", -1))
    for k, dx in enumerate((-(SLOT_GAP / 2 + 0.002), SLOT_GAP / 2 + 0.002))
]
PANEL_BOX = Box("panel", pos=(PANEL_START[0], PANEL_START[1], TOP + PANEL[2] / 2), size=PANEL, mass=PANEL_MASS, color=DARK, yaw=math.pi / 2)
BARS = [Box(f"bar_{side}", pos=(F[0], s * STOW_Y, PLATE_TOP + BAR[2] / 2), size=BAR, mass=BAR_MASS, color=ORANGE) for side, s in (("plus", 1), ("minus", -1))]
OBJECTS = [PANEL_BOX, *BARS, *RING, *PLATES, *KEYWAYS, *SLOT_RIBS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

PANEL_XY_TOL = 0.004  # centre to F; the ring allows 3 mm
PANEL_YAW_TOL = math.radians(3.0)  # the ring allows about 2.1 degrees
FLUSH_TOL = 0.002  # every bottom corner to the table, every top corner to the plate tops
BAR_YAW_TOL = math.radians(10.0)  # long axis to y
BAR_TILT_TOL = math.radians(3.0)  # a face flat
BAR_BOTTOM_TOL = 0.002  # lowest corner to the plate tops
BAR_XY_TOL = 0.004  # centre to the slot's centre line across it; outer end to the ribs' outer faces along it
ONTO = 0.020  # a set bar's length on the panel
EPS = 1e-6
SLOTS = ((F[0], F[1] + SLOT_Y), (F[0], F[1] - SLOT_Y))
LADDER = (0.4, 0.7, 1.0)  # panel seated with 0, 1 or 2 slots filled


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


def _corner_heights(state, size):
    """The z of the box's eight corners, lowest first, from its centre and quaternion."""
    corners = np.asarray(state.pos, float) + (_SIGNS * (np.asarray(size) / 2)) @ _rot(state.quat_xyzw).T
    return np.sort(corners[:, 2])


def _seated(panel) -> bool:
    """Resting in the pocket, flat on the table and flush: centre within 4 mm of F in x and y, long axis within
    3 degrees of x either way round, the four bottom corners within 2 mm of the table and the four top corners
    within 2 mm of the plate tops, so a panel cocked on a ring wall fails."""
    if not panel.resting():
        return False
    if abs(float(panel.pos[0]) - F[0]) > PANEL_XY_TOL + EPS or abs(float(panel.pos[1]) - F[1]) > PANEL_XY_TOL + EPS:
        return False
    ex = _rot(panel.quat_xyzw)[:, 0]
    off = abs(math.atan2(float(ex[1]), float(ex[0])))
    if min(off, math.pi - off) > PANEL_YAW_TOL:
        return False
    z = _corner_heights(panel, PANEL)
    return bool(np.all(np.abs(z[:4] - TOP) <= FLUSH_TOL + EPS) and np.all(np.abs(z[4:] - PLATE_TOP) <= FLUSH_TOL + EPS))


def _set_in(bar, slot, panel) -> bool:
    """Resting flat across the joint in this slot: long axis within 10 degrees of y either way round, a face
    within 3 degrees of flat, lowest corner within 2 mm of the plate tops, centre within 4 mm of the slot's centre
    line across it (x), outer end within 4 mm of the ribs' outer faces along it (y), and its inner end at least
    20 mm inside the panel's edge. The outer end, not the centre, is tested along the slot so that the rule is the
    photo check: a straightedge across the ribs' outer faces."""
    if not bar.resting():
        return False
    r = _rot(bar.quat_xyzw)
    ex = r[:, 0]
    off = abs(math.atan2(float(ex[0]), float(ex[1])))  # angle of the long axis from y
    if min(off, math.pi - off) > BAR_YAW_TOL:
        return False
    if max(abs(float(r[2, 1])), abs(float(r[2, 2]))) < math.cos(BAR_TILT_TOL):  # a side face down, level
        return False
    if abs(float(_corner_heights(bar, BAR)[0]) - PLATE_TOP) > BAR_BOTTOM_TOL + EPS:
        return False
    if abs(float(bar.pos[0]) - slot[0]) > BAR_XY_TOL + EPS:  # across the slot
        return False
    side = 1.0 if slot[1] > F[1] else -1.0
    ends = [np.asarray(bar.pos, float) + s * BAR[0] / 2 * ex for s in (-1, 1)]
    inner, outer = sorted(ends, key=lambda p: abs(float(p[1]) - F[1]))  # toward the pocket, toward the ribs
    if abs(side * (float(outer[1]) - F[1]) - SLOT_OUTER) > BAR_XY_TOL + EPS:  # along the slot
        return False
    local = _rot(panel.quat_xyzw).T @ (inner - np.asarray(panel.pos, float))
    return bool(PANEL[1] / 2 - abs(float(local[1])) >= ONTO - EPS and abs(float(local[0])) <= PANEL[0] / 2)


def _filled(objects) -> int:
    """Slots holding a set bar; either bar may fill either slot, and each slot counts once."""
    panel = objects["panel"]
    return sum(any(_set_in(objects[b.name], slot, panel) for b in BARS) for slot in SLOTS)


def success(objects) -> bool:
    """The panel seated and both slots filled, everything at rest; holds for 1 s."""
    return _seated(objects["panel"]) and _filled(objects) == 2


def distance(objects) -> float:
    """Metres still to go: the panel's centre to F, plus each slot's nearest bar centre to that slot's centre."""
    p = objects["panel"]
    total = math.hypot(float(p.pos[0]) - F[0], float(p.pos[1]) - F[1])
    for sx, sy in SLOTS:
        total += min(math.hypot(float(objects[b.name].pos[0]) - sx, float(objects[b.name].pos[1]) - sy) for b in BARS)
    return total


def score(objects) -> float:
    """0 unless the panel is seated flush; then 0.4, plus 0.3 for each slot holding a set bar: 0, 0.4, 0.7
    or 1.0."""
    if not _seated(objects["panel"]):
        return 0.0
    return LADDER[_filled(objects)]


TRAVEL = TOP + 0.150
PANEL_GRIP_Z = TOP + 0.018  # TCP 10 mm above the panel centre, pad tips 5 mm above the table
PANEL_PLACE_Z = TOP + 0.024  # panel bottom 6 mm above the table, 1 mm above the ring; pad tips 6 mm above the ring
PANEL_ENTRY = 0.012  # the pause above the pocket: panel bottom 13 mm above the ring
PANEL_RELEASE = -0.044  # 88 mm: 4 mm clear of the panel, the outer finger faces about 4 mm inside the plate edges
# Measured in the simulator: with the place commanded at F and yaw 0, the held panel arrived 2.0 mm short in x,
# 0.5 mm left and turned 2.9 degrees (the wrist settles 1.5 degrees past its commanded yaw, and the panel turns
# 1.4 degrees in the grip during the close), more than the 2.1 degrees the ring allows. The place is offset to cancel it.
PANEL_TRIM = (0.0020, -0.0005, math.radians(-2.9))
BAR_GRIP_Z = PLATE_TOP + 0.018  # TCP 10 mm above a stowed bar's centre, pad tips 5 mm above the plate
BAR_PLACE_Z = PLATE_TOP + 0.024  # bar bottom 6 mm above the panel and plate; pad tips 11 mm above them, 1 mm above the ribs
BAR_ENTRY = 0.015  # the pause above the slot: bar bottom 11 mm above the ribs
BAR_OPEN = -0.020  # 40 mm at the stowed bar, 12 mm clear of it, clear of the slot ribs by about 12 mm
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
PARK_LEFT = (0.200, 0.300)
WAIT_RIGHT = (0.220, -0.260)  # the right arm's wait, clear of the left arm working over the pocket


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None, pick_open=FINGER_OPEN, release_open=FINGER_OPEN):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    ``pick_open`` and ``release_open`` open the fingers only part way where a full 94 mm opening would reach a
    fixture. 16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), pick_open, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), pick_open, "descend", yaw),
        Waypoint(0.6, (px, py, pz), pick_open, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), release_open, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), release_open, "retreat", end),
    ]


def demo():
    """Left arm: the panel from its start into the pocket (picked at yaw -90 with the fingers across its 80 mm
    width, set down at yaw 0 less the measured trim and released to 88 mm so the fingers stay inside the plate edges), then bar_plus from
    its keyway into the left slot (picked at yaw 0 with a 40 mm opening, set down turned to yaw -90), then a park.
    Right arm: waits out of the way in front of its base until the left arm has set its bar and parked, then sets
    bar_minus in the right slot the same way at yaw +90. These yaw signs keep joint6 inside its limits. Left 33.9 s,
    right 50.2 s, then the 3 s hold: 1,064 steps at 20 Hz."""
    left = _pick_place(
        (PANEL_START[0], PANEL_START[1], PANEL_GRIP_Z), (F[0] + PANEL_TRIM[0], F[1] + PANEL_TRIM[1], PANEL_PLACE_Z),
        width=PANEL[1], entry=PANEL_ENTRY, travel_z=TRAVEL, yaw=-math.pi / 2, place_yaw=PANEL_TRIM[2], release_open=PANEL_RELEASE,
    )
    left += _pick_place(
        (F[0], STOW_Y, BAR_GRIP_Z), SLOTS[0] + (BAR_PLACE_Z,), width=BAR[1], entry=BAR_ENTRY, travel_z=TRAVEL,
        yaw=0.0, place_yaw=-math.pi / 2, pick_open=BAR_OPEN,
    )
    left += [Waypoint(1.5, (PARK_LEFT[0], PARK_LEFT[1], TRAVEL), FINGER_OPEN, "park", -math.pi / 2)]
    wait = (WAIT_RIGHT[0], WAIT_RIGHT[1], TRAVEL)
    right = [
        Waypoint(1.5, wait, FINGER_OPEN, "out of the way"),
        Waypoint(32.5, wait, FINGER_OPEN, "wait for the left arm to set its bar and park"),
    ]
    right += _pick_place(
        (F[0], -STOW_Y, BAR_GRIP_Z), SLOTS[1] + (BAR_PLACE_Z,), width=BAR[1], entry=BAR_ENTRY, travel_z=TRAVEL,
        yaw=0.0, place_yaw=math.pi / 2, pick_open=BAR_OPEN,
    )
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, roll=0.0, pitch=0.0):  # yaw about z, then roll about the body's x, then pitch about its y
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        r = r @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])

    def body(size, x, y, bottom, q, vel=(0.0, 0.0, 0.0)):
        half = np.abs(_rot(q)) @ (np.asarray(size) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def panel(dx=0.0, dy=0.0, bottom=TOP, yaw=0.0, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        return body(PANEL, F[0] + dx, F[1] + dy, bottom, quat(yaw, roll, pitch), vel)

    def bar(k, along=0.0, across=0.0, bottom=PLATE_TOP, yaw=math.pi / 2, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        """A bar in slot k; ``along`` moves it outward along y, ``across`` along x."""
        sx, sy = SLOTS[k]
        s = 1.0 if sy > F[1] else -1.0
        return body(BAR, sx + across, sy + s * along, bottom, quat(yaw, roll, pitch), vel)

    start_panel = body(PANEL, PANEL_START[0], PANEL_START[1], TOP, quat(math.pi / 2))
    stowed = [body(BAR, F[0], s * STOW_Y, PLATE_TOP, quat()) for s in (1, -1)]

    def scene(p=None, plus=None, minus=None):
        return {"panel": p or start_panel, "bar_plus": plus or stowed[0], "bar_minus": minus or stowed[1]}

    # A panel shifted 3.5 mm in x rests with one end 0.5 mm onto the far ring wall, 5 mm up, the other on the table.
    cock = math.asin(RING_H / PANEL[0])
    cocked = body(PANEL, F[0] + 0.0035, F[1], TOP, quat(0.0, 0.0, -cock))
    # Without a panel a set bar tips about the plate edge until its inner end meets the table.
    tip = math.atan2(PLATE[2], (PLATE_Y - PLATE[1] / 2) - (SLOT_Y - BAR[0] / 2))
    tipped = body(BAR, F[0], SLOT_Y - 0.004, TOP, quat(math.pi / 2, 0.0, -tip))
    # On a panel resting 5 mm proud on the ring all round, a bar pivots on the panel corner: atan(5 / 36), 7.9 deg.
    proud = math.atan2(RING_H, (SLOT_Y + BAR[0] / 2) - PANEL[1] / 2)
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(panel(), bar(0), bar(1)), 1.0, True),
        "panel seated, bars stowed": (scene(panel()), 0.4, False),
        "panel seated, bar_plus set": (scene(panel(), bar(0)), 0.7, False),
        "panel seated, bar_minus set": (scene(panel(), None, bar(1)), 0.7, False),
        "bars swapped between the slots": (scene(panel(), bar(1), bar(0)), 1.0, True),
        "bars set, panel still at its start": (scene(None, bar(0), bar(1)), 0.0, False),
        "no panel, a bar tipped into the pocket": (scene(None, tipped), 0.0, False),
        "panel cocked on the far ring wall, bars set": (scene(cocked, bar(0), bar(1)), 0.0, False),
        "panel cocked, bars tilted 7.9 degrees on it": (scene(cocked, bar(0, bottom=TOP + 0.009, pitch=-proud), bar(1, bottom=TOP + 0.009, pitch=proud)), 0.0, False),
        "panel 5 mm proud, flat": (scene(panel(bottom=TOP + RING_H)), 0.0, False),
        "panel seated, moving": (scene(panel(vel=(0.0, 0.0, 0.02)), bar(0), bar(1)), 0.0, False),
        "panel held 10 mm above the pocket": (scene(panel(bottom=TOP + 0.010), bar(0), bar(1)), 0.0, False),
        "panel seated against a ring wall, 3 mm off in x and y": (scene(panel(dx=0.003, dy=-0.003), bar(0), bar(1)), 1.0, True),
        "panel seated, yawed 2 degrees": (scene(panel(yaw=math.radians(2))), 0.4, False),
        "panel yawed 4 degrees": (scene(panel(yaw=math.radians(4))), 0.0, False),
        "panel 5 mm off in y, flat on the table": (scene(panel(dy=0.005)), 0.0, False),
        "panel upside down, seated": (scene(panel(roll=math.pi), bar(0), bar(1)), 1.0, True),
        "panel reversed end for end, seated": (scene(panel(yaw=math.pi), bar(0), bar(1)), 1.0, True),
        "panel on a frame plate": (scene(body(PANEL, F[0], PLATE_Y, PLATE_TOP, quat()), bar(0)), 0.0, False),
        "bar_plus set, tilted 4 degrees": (scene(panel(), bar(0, pitch=math.radians(4))), 0.4, False),
        "bar_plus set, rolled 4 degrees": (scene(panel(), bar(0, roll=math.radians(4))), 0.4, False),
        "bar_plus set, tilted 2 degrees": (scene(panel(), bar(0, pitch=math.radians(2))), 0.7, False),
        "bar_plus across the joint along x": (scene(panel(), bar(0, yaw=0.0)), 0.4, False),
        "bar_plus yawed 8 degrees in its slot": (scene(panel(), bar(0, yaw=math.pi / 2 + math.radians(8))), 0.7, False),
        "bar_plus yawed 12 degrees": (scene(panel(), bar(0, yaw=math.pi / 2 + math.radians(12))), 0.4, False),
        "bar_plus reversed end for end": (scene(panel(), bar(0, yaw=-math.pi / 2)), 0.7, False),
        "bar_plus rolled a quarter turn about its long axis": (scene(panel(), bar(0, roll=math.pi / 2)), 0.7, False),
        "bar_plus 3 mm off in x, against a rib": (scene(panel(), bar(0, across=0.003)), 0.7, False),
        "bar_plus 5 mm inward, outer end 5 mm short of the rib faces": (scene(panel(), bar(0, along=-0.005)), 0.4, False),
        "bar_plus 3.8 mm inward": (scene(panel(), bar(0, along=-0.0038)), 0.7, False),
        "bar_plus 4.5 mm outward, outer end past the rib faces, 19.5 mm on the panel": (scene(panel(), bar(0, along=0.0045)), 0.4, False),
        "bar_plus against a rib and 3 mm outward": (scene(panel(), bar(0, across=0.003, along=0.003)), 0.7, False),
        "bar_plus yawed 9 degrees, centre 3.8 mm in, outer end 4.2 mm short": (scene(panel(), bar(0, along=-0.0038, yaw=math.pi / 2 + math.radians(9))), 0.4, False),
        "bar_plus laid outside the ribs, 23 mm across": (scene(panel(), bar(0, across=0.023)), 0.4, False),
        "bar_plus 3 mm outward, 21 mm on the panel": (scene(panel(), bar(0, along=0.003)), 0.7, False),
        "panel 3 mm away, bar_plus 2 mm outward: 19 mm on the panel": (scene(panel(dy=-0.003), bar(0, along=0.002)), 0.4, False),
        "bar_plus flat on the panel, fully inside it": (scene(panel(), bar(0, along=-0.026)), 0.4, False),
        "bar_plus on the rib tops": (scene(panel(), bar(0, bottom=PLATE_TOP + RIB_H)), 0.4, False),
        "bar_plus still held 8 mm over its slot, moving": (scene(panel(), bar(0, bottom=PLATE_TOP + 0.008, vel=(0.0, 0.0, -0.02))), 0.4, False),
        "bar_minus stacked on a set bar_plus": (scene(panel(), bar(0), bar(0, bottom=PLATE_TOP + BAR[2])), 0.7, False),
        "bar_plus on the table beside the frame": (scene(panel(), body(BAR, 0.420, 0.100, TOP, quat())), 0.4, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
