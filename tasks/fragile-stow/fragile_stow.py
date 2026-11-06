"""On the station table, in the left arm's lane, a 1 g bench analogue of packing a cargo locker before launch: the
packed locker has to ride out the ascent loads, so a case that holds glass must carry nothing on top of it. A fixed
two-bay locker, open at the top, stands in front of the arm. Each bay takes one 60 x 40 mm case footprint with 4 mm
of clearance per side and is two cases deep. Three loose cases of one size wait in a row beside it: a grey one at
each end and a blue one in the middle, and the blue one holds glass. Three cases and two bays mean that one bay must
take a stack of two, and that stack must not stand on the blue case. A case resting on the blue case, or a stack of
three, scores 0 whatever else happened; otherwise each case lying flat and lengthwise in a bay, on the floor or on
the case below it, earns a third. Every horizontal test uses each case's centre and quaternion.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/fragile-stow/fragile_stow.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Stow the three cases flat and lengthwise in the two locker bays, at most two per bay. The blue case holds glass, so nothing may rest on it." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
CASE = (0.060, 0.040, 0.030)  # long axis along x at the start; the 60 x 40 faces are the flat ones
CASE_MASS = 0.10
POCKET = (0.068, 0.048)  # each bay's floor between the walls: 4 mm of clearance per side around a case
RACK_Y = 0.310  # the locker's centre line, running along x
BAY_X = (0.180, 0.258)  # bay A (near), bay B (far)
WALL_T = 0.010  # end walls and the divider: thickness along x
WALL_H = 0.045  # higher than one case, lower than two
LIP_T = 0.008  # side lips: thickness along y
LIP_H = 0.008
WALL_XS = (BAY_X[0] - POCKET[0] / 2 - WALL_T / 2, (BAY_X[0] + BAY_X[1]) / 2, BAY_X[1] + POCKET[0] / 2 + WALL_T / 2)
LIP_LEN = WALL_XS[2] - WALL_XS[0] + WALL_T  # the full locker length, 166 mm
LIP_OFF = POCKET[1] / 2 + LIP_T / 2  # 28 mm, a lip's centre from the locker's centre line
STARTS = (("case_grey_a", (0.160, 0.185)), ("case_blue", (0.240, 0.185)), ("case_grey_b", (0.320, 0.185)))  # blue in the middle

LOCKER_COLOR = (0.25, 0.25, 0.28)
GREY = (0.62, 0.62, 0.62)
BLUE = (0.15, 0.35, 0.85)

LOCKER = [
    Box("locker_wall_front", pos=(WALL_XS[0], RACK_Y, TOP + WALL_H / 2), size=(WALL_T, POCKET[1], WALL_H), color=LOCKER_COLOR),
    Box("locker_wall_mid", pos=(WALL_XS[1], RACK_Y, TOP + WALL_H / 2), size=(WALL_T, POCKET[1], WALL_H), color=LOCKER_COLOR),
    Box("locker_wall_back", pos=(WALL_XS[2], RACK_Y, TOP + WALL_H / 2), size=(WALL_T, POCKET[1], WALL_H), color=LOCKER_COLOR),
    Box("locker_lip_out", pos=(WALL_XS[1], RACK_Y + LIP_OFF, TOP + LIP_H / 2), size=(LIP_LEN, LIP_T, LIP_H), color=LOCKER_COLOR),
    Box("locker_lip_in", pos=(WALL_XS[1], RACK_Y - LIP_OFF, TOP + LIP_H / 2), size=(LIP_LEN, LIP_T, LIP_H), color=LOCKER_COLOR),
]
CASES = [Box(name, pos=(x, y, TOP + CASE[2] / 2), size=CASE, mass=CASE_MASS, color=BLUE if name == "case_blue" else GREY) for name, (x, y) in STARTS]
BLUE_NAME = "case_blue"
OBJECTS = [*CASES, *LOCKER]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

FLAT_TOL = math.radians(5.0)  # the case's 30 mm axis to vertical, either face up
LEVEL_TOL = 0.004  # lowest point to the bay floor (level 1) or to the top of one case (level 2)
BAY_MARGIN = 0.004  # a case's outline may reach this far past the bay's inner outline
EPS = 1e-4  # 0.1 mm, so a top case exactly 8 mm off centre still counts
OVERLAP = 0.005  # seen from above, a case over the blue case by at least this much
RISE = 0.010  # and reaching this far above the blue case's top
REACH = 0.006  # and coming down to within this of it: it rests on the blue case (or is pressed onto it)
HALF = np.asarray(CASE) / 2
SIGNS = np.array([(sx, sy, sz) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _corners(state) -> np.ndarray:
    """The case's eight corners in the world, from its centre and quaternion."""
    return np.asarray(state.pos, float) + (SIGNS * HALF) @ _rot(state.quat_xyzw).T


def _hull(points) -> np.ndarray:
    """Convex hull of 2-D points, counter-clockwise (monotone chain): the case's outline seen from above."""
    pts = sorted({(round(float(p[0]), 9), round(float(p[1]), 9)) for p in points})

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and (out[-1][0] - out[-2][0]) * (p[1] - out[-2][1]) - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0]) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(pts[::-1]))


def _overlap(a, b) -> float:
    """Penetration depth of two convex outlines (separating-axis test): negative when they are apart."""
    depth = math.inf
    for poly in (a, b):
        for k in range(len(poly)):
            edge = poly[(k + 1) % len(poly)] - poly[k]
            axis = np.array([-edge[1], edge[0]]) / max(float(np.hypot(*edge)), 1e-12)
            pa, pb = a @ axis, b @ axis
            depth = min(depth, min(pa.max(), pb.max()) - max(pa.min(), pb.min()))
    return float(depth)


def _rests_on(upper, lower) -> bool:
    """True when the case with corners ``upper`` rests on the case with corners ``lower``: seen from above it overlaps
    it by 5 mm or more, it rises more than 10 mm above its top and its lowest point is within 6 mm of that top or below
    it. That covers a case lying on the other, leaning on its top edge, or held pressed onto it."""
    top = float(lower[:, 2].max())
    if upper[:, 2].max() <= top + RISE or upper[:, 2].min() > top + REACH:
        return False
    return _overlap(_hull(upper[:, :2]), _hull(lower[:, :2])) >= OVERLAP


def _gated(objects) -> bool:
    """The two hard rules: a grey case resting on the blue case, or a stack of three (a case resting on a case that
    rests on the third), wherever it stands. Either one scores 0."""
    corners = {c.name: _corners(objects[c.name]) for c in CASES}
    on = {(u, v) for u in corners for v in corners if u != v and _rests_on(corners[u], corners[v])}
    if any(v == BLUE_NAME for _, v in on):
        return True
    return any((v, w) in on for u, v in on for w in corners if w not in (u, v))


def _slot(state):
    """(bay, level) for a case lying flat and still inside a bay's outline with its lowest point on the bay floor
    (level 1) or one case height above it (level 2); None otherwise."""
    if not state.resting() or abs(float(_rot(state.quat_xyzw)[2, 2])) < math.cos(FLAT_TOL):
        return None
    corners = _corners(state)
    if float(np.abs(corners[:, 1] - RACK_Y).max()) > POCKET[1] / 2 + BAY_MARGIN + EPS:
        return None
    bottom = float(corners[:, 2].min())
    level = 1 if abs(bottom - TOP) <= LEVEL_TOL else 2 if abs(bottom - TOP - CASE[2]) <= LEVEL_TOL else 0
    for bay, cx in enumerate(BAY_X):
        if level and float(np.abs(corners[:, 0] - cx).max()) <= POCKET[0] / 2 + BAY_MARGIN + EPS:
            return bay, level
    return None


def _stowed(objects) -> list:
    """Names of the stowed cases: flat on a bay floor, or flat on a case that is itself flat on that bay's floor.
    A third case on top of a stack of two is never stowed, and score() gates such a stack to 0."""
    slots = {c.name: _slot(objects[c.name]) for c in CASES}
    floor = {s[0] for s in slots.values() if s and s[1] == 1}
    return [n for n, s in slots.items() if s and (s[1] == 1 or s[0] in floor)]


def success(objects) -> bool:
    """All three cases stowed, nothing on the blue case, no stack of three, every case at rest; holds for 1 s."""
    return not _gated(objects) and len(_stowed(objects)) == 3


def distance(objects) -> float:
    """Metres still to go: each case's centre to the nearest bay centre, summed."""
    return sum(min(math.hypot(float(objects[c.name].pos[0]) - cx, float(objects[c.name].pos[1]) - RACK_Y) for cx in BAY_X) for c in CASES)


def score(objects) -> float:
    """Gate first: 0 if a grey case rests on the blue case or three cases stand in one stack, wherever they are.
    Otherwise a third per stowed case: 0, 1/3, 2/3 or 1."""
    if _gated(objects):
        return 0.0
    return len(_stowed(objects)) / 3.0


TRAVEL = TOP + 0.140
GRIP_H = 0.025  # the TCP 25 mm above a case's bottom: pad tips 12 mm up it, 18 mm of pad on its 60 x 30 faces
PICK_Z = TOP + GRIP_H
PLACE_FLOOR = TOP + 0.005 + GRIP_H  # case bottom 5 mm above the bay floor; pad tips 9 mm above the 8 mm lips
PLACE_STACK = TOP + CASE[2] + 0.005 + GRIP_H  # case bottom 5 mm above the case below
ENTRY_FLOOR = WALL_H + 0.010 - 0.005  # the pause above a bay: case bottom 10 mm above the 45 mm walls
ENTRY_STACK = WALL_H + 0.010 - CASE[2] - 0.005
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), FINGER_OPEN, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.6, (px, py, pz), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), FINGER_OPEN, "retreat", end),
    ]


def demo():
    """Left arm only, yaw 0, the fingers closing across each case's 40 mm width. Grey case A into bay A, the blue case
    alone into bay B, then grey case B on top of grey case A; each a top-down pick, a carry at 14 cm and a 5 mm drop.
    48.6 s of plan, then the 3 s hold: 1,032 steps at 20 Hz."""
    start = dict(STARTS)
    left = []
    left += _pick_place((*start["case_grey_a"], PICK_Z), (BAY_X[0], RACK_Y, PLACE_FLOOR), width=CASE[1], entry=ENTRY_FLOOR, travel_z=TRAVEL)
    left += _pick_place((*start[BLUE_NAME], PICK_Z), (BAY_X[1], RACK_Y, PLACE_FLOOR), width=CASE[1], entry=ENTRY_FLOOR, travel_z=TRAVEL)
    left += _pick_place((*start["case_grey_b"], PICK_Z), (BAY_X[0], RACK_Y, PLACE_STACK), width=CASE[1], entry=ENTRY_STACK, travel_z=TRAVEL)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, pitch=0.0, roll=0.0):  # yaw about the vertical, then pitch about the case's y, then roll about its x
        def q(axis, a):
            s = math.sin(a / 2)
            return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(a / 2)])

        def mul(p, r):
            (x1, y1, z1, w1), (x2, y2, z2, w2) = p, r
            return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])

        return mul(mul(q((0, 0, 1), yaw), q((0, 1, 0), pitch)), q((1, 0, 0), roll))

    def case(x, y, bottom=TOP, vel=(0.0, 0.0, 0.0), **turn):
        qt = quat(**turn)
        half = np.abs(_rot(qt)) @ HALF
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, qt, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def bay(k, level=1, dx=0.0, dy=0.0, **kw):
        return case(BAY_X[k] + dx, RACK_Y + dy, bottom=TOP + (level - 1) * CASE[2], **kw)

    start = {name: case(x, y) for name, (x, y) in STARTS}
    at = dict(STARTS)

    def scene(**moved):
        return {**start, **{f"case_{k}": st for k, st in moved.items()}}

    A, B = 0, 1
    # A grey case leaning 40 degrees on the blue case's far top edge, its lower edge on the table 36 mm beyond it
    # (the other grey case moved into bay A to make room).
    pitch = math.radians(40)
    lean_x = at[BLUE_NAME][0] + CASE[0] / 2 + CASE[2] / math.tan(pitch) - HALF[0] * math.cos(pitch) + HALF[2] * math.sin(pitch)
    cases = {
        "start": (scene(), 0.0, False),
        "blue alone in a, greys stacked in b": (scene(blue=bay(A), grey_a=bay(B), grey_b=bay(B, 2)), 1.0, True),
        "greys stacked in a, blue alone in b": (scene(grey_a=bay(A), grey_b=bay(A, 2), blue=bay(B)), 1.0, True),
        "a grey in each bay, blue on top of one": (scene(grey_a=bay(A), grey_b=bay(B), blue=bay(B, 2)), 1.0, True),
        "a grey on the blue case in a bay": (scene(blue=bay(A), grey_a=bay(A, 2), grey_b=bay(B)), 0.0, False),
        "a grey on the blue case on the table": (scene(grey_a=case(*at[BLUE_NAME], bottom=TOP + CASE[2])), 0.0, False),
        "the nearest-bay plan, near end first: grey in a, a grey on the blue in b": (scene(grey_a=bay(A), blue=bay(B), grey_b=bay(B, 2)), 0.0, False),
        "all three stacked in a, blue on top": (scene(grey_a=bay(A), grey_b=bay(A, 2), blue=bay(A, 3)), 0.0, False),
        "all three stacked in b, blue on top": (scene(grey_a=bay(B), grey_b=bay(B, 2), blue=bay(B, 3)), 0.0, False),
        "all three stacked in a, blue in the middle": (scene(grey_a=bay(A), blue=bay(A, 2), grey_b=bay(A, 3)), 0.0, False),
        "three stacked in a, the top case 15 mm past the bay": (scene(grey_a=bay(A), grey_b=bay(A, 2), blue=bay(A, 3, dx=-0.015)), 0.0, False),
        "three stacked in a, the middle case 10 mm off centre": (scene(grey_a=bay(A), grey_b=bay(A, 2, dy=0.010), blue=bay(A, 3, dy=0.010)), 0.0, False),
        "three stacked on the table": (scene(grey_b=case(*at["case_grey_a"], bottom=TOP + CASE[2]), blue=case(*at["case_grey_a"], bottom=TOP + 2 * CASE[2])), 0.0, False),
        "the blue case held 3 mm above a stack of two greys": (scene(grey_a=bay(A), grey_b=bay(A, 2), blue=case(BAY_X[A], RACK_Y, bottom=TOP + 2 * CASE[2] + 0.003, vel=(0, 0, 0.02))), 0.0, False),
        "the blue case held 50 mm above a stack of two greys": (scene(grey_a=bay(A), grey_b=bay(A, 2), blue=case(BAY_X[A], RACK_Y, bottom=TOP + 2 * CASE[2] + 0.05, vel=(0, 0, 0.02))), 2 / 3, False),
        "blue in a, a grey in b, a grey left on the table": (scene(blue=bay(A), grey_a=bay(B)), 2 / 3, False),
        "only the blue case stowed": (scene(blue=bay(A)), 1 / 3, False),
        "greys stacked in b, blue left on the table": (scene(grey_a=bay(B), grey_b=bay(B, 2)), 2 / 3, False),
        "a grey leaning on the blue case's top edge on the table": (scene(grey_a=case(lean_x, at[BLUE_NAME][1], pitch=pitch), grey_b=bay(A)), 0.0, False),
        "a grey still held 3 mm above the blue case": (scene(blue=bay(A), grey_a=bay(B), grey_b=case(BAY_X[A], RACK_Y, bottom=TOP + CASE[2] + 0.003, vel=(0, 0, 0.02))), 0.0, False),
        "a grey held 50 mm above the blue case": (scene(blue=bay(A), grey_a=bay(B), grey_b=case(BAY_X[A], RACK_Y, bottom=TOP + CASE[2] + 0.05, vel=(0, 0, 0.02))), 2 / 3, False),
        "done but the top case still moving": (scene(blue=bay(A), grey_a=bay(B), grey_b=bay(B, 2, vel=(0.02, 0, 0))), 2 / 3, False),
        "done with the blue case upside down": (scene(blue=bay(A, roll=math.pi), grey_a=bay(B), grey_b=bay(B, 2)), 1.0, True),
        "done with a grey case turned end for end": (scene(blue=bay(A), grey_a=bay(B, yaw=math.pi), grey_b=bay(B, 2)), 1.0, True),
        "a case on its side in a bay": (scene(blue=bay(A, roll=math.pi / 2)), 0.0, False),
        "a case tilted 8 degrees in a bay": (scene(blue=bay(A, roll=math.radians(8))), 0.0, False),
        "a case tilted 4 degrees in a bay counts": (scene(blue=bay(A, roll=math.radians(4))), 1 / 3, False),
        "a case lying on the wall tops": (scene(blue=case(WALL_XS[1], RACK_Y, bottom=TOP + WALL_H)), 0.0, False),
        "the top case 8 mm off centre still counts": (scene(blue=bay(A), grey_a=bay(B), grey_b=bay(B, 2, dy=0.008)), 1.0, True),
        "the top case 10 mm off centre does not": (scene(blue=bay(A), grey_a=bay(B), grey_b=bay(B, 2, dy=0.010)), 2 / 3, False),
        "the top case set crosswise, not lengthwise, overhangs the bay": (scene(blue=bay(A), grey_a=bay(B), grey_b=bay(B, 2, yaw=math.pi / 2)), 2 / 3, False),
        "the top case turned 12 degrees still counts": (scene(blue=bay(A), grey_a=bay(B), grey_b=bay(B, 2, yaw=math.radians(12))), 1.0, True),
        "a grey alone at level 2 height with nothing under it in the bay": (scene(grey_a=bay(B, 2)), 0.0, False),
        "a grey flat beside the locker on the table": (scene(grey_a=case(0.219, 0.400)), 0.0, False),
        "blue on top of a grey on the table, nothing stowed": (scene(blue=case(*at["case_grey_a"], bottom=TOP + CASE[2])), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    lean = _corners(cases["a grey leaning on the blue case's top edge on the table"][0]["case_grey_a"])
    assert abs(float(lean[:, 2].min()) - TOP) < 1e-9 and float(lean[:, 0].min()) < at[BLUE_NAME][0] + CASE[0] / 2 - OVERLAP
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
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
