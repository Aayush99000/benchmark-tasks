"""Clear four glasses from the table into the bus tub in one trip. A 20 x 15 x 5 in bus tub stands on the station table
between the two arms, its long side running away from the robot. Two 16 oz stackable plastic restaurant tumblers stand on
the table on each side of it, 10 cm apart, each still holding three plastic ice cubes and a straw. Each hand has to bring
its two tumblers rim to rim, take both at once (one finger down inside each, pinching the two walls together), lift them
over the tub's 12.5 cm wall and stand them on the tub's floor, then let go and come away.

Run it:  inspect-robots run --embodiment yam_bus -E task=task.py -E control_hz=20 --policy yam_scripted --no-prompt \\
           --instruction "Clear the four glasses from the table into the bus tub in one trip: each hand goes to the tub only once." --scorer task_score

`yam_bus` (glass_bussing/embodiment.py) spawns the tub, the tumblers and their contents and reports every body's pose and
each gripper's fingertip midpoint. Everything is rigid contact and friction: the tumblers are hollow, with 2.5 mm walls;
nothing is attached to a gripper, and the score reads only poses.

What the score keeps through the trial (latched; nothing a robot does clears it):
- Knocked over: a tumbler more than 45 degrees from upright at any moment.
- Off the table: any body below the table top.
- Carries: each time a tumbler's base passes inward over the tub's rim, the gripper nearest it (within 12 cm) is noted.
  Tumblers that pass over the rim in the same gripper within 1 s of each other are one carry. One trip means no gripper
  made more than one carry.
- Nested: a tumbler's base inside another tumbler at any moment (with ice and a straw in each they do not nest; the
  straw is crushed and the upper tumbler rides on the ice).
"""

import math
import os

import numpy as np

from glass_bussing import lean
from glass_bussing import mech as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the fixed starting layout ----------------------------------------------------------------------------------------
TUB_XY = (0.370, 0.0)  # the tub's centre; its long side runs along x, from 11.7 to 62.3 cm in front of the arm bases
ROW_Y = 0.320  # each pair of tumblers stands this far from the midline, outside the tub's long wall
NEAR_X = 0.220  # the nearer tumbler of each pair
GAP = 0.100  # clear gap between the two rims of a pair
FAR_X = NEAR_X + M.RIM_D + GAP
GLASSES = ("g0", "g1", "g2", "g3")  # left near, left far, right near, right far
GLASS_XY = {"g0": (NEAR_X, ROW_Y), "g1": (FAR_X, ROW_Y), "g2": (NEAR_X, -ROW_Y), "g3": (FAR_X, -ROW_Y)}
STRAW_LEAN = {"g0": 0.5 * math.pi, "g1": 0.5 * math.pi, "g2": -0.5 * math.pi, "g3": -0.5 * math.pi}  # away from the tub
ICE = tuple(f"{g}_i{k}" for g in GLASSES for k in range(M.N_ICE))
STRAWS = tuple(f"{g}_s" for g in GLASSES)
CONTENTS = {g: tuple(f"{g}_i{k}" for k in range(M.N_ICE)) + (f"{g}_s",) for g in GLASSES}


def _quat(R):
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.asarray(R, float)))


def _start():
    out = {"tub": ((TUB_XY[0], TUB_XY[1], TOP), (0.0, 0.0, 0.0, 1.0))}
    for g in GLASSES:
        x, y = GLASS_XY[g]
        out[g] = ((x, y, TOP), (0.0, 0.0, 0.0, 1.0))
        for suffix, p, R in M.contents_layout(STRAW_LEAN[g]):
            out[f"{g}_{suffix}"] = ((x + p[0], y + p[1], TOP + p[2]), _quat(R))
    return out


START = _start()  # name -> (position, rotation xyzw) of each body's own frame, world
_MASS = {"tub": M.TUB_MASS, **{g: M.GLASS_MASS for g in GLASSES}, **{n: M.ICE_MASS for n in ICE}, **{n: M.STRAW_MASS for n in STRAWS}}
OBJECTS = [Box(n, pos=tuple(p), size=(0.01, 0.01, 0.01), mass=_MASS[n]) for n, (p, _) in START.items()]  # yam_bus builds the real shapes

# -- the score's criteria ---------------------------------------------------------------------------------------------
TIPPED = math.radians(45.0)  # a tumbler past this from upright has been knocked over
UPRIGHT = math.radians(15.0)  # standing: within this of upright (a 2 cm difference between the rim's high and low sides)
ON_FLOOR = 0.008  # standing on the tub's floor: the base's centre no higher than this above it
HOLDER = 0.12  # the gripper carrying a tumbler has its fingertip midpoint within this of the tumbler's rim centre
TOGETHER_S = 1.0  # tumblers passing over the rim in one gripper within this time are one carry
CLEAR = 0.06  # hands clear: each fingertip midpoint this far from every tumbler's surface
LIN, ANG, HAND_SPEED, STILL_S = 0.01, 0.5, 0.02, 1.0
BELOW = 0.05  # a body's origin this far below the table top is off the table
SUCCESS_HOLD_S = 0.5


def _fresh():
    return {"tipped": False, "off": False, "nested": False, "inside": {g: False for g in GLASSES}, "carries": {"left": [], "right": [], "none": []},
            "still": 0.0, "t": 0.0, "tcp": {}}


_trial = _fresh()


def reset_latch():
    """Forget the trial: the embodiment calls this at every reset. Nothing a robot does clears a latch."""
    _trial.clear()
    _trial.update(_fresh())


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def _in_frame(objects, body, p):
    """World point ``p`` in ``body``'s own frame."""
    b = objects[body]
    return _rot(b.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(b.pos, float))


def tilt(objects, g):
    """A tumbler's angle from upright (rad)."""
    return math.acos(max(-1.0, min(1.0, float(_rot(objects[g].quat_xyzw)[2, 2]))))


def rim_centre(objects, g):
    return np.asarray(objects[g].pos, float) + _rot(objects[g].quat_xyzw)[:, 2] * M.GLASS_H


def over_tub(objects, p):
    """Is world point ``p`` inside the tub's outline (its rim's outer edge), seen from above the tub?"""
    q = _in_frame(objects, "tub", p)
    return abs(q[0]) <= M.TUB_L / 2 and abs(q[1]) <= M.TUB_W / 2


def in_tub(objects, p):
    """Is world point ``p`` inside the tub: between its walls and above its underside?"""
    q = _in_frame(objects, "tub", p)
    hx, hy = M.tub_inside(q[2])
    return abs(q[0]) <= hx and abs(q[1]) <= hy and -0.002 <= q[2] <= M.TUB_H + 0.30


def in_glass(objects, g, p, margin=0.004):
    """Is world point ``p`` inside tumbler ``g``'s hollow?"""
    q = _in_frame(objects, g, p)
    return 0.0 <= q[2] <= M.GLASS_H + margin and math.hypot(q[0], q[1]) <= M.r_in(min(q[2], M.GLASS_H)) + margin


def surface_distance(objects, g, p):
    """Distance from world point ``p`` to tumbler ``g``'s outside surface (0 inside it or in its hollow)."""
    q = _in_frame(objects, g, p)
    rho, z = math.hypot(q[0], q[1]), float(q[2])
    zc = min(max(z, 0.0), M.GLASS_H)
    return math.hypot(max(rho - M.r_out(zc), 0.0), z - zc)


def holds_contents(objects, g):
    """Are the tumbler's three ice cubes and its straw all inside it?"""
    return all(in_glass(objects, g, objects[n].pos) for n in CONTENTS[g])


def standing_in_tub(objects, g):
    """Standing on the tub's floor: base centre between the tub's walls and within ON_FLOOR of its floor, within UPRIGHT
    of upright, and at rest."""
    q = _in_frame(objects, "tub", objects[g].pos)
    hx, hy = M.TUB_BOT
    b = objects[g]
    return bool(abs(q[0]) <= hx and abs(q[1]) <= hy and -0.003 <= q[2] - M.TUB_FLOOR <= ON_FLOOR and tilt(objects, g) <= UPRIGHT
                and np.linalg.norm(b.lin_vel) <= LIN and np.linalg.norm(b.ang_vel) <= ANG)


def delivered(objects):
    """The tumblers standing in the tub with their ice and straw inside."""
    return [g for g in GLASSES if standing_in_tub(objects, g) and holds_contents(objects, g)]


def spilled(objects):
    """Ice cubes and straws that are neither in a tumbler nor in the tub: on the table or the floor."""
    return [n for n in ICE + STRAWS if not in_tub(objects, objects[n].pos) and not any(in_glass(objects, g, objects[n].pos) for g in GLASSES)]


def hands_clear(objects):
    return all(surface_distance(objects, g, objects[f"{a}_tcp"].pos) >= CLEAR for a in ("left", "right") for g in GLASSES)


def carries():
    """Per gripper, the number of separate carries of tumblers over the tub's rim ('none': a tumbler that went in with no
    gripper near it)."""
    out = {}
    for arm, times in _trial["carries"].items():
        n, last = 0, None
        for t in times:
            if last is None or t - last > TOGETHER_S:
                n += 1
            last = t
        out[arm] = n
    return out


def one_trip():
    c = carries()
    return c["left"] <= 1 and c["right"] <= 1 and c["none"] == 0 and not _trial["nested"]


def note_step(objects, dt):
    """Called once per control step with every body's state."""
    tr = _trial
    tr["t"] += dt
    for g in GLASSES:
        if tilt(objects, g) > TIPPED:
            tr["tipped"] = True
        inside = over_tub(objects, objects[g].pos)
        if inside and not tr["inside"][g]:  # its base has just passed inward over the rim
            rc = rim_centre(objects, g)
            d = {a: float(np.linalg.norm(np.asarray(objects[f"{a}_tcp"].pos, float) - rc)) for a in ("left", "right")}
            arm = min(d, key=d.get)
            tr["carries"][arm if d[arm] <= HOLDER else "none"].append(tr["t"])
        tr["inside"][g] = inside
        for h in GLASSES:
            if h != g:
                q = _in_frame(objects, h, objects[g].pos)
                if 0.005 < q[2] < M.GLASS_H - 0.005 and math.hypot(q[0], q[1]) < M.r_in(q[2]):
                    tr["nested"] = True
    if any(objects[n].pos[2] < TOP - BELOW for n in START):
        tr["off"] = True
    moving = any(np.linalg.norm(objects[g].lin_vel) > LIN or np.linalg.norm(objects[g].ang_vel) > ANG for g in GLASSES)
    for a in ("left", "right"):
        p = np.asarray(objects[f"{a}_tcp"].pos, float)
        if a in tr["tcp"] and np.linalg.norm(p - tr["tcp"][a]) / dt > HAND_SPEED:
            moving = True
        tr["tcp"][a] = p
    tr["still"] = 0.0 if moving else tr["still"] + dt


def score(objects):
    """0: a tumbler knocked over (past 45 degrees) at any time; anything off the table; an ice cube or straw lying
       outside the tub and outside every tumbler; or fewer than two tumblers delivered.
    0.25: two or three tumblers delivered.
    0.5: all four delivered, but not in one trip (a gripper made more than one carry over the tub's rim, or a tumbler
       was put inside another), or a hand is still within 6 cm of a tumbler, or something is still moving.
    1.0: all four delivered in one trip, both hands 6 cm or more from every tumbler, everything still for 1 s.
    Delivered: standing on the tub's floor within 15 degrees of upright, at rest, its three ice cubes and straw inside it."""
    tr = _trial
    if tr["tipped"] or tr["off"] or spilled(objects):
        return 0.0
    n = len(delivered(objects))
    if n < 2:
        return 0.0
    if n < 4:
        return 0.25
    return 1.0 if one_trip() and hands_clear(objects) and tr["still"] >= STILL_S else 0.5


def success(objects):
    return score(objects) >= 1.0


def debug_line(objects):
    tr = _trial
    parts = []
    for g in GLASSES:
        p = objects[g].pos
        parts.append(f"{g} ({p[0]:.3f},{p[1]:+.3f},{p[2] - TOP:+.3f}) {math.degrees(tilt(objects, g)):4.1f}d{'c' if holds_contents(objects, g) else 'X'}")
    for a in ("left", "right"):
        p = objects[f"{a}_tcp"].pos
        f = objects.get(f"{a}_fingers")
        parts.append(f"{a[0]} ({p[0]:.3f},{p[1]:+.3f},{p[2] - TOP:+.3f})" + (f" f {1000 * f.pos[0]:.1f},{1000 * f.pos[1]:.1f}" if f is not None else ""))
    q = _in_frame(objects, "tub", (TUB_XY[0], TUB_XY[1], TOP))
    return " | ".join(parts) + f" | tub moved {1000 * math.hypot(q[0], q[1]):.0f}mm | del {len(delivered(objects))} car {carries()} tip {tr['tipped']} nest {tr['nested']} spill {len(spilled(objects))} still {tr['still']:.1f} score {score(objects)}"


# -- the scripted demo: open loop, from the fixed layout only ----------------------------------------------------------
ARMS = {"left": 1, "right": -1}
SEGMENT = 0.03
PUSH_TILT = math.radians(40.0)  # the pushing hand leans this far, fingertips forward, so only the fingertips meet the tumbler
PUSH_Z = 0.022  # height of the fingertips' ends above the table while pushing: low, so the tumbler slides and does not tip
PUSH_W = 0.036  # the pushing fingers stand this far apart: a fork the tumbler's wall sits in
PUSH_OFF = 0.026  # the tumbler's axis runs this far ahead of the fingertips' ends when they push it (measured in a logged run)
STOP_GAP = 0.004  # the push stops with the two rims this far apart
INSERT = 0.028  # the fingertips go this far down inside the rims
PINCH_OPEN = 0.044  # finger opening while going down into the two tumblers
PINCH_W = 0.004  # opening commanded for the pinch (the two walls and the pads' curvature hold the fingers about 10 mm apart)
PINCH_YAW = -0.5 * math.pi  # the pinching hand's fingers close along x, the line the pair stands on
CARRY_TILT = math.radians(25.0)  # the hand leans this far forward for the carry: straight down it cannot rise above the tub's wall
CARRY_Z = 0.31  # fingertip midpoint above the table for the carry: the lower tumbler's base clears the 12.5 cm rim by about 5 cm
PLACE = (0.300, 0.095)  # where each pair's pinch point comes down in the tub (y mirrored for the right hand)
PLACE_UP = 0.004  # the pair is let go with its lowest edges this far above the tub's floor
PARK = (0.200, 0.330, 0.200)
VARIANT = os.environ.get("BUS_DEMO", "")
CAL = dict(push=0.0, place_z=0.0)  # fixed offsets against the arms' give under load, from logged runs (dev/sim.py)


def set_variant(name):
    global VARIANT
    VARIANT = name


def _fingers(width):
    return -width / 2


def _tip_to_tcp(tip, tilt_a, heading=0.0):
    """The fingertip midpoint (TCP) for the fingertips' ends at ``tip`` with the hand leaned ``tilt_a`` toward ``heading``."""
    a = np.array((math.sin(tilt_a) * math.cos(heading), math.sin(tilt_a) * math.sin(heading), -math.cos(tilt_a)))  # approach direction
    return np.asarray(tip, float) - 0.013 * a


class _Plan:
    def __init__(self):
        self.wps = {a: [] for a in ARMS}
        self.t = {a: 0.0 for a in ARMS}
        self.last = {}
        for a in ARMS:
            pos, _ = ik.tcp_pose(np.array(READY, float), np.array(BASES[a]))
            self.last[a] = (tuple(float(v) for v in pos), FINGER_OPEN, 0.0, 0.0, 0.0)

    def go(self, arm, dur, pos=None, finger=None, label="", yaw=None, tilt_a=None, heading=None, whole=False):
        """Move ``arm`` (y, yaw mirrored for the right arm by the caller) in ``dur`` s; long moves are cut into pieces so the
        path stays straight."""
        p0, f0, y0, t0, h0 = self.last[arm]
        p = p0 if pos is None else tuple(float(v) for v in pos)
        f = f0 if finger is None else finger
        y = y0 if yaw is None else yaw
        ta = t0 if tilt_a is None else tilt_a
        h = h0 if heading is None else heading
        n = max(1, math.ceil(np.linalg.norm(np.subtract(p, p0)) / SEGMENT), math.ceil(abs(y - y0) / math.radians(15)), math.ceil(abs(ta - t0) / math.radians(10)))
        if whole or not self.wps[arm]:  # the first move starts from the ready pose, which is not a straight-down grip: one piece
            n = 1
        for k in range(1, n + 1):
            q = tuple(float(v) for v in np.asarray(p0) + (np.asarray(p) - np.asarray(p0)) * k / n)
            self.wps[arm].append(Waypoint(dur / n, q, f0 + (f - f0) * k / n, label, lean.leaned(y0 + (y - y0) * k / n, t0 + (ta - t0) * k / n, h0 + (h - h0) * k / n)))
        self.last[arm] = (p, f, y, ta, h)
        self.t[arm] += dur

    def until(self, arm, t):
        if t > self.t[arm] + 1e-9:
            self.go(arm, t - self.t[arm], label="wait")


def _herd(plan, arm, push_z=PUSH_Z):
    """Push the nearer tumbler along the row until its rim is STOP_GAP from the farther one's."""
    s = ARMS[arm]
    y = s * ROW_Y
    x0 = NEAR_X - PUSH_OFF - 0.015  # fingertips' ends 15 mm short of touching
    x1 = FAR_X - M.RIM_D - STOP_GAP - PUSH_OFF + CAL["push"]
    plan.go(arm, 3.0, _tip_to_tcp((x0, y, TOP + 0.24), PUSH_TILT), _fingers(PUSH_W), "herd: back over the near glass, above its straw", yaw=0.0, tilt_a=PUSH_TILT, heading=0.0)
    plan.go(arm, 2.5, _tip_to_tcp((x0, y, TOP + push_z), PUSH_TILT), label="herd: down behind it")
    plan.go(arm, 4.0, _tip_to_tcp((x1, y, TOP + push_z), PUSH_TILT), label="herd: push")
    plan.go(arm, 0.8, _tip_to_tcp((x1 - 0.025, y, TOP + push_z), PUSH_TILT), label="herd: back off")
    plan.go(arm, 1.5, _tip_to_tcp((x1 - 0.06, y, TOP + 0.20), PUSH_TILT), label="herd: up")


def _pinch_z(base_z):
    """Fingertip midpoint height for the fingertips INSERT below the rims of tumblers standing on a surface at ``base_z``."""
    return base_z + M.GLASS_H - INSERT + 0.013


def _carry(plan, arm, pinch_x, place, label, tilt_a=CARRY_TILT):
    """With the fingers down inside the rim(s) at ``pinch_x`` on the row: pinch, lift, lean, carry over the rim, stand
    the tumbler(s) on the tub's floor at ``place`` and let go."""
    s = ARMS[arm]
    y = s * ROW_Y
    yaw = s * PINCH_YAW
    plan.go(arm, 2.5, (pinch_x, y, TOP + 0.20), _fingers(PINCH_OPEN), f"{label}: over the rims", yaw=yaw, tilt_a=0.0, heading=0.0)
    plan.go(arm, 1.5, (pinch_x, y, _pinch_z(TOP)), label=f"{label}: fingers in")
    plan.go(arm, 1.2, finger=_fingers(PINCH_W), label=f"{label}: pinch")
    plan.go(arm, 0.5, label=f"{label}: settle")
    plan.go(arm, 1.2, (pinch_x, y, _pinch_z(TOP) + 0.03), label=f"{label}: lift")
    plan.go(arm, 3.0, (pinch_x, y, TOP + CARRY_Z), label=f"{label}: up and lean", tilt_a=tilt_a)
    px, py = place[0], s * place[1]
    turn = math.atan2(py - BASES[arm][1], px - BASES[arm][0])  # the hand turns with the arm, so the lean stays in line with the pair
    plan.go(arm, 3.5, (px, py, TOP + CARRY_Z), label=f"{label}: over the tub", yaw=yaw + turn, heading=turn)
    set_z = _pinch_z(TOP + M.TUB_FLOOR) + PLACE_UP + CAL["place_z"]
    plan.go(arm, 4.0, (px, py, set_z + 0.025), label=f"{label}: down and straighten", tilt_a=0.0)  # upright only once low: the wrist cannot point straight down higher up
    plan.go(arm, 1.2, (px, py, set_z), label=f"{label}: down")
    plan.go(arm, 0.5, label=f"{label}: settle")
    if VARIANT == "held":
        return
    plan.go(arm, 1.2, finger=_fingers(PINCH_OPEN), label=f"{label}: let go")
    plan.go(arm, 0.5, label=f"{label}: settle")
    plan.go(arm, 1.2, (px, py, set_z + 0.04), label=f"{label}: fingers out")
    plan.go(arm, 2.0, (px, py, TOP + 0.27), label=f"{label}: up", tilt_a=math.radians(20.0))  # above the straws before moving sideways


def _park(plan, arm):
    plan.go(arm, 2.5, (PARK[0], ARMS[arm] * PARK[1], TOP + PARK[2]), FINGER_OPEN, "away", yaw=0.0, tilt_a=0.0, heading=0.0)


def demo():
    plan = _Plan()
    if VARIANT == "none":
        for arm in ARMS:
            p0 = plan.last[arm][0]
            plan.go(arm, 1.0, (p0[0], p0[1], TOP + 0.17), label="down", whole=True)
        for arm in ARMS:
            plan.go(arm, 3.0, label="wait")
        return plan.wps
    arms = ("left",) if VARIANT == "half" else tuple(ARMS)
    for arm in ARMS:
        if arm not in arms:  # the idle hand comes down from the ready pose and waits
            p0 = plan.last[arm][0]
            plan.go(arm, 1.0, (p0[0], p0[1], TOP + 0.17), label="down")
    pair_x = FAR_X - M.RIM_D / 2 - STOP_GAP / 2  # where the two rims meet after the push
    for arm in arms:
        if VARIANT == "single":  # shortcut: one tumbler at a time, each pinched by one wall
            _carry(plan, arm, FAR_X - M.RIM_D / 2 + M.WALL / 2, (PLACE[0] + 0.045, PLACE[1]), "far glass")
            _carry(plan, arm, NEAR_X + M.RIM_D / 2 - M.WALL / 2, (PLACE[0] - 0.060, PLACE[1]), "near glass")
        else:
            _herd(plan, arm, push_z=M.GLASS_H - 0.012 if VARIANT == "high" else PUSH_Z)
            _carry(plan, arm, pair_x, PLACE, "pair")
        if VARIANT != "held":
            _park(plan, arm)
    end = max(plan.t.values()) + 2.0
    for a in ARMS:
        plan.until(a, end)
    return plan.wps


def ik_errors(wps):
    worst_p = worst_r = jump = 0.0
    bad = []
    for arm, ws in wps.items():
        q = np.array(READY, float)
        for w in ws:
            q0 = q
            q, pe, re = ik.solve(np.asarray(w.pos, float), ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)), q, base_pos=BASES[arm])
            worst_p, worst_r, jump = max(worst_p, pe), max(worst_r, re), max(jump, float(np.abs(q - q0).max()))
            if pe > 1e-3 or re > math.radians(1.0):
                bad.append((arm, w.label[:28], tuple(round(v, 3) for v in w.pos), round(pe * 1000, 1), round(math.degrees(re), 2)))
    return worst_p, worst_r, bad, jump


# -- self-test (no simulator): the demo fits the run limit and IK reaches every waypoint ---------------------------------
if __name__ == "__main__":
    wps = demo()
    t = max(sum(w.duration for w in ws) for ws in wps.values())
    pe, re, bad, jump = ik_errors(wps)
    print(f"demo {t:.1f} s ({t * 20:.0f} steps), IK {pe * 1000:.2f} mm / {math.degrees(re):.2f} deg, {len(bad)} bad, largest joint step {math.degrees(jump):.0f} deg")
    for b in bad[:12]:
        print("  ", b)
    print("tumbler kg empty / with ice and straw:", M.GLASS_MASS, round(M.FULL_MASS, 4), " objects:", len(OBJECTS))
    assert t * 20 < 1800 and not bad
