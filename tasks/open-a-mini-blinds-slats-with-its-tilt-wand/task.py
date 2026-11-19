"""Open a mini blind's slats with its tilt wand. Created 11_17_26 (America/New_York).

On the station table: a made-to-measure 1 in aluminium mini blind in the smallest size its retailer sells, 18 in wide
by 12 in drop (steel headrail 38 x 25 mm, twelve 25 mm slats at 21.5 mm spacing and a bottom rail; cordless lift, so no
lift cord), hung from the head piece of a table-top stand (plywood base 520 x 180 mm, top of the stand 350 mm above the
table), facing the robot, the middle of the slats above (0.35, 0.00). Its clear hexagonal tilt wand (6.35 mm across
flats, sawn to 250 mm, 10 g) hangs on a hook from the tilter stem under the headrail's front edge, 52 mm from the
headrail's left end (177 mm left of centre) and 27 mm in front of the slats, and swings freely when touched. The slats start
closed, room-side edge down, 70 degrees from level. Twisting the wand about its own axis turns the tilter's worm:
ten wand turns per tilt-rod turn, so about 1.9 turns clockwise (seen from above) bring the slats level; turning on
closes them the other way, and at either closed stop the tilter's clutch slips. The worm does not back-drive: pulling
the wand or pushing a slat turns nothing (mini_blind/mech.py).

Run it:  inspect-robots run --embodiment yam_mini_blind -E task=task.py -E object_jitter=0 --policy yam_scripted \\
           --no-prompt --instruction "Open the blind: turn the wand until the slats are level." --scorer task_score

Everything is judged on the final state, as in the real trial (see score()).
"""

import math
import os

import numpy as np

from mini_blind import grip as G
from mini_blind import mech as M
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import Waypoint
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BLIND_XY = np.array((0.35, 0.00))

OBJECTS = [Box("blind", pos=(float(BLIND_XY[0]), float(BLIND_XY[1]), TOP), size=(0.01, 0.01, 0.01))]  # built by mini_blind.mech

START_DEG = -M.CLOSED_DEG  # closed, room-side edge down

# -- criteria (all judged on the final state) ----------------------------------------------------------------------
LEVEL_DEG = 10.0  # mean slat angle within this of level: 1.0
PART_DEG = 25.0  # within this: 0.5
MOVED_DEG = 20.0  # at least this far from the closed start toward level (within 50 degrees of level): 0.25
PLUMB_DEG = 10.0  # the wand hanging within this of plumb, or the top band drops to 0.5
SHIFT_OK = 0.020  # the stand within this (m) of its start, or the top band drops to 0.5
SHIFT_MAX = 0.100  # the stand moved more than this: 0
TIPPED_DEG = 10.0  # the stand tilted more than this: 0
CLEAR = 0.040  # a hand is clear with its fingertip midpoint this far from the wand's axis and from the slats
CLEAR_S = 1.0  # both hands clear for at least the last second of the trial


def _fresh():
    return {"clear": 0.0}


_trial = _fresh()


def reset_latch():
    """Forget the trial: the embodiment calls this at every reset."""
    _trial.clear()
    _trial.update(_fresh())


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slat_angle_deg(objects):
    """Mean measured angle of the twelve slats from level (deg; negative = room-side edge down)."""
    return math.degrees(float(objects["slats"].low[0]))


def wand_plumb_deg(objects):
    """Angle (deg) between the wand and the plumb line."""
    w = objects["wand"]
    d = np.asarray(w.high, float) - np.asarray(w.pos, float)
    return math.degrees(math.acos(float(np.clip(-d[2] / max(float(np.linalg.norm(d)), 1e-9), -1.0, 1.0))))


def stand_shift(objects):
    return float(np.hypot(*(np.asarray(objects["blind"].pos, float)[:2] - BLIND_XY)))


def stand_tilt_deg(objects):
    return math.degrees(math.acos(float(np.clip(_rot(objects["blind"].quat_xyzw)[2, 2], -1.0, 1.0))))


def _seg_dist(p, a, b):
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _slats_dist(p, objects):
    """Distance from world point p to the box the slats sweep (in the stand's frame)."""
    b = objects["blind"]
    local = _rot(b.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(b.pos, float))
    z0, z1 = M.RAIL_Z - M.RAIL["w"] / 2, M.HEADRAIL_TOP
    half = np.array((M.HEADRAIL["d"] / 2, M.WIDTH / 2, (z1 - z0) / 2))
    d = np.abs(local - np.array((0.0, 0.0, (z0 + z1) / 2))) - half
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def hand_clear(objects, hand):
    p = np.asarray(objects[hand].pos, float)
    w = objects["wand"]
    return _slats_dist(p, objects) >= CLEAR and _seg_dist(p, np.asarray(w.pos, float), np.asarray(w.high, float)) >= CLEAR


def note_step(objects, dt):
    """After every control step: how long both hands have been clear without a break (the only thing tracked over time;
    every other criterion is read from the final state)."""
    if all(hand_clear(objects, h) for h in ("left_tcp", "right_tcp")):
        _trial["clear"] += dt
    else:
        _trial["clear"] = 0.0


def band(angle_deg):
    """Credit for the slats' angle alone: the same bands either side of level."""
    a = abs(angle_deg)
    if a <= LEVEL_DEG + 1e-9:
        return 1.0
    if a <= PART_DEG + 1e-9:
        return 0.5
    if a <= M.CLOSED_DEG - MOVED_DEG + 1e-9:
        return 0.25
    return 0.0


def score(objects):
    """From the final state. 0 unless both hands have been clear of the wand and the slats for the last 1 s, the stand
    is within 10 degrees of upright and within 100 mm of its start. Then by the mean slat angle from level: within
    10 degrees 1.0, within 25 degrees 0.5, within 50 degrees (20 degrees or more from the closed start) 0.25, else 0,
    the same either side of level. The 1.0 band also needs the wand hanging within 10 degrees of plumb and the stand
    within 20 mm of its start; otherwise it is 0.5."""
    if _trial["clear"] < CLEAR_S - 1e-9 or stand_tilt_deg(objects) > TIPPED_DEG:
        return 0.0
    shift = stand_shift(objects)
    if shift > SHIFT_MAX:
        return 0.0
    c = band(slat_angle_deg(objects))
    if c >= 1.0 and (wand_plumb_deg(objects) > PLUMB_DEG or shift > SHIFT_OK):
        return 0.5
    return c


def success(objects):
    return False  # judged at the end of the run only: a trial does not stop early


def debug_line(objects, t):
    s = objects["slats"]
    r = lambda v: np.round(1000 * np.asarray(v), 0).tolist()  # noqa: E731
    return (f"BLIND t={t:6.2f} slats {slat_angle_deg(objects):6.1f} ({math.degrees(s.low[1]):.1f}..{math.degrees(s.low[2]):.1f}) "
            f"rod {math.degrees(s.high[0]):6.1f} turns {s.high[1]:6.3f} plumb {wand_plumb_deg(objects):5.1f} "
            f"shift {1000 * stand_shift(objects):5.1f} tilt {stand_tilt_deg(objects):4.1f} clear {_trial['clear']:.2f} "
            f"score {score(objects):.3f} L {r(objects['left_tcp'].pos)} wand end {r(objects['wand'].high)}")


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop from the fixed start; it reads no simulator state. One hand (the left) does it all, as a person does with
# the fingers of one hand: it comes in level, pinches the hanging wand 138 mm below its hook, draws it a little toward
# the robot (clear of the slats), swings round the wand's own axis 60 degrees (one flat of the hexagon) with the wand pinched,
# lets go, backs off while the freed wand swings back and hangs still, comes round to where it began and pinches again;
# twelve such strokes. BLIND_DEMO switches the variants: "nothing": no motion. "one": a single stroke. "half": six strokes.
# "over": sixteen strokes (past level). "far_over": twenty-six strokes (to the other closed stop, clutch slipping). "yank":
# pinch the wand and pull it straight down, twice. "push": the closed gripper pushes the slats' room-side edges up.
# "hold": the full demo, but the hand never lets go.
VARIANT = os.environ.get("BLIND_DEMO", "")


def _env(name, default):
    return float(os.environ.get(name, default))


PINCH_Z = _env("BLIND_PINCH_Z", 0.160)  # pinch height above the table
PSI_A = math.radians(_env("BLIND_PSI_A", 0.0))  # the hand's bearing round the wand at the start of a stroke (0 = fingertips toward the blind)
STROKE_DEG = _env("BLIND_STROKE_DEG", 60.0)  # the hand swings this far round the wand in one stroke: one flat of the hexagon
# (the wrist allows about 100 degrees, but the pads close flat on two of the wand's six flats, so a pinch that starts
# off the flats turns the wand to the nearest pair as it closes; whole flats keep every pinch square)
PULL = _env("BLIND_PULL", 0.035)  # the pinched point is drawn this far toward the robot before turning
WAND_PAST = _env("BLIND_WAND_PAST", 0.005)  # the wand sits this far past the TCP toward the pad tips
GAP_OPEN = _env("BLIND_GAP_OPEN", 0.034)  # pad gap when open round the wand
GAP_PINCH = _env("BLIND_GAP_PINCH", 0.0035)  # commanded pad gap on the 6.35 mm wand: a firm pinch through the finger drive's spring
BACK = _env("BLIND_BACK", 0.080)
BACK_B = _env("BLIND_BACK_B", 0.040)  # the shorter back-off at the end of a stroke, where the wrist is near its limit
STROKES = int(_env("BLIND_STROKES", 12))
LAST_DEG = _env("BLIND_LAST_DEG", STROKE_DEG)  # the last stroke's size
TURN_S = _env("BLIND_TURN_S", 1.5)
SETTLE_S = _env("BLIND_SETTLE_S", 2.0)  # the freed wand swings back and hangs still
HOOK_W = np.array((BLIND_XY[0], BLIND_XY[1], TOP)) + M.HOOK
REACH = HOOK_W[2] - (TOP + PINCH_Z)  # hook to pinched point along the wand


def _axis_angle(u, a):
    u = np.asarray(u, float)
    k = np.array(((0, -u[2], u[1]), (u[2], 0, -u[0]), (-u[1], u[0], 0)))
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def pinch(psi, pull=0.0, back=0.0, drop=0.0):
    """(TCP, rotation) of the left hand pinching the wand `REACH + drop` below the hook, the pinched point drawn `pull`
    toward the robot, the hand at bearing psi round the wand's axis, backed off `back` along its fingertips."""
    u = np.array((-pull, 0.0, -math.sqrt(REACH * REACH - pull * pull))) / REACH  # down the wand
    p = HOOK_W + (REACH + drop) * u
    t_ref = np.array((1.0, 0.0, 0.0)) - u * u[0]
    t_ref /= np.linalg.norm(t_ref)
    tips = _axis_angle(u, -psi) @ t_ref  # u points down, so a turn of -psi about it is +psi seen from above
    rot = G.frame(tips, np.cross(u, tips))
    return p - (WAND_PAST + back) * tips, rot


class _Plan:
    def __init__(self):
        self.wps = {"left": [], "right": []}
        self.t = {"left": 0.0, "right": 0.0}
        self.last = {}
        for a in self.wps:
            pos, rot = ik.tcp_pose(np.array(READY, float), np.array(BASES[a]))
            self.last[a] = (np.asarray(pos, float), FINGER_OPEN, np.asarray(rot, float))
        self.home = self.last["left"]

    def go(self, arm, dur, pos=None, finger=None, rot=None, label=""):
        p0, f0, r0 = self.last[arm]
        p = p0 if pos is None else np.asarray(pos, float)
        f = f0 if finger is None else finger
        r = r0 if rot is None else np.asarray(rot, float)
        self.wps[arm].append(Waypoint(dur, tuple(float(v) for v in p), f, label, G.turned(r)))
        self.last[arm] = (p, f, r)
        self.t[arm] += dur

    def until(self, arm, t):
        if t > self.t[arm] + 1e-9:
            self.go(arm, t - self.t[arm], label="wait")


def _stroke(plan, deg, first, let_go=True):
    opened, pinched = -GAP_OPEN / 2, -GAP_PINCH / 2
    psi_b = PSI_A - math.radians(deg)
    p, r = pinch(PSI_A, back=BACK)
    plan.go("left", 2.5 if first else 1.2, p, opened, r, "round to the wand's front")
    if not first:
        plan.go("left", SETTLE_S, label="wait for the wand to hang still")
    plan.go("left", 0.9, pinch(PSI_A)[0], label="fingers either side of the wand")
    plan.go("left", 0.5, finger=pinched, label="pinch the wand")
    plan.go("left", 0.6, pinch(PSI_A, PULL)[0], rot=pinch(PSI_A, PULL)[1], label="draw it clear of the slats")
    n = max(2, int(round(deg / 10.0)))
    for i in range(1, n + 1):
        p, r = pinch(PSI_A + (psi_b - PSI_A) * i / n, PULL)
        plan.go("left", TURN_S * (deg / STROKE_DEG) / n, p, rot=r, label="turn the wand")
    plan.go("left", 0.2, label="steady")
    if let_go:
        plan.go("left", 0.5, finger=opened, label="let go")
        plan.go("left", 0.6, pinch(psi_b, PULL, back=BACK_B)[0], label="back off")
        p, r = pinch((PSI_A + psi_b) / 2, PULL, back=(BACK + BACK_B) / 2)
        plan.go("left", 0.6, p, rot=r, label="come round")


def demo():
    global LAST_PLAN
    plan = _Plan()
    opened, pinched = -GAP_OPEN / 2, -GAP_PINCH / 2
    if VARIANT == "nothing":  # both hands stay at the ready pose for a full minute
        for a in plan.wps:
            plan.go(a, 60.0, label="wait")
    elif VARIANT == "yank":
        for k in range(2):
            p, r = pinch(PSI_A, back=BACK)
            plan.go("left", 2.5, p, opened, r, "round to the wand's front")
            plan.go("left", 1.2, pinch(PSI_A)[0], label="fingers either side of the wand")
            plan.go("left", 0.6, finger=pinched, label="pinch the wand")
            plan.go("left", 0.8, pinch(PSI_A, drop=0.05)[0], label="yank it down")
            plan.go("left", 0.5, label="hold")
            plan.go("left", 0.6, finger=opened, label="let go")
            plan.go("left", 1.0, pinch(PSI_A, back=BACK)[0], label="back off")
    elif VARIANT == "push":  # closed fingertips come under the slats' room-side edges and sweep up
        x = BLIND_XY[0] + 0.002  # pad tips 2 mm past the slats' centre line, under their lowered room-side edges
        lo, hi = TOP + M.SLAT_Z[9] - 0.020, TOP + M.SLAT_Z[3]
        flat = G.frame((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
        plan.go("left", 2.5, (x - 0.08, 0.02, lo), -0.004, flat, "to the slats")
        plan.go("left", 1.2, (x - 0.013, 0.02, lo), label="fingertips at the slats' edges")
        plan.go("left", 3.0, (x - 0.013, 0.02, hi), label="sweep up the edges")
        plan.go("left", 1.0, label="hold")
        plan.go("left", 1.5, (x - 0.08, 0.02, hi), label="back off")
    else:
        n = {"one": 1, "half": 6, "over": 16, "far_over": 26}.get(VARIANT, STROKES)
        for k in range(n):
            last = k == n - 1
            _stroke(plan, LAST_DEG if last and VARIANT in ("", "hold") else STROKE_DEG, k == 0, let_go=not (last and VARIANT == "hold"))
    if VARIANT not in ("nothing", "hold"):
        p, r = pinch(PSI_A, back=BACK)
        plan.go("left", 1.5, p, rot=r, label="round to the wand's front")
        plan.go("left", 2.5, plan.home[0], FINGER_OPEN, plan.home[2], "away, back to the ready pose")
    end = max(plan.t.values()) + 5.0
    for a in plan.wps:
        plan.until(a, end)
    LAST_PLAN = plan
    return plan.wps


LAST_PLAN = None


def ik_errors(wps):
    from yam_arena.demo import ArmPlan

    bad = []
    worst = (0.0, 0.0)
    for arm, ws in wps.items():
        ap = ArmPlan(list(ws), BASES[arm])
        t = 0.0
        labels = {}
        while t <= ap.duration:
            before = (ap.worst_ik_mm, ap.worst_ik_deg)
            _, label = ap.action7(t)
            if (ap.worst_ik_mm, ap.worst_ik_deg) != before and (ap.worst_ik_mm > 1.0 or ap.worst_ik_deg > 1.0):
                labels[label] = (round(ap.worst_ik_mm, 1), round(ap.worst_ik_deg, 2))
            t += 0.05
        worst = (max(worst[0], ap.worst_ik_mm), max(worst[1], ap.worst_ik_deg))
        bad += [(arm, k, v) for k, v in labels.items()]
    return worst, bad


if __name__ == "__main__":
    print("slat z mm", [round(1000 * z) for z in M.SLAT_Z], "rail", round(1000 * M.RAIL_Z), "drop mm", round(1000 * M.DROP),
          "wand g", round(1000 * M.WAND_MASS, 1), "stand kg", round(M.ROOT_MASS, 2), "turns to level", round(M.turns_to_level(), 2))
    print("hook", np.round(HOOK_W, 3), "pinch", np.round(pinch(0.0)[0], 3), "reach", round(REACH, 4))
    wps = demo()
    t = max((sum(w.duration for w in ws) for ws in wps.values()), default=0.0)
    print(f"demo {t:.1f} s ({t * 20:.0f} steps), IK worst mm/deg, bad:", ik_errors(wps))
