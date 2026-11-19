"""Aim an Anglepoise Type 75 desk lamp onto an open notebook. Created 11_16_26 (America/New_York).

On the station table: an open Moleskine Large hard-cover notebook (26 x 21 cm open, 0.33 kg), loose, its centre at
(0.30, 0.02), with a 19 mm red sticker dot in the middle of its right-hand page at (0.30, -0.045); and, behind it and to
the right, an Anglepoise Type 75 desk lamp standing free on its 19.5 cm cast-iron base (3.1 kg), its base's centre at
(0.58, -0.25). The lamp is built true size from the maker's drawing (desk_lamp/mech.py): a swivel on the base, a lower and
an upper arm of about 30 cm each, and a head knuckle carrying the 14.5 cm bell shade. Like the real lamp it holds any
pose by itself (its springs are an ideal balance here), with friction in every joint. At the start it is parked low and
swung to the right, its shade's mouth 16 cm above the desk, its axis meeting the desk 13 cm from the dot.

Run it:  inspect-robots run --embodiment yam_desk_lamp -E task=task.py -E object_jitter=0 --policy yam_scripted --no-prompt \\
           --instruction "Light the notebook for reading: aim the lamp at the red dot, with the shade at least 30 cm above the desk." \\
           --scorer task_score

The beam's centre is where the shade's axis meets the page, taken from the shade's simulated pose at every step. The score
counts once both hands have been clear of the lamp, and the lamp still, for 3 s (see score()).
"""

import math
import os

import numpy as np

from desk_lamp import grip as G
from desk_lamp import mech as M
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
LAMP_XY = np.array((0.58, -0.25))
BOOK_XY = np.array((0.30, 0.02))
DOT_XY = BOOK_XY + M.DOT_IN_BOOK
PAGE_H = M.BOOK["size"][2]  # the page's height above the desk

OBJECTS = [
    Box("anglepoise", pos=(float(LAMP_XY[0]), float(LAMP_XY[1]), TOP), size=(0.01, 0.01, 0.01)),  # built by desk_lamp.mech
    Box("book", pos=(float(BOOK_XY[0]), float(BOOK_XY[1]), TOP), size=(0.01, 0.01, 0.01)),
]

# -- start and goal poses -------------------------------------------------------------------------------------------
GOAL_TILT = math.radians(15.0)
GOAL_MOUTH_H = 0.345
GOAL_POSE = M.aim_pose(LAMP_XY, DOT_XY, PAGE_H, GOAL_MOUTH_H, GOAL_TILT)
GOAL_Q = M.joints(*GOAL_POSE)
START_SWING = math.radians(15.0)  # parked swung this far clockwise of the goal, seen from above
START_RETILT = math.radians(25.0)  # with the arm where the goal needs it, the shade would still sit this much too far tilted
_low = M.aim_pose(LAMP_XY, DOT_XY, PAGE_H, 0.18, math.radians(20.0))
START_POSE = (GOAL_POSE[0] + START_SWING, _low[1], _low[2], GOAL_TILT + START_RETILT + _low[2] - GOAL_POSE[2])
START_Q = M.joints(*START_POSE)

# -- criteria -------------------------------------------------------------------------------------------------------
MIN_MOUTH_H = 0.30  # the shade's mouth centre at least 30 cm above the desk
BANDS = ((0.030, 1.0), (0.060, 0.5), (0.100, 0.25))  # beam centre within this far of the dot earns this much
LOW_CAP = 0.25  # a shade lower than MIN_MOUTH_H earns at most this
CLEAR_ARM = 0.05  # a hand is clear of the lamp with its fingertip midpoint this far from both arms' centre lines ...
CLEAR_SHADE = 0.11  # ... and this far from the shade's axis (the mouth is 7.2 cm in radius)
SETTLE_S = 3.0  # both hands clear and the lamp still for this long before the reading counts
STILL_DEG = 0.3  # the lamp is still while no joint has moved more than this since the hands came clear
MOVED = 0.020  # at the end, the notebook or the lamp's base more than this (m) from its start: the trial scores 0
TIPPED_DEG = 3.0  # ... or, at the end, the lamp's base or the notebook tilted more than this


def _fresh():
    return {"t": 0.0, "clear": 0.0, "ref": None, "start": None, "worst_move": 0.0}


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


def shade(objects):
    """(mouth centre, unit axis toward the page) in the world, from the head link's simulated pose."""
    h = objects["lamp_head"]
    r = _rot(h.quat_xyzw)
    mouth = np.asarray(h.pos, float) + r @ np.array((M.MOUTH0[0], 0.0, M.MOUTH0[1]))
    return mouth, r @ np.array((0.0, 0.0, -1.0))


def dot_world(objects):
    b = objects["book"]
    return np.asarray(b.pos, float) + _rot(b.quat_xyzw) @ np.array((M.DOT_IN_BOOK[0], M.DOT_IN_BOOK[1], PAGE_H))


def reading(objects):
    """(beam centre's distance from the dot in m, or inf with no spot on the page; mouth centre's height above the desk)."""
    mouth, axis = shade(objects)
    d = dot_world(objects)
    spot = M.beam_point(mouth, axis, d[2])
    err = math.inf if spot is None else float(np.hypot(*(spot[:2] - d[:2])))
    return err, float(mouth[2] - TOP)


def _seg_dist(p, a, b):
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def hand_clear(objects, hand):
    p = np.asarray(objects[hand].pos, float)
    s, e, k = (np.asarray(objects[n].pos, float) for n in ("lamp_lower", "lamp_upper", "lamp_head"))
    mouth, axis = shade(objects)
    top = mouth - M.SHADE_LEN * axis
    return (_seg_dist(p, s, e) >= CLEAR_ARM and _seg_dist(p, e, k) >= CLEAR_ARM and _seg_dist(p, top, mouth) >= CLEAR_SHADE)


def _tilt_deg(q):
    return math.degrees(math.acos(min(1.0, abs(float(_rot(q)[2, 2])))))


def note_step(objects, dt):
    """After every control step: record the largest displacement so far (debug only); are both hands clear; has the lamp
    stayed still. Whether the notebook or the base has moved is judged in score() from their poses at the end."""
    tr = _trial
    tr["t"] += dt
    b, bk = np.asarray(objects["lamp_base"].pos, float), np.asarray(objects["book"].pos, float)
    if tr["start"] is None:
        tr["start"] = (b.copy(), bk.copy())
    mv = max(float(np.linalg.norm(b[:2] - tr["start"][0][:2])), float(np.linalg.norm(bk[:2] - tr["start"][1][:2])))
    tr["worst_move"] = max(tr["worst_move"], mv)
    q = np.degrees(np.r_[objects["lamp_q"].pos, objects["lamp_q"].low[0]])
    if all(hand_clear(objects, h) for h in ("left_tcp", "right_tcp")):
        if tr["ref"] is None or np.max(np.abs(q - tr["ref"])) > STILL_DEG:
            tr["ref"], tr["clear"] = q, 0.0
        else:
            tr["clear"] += dt
    else:
        tr["ref"], tr["clear"] = None, 0.0


def credit(err, mouth_h):
    c = 0.0
    for lim, v in BANDS:
        if err <= lim + 1e-9:
            c = v
            break
    return min(c, LOW_CAP) if mouth_h < MIN_MOUTH_H else c


def displaced(objects):
    """True if, now, the notebook or the lamp's base is more than 20 mm (in plan) from where it started, or either is
    tilted more than 3 degrees. Judged from the current poses only, as in the real trial (tape outlines at the end);
    a bump that leaves both where they were does not count."""
    st = _trial["start"]
    if st is None:
        return False
    b, bk = np.asarray(objects["lamp_base"].pos, float), np.asarray(objects["book"].pos, float)
    mv = max(float(np.linalg.norm(b[:2] - st[0][:2])), float(np.linalg.norm(bk[:2] - st[1][:2])))
    return (mv > MOVED or _tilt_deg(objects["lamp_base"].quat_xyzw) > TIPPED_DEG
            or _tilt_deg(objects["book"].quat_xyzw) > TIPPED_DEG)


def score(objects):
    """0 if, at the time of the reading, the notebook or the lamp's base is more than 20 mm from its start or tilted more
    than 3 degrees, or until both hands have been clear of the lamp and the lamp still for 3 s. Then, by the beam centre's
    distance from the dot: 1.0 within 30 mm, 0.5 within 60 mm, 0.25 within 100 mm, else 0; a shade mouth under 30 cm
    above the desk caps it at 0.25."""
    if displaced(objects) or _trial["clear"] < SETTLE_S - 1e-9:
        return 0.0
    return credit(*reading(objects))


def success(objects):
    return score(objects) >= 1.0


def debug_line(objects, t):
    err, h = reading(objects)
    q = np.degrees(np.r_[objects["lamp_q"].pos, objects["lamp_q"].low[0]])
    r = lambda v: np.round(1000 * np.asarray(v), 0).tolist()  # noqa: E731
    return (f"LAMP t={t:6.2f} err {1000 * err:7.1f} mm h {1000 * h:6.1f} q {np.round(q, 1).tolist()} clear {_trial['clear']:.2f} "
            f"displaced {int(displaced(objects))} (worst {1000 * _trial['worst_move']:.1f}) L {r(objects['left_tcp'].pos)} R {r(objects['right_tcp'].pos)} "
            f"score {score(objects):.3f} S {r(objects['lamp_lower'].pos)} E {r(objects['lamp_upper'].pos)} K {r(objects['lamp_head'].pos)} "
            f"base {r(objects['lamp_base'].pos)} book {r(objects['book'].pos)}")




# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop from the known start: the plan reads no simulator state; it works the lamp's path out offline from the lamp's
# own geometry (mech.fk). The right hand comes in from the robot's side and pinches the upper arm's two rods from above
# and below, 9 cm from the head knuckle, then carries the arm along the lamp's own joint-space path to where the goal
# needs it: the shade swings, rises and comes over the notebook, but rides along on its stiff knuckle, still tilted 25
# degrees too far. Holding the arm there (a push on the shade alone would swing the arm about the elbow), the left hand
# pushes the shade's near side with its closed fingertips, turning the shade on its knuckle until its axis points at the
# dot, and backs away; then the right hand opens and backs off the rods, and both hands park. LAMP_DEMO switches the
# shortcut variants: "nothing": no motion. "one_hand": the left hand tilts the shade without the right hand holding the
# arm. "no_tilt": the arm is carried but the shade is never re-aimed. "low": the same aim with the shade's mouth 22 cm
# above the desk. "hold": the right hand never lets go of the arm.
VARIANT = os.environ.get("LAMP_DEMO", "")


def _env(name, default):
    return float(os.environ.get(name, default))


GRASP_FROM_K = _env("LAMP_GRASP_FROM_K", 0.09)  # the pinch's centre on the upper arm, from the knuckle toward the elbow
TIP_PAST = 0.020  # the fingertips stop this far past the arm's centre plane (the far rod is 8 mm past it)
BACK = 0.09  # a hand comes in and leaves along its fingertip direction from this far out
CARRY_S = _env("LAMP_CARRY_S", 9.0)
TILT_S = _env("LAMP_TILT_S", 5.0)
TILT_OVER = math.radians(_env("LAMP_TILT_OVER", 4.2))  # the push goes this much past the goal tilt, for the arm's give
# The hand carries the arm this much past the goal (deg, swivel, shoulder, elbow): the rods give a little in the pinch.
CARRY_OVER = {"swivel": _env("LAMP_OVER_SWIVEL", -2.5), "shoulder": _env("LAMP_OVER_SHOULDER", -2.5), "elbow": _env("LAMP_OVER_ELBOW", 3.1)}


def carry_target(goal):
    """The pose the hand carries the arm toward: the goal's arm pose plus CARRY_OVER (the shade's tilt is not set here)."""
    o = {k: math.radians(v) for k, v in CARRY_OVER.items()}
    return (goal[0] + o["swivel"], goal[1] + o["shoulder"], goal[2] + o["shoulder"] + o["elbow"], goal[3])
PUSH_IN = _env("LAMP_PUSH_IN", 0.004)  # the pushing fingertips are aimed this far into the shade's surface
PUSH_AT = np.array((M.KNUCKLE_TO_AXIS + 0.068, -(M.SHADE_LEN - M.KNUCKLE_BELOW_TOP) + 0.035))  # head-plane point pushed
PARK = {"left": (0.12, 0.34, TOP + 0.18), "right": (0.12, -0.34, TOP + 0.18)}
DOWN = G.frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))


def lerp_pose(a, b, s):
    return tuple(x + (y - x) * s for x, y in zip(a, b, strict=True))


def carried(pose_from, th1, th2, psi):
    """The pose after the arm is carried to (psi, th1, th2): the shade keeps its angle to the upper arm (its knuckle's
    friction holds it)."""
    return (psi, th1, th2, pose_from[3] - pose_from[2] + th2)


def _world(pose, rz):
    return M.to_world(LAMP_XY, pose[0], rz) + np.array((0.0, 0.0, TOP))


def head_frame(pose):
    """(knuckle position in the world, the head's rotation) for a lamp pose (psi, th1, th2, gam): the head turned psi about
    the vertical and tilted gam about the arm plane's -n axis from the drawing pose."""
    k = _world(pose, M.plane_points(*pose[1:])["K"])
    c, s = math.cos(pose[0]), math.sin(pose[0])
    cg, sg = math.cos(pose[3]), math.sin(pose[3])
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cg, 0.0, -sg], [0.0, 1.0, 0.0], [sg, 0.0, cg]])  # rotation about y by -gam
    return k, rz @ ry


def hand_on_arm(pose, out=0.0):
    """(TCP, rotation) of the right hand pinching the upper arm at `pose`, backed off `out` along its fingertips."""
    p = M.plane_points(*pose[1:])
    g = p["K"] + GRASP_FROM_K * (p["E"] - p["K"]) / M.L2
    u = M.unit(pose[0])
    tips = np.array((u[1], -u[0], 0.0))  # square to the arm's plane, pointing away from the robot's right
    th2 = pose[2]
    closing = -np.array((-math.sin(th2) * u[0], -math.sin(th2) * u[1], math.cos(th2)))  # the wrist's easier way up
    return _world(pose, g) + (TIP_PAST - out) * tips, G.frame(tips, closing)


def push_point(pose, gam):
    """(TCP, rotation) of the left hand's closed fingertips on the shade's near side, the shade tilted gam."""
    p = M.plane_points(*pose[1:3], gam)
    c = p["K"] + M.rot2(PUSH_AT, gam)
    n_out = M.rot2(np.array((1.0, 0.0)), gam)  # the surface's outward normal there (toward +r)
    tcp = c + (0.002 - PUSH_IN) * n_out
    u = M.unit(pose[0])
    tips = np.array((-u[0], -u[1], 0.0))  # pointing back toward the lamp's base
    return _world(pose, tcp), G.frame(tips, (0.0, 0.0, 1.0))


class _Plan:
    def __init__(self):
        self.wps = {"left": [], "right": []}
        self.t = {"left": 0.0, "right": 0.0}
        self.last = {}
        for a in self.wps:
            pos, rot = ik.tcp_pose(np.array(READY, float), np.array(BASES[a]))
            self.last[a] = (np.asarray(pos, float), FINGER_OPEN, np.asarray(rot, float))

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


def demo():
    global LAST_PLAN
    plan = _Plan()
    if VARIANT == "nothing":  # both hands stay at the ready pose for a full minute
        for a in plan.wps:
            plan.go(a, 60.0, label="wait")
        return plan.wps
    goal = GOAL_POSE if VARIANT != "low" else M.aim_pose(LAMP_XY, DOT_XY, PAGE_H, 0.22, GOAL_TILT)
    after = carried(START_POSE, goal[1], goal[2], goal[0])
    aim = carry_target(goal)
    brace = VARIANT != "one_hand"
    if brace:
        p_out, r0 = hand_on_arm(START_POSE, BACK)
        p_in, _ = hand_on_arm(START_POSE)
        plan.go("right", 2.5, p_out + (0.0, 0.0, 0.03), FINGER_OPEN, r0, "to the upper arm")
        plan.go("right", 1.0, p_out, label="level with the rods")
        plan.go("right", 1.5, p_in, label="fingers round the rods")
        plan.go("right", 1.0, finger=FINGER_CLOSED, label="pinch")
        plan.go("right", 0.5, label="settle")
        n = 36
        for i in range(1, n + 1):
            s = ik.smoothstep(i / n)
            pose = carried(START_POSE, *(START_POSE[k] + (aim[k] - START_POSE[k]) * s for k in (1, 2, 0)))
            p, r = hand_on_arm(pose)
            plan.go("right", CARRY_S / n, p, rot=r, label="carry the arm")
        plan.go("right", 0.5, label="hold")
    else:  # the arm is carried by the same hand first, then let go before the tilt
        p_out, r0 = hand_on_arm(START_POSE, BACK)
        plan.go("right", 2.5, p_out + (0.0, 0.0, 0.03), FINGER_OPEN, r0, "to the upper arm")
        plan.go("right", 1.0, p_out, label="level with the rods")
        plan.go("right", 1.5, hand_on_arm(START_POSE)[0], label="fingers round the rods")
        plan.go("right", 1.0, finger=FINGER_CLOSED, label="pinch")
        n = 36
        for i in range(1, n + 1):
            s = ik.smoothstep(i / n)
            pose = carried(START_POSE, *(START_POSE[k] + (aim[k] - START_POSE[k]) * s for k in (1, 2, 0)))
            p, r = hand_on_arm(pose)
            plan.go("right", CARRY_S / n, p, rot=r, label="carry the arm")
        plan.go("right", 0.8, finger=FINGER_OPEN, label="open")
        plan.go("right", 1.2, hand_on_arm(after, BACK)[0], label="back off the rods")
        plan.go("right", 2.0, (0.35, -0.30, TOP + 0.38), label="away")
    plan.until("left", plan.t["right"] - 4.0)
    if VARIANT != "no_tilt":
        g0, g1 = after[3], goal[3] - TILT_OVER
        p0, r = push_point(after, g0 + math.radians(4.0))
        plan.go("left", 3.0, p0 + M.to_world((0, 0), after[0], (0.06, 0.03)), FINGER_CLOSED, r, "to the shade")
        plan.until("left", plan.t["right"])
        plan.go("left", 1.2, p0, label="fingertips to the shade")
        n = 24
        for i in range(1, n + 1):
            gam = g0 + math.radians(4.0) + (g1 - g0 - math.radians(4.0)) * ik.smoothstep(i / n)
            plan.go("left", TILT_S / n, push_point(after, gam)[0], label="tilt the shade")
        plan.go("left", 0.5, label="hold")
        p1, _ = push_point(after, g1)
        plan.go("left", 1.0, p1 + M.to_world((0, 0), after[0], (0.06, 0.0)), label="back off the shade")
        plan.go("left", 2.0, PARK["left"], rot=DOWN, label="away")
    if brace and VARIANT != "hold":
        plan.until("right", plan.t["left"])
        plan.go("right", 0.8, finger=FINGER_OPEN, label="open")
        p, _ = hand_on_arm(aim, BACK)
        plan.go("right", 1.2, p, label="back off the rods")
        plan.go("right", 2.0, (0.35, -0.30, TOP + 0.38), label="away")
    end = max(plan.t.values()) + 8.0
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
    for name, pose in (("start", START_POSE), ("goal", GOAL_POSE), ("carried", carried(START_POSE, GOAL_POSE[1], GOAL_POSE[2], GOAL_POSE[0]))):
        w = M.fk(LAMP_XY, *pose)
        spot = M.beam_point(w["M"], w["axis"], PAGE_H)
        err = None if spot is None else round(1000 * float(np.hypot(*(spot[:2] - DOT_XY))), 1)
        print(name, "deg", [round(math.degrees(a), 1) for a in pose], "mouth h", round(w["M"][2], 3), "K", np.round(w["K"], 3), "spot err mm", err)
    wps = demo()
    t = max((sum(w.duration for w in ws) for ws in wps.values()), default=0.0)
    print(f"demo {t:.1f} s ({t * 20:.0f} steps), IK worst mm/deg, bad:", ik_errors(wps))
