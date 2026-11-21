"""The scripted demo for roll_the_iv_roller_clamp_to_20_drops_a_minute.py: ``plan(M)`` builds both arms' waypoints from
the task module's namespace ``M``. Kept apart so the task file stays short; the task file's ``demo()`` calls it."""

from __future__ import annotations

import math

import numpy as np
from benchkit import quat_xyzw, rpy_matrix, tool_state
from tiltgrip import Grip
from yam_arena import ik, station
from yam_arena.constants import PAD_TIP_PAST_TCP, READY
from yam_arena.demo import Waypoint

# -- the scripted demo ------------------------------------------------------------------------------------------------
# Left arm: pinch the far end of the clamp's housing (toward the bag) across its 20 mm width, hand tilted 30 degrees
# toward the right arm, and keep it still. Right arm: closed fingertips tilted 45 degrees toward the left arm, on the
# wheel's upper face toward shut; rest on it, then roll it open in pushes of at most 3 mm, the first aimed 2 mm short of
# the travel that gives 20 drops a minute, then up to eleven corrections. After every push the fingertip eases 3 mm off
# the face and waits there while the drops show the new rate, as a nurse takes the thumb off the wheel to watch. Each
# correction is worked out when it starts, from the wheel's travel as the simulator reports it (a scripted demo may read
# the simulator's state; nothing is moved by script):
# - under 18.5 drops a minute, the fingertip rolls the wheel on toward open, from the face toward shut, as before;
# - over 21.5, it lifts over the wheel and rolls it back toward shut from the other side: the flat underside of the same
#   closed fingertips laid on the wheel's upper face toward the bag, pressed along that face's normal (a thumb rolling a
#   wheel back), aimed a little under the target, as the setting is finished rolling on; it stays on that side until a
#   push the other way is needed;
# - in between, it stays eased off and watches.
# A push presses the fingertip into the face by the press the wheel needs before it moves at all (learned, for each
# face, from the pushes before) and sends it on along the housing by how far the wheel should move. Pressing open, the
# press goes along the fingertips' own axis, where this arm is stiffest (0.82 N/mm), so 3.2 mm of it is 2.5 N against
# the wheel's 2.1 N breakaway on the face. Pressing back, it goes across the fingertips, where the arm gives through its
# soft wrist (0.15 N/mm), so it takes about 17 mm of press for the same 2.5 N; the arm's sideways give under that press
# is taken out in advance so the fingertip stays on the face.
# Then the right fingertip backs straight off, the left hand opens and lifts, and both arms go well clear and back to
# their start pose, which ends the trial, and hold it.

LEFT_GRIP = Grip.deg(yaw=-90, pitch=30)  # closes along world x, across the housing; tilted toward the right arm
RIGHT_GRIP = Grip.deg(yaw=90, pitch=45)  # closes along world x; fingertips toward the bag side, 45 degrees down
PINCH_IN = 0.012  # m from the housing's far end to the pinch (16 before the roll back): the fingertip rolling the
# wheel back from the bag side stays 10 mm clear of the left hand up to 11 mm of travel (9.3 mm at 12); nearer the end
# (8-10 mm) letting go nudged the wheel in MuJoCo's soft friction
PINCH_Z = 0.018  # m: TCP above the table while pinching (pad tips 3 mm clear of it)
OPEN = -0.020  # finger joint: pads 40 mm apart, round the 20 mm housing
PINCH = -0.008  # pads commanded 16 mm apart: 2 mm into each side wall
EASED_PINCH = -0.0095  # pads 19 mm apart, barely on the walls: letting go (opened at once, the housing's
# spring-back nudged the wheel in MuJoCo's soft friction)
SHUT = 0.0
FIRST_SHORT = 0.002  # m: the first push aims this far short of the target travel ...
SECOND_SHORT = 0.0005  # m: ... the first correction this far ...
THIRD_SHORT = 0.0007  # m: ... the second, which acts only if the first fell well short, this far ...
LAST_SHORT = 0.00005  # m: ... and the rest this far (the full-score band is 0.37 mm wide, the target 0.18 mm inside it)
BACK_UNDER = 0.0005  # m: a correction back toward shut aims this far under the target: rolling back is coarse (the
# arm's soft wrist and its joint friction), so it is aimed to land in the band or a little under it, and the fine
# setting is left to the pushes on, as before
BAND_MARGIN = 1.5  # drops a minute: a wheel within this of the edges of 18-22 a minute is left alone
DEADBAND = 0.00003  # m: a push whose wheel is already this close to its aim (or past it) leaves the wheel alone
# How hard a push presses: the wheel moves by how far the fingertip is sent along the housing plus sqrt 2 x (its press
# into the face along the normal less NEED), where NEED is the press it takes before the wheel moves at all. Each push
# that moves the wheel measures NEED; one that doesn't presses harder. A wheel that creeps under any touch (MuJoCo's
# soft friction) has a NEED near 0; one held hard up to its breakaway (PhysX) about 2.6 mm open, 14 mm back.
BREAKAWAY_PRESS = 0.0032  # m: NEED rolling open for a wheel that didn't move under the resting fingertip: 2.6 N
PRESS_LIMITS = (-0.003, 0.005)  # m: a push goes past the face along its normal within these (backed off, pressed in:
# 5 mm keeps the fingertip's target above the housing's walls); any further goes into lead along the housing
NEED_FLOOR = -0.0005  # m: the least NEED a push that carried the wheel on past its aim is taken to show
NOT_MOVED = 0.00005  # m: a push that moved the wheel less than this didn't break it loose: the next one presses this
RAISE = 0.0003  # m harder at the same place, without asking the wheel to move further, up to ...
STALL_PRESS = 0.0035  # m: ... 2.9 N on the face. Only a first push that didn't move the wheel leaves it to this: one
# that moved it measures NEED, whatever it is
MAX_LEAD = 0.008  # m: how far past the target travel the fingertip may be sent along the housing: past 8.5 mm the
# closed fingertip comes near the left hand's pinch if the wheel never moves
MAX_STEP = 0.003  # m: the most a correction asks the wheel to move, so the fingertip stays on the face ...
FIRST_STEP = 0.004  # m: ... and the first push, whose lead also breaks loose a wheel needing 3.5 N (6 mm of press)
# Rolling back, from the bag side: the closed fingertips' flat underside (UNDER from their axis, facing down and toward
# shut) is laid on the wheel's 45-degree face toward the bag, the tip's end TIP_DOWN down that face from its middle
UNDER = 0.0077  # m: the closed fingertips' half-thickness across their axis (the YAM finger hulls: 15.4 mm)
TIP_END = 0.0128  # m: their end past the TCP along the axis (hulls; the pad tips are 13 mm past it)
TIP_DOWN = 0.0005  # m
BACK_PRESS = 0.0175  # m: NEED rolling back for a wheel not yet pushed back: 2.6 N on the face at 0.15 N/mm
BACK_PRESS_LIMITS = (-0.003, 0.024)  # m (the most: 3.5 N); any further goes into lead along the housing
BACK_RAISE = 0.0015  # m harder after a back push that didn't move the wheel (0.2 N), up to ...
BACK_STALL = 0.022  # m: ... 3.3 N
BACK_STEP = 0.002  # m: the most a correction back asks the wheel to move
BACK_LEAD = 0.004  # m: the most the fingertip may be sent back along the housing past the target
CROSS_Z = 0.031  # m above the table: the fingertip's end crossing over the wheel (its underside 7 mm over the top)
NEAR = 0.006  # m off a face before pressing onto it
FIXES = 11  # corrections after the first push; each one whose wheel is already in the band stays eased off
EASE = 0.003  # m off the face between pushes, while watching
BACK_OFF = 0.020  # m straight back off the face before lifting
LIFT = 0.060
LEFT_PARK = (0.24, 0.22, 0.15)  # above the table, m
RIGHT_PARK = (0.24, -0.22, 0.15)
HOME_S = 3.0  # s from the park to the start pose
HOLD_S = 3.0  # s holding the start pose at the end
JOINT_KP = (80.0, 80.0, 80.0, 10.0, 10.0, 10.0)  # the arm's joint stiffness (N m/rad): its give under a press


def home(arm: str) -> tuple:
    """(TCP position, grip) of the arm's start pose."""
    pos, rot = ik.tcp_pose(READY, station.BASES[arm])
    return tuple(float(v) for v in pos), Grip.from_matrix(rot)


class Live:
    """A Waypoint position (world, m) given by ``fn(states)``. ``np.asarray`` works it out the first time it's read,
    which is when the scripted arm starts toward it, from the latest object states, and keeps that value. Before any
    states exist (a plan built offline) it uses ``fallback(None)``."""

    def __init__(self, fn, latest):
        self.fn, self.latest, self.value = fn, latest, None

    def resolve(self):
        if self.value is None:
            self.value = tuple(float(v) for v in self.fn(self.latest.get("states")))
        return self.value

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.resolve(), dtype=dtype or float)

    def __iter__(self):
        return iter(self.resolve())

    def __len__(self):
        return 3

    def __getitem__(self, i):
        return self.resolve()[i]


def sideways_give(tcp, grip: Grip, normal) -> np.ndarray:
    """How far (m) the TCP gives along a face, per metre it is pressed into the face along ``normal`` (the face's
    outward normal, world): the arm's joint stiffness at that pose, mapped through the TCP Jacobian. The contact pushes
    the fingertip back along the normal only, so the arm also slides along the face; commanding the opposite slide
    keeps the fingertip where it was laid."""
    q, _, _ = ik.solve(np.asarray(tcp, dtype=float), ik.matrix_to_quat_xyzw(np.asarray(grip.rotation)),
                       q_init=np.asarray(READY, dtype=float), base_pos=station.BASES["right"])
    base = station.BASES["right"]
    jac = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        jac[:, i] = (ik.tcp_pose(q + dq, base)[0] - ik.tcp_pose(q - dq, base)[0]) / 2e-6
    give = jac @ np.diag(1.0 / np.asarray(JOINT_KP)) @ jac.T  # m per N at the TCP
    n = np.asarray(normal, dtype=float)
    per_n = give @ n  # the TCP's give under 1 N pushing it out along the normal
    along = per_n - (per_n @ n) * n  # its part along the face
    return along / float(n @ give @ n)  # per metre of press


def plan(M) -> dict:
    """``M`` is the task module's namespace."""
    top = M.TOP
    start = tool_state(M.CLAMP, M.CLAMP.pos, quat_xyzw(rpy_matrix((0.0, 0.0, M.CLAMP_YAW))))
    target = M.travel_for(M.TARGET)
    low = M.travel_for(18 + BAND_MARGIN - 1.0)  # under this it drips under 18.5 a minute: roll it on
    high = M.travel_for(22 - BAND_MARGIN + 1.0)  # over this, over 21.5: roll it back
    a_right = RIGHT_GRIP.approach
    a_left = LEFT_GRIP.approach
    memo: dict = {"face": 1}  # face: 1 the face toward shut (rolls it open), -1 the face toward the bag (rolls it back)
    SQRT2 = math.sqrt(2.0)
    n_back = np.array((1.0, 0.0, 1.0)) / SQRT2  # the bag-side face's outward normal, tool frame
    down_back = np.array((1.0, 0.0, -1.0)) / SQRT2  # down that face, toward the bag

    def clamp(states):
        return states["roller_clamp"] if states else start

    def travel(states) -> float:
        return float(states["roller_clamp"].sliders["wheel"]) if states else 0.0

    def pinch(states):
        p = clamp(states).to_world((M.LENGTH / 2 - PINCH_IN, 0.0, 0.0))
        return np.array((p[0], p[1], top + PINCH_Z))

    def tcp_on_face(states, at: float, back: float = 0.0, press: float = 0.0):
        tip = M.facet_point(clamp(states), at, press) - back * a_right
        return tip - PAD_TIP_PAST_TCP * a_right

    def back_normal(states):
        return clamp(states).rotation() @ n_back

    def tcp_on_back(states, at: float, back: float = 0.0, press: float = 0.0):
        """The TCP with the fingertips' underside on the bag-side face of a wheel ``at`` m from shut, ``press`` m into
        it along its normal (less the arm's give along the face) or ``back`` m off it."""
        c = clamp(states)
        local = np.array((M.WHEEL_X0 + at + M.FACET, 0.0, M.WHEEL_Z + M.FACET)) + TIP_DOWN * down_back
        n = back_normal(states)
        under = c.to_world(local) + (back - press) * n
        return under + UNDER * n - TIP_END * a_right - max(press, 0.0) * memo["give"]

    memo["give"] = np.zeros(3)
    memo["give"] = sideways_give(tcp_on_back(None, target), RIGHT_GRIP, back_normal(None))

    def eased_at(states, face: int, t: float):
        return tcp_on_face(states, t, back=EASE) if face > 0 else tcp_on_back(states, t, back=EASE)

    def send(states, key: str, t: float, move: float, face: int):
        """Push to move the wheel ``move`` m on from ``t`` (toward open on face 1, back toward shut on face -1): NEED
        pressed into the face along its normal (within the limits, any more as lead) and ``move`` as lead along the
        housing."""
        need = memo[f"need{face}"]
        lo, hi = PRESS_LIMITS if face > 0 else BACK_PRESS_LIMITS
        press = float(np.clip(need, lo, hi))
        if face > 0:
            at = min(t + move + SQRT2 * (need - press), target + MAX_LEAD - press / SQRT2)
            M.LATEST["command"] = (key, at, "push", bool(states), press)  # for the run record only
            memo["sent"] = (t, press, at - t, face)
            return tcp_on_face(states, at, press=press)
        at = max(t - move - SQRT2 * (need - press), target - BACK_LEAD)
        M.LATEST["command"] = (key, at, "back", bool(states), press)  # for the run record only
        memo["sent"] = (t, press, t - at, face)
        return tcp_on_back(states, at, press=press)

    def learn(t: float, way: int) -> bool:
        """What the last push did: the press its face needs. True if it didn't move the wheel and this push is on the
        same face, so presses harder at the same place."""
        t0, press, lead, face = memo.pop("sent", (t, 0.0, -2.0, memo["face"]))
        if lead <= -1.0:
            return False
        moved = (t - t0) * face
        stalled = moved <= NOT_MOVED
        need = memo[f"need{face}"]
        if stalled:  # pressed too lightly: from at least touching the face, a little harder each time
            raise_, stall = (RAISE, STALL_PRESS) if face > 0 else (BACK_RAISE, BACK_STALL)
            memo[f"need{face}"] = min(max(need, 0.0) + raise_, max(need, stall))
        else:  # a push carried on past where it was sent says little about how hard the next must press
            memo[f"need{face}"] = max(press + (lead - moved) / SQRT2, NEED_FLOOR)
        return stalled and way == face

    def first(states):
        """The first push, worked out when it starts: the resting fingertip moved the wheel only if it needs no press
        past the face."""
        if not states:  # a plan built offline: the long push where it is sent
            M.LATEST["command"] = ("c1", target - FIRST_SHORT, "push", False, 0.0)
            return tcp_on_face(states, target - FIRST_SHORT)
        t = travel(states)
        memo["need1"] = -t / SQRT2 if t > NOT_MOVED else BREAKAWAY_PRESS
        memo["need-1"] = BACK_PRESS
        return send(states, "c1", t, min(target - FIRST_SHORT - t, FIRST_STEP), 1)

    def correction(k: int, short: float):
        """Correction ``k`` (2 on): its four moves, each worked out when the arm starts toward it. ``way`` decides which
        way the wheel must go, from its travel then: 1 on (aimed ``short`` under the target), -1 back (aimed BACK_UNDER
        under it), 0 leave it. The fingertip lifts over the wheel only to change faces."""
        key = f"c{k}"
        aim = {1: target - short, -1: target - BACK_UNDER}
        mine: dict = {}

        def way(states):
            if "way" not in mine:
                if not states:  # a plan built offline: rolls the wheel on, except the third, which rolls it back
                    mine["way"] = -1 if k == 4 else 1
                else:
                    t = travel(states)
                    mine["way"] = 1 if t < min(low, aim[1] - DEADBAND) else -1 if t > high else 0
            return mine["way"]

        def changing(states) -> bool:
            w = way(states)
            return w != 0 and w != memo["face"]

        def lift(states):
            here = eased_at(states, memo["face"], travel(states))
            return np.array((here[0], here[1], top + CROSS_Z - TIP_END * a_right[2])) if changing(states) else here

        def to_face(states, back: float):
            t, w = travel(states), way(states)
            return tcp_on_face(states, t, back=back) if w > 0 else tcp_on_back(states, t, back=back)

        def over(states):
            if not changing(states):
                return eased_at(states, memo["face"], travel(states))
            near = to_face(states, NEAR)
            return np.array((near[0], near[1], top + CROSS_Z - TIP_END * a_right[2]))

        def near(states):
            return to_face(states, NEAR) if changing(states) else eased_at(states, memo["face"], travel(states))

        def push(states):
            w = way(states)
            if w != 0:
                memo["face"] = w  # the fingertip is at this face now
            if not states:  # offline: press the shut wheel as hard as a push starts out to
                M.LATEST["command"] = (key, 0.0, "push" if w > 0 else "back", False, 0.0)
                return (tcp_on_face(states, 0.0, press=BREAKAWAY_PRESS) if w > 0
                        else tcp_on_back(states, 0.0, press=BACK_PRESS))
            t = travel(states)
            stalled = learn(t, w)
            if w == 0 or (t - aim[w]) * w >= -DEADBAND:  # already there (or past it): stay eased off and watch
                memo["sent"] = (t, 0.0, -2.0, memo["face"])  # nothing sent: the next push learns nothing from this
                M.LATEST["command"] = (key, t, "eased", True, 0.0)
                return eased_at(states, memo["face"], t)
            step = MAX_STEP if w > 0 else BACK_STEP
            return send(states, key, t, 0.0 if stalled else min(abs(aim[w] - t), step), w)

        return lift, over, near, push

    def eased(states):  # off the face the last push used, at the wheel
        t = travel(states)
        if states and M.LATEST.get("command"):  # for the run record: the push is over
            M.LATEST["command"] = (M.LATEST["command"][0], t, "eased", True, 0.0)
        return eased_at(states, memo["face"], t)

    def leave(states, more: float):
        """Off the face the hand is on, then ``more`` m up: from the face toward shut straight back along the
        fingertips, BACK_OFF; from the bag-side face straight up, BACK_OFF (back along its normal leads to the pinch)."""
        t = travel(states)
        if memo["face"] > 0:
            return tcp_on_face(states, t, back=BACK_OFF) + np.array((0.0, 0.0, more))
        return tcp_on_back(states, t, back=EASE) + np.array((0.0, 0.0, BACK_OFF + more))

    L = lambda fn: Live(fn, M.LATEST)  # noqa: E731
    over_pinch = L(lambda st: pinch(st) - LIFT * a_left)
    at_pinch = L(pinch)
    above_face = L(lambda st: tcp_on_face(st, 0.0, back=0.040))
    near_face = L(lambda st: tcp_on_face(st, 0.0, back=0.010))
    on_face = L(lambda st: tcp_on_face(st, 0.0))
    shorts = [SECOND_SHORT, THIRD_SHORT] + [LAST_SHORT] * (FIXES - 2)
    names = ["first", "second", "third"] + [f"{k}th" for k in range(4, FIXES + 1)]

    right = [
        Waypoint(3.0, above_face, SHUT, "over the wheel", RIGHT_GRIP),
        Waypoint(3.5, above_face, SHUT, "wait for the left hand", RIGHT_GRIP),
        Waypoint(2.0, near_face, SHUT, "fingertip to the wheel", RIGHT_GRIP),
        Waypoint(1.5, on_face, SHUT, "onto its face", RIGHT_GRIP),
        Waypoint(3.0, on_face, SHUT, "rest on the face", RIGHT_GRIP),
        Waypoint(8.0, L(first), SHUT, "roll the wheel open", RIGHT_GRIP),
    ]
    ease = L(eased)
    right += [Waypoint(1.0, ease, SHUT, "ease off the wheel", RIGHT_GRIP),
              Waypoint(4.0, ease, SHUT, "watch the drops", RIGHT_GRIP)]
    for k, short in enumerate(shorts):  # each: lift over the wheel if it must change faces, push, ease off, watch
        lift, over, near, push = (L(fn) for fn in correction(k + 2, short))
        ease = L(eased)
        right += [
            Waypoint(1.0, lift, SHUT, "lift over the wheel if it must go the other way", RIGHT_GRIP),
            Waypoint(1.5, over, SHUT, "over the face it needs", RIGHT_GRIP),
            Waypoint(1.0, near, SHUT, "to that face", RIGHT_GRIP),
            Waypoint(3.0, push, SHUT, f"roll the wheel: {names[k]} correction", RIGHT_GRIP),
            Waypoint(1.0, ease, SHUT, "ease off the wheel", RIGHT_GRIP),
            Waypoint(4.0 if k < 4 else 3.0, ease, SHUT, "watch the drops", RIGHT_GRIP),
        ]
    off = L(lambda st: leave(st, 0.0))
    up = L(lambda st: leave(st, LIFT / SQRT2))
    right += [
        Waypoint(1.5, off, SHUT, "straight back off the wheel", RIGHT_GRIP),
        Waypoint(1.5, up, SHUT, "up", RIGHT_GRIP),
    ]
    busy = sum(w.duration for w in right)
    left = [
        Waypoint(3.0, over_pinch, OPEN, "over the housing's far end", LEFT_GRIP),
        Waypoint(2.0, at_pinch, OPEN, "down round the housing", LEFT_GRIP),
        Waypoint(1.0, at_pinch, PINCH, "pinch the housing", LEFT_GRIP),
    ]
    left.append(Waypoint(busy - sum(w.duration for w in left), at_pinch, PINCH, "keep the housing pinched", LEFT_GRIP))
    left += [
        Waypoint(4.0, at_pinch, EASED_PINCH, "let go", LEFT_GRIP),  # slowly, so the housing settles without a jolt
        Waypoint(1.0, at_pinch, OPEN, "open", LEFT_GRIP),
        Waypoint(1.5, over_pinch, OPEN, "lift away", LEFT_GRIP),
        Waypoint(2.0, (M.CLAMP_XY[0] - 0.04, LEFT_PARK[1], top + LEFT_PARK[2]), OPEN, "clear", LEFT_GRIP),
    ]
    right += [Waypoint(2.5, (RIGHT_PARK[0], RIGHT_PARK[1], top + RIGHT_PARK[2]), SHUT, "clear", RIGHT_GRIP)]
    left_s, right_s = (sum(w.duration for w in ws) for ws in (left, right))
    for arm, ws, finger, wait in (("left", left, OPEN, max(0.0, right_s - left_s)),
                                  ("right", right, SHUT, max(0.0, left_s - right_s))):
        pos, grip = home(arm)
        ws.append(Waypoint(HOME_S + wait, pos, finger, "back to the start pose", grip))
        ws.append(Waypoint(HOLD_S, pos, finger, "hold the start pose", grip))
    return {"left": left, "right": right}


def total_s(p: dict) -> float:
    return max(sum(w.duration for w in ws) for ws in p.values())


def steps(p: dict, hz: float = 20.0) -> int:
    return int(math.ceil(total_s(p) * hz))
