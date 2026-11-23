"""Scoring for rockwell-hardness-test, as pure functions of the logged state (no Isaac).

A manual dial Rockwell tester (Insize ISH-R150 class, the HR-150A layout) takes one HRC reading on a test block that
lies on its flat anvil. The operator's steps, from the maker's manual (MN-ISH-R150-E, "Hardness tester operation
procedure"):

1. turn the elevating handwheel clockwise so the block rises slowly onto the diamond indenter "with no impact", until
   the long pointer has made three turns and stands at C with the short pointer on the red dot: the 98.07 N
   preliminary force. "The deviation of long pointer shall not exceed 5 grades, otherwise the pointer shall not be
   allowed to turn back, instead that the test spot shall be changed";
2. push the main-load handle backward: the weights load the indenter to 1471 N over 4-6 s (the oil dashpot sets the
   pace); the total force is held 5 s;
3. pull the handle forward to take the main load off, leaving the preliminary force; the long pointer gives HRC;
4. turn the handwheel counterclockwise to lower the block. Never turn the handwheel while the main load is on.
   Test spots stay at least 3 mm apart and 3 mm from the block's edge.

The simulation stands the material's response in with rules (rules.md §3, construct validity): the indenter's travel
is the block's rise past first contact, read from the handwheel through the elevating screw's pitch, and the main
load's depth comes from the block's certified hardness. The robot still turns the real handwheel and works the real
handle.

The preliminary load is mechanical (revision, 2026-11-23). The block's top meets the indenter's point in PhysX and is
pressed against it by the anvil's spring, whose rate puts the 98.07 N preliminary force on the block at the three-turn
travel. The handwheel feels that force back through the elevating screw: ``wheel_torque`` is a square-thread screw's
raising torque for the force at that travel, held against the wheel while it turns clockwise (scaled down near rest:
it is set once per control step from the wheel's speed). So the wheel turns
freely until the block meets the point, then stiffens to about 0.22 N m (2.7 N at a knob) at the set point. The
screw's lowering torque is left out: set once per control step on a wheel this light, a torque against a backward
jitter pushed the wheel forward past C once the hand let go (official sim, 2026-11-23).

Grades (``score`` stops at 0.75; 1.0 only comes from ``success`` holding 1 s):
- 0.25: the preliminary force set at a fresh spot: the long pointer within 5 divisions of C after its three turns, and
  never more than 5 divisions past it on that approach;
- 0.5: then the main load released with the preliminary force still set, fully applied, and the handwheel left alone;
- 0.75: then held 3-7 s at full load and taken off (handle forward): the dial shows a valid reading;
- 1.0: then the block lowered off the indenter (at least 0.5 mm clear), still on the anvil, the handle forward and both
  grippers at least 4 cm from the handwheel's knobs and the handle's knob.
An approach that overshoots spoils its spot (0 for it); pushing the handle without the preliminary force set, or
turning the handwheel under load, spoils that test. A later test at a fresh spot can still earn full credit, as the
manual allows.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# -- the tester and block (spec: docs/robocurve/rockwell-hardness-test-spec.md) ----------------------------------------
DIVISION = 0.002e-3  # one dial division: 0.002 mm of indenter travel (manual: "Every 0.002 mm ... one unit")
PRE_TRAVEL = 300 * DIVISION  # three turns of the long pointer (100 divisions a turn) to the preliminary force
PRE_TOL = 5 * DIVISION  # "shall not exceed 5 grades"
PITCH = 0.004  # elevating screw rise per handwheel turn (m) [estimate: measure on the bought tester]
START_GAP = 0.0012  # block top below the indenter's point at the start (m)
TRAVEL_STOP = PRE_TRAVEL + 0.001  # the screw's upper stop in the sim, 1 mm past the preliminary force
LOWERED = -0.0005  # block top at least 0.5 mm below the point counts as lowered

# The preliminary load's path (revision 2026-11-23). PRELOAD_N is the standard's (ISO 6508 / ASTM E18: 10 kgf); the
# screw's size and friction are estimates until measured on the tester (a spring scale on a knob, read at C).
PRELOAD_N = 98.07
ANVIL_RATE = PRELOAD_N / (300 * 0.002e-3)  # N/m: the anvil's spring, 98.07 N at the three-turn travel
SCREW_D = 0.020  # the elevating screw's mean diameter (m) [estimate]
SCREW_MU = 0.15  # its thread friction, oiled steel [estimate]
THREAD_HALF = math.radians(15.0)  # trapezoidal thread's flank half-angle
SPEED_FULL = 0.02  # rad/s clockwise: the screw's full raising torque from this speed up (the demo creeps at 0.022),
# scaled down to 0 at rest, so a torque set at one control step can't throw a wheel that has just stopped back

BLOCK_HRC = 62.0  # the test block's certified value [estimate: from the block's certificate]
ELASTIC = 0.030e-3  # extra depth under the total force that springs back when it comes off [estimate]

# The main-load handle (rad about +y from vertical; positive swings its knob back, away from the robot).
HANDLE_FORWARD = math.radians(-20.0)  # its forward stop, where it rests
HANDLE_BACK = math.radians(40.0)  # its back stop
HANDLE_RELEASED = math.radians(30.0)  # past this the main load is released
HANDLE_HOME = math.radians(-15.0)  # within this the main load is off
APPLY_S = 5.0  # the dashpot's application time (manual: 4-6 s)
DWELL = (3.0, 7.0)  # seconds at full load (manual: 5 s; the band is ours)
WHEEL_STILL = DIVISION  # under load, the handwheel may move the block at most one division

BLOCK_RADIUS = 0.032  # a 64 mm test block
SPOT_GAP = 0.003  # manual: 3 mm between spots and from the edge
DT = 0.05  # control step (20 Hz)
CLEAR = 0.04  # grippers at least 4 cm from every knob at the end
PARTIAL_CAP = 0.75


def travel(wheel: float) -> float:
    """The block's rise past first contact with the indenter (m) for a handwheel angle (rad, clockwise from above
    positive: clockwise raises it)."""
    return wheel * PITCH / (2 * math.pi) - START_GAP


def wheel_for(rise: float) -> float:
    """The handwheel angle (rad) that gives a block rise ``rise`` (m)."""
    return (rise + START_GAP) * 2 * math.pi / PITCH


def preliminary_force(rise: float) -> float:
    """The force (N) between the block and the indenter's point at a travel ``rise`` past first contact."""
    return ANVIL_RATE * min(max(rise, 0.0), TRAVEL_STOP)


def screw_angles() -> tuple[float, float]:
    """The elevating screw's lead angle and its thread's friction angle (rad)."""
    return math.atan(PITCH / (math.pi * SCREW_D)), math.atan(SCREW_MU / math.cos(THREAD_HALF))


def wheel_torque(rise: float, speed_cw: float) -> float:
    """The torque (N m) the elevating screw puts on the handwheel against a clockwise (raising) turn at ``speed_cw``
    rad/s and a travel ``rise``: a square thread's raising torque, F r tan(friction angle + lead angle), from
    SPEED_FULL up, scaled down to 0 at rest; 0 turning back (see the module docstring)."""
    f = preliminary_force(rise)
    if f <= 0.0 or speed_cw <= 0.0:
        return 0.0
    lead, friction = screw_angles()
    return f * SCREW_D / 2 * math.tan(friction + lead) * min(1.0, speed_cw / SPEED_FULL)


def divisions_from_c(rise: float) -> float:
    """How far the long pointer stands past C at the end of its three turns (divisions; negative short of it)."""
    return (rise - PRE_TRAVEL) / DIVISION


def preliminary_set(rise: float) -> bool:
    return abs(rise - PRE_TRAVEL) <= PRE_TOL + 1e-12


def permanent_depth() -> float:
    """The depth increase left by the main load on the block (m): HRC = 100 - e / 0.002 mm."""
    return (100.0 - BLOCK_HRC) * DIVISION


@dataclass
class Attempt:
    """One approach of the block to the indenter, from first contact until it is lowered off again."""

    spot: np.ndarray  # where the indenter met the block, in the block's frame (m, xy)
    fresh: bool  # at least SPOT_GAP from earlier spots and the block's edge
    seated: bool  # the block lay flat on the anvil at first contact
    max_rise: float = 0.0
    reached: bool = False  # the preliminary force was set at some step
    pushed_t: float | None = None  # handle went past HANDLE_RELEASED
    pushed_ok: bool = False  # ... with the preliminary force set, at a fresh spot, no overshoot, block seated
    rise_at_push: float = 0.0
    full_t: float | None = None  # main load fully applied
    left_t: float | None = None  # handle came back past HANDLE_RELEASED (removal began)
    removed_t: float | None = None  # handle back home: main load off
    wheel_moved: bool = False  # the handwheel turned under load

    @property
    def overshot(self) -> bool:
        return self.max_rise > PRE_TRAVEL + PRE_TOL + 1e-12

    def stage(self) -> float:
        """This test's grade before the final lowering: 0, 0.25, 0.5 or 0.75."""
        if not (self.fresh and self.seated) or self.overshot or not self.reached:
            return 0.0
        if self.pushed_t is None:
            return 0.25
        if not self.pushed_ok or self.wheel_moved:
            return 0.0
        if self.full_t is None or (self.left_t is not None and self.left_t < self.full_t):
            return 0.25  # released, but not (yet) fully applied, or taken off too soon
        if self.removed_t is None:
            return 0.5
        dwell = self.left_t - self.full_t
        return 0.75 if DWELL[0] <= dwell <= DWELL[1] else 0.5

    def depth(self, t: float) -> float:
        """The main load's extra indenter depth now (m): rising over APPLY_S, springing back ELASTIC on removal."""
        if self.pushed_t is None or not self.pushed_ok:
            return 0.0
        full = permanent_depth() + ELASTIC
        if self.removed_t is not None:
            return permanent_depth()
        end = self.left_t if self.left_t is not None else t
        return full * min(1.0, max(0.0, (end - self.pushed_t) / APPLY_S))


@dataclass
class HardnessRecord:
    """Everything the grade needs from the trial, updated once per control step."""

    t: float = 0.0
    attempts: list = field(default_factory=list)
    spots: list = field(default_factory=list)  # every spot the indenter has met (block frame)
    last_rise: float | None = None
    handle_pushed: bool = False

    def reset(self) -> None:
        self.__init__()

    @property
    def current(self) -> Attempt | None:
        a = self.attempts[-1] if self.attempts else None
        return a if a is not None and self.last_rise is not None and self.last_rise > 0.0 else None

    def best(self) -> Attempt | None:
        done = [a for a in self.attempts if a.stage() > 0]
        return max(done, key=lambda a: a.stage()) if done else None

    def step(self, wheel: float, handle: float, seated: bool, spot, dt: float = DT) -> None:
        """Add a control step: handwheel and handle angles (rad), whether the block lies flat on the anvil, and the
        indenter's axis in the block's frame (m, xy)."""
        self.t += dt
        rise = travel(wheel)
        prev = self.last_rise
        spot = np.asarray(spot, float)[:2]
        if rise > 0.0 and (prev is None or prev <= 0.0):  # first contact: a new test
            fresh = all(np.linalg.norm(spot - s) >= SPOT_GAP for s in self.spots)
            fresh = fresh and _inside_block(spot)
            self.attempts.append(Attempt(spot=spot, fresh=fresh, seated=seated))
            self.spots.append(spot)
        a = self.attempts[-1] if self.attempts else None
        live = a is not None and rise > 0.0
        if live:
            a.max_rise = max(a.max_rise, rise)
            if a.pushed_t is None and preliminary_set(rise) and not a.overshot:
                a.reached = True
        pushed = handle >= HANDLE_RELEASED
        if pushed and not self.handle_pushed:  # the main load released
            if live and a.pushed_t is None:
                a.pushed_t = self.t
                a.rise_at_push = rise
                a.pushed_ok = (preliminary_set(rise) and not a.overshot and a.fresh and a.seated)
        if live and a.pushed_t is not None and a.removed_t is None:
            if prev is not None and abs(rise - a.rise_at_push) > WHEEL_STILL + 1e-12:
                a.wheel_moved = True
            if a.full_t is None and a.left_t is None and self.t - a.pushed_t >= APPLY_S - 1e-9:
                a.full_t = self.t
            if not pushed and self.handle_pushed and a.left_t is None:
                a.left_t = self.t
            if a.left_t is not None and handle <= HANDLE_HOME:
                a.removed_t = self.t
        self.handle_pushed = pushed
        self.last_rise = rise

    def depth(self) -> float:
        a = self.attempts[-1] if self.attempts else None
        return a.depth(self.t) if a is not None and self.last_rise is not None and self.last_rise > 0.0 else 0.0


def _inside_block(spot) -> bool:
    return float(np.linalg.norm(spot)) <= BLOCK_RADIUS - SPOT_GAP


def long_pointer(rise: float, depth: float) -> float:
    """The long pointer's angle (rad, clockwise as the robot sees the dial, 0 at C), wrapped to (-pi, pi]."""
    if rise <= 0.0:
        return 0.0
    a = 2 * math.pi * (min(rise, TRAVEL_STOP) - depth) / (100 * DIVISION)
    return math.atan2(math.sin(a), math.cos(a))


def short_pointer(rise: float) -> float:
    """The short pointer's angle (rad, clockwise as seen): from the black dot (0) to the red dot (SHORT_RED) over the
    long pointer's three turns."""
    return SHORT_RED * min(max(rise, 0.0), TRAVEL_STOP) / PRE_TRAVEL


SHORT_RED = math.radians(90.0)  # the red dot, a quarter turn round the short dial from the black one [estimate]


def reading(rec: HardnessRecord) -> float | None:
    """The HRC the dial shows after a valid test (None before one): the block's certified value, by construction."""
    a = rec.best()
    return BLOCK_HRC if a is not None and a.stage() >= 0.75 else None


def grade(rec: HardnessRecord, wheel: float, handle: float, seated: bool, hands_clear: bool) -> float:
    """The trial's grade now (0, 0.25, 0.5, 0.75 or 1.0); see the module docstring."""
    best = rec.best()
    g = best.stage() if best is not None else 0.0
    if g == 0.75 and travel(wheel) <= LOWERED and seated and handle <= HANDLE_HOME and hands_clear:
        return 1.0
    return g


def score(rec: HardnessRecord, wheel: float, handle: float, seated: bool, hands_clear: bool) -> float:
    return min(grade(rec, wheel, handle, seated, hands_clear), PARTIAL_CAP)


def success(rec: HardnessRecord, wheel: float, handle: float, seated: bool, hands_clear: bool) -> bool:
    return grade(rec, wheel, handle, seated, hands_clear) == 1.0
