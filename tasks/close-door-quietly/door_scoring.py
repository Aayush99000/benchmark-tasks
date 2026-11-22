"""Scoring for close-door-quietly, as a pure function of logged state (NumPy only; no Isaac, no yam_arena).

The door is a mechanism with three joints, each 0 at rest:
- ``door``: the hinge, in rad, 0 against its stop (shut), positive open (towards the robot: the robot stands on the
  side the door swings into, and closes it by pushing it away);
- ``knob``: the knob's turn, in rad, either way (a passage knob retracts its latch turned in both directions);
- ``bolt``: how far the latch bolt is pushed in, in m, 0 standing fully out (13 mm proud), up to THROW.

The knob drives the bolt in: turned ``KNOB_FULL`` or more it holds the bolt fully in. The bolt can also be pushed in
further than the knob holds it by the strike plate's lip as the door swings shut: that is the click a quiet close
avoids. So a scrape is read from the joints alone: the bolt is in further than the knob accounts for.

Every control step, ``DoorRecord.step`` adds the joints and the grippers' pad points to the trial's record. ``grade``
turns the record and the final state into the score:

- 0: a gripper went to the latch (pushing the bolt in by hand), or the door slammed (latch edge above ``SLAM`` m/s
  when it met the stop), or the door is not shut at the end (latch edge more than ``SHUT`` m off the stop);
- 0.25: shut but not latched (the bolt isn't out in the strike's hole);
- 0.5: latched, but the bolt scraped over the strike (the click);
- 0.75: latched with no scrape, but the bolt snapped out (faster than ``SNAP`` m/s) or the door met the stop faster
  than ``GENTLE`` m/s;
- 1.0: latched, no scrape, no snap, met the stop gently, the knob back at rest, the door still and both grippers at
  least ``CLEAR`` m from the knob. ``score`` never gives more than 0.75: 1.0 only comes from ``success`` holding 1 s.

All thresholds are draft values to be set from the latch prototype and, where marked, from the real door.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# -- the door (spec: docs/robocurve/close-door-quietly-spec.md; [UNVERIFIED] until measured on the real set) ----------
# Hinge axis in the world (m); the door's centre plane at y = -0.75 when shut. The door sits at the jamb's swing side,
# which faces the robot, so the knob stands 57 mm proud of the jamb and the 116 mm jamb runs away from the robot. (Swinging
# away from the robot, the knob sat 95 mm deep in the doorway, 58 mm from the jamb: the gripper couldn't get round it,
# smoke run 2026-11-22.) Opening towards the robot, the slab would meet the table's side edge (y -0.65) past about 7 deg.
HINGE_XY = (-0.25, -0.75)
FLOOR_Z = -0.0125  # the station's floor (yam_arena/station.py @ bd62771)
WIDTH = 0.610  # 24 in door: hinge axis to latch edge (m)
THICK = 0.0349  # 1-3/8 in
BACKSET = 0.0603  # 2-3/8 in: latch edge to the knob's axis
KNOB_H = 0.914  # knob axis above the floor (36 in)
# Kwikset Tylo passage knob (200T): a retailer lists it 2-5/8 in across and 2-19/32 in deep (assembled size, so
# approximate). Modelled as a 66.7 mm round knob 32 mm long, its outer face 66 mm off the door.
KNOB_OUT = 0.050  # knob body's centre from the door's face
KNOB_R = 0.0333  # knob radius
KNOB_LEN = 0.032
THROW = 0.013  # bolt standing proud of the latch edge
KNOB_FULL = math.radians(45)  # knob turn that holds the bolt fully in [estimate]
# The knob's cam: a paddle CAM_R either side of the spindle and CAM_T thick, which turned either way pushes the bolt's
# yoke (CAM_GAP from the spindle's axis at rest) in. Sized so 45 deg holds the bolt flush.
CAM_R = 0.020
CAM_T = 0.004
CAM_GAP = 0.0025

# -- thresholds (draft) ---------------------------------------------------------------------------------------------
DT = 0.05  # control step (20 Hz)
SHUT = 0.003  # latch edge within 3 mm of the stop counts as shut
LATCHED_IN = 0.005  # bolt at most 5 mm in (at least 8 mm out in the strike's hole) counts as latched
SCRAPE = 0.0015  # bolt pushed in 1.5 mm more than the knob holds it: the strike's lip is riding it
# Bolt coming out faster than 30 mm/s (over one step) snaps into the strike: audible. Set from the official sim
# (2026-11-22): the demo's 2 s release peaks at 10 mm/s; turning the knob back in one step (the wrist's fastest)
# peaks at 49 mm/s; letting go of a turned knob is faster still.
SNAP = 0.03
GENTLE = 0.10  # latch edge meeting the stop slower than this (m/s) is quiet [to set]
SLAM = 0.30  # faster than this is a slam [to set]
KNOB_REST = math.radians(5)  # knob within 5 deg of rest
STILL = 0.005  # latch edge moving under 5 mm/s
LATCH_KEEP_OUT = 0.015  # a pad point within 15 mm of the bolt's nose or the strike is a hand at the latch
CLEAR = 0.03  # grippers' pad points at least 3 cm beyond the knob's surface at the end
PARTIAL_CAP = 0.75


def edge_gap(door: float) -> float:
    """How far the latch edge stands off the stop (m), along its arc."""
    return max(0.0, door) * WIDTH


def knob_holds_in(knob: float) -> float:
    """How far in (m) the knob, turned ``knob`` rad either way, holds the bolt: how far the cam's paddle reaches
    past the yoke's face, up to the bolt's throw."""
    a = abs(knob)
    return min(THROW, max(0.0, CAM_R * math.sin(a) + CAM_T / 2 * math.cos(a) - CAM_GAP))


def _door_frame(door: float):
    """Unit vectors along the door (hinge to latch edge) and out of its robot-side face, in the world's xy. Opening
    turns the door about +z, towards the robot (+y)."""
    along = np.array([math.cos(door), math.sin(door), 0.0])
    normal = np.array([-math.sin(door), math.cos(door), 0.0])
    return along, normal


def knob_centre(door: float) -> np.ndarray:
    along, normal = _door_frame(door)
    hinge = np.array([HINGE_XY[0], HINGE_XY[1], FLOOR_Z + KNOB_H])
    return hinge + (WIDTH - BACKSET) * along + (THICK / 2 + KNOB_OUT) * normal


def bolt_nose(door: float) -> np.ndarray:
    """The middle of the latch edge at the bolt (where the bolt comes out), in the world."""
    along, _ = _door_frame(door)
    return np.array([HINGE_XY[0], HINGE_XY[1], FLOOR_Z + KNOB_H]) + WIDTH * along


STRIKE = np.array([HINGE_XY[0] + WIDTH, HINGE_XY[1], FLOOR_Z + KNOB_H])  # the strike's hole, on the jamb


def _points(pads) -> np.ndarray:
    pts = [np.asarray(p, float).reshape(-1, 3) for p in pads if p is not None]
    return np.concatenate(pts) if pts else np.zeros((0, 3))


def hand_at_latch(pads, door: float) -> bool:
    pts = _points(pads)
    if not len(pts):
        return False
    return bool(np.min(np.linalg.norm(pts - bolt_nose(door), axis=1)) < LATCH_KEEP_OUT
                or np.min(np.linalg.norm(pts - STRIKE, axis=1)) < LATCH_KEEP_OUT)


def hands_clear(pads, door: float) -> bool:
    pts = _points(pads)
    return bool(len(pts) == 0 or np.min(np.linalg.norm(pts - knob_centre(door), axis=1)) >= KNOB_R + CLEAR)


@dataclass
class DoorRecord:
    """The trial so far. ``prev`` is the last step's (door, knob, bolt)."""

    steps: int = 0
    prev: tuple | None = None
    scraped: bool = False
    hand_at_latch: bool = False
    snap_speed: float = 0.0  # fastest the bolt came out while the door was shut (m/s)
    close_speed: float = 0.0  # latch edge speed on the step it reached the stop, fastest so far (m/s)
    edge_speed: float = 0.0  # latch edge speed on the latest step (m/s)
    first: dict = field(default_factory=dict)  # step each event first happened, for the README's traces

    def reset(self) -> None:
        self.__init__()

    def step(self, door: float, knob: float, bolt: float, pads=(), dt: float = DT) -> "DoorRecord":
        """Add one control step of ``dt`` s: the three joints (rad, rad, m) and the grippers' pad points (world, m)."""
        if bolt - knob_holds_in(knob) > SCRAPE:
            self._event("scraped")
        if hand_at_latch(pads, door):
            self._event("hand_at_latch")
        if self.prev is not None and dt > 0:
            p_door, _, p_bolt = self.prev
            self.edge_speed = abs(edge_gap(p_door) - edge_gap(door)) / dt
            if edge_gap(p_door) > SHUT >= edge_gap(door):
                self.close_speed = max(self.close_speed, (edge_gap(p_door) - edge_gap(door)) / dt)
            if edge_gap(door) <= SHUT and p_bolt > bolt:
                self.snap_speed = max(self.snap_speed, (p_bolt - bolt) / dt)
        self.prev = (float(door), float(knob), float(bolt))
        self.steps += 1
        return self

    def _event(self, name: str) -> None:
        setattr(self, name, True)
        self.first.setdefault(name, self.steps)


def grade(rec: DoorRecord, door: float, knob: float, bolt: float, pads=()) -> float:
    """The score for the trial ``rec`` with the final joints and pad points (see the module's docstring)."""
    if rec.hand_at_latch or rec.close_speed > SLAM or edge_gap(door) > SHUT:
        return 0.0
    if bolt > LATCHED_IN:
        return 0.25
    if rec.scraped:
        return 0.5
    if rec.snap_speed > SNAP or rec.close_speed > GENTLE:
        return 0.75
    if abs(knob) <= KNOB_REST and rec.edge_speed < STILL and hands_clear(pads, door):
        return 1.0
    return 0.75


def score(rec: DoorRecord, door: float, knob: float, bolt: float, pads=()) -> float:
    return min(grade(rec, door, knob, bolt, pads), PARTIAL_CAP)


def success(rec: DoorRecord, door: float, knob: float, bolt: float, pads=()) -> bool:
    return grade(rec, door, knob, bolt, pads) == 1.0
