"""Score the trial from where the box, its lid, the jester's head and the grippers are. NumPy only.

Every rule can be judged from the trial video, a photo and a ruler (README, "Scoring a real trial"):

- lid shut: the lid's front edge within 3 degrees of shut (7 mm above the rim at the front);
- head below the rim: no part of the jester's head above the top of the box's walls (1 mm allowed);
- jester inside: the head's top at most 4 mm above the rim (under a shut lid) and the head wholly inside the
  walls, so nothing of the jester is caught between the lid and the rim (the sleeve and coil are under the head);
- hands clear: no part of the robot touches the toy and each gripper's centre point is at least 50 mm from
  the box (lid shut, crank included);
- box on its marks: its middle within 15 mm and its turn within 10 degrees of the tape marks, standing upright.

Score: 0 for the rest of the trial once the box has been tipped more than 30 degrees. Otherwise the highest of
1.0 if the lid is shut with the jester inside, the box on its marks and the hands clear, all held for 2 s (the
lid then stays shut only because the catch holds it against the spring); 0.6 if the lid is shut with the
jester inside (at the end); 0.3 if at some moment the head was below the rim while the lid was 30 degrees or
less from shut (latched); else 0.
"""

from dataclasses import dataclass, field

import numpy as np

from . import layout

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
SHUT_DEG = 3.                    # lid within this of shut
CLOSING_DEG = 30.                # "while the lid is closing": within this of shut
RIM_SLACK_M = .001               # the head's top may be this far above the rim and still count as below it
INSIDE_TOP_M = .004              # under a shut lid (3 degrees: 3.5 mm at the head) the head's top may be this high
INSIDE_SLACK_M = .003            # the head's middle may be this much nearer a wall than its radius allows
CLEAR_M = .050                   # gripper centre point to the box
TOUCH_N = .05                    # contact force that counts as touching
MARK_M = .015                    # box middle from its tape marks
MARK_DEG = 10.                   # box turned from its marks
TIPPED_DEG = 30.                 # box knocked over: 0 for the rest of the trial
UPRIGHT_DEG = 10.


def box_frame(box_pos, box_rot):
    """Return a function mapping tabletop points into the box frame (origin on the footprint's middle)."""
    origin, rotation = np.asarray(box_pos, dtype=float), np.asarray(box_rot, dtype=float)
    return lambda point: rotation.T @ (np.asarray(point, dtype=float) - origin)


def head_state(head_centre, box_pos, box_rot):
    """(head top above the rim in the box frame, whether the head is wholly inside the walls)."""
    centre = box_frame(box_pos, box_rot)(head_centre)
    top_above_rim = float(centre[2] + layout.HEAD_R - layout.RIM_Z)
    within = bool(max(abs(centre[0]), abs(centre[1])) <= layout.INNER - layout.HEAD_R + INSIDE_SLACK_M)
    return top_above_rim, within


def clearance(tcp, box_pos, box_rot):
    """Smallest distance from a gripper centre point to the box's outline (lid shut, crank included)."""
    to_box = box_frame(box_pos, box_rot)
    low, high = layout.box_hull((0., 0., 0.))
    points = np.atleast_2d([to_box(p) for p in np.atleast_2d(np.asarray(tcp, dtype=float))])
    return float(np.linalg.norm(np.maximum(np.maximum(low - points, points - high), 0.), axis=1).min())


def tilt_deg(box_rot):
    return float(np.degrees(np.arccos(np.clip(np.asarray(box_rot, dtype=float)[2, 2], -1., 1.))))


def yaw_deg(box_rot):
    r = np.asarray(box_rot, dtype=float)
    return float(np.degrees(np.arctan2(r[1, 0], r[0, 0])))


@dataclass
class JackGrade:
    """Running grade of one trial. Call :meth:`update` once per control step; the last value is the score."""
    hold: int = 0
    tipped: bool = False
    pressed_while_closing: bool = False
    start_xy: tuple = (layout.BOX_X, 0.)
    log: dict = field(default_factory=dict)

    def update(self, lid_deg, head_centre, box_pos, box_rot, tcp, touching):
        """``lid_deg``: the lid's opening angle relative to the box. ``head_centre``: the head's middle.
        ``box_pos``, ``box_rot``: the box's footprint middle and rotation. ``tcp``: both gripper centre
        points. ``touching``: whether any part of the robot touches the box, lid or jester. Tabletop frame."""
        values = [np.asarray(v, dtype=float) for v in (lid_deg, head_centre, box_pos, box_rot, tcp)]
        if not all(np.isfinite(v).all() for v in values):
            self.hold = 0
            return self._grade(0., dict(error="nonfinite state"))
        lid_deg = float(lid_deg)
        tilt = tilt_deg(box_rot)
        self.tipped = self.tipped or tilt > TIPPED_DEG
        top, within = head_state(head_centre, box_pos, box_rot)
        below = top <= RIM_SLACK_M
        inside = top <= INSIDE_TOP_M and within
        self.pressed_while_closing = self.pressed_while_closing or (below and lid_deg <= CLOSING_DEG)
        shut = lid_deg <= SHUT_DEG and inside
        gap = clearance(tcp, box_pos, box_rot)
        hands_clear = bool(gap >= CLEAR_M and not touching)
        shift = float(np.hypot(box_pos[0] - self.start_xy[0], box_pos[1] - self.start_xy[1]))
        on_marks = shift <= MARK_M and abs(yaw_deg(box_rot)) <= MARK_DEG and tilt <= UPRIGHT_DEG
        self.hold = self.hold + 1 if shut and hands_clear and on_marks else 0
        if self.tipped:
            score = 0.
        elif shut and self.hold >= HOLD_TICKS:
            score = 1.
        elif shut:
            score = .6
        elif self.pressed_while_closing:
            score = .3
        else:
            score = 0.
        return self._grade(score, dict(lid_deg=round(lid_deg, 2), head_top_above_rim_mm=round(1000. * top, 1),
                                       head_within_walls=within, jester_inside=inside, lid_shut=shut,
                                       hands_clear=hands_clear, gripper_clearance_mm=round(1000. * gap, 1),
                                       box_shift_mm=round(1000. * shift, 1), box_yaw_deg=round(yaw_deg(box_rot), 2),
                                       box_tilt_deg=round(tilt, 2), on_marks=on_marks))

    def _grade(self, score, detail):
        return {"score": score, "hold_ticks": self.hold, "tipped": self.tipped,
                "pressed_while_closing": self.pressed_while_closing, **detail}


@dataclass(frozen=True)
class JackScorer:
    name: str = "jack_in_box_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["jack_grade"]
        if grade["tipped"]:
            text = "0: the box was knocked over"
        elif grade.get("lid_shut"):
            text = "lid shut on the jester" + ("" if grade["score"] == 1. else
                                               ", but not hands-off on its marks for 2 s (the catch not shown to hold it)")
        elif grade["pressed_while_closing"]:
            text = "the jester was pushed below the rim with the lid nearly shut, but the lid did not end shut over it"
        else:
            text = "the jester was never below the rim with the lid nearly shut"
        return Score(value=grade["score"], explanation=text)


def make_scorer():
    return JackScorer()
