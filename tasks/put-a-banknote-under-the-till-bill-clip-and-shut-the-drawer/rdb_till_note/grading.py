"""Scorer: where the note ended up, whether the hold-down is down on it, and whether the drawer is shut.

NumPy only. Everything is judged on the state at the moment of scoring (the end of the trial, or the moment full
marks have been held for one second); nothing is latched. Tiers:

  0     the note is not wholly inside the drawer, or a gripper is still at it
  0.25  the note is wholly inside the drawer and let go
  0.5   ... and wholly inside the middle note compartment (the one that holds the notes)
  0.75  ... and flat, lying under that compartment's hold-down with the hold-down down on it, the other notes
        still in place under it
  1.0   ... and the drawer is shut, both grippers are clear of it, and all of this has lasted one second
"""

from dataclasses import dataclass

import numpy as np

from . import till_model as m

CONTROL_HZ = 20
HOLD_S = 1.0
HOLD_TICKS = round(HOLD_S * CONTROL_HZ)
LET_GO = .030          # a gripper's centre point this far from every part of the note counts as let go
CLEAR = .030           # ... and this far from the drawer and its case counts as clear
FLAT = .010            # no part of the note higher than this above the notes under it
SIDE_TOL = .001        # the note may touch the compartment's walls
TIP_ABOVE, TIP_BELOW = .004, .002      # the hold-down's free end: at most this far above / into the note's top
STACK_MOVED = .005     # the other notes: moved less than this in the till
STACK_LIFTED = .003
SHUT = .002            # drawer front within this of fully home
STILL_NOTE, STILL_DRAWER = .001, .0005


def corners(centres, rotations):
    """The four mid-plane corners of each strip: (strips, 4, 3)."""
    out = []
    for c, r in zip(centres, rotations):
        a, b = r[:, 0] * m.STRIP_L / 2, r[:, 1] * m.NOTE_W / 2
        out.append([c + a + b, c + a - b, c - a + b, c - a - b])
    return np.array(out)


def box_distance(point, low, high):
    return float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - point, point - np.asarray(high)), 0.)))


class TillNoteGrade:
    def __init__(self):
        self.history = []          # (drawer opening, strip centres in the drawer frame) of the last second
        self.held = 0
        self.best = 0.

    def update(self, *, strips, strip_rot, stack, drawer_open, hold_angles, tcp):
        strips, stack = np.asarray(strips, dtype=float), np.asarray(stack, dtype=float)
        s = float(drawer_open)
        x0 = m.drawer_x(s)
        pts = corners(strips, strip_rot)
        flat_pts = pts.reshape(-1, 3)
        d = flat_pts[:, 0] - x0
        y, z = flat_pts[:, 1], flat_pts[:, 2]
        tcp = [np.asarray(p, dtype=float) for p in tcp]

        let_go = all(np.linalg.norm(flat_pts - p, axis=1).min() >= LET_GO and
                     np.linalg.norm(strips - p, axis=1).min() >= LET_GO for p in tcp)
        in_drawer = bool(np.all((d >= m.PANEL_T) & (d <= m.DRAWER_FLOOR_X[1]) & (np.abs(y) <= m.DRAWER_FLOOR_HALF_Y)
                                & (z <= m.PANEL_Z[1]) & (z >= m.DRAWER_FLOOR_Z[1] - .002)))
        lane = m.compartment_of(float(strips[:, 1].mean()))
        yc = m.COMPARTMENT_Y[m.STOCKED]
        in_compartment = bool(in_drawer and np.all(
            (d >= m.BILL_X[0] - SIDE_TOL) & (d <= m.BILL_X[1] + SIDE_TOL)
            & (np.abs(y - yc) <= m.COMPARTMENT_W / 2 + SIDE_TOL) & (z <= m.RIM_Z)))

        stack_top = float(stack[2] + m.STACK_SIZE[2] / 2)
        stack_shift = float(np.hypot(stack[0] - x0 - m.STACK_X, stack[1] - yc))
        stack_ok = bool(stack_shift <= STACK_MOVED and abs(stack_top - m.STACK_TOP_Z) <= STACK_LIFTED)
        above = float(z.max() + m.NOTE_T / 2 - stack_top)
        flat = bool(above <= FLAT)

        # The hold-down's free end, and the note's top surface directly under it.
        angle = float(hold_angles[m.STOCKED])
        tip = m.tip_underside(angle, s)
        tip_gap, under = None, False
        for c, r in zip(strips, strip_rot):
            local = r.T @ (tip - c)
            if abs(local[0]) <= m.STRIP_L / 2 and abs(local[1]) <= m.NOTE_W / 2 and abs(r[2, 2]) > .5:
                top = c[2] - (r[2, 0] * local[0] + r[2, 1] * local[1]) / r[2, 2] + m.NOTE_T / 2 / abs(r[2, 2])
                gap = float(tip[2] - top)
                if tip_gap is None or abs(gap) < abs(tip_gap):
                    tip_gap = gap
        if tip_gap is not None:
            under = bool(-TIP_BELOW <= tip_gap <= TIP_ABOVE)
        stack_under_tip = bool(abs(tip[0] - stack[0]) <= m.STACK_SIZE[0] / 2 and abs(tip[1] - stack[1]) <= m.STACK_SIZE[1] / 2)
        clipped = bool(in_compartment and flat and under and stack_ok and stack_under_tip)

        shut = bool(s <= SHUT)
        low = (m.FRONT_CLOSED_X - s, -m.PANEL_W / 2, 0.)
        high = (m.CASE_FRONT_X + m.CASE_D, m.PANEL_W / 2, m.CASE_H)
        hands_clear = [box_distance(p, low, high) for p in tcp]
        clear = bool(min(hands_clear) >= CLEAR)

        local = strips - (x0, 0., 0.)
        self.history.append((s, local))
        self.history = self.history[-(HOLD_TICKS + 1):]
        still = bool(len(self.history) > HOLD_TICKS
                     and max(abs(h[0] - s) for h in self.history) <= STILL_DRAWER
                     and max(float(np.abs(h[1] - local).max()) for h in self.history) <= STILL_NOTE)

        score = 0.
        if in_drawer and let_go:
            score = .25
            if in_compartment:
                score = .5
                if clipped:
                    score = .75
        done = bool(score == .75 and shut and clear)
        self.held = self.held + 1 if done and still else 0
        if done and self.held >= 1:
            score = 1.
        self.best = max(self.best, score)
        return {
            "score": score, "note_let_go": bool(let_go), "note_in_drawer": in_drawer, "note_compartment": lane,
            "note_in_middle_compartment": in_compartment, "note_flat": flat, "note_under_hold_down": under,
            "other_notes_in_place": stack_ok, "drawer_shut": shut, "grippers_clear": clear, "still_for_1s": still,
            "note_top_above_stack_mm": round(1000 * above, 2),
            "hold_down_tip_above_note_mm": None if tip_gap is None else round(1000 * tip_gap, 2),
            "hold_down_deg": round(float(np.degrees(angle)), 2), "drawer_open_mm": round(1000 * s, 2),
            "stack_shift_mm": round(1000 * stack_shift, 2),
            "gripper_to_drawer_mm": [round(1000 * v, 1) for v in hands_clear],
            "note_ends_in_drawer_mm": [round(1000 * float(v), 1) for v in (d.min(), d.max())],
            "note_side_mm": [round(1000 * float(v), 1) for v in ((y - yc).min(), (y - yc).max())],
        }


@dataclass(frozen=True)
class TillNoteScorer:
    name: str = "till_note_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["till_note_grade"]
        return Score(value=grade["score"],
                     explanation=f"Note in the drawer: {grade['note_in_drawer']}; in the middle compartment: "
                                 f"{grade['note_in_middle_compartment']}; under the hold-down: "
                                 f"{grade['note_under_hold_down']}; flat: {grade['note_flat']}; drawer shut: "
                                 f"{grade['drawer_shut']}; grippers clear: {grade['grippers_clear']}")


def make_scorer():
    return TillNoteScorer()
