"""Dry runs of the scripted demonstration in the kinematic world, scored by the real scorer."""

import numpy as np
import pytest

pytest.importorskip("yam_arena")
pytest.importorskip("scipy")

from dryrun_demo import dry_run  # noqa: E402
from rdb_jack_in_box import fixed_scene as fixture  # noqa: E402
from rdb_jack_in_box import layout as L  # noqa: E402
from rdb_jack_in_box.scripted import HOLD, LID, JackReference, fork_rot, LID_CLEAR  # noqa: E402

PHASES = ["reach", "fork in", "press the jester down", "swing the lid down", "hand-off 1: slide out under the lid",
          "hand-off 2: press the lid's front edge", "fork out", "shut the lid (attempt 1)", "lift off the lid",
          "hands away", "wait"]


def test_demonstration_reaches_full_score_within_the_limit():
    report, reference = dry_run("full")
    assert report["score"] == 1. and report["steps"] < fixture.MAX_STEPS / 2
    assert report["phases"] == PHASES and report["pressed_while_closing"]
    assert report["worst_limit_margin_deg"] > 10. and report["worst_ik_mm"] < .5
    events = {e["event"]: e for e in reference.log["events"]}
    assert events["pressed"]["reached"] and events["pressed"]["head_top_above_rim_mm"] <= -4.
    assert events["left hand out"]["lid_deg"] < 25. and events["shut"]["shut"]


@pytest.mark.parametrize("options", [
    dict(lag_s=.25, hold_offset=(-.004, 0., .005)),
    dict(lag_s=.08, hold_offset=(0., 0., -.005), press_offset=(0., 0., .004)),
    dict(lid_play_deg=8.),
])
def test_demonstration_tolerates_arm_errors_and_fork_play(options):
    report, _ = dry_run("full", **options)
    assert report["score"] == 1.


def test_fork_turns_with_the_lid_inside_the_wrist_limit():
    reference = JackReference()
    reference.state = {"box": np.array((L.BOX_X, 0., 0.))}
    previous = None
    for degrees in np.linspace(105., 15., 19):
        angle = np.radians(degrees)
        q = reference.solve(LID, reference.fork_point(angle), fork_rot(angle),
                            seed=np.asarray((.09, 2.34, 2.26, -1.31, -1.37, -1.12)) if previous is None else previous, iters=300)
        assert reference.worst_ik < 1e-3
        assert abs(q[5]) < np.radians(110.)
        if previous is not None:
            assert np.abs(q - previous).max() < np.radians(25.)      # no flip between neighbouring lid angles
        previous = q
        # the fingers lie in the lid's plane and close across it
        rotation = fork_rot(angle)
        assert abs(rotation[:, 2] @ L.lid_normal(angle)) < 1e-9 and abs(rotation[:, 0] @ L.lid_normal(angle) - 1.) < 1e-9


def test_lid_clearance_table_falls_as_the_hand_slides_out():
    backs, angles = LID_CLEAR
    assert list(backs) == sorted(backs) and list(angles) == sorted(angles, reverse=True)
    assert angles[0] < 45. and angles[-1] <= 20.
    assert HOLD == "left" and LID == "right"
