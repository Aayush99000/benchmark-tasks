"""The first Isaac qualification run, as copied into results/qualification-summary.json: the files it ran are the
files here, what task.json and the README say about it is what it measured, and the demonstration used both hands
at once."""

import hashlib
import json
from pathlib import Path

from rdb_till_note import grading, till_model as m

ROOT = Path(__file__).resolve().parents[1]
RUN = json.loads((ROOT / "results/qualification-summary.json").read_text())
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()
DEMO, DREAM = RUN["scripted"], RUN["dream"]


def sha(path):
    return hashlib.sha256((ROOT / "rdb_till_note" / path).read_bytes()).hexdigest()


def test_the_run_scored_these_files():
    for run in (DEMO, DREAM):
        assert run["scene_sha256"] == sha("fixed_scene.py")
        assert run["asset_sha256"] == sha("till.usda")
        assert run["model_sha256"] == sha("till_model.py")
        assert run["grader_sha256"] == sha("grading.py")


def test_scores_in_task_json():
    assert DEMO["score"] == 1. and DEMO["termination_reason"] == "success"
    assert DREAM["score"] == 0. and DREAM["executed_steps"] == DREAM["step_limit"] == 2400
    assert TASK["scores"] == "Dream-1 run: 0.0; demo: 1.0"


def test_readme_reports_what_was_measured():
    assert f"{DEMO['executed_steps']} steps" in README and f"{DEMO['elapsed_s']:.0f} s" in README
    assert f"{DREAM['elapsed_s']:.0f} s" in README and f"{DREAM['action_requests']} requests" in README
    assert f"{DREAM['closest_gripper_to_note_mm']} mm" in README
    assert f"{DREAM['final_grade']['drawer_open_mm']:.0f} mm open" in README
    events = {e["event"]: e for e in DEMO["demo_events"]}
    assert f"{events['hold-down raised']['deg']:.1f} degrees" in README
    assert f"{events['note laid']['far_end_in_drawer_mm']:.1f} mm" in README
    assert f"{DEMO['peaks']['tab_n']:.1f} N" in README and f"{DEMO['peaks']['drawer_n']:.0f} N" in README
    assert "not yet run" not in README.lower() and "pending" not in README.lower()


def test_final_state_of_the_demonstration():
    final = DEMO["final_grade"]
    assert final["note_under_hold_down"] and final["note_flat"] and final["drawer_shut"] and final["grippers_clear"]
    assert abs(final["hold_down_tip_above_note_mm"]) <= 1000 * grading.TIP_ABOVE
    assert final["stack_shift_mm"] < 1. and DEMO["peaks"]["stack_n"] == 0.
    assert DEMO["peak_interarm_contact_n"] == 0.
    near, far = final["note_ends_in_drawer_mm"]
    assert 1000 * m.BILL_X[0] < near and far < 1000 * m.BILL_X[1]


def test_both_hands_worked_at_the_same_time():
    """The left hand held the hold-down up (both tips loaded, arm near 30 degrees) for the whole time the right
    hand held the note and laid it in."""
    first, last = DEMO["both_hands_loaded_ticks"]
    phases = dict((name, tick) for tick, name in DEMO["phases"])
    assert first <= phases["draw note off the case"] and last >= phases["let go of note"] - 5
    assert DEMO["during_lay"]["left_tips_on_tab_n_min"] > 5.
    assert min(DEMO["during_lay"]["hold_down_deg_range"]) > 25.


def test_dream_never_touched_the_note_or_the_hold_down():
    assert DREAM["peaks"]["note_end_n"] == 0. and DREAM["peaks"]["tab_n"] == 0.
    assert DREAM["hold_down_max_deg"] < -20.
    assert not DREAM["final_grade"]["note_in_drawer"]
