"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

import numpy as np

from rdb_jack_in_box import fixed_scene as fixture
from rdb_jack_in_box import grading, layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def mm(value):
    return f"{1000 * value:.0f}"


def test_required_fields_and_media_when_uploaded():
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "reset_notes", "fidelity", "scores"):
        assert TASK[key]
    assert TASK["category"] == "Societal relatability"
    if "media" in TASK:
        assert re.fullmatch(r"M-[A-Z0-9]+", TASK["media"])
    assert TASK["scores"].startswith("Dream-1 run: 0.0 (1,600 steps;") and "demo: 1.0 (668 steps)" in TASK["scores"]
    assert "final scene and scorer" in TASK["scores"]
    assert "hand" in TASK["title"] and "lid" in TASK["title"]          # the hand-off is the task


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == fixture.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{fixture.MAX_STEPS:,} steps (80 s")
    assert fixture.CONTROL_HZ == grading.CONTROL_HZ == 20 and fixture.MAX_STEPS == 80 * 20
    assert f"{fixture.MAX_STEPS:,} steps" in README


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    assert "{} x {} x {} high".format(*map(mm, L.BOX_PUBLISHED)) in setup and f"{mm(L.TOY_MASS)} g" in setup
    assert "5.25 x 5.25 x 5.5 in, 1.0 lb" in setup
    assert f"centred at ({mm(L.BOX_X)}, 0)" in setup and f"front face is at x = {mm(L.BOX_X - L.HALF)}" in setup
    hinge = L.hinge_world()
    assert f"(x = {mm(hinge[0])}, z = {mm(hinge[2])})" in setup and f"rim) is {mm(L.RIM_Z)} above" in setup
    edge = L.lid_point(L.LID_OPEN, -L.SIDE)
    assert f"open {np.degrees(L.LID_OPEN):.0f} degrees" in setup and f"({mm(edge[0])}, 0, {mm(edge[2])})" in setup
    assert f"about {mm(2 * L.HEAD_R)} across" in setup and f"head {mm(L.POPPED_TOP)} above the table" in setup
    assert f"about {L.spring_force(L.S_POPPED):.1f} N sprung out and {L.spring_force(L.S_SHUT):.1f} N" in setup
    assert f"up to {np.degrees(L.LEAN_LIMIT):.0f} degrees" in setup
    assert f"about {L.CATCH_RATE * (L.CATCH_PRELOAD + L.CATCH_X[1] - L.BARB_X[0]):.1f} N" in setup


def test_scoring_and_context_numbers_match_the_code():
    scoring = TASK["scoring"]
    assert f"within {grading.SHUT_DEG:.0f} degrees of shut ({L.SIDE * np.sin(np.radians(grading.SHUT_DEG)) * 1000:.0f} mm" in scoring
    assert f"at most {mm(grading.INSIDE_TOP_M)} mm above the rim" in scoring
    assert f"at least {mm(grading.CLEAR_M)} mm from the box" in scoring
    assert f"within {mm(grading.MARK_M)} mm and its turn within {grading.MARK_DEG:.0f} degrees" in scoring
    assert f"more than {grading.TIPPED_DEG:.0f} degrees" in scoring and f"held for {grading.HOLD_S} s" in scoring
    assert f"{grading.CLOSING_DEG:.0f} degrees or less from shut" in scoring
    context = TASK["context"]
    assert len(context.split()) <= 50 and "jack-in-the-box" in context


def test_fidelity_states_the_model():
    fidelity = TASK["fidelity"]
    assert f"{mm(2 * L.HEAD_R)} mm ball" in fidelity and f"{L.SPRING_RATE:.0f} N/m" in fidelity
    assert f"{L.LEAN_RATE:.0f} N m/rad" in fidelity and f"{mm(L.WALL)} mm" in fidelity and f"{mm(L.LID_T)} mm" in fidelity
    assert "no software rule" in fidelity and "without colliders" in fidelity
    assert len(fidelity.split()) <= 105


# Read from the first Isaac runs' RESULT.json (scripted and Dream-1), kept here so text and runs stay in step.
MEASURED = {"demo_steps": 668, "demo_wall_s": 133, "dream_steps": 1600, "dream_wall_s": 290, "dream_requests": 36,
            "lid_end_deg": .47, "hook_on_lid_n": 3.1, "lid_over_hand_deg": 40.9, "lid_at_release_deg": (31.8, 29.6),
            "lean_peak_forward_deg": 14.4, "hook_travel_mm": -4.1, "dream_lid_deg": 76, "dream_box_shift_mm": 14}


def test_measured_numbers_are_reported_as_measured():
    record = json.loads((ROOT / "validation.json").read_text())
    assert record["scripted"] == {"score": 1.0, "steps": 668, "wall_seconds": 132.91}
    assert record["dream_1"]["score"] == 0.0 and record["dream_1"]["steps"] == 1600
    assert record["dream_1"]["wall_seconds"] == 289.5
    assert "132.91 s" in README and "289.50 s" in README
    assert "demo: 1.0 (668 steps)" in TASK["scores"]


def test_text_does_not_identify_anyone():
    for path in [ROOT / "task.json", ROOT / "README.md", *ROOT.glob("rdb_jack_in_box/*.py"), *ROOT.glob("tools/*.py")]:
        text = path.read_text()
        assert not re.search("/" + "(Users|home)/|@[a-z]+\\.(edu|com)\\b", text), path
