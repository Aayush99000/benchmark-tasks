"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

import numpy as np

from rdb_till_note import fixed_scene as fixture
from rdb_till_note import grading, till_model as m

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def test_required_fields_and_no_media():
    for key in ("title", "category", "instruction", "context", "setup", "scoring", "run_limit", "reset_notes",
                "fidelity", "scores", "keywords"):
        assert TASK[key]
    assert "media" not in TASK and TASK["category"] == "Impact on jobs"
    assert TASK["scores"] == "PENDING" or re.fullmatch(r"Dream-1 run: [0-9.]+.*; demo: 1\.0.*", TASK["scores"])


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == fixture.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{fixture.MAX_STEPS:,} steps")
    assert fixture.CONTROL_HZ == grading.CONTROL_HZ == 20
    assert f"{fixture.MAX_STEPS:,} actions" in README


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    w, d, h = (round(1000 * v) for v in (m.CASE_W, m.CASE_D, m.CASE_H))
    assert f"case {w} wide x {d} deep x {h} high" in setup
    assert f"front edge at x {1000 * m.CASE_FRONT_X:.0f}" in setup
    assert f"open by {1000 * m.TRAVEL:.0f}: its front face is at x {1000 * m.FRONT_OPEN_X:.0f}" in setup
    assert f"{1000 * m.TILL_W:.0f} wide x {1000 * m.TILL_D:.0f} deep x {1000 * m.TILL_H:.0f} high, rim {1000 * m.RIM_Z:.0f} above" in setup
    length, width, depth = m.BILL_X[1] - m.BILL_X[0], m.COMPARTMENT_W, m.RIM_Z - m.BILL_FLOOR_Z
    assert f"about {1000 * length:.0f} long x {1000 * width:.0f} wide x {1000 * depth:.0f} deep" in setup
    assert "centres at y " + ", ".join(f"{1000 * y:+.0f}".replace("+0", "0") for y in m.COMPARTMENT_Y[:3]) in setup
    x0 = m.FRONT_OPEN_X
    assert f"middle one spans x {1000 * (x0 + m.BILL_X[0]):.0f} to {1000 * (x0 + m.BILL_X[1]):.0f}" in setup
    assert f"an arm {1000 * m.ARM_L:.0f} long and {1000 * m.ARM_W:.0f} wide" in setup
    assert f"{1000 * m.PIVOT_Z:.0f} above the table" in setup
    assert f"tab {1000 * m.ARM_W:.0f} wide, {1000 * (m.TAB_X[1] - m.TAB_X[0]):.0f} long and {1000 * m.TAB_H:.0f} high" in setup
    assert f"{m.STACK_NOTES} used 1 dollar notes" in setup and f"{1000 * m.STACK_MASS:.0f} g" in setup
    assert f"156 x 66 x {1000 * m.STACK_SIZE[2]:.0f}" in setup
    assert f"centred on y {1000 * m.NOTE_START_Y:.0f}" in setup
    assert f"hanging {1000 * m.OVERHANG:.0f} over the case's front edge (near edge at x {1000 * m.NOTE_START_X[0]:.0f}" in setup
    assert abs(m.NOTE_L - .156) < 1e-3 and abs(m.NOTE_W - .0663) < 1e-4
    assert "about 1.5 N" in setup and abs(m.hold_torque(m.HOLD_ON_STACK) / m.ARM_L - 1.5) < .05
    assert "50 degrees takes about 2 N" in setup
    assert 1.8 < m.hold_torque(m.HOLD_ON_STACK + np.radians(50.)) / m.ARM_L < 2.2
    assert m.STOCKED == 2


def test_scoring_numbers_match_the_code():
    scoring = TASK["scoring"]
    assert f"within {1000 * grading.LET_GO:.0f} mm of it" in scoring
    assert f"more than {1000 * grading.FLAT:.0f} mm above the stack" in scoring
    assert f"pushed more than {1000 * grading.STACK_MOVED:.0f} mm out of place or lifted more than {1000 * grading.STACK_LIFTED:.0f} mm" in scoring
    assert f"within {1000 * grading.TIP_ABOVE:.0f} mm of touching it" in scoring
    assert f"within {1000 * grading.SHUT:.0f} mm of fully home" in scoring
    assert f"at least {1000 * grading.CLEAR:.0f} mm clear" in scoring
    assert f"note {1000 * grading.STILL_NOTE:.0f} mm, drawer {1000 * grading.STILL_DRAWER:g} mm" in scoring
    assert grading.HOLD_S == 1. and "lasted 1 second" in scoring
    assert "Possible scores: 0, 0.25, 0.5, 0.75, 1.0." in scoring


def test_fidelity_confesses_the_model():
    fidelity = TASK["fidelity"]
    assert f"{m.STRIPS} rigid strips".replace("8", "eight") in fidelity
    assert f"{1000 * m.NOTE_T:.0f} mm thick" in fidelity and f"{1000 * m.STRIPS * m.STRIP_MASS:.0f} g" in fidelity
    assert f"{m.SLIDE_FRICTION_N:.0f} N friction" in fidelity and "latch are not modelled" in fidelity
    assert "No software rule" in fidelity


def test_text_does_not_identify_anyone():
    for text in (json.dumps(TASK), README):
        assert not re.search(r"/Users/|/home/|@[a-z]+\.(edu|com)\b", text)
