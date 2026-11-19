"""The task file without a simulator: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q

Needs ``yam_arena`` importable (in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena on
PYTHONPATH). Checks the scene loads, the props' real sizes, every row of the outcome table, each latched event and
loophole, that task.json's numbers match the code, the demo's reach, and a kinematic replay of the demo.
"""

import json
import math
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
import splinter_geometry as G  # noqa: E402
import tiltgrip  # noqa: E402
from benchkit import matrix_quat, tool_state  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

TASK = task_file.load(HERE / "draw_a_splinter_out_with_tweezers.py")
M = TASK.module
HZ = 20.0
SPEC = json.loads((HERE / "task.json").read_text())
README = (HERE / "README.md").read_text()
DOWN = tiltgrip.Grip.deg().rotation  # straight down, the fingers closing along world y


def gripper(tip, half_gap=0.040, rot=DOWN):
    """A gripper frame as finger_frames() gives it, its pad tips' midpoint at ``tip``, each pad ``half_gap`` m out."""
    rot = np.asarray(rot)
    return (np.asarray(tip) - rot[:, 2] * PAD_TIP_PAST_TCP, rot, (half_gap, -half_gap))


AWAY = [gripper((0.20, 0.25, M.TOP + 0.15)), gripper((0.15, -0.25, M.TOP + 0.15))]  # both grippers well clear
HOLD_TRAVEL = 0.0008  # each leg in, held lightly
CLAMP_TRAVEL = (G.TIP_GAP - G.PROXY) / 2  # each leg in, tips shut on the stub


# -- helpers ---------------------------------------------------------------------------------------------------------
def xyzw(r):
    w, x, y, z = matrix_quat(r)
    return (x, y, z, w)


def hand(drawn=0.0, tilt=0.0, sway=0.0, dent=0.0, shift=(0.0, 0.0, 0.0), tip_deg=0.0, moving=False):
    pos = np.array(G.ENTRY) + np.asarray(shift)
    quat = xyzw(G.rot_x(math.radians(tip_deg)))
    return tool_state(M.HAND, pos, quat, lin_vel=(0.05, 0, 0) if moving else (0, 0, 0),
                      hinges={"tilt": tilt, "sway": sway}, sliders={"draw": -drawn, "pad": -dent})


def tweezers_at(origin, turn, travel=0.0, moving=False):
    return tool_state(M.TWEEZERS, origin, xyzw(turn), lin_vel=(0.05, 0, 0) if moving else (0, 0, 0),
                      sliders={"leg_l": -travel, "leg_r": travel})


IN_CUP = tweezers_at(G.TWEEZER_POS, G.TWEEZER_TURN)
USE = M.tool_turn(M.PITCH_USE)


def on_stub(drawn=0.0, travel=CLAMP_TRAVEL, lift=0.0, moving=False):
    """The tweezers along the splinter, their tips on the pad either side of it, moved out with it ``drawn`` m."""
    origin = M.use_origin() + G.OUT * drawn + np.array([0.0, 0.0, lift])
    return tweezers_at(origin, USE, travel, moving)


def step(h, tw, frames=AWAY):
    M._track({"hand": h, "tweezers": tw}, frames=frames)


def fresh():
    M.reset_state(hand())


def pull_to(drawn, end_travel=CLAMP_TRAVEL, steps=12, reset=True):
    """Clamp the stub and draw it out to ``drawn`` m, tracking each step; return the final states. ``reset`` starts a
    new trial first; without it, whatever has latched so far stays."""
    if reset:
        fresh()
    for d in np.linspace(0.0, drawn, steps):
        h, tw = hand(drawn=d), on_stub(d)
        step(h, tw)
    return hand(drawn=drawn), on_stub(drawn, end_travel)


def clear_end(drawn=0.013, reset=False):
    """Draw it out (in the same trial, unless ``reset``) and end with everything still."""
    h, tw = pull_to(drawn, reset=reset)
    tw = on_stub(drawn, lift=0.0)
    step(h, tw)
    return h, tw


# -- the scene and the props -----------------------------------------------------------------------------------------
def test_the_scene_loads():
    assert TASK.scene == "station"
    assert {"hand", "tweezers", "block"} <= set(TASK.names)
    assert TASK.free_objects == ("hand", "tweezers")
    assert M.INSTRUCTION == SPEC["instruction"]


def test_the_props_are_real_size():
    pts = G.tweezer_points()
    assert pts[:, 2].min() == pytest.approx(-G.TW_LONG, abs=0.0005)  # Tweezerman slant: 95 mm
    assert pts[:, 0].max() - pts[:, 0].min() == pytest.approx(G.TW_WIDE, abs=0.0005)  # 10 mm wide
    assert G.SPLINTER_LONG == 0.015 and G.SPLINTER_DIA == 0.0018 and G.BURIED + G.PROUD == G.SPLINTER_LONG
    assert math.degrees(G.ENTRY_ANGLE) == pytest.approx(25.0)
    assert G.CUP_TOP_D == pytest.approx(0.0476, abs=1e-4) and G.CUP_BOTTOM_D == pytest.approx(0.0286, abs=1e-4)
    assert G.CUP_TALL == pytest.approx(0.0349, abs=1e-4)
    long = max(x for _, _, x, _, _, _ in G.FINGERS + (("", 0, G.WRIST[0][1], 0, 0, 0),)) - min(
        x for _, x, _, _, _, _ in G.FINGERS)
    assert 0.9 * 0.21 <= long <= 1.1 * 0.21  # life-size hand, about 21 cm
    wide = G.PALM[1][1] - G.THUMB[1][0]
    assert 0.9 * 0.114 <= wide <= 1.1 * 0.114
    assert G.BLOCK == (0.150, 0.150, 0.020)


def test_the_splinter_starts_11_mm_in_at_25_degrees_heading_away_from_the_robot():
    h = hand()
    outer, inner = M.splinter_ends(h)
    assert inner[0] > outer[0]  # it went in away from the robot
    assert np.linalg.norm(inner - outer) == pytest.approx(G.SPLINTER_LONG)
    assert np.degrees(np.arcsin((outer[2] - inner[2]) / G.SPLINTER_LONG)) == pytest.approx(25.0)
    assert (outer - G.ENTRY) @ G.OUT == pytest.approx(G.PROUD)
    assert (inner - G.ENTRY) @ G.IN == pytest.approx(G.BURIED)
    assert outer[2] - G.ENTRY[2] == pytest.approx(G.PROUD * math.sin(G.ENTRY_ANGLE))


def test_the_tweezers_start_at_rest_in_the_cup():
    lean, pos = G.lean_pose()
    assert math.degrees(lean) == pytest.approx(math.degrees(G.LEAN), abs=0.02)
    assert np.allclose(pos, G.TWEEZER_POS, atol=0.0002)
    pts = G.tweezer_points() @ G.TWEEZER_TURN.T + np.array(G.TWEEZER_POS)
    assert pts[:, 2].min() == pytest.approx(G.TOP + G.CUP_FLOOR, abs=0.0002)  # tips on the floor
    cx, cy = G.CUP_XY
    for lo, hi, radius in G.cup_rings():
        z = pts[:, 2] - G.TOP
        ring = pts[(z >= lo) & (z < hi)]
        if len(ring):
            assert np.hypot(ring[:, 0] - cx, ring[:, 1] - cy).max() <= radius + 1e-5


def test_the_tips_open_6_mm_and_squeeze_shut_on_the_stub():
    assert M.tip_gap(IN_CUP) == pytest.approx(G.TIP_GAP, abs=1e-5)
    held = tweezers_at(G.TWEEZER_POS, G.TWEEZER_TURN, HOLD_TRAVEL)
    assert M.tip_gap(held) == pytest.approx(G.TIP_GAP - 2 * HOLD_TRAVEL, abs=1e-5)
    assert M.tip_gap(held) - G.PROXY >= 0.0015  # held lightly, the open tips pass either side of the stub
    assert CLAMP_TRAVEL < G.CLOSE_TRAVEL  # the slides let the tips meet the stub before their stop


def test_the_grip_holds_the_splinter_against_its_track():
    # fully shut, each pad sits CLAMP_TRAVEL in from the grip section's face; the gripper's 5000 N/m drive pushes past
    # that toward its closed stop, less what the legs' springs take
    pad_force = 5000.0 * (G.GRIP_ACROSS / 2 - CLAMP_TRAVEL)
    tip_force = pad_force - G.LEG_RATE * CLAMP_TRAVEL
    assert tip_force > 2.0
    assert 2 * 0.5 * tip_force > 2 * M.HOLD_FORCE  # steel on wood, friction 0.5: twice the track's hold


def _take_frame():
    lean = math.degrees(G.LEAN)
    grip = tiltgrip.Grip.deg(pitch=lean)
    tcp = np.array(M.tcp_for(M.tool_turn(lean), G.TWEEZER_POS))
    return tcp, grip.rotation  # columns: closing, palm normal, approach


def test_the_fingertips_stay_clear_of_the_cup_when_gripping():
    tcp, rot = _take_frame()
    cx, cy = G.CUP_XY
    outer = G.CUP_TOP_D / 2 + G.SIM_WALL
    # the fingers' last 10 mm: 15.5 mm wide across the palm normal, 4 mm thick, either side of the tweezers
    for along in (PAD_TIP_PAST_TCP - 0.010, PAD_TIP_PAST_TCP):
        for across in (-0.00775, 0.00775):
            for side in (-1, 1):
                p = tcp + rot[:, 2] * along + rot[:, 1] * across + rot[:, 0] * side * (G.GRIP_ACROSS / 2 + 0.002)
                beside = np.hypot(p[0] - cx, p[1] - cy) > outer + 0.002
                assert beside or p[2] > G.TOP + G.CUP_TALL + 0.003, p


def test_the_grip_section_lies_under_the_pads():
    tcp, rot = _take_frame()
    turn = M.tool_turn(math.degrees(G.LEAN))
    for s in (G.GRIP_FROM, G.GRIP_TO):
        for x in (-G.TW_WIDE / 2, G.TW_WIDE / 2):
            p = rot.T @ (np.array(G.TWEEZER_POS) + turn @ np.array([x, 0.0, -s]) - tcp)
            assert PAD_TIP_PAST_TCP - 0.040 <= p[2] <= PAD_TIP_PAST_TCP + 1e-6  # within the pads' last 40 mm
            assert abs(p[1]) <= 0.0137


# -- the outcome table -----------------------------------------------------------------------------------------------
def test_doing_nothing_scores_zero():
    fresh()
    h = hand()
    step(h, IN_CUP)
    assert M.grade(h, IN_CUP) == 0.0


@pytest.mark.parametrize("drawn, expected", [(0.0029, 0.0), (0.003, 0.25), (0.0059, 0.25), (0.006, 0.5),
                                             (0.0109, 0.5)])
def test_partly_drawn_out_scores_by_how_far(drawn, expected):
    h, tw = pull_to(drawn)
    assert M.grade(h, tw) == expected


def test_fully_out_whole_untorn_and_clear_scores_one():
    h, tw = clear_end(reset=True)
    assert M.grade(h, tw) == 1.0
    assert M.success({"hand": h, "tweezers": tw})


def test_fully_out_at_exactly_11_mm():
    h, tw = pull_to(G.BURIED)
    assert M.grade(h, tw) == 1.0


def test_the_tips_must_end_5_mm_off_the_finger():
    h, _ = pull_to(0.012)
    near = tweezers_at(M.use_origin() + np.array([0.0, 0.0, 0.003]), USE)  # tips 3 mm over the pad
    step(h, near)
    assert M.grade(h, near) == 0.75


def test_full_credit_needs_everything_still():
    h, tw = pull_to(0.013)
    assert M.grade(h, on_stub(0.013, moving=True)) == 0.75
    assert M.grade(hand(drawn=0.013, moving=True), tw) == 0.75


# -- latched events --------------------------------------------------------------------------------------------------
def test_a_tear_caps_three_quarters_even_after_the_splinter_settles_back():
    fresh()
    tw = on_stub(0.002)
    torn = hand(drawn=0.002, tilt=0.4)  # levered 23 degrees with the tips holding 6 mm out
    assert M.side_load(torn, tw) > M.F_TEAR
    step(torn, tw)
    h, tw = clear_end()
    assert STATE_flag("torn")
    assert M.grade(h, tw) == 0.75


def STATE_flag(name):  # noqa: N802
    return M.STATE[name]


def test_levering_once_it_is_out_tears_nothing():
    h, tw = pull_to(0.013)
    step(hand(drawn=0.013, tilt=0.5), tw)
    assert not M.STATE["torn"]


def test_a_small_wobble_is_not_a_tear():
    fresh()
    tw = on_stub(0.0)
    h = hand(tilt=0.04)  # 2.3 degrees: well under the tear load
    assert M.side_load(h, tw) < M.F_TEAR
    step(h, tw)
    assert not M.STATE["torn"]


def test_the_side_load_is_the_silicone_torque_over_the_lever_to_the_tips():
    tw = on_stub(0.0)
    h = hand(tilt=0.1)
    assert M.side_load(h, tw) == pytest.approx(M.PLAY_RATE * 0.1 / M.lever(h, tw))
    assert M.LEVER_MIN <= M.lever(h, tw) <= G.PROUD


def test_a_gripper_finger_on_the_splinter_latches_zero():
    fresh()
    h = hand()
    outer, _ = M.splinter_ends(h)
    # open wide, one pad face against the stub's side: the pads' midpoint is 4 cm off, but that finger is on it
    step(h, IN_CUP, frames=[gripper(outer - np.array([0.0, 0.040 - G.PROXY / 2, 0.0])), AWAY[1]])
    h, tw = clear_end()
    assert M.STATE["fingered"]
    assert M.grade(h, tw) == 0.0


def test_a_finger_touching_only_the_stub_near_the_skin_counts():
    fresh()
    h = hand()
    mid = (M.splinter_ends(h)[0] + G.ENTRY) / 2
    step(h, IN_CUP, frames=[gripper(mid - np.array([0.0, 0.010 - G.PROXY / 2 - 0.0004, 0.0]), 0.010), AWAY[1]])
    assert M.STATE["fingered"]


def test_a_gripper_finger_1_mm_off_the_splinter_is_not_on_it():
    fresh()
    h = hand()
    outer, _ = M.splinter_ends(h)
    step(h, IN_CUP, frames=[gripper(outer - np.array([0.0, 0.040 - G.PROXY / 2 - 0.001, 0.0])), AWAY[1]])
    assert not M.STATE["fingered"]


def test_closed_fingers_just_over_the_stubs_end_are_not_on_it():
    fresh()
    h = hand()
    outer, _ = M.splinter_ends(h)
    shut = gripper(outer + np.array([0.0, 0.0, 0.003]), 0.0)  # pads shut, their tips 3 mm over its end
    assert M.finger_gap(shut, M.exposed(h)) == pytest.approx(0.003 - 0.0, abs=1e-9)
    step(h, IN_CUP, frames=[shut, AWAY[1]])
    assert not M.STATE["fingered"]


def test_the_buried_part_is_not_counted():
    h = hand()
    pts = M.exposed(h)
    assert len(pts) == round(G.PROUD / 0.0005) + 1
    assert np.allclose(pts[-1], G.ENTRY, atol=1e-9)
    out = M.exposed(hand(drawn=0.013))
    assert np.linalg.norm(out[-1] - out[0]) == pytest.approx(G.SPLINTER_LONG)  # all of it, once out


def test_pushing_it_in_2_mm_latches_zero_even_if_it_is_then_drawn_out():
    fresh()
    step(hand(drawn=-0.002), on_stub(-0.002))
    h, tw = clear_end()
    assert M.STATE["pushed"]
    assert M.grade(h, tw) == 0.0


def test_a_slight_push_in_is_no_fault():
    fresh()
    step(hand(drawn=-0.0015), on_stub(-0.0015))
    h, tw = clear_end()
    assert not M.STATE["pushed"]


def test_squeezed_tips_denting_the_pad_over_1_mm_pinch_the_skin():
    fresh()
    h = hand(dent=0.0012)
    tw = tweezers_at(M.use_origin() - np.array([0.0, 0.0, 0.0012]), USE, CLAMP_TRAVEL)
    step(h, tw)
    assert M.STATE["pinched"]
    h, tw = clear_end()
    assert M.grade(h, tw) == 0.0


def test_open_tips_resting_on_the_pad_do_not_pinch():
    fresh()
    tw = tweezers_at(M.use_origin() - np.array([0.0, 0.0, 0.0012]), USE, 0.0)
    step(hand(dent=0.0012), tw)
    assert not M.STATE["pinched"]


def test_squeezed_tips_resting_lightly_do_not_pinch():
    fresh()
    tw = tweezers_at(M.use_origin() - np.array([0.0, 0.0, 0.0005]), USE, CLAMP_TRAVEL)
    step(hand(dent=0.0005), tw)
    assert not M.STATE["pinched"]


def test_the_hand_moved_over_5_mm_caps_three_quarters():
    fresh()
    step(hand(shift=(0.006, 0.0, 0.0)), IN_CUP)
    h, tw = clear_end()
    assert M.grade(h, tw) == 0.75


def test_the_hand_dragged_2_cm_or_tipped_latches_zero():
    for kwargs in ({"shift": (0.0, 0.021, 0.0)}, {"tip_deg": 11.0}):
        fresh()
        step(hand(**kwargs), IN_CUP)
        h, tw = clear_end()
        assert M.grade(h, tw) == 0.0, kwargs


# -- milestones and loopholes ----------------------------------------------------------------------------------------
def test_drawn_out_then_pushed_back_loses_the_credit():
    h, tw = pull_to(0.007)
    assert M.grade(h, tw) == 0.5
    step(hand(drawn=0.001), on_stub(0.001))
    assert M.grade(hand(drawn=0.001), on_stub(0.001)) == 0.0


def test_pulling_straight_up_tears_the_entry():
    # pulling up levers the stub about the entry: the silicone gives, and the load across it passes the tear load
    fresh()
    tw = on_stub(0.001)
    up = hand(drawn=0.001, tilt=-0.35)
    step(up, tw)
    assert M.STATE["torn"]
    h, tw = clear_end()
    assert M.grade(h, tw) == 0.75


def test_levering_it_sideways_out_scores_at_most_three_quarters():
    fresh()
    for d in np.linspace(0.0, 0.012, 8):
        step(hand(drawn=d, sway=0.35), on_stub(d))
    h, tw = clear_end(0.012)
    assert M.grade(h, tw) == 0.75


def test_grabbing_the_stub_with_the_fingers_and_pulling_scores_zero():
    fresh()
    for d in np.linspace(0.0, 0.013, 8):
        h = hand(drawn=d)
        outer, _ = M.splinter_ends(h)
        step(h, IN_CUP, frames=[gripper(outer + G.IN * 0.002, G.PROXY / 2), AWAY[1]])  # pads shut on the stub
    assert M.grade(hand(drawn=0.013), IN_CUP) == 0.0


def test_dragging_the_whole_hand_toward_the_robot_scores_zero():
    fresh()
    for dx in np.linspace(0.0, -0.025, 6):
        step(hand(shift=(dx, 0.0, 0.0)), on_stub(0.0))
    assert M.grade(hand(shift=(-0.025, 0, 0)), on_stub(0.0)) == 0.0


def test_the_score_does_not_depend_on_where_the_tweezers_end_up_once_clear():
    h, _ = pull_to(0.013)
    for tw in (IN_CUP, on_stub(0.013, lift=0.03), tweezers_at((0.15, -0.15, G.TOP + 0.10), USE)):
        assert M.grade(h, tw) == 1.0


# -- real-bench readings ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize("before, after, torn", [(0.0015, 0.0025, False), (0.0015, 0.0026, True), (0.0020, 0.0030, False),
                                                 (0.0005, 0.0016, True), (0.0018, 0.0018, False)])
def test_a_tear_is_growth_over_1_mm_against_the_trials_own_start(before, after, torn):
    assert M.torn_on_bench(before, after) is torn


def test_full_credit_does_not_turn_on_track_wear():
    # a worn track at the reset limit and a fresh one: the same clean pull (slit unchanged, +0.5 mm) is untorn on both
    for before in (0.0012, M.SLIT_RESET_MAX):
        assert not M.torn_on_bench(before, before + 0.0005)
    assert M.torn_on_bench(M.SLIT_RESET_MAX, M.SLIT_RESET_MAX + 0.0011)


@pytest.mark.parametrize("piece, standing, expected", [
    (0.0, 0.0, 0.5),  # snapped, and nothing left in: capped
    (0.011, 0.0, 0.0),  # the whole buried part left in, nothing drawn
    (0.008, 0.0, 0.25),  # 3 mm of it drawn before it snapped flush
    (0.009, 0.004, 0.5),  # 6 mm drawn, 4 mm of the piece still out
    (0.003, 0.002, 0.5),  # 10 mm drawn: still capped at 0.5
    (0.007, 0.0, 0.25),  # 4 mm drawn
])
def test_a_snapped_splinter_scores_its_remaining_pieces_draw_at_most_half(piece, standing, expected):
    assert M.snapped_score(piece, standing) == expected


# -- the drawn scene: rounded hand, round splinter, the mm label --------------------------------------------------------
def _local_corners(spec):
    from benchkit import turn_matrix
    half = np.array(spec[1]) / 2
    k = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * half
    return np.asarray(spec[0]) + k @ turn_matrix(spec[3] if len(spec) > 3 else 0.0).T


def test_the_hand_is_drawn_rounded_inside_its_colliders():
    boxes = M._hand_boxes()
    colliders = [b for b in boxes if len(b) <= 6 or not b[6]]
    looks = [b for b in boxes if len(b) > 6 and b[6] and "_look" in b[4]]
    assert colliders and all(b[5] == 0.0 for b in colliders)  # the plain boxes collide but aren't drawn
    lo = np.min([_local_corners(b).min(axis=0) for b in colliders], axis=0)
    hi = np.max([_local_corners(b).max(axis=0) for b in colliders], axis=0)
    for b in looks:
        c = _local_corners(b)
        assert (c >= lo - 1e-9).all() and (c <= hi + 1e-9).all(), b[4]
    tips = [b for b in looks if b[4].startswith("middle_look")]
    assert len({round(b[0][0] - b[1][0] / 2, 5) for b in tips}) == 5  # the fingertip steps back: rounded in plan


def test_the_splinter_is_drawn_round_and_dark():
    drawn_boxes = [b for b in M.DRAW.boxes if b[4].startswith("splinter")]
    assert len(drawn_boxes) == 2 and all(b[1] == (G.SPLINTER_LONG, G.SPLINTER_DIA, G.SPLINTER_DIA) for b in drawn_boxes)
    assert max(M.STAINED) < 0.5 and min(M.SKIN) > 0.6  # dark wood on pale skin
    assert "stained dark brown" in SPEC["setup"]


def test_the_scale_label_sits_on_the_fingertips_side():
    label = next(b for b in M._hand_boxes() if b[4] == "scale_label")
    assert label[1][0] == pytest.approx(0.025) and label[1][2] == pytest.approx(0.007)
    assert label[0][1] < -G.FINGERS[0][4] / 2  # on the right (-y) side face
    assert label[0][2] + label[1][2] / 2 == pytest.approx(-0.005)  # top edge 5 mm under the pad
    ticks = [b for b in M._hand_boxes() if b[4].startswith("scale_tick")]
    assert len(ticks) == 26


def test_the_side_camera_stands_off_the_table():
    from yam_arena import station as S
    assert -0.70 < S.TABLE_CENTER[1] - S.TABLE_SIZE[1] / 2  # the table's right edge is inside y -70 cm


# -- the text against the code ---------------------------------------------------------------------------------------
def test_task_json_has_every_field():
    for k in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
              "reset_notes", "fidelity", "scores", "media"):
        assert SPEC.get(k), k
    assert SPEC["category"] == "Societal relevance"
    assert SPEC["scores"].startswith("Dream-1 run: ") and SPEC["scores"].endswith("demo: 1.0")  # as the runs printed
    assert SPEC["media"] == "pending" or re.fullmatch(r"M-\w+", SPEC["media"])


def _mm(v):
    return f"{v * 1000:g} mm"


def test_text_matches_code():
    setup, scoring = SPEC["setup"], SPEC["scoring"]
    for text, value in [
        (setup, "95 mm long"), (setup, f"{G.TW_WIDE * 1000:g} mm wide"), (setup, "25-degree slant"),
        (setup, f"open {G.TIP_GAP * 1000:g} mm"), (setup, f"{G.SPLINTER_LONG * 1000:g} mm"),
        (setup, f"{G.SPLINTER_DIA * 1000:g} mm"), (setup, f"{G.BURIED * 1000:g} mm under the skin"),
        (setup, f"{G.PROUD * 1000:g} mm standing out"), (setup, "25 degrees"),
        (setup, f"x {G.ENTRY[0] * 100:g}, y {G.ENTRY[1] * 100:g}"),
        (setup, f"{(G.ENTRY[2] - G.TOP) * 100:g} cm above the table"),
        (setup, f"x {G.CUP_XY[0] * 100:g}, y {G.CUP_XY[1] * 100:g}"),
        (setup, "47.6 mm"), (setup, "28.6 mm"), (setup, "34.9 mm"),
        (setup, f"about {round(math.degrees(G.LEAN))} degrees"),
        (setup, "15 x 15 x 2 cm"), (setup, f"x {G.BLOCK_POS[0] * 100:g}, y {G.BLOCK_POS[1] * 100:g}"),
        (scoring, f"{M.DRAWN_QUARTER * 1000:g} mm"), (scoring, f"{M.DRAWN_HALF * 1000:g} mm"),
        (scoring, f"{M.DRAWN_OUT * 1000:g} mm"), (scoring, f"{M.PUSHED_IN * 1000:g} mm"),
        (scoring, f"grew by more than {M.SLIT_GROWTH * 1000:g} mm"), (scoring, f"{M.FINGER_MOVED * 1000:g} mm"),
        (SPEC["reset_notes"], f"over {M.SLIT_RESET_MAX * 1000:g} mm"), (scoring, f"at most {M.SNAPPED_CAP:g}"),
        (scoring, f"within {M.TOUCH * 1000:g} mm"), (scoring, f"its last {M.FINGER_LONG * 1000:g} mm"),
        (scoring, f"{M.FINGER_WIDE * 1000:g} mm wide, {M.FINGER_THICK * 1000:g} mm thick"),
        (SPEC["fidelity"], f"{M.F_TEAR:g} N"), (SPEC["fidelity"], f"{G.SPLINTER_DIA * 1000:g} mm round"),
        (setup, f"{(M.LABEL_X[1] - M.LABEL_X[0]) * 1000:g} x 7 mm"), (setup, f"{-M.LABEL_X[0] * 1000:g} mm on the fingertip"),
        (setup, f"{M.LABEL_X[1] * 1000:g} mm toward the palm"), (setup, "top edge 5 mm under"),
        (setup, "x 27, y -70"),
        (scoring, f"{M.HAND_OFF * 100:g} cm"), (scoring, f"{M.HAND_TIP_DEG:g} degrees"),
        (scoring, f"{M.PINCH_DENT * 1000:g} mm"), (scoring, f"{M.CLEAR_OF_SKIN * 1000:g} mm"),
        (scoring, f"{M.F_TEAR:g} N"), (scoring, f"{M.SUCCESS_HOLD_S:g} s"),
    ]:
        assert value in text, value
    for rung in ("0.25", "0.5", "0.75", "1.0"):
        assert rung in scoring
    assert str(M.HOLD_FORCE) in SPEC["fidelity"] and str(int(M.PAD_RATE / 1000)) + " N per mm" in SPEC["fidelity"]


def test_the_run_limit_fits_the_demo():
    steps = int(re.match(r"([\d,]+) steps", SPEC["run_limit"]).group(1).replace(",", ""))
    demo_steps = round(sum(w.duration for w in M.demo()["right"]) * HZ) + 60  # plus yam_scripted's 3 s hold
    assert demo_steps <= steps <= 20 * demo_steps
    assert f"{steps / HZ:g} s" in SPEC["run_limit"] or f"{steps / HZ / 60:g} minute" in SPEC["run_limit"]


def test_the_text_names_no_other_task_and_no_one():
    blob = json.dumps(SPEC) + README
    assert not re.search(r"#\d{2,}|PR \d|pull request \d", blob)
    for word in ("jasontang", "Robocurve", "/Users/", "team "):
        assert word not in json.dumps(SPEC), word


def test_the_code_excerpt_fits_and_shows_the_scorer_first():
    main = (HERE / "draw_a_splinter_out_with_tweezers.py").read_text()
    assert len(main) + len(README) < 40000
    assert main.index("def grade(") < main.index("HAND = tool(")


def test_the_readme_lists_assets_and_safety():
    assert "No third-party assets" in README
    assert "safe" in README.lower()


# -- the demo --------------------------------------------------------------------------------------------------------
def test_every_demo_waypoint_is_reachable():
    mm, deg, _ = tiltgrip.ik_check(M.demo()["right"], station.BASES["right"], HZ)
    assert mm < 2.0 and deg < 2.0


def test_the_demo_ends_holding_still_for_two_seconds():
    plan = M.demo()["right"]
    last = plan[-1]
    assert last.duration >= 2.0 and last.pos == plan[-2].pos and last.finger == plan[-2].finger


def test_the_open_tips_straddle_the_stub_when_set_down():
    tw = on_stub(0.0, travel=HOLD_TRAVEL)
    h = hand()
    outer, _ = M.splinter_ends(h)
    local = [tw.to_local(p) for p in (outer, G.ENTRY)]
    gap_half = M.tip_gap(tw) / 2
    for p in local:
        assert abs(p[1]) + G.PROXY / 2 < gap_half  # the stub sits between the tips, clear of both
    assert np.min(M.over_index(h, M.jaws_world(on_stub(0.0)))) == pytest.approx(0.0, abs=1e-6)  # tips on the skin


def _replay():
    """Play the demo kinematically: the tweezers ride rigidly in the grip from 'grip the tweezers' on; the legs sit
    at the light hold until the fingers close past it, then shut on the stub; once shut, the splinter moves out with the
    tips along its track (to its 16 mm stop). The pad dents under the tips as a 1 N/mm spring in series with the arm's
    0.23 N/mm along the push, so only that share of a set-down below the skin becomes dent."""
    plan_w = M.demo()["right"]
    plan = ArmPlan(plan_w, station.BASES["right"])
    arm_k = 230.0
    share = arm_k / (arm_k + G.PAD_RATE)
    t_grip = sum(w.duration for w in plan_w[: [w.label for w in plan_w].index("grip the tweezers") + 1])
    rel = None
    clamp_at = None
    fresh()
    scores = []
    n = round((plan.duration + 3.0) * HZ)
    prev = None
    for k in range(n):
        t = (k + 1) / HZ
        pos, rot, finger, _ = plan.target(t)
        if t >= t_grip and rel is None:
            g0 = np.eye(4)
            g0[:3, :3], g0[:3, 3] = rot, pos
            tw0 = np.eye(4)
            tw0[:3, :3], tw0[:3, 3] = G.TWEEZER_TURN, G.TWEEZER_POS
            rel = np.linalg.inv(g0) @ tw0
        if rel is None:
            pose = np.eye(4)
            pose[:3, :3], pose[:3, 3] = G.TWEEZER_TURN, G.TWEEZER_POS
            travel = 0.0
        else:
            g = np.eye(4)
            g[:3, :3], g[:3, 3] = rot, pos
            pose = g @ rel
            travel = CLAMP_TRAVEL if finger > -M.HOLD_WIDTH + 0.0005 else HOLD_TRAVEL
        tw = tweezers_at(pose[:3, 3], pose[:3, :3], travel)
        dent = 0.0
        low = np.min(M.over_index(hand(), M.jaws_world(tw)))
        if low < 0:  # the tips are sent below the skin: the arm gives, the pad takes its share
            dent = -low * share
            pose[2, 3] += -low - dent
            tw = tweezers_at(pose[:3, 3], pose[:3, :3], travel)
        drawn = 0.0
        if travel == CLAMP_TRAVEL:
            mid = M.jaws_world(tw).mean(axis=0)
            if clamp_at is None:
                clamp_at = mid
            drawn = float(np.clip((mid - clamp_at) @ G.OUT, 0.0, M.TRACK_END))
        moving = prev is not None and np.linalg.norm(pose[:3, 3] - prev) * HZ > 0.005
        prev = pose[:3, 3].copy()
        tw = tweezers_at(pose[:3, 3], pose[:3, :3], travel, moving)
        h = hand(drawn=drawn, dent=dent)
        step(h, tw, frames=[AWAY[0], (pos, rot, (abs(finger), -abs(finger)))])
        scores.append(M.grade(h, tw))
    return scores, h, tw


def test_a_kinematic_replay_of_the_demo_scores_one():
    scores, h, tw = _replay()
    assert scores[-1] == 1.0
    assert not any(M.STATE[k] for k in ("fingered", "pushed", "pinched", "off", "torn"))
    assert M.drawn(h) >= G.BURIED


def test_the_replay_climbs_the_ladder_in_order():
    scores, _, _ = _replay()
    firsts = [next(i for i, s in enumerate(scores) if s >= rung) for rung in (0.25, 0.5, 0.75)]
    assert firsts == sorted(firsts)
    assert all(b >= a for a, b in zip(scores, scores[1:]) if a < 0.75)


def test_the_gripper_never_comes_near_the_stub():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    outer, _ = M.splinter_ends(hand())
    for k in range(round(plan.duration * HZ)):
        pos, rot, _, _ = plan.target((k + 1) / HZ)
        tip = pos + rot[:, 2] * PAD_TIP_PAST_TCP
        assert np.linalg.norm(tip - outer) > 0.03


def test_the_fingers_open_fully_before_the_grip():
    plan = M.demo()["right"]
    assert plan[0].finger == FINGER_OPEN and plan[1].finger == FINGER_OPEN
