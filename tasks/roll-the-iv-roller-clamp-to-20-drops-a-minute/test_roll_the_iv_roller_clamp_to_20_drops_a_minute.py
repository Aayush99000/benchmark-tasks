"""The task file without a simulator: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q

Needs ``yam_arena`` importable (put a clone of yam-isaac-lab-arena on PYTHONPATH). Checks the props against the real
items, the drip-rate model, every outcome row and loophole, each latch and milestone, the release test, the text
against the code, and the demo: reach, arm clearance, the run limit and a kinematic replay the scorer marks 1.0.
"""

import json
import math
import pathlib
import re
import sys
from types import SimpleNamespace

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from benchkit import quat_xyzw, rpy_matrix, tool_state  # noqa: E402
from yam_arena import analysis, station, task_file  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN, PAD_TIP_PAST_TCP, READY  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

TASK = task_file.load(HERE / "roll_the_iv_roller_clamp_to_20_drops_a_minute.py")
M = TASK.module
D = M  # the demo's plan lives in the task file
import clamp_demo as CD  # noqa: E402  (its waypoints)

HZ = 20.0
SPEC = json.loads((HERE / "task.json").read_text())
README = (HERE / "README.md").read_text()
START_Q = quat_xyzw(rpy_matrix((0.0, 0.0, M.CLAMP_YAW)))


# -- building states by hand ------------------------------------------------------------------------------------------


def clamp(travel=0.0, shift=(0.0, 0.0, 0.0), yaw=0.0):
    """The clamp where it starts (moved ``shift`` m, turned ``yaw`` rad more), wheel ``travel`` m from shut."""
    q = quat_xyzw(rpy_matrix((0.0, 0.0, M.CLAMP_YAW + yaw)))
    return tool_state(M.CLAMP, np.add(M.CLAMP.pos, shift), q, sliders={"wheel": travel})


def cube_hull(centre, half=0.01):
    c = np.asarray(centre, dtype=float)
    normals = np.vstack([np.eye(3), -np.eye(3)])
    d = normals @ c + half
    return SimpleNamespace(planes=np.c_[normals, d])


FAR = [[("gripper", cube_hull((0.10, 0.40, 1.10)))], [("gripper", cube_hull((0.10, -0.40, 1.10)))]]


def on_clamp():
    """A gripper (2 cm cube) sitting on the housing's far end, the other far away."""
    p = clamp().to_world((M.LENGTH / 2 - 0.016, 0.0, 0.02))
    return [[("gripper", cube_hull(p))], FAR[1]]


# both arms back in their start pose: the grippers' hulls there and where they are
HOME = {"grips": [M.gripper(READY, FINGER_OPEN, station.BASES[a])[0] for a in ("left", "right")],
        "at": [M.START["left"], M.START["right"]]}


def home_but(shift_left=(0.0, 0.0, 0.0)):
    """Both arms at their start pose, the left gripper's position moved ``shift_left`` m."""
    return {"grips": HOME["grips"], "at": [M.START["left"] + np.asarray(shift_left), M.START["right"]]}


def near(gap):
    """A gripper whose nearest face is ``gap`` m above the top of the wheel."""
    top = clamp().to_world((M.WHEEL_X0, 0.0, M.WHEEL_Z + M.WHEEL_D / 2))
    return [[("gripper", cube_hull(top + (0.0, 0.0, gap + 0.01)))], FAR[1]]


def run(*phases):
    """Feed (travel, seconds, grips[, shift]) phases through the step hook from a fresh reset; return the score at the
    last step, as at the run limit. ``grips`` is a list of both grippers' hulls, or {"grips": hulls, "at": positions}."""
    M._reset({})
    st = clamp()
    for ph in phases:
        travel, seconds, grips = ph[:3]
        grips, at = (grips["grips"], grips["at"]) if isinstance(grips, dict) else (grips, None)
        shift = ph[3] if len(ph) > 3 else (0.0, 0.0, 0.0)
        st = clamp(travel, shift)
        for _ in range(int(round(seconds * HZ))):
            M._watch({"roller_clamp": st}, grips, at)
    return M.score({"roller_clamp": st})


def at(rate):
    return M.travel_for(rate)


def released(rate, held_s=10.0, clear_s=10.0):
    """Set while holding, then let go and stay clear to the run limit."""
    return run((at(rate), held_s, on_clamp()), (at(rate), clear_s, FAR))


def finished(rate, held_s=10.0):
    """Set while holding, then let go and return both arms to their start pose."""
    return run((at(rate), held_s, on_clamp()), (at(rate), 1.0, HOME))


# -- the scene and the props ------------------------------------------------------------------------------------------


def test_the_scene_loads():
    assert TASK.scene == "station"
    assert TASK.free_objects == ("roller_clamp",)
    assert M.INSTRUCTION == SPEC["instruction"]
    for name in ("saline_bag", "drip_chamber", "pole", "beaker_floor", "tape_bag_side", "tape_beaker_side",
                 "slide_clamp", "drop", "drop_fall", "stream"):
        assert name in TASK.names


# (sim, real): every number from the catalogue page or the item as the setup states it
SIZES = [
    (M.TUBE_OD, 0.0041),  # standard-bore IV tubing
    (M.BAG_SIZE[2], 0.28),  # 1,000 mL bag, filled
    (M.HEAD, 1.0),  # the chamber's saline about 1 m above the line's end, as a bag hangs about 90-100 cm over the site
    (M.DROP_D, 0.0058),  # a 0.1 mL drop (10 drops a mL) is 5.8 mm across
    (M.BEAKER_D, 0.134),
    (M.BEAKER_H, 0.185),
    (M.LENGTH, 0.064),
    (M.WIDTH, 0.020),
    (M.WHEEL_D, 0.014),
]


@pytest.mark.parametrize("sim,real", SIZES)
def test_every_prop_is_full_size(sim, real):
    assert 0.9 <= sim / real <= 1.1


def test_the_pole_is_set_inside_its_height_range():
    hook = M.TOP + M.HOOK_Z - M.FLOOR
    assert 1.02 <= hook <= 2.08  # Drive 13033: 40-82 in


def test_the_masses_are_not_lighter_than_the_real_items():
    assert M.CLAMP_MASS >= 0.008  # a roller clamp and 8 cm of filled tubing
    assert M.WHEEL_MASS >= 0.002


def test_the_wheel_has_no_spring_and_rolls_its_whole_travel():
    wheel = M.CLAMP.sliders[0]
    assert wheel.rate == 0.0 and wheel.damping == M.WHEEL_DAMPING
    assert (wheel.lower, wheel.upper) == (0.0, M.TRAVEL)
    assert M.WHEEL_X0 + M.TRAVEL + M.WHEEL_D / 2 < M.LENGTH / 2  # the wheel stays inside the housing


def test_the_wheel_holds_until_pushed_past_its_breakaway():
    """The wheel has breakaway friction (N) on its joint, handed to the simulator's actuator, not only viscous drag."""
    import inspect

    import benchkit

    wheel = M.CLAMP.sliders[0]
    assert wheel.friction == M.WHEEL_BREAKAWAY and 1.0 <= M.WHEEL_BREAKAWAY <= 20.0  # a thumb manages it: under 20 N
    assert "friction=s.friction" in inspect.getsource(benchkit._arena_object)
    # Isaac Lab 3.0 takes `friction` as the static effort only: the sliding friction is set equal, as physcheck has it
    assert '{"dynamic_friction": s.friction}' in inspect.getsource(benchkit._arena_object)
    assert M.WHEEL_DAMPING == 0.0  # the breakaway alone holds it where it is left
    assert M.REQUIRED_FORCES["roll the wheel"] == pytest.approx(M.WHEEL_BREAKAWAY * math.sqrt(2), abs=0.01)


def test_the_fingertip_end_fits_between_the_walls():
    inner = M.WIDTH - 2 * M.WALL_T
    assert inner / 2 - 0.004 >= 0.002  # the closed tip's last 6 mm is 8 mm across: 2 mm either side
    assert M.WHEEL_Z + M.FACET - 0.0077 * math.cos(math.pi / 4) > M.BASE_T + 0.002


def test_the_tubing_is_taped_where_the_free_tubing_ends():
    for name, p in M.TAPED.items():
        assert np.allclose(clamp().point(name), p)
    assert math.dist(M.TAPED["bag_side"], M.TAPED["beaker_side"]) == pytest.approx(M.LENGTH + 2 * M.STUB)


def test_the_line_runs_from_the_chamber_to_the_beaker():
    up = [o for o in M.OBJECTS if o.name.startswith("line_")]
    assert len(up) >= 6
    assert math.dist(M.UP_ROUTE[0], M.TAPED["bag_side"][:2]) == pytest.approx(M.TAPE_GAP)
    assert math.dist(M.DOWN_ROUTE[0], M.TAPED["beaker_side"][:2]) == pytest.approx(M.TAPE_GAP)
    end = next(o for o in M.OBJECTS if o.name == "line_beaker_in")
    assert end.pos[2] - end.size[2] / 2 == pytest.approx(M.FLOOR + 0.003 + 0.060)  # 6 cm above the beaker's floor
    assert M.SLIDE_Z == pytest.approx(M.CHAMBER_TOP - M.CHAMBER_LEN - 0.12)  # 12 cm below the chamber


def test_the_line_fits_the_sets_three_metres():
    runs = [M.UP_ROUTE, M.DOWN_ROUTE]
    flat = sum(math.dist(a, b) for r in runs for a, b in zip(r, r[1:]))
    hanging = (M.CHAMBER_TOP - M.CHAMBER_LEN) + (M.TOP - M.FLOOR - M.BEAKER_H) + M.BEAKER_H
    assert flat + hanging + M.LENGTH + 2 * M.STUB < 3.0


def test_the_line_is_taped_flat_at_each_bend_and_at_most_ten_cm_apart():
    """Along the table, from the strip by the clamp to the foot of the hanging tubing and to the table's edge."""
    for tag, route in (("up", M.UP_ROUTE), ("down", M.DOWN_ROUTE)):
        names = [n for n, _ in M.TAPE_STRIPS if n.startswith(f"tape_{tag}_")] + (["tape_edge"] if tag == "down" else [])
        spots = [M._along(route, M.FIRST_STRIP)[:2]] + [dict(M.TAPE_STRIPS)[n][:2] for n in names]
        assert max(math.dist(a, b) for a, b in zip(spots, spots[1:])) <= M.TAPE_EVERY + 1e-9
        for bend in route[1:-1]:
            assert min(math.dist(bend, p) for p in spots) < 1e-9
    up_end = dict(M.TAPE_STRIPS)[[n for n, _ in M.TAPE_STRIPS if n.startswith("tape_up_")][-1]]
    assert math.dist(up_end[:2], M.HOOK_XY) == pytest.approx(0.03)  # at the foot of the hanging tubing
    assert dict(M.TAPE_STRIPS)["tape_edge"][1] > -M.EDGE_Y  # on the table, at its edge
    for name, _ in M.TAPE_STRIPS:
        assert name in TASK.names


# -- what the top camera sees: the drip chamber ----------------------------------------------------------------------

CAM = station.top_camera()
CAM_R = np.array(__import__("yam_arena.urdf", fromlist=["x"]).quat_xyzw_to_matrix(np.array(CAM.rot_xyzw)))
LENS = CAM.lens.resized(320, 176)  # the policy's image


def pixel(p):
    c = CAM_R.T @ (np.asarray(p, dtype=float) - np.asarray(CAM.pos))
    return np.array([LENS.cx + LENS.fx * c[0] / -c[2], LENS.cy - LENS.fy * c[1] / -c[2]])


def extent_px(name):
    """(width, height) in pixels of a plate or box on the chamber, seen by the top camera at 320 x 176."""
    o = next(o for o in M.OBJECTS if o.name == name)
    side, up = np.array([-math.sin(o.yaw), math.cos(o.yaw), 0.0]), np.array([0.0, 0.0, 1.0])
    pos = np.asarray(o.pos)
    w = np.linalg.norm(pixel(pos + side * o.size[1] / 2) - pixel(pos - side * o.size[1] / 2))
    h = np.linalg.norm(pixel(pos + up * o.size[2] / 2) - pixel(pos - up * o.size[2] / 2))
    return w, h


PIXELS = {name: extent_px(name) for name in ("drip_chamber", "drop", "drop_fall")}


def test_the_top_camera_sees_the_whole_drip_chamber():
    o = next(o for o in M.OBJECTS if o.name == "drip_chamber")
    for p in M.box_points(o, 0.005):
        u, v = pixel(p)
        assert 10 <= u <= LENS.width - 10 and 10 <= v <= LENS.height - 10, (u, v)


def test_a_drop_is_at_least_a_pixel_and_its_fall_a_streak_of_three():
    assert min(PIXELS["drop"]) >= 1.0
    assert PIXELS["drop_fall"][1] >= 2.5
    assert PIXELS["drip_chamber"][0] >= 3.5 and PIXELS["drip_chamber"][1] >= 12


def test_the_black_board_is_behind_the_whole_chamber_as_the_camera_sees_it():
    """Every sight line from the top camera through the chamber ends on the board, which stands on the table."""
    board = next(o for o in M.OBJECTS if o.name == "backing_board")
    chamber = next(o for o in M.OBJECTS if o.name == "drip_chamber")
    normal = np.array([math.cos(board.yaw), math.sin(board.yaw), 0.0])  # toward the camera
    side = np.array([-math.sin(board.yaw), math.cos(board.yaw), 0.0])
    face = np.asarray(board.pos) + normal * board.size[0] / 2
    cam = np.asarray(CAM.pos)
    assert np.dot(cam - face, normal) > 0  # the board faces the camera
    for p in M.box_points(chamber, 0.005):
        ray = p - cam
        hit = cam + ray * np.dot(face - cam, normal) / np.dot(ray, normal)
        assert np.dot(hit - p, ray) > 0  # behind the chamber, not in front of it
        d = hit - np.asarray(board.pos)
        assert abs(np.dot(d, side)) <= board.size[1] / 2 - 0.01 and abs(d[2]) <= board.size[2] / 2 - 0.01
    assert board.pos[2] - board.size[2] / 2 == pytest.approx(M.TOP)
    assert abs(M.BOARD_XY[1]) + 0.06 < M.EDGE_Y  # on the table
    bag = next(o for o in M.OBJECTS if o.name == "saline_bag")
    assert board.pos[2] + board.size[2] / 2 < bag.pos[2] - bag.size[2] / 2  # under the bag


def test_the_drops_are_drawn_clear_against_the_board_not_coloured():
    """A clear drop shows only as a grey glint on the dark chamber in front of the black board: no blue."""
    luma = lambda c: 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]  # noqa: E731
    for c in (M.DROP, M.CHAMBER, M.SALINE, M.BOARD_BLACK):
        assert max(c) - min(c) <= 0.05  # grey, no hue
    assert luma(M.BOARD_BLACK) < luma(M.CHAMBER) < luma(M.SALINE) < luma(M.DROP)
    assert luma(M.DROP) - luma(M.SALINE) >= 0.25  # the glint stands out from the chamber and its saline


def test_the_drop_faces_the_camera_from_in_front_of_the_chamber():
    chamber = next(o for o in M.OBJECTS if o.name == "drip_chamber")
    drop = next(o for o in M.OBJECTS if o.name == "drop")
    to_cam = np.asarray(CAM.pos) - np.asarray(chamber.pos)
    assert np.dot(np.asarray(drop.pos) - np.asarray(chamber.pos), to_cam) > 0


def test_each_drop_grows_then_falls_and_the_fall_shows_for_one_frame():
    """A real drop falls the 2 cm from the drip tube to the saline in about 0.06 s: about one frame at 20 Hz."""
    assert M.FALL_STEPS == 1 and math.sqrt(2 * 0.02 / 9.81) <= 1.5 / HZ
    M._reset({})
    rate, shown = 20.0, []
    for _ in range(int(7 * HZ)):
        M._show_drip(rate)
        shown.append((M.LOG.shown.get("drop"), M.LOG.shown.get("drop_fall")))
    falls = [i for i, (_, f) in enumerate(shown) if f and not shown[i - 1][1]]
    assert len(falls) == 2  # one drop every 3 s
    for i in falls:
        assert [f for _, f in shown[i:i + M.FALL_STEPS + 1]] == [True] * M.FALL_STEPS + [False]
        assert shown[i - 1][0] and not shown[i][0]  # the hanging drop goes as it falls
    assert sum(1 for d, _ in shown if d) >= 0.5 * len(shown)  # the drop hangs, growing, most of the time


# -- the drip-rate model ----------------------------------------------------------------------------------------------


def test_shut_and_the_first_six_mm_drip_nothing():
    assert M.drip_rate(0.0) == 0.0 and M.drip_rate(M.T_OPEN) == 0.0


def test_the_rate_climbs_with_travel_then_streams():
    rates = [M.drip_rate(t) for t in np.linspace(M.T_OPEN, M.T_STREAM - 1e-6, 50)]
    assert all(b > a for a, b in zip(rates, rates[1:]))
    assert math.isinf(M.drip_rate(M.T_STREAM)) and math.isinf(M.drip_rate(M.TRAVEL))


def test_twenty_drops_a_minute_is_at_the_travel_the_demo_aims_for():
    assert M.drip_rate(M.travel_for(20)) == pytest.approx(20.0)
    assert M.counted(M.travel_for(20)) == 20


def test_the_full_score_band_is_about_four_tenths_of_a_mm_of_wheel():
    width = M.travel_for(22.5) - M.travel_for(17.5)
    assert 0.0003 <= width <= 0.0005


def test_the_rate_goes_with_the_head():
    """Slit flow goes as head x gap^3: half the head halves the rate at the same travel."""
    t = M.travel_for(20)
    M.HEAD /= 2
    try:
        assert M.drip_rate(t) == pytest.approx(10.0)
    finally:
        M.HEAD *= 2


def test_the_stream_starts_at_three_drops_a_second():
    assert M.drip_rate(M.T_STREAM - 1e-6) == pytest.approx(M.R_STREAM, rel=1e-3)
    assert 0.014 < M.T_STREAM < M.TRAVEL


def test_twenty_drops_a_minute_is_two_mL_a_minute():
    assert M.TARGET / M.DROPS_PER_ML == 2.0


# -- the outcome table, lowest first ----------------------------------------------------------------------------------


def test_doing_nothing_scores_zero():
    assert run((0.0, 20.0, FAR)) == 0.0


def test_opened_but_short_of_the_first_drop_scores_zero():
    assert run((M.T_OPEN - 0.001, 10.0, on_clamp()), (M.T_OPEN - 0.001, 10.0, FAR)) == 0.0


@pytest.mark.parametrize("rate", [1, 4, 9, 31, 45, 60])
def test_dripping_far_from_the_rate_scores_a_quarter(rate):
    assert released(rate) == 0.25


@pytest.mark.parametrize("rate", [61, 90, 120, 170])
def test_left_at_a_dangerous_over_rate_scores_zero(rate):
    """Over 60 a minute, three times the order, is a fluid-overload hazard: no credit."""
    assert released(rate) == 0.0


def test_held_at_the_run_limit_at_an_over_rate_scores_zero():
    assert run((at(90), 60.0, on_clamp())) == 0.0


@pytest.mark.parametrize("rate", [10, 12, 15, 25, 28, 30])
def test_within_half_of_the_rate_scores_a_half(rate):
    assert released(rate) == 0.5


@pytest.mark.parametrize("rate", [16, 17, 23, 24])
def test_within_a_fifth_of_the_rate_scores_three_quarters(rate):
    assert released(rate) == 0.75


@pytest.mark.parametrize("rate", [18, 19, 20, 21, 22])
def test_eighteen_to_twenty_two_after_letting_go_scores_one(rate):
    assert released(rate) == 1.0


def test_twenty_a_minute_still_held_at_the_end_scores_a_quarter():
    assert run((at(20), 60.0, on_clamp())) == 0.25


@pytest.mark.parametrize("travel", [round(float(t), 4) for t in np.linspace(0.0, 0.020, 21)])
def test_a_run_limit_trial_scores_only_zero_or_a_quarter_wherever_the_wheel_is(travel):
    """Still held at the run limit, the operator opens the gripper and lifts it clear before counting: wherever that
    leaves the wheel, the score is 0.25 for 1-60 drops and 0 otherwise, so clearing it can't raise the score."""
    score = run((travel, 15.0, on_clamp()))
    n = M.counted(travel)
    assert score == (0.25 if 1 <= n <= M.OVER else 0.0)


def test_every_cheat_scores_below_every_honest_partial():
    cheats = [run((M.TRAVEL, 4.0, on_clamp()), (at(20), 10.0, on_clamp()), (at(20), 10.0, FAR)),
              run((at(20), 10.0, on_clamp(), (0.0, 0.05, 0.0)), (at(20), 10.0, FAR, (0.0, 0.05, 0.0)))]
    assert max(cheats) < 0.25


# -- latches, milestones and the release ------------------------------------------------------------------------------


def test_a_stream_for_more_than_three_seconds_scores_zero_even_after_setting_twenty():
    assert run((M.T_STREAM + 0.001, 3.2, on_clamp()), (at(20), 10.0, on_clamp()), (at(20), 10.0, FAR)) == 0.0


def test_a_stream_of_under_three_seconds_costs_nothing():
    assert run((M.T_STREAM + 0.001, 2.5, on_clamp()), (at(20), 10.0, on_clamp()), (at(20), 10.0, FAR)) == 1.0


def test_two_short_streams_do_not_add_up():
    assert run((M.T_STREAM + 0.001, 2.5, on_clamp()), (at(20), 1.0, on_clamp()), (M.T_STREAM + 0.001, 2.5, on_clamp()),
               (at(20), 10.0, FAR)) == 1.0


def test_tubing_pulled_from_its_tape_scores_zero_even_when_put_back():
    pulled = (0.0, 0.0, 0.035)  # the clamp lifted 3.5 cm on rigid stubs: both taped ends leave their strips
    assert run((at(20), 2.0, on_clamp(), pulled), (at(20), 10.0, on_clamp()), (at(20), 10.0, FAR)) == 0.0


def test_lifting_the_clamp_a_little_or_turning_it_costs_nothing():
    assert run((at(20), 5.0, on_clamp(), (0.0, 0.0, 0.02)), (at(20), 10.0, FAR, (0.0, 0.0, 0.0))) == 1.0
    M._reset({})
    st = clamp(at(20), yaw=0.2)
    assert not M.pulled_loose(st)


def test_the_rate_set_then_shut_again_loses_the_credit():
    assert run((at(20), 10.0, on_clamp()), (0.0, 10.0, on_clamp()), (0.0, 10.0, FAR)) == 0.0


def test_nudged_on_release_is_scored_where_it_was_left():
    assert run((at(20), 10.0, on_clamp()), (at(26), 10.0, FAR)) == 0.5
    assert run((at(20), 10.0, on_clamp()), (at(23), 10.0, FAR)) == 0.75


def test_stepping_back_to_watch_does_not_end_the_trial():
    """Let go at 28 a minute and stand back 15 s to watch the drops, then come back and correct to 20: the trial goes on
    until the arms are back in their start pose or the run limit."""
    assert run((at(28), 5.0, on_clamp()), (at(28), 15.0, FAR), (at(20), 5.0, on_clamp()), (at(20), 10.0, FAR)) == 1.0
    assert not M.LOG.ended
    assert run((at(28), 5.0, on_clamp()), (at(28), 15.0, FAR), (at(20), 5.0, on_clamp()), (at(20), 1.0, HOME)) == 1.0
    assert M.LOG.ended


def test_released_and_clear_at_the_run_limit_is_scored_by_the_count():
    assert released(20, clear_s=0.05) == 1.0
    assert released(26) == 0.5 and not M.LOG.ended


def test_coming_back_to_correct_within_ten_seconds_is_allowed():
    assert run((at(28), 5.0, on_clamp()), (at(28), 6.0, FAR), (at(20), 5.0, on_clamp()), (at(20), 10.0, FAR)) == 1.0


def test_the_trial_ends_when_both_arms_are_back_in_their_start_pose_whatever_the_score():
    """Back in the start pose at 28 a minute: the trial is over at 0.5; coming back to set 20 changes nothing."""
    assert run((at(28), 5.0, on_clamp()), (at(28), 0.05, HOME), (at(20), 5.0, on_clamp()), (at(20), 10.0, FAR)) == 0.5
    assert M.LOG.ended and M.LOG.end_travel == pytest.approx(at(28))


@pytest.mark.parametrize("rate,score", [(20, 1.0), (23, 0.75), (12, 0.5), (40, 0.25), (90, 0.0)])
def test_back_in_the_start_pose_is_scored_by_the_count(rate, score):
    assert finished(rate) == score and M.LOG.ended


def test_nothing_after_the_end_is_scored():
    """Set 20 and return to the start pose: 1.0, even if the arm comes back and opens it to a stream afterwards."""
    assert run((at(20), 5.0, on_clamp()), (at(20), 1.0, HOME), (M.TRAVEL, 5.0, on_clamp()), (M.TRAVEL, 5.0, FAR)) == 1.0
    assert not M.LOG.streamed


def test_the_start_pose_ends_the_trial_only_once_a_gripper_has_come_to_the_clamp():
    """The arms start in their start pose: staying there 15 s before reaching the clamp doesn't end the trial."""
    run((0.0, 15.0, HOME))
    assert not M.LOG.ended
    assert run((0.0, 15.0, HOME), (at(20), 5.0, on_clamp()), (at(20), 1.0, HOME)) == 1.0


def test_both_arms_must_be_within_fifty_mm_of_where_they_started():
    assert run((at(28), 5.0, on_clamp()), (at(28), 1.0, home_but((0.0, 0.0, 0.045))), (at(20), 5.0, on_clamp()),
               (at(20), 1.0, FAR)) == 0.5
    assert run((at(28), 5.0, on_clamp()), (at(28), 1.0, home_but((0.0, 0.0, 0.055))), (at(20), 5.0, on_clamp()),
               (at(20), 1.0, FAR)) == 1.0
    one_home = {"grips": [HOME["grips"][0], on_clamp()[0]], "at": HOME["at"]}  # the other gripper still on the clamp
    run((at(20), 5.0, on_clamp()), (at(20), 5.0, one_home))
    assert not M.LOG.ended


def test_the_start_pose_is_clear_of_the_clamp_and_the_line():
    assert M.clear_of(clamp(), HOME["grips"]) and M.off_line(HOME["grips"]) and not M.kinked(HOME["grips"])


def squeezing(name):
    """A gripper (2 cm cube) centred on a stretch of the line's tubing, squeezing it; the other far away."""
    o = next(o for o in M.OBJECTS if o.name == name)
    return [[("gripper", cube_hull(o.pos))], FAR[1]]


@pytest.mark.parametrize("name", ["line_up_0", "line_up_1", "line_down_0", "line_down_1", "line_drop"])
def test_the_line_kinked_or_squeezed_at_the_end_scores_zero(name):
    """A kink or squeeze left in the line outside the clamp throttles the count: 0, not the 0.25 of a held clamp."""
    assert run((at(20), 5.0, on_clamp()), (at(20), 5.0, squeezing(name))) == 0.0
    assert M.LOG.kinked


def test_a_squeeze_let_go_before_the_end_costs_nothing():
    assert run((at(20), 5.0, on_clamp()), (at(20), 2.0, squeezing("line_up_1")), (at(20), 1.0, HOME)) == 1.0
    assert run((at(20), 5.0, on_clamp()), (at(20), 2.0, squeezing("line_up_1")), (at(20), 5.0, FAR)) == 1.0


def test_resting_beside_the_line_is_not_a_kink():
    o = next(o for o in M.OBJECTS if o.name == "line_up_1")
    beside = [[("gripper", cube_hull(np.asarray(o.pos) + (0.0, 0.0, M.TUBE_OD / 2 + 0.0105)))], FAR[1]]
    assert not M.kinked(beside) and not M.off_line(beside)


def test_a_gripper_on_the_hanging_line_is_not_clear():
    """Squeezing the line, the slide clamp or the bag would throttle a real drip: not clear, so the trial goes on."""
    for name in ("line_drop", "slide_clamp", "saline_bag", "line_up_1"):
        o = next(o for o in M.OBJECTS if o.name == name)
        on_line = [[("gripper", cube_hull(np.asarray(o.pos) + (0.0, 0.0, o.size[2] / 2 + 0.015)))], FAR[1]]
        assert not M.off_line(on_line), name
    assert run((at(20), 5.0, on_clamp()), (at(20), 12.0, [[("gripper", cube_hull((M.HOOK_XY[0], M.HOOK_XY[1] - 0.012,
                                                                                    M.TOP + M.SLIDE_Z)))], FAR[1]])) == 0.25
    assert M.off_line(FAR)


def test_a_gripper_within_fifty_mm_is_not_clear():
    assert run((at(20), 5.0, on_clamp()), (at(20), 10.0, near(0.045))) == 0.25
    assert run((at(20), 5.0, on_clamp()), (at(20), 10.0, near(0.055))) == 1.0


def test_no_grippers_reported_counts_as_clear():
    assert M.clear_of(clamp(), [])


def test_success_waits_for_the_start_pose_and_the_full_score():
    """The simulator stops early only on success: a 1.0 frozen when both arms are back in their start pose."""
    finished(20)
    assert M.success({"roller_clamp": clamp(at(20))})
    released(20)  # released and clear but not back: the trial runs on to the run limit
    assert not M.success({"roller_clamp": clamp(at(20))}) and M.score({"roller_clamp": clamp(at(20))}) == 1.0
    finished(26)
    assert not M.success({"roller_clamp": clamp(at(26))})


def test_distance_is_wheel_travel_from_the_twenty_drop_point():
    assert M.distance({"roller_clamp": clamp(at(20))}) == pytest.approx(0.0)
    assert M.distance({"roller_clamp": clamp(0.0)}) == pytest.approx(M.travel_for(20))


def test_blind_policies_score_zero_or_a_quarter():
    assert run((0.0, 30.0, FAR)) == 0.0  # never touch it
    assert run((M.TRAVEL, 30.0, FAR)) == 0.0  # roll it all the way: a stream
    assert run((at(20), 30.0, on_clamp())) == 0.25  # never let go


def test_the_drop_counter_shows_about_twenty_drops_in_a_minute():
    run((at(20), 60.0, FAR))
    assert M.LOG.drops in (19, 20)


# -- the text against the code ----------------------------------------------------------------------------------------


def _has(text, piece):
    assert piece in text, f"{piece!r} not in the text"


def test_text_matches_code():
    setup, scoring, fidelity = SPEC["setup"], SPEC["scoring"], SPEC["fidelity"]
    cm = lambda v: f"{v * 100:g}"  # noqa: E731
    mm = lambda v: f"{v * 1000:g}"  # noqa: E731
    for code in M.CATALOGUE.values():
        _has(setup, code)
    _has(setup, f"{mm(M.LENGTH)} mm long, {mm(M.WIDTH)} mm wide")
    _has(setup, f"{mm(M.WHEEL_D)} mm wheel")
    _has(setup, f"centred at x {cm(M.CLAMP_XY[0])}, y {cm(M.CLAMP_XY[1])}")
    _has(setup, f"{cm(M.STUB)} cm beyond each end")
    _has(setup, f"middle of the drip chamber is {cm(M.CHAMBER_MID)} cm above the table")
    _has(setup, f"{(M.TOP - M.FLOOR) * 100:g} cm above the floor")
    _has(setup, f"hook is {(M.TOP + M.HOOK_Z - M.FLOOR) * 100:g} cm above the floor")
    _has(setup, f"saline stands {M.HEAD:.1f} m above the line's end")
    _has(setup, f"floor at x {cm(M.POLE_XY[0])}, y {cm(M.POLE_XY[1])}")
    _has(setup, f"left edge (y {cm(M.EDGE_Y)})")
    _has(setup, f"over the table at x {cm(M.HOOK_XY[0])}, y {cm(M.HOOK_XY[1])}")
    _has(setup, f"{(M.HOOK_XY[1] - M.POLE_XY[1]) * -100:g} cm arm")
    _has(setup, f"{(M.CHAMBER_TOP - M.CHAMBER_LEN - M.SLIDE_Z) * 100:g} cm below the chamber")
    _has(setup, f"floor at x {cm(M.BEAKER_XY[0])}, y {cm(M.BEAKER_XY[1])}")
    _has(setup, f"right edge at x {cm(M.OFF_EDGE[0])}")
    _has(setup, f"{(M.LINE_END - 0.003) * 100:g} cm above the beaker's floor")
    _has(setup, f"{M.TRAVEL * 1000:g} mm")
    _has(scoring, f"{M.CLEAR_GAP * 100:g} cm")
    _has(scoring, f"within {M.START_GAP * 100:g} cm of where it started")
    _has(SPEC["instruction"], "return both arms to their start pose")
    _has(scoring, "kinked, folded or squeezed")
    _has(setup, f"at most {cm(M.TAPE_EVERY)} cm apart")
    _has(setup, f"{len(M.TAPE_STRIPS)} more strips")
    _has(scoring, f"{M.STREAM_S:g} s")
    _has(scoring, f"{M.PULL_GAP * 100:g} cm")
    _has(scoring, f"0 for over {M.OVER} drops")
    _has(scoring, f"from 1 to {M.OVER}")
    for lo, hi, value in M.BANDS:
        _has(scoring, f"{value}: {lo}-{hi}")
    _has(scoring, f"{M.DRIPPING:g}")
    _has(fidelity, f"first {M.T_OPEN * 1000:g} mm")
    _has(fidelity, f"/ {M.SPAN * 1000:g} mm")
    _has(fidelity, f"{M.R_PER_M:g} drops a minute per metre of head")
    _has(fidelity, f"is {M.HEAD:.1f} m")
    _has(fidelity, f"({M.R_STREAM:g} a minute) at {M.T_STREAM * 1000:.1f} mm")
    _has(fidelity, f"{M.OVER + 1} a minute needs {M.travel_for(M.OVER + 1) * 1000:.1f} mm")
    _has(fidelity, "no viscous drag")
    band = (M.travel_for(22.5) - M.travel_for(17.5)) * 1000
    _has(fidelity, f"{band:.1f} mm ({band:.2f} mm)")
    _has(fidelity, f"within {band:.2f} mm")
    _has(fidelity, f"{M.HANG_FROM:.0%} of its growth")
    _has(fidelity, f"{M.DROP_D * 1000:g} x {M.FALL_H * 1000:g} mm streak")
    _has(fidelity, f"for {M.FALL_STEPS} frame ({M.FALL_STEPS / HZ:g} s)")
    chamber, drop, fall = PIXELS["drip_chamber"], PIXELS["drop"], PIXELS["drop_fall"]
    _has(fidelity, f"about {chamber[0]:.0f} x {chamber[1]:.0f} px")
    _has(fidelity, f"about {max(drop):.1f} px")
    _has(fidelity, f"about {fall[0]:.1f} x {fall[1]:.0f} px")
    assert re.search(r"4,800 steps", SPEC["run_limit"])
    # the wheel's breakaway and the arm's lag against it
    _has(setup, f"sim uses {M.WHEEL_BREAKAWAY:g} N")
    _has(setup, "push gauge")
    _has(fidelity, f"breakaway friction of {M.WHEEL_BREAKAWAY:g} N")
    _has(fidelity, f"about {LAG_AT_BREAKAWAY * 1000:g} mm short")
    # the black board behind the chamber
    _has(setup, f"{cm(M.BOARD[1])} cm wide, {cm(M.BOARD[2])} cm tall and {mm(M.BOARD[0])} mm thick")
    _has(setup, f"centred at x {M.BOARD_XY[0] * 100:.1f}, y {M.BOARD_XY[1] * 100:.1f}, {cm(M.BOARD_BACK)} cm behind")
    _has(setup, f"{cm(M.BLOCK[1])} x {cm(M.BLOCK[0])} x {cm(M.BLOCK[2])} cm wooden block")
    _has(setup, "500 lux")
    # the end of a trial: clearing a gripper at the run limit, and no settling time
    _has(scoring, "10 cm straight up")
    _has(scoring, "no settling time")
    _has(SPEC["reset_notes"], "after every 10 trials")


def test_no_other_task_is_named_in_the_text():
    text = json.dumps(SPEC) + README
    assert not re.search(r"#\d{2,}", text) and "PR " not in text


def test_the_text_is_blind():
    text = json.dumps(SPEC).lower()
    for word in ("jason", "nanzheng", "/users/", "robocurve team", "bluedot"):
        assert word not in text


def test_the_required_fields_are_filled():
    for k in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit", "reset_notes",
              "fidelity", "scores", "media"):
        assert SPEC.get(k), k
    assert len(SPEC["setup"]) <= 3000 and len(SPEC["scoring"]) <= 2500


def test_the_readme_and_main_file_fit_the_reviewers_excerpt():
    main = (HERE / "roll_the_iv_roller_clamp_to_20_drops_a_minute.py").read_text()
    assert len(README) + len(main) <= 40000


# -- the demo ---------------------------------------------------------------------------------------------------------


def _plan():
    M.LATEST["states"] = None
    return D.plan(SimpleNamespace(**vars(M)))


def test_every_demo_waypoint_is_reachable():
    for arm, ws in _plan().items():
        plan = ArmPlan(list(ws), station.BASES[arm])
        for k in range(int(plan.duration * HZ) + 1):
            plan.action7((k + 1) / HZ)
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_fits_the_run_limit():
    n = D.steps(_plan())
    limit = int(SPEC["run_limit"].split()[0].replace(",", ""))
    assert n <= limit <= 20 * n


def test_the_demo_ends_holding_still_for_two_seconds():
    for ws in _plan().values():
        assert ws[-1].duration >= 2.0 and np.allclose(np.asarray(ws[-1].pos), np.asarray(ws[-2].pos))


# How far short of where it is sent the arm stops the wheel, against its breakaway: the MuJoCo physics check of this
# demo measured about 1.1 mm at 1.5 N and 3.5 mm at 3 N, along this 45-degree push
LAG_AT_BREAKAWAY = 0.0011  # m at WHEEL_BREAKAWAY


# Rolling back from the bag side the arm presses across its soft wrist (0.15 N/mm): about 14 mm along the face's normal
# before 2.1 N moves the wheel (joint stiffness alone; MuJoCo's arm, with its joint friction, needed about 17.8 mm), i.e.
# it stops about 20-25 mm of travel short of where the underside is sent
BACK_NEED = 2.12 / 0.148 * 0.001  # m along the face's normal
BACK_NEED_MUJOCO = 0.0178


def _replay(lag=0.0, carry=0.0, back_need=BACK_NEED, back_carry=0.0):
    """Play the demo kinematically at 20 Hz: the left hand holds the clamp where it lies; the right fingertip pushes
    the wheel when it meets the wheel's face toward shut (rolling it open), or its underside meets the face toward the
    bag (rolling it back). With the wheel's breakaway friction the compliant arm stops ``lag`` m (of travel) short of
    where it is sent open, sqrt 2 x ``back_need`` short rolling it back; a push that moved the wheel 3 mm or more
    carries it ``carry`` (``back_carry``) m further when it ends, a shorter one in proportion. The wheel never moves on
    its own.
    Returns (final score, steps to success, worst arm clearance m, final travel)."""
    M._reset({"roller_clamp": clamp()})
    plans = {arm: ArmPlan(list(ws), station.BASES[arm]) for arm, ws in _plan().items()}
    travel, done, worst = 0.0, None, math.inf
    st = clamp()
    n = max(int(p.duration * HZ) for p in plans.values())
    links = ["link3", "link4", "link5", "gripper", "tip_left", "tip_right"]
    drop = np.asarray(next(o for o in M.OBJECTS if o.name == "drop").pos)
    sight = np.asarray(CAM.pos) + np.linspace(0.02, 0.98, 60)[:, None] * (drop - np.asarray(CAM.pos))
    SIGHT[0] = math.inf
    KINKED[0] = False
    WAYS.clear()
    PATH.clear()
    face, moved, last, began = 1, 0, None, 0.0
    for k in range(n):
        t = (k + 1) / HZ
        grips, arms, points = [], [], []
        for arm, plan in plans.items():
            a, _ = plan.action7(t)
            finger = a[6] * FINGER_OPEN
            q = {j: float(v) for j, v in zip(ARM_JOINTS, a[:6])} | {j: float(finger) for j in FINGER_JOINTS}
            poses = analysis.fk_link_poses_world(q, station.BASES[arm])
            grips.append(analysis.world_hulls(poses, ["gripper", "tip_left", "tip_right"]))
            points.append(poses["gripper"][:3, 3])
            if k % 5 == 0:
                arms.append(analysis.world_hulls(poses, links))
                for _, h in analysis.world_hulls(poses):  # does any link come between the camera and the drop?
                    SIGHT[0] = min(SIGHT[0], float((sight @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1).min()))
            if arm == "right":
                pos, rot, _, _ = plan.target(t)
                cmd = M.LATEST.get("command") or ("", 0, "eased")
                mode = cmd[2]
                if (cmd[0], mode) != last:  # a new command: the push before it is over, and carries on a little
                    if moved:  # a long push carries it on more than a fine one (the GPU: 1.2 mm after a 3 mm push)
                        share = min(1.0, abs(travel - began) / 0.003)
                        travel = float(np.clip(travel + share * (carry if moved > 0 else -back_carry), 0.0, M.TRAVEL))
                    moved, last, began = 0, (cmd[0], mode), travel
                    PATH.append((cmd[0], mode, round(travel * 1000, 2)))
                if mode in ("push", "back"):
                    face = 1 if mode == "push" else -1
                    if not WAYS or WAYS[-1] != mode:
                        WAYS.append(mode)
                if face > 0:
                    new = _pushed(travel, np.asarray(pos) + PAD_TIP_PAST_TCP * rot[:, 2], lag)
                else:
                    new = _pushed_back(travel, np.asarray(pos) + CD.TIP_END * rot[:, 2] + CD.UNDER * rot[:, 1],
                                       math.sqrt(2.0) * back_need)
                if abs(new - travel) > 1e-9 and mode in ("push", "back"):
                    moved = 1 if new > travel else -1
                travel = new
        if arms:
            worst = min(worst, analysis.hull_set_distance(*arms)[0])
        st = clamp(travel)
        M._watch({"roller_clamp": st}, grips, points)
        KINKED[0] = KINKED[0] or (not M.LOG.ended and M.LOG.kinked)
        if done is None and M.success({"roller_clamp": st}):
            done = k + 1
    ENDED[0] = M.LOG.ended
    return M.score({"roller_clamp": st}), done, worst, travel


def _pushed(travel, tip, lag=0.0):
    """The wheel's travel once a fingertip sent to ``tip`` (world) has pushed it, if it meets the wheel's upper face
    toward shut: it stops ``lag`` m short of where it was sent."""
    p = clamp().rotation().T @ (np.asarray(tip) - clamp().pos)
    if abs(p[1]) > M.WHEEL_W / 2 or p[2] > M.WHEEL_Z + M.WHEEL_D / 2 + 0.002 or p[2] < M.WALL_TOP:
        return travel
    need = p[0] - M.WHEEL_X0 + M.FACET - (p[2] - M.WHEEL_Z - M.FACET)  # the face (x - z constant) through the tip
    if need <= travel - 0.003:  # the fingertip is off the face
        return travel
    return float(min(M.TRAVEL, max(travel, need - lag)))


def _pushed_back(travel, under, lag):
    """The wheel's travel once the fingertips' underside, sent to ``under`` (world), has pushed it back, if it meets
    the wheel's upper face toward the bag (within its width along the slope): it stops ``lag`` m short of where the
    underside was sent."""
    p = clamp().rotation().T @ (np.asarray(under) - clamp().pos)
    centre = np.array((M.WHEEL_X0 + travel + M.FACET, 0.0, M.WHEEL_Z + M.FACET))
    along = float(np.dot(p - centre, (1.0, 0.0, -1.0))) / math.sqrt(2.0)  # down the face, toward the bag
    if abs(p[1]) > M.WHEEL_W / 2 or p[2] > M.WHEEL_Z + M.WHEEL_D / 2 + 0.002 or abs(along) > 0.0045:
        return travel
    at = p[0] + p[2] - M.WHEEL_X0 - M.WHEEL_Z - 2 * M.FACET  # the travel whose face (x + z constant) holds it
    if at >= travel + 0.003:  # the underside is off the face
        return travel
    return float(max(0.0, min(travel, at + lag)))


WAYS = []  # the demo's pushes in order, "push" (open) or "back" (toward shut), runs merged
PATH = []  # each command as it starts: (key, mode, the wheel's travel in mm)
ENDED = [False]  # whether the demo's trial ended with both arms back in their start pose
KINKED = [False]  # whether a gripper ever squeezed the line during the demo
SIGHT = [math.inf]  # the closest any arm link comes to the top camera's line of sight to the drop, over the demo


@pytest.fixture(scope="module")
def replay():
    return _replay()


def test_no_arm_blocks_the_cameras_view_of_the_drop_during_the_demo(replay):
    assert SIGHT[0] > 0.01, SIGHT[0]


def test_the_demo_replay_scores_one(replay):
    score, done, _, travel = replay
    assert score == 1.0 and done is not None
    assert 18 <= M.counted(travel) <= 22


def test_the_demo_reaches_full_score_inside_the_run_limit(replay):
    limit = int(SPEC["run_limit"].split()[0].replace(",", ""))
    assert replay[1] <= limit


def test_the_arms_stay_ten_mm_apart(replay):
    assert replay[2] >= 0.010


def test_the_right_fingertip_stays_out_of_the_housing():
    """The closed fingertip goes into the channel between the walls only, never into a wall or the floor. A press
    rolling the wheel back is sent about 17 mm into the wheel's face across the soft wrist (the wheel holds it off,
    as physcheck shows), so its target and the move easing off it are not judged; the fingertip eased off it is."""
    st = clamp()
    plan = ArmPlan(list(_plan()["right"]), station.BASES["right"])
    walls = [b for b in M.CLAMP.boxes[:3]]
    worst = math.inf
    for k in range(0, int(plan.duration * HZ), 2):
        a, label = plan.action7((k + 1) / HZ)
        if (M.LATEST.get("command") or ("", 0, ""))[2] == "back" and label.startswith(("roll the wheel", "ease off")):
            continue
        q = {j: float(v) for j, v in zip(ARM_JOINTS, a[:6])} | {j: 0.0 for j in FINGER_JOINTS}
        hulls = analysis.world_hulls(analysis.fk_link_poses_world(q, station.BASES["right"]), ["tip_left", "tip_right"])
        for centre, size, _ in walls:
            worst = min(worst, analysis.hulls_to_box(hulls, st.to_world(centre), st.rotation(), size)[0])
    assert worst > -0.0005, worst


def test_the_demo_never_streams(replay):
    assert not M.LOG.streamed and not M.LOG.pulled and not KINKED[0]


def test_the_demo_ends_back_in_the_start_pose(replay):
    """The trial ends when both arms are back where they started: the demo's last frames show the clamp let go."""
    assert ENDED[0] and replay[1] is not None


@pytest.mark.parametrize("lag", [LAG_AT_BREAKAWAY, 0.0035, 0.005, 0.0075])  # 1.5 N, 3 N, more, and the GPU's
def test_the_demo_still_scores_one_against_the_wheels_breakaway(lag):
    """Against the wheel's breakaway the compliant arm stops short of where it is sent. The demo's corrections close
    the loop on the wheel's travel, so it still lands in 18-22 without ever streaming. The Isaac GPU run lagged more
    than the old 6 mm lead allowed (0.25 there, 1.0 in MuJoCo), hence the 7.5 mm case."""
    score, done, _, travel = _replay(lag)
    assert score == 1.0 and done is not None, (lag, travel, M.counted(travel))
    assert 18 <= M.counted(travel) <= 22 and not M.LOG.streamed


@pytest.mark.parametrize("carry", [0.0006, 0.0012, pytest.param(0.0025, marks=pytest.mark.xfail(strict=True, reason=(
    "known limit: if EVERY push carries 2.5 mm past its aim (twice the GPU's measured 1.2 mm), the 11 corrections run "
    "out before the band; kept visible on purpose")))])  # MuJoCo's carry, the GPU's (1.2 mm on a 3 mm push), more
def test_an_overshoot_is_rolled_back_by_the_demos_own_corrections(carry):
    """A push that carries the wheel past the band (the GPU demo's third push left it at 28 a minute) is corrected by
    rolling the wheel back from the bag side, then on again from below: the full score comes from the demo's own pushes,
    with a wheel that never moves by itself."""
    score, done, _, travel = _replay(LAG_AT_BREAKAWAY, carry=carry)
    assert score == 1.0 and done is not None, (carry, travel, M.counted(travel), PATH)
    assert 18 <= M.counted(travel) <= 22 and not M.LOG.streamed
    over = [mm for _, _, mm in PATH if mm > M.travel_for(21.5) * 1000]
    assert not over or "back" in WAYS, PATH  # whenever it went over, it was rolled back


@pytest.mark.parametrize("back_need", [BACK_NEED, BACK_NEED_MUJOCO, 0.021])  # as computed, MuJoCo's arm, softer
def test_rolling_back_lands_whatever_the_arm_gives(back_need):
    """Rolling back presses across the arm's soft wrist, so how far short of the face the underside stops is uncertain;
    the demo aims the roll back under the target, learns the press from each push, and finishes rolling on."""
    score, done, _, travel = _replay(LAG_AT_BREAKAWAY, carry=0.0012, back_need=back_need, back_carry=0.0005)
    assert score == 1.0 and done is not None, (back_need, travel, M.counted(travel), PATH)


def test_the_wheel_carries_no_weight():
    """The wheel's own weight is switched off (the housing keeps its own): on a housing tipped by the holding hand its
    weight along the slide (under 1 mN, against 1.5 N) crept it shut through PhysX's joint friction, which a real wheel
    never does. Its breakaway, mass and static = sliding friction are unchanged."""
    import inspect

    import benchkit

    wheel = M.CLAMP.sliders[0]
    assert wheel.weightless and M.WHEEL_WEIGHTLESS
    assert wheel.friction == M.WHEEL_BREAKAWAY == 1.5 and wheel.rate == 0.0 and wheel.damping == 0.0
    assert wheel.mass == M.WHEEL_MASS
    src = inspect.getsource(benchkit._spawn_tool)
    assert "rigid_body(path, p.rest, p.mass, p.weightless)" in src and "spec.weightless or weightless" in src
    assert not any(getattr(s, "weightless", False) for s in [M.CLAMP])  # the housing still has its weight


def test_the_run_record_shows_whether_the_wheel_holds_while_nothing_pushes_it(tmp_path, monkeypatch):
    """While the demo's fingertip is eased off and the left hand holds the housing, the record keeps the wheel's largest
    drift: 0 for a wheel that holds, as a real one does; the GPU's creep (0.01 mm/s) shows within seconds."""
    import run_record

    monkeypatch.setattr(run_record, "RUN_RECORD", tmp_path)
    for creep, expect in ((0.0, 0.0), (-0.00001, 0.04)):
        M._reset({})
        M.LATEST["command"] = ("c5", at(20), "eased", True, 0.0)
        travel = at(20)
        for _ in range(int(5 * HZ)):
            travel += creep / HZ
            M._watch({"roller_clamp": clamp(travel)}, on_clamp())
        assert M.LATEST["hold_drift_mm"] == pytest.approx(expect, abs=0.002), creep
    line = [ln for ln in (tmp_path / f"{pathlib.Path(M.__file__).stem}-trace.txt").read_text().splitlines()
            if ln.startswith("4s ")][-1]  # the creeping wheel's line
    assert "hold drift 0.03" in line and re.search(r" pitch [+-]0\.00 ", line), line


# -- the run record (written on the simulator machine only) -----------------------------------------------------------


def _fake_env(friction=1.5, dynamic=1.5):
    import torch

    def robot():
        names = list(ARM_JOINTS) + list(FINGER_JOINTS)
        q = list(READY) + [FINGER_OPEN] * len(FINGER_JOINTS)
        return SimpleNamespace(joint_names=names, data=SimpleNamespace(joint_pos=torch.tensor([q], dtype=torch.float64)))

    wheel = SimpleNamespace(joint_names=["wheel_slide"], data=SimpleNamespace(
        joint_friction_coeff=torch.tensor([[friction]]), joint_dynamic_friction_coeff=torch.tensor([[dynamic]])))
    scene = {"left_robot": robot(), "right_robot": robot(), "roller_clamp": wheel}
    return SimpleNamespace(unwrapped=SimpleNamespace(scene=scene))


def test_the_run_record_is_written_once_a_second_only_where_the_runs_folder_exists(tmp_path, monkeypatch):
    import benchkit
    import run_record

    trace = tmp_path / f"{pathlib.Path(M.__file__).stem}-trace.txt"
    monkeypatch.setattr(run_record, "RUN_RECORD", tmp_path / "absent")
    run((at(20), 3.0, on_clamp()))
    assert not trace.exists() and not (tmp_path / "absent").exists()

    monkeypatch.setattr(run_record, "RUN_RECORD", tmp_path)
    monkeypatch.setitem(benchkit._ENV, "env", _fake_env())
    M._reset({})
    M.LATEST["command"] = ("c4", at(20) + 0.004, "push", True, 0.001)
    st = clamp(at(20), shift=(0.0, 0.002, 0.0))
    for _ in range(2 * run_record.EVERY):
        M._watch({"roller_clamp": st}, HOME["grips"], HOME["at"])
    lines = trace.read_text().splitlines()
    assert lines[0].startswith("reset wheel joint: friction_coeff 1.5, dynamic_friction_coeff 1.5")
    ticks = [ln for ln in lines if ln.startswith(("1s ", "2s "))]
    assert len(ticks) == 2, lines
    first = ticks[0]
    assert f"wheel {at(20) * 1000:.2f}" in first and "drip 20.0/min" in first
    assert "demo c4 push sent" in first and "lag +4.00 press +1.00" in first
    assert "right tip depth" in first and " held back " in first and "left tip on housing" in first and "gaps " in first
    assert "housing moved +0.0,+2.0,+0.0 yaw +0.0 tilt 0.0" in first
    assert first.endswith("flags clear score 1")  # clear of the clamp, not yet ended: scored as at the run limit
    assert not any(ln.startswith("ended at step") for ln in lines)  # the clamp was never touched, so nothing ended


def test_the_run_record_never_stops_a_trial_when_the_simulator_hides_the_joint_data(monkeypatch, tmp_path):
    import benchkit
    import run_record

    monkeypatch.setitem(benchkit._ENV, "env", SimpleNamespace(unwrapped=SimpleNamespace(scene={})))
    assert run_record.wheel_joint_physics(M).startswith("unreadable")
    monkeypatch.setitem(benchkit._ENV, "env", None)
    assert run_record.wheel_joint_physics(M) == "no simulator"
    monkeypatch.setattr(run_record, "RUN_RECORD", tmp_path)
    run_record.tick({"__file__": M.__file__, "LOG": SimpleNamespace(step=20), "CONTROL_HZ": 20.0}, {}, 0.0)
    trace = tmp_path / f"{pathlib.Path(M.__file__).stem}-trace.txt"
    assert "1s trace failed: KeyError" in trace.read_text()  # a line that can't be made is recorded, never raised
