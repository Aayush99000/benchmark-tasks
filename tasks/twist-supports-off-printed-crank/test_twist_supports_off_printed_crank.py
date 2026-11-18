"""Checks of twist_supports_off_printed_crank.py's scoring and demo, without the simulator. Needs yam_arena but not pytest:

    python -m pytest -q test_twist_supports_off_printed_crank.py      # in Arena's venv
    python test_twist_supports_off_printed_crank.py                   # anywhere yam_arena imports

Write one test per outcome in the task's scoring text (each partial credit, each zero gate, each cheat you thought
of), so a reviewer's "every possible outcome is covered" has evidence behind it.
"""

from pathlib import Path

import numpy as np
from yam_arena import ik
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import Box, Cylinder, ObjectState, Sphere, load

TASK = load(Path(__file__).with_name("twist_supports_off_printed_crank.py"))
M = TASK.module
CONTROL_HZ = 20.0
RUN_LIMIT_STEPS = 2400  # keep equal to task.json's run_limit

from labkit import KitObject, start_state  # noqa: E402  (the task file put its folder on the path)


# -- states -----------------------------------------------------------------------------------------------------------


def at_rest(spec, pos=None, quat=(0, 0, 0, 1.0), moving=False) -> ObjectState:
    """``spec`` at ``pos`` (default: where it starts), turned by ``quat``."""
    if isinstance(spec, KitObject) and pos is None and tuple(quat) == (0, 0, 0, 1.0) and not moving:
        return start_state(spec)  # a ButtonState for a button
    if hasattr(spec, "corners"):  # labkit objects
        pos = start_state(spec).pos if pos is None else np.asarray(pos, float)
        c = spec.corners(pos, quat)
        low, high = c.min(0), c.max(0)
    else:
        pos = np.asarray(spec.pos if pos is None else pos, float)
        if isinstance(spec, Box):
            half = np.asarray(spec.size, float) / 2
        elif isinstance(spec, Cylinder):
            half = np.array([spec.radius, spec.radius, spec.height / 2])
        elif isinstance(spec, Sphere):
            half = np.full(3, spec.radius)
        else:
            half = np.full(3, spec.extent / 2)
        low, high = pos - half, pos + half
    vel = np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat, float), lin_vel=vel,
                       ang_vel=np.zeros(3), low=low, high=high)


def start() -> dict:
    """Every object where it starts, still."""
    return {spec.name: at_rest(spec) for spec in M.OBJECTS}


def run(*frames) -> float:
    """Play a trial frame by frame (score() runs every step, as in the simulator); the last frame's score."""
    import labkit

    for fn in labkit._RESET:  # what a real reset runs: every on_reset hook and Trial
        fn()
    out = 0.0
    for frame in frames:
        out = M.score(frame)
    return out


def with_(base: dict, **states) -> dict:
    return {**base, **states}


# -- the task file ------------------------------------------------------------------------------------------------------


def test_the_task_file_loads():
    assert TASK.scene == "station" and TASK.score and TASK.success and TASK.demo


def test_doing_nothing_scores_zero():
    assert run(start(), start()) == 0.0
    assert not M.success(start())


# -- outcomes --------------------------------------------------------------------------------------------------------
import labkit  # noqa: E402

FAR = dict(left=(0.0, 0.25, 0.95), right=(0.0, -0.25, 0.95))  # both TCPs well away from the part


def crank_at(origin, quat=(0, 0, 0, 1.0), moving=False):
    return at_rest(M.CRANK, origin, quat, moving)


def moved(spec, d, quat=(0, 0, 0, 1.0), moving=False):
    """``spec`` shifted by ``d`` from where it starts (a support carried with the part moves the same)."""
    return at_rest(spec, np.add(spec.pos, d), quat, moving)


def in_bin(spec):
    """A support lying in the bin: its centre over the bin's centre, 1.5 cm above the floor."""
    target = np.array([*M.BIN_POS, M.TOP + M.WALL + 0.015])
    return at_rest(spec, target - np.asarray(spec.box_at))


def crank_in_tray(dz=0.0):
    """The part standing on the tray's floor, its middle over the tray's middle."""
    target = np.array([*M.TRAY_POS, M.TOP + M.WALL + dz])
    return crank_at(target)


def all_done(n_binned=None, tray=True):
    n_binned = len(M.SUPPORTS) if n_binned is None else n_binned
    st = start()
    for k, s in enumerate(M.SUPPORTS):
        st[s.name] = in_bin(s) if k < n_binned else moved(s, (-0.10, 0.03 * k, -0.10))  # off, on the bench
    if tray:
        st["crank"] = crank_in_tray()
    return st


def test_everything_done_scores_full_and_succeeds_once_let_go():
    done = all_done()
    assert run(start(), done) == 1.0
    labkit.set_gripper_points(**FAR)
    assert M.success(done)


def test_success_needs_the_grippers_off_the_part():
    done = all_done()
    assert run(start(), done) == 1.0  # a reset clears the TCPs: set them after it, as a step would
    labkit.set_gripper_points(left=FAR["left"], right=M.centre("crank", done["crank"]) + [0, 0, 0.02])
    assert not M.success(done)
    labkit.set_gripper_points(**FAR)


def test_the_habit_move_scores_zero_part_trayed_with_supports_on():
    """Pick the print up and put it in the tray: the supports come with it (they move with the part)."""
    d = np.array([*M.TRAY_POS, M.TOP + M.WALL]) - np.asarray(M.CRANK.pos)
    carried = {s.name: moved(s, d) for s in M.SUPPORTS}
    trayed = with_(start(), crank=crank_in_tray(), **carried)
    assert run(start(), trayed) == 0.0


def test_pulling_a_support_straight_lifts_the_whole_print_and_scores_zero():
    lifted = with_(start(), crank=moved(M.CRANK, (0, 0, 0.06), moving=True),
                   **{s.name: moved(s, (0, 0, 0.06), moving=True) for s in M.SUPPORTS})
    assert run(start(), lifted) == 0.0


def test_a_turned_print_carried_with_its_supports_has_none_off():
    q = ik.matrix_to_quat_xyzw(labkit._yaw_matrix(-np.pi / 3))
    o = np.asarray(M.CRANK.pos) + [0, 0, 0.05]
    st = with_(start(), crank=crank_at(o, q), **{s.name: at_rest(s, o, q) for s in M.SUPPORTS})
    assert M.supports_off(st) == [False] * len(M.SUPPORTS)


def test_a_support_rocked_a_few_mm_is_not_off():
    st = with_(start(), support_0=moved(M.SUPPORTS[0], (0.003, 0.0, 0.0)))
    assert M.supports_off(st)[0] is False


def test_each_support_off_and_in_the_bin_earns_its_share():
    for n in range(len(M.SUPPORTS) + 1):  # n off and in the bin, the rest off on the bench
        want = M.PER_SUPPORT * n + M.PER_OFF * (len(M.SUPPORTS) - n)
        assert abs(run(start(), all_done(n_binned=n, tray=False)) - want) < 1e-9


def test_supports_off_but_left_on_the_bench_earn_the_off_share():
    st = start()
    for k, s in enumerate(M.SUPPORTS):
        st[s.name] = moved(s, (-0.10, 0.03 * k, -0.10))  # off the bed, on the table in front of the printer
    assert abs(run(start(), st) - M.PER_OFF * len(M.SUPPORTS)) < 1e-9


def test_the_part_in_the_tray_earns_the_tray_share_only_with_all_supports_off():
    n = len(M.SUPPORTS)
    assert abs(run(start(), all_done()) - 1.0) < 1e-9
    most = all_done(n_binned=n - 1)
    last = M.SUPPORTS[-1]
    most[last.name] = moved(last, np.array([*M.TRAY_POS, M.TOP + M.WALL]) - np.asarray(M.CRANK.pos))
    assert abs(run(start(), most) - M.PER_SUPPORT * (n - 1)) < 1e-9  # one support still on: no off share, no tray


def test_all_supports_off_but_on_the_bench_with_the_part_in_the_tray_scores_the_tray_share():
    st = all_done(n_binned=0)  # all four off, on the bench: the off share for each, and the tray share
    assert abs(run(start(), st) - (M.PER_OFF * len(M.SUPPORTS) + M.TRAY_CREDIT)) < 1e-9


def test_the_part_hovering_over_the_tray_is_not_in_it():
    st = all_done()
    st["crank"] = crank_in_tray(dz=0.05)
    assert abs(run(start(), st) - M.PER_SUPPORT * len(M.SUPPORTS)) < 1e-9


def test_a_dropped_part_scores_zero_for_the_rest_of_the_trial():
    up = at_rest(M.CRANK, np.add(M.CRANK.pos, (0, 0, 0.05)))
    falling = with_(start(), crank=ObjectState(pos=up.pos, quat_xyzw=up.quat_xyzw, lin_vel=np.array([0.0, 0.0, -2.0]),
                                               ang_vel=np.zeros(3), low=up.low, high=up.high))
    assert run(start(), falling, all_done()) == 0.0


def test_the_part_off_the_table_scores_zero():
    gone = with_(all_done(), crank=crank_at((0.24, 0.0, M.TOP - 0.20)))
    assert run(start(), gone) == 0.0


def test_the_part_in_the_bin_scores_zero():
    st = all_done(tray=False)
    st["crank"] = crank_at(np.array([*M.BIN_POS, M.TOP + M.WALL]))
    assert run(start(), st) == 0.0


def test_supports_still_moving_in_the_bin_count_but_success_waits_for_them_to_settle():
    st = all_done()
    s = M.SUPPORTS[0]
    st[s.name] = at_rest(s, in_bin(s).pos, moving=True)
    assert run(start(), st) == 1.0
    labkit.set_gripper_points(**FAR)
    assert not M.success(st)


def test_every_trial_starts_fresh():
    falling = with_(start(), crank=ObjectState(
        pos=np.asarray(M.CRANK.pos, float), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.array([0, 0, -2.0]),
        ang_vel=np.zeros(3), low=start()["crank"].low, high=start()["crank"].high))
    run(start(), falling)
    assert run(start(), all_done()) == 1.0


def test_the_shares_add_up_to_one():
    assert abs(M.PER_SUPPORT * len(M.SUPPORTS) + M.TRAY_CREDIT - 1.0) < 1e-9


def test_every_branch_tip_stops_just_under_the_part():
    """Each tree's top (its highest limb) is 0.2 to 1 mm under the bar's underside or the up boss's rim."""
    for s, tree in zip(M.SUPPORTS, M.TREES):
        top = max(p.at[2] + p.size[2] / 2 for p in s.parts)
        z_part = (M.Z_TIP if tree[3][0][2] == M.Z_TIP else M.Z_RIM) / 1000 + 0.0002
        assert z_part - 0.0012 < top <= z_part - 0.0001, (s.name, top, z_part)


def test_every_support_starts_inside_the_crank_s_reach_and_touching_the_bed():
    for s in M.SUPPORTS:
        st = start_state(s)
        assert abs(st.low[2] - M.BED_TOP) < 1e-9, s.name
        assert st.high[2] < M.BED_TOP + 0.0330, s.name  # tips under the part (bar underside 25 mm, rim 33 mm)


# -- the demo ------------------------------------------------------------------------------------------------------------


def _plans() -> dict[str, ArmPlan]:
    return {arm: ArmPlan(ws, BASES[arm]) for arm, ws in TASK.demo().items()}


def test_the_demo_is_reachable_within_the_joint_limits():
    """Played as yam_scripted plays it: 60 IK iterations per 20 Hz step, warm-started."""
    for arm, plan in _plans().items():
        steps = round(plan.duration * CONTROL_HZ) + 5
        actions = np.array([plan.action7((k + 1) / CONTROL_HZ)[0] for k in range(steps)])
        assert plan.worst_ik_mm < 3.0 and plan.worst_ik_deg < 1.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(actions[:, :6] >= ik.LOWER - 1e-9) and np.all(actions[:, :6] <= ik.UPPER + 1e-9), arm


def test_the_demo_fits_the_run_limit_with_room_to_spare():
    longest = max(plan.duration for plan in _plans().values())
    assert longest * CONTROL_HZ < RUN_LIMIT_STEPS * 0.75, longest


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted((n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)):
        try:
            fn()
            print(f"PASS {name}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"FAIL {name}: {type(e).__name__}: {e}")
    print(f"{failed} failed")
    raise SystemExit(1 if failed else 0)
