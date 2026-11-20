"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

``world.py`` stands in for the simulator (arms with a lag, a head that goes down only under the left fingers
or the lid, a lid held by the fork, pressed by fingertips and pushed up by the head, a catch that snaps once
the lid is shut). The demonstration gets the same state it gets in the simulator and ``grading.py`` scores
every step. With ``--clearance`` the arms' collision hulls are checked against the box, the lid and the head.

Run:  python tools/dryrun_demo.py [--yam-arena <checkout>] [--clearance <folder with clearance.py>]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TIP_LINKS = ("tip_left", "tip_right")
BODY_LINKS = ("gripper", "link5", "link4", "link3", "link2")


def box_gap(cloud, low, high):
    """Signed distance of the nearest point of ``cloud`` to an axis-aligned box (negative inside)."""
    d = np.maximum(np.asarray(low) - cloud, cloud - np.asarray(high))
    outside = np.linalg.norm(np.maximum(d, 0.), axis=1)
    return float(np.where(outside > 0., outside, d.max(axis=1)).min())


def oriented_gap(cloud, centre, rotation, half):
    local = (cloud - centre) @ rotation
    return box_gap(local, -half, half)


def lowest(report, key, value, tick):
    if value < report.get(key + "_mm", np.inf):
        report[key + "_mm"], report[key + "_tick"] = round(value, 1), tick


def dry_run(variant="full", *, max_steps=None, clearance=None, verbose=False, **world_options):
    from yam_arena import ik

    from build_scene import box_colliders
    from rdb_jack_in_box import fixed_scene as fixture
    from rdb_jack_in_box import layout as L
    from rdb_jack_in_box.grading import JackGrade
    from rdb_jack_in_box.scripted import JackReference
    from world import World

    world = World(**world_options)
    reference = JackReference(variant)
    actions = reference.sequence()
    grade = JackGrade()
    joints = world.joints.copy()
    walls = [(np.asarray(lo) + (L.BOX_X, 0., 0.), np.asarray(hi) + (L.BOX_X, 0., 0.)) for _, _, lo, hi in box_colliders()]
    report = {"worst_limit_margin_deg": 180., "peak_score": 0.}
    result, steps = None, max_steps or fixture.MAX_STEPS
    for tick in range(1, steps + 1):
        reference.observe(joints, world.state())
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{fixture.ARMS[i]} arm commanded past a joint limit at tick {tick}: {q}")
            report["worst_limit_margin_deg"] = min(report["worst_limit_margin_deg"],
                                                   float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
        joints = world.step(command)
        if clearance is not None and tick % 2 == 0:
            phase = reference.phases[-1][1]
            clouds = [clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fixture.BASES[arm], links=BODY_LINKS)[::6]
                      for i, arm in enumerate(fixture.ARMS)]
            tips = [clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fixture.BASES[arm], links=TIP_LINKS)[::2]
                    for i, arm in enumerate(fixture.ARMS)]
            lowest(report, "min_arm_gap", 1000 * clearance.arm_gap(np.concatenate((clouds[0], tips[0])),
                                                                  np.concatenate((clouds[1], tips[1]))), tick)
            plate = world.lid_plate()
            head = world.head_centre()
            for arm, cloud, tip in zip(fixture.ARMS, clouds, tips, strict=True):
                both = np.concatenate((cloud, tip))
                lowest(report, "min_table_gap", 1000 * float(both[:, 2].min()), tick)
                lowest(report, f"{arm}_body_to_box", 1000 * min(box_gap(cloud, lo, hi) for lo, hi in walls), tick)
                lowest(report, f"{arm}_tips_to_box_walls", 1000 * min(box_gap(tip, lo, hi) for lo, hi in walls), tick)
                lowest(report, f"{arm}_body_to_lid", 1000 * oriented_gap(cloud, *plate), tick)
                lowest(report, f"{arm}_body_to_head", 1000 * (float(np.linalg.norm(cloud - head, axis=1).min()) - L.HEAD_R), tick)
            if phase.startswith("hand-off 1"):
                lowest(report, "left_tips_to_lid_during_handoff", 1000 * oriented_gap(tips[0], *plate), tick)
                lowest(report, "left_body_to_lid_during_handoff", 1000 * oriented_gap(clouds[0], *plate), tick)
        result = grade.update(*world.grade_inputs())
        report["peak_score"] = max(report["peak_score"], result["score"])
        if verbose and tick % 20 == 0:
            print(tick, result["score"], reference.phases[-1][1], f"lid {np.degrees(world.lid):.1f}",
                  f"head top {1000 * world.head_top():.0f}", world.caught)
        if result["score"] == 1.:
            break
    report.update(score=result["score"], steps=tick, worst_ik_mm=round(1000 * reference.worst_ik, 3),
                  phases=[name for _, name in reference.phases], lid_deg=result.get("lid_deg"),
                  hands_clear=result.get("hands_clear"), pressed_while_closing=result["pressed_while_closing"])
    return report, reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-arena")
    parser.add_argument("--clearance", help="folder holding clearance.py (arm collision hulls), optional")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs", *((args.yam_arena,) if args.yam_arena else ())):
        sys.path.insert(0, str(path))
    clearance = None
    if args.clearance:
        sys.path.insert(0, args.clearance)
        import clearance
    cases = (("full, nominal", {}),
             ("full: slow arm, left hand 5 mm high and 4 mm back", dict(lag_s=.25, hold_offset=(-.004, 0., .005))),
             ("full: fast arm, left hand 5 mm low, press 4 mm short", dict(lag_s=.08, hold_offset=(0., 0., -.005),
                                                                          press_offset=(0., 0., .004))),
             ("full: lid with 8 degrees of play in the fork", dict(lid_play_deg=8.)))
    for name, options in cases:
        report, _ = dry_run("full", clearance=clearance if name == "full, nominal" else None, verbose=args.verbose, **options)
        print(name, "->", {k: v for k, v in report.items() if k != "phases"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
