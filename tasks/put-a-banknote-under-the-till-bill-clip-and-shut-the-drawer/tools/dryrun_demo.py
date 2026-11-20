"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

See tests/kinematic_world.py for what the stand-in world does and does not model.

Run:  python tools/dryrun_demo.py [--yam-arena <checkout>] [--verbose]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dry_run(*, clearance=None, verbose=False, **world_options):
    from kinematic_world import KinematicWorld, play
    from rdb_till_note import fixed_scene as fixture
    from rdb_till_note.scripted import TillReference
    from yam_arena import ik

    reference, world = TillReference(), KinematicWorld(clearance=clearance, **world_options)

    def watch(world, reference, result):
        if verbose and world.tick % 20 == 0:
            strips, _ = world.strips()
            print(world.tick, reference.phases[-1][1] if reference.phases else "", result["score"],
                  "hold", round(float(np.degrees(world.hold[2])), 1), "open", round(1000 * world.s),
                  "note x", np.round(1000 * strips[[0, -1], 0]), "z", np.round(1000 * strips[:, 2]))

    result, trace = play(reference, world, fixture.MAX_STEPS, watch=watch)
    margins = min(float(np.degrees(np.minimum(trace[:, i:i + 6] - ik.LOWER, ik.UPPER - trace[:, i:i + 6]).min()))
                  for i in (0, 7))
    low = np.minimum(trace[:, [*range(6), *range(7, 13)]] - np.tile(ik.LOWER, 2), np.tile(ik.UPPER, 2) - trace[:, [*range(6), *range(7, 13)]])
    where = np.unravel_index(np.argmin(low), low.shape)
    steps = np.abs(np.diff(trace, axis=0))
    report = {"score": result["score"], "steps": len(trace), "worst_ik_mm": round(1000 * reference.worst_ik, 3),
              "worst_limit_margin_deg": round(margins, 1),
              "largest_joint_step_rad": round(float(steps[:, [*range(6), *range(7, 13)]].max()), 3),
              "tightest_joint": f"{fixture.ARMS[where[1] // 6]} joint{where[1] % 6 + 1} at tick {where[0]}",
              "note": world.peak, "phases": reference.phases, "final": result}
    if clearance is not None:
        tight = sorted((gap, f"{arm} {name} @tick {tick}") for (arm, name), (gap, tick) in world.min_gap.items())[:8]
        report["tightest_clearances_mm"] = [(round(1000 * gap, 1), what) for gap, what in tight]
        report["min_arm_to_arm_mm"] = round(1000 * world.min_arm_gap, 1)
    return report, reference, world


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-arena")
    parser.add_argument("--clearance", help="folder holding clearance.py (default: this tools folder)")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--events", action="store_true", help="print the demonstration's own log")
    args = parser.parse_args()
    for path in (ROOT, ROOT / "tests", ROOT / "tools/stubs", *((args.yam_arena,) if args.yam_arena else ())):
        sys.path.insert(0, str(path))
    sys.path.insert(0, args.clearance or str(ROOT / "tools"))
    import clearance
    runs = (("nominal", {}), ("shoulders droop 0.03 rad, slow arms", dict(sag=.03, arm_lag_s=.2)),
            ("note hangs 4 mm to one side", dict(note_y_error=.004)))
    for name, options in runs:
        report, reference, world = dry_run(clearance=clearance if name == "nominal" else None,
                                           verbose=args.verbose and name == "nominal", **options)
        print(name, "->")
        for key, value in report.items():
            print("   ", key, value)
        if args.events and name == "nominal":
            for row in reference.log["events"]:
                print("   ", row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
