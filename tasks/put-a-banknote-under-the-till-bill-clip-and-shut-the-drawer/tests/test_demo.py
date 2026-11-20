"""Dry run of the scripted demonstration through the real scorer, without a simulator.

Needs ``yam_arena`` (pure-Python parts), SciPy and ``inspect_robots`` (or the stand-in in tools/stubs); skipped
otherwise. See kinematic_world.py for what this does and does not show.
"""

from pathlib import Path
import unittest

import numpy as np

import conftest  # noqa: F401

try:
    from yam_arena import ik
    import clearance
    from rdb_till_note import fixed_scene as fixture, scripted, till_model as m
    from kinematic_world import KinematicWorld, play
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"dry run needs yam_arena, scipy and inspect_robots: {error}")


def run(reference=None, steps=fixture.MAX_STEPS, **world_options):
    reference, world = reference or scripted.TillReference(), KinematicWorld(**world_options)
    result, trace = play(reference, world, steps)
    return reference, world, result, trace


class DemoDryRunTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference, cls.world, cls.result, cls.trace = run(clearance=clearance)

    def test_full_score_through_the_real_scorer(self):
        self.assertEqual(self.result["score"], 1.)
        self.assertTrue(self.result["note_under_hold_down"] and self.result["note_flat"] and self.result["drawer_shut"])
        self.assertLess(abs(self.result["hold_down_deg"] - np.degrees(m.HOLD_ON_NOTE)), .5)
        low, high = self.result["note_side_mm"]
        self.assertGreater(low, -1000 * m.COMPARTMENT_W / 2)
        self.assertLess(high, 1000 * m.COMPARTMENT_W / 2)
        near, far = self.result["note_ends_in_drawer_mm"]
        self.assertGreater(near, 1000 * m.BILL_X[0] + 2.)
        self.assertLess(far, 1000 * m.BILL_X[1] - 2.)

    def test_fits_the_run_limit(self):
        self.assertLess(len(self.trace), fixture.MAX_STEPS / 2)

    def test_every_target_is_reachable(self):
        self.assertLess(self.reference.worst_ik, .001)
        for i in (0, 7):
            self.assertTrue(np.all(self.trace[:, i:i + 6] >= ik.LOWER + np.radians(10.)))
            self.assertTrue(np.all(self.trace[:, i:i + 6] <= ik.UPPER - np.radians(10.)))
            step = np.abs(np.diff(self.trace[:, i:i + 6], axis=0)).max(axis=0)
            self.assertLess(step.max(), .12)                     # rad per 50 ms step

    def test_both_grasps_held_at_the_first_attempt(self):
        events = {row["event"]: row for row in self.reference.log["events"]}
        self.assertTrue(events["hold-down tab grasp"]["held"] and events["hold-down tab grasp"]["attempt"] == 0)
        self.assertTrue(events["note grasp"]["held"] and events["note grasp"]["attempt"] == 0)
        self.assertGreater(events["hold-down raised"]["deg"], 28.)

    def test_clearances(self):
        """Arm collision hulls against the drawer's solids, and arm against arm."""
        self.assertGreater(self.world.min_arm_gap, .012)
        for (arm, name), (gap, tick) in self.world.min_gap.items():
            if (arm, name) == ("right", "front_panel"):          # the push itself
                continue
            self.assertGreater(gap, .003, f"{arm} arm and {name} at tick {tick}")
        self.assertGreater(self.world.peak["note_to_hold_down_mm"], 15.)   # the carried note under the raised arm

    def test_order_of_phases(self):
        names = [name for _, name in self.reference.phases]
        self.assertEqual(names[:11], ["reach hold-down", "lift hold-down", "reach note", "draw note off the case",
                                      "carry note over the till", "lay note under the hold-down", "let go of note",
                                      "let hold-down down", "reach drawer front", "push drawer shut", "away"])


class RobustnessTest(unittest.TestCase):
    def test_drooping_shoulders_and_slow_arms(self):
        _, _, result, _ = run(sag=.03, arm_lag_s=.2)
        self.assertEqual(result["score"], 1.)

    def test_note_hanging_to_one_side_is_centred_before_it_goes_in(self):
        _, _, result, _ = run(note_y_error=.004)
        self.assertEqual(result["score"], 1.)
        low, high = result["note_side_mm"]
        self.assertLess(max(-low, high), 1000 * m.COMPARTMENT_W / 2)


class ShortcutsTest(unittest.TestCase):
    """Parts of the job left out, played in the same stand-in world: none reaches full marks."""

    def test_drawer_left_open(self):
        class NoPush(scripted.TillReference):
            def shut_drawer(self):
                yield from self.home(("left", "right"), 2.)
                yield from self.hold(3.)

        _, _, result, _ = run(NoPush(), steps=900)
        self.assertEqual(result["score"], .75)

    def test_hold_down_never_let_down(self):
        class KeepUp(scripted.TillReference):
            def let_go(self):
                yield from self.grip("right", scripted.NOTE_OPEN, .5)
                yield from self.move(1.2, {"right": (self.pose["right"][0] - scripted.R_NOTE[:, 2] * .07, scripted.R_NOTE)})

            def shut_drawer(self):
                yield from self.hold(4.)

        _, _, result, _ = run(KeepUp(), steps=800)
        self.assertEqual(result["score"], .5)
        self.assertFalse(result["note_under_hold_down"])

    def test_hold_down_never_lifted(self):
        """The negative control that run.py also runs in the simulator (TILL_DEMO_VARIANT=no_lift): the right hand
        alone does exactly what it does in the demonstration. The note ends on top of the hold-down, not under it."""
        reference, world, result, _ = run(scripted.TillReference("no_lift"))
        self.assertEqual([name for _, name in reference.phases][:4],
                         ["reach note", "draw note off the case", "carry note over the till", "lay note under the hold-down"])
        self.assertLessEqual(result["score"], .5)
        self.assertFalse(result["note_under_hold_down"])

    def test_only_shutting_the_drawer(self):
        class OnlyPush(scripted.TillReference):
            def sequence(self):
                self.pose["right"] = None
                yield from self.first("right", (.16, scripted.PUSH_Y, .16), scripted.R_NOTE, scripted.SEED["note"], 2.)
                yield from self.shut_drawer()
                while True:
                    yield self.emit()

        _, world, result, _ = run(OnlyPush(), steps=500)
        self.assertLess(world.s, .002)
        self.assertEqual(result["score"], 0.)


class ScriptTest(unittest.TestCase):
    def test_script_moves_nothing_but_the_arms(self):
        source = Path(scripted.__file__).read_text()
        for word in ("set_transforms", "set_dof", "body_view", "torch", "apply_forces", "write_pose"):
            self.assertNotIn(word, source)
        code = scripted.TillReference.__init__.__code__
        self.assertEqual(code.co_varnames[:code.co_argcount], ("self", "variant"))     # no environment handle


if __name__ == "__main__":
    unittest.main()
