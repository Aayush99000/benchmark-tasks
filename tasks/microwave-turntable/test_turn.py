"""Regression checks for a loaded turn graded from exterior markings."""
import unittest

import cv2
import numpy as np

from markers import PlanMarks
from scoring import LoadedTurn


class LoadedTurnTest(unittest.TestCase):
    def test_both_marks_return_from_different_initial_bearings(self):
        turn = LoadedTurn()
        for advance in range(360):
            self.assertFalse(turn.sample((350+advance) % 360, (92+advance) % 360, True))
        self.assertTrue(turn.sample(350, 92, True))

    def test_a_missing_mark_breaks_the_interval(self):
        marks = PlanMarks(320, 320, (21, 160, 299, 160))
        image = np.zeros((320, 320, 3), np.uint8)
        cv2.circle(image, (275, 160), 4, (0, 0, 255), -1)  # RGB blue tray stripe
        self.assertIsNone(marks.bearings(image))
        cv2.circle(image, (100, 160), 4, (255, 255, 0), -1)
        self.assertIsNotNone(marks.bearings(image))
        turn = LoadedTurn()
        for a in range(201):
            turn.sample(a, a+40, True)
        self.assertFalse(turn.sample(0, 0, False))
        for a in range(201, 401):
            self.assertFalse(turn.sample(a, a+40, True))

    def test_stall_reversal_slip_and_skipped_quadrant(self):
        for samples in ([(40, 40)]*11, [(33, 33)], [(51, 41)], [(135, 135)]):
            with self.subTest(samples=samples):
                turn = LoadedTurn()
                for a in range(41):
                    turn.sample(a, a, True)
                for a, b in samples:
                    self.assertFalse(turn.sample(a, b, True))
                self.assertGreater(turn.resets, 0)


if __name__ == "__main__":
    unittest.main()
