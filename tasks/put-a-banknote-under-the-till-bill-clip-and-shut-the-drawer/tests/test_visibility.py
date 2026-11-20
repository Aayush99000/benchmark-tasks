"""What the policy cameras see: sizes in the 320 x 176 overhead image, as stated in task.json."""

import unittest

import conftest  # noqa: F401

try:
    import camera_views as cameras
    from rdb_till_note import till_model as m
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"needs yam_arena: {error}")


class OverheadTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.view = cameras.overhead()
        cls.x0 = m.FRONT_OPEN_X

    def test_everything_is_in_view_from_the_first_frame(self):
        for point in ((m.NOTE_START_X[0], m.NOTE_START_Y, m.CASE_H), (m.NOTE_START_X[1], m.NOTE_START_Y, m.CASE_H),
                      (self.x0, -m.PANEL_W / 2, m.PANEL_Z[1]), (self.x0, m.PANEL_W / 2, m.PANEL_Z[1]),
                      (m.CASE_FRONT_X + m.CASE_D, 0., m.CASE_H), tuple(m.tab_centre(m.HOLD_ON_STACK))):
            self.assertTrue(self.view.in_view(point), point)

    def test_sizes_in_pixels(self):
        y, z = m.NOTE_START_Y, m.CASE_H
        note = (self.view.length((m.NOTE_START_X[0], y, z), (m.NOTE_START_X[1], y, z)),
                self.view.length((m.NOTE_START_X[0], y - m.NOTE_W / 2, z), (m.NOTE_START_X[0], y + m.NOTE_W / 2, z)))
        self.assertGreater(note[0], 20.)                           # task.json: "about 23 x 13 pixels"
        self.assertGreater(note[1], 11.)
        stack_x = self.x0 + m.STACK_X
        stack = (self.view.length((stack_x - m.NOTE_L / 2, 0., m.STACK_TOP_Z), (stack_x + m.NOTE_L / 2, 0., m.STACK_TOP_Z)),
                 self.view.length((stack_x, -m.NOTE_W / 2, m.STACK_TOP_Z), (stack_x, m.NOTE_W / 2, m.STACK_TOP_Z)))
        self.assertGreater(stack[0], 22.)
        self.assertGreater(stack[1], 11.)
        gap = self.view.length((self.x0 + m.BILL_X[0], -m.COMPARTMENT_W / 2, m.RIM_Z),
                               (self.x0 + m.BILL_X[0], m.COMPARTMENT_W / 2, m.RIM_Z))
        self.assertGreater(gap, 13.)
        arm = (self.view.length(m.hold_point(m.HOLD_ON_STACK, 0., 0.), m.hold_point(m.HOLD_ON_STACK, m.ARM_L, 0.)),
               self.view.thickness(m.hold_point(m.HOLD_ON_STACK, .05, 0.), (0., 1., 0.), m.ARM_W))
        self.assertGreater(arm[0], 16.)
        # The hold-down is narrow in this view, as task.json says: about 3 pixels. It is what it is on the real till.
        self.assertGreater(arm[1], 2.5)
        self.assertLess(arm[1], 4.)


if __name__ == "__main__":
    unittest.main()
