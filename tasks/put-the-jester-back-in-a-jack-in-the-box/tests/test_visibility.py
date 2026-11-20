"""What the policy cameras see of the toy at 320 x 176, by pinhole projection (README, "What the policy cameras see")."""

import numpy as np
import pytest

pytest.importorskip("yam_arena")

from camera_views import overhead, wrist  # noqa: E402
from yam_arena.constants import READY  # noqa: E402
from rdb_jack_in_box import fixed_scene as fixture  # noqa: E402
from rdb_jack_in_box import layout as L  # noqa: E402


def corners():
    h = L.HALF
    return [np.array((L.BOX_X + sx * h, sy * h, z)) for sx in (-1, 1) for sy in (-1, 1) for z in (0., L.RIM_Z)]


def sizes():
    top = overhead()
    head = L.head_centre(L.S_POPPED, 0.)
    lid_edge = L.lid_point(L.LID_OPEN, -L.SIDE)
    return {
        "box_in_view": all(top.in_view(p) for p in corners()),
        "head_in_view": top.in_view(head), "lid_edge_in_view": top.in_view(lid_edge),
        "box_px": top.length(corners()[0], corners()[4]),
        "head_px": top.thickness(head, (0., 1., 0.), 2 * L.HEAD_R),
        "lid_px": top.length(L.lid_point(L.LID_OPEN, 0., 0., -L.HALF), L.lid_point(L.LID_OPEN, 0., 0., L.HALF)),
        "left_wrist_sees_head": wrist("left", READY, fixture.BASES["left"]).in_view(head),
        "right_wrist_sees_head": wrist("right", READY, fixture.BASES["right"]).in_view(head),
    }


def test_toy_is_in_the_overhead_view_from_the_first_frame():
    s = sizes()
    assert s["box_in_view"] and s["head_in_view"] and s["lid_edge_in_view"]
    assert s["box_px"] > 20 and s["head_px"] > 10


if __name__ == "__main__":
    print({k: (round(v, 1) if isinstance(v, float) else v) for k, v in sizes().items()})
