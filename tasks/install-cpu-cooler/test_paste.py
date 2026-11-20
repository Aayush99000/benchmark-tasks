"""Checks of the paste rule (task.paste_watch) on made-up cooler poses, without Isaac Sim. Run inside the yam_arena
environment:  python test_paste.py"""

import math

import numpy as np

import task
from scenekit import BodyState

SEATED = np.asarray(task.SEATED_POS, float)
LOCKED = {"leg_pos_slide_joint": -task.clip.HOOK_DROP, "leg_pos_flex_joint": 0.0, task.clip.LEVER: task.clip.LEVER_SPAN}


def pose(dx=0.0, dy=0.0, dz=0.0, tilt_deg=0.0, joints=None) -> BodyState:
    """The cooler at the seated pose moved by (dx, dy, dz) m and tilted about the clip's line."""
    yaw, a = task.COOLER_YAW, math.radians(tilt_deg)
    qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    qx = np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    p = SEATED + [dx, dy, dz]
    return BodyState(pos=p, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p,
                     joints=dict(joints or LOCKED))


def run(path) -> float:
    task._STATE.update(free=True, anchor=None, slid=0.0, lifted=False)
    for c in path:
        task.paste_watch(c)
    return task.score({"cooler": path[-1]})


def lower(dx=0.0, dy=0.0):
    return [pose(dx, dy, z) for z in (0.050, 0.020, 0.005, 0.002, 0.0006, 0.0)]


def main() -> None:
    ok = run(lower(0.0004, 0.0001))
    assert ok == 1.0, ok
    # set down 1.2 mm off and nudged back while pressing the tabs (the demo): still fine
    assert run(lower(0.0012) + [pose(0.0006), pose(0.0004, 0.0, 0.0013, 3.9), pose(0.0004)]) == 1.0
    # slid 15 mm into place after touching: 0
    assert run(lower(0.015) + [pose(0.008), pose(0.0)]) == 0.0
    # slid 10.5 mm: 0; 6 mm and 9.5 mm (shuffled into place): fine
    assert run(lower(0.0105) + [pose(0.0)]) == 0.0
    assert run(lower(0.006) + [pose(0.003), pose(0.0)]) == 1.0
    assert run(lower(0.0, 0.0095) + [pose(0.0)]) == 1.0
    # lifted 10 mm after touching and set back in the same place: 0
    assert run(lower() + [pose(dz=0.010), pose()]) == 0.0
    # lifted 2 mm (under the 3 mm limit) and set back: fine
    assert run(lower() + [pose(dz=0.002), pose()]) == 1.0
    # tipped 4 degrees by a tab press (one edge up about 1.4 mm): the low edge stays on, not lifted
    assert run(lower() + [pose(dz=0.0013, tilt_deg=3.9), pose()]) == 1.0
    # raised 2 mm (under the lift limit), slid 15 mm out and back while hovering, set down where it touched: 0
    assert run(lower() + [pose(0.0, 0.0, 0.002), pose(0.015, 0.0, 0.002), pose(0.0, 0.0, 0.002), pose()]) == 0.0
    # raised 1.5 mm, slid 6 mm while hovering, set down: fine
    assert run(lower(0.006) + [pose(0.006, 0.0, 0.0015), pose(0.0, 0.0, 0.0015), pose(0.0)]) == 1.0
    # touched far off the processor (no overlap) and carried over: not a touch
    assert run([pose(0.05, 0.0, 0.0)] + lower()) == 1.0
    # dragged on with the base over the processor's edge, then slid in: 0
    assert run([pose(0.030, 0.0, 0.0)] + [pose(0.015), pose(0.0)]) == 0.0
    print("paste rule: all checks pass")


if __name__ == "__main__":
    main()
