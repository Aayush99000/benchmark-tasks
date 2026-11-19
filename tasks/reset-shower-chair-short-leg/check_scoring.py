"""CPU regression checks for timing, release, alternate heights and resets.
Run with the simulator Python environment: python check_scoring.py
These checks do not replace physical simulation trials.
"""
from pathlib import Path
import numpy as np
from yam_arena import task_file

t = task_file.load(Path(__file__).with_name("task.py")).module


def state(height=17, offset=(0, 0, 0), button=0.0):
    joints = {}
    for name in t.LEGS:
        joints[f"leg_{name}_joint"] = (height - t.START_SETTING[name]) * t.PITCH
        joints[f"btn_{name}_joint"] = button
    rotation = t._rot((0, 0, 1), t.YAW)
    points = np.vstack([t.TIP_POINTS[n] + joints[f"leg_{n}_joint"] * t.LEG_DOWN[n] for n in t.LEGS])
    low = float((points @ rotation.T)[:, 2].min())
    pos = np.array([t.CX, 0, t.TOP - low]) + offset
    return t.m.PartState(pos, np.array(t._xyzw(rotation)), np.zeros(3), np.zeros(3), pos, pos, joints)


def main():
    t._tips = np.full((24, 3), 5.0)
    for height in range(14, 22):
        s = state(height)
        t.TRIAL.reset()
        for _ in range(59):
            t.TRIAL.update(s, 0.05)
            before = t.TRIAL.hold
            # Arena calls success and score; repeated readers must not alter time.
            for _ in range(5):
                assert not t.success({})
                assert t.score({}) == 0.5
            assert t.TRIAL.hold == before
        t.TRIAL.update(s, 0.05)
        assert t.success({}) and t.score({}) == 1.0
    s = state()
    t._tips = s.to_world([[0, 0, t.SEAT_TOP]] * 24)
    t.TRIAL.update(s, 0.05)
    assert t.score({}) == 0.5 and not t.success({}) and t.TRIAL.hold == 0
    t._tips = np.full((24, 3), 5.0)
    t.TRIAL.update(state(button=-0.004), 0.05)
    assert t.score({}) == 0
    t.TRIAL.update(state(offset=(0, 0, 0.03)), 0.05)
    assert t.score({}) == 0.5  # corrected legs, still held above the surface
    t.TRIAL.update(state(offset=(0, 0, -0.01)), 0.05)
    assert t.score({}) == 0  # cannot earn credit through the work surface
    t.TRIAL.update(state(offset=(2, 0, 0)), 0.05)
    assert t.score({}) == 0
    t._reset_latches()
    assert t.score({}) == 0 and not t.success({}) and t.TRIAL.hold == 0
    assert t._tips is None and all(t._latched.values())
    print("PASS: timing, pure reads, all eight equal heights, release, unlatching, support, bounds and reset")


if __name__ == "__main__":
    main()
