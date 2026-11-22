"""The scorer on hand-built states, with no simulator: the ladder, the full result, the 2 s hold with its pull test, the latched failures, and the
screws' torque model (resistance, clutch click, backing off). Runs with pytest, or as a script.

Adapted from the original task's check_score.py (MIT).
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import types
from dataclasses import dataclass


sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # the harness is not needed: only its names, so that mechkit's meshes can be built
sys.modules["yam_arena"].__path__ = []
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")


@dataclass(frozen=True)
class Box:  # the harness's shape, its fields only
    name: str
    pos: tuple
    color: tuple = (0.5, 0.5, 0.5)
    mass: float | None = None
    yaw: float = 0.0
    size: tuple = (0.04, 0.04, 0.04)


@dataclass(frozen=True)
class ObjectState:
    pos: object
    quat_xyzw: object
    lin_vel: object
    ang_vel: object
    low: object
    high: object


tf.TABLE_TOP, tf.Box, tf.ObjectState = {"station": 0.0}, Box, ObjectState
sys.path.insert(0, str(pathlib.Path(__file__).parent / "kit"))
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("fit_comb_busbar.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
t.m.push = lambda *a, **kw: None  # the pull test's force: here the comb's height is given

bad = 0


def check(label, got, want):
    global bad
    ok = got == want if isinstance(want, bool) else abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {label}: {got} (want {want})")


OPEN = t.TURNS_OPEN  # a screw backed out far enough for a pin
SHUT = 0.0


def state(home=True, down=(True, True, True), snug=(False, False, False), hands=0.08, driver_free=True, fell=False, out=(SHUT, SHUT, SHUT), comb_z=0.2, gaps=(0.0005,) * 3):
    return {"home": home, "down": list(down), "snug": list(snug), "hands": hands, "driver_free": driver_free, "fell": fell, "out": list(out), "comb_z": comb_z,
            "gaps": list(gaps)}


# the ladder
for label, c, opened, want in (("nothing done", state(home=False, down=(False,) * 3), 0, 0.0), ("one terminal opened", state(home=False, down=(False,) * 3), 1, 0.03),
                               ("the row opened", state(home=False, down=(False,) * 3), 3, 0.09), ("comb home, no screw down", state(down=(False,) * 3), 3, 0.34),
                               ("one terminal at 2 N m", state(down=(True, False, False)), 3, 0.44), ("two terminals at 2 N m", state(down=(True, True, False)), 3, 0.54),
                               ("three screws on the teeth, none clicked", state(down=(False,) * 3, snug=(True,) * 3), 3, 0.46),
                               ("two at 2 N m, one only snug", state(down=(True, True, False), snug=(False, False, True)), 3, 0.58),
                               ("three down, a hand still on the comb", state(hands=0.03), 3, 0.64)):
    check(f"ladder, {label}", t._ladder(c, opened), want)

# the full result, before its hold
for label, c, want in (("all done", state(), True), ("a fingertip 4.9 cm from the comb", state(hands=0.049), False),
                       ("the screwdriver held or its tip at a head", state(driver_free=False), False), ("one terminal short of 2 N m", state(down=(True, True, False), snug=(False, False, True)), False),
                       ("the comb not home", state(home=False), False)):
    check(f"complete, {label}", t._complete(c), want)


def run(states):
    """Step the trial through hand-built states, the comb's height following them (the pull test reads it)."""
    tr = t.TRIAL
    tr.reset()
    for c in states:
        t.now = lambda env, c=c: c
        tr.step(None)
    return tr


opened_row = (OPEN, OPEN, OPEN)
tr = run([state(down=(False,) * 3, home=False, out=opened_row)] + [state()] * 39)
check("39 steps at the full result: the ladder (0.64), not yet 1.0", tr.score, 0.64)
tr.step(None)
check("40 steps at the full result, the pull test passed: 1.0", tr.score, 1.0)
check("success() then", t.success(None), True)
pulled = [state(out=opened_row)] + [state(comb_z=0.2 + (0.002 if 12 <= k <= 25 else 0.0)) for k in range(60)]
tr = run(pulled)
check("the comb lifts 2 mm in the pull test: stays at the ladder", tr.score, 0.64)
check("success() then", t.success(None), False)
tr = run([state(out=opened_row), state(fell=True)] + [state()] * 50)
check("the comb or screwdriver off the table once: 0 for the rest", tr.score, 0.0)
tr = run([state(down=(True, True, True))] * 5)
check("the comb home, screws read down but no terminal ever opened: the comb only", tr.score, 0.25)
tr = run([state(home=False, down=(False,) * 3, gaps=(0.0036,) * 3)] + [state(home=False, down=(False,) * 3)] * 5)
check("the jaws forced open by the pins, no screw turned: nothing opened", tr.score, 0.0)

check("a jaw pushed open with its screw still at its furthest out: no clamp", t.may_clamp(4.0, 4.0), False)
check("a jaw pushed open with its screw 0.1 turn in: no clamp", t.may_clamp(3.9, 4.0), False)
check("a jaw held open as its screw is driven 1.3 turns back in: it clamps", t.may_clamp(2.7, 4.0), True)
# the torque model: what a screw resists with, and when the driver clicks
term = t.Terminals()
check("a screw turning free resists with the running torque", term.resistance(0), t.TORQUE_RUN)
term.bears[0], term.past[0] = True, t.RISE_DEG / 2
check("halfway up the rise past the tooth: halfway to the preset", term.resistance(0), round(t.TORQUE_RUN + (t.TORQUE_SET - t.TORQUE_RUN) / 2, 9))
term.past[0] = t.RISE_DEG + 5
check("past the rise: the driver's preset, where its clutch slips", term.resistance(0), t.TORQUE_SET)
check("the preset is the breakers' specified terminal torque", t.TORQUE_SET, 2.0)
check("the click needs the rise and the slip together (degrees)", t.RISE_DEG + t.CLICK_DEG, 45.0)
check("a full partial score below 1.0 (all clicked, a hand on the comb)", t._ladder(state(hands=0.03), 3), 0.64)

# the torque model stepped as in a trial: the driver's bit on the middle screw's head, turned in a few degrees a step against the clamped tooth
import math  # noqa: E402

import numpy as np  # noqa: E402


class _Driver:
    def __init__(self):
        self.roll = 0.0

    def rotation(self):  # the driver along +x, turned `roll` about it
        c, s = math.cos(self.roll), math.sin(self.roll)
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])

    def to_world(self, pts):  # its bit's tip on the middle screw's head face
        return np.array([t.head_point(1) + np.array([t.HEAD_FACE - t.head_point(1)[0] + 0.0002, 0.0, 0.0]) for _ in pts])


class _Row:
    def joint(self, name):
        return 0.0


drv, torques = _Driver(), []
t.m.state = lambda env, name: drv if name == "driver" else _Row()
t.m._joint = lambda env, part, link: (types.SimpleNamespace(set_joint_position_target=lambda *a, **k: None), 0)
t.m._tensor = lambda asset, v: v
t.m.twist = lambda env, part, tq: torques.append(np.array(tq))
term = t.Terminals()
term.bears[1] = True
term.step(None)  # the first step only reads the roll
clicked_at = None
for n in range(1, 30):
    drv.roll += math.radians(2.0)  # 2 degrees a step, tightening
    term.step(None)
    if term.torqued[1] and clicked_at is None:
        clicked_at = n
check("turning in against the tooth: the click at 45 degrees (step 23 at 2 degrees a step)", clicked_at, 23)
check("the torque on the driver at the click: the preset", round(float(np.linalg.norm(torques[22])), 6), t.TORQUE_SET)
check("it acts against the turn, about the driver's axis (-x)", bool(torques[22][0] < 0 and abs(torques[22][1]) < 1e-9), True)
check("early in the rise: below the preset", bool(np.linalg.norm(torques[2]) < t.TORQUE_SET), True)
drv.roll -= math.radians(2.0)  # the wrist springs back 2 degrees
term.step(None)
check("a 2 degree spring-back (torque within 10% of the preset): still clicked", term.torqued[1], True)
check("the spring-back leaves the screw clamped, and turns it no further out", term.bears[1] and term.out[1] == 0.0, True)
drv.roll -= math.radians(5.0)  # backed off
term.step(None)
check("backing the screw off undoes the click", term.torqued[1], False)
drv.roll -= math.radians(33.0)  # backed off past the jaw's contact: 30 - 2 - 5 - 33 = -10 degrees
term.step(None)
check("backed off past the contact: the jaw lets go", term.bears[1], False)
check("... and the rest of the turn backs the screw out (degrees)", round(term.out[1] * 360, 6), 10.0)
check("backing off: running torque against the loosening turn (+x)", bool(torques[-1][0] > 0 and abs(float(np.linalg.norm(torques[-1])) - t.TORQUE_RUN) < 1e-9), True)
n_before = len(torques)
term.step(None)  # not turning
check("a still driver: the torque taken off", bool(len(torques) == n_before + 1 and np.linalg.norm(torques[-1]) == 0.0), True)

print("all cases as the scoring text" if not bad else f"{bad} cases differ")


def test_scoring_text():
    assert bad == 0


if __name__ == "__main__":
    sys.exit(1 if bad else 0)
