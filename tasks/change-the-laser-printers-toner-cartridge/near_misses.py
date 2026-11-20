"""Near misses and unintended strategies for the toner cartridge swap.

The score rests on a history (drops, tips, the door's catch, the let-go, the 1 s still hold, unknown states), so each
case drives the scorer's own recorder (`task.record`, which the simulator calls every control step) through a
sequence of object states and hand points, then reads the score."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task  # noqa: E402

DT = 0.05
TOP = task.TOP
FAR = np.array([[0.05, 0.45, TOP + 0.30], [0.05, -0.45, TOP + 0.30]])
FAILS: list[str] = []
OLD_ON_TABLE = (task.OLD_DOWN[0], task.OLD_DOWN[1], TOP)
SEATED = task.SEATED


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def world(old=OLD_ON_TABLE, new=SEATED, door=task.SHUT, old_q=None, new_q=None, old_v=None, new_v=None):
    return {
        "printer": task.PRINTER.state(joints={"door": door}),
        "old_cartridge": task.OLD.state(pos=old, quat_xyzw=old_q, lin_vel=old_v),
        "new_cartridge": task.NEW.state(pos=new, quat_xyzw=new_q, lin_vel=new_v),
    }


def run(n, state, hands=FAR, reset=True):
    if reset:
        task.reset_record()
    for k in range(n):
        s = state(k) if callable(state) else state
        h = hands(k) if callable(hands) else hands
        task.record(s, h, DT)


def case(label, setup, want):
    setup()
    got = task.score(None)
    if abs(got - want) > 1e-9:
        FAILS.append(f"{label}: got {got}, want {want}")
    print(f"{'ok  ' if abs(got - want) <= 1e-9 else 'FAIL'} {label}: {got}")


def near(part, st, gap):
    """A hand point ``gap`` metres above the top of a cartridge's grip bar (plus the far points)."""
    p = np.asarray(st.pos, float)
    top = p + np.array([task.BAR_X, 0.0, task.BAR_Z[1] + gap])
    return np.vstack([FAR, top])


def start():
    return {"printer": task.PRINTER.state(), "old_cartridge": task.OLD.state(), "new_cartridge": task.NEW.state()}


GOOD = world()
OPEN_DONE = world(door=0.0)

HIST_CASES = [
    ("start layout, nothing done", lambda: run(30, start()), 0.0),
    ("full swap, door latched, hands clear, still 1.2 s", lambda: run(24, GOOD), 1.0),
    ("full swap but only 0.5 s still", lambda: run(10, GOOD), 0.75),
    ("full swap, a hand 3 mm from the door", lambda: run(
        24, GOOD, hands=lambda k: np.vstack([FAR, [task.XF - task.T - 0.003, 0.0, TOP + 0.12]])), 0.75),
    ("door 3 degrees short of shut, never caught", lambda: run(24, world(door=task.SHUT - math.radians(3))), 0.5),
    ("door caught once, now 1 degree off shut (held by the catch)", lambda: (
        run(5, GOOD), run(24, world(door=task.SHUT - math.radians(1)), reset=False)), 1.0),
    ("new cartridge seated, door open", lambda: run(24, OPEN_DONE), 0.5),
    ("new cartridge seated but still held (hand 2 mm off)", lambda: run(
        24, OPEN_DONE, hands=lambda k: near(task.NEW, OPEN_DONE["new_cartridge"], 0.002)), 0.25),
    ("new cartridge 4.0 mm short of seated (limit)", lambda: run(
        24, world(door=0.0, new=(SEATED[0] - 0.004, 0.0, SEATED[2]))), 0.5),
    ("new cartridge 4.5 mm short of seated", lambda: run(
        24, world(door=0.0, new=(SEATED[0] - 0.0045, 0.0, SEATED[2]))), 0.25),
    ("new cartridge seated but turned 4 degrees", lambda: run(24, world(door=0.0, new_q=qz(4))), 0.25),
    ("new cartridge left on the open door", lambda: run(
        24, world(door=0.0, new=(task.XF - 0.07, 0.0, TOP + task.HB))), 0.25),
    ("old out but put on the open door", lambda: run(
        24, world(door=0.0, old=(task.XF - 0.07, 0.0, TOP + task.HB))), 0.0),
    ("old out, on the table, still held", lambda: run(
        24, OPEN_DONE, hands=lambda k: near(task.OLD, OPEN_DONE["old_cartridge"], 0.003)), 0.0),
    ("old on the table 5 mm in front of the open door (clear)", lambda: run(
        24, world(door=0.0, old=(task.XF - task.HP - task.T - 0.005 - task.CD / 2, 0.0, TOP), new=task.NEW_START)),
     0.25),
    ("old on the table, its edge 1 mm over the printer's side", lambda: run(
        24, world(door=0.0, old=(0.30, task.W / 2 + task.CL / 2 - 0.001, TOP), new=task.NEW_START)), 0.0),
    ("old propped up 8 degrees on the table", lambda: run(
        24, world(old=(OLD_ON_TABLE[0], OLD_ON_TABLE[1], TOP + 0.002), old_q=qx(8))), 0.0),
    ("old standing 10 mm above the table (on something)", lambda: run(
        24, world(old=(OLD_ON_TABLE[0], OLD_ON_TABLE[1], TOP + 0.010))), 0.0),
    ("swapped the wrong way: new on the table, old back in the bay", lambda: run(
        24, world(old=SEATED, new=OLD_ON_TABLE)), 0.0),
    ("old dropped once (0.8 m/s down), then everything right", lambda: (
        run(1, world(old_v=(0.0, 0.0, -0.8))), run(24, GOOD, reset=False)), 0.0),
    ("new tipped 35 degrees once, then everything right", lambda: (
        run(1, world(new=(0.1, -0.3, TOP + 0.05), new_q=qx(35))), run(24, GOOD, reset=False)), 0.0),
    ("new tipped 29 degrees once while carried, then everything right", lambda: (
        run(1, world(new=(0.1, -0.3, TOP + 0.08), new_q=qx(29))), run(24, GOOD, reset=False)), 1.0),
    ("hand state unknown for one step", lambda: (
        run(1, GOOD, hands=None), run(24, GOOD, reset=False)), 0.0),
    ("object state unknown for one step", lambda: (run(1, None), run(24, GOOD, reset=False)), 0.0),
]

for _label, _setup, _want in HIST_CASES:
    case(_label, _setup, _want)
print(f"{len(HIST_CASES) - len(FAILS)}/{len(HIST_CASES)} as expected")
if FAILS:
    raise SystemExit(f"{len(FAILS)} near-miss case(s) failed: {FAILS}")

run(24, GOOD)  # leave a valid full-score history for check.py's end-state case
CASES = [
    ("end state of a valid history: swapped, door latched, hands clear, still", {}, 1.0, True),
]
