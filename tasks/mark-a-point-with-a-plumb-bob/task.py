"""On the station table (the worktop): a wall shelf fixed 45.7 cm (18 in) above it on a backboard, with a hole near
its front edge. A plumb line comes down through the hole, tied off on top, with a 16 oz brass plumb bob on its end:
the bob's point hangs 6 mm above the worktop. A carpenter's pencil lies in a slanted rest on the right.

The job is to mark the worktop directly below the hole (where a pipe or cable dropped through it would land). The
bob shows that point only while it hangs free and still, and it starts swinging: 2 cm either side of the point, a
swing that would take many minutes to die away by itself. So the bob has to be steadied by hand and let go without
setting it off again, and then the pencil's point has to be brought in under the bob's point (6 mm of room, so the
pencil comes in at a slant) to make one mark, touching neither the bob nor its line.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import plumbkit as K  # noqa: E402
from plumbkit import Bob, Dot, Fixed, Pencil, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Use the plumb bob to mark the worktop directly below the hole in the shelf: steady the swinging bob, "
               "let it hang free and still, make one pencil mark on the worktop under its point, and lift the pencil away.")

# -- layout (m; x forward from the arms, y left, heights above the worktop) ---------------------------------------------
LOT = os.environ.get("PLUMB_LOT", "A")
PLUMB_XY = np.array({"A": (0.320, 0.000), "B": (0.300, 0.030)}[LOT])    # the point on the worktop under the hole
SHELF_UNDER = 0.457                      # the shelf's underside: 18 in above the worktop
SHELF_T = 0.018                          # 18 mm plywood
SHELF_W, SHELF_DEPTH = 0.30, 0.20        # across (y), front to back (x)
HOLE_IN = 0.060                          # the hole, this far in from the shelf's front edge
POINT_GAP = 0.006                        # the hanging bob's point above the worktop
DROP = SHELF_UNDER - POINT_GAP - K.BOB_L  # the line, from the hole's lower edge to the bob's cap
START_OUT = 0.020                        # the bob's point starts this far from plumb (toward the arms), at rest
START_ANGLE = math.asin(START_OUT / (DROP + K.BOB_L))

SLANT = math.radians(50.0)               # the pencil lies in its rest this steeply, point down and toward the middle
REST_POINT = np.array([0.240, -0.190, 0.012])     # where the pencil's point rests
UP_PENCIL = np.array([0.0, -math.cos(SLANT), math.sin(SLANT)])   # from the pencil's point along it to its end
PENCIL_QUAT = K.turn_x(math.pi / 2 - SLANT)
N_DOTS = 8
HIDDEN = np.array([0.30, 0.25, -0.05])   # unused pencil dots wait under the table

BOB, PENCIL = "bob", "pencil"

# -- scoring -----------------------------------------------------------------------------------------------------------
FULL, NEAR = 0.003, 0.010                # a mark this close to the point under the hole: full / half marks
STILL = 0.003                            # the bob is still while its point stays this close to plumb ...
STILL_S = 1.5                            # ... for this long (more than one swing), hanging free
CLEAR = K.BOB_R + 0.020                  # hanging free: every part of both grippers' fingers and palms this far from
                                         # the centre line of the bob and its line (2 cm clear of the bob's body)
ON_TOP = 0.0003                          # the pencil's point is on the worktop: its tip this close to the surface ...
MARK_FORCE = 0.3                         # ... and it marks only while pressed on it this hard (N, along the pencil):
                                         # a light pencil stroke, about the weight of 30 g. An estimate, not a measurement
LIFT = 0.004                             # the point this high above the worktop: lifted off
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)
DOT_EVERY = 0.002                        # a dragged pencil leaves another dot every 2 mm

PLY, PINE, BRASS, DARK = (0.80, 0.66, 0.44), (0.87, 0.74, 0.53), (0.78, 0.62, 0.25), (0.22, 0.22, 0.24)


def pivot() -> np.ndarray:
    """The point the line hangs from: the lower edge of the hole (world, table frame z)."""
    return np.array([PLUMB_XY[0], PLUMB_XY[1], TOP + SHELF_UNDER])


def plumb_point() -> np.ndarray:
    """The point on the worktop directly below the hole."""
    return np.array([PLUMB_XY[0], PLUMB_XY[1], TOP])


def pencil_start() -> np.ndarray:
    """The pencil's point at the start, in its rest."""
    return REST_POINT + [0.0, 0.0, TOP] + 0.0005 * UP_PENCIL


def _rest_part(name: str, local, size, color=PINE) -> Fixed:
    """A piece of the pencil's rest, given in the resting pencil's own frame (x its width, y its thickness, z along
    it from its point)."""
    at = REST_POINT + [0.0, 0.0, TOP] + K._rotation(PENCIL_QUAT) @ np.asarray(local, float)
    return Fixed(name, pos=tuple(float(v) for v in at), size=tuple(size), color=color, quat=PENCIL_QUAT,
                 static_friction=0.5, dynamic_friction=0.4)


def _objects() -> list:
    x0 = PLUMB_XY[0] - HOLE_IN                                        # the shelf's front edge
    shelf = Fixed("shelf", pos=(x0 + SHELF_DEPTH / 2, PLUMB_XY[1], TOP + SHELF_UNDER + SHELF_T / 2),
                  size=(SHELF_DEPTH, SHELF_W, SHELF_T), color=PLY)
    board_h = SHELF_UNDER + SHELF_T
    back = Fixed("backboard", pos=(x0 + SHELF_DEPTH + SHELF_T / 2, PLUMB_XY[1], TOP + board_h / 2),
                 size=(SHELF_T, SHELF_W, board_h), color=PLY)
    # the line's end, tied round a brass screw beside the hole on the shelf's top
    cleat = Fixed("cleat", pos=(PLUMB_XY[0] + 0.012, PLUMB_XY[1], TOP + SHELF_UNDER + SHELF_T + 0.004),
                  size=(0.008, 0.008, 0.008), color=BRASS, metallic=0.8, roughness=0.3)
    knot = Fixed("knot", pos=(PLUMB_XY[0] + 0.004, PLUMB_XY[1], TOP + SHELF_UNDER + SHELF_T + 0.0012),
                 size=(0.018, 0.003, 0.0024), color=(0.95, 0.38, 0.08))
    bob = Bob(BOB, pos=tuple(float(v) for v in pivot()), drop=DROP, size=(2 * K.BOB_R, 2 * K.BOB_R, K.BOB_L))
    half_w = K.PENCIL_W / 2 + 0.00075                                 # the groove: the pencil's width and 1.5 mm
    rest = [_rest_part("rest_ramp", (0.0, -K.PENCIL_T / 2 - 0.003, 0.0565), (2 * half_w + 0.006, 0.006, 0.117)),
            _rest_part("rest_wall_a", (half_w + 0.0015, 0.0005, 0.030), (0.003, 0.008, 0.060)),
            _rest_part("rest_wall_b", (-half_w - 0.0015, 0.0005, 0.030), (0.003, 0.008, 0.060)),
            _rest_part("rest_stop", (0.0, -0.0015, -0.002), (2 * half_w + 0.006, 0.016, 0.004)),
            _rest_part("rest_brace", (0.0, -K.PENCIL_T / 2 - 0.006 - 0.020, 0.085), (2 * half_w + 0.006, 0.040, 0.030)),
            Fixed("rest_post", pos=(REST_POINT[0], REST_POINT[1] - 0.085, TOP + 0.0225), size=(0.0225, 0.030, 0.045),
                  color=PINE),
            Fixed("rest_foot", pos=(REST_POINT[0], REST_POINT[1] - 0.045, TOP + 0.002), size=(0.050, 0.130, 0.004),
                  color=PINE)]
    pencil = Pencil(PENCIL, pos=tuple(float(v) for v in pencil_start()), quat=PENCIL_QUAT,
                    size=(K.PENCIL_W, K.PENCIL_T, K.PENCIL_L))
    dots = [Dot(f"dot_{k}", pos=tuple(float(v) for v in HIDDEN + [0.0, 0.0, TOP]), size=(0.003, 0.003, 0.0002))
            for k in range(N_DOTS)]
    return [shelf, back, cleat, knot, bob, *rest, pencil, *dots]


OBJECTS = _objects()


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, calm_s=0.0, steadied=False, marks=[], down=None, lifted=True, dots=0, last_dot=None,
                  dev=START_OUT, clear=True, point=None, bob=None, force=0.0, press=0.0, lowest=1.0, touch=0.0)


_clear()


def marking(height: float, over_worktop: bool, force: float) -> bool:
    """The pencil makes a mark now: its point is in contact with the worktop (``height`` above the surface within
    ON_TOP, somewhere over the worktop) and is pressed on it with at least MARK_FORCE (``force``: N along the pencil,
    read from the lead's spring). A point held just above the worktop, or resting on it lighter than that, marks
    nothing."""
    return bool(over_worktop and height <= ON_TOP and force >= MARK_FORCE)


def _still_now() -> bool:
    return _STATE["calm_s"] >= STILL_S - 1e-9


@on_reset
def _reset(environment) -> None:
    _clear()
    K.write_joints(environment, BOB, {"top_y": START_ANGLE})          # drawn 2 cm toward the arms, about to be let go
    for k in range(N_DOTS):
        K.write_pose(environment, f"dot_{k}", HIDDEN + [0.0, 0.0, TOP])


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    dt = float(environment.unwrapped.step_dt)
    st = object_states(environment, [BOB, PENCIL])
    bob = np.asarray(st[BOB].pos, float)
    s["bob"] = bob
    s["dev"] = float(np.linalg.norm(bob[:2] - PLUMB_XY))
    s["clear"] = all(K.segment_distance(p, pivot(), bob) >= CLEAR for pts in K.gripper_points(environment) for p in pts)
    s["calm_s"] = s["calm_s"] + dt if (s["clear"] and s["dev"] <= STILL) else 0.0
    if _still_now():
        s["steadied"] = True

    point = np.asarray(st[PENCIL].pos, float)
    s["point"] = point
    height = float(point[2] - TOP)
    force = K.lead_force(st[PENCIL].joints["lead"])
    s["force"] = force
    on_top = TABLE_X[0] < point[0] < TABLE_X[1] and TABLE_Y[0] < point[1] < TABLE_Y[1]
    if on_top:
        s["lowest"] = min(s["lowest"], height)
        if height <= ON_TOP:
            s["touch"] = max(s["touch"], force)
    if height >= LIFT:
        s["lifted"], s["down"] = True, None
    elif marking(height, on_top, force):
        s["press"] = max(s["press"], force)
        d = float(np.linalg.norm(point[:2] - PLUMB_XY))
        if s["down"] is None:                                         # the point has come down: a new mark
            s["down"] = {"d": d, "x": float(point[0]), "y": float(point[1]), "plumbed": _still_now(), "step": s["steps"]}
            s["marks"].append(s["down"])
            s["lifted"], s["last_dot"] = False, None
        else:                                                         # still down: the mark reaches as far as it is dragged
            s["down"]["d"] = max(s["down"]["d"], d)
        if s["dots"] < N_DOTS and (s["last_dot"] is None or np.linalg.norm(point[:2] - s["last_dot"]) >= DOT_EVERY):
            K.write_pose(environment, f"dot_{s['dots']}", [point[0], point[1], TOP + 0.0002])
            s["dots"] += 1
            s["last_dot"] = point[:2].copy()


# -- scoring --------------------------------------------------------------------------------------------------------
def score_from(marks, lifted: bool, still: bool, steadied: bool) -> float:
    """Pure scoring rule. ``marks``: one entry per time the pencil's point was pressed on the worktop, with ``d`` (m, how
    far from the point under the hole the mark reaches) and ``plumbed`` (the bob had been hanging free and still for
    STILL_S when it was made). ``lifted``: the pencil's point is off the worktop now. ``still``: the bob is hanging
    free and still now. ``steadied``: it has done so at some time in the trial."""
    good = [m for m in marks if m["plumbed"] and m["d"] <= FULL]
    near = [m for m in marks if m["plumbed"] and m["d"] <= NEAR]
    if good and len(good) == len(marks) and lifted and still:
        return 1.0
    if near:
        return 0.5
    return 0.25 if steadied else 0.0


def score(objects) -> float:
    """A mark is made only while the pencil's point is on the worktop and pressed on it with at least 0.3 N (see
    ``marking``). 1.0: every pencil mark on the worktop lies within 3 mm of the point under the hole (at least one
    mark), each made while the bob had been hanging free and still (its point within 3 mm of plumb, both grippers'
    fingers at least 2 cm clear of it and its line) for 1.5 s, the pencil lifted off, and the bob still hanging free
    and still. 0.5: a mark made with the bob so hanging lies within 10 mm of the point. 0.25: the bob was steadied
    (hung free and still for 1.5 s) at some time. Else 0."""
    s = _STATE
    return score_from(s["marks"], s["lifted"], _still_now(), s["steadied"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    s = _STATE

    def mm(v):
        return None if v is None else [round(1000 * float(x), 1) for x in v]

    return {"score": score(objects), "bob_off_plumb_mm": round(1000 * s["dev"], 2),
            "bob_point_mm": mm(None if s["bob"] is None else s["bob"] - plumb_point()),
            "calm_s": round(s["calm_s"], 2), "steadied": s["steadied"], "hands_clear": s["clear"],
            "pencil_point_mm": mm(None if s["point"] is None else s["point"] - plumb_point()), "lifted": s["lifted"],
            "pencil_force_N": round(s["force"], 3), "hardest_marking_press_N": round(s["press"], 3),
            "pencil_lowest_mm": round(1000 * s["lowest"], 2), "hardest_press_on_worktop_N": round(s["touch"], 3),
            "marks": [{"mm": round(1000 * m["d"], 2), "plumbed": m["plumbed"], "step": m["step"]} for m in s["marks"]]}


def demo(objects=None, seed=None):
    return {}
