"""On the station table, 25-40 cm out between the arms: a shop's countertop PIN pad, the size of an Ingenico Lane/3000
(83 x 180 mm), lying flat with its 15-key keypad at the near end and its screen at the far end. Pay by card: enter the
PIN 7 3 1 9 and press the green key, covering the keypad with the other hand so the person across the counter can't
see which keys are pressed.

A key registers when it is pushed past its click (0.8 mm of its 1.2 mm travel). The person across the counter is a
fixed eye point, 60 cm beyond the keypad and 45 cm above it; a press is hidden when every corner of the pressed key's
top is out of that eye's sight behind a gripper, at the moment it registers.

Run it:  inspect-robots run --embodiment yam_arena -E task=shielded_pin.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import Slider, body, box  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
CASE = (0.13, 0.13, 0.14)  # charcoal plastic
KEY_GREY = (0.78, 0.78, 0.76)
INK = (0.08, 0.08, 0.09)
SCREEN = (0.05, 0.07, 0.10)
RED, YELLOW, GREEN = (0.78, 0.12, 0.12), (0.92, 0.76, 0.12), (0.12, 0.62, 0.22)

# -- the terminal (fixed): a flat case, keypad at the near end (-x), screen at the far end ----------------------------
TERM_XY = np.array([0.29, 0.0])  # the case's centre
TERM_L, TERM_W, TERM_H = 180.0 * MM, 83.0 * MM, 24.0 * MM  # along x, along y, its top above the table
CASE_TOP = TOP + TERM_H
NEAR_X = TERM_XY[0] - TERM_L / 2
# Keys: 3 columns 22 mm apart across the case (the 1 on the customer's left, +y) and 5 rows 16 mm apart along it; the
# top row (1 2 3) farthest from the customer, the coloured keys nearest.
COL_Y = (22.0 * MM, 0.0, -22.0 * MM)
ROW_X = tuple(NEAR_X + (78.0 - 16.0 * r) * MM for r in range(5))  # row 0 (1 2 3) ... row 4 (red yellow green)
KEY_W, KEY_D, KEY_H = 18.0 * MM, 11.0 * MM, 6.0 * MM  # across (y), along (x), tall
KEY_PROUD = 2.0 * MM  # a key's top above the case at rest
KEY_TRAVEL = 1.2 * MM
CLICK = 0.8 * MM  # pushed this far, the key registers (and releases once back above RELEASE)
RELEASE = 0.4 * MM
KEY_K, KEY_PRELOAD = 3000.0, 0.0  # N/m, no preload: 2.4 N at the click
KEY_CLEAR = 1.5 * MM  # the case keeps this far from every key, anywhere in its travel
# WHY the clearance: the keys collide with the fixed case despite their joints (run pin2, 2026-11-17: a case block 1 mm
# into each key held it 1.1 mm up, and no fingertip could push it down)
KEY_TOP = CASE_TOP + KEY_PROUD
LAYOUT = (("1", "2", "3"), ("4", "5", "6"), ("7", "8", "9"), ("star", "0", "hash"), ("red", "yellow", "green"))
KEY_COLORS = {"red": RED, "yellow": YELLOW, "green": GREEN}

PIN = ("7", "3", "1", "9")
EXPECTED = PIN + ("green",)

# The person across the counter: their eyes 60 cm beyond the keypad's middle (+x) and 45 cm above the keys.
KEYPAD_MID = np.array([(ROW_X[0] + ROW_X[4]) / 2, 0.0, KEY_TOP])
EYE = KEYPAD_MID + np.array([0.60, 0.0, 0.45])


def key_xy(name: str) -> np.ndarray:
    for r, row in enumerate(LAYOUT):
        if name in row:
            return np.array([ROW_X[r], COL_Y[row.index(name)]])
    raise KeyError(name)


def key_corners(name: str, depth: float = 0.0) -> np.ndarray:
    """The four corners of a key's top (world), pushed ``depth`` in."""
    x, y = key_xy(name)
    z = KEY_TOP - depth
    return np.array([[x + sx * KEY_D / 2, y + sy * KEY_W / 2, z] for sx in (-1, 1) for sy in (-1, 1)])


# Digits drawn on the keys as seven-segment strokes (visual only): each segment a thin box on the key's top, read
# from the customer's side (the digit's top towards +x).
_SEG = {"a": (0.0, 1.0, "h"), "b": (0.5, 0.5, "v"), "c": (0.5, -0.5, "v"), "d": (0.0, -1.0, "h"), "e": (-0.5, -0.5, "v"),
        "f": (-0.5, 0.5, "v"), "g": (0.0, 0.0, "h")}
_DIGIT = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc", "7": "abc",
          "8": "abcdefg", "9": "abcdfg"}


def _glyph(name: str) -> list:
    s, w, t = 2.2 * MM, 4.4 * MM, 0.6 * MM  # half the digit's height, its width, stroke
    out = []
    for seg in _DIGIT.get(name, ""):
        u, v, kind = _SEG[seg]  # u across (customer's right = -y), v up the digit (+x)
        cy, cx = -u * w / 2 if kind == "h" else -u * w, v * s
        size = (t, w, 0.1 * MM) if kind == "h" else (s, t, 0.1 * MM)
        out.append(box((cx, cy, KEY_H / 2 + 0.05 * MM), size, INK))
    return out


def _case_shapes() -> tuple[list, list]:
    """The case as boxes (its own frame: centre of its footprint on the table). A block below the keys' reach, and a
    top skin round the keypad: walls between the rows and the columns, and a rim. Nothing comes within KEY_CLEAR of a
    key anywhere in its travel."""
    cx = TERM_XY[0]
    deep = KEY_H - KEY_PROUD + KEY_TRAVEL + KEY_CLEAR  # from the case top down to the block, below a key fully down
    solid = [box((0.0, 0.0, (TERM_H - deep) / 2), (TERM_L, TERM_W, TERM_H - deep), CASE)]

    def slab(x0, x1, y0, y1, depth):
        solid.append(box(((x0 + x1) / 2 - cx, (y0 + y1) / 2, TERM_H - depth / 2), (x1 - x0, y1 - y0, depth), CASE))

    near, far = ROW_X[4] - KEY_D / 2 - KEY_CLEAR, ROW_X[0] + KEY_D / 2 + KEY_CLEAR
    side = COL_Y[0] + KEY_W / 2 + KEY_CLEAR
    slab(NEAR_X, near, -TERM_W / 2, TERM_W / 2, deep)  # rim: the near end
    slab(far, NEAR_X + TERM_L, -TERM_W / 2, TERM_W / 2, deep)  # the far end, under the screen
    for sgn in (-1, 1):
        slab(near, far, *sorted((sgn * side, sgn * TERM_W / 2)), deep)  # the two sides
    skin = 3.0 * MM
    for r in range(4):  # walls between the rows
        slab(ROW_X[r + 1] + KEY_D / 2 + KEY_CLEAR, ROW_X[r] - KEY_D / 2 - KEY_CLEAR, -side, side, skin)
    for c in range(2):  # and between the columns
        slab(near, far, COL_Y[c + 1] + KEY_W / 2 + KEY_CLEAR, COL_Y[c] - KEY_W / 2 - KEY_CLEAR, skin)
    seen = [box(((ROW_X[0] - TERM_XY[0]) + 60.0 * MM, 0.0, TERM_H + 0.05 * MM), (60.0 * MM, 64.0 * MM, 0.1 * MM),
                SCREEN)]
    return solid, seen


_CS, _CV = _case_shapes()
TERMINAL = body("terminal", pos=(*TERM_XY, TOP), friction=0.6, shapes=_CS + _CV,
                visual_only=tuple(range(len(_CS), len(_CS) + len(_CV))))

KEYS: list = []
for _r, _row in enumerate(LAYOUT):
    for _c, _name in enumerate(_row):
        _x, _y = ROW_X[_r], COL_Y[_c]
        _shapes = [box((0.0, 0.0, 0.0), (KEY_D, KEY_W, KEY_H), KEY_COLORS.get(_name, KEY_GREY))] + _glyph(_name)
        _rest = np.array([_x, _y, KEY_TOP - KEY_H / 2])
        KEYS.append(body(f"key_{_name}", pos=tuple(_rest), mass=0.004, friction=0.8, shapes=_shapes,
                         visual_only=tuple(range(1, len(_shapes)))))
        fittingkit.add_slider(Slider("terminal", f"key_{_name}", anchor=tuple(_rest - np.array([*TERM_XY, TOP])),
                                     axis=(0, 0, 1), lower=-KEY_TRAVEL, upper=0.0, stiffness=KEY_K, damping=2.0,
                                     target=KEY_PRELOAD))
KEY_REST_Z = KEY_TOP - KEY_H / 2  # every key's origin at rest

OBJECTS = [TERMINAL, *KEYS]


# -- the line of sight -----------------------------------------------------------------------------------------------
def segment_hits_hull(p0: np.ndarray, p1: np.ndarray, planes: np.ndarray, eps: float = 1e-9) -> bool:
    """Whether the segment p0-p1 passes through a convex hull given by outward planes n . x = d (inside: n . x < d)."""
    u = p1 - p0
    t0, t1 = 0.0, 1.0
    for n, d in zip(planes[:, :3], planes[:, 3]):
        nu, gap = float(n @ u), float(d - n @ p0)
        if abs(nu) < eps:
            if gap < 0.0:
                return False
            continue
        t = gap / nu
        if nu > 0.0:
            t1 = min(t1, t)
        else:
            t0 = max(t0, t)
        if t0 > t1:
            return False
    return True


def hidden(points: np.ndarray, hulls: list, eye: np.ndarray = EYE) -> bool:
    """Every point out of the eye's sight: the line from the eye to it passes through one of the hulls (planes)."""
    return all(any(segment_hits_hull(eye, p, h) for h in hulls) for p in points)


def gripper_hulls() -> list:
    """Both grippers' collision hulls (palm and fingers) in the world, as plane arrays; empty offline."""
    environment = fittingkit._ENV["env"]
    if environment is None:
        return []
    from yam_arena.analysis import world_hulls
    from yam_arena.runtime import link_poses

    scene = environment.unwrapped.scene
    return [h.planes for name in fittingkit.ROBOTS
            for _, h in world_hulls(link_poses(scene[name]), list(fittingkit.FINGER_LINKS))]


# -- the keypad's log ------------------------------------------------------------------------------------------------
class PinPad:
    """Runs before every physics substep: reads each key's depth and logs a press when one passes its click. At each
    press it notes whether the key was hidden from the onlooker's eye, and whether another key was down at the time."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.occluders = None  # offline tests set hull planes here; in the sim they come from the grippers
        self.log = None
        import os
        self.log = os.environ.get("PIN_LOG")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return [k.name for k in KEYS]

    def clear(self) -> None:
        self._n = 0
        self.down = {k.name: False for k in KEYS}
        self.depth = {k.name: 0.0 for k in KEYS}
        self.presses: list[tuple[str, bool, bool]] = []  # (key, hidden, another key down)

    def update(self, frames: dict, acc: dict) -> None:
        self._n += 1
        for k in KEYS:
            f = frames.get(k.name)
            if f is None or not np.all(np.isfinite(f.pos)):
                continue
            d = float(KEY_REST_Z + self.origin[2] - f.pos[2])
            self.depth[k.name] = d
            if not self.down[k.name] and d >= CLICK:
                self.down[k.name] = True
                self._press(k.name[4:], d)
            elif self.down[k.name] and d < RELEASE:
                self.down[k.name] = False
        if self._trace and self._n % 24 == 0:
            self._write_trace()

    _trace = __import__("os").environ.get("PIN_TRACE")

    def _write_trace(self) -> None:
        """Debug only (PIN_TRACE=<file>): every 0.1 s, the deepest key and each gripper's lowest point."""
        environment = fittingkit._ENV["env"]
        parts = []
        if environment is not None:
            from yam_arena.analysis import world_hulls
            from yam_arena.runtime import link_poses
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                v = np.vstack([h.vertices for _, h in world_hulls(link_poses(scene[name]), list(fittingkit.FINGER_LINKS))])
                lo = v[np.argmin(v[:, 2])] - self.origin
                parts.append(f"{name} low ({1000 * lo[0]:6.1f},{1000 * lo[1]:6.1f},{1000 * (lo[2] - KEY_TOP):6.1f})")
        deep = max(self.depth, key=self.depth.get)
        ks = " ".join(f"{n[4:]}:{1000 * d:4.1f}" for n, d in self.depth.items() if abs(d) > 0.0002)
        with open(self._trace, "a") as fh:
            fh.write(f"{self._n / 240.0:6.2f}s deep {deep[4:]} {1000 * self.depth[deep]:5.2f}mm | {' '.join(parts)} | {ks}\n")

    def _press(self, name: str, depth: float) -> None:
        others = sum(self.down.values()) > 1
        hulls = self.occluders if self.occluders is not None else gripper_hulls()
        corners = key_corners(name, depth) + self.origin
        is_hidden = bool(hulls) and hidden(corners, hulls, EYE + self.origin)
        self.presses.append((name, is_hidden, others))
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(f"{self._n / 240.0:7.2f}s press {name:6s} hidden {int(is_hidden)} other-key-down {int(others)}\n")

    # -- what the scorer reads
    def keys(self) -> tuple[str, ...]:
        return tuple(p[0] for p in self.presses)

    def clean(self) -> bool:
        return not any(p[2] for p in self.presses)

    def all_hidden(self) -> bool:
        return all(p[1] for p in self.presses)


MODEL = fittingkit.add_model(PinPad())


# -- scoring ---------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9
CLEAR = 0.03  # both grippers this far from the terminal at the end


def hands_clear() -> bool:
    """No gripper point within 3 cm of the terminal's box (keys included); unknown offline counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0:
        return True
    lo = np.array([NEAR_X, -TERM_W / 2, TOP]) - CLEAR
    hi = np.array([NEAR_X + TERM_L, TERM_W / 2, KEY_TOP]) + CLEAR
    return not bool(np.any(np.all((pts > lo) & (pts < hi), axis=1)))


def success(objects) -> bool:
    """The PIN and the green key, each pressed once, in order, no other key, every press hidden from the onlooker,
    and both hands clear of the terminal. Must hold for 1 s."""
    return MODEL.keys() == EXPECTED and MODEL.clean() and MODEL.all_hidden() and hands_clear()


def score(objects) -> float:
    """0.9 for the PIN confirmed with every press hidden (1.0 once success() has held 1 s); 0.5 for it confirmed but
    seen; 0.25 for the four digits right but not confirmed; 0 otherwise (a wrong key, two keys at once, a digit
    missing)."""
    keys = MODEL.keys()
    if not MODEL.clean():
        return 0.0
    if keys == EXPECTED:
        return COMPLETE_SCORE if MODEL.all_hidden() else 0.5
    if keys == PIN:
        return 0.25
    return 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# The right hand presses each key with its closed fingertips. The left hand, fingers closed, stands just beyond the key
# being pressed (on the onlooker's side) as a screen, and moves with it from key to key. Stock waypoints only point
# straight down: this file adds RotWaypoint (any hand orientation) and has yam_arena.demo.ArmPlan honour it.
class RotWaypoint(Waypoint):
    """A Waypoint with its TCP's orientation given as a rotation matrix (columns: the TCP's x, y, z in the world; z
    points along the fingers)."""

    def __init__(self, duration, pos, finger, label="", rot=None):
        super().__init__(duration, pos, finger, label, 0.0)
        self.rot = None if rot is None else np.asarray(rot, float)


def tcp_rotation(w) -> np.ndarray:
    rot = getattr(w, "rot", None)
    return grip_rotation(w.yaw) if rot is None else rot


def _target(self, t: float):
    """yam_arena.demo.ArmPlan.target, honouring a RotWaypoint's orientation."""
    from yam_arena import ik

    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal, rot = np.asarray(w.pos, dtype=float), tcp_rotation(w)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger), w.label)
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_demo.ArmPlan.target = _target

CLOSED = 0.0
PAD_TIP = 13.0 * MM  # the closed fingertips reach this far past the TCP along the hand
Y_AXIS = np.array([0.0, 1.0, 0.0])
# WHY these poses (searched offline against the YAM's gripper collision hulls, plan_check.py): both hands are turned a
# quarter turn so their thin side (along the fingers' closing, +-4 mm at the tips) faces the other; the screen's other
# side (+-9 mm at the tips, wider above) then covers a key's 18 mm width. Both hands flare to 10 cm wide 8 cm above
# their tips, so they lean apart: the pressing hand 35 degrees back towards the arms, the screen 15 degrees towards the
# onlooker. With the screen's tips 24 mm beyond the key's middle and 5 mm above the key tops, every corner and the
# middle of each pressed key is behind the screen alone, the hands stay about 7 mm apart, and both arms reach every pose.
PRESS_YAW, PRESS_LEAN = math.pi / 2, math.radians(35.0)
SCREEN_YAW, SCREEN_LEAN = -math.pi / 2, math.radians(15.0)
PRESS_BELOW = 7.0 * MM  # the pressing fingertips aimed this far below a key's top; the key's stop holds them at 1.2
TOUCH_ABOVE = 6.0 * MM  # the fingertips first come to rest this far above it, settle, then push slowly
# WHY: aimed 2 mm below, the arm gave way to the key's spring, keys reached only 0.6-0.7 mm and key 3 crossed its
# click twice (run pin4); aimed 5 mm below in one 0.5 s move, the hand hit the key, bounced (7 clicked twice) and
# clicked its neighbour (run pin5); aimed 3.5 mm below with a slow push, the arm (about 550 N/m at the fingertip) held
# the key at only 0.5 mm, by the release point, and key 1 clicked twice (run pin6); aimed 7 mm below, the hand, 0.4 s
# behind its plan, reached each key still moving at 85 mm/s, tapped it, bounced off and pressed again: every key
# twice (run pin7, all 2026-11-17)
PRESS_AHEAD = 5.0 * MM  # and this far beyond its middle (+x)
# WHY ahead: in the sim the leaning hand's lowest point landed 5 mm short of where the plan put it, in the gap between
# rows, on every press (run pin2, 2026-11-17)
HOVER = 25.0 * MM
HOVER_BACK = 10.0 * MM
SCREEN_BEYOND = 24.0 * MM
SCREEN_ABOVE = 5.0 * MM
SCREEN_LIFT = 10.0 * MM  # the screen rides this much higher while moving from key to key


def lean_rotation(yaw: float, lean: float) -> np.ndarray:
    """A straight-down grip turned ``yaw``, then leaned ``lean`` about world y (positive: upper body towards -x)."""
    return axis_angle_matrix(Y_AXIS, -lean) @ grip_rotation(yaw)


def press_rotation() -> np.ndarray:
    return lean_rotation(PRESS_YAW, PRESS_LEAN)


def screen_rotation() -> np.ndarray:
    return lean_rotation(SCREEN_YAW, -SCREEN_LEAN)


def tcp_for_tip(tip, rot) -> np.ndarray:
    """The TCP that puts the fingertips' middle at ``tip`` for a hand turned ``rot``."""
    return np.asarray(tip, float) - PAD_TIP * rot[:, 2]


def press_points(name: str) -> tuple[np.ndarray, np.ndarray]:
    x, y = key_xy(name)
    rot = press_rotation()
    # WHY back 10 mm when up: rising straight up, the leaning hand came within 1 mm of the screen (plan_check.py)
    return (tcp_for_tip((x - HOVER_BACK, y, KEY_TOP + HOVER), rot),
            tcp_for_tip((x + PRESS_AHEAD, y, KEY_TOP - PRESS_BELOW), rot))


def touch_point(name: str) -> np.ndarray:
    x, y = key_xy(name)
    return tcp_for_tip((x + PRESS_AHEAD, y, KEY_TOP + TOUCH_ABOVE), press_rotation())


def screen_point(name: str, lift: float = 0.0) -> np.ndarray:
    x, y = key_xy(name)
    return tcp_for_tip((x + SCREEN_BEYOND, y, KEY_TOP + SCREEN_ABOVE + lift), screen_rotation())


def demo():
    right, left = [], []
    pr, sr = press_rotation(), screen_rotation()

    def both(dur, rp, lp, label):
        right.append(RotWaypoint(dur, tuple(rp), CLOSED, label, pr))
        left.append(RotWaypoint(dur, tuple(lp), CLOSED, label, sr))

    first = EXPECTED[0]
    lift = np.array([0.0, 0.0, 0.05])
    both(2.5, press_points(first)[0] + lift, screen_point(first) + lift, "over the keypad")
    for name in EXPECTED:
        up, down = press_points(name)
        both(0.9, up, screen_point(name, SCREEN_LIFT), f"over {name}, the screen beyond it")
        both(0.4, up, screen_point(name), "screen down")
        both(0.8, touch_point(name), screen_point(name), f"touch {name}")
        both(0.3, touch_point(name), screen_point(name), "settle")
        both(0.6, down, screen_point(name), f"press {name}")
        both(0.3, down, screen_point(name), "hold")
        both(0.4, up, screen_point(name), "let it up")
    end_r = press_points(EXPECTED[-1])[0] + np.array([-0.06, -0.10, 0.10])
    end_l = screen_point(EXPECTED[-1]) + np.array([0.0, 0.12, 0.10])
    both(1.2, end_r, end_l, "hands clear")
    both(1.5, end_r, end_l, "wait")
    return {"left": left, "right": right}
