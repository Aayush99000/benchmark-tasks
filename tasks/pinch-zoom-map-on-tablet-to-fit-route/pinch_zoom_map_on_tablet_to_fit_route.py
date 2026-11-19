"""A 10-inch tablet (Amazon Fire HD 10, 2023: 246 x 164.8 x 8.6 mm, 434 g) lies loose and screen up on the
desk, in landscape, long side across the robot. Its screen shows a street map in a web map viewer (Leaflet 1.9.4,
pinch-zoom on, rotation off, no inertia, no zoom animation) with a red walking route drawn on it. At the start the
map is zoomed in so far that only the middle of the route shows: the route's bounding box is 600 x 300 mm on the
screen, 2.8 screen widths. Three blue place pins sit off the route; tapping one opens its place card.

The screen is live. Every control step the task reads where the robot's fingertips are (a fingertip touches the
glass when its end face rests on it, within 0.2 mm) and runs the viewer's own touch rules: one moving touch pans the
map; two touches zoom it by the ratio of their spacing about the point between them; a quick still touch is a tap
(opens a pin's card, or closes the card); two quick taps zoom in 2x. It then redraws the map, the route, the pins
and any card on the screen, clipped to the screen's edges. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts as _yp  # noqa: E402
from yam_parts import Part, box, cyl  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ------------------------------------------------------------------ the tablet
TOP = TABLE_TOP[SCENE]
MM = 0.001
# Amazon Fire HD 10 (2023, 13th generation): 246 x 164.8 x 8.6 mm, 434 g; 10.1-inch 1920 x 1200 screen at
# 224 ppi, so the picture is 217.6 x 136.0 mm (0.1133 mm per pixel), centred on the front.
TAB_L, TAB_W, TAB_T = 0.246, 0.1648, 0.0086         # long side along y, short side along x, thickness
TAB_MASS = 0.434
TAB_R = 0.010                                        # corner radius (estimate from photos)
TAB_X, TAB_Y = 0.200, -0.060                         # centre on the desk
SCREEN_W, SCREEN_H = 217.6, 136.0                    # picture, mm (landscape: width along -y, height along +x)
GLASS_Z = TAB_T                                      # glass top above the tablet's underside
BLACK = (0.04, 0.04, 0.045)


def _tablet():
    cx = TAB_W / 2 - TAB_R
    cy = TAB_L / 2 - TAB_R
    shapes = [box((0.0, 0.0, TAB_T / 2), (TAB_W, TAB_L - 2 * TAB_R, TAB_T)),
              box((0.0, 0.0, TAB_T / 2), (TAB_W - 2 * TAB_R, TAB_L, TAB_T))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            shapes.append(cyl((sx * cx, sy * cy, TAB_T / 2), TAB_R, TAB_T, "z"))
    # Plastic back on a laminate desk; the same material is the glass face the fingertips slide on (estimate).
    return Part("tablet", (TAB_X, TAB_Y, TOP), shapes, mass=TAB_MASS, color=BLACK, friction=(0.4, 0.35),
                contact_offset=0.001)


TABLET = _tablet()
OBJECTS = [TABLET]
PRESSED_PARTS = ("tablet",)          # the fingertips press on the glass and slide on it; nothing is grasped

# ------------------------------------------------------------------ the map (map units: screen mm at the start view)
# Screen coordinates: u to the screen's right (world -y), v to its top (world +x), mm from the screen centre.
# A view is (s, c): s screen mm per map unit, c the map point at the screen centre; screen = s * (map - c).
START_VIEW = (1.0, (0.0, 0.0))
ZOOM_OUT_LIMIT, ZOOM_IN_LIMIT = 1.0 / 8.0, 4.0       # the viewer's minZoom and maxZoom: start zoom -3 and +2
GRID_X = tuple(-1240.0 + 80.0 * k for k in range(32))
GRID_Y = tuple(-750.0 + 75.0 * k for k in range(21))
MAJOR_X, MAJOR_Y = (-360.0, 440.0), (-300.0, 300.0)
MINOR_HW, MAJOR_HW = 4.5, 8.0                        # street half widths, map units
EXTENT = (-1240.0, 1240.0, -750.0, 750.0)
DIAGONAL = ((-1240.0, -650.0), (1240.0, 560.0))
RIVER = ((-1240.0, -560.0), (-700.0, -470.0), (-300.0, -520.0), (100.0, -430.0), (500.0, -480.0), (900.0, -400.0),
         (1240.0, -450.0))
RIVER_HW = 35.0
PARKS = ((-195.0, -125.0, 5.0, 70.0), (125.0, 195.0, -145.0, -80.0), (525.0, 755.0, 80.0, 295.0),
         (-595.0, -445.0, 155.0, 295.0))
# The walking route, along the streets: its bounding box is 600 x 300 map units, centred 20 right of the start
# view's centre, so at the start only its middle shows.
ROUTE = ((-280.0, -150.0), (-120.0, -150.0), (-120.0, -75.0), (40.0, -75.0), (40.0, 75.0), (200.0, 75.0),
         (200.0, 150.0), (320.0, 150.0))
ROUTE_BOX = (min(p[0] for p in ROUTE), max(p[0] for p in ROUTE), min(p[1] for p in ROUTE), max(p[1] for p in ROUTE))
ROUTE_HW_MM = 1.5                                    # the route is drawn 3 mm wide whatever the zoom
END_R_MM, DOT_R_MM = 3.5, 1.3                        # route-end and pin icons: 7 mm discs with a white dot
PIN_HALF_MM = 3.5                                    # a pin's touch area: its 7 x 7 mm icon
PINS = {"cafe": (-60.0, 37.0), "museum": (70.0, -40.0), "pharmacy": (160.0, 110.0)}
CARD_W_MM, CARD_H_MM, CARD_BAR_MM, CARD_LIFT_MM = 44.0, 24.0, 6.0, 5.0

# Touch rules of the viewer (Leaflet 1.9.4 on Fire OS 8 / Android 11); CSS px taken as 1/1280 of the screen width.
CSS_PX_MM = SCREEN_W / 1280.0                        # 0.17 mm (devicePixelRatio 1.5: an estimate)
# The browser holds back a single touch's moves until it has gone the touch slop (Android TOUCH_SLOP 8 dp) from where
# it landed; only then does Leaflet see it (and its own 3 px clickTolerance is already passed): the map then follows
# the whole offset. A touch that never leaves the slop is a tap (click), however long it was held, as measured in
# Chromium with Leaflet 1.9.4.
TOUCH_SLOP_MM = 8.0 * CSS_PX_MM
DOUBLE_TAP_S = 0.3                                   # Android DOUBLE_TAP_TIMEOUT (Leaflet's own: 0.2 s)
DOUBLE_TAP_MM = 100.0 * CSS_PX_MM                    # Android DOUBLE_TAP_SLOP 100 dp
EDGE_MM = 3.0                                        # a touch landing this close to the screen edge: edge swipe
TOUCH_GAP = 0.0002                                   # touching: end face on the glass (pressed faces rest at 0.04-0.11 mm)

# Scoring constants
FIT_MARGIN_MM = 5.0
FILL_FRACTION = 0.5
PINCH_RATIO = 0.75                                   # a pinch: two touches whose spacing fell to 75 % or less
ZOOMED_OUT = 0.5                                     # 0.5 credit: zoomed out 2x or more
CLEAR_MM = 20.0                                      # hands off: fingertips 20 mm above the glass or off the tablet
MOVED_M, TURNED_DEG = 0.020, 5.0
TIP_DEG, DROP_M = 45.0, 0.020
STEP_M, STEP_DEG = 0.0005, 0.25

TEST_VIEW = None          # offline tests: (s, (cx, cy))
TEST_CARD = None          # offline tests: name of an open card
TEST_PINCHED = False
TEST_TOUCHING = False
TEST_STICKY = ()


def to_screen(view, p):
    s, c = view
    return s * (np.asarray(p, float) - np.asarray(c, float))


def route_box_on_screen(view):
    lo = to_screen(view, (ROUTE_BOX[0], ROUTE_BOX[2]))
    hi = to_screen(view, (ROUTE_BOX[1], ROUTE_BOX[3]))
    return lo, hi


def fits_and_fills(view):
    """The whole route's bounding box at least 5 mm inside every screen edge, and at least half the screen wide."""
    lo, hi = route_box_on_screen(view)
    inside = bool(lo[0] >= -SCREEN_W / 2 + FIT_MARGIN_MM and hi[0] <= SCREEN_W / 2 - FIT_MARGIN_MM
                  and lo[1] >= -SCREEN_H / 2 + FIT_MARGIN_MM and hi[1] <= SCREEN_H / 2 - FIT_MARGIN_MM)
    return inside and bool(hi[0] - lo[0] >= FILL_FRACTION * SCREEN_W)


# ------------------------------------------------------------------ the viewer's touch handling
class Viewer:
    """Leaflet 1.9.4's touch handling (Map.Drag/Draggable, Map.TouchZoom, click and double-click zoom, popups),
    restated in screen millimetres. Touches come in as {id: (u, v)} once per control step."""

    def __init__(self):
        self.s, self.c = START_VIEW[0], np.array(START_VIEW[1], float)
        self.touches = {}
        self.drag = None
        self.pinch = None
        self.card = None
        self.last_click = None
        self.pinched = False
        self.min_ratio = 1.0
        self.edge = False
        self.events = []
        self.t = 0.0

    @property
    def view(self):
        return (self.s, (float(self.c[0]), float(self.c[1])))

    def _log(self, text):
        self.events.append((round(self.t, 2), text))
        print(f"[viewer] {self.t:6.2f} s {text}", flush=True)

    def _zoom_to(self, s_new, anchor_map, anchor_screen):
        self.s = float(np.clip(s_new, ZOOM_OUT_LIMIT, ZOOM_IN_LIMIT))
        self.c = np.asarray(anchor_map, float) - np.asarray(anchor_screen, float) / self.s

    def update(self, t, points):
        self.t = float(t)
        points = {k: np.asarray(p, float) for k, p in points.items()}
        for k, p in points.items():                      # touchmove
            if k in self.touches:
                self._move(k, p)
        for k in [k for k in self.touches if k not in points]:   # touchend
            self._end(k)
        for k in sorted(k for k in points if k not in self.touches):   # touchstart
            self._start(k, points[k])

    def _start(self, k, p):
        if abs(p[0]) > SCREEN_W / 2 - EDGE_MM or abs(p[1]) > SCREEN_H / 2 - EDGE_MM:
            self.edge = True
            self._log(f"touch {k} landed {EDGE_MM:.0f} mm or less from the screen edge (edge swipe)")
        self.touches[k] = {"start": p.copy(), "pos": p.copy(), "t0": self.t, "multi": False, "moved": False}
        n = len(self.touches)
        self._log(f"touch {k} down at ({p[0]:.1f}, {p[1]:.1f}) mm, {n} on the glass")
        if n == 1:
            self.drag = {"id": k, "start": p.copy(), "c0": self.c.copy(), "moved": False}
            return
        for rec in self.touches.values():
            rec["multi"] = True
        if self.drag is not None:
            self._log("drag ends: a second touch landed")
            self.drag = None
        if n == 2 and self.pinch is None:
            (a, ra), (b, rb) = sorted(self.touches.items())
            p1, p2 = ra["pos"], rb["pos"]
            mid = (p1 + p2) / 2
            self.pinch = {"ids": (a, b), "d0": float(np.linalg.norm(p1 - p2)), "s0": self.s,
                          "anchor": self.c + mid / self.s, "moved": False}
            self._log(f"pinch starts, spacing {self.pinch['d0']:.1f} mm, about ({mid[0]:.1f}, {mid[1]:.1f})")

    def _move(self, k, p):
        rec = self.touches[k]
        rec["pos"] = p.copy()
        if np.linalg.norm(p - rec["start"]) >= TOUCH_SLOP_MM:
            rec["moved"] = True
        if self.drag is not None and self.drag["id"] == k and len(self.touches) == 1:
            off = p - self.drag["start"]
            if self.drag["moved"] or rec["moved"]:
                if not self.drag["moved"]:
                    self._log(f"pan starts (touch {k})")
                self.drag["moved"] = True
                self.c = self.drag["c0"] - off / self.s
        if self.pinch is not None and len(self.touches) == 2 and all(i in self.touches for i in self.pinch["ids"]):
            p1, p2 = (self.touches[i]["pos"] for i in self.pinch["ids"])
            d = float(np.linalg.norm(p1 - p2))
            if self.pinch["d0"] > 0:
                ratio = d / self.pinch["d0"]
                self.pinch["moved"] = True
                self._zoom_to(self.pinch["s0"] * ratio, self.pinch["anchor"], (p1 + p2) / 2)
                self.min_ratio = min(self.min_ratio, ratio)
                if ratio <= PINCH_RATIO and not self.pinched:
                    self.pinched = True
                    self._log(f"pinch: spacing down to {100 * ratio:.0f} %")

    def _end(self, k):
        rec = self.touches.pop(k)
        self._log(f"touch {k} up after {self.t - rec['t0']:.2f} s")
        if self.pinch is not None and k in self.pinch["ids"]:
            self._log(f"pinch ends at zoom x{self.s:.3f}")
            self.pinch = None
        if self.drag is not None and self.drag["id"] == k:
            self.drag = None
        if not rec["multi"] and not rec["moved"]:
            self._click(rec["pos"])

    def _click(self, p):
        self._log(f"tap at ({p[0]:.1f}, {p[1]:.1f}) mm")
        if self.card is not None and _in_card(self.view, self.card, p):
            return                                   # a tap on the open card itself does nothing
        hit = None
        for name, at in PINS.items():
            q = to_screen(self.view, at)
            if abs(p[0] - q[0]) <= PIN_HALF_MM and abs(p[1] - q[1]) <= PIN_HALF_MM:
                hit = name
        if hit is not None:
            self.card = hit
            self._log(f"the {hit} card opens")
        elif self.card is not None:
            self._log(f"the {self.card} card closes")
            self.card = None
        last = self.last_click
        if last is not None and self.t - last[0] <= DOUBLE_TAP_S and np.linalg.norm(p - last[1]) <= DOUBLE_TAP_MM:
            anchor = self.c + p / self.s
            self._zoom_to(self.s * 2.0, anchor, p)
            self._log(f"double tap: zoom in to x{self.s:.3f}")
            self.last_click = None
        else:
            self.last_click = (self.t, p.copy())


def _in_card(view, name, p):
    q = to_screen(view, PINS[name])
    return bool(abs(p[0] - q[0]) <= CARD_W_MM / 2 and q[1] + CARD_LIFT_MM <= p[1] <= q[1] + CARD_LIFT_MM + CARD_H_MM)


# ------------------------------------------------------------------ fingertips on the glass
# The fingertip end faces in each finger link's frame (from the robot's collision meshes): 14 x 3.2 mm flat faces,
# 12.8 mm past the tool point, square to the approach axis.
NIB_FACE = {
    "tip_left": [(x, y, -0.0901) for x in (0.0170, 0.0310) for y in (-0.0476, -0.0444)] + [(0.0240, -0.0460, -0.0901)],
    "tip_right": [(x, y, -0.0901) for x in (-0.0310, -0.0170) for y in (0.0444, 0.0476)] + [(-0.0240, 0.0460, -0.0901)],
}
TEST_NIBS = None          # offline tests: {id: (5, 3) array of world points}


def nib_points():
    """{(arm, finger): 5 x 3 world points (4 face corners, then the centre)} from the arms' joint readings."""
    if TEST_NIBS is not None:
        return {k: np.asarray(v, float) for k, v in TEST_NIBS.items()}
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import YAM_URDF, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.urdf import forward_kinematics

    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        names = list(ARM_JOINTS) + list(FINGER_JOINTS)
        ids = art.find_joints(names, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q, float)[ids]
        fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(names, q)})
        base = np.asarray(station.BASES[arm], float)[:3]
        for link, pts in NIB_FACE.items():
            m = fk[link]
            out[(arm, link)] = np.asarray(pts) @ m[:3, :3].T + m[:3, 3] + base
    return out


def nibs_at(u, v, height_mm, spacing_mm=25.0):
    """Offline tests: the right gripper's two fingertip faces level, `height_mm` above the glass at screen (u, v)."""
    out = {}
    for name, side in (("tip_left", 1.0), ("tip_right", -1.0)):
        c = screen_to_world(u, v, TOP + GLASS_Z + height_mm * MM) + np.array([0.0, side * spacing_mm * MM / 2, 0.0])
        corners = [c + np.array([dx, dy, 0.0]) for dx in (-0.007, 0.007) for dy in (-0.0016, 0.0016)]
        out[("right", name)] = np.array(corners + [c])
    return out


def _local(tab, pts):
    return (np.asarray(pts, float) - np.asarray(tab.origin, float)) @ tab.rotation()


def touches(tab, nibs):
    """Screen points {id: (u, v)} of the fingertip faces now on the glass."""
    out = {}
    if not nibs:
        return out
    for k, pts in sorted(nibs.items()):
        loc = _local(tab, pts)
        lowest = float(loc[:4, 2].min())
        u, v = -loc[4, 1] / MM, loc[4, 0] / MM
        if lowest <= GLASS_Z + TOUCH_GAP and lowest >= GLASS_Z - 0.006 and abs(u) <= SCREEN_W / 2 \
                and abs(v) <= SCREEN_H / 2:
            out[f"{k[0]}-{k[1][4:]}"] = (u, v)
    return out


def hands_off(tab, nibs):
    """Every fingertip face 20 mm or more above the glass, or 10 mm or more outside the tablet's outline."""
    if not nibs:
        return True
    for pts in nibs.values():
        loc = _local(tab, pts)
        above = float(loc[:4, 2].min()) - GLASS_Z
        outside = max(float(np.abs(loc[:4, 0]).min()) - TAB_W / 2, float(np.abs(loc[:4, 1]).min()) - TAB_L / 2)
        if above < CLEAR_MM * MM and outside < 0.010:
            return False
    return True


# ------------------------------------------------------------------ drawing the screen
def _clip(poly):
    """Sutherland-Hodgman clip of a convex polygon (n x 2, screen mm) to the screen rectangle."""
    out = [np.asarray(p, float) for p in poly]
    for axis, sign, lim in ((0, 1, SCREEN_W / 2), (0, -1, SCREEN_W / 2), (1, 1, SCREEN_H / 2), (1, -1, SCREEN_H / 2)):
        if not out:
            break
        pts, out = out, []
        for i, cur in enumerate(pts):
            prev = pts[i - 1]
            ci, pi_ = sign * cur[axis] <= lim, sign * prev[axis] <= lim
            if ci != pi_:
                t = (lim - sign * prev[axis]) / (sign * cur[axis] - sign * prev[axis])
                out.append(prev + t * (cur - prev))
            if ci:
                out.append(cur)
    return np.array(out) if len(out) >= 3 else None


def _quad(a, b, hw):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    n = np.linalg.norm(d)
    if n < 1e-9:
        return None
    w = np.array([-d[1], d[0]]) / n * hw
    return np.array([a - w, b - w, b + w, a + w])


def _disc(c, r, n=16):
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)]


LAYERS = (  # name, sRGB colour, height above the glass (mm)
    ("land", (0.80, 0.77, 0.71), 0.10), ("water", (0.60, 0.79, 0.89), 0.15), ("park", (0.68, 0.87, 0.62), 0.15),
    ("street", (1.0, 1.0, 1.0), 0.20), ("major", (0.98, 0.79, 0.52), 0.25), ("route", (0.88, 0.10, 0.10), 0.35),
    ("start", (0.12, 0.62, 0.24), 0.45), ("end", (0.16, 0.16, 0.22), 0.45), ("pin", (0.15, 0.40, 0.86), 0.45),
    ("dot", (1.0, 1.0, 1.0), 0.50), ("card", (0.99, 0.99, 0.99), 0.60), ("card_bar", (0.15, 0.40, 0.86), 0.65),
)


def screen_polys(view, card=None):
    """{layer: [polygons in screen mm]}: what the viewer draws for a view, clipped to the screen."""
    s, _ = view
    S = lambda p: to_screen(view, p)  # noqa: E731
    polys = {name: [] for name, _, _ in LAYERS}
    polys["land"].append(np.array([[-SCREEN_W / 2, -SCREEN_H / 2], [SCREEN_W / 2, -SCREEN_H / 2],
                                   [SCREEN_W / 2, SCREEN_H / 2], [-SCREEN_W / 2, SCREEN_H / 2]]))
    for a, b in zip(RIVER[:-1], RIVER[1:]):
        polys["water"].append(_quad(S(a), S(b), RIVER_HW * s))
        polys["water"].append(_disc(S(b), RIVER_HW * s, 12))
    for x0, x1, y0, y1 in PARKS:
        polys["park"].append(np.array([S((x0, y0)), S((x1, y0)), S((x1, y1)), S((x0, y1))]))
    for x in GRID_X:
        layer, hw = ("major", MAJOR_HW) if x in MAJOR_X else ("street", MINOR_HW)
        polys[layer].append(_quad(S((x, EXTENT[2])), S((x, EXTENT[3])), hw * s))
    for y in GRID_Y:
        layer, hw = ("major", MAJOR_HW) if y in MAJOR_Y else ("street", MINOR_HW)
        polys[layer].append(_quad(S((EXTENT[0], y)), S((EXTENT[1], y)), hw * s))
    polys["major"].append(_quad(S(DIAGONAL[0]), S(DIAGONAL[1]), MAJOR_HW * s))
    pts = [S(p) for p in ROUTE]
    for a, b in zip(pts[:-1], pts[1:]):
        polys["route"].append(_quad(a, b, ROUTE_HW_MM))
    for p in pts[1:-1]:
        polys["route"].append(_disc(p, ROUTE_HW_MM, 8))
    polys["start"].append(_disc(pts[0], END_R_MM))
    polys["end"].append(_disc(pts[-1], END_R_MM))
    polys["dot"] += [_disc(pts[0], DOT_R_MM, 10), _disc(pts[-1], DOT_R_MM, 10)]
    for name, at in PINS.items():
        q = S(at)
        polys["pin"].append(_disc(q, PIN_HALF_MM))
        polys["dot"].append(_disc(q, DOT_R_MM, 10))
    if card is not None:
        q = S(PINS[card])
        x0, x1 = q[0] - CARD_W_MM / 2, q[0] + CARD_W_MM / 2
        y0 = q[1] + CARD_LIFT_MM
        y1, yb = y0 + CARD_H_MM, y0 + CARD_H_MM - CARD_BAR_MM
        polys["card"].append(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]))
        polys["card"].append(np.array([[q[0] - 2.5, y0], [q[0], y0 - 3.0], [q[0] + 2.5, y0]])[::-1])
        polys["card_bar"].append(np.array([[x0, yb], [x1, yb], [x1, y1], [x0, y1]]))
    return {k: [c for c in (_clip(p) for p in v if p is not None) if c is not None] for k, v in polys.items()}


def _mesh_arrays(polys, height_mm):
    """USD mesh arrays in the tablet's frame (screen u -> -y, v -> +x), each polygon one counter-clockwise face."""
    pts, counts, idx = [], [], []
    for p in polys:
        if np.cross(p[1] - p[0], p[2] - p[0]) < 0:
            p = p[::-1]
        base = len(pts)
        for u, v in p:
            pts.append((v * MM, -u * MM, GLASS_Z + height_mm * MM))
        counts.append(len(p))
        idx += list(range(base, base + len(p)))
    return pts, counts, idx


_SCREEN = _RUN.setdefault("pinch_screen", {"paths": [], "drawn": None})
_original_spawn = getattr(_yp._spawn_part, "_pinch_original", _yp._spawn_part)


def _spawn_part(prim_path, cfg, *args, **kwargs):
    """The kit's part spawner, plus the tablet's screen: one drawn-only mesh per map layer under the tablet's body
    (no collider, no mass), with an emissive look, as a lit screen."""
    root = _original_spawn(prim_path, cfg, *args, **kwargs)
    if getattr(cfg, "part_name", "") != "tablet":
        return root
    try:
        import isaaclab.sim as sim_utils
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        from yam_arena.station import linear_rgb

        stage = kwargs.get("stage") or sim_utils.get_current_stage()
        UsdGeom.Xform.Define(stage, f"{prim_path}/screen")
        for name, rgb, _ in LAYERS:
            path = f"{prim_path}/screen/{name}"
            mesh = UsdGeom.Mesh.Define(stage, path)
            mesh.CreateDoubleSidedAttr().Set(True)
            mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
            mat = UsdShade.Material.Define(stage, f"{path}_look")
            shader = UsdShade.Shader.Define(stage, f"{path}_look/Shader")
            shader.CreateIdAttr().Set("UsdPreviewSurface")
            lin = linear_rgb(rgb)
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*lin))
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(0.30 * c for c in lin)))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        _SCREEN["paths"].append(prim_path)
        _SCREEN["drawn"] = None
        draw(START_VIEW, None)
    except Exception as exc:  # pragma: no cover - simulator only
        print(f"pinch-zoom: screen not authored: {exc!r}", flush=True)
    return root


_spawn_part._pinch_original = _original_spawn
_yp._spawn_part = _spawn_part


def draw(view, card):
    """Redraw the screen for a view (only when it changed)."""
    key = (round(view[0], 6), round(view[1][0], 4), round(view[1][1], 4), card)
    if not _SCREEN["paths"] or _SCREEN["drawn"] == key:
        return
    import isaaclab.sim as sim_utils
    from pxr import Gf, UsdGeom, Vt

    stage = sim_utils.get_current_stage()
    polys = screen_polys(view, card)
    for prim_path in _SCREEN["paths"]:
        for name, _, height in LAYERS:
            pts, counts, idx = _mesh_arrays(polys[name], height)
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"{prim_path}/screen/{name}"))
            mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(counts))
            mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(idx))
    _SCREEN["drawn"] = key


# ------------------------------------------------------------------ the trial: touches in, screen out, every step
def _viewer():
    v = _RUN.get("pinch_viewer")
    if v is None or v[0] != _RUN["episode"]:
        v = (_RUN["episode"], Viewer(), {"step": None, "touching": False, "off": True, "pose": None, "quiet": False})
        _RUN["pinch_viewer"] = v
    return v[1], v[2]


def tick(objects):
    """Once per control step: read the fingertips, run the viewer, redraw the screen. Offline: nothing moves."""
    tab = objects["tablet"]
    if _RUN["t"] is None:
        nibs = nib_points()
        return None, {"touching": bool(TEST_TOUCHING or touches(tab, nibs)), "off": hands_off(tab, nibs),
                      "quiet": True}
    viewer, rec = _viewer()
    token = _RUN["step"]
    if rec["step"] == token:
        return viewer, rec
    rec["step"] = token
    nibs = nib_points()
    pts = touches(tab, nibs)
    viewer.update(float(_RUN["t"]), pts)
    rec["touching"] = bool(pts)
    rec["off"] = hands_off(tab, nibs)
    pose = np.r_[tab.to_world((0, 0, 0)), tab.to_world((0, 0.1, 0))]
    rec["quiet"] = rec["pose"] is not None and bool(np.all(np.abs(pose - rec["pose"]) < STEP_M))
    rec["pose"] = pose
    try:
        draw(viewer.view, viewer.card)
    except Exception as exc:  # pragma: no cover - simulator only
        print(f"pinch-zoom: draw failed: {exc!r}", flush=True)
    if token % 10 == 0 or pts:
        lo, hi = route_box_on_screen(viewer.view)
        hts = {f"{k[0][0]}{k[1][4]}": round((float(_local(tab, p)[:4, 2].min()) - GLASS_Z) / MM, 2)
               for k, p in (nibs or {}).items()}
        print(f"PZLOG {token} {float(_RUN['t']):.2f} zoom={viewer.s:.4f} c=({viewer.c[0]:.1f},{viewer.c[1]:.1f}) "
              f"route=({lo[0]:.1f},{lo[1]:.1f})..({hi[0]:.1f},{hi[1]:.1f}) touches={ {k: (round(float(p[0]), 1), round(float(p[1]), 1)) for k, p in pts.items()} } "
              f"nib_mm={hts} card={viewer.card} tab=({tab.origin[0]:.4f},{tab.origin[1]:.4f}) yaw={_yaw_deg(tab):.2f}",
              flush=True)
    return viewer, rec


def _yaw_deg(tab):
    r = tab.rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def tablet_in_place(tab):
    d = np.asarray(tab.origin, float)[:2] - np.array([TAB_X, TAB_Y])
    return bool(np.linalg.norm(d) <= MOVED_M and abs(_yaw_deg(tab)) <= TURNED_DEG and tab.tilt_deg() <= 3.0)


def status(objects):
    tab = objects["tablet"]
    viewer, rec = tick(objects)
    view = TEST_VIEW if TEST_VIEW is not None else (viewer.view if viewer is not None else START_VIEW)
    card = TEST_CARD if viewer is None else viewer.card
    return dict(view=view, card=card, fit=fits_and_fills(view), pinched=bool(TEST_PINCHED or (viewer and viewer.pinched)),
                edge=bool(viewer and viewer.edge), touching=bool(rec["touching"]), off=bool(rec["off"]),
                quiet=bool(rec["quiet"]), in_place=tablet_in_place(tab))


def score(objects):
    tab = objects["tablet"]
    st = status(objects)
    turned = sticky("tablet turned over", tab.tilt_deg() > TIP_DEG)
    fell = sticky("tablet off the desk", tab.origin[2] < TOP - DROP_M)
    edge = sticky("edge swipe", st["edge"])
    pinched = ever("pinched", st["pinched"])
    settled = (not st["touching"]) and st["off"] and st["quiet"]
    seconds = held_for("hands off, tablet still", settled)
    if turned or fell or edge:
        return 0.0
    if settled and seconds + 1e-9 >= STILL_S:
        if st["fit"]:
            return 1.0 if st["card"] is None and st["in_place"] else 0.75
        if st["view"][0] <= ZOOMED_OUT:
            return 0.5
    return 0.25 if pinched else 0.0


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo (open loop: the start layout and time)
NIB_PAST_TCP = 0.0128      # fingertip end faces past the tool point (robot collision meshes)
NIB_HALF = 0.00145         # a face's centre sits this far outside the finger's inner face
PRESS = 0.004              # commanded this far below the glass: a light press (about 2 N on this soft arm)
LAND_MM, END_MM = 90.0, 25.0     # fingertip centre spacing at landing and after the pinch
PINCH_AT = (-8.0, 0.0)     # screen point between the fingertips (mm): keeps the zoomed route centred
STEADY = False             # control: the left hand presses the tablet's left end down while the right pinches
DEMO_PACE = 1.0


def finger_for(spacing_mm):
    """Finger joint target for a fingertip centre spacing."""
    return -max(spacing_mm * MM / 2 - NIB_HALF, 0.0)


def screen_to_world(u, v, z):
    return np.array([TAB_X + v * MM, TAB_Y - u * MM, z])


def demo():
    plan = {"left": [], "right": []}
    st = {}

    def add(dt, label, left=None, right=None, lf=None, rf=None, ly=None, ry=None):
        for arm, pos, f, yaw in (("left", left, lf, ly), ("right", right, rf, ry)):
            if pos is not None:
                st[arm] = np.asarray(pos, float)
            if f is not None:
                st[arm + "f"] = f
            if yaw is not None:
                st[arm + "y"] = yaw
            plan[arm].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in st[arm]), st[arm + "f"], label,
                                      st[arm + "y"]))

    glass = TOP + GLASS_Z
    contact = glass + NIB_PAST_TCP
    park = {"left": np.array([0.12, 0.22, TOP + 0.13]), "right": np.array([0.10, -0.22, TOP + 0.13])}
    st.update(left=park["left"], leftf=FINGER_OPEN, lefty=0.0, right=park["right"], rightf=FINGER_OPEN, righty=0.0)
    mid = screen_to_world(*PINCH_AT, contact)
    steady = np.array([TAB_X, TAB_Y + TAB_L / 2 - 0.006, contact])     # on the left-end bezel, off the screen
    add(1.5, "approach")
    if STEADY:
        add(2.0, "left: over the tablet's left end, fingers closed", left=steady + [0, 0, 0.04], lf=0.0, ly=-math.pi / 2)
        add(1.0, "left: press the bezel down", left=steady - [0, 0, 0.006])
    add(2.5, "right: over the map, fingertips 90 mm apart", right=mid + [0, 0, 0.04], rf=finger_for(LAND_MM))
    add(1.0, "right: down to 8 mm above the glass", right=mid + [0, 0, 0.008])
    add(1.2, "right: both fingertips onto the glass together", right=mid - [0, 0, PRESS])
    add(0.6, "right: settle, both touching")
    add(2.4, "right: pinch: fingertips slide together on the glass", rf=finger_for(END_MM))
    add(0.4, "right: hold")
    add(0.8, "right: lift both fingertips straight off", right=mid + [0, 0, 0.04])
    add(1.5, "right: away", right=park["right"])
    if STEADY:
        add(1.0, "left: off the tablet", left=steady + [0, 0, 0.05])
        add(1.5, "left: away", left=park["left"], lf=FINGER_OPEN, ly=0.0)
    add(3.0, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
