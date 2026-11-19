"""Jump-start a flat car battery from a donor battery with a pair of jump leads and four sprung clamps.

On the station table stand two full-size 12 V car batteries (BCI group 26R cases, 208 x 173 mm, 197 mm tall
to the top of the posts), long side across the table, posts on the edge nearest the robot. The LEFT one is flat
(the "dead" battery), the RIGHT one is the donor. Each has two tapered lead posts on its grey lid: the + post
(17.5 mm across at the top, 19.0 mm at the foot, 18 mm tall) at the end towards the robot's right, the - post
(15.9 mm / 17.4 mm) at the other end. A flipped-open red (+) or black (-) terminal cover lies just behind each
post, over the car's own cable. Each battery is installed in a mock-up of its car's engine bay: a tray with a 25 mm
lip, a steel hold-down bar across the lid between the posts on J-bolts, the car's own top-post terminal clamp on
each post (a collar 25 / 23.4 mm across, top flush with the post) with its cable back over the lid, the bay's inner wing and strut tower behind, a fuse box (flat
car) or air box (donor) at the battery's inner end, and the donor's wing edge 5 cm outboard of its - post. Farther
left, at least 15 cm from the flat battery, the flat car's engine ground point: an unpainted steel stud (12 mm
across, 25 mm tall) on a steel bracket on a ledge of the engine, the engine behind it, the coolant tank beside.

A red and a black jump lead, both 2 m long, lie in a loose heap on the table in front of the batteries, as tipped out
of their bag: each lead in irregular loops, the red loops over the black ones, the four clamps lying flat round the
edge of the heap (see the pile section below). Each lead ends in a sprung booster clamp at both ends (four clamps). A clamp is two
copper-plated steel halves on a pivot pin with a torsion spring (153 mm long, 19 mm strip, 38 mm maximum jaw
spread, vinyl handle covers in the lead's colour). It is shut at rest; squeezing the two handles together opens the
jaws. Each clamp lies flat on the table, jaws pointing roughly away from the robot.

A clamp is ATTACHED to a post when nothing holds it and its own spring keeps the jaws closed on the post: the post
stands between the two jaws inside their length, the jaws are propped open by it (3 deg or more, short of the
37 deg stop), the jaws overlap the post's height, the clamp is level within 30 deg, and no gripper is within 4 cm.
The safe jump-start order (HSE INDG139) is: (1) red to the flat battery's +, (2) the other red end to the donor's
+, (3) black to the donor's -, (4) the other black end to the engine ground stud, never to the flat battery's -.
The two red connections may be made in either order (both are on the positive side, so neither order can close the
circuit); both must be in place before the black lead goes on the donor's -, and that before the engine stud.
The order is judged on electrical CONTACT, not on attachment: a clamp is in contact with a post as soon as any point
of its jaw metal comes within 3 mm of the post, whether a gripper still holds it or not, so touching a post out of
order while holding the clamp is already a violation. The red-to-black jaw rule, and the rule that no red jaw may
touch bare earthed car metal (hold-downs, body, engine), apply as soon as any clamp is in contact with a post. Full score needs all four clamps ATTACHED at the end. Scoring: see score().
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cablekit  # noqa: E402,F401
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

RED = (0.80, 0.06, 0.05)
BLACK = (0.06, 0.06, 0.06)
CASE = (0.13, 0.14, 0.16)
LID = (0.55, 0.57, 0.60)
COPPER = (0.78, 0.45, 0.22)
STEEL = (0.62, 0.62, 0.64)
BARE = (0.74, 0.75, 0.77)
LEAD_GREY = (0.45, 0.46, 0.48)
ENGINE = (0.28, 0.29, 0.31)
WHITE = (0.92, 0.92, 0.90)

# -- batteries: BCI group 26R, 208 L x 173 W x 197 H overall (to the top of the posts) -------------------------------
BAT_L, BAT_W, BAT_H = 0.208, 0.173, 0.197
POST_H = 0.018
LID_TOP = BAT_H - POST_H  # lid surface 179 mm above the table
LID_T = 0.015
POST_INSET = 0.028  # post axis from the front edge and from the end of the case
BATS = {"dead": (0.42, 0.20), "donor": (0.42, -0.20)}  # case centres (x, y); long side along y
# the car's own top-post terminal clamp on each post: a lead-alloy collar round the post, top flush with the post top
TERM_R = {"pos": 0.0125, "neg": 0.0117}  # 25 / 23.4 mm across (a 3.75 mm wall round the post's top diameter)
TONGUE_T = 0.002
WING_X, WING_H, STRUT_H = 0.535, 0.45, 0.45  # inner wing (bay wall) 3 cm behind the trays
TRAY = (0.24, 0.24, 0.26)
STEEL_DARK = (0.34, 0.35, 0.37)
BODY = (0.50, 0.56, 0.66)  # body-colour sheet metal
FUSE = (0.22, 0.22, 0.24)
POS_R = (0.0095, 0.00875)  # + post radius at the foot, at the top (SAE: 17.5 mm across at the top)
NEG_R = (0.0087, 0.00795)  # - post (SAE: 15.9 mm across at the top)


def _post_xy(bat: str, sign: str):
    cx, cy = BATS[bat]
    x = cx - BAT_W / 2 + POST_INSET
    inner = cy - math.copysign(BAT_L / 2 - POST_INSET, cy)  # post at the end nearer the table's midline
    outer = cy + math.copysign(BAT_L / 2 - POST_INSET, cy)
    return (x, inner if sign == "pos" else outer)  # both + posts at the inner ends


# -- engine ground point: steel stud on a bracket on the engine block ------------------------------------------------
STUD_XY = (0.42, 0.46)
ENG_SIZE = (0.10, 0.10, 0.050)
BRACKET_T = 0.008
STUD_FOOT = ENG_SIZE[2] + BRACKET_T  # 58 mm above the table
STUD_R, STUD_H = 0.006, 0.025

# name -> axis xy, foot z, top z (above the table), radius at foot, radius at top
POSTS = {
    # battery terminals: the post with the car's terminal clamp round it (one conductor, the clamp's outer surface)
    "dead_pos": (_post_xy("dead", "pos"), LID_TOP, BAT_H, TERM_R["pos"], TERM_R["pos"]),
    "dead_neg": (_post_xy("dead", "neg"), LID_TOP, BAT_H, TERM_R["neg"], TERM_R["neg"]),
    "donor_pos": (_post_xy("donor", "pos"), LID_TOP, BAT_H, TERM_R["pos"], TERM_R["pos"]),
    "donor_neg": (_post_xy("donor", "neg"), LID_TOP, BAT_H, TERM_R["neg"], TERM_R["neg"]),
    "ground": (STUD_XY, STUD_FOOT, STUD_FOOT + STUD_H, STUD_R, STUD_R),
}
STEPS = [("red", "dead_pos"), ("red", "donor_pos"), ("black", "donor_neg"), ("black", "ground")]
ALLOWED = {"red": {"dead_pos", "donor_pos"}, "black": {"donor_neg", "ground"}}


def _battery(tag: str) -> list:
    cx, cy = BATS[tag]
    out = [Box(f"{tag}_case", pos=(cx, cy, TOP + (LID_TOP - LID_T) / 2), size=(BAT_W, BAT_L, LID_TOP - LID_T),
               color=CASE),
           Box(f"{tag}_lid", pos=(cx, cy, TOP + LID_TOP - LID_T / 2), size=(BAT_W, BAT_L, LID_T), color=LID)]
    for sign, col in (("pos", RED), ("neg", BLACK)):
        x, y = _post_xy(tag, sign)
        # the car's terminal clamp (the collar round the post is the post mesh, see _install_spawn): its flat tongue
        # back to the cable, the cable (12 mm) back over the lid and down behind the case
        out.append(Box(f"{tag}_{sign}_lug", pos=(x + 0.0275, y, TOP + LID_TOP + TONGUE_T / 2), size=(0.019, 0.010, TONGUE_T),
                       color=BARE))
        back = cx + BAT_W / 2
        out.append(Box(f"{tag}_{sign}_cable", pos=((x + 0.037 + back + 0.006) / 2, y, TOP + LID_TOP + 0.006),
                       size=(back + 0.006 - x - 0.037, 0.012, 0.012), color=col))
        out.append(Box(f"{tag}_{sign}_cable_down", pos=(back + 0.012, y, TOP + LID_TOP / 2 + 0.006),
                       size=(0.012, 0.012, LID_TOP + 0.012), color=col))
        # its flipped-open terminal cover lying back over the cable
        out.append(Box(f"{tag}_{sign}_cover", pos=(x + 0.055, y, TOP + LID_TOP + 0.017), size=(0.030, 0.034, 0.010),
                       color=col))
        # moulded polarity mark on the lid, 4 cm in front of the post (robot side)
        out.append(Box(f"{tag}_{sign}_mark_h", pos=(x + 0.030 - 0.062, y, TOP + LID_TOP + 0.0005),
                       size=(0.004, 0.016, 0.001), color=col if sign == "pos" else BLACK))
        if sign == "pos":
            out.append(Box(f"{tag}_{sign}_mark_v", pos=(x + 0.030 - 0.062, y, TOP + LID_TOP + 0.0005),
                           size=(0.016, 0.004, 0.001), color=col))
    # battery tray (25 mm lip all round) and the hold-down: a steel bar across the lid between the posts, hooked
    # over the front and back of the case to J-bolts in the tray
    x0, x1, y0, y1 = cx - BAT_W / 2, cx + BAT_W / 2, cy - BAT_L / 2, cy + BAT_L / 2
    for nm, (a, b, c_, d) in {"front": (x0 - 0.006, x0, y0 - 0.006, y1 + 0.006), "back": (x1, x1 + 0.006, y0 - 0.006, y1 + 0.006),
                              "in": (x0, x1, y0 - 0.006, y0), "out": (x0, x1, y1, y1 + 0.006)}.items():
        out.append(Box(f"{tag}_tray_{nm}", pos=((a + b) / 2, (c_ + d) / 2, TOP + 0.0125), size=(b - a, d - c_, 0.025),
                       color=TRAY))
    out.append(Box(f"{tag}_holddown", pos=(cx, cy, TOP + LID_TOP + 0.004), size=(BAT_W + 0.012, 0.025, 0.008),
                   color=STEEL_DARK))
    for nm, xx in (("front", x0 - 0.003), ("back", x1 + 0.003)):
        out.append(Box(f"{tag}_jbolt_{nm}", pos=(xx, cy, TOP + (LID_TOP + 0.008) / 2), size=(0.006, 0.010, LID_TOP + 0.008),
                       color=STEEL_DARK))
    return out


def _bay(tag: str) -> list:
    """The rest of the car's engine bay round its battery: the inner wing (bay wall) behind it with the strut tower,
    and the box that sits at the battery's inner end (fuse box on the flat car, air box on the donor)."""
    cx, cy = BATS[tag]
    s = 1.0 if cy > 0 else -1.0
    y0, y1 = (0.03, 0.40) if tag == "dead" else (-0.63, -0.03)  # the flat car's wing stops at its engine
    out = [Box(f"{tag}_inner_wing", pos=(WING_X + 0.0075, (y0 + y1) / 2, TOP + WING_H / 2), size=(0.015, y1 - y0, WING_H),
               color=BODY),
           Cylinder(f"{tag}_strut_tower", pos=(WING_X + 0.065, s * (0.30 if tag == "dead" else 0.37), TOP + STRUT_H / 2),
                    radius=0.080,
                    height=STRUT_H, color=BODY)]
    if tag == "dead":
        out.append(Box("dead_fuse_box", pos=(0.460, 0.058, TOP + 0.060), size=(0.130, 0.060, 0.120), color=FUSE))
    else:
        out.append(Box("donor_air_box", pos=(0.455, -0.051, TOP + 0.100), size=(0.140, 0.074, 0.200), color=FUSE))
        # the donor's front wing edge outboard of its battery
        out.append(Box("donor_wing_edge", pos=(0.4425, -0.3375, TOP + 0.200), size=(0.215, 0.015, 0.400), color=BODY))
    return out


ENGINE_OBJS = [
    Box("engine_block", pos=(STUD_XY[0] + 0.02, STUD_XY[1] + 0.01, TOP + ENG_SIZE[2] / 2), size=ENG_SIZE, color=ENGINE),
    Box("ground_bracket", pos=(STUD_XY[0], STUD_XY[1], TOP + ENG_SIZE[2] + BRACKET_T / 2), size=(0.040, 0.050, BRACKET_T),
        color=BARE),
    Cylinder("ground_stud", pos=(STUD_XY[0], STUD_XY[1], TOP + STUD_FOOT + STUD_H / 2), radius=STUD_R, height=STUD_H,
             color=BARE),
    # the rest of the flat car's engine round the ground point: the end of the engine (block and head) behind the bracket ledge, and the
    # coolant reservoir beside it
    Box("engine_head", pos=(0.6125, 0.520, TOP + 0.150), size=(0.275, 0.240, 0.300), color=ENGINE),
    Box("coolant_tank", pos=(0.425, 0.5925, TOP + 0.075), size=(0.090, 0.095, 0.150), color=WHITE),
]

# -- clamp geometry (half frame: origin on the pivot pin, +x towards the jaw tips, z up; half A is the +y half) -----
# Sized from a 200 A battery clip datasheet: overall 153 mm, 19 mm wide strip, 38 mm maximum jaw spread.
JH = 0.019  # strip width (vertical when the clamp lies flat)
JAW_T = 0.004
JAW_X0, JAW_X1 = 0.006, 0.060  # jaw from 6 mm to 60 mm in front of the pin
TOOTH = 0.003
H_START = (-0.002, 0.006)  # handle bar centreline, pin end
H_END = (-0.093, 0.036)  # handle end (153 mm overall with the 60 mm jaw)
H_T = 0.008
GRIP_X = -0.055  # where a gripper pinches the handles
OPEN_MAX_DEG = 37.0  # relative opening at the stop: 2 x 18.5 deg -> 38 mm at the jaw tips
SPRING_K = 1.24  # N m/rad on the relative angle
SPRING_PRELOAD = 0.90  # N m with the jaws shut
HALF_MASS = 0.05


def _bar(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    return (a + b) / 2, float(np.linalg.norm(d)), math.atan2(d[1], d[0])


def half_parts(s: float):
    """(name, centre xyz, size xyz, yaw, look) for one half; s = +1 (A) or -1 (B)."""
    hc, hl, hy = _bar((H_START[0], s * H_START[1]), (H_END[0], s * H_END[1]))
    return [
        ("jaw", ((JAW_X0 + JAW_X1) / 2, s * JAW_T / 2, 0.0), (JAW_X1 - JAW_X0, JAW_T, JH), 0.0, "copper"),
        ("tooth", (JAW_X1 - 0.003, -s * TOOTH / 2, 0.0), (0.006, TOOTH, JH), 0.0, "copper"),
        ("web", (0.0, s * 0.004, 0.0), (0.014, 0.008, JH), 0.0, "copper"),
        ("handle", (hc[0], hc[1], 0.0), (hl, H_T, JH), hy, "vinyl"),
    ]


# metal sample points of one half (half frame): along the jaw centreline and across the tooth lip, three heights
_JAW_PTS = {s: np.array([(x, s * JAW_T / 2, z) for x in np.linspace(JAW_X0, JAW_X1, 7)
                         for z in (-JH / 2 + 0.002, 0.0, JH / 2 - 0.002)]
                        + [(x, -s * TOOTH / 2, z) for x in (JAW_X1 - 0.005, JAW_X1 - 0.001)
                           for z in (-JH / 2 + 0.002, 0.0, JH / 2 - 0.002)]) for s in (1.0, -1.0)}
# body sample points (for gripper release distance)
_BODY_PTS = {s: np.array([(0.0, 0.0, 0.0), (GRIP_X, s * 0.024, 0.0), (H_END[0], s * H_END[1], 0.0),
                          (-0.03, s * 0.016, 0.0), ((JAW_X0 + JAW_X1) / 2, s * JAW_T / 2, 0.0)]) for s in (1.0, -1.0)}


class Clamp:
    def __init__(self, name: str, colour: str, pivot, weld_half: str):
        self.name, self.colour, self.pivot, self.weld = name, colour, np.asarray(pivot, float), weld_half
        col = STEEL
        self.a = Box(f"{name}_a", pos=tuple(float(v) for v in self.pivot), size=(0.012, 0.012, JH), color=col,
                     mass=HALF_MASS)
        self.b = Box(f"{name}_b", pos=tuple(float(v) for v in self.pivot), size=(0.012, 0.012, JH), color=col,
                     mass=HALF_MASS)

    def world(self, p):
        return self.pivot + np.array([p[0], p[1], p[2] if len(p) > 2 else 0.0])

    @property
    def grip(self):
        return self.world((GRIP_X, 0.0, 0.0))


# ---- scoring: contact tracking, safe-order rule, score / success ----

TOUCH = 0.003  # metal-to-metal "touch" distance
RELEASE = 0.04  # no gripper TCP within 4 cm of a clamp's body points
LEVEL_COS = math.cos(math.radians(30))  # a clamp biting a terminal clamp near its jaw tips hangs nose-up a little


def attached_to(c: Clamp, s: dict, grips) -> str | None:
    """Name of the post this clamp's spring holds it on, or None."""
    if s is None or grips is None:
        return None
    if any(float(np.linalg.norm(s["body"] - g, axis=1).min()) < RELEASE for g in grips):
        return None
    if not (3.0 <= s["opening"] <= OPEN_MAX_DEG - 1.0) or s["R"][2, 2] < LEVEL_COS:
        return None
    for name, ((x, y), foot, top, _, _) in POSTS.items():
        d = s["R"].T @ (np.array([x, y, TOP + (foot + top) / 2]) - s["pivot"])
        cz = s["pivot"][2] - TOP
        if (JAW_X0 + 0.004 <= d[0] <= JAW_X1 - 0.002 and abs(d[1]) <= 0.006
                and cz - JH / 2 <= top - 0.005 and cz + JH / 2 >= foot + 0.005):
            return name
    return None


def _track() -> None:
    """Called every control step via score(): attachment events, order, touch rules (all latched). The embodiment
    calls success(), distance() and score() each step, all through score(): only the first call per step counts."""
    env = _ENV["env"]
    key = getattr(getattr(env, "unwrapped", None), "common_step_counter", None)
    if key is not None:
        if key == _EP.get("step_key"):
            return
        _EP["step_key"] = key
    grips = grippers()
    states = {c.name: clamp_state(c) for c in CLAMPS}
    if grips is None:
        return
    if any(s is None for s in states.values()):
        _EP["nan"] = True
        return
    att = {}
    for c in CLAMPS:
        p = attached_to(c, states[c.name], grips)
        if p is not None:
            att[c.name] = (c.colour, p)
    sat = [any(v == st for v in att.values()) for st in STEPS]
    # electrical CONTACT, held or not: jaw metal within TOUCH of an allowed post (an attached clamp is in contact too)
    contact = {}
    for c in CLAMPS:
        posts = {p for p in ALLOWED[c.colour] if _post_gap(states[c.name]["jaw"], p) <= TOUCH}
        if c.name in att:
            posts.add(att[c.name][1])
        contact[c.name] = posts
    csat = [any(st[1] in contact[c.name] for c in CLAMPS if c.colour == st[0]) for st in STEPS]
    v = _EP["violation"]
    # wrong terminal for the colour (red on a - post or the ground; black on a + post or the flat battery's -)
    for c in CLAMPS:
        for name in POSTS:
            if name not in ALLOWED[c.colour] and _post_gap(states[c.name]["jaw"], name) <= TOUCH:
                v = v or f"{c.name} jaw touched {name}"
    # metal of a red clamp touching grounded car metal (hold-down, J-bolts, body, strut tower, engine, ground bracket,
    # the - cable lugs) while any clamp touches a post: on a live red lead that is a short to earth
    if any(contact.values()):
        for r in (RED_L, RED_R):
            for o in GROUNDED:
                if _obj_gap(states[r.name]["jaw"], o) <= TOUCH + JAW_T / 2:
                    v = v or f"{r.name} jaw touched grounded {o.name} while live"
    # metal of a red clamp touching metal of a black clamp while any clamp touches a post (held or released)
    if any(contact.values()):
        for r in (RED_L, RED_R):
            for b in (BLACK_L, BLACK_R):
                dd = np.linalg.norm(states[r.name]["jaw"][:, None, :] - states[b.name]["jaw"][None, :, :], axis=2)
                if float(dd.min()) <= JAW_T + TOUCH:
                    v = v or f"{r.name} touched {b.name} while live"
    # safe order, on first CONTACT (a clamp still held in the gripper counts): each connection must be made with
    # exactly the earlier ones in contact; the two red (+) connections, steps 1 and 2, may come in either order,
    # but both before either black one
    for k, st in enumerate(STEPS):
        was = _EP["prev_csat"][k] if "prev_csat" in _EP else False
        if csat[k] and not was:
            if k < 2:
                ok = not any(csat[2:])
            else:
                ok = all(csat[:k]) and not any(csat[k + 1:])
            _EP["events"].append((round(_EP["t"] / 20.0, 2), k + 1, ok))
            if not ok:
                v = v or f"step {k + 1} {st} contact out of order"
            _EP["done"][k] = True
    _EP["prev_csat"] = csat
    _EP["contact"] = {k: sorted(p) for k, p in contact.items() if p}
    _EP["prev_sat"] = sat
    _EP["att"] = att
    # settled: every pivot moved under 2 cm over the last 20 control steps (1 s at 20 Hz). Contact jitter makes the
    # instantaneous body velocity of a spring-loaded clamp noisy, so the pose drift over a second is used instead.
    hist = _EP.setdefault("piv_hist", [])
    hist.append(np.concatenate([states[c.name]["pivot"] for c in CLAMPS]))
    del hist[:-21]
    if len(hist) == 21:
        _EP["speed"] = max(float(np.linalg.norm(hist[-1][i:i + 3] - hist[0][i:i + 3])) for i in range(0, len(hist[0]), 3))
    else:
        _EP["speed"] = 1.0
    _EP["violation"] = v
    _EP["t"] += 1


def score(objects) -> float:
    _track()
    if _EP["violation"] or _EP["nan"] or "prev_sat" not in _EP:
        return 0.0
    sat = _EP["prev_sat"]
    n = int(sat[0]) + int(sat[1])  # red connections, either order
    if n == 2:
        for k in (2, 3):
            if not sat[k]:
                break
            n += 1
    return 0.25 * n


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP.get("speed", 1.0) < 0.02


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    sat = _EP.get("prev_sat", [False] * 4)
    return {"red_dead_pos": int(sat[0]), "red_donor_pos": int(sat[1]), "black_donor_neg": int(sat[2]),
            "black_ground": int(sat[3]), "no_violation": int(not _EP.get("violation"))}


R_LEAD = 0.005  # 10 mm over the insulation
LEAD_LEN = 2.0  # both leads, between the clamps (a real 2 m, 200 A set: Maypole MP3515)
LEAD_SEG = 0.04  # rigid segment length
LEAD_KG_PER_M = 0.12
CLAMP_Z = TOP + JH / 2 + 0.0005
LEAD_Z = TOP + R_LEAD + 0.0005
FERRULE_GAP = 0.022  # cable tip to the clamp's handle end (ferrule 20 mm + 2 mm)
FERRULE = (0.020, 0.012, 0.012)


def _lead(colour: str, x: float, length: float):
    """The lead as authored: straight along -y at x, a clamp at each end (jaws +x). This only fixes how each clamp
    is welded to its ferrule; every reset lays both leads in the pile (see the pile section below)."""
    y0 = length / 2
    he_left = np.array([x, y0 + FERRULE_GAP])  # handle end of the left clamp's B half (the -y half)
    he_right = np.array([x, -y0 - FERRULE_GAP])  # handle end of the right clamp's A half (the +y half)
    left = Clamp(f"{colour}_l", colour, (he_left[0] - H_END[0], he_left[1] + H_END[1], CLAMP_Z), "b")
    right = Clamp(f"{colour}_r", colour, (he_right[0] - H_END[0], he_right[1] - H_END[1], CLAMP_Z), "a")
    col = RED if colour == "red" else BLACK
    ferrule = Plug(size=FERRULE, mass=0.01, color=col)
    spec = cable(f"{colour}_lead", start=(x, y0, LEAD_Z), yaw=-math.pi / 2, length=length,
                 radius=R_LEAD, segment=LEAD_SEG, cable_mass=LEAD_KG_PER_M * length, start_plug=ferrule,
                 end_plug=ferrule, color=col, bend_stiffness=0.01, bend_damping=0.001)
    return spec, left, right


RED_LEAD, RED_L, RED_R = _lead("red", 0.22, LEAD_LEN)
BLACK_LEAD, BLACK_L, BLACK_R = _lead("black", 0.13, LEAD_LEN)
LEADS = {"red": RED_LEAD, "black": BLACK_LEAD}
CLAMPS = [RED_L, RED_R, BLACK_L, BLACK_R]

# -- the pile: both leads dropped in a loose heap in front of the batteries -------------------------------------------
# Each lead is laid as an irregular hank of loops (as it comes out of the bag) between its two clamps, which lie flat
# on the table at the edge of the heap. The black lead is laid first, the red one after it, so red lies over black
# wherever they cross; a lead also lies over its own earlier loops. Every crossing is lifted just clear of what is
# under it and the heap then drops and settles under gravity (PILE_SETTLE_S) before the first observation. The loop
# wobble comes from the fixed seed PILE_SEED, so the pile is the same on every reset; with a reset seed and
# object_jitter > 0 the whole pile is shifted by up to object_jitter in x and y, like the other loose objects.
PILE_SEED = 381
PILE_SETTLE_S = 3.0
# designed clamp spots: pivot (x, y), yaw of the jaws (rad, 0 = pointing away from the robot)
PILE_CLAMPS = {"red_l": (0.262, 0.255, 0.25), "red_r": (0.255, -0.255, -0.20),
               "black_l": (0.205, 0.335, -0.20), "black_r": (0.200, -0.330, 0.25)}
# hank: centre (x, y), semi-axes (along x, along y), ellipse turn (rad); laid black first. Full turns between the
# two tails, and the winding sense (+1 counter-clockwise from above).
PILE_TURNS = {"black": 2, "red": 1}
PILE_SENSE = {"black": 1.0, "red": -1.0}
PILE_HANKS = {"black": ((0.154, -0.008), (0.065, 0.110), 0.28), "red": ((0.202, 0.006), (0.060, 0.100), 0.12)}
PILE_SLOPE = 0.2
_CLEAR = 2 * R_LEAD / math.cos(math.pi / 8) + 0.004  # crossing: centre-to-centre lift (octagonal colliders + 4 mm)


def _rz(t: float) -> np.ndarray:
    return np.array([[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]])


def _clamp_end(name: str, dxy=(0.0, 0.0)):
    """Cable tip (xy) and the unit direction from it into the cable, for a clamp at its designed spot."""
    x, y, yaw = PILE_CLAMPS[name]
    p = np.array([x + dxy[0], y + dxy[1]])
    R = _rz(yaw)[:2, :2]
    s = -1.0 if name.endswith("_l") else 1.0  # the lead enters the B half (start, _l) or the A half (end, _r)
    u = R @ np.array([0.0, s])
    he = p + R @ np.array([H_END[0], s * H_END[1]])
    return he + u * FERRULE_GAP, u


def _catmull(pts: np.ndarray, step: float = 0.003) -> np.ndarray:
    """Centripetal Catmull-Rom curve through pts (N, 2), sampled about every ``step`` m."""
    P = np.vstack([2 * pts[0] - pts[1], pts, 2 * pts[-1] - pts[-2]])
    out = [P[1]]
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        t0 = 0.0
        t1 = t0 + np.linalg.norm(p1 - p0) ** 0.5
        t2 = t1 + np.linalg.norm(p2 - p1) ** 0.5
        t3 = t2 + np.linalg.norm(p3 - p2) ** 0.5
        k = max(2, int(np.linalg.norm(p2 - p1) / step))
        for t in np.linspace(t1, t2, k + 1)[1:]:
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return np.array(out)


def _ellipse(c, ax, ay, rot, a, sense):
    """Point and unit tangent (in the winding sense) of the hank ellipse at angle a."""
    R = _rz(rot)[:2, :2]
    p = c + R @ np.array([ax * math.cos(a), ay * math.sin(a)])
    t = sense * (R @ np.array([-ax * math.sin(a), ay * math.cos(a)]))
    return p, t / np.linalg.norm(t)


def _join(c, ax, ay, rot, tip, u, sense, into: bool):
    """Angle on the ellipse where a tail leaving ``tip`` along ``u`` runs on into the winding (``into``) or where the
    winding runs off into a tail arriving at ``tip`` against ``u``: near the tail's line and tangent to it."""
    best = (1e9, 0.0)
    for a in np.linspace(-math.pi, math.pi, 181):
        p, t = _ellipse(c, ax, ay, rot, a, sense)
        d = p - tip
        along = d @ u
        if along < 0.06:
            continue
        side = np.linalg.norm(d - along * u)
        align = t @ (u if into else -u)
        cost = side + 0.15 * (1.0 - align) + 0.2 * max(0.0, along - 0.20)
        if cost < best[0]:
            best = (cost, a)
    return best[1]


HANK_PTS_PER_TURN = 10


def _hank_path(colour: str, scale: float, rng_state: int, dxy) -> np.ndarray:
    """2D centreline of one lead: from its start tip straight on into an irregular hank of loops, round it about
    PILE_TURNS times, and off to its end tip. ``scale`` sizes the loops (it is tuned so the lead is exactly long
    enough to reach both clamps)."""
    rng = np.random.default_rng(rng_state)
    (cx, cy), (ax, ay), rot = PILE_HANKS[colour]
    c = np.array([cx + dxy[0], cy + dxy[1]])
    ts, us = _clamp_end(f"{colour}_l", dxy)
    te, ue = _clamp_end(f"{colour}_r", dxy)
    sense = PILE_SENSE[colour]
    a0 = _join(c, ax, ay, rot, ts, us, sense, True)  # chosen on the nominal ellipse, so the length is smooth in scale
    a1 = _join(c, ax, ay, rot, te, ue, sense, False)
    ax, ay = ax * scale, ay * scale
    sweep = (sense * (a1 - a0)) % (2 * math.pi) + 2 * math.pi * PILE_TURNS[colour]
    n = int(math.ceil(sweep / (2 * math.pi) * HANK_PTS_PER_TURN))
    wob = rng.normal(0.0, 1.0, (n + 1, 3))
    wob[0] = wob[-1] = 0.0
    pts = [ts, ts + 0.05 * us]
    for k in range(n + 1):
        a = a0 + sense * sweep * k / n
        p, _ = _ellipse(c, ax * (1 + 0.08 * wob[k, 0]), ay * (1 + 0.08 * wob[k, 0]), rot, a, sense)
        pts.append(p + np.array([0.010 * wob[k, 1], 0.012 * wob[k, 2]]))
    pts += [te + 0.05 * ue, te]
    p = _catmull(np.array(pts))
    # relax the bends (no turn tighter than the stiff lead takes): Laplacian smoothing, the 5 cm tails held straight
    k0 = int(np.searchsorted(np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))], 0.05))
    for _ in range(200):
        p[k0:-k0] += 0.5 * (0.5 * (p[k0 - 1:-k0 - 1] + p[k0 + 1:len(p) - k0 + 1]) - p[k0:-k0])
    return p


def _drape(paths: list, tail: float = 0.08) -> list:
    """Heights for 2D centrelines laid one after the other. The last ``tail`` m at each end (next to the clamps) lie
    flat on the table and go down first; then each lead's body is laid in order, each point on the table or just
    over whatever is already there (the tails, earlier leads, and its own loops more than 8 cm back along it), with
    slopes of at most PILE_SLOPE so the crossings are smooth."""
    zs = [np.full(len(p), LEAD_Z) for p in paths]
    arcs = [np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))] for p in paths]
    tails = [(a < tail) | (a > a[-1] - tail) for a in arcs]
    tail_xy = np.vstack([p[t] for p, t in zip(paths, tails)])
    for _ in range(8):
        changed = False
        for ci, p in enumerate(paths):
            z = zs[ci]
            for i in np.nonzero(~tails[ci])[0]:
                req = LEAD_Z
                d = np.linalg.norm(tail_xy - p[i], axis=1)
                if (d < _CLEAR).any():
                    req = LEAD_Z + float(np.sqrt(_CLEAR ** 2 - d[d < _CLEAR].min() ** 2))
                for cj in range(ci + 1):
                    if cj < ci:
                        m = ~tails[cj]
                    else:
                        m = np.zeros(len(p), bool)
                        m[:max(0, int(np.searchsorted(arcs[ci], arcs[ci][i] - 0.08)))] = True
                        m &= ~tails[ci]
                    d = np.linalg.norm(paths[cj][m] - p[i], axis=1)
                    near = d < _CLEAR
                    if near.any():
                        req = max(req, float((zs[cj][m][near] + np.sqrt(_CLEAR ** 2 - d[near] ** 2)).max()))
                if req > z[i] + 1e-6:
                    z[i] = req
                    changed = True
            ds = np.diff(arcs[ci])
            body = ~tails[ci]
            for i in range(1, len(z)):
                if body[i]:
                    z[i] = max(z[i], z[i - 1] - PILE_SLOPE * ds[i - 1])
            for i in range(len(z) - 2, -1, -1):
                if body[i]:
                    z[i] = max(z[i], z[i + 1] - PILE_SLOPE * ds[i])
        if not changed:
            break
    return [np.c_[p, z] for p, z in zip(paths, zs)]


def _chord_end(spec, path3) -> float:
    """How far past the path's end (along its last direction, m) the chain of segments laid along path3 ends."""
    import cableshape

    sh = cableshape.shape_points(spec, path3)
    tip = sh.centres[-1] + sh.dirs[-1] * spec.seg_len / 2
    t = path3[-1] - path3[-6]
    return float((tip - path3[-1]) @ (t / np.linalg.norm(t)))


_PILE_CACHE: dict = {}


def pile_layout(dxy=(0.0, 0.0)) -> dict:
    """{lead name: 3D centreline (N, 3)} for the pile shifted by dxy. Pure numpy; cached."""
    key = (round(float(dxy[0]), 4), round(float(dxy[1]), 4))
    if key in _PILE_CACHE:
        return _PILE_CACHE[key]
    rng = np.random.default_rng(PILE_SEED)
    seeds = {c: int(rng.integers(1 << 30)) for c in ("black", "red")}
    laid = []
    for colour in ("black", "red"):
        spec = LEADS[colour]

        def arc(sc):
            p3 = _drape(laid + [_hank_path(colour, sc, seeds[colour], dxy)])[-1]
            return float(np.linalg.norm(np.diff(p3, axis=0), axis=1).sum()), p3

        target = spec.length
        for _ in range(8):  # loop size so that the 50 segments end exactly at the end clamp's tip
            lo, hi = 0.3, 2.6
            for _ in range(16):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if arc(mid)[0] < target else (lo, mid)
            over = _chord_end(spec, arc(mid)[1])
            if abs(over) < 0.001:
                break
            target += over
        laid.append(_hank_path(colour, mid, seeds[colour], dxy))
    p3 = _drape(laid)
    out = {"black_lead": p3[0], "red_lead": p3[1]}
    _PILE_CACHE[key] = out
    return out


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_jl_hook", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != BLACK_LEAD.name:  # the last object: every other prim exists now
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        looks = f"{root}/JLooks"
        mats = {}
        for key, col, rough, met in (("copper", COPPER, 0.35, 0.6), ("red", RED, 0.5, 0.0), ("black", BLACK, 0.5, 0.0),
                                     ("lead", LEAD_GREY, 0.45, 0.3)):
            p = f"{looks}/{key}"
            m = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=rough, metallic=met)
            m.func(p, m)
            mats[key] = p
        jaw_phys = f"{looks}/jaw_phys"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.7)
        pm.func(jaw_phys, pm)
        vinyl_phys = f"{looks}/vinyl_phys"
        pv = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0)
        pv.func(vinyl_phys, pv)

        def cube(path, c, s, yaw, look, phys=None, patch=0.0):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            g.AddOrientOp().Set(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            if patch:
                cp.CreateTorsionalPatchRadiusAttr(patch)
                cp.CreateMinTorsionalPatchRadiusAttr(patch / 2)
            if phys:
                bind_physics_material(str(g.GetPath()), phys, stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)

        def xf(path):
            return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)

        def filt(p0, p1):
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(p0)).CreateFilteredPairsRel().AddTarget(Sdf.Path(p1))

        for c in CLAMPS:
            vinyl = mats["red"] if c.colour == "red" else mats["black"]
            pa, pb = f"{root}/{c.a.name}", f"{root}/{c.b.name}"
            for s, p in ((1.0, pa), (-1.0, pb)):
                for name, ctr, size, yaw, look in half_parts(s):
                    cube(f"{p}/{name}", ctr, size, yaw, mats["copper"] if look == "copper" else vinyl,
                         jaw_phys if look == "copper" else vinyl_phys, patch=0.0 if look == "copper" else 0.008)
            filt(pa, pb)
            j = UsdPhysics.RevoluteJoint.Define(stage, f"{pa}/pivot_pin")
            j.CreateBody0Rel().SetTargets([Sdf.Path(pa)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(pb)])
            j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateAxisAttr("Z")
            j.CreateLowerLimitAttr(-OPEN_MAX_DEG)
            j.CreateUpperLimitAttr(0.0)
            j.CreateExcludeFromArticulationAttr(True)
            drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
            drv.CreateTypeAttr("force")
            drv.CreateStiffnessAttr(SPRING_K * math.pi / 180)  # USD angular gains are per degree
            drv.CreateDampingAttr(0.004 * math.pi / 180)
            drv.CreateTargetPositionAttr(math.degrees(SPRING_PRELOAD / SPRING_K))
        # weld each lead's ferrules to its clamps, at their authored relative pose
        for colour, spec in LEADS.items():
            cp = f"{root}/{spec.name}"
            lc, rc = (RED_L, RED_R) if colour == "red" else (BLACK_L, BLACK_R)
            for end, cl in (("start", lc), ("end", rc)):
                plug = f"{cp}/{end}_plug"
                half = f"{root}/{(cl.b if cl.weld == 'b' else cl.a).name}"
                rel = xf(plug) * xf(half).GetInverse()
                t, q = rel.ExtractTranslation(), rel.ExtractRotationQuat()
                w = UsdPhysics.FixedJoint.Define(stage, f"{cp}/{end}_ferrule_weld")
                w.CreateBody0Rel().SetTargets([Sdf.Path(half)])
                w.CreateBody1Rel().SetTargets([Sdf.Path(plug)])
                w.CreateLocalPos0Attr(Gf.Vec3f(*map(float, t)))
                w.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), *map(float, q.GetImaginary())))
                w.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
                w.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                w.CreateExcludeFromArticulationAttr(True)
                filt(half, plug)
                other = f"{root}/{(cl.a if cl.weld == 'b' else cl.b).name}"
                filt(other, plug)
        # battery terminals (static convex hulls) on the lids: each post with the car's terminal clamp round it
        for name, ((x, y), foot, top, r0, r1) in POSTS.items():
            if name == "ground":
                continue
            bat = f"{root}/{name.split('_')[0]}_lid"
            cx, cy = BATS[name.split("_")[0]]
            pts = []
            for z, r in ((0.0, r0), (top - foot, r1)):
                for k in range(24):
                    a_ = 2 * math.pi * k / 24
                    pts.append(Gf.Vec3f(r * math.cos(a_), r * math.sin(a_), z))
            mesh = UsdGeom.Mesh.Define(stage, f"{bat}/post_{name.split('_')[1]}")
            mesh.CreatePointsAttr(pts)
            idx = list(range(23, -1, -1)) + list(range(24, 48))
            for k in range(24):
                k1 = (k + 1) % 24
                idx += [k, k1, 24 + k1, 24 + k]
            mesh.CreateFaceVertexCountsAttr([24, 24] + [4] * 24)
            mesh.CreateFaceVertexIndicesAttr(idx)
            mesh.AddTranslateOp().Set(Gf.Vec3d(x - cx, y - cy, LID_T / 2))
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
            bind_visual_material(str(mesh.GetPath()), mats["lead"], stage=stage)
            bind_physics_material(str(mesh.GetPath()), jaw_phys, stage=stage)
        print("[jl] 4 sprung clamps, 2 leads welded, 4 tapered posts authored", flush=True)
        return prim

    spawn._jl_hook = True
    cablekit._spawn_cable = spawn


_install_spawn()

OBJECTS = [*_battery("dead"), *_battery("donor"), *_bay("dead"), *_bay("donor"), *ENGINE_OBJS,
           *(h for c in CLAMPS for h in (c.a, c.b)), RED_LEAD, BLACK_LEAD]

# bare metal bonded to the car body or engine (earth), in both cars: a live red jaw on any of it is a short
_GROUND_TAGS = ("holddown", "jbolt", "inner_wing", "strut_tower", "wing_edge", "engine_block", "engine_head",
                "ground_bracket", "neg_lug")
GROUNDED = [o for o in OBJECTS if any(t in o.name for t in _GROUND_TAGS)]

TWIN_EXTRA = [
    {"name": "booster_clamp", "kind": "body", "size_mm": [153, 80, 19], "colour": "copper jaws, red/black vinyl handles",
     "note": "x4; 200 A battery clip (153 mm, 38 mm max jaw spread); two halves on a pin with a torsion spring"},
    {"name": "clamp_pivot", "kind": "joint", "real": "riveted pivot pin + torsion spring",
     "note": "0.9 N m preload shut, 1.24 N m/rad, 37 deg stop (~30 N handle squeeze at full open)"},
    {"name": "battery_posts", "kind": "body", "size_mm": [19, 19, 18], "colour": "lead grey",
     "note": "SAE tapered posts: + 17.5 mm top / 19.0 foot, - 15.9 / 17.4; on a real 26R battery"},
    {"name": "ferrule", "kind": "joint", "real": "crimped cable lug inside the handle", "note": "lead fixed to one half"},
]

# -- live state ------------------------------------------------------------------------------------------------------
_ENV = {"env": None}
_EP: dict = {}


def _reset_episode() -> None:
    _EP.clear()
    _EP.update(violation="", events=[], prev=set(), done=[False] * 4, t=0, nan=False)


_reset_episode()


def _install_hooks():
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_jl_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._jl_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()


# -- laying the pile at every reset ---------------------------------------------------------------------------------


def _frame_quat(R) -> tuple:
    import cableshape

    return tuple(float(v) for v in cableshape._mat_to_quat_xyzw(R))


def _authored_plug(spec, end: str):
    """Pose (pos, rot) of a ferrule as authored (straight lead along -y)."""
    d = spec.direction
    R = np.c_[d, np.cross([0.0, 0.0, 1.0], d), [0.0, 0.0, 1.0]]
    p0 = np.asarray(spec.start, float)
    pos = p0 - d * FERRULE[0] / 2 if end == "start" else p0 + d * (spec.length + FERRULE[0] / 2)
    return pos, R


def pile_writes(dxy=(0.0, 0.0)) -> dict:
    """Everything a reset writes (pure numpy): per lead the root pose and the joint coordinates; per clamp the pose.
    Each end segment's frame fixes its ferrule, and through the weld the clamp, so the clamps lie where the lead
    leaves them. The chain cannot twist, so a lead that climbs over crossings arrives at its far end slightly rolled;
    the roll is split between the two ends (seg_0 is turned back by half of it)."""
    import cableshape

    lay = pile_layout(dxy)
    out = {"leads": {}, "clamps": {}}
    for colour, spec in LEADS.items():
        sh = cableshape.shape_points(spec, lay[spec.name])
        F = sh.frames
        roll = math.atan2(float(F[-1][2, 1]), float(F[-1][2, 2]))  # end frame's turn about its own axis
        b = roll / 2
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(b), -math.sin(b)], [0.0, math.sin(b), math.cos(b)]])
        F = [f @ Rx for f in F]
        q = np.array([cableshape._mat_to_rotvec(F[i].T @ F[i + 1])[1:] for i in range(len(F) - 1)])
        L = spec.seg_len
        plugs = {"start": (sh.centres[0] - sh.dirs[0] * (L / 2 + FERRULE[0] / 2), F[0]),
                 "end": (sh.centres[-1] + sh.dirs[-1] * (L / 2 + FERRULE[0] / 2), F[-1])}
        out["leads"][spec.name] = dict(root=(sh.centres[0], F[0]), q=q, roll_deg=math.degrees(roll),
                                       maxq=float(np.abs(q).max()))
        lc, rc = (RED_L, RED_R) if colour == "red" else (BLACK_L, BLACK_R)
        for end, cl in (("start", lc), ("end", rc)):
            pa, Ra = _authored_plug(spec, end)
            pn, Rn = plugs[end]
            out["clamps"][cl.name] = (pn + Rn @ Ra.T @ (cl.pivot - pa), Rn @ Ra.T)
    return out


_PILE_INFO: dict = {}


def _lay_pile(emb, seed) -> None:
    import torch

    import cableshape

    env, base, ready = cableshape._ctx(emb)
    dev = base.device
    origin = cableshape._t(base.scene.env_origins)[0]
    jit = float(getattr(emb, "object_jitter", 0.0) or 0.0)
    dxy = (0.0, 0.0)
    if seed is not None and jit > 0:
        dxy = tuple(np.random.default_rng([int(seed), PILE_SEED]).uniform(-jit, jit, 2))
    W = pile_writes(dxy)

    def pose_t(p, R, lift=0.0):
        t = torch.zeros((1, 7), dtype=torch.float32, device=dev)
        t[0, :3] = origin + torch.tensor(np.asarray(p, float) + [0.0, 0.0, lift], dtype=torch.float32, device=dev)
        t[0, 3:7] = torch.tensor(_frame_quat(R), dtype=torch.float32, device=dev)
        return t

    lift = 0.0015
    for colour, spec in LEADS.items():
        w = W["leads"][spec.name]
        art = base.scene[spec.name]
        i1, i2 = cableshape._dof_index(art)
        jp = cableshape._t(art.data.joint_pos)
        q = torch.zeros_like(jp)
        q[0, i1] = torch.tensor(w["q"][:, 0], dtype=q.dtype, device=dev)
        q[0, i2] = torch.tensor(w["q"][:, 1], dtype=q.dtype, device=dev)
        art.write_root_pose_to_sim_index(root_pose=pose_t(*w["root"], lift))
        art.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=dev))
        art.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q))
        # coil memory: the lead keeps the turn of its loops (in-plane bends), not the lifts over crossings
        tgt = torch.zeros_like(q)
        tgt[0, i2] = q[0, i2]
        art.set_joint_position_target_index(target=tgt)
    for c in CLAMPS:
        p, R = W["clamps"][c.name]
        for h in (c.a, c.b):
            o = base.scene[h.name]
            o.write_root_pose_to_sim_index(root_pose=pose_t(p, R, lift))
            o.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=dev))
    base.scene.write_data_to_sim()
    base.sim.step(render=False)
    base.scene.update(dt=base.physics_dt)
    for _ in range(max(1, round(PILE_SETTLE_S / base.step_dt))):
        env.step(ready)
    _PILE_INFO.update(dxy=dxy, rolls={k: round(v["roll_deg"], 1) for k, v in W["leads"].items()})
    from cablekit import _cable_state

    ext = {}
    for spec in LEADS.values():
        pts = _cable_state(env, spec).points
        if not np.isfinite(pts).all():
            raise RuntimeError(f"{spec.name} went NaN while the pile settled")
        ext[spec.name] = (np.round(pts.min(axis=0)[:2], 3).tolist(), np.round(pts.max(axis=0)[:2], 3).tolist(),
                          round(float(pts[:, 2].max() - TOP), 3))
    _PILE_INFO["extent"] = ext
    print(f"[jl] pile laid (shift {np.round(dxy, 3).tolist()}, end rolls {_PILE_INFO['rolls']}) and settled "
          f"{PILE_SETTLE_S} s: xy min, xy max, top above the table {ext}", flush=True)


def _install_pile_reset() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_jl_pile", False):
        return
    base_reset = cls.reset

    def reset(self, scene, *a, seed=None, **k):
        base_reset(self, scene, *a, seed=seed, **k)
        _lay_pile(self, seed)
        for _ in range(4):
            self._base.sim.render()
        self._t = 0.0
        return self._observation()

    reset._jl_pile = True
    reset._task_reset_hooks = getattr(base_reset, "_task_reset_hooks", [])
    cls.reset = reset


_install_pile_reset()


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _body(name):
    env = _ENV["env"]
    if env is None:
        return None
    base = env.unwrapped
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    ro = base.scene.rigid_objects[name].data
    p = ro.root_pose_w.torch[0].detach().cpu().numpy().astype(np.float64)
    v = ro.root_lin_vel_w.torch[0].detach().cpu().numpy().astype(np.float64)
    if not (np.isfinite(p).all() and np.isfinite(v).all()):
        return None
    return p[:3] - o, _rot(p[3:7]), v


def grippers():
    env = _ENV["env"]
    if env is None:
        return None
    from yam_arena.runtime import tcp_world
    base = env.unwrapped
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - o for k in ("left_robot", "right_robot")]


def clamp_state(c: Clamp):
    """Live clamp: pivot, mid-frame rotation, opening (deg, + = open), jaw metal points, body points, speed."""
    a, b = _body(c.a.name), _body(c.b.name)
    if a is None or b is None:
        return None
    rel = a[1].T @ b[1]
    opening = -math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    ha = math.radians(opening) / 2
    cz, sz = math.cos(-ha), math.sin(-ha)  # mid frame = A turned back by half the opening
    Rm = a[1] @ np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    piv = (a[0] + b[0]) / 2
    jaw = np.vstack([a[0] + _JAW_PTS[1.0] @ a[1].T, b[0] + _JAW_PTS[-1.0] @ b[1].T])
    body = np.vstack([a[0] + _BODY_PTS[1.0] @ a[1].T, b[0] + _BODY_PTS[-1.0] @ b[1].T])
    return dict(pivot=piv, R=Rm, opening=opening, jaw=jaw, body=body,
                speed=max(float(np.linalg.norm(a[2])), float(np.linalg.norm(b[2]))))


def _obj_gap(pts: np.ndarray, o) -> float:
    """Smallest distance from points to a fixed box (no yaw) or upright cylinder of the scene."""
    p = np.asarray(o.pos)
    if isinstance(o, Box):
        q = np.abs(pts - p) - np.asarray(o.size) / 2
        out = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    else:
        rad = np.linalg.norm(pts[:, :2] - p[:2], axis=1) - o.radius
        dz = np.abs(pts[:, 2] - p[2]) - o.height / 2
        q = np.stack([rad, dz], axis=1)
        out = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    return float(out.min())


def _post_gap(pts: np.ndarray, name: str) -> float:
    """Smallest distance from points to the post's solid (truncated cone), minus half a jaw thickness."""
    (x, y), foot, top, r0, r1 = POSTS[name]
    z = pts[:, 2] - TOP
    zc = np.clip(z, foot, top)
    r = r0 + (r1 - r0) * (zc - foot) / (top - foot)
    dr = np.maximum(np.hypot(pts[:, 0] - x, pts[:, 1] - y) - r, 0.0)
    dz = np.abs(z - zc)
    return float(np.hypot(dr, dz).min()) - JAW_T / 2


VARIATION = {"pile": {"dx": 0.0, "dy": 0.0, "note": "with a reset seed and object_jitter > 0 the whole pile shifts by up to "
                               "object_jitter in x and y (_lay_pile); the loops themselves come from PILE_SEED"}}
