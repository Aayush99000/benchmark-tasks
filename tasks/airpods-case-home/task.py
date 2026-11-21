"""Put both AirPods Pro 2 earbuds back in their MagSafe charging case, each stem-first into its own well, and shut the lid so its magnet holds it.

An Apple AirPods Pro 2 case (45.2 x 60.6 x 21.7 mm, 50.8 g) stands on the bench with its lid open at 115 degrees, hinged along its back (+x) top edge. Two keyed wells in its top
take the buds (5.3 g each, 30.9 mm long, 21.8 mm across the head), which start loose on the bench. The right hand holds the case low at one end while the left hand seats the buds
and pushes the lid over and shut with a fingertip.

The case is one free mechanism (frame plus the lid on a hinge). The wells are real cavities (a bore with a chamfered mouth, two key slots), the buds have a head, a stem, a side rib
and a front bump, so a bud goes in only stem first, in its own well, the right way round; one left proud holds the lid open because the lid's cavity is only 1 mm taller than a seated head.
Code: two magnets and the scorer. A well's magnet pulls a bud 0.4 N toward its seat, only within 7 mm of it, and the case takes the reaction; the lid's\nmagnet is a drive in its own hinge, 1 N at the lid's edge over its last 5 degrees, so frame and lid pull on each other.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,left_only,swap,one_bud,proud
Final:  python demokit.py task.py OUT
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put both earbuds back in their charging case and close the lid."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HOLD_STEPS = 20  # 1 s at 20 Hz
MM = 1e-3

LOOKS = {"case": m.Look(m.rgb(0.93, 0.93, 0.92), metallic=0.0, roughness=0.35), "lid": m.Look(m.rgb(0.95, 0.95, 0.94), metallic=0.0, roughness=0.3),
         "bud": m.Look(m.rgb(0.97, 0.97, 0.97), metallic=0.0, roughness=0.25), "tip": m.Look(m.rgb(0.8, 0.8, 0.82), metallic=0.0, roughness=0.6),
         "well": m.Look(m.rgb(0.45, 0.46, 0.5), metallic=0.1, roughness=0.5)}

# -- the case (Apple: 45.2 high x 60.6 wide x 21.7 deep, 50.8 g). Origin: the middle of its footprint, on the bench. Width along y, depth along x, hinge at the back (+x) ------------------
CX, CY = float(os.environ.get("AP_CX", "0.34")), float(os.environ.get("AP_CY", "0.02"))
HX, HY = 0.01085, 0.0303  # half depth, half width: Apple's 21.7 x 60.6 mm
LID_HY = 0.0299  # the lid is 59.8 wide: 0.4 mm clear of each end of the 60.6 mm body
H_TOT = 0.0452
RIM = 0.0347  # the body's top, the wells' rim; the lid's cap above it is 10.5 mm
CAP = H_TOT - RIM
PLATE = 0.0015
WALL = 0.0012
WELL_Y = 0.017  # well centres at +-17 mm: heads 21.8 wide leave 12.2 mm between them
BORE = 0.0046  # bore half width in y (stem 7.6 mm across y)
BOREX = 0.0036  # and in x (stem 5.7 mm)
BORE_FLOOR = RIM - 0.0238
LEAD = 0.0025  # the bore's chamfered mouth: 2.5 mm wider each side at the rim, 4 mm deep (58 degrees)
FUNX, FUNY = BOREX + LEAD, BORE + LEAD
FUN_FLOOR = RIM - 0.004
RIB_Y1 = 0.006  # the side rib stands 2.2 mm off the stem, more than the stem's play in its bore (2 x 0.8 mm), so a bud in the wrong well can't pass it
KEY_Z0, KEY_Z1 = -0.010, -0.002  # rib and front bump run down the stem's top 10 mm: they meet the well only over the last 6 mm home
SLOT = 0.0023  # key slots' half width: the rib (3.0 mm) and the bump (3.0 mm) with 0.8 mm a side, as the stem in its bore
M_CASE, M_LID = 0.0428, 0.008
HINGE = np.array([HX, 0.0, RIM])  # in the case frame (x, y, z): the lid's hinge line runs along y
OPEN_DEG = 115.0

# bud (Apple: 30.9 long, 21.8 wide, 24.0 deep with the ear tip, 5.3 g). Origin: the middle of the head's underside (the seat plane); the stem (Apple's drawing: 5.7 wide, 23.3 long)
# hangs below along -z; head 7.6 high, so the bud is 30.9 long; head 21.8 wide (y) and 17.3 deep (x): the most a 21.7 mm case holds under its lid with the bore's play (the ear tip is not modelled).
BUD_M = float(os.environ.get("AP_BUDM", "0.0053"))
HEAD_A, HEAD_B, HEAD_H = 0.00865, 0.0109, 0.0076
STEM_X, STEM_Y, STEM_L = 0.0057, 0.0076, 0.0233
BUDS = {"bud_l": +1.0, "bud_r": -1.0}  # the left earbud's well is at +y, the right's at -y


def _cells(outline, cuts):
    """Axis-aligned boxes (x0, x1, y0, y1) covering `outline` (x0, x1, y0, y1) minus the rectangles in `cuts`."""
    ox0, ox1, oy0, oy1 = outline
    xs = sorted({ox0, ox1, *[c[0] for c in cuts if ox0 < c[0] < ox1], *[c[1] for c in cuts if ox0 < c[1] < ox1]})
    ys = sorted({oy0, oy1, *[c[2] for c in cuts if oy0 < c[2] < oy1], *[c[3] for c in cuts if oy0 < c[3] < oy1]})
    out = []
    for xa, xb in zip(xs[:-1], xs[1:]):
        run = None
        for ya, yb in zip(ys[:-1], ys[1:]):
            cx, cy = (xa + xb) / 2, (ya + yb) / 2
            solid = not any(c[0] < cx < c[1] and c[2] < cy < c[3] for c in cuts)
            if solid:
                run = (run[0] if run else ya, yb)
            else:
                if run:
                    out.append((xa, xb, run[0], run[1]))
                run = None
        if run:
            out.append((xa, xb, run[0], run[1]))
    return out


def _bore_cuts():
    cuts = []
    for s in (+1.0, -1.0):
        yc = s * WELL_Y
        cuts += [(-BOREX, BOREX, yc - BORE, yc + BORE)]
        cuts += [(-SLOT, SLOT, yc + BORE - 0.0001, yc + RIB_Y1 + 0.0008) if s > 0 else (-SLOT, SLOT, yc - RIB_Y1 - 0.0008, yc - BORE + 0.0001)]  # rib slot on the outer side
        cuts += [(-0.0056, -BOREX + 0.0001, yc - SLOT, yc + SLOT)]  # bump slot at the front
    return cuts


def _fun_cuts():
    return [(-FUNX, FUNX, s * WELL_Y - FUNY, s * WELL_Y + FUNY) for s in (+1.0, -1.0)] + _bore_cuts()


def _hull_mesh(points) -> m.Mesh:
    """A closed mesh of the convex hull of a few points (brute force, outward faces)."""
    p = np.asarray(points, float)
    c = p.mean(axis=0)
    faces = []
    n = len(p)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(p[j] - p[i], p[k] - p[i])
                if np.linalg.norm(nrm) < 1e-12:
                    continue
                s = (p - p[i]) @ nrm
                if np.all(s <= 1e-12) or np.all(s >= -1e-12):
                    if (c - p[i]) @ nrm > 0:
                        faces.append((i, k, j))
                    else:
                        faces.append((i, j, k))
    return m.Mesh(p, np.array(faces, int))


def _wedges():
    """The chamfer round each bore's mouth: sloped wedges from the bore's edge at the funnel floor up to the rim, cut where the key slots run."""
    out = []
    zf, zr = FUN_FLOOR, RIM
    for s in (+1.0, -1.0):
        yc = s * WELL_Y
        y0, y1 = yc - FUNY, yc + FUNY
        # front (-x) side, split round the bump slot; back (+x) side whole
        for ya, yb in ((y0, yc - SLOT), (yc + SLOT, y1)):
            out.append([(-FUNX, ya, zf), (-FUNX, yb, zf), (-BOREX, ya, zf), (-BOREX, yb, zf), (-FUNX, ya, zr), (-FUNX, yb, zr)])
        out.append([(FUNX, y0, zf), (FUNX, y1, zf), (BOREX, y0, zf), (BOREX, y1, zf), (FUNX, y0, zr), (FUNX, y1, zr)])
        # inner side whole; outer side split round the rib slot
        for side, xs in ((-s, ((-BOREX, BOREX),)), (s, ((-BOREX, -SLOT), (SLOT, BOREX)))):
            ye, yo = yc + side * BORE, yc + side * FUNY
            for xa, xb in xs:
                out.append([(xa, ye, zf), (xb, ye, zf), (xa, yo, zf), (xb, yo, zf), (xa, yo, zr), (xb, yo, zr)])
    return [_hull_mesh(w) for w in out]


def _body_boxes():
    out = [(-HX, HX, -HY, HY, 0.0, BORE_FLOOR)]
    out += [(*c, BORE_FLOOR, FUN_FLOOR) for c in _cells((-HX, HX, -HY, HY), _bore_cuts())]
    out += [(*c, FUN_FLOOR, RIM) for c in _cells((-HX, HX, -HY, HY), _fun_cuts())]
    return out


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _cb(name, x0, x1, y0, y1, z0, z1):
    return m.col_box(name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _lid_boxes():
    """The cap closed, in (x, y, z) of the case frame: plate on top, front and back walls, two side walls; hollow below the plate."""
    x0, x1, z0, z1 = -HX, HX, RIM, H_TOT
    return [(x0, x1, -LID_HY, LID_HY, z1 - PLATE, z1), (x0, x0 + WALL, -LID_HY, LID_HY, z0, z1 - PLATE), (x1 - WALL, x1, -LID_HY, LID_HY, z0, z1 - PLATE),
            (x0 + WALL, x1 - WALL, -LID_HY, -LID_HY + WALL, z0, z1 - PLATE), (x0 + WALL, x1 - WALL, LID_HY - WALL, LID_HY, z0, z1 - PLATE)]


def _open(mesh: m.Mesh) -> m.Mesh:
    """Turn a closed-lid mesh about the hinge line to the open pose: +100 degrees about +y."""
    return mesh.moved(-HINGE).turned(OPEN_DEG, "y").moved(HINGE)


def _case() -> m.Part:
    body = _body_boxes()
    frame_parts = [(f"b{i}", _bx(*b), "case") for i, b in enumerate(body)]
    frame_cols = [_cb(f"c{i}", *b) for i, b in enumerate(body)]
    for i, w in enumerate(_wedges()):
        frame_parts.append((f"w{i}", w, "well"))
        frame_cols.append(m.col_hull(f"wc{i}", w))
    lid = _lid_boxes()
    lid_parts = [(f"l{i}", _open(_bx(*b)), "lid") for i, b in enumerate(lid)]
    lid_cols = [m.col_hull(f"lc{i}", _open(_bx(*b))) for i, b in enumerate(lid)]
    link = m.hinge("lid", "Y", tuple(HINGE), -OPEN_DEG, 0.0, lid_parts, lid_cols, mass=M_LID, gravity=True, damping=0.003, friction=0.0008, armature=1e-5, inertia=(2.7e-6, 1.0e-6, 2.7e-6))
    return m.mechanism("case", (CX, CY, TOP + 0.0002), [link], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, friction=(0.4, 0.4), solver_iters=96, free_mass=M_CASE,
                       frame_inertia=(1.6e-5, 4.6e-6, 1.5e-5))


def _bud(name: str, sign: float, pos) -> m.Part:
    a, b, c = HEAD_A, HEAD_B, 0.003
    oct_ = np.array([(a - c, -b), (a, -b + c), (a, b - c), (a - c, b), (-a + c, b), (-a, b - c), (-a, -b + c), (-a + c, -b)])
    low = m.prism(oct_, 0.0, 0.005)
    high = m.prism(oct_ * 0.72, 0.005, HEAD_H)
    head = m.join(low, high)
    stem = _bx(-STEM_X / 2, STEM_X / 2, -STEM_Y / 2, STEM_Y / 2, -STEM_L, 0.0)
    ry0, ry1 = sorted((sign * 0.0036, sign * RIB_Y1))
    rib = _bx(-0.0015, 0.0015, ry0, ry1, KEY_Z0, KEY_Z1)
    bump = _bx(-0.0048, -0.0026, -0.0015, 0.0015, KEY_Z0, KEY_Z1)
    parts = [("head", head, "bud"), ("stem", stem, "bud"), ("rib", rib, "tip"), ("bump", bump, "tip")]
    cols = [m.col_hull("head_c", head), _cb("stem_c", -STEM_X / 2, STEM_X / 2, -STEM_Y / 2, STEM_Y / 2, -STEM_L, 0.0), _cb("rib_c", -0.0015, 0.0015, ry0, ry1, KEY_Z0, KEY_Z1),
            _cb("bump_c", -0.0048, -0.0026, -0.0015, 0.0015, KEY_Z0, KEY_Z1)]
    s2 = math.sqrt(0.5)
    return m.free(name, pos, parts, cols, LOOKS, mass=BUD_M, friction=(0.5, 0.5), torsional_patch=None, solver_iters=192, start_quat=(0.0, s2, 0.0, s2))  # on its side: stem along -x, bump up


BUD_XY = {"bud_l": (0.26, 0.14), "bud_r": (0.26, 0.22)}  # lying on the bench, left of the case, stems toward the robot (-x)
OBJECTS = [_case()] + [_bud(n, s, (BUD_XY[n][0], BUD_XY[n][1], TOP + HEAD_A + 0.0002)) for n, s in BUDS.items()]

CASE_MAGNET_N = float(os.environ.get("AP_MAGN", "0.4"))   # each well's magnet: a pull toward its seat, only once the bud's seat plane is within MAG_R of it
MAG_R = float(os.environ.get("AP_MAGR", "0.007"))
LID_MAGNET_N = 1.0    # draws the lid shut over its last 5 degrees and holds it


LID_MAG_NM = LID_MAGNET_N * 2 * HX  # the same pull as a torque at the hinge: 1 N at the lid's front edge, 21.7 mm from the hinge


def _lid_magnet(env, on: bool) -> None:
    """The lid's magnet as a drive in its own hinge, so its pull is internal to the case (frame and lid pull on each other): over the last
    5 degrees a spring toward shut, capped at LID_MAG_NM; off, the hinge's own gains again."""
    asset, j = m._joint(env, "case", "lid")
    if on:
        m._TOUCHED.add(("case", "lid"))
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, LID_MAG_NM), joint_ids=[j])
        asset.write_joint_stiffness_to_sim(m._tensor(asset, 2.0), joint_ids=[j])
        asset.set_joint_position_target(m._tensor(asset, math.radians(-OPEN_DEG)), joint_ids=[j])
    else:
        m.unlock(env, "case", "lid")


LID_SHUT_DEG = -OPEN_DEG + 1.0
SEAT_MM = 0.5


def _wedge(env):
    return m.state(env, "case"), {n: m.state(env, n) for n in BUDS}


def bud_rel(case, bud, sign):
    """The bud in the case's frame: seat-plane centre offset from its well, proud (m), tilt (deg), yaw error (deg)."""
    p = case.to_local(bud.pos)[0]
    R = case.rotation().T @ bud.rotation()
    tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return p[0], p[1] - sign * WELL_Y, p[2] - RIM, tilt, yaw


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held = 0, 0
        self.dead, self.score, self.full = False, 0.0, False
        self.p0 = None
        self.last = {}
        self.reac_on = False
        self.lid_mag = False
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        case, buds = _wedge(env)
        if self.p0 is None:
            self.p0 = case.pos.copy()
        lid_deg = case.deg("lid")
        reac = []  # (point, force) on the case's frame: the other side of each well magnet's pull
        mag = lid_deg <= -OPEN_DEG + 5.0  # the lid's magnet: over its last 5 degrees
        if mag != self.lid_mag:
            _lid_magnet(env, mag)
            self.lid_mag = mag
        seated, pr, rel = {}, {}, {}
        Rc = case.rotation()
        for n, sgn in BUDS.items():
            dx, dy, proud, tilt, yaw = bud_rel(case, buds[n], sgn)
            rel[n] = (dx, dy, proud, tilt, yaw)
            # each well's magnet (both wells pull any bud): within MAG_R of a seat, a pull of CASE_MAGNET_N toward a point 2 mm under it
            pl = case.to_local(buds[n].pos)[0]
            f = np.zeros(3)
            for s2 in (+1.0, -1.0):
                seat = np.array([0.0, s2 * WELL_Y, RIM])
                if np.linalg.norm(pl - seat) < MAG_R:
                    d_ = seat - np.array([0.0, 0.0, 0.002]) - pl
                    f = Rc @ (d_ / np.linalg.norm(d_)) * CASE_MAGNET_N
                    reac.append((case.to_world(seat)[0], -f))
            m.push(env, n, f)
            seated[n] = bool(abs(dx) < 0.0010 and abs(dy) < 0.0010 and abs(proud) <= SEAT_MM * MM and tilt < 5.0 and abs(yaw) < 10.0)
            pr[n] = proud
        if reac:  # Newton's third law: the case takes each well magnet's pull back, at the seats (the pulls are near vertical, so one force at their centroid)
            F = sum(f for _, f in reac)
            w = np.array([np.linalg.norm(f) for _, f in reac])
            at = sum(wi * pt for wi, (pt, _) in zip(w, reac)) / max(w.sum(), 1e-9)
            m.push(env, "case", F, at=at)
        elif self.reac_on:
            m.push(env, "case", (0.0, 0.0, 0.0))
        self.reac_on = bool(reac)
        tip = case.tilt_deg()
        flipped = tip > 20.0 or case.pos[2] < TOP - 0.01
        off = any(b.pos[2] < TOP - 0.02 for b in buds.values())
        if flipped or off:
            self.dead = True
        nseat = sum(seated.values())
        shut = lid_deg <= LID_SHUT_DEG
        where = bool(np.linalg.norm(case.pos[:2] - self.p0[:2]) <= 0.03 and tip < 5.0 and abs(case.pos[2] - self.p0[2]) < 0.01)
        tier = 0.0 if (self.dead or not where) else 0.25 * nseat
        if not self.dead and nseat == 2 and shut and where:
            tier = 1.0
        tips = m.fingertips(env)
        tl = case.to_local(tips)
        q = np.abs(tl - np.array([0.0, 0.0, H_TOT / 2])) - np.array([HX, HY, H_TOT / 2])
        clear = bool(np.linalg.norm(np.maximum(q, 0.0), axis=1).min() >= 0.03)
        still = all(np.linalg.norm(x.lin_vel) < 0.005 and np.linalg.norm(x.ang_vel) < 0.2 for x in (case, *buds.values()))
        self.held = self.held + 1 if (clear and still) else 0
        # Judged on the state at the end: the tier of this step, once the hands have been clear and everything still for the last HOLD_STEPS. Nothing is carried over:
        # a seated bud pulled out, a case moved, or hands back at the case drops the score at once. Only a failure (dead: tipped past 20 degrees, a bud off the bench) latches.
        self.score = 0.0 if (self.dead or self.held < HOLD_STEPS) else tier
        self.full = bool(self.held >= HOLD_STEPS and tier == 1.0)
        self.n += 1
        self.last = dict(seated=seated, proud_mm={n: round(v * 1000, 2) for n, v in pr.items()}, lid_deg=round(lid_deg, 1), shut=shut, tilt=round(tip, 1), dead=self.dead, where=where, tier=tier, clear=clear,
                         still=still, held=self.held)
        if self.n % 20 == 0 and os.environ.get("AP_TRACE"):
            print(f"[ap] step {self.n}: {self.last} rel { {n: tuple(round(v * (1000 if i < 3 else 1), 1) for i, v in enumerate(r)) for n, r in rel.items()} }", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both buds seated in their own wells (stem fully in, head within 0.5 mm of the rim), the lid shut within 1 degree, the case upright within 3 cm of its start and never tipped past 20 degrees, hands 3 cm clear, still 1 s."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------

CASE_TCP_Z = float(os.environ.get("AP_HOLDZ", "0.015"))
GRIP_N = 25.0
HOLD_DY = float(os.environ.get("AP_HOLDDY", "0.0"))
TILT = float(os.environ.get("AP_TILT", "20"))  # the pick's tilt from straight down; after the stem-down turn the fingers point 20 degrees below level, so their lower edge clears the rim


def _case_pos(ctx):
    return ctx.pos("case")


def demo_script(ctx):
    """Modes: full. none: the arms wait. left_only: the left hand does everything, the case unheld. swap: the right bud goes in the left bud's well.
    lid_only: the lid is shut with no buds. one_bud: one bud seated, the other left on the bench, the lid shut. proud: the second bud let go 12 mm up, outside the magnet's reach.
    remove_after: both buds seated and the case let go, the hands clear for 2.5 s (the score reads 0.5), then the left hand pulls the right bud out and sets it on the bench: 0.25.
    case_moved: both buds seated and the case let go, the hands clear for 2.5 s (0.5), then the right hand pushes the case 6 cm along the bench: 0.
    Debug: hold (the holding hand only), one (one bud only), lid (the lid only, no buds)."""
    mode = ctx.mode
    if mode not in ("full", "none", "left_only", "swap", "lid_only", "one_bud", "proud", "hold", "one", "lid", "one_free", "one_l", "remove_after", "case_moved"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    c = _case_pos(ctx)
    # the right hand holds the case low at its two ends, fingers level pointing +x, jaws closing along y
    if mode not in ("left_only", "one_free"):
        hold = np.array([c[0], c[1] - HY + HOLD_DY, TOP + CASE_TCP_Z])
        yield from _hold_case(R, hold)
    if mode == "hold":
        yield from L.wait(3.0)
        print(f"[ap] hold {TRIAL.last}", flush=True)
        return
    yield from L.wait(0.2)
    order = {"lid_only": [], "lid": [], "one_bud": ["bud_r"], "one": ["bud_r"], "one_free": ["bud_r"], "one_l": ["bud_l"]}.get(mode, ["bud_r", "bud_l"])
    for name in order:
        yield from _seat_bud(ctx, L, name, wrong=(mode == "swap"), rel=(0.012 if (mode == "proud" and name == "bud_l") else None))
    if mode in ("one", "one_free", "one_l"):
        yield from L.wait(1.0)
        print(f"[ap] end {TRIAL.last}", flush=True)
        return
    if mode in ("remove_after", "case_moved"):
        yield from _let_go_case(R)
        yield from L.wait(2.5)
        print(f"[ap] both seated, hands clear 2.5 s: score {TRIAL.score} {TRIAL.last}", flush=True)
        if mode == "remove_after":
            yield from _hold_case(R, hold)  # the hand comes back to steady the case: a bud pulled from its magnet drags a free case along
            yield from _pull_bud(ctx, L, "bud_r")
            yield from _let_go_case(R)
        else:
            yield from _shove_case(ctx, R, hold)
        yield from L.wait(3.0)
        print(f"[ap] end: score {TRIAL.score} {TRIAL.last}", flush=True)
        return
    yield from _push_lid(ctx, L)
    if mode != "left_only":
        yield from _let_go_case(R)
    yield from L.wait(2.5)
    print(f"[ap] end {TRIAL.last}", flush=True)


def _hold_case(R, hold):
    yield from R.move(hold + [0.0, -0.06, 0.04], grip=d.tilted(60, 90), finger=d.FINGER_OPEN)
    yield from R.move(hold, t=1.2)
    yield from R.close(2 * HX, GRIP_N)


def _let_go_case(R):
    yield from R.close(0.034, 0.5, label="let go")  # 7 mm clear of each face, then out the way the fingers came in
    yield from R.move(R.at() + [0.0, -0.05, 0.03], t=1.0)
    yield from R.move(R.at() + [0.0, -0.02, 0.06], t=0.8)


GRIP_OFF = {}  # per bud: the hand's TCP from the bud's origin at the moment it let go


def _pull_bud(ctx, L, name):
    """Take a seated bud out again: back to the grip it was let go in, close on its head, lift it out against the well's magnet and set it down on the bench."""
    off = GRIP_OFF[name]
    up = np.array([0.0, 0.0, 0.05])
    yield from L.move(ctx.pos(name) + off + np.array([0.0, 0.0, 0.10]), t=2.0, label="high over seated bud")
    yield from L.close(0.028, 0.5, label="jaws as at the release")  # the width it let go at: open wider, the far finger would land on the other head
    yield from L.move(ctx.pos(name) + off + up, t=1.0, label="over seated bud")
    yield from L.servo(lambda t: ctx.pos(name) + off + up, timeout=1.0, tol=0.0006, label="over seated bud")
    yield from L.move(ctx.pos(name) + off, t=1.0, label="down to the head")
    yield from L.servo(lambda t: ctx.pos(name) + off, timeout=1.0, tol=0.0006, label="down to the head")
    yield from L.close(0.0218, GRIP_N, label="grip the seated bud")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.04]), t=1.5, label="pull out")
    print(f"[ap] {name} pulled: {TRIAL.last}", flush=True)
    yield from L.move(np.array([0.25, ctx.pos("case")[1] - 0.01, TOP + STEM_L + 0.02]) + off, t=2.0, label="over the bench")
    yield from L.open(label="set down")
    yield from L.wait(0.5)
    yield from L.move(L.at() + np.array([-0.05, 0.0, 0.08]), t=1.0, label="away")


def _shove_case(ctx, R, hold):
    """Push the case 6 cm along the bench (+y), from its -y end, with the closed fingers."""
    start = hold + np.array([0.0, -0.045, 0.0])
    yield from R.move(start + np.array([0.0, 0.0, 0.05]), finger=d.FINGER_CLOSED, t=1.5, label="back to the case")
    yield from R.move(start, t=0.8)
    yield from R.move(hold + np.array([0.0, 0.07, 0.0]), t=3.0, label="shove")
    yield from R.move(R.at() + np.array([0.0, -0.03, 0.06]), t=1.0, label="off")
    yield from R.move(R.at() + np.array([0.0, -0.06, 0.0]), t=1.0, label="away")


PICK_DZ = float(os.environ.get("AP_PICKDZ", "0.004"))  # TCP height over the lying head's middle


def _unit(v):
    return v / np.linalg.norm(v)


LEVEL_N = int(os.environ.get("AP_LEVELN", "4"))


def _level_bud(ctx, arm, name):
    """Level the held bud by feedback: read its stem axis and yaw, turn the hand about the bud's origin to bring them to the case's."""
    for _ in range(LEVEL_N):
        R = ctx.state(name).rotation()
        zb = R[:, 2]
        tilt = math.degrees(math.acos(float(np.clip(zb[2], -1, 1))))
        if tilt > 0.4:
            n = _unit(np.cross(zb, [0.0, 0.0, 1.0]))
            yield from arm.arc(ctx.pos(name), n, tilt, t=0.5, label="level")
            yield from arm.wait(0.25)
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        if abs(yaw) > 1.5:
            yield from arm.arc(ctx.pos(name), (0.0, 0.0, 1.0), -yaw, t=0.6, label="yaw")
    R = ctx.state(name).rotation()
    print(f"[ap] {name} leveled: tilt {math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1)))):.1f} deg yaw {math.degrees(math.atan2(R[1, 0], R[0, 0])):.1f}", flush=True)


REL = float(os.environ.get("AP_REL", "0.0055"))  # let go with the seat plane this far over the rim: inside the magnet's reach (MAG_R)
HEAD_GX = float(os.environ.get("AP_HEADGX", "0.007"))  # the pinch's centre over the head's underside: high on the 8 mm head, so the fingers clear the rim at the release
DESC = float(os.environ.get("AP_DESC", "0.008"))  # lowering speed, m/s
MID_Z = float(os.environ.get("AP_MIDZ", "0.012"))  # how far down the stem the steered point is


def _tip(ctx, name):
    """The stem's end, from the bud's pose."""
    return ctx.state(name).to_world(np.array([[0.0, 0.0, -STEM_L]]))[0]


def _seat(ctx, sgn, dz=0.0):
    """A point on the well's axis, dz over the rim, from the case's pose."""
    return ctx.state("case").to_world(np.array([[0.0, sgn * WELL_Y, RIM + dz]]))[0]


def _seat_bud(ctx, L, name, wrong=False, rel=None):
    """Pick a lying bud, turn it stem down, steer its stem's end down the well's axis by its own pose, and let go with the seat plane `rel` over the rim."""
    sgn = BUDS[name] * (-1.0 if wrong else 1.0)
    rel = REL if rel is None else rel
    head = lambda: ctx.pos(name) + np.array([HEAD_GX, 0.0, 0.0])
    yield from L.pick(head, PICK_DZ, width=0.0218, grip=d.tilted(TILT, 180, flip=True), above=0.09, lift=0.10, force=GRIP_N, label=f"pick {name}")
    yield from L.turn(-90.0, axis=(0.0, 1.0, 0.0), t=1.6, label="stem down")
    yield from L.servo(L.at(), timeout=0.6, label="settle")
    off = L.tcp() - _tip(ctx, name)
    over = _seat(ctx, sgn, 0.012)
    yield from L.move(_seat(ctx, sgn, 0.045) + off, t=2.0, label="high over well")  # carried high over the open case, then straight down
    yield from L.move(over + off, t=1.0, label="over well")
    yield from L.servo(over, timeout=1.5, read=lambda: _tip(ctx, name), tol=0.0006, label="over well")
    yield from _level_bud(ctx, L, name)
    yield from L.servo(lambda t: _seat(ctx, sgn, 0.002), timeout=2.0, read=lambda: _tip(ctx, name), tol=0.0003, label="at mouth")
    z_end = rel - STEM_L
    mid = lambda: ctx.state(name).to_world(np.array([[0.0, 0.0, -MID_Z]]))[0]  # the point steered down the axis: the stem's middle

    def goal(t):
        tip_z = ctx.state("case").to_local(np.array([_tip(ctx, name)]))[0][2] - RIM
        z = max(z_end, 0.002 - DESC * t, tip_z - 0.0025)
        return _seat(ctx, sgn, z + (STEM_L - MID_Z))

    yield from L.servo(goal, timeout=(0.002 - z_end) / DESC + 2.0, read=mid, tol=0.0006, label="lower")
    lag = (L.tcp() - d.ik.tcp_pose(L.q, L.base)[0]) * 1000
    print(f"[ap] {name} lowered: tip-goal {np.round((_tip(ctx, name) - goal(99.0)) * 1000, 1)} mm, hand-command {np.round(lag, 1)} mm, {TRIAL.last}", flush=True)
    yield from L.close(0.028, 0.5, label="release")  # open only 3 mm a side: the other bud's head stands 3 mm beyond the outer finger
    yield from L.wait(0.8)
    GRIP_OFF[name] = L.tcp() - ctx.pos(name)
    print(f"[ap] {name} released: {TRIAL.last}", flush=True)
    yield from L.move(L.at() + np.array([-0.05, 0.0, 0.0]), t=1.0, label="back")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.07]), t=0.8, label="up")


FIN_HX = float(os.environ.get("AP_FINHX", "0.004"))  # closed fingers, jaws along x: half their depth along x
FIN_DZ = float(os.environ.get("AP_FINDZ", "0.005"))  # their underside below the TCP (README 22)
LID_LEAD = float(os.environ.get("AP_LIDLEAD", "25"))  # degrees the fingertip's goal runs ahead of the lid
LID_X = float(os.environ.get("AP_LIDX", "-0.013"))  # where the fingertip pushes: this far from the hinge along the lid, 6.5 mm in from its far edge


def _lid_face(ctx, phi_deg, x_rel=None, out=0.0):
    """A point on the lid's outer face, `x_rel` from the hinge (closed coordinates), with the lid `phi_deg` open from shut, in the world."""
    x_rel = LID_X if x_rel is None else x_rel
    ph = math.radians(phi_deg)
    z_rel = CAP + out
    p = np.array([x_rel * math.cos(ph) + z_rel * math.sin(ph), 0.0, -x_rel * math.sin(ph) + z_rel * math.cos(ph)])
    return ctx.state("case").to_world(np.array([HINGE + p + np.array([0.0, 0.012, 0.0])]))[0]


def _push_lid(ctx, L):
    """Close the lid with a fingertip: from behind its far edge, push it over the top and down until its magnet takes it. Closed loop on the lid's hinge angle."""
    g = d.down(-90)
    edge = np.array([-FIN_HX, 0.0, -FIN_DZ])  # the fingers' lower front edge, from the TCP
    phi = lambda: ctx.deg("case", "lid") + OPEN_DEG
    start = ctx.state("case").to_world(np.array([HINGE + np.array([0.026, 0.012, 0.009])]))[0] - edge
    yield from L.move(start + np.array([0.0, 0.0, 0.05]), grip=g, finger=d.FINGER_CLOSED, t=2.0, label="behind lid")
    yield from L.move(start, t=1.0, label="down behind lid")
    print(f"[ap] behind lid: phi {phi():.1f} edge {np.round((ctx.state('case').to_local(np.array([L.tcp() + edge]))[0] - HINGE) * 1000, 1)}", flush=True)
    k = [0]

    def target():
        tg = _lid_face(ctx, max(-3.0, phi() - LID_LEAD)) - edge
        k[0] += 1
        if k[0] % 10 == 1:
            hl = ctx.state("case").to_local(np.array([L.tcp() + edge, tg + edge]))
            print(f"[ap] push {k[0]}: phi {phi():.1f} edge {np.round((hl[0] - HINGE) * 1000, 1)} goal {np.round((hl[1] - HINGE) * 1000, 1)} (mm from hinge, case frame)", flush=True)
        return tg

    yield from L.follow(target, t=8.0, speed=0.05, until=lambda: phi() < 0.8, label="push lid")
    print(f"[ap] lid pushed: phi {phi():.1f} {TRIAL.last}", flush=True)
    yield from L.wait(0.4)
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.06]), t=0.8, label="off the lid")
    yield from L.move(L.at() + np.array([-0.06, 0.0, 0.0]), t=0.8, label="away")


def size_check() -> None:
    """Print the scene's key sizes against the published ones (Apple's), in mm and g. Run: python task.py --sizes (no simulator is started)."""
    rows = [("case depth (x)", 2 * HX * 1e3, 21.7), ("case width (y)", 2 * HY * 1e3, 60.6), ("case height, lid shut", H_TOT * 1e3, 45.2), ("case mass, g", (M_CASE + M_LID) * 1e3, 50.8),
            ("bud length (head + stem)", (HEAD_H + STEM_L) * 1e3, 30.9), ("bud head width (y)", 2 * HEAD_B * 1e3, 21.8), ("bud head depth (x), ear tip not modelled", 2 * HEAD_A * 1e3, 24.0),
            ("bud stem width (x)", STEM_X * 1e3, 5.7), ("bud stem length", STEM_L * 1e3, 23.3), ("bud mass, g", BUD_M * 1e3, 5.3), ("lid open, degrees", OPEN_DEG, 115.0)]
    print(f"{'part':44s} {'scene':>8s} {'published':>10s}")
    for name, got, want in rows:
        print(f"{name:44s} {got:8.1f} {want:10.1f}")


if __name__ == "__main__" and "--sizes" in sys.argv:
    size_check()
