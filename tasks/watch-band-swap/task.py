"""Change an Apple Watch band: hold the band-release button while the old band half slides out of its slot, then slide the new half in until it clicks.

An Apple Watch Series 11 46 mm case (Apple: 46 x 39 x 9.7 mm) lies face down on a pad. Across its crown end runs a band slot along y; the old (black) pin half's bar sits in it, its strap lying
on the pad toward +x. On the back (up) stands the band release button; a catch (the pawl) holds the bar at the slot's centre until the button is pressed. The new (white) half lies on the pad.

Rigid bodies. The case is one free mechanism: frame (body, slot, end stop, the old hole half fixed on its far end) plus two links, the button (sprung slide) and the catch (a slide that drops into the bar's notch).
The two band halves are free bodies: a bar (lug), a neck through the slot's slit, and a rigid strap. Code: the catch is driven by the button's joint and the bar's pose (it drops into the notch when the bar
is within 0.5 mm of centred and the button is up, and lifts out while the button is pressed).

Debug:  python demokit.py task.py OUT --no-record --mode full,none,no_button,half_in
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
INSTRUCTION = ("Change the band's top half for the new one: hold the band release button for the top slot while you slide the old half out, then slide the new half in, "
               "printed side up, until it clicks.")
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HOLD_STEPS = 20  # 1 s at 20 Hz
MM = 1e-3

LOOKS = {"pad": m.Look(m.rgb(0.55, 0.58, 0.62), metallic=0.0, roughness=0.95), "case": m.Look(m.rgb(0.80, 0.80, 0.82), metallic=0.6, roughness=0.35),
         "glass": m.Look(m.rgb(0.05, 0.05, 0.06), metallic=0.0, roughness=0.1), "button": m.Look(m.rgb(0.95, 0.35, 0.10), metallic=0.2, roughness=0.4),
         "pawl": m.Look(m.rgb(0.9, 0.7, 0.1), metallic=0.3, roughness=0.4), "black": m.Look(m.rgb(0.07, 0.07, 0.08), metallic=0.0, roughness=0.55),
         "white": m.Look(m.rgb(0.93, 0.93, 0.92), metallic=0.0, roughness=0.55), "stripe": m.Look(m.rgb(0.1, 0.4, 0.9), metallic=0.0, roughness=0.5)}

# ---- sizes, mm in the case frame unless noted. Origin: the case's footprint centre on the pad. Crown end +x, band slot along y.
CLR = float(os.environ.get("WB_CLR", "1.0"))  # slot clearance per side (real: about 0.2, an estimate; widened for the simulator)
CX, CY = float(os.environ.get("WB_CX", "0.338")), float(os.environ.get("WB_CY", "0.0"))
PAD_C, PAD_SIZE, PAD_T = (0.34, -0.08), (0.25, 0.24), 0.003
PADTOP = TOP + PAD_T
HL, HW, HT = 23.0, 19.5, 9.7  # case half length (x), half width (y), thickness: 46 x 39 x 9.7 (Apple)
XS = 17.0  # slot centre x
LUG_X, LUG_H, LUG_L = 3.6, 3.0, 24.0  # bar: 3.6 deep, 3.0 tall, 24 long
ZC = 3.3  # bar centre height when the strap lies on the pad
TX0, TX1 = XS - LUG_X / 2 - CLR, XS + LUG_X / 2 + CLR
TZ0, TZ1 = ZC - LUG_H / 2 - CLR, ZC + LUG_H / 2 + CLR
YSTOP = LUG_L / 2 + 0.5  # the slot's far end: a bar centred +0.5 mm touches it
SLIT_Z0, SLIT_Z1 = 0.8, 4.0
BTN_Y, BTN_X, BTN_W = 0.0, 3.5, 8.0
PAWL_Y, PAWL_X, PAWL_W = -8.0, 3.0, 2.0  # the notch's centre is 8 mm from the bar's middle, on the -y side
NOTCH_W, NOTCH_D = 3.0, 1.2
PAWL_UP = 2.4e-3  # retract travel (up out of the slot)
PRESS_ON = 0.3e-3  # the button counts as pressed when it is in by this much
SEAT = 0.5  # mm: a bar within this of centred drops the catch
NECK_Z0, NECK_Z1 = 1.8, 3.0  # neck height in the case frame
STRAP_X0 = 24.5  # strap starts this far from the case's origin along x (1.5 mm clear of the case end)
STRAP_L, STRAP_W, STRAP_T = 90.0 - 3.6 - 5.0, 24.0, 3.0
HOLE_L = 115.0

M_CASE, M_BTN, M_PAWL, M_PIN, M_PIN_NEW = 0.052, 0.002, 0.002, 0.012, 0.012


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box(((x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM), ((x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM))


def _cb(name, x0, x1, y0, y1, z0, z1):
    return m.col_box(name, ((x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM), ((x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM))


def _cells(outline, cuts):
    ox0, ox1, oy0, oy1 = outline
    xs = sorted({ox0, ox1, *[c[0] for c in cuts if ox0 < c[0] < ox1], *[c[1] for c in cuts if ox0 < c[1] < ox1]})
    ys = sorted({oy0, oy1, *[c[2] for c in cuts if oy0 < c[2] < oy1], *[c[3] for c in cuts if oy0 < c[3] < oy1]})
    out = []
    for xa, xb in zip(xs[:-1], xs[1:]):
        run = None
        for ya, yb in zip(ys[:-1], ys[1:]):
            cx, cy = (xa + xb) / 2, (ya + yb) / 2
            if not any(c[0] < cx < c[1] and c[2] < cy < c[3] for c in cuts):
                run = (run[0] if run else ya, yb)
            else:
                if run:
                    out.append((xa, xb, run[0], run[1]))
                run = None
        if run:
            out.append((xa, xb, run[0], run[1]))
    return out


def _case_boxes():
    """(x0, x1, y0, y1, z0, z1) boxes of the case body: tunnel along y from the -y mouth to the end stop, slit on its +x side, a hole in the ceiling for the button and another for the catch."""
    B = [(-HL, TX0, -HW, HW, 0.0, HT)]  # body behind the slot
    B.append((TX0, TX1, -HW, HW, 0.0, TZ0))  # tunnel floor
    B.append((TX1, HL, -HW, HW, 0.0, SLIT_Z0))  # floor under the slit
    holes = [(XS - BTN_X / 2 - 0.2, XS + BTN_X / 2 + 0.2, BTN_Y - BTN_W / 2 - 0.2, BTN_Y + BTN_W / 2 + 0.2),
             (XS - PAWL_X / 2 - 0.2, XS + PAWL_X / 2 + 0.2, PAWL_Y - PAWL_W / 2 - 0.2, PAWL_Y + PAWL_W / 2 + 0.2)]
    for c in _cells((TX0, TX1, -HW, HW), holes):
        B.append((c[0], c[1], c[2], c[3], TZ1, HT))  # ceiling with its two holes
    B.append((TX1, HL, -HW, HW, SLIT_Z1, HT))  # above the slit
    B.append((TX0, HL, YSTOP, HW, TZ0, TZ1))  # end stop
    return B


def _btn_link():
    x0, x1 = XS - BTN_X / 2, XS + BTN_X / 2
    y0, y1 = BTN_Y - BTN_W / 2, BTN_Y + BTN_W / 2
    z0, z1 = TZ1 + 0.2, HT + 0.5
    return m.slider("button", "Z", -0.5 * MM, 0.0, [("cap", _bx(x0, x1, y0, y1, z0, z1), "button")], [_cb("btn_c", x0, x1, y0, y1, z0, z1)], at=(XS * MM, BTN_Y * MM, HT * MM),
                    mass=M_BTN, spring=7500.0, damping=3.0, friction=0.0, effort=60.0)


def _pawl_link():
    x0, x1 = XS - PAWL_X / 2, XS + PAWL_X / 2
    y0, y1 = PAWL_Y - PAWL_W / 2, PAWL_Y + PAWL_W / 2
    z0 = ZC + LUG_H / 2 - NOTCH_D + 0.1  # engaged: its foot 0.1 mm above the notch floor
    z1 = HT + 2.0
    return m.slider("pawl", "Z", 0.0, PAWL_UP, [("pawl", _bx(x0, x1, y0, y1, z0, z1), "pawl")], [_cb("pawl_c", x0, x1, y0, y1, z0, z1)], at=(XS * MM, PAWL_Y * MM, HT * MM),
                    mass=M_PAWL, spring=3000.0, damping=3.0, friction=0.0, effort=20.0)


def _case() -> m.Part:
    boxes = _case_boxes()
    parts = [(f"b{i}", _bx(*b), "case") for i, b in enumerate(boxes)]
    cols = [_cb(f"c{i}", *b) for i, b in enumerate(boxes)]
    hole = (-HL - HOLE_L, -HL, -STRAP_W / 2, STRAP_W / 2, 0.0, STRAP_T)  # the old hole half, fixed to the case (not scored apart)
    parts.append(("hole_half", _bx(*hole), "black"))
    cols.append(_cb("hole_c", *hole))
    parts.append(("glass", _bx(-HL + 1, HL - 1, -HW + 1, HW - 1, 0.0, 0.3), "glass"))
    return m.mechanism("case", (CX, CY, PADTOP + 0.0003), [_btn_link(), _pawl_link()], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.5, 0.5), solver_iters=64, free_mass=M_CASE,
                       frame_inertia=(1.0e-5, 1.2e-4, 1.3e-4))


def _half(name, pos, look, mass):
    """A pin half. Origin: the bar's centre. Bar along y, strap toward +x. Bar = two upper blocks round a notch on a lower slab; neck; strap (rigid)."""
    h = LUG_H / 2
    nz = h - NOTCH_D  # notch floor, relative to the bar's centre
    ny0, ny1 = PAWL_Y - NOTCH_W / 2, PAWL_Y + NOTCH_W / 2
    lx = LUG_X / 2
    boxes = [(-lx, lx, -LUG_L / 2, LUG_L / 2, -h, nz), (-lx, lx, -LUG_L / 2, ny0, nz, h), (-lx, lx, ny1, LUG_L / 2, nz, h)]
    neck = (lx, STRAP_X0 - XS, -STRAP_W / 2, STRAP_W / 2, NECK_Z0 - ZC, NECK_Z1 - ZC)
    strap = (STRAP_X0 - XS, STRAP_X0 - XS + STRAP_L, -STRAP_W / 2, STRAP_W / 2, -ZC, -ZC + STRAP_T)
    stripe = (STRAP_X0 - XS + 20.0, STRAP_X0 - XS + 60.0, -1.0, 1.0, -ZC + STRAP_T, -ZC + STRAP_T + 0.2)
    parts = [(f"p{i}", _bx(*b), look) for i, b in enumerate(boxes)] + [("neck", _bx(*neck), look), ("strap", _bx(*strap), look), ("print", _bx(*stripe), "stripe")]
    cols = [_cb(f"pc{i}", *b) for i, b in enumerate(boxes)] + [_cb("neck_c", *neck), _cb("strap_c", *strap)]
    return m.free(name, pos, parts, cols, LOOKS, mass=mass, friction=(0.6, 0.6), torsional_patch=None, solver_iters=96)


OLD_POS = (CX + XS * MM, CY, PADTOP + ZC * MM + 0.0003)
NEW_POS = (float(os.environ.get("WB_NX", "0.36")), float(os.environ.get("WB_NY", "-0.15")), PADTOP + ZC * MM + 0.0003)
PAD = m.fixed("pad", (PAD_C[0], PAD_C[1], TOP), [("pad", m.box((PAD_SIZE[0], PAD_SIZE[1], PAD_T), (0, 0, PAD_T / 2)), "pad")], [m.col_box("pad_c", (PAD_SIZE[0], PAD_SIZE[1], PAD_T), (0, 0, PAD_T / 2))],
              LOOKS, friction=(0.6, 0.6))
OBJECTS = [PAD, _case(), _half("old_half", OLD_POS, "black", M_PIN), _half("new_half", NEW_POS, "white", M_PIN_NEW)]


# ---------------------------------------------------------------------------------------------------------------------------
# state and scorer

def bar_rel(case, bar):
    """The bar's centre in the case frame: (x - slot x, y, z - ZC) in mm, its turn about the case's z (deg) and its tilt from level (deg)."""
    p = case.to_local(bar.pos)[0] * 1000.0
    R = case.rotation().T @ bar.rotation()
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))
    return p[0] - XS, p[1], p[2] - ZC, yaw, tilt


def aligned(rel) -> bool:
    """The bar's notch is under the catch: centred within SEAT along the slot, in the slot's cross-section, square."""
    dx, dy, dz, yaw, tilt = rel
    return bool(abs(dy) <= SEAT and abs(dx) < CLR + 0.5 and abs(dz) < CLR + 0.5 and abs(yaw) < 3.0 and tilt < 4.0)


def in_slot(rel) -> bool:
    dx, dy, dz, yaw, tilt = rel
    return bool(dy > -(HW + LUG_L / 2) and abs(dx) < 4.5 and abs(dz) < 3.0)


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held = 0, 0
        self.dead, self.score, self.full = False, 0.0, False
        self.p0 = None
        self.pressed_max = 0.0
        self.last = {}
        self.engaged = True
        if env is not None:
            self.step(env)

    def drive(self, env, case, rels):
        """The catch: lifted while the button is in; dropped into the notch when a bar is centred and the button is up."""
        bj = case.joint("button")
        pressed = bj <= -PRESS_ON
        want_in = (not pressed) and any(aligned(r) for r in rels)
        if want_in != self.engaged or self.n < 2:
            self.engaged = want_in
        m.set_target(env, "case", "pawl", 0.0 if self.engaged else PAWL_UP)
        return pressed

    def step(self, env) -> None:
        case, old, new = m.state(env, "case"), m.state(env, "old_half"), m.state(env, "new_half")
        if self.p0 is None:
            self.p0 = case.pos.copy()
        ro, rn = bar_rel(case, old), bar_rel(case, new)
        pressed = self.drive(env, case, [ro, rn])
        pj = case.joint("pawl")
        self.pressed_max = max(self.pressed_max, -case.joint("button"))
        tip = case.tilt_deg()
        old_out = not in_slot(ro)
        latched = bool(aligned(rn) and pj <= 0.4e-3 and not pressed and rn[3] is not None)
        up = new.rotation()[2, 2] > 0.95
        seated = latched and up
        where = bool(np.linalg.norm(case.pos[:2] - self.p0[:2]) <= 0.03 and tip < 5.0 and abs(case.pos[2] - self.p0[2]) < 0.01)
        flipped = tip > 20.0 or case.pos[2] < TOP - 0.01
        gone = any(b.pos[2] < TOP - 0.02 for b in (old, new))  # a half off the table edge
        if flipped:
            self.dead = True
        tier = 0.0
        if not self.dead:
            if old_out and seated and where:
                tier = 1.0
            elif old_out and where:
                tier = 0.25
        tips = m.fingertips(env)
        clear = self.clear(tips, case, new, old)
        still = all(np.linalg.norm(x.lin_vel) < 0.005 and np.linalg.norm(x.ang_vel) < 0.2 for x in (case, old, new))
        self.held = self.held + 1 if (clear and still) else 0
        if self.held >= HOLD_STEPS:
            self.score = tier
        if self.dead:
            self.score = 0.0
        self.full = bool(self.held >= HOLD_STEPS and tier == 1.0)
        self.n += 1
        self.last = dict(old=tuple(round(v, 2) for v in ro), new=tuple(round(v, 2) for v in rn), btn_mm=round(case.joint("button") * 1000, 2), pawl_mm=round(pj * 1000, 2), pressed=pressed,
                         old_out=old_out, latched=latched, seated=seated, where=where, tilt=round(tip, 1), tier=tier, clear=clear, still=still, held=self.held)
        if self.n % 20 == 0 and os.environ.get("WB_TRACE"):
            l = self.last
            print(f"[wb] step {self.n}: old {l['old'][:3]} btn {l['btn_mm']} pawl {l['pawl_mm']} new {l['new'][:3]}", flush=True)

    @staticmethod
    def clear(tips, case, new, old) -> bool:
        """Every fingertip of both hands 3 cm from the case with its hole half, and from the new half and the old half (boxes round them)."""
        lo = []
        for who, box in ((case, (-HL - HOLE_L, HL, -HW, HW, 0.0, HT)), (new, (-LUG_L / 2, STRAP_X0 - XS + STRAP_L, -STRAP_W / 2, STRAP_W / 2, -ZC, ZC)),
                         (old, (-LUG_L / 2, STRAP_X0 - XS + STRAP_L, -STRAP_W / 2, STRAP_W / 2, -ZC, ZC))):
            q = who.to_local(tips) * 1000.0
            b = np.array(box)
            ex = np.maximum(np.maximum(b[[0, 2, 4]] - q, q - b[[1, 3, 5]]), 0.0)
            lo.append(float(np.linalg.norm(ex, axis=1).min()))
        return bool(min(lo) >= 30.0)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Old half out of the slot, new half in it centred within 0.5 mm with the catch dropped into its notch and the button up, printed side up; the watch within 3 cm, level; hands 3 cm clear, still 1 s."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# ---------------------------------------------------------------------------------------------------------------------------
# the demo

GRIP_N = float(os.environ.get("WB_GRIP_N", "30"))
PRESS_N = float(os.environ.get("WB_PRESS_N", "4"))
STRAP_GRIP_DX = float(os.environ.get("WB_GDX", "0.07"))  #  # grip the strap this far past the neck's end... from the bar's centre along x (m)
FINGER_LOW = float(os.environ.get("WB_FLOW", "0.003"))  # TCP height over the pad when gripping the strap's edges
LEFT_TILT = float(os.environ.get("WB_LTILT", "30"))
LEFT_OPEN = float(os.environ.get("WB_LOPEN", "0.015"))  # half the jaw opening on the button press
BTN_DY = float(os.environ.get("WB_BTNDY", "0.0"))
BTN_DX = float(os.environ.get("WB_BTNDX", "0.002"))


def _env(ctx):
    return ctx.emb._env


def _tipc(ctx, side):
    """The three tip points of the finger of `side` ('lo' for the lower-y finger) read from the arm's own geometry."""
    t = m.fingertips(_env(ctx))
    base = 0 if ctx.left is not None and side[0] == "L" else 12
    pts = t[base:base + 12].reshape(2, 3, 2, 3)  # finger, dy, dz, xyz
    tipp = pts[:, :, 0, :].mean(axis=1)  # each finger's tip centre
    return tipp


def demo_script(ctx):
    """Modes: full. none: the arms wait. no_button: the right hand draws the old half without the button held. half_in: full, but the new half is pushed in only 6 mm short of seated."""
    mode = ctx.mode
    if mode not in ("full", "none", "no_button", "half_in", "press_only", "pick_new"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    case = ctx.pos("case")
    btn = np.array([CX + XS * MM + BTN_DX, CY + BTN_DY, PADTOP + (HT + 0.5) * MM])
    gl = d.tilted(LEFT_TILT, 0)
    # left: one fingertip's end edge on the button, the other finger hangs off the case's +y side
    if mode != "no_button":
        yield from L.move(btn + [0.0, 0.0, 0.06], grip=gl, finger=-LEFT_OPEN, t=2.5, label="over button")
        tips = _tipc(ctx, "L")
        lo = int(np.argmin(tips[:, 1]))
        off = tips[lo] - L.tcp()
        yield from L.servo(btn + [0.0, 0.0, 0.02] - off, timeout=2.0, label="over button 2")
        res = yield from L.press((0.0, 0.0, -1.0), PRESS_N, 0.03, label="press button")
        tips = _tipc(ctx, "L")
        print(f"[wb] pressed: tip(lo) rel button mm {np.round((tips[lo] - btn) * 1000, 1)} tcp rel {np.round((L.tcp() - btn) * 1000, 1)} {TRIAL.last}", flush=True)
        if mode == "press_only":
            yield from L.wait(1.5)
            print(f"[wb] press hold: {TRIAL.last}", flush=True)
            return
    if mode == "pick_new":
        nw = ctx.pos("new_half")
        gp = nw + np.array([STRAP_GRIP_DX, 0.0, 0.0])
        gz = PADTOP + FINGER_LOW
        yield from R.move([gp[0], gp[1], gz + 0.06], grip=d.down(0), finger=d.FINGER_OPEN, t=2.5, label="over new strap")
        yield from R.move([gp[0], gp[1], gz], t=1.2, label="down new")
        print(f"[wb] right tcp z over pad mm {(R.tcp()[2] - PADTOP) * 1000:.1f} tips {np.round(_tipc(ctx, 'R')[:, 2] * 1000 - PADTOP * 1000, 1)}", flush=True)
        yield from R.close(STRAP_W * MM, GRIP_N, label="grip strap")
        yield from R.move(R.at() + [0.0, 0.0, 0.03], t=1.0, label="lift")
        yield from R.wait(1.0)
        print(f"[wb] new half after lift: {ctx.pos('new_half') - nw}", flush=True)
        return
    # right: grip the old strap at its edges and draw it out along -y
    old = ctx.pos("old_half")
    gp = old + np.array([STRAP_GRIP_DX, 0.0, 0.0])
    gz = PADTOP + FINGER_LOW
    yield from R.move([gp[0], gp[1], gz + 0.06], grip=d.down(0), finger=d.FINGER_OPEN, t=2.5, label="over old strap")
    yield from R.move([gp[0], gp[1], gz], t=1.2, label="down")
    yield from R.close(STRAP_W * MM, GRIP_N, label="grip strap")
    y0 = ctx.pos("old_half")[1]
    yield from R.move(R.at() + [0.0, -float(os.environ.get("WB_DRAW", "0.06")), 0.0], t=4.0, label="draw out")
    print(f"[wb] drawn: {TRIAL.last}", flush=True)
    if mode == "no_button":
        yield from R.wait(1.0)
        return
    yield from R.move(R.at() + [0.0, 0.0, 0.05], t=1.0, label="lift old")
    yield from R.move([0.40, -0.26, gz + 0.05], t=2.0, label="away")
    yield from R.open(label="drop old")
    yield from L.move(L.at() + [0.0, 0.0, 0.06], t=1.0, label="left up")
    # the left hand pins the watch at the back, 5 cm from the slot, with the button free
    pin = np.array([CX - 0.013, CY, PADTOP + HT * MM])
    yield from L.move(pin + [0.0, 0.0, 0.05], t=1.5, label="over pin")
    tips = _tipc(ctx, "L")
    lo = int(np.argmin(tips[:, 1]))
    off = tips[lo] - L.tcp()
    yield from L.servo(pin + [0.0, 0.0, 0.03] - off, timeout=2.0, label="over pin 2")
    yield from L.press((0.0, 0.0, -1.0), 4.0, 0.03, label="pin watch")
    # right: pick the new half by its strap, bring the bar to the slot's mouth, level and square, and push it in
    nw = ctx.pos("new_half")
    gp = nw + np.array([STRAP_GRIP_DX, 0.0, 0.0])
    yield from R.move([gp[0], gp[1], gz + 0.06], grip=d.down(0), finger=d.FINGER_OPEN, t=2.5, label="over new strap")
    yield from R.move([gp[0], gp[1], gz], t=1.2, label="down new")
    yield from R.close(STRAP_W * MM, GRIP_N, label="grip new strap")
    yield from R.move(R.at() + [0.0, 0.0, 0.012], t=0.8, label="lift new")
    slot = lambda: ctx.pos("case") + np.array([XS * MM, 0.0, 0.0])
    zc = lambda: ctx.pos("case")[2] + ZC * MM + 0.0003
    mouth_y = lambda: ctx.pos("case")[1] - (HW + 12.0 + 8.0) * MM  # bar's +y end 8 mm outside the mouth
    goal0 = lambda t: np.array([slot()[0], mouth_y(), zc() + 0.003])
    yield from R.servo(goal0, timeout=6.0, read=lambda: ctx.pos("new_half"), tol=0.0008, label="to mouth")
    yield from R.servo(lambda t: np.array([slot()[0], mouth_y(), zc()]), timeout=3.0, read=lambda: ctx.pos("new_half"), tol=0.0006, label="level at mouth")
    rn = bar_rel(ctx.state("case"), ctx.state("new_half"))
    print(f"[wb] at mouth: bar rel {rn}, {TRIAL.last}", flush=True)
    end = 0.0003 if mode != "half_in" else -0.006
    s_end = lambda: ctx.pos("case")[1] + end
    v = 0.006
    ytarget = lambda t: np.array([slot()[0], min(mouth_y() + v * t, s_end()), zc()])
    yield from R.servo(ytarget, timeout=12.0, read=lambda: ctx.pos("new_half"), tol=0.0006, label="insert")
    print(f"[wb] inserted: {TRIAL.last}", flush=True)
    yield from R.wait(0.5)
    yield from R.open(label="release new")
    yield from R.move(R.at() + [0.0, 0.0, 0.08], t=1.0, label="right up")
    yield from L.move(L.at() + [0.0, 0.0, 0.08], t=1.0, label="left up")
    yield from L.wait(2.5)
    print(f"[wb] end {TRIAL.last}", flush=True)
