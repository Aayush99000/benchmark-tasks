"""The scripted reference demo (closed loop on the tools' poses, which the cameras see, and the arms' own joints).

Modes (TASK_DEMO): full; skip (rinses two teeth only); nosuction (never places the ejector); outside (squeezes with the
nozzle outside the lips); none (does nothing).
"""

from __future__ import annotations

import math

import numpy as np

import demokit as d
import mouth as M
import parts as P
from yam_arena.constants import FINGER_OPEN

GRIP_N = float(__import__('os').environ.get('DENTAL_GRIP_N', 30.0))  # N a finger on the bulb to hold it
EJ_READY = 0.35  # the ejector's fingers come down closed to this fraction of fully open
EJ_GRASP_DZ = -0.003  # m, the ejector grasp height from the handle's centre
CHIN_SIDE = np.array([0.32, 0.03, 0.81])  # in front of the opening, past the chin, clear of the face
STANDOFF = 0.018  # m from the nozzle's end to the dot, along the nozzle (the nozzle is held a little higher, for the stream's fall)
APPROACH = 0.025  # m back along the nozzle, where the nozzle waits before going in
SETTLE_DEG = 17.0  # how far the bulb turns toward upright in the fingers once lifted (measured in the simulator)
SQUEEZE_STEP = 0.0016  # m each finger's target closes for each tooth: the gap across the bulb closes about 0.9 mm, about 0.9 mL


def _rotation_onto(a, b) -> np.ndarray:
    return P._align(np.asarray(a, float), np.asarray(b, float))


def script(ctx):
    B, E = ctx.right, ctx.left  # the right hand works the bulb syringe; the left keeps the saliva ejector in place
    mode = ctx.mode
    if mode == "none":
        yield from B.wait(5.0)
        return

    def bulb_tip():
        st = ctx.state("bulb")
        return st.pos + st.rotation() @ np.array([0, 0, P.TIP_Z])

    def ej_tip():
        st = ctx.state("ejector")
        return st.pos + st.rotation() @ P.EJ_TIP_LOCAL

    def gap(arm) -> float:
        return 2 * float(ctx.observed(arm.side)[6]) * abs(FINGER_OPEN)

    def bulb_axis():
        return ctx.state("bulb").rotation() @ np.array([0, 0, -1.0])

    def steer(arm, point_fn, goal, t, tol=0.0015, label="steer"):
        """Move the arm so that a point on the tool it holds reaches `goal` (a point, or a function giving one from what is
        seen now): a straight move, then a servo on what is seen."""
        goal_fn = goal if callable(goal) else (lambda g=np.asarray(goal, float): g)
        yield from arm.move(arm.at() + goal_fn() - point_fn(), t=t, label=label)
        good = 0
        for _ in range(30):
            err = goal_fn() - point_fn()
            good = good + 1 if np.linalg.norm(err) < tol else 0
            if good >= 3:
                break
            c = arm.at() + np.clip(0.4 * err, -0.003, 0.003)
            arm._solve(c + arm.trim, arm.grip.rot)
            yield

    # 1. pick up both tools
    # The bulb is taken from its ring with the fingers leaning, so that with the fingers straight down over the mouth the
    # bulb leans as BULB_AXIS says: the arm reaches the mouth only with its fingers pointing down.
    rinse_grip = d.down(90)  # the fingers close along x, across the bulb, clear of the ejector
    # The soft bulb settles about SETTLE_DEG in the fingers as it is lifted, toward upright: pick it leaning that much more.
    perp = np.cross(M.BULB_AXIS, [0.0, 0.0, 1.0])
    lean = d.axis_angle_matrix(perp / np.linalg.norm(perp), math.radians(SETTLE_DEG)) @ M.BULB_AXIS
    pick_grip = d._as_grip(_rotation_onto([0, 0, -1], lean).T @ rinse_grip.rot)
    def take(arm, name, lift, ready=None, **kw):
        """Grip, wait for the fingers to finish closing (they close at a limited speed), then lift. With `ready`, the
        fingers come down already closed to that opening (a fraction of fully open), so that the last close is short."""
        if ready is None:
            yield from arm.pick(name, 0.0, lift=0.0, **kw)
        else:
            above = np.array([0.0, 0.0, kw["above"]])
            yield from arm.move(ctx.pos(name) + above, grip=kw["grip"], finger=FINGER_OPEN, label=f"{name}: approach")
            yield from arm.servo(ctx.pos(name) + above, timeout=1.0, label=f"{name}: approach")
            dz = np.array([0.0, 0.0, EJ_GRASP_DZ])
            yield from arm.move(ctx.pos(name) + dz, finger=ready * FINGER_OPEN, label=f"{name}: in")
            yield from arm.servo(ctx.pos(name) + dz, timeout=1.0, label=f"{name}: in")
            yield from arm.close(kw["width"], kw["force"], label=f"{name}: close")
        yield from arm.wait(0.8)
        yield from arm.move(arm.at() + [0.0, 0.0, lift], t=0.8, label=f"{name}: lift")

    picks = [take(B, "bulb", 0.08, width=M.BULB_D, grip=pick_grip, above=0.02, back=0.06, force=GRIP_N, label="bulb")]
    if mode != "nosuction":
        picks.append(take(E, "ejector", 0.06, ready=EJ_READY, width=0.014, grip=d.down(-90), above=0.06, force=40.0, label="ejector"))
    yield from ctx.together(*picks)
    # turn the bulb to its rinsing lean while still over the holder's drip cup, and let the grip on it settle there
    yield from B.move(B.at() + [0.0, 0.0, 0.03], grip=rinse_grip, t=1.5, label="bulb turned over its holder")
    yield from B.wait(1.5)

    # 2. the ejector's tip onto the floor of the mouth against the throat sill, left of the tongue, where the water collects;
    # the bulb leaning over the mouth
    sill = M.world(*M.EJ_TARGET)  # the tip's centre 6 mm in front of the sill, 5 mm above the floor
    over0 = M.aim(M.SPOTS[0], M.BULB_AXIS, STANDOFF) - M.BULB_AXIS * APPROACH
    wait_pose = over0 + M.BULB_AXIS * P.TIP_Z + [0.0, -0.08, 0.0]  # beside the face, out of the ejector's way
    moves = [B.move(wait_pose, grip=rinse_grip, t=3.0, label="bulb beside the face")]
    if mode != "nosuction":
        moves.append(steer(E, ej_tip, CHIN_SIDE, 2.5, tol=0.004, label="ejector in front of the mouth, on the chin side"))
    yield from ctx.together(*moves)
    if mode != "nosuction":
        yield from steer(E, ej_tip, sill - M.EJ_AXIS * 0.07, 1.5, tol=0.003, label="ejector at the opening")
        yield from steer(E, ej_tip, sill - M.EJ_AXIS * 0.03, 1.2, label="ejector past the lips")
        yield from steer(E, ej_tip, sill, 1.2, label="ejector onto the floor against the sill")
    yield from B.move(over0 + M.BULB_AXIS * P.TIP_Z, t=1.5, label="bulb over the face")

    # 3. rinse each dot: nozzle in, squeeze a little more, next. All the while the other hand keeps the ejector's tip in
    # the water against the sill, closing the loop on where the tip is seen.
    holding = {"on": mode != "nosuction"}

    def square(arm, name, upright_only=False):
        """Turn the hand so that the tool it holds lies exactly as it started (or, with `upright_only`, just stands upright).
        A soft grip lets a tool turn a little in the fingers; this undoes that, from what is seen."""
        for _ in range(3):
            R_tool = ctx.state(name).rotation()
            corr = _rotation_onto(R_tool @ np.array([0, 0, 1.0]), [0, 0, 1.0]) if upright_only else R_tool.T
            if math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(corr) - 1) / 2)))) < 1.5:
                return
            yield from arm.move(arm.at(), grip=d._as_grip(corr @ arm.grip.rot), t=0.6, label=f"square the {name}")
            yield from arm.wait(0.3)

    def keep_ejector():
        while holding["on"]:
            err = sill - ej_tip()
            if np.linalg.norm(err) > 0.001:
                E._solve(E.at() + np.clip(0.3 * err, -0.002, 0.002) + E.trim, E.grip.rot)
            yield

    def bulb_work():
        teeth = [0, 1] if mode == "skip" else [0, 1, 2]
        for k in teeth:
            shift = np.array([0.0, 0.07, 0.03]) if mode == "outside" else np.zeros(3)  # outside: over the left cheek, outside the lips

            def goal(k=k, back=0.0):  # aimed along the bulb's axis as it is seen now
                a = bulb_axis()
                return M.aim(M.SPOTS[k], a, STANDOFF) - a * back + shift

            if k == teeth[0]:
                yield from steer(B, bulb_tip, lambda: goal(back=APPROACH), 1.0, tol=0.003, label="nozzle over the opening")
            yield from steer(B, bulb_tip, goal, 1.0, label=f"nozzle over tooth {M.TARGETS[k]}")
            f = min(0.0, B.finger + SQUEEZE_STEP)  # open loop: the fingers' target closes by SQUEEZE_STEP over 0.8 s
            yield from B.move(B.at(), finger=f, t=0.8, label="squeeze")
            yield from B.wait(0.4)

        # 4. bulb out and home, while the ejector keeps the mouth clear
        yield from steer(B, bulb_tip, bulb_tip() - bulb_axis() * APPROACH, 1.0, tol=0.004, label="nozzle out")
        yield from B.move(B.at() + wait_pose - ctx.pos("bulb"), t=1.5, label="bulb beside the face")
        home_b = np.array(P.BULB_START)
        # the bulb may have turned a little in the fingers while squeezed: turn the grip so that it hangs straight down
        st = ctx.state("bulb")
        upright = _rotation_onto(st.rotation() @ np.array([0, 0, -1.0]), [0, 0, -1]) @ B.grip.rot
        # high enough that the nozzle's end stays above the ring, then upright, then straight down
        yield from B.move(B.at() + home_b + [0, 0, 0.10] - ctx.pos("bulb"), grip=d._as_grip(upright), t=2.5, label="bulb over its holder")
        yield from square(B, "bulb", upright_only=True)
        yield from steer(B, lambda: ctx.pos("bulb"), home_b + [0, 0, 0.10], 0.5, tol=0.002, label="bulb over its ring")
        yield from square(B, "bulb", upright_only=True)
        yield from steer(B, lambda: ctx.pos("bulb"), home_b + [0, 0, 0.03], 1.2, tol=0.002, label="bulb above its ring")
        yield from steer(B, lambda: ctx.pos("bulb"), home_b + [0, 0, 0.004], 1.0, tol=0.002, label="bulb onto its ring")
        yield from B.open(t=0.6, label="let go of the bulb")  # it drops 4 mm into the ring and settles upright
        yield from B.wait(0.5)
        yield from B.move(B.at() + [0, 0, 0.012], t=0.5, label="fingers clear of the bulb")
        yield from B.move(B.at() - 0.06 * B.approach_axis(), t=0.8, label="fingers back along their axis")
        yield from B.move(B.at() + [0, 0, 0.05], t=0.6, label="fingers up")
        holding["on"] = False

    yield from ctx.together(bulb_work(), keep_ejector())
    if mode != "nosuction":
        yield from E.wait(2.0)  # the vestibule drains back to the sill for a few seconds after the last rinse
        yield from steer(E, ej_tip, sill - M.EJ_AXIS * 0.075, 1.5, tol=0.004, label="ejector out along its leg")
        yield from steer(E, ej_tip, CHIN_SIDE, 1.5, tol=0.006, label="ejector away on the chin side")
        home_e = np.array(P.EJ_START)
        yield from E.move(E.at() + home_e + [0, 0, 0.04] - ctx.pos("ejector"), t=2.5, label="ejector over its cradle")
        yield from square(E, "ejector")
        yield from steer(E, lambda: ctx.pos("ejector"), home_e + [0, 0, 0.004], 1.0, tol=0.002, label="ejector into its cradle")
        yield from E.open(t=0.6, label="let go of the ejector")
        yield from E.move(E.at() + [0, 0, 0.08], t=0.8, label="fingers up")
    yield from B.wait(2.0)
