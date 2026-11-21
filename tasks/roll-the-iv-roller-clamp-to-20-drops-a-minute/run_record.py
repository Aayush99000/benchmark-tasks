"""The run record for roll_the_iv_roller_clamp_to_20_drops_a_minute.py, kept apart so the task file stays short: what
the task file's hooks append to ``~/runs/<task file stem>-trace.txt`` on the simulator machine (only where that folder
exists; nothing changes in the trial). ``M`` is the task module's namespace."""

from __future__ import annotations

import math
import pathlib
from types import SimpleNamespace

import numpy as np
from benchkit import _np, joint_name
from clamp_demo import UNDER
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, PAD_TIP_PAST_TCP

RUN_RECORD = pathlib.Path.home() / "runs"  # the simulator machine's run folder (collect.sh copies it back); absent elsewhere
EVERY = 20  # control steps between lines: once a second at 20 Hz
SEEN = {"travel": 0.0, "eased": None, "from": 0.0}  # the wheel's travel at the last line, for its speed; the demo
# command it was eased off after, and the wheel's travel a second into that wait
SETTLE_S = 1.0  # s after the fingertip starts easing off before the wheel's hold is measured


def _record(g: dict, line: str) -> None:
    """Append ``line`` to ``~/runs/<task file stem>-trace.txt``, only where that folder exists."""
    if RUN_RECORD.is_dir():
        with open(RUN_RECORD / f"{pathlib.Path(g['__file__']).stem}-trace.txt", "a") as f:
            f.write(line + "\n")


def reset(g: dict) -> None:
    """At each reset (``g`` the task module's globals): forget the demo's last command, record the wheel's joint."""
    g["LATEST"].pop("command", None)
    g["LATEST"]["hold_drift_mm"] = 0.0
    SEEN.update(travel=0.0, eased=None, **{"from": 0.0})
    if RUN_RECORD.is_dir():
        _record(g, f"reset wheel joint: {wheel_joint_physics(SimpleNamespace(**g))}")


def tick(g: dict, states, rate: float) -> None:
    """After each control step: the wheel's hold while nothing pushes it (``hold``), and once a second a line
    (``trace_line``). Never lets the record end a trial."""
    try:
        hold(g)
    except Exception:  # the record is diagnostic
        pass
    if g["LOG"].step % EVERY or not RUN_RECORD.is_dir():
        return
    M = SimpleNamespace(**g)
    try:
        _record(g, trace_line(M, states, rate))
    except Exception as err:  # the record is diagnostic
        _record(g, f"{M.LOG.step / M.CONTROL_HZ:.0f}s trace failed: {type(err).__name__}: {err}")


def ended(g: dict, states) -> None:
    """When both arms are back at the start: where the wheel was left, its count and the score."""
    if RUN_RECORD.is_dir():
        M = SimpleNamespace(**g)
        try:
            _record(g, f"ended at step {M.LOG.step} ({M.LOG.step / M.CONTROL_HZ:.2f} s): wheel "
                       f"{M.LOG.travel * 1000:.2f} mm, {M.counted(M.LOG.travel)} drops in the minute, "
                       f"score {M.grade(states):g}")
        except Exception as err:  # the record is diagnostic
            print(f"[run_record] {type(err).__name__}: {err}")


def hold(g: dict) -> None:
    """While the scripted demo's fingertip is eased off the wheel (its command "eased") and the left hand still holds
    the housing: how far the wheel has moved since SETTLE_S into that wait, the largest kept in
    ``LATEST["hold_drift_mm"]``. A real wheel stays where it is left: this stays 0 unless the simulator creeps it."""
    cmd, log = g["LATEST"].get("command"), g["LOG"]
    if not cmd or cmd[2] != "eased" or not cmd[3] or log.ended:
        SEEN["eased"] = None
        return
    key = (cmd[0], cmd[1])
    if SEEN["eased"] != key:
        SEEN["eased"], SEEN["since"] = key, log.step
        return
    if log.step - SEEN["since"] == int(round(SETTLE_S * g["CONTROL_HZ"])):
        SEEN["from"] = log.travel
    elif log.step - SEEN["since"] > SETTLE_S * g["CONTROL_HZ"]:
        drift = abs(log.travel - SEEN["from"]) * 1000
        g["LATEST"]["hold_drift_mm"] = round(max(g["LATEST"].get("hold_drift_mm", 0.0), drift), 4)


def _array(x) -> np.ndarray:
    try:
        return _np(x)
    except Exception:  # a warp array without a torch view
        return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=float)


def wheel_joint_physics(M) -> str:
    """The wheel joint's friction, damping and stiffness as the simulator holds them, by Isaac Lab's names (static
    friction is an effort in N from Isaac Sim 5 on; a coefficient before), and PhysX's own [static, dynamic, viscous]."""
    env = M._ENV["env"]
    if env is None:
        return "no simulator"
    try:
        asset = env.unwrapped.scene[M.CLAMP.name]
        j = list(asset.joint_names).index(joint_name(M.CLAMP.sliders[0]))
        out = []
        for attr in ("joint_friction_coeff", "joint_dynamic_friction_coeff", "joint_viscous_friction_coeff",
                     "joint_damping", "joint_stiffness", "joint_armature"):
            value = getattr(asset.data, attr, None)
            if value is not None:
                out.append(f"{attr.removeprefix('joint_')} {float(_array(value)[0][j]):.4g}")
        for view_name in ("root_physx_view", "root_view"):
            get = getattr(getattr(asset, view_name, None), "get_dof_friction_properties", None)
            if get is not None:
                out.append(f"physx friction props {np.round(_array(get())[0][j], 4).tolist()}")
                break
        return ", ".join(out) or "no joint data"
    except Exception as err:  # the record is diagnostic: never let it end a trial
        return f"unreadable ({type(err).__name__}: {err})"


def arm_tips(M) -> dict:
    """Each arm's pad tips (midway between the closed pads, PAD_TIP_PAST_TCP past the TCP along the approach), its
    finger gap (m) and the direction its closed fingertips' underside faces, from the arms' joints; empty outside the
    simulator."""
    env = M._ENV["env"]
    out = {}
    if env is None:
        return out
    for arm in ("left", "right"):
        robot = env.unwrapped.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        joints = _np(robot.data.joint_pos)[0]
        pos, rot = ik.tcp_pose(joints[[names.index(j) for j in ARM_JOINTS]], station.BASES[arm])
        gap = float(sum(abs(joints[names.index(j)]) for j in FINGER_JOINTS))
        out[arm] = (np.asarray(pos) + PAD_TIP_PAST_TCP * np.asarray(rot)[:, 2], gap, np.asarray(rot)[:, 1])
    return out


def trace_line(M, states, rate: float) -> str:
    """One second of the run: the wheel (travel, speed, drip), the demo's command if a demo plays, the right pad tip
    against the wheel's 45-degree face toward shut (depth into it, how far up it toward the top edge, which is 2.9 mm
    up from the face's middle, sideways, and while the demo pushes, how far the arm is held back from where it is
    sent along the face's normal: about 1.2 mm per newton of push in the arm's pose), the fingertips' underside against
    the wheel's face toward the bag (and while a correction rolls it back, how far the arm is held back: about 6.7 mm
    per newton), the housing (moved from where it was spawned, which includes its 0.5 mm settle; turned; tipped; its
    pitch along the wheel's slide, + the bag end up), the wheel's largest drift while nothing pushed it (``hold``),
    where the left hand's pad tip sits on the housing (a drift is slip) and both finger gaps, the scorer's flags and
    the score so far. Lengths in mm."""
    clamp = states["roller_clamp"]
    t = M.LOG.travel
    speed = (t - SEEN["travel"]) * M.CONTROL_HZ / EVERY
    SEEN["travel"] = t
    drip = "stream" if math.isinf(rate) else f"{rate:.1f}/min"
    parts = [f"{M.LOG.step / M.CONTROL_HZ:.0f}s wheel {t * 1000:.2f} speed {speed * 1000:+.2f}/s drip {drip}"]
    cmd = M.LATEST.get("command")
    if cmd:
        key, sent, mode, live, press = (*cmd, 0.0)[:5]
        parts.append(f"demo {key} {mode} sent {sent * 1000:.2f} lag {(sent - t) * 1000:+.2f} press {press * 1000:+.2f}"
                     f"{'' if live else ' OFFLINE'}")
    rot = clamp.rotation()
    try:
        tips = arm_tips(M)
    except Exception as err:  # diagnostic only
        tips, parts = {}, parts + [f"arms unreadable ({type(err).__name__})"]
    if "right" in tips:
        d = rot.T @ (tips["right"][0] - clamp.pos) - np.array((M.WHEEL_X0 + t - M.FACET, 0.0, M.WHEEL_Z + M.FACET))
        depth = float(np.dot(d, (1.0, 0.0, -1.0))) / math.sqrt(2.0)  # into the face; negative: off it
        up = float(np.dot(d, (1.0, 0.0, 1.0))) / math.sqrt(2.0)  # up the face toward the wheel's top
        held = ""
        if cmd and cmd[2] == "push":  # the demo sends the tip `press` past the face of travel `sent` along its normal:
            sent_depth = (cmd[1] - t) / math.sqrt(2.0) + (cmd[4] if len(cmd) > 4 else 0.0)  # that far past this face
            held = f" held back {(sent_depth - depth) * 1000:+.1f}"  # the arm's give: its push, about 0.8 N/mm
        parts.append(f"right tip depth {depth * 1000:+.1f} up {up * 1000:+.1f} side {d[1] * 1000:+.1f}{held}")
        # the fingertips' underside against the wheel's face toward the bag (where a correction back presses)
        under = rot.T @ (tips["right"][0] + UNDER * tips["right"][2] - clamp.pos)
        e = under - np.array((M.WHEEL_X0 + t + M.FACET, 0.0, M.WHEEL_Z + M.FACET))
        back = -float(np.dot(e, (1.0, 0.0, 1.0))) / math.sqrt(2.0)  # into the face; negative: off it
        held = ""
        if cmd and cmd[2] == "back":  # sent `press` past the face of travel `sent` along its normal
            sent_back = (t - cmd[1]) / math.sqrt(2.0) + (cmd[4] if len(cmd) > 4 else 0.0)
            held = f" held back {(sent_back - back) * 1000:+.1f}"  # the arm's give: about 0.15 N/mm this way
        parts.append(f"underside on bag face {back * 1000:+.1f}{held}")
    moved = (clamp.pos - np.asarray(M.CLAMP.pos, dtype=float)) * 1000
    yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]) - M.CLAMP_YAW)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(rot[2, 0])))))  # + : the bag end up, along the wheel
    parts.append(f"housing moved {moved[0]:+.1f},{moved[1]:+.1f},{moved[2]:+.1f} yaw {(yaw + 180) % 360 - 180:+.1f} "
                 f"tilt {clamp.tilt_deg():.1f} pitch {pitch:+.2f}")
    parts.append(f"hold drift {M.LATEST.get('hold_drift_mm', 0.0):.2f}")
    if "left" in tips:
        local = rot.T @ (tips["left"][0] - clamp.pos) * 1000
        parts.append(f"left tip on housing {local[0]:+.1f},{local[1]:+.1f},{local[2]:+.1f}")
    if tips:
        parts.append("gaps " + "/".join(f"{tips[a][1] * 1000:.1f}" for a in ("left", "right") if a in tips))
    flags = [n for n in ("touched", "clear", "kinked", "ended", "streamed", "pulled") if getattr(M.LOG, n)]
    parts.append(f"flags {','.join(flags) or '-'} score {M.grade(states):g}")
    return " | ".join(parts)
