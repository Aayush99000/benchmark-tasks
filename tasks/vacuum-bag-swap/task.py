"""Housekeeping: "Put a new bag in the vacuum."

A bagged canister vacuum stands free on its wheels with its dust-compartment lid swung open. The full
bag hangs in the compartment from its cardboard collar, which stands in the collar holder on the front
wall over the inlet. The old bag comes up out of the holder by its collar's pull tab and goes in the
wastebasket; the new bag comes out of its carton and goes down the holder's channels until its collar
stops over the inlet; then the lid is swung shut and pressed down until it latches. The vacuum rolls at
a touch, so one hand steadies it by its front wall while the other works.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

import simkit  # noqa: E402
import vac as V  # noqa: E402
from scoring import TrialScorer, to_vac  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "vacuum-bag-swap"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
SCENE_ONLY = False                     # static render for scene approval: arms rest, a short settle run
MAX_STEPS = 80 if SCENE_ONLY else 2000
OBJECT_NAMES = ("vacuum", "old_bag", "new_bag", "lid")

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = (V.vacuum_object(), V.lid_object(), V.old_bag_object(), V.hose_object(), V.carton_object(), V.new_bag_object(),
             V.bin_object())
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_vacuum_bag_swap", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


# -- the lid's latch ------------------------------------------------------------------------------------
LATCH_LOCAL = np.array([V.FRONT_X + 0.012, V.YC, V.RIM_Z + 0.060])   # the latch's thumb button, lid shut (vacuum frame)
LATCH_DEG = 2.0                       # the lid within this of shut on the rim ...
LATCH_REACH = 0.06                    # ... with a hand pressing down on it within this of the button: it latches


def lid_angle(vac_pose, lid_pose):
    """The lid's angle on its hinge, degrees: 0 as left open, -100 shut on the rim."""
    rv = V.quat_matrix(np.asarray(vac_pose, float)[3:7])
    rl = V.quat_matrix(np.asarray(lid_pose, float)[3:7])
    rel = rv.T @ rl
    return math.degrees(math.atan2(float(rel[0, 2]), float(rel[0, 0])))


def _arr(x):
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)


class Dynamics:
    """The lid's latch, from measured poses only (a disclosed stand-in for its spring catch).

    The lid swings on a damped hinge on the body; shut, it rests on the rim (its hinge's stop). Once it
    is within 2 degrees of shut and a hand's tool centre is within 60 mm of the latch button, the latch
    has caught: from then on the hinge is held by a very large damping (no spring, no target), so the lid
    stays shut. Nothing is snapped to a pose.
    """

    def __init__(self, env):
        import omni.usd
        self.base = env.unwrapped
        stage = omni.usd.get_context().get_stage()
        self.hinge = None
        for prim in stage.Traverse():
            if str(prim.GetPath()).endswith("/lid/hinge_joint"):
                self.hinge = prim
        if self.hinge is None:
            raise RuntimeError("lid hinge not found on the stage")

    def reset(self):
        self.latched = False
        self.hinge.GetAttribute("drive:angular:physics:damping").Set(float(V.LID_DAMP))

    def before_step(self, step, action):
        pass

    def after_step(self, step, objects, root_poses, robot):
        vac = np.asarray(root_poses["vacuum"], float)
        deg = lid_angle(vac, root_poses["lid"])
        rv = V.quat_matrix(vac[3:7])
        latch = vac[:3] + rv @ (LATCH_LOCAL - np.asarray(V.VAC_POS))
        near = min(float(np.linalg.norm(np.asarray(robot[s]["tcp"], float)[:3] - latch)) for s in ("left", "right"))
        if not self.latched and deg <= -V.LID_OPEN_DEG + LATCH_DEG and near <= LATCH_REACH:
            self.latched = True
            self.hinge.GetAttribute("drive:angular:physics:damping").Set(float(V.LID_LATCH_DAMP))
        pads = []
        for side in ("left", "right"):
            art = self.base.scene[f"{side}_robot"]
            i = list(art.body_names).index("gripper")
            w, x, y, z = (float(v) for v in _arr(art.data.body_quat_w)[0, i])      # Isaac Lab: w, x, y, z
            axis = V.quat_matrix((x, y, z, w))[:, 0]                              # the fingers close along x
            tcp = np.asarray(robot[side]["tcp"], float)[:3]
            half = max(0.0, -float(sum(robot[side]["fingers"]))) / 2
            pads += [[float(v) for v in tcp + half * axis], [float(v) for v in tcp - half * axis]]
        return {"lid_deg": round(deg, 2), "latched": self.latched, "latch_xyz": [float(v) for v in latch],
                "hand_to_latch_mm": round(1000 * near, 1), "pads": pads}


def make_dynamics(env):
    return Dynamics(env)


def make_scorer():
    return TrialScorer()


# -- demo ----------------------------------------------------------------------------------------------
# Left hand: pinches the old collar's tab from the left side, draws the bag straight up out of the
# holder, carries it to the wastebasket and lets it drop in, then steadies the vacuum by its front wall.
# Right hand: steadies the front wall while the old bag comes out; then takes the new bag out of its
# carton by the tab, turns it square to the holder and runs it down the channels to the stop; then
# swings the lid shut by its side skirt and presses it down until it latches.

REST = {"left": (0.16, 0.14, V.TOP + 0.16), "right": (0.14, -0.12, V.TOP + 0.16)}
RISE = {"left": (0.16, 0.08, V.TOP + 0.26), "right": (0.16, -0.08, V.TOP + 0.26)}   # over the vacuum's rim, out of rest


def frame(a, close):
    """Tool rotation: approach ``a``, fingers closing along ``close`` (made square to it)."""
    a = np.asarray(a, float)
    a = a / np.linalg.norm(a)
    c = np.asarray(close, float)
    c = c - (c @ a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def _pitch(p_deg):
    p = math.radians(p_deg)
    return math.cos(p), math.sin(p)


def side_r(p=0.0):        # right hand, coming in from the right (+y), fingers closing along x
    c, s = _pitch(p)
    return frame((0.0, c, -s), (1.0, 0.0, 0.0))


def side_l(p=0.0):        # left hand, coming in from the left (-y), fingers closing along x
    c, s = _pitch(p)
    return frame((0.0, -c, -s), (-1.0, 0.0, 0.0))


def front(p):              # coming in from the arms' side (+x), pitched down, fingers closing along y
    c, s = _pitch(p)
    return frame((c, 0.0, -s), (0.0, -1.0, 0.0))


F_CARD = FINGER_CLOSED                 # the 2 mm card squeezed hard (commanded shut)
F_WALL = -(0.020 - 0.001) / 2         # the 20 mm side wall gripped
F_NEAR = -0.0135                      # fingers 27 mm apart coming down onto the 20 mm wall
F_SKIRT = FINGER_CLOSED                # the lid's 4 mm side skirt gripped hard
F_HALF = -0.014

PAD_OFF_L = -0.009                    # the left hand aims this far behind the collar: both its fingers then
                                      # stay inside the compartment (they reach ~80 mm below the tool centre)
TAB_UP = 0.055                        # the pinch: this far above the collar's top edge (on the 75 mm tab)
OLD_TAB = np.array([V.COLLAR_X - PAD_OFF_L, V.YC, V.SEAT_Z + V.COLLAR[2] + TAB_UP])
WALL_PAD = 0.005                      # these pads close this far toward +y of the tool centre: aim off by it
STEADY = {"right": np.array([0.310, V.YC - (V.IN_HALF_W + V.HALF_W) / 2 - WALL_PAD, V.RIM_Z - 0.008]),   # the side
          "left": np.array([0.310, V.YC + (V.IN_HALF_W + V.HALF_W) / 2 - WALL_PAD, V.RIM_Z - 0.008])}    # wall's top
STEADY_P = 45.0
SOCKET = np.array([V.FRONT_X - 0.008, V.YC, V.INLET_Z])   # the hose socket on the front: outside the lid's sweep
SOCKET_P = 25.0
F_SOCK = -(0.060 - 0.002) / 2         # the 60 mm socket gripped
F_SOCK_NEAR = -0.036                  # fingers 72 mm apart coming round it
LID_LET_GO = 72.0                     # swung forward past ~85 degrees open the lid falls shut by itself
LIFT = 0.205                          # the old bag drawn straight up this far: clear of the rim
BIN_DROP = np.array([V.BIN_C[0], V.BIN_C[1] - 0.010, V.TOP + 0.480])
NB = np.array(V.new_bag_pos())
NEW_TAB = np.array([NB[0], NB[1], NB[2] + V.COLLAR[2] / 2 + TAB_UP])
PAD_OFF = 0.0                         # where the card sits in this hand's pads varies by a few mm: aim at the middle
PUSH = 0.025                          # the hand presses on past the stop: the card slips this much in the pads
START_Z = V.RIM_Z + 0.015             # the new collar's bottom edge just over the channels' lead-in
CARRY_Z = V.TOP + 0.440                # a bag carried by its tab this high clears the rim (it hangs 205-215 mm below)


def _w(t, p, f, label, rot):
    return Waypoint(t, tuple(float(v) for v in p), f, label, rot)


def _lid_point(deg_open, dx=-0.120, dz=0.040, y=-0.143):
    """A point on the lid (laid out closed at (dx, dz) from the hinge) with the lid open ``deg_open``."""
    t = math.radians(deg_open)
    return np.array([V.HINGE_X + dx * math.cos(t) + dz * math.sin(t), y, V.RIM_Z - dx * math.sin(t) + dz * math.cos(t)])


def demo():
    if SCENE_ONLY:
        return {side: [_w(1.5, REST[side], FINGER_OPEN, "look", DOWN), _w(MAX_STEPS / HZ, REST[side], FINGER_OPEN, "hold", DOWN)]
                for side in ("left", "right")}
    up = np.array([0.0, 0.0, 1.0])
    # -- left: the old bag out and in the bin ----------------------------------------------------------
    left = [
        _w(1.5, REST["left"], FINGER_OPEN, "look", DOWN),
        _w(1.5, RISE["left"], FINGER_OPEN, "up over the rim", side_l()),
        _w(2.0, OLD_TAB + np.array([0.0, 0.0, 0.06]), F_HALF, "over the old collar's tab", side_l()),
        _w(1.5, OLD_TAB + np.array([0.0, 0.0, 0.06]), F_HALF, "wait for the vacuum to be held", side_l()),
        _w(1.5, OLD_TAB, F_HALF, "straight down round the tab", side_l()),
        _w(0.8, OLD_TAB, F_CARD, "pinch the tab", side_l()),
        _w(6.0, OLD_TAB + LIFT * up, F_CARD, "draw the bag straight up out of the holder", side_l()),
        _w(1.5, OLD_TAB + LIFT * up + np.array([-0.02, 0.10, 0.01]), F_CARD, "lift it over the side", side_l(10)),
        _w(2.5, BIN_DROP, F_CARD, "over the wastebasket", front(25)),
        _w(1.0, BIN_DROP - 0.03 * up, F_CARD, "lower it in", front(25)),
        _w(0.8, BIN_DROP - 0.03 * up, FINGER_OPEN, "let it drop", front(25)),
        _w(1.2, BIN_DROP + 0.02 * up, FINGER_OPEN, "draw back", front(25)),
        _w(2.5, STEADY["left"] + np.array([-0.06, 0.0, 0.05]), F_NEAR, "over the vacuum's left side wall", front(STEADY_P)),
        _w(1.2, STEADY["left"], F_NEAR, "round the side wall", front(STEADY_P)),
        _w(0.8, STEADY["left"], F_WALL, "hold the vacuum steady", front(STEADY_P)),
    ]
    t_left_hold = sum(w.duration for w in left)
    # -- right: steady, then the new bag in, then the lid ----------------------------------------------
    k_up = next(i for i, w in enumerate(left) if w.label.startswith("draw the bag"))
    t_out = sum(w.duration for w in left[:k_up + 1])   # the old bag drawn clear of the rim
    right = [
        _w(1.5, REST["right"], FINGER_OPEN, "look", DOWN),
        _w(1.5, RISE["right"], FINGER_OPEN, "up over the rim", side_r()),
        _w(1.5, STEADY["right"] + np.array([-0.06, 0.0, 0.05]), F_NEAR, "over the vacuum's right side wall", front(STEADY_P)),
        _w(1.2, STEADY["right"], F_NEAR, "round the side wall", front(STEADY_P)),
        _w(0.8, STEADY["right"], F_WALL, "hold the vacuum steady", front(STEADY_P)),
    ]
    t = sum(w.duration for w in right)
    right += [
        _w(max(0.5, t_out + 0.5 - t), STEADY["right"], F_WALL, "hold it while the old bag comes out", front(STEADY_P)),
        _w(0.6, STEADY["right"], FINGER_OPEN, "let go of the wall", front(STEADY_P)),
        _w(1.2, STEADY["right"] + np.array([-0.06, 0.0, 0.05]), FINGER_OPEN, "back off the wall", front(STEADY_P)),
        _w(2.5, NEW_TAB + np.array([-0.07, 0.0, 0.02]), F_HALF, "in front of the carton", front(0)),
        _w(1.5, NEW_TAB, F_HALF, "round the new collar's tab", front(0)),
        _w(0.8, NEW_TAB, F_CARD, "pinch the tab", front(0)),
        _w(2.0, NEW_TAB + np.array([0.0, 0.0, CARRY_Z - NEW_TAB[2]]), F_CARD, "lift the bag out of the carton", front(0)),
    ]
    above = np.array([V.COLLAR_X - PAD_OFF, V.YC, START_Z + V.COLLAR[2] + TAB_UP])
    t = sum(w.duration for w in right)
    right += [
        _w(3.0, np.array([above[0], above[1] - 0.06, CARRY_Z]), F_CARD, "turn it square to the holder", side_r()),
        _w(max(1.5, t_left_hold + 0.3 - t - 3.0), above, F_CARD, "over the channels", side_r()),
        _w(1.5, above - 0.060 * up, F_CARD, "into the channels' lead-in", side_r()),
        _w(2.5, above - (START_Z - V.SEAT_Z + PUSH) * up, F_CARD, "down the channels and press it onto the stop", side_r()),
        _w(0.6, above - (START_Z - V.SEAT_Z + PUSH) * up, F_HALF, "let go of the tab", side_r()),
        _w(1.2, above - (START_Z - V.SEAT_Z - 0.010) * up + np.array([0.0, -0.08, 0.0]), F_HALF, "back out", side_r()),
    ]
    # the lid: grip its right side skirt, swing it forward past upright, let it fall shut, press it down
    g0, g1 = _lid_point(100.0), _lid_point(LID_LET_GO)
    right += [
        _w(2.5, g0 + np.array([-0.06, 0.0, 0.0]), F_HALF, "in front of the lid's side skirt", front(20)),
        _w(1.2, g0, F_HALF, "round the skirt", front(20)),
        _w(0.8, g0, F_SKIRT, "grip the skirt", front(20)),
    ]
    n = 4
    p_end = 20.0 - 0.6 * (100.0 - LID_LET_GO)          # the hand turns with the lid, most of the way
    for i in range(1, n + 1):
        deg = 100.0 + (LID_LET_GO - 100.0) * i / n
        right.append(_w(0.6, _lid_point(deg), F_SKIRT, "swing the lid forward past upright",
                        front(20.0 + (p_end - 20.0) * i / n)))
    right += [
        _w(0.5, g1, F_HALF, "let it go", front(p_end)),
        _w(1.0, g1 + np.array([-0.07, 0.0, 0.0]), F_HALF, "draw the fingers back off the skirt", front(p_end)),
        _w(1.0, g1 + np.array([-0.09, -0.07, 0.02]), F_HALF, "out of its way", front(p_end)),
        _w(2.0, g1 + np.array([-0.10, -0.07, 0.04]), FINGER_CLOSED, "clear while it swings shut", front(30)),
    ]
    press = np.array([V.FRONT_X + 0.022, V.YC - 0.030, V.RIM_Z + 0.082])
    right += [
        _w(2.0, press + 0.04 * up, FINGER_CLOSED, "over the lid's front", front(30)),
        _w(1.2, press - 0.012 * up, FINGER_CLOSED, "press it down till it latches", front(30)),
        _w(0.8, press + 0.05 * up, FINGER_CLOSED, "lift off", front(30)),
        _w(1.5, RISE["right"], FINGER_OPEN, "up and away", side_r()),
        _w(2.0, REST["right"], FINGER_OPEN, "stand clear", DOWN),
        _w(3.0, REST["right"], FINGER_OPEN, "done", DOWN),
    ]
    # while the new bag goes in the left hand holds the side wall; once the right hand is out it moves to
    # the hose socket on the front (clear of the lid's sweep) and keeps hold of the vacuum there until the
    # lid is latched
    k_out = next(i for i, w in enumerate(right) if w.label == "back out")
    t_out_new = sum(w.duration for w in right[:k_out + 1])
    t_right_done = sum(w.duration for w in right) - 5.0
    left += [
        _w(max(0.5, t_out_new - t_left_hold), STEADY["left"], F_WALL, "hold the vacuum steady while the new bag goes in",
           front(STEADY_P)),
        _w(0.4, STEADY["left"], F_NEAR, "let go of the wall", front(STEADY_P)),
        _w(0.8, STEADY["left"] + np.array([-0.08, 0.0, 0.08]), F_NEAR, "up off the rim", front(STEADY_P)),
        _w(1.5, SOCKET + np.array([-0.07, 0.0, 0.07]), F_SOCK_NEAR, "in front of the vacuum's hose socket", front(SOCKET_P)),
        _w(1.0, SOCKET, F_SOCK_NEAR, "round the socket", front(SOCKET_P)),
        _w(0.6, SOCKET, F_SOCK, "hold the vacuum steady by its hose socket", front(SOCKET_P)),
    ]
    t_left_hold = sum(w.duration for w in left)
    left += [
        _w(max(0.5, t_right_done - t_left_hold), SOCKET, F_SOCK, "hold the vacuum steady by its hose socket", front(SOCKET_P)),
        _w(0.5, SOCKET, F_SOCK_NEAR, "let go of the socket", front(SOCKET_P)),
        _w(1.2, SOCKET + np.array([-0.07, 0.0, 0.07]), F_SOCK_NEAR, "back off", front(SOCKET_P)),
        _w(2.0, REST["left"], FINGER_OPEN, "stand clear", DOWN),
        _w(6.0, REST["left"], FINGER_OPEN, "done", DOWN),
    ]
    return {"left": left, "right": right}
