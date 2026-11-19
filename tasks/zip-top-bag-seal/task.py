"""Food prep: "Seal the bag."

A filled 1-quart zip-top freezer bag of chopped vegetables lies on a butcher block, its press-to-seal
track open and standing out past the block's near edge. One hand holds the bag down at a corner while
the other pinches the track shut a pad-width at a time from that corner to the other corner, pressing the
rib into its groove the whole way; then both hands let go and stand clear.

A track cell closes only from real PhysX contacts: one hand's two finger pads pinching it from above and
below, each with at least 10 N, for 0.15 s (Dynamics). There is no position or jaw-gap trigger.
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
import bag as B  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "zip-top-bag-seal"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 1600
OBJECT_NAMES = ("bag", *(f"coin{i + 1}" for i in range(6)))

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = (B.board_object(), B.scraps_object(), B.bowl_object(), B.bag_box_object(), B.bag_object(),
             *(B.coin_object(i) for i in range(len(B.COINS))))
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_zip_top_bag_seal", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


# -- the track's closing rule ------------------------------------------------------------------------
F_SEAL = 10.0                        # N: each pad pressing on the track at least this hard (a firm finger-and-
                                     # thumb pinch; YAM's gripper can squeeze up to about 50 N)
OPPOSED = -0.5                       # the two pads' forces on the track point opposite ways (a pinch, not a push)
DWELL = 3                            # control steps (0.15 s) a cell must be pinched that hard to snap shut
BAND_X = 0.0045                      # contact points on the track: within this of its centreline across the bag
BAND_Z = (-0.002, 0.008)             # ... and between just under the groove and the open rib above it

SHARED = {}


def _arr(x):
    x = getattr(x, "torch", x)
    x = x.numpy() if hasattr(x, "numpy") and not hasattr(x, "detach") else x
    return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)


class Dynamics:
    """The track closing, from PhysX contacts (a disclosed stand-in for the rib snapping into its groove).

    A rigid contact view on the bag, filtered to the four finger-pad links (each hand's tip_left and
    tip_right), gives the real contact force of each pad on the bag and the points where they touch. A
    cell of the track is pressed where one hand's two pads both press on the track (their contact points
    within 4.5 mm of its centreline and at its height), each at least 10 N, their forces opposed (pinching
    it from above and below), over that cell's length. A cell held so for 0.15 s snaps shut, stays shut and
    reads green. A hand passing beside the track, touching it lightly, or pushing it from one side only
    closes nothing. The rib's 3 mm travel into its groove is not simulated (the pads pinch the rib and groove
    profiles as they stand); a pinch pulled along the track drags the bag by friction.
    """

    def __init__(self, env):
        import omni.usd
        from pxr import UsdPhysics
        self.base = env.unwrapped
        self.stage = omni.usd.get_context().get_stage()
        index = {i: (name, k) for name, ids in B.TRACK_PIECES.items() for k, i in enumerate(ids)}
        self.prims = {}
        self.bag_path = None
        self.tips = {}
        for prim in self.stage.Traverse():
            path = str(prim.GetPath())
            if "/bag/piece" in path:
                tail = path.rsplit("/bag/piece", 1)[1]
                if tail.isdigit() and int(tail) in index:
                    self.prims[index[int(tail)]] = prim
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                if path.endswith("/bag"):
                    self.bag_path = path
                leaf = path.rsplit("/", 1)[-1]
                if leaf in ("tip_left", "tip_right"):
                    side = "left" if "Left" in path else "right" if "Right" in path else None
                    if side is not None:
                        self.tips[(side, leaf)] = path
        if len(self.prims) != 5 * B.N_CELLS:
            raise RuntimeError(f"bag track pieces not found on the stage ({len(self.prims)})")
        if self.bag_path is None or len(self.tips) != 4:
            raise RuntimeError(f"bag body or finger pads not found: {self.bag_path} {sorted(self.tips.items())}")
        print("[zip] contact view on", self.bag_path, "filtered to", sorted(self.tips.items()), flush=True)
        self.order = [("left", "tip_left"), ("left", "tip_right"), ("right", "tip_left"), ("right", "tip_right")]
        self.cview = None

    def _contacts(self):
        """Per pad: its net contact force on the bag (N, world) and its contact points (world)."""
        if self.cview is None:
            from isaaclab_physx.physics.physx_manager import PhysxManager
            psv = PhysxManager.get_physics_sim_view()
            self.cview = psv.create_rigid_contact_view([self.bag_path], filter_patterns=[[self.tips[o] for o in self.order]],
                                                       max_contact_data_count=256)
        dt = 1.0 / 240.0
        F = _arr(self.cview.get_contact_force_matrix(dt=dt)).reshape(len(self.order), 3)
        forces, points, normals, seps, counts, starts = (_arr(v) for v in self.cview.get_contact_data(dt=dt))
        counts = counts.reshape(-1)
        starts = starts.reshape(-1)
        points = points.reshape(-1, 3)
        forces = forces.reshape(-1)
        out = {}
        for j, o in enumerate(self.order):
            n, s0 = int(counts[j]), int(starts[j])
            pts = points[s0:s0 + n][np.abs(forces[s0:s0 + n]) > 1e-6] if n else np.zeros((0, 3))
            out[o] = (F[j], pts)
        return out

    def _show(self, k, closed):
        from pxr import UsdGeom
        for name, vis in (("groove", not closed), ("rib", not closed), ("sealed", closed)):
            im = UsdGeom.Imageable(self.prims[(name, k)])
            im.MakeVisible() if vis else im.MakeInvisible()

    def reset(self):
        self.closed = [False] * B.N_CELLS
        self.held = [0] * B.N_CELLS
        self.peak = 0.0
        for k in range(B.N_CELLS):
            self._show(k, False)

    def before_step(self, step, action):
        pass

    def after_step(self, step, objects, root_poses, robot):
        bag = np.asarray(root_poses["bag"], float)
        r = B.quat_matrix(bag[3:7])
        origin = bag[:3]
        track_c = np.array([B.TRACK_X - B.ROOT_X, 0.0, B.GROOVE_Z - B.ROOT_Z])
        con = self._contacts()
        pinch = {}
        pressed = set()
        for side in ("left", "right"):
            (fa, pa), (fb, pb) = con[(side, "tip_left")], con[(side, "tip_right")]
            ma, mb = float(np.linalg.norm(fa)), float(np.linalg.norm(fb))
            opp = float(fa @ fb) / (ma * mb) if ma > 1e-6 and mb > 1e-6 else 1.0
            spans = []
            for pts in (pa, pb):
                loc = (pts - origin) @ r - track_c                  # bag frame, from the groove's centreline
                on = loc[(np.abs(loc[:, 0]) <= BAND_X) & (loc[:, 2] >= BAND_Z[0]) & (loc[:, 2] <= BAND_Z[1])]
                spans.append((float(on[:, 1].min()), float(on[:, 1].max())) if len(on) else None)
            squeeze = min(ma, mb)
            cells = []
            if squeeze >= F_SEAL and opp <= OPPOSED and spans[0] and spans[1]:
                lo, hi = max(spans[0][0], spans[1][0]), min(spans[0][1], spans[1][1])
                lo, hi = min(lo, hi), max(lo, hi)
                for k in range(B.N_CELLS):
                    yc = B.cell_y(k) - B.BAG_Y
                    if lo - B.CELL / 2 <= yc <= hi + B.CELL / 2:
                        cells.append(k)
            pressed.update(cells)
            if spans[0] and spans[1]:
                self.peak = max(self.peak, squeeze if opp <= OPPOSED else 0.0)
            where = []
            for pts in (pa, pb):
                loc = (pts - origin) @ r - track_c
                where.append([round(1000 * float(v), 1) for v in (loc[:, 0].min(), loc[:, 0].max(), loc[:, 2].min(), loc[:, 2].max())]
                             if len(loc) else [])
            pinch[side] = {"pad_N": [round(ma, 1), round(mb, 1)], "opposed": round(opp, 2), "on_track": [s is not None for s in spans],
                           "cells": cells, "pad_F": [[round(float(v), 1) for v in fa], [round(float(v), 1) for v in fb]],
                           "where_mm": where}
        for k in range(B.N_CELLS):
            self.held[k] = self.held[k] + 1 if k in pressed else 0
            if self.held[k] >= DWELL and not self.closed[k]:
                self.closed[k] = True
                self._show(k, True)
        SHARED["bag"] = bag
        return {"closed": int(sum(self.closed)), "closed_cells": "".join("1" if c else "0" for c in self.closed),
                "pinch": pinch, "peak_pinch_N": round(self.peak, 1)}


def make_dynamics(env):
    return Dynamics(env)


def make_scorer():
    return TrialScorer()


# -- demo ----------------------------------------------------------------------------------------------
# The left hand comes in from the left side and pinches the mouth's left corner, track and flanges
# together, and holds it there; the right hand pinches the track beside it and slides along it to the
# right corner, pulling along the track against the held corner. Then the left lets go and the right
# comes back to press home the bit of track the left hand was holding.

REST = {"left": (0.16, 0.24, B.TOP + 0.16), "right": (0.16, -0.24, B.TOP + 0.16)}
PITCH = math.radians(25.0)           # each hand comes at the track from above as well
A_R = np.array([math.cos(PITCH), 0.0, -math.sin(PITCH)])               # right hand: in across the mouth (+x)
ROT_R = np.column_stack([np.array([math.sin(PITCH), 0.0, math.cos(PITCH)]), np.array([0.0, -1.0, 0.0]), A_R])
A_L = np.array([0.0, -math.cos(PITCH), -math.sin(PITCH)])              # left hand: in from the left side (-y)
ROT_L = np.column_stack([np.array([0.0, -math.sin(PITCH), math.cos(PITCH)]), np.array([-1.0, 0.0, 0.0]), A_L])
F_PINCH = FINGER_CLOSED                        # squeeze the track hard: the pads stop on the 6.2 mm
                                               # rib-over-groove stack and press it at about 15-19 N each
                                               # (measured), over the 10 N needed
F_EASE = -0.005                                # pads 10 mm apart: eased off the track to step along it
F_HOLD = FINGER_CLOSED                         # the holding hand pinches its corner hard
F_HALF = -0.012                                # pads 24 mm apart coming in round the track
CORNER_L = np.array([B.TRACK_X + 0.002, B.BAG_Y + B.BAG_W / 2 - 0.0075, B.GROOVE_Z])   # the left hand's pinch
Y0, Y1 = B.BAG_Y + 0.060, B.BAG_Y - 0.086      # the right hand's slide, from beside the held corner
Y_LAST = B.BAG_Y + 0.073                       # then the held corner, pressed home
AHEAD = 0.004                                  # the track this far ahead of the TCP


def _at(p, a, back=0.0, up=0.0):
    return tuple(map(float, np.asarray(p, float) - a * (AHEAD + back) + np.array([0.0, 0.0, up])))


def _r(y, back=0.0, up=0.0):
    return _at((B.TRACK_X, y, B.GROOVE_Z), A_R, back, up)


STEPS = 20                           # pinches along the 146 mm from beside the held corner, 7.3 mm apart
                                     # (each pinch's contact patch is narrow, so they overlap to miss no cell)


def demo():
    right = [
        Waypoint(1.5, REST["right"], FINGER_OPEN, "look", DOWN),
        Waypoint(2.5, _r(Y0, back=0.060, up=0.030), F_HALF, "in front of the mouth", ROT_R),
        Waypoint(1.5, _r(Y0, back=0.030), F_HALF, "level with the track", ROT_R),
        Waypoint(1.5, _r(Y0), F_HALF, "round the track beside the held corner", ROT_R),
        Waypoint(1.0, _r(Y0), F_PINCH, "pinch it", ROT_R),
    ]
    # press along the track a pad-width at a time: ease off, step 15 mm along, pinch it hard again (a
    # pinch dragged along at sealing force would pull the held corner out of the other hand)
    n = STEPS
    for i in range(1, n + 1):
        y = Y0 + (Y1 - Y0) * i / n
        right += [
            Waypoint(0.3, _r(Y0 + (Y1 - Y0) * (i - 1) / n), F_EASE, "ease off the track", ROT_R),
            Waypoint(0.4, _r(y), F_EASE, "step along it", ROT_R),
            Waypoint(0.4, _r(y), F_PINCH, "pinch it shut here", ROT_R),
            Waypoint(0.4, _r(y), F_PINCH, "hold the pinch", ROT_R),
        ]
    right += [
        Waypoint(0.5, _r(Y1 + 0.004), F_HALF, "ease the pinch", ROT_R),
        Waypoint(0.8, _r(Y1 - 0.004), F_HALF, "round the far corner", ROT_R),
        Waypoint(1.0, _r(Y1 - 0.004), F_PINCH, "press the far corner home", ROT_R),
        Waypoint(0.8, _r(Y1), F_HALF, "let go of the track", ROT_R),
        Waypoint(1.2, _r(Y1, back=0.040, up=0.020), F_HALF, "draw back off the mouth", ROT_R),
    ]
    t_back = sum(w.duration for w in right)
    right += [
        Waypoint(3.0, _r(Y_LAST, back=0.040, up=0.020), F_HALF, "across to the held corner", ROT_R),
        Waypoint(1.5, _r(Y_LAST, back=0.030), F_HALF, "level with the track", ROT_R),
        Waypoint(1.2, _r(Y_LAST), F_HALF, "round the track at the corner", ROT_R),
        Waypoint(1.0, _r(Y_LAST), F_PINCH, "press the last of it home", ROT_R),
        Waypoint(0.3, _r(Y_LAST), F_EASE, "ease off", ROT_R),
        Waypoint(0.4, _r(Y_LAST - 0.010), F_EASE, "step back along it", ROT_R),
        Waypoint(0.4, _r(Y_LAST - 0.010), F_PINCH, "pinch the join with the run", ROT_R),
        Waypoint(0.4, _r(Y_LAST - 0.010), F_PINCH, "hold the pinch", ROT_R),
        Waypoint(0.8, _r(Y_LAST - 0.010), F_HALF, "let go", ROT_R),
        Waypoint(1.2, _r(Y_LAST - 0.010, back=0.050, up=0.020), F_HALF, "draw back", ROT_R),
        Waypoint(2.0, REST["right"], FINGER_OPEN, "stand clear", DOWN),
        Waypoint(2.5, REST["right"], FINGER_OPEN, "done", DOWN),
    ]
    left = [
        Waypoint(1.5, REST["left"], FINGER_OPEN, "look", DOWN),
        Waypoint(2.5, _at(CORNER_L, A_L, back=0.060, up=0.030), F_HALF, "beside the mouth's left corner", ROT_L),
        Waypoint(1.5, _at(CORNER_L, A_L, back=0.030), F_HALF, "level with the track", ROT_L),
        Waypoint(1.2, _at(CORNER_L, A_L), F_HALF, "round the corner", ROT_L),
        Waypoint(0.8, _at(CORNER_L, A_L), F_HOLD, "pinch the corner and hold it", ROT_L),
    ]
    t = sum(w.duration for w in left)
    left += [
        Waypoint(max(0.5, t_back - t), _at(CORNER_L, A_L), F_HOLD, "hold the corner", ROT_L),
        Waypoint(0.6, _at(CORNER_L, A_L), F_HALF, "let go of the corner", ROT_L),
        Waypoint(1.2, _at(CORNER_L, A_L, back=0.060, up=0.030), F_HALF, "draw back off the bag", ROT_L),
        Waypoint(1.5, REST["left"], FINGER_OPEN, "stand clear", DOWN),
        Waypoint(12.0, REST["left"], FINGER_OPEN, "done", DOWN),
    ]
    return {"left": left, "right": right}
