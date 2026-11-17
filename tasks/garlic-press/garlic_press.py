"""On the station table, about 26 cm out: a two-handle garlic press (15.5 cm long, silver metal with black grips)
lying on the table, open. Its lower handle lies flat along the table, its black grip towards the left arm; at its
other end, a little right of the middle, is the chamber, an open-topped cup 34 mm long, 24 mm wide and 24 mm deep
with a perforated floor. The top handle is hinged just beyond the chamber (towards the right arm) and stands swung
back 25 degrees past upright, resting open on its stop, with the plunger on its underside. A peeled garlic clove
(26 x 16 x 14 mm) lies on the table in front of the right arm, beyond the press, its length pointing away from
the robot.

Crush the clove in the press: put it in the chamber lengthways (it fits only along the chamber's length), swing the
top handle back over and press it down until the plunger has squeezed the clove through the floor.

Run it:  inspect-robots run --embodiment yam_arena -E task=garlic_press.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import Hinge, body, box, cyl  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001

SILVER = (0.70, 0.71, 0.73)
GRATE = (0.42, 0.43, 0.45)
HOLE = (0.10, 0.10, 0.11)
GRIP = (0.08, 0.08, 0.09)  # black rubber
GARLIC = (0.96, 0.82, 0.40)  # a peeled clove; a deeper yellow than real, to show at 320 px from the top camera
PASTE = (0.93, 0.82, 0.45)
fittingkit.METALLIC[SILVER] = 0.35

# -- the press -------------------------------------------------------------------------------------------------------
# The lower body's origin is on the table under the hinge axis. In its own frame the press points along -y: the
# chamber next to the hinge, the lower handle running on from it. It lies turned half a turn (PRESS_YAW), so on the
# table the handles point towards the left arm (+y) and the hinge is on the right. Sizes [estimate] from a common
# cast press (Zyliss Susi 3: 15.5 cm long, 5.7 cm tall); the chamber holds one large clove.
PRESS_XY = np.array([0.26, -0.06])
PRESS_YAW = math.pi
HINGE_Z = 39.0 * MM  # the hinge axis (along x) above the table
CH_X = 12.0 * MM  # the chamber's inside: |x| < 12 mm ...
CH_Y0, CH_Y1 = -6.0 * MM, -40.0 * MM  # ... and y from -40 to -6 mm (34 mm long)
WALL = 3.0 * MM
FOOT_TOP = 8.0 * MM  # the chamber stands on two feet; the perforated floor is above them
GRATE_TOP = 10.0 * MM
RIM_Z = 34.0 * MM  # the chamber's walls end here (24 mm deep inside)
HANDLE_END = -155.0 * MM
LOWER_BAR_TOP = 12.0 * MM
BAR_W, BAR_T = 22.0 * MM, 8.0 * MM  # the top handle's bar: width, thickness

OPEN_DEG = 115.0  # the top handle at rest: swung back 25 degrees past upright onto its stop
OPEN = math.radians(OPEN_DEG)

# The plunger, on the top handle: its face 20 mm wide (x) and 26 mm long (y), from 10 to 36 mm out from the hinge
# and 24 mm below the hinge axis when the press is shut (5 mm above the floor).
PL_NEAR, PL_FAR = 10.0 * MM, 36.0 * MM
PL_MID = (PL_NEAR + PL_FAR) / 2
PL_DEPTH = 24.0 * MM


def _lower_shapes() -> tuple[list, list]:
    yc, ylen = (CH_Y0 + CH_Y1) / 2, CH_Y0 - CH_Y1
    wall_h = RIM_Z - FOOT_TOP
    solid = [
        # the chamber: two side walls, two end walls, the perforated floor, and the two feet it stands on
        box((CH_X + WALL / 2, yc, FOOT_TOP + wall_h / 2), (WALL, ylen + 2 * WALL, wall_h), SILVER),
        box((-CH_X - WALL / 2, yc, FOOT_TOP + wall_h / 2), (WALL, ylen + 2 * WALL, wall_h), SILVER),
        box((0.0, CH_Y0 + WALL / 2, FOOT_TOP + wall_h / 2), (2 * CH_X, WALL, wall_h), SILVER),
        box((0.0, CH_Y1 - WALL / 2, FOOT_TOP + wall_h / 2), (2 * CH_X, WALL, wall_h), SILVER),
        box((0.0, yc, (FOOT_TOP + GRATE_TOP) / 2), (2 * CH_X, ylen, GRATE_TOP - FOOT_TOP), GRATE),
        box((0.0, CH_Y1 - WALL + 3.0 * MM, FOOT_TOP / 2), (2 * (CH_X + WALL), 6.0 * MM, FOOT_TOP), SILVER),
        box((0.0, CH_Y0 + WALL - 3.0 * MM, FOOT_TOP / 2), (2 * (CH_X + WALL), 6.0 * MM, FOOT_TOP), SILVER),
        # the hinge's two cheeks, either side of the top handle
        box((CH_X + WALL / 2, 1.5 * MM, (FOOT_TOP + 45.0 * MM) / 2), (WALL, 9.0 * MM, 45.0 * MM - FOOT_TOP), SILVER),
        box((-CH_X - WALL / 2, 1.5 * MM, (FOOT_TOP + 45.0 * MM) / 2), (WALL, 9.0 * MM, 45.0 * MM - FOOT_TOP), SILVER),
        # the lower handle, flat on the table, with its black grip
        box((0.0, (CH_Y1 - WALL + HANDLE_END) / 2, LOWER_BAR_TOP / 2),
            (BAR_W, CH_Y1 - WALL - HANDLE_END, LOWER_BAR_TOP), SILVER),
    ]
    seen = [box((0.0, HANDLE_END + 35.0 * MM, LOWER_BAR_TOP / 2), (BAR_W + 2.0 * MM, 70.0 * MM, LOWER_BAR_TOP + 2.0 * MM),
                GRIP),
            cyl((0.0, 0.0, HINGE_Z), 3.0 * MM, 2 * (CH_X + WALL) + 2.0 * MM, SILVER, axis="X")]
    # the floor's holes, drawn as dark dots on its top
    for i in range(5):
        for j in range(3):
            seen.append(box((-8.0 * MM + 8.0 * MM * j, CH_Y1 + 5.0 * MM + 6.0 * MM * i, GRATE_TOP + 0.05 * MM),
                            (2.0 * MM, 2.0 * MM, 0.1 * MM), HOLE))
    return solid, seen


def _upper_shapes() -> tuple[list, list]:
    """The top handle in its own frame: origin on the hinge axis, axes the lower body's with the press shut."""
    bar_len = -HANDLE_END + 4.0 * MM
    solid = [box((0.0, -bar_len / 2 + 4.0 * MM, 0.0), (BAR_W, bar_len, BAR_T), SILVER),
             box((0.0, HANDLE_END + 35.0 * MM, 0.0), (BAR_W + 2.0 * MM, 70.0 * MM, BAR_T + 2.0 * MM), GRIP)]
    # WHY the plunger collides with nothing: the clove is a rigid part, so its squashing is the model's (GarlicPress),
    # which resists the plunger as the clove would and logs the crush.
    seen = [box((0.0, -PL_MID, -BAR_T / 2 - (PL_DEPTH - BAR_T / 2) / 2), (20.0 * MM, PL_FAR - PL_NEAR,
                                                                            PL_DEPTH - BAR_T / 2), SILVER)]
    return solid, seen


_LS, _LV = _lower_shapes()
PRESS_POS = np.array([*PRESS_XY, TOP])
R_PRESS = np.array([[math.cos(PRESS_YAW), -math.sin(PRESS_YAW), 0.0], [math.sin(PRESS_YAW), math.cos(PRESS_YAW), 0.0],
                    [0.0, 0.0, 1.0]])
PRESS = body("press", pos=tuple(PRESS_POS), mass=0.13, yaw=PRESS_YAW, com=(0.0, -0.07, 0.008), friction=1.0, shapes=_LS + _LV,
             visual_only=tuple(range(len(_LS), len(_LS) + len(_LV))))

_US, _UV = _upper_shapes()


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# spawned open: turned -OPEN about the press's x (the bar from along its -y to up and past upright)
TOP_QUAT = fittingkit.matrix_to_quat_rel(R_PRESS @ rot_x(-OPEN))
PRESS_TOP = body("press_top", pos=tuple(PRESS_POS + np.array([0.0, 0.0, HINGE_Z])), mass=0.07,
                 com=(0.0, -0.068, -0.002), friction=1.0, quat=TOP_QUAT, shapes=_US + _UV,
                 visual_only=tuple(range(len(_US), len(_US) + len(_UV))))
# The hinge's zero is where the top handle is spawned (open, on its stop); shutting it turns it +OPEN about x.
fittingkit.add_hinge(Hinge("press", "press_top", anchor=(0.0, 0.0, HINGE_Z), axis=(1.0, 0.0, 0.0), damping=0.01,
                           lower=0.0, upper=OPEN))

# -- the clove -------------------------------------------------------------------------------------------------------
CLOVE_L, CLOVE_W, CLOVE_H = 26.0 * MM, 16.0 * MM, 14.0 * MM  # a large peeled clove [estimate]
# WHY here (2026-11-17, setup screenshot): at (0.19, -0.20) it lay in the open handle's shadow and vanished
CLOVE_XY = np.array([0.32, -0.21])
# WHY a plain block (2026-11-17, first sim run): with a flat 7 mm base and a rounded back the fingers held only the
# base's edges and the clove slipped out on the lift.
# WHY 30 g, not a real clove's 7 g (2026-11-17, sim runs garlic-gp5 to gp8, fingertip positions logged): at 7 g the
# closing fingers pass straight through it (PhysX, a light body squeezed by the stiff gripper); at 15 g they sink
# 5 mm into it; at 30 g they stop 0.4 mm in and it lifts cleanly.
CLOVE_MASS = 0.03
_CS = [box((0.0, 0.0, 0.0), (CLOVE_L, CLOVE_W, CLOVE_H), GARLIC)]
CLOVE = body("clove", pos=(*CLOVE_XY, TOP + CLOVE_H / 2), mass=CLOVE_MASS, friction=1.0, shapes=_CS,
             contact_offset=0.01)

# The squeezed garlic: drawn under the chamber's floor once the clove is crushed, parked out of sight until then.
_PS = [box((0.0, 0.0, 0.0), (18.0 * MM, 22.0 * MM, 5.0 * MM), PASTE),
       box((3.0 * MM, -4.0 * MM, 1.5 * MM), (10.0 * MM, 12.0 * MM, 4.0 * MM), PASTE)]
PASTE_LOCAL = np.array([0.0, (CH_Y0 + CH_Y1) / 2, 3.0 * MM])  # in the press's frame, between its feet
PARK = {"paste": np.array([0.26, 0.0, TOP - 0.20]), "clove": np.array([0.30, 0.0, TOP - 0.20])}  # under the table
PASTE_BODY = body("paste", pos=tuple(PARK["paste"]), mass=0.005, shapes=_PS, visual_only=(0, 1),
                  inertia=(1e-7, 1e-7, 1e-7))

OBJECTS = [PRESS, PRESS_TOP, CLOVE, PASTE_BODY]


# -- the crush -------------------------------------------------------------------------------------------------------
F_CRUSH = 10.0  # N, pressed down at the top handle's grip point, to squeeze the clove [estimate]
GRIP_L = 130.0 * MM  # the grip point's distance from the hinge
T_CRUSH = F_CRUSH * GRIP_L  # N m about the hinge
K_CLOVE = 40.0  # N m/rad: the clove's elastic stiffness at the hinge, before it yields
C_CLOVE = 0.02  # N m s/rad
CRUSHED_GAP = 6.0 * MM  # crushed once the plunger's face is this close to the floor
MASH_DEG = 30.0  # the handle shut past this with the clove on the rim mashes it there


def plunger_face_z(theta: float) -> float:
    """Height above the table of the middle of the plunger's face, the top handle open ``theta`` (rad)."""
    return HINGE_Z + PL_MID * math.sin(theta) - PL_DEPTH * math.cos(theta)


def open_angle(rot_lower: np.ndarray, rot_upper: np.ndarray) -> float:
    """How far the top handle is open (rad; 0 shut), from the two bodies' rotations."""
    rel = rot_lower.T @ rot_upper
    return -math.atan2(rel[2, 1], rel[1, 1])


THETA_CRUSH = None  # filled in below: the open angle at which the plunger reaches CRUSHED_GAP


def _solve_theta(z: float) -> float:
    lo, hi = 0.0, math.pi / 2  # plunger_face_z rises with theta on [0, pi/2]
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if plunger_face_z(mid) < z else (lo, mid)
    return (lo + hi) / 2


THETA_CRUSH = _solve_theta(GRATE_TOP + CRUSHED_GAP)
THETA_FLAT_CONTACT = _solve_theta(GRATE_TOP + CLOVE_H)  # a clove lying flat in the chamber meets the plunger here


def clove_in_press(clove_pos, clove_rot, press_pos, press_rot) -> tuple[np.ndarray, float]:
    """The clove's centre in the press's frame, and its top (highest corner) above the press's base."""
    lo, hi = CLOVE.bounds_local()
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    world = corners @ np.asarray(clove_rot).T + np.asarray(clove_pos)
    local = (world - np.asarray(press_pos)) @ np.asarray(press_rot)
    centre = np.asarray(press_rot).T @ (np.asarray(clove_pos) - np.asarray(press_pos))
    return centre, float(local[:, 2].max())


def seated(centre: np.ndarray) -> bool:
    """The clove's centre is inside the chamber's walls and below its rim: it lies in the chamber."""
    x, y, z = centre
    return bool(abs(x) < CH_X and CH_Y1 < y < CH_Y0 and z < RIM_Z)


def on_head(centre: np.ndarray) -> bool:
    """The clove's centre is over the chamber's outside footprint, at or above its rim (on the rim, not in it)."""
    x, y, z = centre
    return bool(abs(x) < CH_X + WALL + 5.0 * MM and CH_Y1 - WALL - 5.0 * MM < y < 5.0 * MM and z >= RIM_Z - 2.0 * MM)


class GarlicPress:
    """Runs before every physics substep. Reads the top handle's angle and where the clove is; once the plunger meets
    a clove lying in the chamber it pushes back like the clove (stiff, then yielding at T_CRUSH: pressing harder only
    squashes it further), and when the plunger has squeezed it to CRUSHED_GAP it logs the crush, takes the clove away
    and draws the squeezed garlic under the floor. A clove on the rim when the handle comes down is logged mashed."""

    bodies = ["press", "press_top", "clove", "paste"]

    def __init__(self):
        self.origin = np.zeros(3)
        self.dt = 1.0 / 240.0
        import os
        self.log = os.environ.get("GARLIC_LOG")
        self.debug = os.environ.get("GARLIC_DEBUG")
        self.clear()

    def clear(self) -> None:
        self._n = 0
        self.theta = OPEN
        self.crushed = False
        self.contact = False  # the plunger has come down onto the seated clove
        self.mashed = False
        self.seated = False
        self.yield_at = None  # the open angle down to which the clove has been squashed
        self.torque = 0.0
        self.poses = {}
        self._park()

    def _park(self) -> None:
        q = (0.0, 0.0, 0.0, 1.0)
        self.poses = {"paste": (PARK["paste"] + self.origin, q)}
        if self.crushed:
            self.poses["clove"] = (PARK["clove"] + self.origin, q)

    def _note(self, what: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(f"{self._n / 240.0:7.2f}s {what} (open {math.degrees(self.theta):.1f} deg)\n")

    def _debug_tips(self, frames: dict) -> None:
        """With GARLIC_DEBUG set: every 0.25 s, where the right fingertips are against the clove (for tuning)."""
        environment = fittingkit._ENV["env"]
        cl = frames.get("clove")
        if environment is None or cl is None:
            return
        from yam_arena.analysis import world_hulls
        from yam_arena.runtime import link_poses

        scene = environment.unwrapped.scene
        robot = scene["right_robot"]
        poses = link_poses(robot)
        c = cl.pos - self.origin
        parts = []
        for link in ("tip_left", "tip_right"):
            v = np.vstack([h.vertices for _, h in world_hulls(poses, [link])]) - self.origin
            low = v[v[:, 2] < v[:, 2].min() + 0.015]
            parts.append(f"{link} x {low[:, 0].min():.4f}..{low[:, 0].max():.4f} y {low[:, 1].min():.4f}.."
                         f"{low[:, 1].max():.4f} z {low[:, 2].min():.4f}")
        jp = fittingkit._np(robot.data.joint_pos)[0]
        with open(self.debug, "a") as fh:
            fh.write(f"{self._n / 240.0:6.2f}s fingers {np.round(jp[6:], 4).tolist()} | " + " | ".join(parts)
                     + f" | clove {c[0]:.4f} {c[1]:.4f} {c[2]:.4f}\n")

    def update(self, frames: dict, acc: dict) -> None:
        self._n += 1
        if self.debug and self._n % 60 == 0:
            self._debug_tips(frames)
        lo, up, cl = frames.get("press"), frames.get("press_top"), frames.get("clove")
        if lo is None or up is None or cl is None or not all(np.all(np.isfinite(f.pos)) for f in (lo, up, cl)):
            return
        self.theta = theta = open_angle(lo.rot, up.rot)
        self.torque = 0.0
        if not self.crushed:
            centre, top = clove_in_press(cl.pos, cl.rot, lo.pos, lo.rot)
            self.seated = seated(centre)
            if self.seated:
                if plunger_face_z(theta) <= top:
                    if not self.contact:
                        self._note("plunger on the clove")
                    self.contact = True
                    if self.yield_at is None:
                        self.yield_at = theta
                if self.yield_at is not None and theta < self.yield_at:
                    elastic = K_CLOVE * (self.yield_at - theta)
                    if elastic > T_CRUSH:  # it yields: squashed further, it pushes back no harder
                        self.yield_at = theta + T_CRUSH / K_CLOVE
                        elastic = T_CRUSH
                    spin = float((up.spin - lo.spin) @ lo.rot[:, 0])  # + while shutting
                    self.torque = max(0.0, elastic + C_CLOVE * spin)
                if plunger_face_z(theta) <= GRATE_TOP + CRUSHED_GAP:
                    self.crushed = True
                    self.torque = 0.0
                    self._note("crushed")
            elif on_head(centre) and theta < math.radians(MASH_DEG) and not self.mashed:
                self.mashed = True
                self._note("mashed on the rim")
        if self.torque:
            axis = lo.rot[:, 0]
            fittingkit._torque(acc, "press_top", -self.torque * axis)
            fittingkit._torque(acc, "press", self.torque * axis)
        self._park()
        if self.crushed:
            q = fittingkit.matrix_to_quat_rel(lo.rot)
            self.poses["paste"] = (lo.world(PASTE_LOCAL), q)


MODEL = fittingkit.add_model(GarlicPress())


# -- scoring ---------------------------------------------------------------------------------------------------------
MAX_TILT_DEG = 20.0


def press_upright(objects) -> bool:
    """The press's lower half lies on the table, tilted less than 20 degrees."""
    p = objects["press"]
    return bool(p.tilt_deg() < MAX_TILT_DEG and abs(float(p.pos[2]) - TOP) < 0.02)


def success(objects) -> bool:
    """The clove crushed by the plunger in the chamber, the press still lying on the table. Must hold for 1 s."""
    return MODEL.crushed and press_upright(objects)


def score(objects) -> float:
    """1.0 crushed (success); 0.5 the clove lying in the chamber with the plunger down on it, not crushed; 0.25 the
    clove lying in the chamber; 0 otherwise, and 0 if the clove was mashed on the rim or the press was knocked over."""
    if MODEL.mashed or not press_upright(objects):
        return 0.0
    if MODEL.crushed:
        return 1.0
    if MODEL.seated:
        return 0.5 if MODEL.contact else 0.25
    return 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# The right arm picks the clove up across its width, carries it round the open handle, turns it a quarter turn over
# the chamber and drops it in from 4 mm above the rim. Then, fingers closed, it goes round the open handle's top end
# (sideways: the handle is 22 mm wide, and a straight-down grip can't reach over its 18 cm top) and pushes it back
# past upright, so it falls onto the clove. The left arm then presses the handle's
# grip down past shut (the arm's compliance turns the aim into force).
# WHY this split (plan_check.py, the YAM's gripper hulls): the open handle leans out over the hinge's side, so the
# hand that tips it must come from above its end on that side (the right); the left hand reaches the grip.
TIP = PAD_TIP_PAST_TCP  # the fingertips are this far below the TCP
YAW_Q = math.pi / 2  # the grip turned a quarter turn: fingers closing along x (the right wrist reaches +90, not -90)


def press_point(local) -> np.ndarray:
    """A point in the lower body's frame (at its start pose), in the world."""
    return PRESS_POS + R_PRESS @ np.asarray(local, float)


def handle_point(along: float, theta: float, above: float = BAR_T / 2) -> np.ndarray:
    """The point ``along`` the top handle from the hinge, ``above`` its centre line on its top face, with the handle
    open ``theta``, in the world."""
    y = -along * math.cos(theta) + above * math.sin(theta)
    z = HINGE_Z + along * math.sin(theta) + above * math.cos(theta)
    return press_point((0.0, y, z))


HANDLE_LEN = -HANDLE_END
CHAMBER = press_point((0.0, -27.0 * MM, 0.0))  # the clove dropped 4 mm off the chamber's middle, away from the open
# handle (the chamber takes the clove's centre from -19 to -27 mm)
GRASP_TIPS = 5.0 * MM  # the fingertips this far above the table when grasping the clove
DROP_GAP = 4.0 * MM  # the clove's underside this far above the rim when let go
ASIDE_X = 0.20  # the right hand goes round the open handle here, 4.9 cm short of it in x
TIP_HEIGHT = 0.165  # the right fingertips' height while tipping the handle back (its end 18 cm up, 19 when upright)
TIP_BEYOND = 25.0 * MM  # they come down this far beyond the open handle's end
TIP_PAST = 30.0 * MM  # and push it to this far past the hinge
PUSH_AIM = 30.0 * MM  # the left fingertips aimed this far below the shut handle's top
FINGER_S = 1.6  # a full open or close takes at least this long: the step guard allows 5% of the travel per step


def demo():
    o, c = FINGER_OPEN, FINGER_CLOSED
    cx, cy = CLOVE_XY
    grasp_z = TOP + GRASP_TIPS + TIP
    drop_z = TOP + RIM_Z + DROP_GAP + GRASP_TIPS + TIP  # the clove's underside GRASP_TIPS below the fingertips
    travel = TOP + 0.12
    tip_z = TOP + TIP_HEIGHT + TIP
    chx, chy = CHAMBER[0], CHAMBER[1]
    end = handle_point(HANDLE_LEN, OPEN, 0.0)  # the open handle's top end
    beyond = end[1] - TIP_BEYOND  # it leans out to -y
    past = press_point((0.0, -TIP_PAST, 0.0))[1]
    right = [
        Waypoint(1.6, (cx, cy, travel), o, "over the clove"),
        Waypoint(1.2, (cx, cy, grasp_z), o, "down to the clove"),
        Waypoint(FINGER_S, (cx, cy, grasp_z), c, "grip it across its width"),
        Waypoint(1.0, (cx, cy, travel), c, "lift"),
        Waypoint(1.2, (ASIDE_X, cy, travel), c, "back towards the arms"),
        Waypoint(1.4, (ASIDE_X, chy, travel), c, "along, clear of the open handle"),
        Waypoint(1.4, (chx, chy, travel), c, "over the chamber, turned a quarter turn", YAW_Q),
        Waypoint(1.2, (chx, chy, drop_z), c, "down to the rim", YAW_Q),
        Waypoint(FINGER_S, (chx, chy, drop_z), o, "let it drop in", YAW_Q),
        Waypoint(FINGER_S, (chx, chy, tip_z), c, "up, closing", YAW_Q),
        Waypoint(1.2, (ASIDE_X, chy, tip_z), c, "back, beside the open handle"),
        Waypoint(1.4, (ASIDE_X, beyond, tip_z), c, "round past its end"),
        Waypoint(1.2, (PRESS_XY[0], beyond, tip_z), c, "in beyond its end"),
        Waypoint(1.6, (PRESS_XY[0], past, tip_z), c, "tip it back over"),
        Waypoint(1.0, (ASIDE_X, past, tip_z), c, "back and clear"),
    ]
    t_tipped = sum(w.duration for w in right)
    right.append(Waypoint(FINGER_S, (ASIDE_X, -0.20, travel + 0.04), o, "out of the way"))
    hover = (0.22, 0.20, TOP + 0.16)  # a straight-down grip, out of the right hand's way
    above = handle_point(GRIP_L, THETA_FLAT_CONTACT) + np.array([0.0, 0.0, 0.03 + TIP])
    shut = handle_point(GRIP_L, 0.0) + np.array([0.0, 0.0, TIP])
    left = [
        Waypoint(2.0, hover, o, "to the side"),
        Waypoint(t_tipped - 2.0, hover, o, "wait"),  # until the handle is down and the right hand up
        Waypoint(1.8, tuple(above), c, "over the handle's grip"),
        Waypoint(1.0, tuple(above - np.array([0.0, 0.0, 0.03])), c, "onto it"),
        Waypoint(2.0, tuple(shut - np.array([0.0, 0.0, PUSH_AIM])), c, "press it down"),
        Waypoint(1.5, tuple(shut - np.array([0.0, 0.0, PUSH_AIM])), c, "hold"),
        Waypoint(1.0, tuple(shut + np.array([0.0, 0.0, 0.06])), c, "let go"),
        Waypoint(1.5, tuple(shut + np.array([0.0, 0.0, 0.06])), c, "wait"),
    ]
    right.append(Waypoint(sum(w.duration for w in left) - sum(w.duration for w in right), right[-1].pos, o, "wait"))
    return {"left": left, "right": right}
