"""lancetkit: a lancing device with its end cap and arming barrel, a twist-top lancet and a dish, for yam_arena task
files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from lancetkit import Barrel, Cap, Device, Dish, Lancet, Tab, on_reset, on_step

Every part is a loose rigid body whose length runs along its own z:

- :class:`Device` is the lancing device's body. ``pos`` is the tip of its nose (the mouth of the lancet holder); its z
  runs back toward the arming barrel. The holder is a socket in the nose.
- :class:`Barrel` is the arming barrel on the body's rear end. It slides back on the body against a return spring.
- :class:`Cap` is the end cap. ``pos`` is the middle of its open mouth; its z points out of the mouth.
- :class:`Lancet` is a lancet without its protective tab. ``pos`` is its rounded insertion end; its z runs to the needle.
- :class:`Tab` is the lancet's twist-off protective tab. ``pos`` is where it joins the lancet.
- :class:`Dish` is a loose open dish (``pos`` the middle of its base, ``size`` its inside diameter twice and its height).

The kit joins the parts with joints, authored afresh at every reset from where the parts are:

- ``neck``: the tab to the lancet, a moulded neck. About the lancet's axis it is a spring that yields at
  ``NECK_TORQUE``; it is released (:func:`release`) once the tab has been twisted ``NECK_TWIST``. Pulled, it lets go at
  ``NECK_FORCE``.
- ``click``: the cap to the body, a snap that lets go at ``CAP_HOLD``. :func:`snap` makes it again when a cap is pushed
  fully home.
- ``seat``: the lancet to the body, the holder's grip, which lets go at ``SEAT_HOLD``. :func:`snap` makes it when a lancet
  is pushed fully home in the socket.
- ``slide``: the barrel on the body, a slide with stops and a return spring.

``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``, passive breakable joints re-armed on reset). Pure
Python at import time; Isaac Sim is reached only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Barrel", "Cap", "Device", "Dish", "KitState", "Lancet", "Tab", "apart", "barrel_pull", "cap_fit", "cap_on",
           "env", "finger_points", "gripper_points", "install", "joined", "needle", "on_reset", "on_step", "place",
           "release", "seat_fit", "seated", "segment_distance", "snap", "to_local", "turn_x", "twist", "write_pose"]

# -- the device's body (m): origin at the tip of its nose, z running back toward the barrel --------------------------------
NOSE_L, NOSE_R = 0.012, 0.0045           # the nose that the cap fits over
EDGE = 0.0010                            # the radius of its rounded front edge
MOUTH = 0.0008                           # the radius of the socket's rounded mouth
SOCKET_R, SOCKET_DEPTH = 0.0024, 0.009   # the lancet holder: a round socket in the nose
BODY_R, BODY_END = 0.0085, 0.090         # the body's radius; where it ends and the barrel begins
FLAT = 0.008                             # the width of the flat it lies on
BODY_MASS = 0.022
# -- the arming barrel: origin on its front face, which sits on the body's rear end -------------------------------------
BARREL_L, BARREL_MASS = 0.022, 0.004
COCK_TRAVEL, COCK_AT = 0.008, 0.0065     # how far it pulls back; the device is cocked once it has been pulled this far
COCK_K, COCK_DAMP = 200.0, 0.6           # the return spring, N/m (1.6 N at full travel), and its damping, N s/m
# -- the end cap: origin at the middle of its open mouth, z pointing out of the mouth ----------------------------------------
CAP_L, CAP_R = 0.034, 0.0085
CAP_LEAD, CAP_LEAD_L = 0.0070, 0.003     # the mouth's lead-in: its radius and depth
CAP_BORE, CAP_CHAMBER = 0.0056, 0.0042   # the bore that fits the nose; the chamber beyond it, round the lancet
CAP_MASS = 0.003
CAP_HOLD = 2.0                           # N: the cap's snap lets go at this pull
CAP_HOME = 0.001                         # the cap snaps on within this of fully home
# -- the lancet: origin at its rounded insertion end, z running to the needle ---------------------------------------------
LANCET_R, LANCET_L = 0.0020, 0.0245
NEEDLE_L = 0.003
LANCET_MASS = 0.0004
SEAT_HOLD = 2.5                          # N: the holder's grip on a seated lancet lets go at this pull
SEAT_HOME = 0.001                        # the holder grips a lancet pushed to within this of the socket's floor
# -- the protective tab: origin where it joins the lancet, z as the lancet's ------------------------------------------------
TAB_R, TAB_L, TAB_MASS = 0.0032, 0.008, 0.0002
# The moulded neck between tab and lancet. Estimates, not measurements: it springs back from a small twist, yields at a
# light finger torque and shears once the tab has been twisted an eighth of a turn; it stands a pull several times the
# holder's grip, so a tab that is pulled brings the lancet out with it.
NECK_TORQUE = 0.004                      # N m: the torque at which the neck yields and the tab turns on it
NECK_YIELD = 5.0                         # deg: below that torque it is a spring, reaching the torque at this twist
NECK_TWIST = math.radians(45.0)          # the neck shears once the tab has been twisted this far
NECK_DRAG = 0.00002                      # N m s/deg
NECK_FORCE, NECK_BEND = 8.0, 0.05        # N, N m: the pull and the bending it stands
# Two joined parts count as one piece within these. A joint between parts of a fraction of a gram gives a little under
# load, so the limits sit well outside that give; a tab that has come off passes them within a control step.
PART_SHIFT, PART_ANGLE = 0.004, math.radians(20.0)
ALIGNED = math.radians(10.0)             # a lancet or a cap this close to in line with the body can be home

FRICTION = (0.6, 0.5)
NOSE_FRICTION = (0.2, 0.15)              # the nose and the holder's mouth, against the cap and the lancet
PALM_BACK = 0.065                       # a gripper's palm, this far back from its grasp point
FINGER_THICK = 0.012                    # a finger's outer face, this far out from its pad


class _Part(Box):
    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class Device(_Part):
    """The lancing device's body. ``barrel``, ``cap``: the names of its :class:`Barrel` and :class:`Cap`."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = BODY_MASS
    barrel: str = "barrel"
    cap: str = "cap"


@dataclass(frozen=True)
class Barrel(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = BARREL_MASS


@dataclass(frozen=True)
class Cap(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = CAP_MASS


@dataclass(frozen=True)
class Lancet(_Part):
    """A lancet without its tab. ``tab`` names the :class:`Tab` moulded onto it."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = LANCET_MASS
    tab: str = "tab"


@dataclass(frozen=True)
class Tab(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = TAB_MASS


@dataclass(frozen=True)
class Dish(_Part):
    """A loose open dish. ``size``: inside diameter, inside diameter, height."""
    mass: float | None = 0.03
    wall: float = 0.002
    friction: tuple = (0.6, 0.5)


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``pos`` is its own origin (see each class)."""

    joints: dict = field(default_factory=dict)


# -- geometry (pure) -------------------------------------------------------------------------------------------------

def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat(rot) -> tuple:
    """The quaternion (xyzw) of a rotation matrix."""
    m = np.asarray(rot, float)
    t = float(np.trace(m))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i], q[j], q[k], q[3] = s / 4, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s, (m[k, j] - m[j, k]) / s
    return tuple(q)


def turn_x(angle: float) -> tuple:
    """The quaternion (xyzw) of a turn by ``angle`` about x."""
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def place(pos, quat, local) -> np.ndarray:
    """The world point of ``local`` in a part at ``pos``, ``quat``."""
    return np.asarray(pos, float) + _rotation(quat) @ np.asarray(local, float)


def to_local(point, pos, quat) -> np.ndarray:
    """``point`` (world) in the frame of a body at ``pos``, ``quat``."""
    return _rotation(quat).T @ (np.asarray(point, float) - np.asarray(pos, float))


def segment_distance(p, a, b) -> float:
    """The distance from point ``p`` to the straight piece from ``a`` to ``b``."""
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def apart(pos0, quat0, anchor0, pos1, quat1) -> tuple:
    """How far part 1's origin is from ``anchor0`` in part 0 (m), and the angle between the two parts' axes (rad)."""
    r0, r1 = _rotation(quat0), _rotation(quat1)
    shift = float(np.linalg.norm(np.asarray(pos0, float) + r0 @ np.asarray(anchor0, float) - np.asarray(pos1, float)))
    angle = math.acos(float(np.clip(r0[:, 2] @ r1[:, 2], -1.0, 1.0)))
    return shift, angle


def twist(lancet_quat, tab_quat) -> float:
    """How far the tab has been turned on the lancet about the lancet's axis (rad, signed)."""
    r = _rotation(lancet_quat).T @ _rotation(tab_quat)
    return math.atan2(float(r[1, 0]), float(r[0, 0]))


def joined(lancet_pos, lancet_quat, tab_pos, tab_quat) -> bool:
    """The tab is still on its lancet."""
    shift, angle = apart(lancet_pos, lancet_quat, (0.0, 0.0, LANCET_L), tab_pos, tab_quat)
    return shift <= PART_SHIFT and angle <= PART_ANGLE


def _axial(dev_pos, dev_quat, point) -> tuple:
    """A point in the device's frame: how far back from the nose tip it is along the axis, and how far off the axis."""
    p = to_local(point, dev_pos, dev_quat)
    return float(p[2]), float(math.hypot(p[0], p[1]))


def seat_fit(dev_pos, dev_quat, lancet_pos, lancet_quat) -> dict:
    """How a lancet sits in the holder. ``depth``: how far its insertion end is inside the nose (m, negative outside);
    ``off``: that end's distance from the device's axis; ``angle``: how far the lancet is from in line, needle
    outward (rad)."""
    depth, off = _axial(dev_pos, dev_quat, lancet_pos)
    cos = -float(_rotation(dev_quat)[:, 2] @ _rotation(lancet_quat)[:, 2])
    return {"depth": depth, "off": off, "angle": math.acos(max(-1.0, min(1.0, cos)))}


def seated(fit: dict) -> bool:
    """The lancet is home in the holder: its end within SEAT_HOME of the socket's floor, inside the socket, in line."""
    return bool(fit["depth"] >= SOCKET_DEPTH - SEAT_HOME and fit["off"] <= SOCKET_R and fit["angle"] <= ALIGNED)


def cap_fit(dev_pos, dev_quat, cap_pos, cap_quat) -> dict:
    """How a cap sits on the nose. ``gap``: how far its mouth is from fully home against the body's shoulder (m);
    ``off``: its mouth's distance from the device's axis; ``angle``: how far it is from in line (rad)."""
    z, off = _axial(dev_pos, dev_quat, cap_pos)
    cos = float(_rotation(dev_quat)[:, 2] @ _rotation(cap_quat)[:, 2])
    return {"gap": NOSE_L - z, "off": off, "angle": math.acos(max(-1.0, min(1.0, cos)))}


def cap_on(fit: dict) -> bool:
    """The cap is fully on: its mouth within CAP_HOME of the body's shoulder, round the nose, in line."""
    return bool(abs(fit["gap"]) <= CAP_HOME and fit["off"] <= CAP_BORE - NOSE_R + 0.0005 and fit["angle"] <= ALIGNED)


def barrel_pull(dev_pos, dev_quat, barrel_pos) -> float:
    """How far the arming barrel is pulled back from its rest on the body's rear end (m)."""
    return _axial(dev_pos, dev_quat, barrel_pos)[0] - BODY_END


def needle(lancet_pos, lancet_quat) -> tuple:
    """The needle's base and tip."""
    return place(lancet_pos, lancet_quat, (0.0, 0.0, LANCET_L)), place(lancet_pos, lancet_quat, (0.0, 0.0, LANCET_L + NEEDLE_L))


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


# -- authoring helpers -----------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.5, metal=0.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def glass(self, color, ior=1.49, thin=True) -> str:
        """A see-through look (the real-time renderer ignores a plain surface's opacity)."""
        key = ("glass", tuple(color), ior)
        if key not in self.looks:
            path = f"{self.path}/Looks/g{len(self.looks)}"
            try:
                vis = self.sim_utils.GlassMdlCfg(glass_color=tuple(float(c) for c in color), frosting_roughness=0.15,
                                                 thin_walled=thin, glass_ior=ior)
                vis.func(path, vis)
            except Exception as err:                                          # noqa: BLE001
                print(f"[lancetkit] glass look unavailable ({err!r}); drawn as a plain surface", file=sys.stderr, flush=True)
                from yam_arena.station import linear_rgb

                vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.3)
                vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df, combine="average") -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df, combine)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, mass, lin_damp=0.05, ang_damp=0.1, iterations=None):
        from pxr import PhysxSchema, UsdPhysics

        prim = self.UsdGeom.Xform.Define(self.stage, path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        if iterations:
            rb.CreateSolverPositionIterationCountAttr(int(iterations))
            rb.CreateSolverVelocityIterationCountAttr(4)
        return prim

    def collide(self, prim, mat, offset=0.0008):
        from isaaclab.sim.utils import bind_physics_material
        from pxr import PhysxSchema, UsdPhysics

        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)

    def paint(self, prim, look):
        from isaaclab.sim.utils import bind_visual_material

        bind_visual_material(str(prim.GetPath()), look, stage=self.stage)

    def _finish(self, prim, look, mat, offset=0.0008):
        if look:
            self.paint(prim, look)
        else:
            self.UsdGeom.Imageable(prim).MakeInvisible()
        if mat:
            self.collide(prim, mat, offset)
        return prim

    def box(self, path, centre, size, look, mat=None, yaw=0.0, offset=0.0008):
        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if yaw:
            cube.AddOrientOp().Set(self.Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        return self._finish(cube.GetPrim(), look, mat, offset)

    def cyl(self, path, r, z0, z1, look, mat=None, offset=0.0008):
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(z1 - z0))
        c.AddTranslateOp().Set(self.Gf.Vec3d(0.0, 0.0, float(z0 + z1) / 2))
        return self._finish(c.GetPrim(), look, mat, offset)

    def ball(self, path, r, centre, look, mat=None, offset=0.0008):
        s = self.UsdGeom.Sphere.Define(self.stage, path)
        s.CreateRadiusAttr(float(r))
        s.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        return self._finish(s.GetPrim(), look, mat, offset)

    def ring(self, stem, r_in, wall, z0, z1, look, mat, pieces=16):
        """A round wall from flat pieces, each turned to face the axis."""
        width = 2 * (r_in + wall) * math.tan(math.pi / pieces) + 0.0002
        for k in range(pieces):
            t = 2 * math.pi * k / pieces
            self.box(f"{stem}_{k}", ((r_in + wall / 2) * math.cos(t), (r_in + wall / 2) * math.sin(t), (z0 + z1) / 2),
                     (wall, width, z1 - z0), look, mat, yaw=t)


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned


STEEL, GREY, BLUE, WHITE, CLEAR, YELLOW = ((0.75, 0.76, 0.78), (0.32, 0.34, 0.38), (0.16, 0.38, 0.72), (0.95, 0.95, 0.93),
                                           (0.85, 0.90, 0.95), (0.95, 0.80, 0.15))


def _start(cfg, prim_path, translation, orientation, iterations=None):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass), iterations=iterations)
    return spec, a, a.mat("part", *FRICTION)


def _spawn_device(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The body: a nose with the holder's socket in it, a round body with a flat to lie on, a trigger button."""
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    blue, grey = a.look(BLUE, rough=0.5), a.look(GREY, rough=0.5)
    slick = a.mat("nose", *NOSE_FRICTION, combine="min")                            # smooth moulded plastic: it rules
    # The nose's wall stays inside its rounded front edge, a close ring of beads that stands proud of it all round:
    # whatever is pushed over the nose rides up a curved surface and never meets a square face.
    a.ring(f"{prim_path}/nose", SOCKET_R, NOSE_R - 0.0003 - SOCKET_R, EDGE, NOSE_L, grey, slick, pieces=16)
    a.cyl(f"{prim_path}/floor", SOCKET_R, SOCKET_DEPTH, NOSE_L, grey, slick)            # the socket's floor
    for k in range(24):
        t = 2 * math.pi * k / 24
        a.ball(f"{prim_path}/edge_{k}", EDGE, ((NOSE_R + 0.0001 - EDGE) * math.cos(t), (NOSE_R + 0.0001 - EDGE) * math.sin(t), EDGE),
               grey, slick)
    for k in range(16):                                                              # ... and the socket's rounded mouth
        t = 2 * math.pi * (k + 0.5) / 16
        a.ball(f"{prim_path}/mouth_{k}", MOUTH, ((SOCKET_R + MOUTH) * math.cos(t), (SOCKET_R + MOUTH) * math.sin(t), MOUTH), grey, slick)
    a.cyl(f"{prim_path}/body", BODY_R, NOSE_L, BODY_END, blue, m)
    a.box(f"{prim_path}/flat", (0.0, BODY_R - 0.001, (NOSE_L + BODY_END) / 2), (FLAT, 0.002, BODY_END - NOSE_L - 0.006), blue, m)
    a.box(f"{prim_path}/button", (0.0, -BODY_R, 0.034), (0.006, 0.003, 0.012), a.look(YELLOW, rough=0.5), m)
    a.cyl(f"{prim_path}/dial", BODY_R + 0.0003, 0.070, 0.076, grey)                 # the depth dial, drawn
    return a.root


def _spawn_barrel(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    grey = a.look(GREY, rough=0.6)
    a.cyl(f"{prim_path}/sleeve", BODY_R, 0.0, BARREL_L, grey, m)
    a.box(f"{prim_path}/flat", (0.0, BODY_R - 0.001, BARREL_L / 2), (FLAT, 0.002, BARREL_L - 0.004), grey, m)
    return a.root


def _spawn_cap(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A sleeve: a lead-in at the mouth, a bore that fits the nose, a chamber round the lancet and a closed end."""
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    clear = a.look(CLEAR, rough=0.3)
    a.ring(f"{prim_path}/lead", CAP_LEAD, CAP_R - CAP_LEAD, -CAP_LEAD_L, 0.0, clear, m, pieces=12)
    a.ring(f"{prim_path}/bore", CAP_BORE, CAP_R - CAP_BORE, -NOSE_L, -CAP_LEAD_L, clear, m, pieces=12)
    a.ring(f"{prim_path}/chamber", CAP_CHAMBER, CAP_R - CAP_CHAMBER, -CAP_L + 0.002, -NOSE_L, clear, m, pieces=12)
    a.cyl(f"{prim_path}/end", CAP_R, -CAP_L, -CAP_L + 0.002, clear, m)
    return a.root


def _spawn_lancet(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A round plastic body with a rounded insertion end, and the needle drawn at the other end (it has no collider)."""
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    white = a.look(WHITE, rough=0.5)
    a.ball(f"{prim_path}/end", LANCET_R, (0.0, 0.0, LANCET_R), white, m)
    a.cyl(f"{prim_path}/shaft", LANCET_R, LANCET_R, LANCET_L, white, m)
    a.cyl(f"{prim_path}/needle", 0.0004, LANCET_L, LANCET_L + NEEDLE_L, a.look(STEEL, rough=0.2, metal=1.0))
    return a.root


def _spawn_tab(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.cyl(f"{prim_path}/disc", TAB_R, 0.0005, TAB_L, a.look(WHITE, rough=0.5), m)
    return a.root


# -- joints ------------------------------------------------------------------------------------------------------------
# name -> (body 0's kind, body 1's kind, the anchor in body 0, a slide or a fixed joint, break force, break torque)
_JOINT_KINDS = {"neck": ("lancet", "tab", (0.0, 0.0, LANCET_L), "twist", NECK_FORCE, NECK_BEND),
                "click": ("device", "cap", (0.0, 0.0, NOSE_L), "fixed", CAP_HOLD, 1.0),
                "seat": ("device", "lancet", (0.0, 0.0, SOCKET_DEPTH), "fixed", SEAT_HOLD, 0.05),
                "slide": ("device", "barrel", (0.0, 0.0, BODY_END), "slide", None, None)}


def _named() -> dict:
    """The scene's parts by kind (one of each)."""
    kinds = {Device: "device", Barrel: "barrel", Cap: "cap", Lancet: "lancet", Tab: "tab"}
    return {kinds[type(s)]: n for n, s in _SPECS.items() if type(s) in kinds and n in _PATHS}


def _joint_path(name: str) -> str:
    return f"{_PATHS[_named()[_JOINT_KINDS[name][0]]]}/{name}_joint"


def _author(name: str, environment) -> bool:
    """Author joint ``name`` from where its two bodies are now: disabled first, its frames set, enabled last."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    kind0, kind1, anchor, style, force, torque = _JOINT_KINDS[name]
    parts = _named()
    if kind0 not in parts or kind1 not in parts:
        return False
    (p0, q0), (p1, q1) = _pose(environment, parts[kind0]), _pose(environment, parts[kind1])
    r0, r1 = _rotation(q0), _rotation(q1)
    world = p0 + r0 @ np.asarray(anchor, float)
    local1, quat1 = r1.T @ (world - p1), _quat(r1.T @ r0)
    if style == "slide":
        local1 = np.zeros(3)                         # the barrel's own origin rides the slide, wherever it is now
    stage = get_current_stage()
    cls = {"slide": UsdPhysics.PrismaticJoint, "twist": UsdPhysics.RevoluteJoint}.get(style, UsdPhysics.FixedJoint)
    joint = cls.Define(stage, _joint_path(name))
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([_PATHS[parts[kind0]]])
    joint.CreateBody1Rel().SetTargets([_PATHS[parts[kind1]]])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in anchor)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*(float(v) for v in local1)))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(float(quat1[3]), Gf.Vec3f(*(float(v) for v in quat1[:3]))))
    joint.CreateExcludeFromArticulationAttr().Set(True)
    if style == "slide":
        joint.CreateCollisionEnabledAttr().Set(False)
        joint.CreateAxisAttr().Set("Z")
        joint.CreateLowerLimitAttr().Set(0.0)
        joint.CreateUpperLimitAttr().Set(COCK_TRAVEL)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(COCK_K)
        drive.CreateDampingAttr().Set(COCK_DAMP)
        drive.CreateTargetPositionAttr().Set(0.0)
    else:
        if style == "twist":                         # free to turn about the axis, against a torque that tops out
            joint.CreateAxisAttr().Set("Z")
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(NECK_TORQUE / NECK_YIELD)       # N m per degree
            drive.CreateDampingAttr().Set(NECK_DRAG)
            drive.CreateMaxForceAttr().Set(NECK_TORQUE)
            drive.CreateTargetPositionAttr().Set(0.0)
        joint.CreateCollisionEnabledAttr().Set(name != "neck")       # a cap or a lancet still meets the nose once it is off
        joint.CreateBreakForceAttr().Set(float(force))
        joint.CreateBreakTorqueAttr().Set(float(torque))
    joint.CreateJointEnabledAttr().Set(True)       # last write, after the frames
    return True


def snap(environment, name: str) -> bool:
    """Make the ``click`` (cap on the nose) or the ``seat`` (lancet in the holder) from where the parts are now."""
    return _author(name, environment)


def release(environment, name: str) -> None:
    """Let joint ``name`` go for good (the neck, once the tab has been twisted far enough)."""
    from isaaclab.sim.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(_joint_path(name))
    if prim and prim.IsValid():
        prim.GetAttribute("physics:jointEnabled").Set(False)


def _spawn_dish(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A round open cup: a floor disc and a ring of flat wall pieces, one loose body with its base at the origin."""
    spec: Dish = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass))
    m = a.mat("dish", *spec.friction)
    look = a.look(spec.color, rough=0.5)
    r_in, h, w = float(spec.size[0]) / 2, float(spec.size[2]), float(spec.wall)
    a.cyl(f"{prim_path}/floor", r_in + w, 0.0, w, look, m)
    a.ring(f"{prim_path}/wall", r_in, w, 0.0, h, look, m, pieces=20)
    return a.root


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    _PATHS.pop(spec.name, None)
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


_SPAWNERS = {Device: _spawn_device, Barrel: _spawn_barrel, Cap: _spawn_cap, Lancet: _spawn_lancet, Tab: _spawn_tab,
             Dish: _spawn_dish}


# -- run time ----------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def write_pose(environment, name: str, pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    asset = environment.unwrapped.scene[name]
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + _origin(environment)), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def _pose(environment, name: str):
    pose = _np(environment.unwrapped.scene[name].data.root_link_pose_w)[0]
    return pose[:3], pose[3:7]


def _disarm() -> None:
    """Switch every joint off (before a reset puts the parts back)."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for name in _JOINT_KINDS:
        try:
            prim = stage.GetPrimAtPath(_joint_path(name))
        except KeyError:
            continue
        if prim and prim.IsValid():
            prim.GetAttribute("physics:jointEnabled").Set(False)


def _arm(environment) -> None:
    """Join the parts as the reset left them: the tab on its lancet, the barrel on its slide, and the cap or the
    lancet to the body if the layout starts with it home."""
    parts = _named()
    _author("slide", environment)
    if {"lancet", "tab"} <= set(parts) and joined(*_pose(environment, parts["lancet"]), *_pose(environment, parts["tab"])):
        _author("neck", environment)
    if {"device", "cap"} <= set(parts) and cap_on(cap_fit(*_pose(environment, parts["device"]), *_pose(environment, parts["cap"]))):
        _author("click", environment)
    if {"device", "lancet"} <= set(parts) and seated(seat_fit(*_pose(environment, parts["device"]), *_pose(environment, parts["lancet"]))):
        _author("seat", environment)


def _box_bounds(pos, rot, lo, hi):
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ rot.T + pos
    return pts.min(0), pts.max(0)


def _extent(spec) -> tuple:
    """A part's radius and the span of its own z it occupies."""
    if isinstance(spec, Device):
        return (BODY_R, 0.0, BODY_END)
    if isinstance(spec, Barrel):
        return (BODY_R, 0.0, BARREL_L)
    if isinstance(spec, Cap):
        return (CAP_R, -CAP_L, 0.0)
    if isinstance(spec, Lancet):
        return (LANCET_R, 0.0, LANCET_L + NEEDLE_L)
    if isinstance(spec, Tab):
        return (TAB_R, 0.0, TAB_L)
    return (float(spec.size[0]) / 2 + float(spec.wall), 0.0, float(spec.size[2]))


def _kit_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    r, z0, z1 = _extent(spec)
    low, high = _box_bounds(pos, _rotation(quat), (-r, -r, z0), (r, r, z1))
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=low, high=high)


def finger_points(environment) -> list:
    """The grasp point of each gripper, left then right (world, relative to the env origin)."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


def gripper_points(environment) -> list:
    """Points on each gripper that could touch something held between or beside its fingers, left gripper then right
    (world, relative to the env origin): along both fingers' inner and outer faces, from their tips back to the palm,
    and the palm's middle. The fingers' opening is read from their joints."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        rot = _rotation(pose[3:7])
        tcp = pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin
        q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        pts = [tcp - PALM_BACK * rot[:, 2]]
        # joint7 carries the finger on the gripper's -x side, joint8 the one on its +x side; each pad is |q| from the middle
        for side, joint in ((-1.0, "joint7"), (1.0, "joint8")):
            half = abs(float(q[names.index(joint)]))
            for depth in (PAD_TIP_PAST_TCP, 0.0, -PALM_BACK / 2, -PALM_BACK):
                for face in (0.0, FINGER_THICK):
                    pts.append(tcp + side * (half + face) * rot[:, 0] + depth * rot[:, 2])
        out.append(pts)
    return out


# -- hooking yam_arena -------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_lancetkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        for kind, spawn in _SPAWNERS.items():
            if isinstance(spec, kind):
                return _rigid_object(spec, spawn)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), tuple(_SPAWNERS))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._lancetkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_lancetkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        # The joints have to be whole before the reset's settling steps and first picture: switch them off, let the
        # environment put the parts back, then author them afresh from where the parts are.
        base, saved, had = None, None, False
        try:
            self._ensure_sim(None)
            base = self._env.unwrapped
            env_reset = base.reset
            saved, had = vars(base).get("reset"), "reset" in vars(base)

            def reset_then_arm(*aa, **kk):
                _disarm()
                result = env_reset(*aa, **kk)
                _arm(self._env)
                return result

            base.reset = reset_then_arm
        except Exception as err:                     # noqa: BLE001
            print(f"[lancetkit] joints not re-armed around the reset: {err!r}", file=sys.stderr, flush=True)
            base = None
        try:
            obs = stock_reset(self, *a, **k)
        finally:
            if base is not None:
                if had:
                    base.reset = saved
                else:
                    del base.reset
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._lancetkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
