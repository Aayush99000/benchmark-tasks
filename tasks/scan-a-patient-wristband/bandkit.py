"""bandkit: a training forearm that turns on a sprung pivot, its identification wristband, and a handheld barcode
scanner with a sprung trigger, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from bandkit import Arm, Fixed, Scanner, grippers, on_reset, on_step, show

- :class:`Fixed` is a fixed box with its own friction and look (the arm board is one).
- :class:`Arm` is the forearm and hand of an adult training arm lying along +y, fingers away from its elbow. One
  articulation with a fixed base: a bearing block at the elbow (``mount``) and the forearm, wrist and hand as one
  rigid part (``limb``) that turns about the forearm's long axis (``turn``, rad: 0 palm down, a quarter turn thumb
  up) against a torsion spring that returns it palm down. An identification band is fastened round the wrist; its
  label panel, with a Data Matrix symbol, is on the palm side.
- :class:`Scanner` is a cordless pistol-grip imager: one loose articulation, the housing (foot, handle, head) and
  its trigger on a sprung slide (``trigger``, m: 0 released, negative pulled). Its frame: origin on the handle's
  axis at the foot's underside, z up the handle, x the way it reads. A good-decode light on the back of its head is
  drawn lit or unlit with :func:`show`.

The geometry the task file needs for its reading rule is here as plain functions (:func:`symbol_points`,
:func:`scanner_optics`, :func:`hand_boxes`, :func:`gripper_boxes`, :func:`segment_hits_box`); they need no
simulator. ``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``). Pure Python at import time; Isaac Sim is reached
only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Arm", "Fixed", "KitState", "Scanner", "env", "gripper_boxes", "grippers", "hand_boxes", "install",
           "on_reset", "on_step", "scanner_optics", "segment_hits_box", "show", "symbol_points", "write_joint",
           "write_pose"]

# -- the training arm, in the limb's frame (m): origin on the forearm's axis at its elbow end, y along the axis toward
# -- the fingers; with the forearm palm down, z is up (the back of the hand) and the thumb is on the -x side ---------
FOREARM_R, FOREARM_Y = 0.030, (0.0, 0.215)
WRIST_R, WRIST_Y = 0.027, (0.215, 0.275)
PALM = ((-0.040, 0.040), (0.275, 0.365), (-0.011, 0.011))          # x, y, z extents
FINGERS = ((-0.038, 0.038), (0.365, 0.455), (-0.009, 0.009))
TIP_Y = FINGERS[1][1]                                               # the longest fingertip
# The four fingers lie side by side, straight (drawn as rounded rods inside the FINGERS slab, which is what collides):
# index, middle, ring, little, from the thumb's side; each is (its centre line's x, its tip's y).
FINGER_R = 0.0085
FINGER_RODS = ((-0.0285, 0.4490), (-0.0095, TIP_Y), (0.0095, 0.4510), (0.0285, 0.4360))
THUMB_R = 0.009
THUMB_ROD = ((-0.036, 0.300), (-0.066, 0.362))                      # its centre line, from the palm's edge outward
THUMB = ((-0.076, -0.040), (0.292, 0.372), (-THUMB_R, THUMB_R))     # the box round it
BAND_Y, BAND_W, BAND_T = 0.245, 0.0254, 0.0003                      # a 1 in band, 0.3 mm thick, round the wrist
BAND_R = WRIST_R + BAND_T
PANEL = 0.60                                                        # the label panel reaches this far (rad) each way
MOUNT = ((-0.035, 0.035), (-0.060, -0.004))                         # the bearing block's footprint, x and y

# The band's symbol: a 22 x 22 ECC 200 Data Matrix printed with 10 mil modules (5.6 mm square), encoding the text
# "TRAINING BAND 0417 DOE JANE 1961-03-07" (a made-up training record). Rows from the top, 1 = dark.
MODULE = 0.000254
MATRIX = ("1010101010101010101010", "1111110011110100010011", "1010111010100110101000", "1011001001001100100011",
          "1010111110000010111000", "1010110010010100100001", "1101111101000000110010", "1000110101100001001111",
          "1001011101111111110100", "1000100011100011010111", "1001000111110001111110", "1100010001101111001011",
          "1010011100101000100110", "1001000101011000110011", "1011000111000000111010", "1111000001000010001011",
          "1111010010101110001100", "1010000010010111011001", "1001110001001101110000", "1011010001010110000011",
          "1010011100100000101010", "1111111111111111111111")
SYMBOL = MODULE * len(MATRIX)

# -- the scanner, in its own frame (m): origin on the handle's axis at the foot's underside, z up the handle, x the
# -- way it reads, y to its left. 17.5 cm tall, 6.6 cm wide, 9.0 cm deep, as a cordless pistol-grip imager ---------
FOOT = ((-0.030, 0.030), (-0.030, 0.030), (0.0, 0.022))
HANDLE = ((-0.016, 0.016), (-0.014, 0.014), (0.022, 0.120))
HEAD = ((-0.040, 0.050), (-0.033, 0.033), (0.120, 0.175))
WINDOW = (HEAD[0][1], 0.0, 0.1475)                                  # the middle of the scan window, on the head's front
TRIGGER_AT = (HANDLE[0][1], 0.0, 0.095)                             # where the trigger meets the handle's front
TRIGGER_SIZE = (0.008, 0.014, 0.025)                                # how far it stands proud, its width, its length
TRIGGER_TRAVEL = 0.005
TRIGGER_MASS = 0.004
GRIP = (0.0015, 0.0, TRIGGER_AT[2])                                 # between a pulled trigger and the handle's back

# -- a gripper, in its tool frame (x the closing direction, z the approach): boxes that bound its two fingers and
# -- the body behind them, for line-of-sight checks -----------------------------------------------------------------
FINGER_BOX = (0.010, 0.040, (-0.045, 0.013))                        # thick (x), wide (y), z extent
BODY_BOX = ((-0.051, 0.051), (-0.034, 0.034), (-0.150, -0.045))


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction and look."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.6
    metallic: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Arm(Box):
    """A training forearm on its pivot. ``pos`` is the elbow end of the forearm's axis; the forearm lies along +y.
    ``height`` is the axis's height above the table (the bearing block stands on the table). ``spring`` (N m/rad)
    returns the forearm palm down, ``damping`` (N m s/rad) steadies it, ``limit`` (rad) is how far it can be turned.
    ``mass`` is the turning part's (forearm, wrist and hand)."""

    height: float = 0.108
    spring: float = 0.035
    damping: float = 0.003
    limit: float = math.radians(170)
    friction: tuple = (0.9, 0.8)                    # silicone skin
    skin: tuple = (0.87, 0.68, 0.56)
    band: tuple = (0.20, 0.50, 0.85)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: an Arm needs the mass of its turning part")


@dataclass(frozen=True)
class Scanner(Box):
    """A loose handheld scanner. ``pos`` is its frame's origin, ``quat`` (xyzw) its orientation. ``mass`` is the
    whole scanner's. ``trigger_spring`` (N/m) resists the trigger over its 5 mm of travel."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    trigger_spring: float = 500.0
    trigger_damping: float = 1.0
    friction: tuple = (0.7, 0.6)
    shell: tuple = (0.93, 0.94, 0.95)
    trim: tuple = (0.10, 0.36, 0.74)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= TRIGGER_MASS:
            raise ValueError(f"{self.name}: a Scanner must be loose (give it a mass)")


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``joints`` maps its joints to their positions (rad or m) and
    ``joint_vel`` to their speeds."""

    joints: dict = field(default_factory=dict)
    joint_vel: dict = field(default_factory=dict)


# -- geometry (no simulator needed) -----------------------------------------------------------------------------------

def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def turned(angle: float) -> np.ndarray:
    """The limb's rotation when the forearm is turned ``angle`` (rad) about its axis (+y): a positive angle raises
    the thumb."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def symbol_points(elbow, turn: float):
    """The band's symbol with the forearm turned ``turn``: its centre and four corners (world) and the band's outward
    normal at each, given the elbow end of the forearm's axis (world). The symbol follows the band's curve."""
    half = SYMBOL / 2
    pts, normals = [], []
    for arc, along in ((0.0, 0.0), (-half, -half), (-half, half), (half, -half), (half, half)):
        phi = arc / BAND_R
        out = turned(turn) @ np.array([-math.sin(phi), 0.0, -math.cos(phi)])
        pts.append(np.asarray(elbow, float) + BAND_R * out + np.array([0.0, BAND_Y + along, 0.0]))
        normals.append(out)
    return np.array(pts), np.array(normals)


def scanner_optics(pos, quat_xyzw):
    """From the scanner's pose: the middle of its scan window (world), the direction it reads along, and the
    imager's horizontal and vertical directions."""
    rot = _rotation(quat_xyzw)
    return np.asarray(pos, float) + rot @ np.array(WINDOW), rot[:, 0], rot[:, 1], rot[:, 2]


def hand_boxes(elbow, turn: float) -> list:
    """The training hand's palm, fingers and thumb as boxes (centre, rotation, half sizes), world."""
    rot = turned(turn)
    out = []
    for (x0, x1), (y0, y1), (z0, z1) in (PALM, FINGERS, THUMB):
        centre = np.asarray(elbow, float) + rot @ np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        out.append((centre, rot, np.array([x1 - x0, y1 - y0, z1 - z0]) / 2))
    return out


def gripper_boxes(tcp, rot, gap: float) -> list:
    """One gripper as three boxes (centre, rotation, half sizes), world: its two fingers, ``gap`` apart at their
    pads, and its body behind them. ``tcp`` is its grasp point and ``rot`` its tool rotation (x closing, z approach)."""
    tcp, rot = np.asarray(tcp, float), np.asarray(rot, float)
    thick, wide, (z0, z1) = FINGER_BOX
    out = []
    for side in (-1.0, 1.0):
        centre = tcp + rot @ np.array([side * (gap / 2 + thick / 2), 0.0, (z0 + z1) / 2])
        out.append((centre, rot, np.array([thick, wide, z1 - z0]) / 2))
    (x0, x1), (y0, y1), (z0, z1) = BODY_BOX
    out.append((tcp + rot @ np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]), rot,
                np.array([x1 - x0, y1 - y0, z1 - z0]) / 2))
    return out


def segment_hits_box(p0, p1, box) -> bool:
    """Whether the straight line from ``p0`` to ``p1`` passes through ``box`` (centre, rotation, half sizes)."""
    centre, rot, half = box
    a = rot.T @ (np.asarray(p0, float) - centre)
    d = rot.T @ (np.asarray(p1, float) - centre) - a
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if abs(a[i]) > half[i]:
                return False
            continue
        ta, tb = (-half[i] - a[i]) / d[i], (half[i] - a[i]) / d[i]
        t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
        if t0 > t1:
            return False
    return True


def box_corners(box) -> np.ndarray:
    centre, rot, half = box
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    return centre + (signs * half) @ rot.T


def scanner_boxes(pos, quat_xyzw) -> list:
    """The scanner's foot, handle and head as boxes (centre, rotation, half sizes), world."""
    rot = _rotation(quat_xyzw)
    out = []
    for (x0, x1), (y0, y1), (z0, z1) in (FOOT, HANDLE, HEAD):
        centre = np.asarray(pos, float) + rot @ np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        out.append((centre, rot, np.array([x1 - x0, y1 - y0, z1 - z0]) / 2))
    return out


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


# -- authoring helpers --------------------------------------------------------------------------------------------------

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

    def look(self, color, rough=0.5, metal=0.0, opacity=1.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal, opacity)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal,
                                                   opacity=opacity)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, pos, mass, lin_damp=0.05, ang_damp=0.05, awake=False):
        from pxr import PhysxSchema, UsdPhysics

        xf = self.UsdGeom.Xform.Define(self.stage, path)
        if path != self.path:
            xf.AddTranslateOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*pos))
            xf.AddOrientOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        if awake:                                                         # a slow turn must not be put to sleep
            rb.CreateSleepThresholdAttr(0.0)
            rb.CreateStabilizationThresholdAttr(0.0)
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

    def _finish(self, prim, look, mat):
        if look:
            self.paint(prim, look)
        else:
            self.UsdGeom.Imageable(prim).MakeInvisible()
        if mat:
            self.collide(prim, mat)
        return prim

    def box(self, path, centre, size, look, mat=None, quat_wxyz=None):
        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if quat_wxyz is not None:
            cube.AddOrientOp().Set(self.Gf.Quatf(*(float(v) for v in quat_wxyz)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        return self._finish(cube.GetPrim(), look, mat)

    def span(self, path, extents, look, mat=None):
        """A box given by its (x, y, z) extents."""
        (x0, x1), (y0, y1), (z0, z1) = extents
        return self.box(path, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), look, mat)

    def cyl(self, path, r, a0, a1, look, mat=None, axis="Y", centre=(0.0, 0.0)):
        """A cylinder along ``axis`` from ``a0`` to ``a1``; ``centre`` is its axis in the other two coordinates."""
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(a1 - a0))
        mid = float(a0 + a1) / 2
        at = {"X": (mid, centre[0], centre[1]), "Y": (centre[0], mid, centre[1]), "Z": (centre[0], centre[1], mid)}[axis]
        c.AddTranslateOp().Set(self.Gf.Vec3d(*at))
        return self._finish(c.GetPrim(), look, mat)

    def rod(self, path, p0, p1, r, look, mat=None):
        """A round-ended rod of radius ``r`` whose centre line runs from ``p0`` to ``p1``."""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        length = float(np.linalg.norm(d))
        d = d / length
        c = self.UsdGeom.Capsule.Define(self.stage, path)
        c.CreateAxisAttr("Y")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(length)
        c.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in (p0 + p1) / 2)))
        axis = np.cross([0.0, 1.0, 0.0], d)
        n = float(np.linalg.norm(axis))
        if n > 1e-9:                                                      # turn the rod's y axis onto its centre line
            half = math.atan2(n, float(d[1])) / 2
            v = axis / n * math.sin(half)
            c.AddOrientOp().Set(self.Gf.Quatf(float(math.cos(half)), float(v[0]), float(v[1]), float(v[2])))
        return self._finish(c.GetPrim(), look, mat)


# -- spawning ------------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}


def _fixed_object(scene, spec: Fixed):
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness,
                                       metallic=spec.metallic)
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size),
                              physics_material=_material(sim_utils, spec.static_friction, spec.dynamic_friction),
                              visual_material=look, collision_props=scene.PhysxCollisionPropertiesCfg())
    return scene.Object(name=spec.name, object_type=scene.ObjectType.BASE, spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                                rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _qy(angle):
    return (math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0)          # wxyz, a turn about y


def _spawn_arm(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The training arm: a bearing block fixed to the world and the limb turning on it about the forearm's axis."""
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    spec: Arm = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    _PATHS[spec.name] = prim_path
    skin = a.look(spec.skin, rough=0.55)
    crease = a.look(tuple(0.72 * c for c in spec.skin), rough=0.7)
    nail = a.look((0.96, 0.84, 0.80), rough=0.3)
    steel = a.look((0.55, 0.57, 0.60), rough=0.35, metal=0.8)
    band = a.look(spec.band, rough=0.4)
    paper = a.look((0.97, 0.97, 0.95), rough=0.6)
    ink = a.look((0.03, 0.03, 0.03), rough=0.6)
    clasp = a.look((0.15, 0.35, 0.70), rough=0.4)
    m_skin = a.mat("skin", *spec.friction)
    m_steel = a.mat("steel", 0.5, 0.4)

    mount, limb = f"{prim_path}/mount", f"{prim_path}/limb"
    a.body(mount, (0.0, 0.0, 0.0), 0.5)
    (mx0, mx1), (my0, my1) = MOUNT
    a.span(f"{mount}/block", ((mx0, mx1), (my0, my1), (-spec.height, FOREARM_R + 0.010)), steel, m_steel)
    a.cyl(f"{mount}/journal", 0.012, my1, 0.0, steel)                     # the pivot's shaft, drawn

    a.body(limb, (0.0, 0.0, 0.0), spec.mass, ang_damp=0.0, awake=True)
    a.cyl(f"{limb}/forearm", FOREARM_R, *FOREARM_Y, skin, m_skin)
    a.cyl(f"{limb}/wrist", WRIST_R, *WRIST_Y, skin, m_skin)
    a.span(f"{limb}/palm", PALM, skin, m_skin)
    a.span(f"{limb}/fingers", FINGERS, None, m_skin)                      # what collides: the fingers held together
    knuckle = FINGERS[1][0]
    for i, (x, tip) in enumerate(FINGER_RODS):                            # what is drawn: four fingers with nails
        a.rod(f"{limb}/finger_{i}", (x, knuckle - 0.004, 0.0), (x, tip - FINGER_R, 0.0), FINGER_R, skin)
        a.box(f"{limb}/nail_{i}", (x, tip - 0.010, FINGER_R - 0.0012), (0.011, 0.012, 0.0030), nail)
    (tx0, ty0), (tx1, ty1) = THUMB_ROD
    a.rod(f"{limb}/thumb", (tx0, ty0, 0.0), (tx1, ty1, 0.0), THUMB_R, skin, m_skin)
    a.rod(f"{limb}/heel", (PALM[0][0] + 0.004, PALM[1][0] + 0.004, 0.0), (PALM[0][0] + 0.004, PALM[1][1] - 0.010, 0.0),
          PALM[2][1], skin)                                               # the palm's rounded edges, drawn
    a.rod(f"{limb}/edge", (PALM[0][1] - 0.004, PALM[1][0] + 0.004, 0.0), (PALM[0][1] - 0.004, PALM[1][1] - 0.006, 0.0),
          PALM[2][1], skin)
    for i, y in enumerate((WRIST_Y[1] - 0.006, WRIST_Y[1] - 0.011)):      # the wrist's creases, drawn
        a.cyl(f"{limb}/crease_{i}", WRIST_R + 0.00015, y - 0.0004, y + 0.0004, crease)
    # the band round the wrist: a 0.3 mm strip, its clasp on the back of the wrist and its label panel on the palm side
    a.cyl(f"{limb}/band", BAND_R, BAND_Y - BAND_W / 2, BAND_Y + BAND_W / 2, band)
    a.box(f"{limb}/clasp", (0.0, BAND_Y, BAND_R + 0.0012), (0.014, 0.012, 0.0024), clasp)
    facets, step = 7, 2 * PANEL / 7
    for k in range(facets):
        phi = -PANEL + (k + 0.5) * step
        r = BAND_R + 0.00012
        a.box(f"{limb}/panel_{k}", (-r * math.sin(phi), BAND_Y, -r * math.cos(phi)),
              (2 * r * math.tan(step / 2) + 0.0002, BAND_W - 0.004, 0.0001), paper, quat_wxyz=_qy(phi))
    n, z = len(MATRIX), -(BAND_R + 0.00030)
    for i, row in enumerate(MATRIX):                                      # the symbol, one box for each run of dark modules
        j = 0
        while j < n:
            if row[j] != "1":
                j += 1
                continue
            k = j
            while k < n and row[k] == "1":
                k += 1
            # seen from outside with the fingers upward, column 0 is on the left: that is the +x side
            x = (n / 2 - (j + k) / 2) * MODULE
            y = BAND_Y + (n / 2 - i - 0.5) * MODULE
            a.box(f"{limb}/m_{i}_{j}", (x, y, z), ((k - j) * MODULE, MODULE, 0.00006), ink)
            j = k

    stage = a.stage
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{limb}/turn")
    j.CreateAxisAttr("Y")
    j.CreateBody0Rel().SetTargets([Sdf.Path(mount)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(limb)])
    j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(math.degrees(spec.limit))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.spring) * math.pi / 180)         # USD takes N m per degree
    drive.CreateDampingAttr(float(spec.damping) * math.pi / 180)
    drive.CreateTargetPositionAttr(0.0)
    # the bearing block is fixed to the world where it stands
    world = UsdGeom.Xformable(a.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p, q = world.ExtractTranslation(), world.ExtractRotationQuat()
    fix = UsdPhysics.FixedJoint.Define(stage, f"{mount}/fix")
    fix.CreateBody1Rel().SetTargets([Sdf.Path(mount)])
    fix.CreateLocalPos0Attr(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
    fix.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), *(float(v) for v in q.GetImaginary())))
    fix.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fix.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    UsdPhysics.ArticulationRootAPI.Apply(a.root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(a.root)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    art.CreateSleepThresholdAttr(0.0)
    art.CreateStabilizationThresholdAttr(0.0)
    return a.root


def _spawn_scanner(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The scanner: its housing (the articulation's root, loose) and its trigger on a sprung slide."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Scanner = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    _PATHS[spec.name] = prim_path
    shell = a.look(spec.shell, rough=0.4)
    trim = a.look(spec.trim, rough=0.45)
    glass = a.look((0.05, 0.02, 0.02), rough=0.1)
    unlit = a.look((0.20, 0.22, 0.21), rough=0.3)
    lit = a.look((0.10, 1.00, 0.25), rough=0.2)
    m = a.mat("shell", *spec.friction)

    housing, trigger = f"{prim_path}/housing", f"{prim_path}/trigger"
    a.body(housing, (0.0, 0.0, 0.0), spec.mass - TRIGGER_MASS)
    a.span(f"{housing}/foot", FOOT, trim, m)
    a.span(f"{housing}/handle", HANDLE, shell, m)
    a.span(f"{housing}/head", HEAD, shell, m)
    (hx0, hx1), (hy0, hy1), (hz0, hz1) = HEAD
    a.cyl(f"{housing}/back", HANDLE[1][1], HANDLE[2][0], HANDLE[2][1], shell, axis="Z", centre=(HANDLE[0][0], 0.0))
    a.box(f"{housing}/bumper", (hx1 - 0.004, 0.0, (hz0 + hz1) / 2), (0.0085, hy1 - hy0 + 0.002, hz1 - hz0 + 0.002),
          a.look((0.12, 0.12, 0.13), rough=0.7))
    a.box(f"{housing}/chin", ((hx0 + hx1) / 2 - 0.006, 0.0, hz0 + 0.004), (hx1 - hx0 - 0.014, hy1 - hy0 + 0.0008, 0.008), trim)
    a.box(f"{housing}/window", (hx1 + 0.0002, 0.0, WINDOW[2]), (0.0004, 0.050, 0.040), glass)
    a.box(f"{housing}/bezel", ((hx0 + hx1) / 2, 0.0, hz1 + 0.0002), (hx1 - hx0 - 0.010, 0.030, 0.0004), trim)
    led = ((hx0 - 0.0003, 0.0, 0.158), (0.0006, 0.034, 0.010))            # the good-decode light, on the head's back
    a.box(f"{housing}/led_off", *led, unlit)
    a.box(f"{housing}/led_on", (led[0][0] - 0.0002, 0.0, led[0][2]), led[1], lit)
    a.UsdGeom.Imageable(a.stage.GetPrimAtPath(f"{housing}/led_on")).MakeInvisible()

    a.body(trigger, TRIGGER_AT, TRIGGER_MASS)
    a.box(f"{trigger}/blade", (TRIGGER_SIZE[0] / 2, 0.0, 0.0), TRIGGER_SIZE, trim, m)

    stage = a.stage
    j = UsdPhysics.PrismaticJoint.Define(stage, f"{trigger}/trigger")
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path(housing)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(trigger)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*TRIGGER_AT))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLowerLimitAttr(-TRIGGER_TRAVEL)
    j.CreateUpperLimitAttr(0.0)
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.trigger_spring))
    drive.CreateDampingAttr(float(spec.trigger_damping))
    drive.CreateTargetPositionAttr(0.0)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(housing))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(housing))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return a.root


def _articulation(spec, spawn, actuators: dict, quat=(0.0, 0.0, 0.0, 1.0)):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(SpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            acts = {n: ImplicitActuatorCfg(joint_names_expr=[n], stiffness=k, damping=c)
                    for n, (k, c) in actuators.items()}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=acts)
            return self._add_initial_pose_to_cfg(cfg)

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=tuple(float(v) for v in quat)))


# -- run time ------------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def write_pose(environment, name: str, pos, quat_xyzw) -> None:
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


def write_joint(environment, name: str, joint: str, position: float) -> None:
    """Set one joint of kit object ``name`` to ``position``, at rest."""
    import torch

    asset = environment.unwrapped.scene[name]
    ids = [list(asset.joint_names).index(joint)]
    dev = _t(asset.data.joint_pos).device
    pos = torch.full((1, 1), float(position), device=dev)
    fn = getattr(asset, "write_joint_state_to_sim_index", None) or asset.write_joint_state_to_sim
    fn(position=pos, velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=[0])


def _joints(asset):
    names = list(asset.joint_names)
    q, v = _np(asset.data.joint_pos)[0], _np(asset.data.joint_vel)[0]
    return {n: float(q[i]) for i, n in enumerate(names)}, {n: float(v[i]) for i, n in enumerate(names)}


def _arm_state(environment, spec: Arm) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos = pose[:3] - _origin(environment)
    joints, vel = _joints(asset)
    reach = max(abs(PALM[0][0]), abs(THUMB[0][0]))
    low = pos + np.array([-reach, MOUNT[1][0], -reach])
    high = pos + np.array([reach, TIP_Y, reach])
    return KitState(pos=pos, quat_xyzw=pose[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=low, high=high,
                    joints=joints, joint_vel=vel)


def _scanner_state(environment, spec: Scanner) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = np.zeros(6)
    for attr in ("root_com_vel_w", "root_vel_w", "root_link_vel_w"):
        if hasattr(asset.data, attr):
            vel = _np(getattr(asset.data, attr))[0]
            break
    pts = np.vstack([box_corners(b) for b in scanner_boxes(pos, quat)])
    joints, jvel = _joints(asset)
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                    joints=joints, joint_vel=jvel)


def grippers(environment) -> list:
    """Each gripper, left then right: its grasp point (world, relative to the env origin), its tool rotation
    (x closing, z approach) and the gap between its pads (m), read from its finger joints."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        rot = _rotation(pose[3:7])
        q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        gap = sum(abs(float(q[names.index(j)])) for j in ("joint7", "joint8"))
        out.append({"tcp": pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin, "rot": rot, "gap": gap})
    return out


_SHOWN: dict = {}
_WARNED: set = set()


def show(name: str, part: str, visible: bool) -> None:
    """Show or hide a drawn part of kit object ``name`` (``part`` is its path under the object). Drawing only: a
    failure is reported once and never stops a trial."""
    if _ENV["env"] is None or name not in _PATHS or _SHOWN.get((name, part)) == visible:
        return
    _SHOWN[(name, part)] = visible
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdGeom

        img = UsdGeom.Imageable(get_current_stage().GetPrimAtPath(f"{_PATHS[name]}/{part}"))
        img.MakeVisible() if visible else img.MakeInvisible()
    except Exception as err:                                              # noqa: BLE001
        if (name, part) not in _WARNED:
            _WARNED.add((name, part))
            print(f"[bandkit] could not draw {name}/{part}: {err!r}", file=sys.stderr, flush=True)


# -- hooking yam_arena ------------------------------------------------------------------------------------------------

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
    if getattr(scene, "_bandkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Arm):
            return _articulation(spec, _spawn_arm, {"turn": (spec.spring, spec.damping)})
        if isinstance(spec, Scanner):
            return _articulation(spec, _spawn_scanner, {"trigger": (spec.trigger_spring, spec.trigger_damping)},
                                 quat=spec.quat)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        kit = {n for n in names if isinstance(_SPECS.get(n), (Arm, Scanner))}
        plain = [n for n in names if n not in kit]
        out = stock_states(environment, plain) if plain else {}
        for n in kit:
            spec = _SPECS[n]
            out[n] = _arm_state(environment, spec) if isinstance(spec, Arm) else _scanner_state(environment, spec)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._bandkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_bandkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        _SHOWN.clear()
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._bandkit_patched = True


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
