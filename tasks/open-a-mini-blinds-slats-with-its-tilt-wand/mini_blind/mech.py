"""A 1 in aluminium mini blind, 18 in wide by 12 in drop (the smallest size its retailer makes), with its wand tilter,
on a table-top stand, built true size. Created 11_17_26 (America/New_York); 18 in product size set 11_17_26 14:00.

Geometry is NumPy only, so the task file and the tests use it without Isaac Sim; the USD builder imports pxr when called.

Sources (README "Where the numbers come from"):
- The simulated product (README names it and gives its price): a made-to-measure 1 in mini blind sold at retail from
  18 in wide by 12 in high; headrail 1-1/2 in by 1 in; 6 gauge aluminium slats; phosphate-treated steel headrail and
  bottom rail; clear wand for tilting, supplied at least 24 in long; cordless lift (no lift cord hangs from it).
- Not on that product's page, taken from a contract 1 in mini blind cut sheet with the same headrail (README names
  it): ladder spacing 21.5 mm; hexagonal tilt rod; clear hexagonal wand; enclosed worm and gear tilter with a clutch.
- Slats nominally 1 in wide; wand "approximately 1/4 in" across, hexagonal: a second contract maker's specification
  page.
- Worm ratio: a tilter patent gives 5:1 to 15:1 between worm and gear; 10:1 is used here (the middle).
- Not published, chosen and stated: the slat angle at the closed stops (70 degrees from level), the tilter's turning
  torque (0.015 N m at the wand), the wand's stand-off (27 mm in front of the slats' centre line), the tilter's place
  along the headrail (52 mm from its left end), the wand's length (the supplied wand, 24 in or longer, sawn to 250 mm
  so that it hangs clear of the table from a blind whose top is 350 mm above it), the bottom rail's section (25 x 10 mm), the hook's swing damping, and the stand.

Frames (m). The blind's frame sits on the table under the middle of the slats: x away from the robot (the blind's back),
y along the headrail toward the robot's left, z up. One articulation with a free root (stand, headrail). Each slat and
the bottom rail turn on a revolute joint about y through their own centre line; angle 0 is level, positive lifts the
room-side (front, -x) edge. The tilter stem turns about the vertical without limit; the wand hangs from it on a hook
modelled as a universal joint (two revolute joints about x and y), so it swings freely and carries twist to the stem.

The worm, gear, tilt rod, drums and ladder cords are one kinematic relation, kept by Tilter below: the tilt rod turns
1/RATIO of the stem's turning and stops at +-CLOSED_DEG while the stem keeps turning (the clutch); every slat is driven
to the tilt rod's angle. Nothing feeds back from the slats to the stem (a worm does not back-drive).
"""

import math

import numpy as np

IN = 0.0254
# -- the blind --------------------------------------------------------------------------------------------------------
WIDTH = 18 * IN  # headrail length: the product's smallest width (its smallest height, 12 in, is the drop)
HEADRAIL = dict(d=1.5 * IN, h=1.0 * IN)  # product page
SLAT = dict(w=0.025, len=WIDTH - 0.005, t=0.0015)  # 1 in nominal; t is the collider's thickness (the crowned slat's envelope)
SLAT_METAL_T = 0.006 * IN  # 6 gauge (0.006 in) aluminium, for the mass
PITCH = 0.0215  # contract ladder (cut sheet; not on the product page: count the slats of a unit)
N_SLATS = 12
RAIL = dict(w=0.025, h=0.010)  # bottom rail (C-shaped steel), section chosen
FIRST_SLAT_DROP = 0.015  # top slat's centre line below the headrail
CLOSED_DEG = 70.0  # the slats' angle from level at either closed stop (chosen; measure with a protractor)
RATIO = 10.0  # wand turns per tilt-rod turn (patent range 5 to 15)
TILTER_TORQUE_NM = 0.015  # to turn the wand (chosen; measure with a spring scale on a 50 mm lever: 0.3 N)
DYNAMIC_FRACTION = 0.8
WAND = dict(af=0.25 * IN, len=0.250)  # hexagon across flats; length from the hook (supplied 24 in or longer, sawn to this)
WAND_DENSITY = 1180.0  # acrylic ("Lucite")
WAND_STANDOFF = 0.027  # wand axis in front of the slats' centre line (8 mm in front of the headrail's face)
TILTER_END = 0.052  # the tilter's stem from the headrail's left end, seen from the room (chosen; measure with a ruler)
TILTER_Y = WIDTH / 2 - TILTER_END
HOOK_DROP = 0.008  # hook below the headrail
SWING_LIMIT_DEG = 70.0
SWING_DAMPING = 2.5e-4  # N m s/rad at the hook (chosen: a freed wand swings a few times, then hangs still)
# -- the stand --------------------------------------------------------------------------------------------------------
BOARD = dict(x=0.180, y=0.520, t=0.012)  # plywood base
LINTEL = dict(x=0.060, y=0.500, t=0.019)  # softwood head piece the blind's brackets screw to
POST = dict(s=0.020, x=0.032, y=0.215)  # two posts behind the slats
TOP_Z = 0.350  # top of the lintel above the table
HEADRAIL_TOP = TOP_Z - LINTEL["t"]
HEADRAIL_BOTTOM = HEADRAIL_TOP - HEADRAIL["h"]
SLAT_Z = tuple(HEADRAIL_BOTTOM - FIRST_SLAT_DROP - k * PITCH for k in range(N_SLATS))
RAIL_Z = SLAT_Z[-1] - PITCH
HOOK = np.array((-WAND_STANDOFF, TILTER_Y, HEADRAIL_BOTTOM - HOOK_DROP))
SLAT_MASS = 2700.0 * SLAT["w"] * SLAT["len"] * SLAT_METAL_T  # 4 g
RAIL_MASS = 0.040
WAND_AREA = math.sqrt(3) / 2 * WAND["af"] ** 2
WAND_MASS = WAND_DENSITY * WAND_AREA * WAND["len"]  # 10 g
STAND_MASS = 0.67 + 0.28 + 2 * 0.066  # board, lintel, posts
HEADRAIL_MASS = 0.30  # steel channel with tilter, rod, drums and the cordless lift's spring (estimate; weigh a unit)
ROOT_MASS = STAND_MASS + HEADRAIL_MASS
SLAT_DRIVE = dict(stiffness=0.02, damping=2.0e-4, max_force=0.05)  # N m/rad, N m s/rad, N m: holds a slat at the tilt rod's angle
MU = {"wood": 0.5, "metal": 0.4, "plastic": 0.6}
SLAT_DOFS = tuple(f"slat{k:02d}" for k in range(N_SLATS)) + ("rail",)
DOFS = ("stem", "swingx", "swingy") + SLAT_DOFS
LINKS = ("frame", "stem", "cross", "wand") + SLAT_DOFS
DROP = HEADRAIL_TOP - (RAIL_Z - RAIL["h"] / 2)  # headrail top to bottom rail, slats level


def wrap(d):
    """An angle difference brought into (-pi, pi]."""
    return (d + math.pi) % (2 * math.pi) - math.pi


class Tilter:
    """The worm, gear and clutch. `stem` is the stem's unwrapped angle (rad, positive anticlockwise seen from above);
    `tilt` the tilt rod's (and so the slats') angle (rad). Turning the wand clockwise seen from above lifts the slats'
    room-side edge. At either stop the clutch slips: the stem goes on turning and the tilt rod stays."""

    def __init__(self, tilt_deg):
        self.tilt = math.radians(tilt_deg)
        self.stem = 0.0
        self._last = None

    def feed(self, stem_reading):
        """Take the stem joint's reading (rad, wrapped anyhow) and return the tilt rod's angle (rad)."""
        if self._last is not None:
            d = wrap(float(stem_reading) - self._last)
            self.stem += d
            lim = math.radians(CLOSED_DEG)
            self.tilt = min(max(self.tilt - d / RATIO, -lim), lim)
        self._last = float(stem_reading)
        return self.tilt


def turns_to_level(start_deg=-CLOSED_DEG):
    """Wand turns from a slat angle to level."""
    return abs(start_deg) * RATIO / 360.0


def wand_bottom(swing_x=0.0, swing_y=0.0):
    """The wand's lower end in the blind's frame for the hook's two swing angles (rad)."""
    cx, sx, cy, sy = math.cos(swing_x), math.sin(swing_x), math.cos(swing_y), math.sin(swing_y)
    rx = np.array(((1, 0, 0), (0, cx, -sx), (0, sx, cx)))
    ry = np.array(((cy, 0, sy), (0, 1, 0), (-sy, 0, cy)))
    return HOOK + rx @ ry @ np.array((0.0, 0.0, -WAND["len"]))


# -- USD ----------------------------------------------------------------------------------------------------------
COLORS = {  # sRGB base colour, metallic, roughness, opacity
    "ply": ((0.78, 0.66, 0.48), 0.0, 0.7, 1.0), "pine": ((0.86, 0.76, 0.58), 0.0, 0.7, 1.0),
    "rail": ((0.93, 0.93, 0.91), 0.0, 0.4, 1.0), "slat": ((0.95, 0.95, 0.93), 0.0, 0.35, 1.0),
    "wand": ((0.55, 0.75, 0.85), 0.0, 0.15, 0.75), "hook": ((0.75, 0.76, 0.78), 1.0, 0.3, 1.0),
}


def _schema(prim, schema, **attrs):
    from pxr import Sdf

    if not prim.ApplyAPI(schema):
        prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


def _lin(c):
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


class _Builder:
    def __init__(self, stage, root):
        self.stage, self.root = stage, root
        self.looks, self.phys = {}, {}

    def look(self, name):
        from pxr import Gf, Sdf, UsdShade

        if name not in self.looks:
            color, metal, rough, opacity = COLORS[name]
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Looks/{name}")
            sh = UsdShade.Shader.Define(self.stage, f"{self.root}/Looks/{name}/shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*_lin(color)))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            if opacity < 1.0:
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.looks[name] = mat
        return self.looks[name]

    def surface(self, name):
        from pxr import UsdPhysics, UsdShade

        if name not in self.phys:
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Physics/{name}")
            pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            pm.CreateStaticFrictionAttr(MU[name])
            pm.CreateDynamicFrictionAttr(MU[name])
            self.phys[name] = mat
        return self.phys[name]

    def _finish(self, g, look, collide):
        from pxr import UsdPhysics, UsdShade

        prim = g.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            _schema(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": 0.0015, "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        return g

    def box(self, path, center, size, look, collide=None):
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        return self._finish(c, look, collide)

    def cyl(self, path, center, r, h, look, collide=None, axis="Z"):
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        return self._finish(c, look, collide)

    def hex_prism(self, path, af, z0, z1, look, collide=None):
        """A hexagonal prism along z from z0 to z1, `af` across flats, two flats square to y; its collider is its own
        convex hull."""
        from pxr import Gf, UsdGeom, UsdPhysics

        r = af / math.sqrt(3)  # centre to corner
        ring = [(r * math.cos(math.radians(60 * i)), r * math.sin(math.radians(60 * i))) for i in range(6)]
        pts = [Gf.Vec3f(x, y, z0) for x, y in ring] + [Gf.Vec3f(x, y, z1) for x, y in ring]
        counts, idx = [], []
        for i in range(6):
            j = (i + 1) % 6
            counts.append(4)
            idx += [i, j, j + 6, i + 6]
        counts += [6, 6]
        idx += [5, 4, 3, 2, 1, 0] + [6, 7, 8, 9, 10, 11]  # ring runs anticlockwise seen from +z
        m = UsdGeom.Mesh.Define(self.stage, path)
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(idx)
        m.CreateSubdivisionSchemeAttr("none")
        g = self._finish(m, look, collide)
        if collide:
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        return g

    def rigid(self, prim, mass, com, inertia, gravity=True, damping=0.05):
        from pxr import Gf, UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim)
        _schema(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": 32,
                                              "physxRigidBody:solverVelocityIterationCount": 4,
                                              "physxRigidBody:maxDepenetrationVelocity": 0.2,
                                              "physxRigidBody:disableGravity": not gravity,
                                              "physxRigidBody:angularDamping": float(damping),
                                              "physxRigidBody:linearDamping": float(damping)})
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*(max(float(v), 1e-7) for v in inertia)))


def _xform(stage, path, pos=(0.0, 0.0, 0.0)):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    return x.GetPrim()


def _revolute(stage, path, body0, body1, pos0, axis, limits_deg=None, drive=None):
    """A revolute joint at `pos0` in body0's frame and at body1's origin. `drive` = (stiffness, damping, max force) in
    N m/rad, N m s/rad, N m."""
    from pxr import Gf, Sdf, UsdPhysics

    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if limits_deg is not None:
        j.CreateLowerLimitAttr(float(limits_deg[0]))
        j.CreateUpperLimitAttr(float(limits_deg[1]))
    if drive is not None:
        k, d, f = drive
        dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        dr.CreateTypeAttr("force")
        dr.CreateStiffnessAttr(float(k * math.pi / 180.0))  # USD angular drives are per degree
        dr.CreateDampingAttr(float(d * math.pi / 180.0))
        dr.CreateMaxForceAttr(float(f))
        dr.CreateTargetPositionAttr(0.0)
    return j.GetPrim()


def build_blind(stage, path):
    """The blind on its stand under `path`, whose Xform sits on the table under the middle of the slats. Links are built
    with the slats level and the wand plumb; the embodiment sets the start pose through the joints. Returns the root
    prim."""
    from pxr import UsdPhysics

    root_prim = stage.GetPrimAtPath(path)
    b = _Builder(stage, path)
    frame = f"{path}/frame"
    fp = _xform(stage, frame)
    m = ROOT_MASS
    b.rigid(fp, m, (0.012, 0.0, 0.11), (m * (BOARD["y"] ** 2 + 0.35 ** 2) / 12, m * (0.18 ** 2 + 0.35 ** 2) / 12, m * (BOARD["y"] ** 2 + 0.18 ** 2) / 12),
            damping=0.05)
    UsdPhysics.ArticulationRootAPI.Apply(fp)
    _schema(fp, "PhysxArticulationAPI", **{"physxArticulation:solverPositionIterationCount": 32,
                                           "physxArticulation:solverVelocityIterationCount": 4,
                                           "physxArticulation:enabledSelfCollisions": True})
    b.box(f"{frame}/board", (0.0, 0.0, BOARD["t"] / 2), (BOARD["x"], BOARD["y"], BOARD["t"]), "ply", "wood")
    lx = POST["x"] + POST["s"] / 2 - LINTEL["x"] / 2  # the lintel's back flush with the posts' backs
    b.box(f"{frame}/lintel", (lx, 0.0, TOP_Z - LINTEL["t"] / 2), (LINTEL["x"], LINTEL["y"], LINTEL["t"]), "pine", "wood")
    for s, name in ((1, "post_l"), (-1, "post_r")):
        h = HEADRAIL_TOP - BOARD["t"]
        b.box(f"{frame}/{name}", (POST["x"], s * POST["y"], BOARD["t"] + h / 2), (POST["s"], POST["s"], h), "pine", "wood")
    b.box(f"{frame}/headrail", (0.0, 0.0, HEADRAIL_TOP - HEADRAIL["h"] / 2), (HEADRAIL["d"], WIDTH, HEADRAIL["h"]), "rail", "metal")
    b.cyl(f"{frame}/stem_boss", (HOOK[0], HOOK[1], HEADRAIL_BOTTOM + 0.004), 0.004, 0.008, "hook", None)

    # stem, hook (universal joint) and wand
    stem, cross, wand = f"{path}/stem", f"{path}/cross", f"{path}/wand"
    for link, mass in ((stem, 0.002), (cross, 0.001)):
        b.rigid(_xform(stage, link, HOOK), mass, (0.0, 0.0, 0.0), (1e-7, 1e-7, 1e-7), gravity=False, damping=0.0)
    b.cyl(f"{stem}/pin", (0.0, 0.0, HOOK_DROP / 2), 0.002, HOOK_DROP, "hook", None)
    wp = _xform(stage, wand, HOOK)
    L, wm = WAND["len"], WAND_MASS
    b.rigid(wp, wm, (0.0, 0.0, -L / 2), (wm * L * L / 12, wm * L * L / 12, wm * WAND["af"] ** 2 / 8), damping=0.0)
    b.hex_prism(f"{wand}/rod", WAND["af"], -L, -0.004, "wand", "plastic")
    b.cyl(f"{wand}/tip", (0.0, 0.0, -0.004), 0.0025, 0.008, "hook", None)
    _revolute(stage, f"{path}/j_stem", frame, stem, HOOK, "Z")
    swing = (0.0, SWING_DAMPING, 1.0)
    _revolute(stage, f"{path}/j_swingx", stem, cross, (0.0, 0.0, 0.0), "X", (-SWING_LIMIT_DEG, SWING_LIMIT_DEG), swing)
    _revolute(stage, f"{path}/j_swingy", cross, wand, (0.0, 0.0, 0.0), "Y", (-SWING_LIMIT_DEG, SWING_LIMIT_DEG), swing)

    # slats and bottom rail
    drive = (SLAT_DRIVE["stiffness"], SLAT_DRIVE["damping"], SLAT_DRIVE["max_force"])
    for name, z in zip(SLAT_DOFS, (*SLAT_Z, RAIL_Z)):
        link = f"{path}/{name}"
        lp = _xform(stage, link, (0.0, 0.0, z))
        if name == "rail":
            w, t, mass, look = RAIL["w"], RAIL["h"], RAIL_MASS, "rail"
        else:
            w, t, mass, look = SLAT["w"], SLAT["t"], SLAT_MASS, "slat"
        b.rigid(lp, mass, (0.0, 0.0, 0.0), (mass * SLAT["len"] ** 2 / 12, mass * w * w / 12, mass * SLAT["len"] ** 2 / 12),
                gravity=False, damping=0.0)
        b.box(f"{link}/body", (0.0, 0.0, 0.0), (w, SLAT["len"], t), look, "metal")
        _revolute(stage, f"{path}/j_{name}", frame, link, (0.0, 0.0, z), "Y", (-89.0, 89.0), drive)
    return root_prim
