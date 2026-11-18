"""The tumblers, their ice and straws, and the bus tub. The geometry at the top is NumPy only, so the task file and the
tests use it without Isaac Sim; the USD builders import pxr when called.

Frames (m):
- Tumbler: origin at the centre of its underside, z up its axis. A hollow tapered cup: a ring of NSEG flat wall panels
  (each a convex collider with the real wall thickness) on a solid floor disc. Nothing fills the inside.
- Ice cube: origin at its centre. Straw: origin at its middle, z along its length (a capsule).
- Bus tub: origin at the centre of its underside, x along its length, z up. A floor, four walls and the rolled rim as a
  flat lip round the top, the walls leaning out as the real tub's do; a loose body, not fixed to the table.
"""

import math

import numpy as np

# -- tumbler: a 16 oz stackable pebbled SAN restaurant tumbler --------------------------------------------------------
RIM_D = 0.0826  # outside diameter at the rim (3 1/4 in)
BASE_D = 0.0603  # outside diameter at the base (2 3/8 in)
GLASS_H = 0.1381  # height (5 7/16 in)
WALL = 0.0025  # wall thickness
FLOOR_T = 0.004  # floor thickness
GLASS_MASS = 0.085  # kg, empty
NSEG = 24  # flat wall panels round the tumbler
TAPER = math.atan((RIM_D - BASE_D) / 2 / GLASS_H)  # the wall's lean from the axis (4.6 degrees)


def r_out(z):
    """Outside radius at height z above the underside."""
    return BASE_D / 2 + (RIM_D - BASE_D) / 2 * z / GLASS_H


def r_in(z):
    """Inside radius at height z above the underside."""
    return r_out(z) - WALL


def _glass_mass_properties(n=400):
    """Centre of mass height and principal inertia (about the centre of mass) of the empty tumbler: a thin tapered
    shell plus the floor disc, of one density."""
    zs = (np.arange(n) + 0.5) * GLASS_H / n
    dz = GLASS_H / n
    rm = (r_out(zs) + r_in(zs)) / 2
    shell = 2 * math.pi * rm * WALL * dz / math.cos(TAPER)  # volume of each ring
    floor_r = r_in(FLOOR_T / 2)
    floor = math.pi * floor_r**2 * FLOOR_T
    rho = GLASS_MASS / (shell.sum() + floor)
    m_ring, m_floor = rho * shell, rho * floor
    com = (float((m_ring * zs).sum()) + m_floor * FLOOR_T / 2) / GLASS_MASS
    izz = float((m_ring * rm**2).sum()) + m_floor * floor_r**2 / 2
    ixx = float((m_ring * (rm**2 / 2 + (zs - com) ** 2)).sum()) + m_floor * (floor_r**2 / 4 + (FLOOR_T / 2 - com) ** 2)
    return com, (ixx, ixx, izz)


GLASS_COM_Z, GLASS_INERTIA = _glass_mass_properties()

# -- contents ---------------------------------------------------------------------------------------------------------
ICE = 0.0254  # edge of a reusable plastic ice cube (1 in)
ICE_MASS = 0.013  # kg
N_ICE = 3  # per tumbler
STRAW_L = 0.197  # 7 3/4 in jumbo straw
STRAW_R = 0.003  # 6 mm across
STRAW_MASS = 0.0013  # kg (paper)
CONTENT_MASS = N_ICE * ICE_MASS + STRAW_MASS
FULL_MASS = GLASS_MASS + CONTENT_MASS


def contents_layout(lean):
    """Where a tumbler's three ice cubes and its straw start, in the tumbler's frame: [(name suffix, position, rotation
    matrix)]. The cubes start one above another, a little apart, and fall into place when the simulation starts (two 1 in
    cubes do not lie flat side by side on the 55 mm floor); the straw stands on the floor beside them and leans on the rim
    toward the horizontal direction ``lean`` (rad, in the tumbler's frame)."""
    c, s = math.cos(lean), math.sin(lean)
    Rz = np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
    q = math.sqrt(0.5)
    R45 = np.array(((q, -q, 0.0), (q, q, 0.0), (0.0, 0.0, 1.0)))
    cubes = [((0.0, 0.009, FLOOR_T + ICE / 2 + 0.001), np.eye(3)), ((0.0, 0.009, FLOOR_T + 1.5 * ICE + 0.003), R45),
             ((0.0, 0.009, FLOOR_T + 2.5 * ICE + 0.005), np.eye(3))]
    out = [(f"i{k}", Rz @ np.array(p), Rz @ R) for k, (p, R) in enumerate(cubes)]
    # the straw: from the floor beside the cubes up to the rim, leaning toward +x
    z0 = FLOOR_T + STRAW_R + 0.001
    foot = np.array((-0.012, -0.019, z0))
    top_xy = np.array((r_in(GLASS_H) - STRAW_R - 0.002, -0.005))
    axis = np.array((top_xy[0] - foot[0], top_xy[1] - foot[1], GLASS_H - z0))
    axis /= np.linalg.norm(axis)
    mid = foot + axis * (STRAW_L / 2 - STRAW_R)
    x = np.cross((0.0, 1.0, 0.0), axis)
    x /= np.linalg.norm(x)
    R = np.column_stack((x, np.cross(axis, x), axis))
    out.append(("s", Rz @ mid, Rz @ R))
    return out


# -- bus tub: a 20 x 15 x 5 in polyethylene bus box ------------------------------------------------------------------
TUB_L, TUB_W, TUB_H = 0.506, 0.389, 0.125  # outside, over the rim (19 15/16 x 15 5/16 x 4 15/16 in)
TUB_LIP = 0.010  # the rolled rim, as a flat lip this wide round the top
TUB_LIP_T = 0.005
TUB_WALL = 0.003
TUB_FLOOR = 0.008  # the floor's top above the table: the inside is 4 5/8 in (117 mm) deep
TUB_MASS = 0.635  # kg (1.4 lb)
TUB_BOT = (0.432 / 2, 0.321 / 2)  # inside half-length and half-width at the floor (17 x 12 5/8 in)
TUB_TOP = (TUB_L / 2 - TUB_LIP - TUB_WALL, TUB_W / 2 - TUB_LIP - TUB_WALL)  # and at the rim: the walls lean out about 11 degrees
TUB_INERTIA = tuple(TUB_MASS / 12 * v for v in (TUB_W**2 + TUB_H**2, TUB_L**2 + TUB_H**2, TUB_L**2 + TUB_W**2))


def tub_inside(z):
    """Inside half-length and half-width of the tub at height z above its underside."""
    f = min(1.0, max(0.0, (z - TUB_FLOOR) / (TUB_H - TUB_FLOOR)))
    return TUB_BOT[0] + f * (TUB_TOP[0] - TUB_BOT[0]), TUB_BOT[1] + f * (TUB_TOP[1] - TUB_BOT[1])


MU = {"san": 0.4, "ice": 0.3, "paper": 0.4, "tub": 0.3}  # friction; PhysX averages the two bodies in contact
COLORS = {  # sRGB base colour, roughness, opacity
    "san": ((0.10, 0.28, 0.75), 0.25, 0.55), "ice": ((0.86, 0.93, 0.97), 0.15, 0.8), "straw": ((0.85, 0.10, 0.10), 0.6, 1.0),
    "tub": ((0.58, 0.60, 0.62), 0.45, 1.0),
}


# -- USD --------------------------------------------------------------------------------------------------------------
def _physx(prim, schema, **attrs):
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
            color, rough, opacity = COLORS[name]
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Looks/{name}")
            sh = UsdShade.Shader.Define(self.stage, f"{self.root}/Looks/{name}/shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*_lin(color)))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
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
            pm.CreateRestitutionAttr(0.0)
            self.phys[name] = mat
        return self.phys[name]

    def _finish(self, g, look, collide, offset=0.001, hull=False):
        from pxr import UsdPhysics, UsdShade

        prim = g.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            if hull:
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            _physx(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": float(offset), "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        return g

    def box(self, path, lo, hi, look, collide=None):
        from pxr import Gf, UsdGeom

        lo, hi = np.array(lo, float), np.array(hi, float)
        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
        return self._finish(c, look, collide)

    def hull(self, path, points, faces, look, collide=None):
        """A convex mesh from its corner points and faces (lists of point indices)."""
        from pxr import Gf, UsdGeom, Vt

        m = UsdGeom.Mesh.Define(self.stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in points]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        return self._finish(m, look, collide, hull=True)

    def capsule(self, path, r, length, look, collide=None):
        """A capsule along z, ``length`` end to end."""
        from pxr import UsdGeom

        c = UsdGeom.Capsule.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(length - 2 * r))
        return self._finish(c, look, collide, offset=0.0005)

    def rigid(self, prim, mass, com=None, inertia=None, iters=32, depen=1.0):
        from pxr import Gf, UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(False)
        _physx(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": iters, "physxRigidBody:solverVelocityIterationCount": 2,
                                             "physxRigidBody:maxDepenetrationVelocity": float(depen), "physxRigidBody:enableCCD": True,
                                             "physxRigidBody:sleepThreshold": 0.0})
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))


def wall_panel(k):
    """Corner points and faces of wall panel k of a tumbler: a tapered slab the full height of the wall."""
    a0, a1 = 2 * math.pi * k / NSEG, 2 * math.pi * (k + 1) / NSEG
    pts = [(r * math.cos(a), r * math.sin(a), z) for z in (0.0, GLASS_H) for r in (r_in(z), r_out(z)) for a in (a0, a1)]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return pts, faces


def floor_disc():
    """Corner points and faces of a tumbler's floor: an NSEG-sided prism inside the wall."""
    pts = [(r_in(z) * math.cos(2 * math.pi * k / NSEG), r_in(z) * math.sin(2 * math.pi * k / NSEG), z) for z in (0.0, FLOOR_T) for k in range(NSEG)]
    faces = [tuple(reversed(range(NSEG))), tuple(range(NSEG, 2 * NSEG))] + [(k, (k + 1) % NSEG, NSEG + (k + 1) % NSEG, NSEG + k) for k in range(NSEG)]
    return pts, faces


def build_glass(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, GLASS_MASS, com=(0.0, 0.0, GLASS_COM_Z), inertia=GLASS_INERTIA)
    for k in range(NSEG):
        b.hull(f"{path}/wall{k}", *wall_panel(k), "san", "san")
    b.hull(f"{path}/floor", *floor_disc(), "san", "san")
    return prim


def build_ice(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, ICE_MASS, inertia=(ICE_MASS * ICE**2 / 6,) * 3)
    h = ICE / 2
    b.box(f"{path}/cube", (-h, -h, -h), (h, h, h), "ice", "ice")
    return prim


def build_straw(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    it = STRAW_MASS * STRAW_L**2 / 12
    b.rigid(prim, STRAW_MASS, inertia=(it, it, max(STRAW_MASS * STRAW_R**2, 2e-8)))
    b.capsule(f"{path}/tube", STRAW_R, STRAW_L, "straw", "paper")
    return prim


_BOX_FACES = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]


def tub_wall(axis, sign):
    """Corner points of one leaning wall of the tub: ``axis`` 0 is an end wall (across x), 1 a side wall; ``sign`` its side."""
    pts = []
    for z, (ix, iy) in ((TUB_FLOOR, TUB_BOT), (TUB_H, TUB_TOP)):
        for t in (0.0, TUB_WALL):
            for s in (-1.0, 1.0):
                if axis == 0:
                    pts.append((sign * (ix + t), s * (iy + TUB_WALL), z))
                else:
                    pts.append((s * ix, sign * (iy + t), z))
    return pts, _BOX_FACES


def build_tub(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, TUB_MASS, com=(0.0, 0.0, TUB_H * 0.35), inertia=TUB_INERTIA)
    bx, by = TUB_BOT[0] + TUB_WALL, TUB_BOT[1] + TUB_WALL
    b.box(f"{path}/floor", (-bx, -by, 0.0), (bx, by, TUB_FLOOR), "tub", "tub")
    for name, axis, sign in (("wall_xn", 0, -1.0), ("wall_xp", 0, 1.0), ("wall_yn", 1, -1.0), ("wall_yp", 1, 1.0)):
        b.hull(f"{path}/{name}", *tub_wall(axis, sign), "tub", "tub")
    hx, hy = TUB_TOP[0], TUB_TOP[1]
    z0 = TUB_H - TUB_LIP_T
    b.box(f"{path}/lip_xn", (-TUB_L / 2, -TUB_W / 2, z0), (-hx, TUB_W / 2, TUB_H), "tub", "tub")
    b.box(f"{path}/lip_xp", (hx, -TUB_W / 2, z0), (TUB_L / 2, TUB_W / 2, TUB_H), "tub", "tub")
    b.box(f"{path}/lip_yn", (-hx, -TUB_W / 2, z0), (hx, -hy, TUB_H), "tub", "tub")
    b.box(f"{path}/lip_yp", (-hx, hy, z0), (hx, TUB_W / 2, TUB_H), "tub", "tub")
    return prim
