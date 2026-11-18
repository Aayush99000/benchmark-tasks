"""USD builders for the dispenser (static body, the paddle-wheel articulation and the cereal) and the bowl. They import
pxr when called, inside Isaac Sim. Created 11_16_26 (America/New_York).

The dispenser's housing, funnel, canister, column and foot are static colliders. The wheel is a fixed-base
articulation: an anchor fixed to the world, the wheel (hub, six rigid inner vanes, shaft and knob with its grip bar) on
a revolute joint about the D frame's x axis with a static friction torque, and six soft vane tips, each on its own
revolute hinge with a spring. The cereal pieces are free rigid bodies under the dispenser's prim; they are not scene
objects of the harness, and the embodiment reads them through one batched rigid-body view.
"""

import math
import os

import numpy as np

from . import geometry as G

COLORS = {  # sRGB base colour, metallic, roughness, opacity
    "chrome": ((0.78, 0.79, 0.81), 1.0, 0.25, 1.0),
    "black": ((0.06, 0.06, 0.07), 0.0, 0.5, 1.0),
    "clear": ((0.92, 0.95, 0.97), 0.0, 0.05, 0.22),
    "smoke": ((0.25, 0.27, 0.30), 0.0, 0.2, 0.55),
    "wheel": ((0.85, 0.85, 0.83), 0.0, 0.5, 1.0),
    "tip": ((0.30, 0.55, 0.85), 0.0, 0.7, 1.0),
    "knob": ((0.08, 0.08, 0.09), 0.1, 0.4, 1.0),
    "cereal": ((0.80, 0.58, 0.28), 0.0, 0.85, 1.0),
    "bowl": ((0.40, 0.62, 0.38), 0.0, 0.45, 1.0),
    "wall": ((0.86, 0.85, 0.82), 0.0, 0.8, 1.0),
}
# friction (static, dynamic) and restitution; PhysX averages the pair's values
MU = {"plastic": (0.35, 0.30, 0.15), "cereal": (0.45, 0.40, 0.15), "rubber": (0.9, 0.8, 0.05), "pp": (0.35, 0.30, 0.15)}
OFFSET_FIXED = 0.0005  # contact offset of housing and wheel colliders: pairs of them never meet across the 1.5 mm gaps
OFFSET_PIECE = 0.0015


def _lin(c):
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


def _schema(prim, schema, **attrs):
    from pxr import Sdf

    if ":" in schema:  # a multiple-apply schema instance, e.g. PhysxJointAxisAPI:angular
        prim.AddAppliedSchema(schema)
    else:
        try:
            ok = prim.ApplyAPI(schema)
        except Exception:
            ok = False
        if not ok:
            prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


class Builder:
    def __init__(self, stage, root):
        self.stage, self.root = stage, root
        self.looks, self.mats = {}, {}

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

    def material(self, name):
        from pxr import UsdPhysics, UsdShade

        if name not in self.mats:
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Physics/{name}")
            pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            s, d, e = MU[name]
            pm.CreateStaticFrictionAttr(s)
            pm.CreateDynamicFrictionAttr(d)
            pm.CreateRestitutionAttr(e)
            self.mats[name] = mat
        return self.mats[name]

    def finish(self, g, look=None, collide=None, offset=OFFSET_FIXED, visible=True):
        from pxr import UsdGeom, UsdPhysics, UsdShade

        prim = g.GetPrim()
        if look:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            _schema(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": float(offset), "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.material(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        if not visible:
            UsdGeom.Imageable(prim).MakeInvisible()
        return g

    def box(self, path, center, size, look=None, collide=None, rot_x=0.0, offset=OFFSET_FIXED, visible=True):
        """A box of edge lengths ``size`` centred at ``center``, turned rot_x degrees about the x axis."""
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if rot_x:
            c.AddRotateXOp().Set(float(rot_x))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        return self.finish(c, look, collide, offset, visible)

    def cyl(self, path, center, r, h, look=None, collide=None, axis="Z", offset=OFFSET_FIXED):
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        return self.finish(c, look, collide, offset)

    def hull(self, path, verts, collide, offset=OFFSET_FIXED, look=None):
        """A convex collider from its vertices (cooked by PhysX as their convex hull); visible with ``look``."""
        from pxr import Gf, UsdGeom, UsdPhysics, Vt

        m = UsdGeom.Mesh.Define(self.stage, path)
        verts = np.asarray(verts, float)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, v)) for v in verts]))
        faces = _hull_faces(verts)
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
        m.CreateSubdivisionSchemeAttr("none")
        self.finish(m, look, collide, offset, visible=look is not None)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        return m

    def mesh(self, path, pts, quads, look, double=True):
        from pxr import Gf, UsdGeom, Vt

        m = UsdGeom.Mesh.Define(self.stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([len(q) for q in quads]))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for q in quads for i in q]))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(bool(double))
        return self.finish(m, look)

    def rigid(self, prim, mass, iters=16, depen=0.5, ccd=False, sleep=None):
        from pxr import UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim)
        attrs = {"physxRigidBody:solverPositionIterationCount": iters, "physxRigidBody:solverVelocityIterationCount": 1,
                 "physxRigidBody:maxDepenetrationVelocity": float(depen), "physxRigidBody:enableCCD": bool(ccd)}
        if sleep is not None:
            attrs["physxRigidBody:sleepThreshold"] = float(sleep)
        _schema(prim, "PhysxRigidBodyAPI", **attrs)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))


_FACES = {}


def _hull_faces(v):
    """Triangles of the convex hull of a small point set (brute force, cached by the point set)."""
    key = np.round(v, 7).tobytes()
    if key not in _FACES:
        _FACES[key] = _hull_faces_uncached(v)
    return _FACES[key]


def _hull_faces_uncached(v):
    n = len(v)
    faces = []
    c = v.mean(0)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(v[j] - v[i], v[k] - v[i])
                ln = np.linalg.norm(nrm)
                if ln < 1e-12:
                    continue
                nrm /= ln
                d = (v - v[i]) @ nrm
                if (d <= 1e-9).all() or (d >= -1e-9).all():
                    if (v[i] - c) @ nrm < 0:
                        faces.append((i, k, j))
                    else:
                        faces.append((i, j, k))
    return faces or [(0, 1, 2)]


def _torus(r_major, r_minor, nu=14, nv=6):
    pts, quads = [], []
    for i in range(nu):
        a = 2 * math.pi * i / nu
        for j in range(nv):
            b = 2 * math.pi * j / nv
            r = r_major + r_minor * math.cos(b)
            pts.append((r * math.cos(a), r * math.sin(a), r_minor * 1.25 * math.sin(b)))
    for i in range(nu):
        for j in range(nv):
            i2, j2 = (i + 1) % nu, (j + 1) % nv
            quads.append((i * nv + j, i2 * nv + j, i2 * nv + j2, i * nv + j2))
    return pts, quads


def _xform(stage, path, t=(0.0, 0.0, 0.0), quat_wxyz=None):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, t)))
    if quat_wxyz is not None:
        w, a, b, c = map(float, quat_wxyz)
        x.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(a, b, c)))
    return x.GetPrim()


def _quat_x(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), math.sin(h), 0.0, 0.0)


def build_dispenser(stage, path):
    """The static dispenser, its wheel articulation and its cereal, under ``path`` (the D frame)."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    b = Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    # mounting board (an upright panel standing on the counter), the dispenser's back plate and the bracket
    bd = G.BOARD
    b.box(f"{path}/board", ((bd["x0"] + bd["x1"]) / 2, 0.0, bd["h"] / 2), (bd["x1"] - bd["x0"], bd["w"], bd["h"]), "wall", "plastic")
    bk = G.BACK_PLATE
    zt = G.CANISTER_TOP + G.LID_T
    b.box(f"{path}/back_plate", ((bk["x0"] + bk["x1"]) / 2, 0.0, (bk["z0"] + zt) / 2), (bk["x1"] - bk["x0"], bk["w"], zt - bk["z0"]), "black", "plastic")
    xb = -G.X_END - G.T_SHELL
    b.box(f"{path}/bracket", ((bk["x1"] + xb) / 2, 0.0, G.WHEEL_Z), (xb - bk["x1"] + 0.002, 0.05, 0.05), "black", "plastic")
    xc0, xc1 = bk["x1"], -(G.CANISTER_R + 0.003)  # a strap from the back plate round the canister's back
    b.box(f"{path}/collar", ((xc0 + xc1) / 2, 0.0, G.FUNNEL_Z1 + 0.03), (xc1 - xc0, 0.04, 0.02), "black", "plastic")
    # wheel chamber: shell pieces round the bore (open at the inlet and outlet) and two end plates
    for i, v in enumerate(G.chamber_pieces()):
        b.hull(f"{path}/chamber_{i:02d}", v, "plastic")
    hs = G.R_BORE + G.T_SHELL
    for k, sx in enumerate((1, -1)):
        b.box(f"{path}/end_{k}", (sx * (G.X_END + G.T_SHELL / 2), 0.0, G.WHEEL_Z), (G.T_SHELL, 2 * hs, 2 * hs), "smoke", "plastic")
    # visible housing skin round the chamber
    b.cyl(f"{path}/v_housing", (0.0, 0.0, G.WHEEL_Z), hs + 0.001, 2 * G.X_END, "smoke", axis="X")
    # spout: a short rectangular chute from the outlet down to the mouth
    zt, zb = G.OUTLET_Z - 0.002, G.SPOUT_Z  # stops below the vane tips' sweep
    for k, sy in enumerate((1, -1)):
        b.box(f"{path}/spout_side_{k}", (0.0, sy * (G.OUTLET_Y + 0.0015), (zt + zb) / 2), (2 * G.X_END + 2 * G.T_SHELL, 0.003, zt - zb), "smoke", "plastic")
    for k, sx in enumerate((1, -1)):
        b.box(f"{path}/spout_end_{k}", (sx * (G.X_END + 0.0015), 0.0, (zt + zb) / 2), (0.003, 2 * G.OUTLET_Y + 0.006, zt - zb), "smoke", "plastic")
    # funnel and clear canister with its lid
    pieces, _, _ = G.funnel_pieces()
    for i, v in enumerate(pieces):
        b.hull(f"{path}/funnel_{i}", v, "plastic", look="clear")
    for i, v in enumerate(G.canister_pieces()):
        b.hull(f"{path}/canister_{i:02d}", v, "plastic")
    b.cyl(f"{path}/v_canister", (0.0, 0.0, (G.FUNNEL_Z1 + G.CANISTER_TOP) / 2), G.CANISTER_R + 0.0015, G.CANISTER_TOP - G.FUNNEL_Z1, "clear")
    b.cyl(f"{path}/lid", (0.0, 0.0, G.CANISTER_TOP + G.LID_T / 2), G.CANISTER_R + 0.006, G.LID_T, "black", "plastic")

    _build_wheel(stage, b, f"{path}/Wheel")
    _build_cereal(stage, b, f"{path}/Cereal")
    return prim


def _build_wheel(stage, b, root):
    from pxr import Gf, PhysxSchema, UsdPhysics

    _xform(stage, root)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(root))
    _schema(stage.GetPrimAtPath(root), "PhysxArticulationAPI", **{"physxArticulation:enabledSelfCollisions": False,
                                                                 "physxArticulation:solverPositionIterationCount": 16,
                                                                 "physxArticulation:solverVelocityIterationCount": 1,
                                                                 "physxArticulation:sleepThreshold": 0.0,
                                                                 "physxArticulation:stabilizationThreshold": 0.0})
    anchor = _xform(stage, f"{root}/anchor", (0.0, 0.0, G.WHEEL_Z))
    b.rigid(anchor, 0.05)
    fix = UsdPhysics.FixedJoint.Define(stage, f"{root}/fix")
    fix.CreateBody1Rel().SetTargets([anchor.GetPath()])
    from pxr import Usd, UsdGeom

    world = UsdGeom.Xformable(anchor).ComputeLocalToWorldTransform(Usd.TimeCode.Default())  # body0 is the world
    t = world.ExtractTranslation()
    r = world.ExtractRotationQuat()
    fix.CreateLocalPos0Attr().Set(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
    fix.CreateLocalRot0Attr().Set(Gf.Quatf(float(r.GetReal()), Gf.Vec3f(*map(float, r.GetImaginary()))))
    fix.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    wheel = _xform(stage, f"{root}/wheel", (0.0, 0.0, G.WHEEL_Z))
    b.rigid(wheel, G.WHEEL_MASS + G.KNOB_MASS, iters=16)
    w = wheel.GetPath().pathString
    zc = -G.WHEEL_Z  # the wheel prim sits on the axis; geometry below is written in D coordinates and shifted
    b.cyl(f"{w}/hub", (0.0, 0.0, 0.0), G.R_HUB, G.L_WHEEL, "wheel", "plastic", axis="X")
    for i, (c, size, a) in enumerate(G.vane_boxes()):
        b.box(f"{w}/vane_{i}", (c[0], c[1], c[2] + zc), size, "wheel", "plastic", rot_x=a)
    b.cyl(f"{w}/v_shaft", (G.X_END + 0.004, 0.0, 0.0), 0.004, 0.012, "chrome", axis="X")
    kx = G.KNOB_X0 + G.KNOB["t"] / 2
    b.cyl(f"{w}/knob", (kx, 0.0, 0.0), G.KNOB["r"], G.KNOB["t"], "knob", "rubber", axis="X")
    b.box(f"{w}/bar", (G.BAR_X, 0.0, 0.0), (G.BAR["h"], G.BAR["len"], G.BAR["w"]), "knob", "rubber", rot_x=G.BAR_ANGLE0)

    rev = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/turn")
    rev.CreateBody0Rel().SetTargets([anchor.GetPath()])
    rev.CreateBody1Rel().SetTargets([wheel.GetPath()])
    rev.CreateAxisAttr("X")
    rev.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    rev.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    # The seals' drag: a velocity drive toward standstill whose torque saturates at the friction torque acts as
    # Coulomb friction (it holds the wheel against any load below WHEEL_FRICTION and resists turning with it).
    motor = os.environ.get("CEREAL_MOTOR")  # dev only: deg/s, turns the wheel with no robot
    drive = UsdPhysics.DriveAPI.Apply(rev.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(float(G.WHEEL_HOLD_DAMPING * math.pi / 180.0))  # per degree per second in USD
    drive.CreateTargetVelocityAttr(float(motor) if motor else 0.0)
    drive.CreateMaxForceAttr(5.0 if motor else float(G.WHEEL_FRICTION))

    for i, a in enumerate(G.VANE_ANGLES):
        hinge, c, size, _ = G.tip_box(a)
        tip = _xform(stage, f"{root}/tip_{i}", (0.0, hinge[1], hinge[2]))
        b.rigid(tip, G.TIP_MASS, iters=16)
        t = tip.GetPath().pathString
        b.box(f"{t}/blade", (0.0, c[1] - hinge[1], c[2] - hinge[2]), size, "tip", "rubber", rot_x=a)
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/hinge_{i}")
        h.CreateBody0Rel().SetTargets([wheel.GetPath()])
        h.CreateBody1Rel().SetTargets([tip.GetPath()])
        h.CreateAxisAttr("X")
        h.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, float(hinge[1]), float(hinge[2] - G.WHEEL_Z)))
        h.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        h.CreateLowerLimitAttr(-G.TIP_LIMIT)
        h.CreateUpperLimitAttr(G.TIP_LIMIT)
        d = UsdPhysics.DriveAPI.Apply(h.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateTargetPositionAttr(0.0)
        d.CreateStiffnessAttr(float(G.TIP_STIFFNESS * math.pi / 180.0))  # USD angular drive gains are per degree
        d.CreateDampingAttr(float(G.TIP_DAMPING * math.pi / 180.0))
        d.CreateMaxForceAttr(1.0)


def _build_cereal(stage, b, root):
    """The pieces: one shared hull and torus mesh, a rigid body per piece, at the starting lattice."""
    from pxr import Gf, UsdGeom

    _xform(stage, root)
    hull = G.piece_hull()
    pts, quads = _torus((G.PIECE_D + G.PIECE_HOLE) / 4, (G.PIECE_D - G.PIECE_HOLE) / 4)
    rng = np.random.default_rng(1)
    for i, p in enumerate(G.initial_pieces()):
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        ang = math.radians(rng.uniform(0, 25))
        q = (math.cos(ang / 2), *(math.sin(ang / 2) * ax))
        piece = _xform(stage, f"{root}/p_{i:03d}", p, q)
        b.rigid(piece, G.PIECE_MASS, iters=8, depen=0.3, sleep=5e-5)
        pp = piece.GetPath().pathString
        b.hull(f"{pp}/c", hull, "cereal", offset=OFFSET_PIECE)
        b.mesh(f"{pp}/v", pts, quads, "cereal", double=False)


def build_bowl(stage, path):
    """The bowl: a rigid shell (32 x 6 wall pieces and a floor disc) with its foot ring."""
    b = Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, G.BOWL["mass"], iters=16, depen=0.5, ccd=True)
    r0 = G.BOWL["r_foot"]
    floor = [(r0 * math.cos(2 * math.pi * j / 24), r0 * math.sin(2 * math.pi * j / 24), z) for j in range(24) for z in (G.BOWL["foot_h"], G.BOWL_FLOOR_Z)]
    b.hull(f"{path}/c_floor", floor, "pp")
    foot = [((r0 - 0.004 * k) * math.cos(2 * math.pi * j / 24), (r0 - 0.004 * k) * math.sin(2 * math.pi * j / 24), z) for j in range(24) for k in (0, 1) for z in (0.0, G.BOWL["foot_h"])]
    b.hull(f"{path}/c_foot", foot, "pp")
    for i, v in enumerate(G.bowl_wall_pieces()):
        b.hull(f"{path}/c_wall_{i:03d}", v, "pp")
    # visible shell: inner profile, rim, outer profile, foot
    prof = G.BOWL_PROFILE
    loop = [(0.0005, G.BOWL_FLOOR_Z)] + [tuple(p) for p in prof] + [(G.BOWL["r_rim"], G.BOWL["h"])]
    loop += [(p[0] + G.BOWL["t"], p[1]) for p in prof[::-1]] + [(r0, 0.0), (0.0005, 0.0)]
    n = 64
    k = len(loop)
    pts = [(r * math.cos(2 * math.pi * j / n), r * math.sin(2 * math.pi * j / n), z) for j in range(n) for r, z in loop]
    quads = [(j * k + i, ((j + 1) % n) * k + i, ((j + 1) % n) * k + (i + 1) % k, j * k + (i + 1) % k) for j in range(n) for i in range(k)]
    b.mesh(f"{path}/v_shell", pts, quads, "bowl")
    return prim
