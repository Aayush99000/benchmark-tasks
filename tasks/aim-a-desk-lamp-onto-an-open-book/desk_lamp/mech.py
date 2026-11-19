"""An Anglepoise Type 75 desk lamp, built true size from the maker's dimensioned drawing, and an open hardcover notebook
with a target dot. Created 11_16_26 (America/New_York).

Geometry and kinematics are NumPy only, so the task file and the tests use them without Isaac Sim; the USD builders
import pxr when called.

Sources (README "Where the numbers come from"): Anglepoise's Type 75 desk lamp technical sheets (US V4, UK V2.0): shade
5.7 in (14.5 cm) across and 7.5 in (19.05 cm) tall, base 7.7 in (19.5 cm) across, maximum horizontal reach 28 in from base
to shade, maximum vertical reach 35.4 in; net weight 3.1 kg (Anglepoise support page). The pivot layout is read off the
sheet's side-view drawing in its vector coordinates: pivot centres are drawn as small circles, and the scale comes from the
printed dimensions (7.90 pt per inch across, from the 12.2, 7.7 and 5.7 in dimensions; 8.34 pt per inch up, from the
20.5 and 7.5 in dimensions). Result: lower arm 30.1 cm and upper arm 30.4 cm between pivot centres, shoulder pivot 21.4 cm
above the desk and 2.6 cm behind the swivel axis, head knuckle 4.3 cm from the shade's axis and 4.8 cm below its top. The
arms laid out flat then reach 28.4 in from the base's axis to the shade's far rim, against the published 28 in.

Frames (m). The lamp is one articulation with a free root: the base (cast iron, resting on the desk) carries the swivel
knuckle on a vertical revolute joint; the lower arm, the upper arm and the head each turn on a horizontal revolute joint
whose axis is square to the arm's plane (all three axes parallel). In the arm's plane, r is the horizontal distance from the
swivel axis toward the head and z the height above the desk. The links are built in the drawing's pose (heading 0, i.e.
the head toward world +x; lower arm leaning back at TH1_REST; upper arm level; shade axis straight down), where every
joint angle is zero.

Angles in the arm's plane: th1 is the lower arm's elevation measured from +r (it leans back, so th1 > 90 deg), th2 the
upper arm's elevation from +r (pointing from the elbow toward the head), gam the shade's tilt from straight down
(positive swings the mouth toward +r). The joints are relative: q_sh = th1 - TH1_REST, q_el = (th2 - th1) - (0 - TH1_REST),
q_hd = gam - th2, each about the arm plane's -n axis (n = z x r_hat), so positive angles raise toward +r.

Springs: the real lamp's springs balance the arms in any pose (its sheet: "constant tension spring technology"); spring
rates and anchor points are not published. The simulation stands in an ideal balance: the moving links keep their mass
and inertia but carry no weight (gravity off on them), and Coulomb joint friction holds the pose. The base carries the
whole 3.1 kg on the desk.
"""

import math

import numpy as np

IN = 0.0254

# -- the lamp (drawing pose, arm-plane coordinates r, z in m) ---------------------------------------------------------
BASE_R = 0.0975  # base 19.5 cm across (sheet)
BASE_H = 0.018  # the cast-iron cone's edge-to-top height, from the drawing
POST = dict(r=0.016, h=(BASE_H, 0.095))  # the base's post and swivel collar, from the drawing
SHOULDER = (-0.026, 0.214)  # lower arm's pivot: 2.6 cm behind the swivel axis, 21.4 cm up
L1 = 0.301  # lower arm, pivot to pivot
L2 = 0.304  # upper arm, elbow pivot to head knuckle
TH1_REST = math.radians(116.0)  # lower arm's elevation in the drawing (leans back)
KNUCKLE_TO_AXIS = 0.043  # head knuckle to the shade's axis
KNUCKLE_BELOW_TOP = 0.048
SHADE_LEN = 7.5 * IN  # 19.05 cm
SHADE_MOUTH_R = 5.7 * IN / 2  # 7.24 cm
SHADE_CAP_R = 0.031  # top cap 6.2 cm across, from the drawing
SHADE_CAP_LEN = 0.083  # straight cap before the bell flares, from the drawing
MOUTH0 = np.array((KNUCKLE_TO_AXIS, -(SHADE_LEN - KNUCKLE_BELOW_TOP)))  # mouth centre from the knuckle, drawing pose
ROD_R = 0.0045  # arm rods 9 mm across (drawing)
ROD_GAP = 0.016  # the two rods of each arm, centre to centre across the arm's plane
ELBOW_DISC_R = 0.022  # the elbow's round cover plate

MASS = {"base": 3.1, "swivel": 0.15, "lower": 0.30, "upper": 0.15, "head": 0.30}  # kg; moving parts are estimates
# Coulomb joint friction (N m, PhysX static = dynamic effort). Estimates, set so that a push of about 3 N on the shade's rim
# turns the head, while the same push swings the arm about the elbow unless the arm is held (see the README).
FRICTION = {"swivel": 0.35, "shoulder": 0.60, "elbow": 0.45, "head": 0.30}
DOFS = ("swivel", "shoulder", "elbow", "head")

# -- the notebook ---------------------------------------------------------------------------------------------------
# Moleskine Classic Notebook, Large, hard cover: 13 x 21 cm, 240 pages, about 0.33 kg; open flat it is 26 x 21 cm.
BOOK = dict(size=(0.210, 0.260, 0.009), mass=0.33)  # x (along the spine) by y (across the open spread) by thickness
DOT_R = 0.0095  # a 19 mm round sticker dot on the right-hand page
DOT_IN_BOOK = np.array((0.0, -0.065))  # the dot's offset from the book's centre: the middle of the right-hand page


def unit(a):
    return np.array((math.cos(a), math.sin(a)))


def rot2(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array((c * v[0] - s * v[1], s * v[0] + c * v[1]))


def plane_points(th1, th2, gam):
    """Arm-plane (r, z) points: shoulder S, elbow E, knuckle K, the mouth's centre M, the shade's axis (unit, toward
    the page), and the shade's top centre T."""
    s = np.array(SHOULDER)
    e = s + L1 * unit(th1)
    k = e + L2 * unit(th2)
    m = k + rot2(MOUTH0, gam)
    axis = rot2(np.array((0.0, -1.0)), gam)
    t = m - SHADE_LEN * axis
    return dict(S=s, E=e, K=k, M=m, axis=axis, T=t)


def to_world(base_xy, psi, rz):
    """An arm-plane point (r, z) at swivel heading psi (rad) as a world point (x, y, z above the desk)."""
    u = unit(psi)
    return np.array((base_xy[0] + rz[0] * u[0], base_xy[1] + rz[0] * u[1], rz[1]))


def fk(base_xy, psi, th1, th2, gam):
    """World points (z above the desk) and the shade's axis for a lamp pose."""
    p = plane_points(th1, th2, gam)
    out = {k: to_world(base_xy, psi, v) for k, v in p.items() if k != "axis"}
    u = unit(psi)
    out["axis"] = np.array((p["axis"][0] * u[0], p["axis"][0] * u[1], p["axis"][1]))
    return out


def beam_point(mouth, axis, page_z):
    """Where the shade's axis meets the horizontal plane at page_z; None if the shade does not face down onto it."""
    if axis[2] > -0.05:  # within about 3 deg of level or facing up: no spot on the page
        return None
    t = (page_z - mouth[2]) / axis[2]
    if t < 0.0:
        return None
    return mouth + t * axis


def arm_ik(k_rz):
    """(th1, th2) that put the head knuckle at arm-plane point k_rz, with the elbow up and back (the lamp's own way)."""
    s = np.array(SHOULDER)
    d = k_rz - s
    dist = float(np.linalg.norm(d))
    if not abs(L1 - L2) <= dist <= L1 + L2:
        raise ValueError(f"knuckle out of reach: {dist:.3f} m from the shoulder")
    a = math.atan2(d[1], d[0])
    b = math.acos((L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist))
    th1 = a + b  # elbow above the shoulder-knuckle line
    e = s + L1 * unit(th1)
    kk = k_rz - e
    return th1, math.atan2(kk[1], kk[0])


def aim_pose(base_xy, spot_xy, page_z, mouth_h, gam):
    """Lamp pose (psi, th1, th2, gam) with the shade's mouth centre mouth_h above the desk, tilted gam, its axis meeting
    the page at spot_xy."""
    d = np.asarray(spot_xy, float) - np.asarray(base_xy, float)
    psi = math.atan2(d[1], d[0])
    r_spot = float(np.linalg.norm(d))
    axis = rot2(np.array((0.0, -1.0)), gam)
    t = (page_z - mouth_h) / axis[1]
    m = np.array((r_spot, page_z)) - t * axis
    k = m - rot2(MOUTH0, gam)
    th1, th2 = arm_ik(k)
    return psi, th1, th2, gam


def joints(psi, th1, th2, gam):
    """Relative joint angles (rad) in the articulation for a lamp pose: swivel, shoulder, elbow, head."""
    return {"swivel": psi, "shoulder": th1 - TH1_REST, "elbow": (th2 - th1) + TH1_REST, "head": gam - th2}


def pose_from_joints(q):
    psi = q["swivel"]
    th1 = q["shoulder"] + TH1_REST
    th2 = q["elbow"] + th1 - TH1_REST
    return psi, th1, th2, q["head"] + th2


def skeleton(base_xy, psi, th1, th2, gam, n=6):
    """Points along the lamp's moving parts (lower arm, upper arm, shade axis) for hand-clearance tests."""
    w = fk(base_xy, psi, th1, th2, gam)
    pts = []
    for a, b in (("S", "E"), ("E", "K"), ("T", "M")):
        for k in range(n + 1):
            pts.append(w[a] + (w[b] - w[a]) * k / n)
    return np.array(pts)


# -- USD ----------------------------------------------------------------------------------------------------------
COLORS = {  # sRGB base colour, metallic, roughness
    "lamp": ((0.07, 0.07, 0.08), 0.1, 0.45), "chrome": ((0.78, 0.79, 0.80), 1.0, 0.2), "spring": ((0.55, 0.56, 0.58), 1.0, 0.35),
    "inner": ((0.93, 0.93, 0.90), 0.0, 0.5), "bulb": ((1.0, 0.96, 0.85), 0.0, 0.3),
    "cover": ((0.08, 0.10, 0.16), 0.0, 0.6), "page": ((0.96, 0.94, 0.86), 0.0, 0.85), "dot": ((0.85, 0.08, 0.08), 0.0, 0.5),
    "edge": ((0.90, 0.88, 0.80), 0.0, 0.9),
}
MU = {"metal": 0.5, "paper": 0.5, "base": 0.6}


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
            color, metal, rough = COLORS[name]
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Looks/{name}")
            sh = UsdShade.Shader.Define(self.stage, f"{self.root}/Looks/{name}/shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*_lin(color)))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            if name == "bulb":
                sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(4.0, 3.6, 2.8))
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
            _schema(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": 0.002, "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        return g

    def box(self, path, center, size, look, collide=None, rot_y_deg=0.0):
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if rot_y_deg:
            c.AddRotateYOp().Set(float(rot_y_deg))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        return self._finish(c, look, collide)

    def cyl(self, path, center, r, h, look, collide=None, axis="Z", rot_y_deg=0.0):
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if rot_y_deg:
            c.AddRotateYOp().Set(float(rot_y_deg))
        return self._finish(c, look, collide)

    def rod(self, path, a, b, r, look, y=0.0, collide="metal"):
        """A round rod in the arm's plane (x = r, z) from a to b, at y across the plane."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        mid = (a + b) / 2
        d = b - a
        ang = math.degrees(math.atan2(d[0], d[1]))  # a Z cylinder turned about y onto the rod's direction
        return self.cyl(path, (mid[0], y, mid[1]), r, float(np.linalg.norm(d)), look, collide, "Z", ang)

    def rigid(self, prim, mass, com, inertia, gravity=True):
        from pxr import Gf, UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim)
        _schema(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": 32,
                                              "physxRigidBody:solverVelocityIterationCount": 4,
                                              "physxRigidBody:maxDepenetrationVelocity": 0.2,
                                              "physxRigidBody:disableGravity": not gravity,
                                              "physxRigidBody:angularDamping": 0.05, "physxRigidBody:linearDamping": 0.05})
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))


def _xform(stage, path, pos=(0.0, 0.0, 0.0)):
    from pxr import Gf, UsdGeom

    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    return x.GetPrim()


def _joint(stage, path, body0, body1, pos0, pos1, axis):
    """Revolute joint about `axis` ("Z" or "Y") between two links whose frames are not rotated relative to each other in
    the drawing pose. A Y joint's positive angle turns +x toward -z, so the builder flips nothing: the task's angles are
    about -Y (see the module docstring) and the embodiment negates them."""
    from pxr import Gf, Sdf, UsdPhysics

    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, pos1)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    return j.GetPrim()


def build_lamp(stage, path):
    """The lamp under `path`, whose Xform stands on the desk at the base's centre with heading 0 (the embodiment places
    it and sets the start pose through the joints). Link frames: base and swivel at the base's centre on the desk; lower
    arm at the shoulder pivot; upper arm at the elbow pivot; head at the knuckle. Returns the root prim."""
    from pxr import UsdPhysics

    root_prim = stage.GetPrimAtPath(path)
    b = _Builder(stage, path)
    p = plane_points(TH1_REST, 0.0, 0.0)
    S, E, K = p["S"], p["E"], p["K"]

    base = f"{path}/base"
    bp = _xform(stage, base)
    b.rigid(bp, MASS["base"], (0.0, 0.0, 0.008), (0.008, 0.008, 0.015))
    UsdPhysics.ArticulationRootAPI.Apply(bp)  # the base is the articulation's (free) root: the arm hangs from it
    _schema(bp, "PhysxArticulationAPI", **{"physxArticulation:solverPositionIterationCount": 32,
                                           "physxArticulation:solverVelocityIterationCount": 4,
                                           "physxArticulation:enabledSelfCollisions": False})
    b.cyl(f"{base}/foot", (0.0, 0.0, 0.005), BASE_R, 0.010, "lamp", "base")
    b.cyl(f"{base}/cone", (0.0, 0.0, 0.014), 0.055, 0.008, "lamp", "base")
    b.cyl(f"{base}/boss", (0.0, 0.0, 0.021), 0.025, 0.010, "lamp", None)

    sw = f"{path}/swivel"
    sp = _xform(stage, sw)
    b.rigid(sp, MASS["swivel"], (-0.01, 0.0, 0.12), (4e-4, 4e-4, 1e-4), gravity=False)
    b.cyl(f"{sw}/post", (0.0, 0.0, sum(POST["h"]) / 2), POST["r"], POST["h"][1] - POST["h"][0], "chrome", "metal")
    b.rod(f"{sw}/yoke", (0.0, POST["h"][1]), S, 0.008, "lamp", collide="metal")
    for k, y in enumerate((-0.012, 0.012)):  # the three balance springs, drawn (they act through the ideal balance)
        b.rod(f"{sw}/spring{k}", (0.0, 0.10), (S[0] + 0.012, S[1] - 0.01), 0.006, "spring", y=y, collide=None)
    b.rod(f"{sw}/spring2", (-0.004, 0.10), (S[0] - 0.012, S[1] - 0.012), 0.006, "spring", collide=None)
    _joint(stage, f"{path}/j_swivel", base, sw, (0.0, 0.0, POST["h"][0]), (0.0, 0.0, POST["h"][0]), "Z")

    lo = f"{path}/lower"
    lp = _xform(stage, lo, (S[0], 0.0, S[1]))
    rel = lambda v: np.asarray(v) - S  # noqa: E731
    mid = rel((S + E) / 2)
    b.rigid(lp, MASS["lower"], (mid[0], 0.0, mid[1]), (2.5e-3, 2.5e-3, 1e-4), gravity=False)
    for k, y in enumerate((-ROD_GAP / 2, ROD_GAP / 2)):
        b.rod(f"{lo}/rod{k}", rel(S), rel(E), ROD_R, "lamp", y=y)
    b.cyl(f"{lo}/pin", (0.0, 0.0, 0.0), 0.007, ROD_GAP + 0.012, "chrome", None, "Y")
    _joint(stage, f"{path}/j_shoulder", sw, lo, (S[0], 0.0, S[1]), (0.0, 0.0, 0.0), "Y")

    up = f"{path}/upper"
    upp = _xform(stage, up, (E[0], 0.0, E[1]))
    rel = lambda v: np.asarray(v) - E  # noqa: E731
    mid = rel((E + K) / 2)
    b.rigid(upp, MASS["upper"], (mid[0], 0.0, mid[1]), (1.2e-3, 1.2e-3, 5e-5), gravity=False)
    for k, y in enumerate((-ROD_GAP / 2, ROD_GAP / 2)):
        b.rod(f"{up}/rod{k}", rel(E), rel(K), ROD_R, "lamp", y=y)
    b.cyl(f"{up}/disc", (0.0, 0.0, 0.0), ELBOW_DISC_R, ROD_GAP + 0.010, "lamp", "metal", "Y")
    _joint(stage, f"{path}/j_elbow", lo, up, (E[0] - S[0], 0.0, E[1] - S[1]), (0.0, 0.0, 0.0), "Y")

    hd = f"{path}/head"
    hp = _xform(stage, hd, (K[0], 0.0, K[1]))
    ax_r = KNUCKLE_TO_AXIS
    top = KNUCKLE_BELOW_TOP
    b.rigid(hp, MASS["head"], (ax_r, 0.0, top - SHADE_LEN * 0.45), (8e-4, 8e-4, 4e-4), gravity=False)
    b.cyl(f"{hd}/knuckle", (0.0, 0.0, 0.0), 0.011, 0.026, "chrome", "metal", "Y")
    b.box(f"{hd}/bracket", (ax_r / 2, 0.0, 0.0), (ax_r, 0.012, 0.016), "lamp", "metal")
    b.cyl(f"{hd}/cap", (ax_r, 0.0, top - SHADE_CAP_LEN / 2), SHADE_CAP_R, SHADE_CAP_LEN, "lamp", "metal")
    bell = SHADE_LEN - SHADE_CAP_LEN
    radii = (0.036, 0.045, 0.056, 0.066, SHADE_MOUTH_R)  # the bell, flaring to the mouth (drawing)
    for k, rr in enumerate(radii):
        z1 = top - SHADE_CAP_LEN - bell * k / len(radii)
        b.cyl(f"{hd}/bell{k}", (ax_r, 0.0, z1 - bell / len(radii) / 2), rr, bell / len(radii), "lamp", "metal")
    mouth_z = top - SHADE_LEN
    b.cyl(f"{hd}/inside", (ax_r, 0.0, mouth_z + 0.0006), SHADE_MOUTH_R - 0.003, 0.001, "inner", None)
    b.cyl(f"{hd}/bulb", (ax_r, 0.0, mouth_z + 0.0016), 0.030, 0.001, "bulb", None)
    _joint(stage, f"{path}/j_head", up, hd, (K[0] - E[0], 0.0, K[1] - E[1]), (0.0, 0.0, 0.0), "Y")
    return root_prim


def build_book(stage, path):
    """The open notebook under `path` (its Xform at the book's centre on the desk): one loose rigid body."""
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    sx, sy, h = BOOK["size"]
    b.rigid(prim, BOOK["mass"], (0.0, 0.0, h / 2), (BOOK["mass"] * (sy * sy) / 12, BOOK["mass"] * (sx * sx) / 12, BOOK["mass"] * (sx * sx + sy * sy) / 12))
    b.box(f"{path}/cover", (0.0, 0.0, 0.0015), (sx + 0.006, sy + 0.006, 0.003), "cover", "paper")
    b.box(f"{path}/pages", (0.0, 0.0, 0.003 + (h - 0.003) / 2), (sx, sy, h - 0.003), "page", "paper")
    b.box(f"{path}/gutter", (0.0, 0.0, h + 0.0002), (sx, 0.003, 0.0004), "edge", None)
    b.cyl(f"{path}/dot", (DOT_IN_BOOK[0], DOT_IN_BOOK[1], h + 0.0003), DOT_R, 0.0005, "dot", None)
    return prim
