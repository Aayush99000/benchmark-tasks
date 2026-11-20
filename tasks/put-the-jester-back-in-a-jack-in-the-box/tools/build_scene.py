"""Regenerate ``rdb_jack_in_box/jack_in_box.usda`` from ``layout.py``.

Run from the task folder: ``python tools/build_scene.py``. Needs only NumPy.

Five rigid bodies joined by four joints, all one tree rooted on the loose box (no articulation, no loop):

- ``box``: base plate, four walls and the crank, loose on the table;
- ``lid`` on a hinge at the box's back top edge (0 to 105 degrees, a loose tin hinge), with the tongue and
  barb under its front edge;
- ``coil``: the lower part of the jester's spring, on a hinge at its foot that lets it bend over toward the
  front or the back against a bending spring;
- ``head``: the jester's head and the top of its sleeve, on a slide along the coil with the coil's spring;
- ``catch``: the sprung tin hook inside the front wall, on a short slide with a spring.

Every part a finger, the lid or the head can touch is a box, cylinder, sphere or one convex hull (the hook's
sloped tooth) with collision. The printed tin, the jester's face, cap, bells and ruff are drawn on top and
have no collider; they never stand outside the colliders by more than a few millimetres (README).
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_jack_in_box import fixed_scene as scene  # noqa: E402
from rdb_jack_in_box import layout as L  # noqa: E402

# name: colour (linear RGB), roughness, metallic, friction key (None: display only)
MATERIALS = {
    "tin_red": ((.62, .05, .04), .35, .5, "tin"), "tin_blue": ((.03, .10, .45), .35, .5, "tin"),
    "tin_yellow": ((.85, .62, .05), .35, .5, "tin"), "tin_green": ((.05, .35, .10), .35, .5, "tin"),
    "tin_base": ((.55, .55, .58), .4, .8, "base"), "steel": ((.70, .71, .74), .3, 1., "tin"),
    "wood_red": ((.55, .04, .03), .5, 0., "tin"),
    "skin": ((.95, .72, .58), .8, 0., "plush"), "plush_red": ((.70, .04, .06), .9, 0., "plush"),
    "plush_green": ((.06, .40, .12), .9, 0., "plush"), "plush_white": ((.90, .90, .88), .9, 0., "plush"),
    "plush_yellow": ((.90, .70, .08), .9, 0., "plush"), "black": ((.02, .02, .02), .5, 0., None),
    "bell": ((.90, .72, .20), .25, 1., None),
}
_M = "</Fixture/Materials/"
GROUND_LIFT = .0005                 # bodies are spawned this far above where they rest


def _fmt(v):
    return ", ".join(f"{float(c):.6g}" for c in v)


def _orient(quat_wxyz):
    return f"quatf xformOp:orient = ({_fmt(quat_wxyz)})\n" if quat_wxyz is not None else ""


def _prim(kind, name, material, centre, *, collide, scale=None, orient=None, extra="", indent=16):
    pad = " " * indent
    apis = ['"MaterialBindingAPI"']
    if collide:
        apis += ['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"']
    ops = ['"xformOp:translate"'] + (['"xformOp:orient"'] if orient is not None else []) + \
          (['"xformOp:scale"'] if scale is not None else [])
    text = (f'{pad}def {kind} "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {_M}{material}>\n")
    if collide:
        text += (f"{pad}    rel material:binding:physics = {_M}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = 0.001\n{pad}    float physxCollision:restOffset = 0\n")
    text += "".join(f"{pad}    {line}\n" for line in extra.splitlines())
    text += f"{pad}    double3 xformOp:translate = ({_fmt(centre)})\n"
    if orient is not None:
        text += f"{pad}    {_orient(orient)}"
    if scale is not None:
        text += f"{pad}    float3 xformOp:scale = ({_fmt(scale)})\n"
    return text + f"{pad}    uniform token[] xformOpOrder = [{', '.join(ops)}]\n{pad}}}\n\n"


def box(name, material, low, high, *, collide=True, **kw):
    low, high = np.asarray(low, float), np.asarray(high, float)
    return _prim("Cube", name, material, (low + high) / 2, collide=collide, scale=(high - low) / 2,
                 extra="double size = 2", **kw)


def cylinder(name, material, centre, radius, height, axis="Z", *, collide=True, **kw):
    return _prim("Cylinder", name, material, centre, collide=collide,
                 extra=f'uniform token axis = "{axis}"\ndouble height = {height:.6g}\ndouble radius = {radius:.6g}', **kw)


def sphere(name, material, centre, radius, *, collide=True, **kw):
    return _prim("Sphere", name, material, centre, collide=collide, extra=f"double radius = {radius:.6g}", **kw)


def cone(name, material, centre, radius, height, orient, **kw):
    return _prim("Cone", name, material, centre, collide=False, orient=orient,
                 extra=f'uniform token axis = "Z"\ndouble height = {height:.6g}\ndouble radius = {radius:.6g}', **kw)


def hull(name, material, points, indent=16):
    """A convex-hull collider from a prism of points (first half one end cap, second half the other)."""
    pad = " " * indent
    n = len(points) // 2
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return (f'{pad}def Mesh "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {_M}{material}>\n{pad}    rel material:binding:physics = {_M}{material}>\n"
            f'{pad}    uniform token physics:approximation = "convexHull"\n'
            f"{pad}    float physxCollision:contactOffset = 0.001\n{pad}    float physxCollision:restOffset = 0\n"
            f"{pad}    int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"{pad}    int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"{pad}    point3f[] points = [{', '.join('(' + _fmt(p) + ')' for p in points)}]\n"
            f'{pad}    uniform token subdivisionScheme = "none"\n{pad}}}\n\n')


def wxyz(quat_xyzw):
    x, y, z, w = quat_xyzw
    return (w, x, y, z)


def axis_quat(direction):
    """(w, x, y, z) turning +z onto ``direction``."""
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    axis = np.cross((0., 0., 1.), d)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return (1., 0., 0., 0.) if d[2] > 0 else (0., 1., 0., 0.)
    angle = math.atan2(s, d[2])
    axis /= s
    return (math.cos(angle / 2), *(axis * math.sin(angle / 2)))


def body(name, mass, position, orient_xyzw, children, no_contact_with=()):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    pairs = ""
    if no_contact_with:
        apis.append('"PhysicsFilteredPairsAPI"')
        pairs = ("            rel physics:filteredPairs = [\n"
                 + ",\n".join(f"                </Fixture/Actors/{other}>" for other in no_contact_with) + "\n            ]\n")
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            + pairs +
            f"            float physics:mass = {mass:.6g}\n"
            "            bool physics:rigidBodyEnabled = 1\n"
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"            quatf xformOp:orient = ({_fmt(wxyz(orient_xyzw))})\n"
            f"            double3 xformOp:translate = ({_fmt(position)})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def joint(kind, name, body0, body1, anchor0, axis, low, high, *, stiffness, damping, target=0.):
    """A joint between two bodies, outside any articulation. Revolute: radians here; USD wants degrees and
    angular drive gains per degree. ``anchor0`` is in body0's frame; body1's origin sits on it."""
    angular = kind == "PhysicsRevoluteJoint"
    drive = "angular" if angular else "linear"
    k = stiffness * math.pi / 180 if angular else stiffness
    c = damping * math.pi / 180 if angular else damping
    to = (lambda v: math.degrees(v)) if angular else (lambda v: v)
    return (f'        def {kind} "{name}" (\n            prepend apiSchemas = ["PhysicsDriveAPI:{drive}"]\n        )\n        {{\n'
            f"            float drive:{drive}:physics:damping = {c:.6g}\n"
            f"            float drive:{drive}:physics:stiffness = {k:.6g}\n"
            f"            float drive:{drive}:physics:targetPosition = {to(target):.6g}\n"
            f"            float drive:{drive}:physics:targetVelocity = 0\n"
            f'            uniform token drive:{drive}:physics:type = "force"\n'
            f'            uniform token physics:axis = "{axis}"\n'
            f"            rel physics:body0 = </Fixture/Actors/{body0}>\n"
            f"            rel physics:body1 = </Fixture/Actors/{body1}>\n"
            "            bool physics:collisionEnabled = 0\n"
            "            uniform bool physics:excludeFromArticulation = 1\n"
            f"            point3f physics:localPos0 = ({_fmt(anchor0)})\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            "            quatf physics:localRot0 = (1, 0, 0, 0)\n"
            "            quatf physics:localRot1 = (1, 0, 0, 0)\n"
            f"            float physics:lowerLimit = {to(low):.6g}\n"
            f"            float physics:upperLimit = {to(high):.6g}\n"
            "        }\n\n")


def material(name):
    color, roughness, metallic, key = MATERIALS[name]
    friction = L.FRICTION[key] if key else .5
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({_fmt(color)})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


# -- geometry, each in its body's frame ------------------------------------------------------------------------
def box_colliders():
    """(name, material, low, high) of the box's colliding boxes, box frame."""
    h, w, t = L.HALF, L.WALL, L.FLOOR_T
    return [("base", "tin_base", (-h, -h, 0.), (h, h, t)),
            ("wall_front", "tin_red", (-h, -h, t), (-h + w, h, L.RIM_Z)),
            ("wall_back", "tin_red", (h - w, -h, t), (h, h, L.RIM_Z)),
            ("wall_left", "tin_blue", (-h + w, h - w, t), (h - w, h, L.RIM_Z)),
            ("wall_right", "tin_blue", (-h + w, -h, t), (h - w, -h + w, L.RIM_Z)),
            ("crank_arm", "steel", (-.004, -h - .020, L.CRANK_Z - .034), (.004, -h - .016, L.CRANK_Z + .004))]


def crank_round():
    return (cylinder("crank_shaft", "steel", (0., -L.HALF - .008, L.CRANK_Z), .004, .016, "Y")
            + cylinder("crank_knob", "wood_red", (0., -L.HALF - .025, L.CRANK_Z - .030), .006, .012, "Y"))


def box_prints():
    """Printed tin, drawn 0.4 mm proud of the walls; no collider."""
    h, e, out = L.HALF, .012, .0004
    text = ""
    for i, normal in enumerate(((-1., 0.), (1., 0.), (0., 1.), (0., -1.))):
        n = np.array((*normal, 0.))
        along = np.array((abs(normal[1]), abs(normal[0]), 0.))          # across the face, horizontal
        face = n * (h + out / 2)
        for name, material, half_width, z0, z1, depth in (("panel", "tin_yellow", h - e, .022, L.RIM_Z - .022, out),
                                                         ("band_low", "tin_blue", h, .008, .016, out),
                                                         ("band_high", "tin_blue", h, L.RIM_Z - .012, L.RIM_Z - .004, out)):
            half = along * half_width + np.abs(n) * depth / 2 + (0., 0., (z1 - z0) / 2)
            text += _prim("Cube", f"print_{name}_{i}", material, face + (0., 0., (z0 + z1) / 2), collide=False,
                          scale=half, extra="double size = 2")
        half = along * .013 + np.abs(n) * .0004 + (0., 0., .013)
        turn = (math.cos(math.pi / 8), *(n * math.sin(math.pi / 8)))
        text += _prim("Cube", f"print_diamond_{i}", "tin_green", n * (h + out) + (0., 0., L.RIM_Z / 2), collide=False,
                      scale=half, orient=turn, extra="double size = 2")
    for k, (x, y) in enumerate(((-h, -h), (-h, h), (h, -h), (h, h))):
        text += cylinder(f"seam_{k}", "steel", (x, y, (L.RIM_Z + L.FLOOR_T) / 2), .0022, L.RIM_Z - L.FLOOR_T, collide=False)
    text += cylinder("spring_cup", "steel", (0., 0., L.FLOOR_T + .006), .028, .012, collide=False)
    return text


def lid_colliders():
    """(name, material, low, high) in the lid's frame (origin on the hinge line, lid shut: x toward the front
    edge is negative, z up from the lid's mid-plane)."""
    hx, hz = L.HINGE
    t = L.LID_T / 2
    to = lambda x, z: (x - hx, z - hz)          # box frame (x, z) to lid frame
    tx0, tz0 = to(L.TONGUE_X[0], L.TONGUE_Z[0])
    tx1, tz1 = to(L.TONGUE_X[1], L.TONGUE_Z[1])
    bx0, bz0 = to(L.BARB_X[0], L.BARB_Z[0])
    bx1, bz1 = to(L.BARB_X[1], L.BARB_Z[1])
    return [("plate", "tin_blue", (-L.SIDE - .0005, -L.HALF - .0005, -t), (0., L.HALF + .0005, t)),
            ("tongue", "steel", (tx0, -L.CATCH_Y, tz0), (tx1, L.CATCH_Y, tz1 - t)),
            ("barb", "steel", (bx0, -L.CATCH_Y, bz0), (bx1, L.CATCH_Y, bz1))]


def lid_prints():
    t = L.LID_T / 2 + .0003
    c = -L.SIDE / 2
    return (box("print_top", "tin_yellow", (c - .052, -.052, t - .0003), (c + .052, .052, t), collide=False)
            + cylinder("print_star", "tin_red", (c, 0., t + .0002), .030, .0006, collide=False)
            + cylinder("rolled_front", "steel", (-L.SIDE, 0., -.0005), .0028, L.SIDE, "Y", collide=False)
            + cylinder("hinge_knuckle", "steel", (0., 0., 0.), .0030, L.SIDE - .010, "Y", collide=False))


def catch_points():
    """Profile of the hook in the box frame (x, z) at rest, extruded across y: a flat underside and a top that
    slopes down toward the back, so the barb coming down pushes it forward."""
    x0, x1 = L.CATCH_X
    zb, zt = L.CATCH_Z
    profile = [(x0, zb), (x1, zb), (x1, zb + .0015), (x1 - .0045, zt), (x0, zt)]
    return [(x, -L.CATCH_Y, z) for x, z in profile] + [(x, L.CATCH_Y, z) for x, z in profile]


def head_colliders():
    r, z0, z1 = L.HEAD_COLUMN
    return (sphere("ball", "skin", (0., 0., 0.), L.HEAD_R)
            + cylinder("sleeve", "plush_red", (0., 0., -(z0 + z1) / 2), r, z1 - z0))


def head_decor():
    """Face, cap with three drooping bells and a ruff, drawn on the head; no collider."""
    r = L.HEAD_R
    text = sphere("cap", "plush_red", (0., 0., .010), r + .0012, collide=False, scale=(1., 1., .78))
    for i, yaw in enumerate((math.pi / 2, -math.pi / 2, 0.)):
        side = np.array((math.cos(yaw), math.sin(yaw), 0.))
        base = r * (math.sin(math.radians(48)) * side + (0., 0., math.cos(math.radians(48))))
        direction = math.cos(math.radians(10)) * side + (0., 0., -math.sin(math.radians(10)))
        text += cone(f"cap_point_{i}", ("plush_green", "plush_yellow", "plush_green")[i], base + .008 * direction, .010, .016,
                     axis_quat(direction))
        text += sphere(f"bell_{i}", "bell", base + .019 * direction, .0045, collide=False)
    for side in (-1, 1):
        text += sphere(f"eye_{'lr'[side > 0]}", "black", (-r * .93, side * .010, .002), .0035, collide=False)
        text += sphere(f"cheek_{'lr'[side > 0]}", "plush_red", (-r * .86, side * .017, -.010), .004, collide=False)
    text += sphere("nose", "plush_red", (-r - .002, 0., -.006), .0055, collide=False)
    text += box("mouth", "plush_red", (-r * .92 - .0008, -.007, -.018), (-r * .92 + .0008, .007, -.016), collide=False)
    text += sphere("ruff", "plush_white", (0., 0., -r - .002), .030, collide=False, scale=(1., 1., .30))
    for k in range(4):
        z = -L.HEAD_COLUMN[1] - .010 - .013 * k
        text += cylinder(f"stripe_{k}", ("plush_green", "plush_yellow")[k % 2], (0., 0., z), L.HEAD_COLUMN[0] + .0004, .006,
                         collide=False)
    return text


def coil_colliders():
    r, z0, z1 = L.LINK_COLUMN
    text = cylinder("sleeve", "plush_green", (0., 0., (z0 + z1) / 2), r, z1 - z0)
    for k in range(5):
        text += cylinder(f"stripe_{k}", ("plush_yellow", "plush_red")[k % 2], (0., 0., z0 + .012 + .018 * k), r + .0004, .006,
                         collide=False)
    return text


def usd():
    top = scene.TOP
    start = {name: (np.asarray(p) + (0., 0., top), q) for name, (p, q) in L.start_poses().items()}
    parts = "".join(box(n, m, lo, hi) for n, m, lo, hi in box_colliders()) + crank_round() + box_prints()
    bodies = body("box", L.BOX_MASS, *start["box"], parts, no_contact_with=("lid", "coil", "catch"))
    bodies += body("lid", L.LID_MASS, *start["lid"],
                   "".join(box(n, m, lo, hi) for n, m, lo, hi in lid_colliders()) + lid_prints())
    bodies += body("coil", L.LINK_MASS, *start["coil"], coil_colliders(), no_contact_with=("head",))
    bodies += body("head", L.HEAD_MASS, *start["head"], head_colliders() + head_decor())
    bodies += body("catch", L.CATCH_MASS, *start["catch"], hull("hook", "steel", catch_points()))
    joints = joint("PhysicsRevoluteJoint", "lid_hinge", "box", "lid", (L.HINGE[0], 0., L.HINGE[1]), "Y", 0., L.LID_OPEN,
                   stiffness=0., damping=L.LID_DAMPING)
    joints += joint("PhysicsRevoluteJoint", "coil_lean", "box", "coil", (0., 0., L.PIVOT_Z), "Y", -L.LEAN_LIMIT, L.LEAN_LIMIT,
                    stiffness=L.LEAN_RATE, damping=L.LEAN_DAMPING)
    joints += joint("PhysicsPrismaticJoint", "coil_spring", "coil", "head", (0., 0., 0.), "Z", L.S_LOW, L.S_POPPED,
                    stiffness=L.SPRING_RATE, damping=L.SPRING_DAMPING, target=L.S_FREE)
    joints += joint("PhysicsPrismaticJoint", "catch_slide", "box", "catch", (0., 0., 0.), "X", -L.CATCH_TRAVEL, 0.,
                    stiffness=L.CATCH_RATE, damping=2., target=L.CATCH_PRELOAD)
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print(f"wrote {scene.USD.name}: {len(L.ACTORS)} bodies, 4 joints; spring {L.spring_force(L.S_POPPED):.1f} N sprung out, "
          f"{L.spring_force(L.S_SHUT):.1f} N with the head level with the rim; masses {dict(zip(L.ACTORS, L.MASSES))}")
