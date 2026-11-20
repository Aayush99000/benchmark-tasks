"""Regenerate ``till.usda`` from ``till_model.py``.   Run:  python -m rdb_till_note.build_scene

Three things are in the file:

* ``/Fixture/Till``: one fixed-base articulation. The steel case is fixed to the world; the drawer (with the till
  insert moulded onto it) runs on a prismatic joint; each of the five bill hold-downs is hinged to the drawer and
  sprung downward.
* ``/Fixture/Actors/stack``: the notes already in the middle compartment, one free rigid block.
* ``/Fixture/Actors/strip_0 .. strip_7``: the customer's note, a floating articulation of eight rigid strips
  joined by seven sprung hinges across its width. Each strip's collider is a 2 mm box that is not drawn; what is
  drawn is a thin printed sheet in the middle of it.

All colliders are boxes. Printed marks, the lock and the hinge knuckles are drawn only.
"""

import math

import numpy as np

from . import fixed_scene as scene
from . import till_model as m

TOP = scene.TOP
# colour (sRGB-ish linear guess), roughness, metallic, friction
MATERIALS = {
    "case_steel": ((.035, .035, .04), .45, .6, .4),
    "panel_steel": ((.03, .03, .033), .35, .6, .4),
    "till_abs": ((.05, .05, .055), .55, 0., .35),
    "hold_plastic": ((.16, .16, .17), .5, 0., .8),
    "bright_steel": ((.75, .76, .78), .25, 1., .4),
    "paper": ((.74, .79, .68), .9, 0., m.NOTE_FRICTION),
    "ink": ((.13, .27, .17), .9, 0., m.NOTE_FRICTION),
    "ink_light": ((.42, .55, .44), .9, 0., m.NOTE_FRICTION),
    "paper_edge": ((.62, .67, .58), .9, 0., m.NOTE_FRICTION),
}
PREFIX = "</Fixture/Materials/"
SHEET_T = .0004            # drawn thickness of the note
INK_T = .00006


def cube(name, material, centre, half, *, collide=True, visible=True, pad="            "):
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    text = f'{pad}def Cube "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
    text += f"{pad}    rel material:binding = {PREFIX}{material}>\n"
    if collide:
        text += (f"{pad}    rel material:binding:physics = {PREFIX}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = 0.001\n{pad}    float physxCollision:restOffset = 0\n")
    if not visible:
        text += f'{pad}    uniform token purpose = "guide"\n'
    text += (f"{pad}    double size = 2\n"
             f"{pad}    float3 xformOp:scale = ({half[0]:.6g}, {half[1]:.6g}, {half[2]:.6g})\n"
             f"{pad}    double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
             f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n{pad}}}\n\n')
    return text


def slab(name, material, x, y, z, **kwargs):
    """A box given by its (low, high) extents on each axis."""
    return cube(name, material, [(a + b) / 2 for a, b in (x, y, z)], [(b - a) / 2 for a, b in (x, y, z)], **kwargs)


def drawn_cylinder(name, material, centre, radius, height, axis, pad="            "):
    return (f'{pad}def Cylinder "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {PREFIX}{material}>\n"
            f'{pad}    uniform token axis = "{axis}"\n{pad}    double height = {height:.6g}\n'
            f"{pad}    double radius = {radius:.6g}\n"
            f"{pad}    double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate"]\n{pad}}}\n\n')


def body(name, mass, position, children, *, pitch=0., root=False, inertia=None, pad="        "):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        extra = (f"{pad}    bool physxArticulation:enabledSelfCollisions = 1\n"
                 f"{pad}    int physxArticulation:solverPositionIterationCount = 64\n"
                 f"{pad}    int physxArticulation:solverVelocityIterationCount = 8\n"
                 f"{pad}    float physxArticulation:sleepThreshold = 0\n"
                 f"{pad}    float physxArticulation:stabilizationThreshold = 0\n")
    if inertia is not None:
        extra += f"{pad}    float3 physics:diagonalInertia = ({inertia[0]:.6g}, {inertia[1]:.6g}, {inertia[2]:.6g})\n"
    w, y = math.cos(pitch / 2), math.sin(pitch / 2)          # rotation about +y
    return (f'{pad}def Xform "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            f"{pad}    float physics:mass = {mass:.6g}\n{pad}    bool physics:rigidBodyEnabled = 1\n" + extra +
            f"{pad}    float physxContactReport:threshold = 0\n"
            f"{pad}    float physxRigidBody:maxDepenetrationVelocity = 1\n"
            f"{pad}    float physxRigidBody:sleepThreshold = 0\n"
            f"{pad}    float physxRigidBody:angularDamping = 0.5\n"
            f"{pad}    float physxRigidBody:linearDamping = 0.1\n"
            f"{pad}    int physxRigidBody:solverPositionIterationCount = 64\n"
            f"{pad}    int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"{pad}    quatf xformOp:orient = ({w:.8g}, 0, {y:.8g}, 0)\n"
            f"{pad}    double3 xformOp:translate = ({position[0]:.6g}, {position[1]:.6g}, {position[2]:.6g})\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}{pad}}}\n\n")


def joint(kind, name, body0, body1, anchor0, anchor1, axis, limits, *, stiffness=0., damping=0., target=0.,
          max_force=None, armature=0., pad="        "):
    """``kind`` is "Revolute" (limits, stiffness and target in degrees) or "Prismatic" (metres)."""
    drive = "angular" if kind == "Revolute" else "linear"
    text = (f'{pad}def Physics{kind}Joint "{name}" (\n'
            f'{pad}    prepend apiSchemas = ["PhysicsDriveAPI:{drive}", "PhysxJointAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    float drive:{drive}:physics:damping = {damping:.7g}\n"
            f"{pad}    float drive:{drive}:physics:stiffness = {stiffness:.7g}\n"
            f"{pad}    float drive:{drive}:physics:targetPosition = {target:.7g}\n"
            f"{pad}    float drive:{drive}:physics:targetVelocity = 0\n"
            f'{pad}    uniform token drive:{drive}:physics:type = "force"\n')
    if max_force is not None:
        text += f"{pad}    float drive:{drive}:physics:maxForce = {max_force:.7g}\n"
    text += (f'{pad}    uniform token physics:axis = "{axis}"\n'
             f"{pad}    rel physics:body0 = <{body0}>\n{pad}    rel physics:body1 = <{body1}>\n"
             f"{pad}    point3f physics:localPos0 = ({anchor0[0]:.6g}, {anchor0[1]:.6g}, {anchor0[2]:.6g})\n"
             f"{pad}    point3f physics:localPos1 = ({anchor1[0]:.6g}, {anchor1[1]:.6g}, {anchor1[2]:.6g})\n"
             f"{pad}    quatf physics:localRot0 = (1, 0, 0, 0)\n{pad}    quatf physics:localRot1 = (1, 0, 0, 0)\n"
             f"{pad}    float physics:lowerLimit = {limits[0]:.7g}\n{pad}    float physics:upperLimit = {limits[1]:.7g}\n"
             f"{pad}    float physxJoint:armature = {armature:.7g}\n"
             f"{pad}    float physxJoint:maxJointVelocity = {720 if kind == 'Revolute' else 2}\n{pad}}}\n\n")
    return text


def material(name):
    color, roughness, metallic, friction = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


# ---------------------------------------------------------------------------------------------------------
def note_art():
    """The printed face of a note as rectangles (material, x0, x1, y0, y1) in note coordinates: x along the note
    from 0 to NOTE_L, y across it. A generic green engraving (frame, oval, seals, corner blocks), not a copy of
    any real note."""
    L, half = m.NOTE_L, m.NOTE_W / 2
    marks = [("ink", .004, L - .004, half - .0055, half - .004), ("ink", .004, L - .004, -half + .004, -half + .0055),
             ("ink", .004, .0055, -half + .004, half - .004), ("ink", L - .0055, L - .004, -half + .004, half - .004)]
    cx, a, b, n = L / 2, .019, .024, 10
    for k in range(n):
        x0, x1 = cx - a + 2 * a * k / n, cx - a + 2 * a * (k + 1) / n
        h = b * math.sqrt(max(0., 1 - (((x0 + x1) / 2 - cx) / a) ** 2))
        marks.append(("ink_light", x0, x1, -h, h))
    for x in (.012, L - .012):
        for y in (-half + .012, half - .012):
            marks.append(("ink", x - .004, x + .004, y - .004, y + .004))
    marks.append(("ink", L * .74 - .006, L * .74 + .006, -.006, .006))
    marks.append(("ink_light", L * .26 - .006, L * .26 + .006, -.006, .006))
    return marks


def printed(prefix, x_range, centre_x, z_faces, pad="            "):
    """The art between note coordinates x_range, drawn on the given face heights, relative to ``centre_x``."""
    text = ""
    for k, (ink, x0, x1, y0, y1) in enumerate(note_art()):
        x0, x1 = max(x0, x_range[0]), min(x1, x_range[1])
        if x1 - x0 < 1e-5:
            continue
        for side, z in enumerate(z_faces):
            text += slab(f"{prefix}{k:02d}_{side}", ink, (x0 - centre_x, x1 - centre_x), (y0, y1),
                         (z - INK_T / 2, z + INK_T / 2), collide=False, pad=pad)
    return text


def till_articulation():
    hw = m.CASE_W / 2
    x0, x1 = m.CASE_FRONT_X, m.CASE_FRONT_X + m.CASE_D
    case = slab("top", "case_steel", (x0, x1), (-hw, hw), (m.CASE_H - m.CASE_TOP_T, m.CASE_H))
    case += slab("bottom", "case_steel", (x0, x1), (-hw, hw), (0., m.CASE_BOTTOM_T))
    for name, sign in (("side_right", -1), ("side_left", 1)):
        ys = sorted((sign * hw, sign * (hw - m.CASE_WALL)))
        case += slab(name, "case_steel", (x0, x1), ys, (m.CASE_BOTTOM_T, m.CASE_H - m.CASE_TOP_T))
    case += slab("back", "case_steel", (x1 - m.CASE_WALL, x1), (-hw + m.CASE_WALL, hw - m.CASE_WALL),
                 (m.CASE_BOTTOM_T, m.CASE_H - m.CASE_TOP_T))
    text = body("case", m.DRAWER_TOTAL_MASS - m.DRAWER_MASS, (0., 0., TOP), case)

    tw = m.TILL_W / 2
    d = slab("front_panel", "panel_steel", (0., m.PANEL_T), (-m.PANEL_W / 2, m.PANEL_W / 2), m.PANEL_Z)
    d += drawn_cylinder("lock", "bright_steel", (-.0015, 0., .078), .009, .003, "X")
    d += slab("keyhole", "case_steel", (-.0032, -.0030), (-.001, .001), (.074, .082), collide=False)
    d += slab("floor", "case_steel", m.DRAWER_FLOOR_X, (-m.DRAWER_FLOOR_HALF_Y, m.DRAWER_FLOOR_HALF_Y), m.DRAWER_FLOOR_Z)
    for name, sign in (("wall_right", -1), ("wall_left", 1)):
        ys = sorted((sign * m.DRAWER_FLOOR_HALF_Y, sign * (m.DRAWER_FLOOR_HALF_Y - m.DRAWER_SIDE_T)))
        d += slab(name, "case_steel", m.DRAWER_FLOOR_X, ys, m.DRAWER_SIDE_Z)
    d += slab("wall_back", "case_steel", (m.DRAWER_FLOOR_X[1] - m.DRAWER_SIDE_T, m.DRAWER_FLOOR_X[1]),
              (-m.DRAWER_FLOOR_HALF_Y, m.DRAWER_FLOOR_HALF_Y), m.DRAWER_SIDE_Z)
    # the till insert
    d += slab("till_coin_floor", "till_abs", (m.TILL_X0, m.DIVIDER_X[0]), (-tw, tw), (m.TILL_Z0, m.COIN_FLOOR_Z))
    d += slab("till_bill_floor", "till_abs", (m.DIVIDER_X[0], m.REAR_X[0]), (-tw, tw), (m.TILL_Z0, m.BILL_FLOOR_Z))
    d += slab("till_front", "till_abs", (m.TILL_X0, m.COIN_X[0]), (-tw, tw), (m.TILL_Z0, m.RIM_Z))
    d += slab("till_divider", "till_abs", m.DIVIDER_X, (-tw, tw), (m.TILL_Z0, m.RIM_Z))
    d += slab("till_rear_wall", "till_abs", (m.REAR_X[0], m.REAR_X[0] + m.WALL), (-tw, tw), (m.TILL_Z0, m.RIM_Z))
    d += slab("till_hinge_block", "till_abs", (m.REAR_X[0] + m.WALL, m.REAR_X[1]), (-tw, tw), (m.TILL_Z0, m.HINGE_BLOCK_Z))
    for k in range(6):
        y = -tw + k * m.PITCH
        d += slab(f"till_wall_{k}", "till_abs", (m.TILL_X0, m.REAR_X[0]), (y - m.WALL / 2, y + m.WALL / 2),
                  (m.TILL_Z0, m.RIM_Z))
    for k, y in enumerate(m.COMPARTMENT_Y):
        d += drawn_cylinder(f"knuckle_{k}", "till_abs", (m.PIVOT_X + .004, y, m.PIVOT_Z), .0045, .024, "Y")
    text += body("drawer", m.DRAWER_MASS, (m.FRONT_OPEN_X, 0., TOP), d)

    arm = cube("arm", "hold_plastic", (-(m.ARM_L - .003) / 2, 0., 0.), ((m.ARM_L + .003) / 2, m.ARM_W / 2, m.ARM_T / 2))
    arm += cube("tab", "hold_plastic", (-sum(m.TAB_X) / 2, 0., m.ARM_T / 2 + m.TAB_H / 2),
                ((m.TAB_X[1] - m.TAB_X[0]) / 2, m.ARM_W / 2, m.TAB_H / 2))
    for k, y in enumerate(m.COMPARTMENT_Y):
        angle = m.HOLD_ON_STACK + math.radians(.3) if k == m.STOCKED else m.HOLD_ON_FLOOR
        text += body(f"hold_{k}", m.HOLD_MASS, (m.FRONT_OPEN_X + m.PIVOT_X, y, TOP + m.PIVOT_Z), arm, pitch=angle)

    here = "/Fixture/Till/"
    text += ('        def PhysicsFixedJoint "root_joint"\n        {\n'
             f"            rel physics:body1 = <{here}case>\n"
             f"            point3f physics:localPos0 = (0, 0, {TOP:.6g})\n            point3f physics:localPos1 = (0, 0, 0)\n"
             "            quatf physics:localRot0 = (1, 0, 0, 0)\n            quatf physics:localRot1 = (1, 0, 0, 0)\n        }\n\n")
    # Slide friction: a velocity drive toward rest whose force is capped at the friction force.
    text += joint("Prismatic", "slide", here + "case", here + "drawer", (m.FRONT_CLOSED_X, 0., 0.), (0., 0., 0.), "X",
                  (-(m.TRAVEL + .002), 0.), damping=2000., max_force=m.SLIDE_FRICTION_N, armature=.01)
    per_deg = math.pi / 180.
    for k, y in enumerate(m.COMPARTMENT_Y):
        text += joint("Revolute", f"hinge_{k}", here + "drawer", here + f"hold_{k}", (m.PIVOT_X, y, m.PIVOT_Z),
                      (0., 0., 0.), "Y", m.HOLD_LIMITS_DEG, stiffness=m.HOLD_SPRING * per_deg,
                      damping=m.HOLD_DAMPING * per_deg, target=m.HOLD_SPRING_REST_DEG, armature=m.HOLD_ARMATURE)
    return text


def loose_bodies():
    sx, sy, sz = (v / 2 for v in m.STACK_SIZE)
    stack = cube("notes", "paper_edge", (0., 0., 0.), (sx, sy, sz))
    stack += slab("face", "paper", (-sx, sx), (-sy, sy), (sz, sz + INK_T), collide=False)
    stack += printed("print", (0., m.NOTE_L), m.NOTE_L / 2, (sz + 1.5 * INK_T,))
    text = body("stack", m.STACK_MASS, m.STACK_START + (0., 0., TOP), stack)
    inertia = (4e-7, 2e-7, 5e-7)        # about the hinge axis (y) raised from 3e-8 kg m^2 for solver stability
    for i in range(m.STRIPS):
        strip = cube("collider", "paper", (0., 0., 0.), (m.STRIP_L / 2, m.NOTE_W / 2, m.NOTE_T / 2), visible=False)
        strip += cube("sheet", "paper", (0., 0., 0.), (m.STRIP_L / 2 + .0004, m.NOTE_W / 2, SHEET_T / 2), collide=False)
        strip += printed("print", (i * m.STRIP_L, (i + 1) * m.STRIP_L), (i + .5) * m.STRIP_L,
                         (SHEET_T / 2 + INK_T / 2, -SHEET_T / 2 - INK_T / 2))
        text += body(f"strip_{i}", m.STRIP_MASS, m.NOTE_START[i] + (0., 0., TOP), strip, root=(i == 0), inertia=inertia)
    return text


def note_joints():
    per_deg = math.pi / 180.
    text = ""
    for i in range(1, m.STRIPS):
        text += joint("Revolute", f"fold_{i}", f"/Fixture/Actors/strip_{i - 1}", f"/Fixture/Actors/strip_{i}",
                      (m.STRIP_L / 2, 0., 0.), (-m.STRIP_L / 2, 0., 0.), "Y", (-m.HINGE_LIMIT_DEG, m.HINGE_LIMIT_DEG),
                      stiffness=m.HINGE_K * per_deg, damping=m.HINGE_DAMPING * per_deg, armature=m.HINGE_ARMATURE)
    return text


def usd():
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            '    def Xform "Till" (\n        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n'
            "    )\n    {\n"
            "        bool physxArticulation:enabledSelfCollisions = 0\n"
            "        int physxArticulation:solverPositionIterationCount = 32\n"
            "        int physxArticulation:solverVelocityIterationCount = 4\n"
            "        float physxArticulation:sleepThreshold = 0\n"
            "        float physxArticulation:stabilizationThreshold = 0\n\n"
            + till_articulation() + "    }\n\n"
            '    def Scope "Actors"\n    {\n' + loose_bodies() + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + note_joints() + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print(f"wrote {scene.USD} ({len(usd().splitlines())} lines)")
