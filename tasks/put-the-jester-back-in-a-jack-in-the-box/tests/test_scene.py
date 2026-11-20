"""The generated scene file agrees with layout.py, and the layout with the published sizes."""

import importlib.util
import re
from pathlib import Path

import numpy as np

from rdb_jack_in_box import fixed_scene as fixture
from rdb_jack_in_box import layout as L

ROOT = Path(__file__).resolve().parents[1]
TEXT = fixture.USD.read_text()


def builder():
    spec = importlib.util.spec_from_file_location("build_scene", ROOT / "tools/build_scene.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def block(kind, name):
    start = TEXT.index(f'def {kind} "{name}"')
    following = re.search(r'\n        def (Xform|PhysicsPrismaticJoint|PhysicsRevoluteJoint|Material) "', TEXT[start + 10:])
    return TEXT[start:start + 10 + following.start()] if following else TEXT[start:]


def test_scene_file_is_what_the_generator_writes():
    assert builder().usd() == TEXT


def test_published_sizes_and_mass():
    inch = .0254
    assert np.allclose(L.BOX_PUBLISHED, (5.25 * inch, 5.25 * inch, 5.5 * inch))
    assert abs(L.RIM_Z + L.LID_T - 5.5 * inch) < 1e-9 and abs(L.POPPED_TOP - 10 * inch) < 1e-9
    assert abs(sum(L.MASSES) - L.TOY_MASS) < 1e-9 and abs(L.TOY_MASS - .4536) < .001


def test_bodies_masses_start_poses_and_no_articulation():
    for name, mass in zip(L.ACTORS, L.MASSES, strict=True):
        body = block("Xform", name)
        assert f"float physics:mass = {mass:.6g}" in body and "PhysicsRigidBodyAPI" in body
        (x, y, z), _ = fixture.WORLD_START[name]
        assert f"double3 xformOp:translate = ({x:.6g}, {y:.6g}, {z:.6g})" in body
    assert "PhysicsArticulationRootAPI" not in TEXT and "PhysicsFixedJoint" not in TEXT


def test_joints_form_one_tree_on_the_loose_box():
    joints = re.findall(r'def Physics(\w+)Joint "(\w+)".*?body0 = </Fixture/Actors/(\w+)>.*?body1 = </Fixture/Actors/(\w+)>',
                        TEXT, re.S)
    assert sorted((n, a, b) for _, n, a, b in joints) == [("catch_slide", "box", "catch"), ("coil_lean", "box", "coil"),
                                                          ("coil_spring", "coil", "head"), ("lid_hinge", "box", "lid")]
    hinge = block("PhysicsRevoluteJoint", "lid_hinge")
    assert "physics:lowerLimit = 0\n" in hinge and "physics:upperLimit = 105\n" in hinge
    assert "drive:angular:physics:stiffness = 0\n" in hinge
    spring = block("PhysicsPrismaticJoint", "coil_spring")
    assert f"drive:linear:physics:stiffness = {L.SPRING_RATE:.6g}\n" in spring
    assert f"physics:lowerLimit = {L.S_LOW:.6g}\n" in spring and f"physics:upperLimit = {L.S_POPPED:.6g}\n" in spring
    assert f"targetPosition = {L.S_FREE:.6g}\n" in spring
    lean = block("PhysicsRevoluteJoint", "coil_lean")
    assert "physics:lowerLimit = -30\n" in lean and "physics:upperLimit = 30\n" in lean
    assert f"stiffness = {L.LEAN_RATE * np.pi / 180:.6g}\n" in lean
    catch = block("PhysicsPrismaticJoint", "catch_slide")
    assert f"physics:lowerLimit = {-L.CATCH_TRAVEL:.6g}\n" in catch and f"stiffness = {L.CATCH_RATE:.6g}\n" in catch


def test_filtered_pairs_leave_the_working_contacts_on():
    box = block("Xform", "box")
    for other in ("lid", "coil", "catch"):
        assert f"</Fixture/Actors/{other}>" in box
    assert "</Fixture/Actors/head>" not in box               # the head meets the walls
    assert "</Fixture/Actors/head>" in block("Xform", "coil")
    for name in ("lid", "head", "catch"):
        assert "filteredPairs" not in block("Xform", name)   # lid on head, barb on hook, fingers on all


def test_spring_and_catch_forces_are_within_the_arms():
    assert .5 < L.spring_force(L.S_POPPED) < 1.5 and 5. < L.spring_force(L.S_SHUT) < 7.
    assert L.CATCH_RATE * (L.CATCH_PRELOAD + (L.CATCH_X[1] - L.BARB_X[0])) < 3.
    # the barb passes the hook only once the lid is shut: it ends 1 mm under the hook's flat underside
    assert abs(L.CATCH_Z[0] - L.BARB_Z[1] - .001) < 1e-9 and L.CATCH_X[1] - L.BARB_X[0] >= .0025


def test_lid_and_head_geometry():
    hinge = L.hinge_world()
    assert np.allclose(L.lid_point(0., -L.SIDE), (L.BOX_X - L.HALF, 0., hinge[2]))
    assert L.lid_point(L.LID_OPEN, -L.SIDE)[2] > .25          # front edge high in the air when open
    assert abs(L.lid_angle_from_rotation(L.rot_y(.7)) - .7) < 1e-9
    assert abs(L.head_centre(L.S_POPPED, 0.)[2] + L.HEAD_R - L.POPPED_TOP) < 1e-9
    # the head fits inside the walls with room to lean; the head's sleeve stays above the floor fully pressed
    assert L.INNER - L.HEAD_R > .025 and L.PIVOT_Z + L.S_LOW - L.HEAD_COLUMN[2] > L.FLOOR_T


def test_every_collider_is_a_simple_shape_and_display_parts_do_not_collide():
    prims = re.findall(r'def (Cube|Sphere|Cylinder|Cone|Mesh) "([^"]+)" \(\n\s+prepend apiSchemas = \[([^\]]+)\]', TEXT)
    colliding = {name for kind, name, apis in prims if "PhysicsCollisionAPI" in apis}
    assert {"base", "wall_front", "wall_back", "wall_left", "wall_right", "plate", "tongue", "barb", "ball", "sleeve",
            "hook", "crank_shaft", "crank_knob", "crank_arm"} <= colliding
    for kind, name, apis in prims:
        if name.startswith(("print_", "cap", "bell", "eye", "cheek", "nose", "mouth", "ruff", "stripe", "seam", "rolled",
                            "hinge_knuckle", "spring_cup")):
            assert "PhysicsCollisionAPI" not in apis, name
        if kind == "Mesh":
            assert name == "hook"
    assert TEXT.count("PhysicsMeshCollisionAPI") == 1 and 'physics:approximation = "convexHull"' in TEXT
