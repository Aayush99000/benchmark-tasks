"""The generated scene file agrees with till_model.py, and the model's numbers agree with each other."""

import re

import numpy as np

from rdb_till_note import build_scene, fixed_scene as fixture, till_model as m

TEXT = fixture.USD.read_text()


def block(kind, name):
    start = TEXT.index(f'def {kind} "{name}"')
    following = re.search(r'\n        def (Xform|PhysicsRevoluteJoint|PhysicsPrismaticJoint|PhysicsFixedJoint|Material) "', TEXT[start + 10:])
    return TEXT[start:start + 10 + following.start()] if following else TEXT[start:]


def test_scene_file_is_what_the_generator_writes():
    assert build_scene.usd() == TEXT


def test_bodies_masses_and_articulations():
    masses = {"case": m.DRAWER_TOTAL_MASS - m.DRAWER_MASS, "drawer": m.DRAWER_MASS, "stack": m.STACK_MASS,
              **dict.fromkeys(fixture.HOLDS, m.HOLD_MASS), **dict.fromkeys(fixture.STRIPS, m.STRIP_MASS)}
    for name, mass in masses.items():
        body = block("Xform", name)
        assert f"float physics:mass = {mass:.6g}" in body and "PhysicsRigidBodyAPI" in body, name
    assert TEXT.count("PhysicsArticulationRootAPI") == 2                 # the drawer unit, and the note
    assert "PhysicsArticulationRootAPI" in block("Xform", "strip_0")
    assert TEXT.count("def PhysicsRevoluteJoint") == 5 + m.STRIPS - 1 and TEXT.count("def PhysicsPrismaticJoint") == 1
    assert abs(sum(masses[k] for k in ("case", "drawer")) - 7.6) < 1e-9
    assert abs(m.STRIPS * m.STRIP_MASS - .008) < 1e-12


def test_every_collider_is_a_box_and_nothing_is_soft():
    assert "PhysicsMeshCollisionAPI" not in TEXT and "compliant" not in TEXT.lower() and "Deformable" not in TEXT
    colliders = re.findall(r'def (\w+) "\w+" \(\n\s+prepend apiSchemas = \[[^\]]*PhysicsCollisionAPI', TEXT)
    assert colliders and set(colliders) == {"Cube"}


def test_note_strips_collide_but_are_drawn_as_a_thin_sheet():
    strip = block("Xform", "strip_3")
    collider = strip[strip.index('def Cube "collider"'):strip.index('def Cube "sheet"')]
    assert 'purpose = "guide"' in collider and "PhysicsCollisionAPI" in collider
    assert f"({m.STRIP_L / 2:.6g}, {m.NOTE_W / 2:.6g}, {m.NOTE_T / 2:.6g})" in collider
    sheet = strip[strip.index('def Cube "sheet"'):]
    assert "PhysicsCollisionAPI" not in sheet.split("def Cube")[1]
    assert "print" in strip                                              # the printed face


def test_springs_and_limits():
    per_deg = np.pi / 180
    fold = block("PhysicsRevoluteJoint", "fold_4")
    assert f"stiffness = {m.HINGE_K * per_deg:.7g}" in fold and 'physics:axis = "Y"' in fold
    hinge = block("PhysicsRevoluteJoint", "hinge_2")
    assert f"stiffness = {m.HOLD_SPRING * per_deg:.7g}" in hinge and f"targetPosition = {m.HOLD_SPRING_REST_DEG:.7g}" in hinge
    assert f"lowerLimit = {m.HOLD_LIMITS_DEG[0]:.7g}" in hinge
    slide = block("PhysicsPrismaticJoint", "slide")
    assert f"maxForce = {m.SLIDE_FRICTION_N:.7g}" in slide and f"lowerLimit = {-(m.TRAVEL + .002):.7g}" in slide


def test_model_numbers_hang_together():
    assert abs(m.hold_torque(m.HOLD_ON_STACK) / m.ARM_L - 1.5) < .05          # about 1.5 N at the free end
    assert 1.8 < m.hold_torque(np.radians(30.)) / m.ARM_L < 2.2
    assert abs(m.tip_underside(m.HOLD_ON_STACK)[2] - m.STACK_TOP_Z) < 1e-6
    assert abs(m.tip_underside(m.HOLD_ON_NOTE)[2] - m.STACK_TOP_Z - m.NOTE_T) < 1e-6
    assert m.COMPARTMENT_W - m.NOTE_W > .003 and m.COMPARTMENT_W - m.NOTE_W < .005   # about 4 mm of play
    assert m.BILL_X[1] - m.BILL_X[0] - m.NOTE_L > .012
    # shut, nothing in the drawer is taller than the space under the case top
    tab_top = m.hold_point(m.HOLD_ON_NOTE, m.TAB_X[0], m.ARM_T / 2 + m.TAB_H)[2]
    assert max(tab_top, m.HINGE_BLOCK_Z, m.RIM_Z) < m.CASE_H - m.CASE_TOP_T - .005
    assert m.FRONT_OPEN_X + m.REAR_X[1] < m.CASE_FRONT_X + .005               # the whole till is out from under the case
    # the customer's note: most of it on the case, a hinge just outside the edge
    assert m.NOTE_START_X[0] == m.CASE_FRONT_X - m.OVERHANG and m.OVERHANG < m.NOTE_L / 2
    assert m.support_height(m.CASE_FRONT_X + .01, m.NOTE_START_Y, m.TRAVEL) == m.CASE_H
    x0 = m.drawer_x(m.TRAVEL)
    assert m.support_height(x0 + m.STACK_X, 0., m.TRAVEL) == m.BILL_FLOOR_Z
    assert m.support_height(x0 + m.DIVIDER_X[0] + .001, 0., m.TRAVEL) == m.RIM_Z
    assert m.compartment_of(0.) == m.STOCKED and m.compartment_of(m.PITCH / 2) is None
