"""Offline checks of mechanism.py's chained joints and base contacts (no Isaac):

    PYTHONPATH=<yam-isaac-lab-arena checkout>:. python -m pytest -q test_mechanism_chain.py

Needs NumPy, pytest and Pixar's ``usd-core`` (for ``pxr``). Isaac Lab's helpers and NVIDIA's ``PhysxSchema`` are not in
usd-core, so they are replaced here by stand-ins that only author plain prims; the test then reads the authored stage:
which bodies each joint joins, where its anchors are, and which pairs are filtered. It does not run physics.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import pytest

pxr = pytest.importorskip("pxr")
from pxr import Usd, UsdPhysics  # noqa: E402

STAGE: dict = {}


class _NoOp:
    """Stands in for a PhysxSchema API: Apply(prim) returns an object whose Create...Attr calls do nothing."""

    @classmethod
    def Apply(cls, prim):  # noqa: N802 (USD's spelling)
        return cls()

    def __getattr__(self, name):
        return lambda *a, **k: None


def _install_stubs() -> None:
    physx = types.ModuleType("pxr.PhysxSchema")
    for api in ("PhysxCollisionAPI", "PhysxRigidBodyAPI", "PhysxArticulationAPI"):
        setattr(physx, api, type(api, (_NoOp,), {}))
    sys.modules["pxr.PhysxSchema"] = physx
    pxr.PhysxSchema = physx

    def cfg_class(name):
        def init(self, **kw):
            self.__dict__.update(kw)
        return type(name, (), {"__init__": init, "func": staticmethod(lambda path, cfg: STAGE["s"].DefinePrim(path))})

    sim = types.ModuleType("isaaclab.sim")
    sim.RigidBodyMaterialCfg = cfg_class("RigidBodyMaterialCfg")
    sim.PreviewSurfaceCfg = cfg_class("PreviewSurfaceCfg")
    utils = types.ModuleType("isaaclab.sim.utils")
    utils.bind_physics_material = utils.bind_visual_material = lambda *a, **k: None
    stage_mod = types.ModuleType("isaaclab.sim.utils.stage")
    stage_mod.get_current_stage = lambda: STAGE["s"]
    isaaclab = sys.modules.setdefault("isaaclab", types.ModuleType("isaaclab"))
    isaaclab.sim = sim
    sim.utils = utils
    utils.stage = stage_mod
    sys.modules.update({"isaaclab.sim": sim, "isaaclab.sim.utils": utils, "isaaclab.sim.utils.stage": stage_mod})


_install_stubs()
import mechanism as M  # noqa: E402

MATERIALS = {"default": (0.8, 0.7)}


def box(center, size):
    return M.Shape("box", center, size)


BASE = M.Link("jamb", 5.0, (box((0.62, 0.0, 0.9), (0.02, 0.10, 2.0)), box((0.62, 0.03, 0.9), (0.004, 0.025, 0.06))))
DOOR = M.Link("door", 9.0, (box((0.305, 0.0, 1.0), (0.60, 0.035, 2.0)),))
KNOB = M.Link("knob", 0.3, (M.Shape("cylinder", (0.55, 0.08, 0.9), (0.028, 0.03), axis="Y"),))
BOLT = M.Link("bolt", 0.05, (box((0.612, 0.0, 0.9), (0.026, 0.015, 0.025)),))
JOINTS = (
    M.Joint("door", "revolute", "door", (0.0, 0.0, 1.0), "Z", 0.0, 1.5, damping=2.0),
    M.Joint("knob", "revolute", "knob", (0.55, 0.0, 0.9), "Y", -0.9, 0.9, stiffness=0.6, parent="door"),
    M.Joint("bolt", "prismatic", "bolt", (0.612, 0.0, 0.9), "X", -0.013, 0.0, stiffness=300.0, target=0.002,
            parent="door"),
)


def door_mechanism(joints=JOINTS, base_contacts=("bolt",)):
    return M.mechanism("door_unit", (0.0, 0.0, 0.0), BASE, (DOOR, KNOB, BOLT), joints, MATERIALS,
                       base_contacts=base_contacts)


def author(spec) -> Usd.Stage:
    STAGE["s"] = Usd.Stage.CreateInMemory()
    M._SPECS[spec.name] = spec
    M._spawn("/World/door_unit", SimpleNamespace(mechanism_name=spec.name))
    return STAGE["s"]


def joint_at(stage, link, name):
    return stage.GetPrimAtPath(f"/World/door_unit/{link}/{name}")


def targets(rel) -> list[str]:
    return sorted(str(p).rsplit("/", 1)[1] for p in rel.GetTargets())


def test_chained_joints_join_the_named_parent():
    stage = author(door_mechanism())
    for link, parent in (("door", "jamb"), ("knob", "door"), ("bolt", "door")):
        joint = UsdPhysics.Joint(joint_at(stage, link, link))
        assert targets(joint.GetBody0Rel()) == [parent]
        assert targets(joint.GetBody1Rel()) == [link]


def test_anchor_is_the_same_point_in_both_bodies():
    stage = author(door_mechanism())
    for joint in JOINTS:
        usd = UsdPhysics.Joint(joint_at(stage, joint.child, joint.name))
        assert tuple(usd.GetLocalPos0Attr().Get()) == pytest.approx(joint.anchor)
        assert tuple(usd.GetLocalPos1Attr().Get()) == pytest.approx(joint.anchor)


def test_bodies_start_at_the_mechanism_frame():
    # what makes "the anchor is the same point in both bodies" true for a moving parent too
    from pxr import Gf, UsdGeom

    stage = author(door_mechanism())
    for link in ("jamb", "door", "knob", "bolt"):
        local = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/door_unit/{link}")).GetLocalTransformation()
        assert Gf.IsClose(local, Gf.Matrix4d(1.0), 1e-12)


def test_filtering_base_contacts_and_parents():
    stage = author(door_mechanism())
    base = UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath("/World/door_unit/jamb"))
    assert targets(base.GetFilteredPairsRel()) == ["door", "knob"]  # the bolt meets the strike
    for child in ("knob", "bolt"):
        api = UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath(f"/World/door_unit/{child}"))
        assert targets(api.GetFilteredPairsRel()) == ["door"]
    # knob and bolt are siblings: no filter between them, so the knob's cam can push the bolt
    assert "bolt" not in targets(UsdPhysics.FilteredPairsAPI(
        stage.GetPrimAtPath("/World/door_unit/knob")).GetFilteredPairsRel())


def test_joints_leave_their_pair_uncolliding():
    stage = author(door_mechanism())
    for joint in JOINTS:
        assert UsdPhysics.Joint(joint_at(stage, joint.child, joint.name)).GetCollisionEnabledAttr().Get() is False


def test_limits_in_usd_units():
    stage = author(door_mechanism())
    door = UsdPhysics.RevoluteJoint(joint_at(stage, "door", "door"))
    assert door.GetUpperLimitAttr().Get() == pytest.approx(85.9437, abs=1e-3)  # 1.5 rad in degrees
    bolt = UsdPhysics.PrismaticJoint(joint_at(stage, "bolt", "bolt"))
    assert (bolt.GetLowerLimitAttr().Get(), bolt.GetUpperLimitAttr().Get()) == pytest.approx((-0.013, 0.0))


def test_without_parents_it_authors_as_before():
    flat = tuple(M.Joint(j.name, j.kind, j.child, j.anchor, j.axis, j.lower, j.upper) for j in JOINTS)
    stage = author(door_mechanism(flat, base_contacts=()))
    for joint in flat:
        assert targets(UsdPhysics.Joint(joint_at(stage, joint.child, joint.name)).GetBody0Rel()) == ["jamb"]
    base = UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath("/World/door_unit/jamb"))
    assert targets(base.GetFilteredPairsRel()) == ["bolt", "door", "knob"]
    assert not stage.GetPrimAtPath("/World/door_unit/knob").HasAPI(UsdPhysics.FilteredPairsAPI)


@pytest.mark.parametrize("bad, message", [
    (dict(knob=dict(parent="knob")), "must be another moving link"),
    (dict(knob=dict(parent="jamb")), "must be another moving link"),
    (dict(knob=dict(parent="nobody")), "must be another moving link"),
    (dict(door=dict(parent="knob")), "loop"),
])
def test_bad_trees_are_refused(bad, message):
    joints = tuple(M.Joint(**{**j.__dict__, **bad.get(j.name, {})}) for j in JOINTS)
    with pytest.raises(ValueError, match=message):
        door_mechanism(joints)._check()


def test_unknown_base_contact_is_refused():
    with pytest.raises(ValueError, match="base_contacts"):
        door_mechanism(base_contacts=("strike",))._check()


def test_start_positions_follow_the_joints():
    joints = tuple(M.Joint(**{**j.__dict__, "start": 0.14 if j.name == "door" else 0.0}) for j in JOINTS)
    spec = door_mechanism(joints)
    spec._check()
    assert M.start_positions(spec) == [0.14, 0.0, 0.0]
    assert M.start_positions(door_mechanism()) == [0.0, 0.0, 0.0]  # default: every joint at 0, as before


def test_start_outside_the_limits_is_refused():
    joints = tuple(M.Joint(**{**j.__dict__, "start": -0.2 if j.name == "door" else 0.0}) for j in JOINTS)  # below 0
    with pytest.raises(ValueError, match="start"):
        door_mechanism(joints)._check()


def test_a_turned_box_is_authored_turned_and_bounded():
    import math

    from pxr import Gf, UsdGeom

    lip = M.Shape("box", (0.63, -0.02, 0.9), (0.02, 0.02, 0.05), yaw=math.pi / 4)
    base = M.Link("jamb", 5.0, (*BASE.shapes, lip))
    spec = M.mechanism("door_unit", (0.0, 0.0, 0.0), base, (DOOR, KNOB, BOLT), JOINTS, MATERIALS,
                       base_contacts=("bolt",))
    stage = author(spec)
    prim = stage.GetPrimAtPath(f"/World/door_unit/jamb/box_{len(BASE.shapes)}")
    xf = UsdGeom.Xformable(prim)
    assert [op.GetOpName() for op in xf.GetOrderedXformOps()] == ["xformOp:translate", "xformOp:rotateZ",
                                                                  "xformOp:scale"]
    # the unit cube's corner (0.5, 0.5) scales to (0.01, 0.01), and turned 45 deg lands straight +y, 0.01 * sqrt(2) out
    corner = xf.GetLocalTransformation().Transform(Gf.Vec3d(0.5, 0.5, 0.0))
    assert (corner[0], corner[1]) == pytest.approx((0.63, -0.02 + 0.01 * math.sqrt(2)))
    lo, hi = lip.bounds()
    assert hi[0] - lo[0] == pytest.approx(0.02 * math.sqrt(2)) and hi[2] - lo[2] == pytest.approx(0.05)
    assert not stage.GetPrimAtPath("/World/door_unit/jamb/box_0").GetAttribute("xformOp:rotateZ").IsValid()


def test_only_boxes_turn():
    bad = M.Link("jamb", 5.0, (M.Shape("cylinder", (0.6, 0.0, 0.9), (0.01, 0.02), yaw=0.3),))
    with pytest.raises(ValueError, match="only a box"):
        M.mechanism("door_unit", (0, 0, 0), bad, (DOOR, KNOB, BOLT), JOINTS, MATERIALS)._check()
