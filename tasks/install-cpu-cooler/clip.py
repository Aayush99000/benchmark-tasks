"""The cooler's clip springs and lever, applied before every physics step (240 Hz).

The cooler (assets/cooler.usda) is a floating-base articulation. Its fixed clip strap (-y) is part of its body. The
loose strap (+y) has a slide along z (``leg_pos_slide_joint``, 0 = loose, negative = pushed down) sprung up, and a
flex about x at its top (``leg_pos_flex_joint``, 0 = upright, outwards positive) sprung upright. The lever
(``lever_joint``, on the slide; 0 = unlocked, along -x, its end raised 20 degrees; LEVER_SPAN = locked, along +x; it swings over the top
in the plane of the shroud's side) has an over-centre spring: past straight up it is pulled over the top
to locked and held stiffly, otherwise back to unlocked. Its cam raises the loose strap's spring target by up to CAM_LIFT as it goes over,
clamping the cooler. Values as work/cooler/make_cooler.py authors them.
Isaac Lab writes every articulation's joint targets to PhysX before each physics substep (``write_data_to_sim``);
this wraps that call on the cooler, so it decides on the joint positions PhysX reported after the previous substep.
"""

from __future__ import annotations

import math

PER_DEG = 180.0 / math.pi  # N m per degree -> N m per radian
LEGS = ("pos",)
SIGN = {"neg": -1.0, "pos": 1.0}
LEG_STIFFNESS = 600.0  # N/m: hooked, the loose strap pulls up about 4.3 N
LEG_DAMPING = 20.0
FLEX_STIFFNESS = 0.004 * PER_DEG
FLEX_DAMPING = 0.0005 * PER_DEG
FLEX_PRELOAD = math.radians(1.0)  # held this far inwards, against the upright stop
LEVER = "lever_joint"
LEVER_SPAN = math.radians(180.0)  # from 20 degrees up on one side to 20 degrees down on the other
LEVER_SNAP = math.radians(110.0)  # over centre: past straight up (70 degrees of travel) the spring takes it over
LOCKED_AT = math.radians(150.0)  # counts as locked (for scoring) past this
LEVER_STIFFNESS = 0.0005 * PER_DEG  # unlocked, a light detent; forty times this locked
LEVER_DAMPING = 0.0002 * PER_DEG
CAM_LIFT = 0.004
HOOK_DROP = 0.0071  # the loose strap's travel from loose to hooked (its window's bottom edge under the lug)
HOOK_TOL = (0.0006, 0.0010)  # hooked: travel within this below and above HOOK_DROP (locked, the cam pulls it up
# against the lug's underside and the cooler's fixed end up into its strap's slack, about 0.5 mm)...
FLEX_TOL = math.radians(1.5)  # ...and hanging within this of upright (the lug in its window)


class ClipSpring:
    def __init__(self, cooler):
        import torch

        self.cooler = cooler
        self.dev = cooler.device
        names = [f"leg_{n}_slide_joint" for n in LEGS] + [f"leg_{n}_flex_joint" for n in LEGS] + [LEVER]
        self.ids = dict(zip(names, cooler.find_joints(names, preserve_order=True)[0]))
        self.lever = self.ids[LEVER]
        self.cam = self.ids["leg_pos_slide_joint"]
        stiff = [LEG_STIFFNESS] * len(LEGS) + [FLEX_STIFFNESS] * len(LEGS) + [LEVER_STIFFNESS]
        damp = [LEG_DAMPING] * len(LEGS) + [FLEX_DAMPING] * len(LEGS) + [LEVER_DAMPING]
        ids = [self.ids[n] for n in names]
        cooler.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([stiff], device=self.dev), joint_ids=ids)
        cooler.write_joint_damping_to_sim_index(damping=torch.tensor([damp], device=self.dev), joint_ids=ids)
        self.locked = False
        self.reset()
        write = cooler.write_data_to_sim

        def write_with_spring():
            self.update()
            write()

        cooler.write_data_to_sim = write_with_spring

    def q(self) -> dict:
        q = self.cooler.data.joint_pos
        q = getattr(q, "torch", q)[0]
        return {n: float(q[i]) for n, i in self.ids.items()}

    def _targets(self, lever: float, cam: float) -> None:
        import torch

        ids = [self.ids[f"leg_{n}_flex_joint"] for n in LEGS] + [self.lever, self.cam]
        t = [-SIGN[n] * FLEX_PRELOAD for n in LEGS] + [lever, cam]
        self.cooler.set_joint_position_target_index(target=torch.tensor([t], device=self.dev), joint_ids=ids)

    def _stiff_lever(self, locked: bool) -> None:
        import torch

        k = LEVER_STIFFNESS * (40.0 if locked else 1.0)
        self.cooler.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]], device=self.dev),
                                                       joint_ids=[self.lever])
        self.locked = locked

    def update(self) -> None:
        lever = self.q()[LEVER]
        locked = lever > LEVER_SPAN - LEVER_SNAP
        if locked != self.locked:
            self._stiff_lever(locked)
        self._targets(LEVER_SPAN if locked else 0.0, CAM_LIFT * min(max(lever / LEVER_SPAN, 0.0), 1.0))

    def reset(self) -> None:
        self._stiff_lever(False)
        self._targets(0.0, 0.0)

    def state(self) -> dict:
        """{'lever': travel from unlocked (rad), 'locked': bool, 'pos': (travel down m, flex out rad, hooked)}."""
        q = self.q()
        out = {"lever": q[LEVER], "locked": q[LEVER] > LOCKED_AT}
        for n in LEGS:
            drop = -q[f"leg_{n}_slide_joint"]
            flex = SIGN[n] * q[f"leg_{n}_flex_joint"]
            out[n] = (drop, flex, HOOK_DROP - HOOK_TOL[1] < drop < HOOK_DROP + HOOK_TOL[0] and flex < FLEX_TOL)
        return out
