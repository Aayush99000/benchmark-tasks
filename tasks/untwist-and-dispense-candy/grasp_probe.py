"""Fixed joint plans for physical grasp, lift, and release experiments at 20 Hz."""
from dataclasses import dataclass

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

from reach_audit import solve


@dataclass(frozen=True)
class Target:
    surface: str
    arm: str
    point: tuple
    # offsets in metres from the contact pose
    lift: tuple


TARGETS = {
    'hospital-corner':Target('flat_sheet','left',(.282,.125,.684),(0.,0.,.10)),
    'candy': Target('wrapper', 'left', (.38, .031, .757), (0., .012, .085)),
    'duvet': Target('cover', 'right', (.251, -.19, .694), (-.024, .01, .11)),
    'pallet': Target('film', 'right', (.369, -.02, .761), (-.085, 0., .045)),
    'stocking': Target('stocking', 'right', (.35, -.13, .7635), (0., -.02, .09)),
}
# The hold before approaching is included in the trial. The negative control
# uses exactly the same path and timing with the jaws open throughout.
STEPS = 210
BASELINE = 100
LIFT_START = 148
LIFT_END = 178
RELEASE = 195


def target_for(task, variant=None):
    if variant == 'staged-cuff':
        return Target('stocking','right',(.338,-.235,.7635),(0.,-.02,.09))
    if variant == 'single-fold':
        return Target('cover','right',(.17,-.16,.626),(-.005,0.,.10))
    if variant == 'front-edge':
        return Target('film','right',(.349,.0,.727),(-.105,-.010,.035))
    return TARGETS[task]


def joint_plan(task, close, variant=None):
    target = target_for(task,variant)
    base = station.BASES[target.arm]
    point = np.asarray(target.point)
    down = np.diag([1., -1., -1.])
    if task == 'candy':
        down = np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
    if variant == 'single-fold':
        from yam_arena.urdf import axis_angle_matrix
        # Oblique approach beneath the overhang, outside the mattress.
        down = axis_angle_matrix(np.array([0.,1.,0.]),np.radians(-55.)) @ down
    if variant == 'front-edge':
        from yam_arena.urdf import axis_angle_matrix
        down = axis_angle_matrix(np.array([0.,1.,0.]),np.radians(-30.)) @ down
    lifted = point + target.lift
    raised = point + [0., 0., .09]
    seed = READY
    poses = []
    for location in (raised, point, lifted):
        q, error_m, error_rad = solve(location, down, seed, base)
        if error_m > .002 or error_rad > .03:
            raise ValueError(f'{task} waypoint {location} has IK error {error_m:g} m, {error_rad:g} rad')
        poses.append(q)
        seed = q
    open_jaws, closed_jaws = 1., (0. if close else 1.)
    knots = [(0, READY, open_jaws), (BASELINE, READY, open_jaws),
             (121, poses[0], open_jaws), (137, poses[1], open_jaws),
             (LIFT_START, poses[1], closed_jaws), (LIFT_END, poses[2], closed_jaws),
             (RELEASE - 1, poses[2], closed_jaws), (RELEASE + 4, poses[2], open_jaws),
             (STEPS, poses[2], open_jaws)]
    plan = np.tile(np.r_[READY, 1., READY, 1.], (STEPS + 1, 1))
    arm_slice = slice(0, 7) if target.arm == 'left' else slice(7, 14)
    for (a, qa, ga), (b, qb, gb) in zip(knots, knots[1:]):
        start, finish = np.r_[qa, ga], np.r_[qb, gb]
        for step in range(a, b + 1):
            t = (step - a) / (b - a)
            plan[step, arm_slice] = start + (finish - start) * t*t*(3 - 2*t)
    return plan
