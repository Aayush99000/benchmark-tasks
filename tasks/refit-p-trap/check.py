"""Offline checks (no GPU): demo reach/timing, and the slip-joint model with synthetic frames. One task file per process."""
import math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, '/data/yam-isaac-lab-arena')
from yam_arena.task_file import load
from yam_arena.demo import ArmPlan
from yam_arena import station
TASK = load(Path(sys.argv[1] if len(sys.argv) > 1 else 'refit_p_trap.py'))
M = TASK.module
import ptrapkit
from ptrapkit import Frame, Hand

def reach():
    plans = M.demo()
    for arm in ('left', 'right'):
        plan = ArmPlan(list(plans[arm]), station.BASES[arm])
        worst = (0, 0, '')
        t, dt = 0.0, 0.05
        bad = []
        while t <= plan.duration:
            plan.action7(t)
            pos, rot, f, label = plan.target(t)
            if plan.worst_ik_mm > worst[0] + 1e-9:
                worst = (plan.worst_ik_mm, plan.worst_ik_deg, label)
            t += dt
        print(f'{arm}: duration {plan.duration:.1f}s ({plan.duration*20:.0f} steps) worst IK {worst[0]:.1f} mm / {plan.worst_ik_deg:.1f} deg at "{worst[2]}"')

def per_label_ik():
    plans = M.demo()
    for arm in ('left', 'right'):
        plan = ArmPlan(list(plans[arm]), station.BASES[arm])
        t, dt = 0.0, 0.05
        lab = {}
        from yam_arena import ik
        while t <= plan.duration:
            q7, label = plan.action7(t)
            pos, rot, f, _ = plan.target(t)
            p, r = ik.tcp_pose(plan.q, plan.base_pos)
            e = np.linalg.norm(p - pos) * 1000
            lab[label] = max(lab.get(label, 0), e)
            t += dt
        print(arm, {k: round(v, 1) for k, v in lab.items() if v > 3})

def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])

def jframe(dz=0.0, dxy=(0, 0)):
    # the J-bend upright, seated (+dz), its inlet leg axis at (XI, YI)+dxy
    pos = np.array([M.XI + dxy[0], M.YI + dxy[1], M.Z_JB + dz])
    return Frame(pos=pos, rot=np.eye(3), vel=np.zeros(3), spin=np.zeros(3), com=pos)

def nframe(xy, z):
    pos = np.array([xy[0], xy[1], z])
    return Frame(pos=pos, rot=np.eye(3), vel=np.zeros(3), spin=np.zeros(3), com=pos)

def hand(xy, z, yaw_deg, gap):
    a = math.radians(yaw_deg)
    return Hand(tcp=np.array([xy[0], xy[1], z]), approach=np.array([math.cos(a), math.sin(a), 0]),
                closing=np.array([-math.sin(a), math.cos(a), 0]), gap=gap)

def model_test():
    m = M.MODEL; m.clear(); I = M.INLET
    acc = {}
    # J seated, inlet nut lowered onto the thread start
    nz = M.Z_JB + M.IN_TOP + M.NUT_H / 2
    frames = {'jbend': jframe(), 'nut_in': nframe((M.XI, M.YI), nz), 'nut_out': nframe((M.XO, M.YO), M.OUT_NUT_PARK + M.NUT_H/2)}
    m.update(frames, acc)
    print('inside %.1f mm radial %.2f mm tilt %.1f threaded %s' % (I.inside*1000, I.radial*1000, I.tilt, I.threaded))
    # a hand grips and swings 90 deg clockwise in 30 steps, 17 times
    for k in range(17):
        for s in range(31):
            yaw = 60 - 90 * s / 30
            frames['hand_right'] = hand((M.XI, M.YI), I._nut_pose_z(), yaw, 0.062)
            m.update(frames, acc)
        frames['hand_right'] = hand((M.XI, M.YI), I._nut_pose_z(), -30, 0.094)  # let go
        m.update(frames, acc)
        for s in range(31):
            frames['hand_right'] = hand((M.XI, M.YI), I._nut_pose_z(), -30 + 90 * s / 30, 0.094)
            m.update(frames, acc)
    print('turns %.2f snug %s caught %s nut_z-leg_top %.1f mm' % (I.turns(), I.snug(), I.caught, (I.nut_z - M.NUT_H/2 - I.leg_top)*1000))
    # a misaligned J-bend (3 mm off) does not thread
    m.clear(); frames = {'jbend': jframe(dxy=(0.003, 0)), 'nut_in': nframe((M.XI, M.YI), nz), 'nut_out': frames['nut_out']}
    m.update(frames, {}); print('3 mm off: threaded', I.threaded, 'radial %.1f' % (I.radial*1000))
    # the J-bend 8 mm low: the pipe only 22 mm in... and 25 mm low: not on
    for dz in (-0.008, -0.025):
        m.clear(); frames['jbend'] = jframe(dz=dz); frames['nut_in'] = nframe((M.XI, M.YI), nz + dz)
        m.update(frames, {}); print(f'J {dz*1000:.0f} mm low: inside {I.inside*1000:.1f} mm threaded {I.threaded}')


def end_errors():
    from yam_arena import ik
    plans = M.demo()
    for arm in ('left', 'right'):
        ws = list(plans[arm]); plan = ArmPlan(ws, station.BASES[arm])
        t, dt, k, acc = 0.0, 0.05, 0, 0.0
        ends = np.cumsum([w.duration for w in ws])
        out = []
        while t <= plan.duration and k < len(ws):
            plan.action7(t)
            if t + dt > ends[k]:
                pos, rot, f, label = plan.target(t)
                p, r = ik.tcp_pose(plan.q, plan.base_pos)
                e = np.linalg.norm(p - pos) * 1000
                if e > 2: out.append(f'{ws[k].label}:{e:.0f}')
                k += 1
            t += dt
        print(arm, 'waypoint-end misses >2mm:', out[:20])

def hand_clearance():
    """Min distance between the two grippers' key points (pad tips, pad middles, finger roots, palm) along the plan."""
    from yam_arena.constants import FINGER_OPEN
    plans = M.demo()
    P = {a: ArmPlan(list(plans[a]), station.BASES[a]) for a in ('left', 'right')}
    t, worst = 0.0, (9.0, 0, '', '')
    while t <= P['left'].duration:
        pts = {}
        for a, plan in P.items():
            pos, rot, f, label = plan.target(t)
            gap = 0.094 * f / FINGER_OPEN
            c, ap = rot[:, 0], rot[:, 2]
            pts[a] = ([pos + s * c * (gap / 2 + 0.008) + ap * d for s in (-1, 1) for d in (0.013, -0.004, -0.021, -0.045)]
                      + [pos - 0.07 * ap + s * c * 0.03 for s in (-1, 0, 1)], label)
        dmin = min(np.linalg.norm(p - q) for p in pts['left'][0] for q in pts['right'][0])
        if dmin < worst[0]:
            worst = (dmin, t, pts['left'][1], pts['right'][1])
        t += 0.05
    print('closest hands: %.0f mm at t=%.1f (left "%s", right "%s")' % (worst[0] * 1000, worst[1], worst[2], worst[3]))

def joint_jumps():
    plans = M.demo()
    for arm in ('left', 'right'):
        plan = ArmPlan(list(plans[arm]), station.BASES[arm])
        t, prev, worst = 0.0, None, []
        while t <= plan.duration:
            a, label = plan.action7(t)
            if prev is not None:
                d = np.abs(a[:6] - prev[:6])
                if d.max() > 0.08:
                    worst.append((round(t, 2), label, round(float(d.max()), 3), int(d.argmax()) + 1))
            prev = a
            t += 0.05
        print(arm, 'joint steps > 0.08 rad per 0.05 s:', len(worst), worst[:8])

if __name__ == "__main__":
    for name in (sys.argv[2:] or ["reach", "end_errors", "hand_clearance"]):
        globals()[name]()
