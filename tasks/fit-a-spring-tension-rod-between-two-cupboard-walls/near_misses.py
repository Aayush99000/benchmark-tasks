"""Near-successes and unintended strategies, scored offline by factory/check.py. Rod states come from the part's own
kinematics (ROD.state), so the bounding box and the caps move with the slide and any tilt. Offline there is no
robot, so the release check passes; _run() below also checks the release rule with a robot part placed on the rod."""

import math
import os

import task
from task import BAND_Z0, BAND_Z1, CAP_D, FREE, GAP, ROD, TOP, WALL_CX, WALL_H, WALL_X0, WALL_X1

R = CAP_D / 2  # cap radius
CAP_T = task.CAP_T

os.environ[task.OFFLINE_FLAG] = "1"  # the task credits these offline cases as let go

FIT_Q = FREE - GAP  # 25 mm squeezed when spanning the gap


def qx(deg):
    """Rolled about x: the +y end (tube cap) goes up for positive angles."""
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _face_span(a):
    """Cap face to cap face (m) along a rod turned `a` rad (tilted or skewed) whose cap corners just touch the walls."""
    return (GAP - 2 * R * math.sin(a)) / math.cos(a)


def _turned(deg, z, x, quat, along, vel=None):
    """A fitted rod turned `deg` so that its cap corners touch both walls. `along(a)` gives the axis direction."""
    a = math.radians(deg)
    q = FREE - _face_span(a) if deg else FREE - GAP
    d = along(a)
    o = [-(q / 2) * c for c in d]  # the axis midpoint sits at (x, 0, z); the body origin is q/2 back along the axis
    return {"rod": ROD.state(joints={"slide": q}, pos=(x + o[0], o[1], TOP + z + o[2]),
                             quat_xyzw=quat(deg) if deg else None, lin_vel=vel)}


def fitted(z, x=WALL_CX, tilt=0.0, vel=None):
    """Spanning the gap, axis midpoint at height z, the +y end raised by `tilt` degrees."""
    return _turned(tilt, z, x, qx, lambda a: (0.0, math.cos(a), math.sin(a)), vel)


def skewed(deg, z=None):
    """Fitted level but turned `deg` about the vertical (the +y cap further back)."""
    return _turned(deg, MID if z is None else z, WALL_CX, qz, lambda a: (-math.sin(a), math.cos(a), 0.0))


def _solve(gap_between_centres):
    """Angle (deg) at which the two tip centres are `gap_between_centres` apart across the axis' turn."""
    lo, hi = 0.0, 0.5
    for _ in range(80):
        a = (lo + hi) / 2
        off = (_face_span(a) - CAP_T) * math.sin(a)
        lo, hi = (a, hi) if off < gap_between_centres else (lo, a)
    return math.degrees((lo + hi) / 2)


MID = (BAND_Z0 + BAND_Z1) / 2


def short(gap):
    """Level in the band, tube cap on its wall, the bar's cap `gap` m short of the other wall."""
    return {"rod": ROD.state(joints={"slide": FIT_Q + gap}, pos=(WALL_CX, -FIT_Q / 2, TOP + MID))}


def skew_deg_for_q(q):
    """Skew (deg) at which a rod spanning the gap with its cap corners on the walls is squeezed by q (m)."""
    lo, hi = 0.0, 60.0
    for _ in range(80):
        d = (lo + hi) / 2
        lo, hi = (d, hi) if FREE - _face_span(math.radians(d)) > q else (lo, d)
    return (lo + hi) / 2


def tilt_for(dz):
    return _solve(dz)


def skew_for(dx):
    return _solve(dx)


CASES = [
    ("fitted level in the middle of the band", fitted(MID), 1.0, True),
    ("fitted at the bottom of the band", fitted(BAND_Z0), 1.0, True),
    ("fitted at the top of the band", fitted(BAND_Z1), 1.0, True),
    ("self-supporting, 1 mm below the band: partial", fitted(BAND_Z0 - 0.001), 0.5, False),
    ("self-supporting, 1 mm above the band: partial", fitted(BAND_Z1 + 0.001), 0.5, False),
    ("self-supporting but off-height, 40 mm up: partial", fitted(0.040), 0.5, False),
    ("self-supporting, top of the caps level with the wall tops (limit)", fitted(WALL_H - R), 0.5, False),
    ("tip tops 1 mm above the wall tops, centres on the walls: partial", fitted(WALL_H - R + 0.001), 0.5,
     False),
    ("tip centres 1 mm above the wall tops: not bearing", fitted(WALL_H + 0.001), 0.0, False),
    ("fitted, ends 10 mm apart in height (limit)", fitted(MID, tilt=tilt_for(0.010)), 1.0, True),
    ("self-supporting but off-level, ends 11 mm apart: partial", fitted(MID, tilt=tilt_for(0.011)), 0.5, False),
    ("self-supporting but off-level, ends 31 mm apart: partial", fitted(MID, tilt=tilt_for(0.031)), 0.5, False),
    ("self-supporting low, rod 3 mm clear of the table (limit): partial",
     fitted(R + 0.003), 0.5, False),
    ("squeezed between the walls but 2 mm off the table: not credited", fitted(R + 0.002), 0.0, False),
    ("squeezed between the walls, resting on the table", fitted(R), 0.0, False),
    ("self-supporting, skewed 10 deg about the vertical (allowed in the real-trial text): partial", skewed(10), 0.5, False),
    ("fitted, tips 9 mm apart front to back", skewed(skew_for(0.009)), 1.0, True),
    ("self-supporting, tips 11 mm apart front to back: partial", skewed(skew_for(0.011)), 0.5, False),
    ("fitted but still sliding down", fitted(MID, vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("off-height and sliding down", fitted(0.040, vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("tips centred on the walls' front edge, half on: partial", fitted(MID, x=WALL_X0), 0.5, False),
    ("tip centres 1 mm in front of the walls' front edge: not bearing", fitted(MID, x=WALL_X0 - 0.001), 0.0, False),
    ("tip centres 1 mm behind the walls' back edge: not bearing", fitted(MID, x=WALL_X1 + 0.001), 0.0, False),
    ("spanning the gap, tips flush with the front edge", fitted(MID, x=WALL_X0 + R), 1.0, True),
    ("tips 1 mm over the front edge: partial", fitted(MID, x=WALL_X0 + R - 0.001), 0.5, False),
    ("spanning the gap at the back edge, tips flush", fitted(MID, x=WALL_X1 - R), 1.0, True),
    ("one tip face 0.5 mm off its wall (contact tolerance; not a state a released rod can rest in)", short(0.0005), 1.0,
     True),
    ("one tip face 0.6 mm off its wall (not touching)", short(0.0006), 0.0, False),
    ("one tip face 1.0 mm off its wall (not touching)", short(0.001), 0.0, False),
    ("one tip 3 mm short of its wall (not touching)", short(0.003), 0.0, False),
    ("wedged across the gap at a steep skew, spring squeezed 4 mm: not pressed", skewed(skew_deg_for_q(0.004)), 0.0,
     False),
    ("wedged across the gap at a skew, squeezed 5 mm (limit; skew allowed in the real-trial text): partial", skewed(skew_deg_for_q(0.005)), 0.5,
     False),
    ("lying on the table between the walls, unsqueezed",
     {"rod": ROD.state(pos=(WALL_CX, 0.0, TOP + R), quat_xyzw=qz(25))}, 0.0, False),
    ("lying across the wall tops",
     {"rod": ROD.state(pos=(WALL_CX, 0.0, TOP + WALL_H + R))}, 0.0, False),
    ("still on its risers", {}, 0.0, False),
    ("squeezed on the risers", {"rod": ROD.state(joints={"slide": 0.03})}, 0.0, False),
]


def _release_cases(start) -> int:
    """The release rule, which check.py cannot reach offline: a robot part within 5 mm of the rod, or no measurement."""
    bad = 0
    checks = [
        ("held by a gripper (0 mm), fitted in the band", fitted(MID), 0.0, {"rod": 0.0}),
        ("held by a gripper (0 mm), off-height", fitted(0.040), 0.0, {"rod": 0.0}),
        ("an arm link 4.9 mm from the rod, fitted in the band", fitted(MID), 0.0, {"rod": 0.0049}),
        ("nearest arm link 5.0 mm from the rod, fitted in the band (limit)", fitted(MID), 1.0, {"rod": 0.005}),
        ("nearest arm link 5.0 mm from the rod, off-level (limit): partial", fitted(MID, tilt=tilt_for(0.011)), 0.5,
         {"rod": 0.005}),
    ]
    for label, changes, want, gaps in checks:
        task._GAP.clear()
        task._GAP.update(gaps)
        st = {**start, **changes}
        got = 1.0 if task.success(st) else float(task.score(st))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._GAP.clear()
    os.environ[task.OFFLINE_FLAG] = "0"  # in the simulator, before the first step has measured the arms
    got = float(task.score({**start, **fitted(MID)}))
    os.environ[task.OFFLINE_FLAG] = "1"
    flag = "ok  " if got == 0.0 else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag} {got:4.2f}  no arm measurement yet: fails closed")
    return bad + _hold_cases(start) + _surface_cases(start)


def _face_pose(tri, rod, lift):
    """A 4x4 pose that lays triangle ``tri`` (3, 3; link frame) flat over the fitted rod's outer tube, ``lift`` m above
    its top, turned so that every corner is more than 10.5 mm sideways from the tube's axis (none of them near it)."""
    import numpy as np

    a, b, c = tri
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n)
    # rotation taking the face normal to -z (face down onto the rod)
    z = -n
    x = (b - a) / np.linalg.norm(b - a)
    y = np.cross(z, x)
    r0 = np.array([x, y, z])  # rows: link-frame axes mapped to world x, y, z
    boxes = task.ROD.world_boxes(task._poses(rod))
    _, ctr, _, size = max(boxes, key=lambda bx: bx[3][1])  # the outer tube, the longest box
    top = ctr[2] + size[2] / 2
    for w1 in np.linspace(0.1, 0.8, 15):
        for w2 in np.linspace(0.1, 0.8, 15):
            if w1 + w2 > 0.9:
                continue
            pt = a + w1 * (b - a) + w2 * (c - a)
            for th in np.linspace(0, np.pi, 90, endpoint=False):
                rz = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
                r = rz @ r0
                t = np.array([ctr[0], ctr[1], top + lift]) - r @ pt
                v = tri @ r.T + t
                if np.all(np.abs(v[:, 0] - ctr[0]) > size[0] / 2 + 0.005):
                    pose = np.eye(4)
                    pose[:3, :3], pose[:3, 3] = r, t
                    return pose
    return None


def _surface_cases(start) -> int:
    """The release rule against a link face lying flat on the rod with its corners far away: the old vertex-only
    distance missed this; the sampled-surface distance must not."""
    import numpy as np
    from yam_arena.analysis import box_signed_distance, read_stl

    bad = 0
    rod = fitted(MID)["rod"]
    boxes = task.ROD.world_boxes(task._poses(rod))

    def corners_gap(tri, pose):
        v = tri @ pose[:3, :3].T + pose[:3, 3]
        return min(float(box_signed_distance(v, c, r, sz).min()) for _, c, r, sz in boxes)

    def case(label, tri, lift, want):
        nonlocal bad
        pose = _face_pose(tri, rod, lift)
        if pose is None:
            print(f"FAIL       {label}: no placement found")
            bad += 1
            return
        gap = task.robot_gap({"face": pose}, rod, {"face": task._sample(tri[None])})
        task._GAP.clear()
        task._GAP["rod"] = gap
        st = {**start, "rod": rod}
        got = 1.0 if task.success(st) else float(task.score(st))
        ok = got == want
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {got:4.2f}  {label}: reads {gap * 1000:.1f} mm "
              f"(corners alone {corners_gap(tri, pose) * 1000:.1f} mm)")

    plate = np.array([[[-0.05, -0.05, 0], [0.05, -0.05, 0], [0.05, 0.05, 0]],
                      [[-0.05, -0.05, 0], [0.05, 0.05, 0], [-0.05, 0.05, 0]]])
    case("synthetic 100 mm plate face 0.5 mm over the rod", plate[0], 0.0005, 0.0)
    case("synthetic 100 mm plate face 6 mm over the rod", plate[0], 0.006, 1.0)
    import xml.etree.ElementTree as ET

    from yam_arena import ASSET_DIR, YAM_URDF
    for link in ("tip_left", "gripper", "link5", "link3"):
        el = next(e for e in ET.parse(YAM_URDF).getroot().iter("link") if e.get("name") == link)
        tris = np.concatenate([read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename"))
                               for col in el.findall("collision")])
        big = tris[np.argmax(np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1))]
        case(f"{link}'s largest hull face 0.5 mm over the rod", big, 0.0005, 0.0)
    task._GAP.clear()
    return bad


def _hold_cases(start) -> int:
    """The 10 s hands-off hold and its 300 g test load, as the simulator's on_step hook runs them (20 Hz)."""
    bad = 0
    os.environ[task.OFFLINE_FLAG] = "0"
    fit, low = fitted(MID)["rod"], fitted(MID - 0.003)["rod"]  # `low`: both caps 3 mm lower (slipped)

    def run(states, released=lambda t: True):
        task._HOLD.clear()
        loads = []
        for k, (t, rod) in enumerate(states):
            loads.append(task.hold_step(t, rod, released(t)))
        return loads

    def report(label, ok):
        nonlocal bad
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}       {label}")

    ts = [k / 20 for k in range(241)]  # 12 s
    loads = run([(t, fit) for t in ts])
    on = [t for t, f in zip(ts, loads) if f]
    report(f"test load on from {on[0]:.2f} s to {on[-1]:.2f} s of the hold (5 s to 10 s)",
           abs(on[0] - 5.0) < 1e-6 and abs(on[-1] - 9.95) < 1e-6 and max(loads) == task.LOAD_N)
    task._GAP.clear()
    task._GAP["rod"] = 0.01
    for held, want in ((9.9, 0.0), (10.0, 1.0)):
        run([(t, fit) for t in ts if t <= held + 1e-9])
        st = {**start, "rod": fit}
        got = 1.0 if task.success(st) else float(task.score(st))
        report(f"fitted in the band, hands off {held:.1f} s: {got:.1f} (want {want:.1f})", got == want)
    run([(t, low if t >= 7.0 else fit) for t in ts])
    report(f"tips slip 3 mm under the test load at 7 s: hold restarts ({task._HOLD['held']:.2f} s held at 12 s)",
           task._HOLD["held"] < 5.0)
    run([(t, fit) for t in ts], released=lambda t: not (6.0 <= t < 6.5))
    report(f"a robot part back near the rod at 6 s: hold restarts ({task._HOLD['held']:.2f} s held at 12 s)",
           task._HOLD["held"] < 6.0)
    shake = fitted(MID, vel=(0.0, 0.0, -0.02))["rod"]  # moving 2 cm/s, in place
    run([(t, shake if 6.0 <= t < 6.2 else fit) for t in ts])
    report(f"jiggling 2 cm/s for 0.2 s under the load (6 s): hold continues ({task._HOLD['held']:.2f} s held)",
           task._HOLD["held"] >= 10.0)
    run([(t, shake if 3.0 <= t < 3.2 else fit) for t in ts])
    report(f"moving 2 cm/s before the load (3 s): hold restarts ({task._HOLD['held']:.2f} s held at 12 s)",
           task._HOLD["held"] < 9.0)
    loads = run([(t, fit) for t in ts])
    report(f"load hung gently: {loads[100]:.2f} N at 5.0 s, {loads[110]:.2f} N at 5.5 s, {loads[199]:.2f} N at 9.95 s",
           0 < loads[100] < task.LOAD_N and loads[110] == task.LOAD_N and 0 < loads[199] < task.LOAD_N)
    run([(t, low if t >= 11.0 else fit) for t in ts])
    report("tips move 3 mm after the test load is off (11 s): the hold has passed and keeps counting",
           task._HOLD["held"] >= 10.0)
    task._HOLD.clear()
    st = {**start, "rod": fit}
    report("no hold measured yet: fails closed", float(task.score(st)) == 0.0)
    task._GAP.clear()
    os.environ[task.OFFLINE_FLAG] = "1"
    return bad


def _run() -> int:
    start = {"rod": ROD.state()}  # score() reads only the rod
    bad = 0
    for label, changes, want, want_success in CASES:
        st = {**start, **changes}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad + _release_cases(start)


if __name__ == "__main__":
    raise SystemExit(1 if _run() else 0)
