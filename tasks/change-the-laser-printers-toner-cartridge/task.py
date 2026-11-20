"""On the station table, facing the arms: a light grey desktop mono laser printer (38 cm wide, 36 cm deep, 22 cm
tall). Its front door is open: the door is hinged along the bottom of the front and folds down flat toward the arms,
and its upper end turns up as a short lip (shut, that part is the front of the printer's top). Inside, in the toner
bay behind the door, sits the used toner cartridge, a black moulded cartridge with a grey carry handle along the top
of its front. A new cartridge of the same kind, with an orange handle, stands on the table to the right of the
printer. The table to the left of the printer is clear.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# --- the printer (world: x away from the arms, y to the robot's left, z up; heights above the table) ------------
# Outside size as published for the HP LaserJet Pro M404n (381 x 357 x 216 mm). Part frame: origin on the table under
# the middle of the front face.
XF = 0.24  # front face
W, D, H = 0.381, 0.357, 0.216
HB = 0.035  # bay floor (the paper cassette is under it)
BH = 0.170  # bay half-width between the side guides (cartridge 33 cm long: 5 mm play each end)
BAY_D = 0.108  # bay depth from the front face to the back stop
FL = BAY_D  # depth of the door's top lip (shut, the front of the top over the whole bay)
T = 0.012  # door and top thickness
HP = H - HB  # door panel height, hinge to top

GREY, GREY_D, BAY_C = (0.80, 0.80, 0.78), (0.62, 0.62, 0.61), (0.20, 0.20, 0.21)
PANEL_C, TRAY_C = (0.12, 0.13, 0.15), (0.30, 0.30, 0.31)


def _b(x0, x1, y0, y1, z0, z1, color, collide=True, friction=None):
    """A box from part-frame extents (x from the front face, z above the table)."""
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color,
                     collide=collide, friction=friction)


ABS = (0.35, 0.30)
PRINTER_SHAPES = [
    _b(0.0, D, -W / 2, W / 2, 0.0, HB, GREY, friction=ABS),  # base: cassette and bay floor
    _b(0.0, D, BH, W / 2, HB, H, GREY, friction=ABS),  # side walls (their inner faces are the bay's guides)
    _b(0.0, D, -W / 2, -BH, HB, H, GREY, friction=ABS),
    _b(BAY_D, D, -BH, BH, HB, H, GREY, friction=ABS),  # back stop and the body behind the bay
    # looks only
    _b(-0.002, 0.0, -0.172, 0.172, 0.006, HB - 0.006, GREY_D, collide=False),  # paper cassette front
    _b(0.0005, BAY_D - 0.001, -BH + 0.001, BH - 0.001, HB - 0.0005, HB, BAY_C, collide=False),  # dark bay floor
    _b(BAY_D - 0.001, BAY_D, -BH + 0.001, BH - 0.001, HB, H - T, BAY_C, collide=False),  # dark back of the bay
    _b(0.16, 0.33, -0.15, 0.15, H, H + 0.002, TRAY_C, collide=False),  # output tray
    _b(0.118, 0.150, -0.185, -0.105, H, H + 0.004, PANEL_C, collide=False),  # control panel
    _b(0.122, 0.142, 0.150, 0.172, H, H + 0.004, PANEL_C, collide=False),  # door release button
]

# The door, written open (joint 0, resting on its stop): the panel lies flat toward the arms with its inner face
# level with the bay floor, and the lip stands up at its end. Turning it +90 degrees about the hinge (y axis through
# the bay floor's front edge) shuts it.
DOOR_HW = BH + 0.004
DOOR_SHAPES = (
    _b(-HP, 0.0, -DOOR_HW, DOOR_HW, HB - T, HB, GREY, friction=ABS),  # panel
    _b(-HP, -HP + T, -DOOR_HW, DOOR_HW, HB - T, HB + FL, GREY, friction=ABS),  # top lip
)
SHUT = math.pi / 2
LATCH_AT = math.radians(1.5)  # the catch engages within 1.5 degrees of shut
DOOR_MASS = 0.35

PRINTER = parts.Part(
    "printer", pos=(XF, 0.0, TOP), shapes=PRINTER_SHAPES,
    joints=[parts.Hinge("door", shapes=DOOR_SHAPES, pivot=(0.0, 0.0, HB), axis=(0.0, 1.0, 0.0), lower=0.0,
                        upper=SHUT, mass=DOOR_MASS, spring=0.0, damping=0.02, max_effort=20.0)],
)

# --- the toner cartridges (part frame: origin at the bottom centre of the body; x toward the printer's back) -----
CL, CD, CH = 0.330, 0.100, 0.085  # length (y), depth (x), height of the body
CAP = 0.008  # grey end caps
BAR = (0.014, 0.160, 0.020)  # handle grip bar: thick (x), long (y), tall (z)
BAR_X = -CD / 2 + 0.030  # bar centre, 3 cm behind the front face
BAR_Z = (CH + 0.015, CH + 0.015 + BAR[2])  # bottom and top of the bar: a 15 mm finger gap under it
POST = (0.014, 0.015)  # handle posts: thick (x), wide (y)
POST_Y = BAR[1] / 2 - POST[1] / 2
CART_MASS = 0.65
BLACK, CAP_C, LABEL_C = (0.06, 0.06, 0.07), (0.42, 0.42, 0.43), (0.85, 0.85, 0.83)
OLD_H, NEW_H = (0.55, 0.55, 0.56), (0.95, 0.45, 0.08)  # handle colours: grey (used), orange (new)


def _cartridge(name, pos, handle_c):
    def c(x0, x1, y0, y1, z0, z1, color, collide=True, friction=None):
        return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                         color=color, collide=collide, friction=friction)

    grip = (0.6, 0.5)
    shapes = [
        c(-CD / 2, CD / 2, -CL / 2 + CAP, CL / 2 - CAP, 0.0, CH, BLACK, friction=ABS),  # body
        c(-CD / 2, CD / 2, CL / 2 - CAP, CL / 2, 0.0, CH, CAP_C, friction=ABS),  # end caps
        c(-CD / 2, CD / 2, -CL / 2, -CL / 2 + CAP, 0.0, CH, CAP_C, friction=ABS),
        c(BAR_X - BAR[0] / 2, BAR_X + BAR[0] / 2, -BAR[1] / 2, BAR[1] / 2, BAR_Z[0], BAR_Z[1], handle_c,
          friction=grip),  # grip bar
        *[c(BAR_X - POST[0] / 2, BAR_X + POST[0] / 2, s * POST_Y - POST[1] / 2, s * POST_Y + POST[1] / 2, CH,
            BAR_Z[0], handle_c, friction=grip) for s in (-1, 1)],  # posts
        # looks only: label on the front, the hopper's rounded top back, the drum shutter strip
        c(-CD / 2 - 0.001, -CD / 2, -0.09, 0.09, 0.02, 0.065, LABEL_C, collide=False),
        parts.cylinder(0.028, CL - 2 * CAP - 0.002, at=(CD / 2 - 0.028, 0.0, CH - 0.028 + 0.0015), color=(0.1, 0.1, 0.11),
                       axis="y", collide=False),
        c(0.020, CD / 2 - 0.002, -0.14, 0.14, CH, CH + 0.001, (0.16, 0.32, 0.18), collide=False),
    ]
    return parts.Part(name, pos=pos, shapes=shapes, mass=CART_MASS, friction=ABS)


SEAT_GAP = 0.006  # the seated cartridge's front stands 6 mm behind the shut door's inner face
SEATED = (XF + SEAT_GAP + CD / 2, 0.0, TOP + HB)  # body bottom centre when seated (its back 2 mm off the stop)
NEW_START = (0.19, -0.39, TOP)
OLD = _cartridge("old_cartridge", SEATED, OLD_H)
NEW = _cartridge("new_cartridge", NEW_START, NEW_H)
OBJECTS = [PRINTER, OLD, NEW]
CARTS = ("old_cartridge", "new_cartridge")

# --- scoring ------------------------------------------------------------------------------------------------------
CLEAR = 0.005  # let go: no hand point within 5 mm of a cartridge (or, for success, of the door)
FALL_V = 0.5  # m/s downward: a cartridge dropped
TIP_MAX = 30.0  # degrees: a cartridge tipped far enough to spill toner
SEAT_XY, SEAT_Z, SEAT_DEG = 0.004, 0.003, 3.0
TABLE_TOL, LEVEL_DEG = 0.005, 5.0
STILL_S, STILL_MM = 1.0, 0.002
OPEN_DOOR_FOOT = (XF - HP - T, XF, -DOOR_HW, DOOR_HW)  # x0, x1, y0, y1 of the open door on the table
PRINTER_FOOT = (XF, XF + D, -W / 2, W / 2)


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def tilt_deg(q) -> float:
    """How far a body's up axis leans from vertical (degrees)."""
    return float(np.degrees(np.arccos(np.clip(_rot(q)[2, 2], -1.0, 1.0))))


def yaw_deg(q) -> float:
    r = _rot(q)
    return float(np.degrees(math.atan2(r[1, 0], r[0, 0])))


def cart_boxes(part, st):
    return part.world_boxes({"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))})


def cart_corners(part, st) -> np.ndarray:
    return np.concatenate([c + (np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                                * (s / 2)) @ r.T for _, c, r, s in cart_boxes(part, st)])


def _overlaps(pts, foot) -> bool:
    x0, x1, y0, y1 = foot
    lo, hi = pts[:, :2].min(0), pts[:, :2].max(0)
    return bool(lo[0] < x1 - EPS and hi[0] > x0 + EPS and lo[1] < y1 - EPS and hi[1] > y0 + EPS)


def out_on_table(part, st) -> bool:
    """Wholly clear of the printer's and the open door's footprints, its lowest point within 5 mm of the table and
    level within 5 degrees."""
    pts = cart_corners(part, st)
    return bool(not _overlaps(pts, PRINTER_FOOT) and not _overlaps(pts, OPEN_DOOR_FOOT)
                and pts[:, 2].min() - TOP <= TABLE_TOL + EPS and tilt_deg(st.quat_xyzw) <= LEVEL_DEG + EPS)


def seated(st) -> bool:
    p = np.asarray(st.pos, float)
    return bool(abs(p[0] - SEATED[0]) <= SEAT_XY + EPS and abs(p[1] - SEATED[1]) <= SEAT_XY + EPS
                and abs(p[2] - SEATED[2]) <= SEAT_Z + EPS and abs(yaw_deg(st.quat_xyzw)) <= SEAT_DEG + EPS
                and tilt_deg(st.quat_xyzw) <= SEAT_DEG + EPS)


def door_boxes(st):
    poses = {"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))}
    for name, (p, q) in st.links.items():
        poses[name] = (np.asarray(p, float), _rot(q))
    return PRINTER.world_boxes({"door": poses["door"]})


def gap_to(hand_pts, boxes) -> float:
    from yam_arena.analysis import box_signed_distance

    return float(min(box_signed_distance(hand_pts, c, r, s).min() for _, c, r, s in boxes))


# --- the trial record (in a module of its own so a second copy of this file shares it) ---------------------------
_REC: dict = sys.modules.setdefault("_toner_swap_record", type(sys)("_toner_swap_record")).__dict__.setdefault("rec", {})


def reset_record() -> None:
    _REC.clear()
    _REC.update(t=0.0, steps=0, known=True, reason="", failed="", latched=False, door=0.0, old_out=False,
                new_seated=False, old_free=False, new_free=False, door_free=False, still=False, hist=[], last_t=None)


def record(states, hand_pts, dt: float) -> None:
    """One control step. ``states`` maps object names to their states and ``hand_pts`` holds every sampled point of
    both grippers and all four fingertips (world); None for either when the simulator could not give it, which fails
    the trial."""
    if not _REC:
        reset_record()
    try:
        if states is None or any(n not in states for n in ("printer", *CARTS)):
            raise ValueError("object state unknown")
        if hand_pts is None or len(hand_pts) == 0 or not np.isfinite(hand_pts).all():
            raise ValueError("hand state unknown")
        pr, old, new = states["printer"], states["old_cartridge"], states["new_cartridge"]
        q = float(pr.joints["door"])
        if not math.isfinite(q):
            raise ValueError("door state not finite")
        gaps = {n: gap_to(hand_pts, cart_boxes(p, states[n])) for n, p in (("old_cartridge", OLD),
                                                                            ("new_cartridge", NEW))}
        gaps["door"] = gap_to(hand_pts, door_boxes(pr))
        vals = []
        for n, p in (("old_cartridge", OLD), ("new_cartridge", NEW)):
            s = states[n]
            pts = cart_corners(p, s)
            if not (np.isfinite(pts).all() and np.isfinite(s.lin_vel).all()):
                raise ValueError(f"{n} state not finite")
            vals.append((n, s, pts))
    except (KeyError, ValueError, AttributeError, TypeError) as e:
        _REC["known"] = False
        _REC["reason"] = str(e)
        _REC["t"] += dt
        _REC["steps"] += 1
        return
    for n, s, pts in vals:
        if float(s.lin_vel[2]) < -FALL_V - EPS:
            _REC["failed"] = _REC["failed"] or f"{n} dropped"
        if tilt_deg(s.quat_xyzw) > TIP_MAX + EPS:
            _REC["failed"] = _REC["failed"] or f"{n} tipped"
        if pts[:, 2].min() < TOP - 0.02:
            _REC["failed"] = _REC["failed"] or f"{n} off the table"
    if q >= SHUT - LATCH_AT - EPS:
        _REC["latched"] = True
    _REC["door"] = q
    _REC["old_out"] = out_on_table(OLD, old)
    _REC["new_seated"] = seated(new)
    _REC["old_free"] = bool(gaps["old_cartridge"] > CLEAR + EPS)
    _REC["new_free"] = bool(gaps["new_cartridge"] > CLEAR + EPS)
    _REC["door_free"] = bool(gaps["door"] > CLEAR + EPS)
    snap = np.concatenate([np.asarray(old.pos, float), np.asarray(new.pos, float),
                           cart_corners(OLD, old).ravel(), cart_corners(NEW, new).ravel(), [q * 0.2]])
    hist = _REC["hist"]
    hist.append((_REC["t"], snap))
    while hist and hist[0][0] < _REC["t"] - STILL_S - 1e-9:
        hist.pop(0)
    _REC["still"] = bool(hist[0][0] <= _REC["t"] - STILL_S + 1e-9 + dt
                         and max(float(np.abs(h - snap).max()) for _, h in hist) <= STILL_MM + EPS)
    _REC["t"] += dt
    _REC["steps"] += 1


def _ok() -> bool:
    return bool(_REC and _REC["known"] and _REC["steps"] > 0 and not _REC["failed"])


def tier() -> float:
    if not _ok() or not (_REC["old_out"] and _REC["old_free"]):
        return 0.0
    if not (_REC["new_seated"] and _REC["new_free"]):
        return 0.25
    if not (_REC["latched"] and _REC["door"] >= SHUT - math.radians(2.0) - EPS):
        return 0.5
    if not (_REC["door_free"] and _REC["still"]):
        return 0.75
    return 1.0


def success(objects):
    return tier() >= 1.0


def score(objects):
    """0.25: the used cartridge out of the printer, let go on the table clear of the printer and its open door.
    0.5: also the new cartridge seated in the bay and let go. 0.75: also the door shut and latched. 1.0: also both
    hands clear of the cartridges and the door and everything still for 1 s. 0 for the whole trial once a cartridge
    was dropped, tipped past 30 degrees or left the table, or the hand or object state was unknown at any step."""
    return tier()


reset_record()

# --- simulator hooks ---------------------------------------------------------------------------------------------
HAND_LINKS = ("tip_left", "tip_right", "gripper")
SAMPLE = 0.003
_HAND_PTS: dict = {}


def hand_points(link: str) -> np.ndarray:
    """Points on a hand link's collision surfaces (link frame): every vertex plus each triangle face sampled about
    every 3 mm, so a flat face resting on a part is seen even when no vertex is near it."""
    if link not in _HAND_PTS:
        import xml.etree.ElementTree as ET

        from yam_arena import ASSET_DIR, YAM_URDF, analysis
        from yam_arena.urdf import rpy_matrix

        pts = []
        for lk in ET.parse(YAM_URDF).getroot().iter("link"):
            if lk.get("name") != link:
                continue
            for col in lk.findall("collision"):
                origin = col.find("origin")
                rot, off = np.eye(3), np.zeros(3)
                if origin is not None:
                    rot = rpy_matrix(np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]))
                    off = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
                tris = analysis.read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ rot.T + off
                pts.append(_sample_triangles(tris))
        if not pts:
            raise ValueError(f"no collision mesh for {link}")
        _HAND_PTS[link] = np.concatenate(pts)
    return _HAND_PTS[link]


def _sample_triangles(tris) -> np.ndarray:
    pts = []
    for t in np.asarray(tris, float):
        edge = max(np.linalg.norm(t[1] - t[0]), np.linalg.norm(t[2] - t[1]), np.linalg.norm(t[0] - t[2]))
        n = max(1, int(math.ceil(edge / SAMPLE)))
        for i in range(n + 1):
            for j in range(n + 1 - i):
                pts.append(t[0] + (t[1] - t[0]) * i / n + (t[2] - t[0]) * j / n)
    return np.unique(np.round(np.array(pts) / 0.0015), axis=0) * 0.0015


def world_hand_points(link_poses) -> np.ndarray:
    """All hand points in the world from (link name, 4x4 world pose) pairs; all six hand links must be present."""
    if len(link_poses) != 2 * len(HAND_LINKS):
        raise ValueError("hand state unknown")
    return np.concatenate([hand_points(n) @ m[:3, :3].T + m[:3, 3] for n, m in link_poses])


def _sim_hand_points(sim) -> np.ndarray:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = parts._quat_to_matrix(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                out.append((n, m))
    return world_hand_points(out)


LATCH_K = 200.0  # N m/rad: the caught latch holds the door shut


@parts.on_reset
def _on_reset(sim) -> None:
    reset_record()
    sim.set_stiffness("printer", "door", 0.0)
    _REC["last_t"] = sim.t


@parts.on_step
def _on_step(sim) -> None:
    if _REC.get("last_t") == sim.t:  # a second copy of this module sharing the record: step it once
        return
    dt = sim.t - (_REC["last_t"] if _REC.get("last_t") is not None else 0.0)
    _REC["last_t"] = sim.t
    try:
        states = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        states = None
    try:
        pts = _sim_hand_points(sim)
    except Exception:  # noqa: BLE001  unknown hand state: no credit
        pts = None
    was = _REC.get("latched", False)
    record(states, pts, max(dt, 0.0))
    if _REC["latched"] and not was:  # the catch engages: from now on it holds the door shut
        sim.set_target("printer", "door", SHUT)
        sim.set_stiffness("printer", "door", LATCH_K)
    if os.environ.get("TRACE_OUT") and _REC["steps"] % 5 == 0:
        import json

        row = dict(t=round(_REC["t"], 3), known=_REC["known"], reason=_REC["reason"], failed=_REC["failed"],
                   door_deg=round(math.degrees(_REC["door"]), 2), latched=_REC["latched"], old_out=_REC["old_out"],
                   new_seated=_REC["new_seated"], old_free=_REC["old_free"], new_free=_REC["new_free"],
                   door_free=_REC["door_free"], still=_REC["still"], score=tier())
        try:
            for n in CARTS:
                s = states[n]
                row[n] = dict(pos=[round(float(v), 4) for v in s.pos], tilt=round(tilt_deg(s.quat_xyzw), 2),
                              yaw=round(yaw_deg(s.quat_xyzw), 2), vz=round(float(s.lin_vel[2]), 3))
        except Exception:  # noqa: BLE001
            pass
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("toner.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# --- demo: fixed waypoints from the published layout, no state read ----------------------------------------------
# Left hand: take the used cartridge by its handle, slide it out along the bay floor onto the open door, lift it
# clear, carry it to the left of the printer and set it down. Right hand: pick the new cartridge up by its handle,
# bring it in front of the bay once the left side is clear, set it on the open door, slide it in to the back stop,
# let go; then, fingers shut, push the door's lip and panel up and in until the door catches.
TIP = 0.013  # pad tips below the TCP
GRIP_DZ = BAR_Z[0] + 0.003 + TIP  # TCP above the cartridge's bottom: pad tips 3 mm above the bar's underside
PINCH = -(BAR[0] / 2 - 0.003)
NEAR = -(BAR[0] / 2 + 0.010)
CLOSED = 0.0
YAW = math.pi / 2  # fingers close along x, across the bar
OUT = 0.120  # how far the cartridge is drawn out (front of the bay to clear of the printer)
LIFT = 0.035
OLD_DOWN = (0.215, 0.395)  # where the left hand sets the used cartridge (body centre)
PARK_Z = 0.18
PUSH_DZ = 0.06  # the shut fingers push the door 6 cm above its hinge: low on the lip, then on the panel's end


def _w(t, x, y, z, finger, label, yaw=YAW):
    return Waypoint(t, (x, y, TOP + z), finger, label, yaw=yaw)


def demo():
    bx = SEATED[0] + BAR_X  # bar x when seated
    zs = HB + GRIP_DZ  # grip height, seated or on the open door
    ox = bx - OUT
    left = [
        _w(1.5, XF - 0.06, 0.30, PARK_Z, NEAR, "back from beside the printer"),
        _w(1.5, XF - 0.06, 0.0, zs + 0.025, NEAR, "in front of the open printer"),
        _w(1.0, bx - 0.02, 0.0, zs + 0.025, NEAR, "in over the bay"),
        _w(1.0, bx, 0.0, zs + 0.025, NEAR, "over the handle"),
        _w(1.5, bx, 0.0, zs, NEAR, "down round the bar"),
        _w(0.8, bx, 0.0, zs, PINCH, "take the handle"),
        _w(3.0, ox, 0.0, zs + 0.003, PINCH, "slide it out onto the door"),
        _w(1.2, ox, 0.0, zs + LIFT, PINCH, "lift off the door"),
        _w(3.5, OLD_DOWN[0] + BAR_X, OLD_DOWN[1], zs + LIFT, PINCH, "carry it to the left"),
        _w(2.0, OLD_DOWN[0] + BAR_X, OLD_DOWN[1], GRIP_DZ + 0.004, PINCH, "set it down on the table"),
        _w(0.8, OLD_DOWN[0] + BAR_X, OLD_DOWN[1], GRIP_DZ + 0.004, NEAR, "let go"),
        _w(1.0, OLD_DOWN[0] + BAR_X, OLD_DOWN[1], GRIP_DZ + 0.06, NEAR, "up off the handle"),
        _w(1.5, 0.12, 0.33, PARK_Z, NEAR, "park left"),
    ]
    # the left wrist reaches the middle of the table turned the other way round (the fingers still close along x)
    left = [Waypoint(w.duration, w.pos, w.finger, w.label, yaw=-YAW) for w in left]
    nx = NEW_START[0] + BAR_X
    ny = NEW_START[1]
    right = [
        _w(2.5, nx, ny, GRIP_DZ + 0.05, NEAR, "over the new cartridge's handle"),
        _w(1.5, nx, ny, GRIP_DZ, NEAR, "down round the bar"),
        _w(0.8, nx, ny, GRIP_DZ, PINCH, "take the handle"),
        _w(1.5, nx, ny, zs + LIFT, PINCH, "lift it"),
        _w(1.5, ox, ny, zs + LIFT, PINCH, "in front of the printer's right side"),
        _w(5.0, ox, ny, zs + LIFT, PINCH, "wait for the left side to clear"),
        _w(3.0, ox, 0.0, zs + LIFT, PINCH, "in front of the bay"),
        _w(1.5, ox, 0.0, zs + 0.002, PINCH, "set it on the open door"),
        _w(3.0, bx + 0.008, 0.0, zs + 0.002, PINCH, "slide it in to the stop"),
        _w(0.5, bx + 0.008, 0.0, zs + 0.002, PINCH, "hold it home"),
        _w(0.8, bx + 0.008, 0.0, zs + 0.002, NEAR, "let go"),
        _w(1.0, bx, 0.0, zs + 0.025, NEAR, "up off the handle"),
        _w(1.0, bx - 0.08, 0.0, PARK_Z - 0.02, NEAR, "out of the bay"),
        _w(1.5, XF - HP - 0.035, -0.08, PARK_Z, CLOSED, "in front of the door", 0.0),
        *door_push(),
    ]
    return {"left": left, "right": right}


LEAN = 45.0  # degrees: the shut fingertips lean forward (toward +x) to press on the door's lip
PUSH_H = 0.06  # m along the lip from the panel's end: where the fingertips press
PUSH_IN = 0.025  # m the press is aimed past the lip's face (the arm gives way; this is the press force)


def _lip_point(a, h=PUSH_H, out=0.0):
    """World point on the lip's outer face, h along the lip, out along its outward normal, with the door turned a
    (rad) up from open: the door's published geometry, no state read."""
    c, s = math.cos(a), math.sin(a)
    x = -HP * c + h * s - out * c
    z = HP * s + h * c + out * s
    return XF + x, HB + z


def door_push():
    """The right hand, fingers shut and leaning forward, follows the lip up its arc from open to shut, pressing on
    it, then lifts off and parks."""
    ax, az = math.sin(math.radians(LEAN)), -math.cos(math.radians(LEAN))  # TCP -> fingertip direction
    y = -0.08
    out = []

    def tw(t, a, off, label):
        tx, tz = _lip_point(a, out=off)
        return tilt.Waypoint(t, (tx - ax * TIP, y, TOP + tz - az * TIP), CLOSED, label, yaw=0.0, tilt=LEAN,
                             heading=0.0)

    out.append(tw(1.5, 0.0, 0.015, "lean in, fingertips in front of the lip"))
    out.append(tw(0.8, 0.0, -PUSH_IN, "press on the lip"))
    for deg in range(10, 101, 10):
        out.append(tw(0.7, math.radians(min(deg, 92)), -PUSH_IN, f"press the door up ({deg} deg)"))
    out.append(tw(0.8, SHUT, 0.03, "lift off the shut door"))
    out.append(_w(1.5, 0.12, -0.30, PARK_Z, CLOSED, "park right", 0.0))
    return out
