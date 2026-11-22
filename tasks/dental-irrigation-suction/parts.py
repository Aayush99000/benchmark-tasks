"""The scene's parts: the phantom head (typodont in a printed shell), the bulb syringe, the saliva ejector and their holders.

Meshes are built in mouth millimetres (see mouth.py) and placed in the world. hardware/make_stl.py writes the printable
shell parts from the same numbers.
"""

from __future__ import annotations

import math
import os

import numpy as np

import mechkit as m
import mouth as M

LOOKS = {k: m.Look(color=m.rgb(*v), roughness=r) for k, (v, r) in {
    "skin": ((0.93, 0.78, 0.68), 0.55), "lip": ((0.80, 0.42, 0.42), 0.45), "gum": ((0.86, 0.47, 0.52), 0.4),
    "tongue": ((0.82, 0.38, 0.40), 0.5), "mucosa": ((0.74, 0.30, 0.34), 0.5), "throat": ((0.30, 0.08, 0.10), 0.7),
    "tooth": ((0.97, 0.95, 0.88), 0.25), "dot": ((0.10, 0.35, 0.95), 0.3), "water": ((0.55, 0.80, 0.98), 0.05),
    "bulb": ((0.25, 0.45, 0.80), 0.35), "nozzle": ((0.85, 0.88, 0.92), 0.3), "hole": ((0.05, 0.05, 0.06), 0.8),
    "ejector": ((0.80, 0.90, 0.98), 0.15), "ejtip": ((0.98, 0.98, 0.98), 0.4), "handle": ((0.92, 0.92, 0.90), 0.4),
    "stand": ((0.62, 0.64, 0.66), 0.6), "towel": ((0.55, 0.78, 0.88), 0.9), "cup": ((0.92, 0.94, 0.96), 0.2),
    "prop": ((0.72, 0.74, 0.70), 0.7),
}.items()}

HEAD_POS = tuple(float(v) for v in M.O)  # the head part's origin: the lower incisal point


def _rel(xyz) -> np.ndarray:
    return np.asarray(xyz, float) - M.O


def _mm_mesh(mesh: m.Mesh, jaw: str = "lower") -> m.Mesh:
    """A mesh authored in mouth millimetres (p, l, c), placed in the head part's frame."""
    v = mesh.v
    w = M.upper(v[:, 0], v[:, 1], v[:, 2]) if jaw == "upper" else M.world(v[:, 0], v[:, 1], v[:, 2])
    return m.Mesh(_rel(w), mesh.f)


def blob(semi, center, e: float = 0.55, rings: int = 10, segments: int = 20) -> m.Mesh:
    """A rounded box (a superellipsoid) with half-sizes `semi`, centred at `center`."""
    s = m.sphere(1.0, rings=rings, segments=segments)
    v = np.sign(s.v) * np.abs(s.v) ** e * np.asarray(semi, float) + np.asarray(center, float)
    return m.Mesh(v, s.f)


def ellipsoid(semi, center, rings: int = 14, segments: int = 28) -> m.Mesh:
    s = m.sphere(1.0, rings=rings, segments=segments)
    return m.Mesh(s.v * np.asarray(semi, float) + np.asarray(center, float), s.f)


def _crown(p, l, md, bl, h, molar: bool, jaw: str):
    """A crown from c=-h to its occlusal table at c=FOSSA_C, plus cusp tips up to c=0 (in its jaw's closed frame; the upper jaw is mirrored)."""
    sgn = 1.0 if jaw == "lower" else -1.0
    top = M.FOSSA_C
    body = blob((md / 2, bl / 2, (h + top) / 2), (p, l, sgn * (-(h + top) / 2 + top)), e=0.6)
    cusps = []
    if molar:
        for dp in (-md / 4, md / 4):
            for dl in (-bl / 4, bl / 4):
                cusps.append(ellipsoid((md / 5, bl / 5, 1.6), (p + dp, l + dl, sgn * (top + 0.2)), rings=6, segments=10))
    return body, cusps


def head() -> m.Part:
    parts, cols = [], []

    def add(name, mesh, look, collide=False, hidden=False):
        parts.append((name, mesh, look, hidden) if hidden else (name, mesh, look))
        if collide:
            cols.append(m.col_hull(name + "_c", mesh))

    # teeth and gums, both jaws
    for jaw, table in (("lower", M.LOWER), ("upper", M.UPPER)):
        sgn = 1.0 if jaw == "lower" else -1.0
        for k, (p, l, md, bl, h) in table.items():
            for s, side in ((1, "L"), (-1, "R")):
                molar = k in "4567"
                body, cusps = _crown(p, s * l, md, bl, h, molar, jaw)
                add(f"{jaw}{side}{k}", _mm_mesh(body, jaw), "tooth", collide=True)
                for i, c in enumerate(cusps):
                    add(f"{jaw}{side}{k}cusp{i}", _mm_mesh(c, jaw), "tooth")
                gum = blob((md / 2 + 0.6, bl / 2 + 0.5, 6.0), (p, s * l, sgn * (-h - 4.5)), e=0.7)
                add(f"{jaw}{side}{k}gum", _mm_mesh(gum, jaw), "gum", collide=True)
    # the tongue (soft: its collider stands TONGUE_GIVE inside its surface), the floor, cheeks and throat
    t = M.TONGUE
    add("tongue", _mm_mesh(ellipsoid(t["semi"], t["center"])), "tongue")
    cols.append(m.col_hull("tongue_c", _mm_mesh(ellipsoid([v - M.TONGUE_GIVE for v in t["semi"]], t["center"], rings=8, segments=16))))
    floor = blob((40.0, 38.0, 2.0), (30.0, 0.0, M.FLOOR_C - 2.0), e=0.3)
    add("floor", _mm_mesh(floor), "mucosa", collide=True)
    for s, side in ((1, "L"), (-1, "R")):
        cheek = blob((36.0, 2.0, 24.0), (30.0, s * 38.5, 8.0), e=0.3)
        add("cheek" + side, _mm_mesh(cheek), "mucosa", collide=True)
        vest = blob((18.0, 2.6, 0.6), (30.0, s * 33.5, M.FLOOR_C + 0.6), e=0.4)
        add("vestibule_water" + side, _mm_mesh(vest), "water", hidden=True)
    palate = blob((32.0, 22.0, 3.0), (24.0, 0.0, 14.0), e=0.5)
    add("palate", _mm_mesh(palate, "upper"), "mucosa", collide=True)
    throat = blob((4.0, 30.0, 20.0), (72.0, 0.0, 0.0), e=0.3)
    add("throat", _mm_mesh(throat), "throat", collide=True)
    # the throat sill across the back of the floor, and the level water lying against it on each side (task.py moves and
    # stretches it as it fills: its surface is level, so it lies along the world x axis)
    sill = blob((2.0, 34.0, M.SILL_H / 2), (M.SILL_P + 2.0, 0.0, M.FLOOR_C + M.SILL_H / 2), e=0.3)
    add("sill", _mm_mesh(sill), "mucosa", collide=True)
    for side in ("left", "right"):
        add("sill_water_" + side, m.box((M.SILL_REACH / 1000, (M.SILL_L[1] - M.SILL_L[0]) / 1000, 0.0006), (0, 0, 0)), "water", hidden=True)
    # a mouth prop between the right molars holds the jaws open (hardware/: jaw_spacer.stl)
    for name, mesh in mouth_prop_meshes():
        add(name, _mm_mesh(mesh), "prop")
    # the blue gel dots on the three target teeth
    for i, spot in enumerate(M.SPOTS):
        n = M.N_AX
        disc = m.cylinder(0.0021, 0.0006, (0, 0, 0.0001), segments=16)
        rot = _align(np.array([0, 0, 1.0]), n)
        add(f"dot{i}", m.Mesh(disc.v @ rot.T + _rel(spot) + n * 0.0004, disc.f), "dot")
    # lips: a ring round the opening, and the face around it
    ring = _lip_outline(scale_out=0.0045)
    add("lips", m.tube(np.vstack([ring, ring[:1]]), 0.0045, sides=10), "lip")
    for i in range(len(ring)):
        a, b = ring[i], ring[(i + 1) % len(ring)]
        cols.append(m.col_hull(f"lip_c{i}", m.tube([a, b], 0.0045, sides=8)))
    face, sectors = _face()
    add("face", face, "skin")
    for i, sec in enumerate(sectors):
        cols.append(m.col_hull(f"face_c{i}", sec))
    add("nose", _face_feature("nose"), "skin")
    add("chin", _face_feature("chin"), "skin")
    # the stand under the shell and the towel on the table
    # a 4 mm base plate under the head, and a 40 mm block under the back of the head (toward the robot), clear of the mouth
    for name, mesh in stand_meshes():
        add(name, mesh, "stand", collide=True)
    add("towel", m.box((0.30, 0.26, 0.002), (-0.03, 0.0, M.TOP - M.O[2] + 0.001)), "towel")
    # the throat cup beside the stand, on its drain tube
    cup_at = np.array([-0.07, 0.10, M.TOP - M.O[2]])
    add("throat_cup", m.lathe([(0.0, 0.0), (0.016, 0.0), (0.016, 0.045), (0.0145, 0.045), (0.0145, 0.002), (0.0, 0.002)], 24).moved(cup_at), "cup")
    add("drain", m.tube([_rel(M.world(70, 8, -10)), cup_at + [0.0, -0.03, 0.06], cup_at + [0.0, 0.0, 0.05]], 0.003), "nozzle")
    return m.fixed("head", HEAD_POS, parts, cols, LOOKS, friction=(0.6, 0.5))


STAND_PLATE = ((0.160, 0.130, 0.004), (-0.065, 0.0, 0.002))  # size, centre (x, y from the lower incisal point; z above the table), m
STAND_BLOCK = ((0.070, 0.130, 0.040), (-0.110, 0.0, 0.020))


def upper_closed(xyz) -> np.ndarray:
    """Inverse of mouth.upper for (p, c): a point of the opened upper jaw, back in its closed position (mouth mm)."""
    p, l, c = xyz
    a = math.radians(M.OPEN_DEG)
    dp, dc = p - M.HINGE[0], c - M.HINGE[1]
    return np.array([M.HINGE[0] + dp * math.cos(a) - dc * math.sin(a), l, M.HINGE[1] + dp * math.sin(a) + dc * math.cos(a)])


def upper_open_mm(p, l, c) -> np.ndarray:
    """mouth.upper, but in mouth millimetres."""
    return M.mouth(M.upper(p, l, c))


PROP_P = (26.0, 46.0)  # mm: the mouth prop spans the lower right first and second molars
PROP_L = (-31.0, -17.0)  # between the molars' lingual and buccal faces
PROP_FLANGE = 1.5  # mm thick flanges straddle each jaw's molars, 3.5 mm deep


def mouth_prop_meshes():
    """The mouth prop (bite block) between the right molars: a wedge from the lower molars' cusp tips (c = 0) to the opened
    upper molars' cusp tips, with flanges that straddle each jaw's molars on the cheek and tongue sides, so it sits in one
    place on both jaws. Meshes in mouth millimetres (convex hulls)."""
    out = []
    (p0, p1), (l0, l1) = PROP_P, PROP_L
    low = [(p, l, 0.0) for p in (p0, p1) for l in (l0, l1)]
    up = [tuple(upper_open_mm(p, l, 0.0)) for p in (p0 - 2, p1 - 2) for l in (l0, l1)]
    out.append(("prop", _hull_mesh(np.array(low + up, float))))
    f = PROP_FLANGE
    for k, (la, lb) in enumerate(((l0 - f, l0), (l1, l1 + f))):
        pts = [(p, l, c) for p in (p0, p1) for l in (la, lb) for c in (0.5, -3.5)]
        out.append((f"prop_lower{k}", _hull_mesh(np.array(pts, float))))
        pts = [tuple(upper_open_mm(p, l, c)) for p in (p0 - 2, p1 - 2) for l in (la - 1.0, lb - 1.0) for c in (-0.5, 3.5)]
        out.append((f"prop_upper{k}", _hull_mesh(np.array(pts, float))))
    return out


def stand_meshes():
    """The stand's two boxes, in the head part's frame."""
    return [(n, m.box(size, (c[0], c[1], M.TOP + c[2] - M.O[2]))) for n, (size, c) in (("stand", STAND_PLATE), ("stand_block", STAND_BLOCK))]


def _align(a, b) -> np.ndarray:
    """The rotation turning unit vector a onto unit vector b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def _lip_outline(scale_out: float = 0.0, n: int = 28) -> np.ndarray:
    """Points round the opening's outline (in the head part's frame), pushed out by `scale_out` metres."""
    out = []
    for th in np.linspace(0, 2 * np.pi, n, endpoint=False):
        c, s = math.cos(th), math.sin(th)
        u = np.sign(c) * abs(c) ** (2 / M.LIP_POWER) * (M.LIP_SEMI_V + scale_out)
        w = np.sign(s) * abs(s) ** (2 / M.LIP_POWER) * (M.LIP_SEMI_W + scale_out)
        out.append(_rel(M.LIP_CENTRE + u * M.LIP_V + w * M.L_AX))
    return np.array(out)


_FACE_C = None


def _face_frame():
    """The face: half an ellipsoid round the opening. Axes: n out of the face, v toward the crown, w the patient's left."""
    n, v, w = M.LIP_NORMAL, M.LIP_V, M.L_AX
    centre = M.LIP_CENTRE - 0.075 * n + 0.025 * v
    return centre, n, v, w, (0.082, 0.120, 0.078)  # semi-axes along n, v, w


def _face_point(a, b):
    """The face's surface at angles a (from n, outward) and b (round n)."""
    centre, n, v, w, (sn, sv, sw) = _face_frame()
    return centre + sn * math.cos(a) * n + sv * math.sin(a) * math.cos(b) * v + sw * math.sin(a) * math.sin(b) * w


def _inside_opening(x) -> bool:
    d = x - M.LIP_CENTRE
    u, w = d @ M.LIP_V, d @ M.L_AX
    return abs(u / (M.LIP_SEMI_V + 0.008)) ** M.LIP_POWER + abs(w / (M.LIP_SEMI_W + 0.008)) ** M.LIP_POWER <= 1.0


def _face():
    """The visible face shell with the opening cut out, and convex sectors of it (a few mm thick) for colliders."""
    A, B = 24, 40
    grid = np.array([[_face_point(a, b) for b in np.linspace(0, 2 * np.pi, B, endpoint=False)] for a in np.linspace(0, math.radians(80), A)])
    verts, faces = [], []
    idx = {}
    for i in range(A):
        for j in range(B):
            idx[i, j] = len(verts)
            verts.append(_rel(grid[i, j]))
    for i in range(A - 1):
        for j in range(B):
            q = [(i, j), (i + 1, j), (i + 1, (j + 1) % B), (i, (j + 1) % B)]
            if all(_inside_opening(grid[a]) for a in q):
                continue
            if any(_inside_opening(grid[a]) for a in q) and sum(_inside_opening(grid[a]) for a in q) >= 2:
                continue
            faces += [[idx[q[0]], idx[q[3]], idx[q[1]]], [idx[q[1]], idx[q[3]], idx[q[2]]]]
    faces = np.array(faces)
    verts = np.array(verts)
    face = m.Mesh(np.vstack([verts, verts]), np.vstack([faces, faces[:, ::-1] + len(verts)]))  # both sides: it is a shell, seen from any side
    # colliders: small patches of the shell outside the opening (about 7 x 18 degrees each, so that a patch's hull
    # stays within a few millimetres of the surface), each 4 mm thick, out to 50 degrees from the opening's axis
    centre, n, *_ = _face_frame()
    sectors = []
    for i0 in range(0, 15, 2):
        for j0 in range(0, B, 2):
            pts = []
            for i in range(i0, i0 + 3):
                for j in range(j0, j0 + 3):
                    x = grid[i, j % B]
                    if not _inside_opening(x):
                        pts += [_rel(x), _rel(x - 0.004 * (x - centre) / np.linalg.norm(x - centre))]
            if len(pts) >= 8:
                sectors.append(_hull_mesh(np.array(pts)))
    return face, sectors


def _hull_mesh(pts: np.ndarray) -> m.Mesh:
    from scipy.spatial import ConvexHull

    h = ConvexHull(pts)
    return m.Mesh(pts[h.vertices], np.array([[list(h.vertices).index(k) for k in s] for s in h.simplices]))


def _face_feature(kind: str) -> m.Mesh:
    centre, n, v, w, (sn, sv, sw) = _face_frame()
    if kind == "nose":
        base = _face_point(math.radians(34), 0.0)
        e = ellipsoid((0.011, 0.024, 0.016), (0, 0, 0), rings=10, segments=18)
        rot = np.stack([w, v, n], axis=1)
        return m.Mesh(e.v @ rot.T + _rel(base) + 0.004 * n, e.f)
    base = _face_point(math.radians(44), math.pi)
    e = ellipsoid((0.030, 0.018, 0.012), (0, 0, 0), rings=10, segments=18)
    rot = np.stack([w, v, n], axis=1)
    return m.Mesh(e.v @ rot.T + _rel(base), e.f)


# -- the tools -------------------------------------------------------------------------------------

BULB_K = 2500.0  # soft-contact stiffness of the PVC bulb; in the simulator this gives about 6.5 N/mm across the bulb
PAD_OFFSET = 0.0014  # m from the gripper joint's opening to the finger pad's face, measured on a rigid 52 mm body
BULB_SEMI = (M.BULB_D / 2, M.BULB_D / 2, M.BULB_H / 2)
NECK = 0.006
TIP_Z = -(M.BULB_H / 2 + NECK + M.TIP_L)  # the nozzle's end, in the bulb's frame (its axis is -z)
HOLDER_XY = (0.36, -0.20)  # on the right arm's side
RING_Z = M.TOP + 0.070
RING_R = 0.016
BULB_START = (HOLDER_XY[0], HOLDER_XY[1], RING_Z + BULB_SEMI[2] * math.sqrt(1 - (RING_R / BULB_SEMI[0]) ** 2) + 0.0005)


def bulb() -> m.Part:
    body = ellipsoid(BULB_SEMI, (0, 0, 0), rings=16, segments=32)
    neck = m.cylinder(0.0055, NECK, (0, 0, -BULB_SEMI[2] - NECK / 2 + 0.001), segments=20)
    nozzle = m.lathe([(0.0, TIP_Z), (0.0021, TIP_Z), (0.0045, -BULB_SEMI[2] - NECK), (0.0, -BULB_SEMI[2] - NECK)], 20)
    hole = m.cylinder(0.0011, 0.0004, (0, 0, TIP_Z - 0.0001), segments=10)
    parts = [("bulb", body, "bulb"), ("neck", neck, "nozzle"), ("nozzle", nozzle, "nozzle"), ("hole", hole, "hole")]
    cols = [m.col_hull("bulb_c", ellipsoid(BULB_SEMI, (0, 0, 0), rings=8, segments=16)), m.col_hull("nozzle_c", nozzle)]
    k = float(os.environ.get("DENTAL_BULB_K", BULB_K))  # debug: another stiffness
    # 25 g of PVC and 20 mL of water, which lies in the lower part of the upright bulb: the centre of mass 10 mm below its middle
    return m.free("bulb", BULB_START, parts, cols, LOOKS, mass=0.045, friction=(1.2, 1.0), compliant=None if k <= 0 else (k, k / 100.0), com=(0.0, 0.0, -0.010))


def bulb_holder() -> m.Part:
    x, y = HOLDER_XY
    parts, cols = [], []
    ring = [(RING_R + 0.003) * np.array([math.cos(a), math.sin(a), 0]) for a in np.linspace(0, 2 * np.pi, 13)]
    parts.append(("ring", m.tube(np.array(ring) + [0, 0, RING_Z - M.TOP], 0.003, sides=8), "stand"))
    for i in range(12):
        a, b = ring[i] + [0, 0, RING_Z - M.TOP], ring[i + 1] + [0, 0, RING_Z - M.TOP]
        cols.append(m.col_hull(f"ring_c{i}", m.tube([a, b], 0.003, sides=6)))
    post = m.box((0.010, 0.010, RING_Z - M.TOP), (0, -0.042, (RING_Z - M.TOP) / 2))  # on the outer side, clear of fingers closing along x
    arm = m.box((0.010, 0.026, 0.006), (0, -0.032, RING_Z - M.TOP))
    base = m.cylinder(0.050, 0.006, (0, 0, 0.003))
    # a wide, shallow drip cup: the nozzle hangs in it without touching, so the bulb settles upright in the ring
    cup = m.lathe([(0.0, 0.006), (0.032, 0.006), (0.032, 0.020), (0.030, 0.020), (0.030, 0.008), (0.0, 0.008)], 32)
    for n_, mesh in (("post", post), ("arm", arm), ("base", base)):
        parts.append((n_, mesh, "stand"))
        cols.append(m.col_hull(n_ + "_c", mesh))
    parts.append(("drip_cup", cup, "cup"))
    return m.fixed("bulb_holder", (x, y, M.TOP), parts, cols, LOOKS, friction=(0.4, 0.35))  # smooth PETG


# The saliva ejector: a 14 mm handle along y, the 6.5 mm tube from its -y end to the bend, and the working leg down to the tip.
# The left arm holds it; the handle stays outboard of the mouth, on the patient's left.
EJ_HANDLE = 0.080
EJ_R = 0.00325
EJ_REACH_Y = -0.105  # from the handle's centre to the bend, along -y
EJ_BEND = np.array([0.0, EJ_REACH_Y, 0.0])
EJ_TIP_LOCAL = EJ_BEND + M.EJ_AXIS * M.EJ_LEG
CRADLE_XY = (0.36, 0.24)  # on the left arm's side
CRADLE_Z = M.TOP + 0.105
EJ_START = (CRADLE_XY[0], CRADLE_XY[1], CRADLE_Z + 0.0075)


def ejector() -> m.Part:
    handle = m.cylinder(0.007, EJ_HANDLE, (0, 0, 0), axis="y", segments=24)
    path = [np.array([0, -(EJ_HANDLE / 2 - 0.004), 0]), np.array([0, EJ_REACH_Y + 0.012, 0]), EJ_BEND + [0, 0.0, -0.004] + M.EJ_AXIS * 0.006,
            EJ_BEND + M.EJ_AXIS * 0.02, EJ_TIP_LOCAL - M.EJ_AXIS * 0.004]
    tube = m.tube(path, EJ_R, sides=12)
    cap = ellipsoid((0.004, 0.004, 0.0045), EJ_TIP_LOCAL, rings=8, segments=14)
    parts = [("handle", handle, "handle"), ("tube", tube, "ejector"), ("cap", cap, "ejtip")]
    cols = [m.col_hull("handle_c", m.cylinder(0.007, EJ_HANDLE, (0, 0, 0), axis="y", segments=12))]
    for i in range(len(path) - 1):
        cols.append(m.col_hull(f"tube_c{i}", m.tube([path[i], path[i + 1]], EJ_R, sides=8)))
    cols.append(m.col_hull("cap_c", ellipsoid((0.004, 0.004, 0.0045), EJ_TIP_LOCAL, rings=5, segments=8)))
    return m.free("ejector", EJ_START, parts, cols, LOOKS, mass=0.025, friction=(1.0, 0.9))


def ejector_cradle() -> m.Part:
    x, y = CRADLE_XY
    h = CRADLE_Z - M.TOP
    parts, cols = [], []
    for k, dy in enumerate((-0.035, 0.035)):  # clear of fingers 36 mm wide that grip the handle between them
        for side in (-1, 1):
            jaw = m.box((0.006, 0.012, 0.014), (side * 0.010, dy, h + 0.004))
            parts.append((f"jaw{k}{side + 1}", jaw, "stand"))
            cols.append(m.col_hull(f"jaw_c{k}{side + 1}", jaw))
        post = m.box((0.026, 0.012, h), (0, dy, h / 2))
        parts.append((f"post{k}", post, "stand"))
        cols.append(m.col_hull(f"post_c{k}", post))
    base = m.box((0.05, 0.09, 0.006), (0, 0, 0.003))
    parts.append(("base", base, "stand"))
    cols.append(m.col_hull("base_c", base))
    return m.fixed("ejector_cradle", (x, y, M.TOP), parts, cols, LOOKS)


def water_marks() -> m.Part:
    """What water is seen doing, moved by task.py: drops along the stream, drops into the ejector."""
    parts = []
    for j in range(14):
        parts.append((f"jet{j}", m.sphere(0.0011, rings=4, segments=8), "water", True))
    for j in range(8):
        parts.append((f"draw{j}", m.sphere(0.0010, rings=4, segments=8), "water", True))
    return m.fixed("water_marks", (0.0, 0.0, 0.0), parts, [], LOOKS)
