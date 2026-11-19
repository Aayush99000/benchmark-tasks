"""3D-printable parts for the grinder mock's adjustable fittings (PETG or nylon, 100% infill for the cams).

    python make_parts_stl.py       # writes cad/face_cam_lower.stl, cad/cam_lever.stl

NumPy only. Millimetres. Sizes match BUILD.md and the drawings (make_drawings.py).
- face_cam_lower.stl: the fixed cam washer, set in the platform's counterbore (two pins key it against turning).
- cam_lever.stl: the T-handle, its stem and, at the stem's foot, the mating cam washer, in one print.
"""

import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
R_IN, R_OUT = 2.9, 9.0  # cam washer bore (M5 clearance) and outside radius
CAM_BASE, CAM_RISE = 3.0, 2.0  # base thickness and ramp rise
N = 360  # one facet per degree


def rise(deg):
    """Ramp height at an angle (two ramps per turn): rises 0-80 deg, flat 80-90 deg (holds the lock), then drops
    back at the stop wall and stays low to 180 deg."""
    a = deg % 180.0
    if a < 80.0:
        return CAM_RISE * a / 80.0
    if a < 90.0:
        return CAM_RISE
    return 0.0


def ring(top, z0=0.0, flip=False):
    """A closed annulus from z0 up to z0 + top(deg) (or hanging down from z0 when flip), as triangles."""
    tris = []
    ang = [2 * math.pi * i / N for i in range(N + 1)]
    deg = [360.0 * i / N for i in range(N + 1)]
    s = -1.0 if flip else 1.0
    for i in range(N):
        a0, a1 = ang[i], ang[i + 1]
        h0, h1 = top(deg[i]), top(deg[i + 1])
        pi0 = np.array([R_IN * math.cos(a0), R_IN * math.sin(a0), z0])
        pi1 = np.array([R_IN * math.cos(a1), R_IN * math.sin(a1), z0])
        po0 = np.array([R_OUT * math.cos(a0), R_OUT * math.sin(a0), z0])
        po1 = np.array([R_OUT * math.cos(a1), R_OUT * math.sin(a1), z0])
        up = lambda p, h: p + np.array([0.0, 0.0, s * h])  # noqa: E731
        ti0, ti1, to0, to1 = up(pi0, h0), up(pi1, h1), up(po0, h0), up(po1, h1)
        tris += [(pi0, po1, po0), (pi0, pi1, po1)]  # base
        tris += [(ti0, to0, to1), (ti0, to1, ti1)]  # cam face
        tris += [(po0, po1, to1), (po0, to1, to0)]  # outer wall
        tris += [(pi0, ti1, pi1), (pi0, ti0, ti1)]  # inner wall
    if flip:
        tris = [(a, c, b) for a, b, c in tris]
    return tris


def cuboid(x0, x1, y0, y1, z0, z1):
    p = [np.array([x, y, z], float) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    f = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4), (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6),
         (1, 3, 5), (3, 7, 5)]
    return [(p[a], p[b], p[c]) for a, b, c in f]


def write_stl(path, tris, name):
    with open(path, "w") as fh:
        fh.write(f"solid {name}\n")
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            nn = np.linalg.norm(n)
            n = n / nn if nn > 0 else n
            fh.write(f" facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n  outer loop\n")
            for v in (a, b, c):
                fh.write(f"   vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
            fh.write("  endloop\n endfacet\n")
        fh.write(f"endsolid {name}\n")


def main():
    out = HERE / "cad"
    out.mkdir(exist_ok=True)
    # lower washer: base plus two ramps, cam face up; two 2 mm key pins under it fit holes in the counterbore floor
    lower = ring(lambda d: CAM_BASE + rise(d))
    for x in (-6.0, 6.0):
        lower += cuboid(x - 1.0, x + 1.0, -1.0, 1.0, -2.0, 0.0)
    write_stl(out / "face_cam_lower.stl", lower, "face_cam_lower")
    # lever: mating washer hanging from the stem's foot (its ramps the complement of the lower's), stem, T-bar
    stem_h, stem_w, bar = 13.0, 14.0, (12.0, 45.0, 12.0)
    lever = ring(lambda d: CAM_BASE + (CAM_RISE - rise(d)), z0=CAM_BASE + CAM_RISE, flip=True)
    z = CAM_BASE + CAM_RISE
    lever += cuboid(-stem_w / 2, stem_w / 2, -stem_w / 2, stem_w / 2, z, z + stem_h)
    lever += cuboid(-bar[0] / 2, bar[0] / 2, -bar[1] / 2, bar[1] / 2, z + stem_h, z + stem_h + bar[2])
    write_stl(out / "cam_lever.stl", lever, "cam_lever")
    print("wrote", sorted(p.name for p in out.glob("*.stl")))


if __name__ == "__main__":
    main()
