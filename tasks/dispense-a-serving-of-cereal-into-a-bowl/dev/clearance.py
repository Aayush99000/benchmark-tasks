"""Offline: the smallest gap between the paddle wheel's swept volume and every static collider of the dispenser.
Created 11_16_26 (America/New_York). Run from the task folder with any Python that has NumPy."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cereal_dispenser import geometry as G  # noqa: E402


def inside_hull(points, verts):
    """Signed distance (negative inside) of points to the convex hull of verts, via its face planes."""
    from itertools import combinations

    c = verts.mean(0)
    planes = []
    for i, j, k in combinations(range(len(verts)), 3):
        n = np.cross(verts[j] - verts[i], verts[k] - verts[i])
        ln = np.linalg.norm(n)
        if ln < 1e-12:
            continue
        n /= ln
        d = (verts - verts[i]) @ n
        if (d <= 1e-9).all():
            planes.append((n, n @ verts[i]))
        elif (d >= -1e-9).all():
            planes.append((-n, -n @ verts[i]))
    return np.max([points @ n - d for n, d in planes], axis=0)


# points filling the wheel's swept cylinder (vane tips at R_TIP, along the vanes' length)
r = np.linspace(G.R_HUB, G.R_TIP, 12)
a = np.radians(np.arange(0, 360, 2.0))
x = np.linspace(-G.L_WHEEL / 2, G.L_WHEEL / 2, 15)
R, A, X = np.meshgrid(r, a, x, indexing="ij")
pts = np.c_[X.ravel(), (R * np.cos(A)).ravel(), (G.WHEEL_Z + R * np.sin(A)).ravel()]
pieces = {"chamber": G.chamber_pieces(), "funnel": G.funnel_pieces()[0]}
for name, ps in pieces.items():
    gap = min(inside_hull(pts, v).min() for v in ps)
    print(f"{name:8s} smallest gap to the swept wheel: {gap * 1000:6.2f} mm")
