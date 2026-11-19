"""Build cover.npz (data only; no simulator) from Menlo's ASV1_100_04C.STL and the merged upper body:
- the back cover 100_04C at its place on the torso (pelvis frame, metres), decimated for display;
- thin box colliders along the cover's outer profile (the handle slot left open), and the 4 check points;
- the torso shell opened where the cover goes: its display mesh with the cover's area cut out, and colliders in pieces (front, above,
  below) plus the bay, a block whose rear face lies just inside the cover.
Run from the task folder: python dev/make_cover.py <path to ASV1_100_04C.stl>
"""
import sys, pathlib
import numpy as np, trimesh
from scipy.spatial import ConvexHull, cKDTree

HERE = pathlib.Path(__file__).resolve().parents[1]
UB = np.load(HERE / "upper_body.npz")
tv, tf = np.asarray(UB["v_waist_yaw_link"], float), np.asarray(UB["f_waist_yaw_link"], int)
tor = trimesh.Trimesh(tv, tf, process=False)

cov = trimesh.load(sys.argv[1])
P = cov.vertices / 1000.0 * np.array([-1.0, -1.0, 1.0])  # the maker's frame (x back) to the pelvis frame (x forward)
samp, T = tor.sample(30000, seed=0) if "seed" in trimesh.sample.sample_surface.__code__.co_varnames else tor.sample(30000), None
tree = cKDTree(samp)
T = np.array([tv[:, 0].min() - P[:, 0].min(), 0.0, 0.40 - P[:, 2].max()])
for _ in range(60):  # translation-only fit of the cover's surface to the torso's back
    d, i = tree.query(P + T)
    m = d < np.percentile(d, 80)
    T += (samp[i[m]] - (P + T)[m]).mean(0)
C = P + T
gap = np.median(tree.query(C)[0])
print(f"cover fit: offset {np.round(T, 4)}, median gap {1000 * gap:.1f} mm")

# outer profile near the middle (|y| < 3 cm), 1 cm bins
zs = np.arange(0.090, 0.4061, 0.01)
prof = []
for z in zs:
    s = C[(np.abs(C[:, 2] - z) < 0.005) & (np.abs(C[:, 1]) < 0.03) & (C[:, 0] < -0.105)]
    w = C[(np.abs(C[:, 2] - z) < 0.005) & (C[:, 0] < -0.105)]
    prof.append((z, s[:, 0].min() if len(s) else np.nan, w[:, 1].min(), w[:, 1].max()))
prof = np.array(prof)
ok = ~np.isnan(prof[:, 1])
prof[:, 1] = np.interp(prof[:, 2 - 2], prof[ok, 0], prof[ok, 1])  # fill the slot's bins from their neighbours
TH = 0.003
SLOT = (0.356, 0.376, -0.059, 0.066)  # the handle slot: z from, z to, y from, y to (from the STL)
slabs = []  # (centre, size (along the chord, y, thickness), pitch about y)
for k in range(len(prof) - 1):
    z1, x1, ya, yb = prof[k]
    z2, x2, yc, yd = prof[k + 1]
    t = np.array([x2 - x1, 0.0, z2 - z1]); L = np.linalg.norm(t); t /= L
    n_in = np.array([t[2], 0.0, -t[0]])  # inward normal (+x side of the chord)
    if n_in[0] < 0: n_in = -n_in
    y0, y1 = max(ya, yc), min(yb, yd)
    mid = np.array([(x1 + x2) / 2, 0.0, (z1 + z2) / 2]) + n_in * TH / 2
    pitch = float(np.arctan2(t[0], t[2]))  # the chord's lean from vertical (x per z)
    zlo, zhi = min(z1, z2), max(z1, z2)
    if zhi > SLOT[0] + 0.002 and zlo < SLOT[1] - 0.002:  # leave the slot open: two side pieces
        for a, b in ((y0, SLOT[2]), (SLOT[3], y1)):
            slabs.append((mid + [0, (a + b) / 2, 0], (L, b - a, TH), pitch))
    else:
        slabs.append((mid + [0, (y0 + y1) / 2, 0], (L, y1 - y0, TH), pitch))


def box_pts(c, size, pitch):
    L, W, H = size
    out = []
    for sl in (-1, 1):
        for sw in (-1, 1):
            for sh in (-1, 1):
                p = np.array([sh * H / 2, sw * W / 2, sl * L / 2])  # local: thickness along x, chord along z
                cp, sp = np.cos(pitch), np.sin(pitch)
                out.append(c + np.array([cp * p[0] + sp * p[2], p[1], -sp * p[0] + cp * p[2]]))
    return np.array(out)


# the bay: the cover's points below the slot moved 4 mm in, and a front face at x = -0.07
GAP_IN = 0.004
BAY_TOP = 0.35  # the bay stops below the handle slot, so fingers behind the bar stay clear of it
bay_pts = np.vstack([C[(C[:, 0] < -0.105) & (C[:, 2] <= BAY_TOP)] + [GAP_IN, 0, 0],
                     np.c_[np.full(4, -0.07), [-0.10, 0.10, -0.10, 0.10], [0.10, 0.10, BAY_TOP, BAY_TOP]]])
# beside the handle bar, above the bay: two more blocks the cover's top rests on, leaving a 60 mm notch behind the bar for the fingertips
NOTCH = 0.030
upper = {}
for side, sgn in (("l", 1.0), ("r", -1.0)):
    sel = (C[:, 0] < -0.105) & (C[:, 2] > BAY_TOP - 0.01) & (sgn * (C[:, 1] - 0.003) > NOTCH)
    pts = C[sel] + [GAP_IN, 0, 0]
    yo = 0.003 + sgn * NOTCH
    yf = 0.003 + sgn * 0.095
    front = np.array([[-0.07, yo, BAY_TOP - 0.01], [-0.07, yf, BAY_TOP - 0.01], [-0.07, yo, 0.40], [-0.07, yf, 0.40]])
    upper[side] = np.vstack([pts, front])
bay = ConvexHull(bay_pts)
# the cover's profile is not convex (a kink at mid-height, where its inner flange starts), so one hull of it would stand proud of the cover
# there: the bay is instead a stack of 2 cm bands, each the hull of its own stretch of the cover moved 4 mm in, with the front face at x = -0.07
bands = []
zb = np.arange(0.09, BAY_TOP + 1e-9, 0.02)
for z0, z1 in zip(zb[:-1], zb[1:]):
    sel = (C[:, 0] < -0.105) & (C[:, 2] >= z0 - 0.001) & (C[:, 2] <= z1 + 0.001)
    pts = C[sel] + [GAP_IN, 0, 0]
    ylo, yhi = pts[:, 1].min(), pts[:, 1].max()
    front = np.array([[-0.07, ylo, z0], [-0.07, yhi, z0], [-0.07, ylo, z1], [-0.07, yhi, z1]])
    bands.append(np.vstack([pts, front]))
eq = bay.equations
allc = np.vstack([box_pts(*s) for s in slabs])
eqs = [ConvexHull(p).equations for p in bands] + [ConvexHull(p).equations for p in upper.values()]
dense = []
for (cc, (L, W, H), p) in slabs:
    for a_ in np.linspace(-1, 1, 7):
        for b_ in np.linspace(-1, 1, 15):
            for d_ in (-1, 0, 1):
                cp_, sp_ = np.cos(p), np.sin(p)
                q_ = np.array([d_ * H / 2, b_ * W / 2, a_ * L / 2])
                dense.append(cc + np.array([cp_ * q_[0] + sp_ * q_[2], q_[1], -sp_ * q_[0] + cp_ * q_[2]]))
allc = np.array(dense)  # points over the slabs' faces, not only their corners: a hull can meet a box face between its corners
def clear(q):
    return all(((q @ e[:, :3].T + e[:, 3]).max(axis=1) >= -1e-5).all() for e in eqs)
shift = 0.0
if not clear(allc):
    # move the cover straight out (-x) until no slab corner is inside the bay or the blocks beside the bar
    for dx in np.arange(0.0, 0.02, 0.0002):
        if clear(allc - [dx, 0, 0]):
            shift = dx; break
gapin = next((dx for dx in np.arange(0.0, 0.01, 0.0002) if not clear(allc - [shift, 0, 0] + [dx + 0.0002, 0, 0])), None)
print(f"pressed in, the cover meets the bay or the blocks after {1000 * gapin:.1f} mm" if gapin is not None else "no contact within 10 mm")
print(f"slabs {len(slabs)}; the cover rests {1000 * shift:.1f} mm out from its fitted place on the bay")
C_seat = C - [shift, 0, 0]
slabs = [(c - [shift, 0, 0], s, p) for c, s, p in slabs]

# torso: cut the display mesh where the cover lies, and collider pieces
cen = tv[tf].mean(axis=1)
cut = (cen[:, 0] < -0.10) & (cen[:, 2] > 0.095) & (cen[:, 2] < 0.40) & (np.abs(cen[:, 1] - 0.004) < 0.108)
tf_open = tf[~cut]
pieces = {"front": tv[tv[:, 0] >= -0.07], "above": tv[(tv[:, 0] < -0.06) & (tv[:, 2] >= 0.41)], "below": tv[(tv[:, 0] < -0.06) & (tv[:, 2] <= 0.09)]}
out = dict(cover_v=C_seat, cover_f=np.asarray(cov.faces, int), cover_offset=T, cover_shift=shift,
           slab_c=np.array([s[0] for s in slabs]), slab_size=np.array([s[1] for s in slabs]), slab_pitch=np.array([s[2] for s in slabs]),
           torso_f_open=tf_open, bay_v=bay_pts[bay.vertices], bay_eq=eq)
for side, pts in upper.items():
    h = trimesh.convex.convex_hull(pts)
    out[f"bay_{side}_v"], out[f"bay_{side}_f"], out[f"bay_{side}_eq"] = np.asarray(h.vertices), np.asarray(h.faces), ConvexHull(pts).equations
out["n_bands"] = len(bands)
for i, pts in enumerate(bands):
    h = trimesh.convex.convex_hull(pts)
    out[f"band{i}_v"], out[f"band{i}_f"], out[f"band{i}_eq"] = np.asarray(h.vertices), np.asarray(h.faces), ConvexHull(pts).equations
# display mesh of the bay
bh = trimesh.convex.convex_hull(bay_pts)
out["bay_mv"], out["bay_mf"] = np.asarray(bh.vertices), np.asarray(bh.faces)
for k, pts in pieces.items():
    h = trimesh.convex.convex_hull(pts)
    out[f"t_{k}_v"], out[f"t_{k}_f"] = np.asarray(h.vertices), np.asarray(h.faces)
    out[f"t_{k}_eq"] = ConvexHull(pts).equations
# 4 check points: the cover's outer corners (top two by the bar, bottom two)
top = C_seat[C_seat[:, 2] > 0.395]; bot = C_seat[C_seat[:, 2] < 0.10]
out["checks"] = np.array([top[top[:, 1].argmin()], top[top[:, 1].argmax()], bot[bot[:, 1].argmin()], bot[bot[:, 1].argmax()]])
# decimate the display mesh
dm = trimesh.Trimesh(C_seat, cov.faces, process=True)
try:
    dm = dm.simplify_quadric_decimation(face_count=6000)
except Exception as e:
    print("no decimation:", e)
out["cover_v"], out["cover_f"] = np.asarray(dm.vertices), np.asarray(dm.faces)
c0 = np.asarray(dm.vertices).mean(0)
out["cover_c0"] = c0
print("cover display", out["cover_v"].shape, out["cover_f"].shape, "centre", np.round(c0, 4), "checks", np.round(out["checks"], 3))
print("torso faces cut", int(cut.sum()), "of", len(tf))
np.savez_compressed(HERE / "cover.npz", **out)
print("wrote", HERE / "cover.npz", (HERE / "cover.npz").stat().st_size // 1024, "kB")
