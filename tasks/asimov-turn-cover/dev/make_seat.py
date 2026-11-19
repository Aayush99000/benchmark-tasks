"""Build seat.npz (data only; no simulator): the real shell's ledges that the back cover touches, and the cover's colliders, from Menlo's CAD.

Inputs (CERN-OHL-S-2.0, github.com/menloresearch/asimov-1, mechanical/ASV1/100/FABRICATION/MJF_PA12/):
  ASV1_100_01C.STEP (torso shell) and ASV1_100_04C.STEP (back cover), each tessellated to STL with dev/step_to_stl.py (0.3 mm).
Run from the task folder:  python dev/make_seat.py <shell.stl> <cover.stl> [out_dir]

What it does
 1. Puts the shell STL in the pelvis frame of upper_body.npz (the torso link is the same part: translation-only fit, median gap about 5 mm,
    because the URDF mesh is a simplified copy) and the cover where cover.npz has it (its place on the back).
 2. Fits the cover's inner face to the shell's rear-facing faces (translation only), and keeps every rear-facing shell face within 2 mm of the
    cover's inner face: the ledges. Each is moved onto the cover's inner face, so the cover just touches it at its place.
 3. The ledges in 15 mm cells (y, z) become convex pads 3 mm thick (the sim's colliders for the shell's cover interface), and the same ledges
    merged by contact into blocks 8 mm thick (the printed blocks of the stand-in).
 4. The cover's colliders: convex hulls of its own mesh in 22 mm cells, each split by the way its face turns (its real shape, thick shell,
    no hull across the inside of a corner; the handle slot and the handle bar left to the handle box).
Outputs: seat.npz (pads, cover cells, summary), blocks.stl (stand-in blocks, in the cover's frame), ledge_report.txt.
"""
import sys, pathlib
import numpy as np, trimesh
from scipy.spatial import cKDTree, ConvexHull
from scipy import ndimage

HERE = pathlib.Path(__file__).resolve().parents[1]
shell = trimesh.load(sys.argv[1])
cov = trimesh.load(sys.argv[2])
OUT = pathlib.Path(sys.argv[3]) if len(sys.argv) > 3 else HERE
UB = np.load(HERE / "upper_body.npz")
CV = np.load(HERE / "cover.npz")
rng = np.random.default_rng(0)

# 1. frames. STEP frame: mm, the shell's own. Pelvis frame: m. pelvis_mm = step + T_SHELL.
tor = trimesh.Trimesh(UB["v_waist_yaw_link"], UB["f_waist_yaw_link"], process=False)
ts = trimesh.sample.sample_surface(tor, 40000, seed=0)[0] * 1000.0
ms = np.vstack([shell.vertices, shell.triangles_center])
tree = cKDTree(ms)
T_SHELL = np.array([-4.0, 0.0, 452.0])
for _ in range(60):
    d, i = tree.query(ts - T_SHELL)
    k = d < np.percentile(d, 60)
    T_SHELL -= (ms[i[k]] - (ts - T_SHELL)[k]).mean(0)
print("shell to pelvis frame (mm):", T_SHELL.round(2), "median gap", np.median(tree.query(ts - T_SHELL)[0]).round(1), "mm")

# the cover at its place (pelvis frame, mm), the same arithmetic as make_cover.py (its cover_v is a decimated copy of this)
P = cov.vertices * np.array([-1.0, -1.0, 1.0]) + (np.asarray(CV["cover_offset"], float) - [float(CV["cover_shift"]), 0, 0]) * 1000.0
cover_pel = trimesh.Trimesh(P, cov.faces, process=False)
cover = trimesh.Trimesh(P - T_SHELL, cov.faces, process=False)  # in the shell's frame
cs, fi = trimesh.sample.sample_surface(cover, 150000, seed=3)
cn = cover.face_normals[fi]
inner = cn[:, 0] > 0.2  # faces turned into the torso: the cover's inner face
cin, cnin = cs[inner], cn[inner]
outer = cn[:, 0] < -0.05  # faces turned to the back: the cover's outer face
cout_pel, nout = (cs[outer] + T_SHELL) / 1000.0, cn[outer]  # outer face samples in the cover's place frame (m) and their normals
# keep only the outer envelope: in each 3 mm (y, z) bin the sample nearest the back (the inner flange and ribs face the back too, but lie behind the wall)
_b = np.floor(cout_pel[:, 1:3] / 0.003).astype(np.int64)
_k = _b[:, 0] * 100000 + _b[:, 1]
_o = np.lexsort((cout_pel[:, 0], _k))
_first = np.r_[True, _k[_o][1:] != _k[_o][:-1]]
_keep = _o[_first]

cout_pel, nout = cout_pel[_keep], nout[_keep]
WALL = 0.0025  # m: the collider wall behind the outer face

TOL = 2.0  # mm: a rear-facing shell face this close to the cover's inner face (after the fit) is a ledge it touches
# 2. fit and ledges
fc, fn, area = shell.triangles_center, shell.face_normals, shell.area_faces
rear = fn[:, 0] < -0.3  # shell faces turned to the back, toward the cover
# translation-only fit of the cover to the ledge faces (cover moves by +T_FIT)
T_FIT = np.zeros(3)
for _ in range(80):
    d, i = cKDTree(fc[rear]).query(cin + T_FIT)
    k = d < np.percentile(d, 40)
    T_FIT += (fc[rear][i[k]] - (cin + T_FIT)[k]).mean(0)
print("cover fitted to the ledge faces by (mm):", T_FIT.round(2))
tin = cKDTree(cin + T_FIT)
d, i = tin.query(fc)
sel = rear & (d < TOL)
led = fc[sel]
print("ledge faces:", int(sel.sum()), "area", int(area[sel].sum()), "mm2; x range", led[:, 0].min().round(1), led[:, 0].max().round(1))
# each ledge face moved onto the cover's collider wall (its outer face moved in by WALL), so it touches at the place; pelvis frame, m
oj = cKDTree(cout_pel).query((fc[sel] - T_FIT + T_SHELL) / 1000.0)[1]
ledge_pts = cout_pel[oj] - (WALL + 0.0002) * nout[oj]
ledge_n = -nout[oj]  # into the torso
q, qn = (cin + T_FIT)[i[sel]], cnin[i[sel]]
fit_s = ((led - q) * qn).sum(1)
print("ledge face offset from the cover's inner face, mm (1,5,50,95,99 pct):", np.percentile(fit_s, [1, 5, 50, 95, 99]).round(2))
dwall = cKDTree(cin / 1000.0 + T_SHELL / 1000.0).query(cout_pel[::20])[0] * 1000.0
print("cover wall thickness (outer face to nearest inner-face sample), mm (10,50,90 pct):", np.percentile(dwall, [10, 50, 90]).round(1))

# 3. pads and blocks
CELL, PAD = 15.0 / 1000.0, 0.003
ij = np.floor(ledge_pts[:, 1:3] / CELL).astype(int)
pads, keys = [], []
for key in np.unique(ij, axis=0):
    s = (ij == key).all(1)
    if s.sum() < 8:
        continue
    pk, nk = ledge_pts[s], ledge_n[s]
    try:
        ConvexHull(np.vstack([pk, pk + PAD * nk]))
    except Exception:
        continue
    pads.append(np.vstack([pk, pk + PAD * nk])); keys.append(key)
keys = np.array(keys)
print("pads:", len(pads))
out = dict(n_pads=len(pads))
for k, pts in enumerate(pads):
    h = trimesh.convex.convex_hull(pts)
    out[f"pad{k}_v"], out[f"pad{k}_f"], out[f"pad{k}_eq"] = np.asarray(h.vertices), np.asarray(h.faces), ConvexHull(pts).equations

# blocks: pads connected by touching cells, 8 mm thick, for printing
g = np.zeros(keys.max(0) - keys.min(0) + 1, bool)
g[keys[:, 0] - keys.min(0)[0], keys[:, 1] - keys.min(0)[1]] = True
lab, nb = ndimage.label(g, structure=np.ones((3, 3)))
BLOCK = 0.008
report = ["Stand-in ledge blocks (cover frame = pelvis axes, origin at the cover's display centre; mm). Each block is the hull of the shell's rear-facing ledge faces",
          "within 2 mm of the cover's inner face (cells of 15 mm), 8 mm thick on the torso side. Print in PA12 or PETG, glue the back face to the plywood."]
blocks = []
C0 = np.asarray(CV["cover_c0"], float)
for b in range(1, nb + 1):
    cells = np.array([[a, c] for a, c in zip(*np.nonzero(lab == b))]) + keys.min(0)
    s = np.array([(keys == c).all(1).any() for c in cells])
    idx = [j for j, kk in enumerate(keys) if lab[kk[0] - keys.min(0)[0], kk[1] - keys.min(0)[1]] == b]
    pts = np.vstack([pads[j][: len(pads[j]) // 2] for j in idx])
    ns = np.vstack([ledge_n[(ij == keys[j]).all(1)] for j in idx])
    # a block is one piece only if convex enough: hull per cell, merged as separate solids in one STL
    meshes = []
    for j in idx:
        pk = pads[j][: len(pads[j]) // 2]; nk = ledge_n[(ij == keys[j]).all(1)]
        meshes.append(trimesh.convex.convex_hull(np.vstack([pk, pk + BLOCK * nk])))
    m = trimesh.util.concatenate(meshes)
    blocks.append(m)
    lo, hi = (m.bounds - C0 * 0) * 1000.0
    report.append(f"block {b}: {len(idx)} cells, y {lo[1]:.0f} to {hi[1]:.0f}, z {lo[2]:.0f} to {hi[2]:.0f}, x {lo[0]:.0f} to {hi[0]:.0f} (pelvis frame, mm)")
trimesh.util.concatenate(blocks).export(OUT / "blocks.stl")
out["n_blocks"] = nb

# 4. the cover's colliders: its outer face, WALL thick, in 22 mm cells (hulls), with the handle slot and the handle bar left out
SLOT = (0.356, 0.376, -0.059, 0.066)
CC = 0.022
cij = np.floor(cout_pel[:, 1:3] / CC).astype(int)
# each cell split by which way its outer face turns (bins of 30 degrees about the vertical, from straight back to the sides), so a cell where
# the back turns into a side wing becomes a few flat pieces, not one hull filling the inside of the corner (where the shell's side rails sit)
NB = np.round(np.degrees(np.arctan2(nout[:, 1], -nout[:, 0])) / 30.0).astype(int)
cijn = np.c_[cij, NB]
cells = []
for keyn in np.unique(cijn, axis=0):
    key = keyn[:2]
    s_ = (cijn == keyn).all(1)
    pk, nk = cout_pel[s_], nout[s_]
    if len(pk) < 3:
        continue
    cy, cz = (key[0] + 0.5) * CC, (key[1] + 0.5) * CC
    if SLOT[0] - CC / 2 < cz < SLOT[1] + CC / 2 and SLOT[2] - CC / 2 < cy < SLOT[3] + CC / 2:
        continue
    if cz > SLOT[1] - CC / 2 and abs(cy - 0.003) < 0.075:  # the handle bar: the handle box stands for it
        continue
    try:
        h = trimesh.convex.convex_hull(np.vstack([pk, pk - WALL * nk]))
    except Exception:
        continue
    cells.append((np.asarray(h.vertices), np.asarray(h.faces)))
# the same with the handle bar's cells kept (variant "bar")
cellsb = []
for key in np.unique(cij, axis=0):
    s_ = (cij == key).all(1)
    pk, nk = cout_pel[s_], nout[s_]
    if len(pk) < 6:
        continue
    cy, cz = (key[0] + 0.5) * CC, (key[1] + 0.5) * CC
    if SLOT[0] - CC / 2 < cz < SLOT[1] + CC / 2 and SLOT[2] - CC / 2 < cy < SLOT[3] + CC / 2:
        continue
    try:
        h = trimesh.convex.convex_hull(np.vstack([pk, pk - WALL * nk]))
    except Exception:
        continue
    cellsb.append((np.asarray(h.vertices), np.asarray(h.faces)))
out["n_cellsb"] = len(cellsb)
for k, (v, f) in enumerate(cellsb):
    out[f"cb{k}_v"], out[f"cb{k}_f"] = v, f
# the foam rest the cover starts on: the same cells, from 3 mm behind the collider wall's inner face (hull chords sag by about 0.4 mm) to 25 mm in (so its surface follows the cover's, wherever the wall is steep)
rest = []
for key in np.unique(cij, axis=0):
    s_ = (cij == key).all(1)
    pk, nk = cout_pel[s_], nout[s_]
    if len(pk) < 6 or (key[1] + 1) * CC > 0.350:  # none above 35 cm: the lower finger goes into the handle slot, as the old rest left it clear
        continue
    try:
        h = trimesh.convex.convex_hull(np.vstack([pk - (WALL + 0.003) * nk, pk - 0.025 * nk]))
    except Exception:
        continue
    rest.append((np.asarray(h.vertices), np.asarray(h.faces)))
out["n_rest"] = len(rest)
for k, (v, f) in enumerate(rest):
    out[f"rs{k}_v"], out[f"rs{k}_f"] = v, f
out["n_cells"] = len(cells)
for k, (v, f) in enumerate(cells):
    out[f"cc{k}_v"], out[f"cc{k}_f"] = v, f
print("cover cells:", len(cells))
np.savez_compressed(OUT / "seat.npz", **out)
(OUT / "ledge_report.txt").write_text("\n".join(report) + "\n")
print("\n".join(report))
