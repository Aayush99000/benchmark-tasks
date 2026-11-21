"""Reduced position-based model of a foil-laminated (dead-fold) wrap sheet. It takes no phase or goal inputs.

The rigid simulator supplies, every control step, the burger pose, the measured TCP poses and jaw openings and the
finger boxes; this module moves the sheet and nothing else.
- Grid of nodes 1.25 cm apart. In-plane springs (edges and diagonals) are stiff in tension and give way under
  compression by crumpling (their rest length shortens plastically), as a sheet pleats around a round bun.
- Bending: skip-one springs along rows, columns and both diagonals. Fold memory: when a bending spring's strain
  passes a small yield band its rest length follows (plastic crease), so a fold stays after release and springs
  back only by that band; pulling a fold flat again unfolds it.
- Contact: analytic burger (cylinder with a rounded crown edge), board top and sides, counter plane and finger
  boxes, with friction on contact nodes. Nodes under the burger's base are held by its weight and move with it.
- Layers: no sheet-on-sheet collision. Each node gets a layer index (the fold in which it first left the board),
  and its burger contact is offset by that many sheet thicknesses, so later folds lie over earlier ones.
- Grasp rule: a jaw that closes (opening < 12 mm after being open) grips the sheet EDGES lying between its
  fingertips: one edge, or several edges stacked by earlier folds, within the gap the fingers closed from. A jaw
  that is already shut (pressing) picks nothing up. A gripped node lets go when the jaw opens past 23 mm or the
  node is pulled 3.5 cm away from the fingers.
- Tear: stretch beyond 1.8x for 0.25 s (or 3x at once) latches the sheet as torn.
Serial Gauss-Seidel in a Warp CPU kernel. Uncalibrated stand-in, not a thin-shell FEM.
"""
from __future__ import annotations

import numpy as np
import warp as wp

wp.config.quiet = True
DX = 0.0125
_DEV = "cpu"


@wp.func
def burger_sdf(p: wp.vec3, r: float, h: float, rr: float):
    """SDF (x) and outward normal (yzw) of an upright cylinder, base at z=0 (extended far below), top edge
    rounded by rr. Returned as vec4(d, nx, ny, nz)."""
    rho = wp.sqrt(p[0] * p[0] + p[1] * p[1])
    ux = 1.0
    uy = 0.0
    if rho > 1.0e-9:
        ux = p[0] / rho
        uy = p[1] / rho
    zc = h - rr
    qr = rho - (r - rr)
    qz = p[2] - zc
    if qz > 0.0:
        if qr > 0.0:
            ln = wp.sqrt(qr * qr + qz * qz)
            return wp.vec4(ln - rr, ux * qr / ln, uy * qr / ln, qz / ln)
        return wp.vec4(qz - rr, 0.0, 0.0, 1.0)
    ds = rho - r
    db = -0.05 - p[2]
    if ds >= db:
        return wp.vec4(ds, ux, uy, 0.0)
    return wp.vec4(db, 0.0, 0.0, -1.0)


@wp.kernel
def substeps_kernel(x: wp.array(dtype=wp.vec3), v: wp.array(dtype=wp.vec3), w: wp.array(dtype=float),
                    target: wp.array(dtype=wp.vec3), ca: wp.array(dtype=int), cb: wp.array(dtype=int),
                    crest: wp.array(dtype=float), cflat: wp.array(dtype=float), ck: wp.array(dtype=float), cpl: wp.array(dtype=int),
                    layer_off: wp.array(dtype=float), contact: wp.array(dtype=int), old: wp.array(dtype=wp.vec3),
                    cnrm: wp.array(dtype=wp.vec3),
                    bpos: wp.vec3, brot: wp.mat33, br: float, bh: float, brr: float,
                    board_c: wp.vec3, board_half: wp.vec3, table_z: float,
                    box_T: wp.array(dtype=wp.mat44), box_lo: wp.array(dtype=wp.vec3), box_hi: wp.array(dtype=wp.vec3),
                    nbox: int, h: float, nsub: int, iters: int, mu: float, ey: float, damp: float, flat2: float,
                    kcomp: float, eyc: float, cmin: float):
    n = x.shape[0]
    nc = ca.shape[0]
    brt = wp.transpose(brot)
    for s in range(nsub):
        for i in range(n):
            old[i] = x[i]
            contact[i] = 0
            if w[i] > 0.0:
                vi = v[i] * damp
                vi = vi + wp.vec3(0.0, 0.0, -9.81 * h)
                v[i] = vi
                x[i] = x[i] + vi * h
            else:
                x[i] = target[i]
        for it in range(iters):
            for c in range(nc):
                a = ca[c]
                b = cb[c]
                wa = w[a]
                wb = w[b]
                ws = wa + wb
                if ws > 0.0:
                    d = x[b] - x[a]
                    ln = wp.length(d)
                    if ln > 1.0e-9:
                        kk = ck[c]
                        if cpl[c] == 2 and ln < crest[c]:
                            kk = kk * kcomp
                        corr = d * ((ln - crest[c]) / ln * kk / ws)
                        x[a] = x[a] + corr * wa
                        x[b] = x[b] - corr * wb
            # collisions
            for i in range(n):
                if w[i] > 0.0:
                    p = x[i]
                    # burger
                    lp = brt * (p - bpos)
                    sd = burger_sdf(lp, br, bh, brr)
                    off = layer_off[i]
                    if sd[0] < off:
                        nl = wp.vec3(sd[1], sd[2], sd[3])
                        lp = lp + nl * (off - sd[0])
                        p = brot * lp + bpos
                        contact[i] = 1
                        cnrm[i] = brot * nl
                    # board box
                    rel = p - board_c
                    hx = board_half[0] + 0.0006
                    hy = board_half[1] + 0.0006
                    hz = board_half[2] + 0.0006
                    if wp.abs(rel[0]) < hx and wp.abs(rel[1]) < hy and wp.abs(rel[2]) < hz:
                        px = hx - wp.abs(rel[0])
                        py = hy - wp.abs(rel[1])
                        pz = hz - wp.abs(rel[2])
                        if pz <= px and pz <= py:
                            sg = wp.sign(rel[2])
                            rel = wp.vec3(rel[0], rel[1], sg * hz)
                            cnrm[i] = wp.vec3(0.0, 0.0, sg)
                        elif px <= py:
                            sg = wp.sign(rel[0])
                            rel = wp.vec3(sg * hx, rel[1], rel[2])
                            cnrm[i] = wp.vec3(sg, 0.0, 0.0)
                        else:
                            sg = wp.sign(rel[1])
                            rel = wp.vec3(rel[0], sg * hy, rel[2])
                            cnrm[i] = wp.vec3(0.0, sg, 0.0)
                        p = rel + board_c
                        contact[i] = 1
                    if p[2] < table_z + 0.0006:
                        p = wp.vec3(p[0], p[1], table_z + 0.0006)
                        contact[i] = 1
                        cnrm[i] = wp.vec3(0.0, 0.0, 1.0)
                    # finger boxes
                    for k in range(nbox):
                        T = box_T[k]
                        R = wp.mat33(T[0, 0], T[0, 1], T[0, 2], T[1, 0], T[1, 1], T[1, 2], T[2, 0], T[2, 1], T[2, 2])
                        t = wp.vec3(T[0, 3], T[1, 3], T[2, 3])
                        q = wp.transpose(R) * (p - t)
                        lo = box_lo[k] - wp.vec3(0.0008, 0.0008, 0.0008)
                        hi = box_hi[k] + wp.vec3(0.0008, 0.0008, 0.0008)
                        if q[0] > lo[0] and q[0] < hi[0] and q[1] > lo[1] and q[1] < hi[1] and q[2] > lo[2] and q[2] < hi[2]:
                            best = 1.0e9
                            ax = 0
                            side = 0.0
                            for j in range(3):
                                dl = q[j] - lo[j]
                                dh = hi[j] - q[j]
                                if dl < best:
                                    best = dl
                                    ax = j
                                    side = lo[j]
                                if dh < best:
                                    best = dh
                                    ax = j
                                    side = hi[j]
                            if ax == 0:
                                q = wp.vec3(side, q[1], q[2])
                            elif ax == 1:
                                q = wp.vec3(q[0], side, q[2])
                            else:
                                q = wp.vec3(q[0], q[1], side)
                            p = R * q + t
                            contact[i] = 1
                            cnrm[i] = R * wp.vec3(0.0, 0.0, -1.0)
                    x[i] = p
        # friction: remove a share of the tangential slip of contact nodes
        for i in range(n):
            if contact[i] == 1 and w[i] > 0.0:
                dxs = x[i] - old[i]
                nn = cnrm[i]
                dn = wp.dot(dxs, nn)
                tang = dxs - nn * dn
                x[i] = x[i] - tang * mu
        # fold memory
        for c in range(nc):
            if cpl[c] == 1:
                ln = wp.length(x[cb[c]] - x[ca[c]])
                r0 = crest[c]
                if ln < r0 * (1.0 - ey):
                    crest[c] = ln / (1.0 - ey)
                elif ln > r0 * (1.0 + ey):
                    crest[c] = wp.min(cflat[c], ln / (1.0 + ey))
            elif cpl[c] == 2:
                # in-plane: the sheet resists stretch but gives way under compression by crumpling (plastic)
                ln = wp.length(x[cb[c]] - x[ca[c]])
                r0 = crest[c]
                if ln < r0 * (1.0 - eyc):
                    crest[c] = wp.max(cflat[c] * cmin, ln / (1.0 - eyc))
                elif ln > r0:
                    crest[c] = wp.min(cflat[c], ln)
        for i in range(n):
            vi = (x[i] - old[i]) / h
            sp = wp.length(vi)
            if sp > 2.0:
                vi = vi * (2.0 / sp)
            v[i] = vi


class Sheet:
    def __init__(self, centre, size, z, *, board, burger, table_z, bend_k=0.5, yield_strain=0.005, shear_k=0.8,
                 iters=12, substeps=10, thickness=0.0012, kcomp=0.5, eyc=0.03, cmin=0.6, mu=0.85):
        sx, sy = (size, size) if np.isscalar(size) else size
        self.nx = nx = int(round(sx / DX)) + 1
        self.ny = ny = int(round(sy / DX)) + 1
        self.n = nx
        self.size = (sx, sy)
        xs = centre[0] + (np.arange(nx) - (nx - 1) / 2) * DX
        ys = centre[1] + (np.arange(ny) - (ny - 1) / 2) * DX
        X, Y = np.meshgrid(xs, ys, indexing="ij")  # row i along x, column j along y
        self.x = np.stack([X, Y, np.full_like(X, z)], axis=-1).reshape(-1, 3)
        self.v = np.zeros_like(self.x)
        self.ids = ids = np.arange(nx * ny).reshape(nx, ny)
        self.iters, self.substeps, self.th = iters, substeps, thickness
        self.kcomp, self.eyc, self.cmin = kcomp, eyc, cmin
        self.board, self.table_z, self.burger, self.mu, self.ey = board, table_z, burger, mu, yield_strain
        A, B, R0, K, P = [], [], [], [], []

        def add(a, b, rest, k, pl=0):
            a = np.asarray(a).ravel(); b = np.asarray(b).ravel()
            A.append(a); B.append(b); R0.append(np.full(len(a), rest)); K.append(np.full(len(a), k))
            P.append(np.full(len(a), pl))

        cp = 2  # in-plane springs: stiff in tension, crumple (plastic) in compression
        add(ids[:, :-1], ids[:, 1:], DX, 1.0, cp)
        add(ids[:-1, :], ids[1:, :], DX, 1.0, cp)
        dg = DX * np.sqrt(2)
        add(ids[:-1, :-1], ids[1:, 1:], dg, shear_k, cp)
        add(ids[:-1, 1:], ids[1:, :-1], dg, shear_k, cp)
        add(ids[:, :-2], ids[:, 2:], 2 * DX, bend_k, 1)
        add(ids[:-2, :], ids[2:, :], 2 * DX, bend_k, 1)
        add(ids[:-2, :-2], ids[2:, 2:], 2 * dg, bend_k, 1)
        add(ids[:-2, 2:], ids[2:, :-2], 2 * dg, bend_k, 1)
        self.ca, self.cb = np.concatenate(A), np.concatenate(B)
        self.crest0 = np.concatenate(R0)
        self.crest = self.crest0.copy()
        self.ck, self.cpl = np.concatenate(K), np.concatenate(P)
        f = []
        for i in range(nx - 1):
            for j in range(ny - 1):
                a, b, c, d = ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]
                f.extend([(a, b, c), (a, c, d)])
        self.faces = np.asarray(f, np.int32)
        edge = np.zeros((nx, ny), bool)
        edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
        self.edge_nodes = ids[edge]
        self.pins = {}
        self.armed = {}
        self.sweep = {}
        self.overstrain = 0.0
        self.torn = False
        self.layer = np.full(nx * ny, -1)
        self.fold = 0
        self._pinned_before = False
        self.under = {}
        self.bad = False
        self.max_strain = 1.0
        N = nx * ny
        d = _DEV
        self._x = wp.array(self.x, dtype=wp.vec3, device=d)
        self._v = wp.array(self.v, dtype=wp.vec3, device=d)
        self._ca = wp.array(self.ca.astype(np.int32), dtype=int, device=d)
        self._cb = wp.array(self.cb.astype(np.int32), dtype=int, device=d)
        self._crest = wp.array(self.crest, dtype=float, device=d)
        self._cflat = wp.array(self.crest0, dtype=float, device=d)
        self._ck = wp.array(self.ck, dtype=float, device=d)
        self._cpl = wp.array(self.cpl.astype(np.int32), dtype=int, device=d)
        self._contact = wp.zeros(N, dtype=int, device=d)
        self._old = wp.zeros(N, dtype=wp.vec3, device=d)
        self._cn = wp.zeros(N, dtype=wp.vec3, device=d)
        self.contact = np.zeros(N, bool)

    # ------------------------------------------------------------------ grasp rule
    def update_pins(self, tcp, rot, aperture):
        """pins[arm] = list of (node, offset in TCP frame). A closing jaw grips every sheet edge lying between its
        fingertips (one edge, or several edges stacked by earlier folds)."""
        for arm in range(len(tcp)):
            if arm in self.pins:
                keep = []
                for node, off in self.pins[arm]:
                    target = tcp[arm] + rot[arm] @ off
                    if aperture[arm] <= 0.023 and np.linalg.norm(self.x[node] - target) <= 0.035:
                        keep.append((node, off))
                if keep:
                    self.pins[arm] = keep
                else:
                    del self.pins[arm]
                continue
            hist = self.sweep.setdefault(arm, [])
            hist.append(float(aperture[arm]))
            del hist[:-20]  # the last second of jaw openings
            if aperture[arm] > 0.020:
                self.armed[arm] = True
            if aperture[arm] > 0.012 or not self.armed.get(arm, False):
                continue
            # a closing jaw tries once; a jaw that is already shut (pressing) never picks the sheet up
            self.armed[arm] = False
            # the closing fingers sweep the gap they started from and gather the edges in it
            half = float(np.clip(max(hist) / 2, 0.012, 0.045))
            cand = self.edge_nodes
            local = (self.x[cand] - tcp[arm]) @ rot[arm]  # TCP frame: x closing axis, z approach
            good = (np.abs(local[:, 0]) < half) & (np.abs(local[:, 1]) < 0.011) & (local[:, 2] > -0.006) \
                & (local[:, 2] < 0.022)
            taken = {nd for lst in self.pins.values() for nd, _ in lst}
            sel = [c for c in cand[good][np.argsort(np.linalg.norm(local[good], axis=1))] if c not in taken][:8]
            if sel:
                offs = [(self.x[c] - tcp[arm]) @ rot[arm] for c in sel]
                for o in offs:
                    o[0] = np.clip(o[0], -0.003, 0.003)  # squeezed between the pads
                self.pins[arm] = [(int(c), o) for c, o in zip(sel, offs)]
        if self.pins and not self._pinned_before:
            self.fold += 1
        self._pinned_before = bool(self.pins)

    def _height_above_support(self, x):
        z = x[:, 2] - self.table_z
        if self.board:
            bd = self.board
            on = (np.abs(x[:, 0] - bd["cx"]) < bd["hx"]) & (np.abs(x[:, 1] - bd["cy"]) < bd["hy"])
            z = np.where(on, x[:, 2] - bd["top"], z)
        return z

    # ------------------------------------------------------------------ step
    def step(self, tcp, rot, aperture, burger_pos, burger_rot, finger_boxes=(), dt=0.05):
        self.update_pins(tcp, rot, aperture)
        x = self.x
        bp, bR = np.asarray(burger_pos, float), np.asarray(burger_rot, float)
        bg = self.burger
        N = len(x)
        w = np.ones(N, np.float32)
        target = x.copy()
        loc = (x - bp) @ bR
        rho = np.linalg.norm(loc[:, :2], axis=1)
        under_now = (rho < bg["r"] - 0.001) & (loc[:, 2] < 0.004) & (loc[:, 2] > -0.006)
        pinned = {node for lst in self.pins.values() for node, _ in lst}
        for nd in list(self.under):
            if not under_now[nd] or nd in pinned:
                del self.under[nd]
        for nd in np.flatnonzero(under_now):
            if nd not in self.under and nd not in pinned:
                self.under[int(nd)] = np.array([loc[nd, 0], loc[nd, 1], 0.0003])
        for nd, lp in self.under.items():
            target[nd] = bp + bR @ lp
            w[nd] = 0.0
        for k, lst in self.pins.items():
            for node, off in lst:
                target[node] = tcp[k] + rot[k] @ off
                w[node] = 0.0
        layer_off = (np.maximum(self.layer, 0) * self.th + 0.0015).astype(np.float32)
        bd = self.board
        bc = wp.vec3(bd["cx"], bd["cy"], bd["top"] - bd["t"] / 2)
        bh = wp.vec3(bd["hx"], bd["hy"], bd["t"] / 2)
        T = np.zeros((max(1, len(finger_boxes)), 4, 4), np.float32)
        lo = np.zeros((max(1, len(finger_boxes)), 3), np.float32)
        hi = np.zeros_like(lo)
        for k, (Tk, l, hgh) in enumerate(finger_boxes):
            T[k], lo[k], hi[k] = Tk, l, hgh
        d = _DEV
        self._x.assign(x.astype(np.float32))
        self._v.assign(self.v.astype(np.float32))
        wp.launch(substeps_kernel, dim=1, device=d, inputs=[
            self._x, self._v, wp.array(w, dtype=float, device=d), wp.array(target.astype(np.float32), dtype=wp.vec3, device=d),
            self._ca, self._cb, self._crest, self._cflat, self._ck, self._cpl, wp.array(layer_off, dtype=float, device=d),
            self._contact, self._old, self._cn,
            wp.vec3(*bp), wp.mat33(*bR.ravel()), float(bg["r"]), float(bg["h"]), float(bg["rr"]),
            bc, bh, float(self.table_z),
            wp.array(T, dtype=wp.mat44, device=d), wp.array(lo, dtype=wp.vec3, device=d),
            wp.array(hi, dtype=wp.vec3, device=d), len(finger_boxes),
            dt / self.substeps, self.substeps, self.iters, self.mu, self.ey, 0.985, 2 * DX,
            self.kcomp, self.eyc, self.cmin])
        self.x = x = self._x.numpy().astype(np.float64)
        self.v = self._v.numpy().astype(np.float64)
        self.contact = self._contact.numpy().astype(bool)
        lifted = self._height_above_support(x) > 0.012
        new = lifted & (self.layer < 0)
        self.layer[new] = max(self.fold, 1)
        st = np.linalg.norm(x[self.cb[:len(self.cb)]] - x[self.ca], axis=1)
        m = self.cpl != 1
        self.max_strain = float((st[m] / self.crest0[m]).max())  # stretch past the flat length
        self.overstrain = self.overstrain + dt if self.max_strain > 1.8 else max(0.0, self.overstrain - 0.1 * dt)
        if not np.isfinite(x).all():
            self.bad = True
        # a sheet stretched far past its length for a while would tear: latched failure
        self.torn |= self.bad or self.max_strain > 3.0 or self.overstrain > 0.25
        return x

    def triangles(self):
        return self.x[self.faces]


def ray_hits(origins, dirs, tris, eps=1e-12):
    """For each ray, whether it hits any triangle (Moller-Trumbore, vectorised). origins/dirs (M,3), tris (T,3,3)."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    out = np.zeros(len(origins), bool)
    for s in range(0, len(origins), 64):
        o = origins[s:s + 64, None, :]
        d = dirs[s:s + 64, None, :]
        p = np.cross(d, e2[None])
        det = (e1[None] * p).sum(-1)
        ok = np.abs(det) > eps
        inv = 1.0 / np.where(ok, det, 1.0)
        tv = o - v0[None]
        u = (tv * p).sum(-1) * inv
        q = np.cross(tv, e1[None])
        vv = (d * q).sum(-1) * inv
        t = (e2[None] * q).sum(-1) * inv
        hit = ok & (u >= 0) & (vv >= 0) & (u + vv <= 1) & (t > 1e-4)
        out[s:s + 64] = hit.any(1)
    return out


def coverage(tris, bp, bR, r, h, n_ring=24, n_h=4, n_top=5):
    """Fraction of burger surface sample points hidden by the sheet, seen from above and from +-x, +-y.

    Rays start just inside the burger (below the rounded crown edge / inside the side wall), so a sheet pressed snug
    onto the surface still counts as covering it; the sheet never enters the burger, so nothing inside is counted."""
    res = {}
    g = np.linspace(-r * 0.92, r * 0.92, 2 * n_top + 1)
    P = np.array([(a, b, h - 0.014) for a in g for b in g if a * a + b * b <= (0.92 * r) ** 2])
    W = P @ bR.T + bp
    res["top"] = float(ray_hits(W, np.tile(bR[:, 2], (len(W), 1)), tris).mean())
    angs = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
    zs = np.linspace(0.12 * h, 0.88 * h, n_h)
    for name, d in (("+x", [1, 0, 0]), ("-x", [-1, 0, 0]), ("+y", [0, 1, 0]), ("-y", [0, -1, 0])):
        d = np.asarray(d, float)
        pts = []
        for a in angs:
            nrm = np.array([np.cos(a), np.sin(a), 0])
            if nrm @ (bR.T @ d) > 0.2:
                for z in zs:
                    pts.append([(r - 0.004) * nrm[0], (r - 0.004) * nrm[1], z])
        W = np.array(pts) @ bR.T + bp
        res[name] = float(ray_hits(W, np.tile(d, (len(W), 1)), tris).mean())
    return res
