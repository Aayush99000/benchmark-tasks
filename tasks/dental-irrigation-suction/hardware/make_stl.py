"""Write the printable parts as STL files (millimetres), from the same numbers as the simulation.

    python hardware/make_stl.py            writes hardware/stl/*.stl

Parts (print in PETG unless noted; 0.2 mm layers, 4 walls, 30 % infill):
  face_shell.stl     the face round the opening, a 4 mm shell of overlapping patches, opening 45 x 64 mm (skin-tone PETG);
                     patches that would meet the typodont's gums or bases are left out, and two 4 mm pins on its inside
                     locate it on the tray
  lips.stl           the ring round the opening, 9 mm round section (TPU 95A)
  oral_tray.stl      floor of the mouth, cheeks, the throat sill and the throat channel with its drain hole. The floor is
                     open round the lower arch (a horseshoe slot 2 mm clear of the gum), so that the typodont's gum comes
                     up through it; the floor stands on four bosses on the lower base, screwed down with M3 screws
  tongue.stl         adult tongue, 60 x 26 x 20 mm (TPU 95A, 15 % gyroid infill, so that it gives a little)
  jaw_spacer.stl     the mouth prop: a wedge between the right first and second molars, with flanges that straddle the
                     molars of both jaws, holds the jaws 28 degrees open (45 mm between the incisal edges); TPU 95A
  stand.stl          a 4 mm base plate and a 40 mm block under the back of the head: holds the shell at its height and tilt
  bulb_holder.stl    ring stand with drip cup for the bulb syringe
  ejector_handle.stl 14 mm x 80 mm handle with a 6.6 mm bore: the ejector's straight end and the 1/4 in tubing meet inside it
  ejector_cradle.stl two V-notch posts that hold the handle level

Frames: oral_tray, tongue and jaw_spacer are in mouth millimetres (x toward the throat along the lower occlusal plane,
y to the patient's right, z toward the upper jaw, origin at the lower incisal point; the README gives l, to the left): they fit the typodont whatever its
tilt. face_shell, lips and stand are in table millimetres as mounted (x away from the robot, y left, z up, same origin),
so the stand sets the 35 degree tilt. Holders and handle are in their own frames.

The typodont's bases are taken at nominal sizes (TYPODONT below; measure yours and change them if they differ). The script
checks that no printed part meets the typodont, and prints the clearances.
"""

from __future__ import annotations

import math
import pathlib
import struct
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import mouth as M  # noqa: E402


def _sphere(rings=16, segs=32):
    v, f = [], []
    for i in range(rings + 1):
        a = math.pi * i / rings - math.pi / 2
        for j in range(segs):
            b = 2 * math.pi * j / segs
            v.append((math.cos(a) * math.cos(b), math.cos(a) * math.sin(b), math.sin(a)))
    for i in range(rings):
        for j in range(segs):
            p, q = i * segs + j, i * segs + (j + 1) % segs
            f += [(p, q, q + segs), (p, q + segs, p + segs)]
    return np.array(v), np.array(f)


def blob(semi, center, e=1.0, rings=16, segs=32):
    v, f = _sphere(rings, segs)
    return np.sign(v) * np.abs(v) ** e * np.asarray(semi) + np.asarray(center), f


def box(size, center):
    return blob(np.asarray(size) / 2, center, e=0.05, rings=8, segs=16)


def cylinder(r, h, center, segs=48):
    th = np.linspace(0, 2 * math.pi, segs, endpoint=False)
    c = np.asarray(center, float)
    v = [(c[0] + r * math.cos(t), c[1] + r * math.sin(t), c[2] - h / 2) for t in th] + [(c[0] + r * math.cos(t), c[1] + r * math.sin(t), c[2] + h / 2) for t in th]
    v += [tuple(c - [0, 0, h / 2]), tuple(c + [0, 0, h / 2])]
    f = []
    for i in range(segs):
        j = (i + 1) % segs
        f += [(i, j, segs + j), (i, segs + j, segs + i), (2 * segs, j, i), (2 * segs + 1, segs + i, segs + j)]
    return np.array(v), np.array(f)


def annulus(r_out, r_in, h, center, segs=48):
    """A tube: outer radius, bore radius, length along z."""
    th = np.linspace(0, 2 * math.pi, segs, endpoint=False)
    c = np.asarray(center, float)
    v = []
    for r in (r_out, r_in):
        for z in (-h / 2, h / 2):
            v += [(c[0] + r * math.cos(t), c[1] + r * math.sin(t), c[2] + z) for t in th]
    f = []
    for i in range(segs):
        j = (i + 1) % segs
        ob, ot, ib, it = 0, segs, 2 * segs, 3 * segs
        f += [(ob + i, ob + j, ot + j), (ob + i, ot + j, ot + i)]  # outside
        f += [(ib + i, it + j, ib + j), (ib + i, it + i, it + j)]  # bore
        f += [(ot + i, ot + j, it + j), (ot + i, it + j, it + i)]  # top
        f += [(ob + i, ib + j, ob + j), (ob + i, ib + i, ib + j)]  # bottom
    return np.array(v), np.array(f)


def tube_ring(points, r, sides=16):
    """A closed ring of round section through `points` (closed loop)."""
    pts = np.asarray(points, float)
    n = len(pts)
    v, f = [], []
    for k in range(n):
        d = pts[(k + 1) % n] - pts[k - 1]
        d /= np.linalg.norm(d)
        u = np.cross(d, [0, 0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0, 0])
        u /= np.linalg.norm(u)
        w = np.cross(d, u)
        for s in range(sides):
            a = 2 * math.pi * s / sides
            v.append(pts[k] + r * (math.cos(a) * u + math.sin(a) * w))
    for k in range(n):
        for s in range(sides):
            a, b = k * sides + s, k * sides + (s + 1) % sides
            c, d = ((k + 1) % n) * sides + s, ((k + 1) % n) * sides + (s + 1) % sides
            f += [(a, c, b), (b, c, d)]
    return np.array(v), np.array(f)


def write(name, bodies, mouth_frame=False):
    """Several closed bodies in one STL (slicers merge overlapping bodies). Mouth-frame parts are written with y to the
    patient's right (y = -l), which makes the frame right-handed, so that the print is not a mirror image."""
    tris = []
    for v, f in bodies:
        if mouth_frame:
            v, f = v * [1.0, -1.0, 1.0], f[:, ::-1]
        for a, b, c in f:
            tris.append((v[a], v[b], v[c]))
    path = HERE / "stl" / name
    path.parent.mkdir(exist_ok=True)
    with open(path, "wb") as out:
        out.write(b"printable part".ljust(80, b" "))
        out.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            out.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    print(f"{path.relative_to(HERE.parent)}: {len(tris)} triangles")


def cuboid(lo, hi):
    """An axis-aligned box from corner lo to corner hi: 8 vertices, 12 triangles, outward normals."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = np.array([(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return v, np.array(f)


def along_p(body):
    """A body built along z, turned to lie along x (p): swap x and z, and flip the faces to keep them outward."""
    v, f = body
    return v[:, [2, 1, 0]], f[:, ::-1]


def runs(keep, p_range, l_cells, c_cells, dp=0.5):
    """Solid made of boxes: for each (l, c) cell, the runs along p of cells whose centre `keep(p, l, c)` accepts."""
    out = []
    ps = np.arange(p_range[0], p_range[1], dp)
    for l0, l1 in l_cells:
        for c0, c1 in c_cells:
            ok = [keep(p + dp / 2, (l0 + l1) / 2, (c0 + c1) / 2) for p in ps]
            start = None
            for i, k in enumerate(ok + [False]):
                if k and start is None:
                    start = i
                elif not k and start is not None:
                    out.append(cuboid((ps[start], l0, c0), (ps[start] + (i - start) * dp, l1, c1)))
                    start = None
    return out


def cells(a, b, step):
    edges = np.arange(a, b + 1e-9, step)
    return list(zip(edges[:-1], edges[1:]))


# The typodont (Kilgore/Nissin 200-type, generic M8011), nominal: each jaw's base is a block 75 mm wide and 58 mm deep
# under its gum, 10 mm high, so that the jaw is 30 mm from its occlusal plane to the bottom of its base. In mouth mm; the
# upper jaw's numbers are in its closed position.
TYPODONT = dict(width=75.0, lower_p=(-4.0, 54.0), upper_p=(-6.0, 54.0), base_top=20.0, base_bottom=30.0)
CLEAR = 1.0  # mm every printed part keeps from the typodont
SLOT_CLEAR = 2.0  # mm the floor's horseshoe slot keeps from the gum
FLOOR_T = 3.0
BASE_SCREWS = [(28.0, 35.0), (28.0, -35.0), (52.0, 11.0), (52.0, -11.0)]  # M3 through the floor's bosses into the lower base
TONGUE_SCREWS = [(25.0, 0.0), (45.0, 0.0)]  # M3 up through the floor into the tongue
DRAIN = (70.5, 0.0, 3.0)  # p, l, radius: the hole to the throat cup
FACE_PINS = [(39.5, 20.0), (-39.5, 20.0)]  # (l, c): pins along p from the face shell into bosses at the cheeks' front ends


def in_gums(p, l, c, table, grow, upper=False):
    """Is the point (closed frame for the upper jaw) within `grow` of a gum or crown of that jaw?"""
    cc = -c if upper else c
    for pp, ll, md, bl, h in table.values():
        for s in (1, -1):
            if abs(p - pp) <= md / 2 + 0.6 + grow and abs(l - s * ll) <= bl / 2 + 0.5 + grow and -(h + 10.5 + grow) <= cc <= grow:
                return True
    return False


def in_typodont(p, l, c, grow=CLEAR) -> bool:
    """Is the point (mouth mm) within `grow` of the typodont: either jaw's gums and crowns, or either base?"""
    t = TYPODONT
    if in_gums(p, l, c, M.LOWER, grow):
        return True
    if t["lower_p"][0] - grow <= p <= t["lower_p"][1] + grow and abs(l) <= t["width"] / 2 + grow and -t["base_bottom"] - grow <= c <= -t["base_top"] + grow:
        return True
    q = upper_closed(p, l, c)
    if in_gums(q[0], q[1], q[2], M.UPPER, grow, upper=True):
        return True
    return t["upper_p"][0] - grow <= q[0] <= t["upper_p"][1] + grow and abs(l) <= t["width"] / 2 + grow and t["base_top"] - grow <= q[2] <= t["base_bottom"] + grow


def upper_closed(p, l, c):
    a = math.radians(M.OPEN_DEG)
    dp, dc = p - M.HINGE[0], c - M.HINGE[1]
    return (M.HINGE[0] + dp * math.cos(a) - dc * math.sin(a), l, M.HINGE[1] + dp * math.sin(a) + dc * math.cos(a))


def face_frame():
    import parts as P

    centre, n, v, w, semi = P._face_frame()
    return centre, np.stack([n, v, w], axis=1), np.asarray(semi)


def inside_face(p, l, c, thickness=0.004) -> bool:
    """Is the point (mouth mm) inside the face shell's inner surface?"""
    centre, R, semi = face_frame()
    y = R.T @ (M.world(p, l, c) - centre) / (semi - thickness)
    return float(y @ y) < 1.0


def in_stand(p, l, c, grow=CLEAR) -> bool:
    import parts as P

    x = (M.world(p, l, c) - M.O) * 1000
    for size, centre in (P.STAND_PLATE, P.STAND_BLOCK):
        lo = (np.array([centre[0], centre[1], M.TOP + centre[2] - M.O[2]]) - np.array(size) / 2) * 1000 - grow
        hi = lo + np.array(size) * 1000 + 2 * grow
        if np.all(x >= lo) and np.all(x <= hi):
            return True
    return x[2] < (M.TOP - M.O[2]) * 1000  # below the table top


def face_pin_p(l, c):
    """p where the face shell's inner surface crosses the line (l, c) going back from the face."""
    for p in np.arange(-20.0, 60.0, 0.05):
        if inside_face(p, l, c):
            return float(p)
    raise ValueError("no face there")


def main():
    # Mouth-frame parts are written in mouth millimetres: x = p (toward the throat), y = l (patient's left), z = c.
    fc = M.FLOOR_C
    holes = [(p, l, 1.7) for p, l in BASE_SCREWS + TONGUE_SCREWS] + [DRAIN]

    def clear_of_all(p, l, c):
        return inside_face(p, l, c) and not in_typodont(p, l, c) and not in_stand(p, l, c)

    def floor(p, l, c):
        if in_gums(p, l, -10.0, M.LOWER, SLOT_CLEAR):  # the horseshoe slot round the lower arch's gum
            return False
        if any(math.hypot(p - hp, l - hl) <= r for hp, hl, r in holes):
            return False
        return clear_of_all(p, l, c)

    wall_l = [(-41.0, -38.0), (38.0, 41.0)]
    tray = runs(floor, (-12.0, 76.5), cells(-41.0, 41.0, 0.5), [(fc - FLOOR_T, fc)])  # the open floor and the throat channel
    tray += runs(clear_of_all, (-12.0, 76.5), wall_l, cells(fc - FLOOR_T, fc + 44.0, 1.0))  # cheeks
    tray += runs(clear_of_all, (M.SILL_P, M.SILL_P + 4.0), [(-38.0, 38.0)], [(fc, fc + M.SILL_H)], dp=4.0)  # the throat sill
    tray += runs(clear_of_all, (73.5, 76.5), [(-38.0, 38.0)], cells(fc, fc + 38.0, 1.0), dp=3.0)  # back wall over the throat
    gap = -TYPODONT["base_top"] - (fc - FLOOR_T)  # mm from the floor's underside down to the lower base's top (negative: down)
    for p, l in BASE_SCREWS:  # bosses from the floor's underside down onto the lower base, 3.4 mm bore for M3
        tray.append(annulus(4.0, 1.7, abs(gap), (p, l, fc - FLOOR_T + gap / 2)))
    pins = []
    for l, c in FACE_PINS:  # the face shell's pins go 8 mm into these bosses at the cheeks' front ends
        p0 = face_pin_p(l, c)
        tray.append(along_p(annulus(3.6, 2.15, 8.0, (c, l, p0 + 0.5 + 4.0))))
        pins.append((p0, l, c))
    write("oral_tray.stl", tray, mouth_frame=True)
    t = M.TONGUE
    write("tongue.stl", [blob(t["semi"], t["center"], rings=24, segs=48)], mouth_frame=True)
    import parts as P

    write("jaw_spacer.stl", [(mesh.v, mesh.f) for _, mesh in P.mouth_prop_meshes()], mouth_frame=True)
    # face and lips, in world millimetres relative to the lower incisal point (the stand puts that point 60 mm above the table)
    _, sectors = P._face()  # closed convex patches, 4 mm thick, that overlap into the shell round the opening
    kept = [sec for sec in sectors if not any(in_typodont(*M.mouth(x + M.O)) for x in sec.v)]
    face = [(sec.v * 1000, sec.f) for sec in kept]
    for p0, l, c in pins:  # 3.9 mm pins, from inside the shell's thickness to 7.5 mm behind its inner surface
        v, f = along_p(cylinder(1.95, 10.5, (c, l, p0 + 7.5 - 5.25)))
        face.append(((M.world(v[:, 0], v[:, 1], v[:, 2]) - M.O) * 1000, f[:, ::-1] if np.linalg.det(np.stack([M.P_AX, M.L_AX, M.N_AX], 1)) < 0 else f))
    write("face_shell.stl", face)
    print(f"face_shell: {len(kept)} of {len(sectors)} patches; left out where they would meet the typodont")
    write("lips.stl", [tube_ring(P._lip_outline(scale_out=0.0045) * 1000, 4.5)])
    write("stand.stl", [box(tuple(v * 1000 for v in size), (c[0] * 1000, c[1] * 1000, (M.TOP + c[2] - M.O[2]) * 1000)) for size, c in (P.STAND_PLATE, P.STAND_BLOCK)])
    write("bulb_holder.stl", [tube_ring([(19 * math.cos(a), 19 * math.sin(a), 70) for a in np.linspace(0, 2 * math.pi, 40, endpoint=False)], 3),
                              box((10, 10, 70), (0, -42, 35)), box((10, 26, 6), (0, -32, 70)), cylinder(50, 6, (0, 0, 3)),
                              annulus(32, 30, 14, (0, 0, 13))])
    write("ejector_handle.stl", [annulus(7, 3.3, 80, (0, 0, 40))])
    write("ejector_cradle.stl", [box((26, 12, 105), (0, -35, 52.5)), box((26, 12, 105), (0, 35, 52.5)), box((50, 90, 6), (0, 0, 3))])
    check(pins)


def check(pins):
    """Print the fit: the typodont's lowest point over the stand, and that no printed mouth part meets it."""
    t = TYPODONT
    low = min((M.world(p, l, -t["base_bottom"])[2] - M.TOP) * 1000 for p in t["lower_p"] for l in (-t["width"] / 2, t["width"] / 2))
    plate = P_PLATE_TOP()
    print(f"typodont: lower base's lowest edge {low:.1f} mm above the table, {low - plate:.1f} mm above the stand plate")
    print(f"face pins at p = {', '.join(f'{p:.1f}' for p, _, _ in pins)} mm (l = +-{abs(pins[0][1]):.1f}, c = {pins[0][2]:.1f})")


def P_PLATE_TOP():
    import parts as P

    return P.STAND_PLATE[0][2] * 1000
if __name__ == "__main__":
    main()
