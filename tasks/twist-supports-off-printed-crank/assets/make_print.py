"""Build the print: an offset crank link (two 28 mm bosses, 8 mm bores, 80 mm apart, joined by a 16 x 8 mm bar; one
boss on the bed, the bar 25 mm up on it, the other boss on top of the bar's far end) as it sits on the printer's bed,
with four slicer-style tree supports in the same filament: two under the bar, two beside it under the up boss's
overhanging rim. Their 0.8 mm tips stop 0.2 mm under the part (the slicer's top Z gap).

Writes, next to this file:
- crank.stl: the part alone (mm, as it lies on the bed);
- crank_with_supports.stl: the part and its four tree supports as one print (mm, as it lies on the bed), each tip
  0.2 mm under the part: print this with the slicer's own supports turned off, so a real trial has the same supports
  as the simulation;
- crank.usda, support_0.usda .. support_3.usda: the simulation's visual meshes (m), in the task's body frame: the
  print turned 90 degrees so its bar runs along the table's y, origin on the bed under the part's middle.

    python make_print.py    (needs numpy, trimesh, shapely, mapbox-earcut)
"""

from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Point
from shapely.geometry import Point as _P
from shapely.geometry import box
from shapely.geometry import box as _box

HERE = Path(__file__).resolve().parent
MM = 0.001
ORANGE = (0.93, 0.42, 0.12)  # orange PLA: part and supports are the same filament
TURN = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1])  # print x -> table y


def prism(poly, z0, h):
    m = trimesh.creation.extrude_polygon(poly, h)
    m.apply_translation([0, 0, z0])
    m.apply_scale(MM)
    return m


def rounded(x0, y0, x1, y1, r):
    return box(x0 + r, y0 + r, x1 - r, y1 - r).buffer(r, resolution=8)


# -- the part: an offset crank link (mm), as printed: one pivot boss down on the bed, the bar 25 mm up on it, the
# other boss up at the far end. Bar 16 mm wide, 8 mm thick, rounded ends; bosses 28 mm across (wider than the bar, so
# no orientation avoids supports), 25 mm tall, 8 mm bores.

PITCH, BAR_W, BAR_T, BOSS_R, BOSS_H, BORE_R = 80.0, 16.0, 8.0, 14.0, 25.0, 4.0
xa, xb = -PITCH / 2, PITCH / 2  # the up boss (a) and the down boss (b)
bar_poly = _box(xa, -BAR_W / 2, xb, BAR_W / 2).union(_P(xa, 0).buffer(BAR_W / 2, resolution=32))
bar_poly = bar_poly.union(_P(xb, 0).buffer(BAR_W / 2, resolution=32))
bar_poly = bar_poly.difference(_P(xa, 0).buffer(BORE_R, resolution=24)).difference(_P(xb, 0).buffer(BORE_R, resolution=24))
ring = lambda x: _P(x, 0).buffer(BOSS_R, resolution=32).difference(_P(x, 0).buffer(BORE_R, resolution=24))  # noqa: E731
boss_down = prism(ring(xb), 0, BOSS_H)
bar = prism(bar_poly, BOSS_H, BAR_T)
boss_up = prism(ring(xa), BOSS_H + BAR_T, BOSS_H)
part = trimesh.util.concatenate([boss_down, bar, boss_up])
UNDER = BOSS_H  # the bar's underside, 25 mm above the bed


# -- tree supports: trunks from the bed, tapering, splitting into branches whose thin tips touch the overhangs' -------
# undersides (0.2 mm gap, about 0.8 mm tips), joints smoothed with spheres, as Cura tree / PrusaSlicer organic supports.
def foot(xy, r_top, r_bed=None, h=1.6):
    """The flared foot where a trunk meets the bed: a short cone, wider at the bed."""
    r_bed = r_bed or r_top * 1.8
    c = trimesh.creation.cylinder(radius=1.0, height=h, sections=24)
    z = c.vertices[:, 2] / h + 0.5
    c.vertices[:, :2] *= (r_bed + (r_top - r_bed) * z)[:, None]
    c.apply_translation([xy[0], xy[1], h / 2])
    c.apply_scale(MM)
    return c


def limb(a, b, r0, r1, ball_a=True):
    """A tapered cylinder from a (radius r0) to b (radius r1), mm, with a sphere at each end for smooth joints."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    v = b - a
    length = float(np.linalg.norm(v))
    cyl = trimesh.creation.cylinder(radius=1.0, height=length, sections=20)
    # taper: scale each vertex's xy by its height along the cylinder
    z = cyl.vertices[:, 2] / length + 0.5  # 0 at a, 1 at b
    rad = r0 + (r1 - r0) * z
    cyl.vertices[:, :2] *= rad[:, None]
    cyl.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], v / length))
    cyl.apply_translation((a + b) / 2)
    parts = [cyl]
    for p, r in (((a, r0), (b, r1)) if ball_a else ((b, r1),)):
        s = trimesh.creation.icosphere(subdivisions=2, radius=r)
        s.apply_translation(p)
        parts.append(s)
    out = trimesh.util.concatenate(parts)
    out.apply_scale(MM)
    return out


def tree(base_xy, split_z, tips, r_base=3.2):
    """One tree: a trunk from the bed at base_xy up to a split at split_z, then a branch to each tip (x, y, z_tip),
    each branch ending in a thin tip just under the overhang."""
    bx, by = base_xy
    split = (bx, by, split_z)
    parts = [foot((bx, by), r_base), limb((bx, by, 0.0), split, r_base, r_base * 0.75, ball_a=False)]
    for tx, ty, tz in tips:
        mid = ((bx + tx) / 2, (by + ty) / 2, split_z + (tz - split_z) * 0.55)
        parts.append(limb(split, mid, r_base * 0.7, 1.6))
        parts.append(limb(mid, (tx, ty, tz - 1.2), 1.6, 0.8))  # tip 0.8 mm, ends 0.2 mm under the overhang
    return trimesh.util.concatenate(parts)


GAP = 0.2
z_tip = UNDER - GAP
z_rim = UNDER + BAR_T - GAP  # the up boss's rim overhangs the bar on both sides, 33 mm up
support_trees = [
    # under the raised bar (its underside 25 mm up): two trees 35 mm apart, splitting 20 mm up into six tips
    # (high enough that a hand's fingers close on the bare trunk below the branches)
    tree((-35, 0), 20, [(-41, -5, z_tip), (-41, 5, z_tip), (-35, -5, z_tip), (-35, 5, z_tip), (-29, -5, z_tip),
                        (-29, 5, z_tip)]),
    tree((0, 0), 20, [(-6, -5, z_tip), (-6, 5, z_tip), (0, -5, z_tip), (0, 5, z_tip), (6, -5, z_tip),
                      (6, 5, z_tip)]),
    # beside the bar, up to the up boss's overhanging rim
    tree((-40, 18), 18, [(-45, 12, z_rim), (-35, 12, z_rim), (-50, 9.5, z_rim)], r_base=2.6),
    tree((-40, -18), 18, [(-45, -12, z_rim), (-35, -12, z_rim), (-31, -9.5, z_rim)], r_base=2.6),
]



def _lin(c: float) -> float:  # sRGB -> linear, as yam_arena.station.linear_rgb
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _fmt(a: np.ndarray, nd: int) -> str:
    return ", ".join("(" + ", ".join(f"{v:.{nd}f}".rstrip("0").rstrip(".") or "0" for v in row) + ")"
                     for row in a)


def write_usda(path, parts, origin=(0.0, 0.0, 0.0), nd: int = 5) -> int:
    """Write ``parts`` (name, vertices, faces, rgb, roughness) to ``path``, shifted by ``-origin``. Returns bytes."""
    lines = ['#usda 1.0', '(', '    defaultPrim = "root"', '    metersPerUnit = 1', '    upAxis = "Z"', ')', '',
             'def Xform "root"', '{']
    lines += ['    def Scope "Looks"', '    {']
    for name, _, _, rgb, rough in parts:
        r, g, b = (_lin(c) for c in rgb)
        lines += [f'        def Material "{name}_look"', '        {',
                  f'            token outputs:surface.connect = </root/Looks/{name}_look/shader.outputs:surface>',
                  '            def Shader "shader"', '            {',
                  '                uniform token info:id = "UsdPreviewSurface"',
                  f'                color3f inputs:diffuseColor = ({r:.4f}, {g:.4f}, {b:.4f})',
                  f'                float inputs:roughness = {rough}', '                float inputs:metallic = 0',
                  '                token outputs:surface', '            }', '        }']
    lines += ['    }']
    o = np.asarray(origin, float)
    for name, v, f, _, _ in parts:
        v, f = np.asarray(v, float) - o, np.asarray(f, int)
        lo, hi = v.min(0), v.max(0)
        lines += [f'    def Mesh "{name}" (', '        prepend apiSchemas = ["MaterialBindingAPI"]', '    )', '    {',
                  f'        float3[] extent = [{_fmt(np.array([lo, hi]), nd)}]',
                  f'        int[] faceVertexCounts = [{", ".join("3" for _ in range(len(f)))}]',
                  f'        int[] faceVertexIndices = [{", ".join(str(i) for i in f.ravel())}]',
                  f'        point3f[] points = [{_fmt(v, nd)}]',
                  '        uniform token subdivisionScheme = "none"',
                  '        uniform bool doubleSided = 1',
                  f'        rel material:binding = </root/Looks/{name}_look>', '    }']
    lines += ['}', '']
    text = "\n".join(lines)
    Path(path).write_text(text)
    return len(text.encode())


def main() -> None:
    (part_mm := part.copy()).apply_scale(1 / MM)
    part_mm.export(HERE / "crank.stl")
    whole = trimesh.util.concatenate([part] + support_trees)
    whole.apply_scale(1 / MM)
    whole.export(HERE / "crank_with_supports.stl")
    for name, m in [("crank", part)] + [(f"support_{k}", t) for k, t in enumerate(support_trees)]:
        m = m.copy()
        m.apply_transform(TURN)
        size = write_usda(HERE / f"{name}.usda", [(name, m.vertices, m.faces, ORANGE, 0.6)])
        lo, hi = m.bounds
        print(f"{name}: {len(m.faces)} faces, {size / 1e3:.0f} kB, bounds (m) {lo.round(4)} .. {hi.round(4)}")


if __name__ == "__main__":
    main()
