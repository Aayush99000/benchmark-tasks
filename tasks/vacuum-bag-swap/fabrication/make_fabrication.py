"""Writes the fabrication files for a real trial of vacuum-bag-swap, from the same dimensions the
simulation uses (vac.py):

  holder.stl   the collar holder: a mounting plate with a 50 mm hole for the inlet, two short channels
               (12 mm gap behind lips that cover the collar's outer 4 mm, 30 mm tall, chamfered
               lead-ins) and the stop ledge. Print in PLA/PETG, 0.2 mm layers, plate face down.
  bevel.stl    the bevel strip glued along the inner top edge of the compartment's front wall over the
               holder, so a collar let down on the wall's top slides into the channels.
  collar.svg   1:1 cutting template for the bag collar and grip tab (2 mm greyboard) and the gasket
               ring (3 mm EPDM foam), with the inlet hole.

Run:  python3 fabrication/make_fabrication.py      (from the task folder; writes next to this file)
Holder frame: x out of the plate into the compartment, y across, z up from the compartment floor's
level of the bench; millimetres in the outputs.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import vac as V  # noqa: E402

MM = 1000.0
PLATE_T = 0.0015                       # the printed plate's thickness (the channels are measured from its face)


def _box(cx, cy, cz, sx, sy, sz, pitch=0.0):
    """Triangles of a box (centre, size), turned ``pitch`` radians about y."""
    c, s = math.cos(pitch), math.sin(pitch)
    vs = []
    for dx in (-sx / 2, sx / 2):
        for dy in (-sy / 2, sy / 2):
            for dz in (-sz / 2, sz / 2):
                x, z = dx * c + dz * s, -dx * s + dz * c
                vs.append((cx + x, cy + dy, cz + z))
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, cc, d in faces:
        tris += [(vs[a], vs[b], vs[cc]), (vs[a], vs[cc], vs[d])]
    return tris


def _stl(name, tris, path):
    lines = [f"solid {name}"]
    for t in tris:
        lines += ["  facet normal 0 0 0", "    outer loop"]
        lines += [f"      vertex {p[0] * MM:.3f} {p[1] * MM:.3f} {p[2] * MM:.3f}" for p in t]
        lines += ["    endloop", "  endfacet"]
    lines.append(f"endsolid {name}")
    path.write_text("\n".join(lines) + "\n")


def holder():
    z0 = V.SEAT_Z - V.TOP - 0.025                 # plate from 25 mm below the stop ...
    z1 = V.INLET_Z - V.TOP + 0.040                # ... to 40 mm above the inlet's axis
    zi = V.INLET_Z - V.TOP
    hole = 0.025                                  # 50 mm square opening round the 38 mm inlet
    w = 2 * (V.LIP_Y + V.LIP[1] / 2 + 0.006 + 0.005)
    t = []
    px = -PLATE_T / 2
    t += _box(px, 0.0, (z0 + zi - hole) / 2, PLATE_T, w, (zi - hole) - z0)              # plate below the hole
    t += _box(px, 0.0, (zi + hole + z1) / 2, PLATE_T, w, z1 - (zi + hole))              # above it
    for s in (-1, 1):                                                                   # beside it
        t += _box(px, s * (hole + (w / 2 - hole) / 2), zi, PLATE_T, w / 2 - hole, 2 * hole)
    lz0, lz1 = V.TOP + 0.045 - V.TOP, V.LIP_TOP - V.TOP
    for s in (-1, 1):
        t += _box(V.SLOT + V.LIP[0] / 2, s * V.LIP_Y, (lz0 + lz1) / 2, V.LIP[0], V.LIP[1], lz1 - lz0)          # lip
        t += _box((V.SLOT + V.LIP[0]) / 2, s * (V.LIP_Y + 0.009), (lz0 + lz1) / 2, V.SLOT + V.LIP[0], 0.006,
                  lz1 - lz0)                                                                                  # side wall
        t += _box(V.SLOT + V.LIP[0] / 2 + 0.004, s * V.LIP_Y, lz1 + 0.004, 0.003, V.LIP[1], 0.014, pitch=0.6)  # lead-in
    t += _box(V.STOP_D / 2, 0.0, V.SEAT_Z - V.TOP - 0.003, V.STOP_D, 0.110, 0.006)                           # stop
    return t


def bevel():
    return _box(-0.004, 0.0, 0.006, 0.020, 0.150, 0.003, pitch=0.70)


def collar_svg():
    w, h = V.COLLAR[1] * MM, V.COLLAR[2] * MM
    tw, th = V.TAB[1] * MM, V.TAB[2] * MM
    hz = V.HOLE_Z * MM                           # hole centre above the collar's bottom edge
    m = 10.0
    W, H = w + 2 * m + 70, h + th + 2 * m
    top = m
    x0 = m
    card = (f'<path d="M {x0} {top + th + h} h {w} v {-h} h {-(w - tw) / 2} v {-th} h {-tw} v {th} h {-(w - tw) / 2} Z" '
            'fill="none" stroke="black" stroke-width="0.3"/>')
    hole = f'<circle cx="{x0 + w / 2}" cy="{top + th + h - hz}" r="19" fill="none" stroke="black" stroke-width="0.3"/>'
    gx, gy = x0 + w + 40, top + th + h - hz
    ring = (f'<circle cx="{gx}" cy="{gy}" r="26" fill="none" stroke="red" stroke-width="0.3"/>'
            f'<circle cx="{gx}" cy="{gy}" r="19" fill="none" stroke="red" stroke-width="0.3"/>')
    text = (f'<text x="{x0}" y="{H - 2}" font-size="3">collar {w:.0f} x {h:.0f} mm + tab {tw:.0f} x {th:.0f} mm, '
            f'2 mm greyboard; hole 38 mm; red: gasket ring 52/38 mm, 3 mm EPDM foam</text>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">'
            f'{card}{hole}{ring}{text}</svg>\n')


if __name__ == "__main__":
    _stl("holder", holder(), HERE / "holder.stl")
    _stl("bevel", bevel(), HERE / "bevel.stl")
    (HERE / "collar.svg").write_text(collar_svg())
    print("wrote", ", ".join(p.name for p in sorted(HERE.glob("*.st*")) + [HERE / "collar.svg"]))
