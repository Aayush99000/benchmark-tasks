"""The buildable stool: its printed parts as STL files, a check of how they move together, and the dimensioned drawing.

    pip install manifold3d trimesh matplotlib numpy
    python build_parts.py            # writes stl/*.stl and drawing.png, and prints the check

Every number comes from stool_dimensions.py (mm). The check turns each leg from folded (0) to past its stop (101 degrees)
against the top and, at each angle, finds how far the latch's bolt has to be pushed back to clear the leg's wedge: the
latch's travel as the CAD has it, to compare with the simulator's model (stool_dimensions.bolt_push_mm).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from manifold3d import CrossSection, Manifold

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stool_dimensions as D  # noqa: E402

SEG = 96  # segments in a full circle


# -- helpers ------------------------------------------------------------------------------------------------------------
def box(x, y, z) -> Manifold:
    """An axis-aligned box from (x0, x1), (y0, y1), (z0, z1)."""
    return Manifold.cube([x[1] - x[0], y[1] - y[0], z[1] - z[0]]).translate([x[0], y[0], z[0]])


def cyl_x(x, y, z, r) -> Manifold:
    """A cylinder along x from x[0] to x[1], its axis at (y, z)."""
    return Manifold.cylinder(x[1] - x[0], r, circular_segments=SEG).rotate([0, 90, 0]).translate([x[0], y, z])


YZ_TO_XYZ = np.array([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])


def prism_x(poly_yz, x) -> Manifold:
    """A polygon drawn in (y, z), extruded along x from x[0] to x[1]."""
    return CrossSection([poly_yz]).extrude(x[1] - x[0]).transform(YZ_TO_XYZ).translate([x[0], 0, 0])


def sector(r0, r1, a0, a1, centre=(0.0, 0.0), n=None):
    """An annular sector in (y, z): radii r0-r1, angles a0-a1 (degrees, from +y towards +z), counter-clockwise."""
    n = n or max(8, int(abs(a1 - a0) / 2))
    angs = np.radians(np.linspace(a0, a1, n + 1))
    outer = [(centre[0] + r1 * math.cos(a), centre[1] + r1 * math.sin(a)) for a in angs]
    inner = [(centre[0] + r0 * math.cos(a), centre[1] + r0 * math.sin(a)) for a in angs[::-1]] if r0 > 0 else [centre]
    return outer + inner


def rot_x_about_axis(part: Manifold, deg: float, y: float, z: float) -> Manifold:
    return part.translate([0, -y, -z]).rotate([deg, 0, 0]).translate([0, y, z])


# -- the parts ----------------------------------------------------------------------------------------------------------
def top_end_features():
    """(added, cut) at the +y end, in the top's frame."""
    hy, hz = D.HINGE_Y, D.HINGE_Z
    add, cut = [], []
    for ex in D.EAR_X:  # an ear: round the rod like the knuckle, with a web up to the underside
        add.append(cyl_x(ex, hy, hz, D.KNUCKLE_R))
        add.append(box(ex, (hy - D.KNUCKLE_R, hy + D.KNUCKLE_R), (hz, -D.TOP_T / 2 + 0.01)))
    add.append(box(D.BOSS_X, (hy + D.BOSS_Y[0], hy + D.BOSS_Y[1]), (-D.TOP_T / 2, D.TOP_T / 2)))
    cut.append(cyl_x((D.EAR_X[0][0] - 1, D.EAR_X[1][1] + 1), hy, hz, D.ROD_HOLE_D / 2))
    cut.append(prism_x(sector(0.0, D.POCKET_R, *D.POCKET_DEG, centre=(hy, hz)), D.POCKET_X))
    c = D.CHANNEL_CLEAR
    by = hy + D.BOLT_Y
    cut.append(box((D.CHANNEL_X[0] - 0.5, D.CHANNEL_X[1]), (by - D.BOLT_W / 2 - c, by + D.BOLT_W / 2 + c),
                   (hz + D.BOLT_R[0] - c, hz + D.BOLT_R[1] + c)))
    cut.append(cyl_x(D.BORE_X, by, D.BOLT_AXIS_Z, D.BORE_D / 2))
    cut.append(cyl_x((D.WALL_X[0] - 0.1, D.WALL_X[1] + 0.1), by, D.BOLT_AXIS_Z, 3.4 / 2))
    for sx in (-1, 1):  # the rib's screws, counterbored from below for their heads
        cut.append(Manifold.cylinder(D.TOP_T + 1, 1.7, circular_segments=32).translate(
            [sx * D.RIB_SCREW_X, D.RIB_Y, -D.TOP_T / 2 - 0.5]))
        cut.append(Manifold.cylinder(3.5, 3.0, circular_segments=32).translate(
            [sx * D.RIB_SCREW_X, D.RIB_Y, -D.TOP_T / 2 - 0.5]))
    cut.append(Manifold.cylinder(D.BALLAST_DEPTH + 0.5, D.BALLAST_D / 2, circular_segments=SEG).translate(
        [0, D.BALLAST_Y, -D.TOP_T / 2 - 0.5]))
    return add, cut


def top() -> Manifold:
    slab = box((-D.TOP_D / 2, D.TOP_D / 2), (-D.TOP_L / 2, D.TOP_L / 2), (-D.TOP_T / 2, D.TOP_T / 2))
    add, cut = top_end_features()
    add = Manifold.batch_boolean(add, _add_op())
    cut = Manifold.batch_boolean(cut, _add_op())
    one_end = (slab + add) - cut
    other = (add.mirror([0, 1, 0]))
    whole = one_end + other
    return whole - cut.mirror([0, 1, 0])


def _add_op():
    from manifold3d import OpType
    return OpType.Add


def wedge_x(beta_deg: float) -> float:
    """The wedge face's x at angle ``beta_deg`` in the leg's frame."""
    frac = (beta_deg - D.WEDGE_TRAIL_DEG) / D.WEDGE_SPAN_DEG
    return D.WEDGE_HIGH_X + frac * (D.WEDGE_LOW_X - D.WEDGE_HIGH_X)


def wedge(steps: int = 90) -> Manifold:
    """The latch's wedge in the +y leg's frame (axis at the origin): a sector whose face (+x) falls from 85 at its
    trailing end to 82 at its leading end: a flat sector prism, its face then moved to the wedge's height by angle."""
    flat = prism_x(sector(*D.WEDGE_R, D.WEDGE_TRAIL_DEG, D.WEDGE_LEAD_DEG, n=steps), (0.0, 1.0))

    def lift(v):
        v = np.asarray(v, float).copy()
        beta = np.degrees(np.arctan2(v[:, 2], v[:, 1]))
        top = np.array([wedge_x(b) for b in beta])
        v[:, 0] = D.WEDGE_X0 + v[:, 0] * (top - D.WEDGE_X0)
        return v

    return flat.warp_batch(lift)


def leg() -> Manifold:
    """The +y leg's printed part (knuckle, panel, wedge) in its own frame: folded, the panel pointing -y."""
    knuckle = cyl_x(D.KNUCKLE_X, 0.0, 0.0, D.KNUCKLE_R)
    panel_end = -(D.LEG_L - D.FOOT_T)
    panel = box((-D.LEG_W / 2, D.LEG_W / 2), (panel_end, 0.0), (D.LEG_Z - D.LEG_T / 2, D.LEG_Z + D.LEG_T / 2))
    body = knuckle + panel + wedge()
    body -= cyl_x((D.KNUCKLE_X[0] - 1, D.KNUCKLE_X[1] + 1), 0.0, 0.0, D.ROD_HOLE_D / 2)
    for x in (-50.0, 0.0, 50.0):  # pilot holes for the foot rail's three screws
        body -= Manifold.cylinder(14.0, 1.25, circular_segments=24).rotate([-90, 0, 0]).translate([x, panel_end, D.LEG_Z])
    return body


def bolt() -> Manifold:
    """The latch bolt in its own frame: the knife edge along z at x = 0, y = -W/2; the bevel on +y; back end at x = LEN."""
    w, h = D.BOLT_W, D.BOLT_H
    body = box((0.0, D.BOLT_LEN), (-w / 2, w / 2), (-h / 2, h / 2))
    # the bevel: cut away x < (y + W/2) * BEVEL / W, drawn in (y, x) counter-clockwise, extruded along z
    cutter = CrossSection([[(-w / 2 - 0.01, -1.0), (w / 2 + 0.01, -1.0), (w / 2 + 0.01, D.BOLT_BEVEL + 0.01),
                            (-w / 2 - 0.01, -0.01)]])
    cutter = cutter.extrude(h + 2).translate([0, 0, -h / 2 - 1])
    cutter = cutter.transform(np.array([[0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]))
    body -= cutter
    body -= Manifold.cylinder(6.5, 1.25, circular_segments=24).rotate([0, 90, 0]).translate([D.BOLT_LEN - 6.0, 0, 0])
    return body


def rib() -> Manifold:
    """A grip rib, standing on its base (z = 0), with M3 nut pockets in its base."""
    body = box((-D.RIB_W / 2, D.RIB_W / 2), (-D.RIB_T / 2, D.RIB_T / 2), (0.0, D.RIB_H))
    for sx in (-1, 1):
        body -= Manifold.cylinder(14.0, 1.7, circular_segments=24).translate([sx * D.RIB_SCREW_X, 0, -0.1])
        body -= Manifold.cylinder(2.6, 3.2, circular_segments=6).translate([sx * D.RIB_SCREW_X, 0, -0.1])
    return body


def knob() -> Manifold:
    """The release knob: a cap pressed over the M3 screw's head (5.5 mm, 3 mm high)."""
    body = Manifold.cylinder(D.KNOB_X[1] - D.KNOB_X[0], D.KNOB_D / 2, circular_segments=SEG)
    return body - Manifold.cylinder(3.2, 2.85, circular_segments=48).translate([0, 0, -0.1])


# -- placing the moving parts ---------------------------------------------------------------------------------------------
def leg_in_top(leg_part: Manifold, opening_deg: float) -> Manifold:
    """The +y leg in the top's frame, open ``opening_deg`` from folded."""
    return rot_x_about_axis(leg_part.translate([0, D.HINGE_Y, D.HINGE_Z]), opening_deg, D.HINGE_Y, D.HINGE_Z)


def bolt_in_top(bolt_part: Manifold, push_mm: float) -> Manifold:
    """The +y end's bolt in the top's frame, pushed back ``push_mm`` from rest: its knife edge at x = 81.5 + push,
    straight above the hinge axis, on the bolt's -y side (the side the +y leg's wedge leaves from)."""
    return bolt_part.translate([D.BOLT_TIP_X + push_mm, D.HINGE_Y + D.BOLT_Y, D.BOLT_AXIS_Z])


# -- the check ------------------------------------------------------------------------------------------------------------
def overlap(a: Manifold, b: Manifold) -> float:
    return (a ^ b).volume()


def needed_push(bolt_part, leg_at, tol=0.02) -> float:
    """The least push back (mm) that clears the bolt from the leg (0 if it's clear at rest)."""
    if overlap(bolt_in_top(bolt_part, 0.0), leg_at) < 1e-3:
        return 0.0
    lo, hi = 0.0, D.RELEASE_MM + 2.0
    if overlap(bolt_in_top(bolt_part, hi), leg_at) > 1e-3:
        return float("inf")
    while hi - lo > tol:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if overlap(bolt_in_top(bolt_part, mid), leg_at) > 1e-3 else (lo, mid)
    return hi


def check(top_part, leg_part, bolt_part) -> dict:
    out = {"push": [], "top_overlap": []}
    for deg in np.arange(0.0, 101.01, 0.5):
        at = leg_in_top(leg_part, float(deg))
        out["top_overlap"].append((float(deg), overlap(at, top_part)))
        out["push"].append((float(deg), needed_push(bolt_part, at)))
    out["bolt_in_channel"] = max(overlap(bolt_in_top(bolt_part, p), top_part) for p in (0.0, 2.0, D.RELEASE_MM))
    folded = leg_in_top(leg_part, 0.0) + top_part
    out["lowest_folded"] = folded.bounding_box()[2] - (D.HINGE_Z - D.KNUCKLE_R)  # 0: resting on knuckles and ears
    return out


def report(out) -> None:
    print("leg against the top: largest overlap (mm^3) by range")
    for lo, hi in ((0, 99.5), (99.5, 100.0), (100.5, 101.0)):
        worst = max(v for d, v in out["top_overlap"] if lo <= d <= hi)
        print(f"  {lo:5.1f}-{hi:5.1f} deg: {worst:8.3f}")
    print("bolt push back (mm), CAD vs the simulator's model:")
    for d, p in out["push"]:
        if d % 2 == 0 or 95 <= d <= 99:
            print(f"  {d:5.1f} deg: CAD {p:5.2f}   model {D.bolt_push_mm(d):5.2f}")
    print(f"lowest point, folded, below the knuckles' bottom: {out['lowest_folded']:.3f} mm")
    print(f"bolt against its channel, out to released (mm^3): {out['bolt_in_channel']:.3f}")
    print(f"spring: {D.SPRING_PRELOAD_N:.2f} N with the bolt out; {D.spring_force_n(D.S_AT_LATCH):.2f} N at the wedge's "
          f"high end; {D.spring_force_n(D.RELEASE_MM):.2f} N released ({D.SPRING_INSTALLED - D.RELEASE_MM:.1f} mm long, "
          f"solid {D.SPRING_SOLID})")
    print(f"fold-back torque: {D.fold_back_torque_nm(80.0):.3f} N m at 80 deg, "
          f"{D.fold_back_torque_nm(97.99):.3f} at 98")


def write_stl(part: Manifold, path: Path) -> None:
    import trimesh
    m = part.to_mesh()
    assert len(part.decompose()) == 1, f"{path}: not one solid"
    # process=False: trimesh's vertex merge would join near-coincident vertices of the wedge's fine steps
    mesh = trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3], np.asarray(m.tri_verts), process=False)
    assert mesh.is_watertight and mesh.volume > 0, path
    mesh.export(path)


def main() -> None:
    parts = {"top": top(), "leg_plus_y": leg(), "bolt": bolt(), "rib": rib(), "knob": knob()}
    parts["leg_minus_y"] = parts["leg_plus_y"].mirror([0, 1, 0])
    stl = HERE / "stl"
    stl.mkdir(exist_ok=True)
    for name, p in parts.items():
        write_stl(p, stl / f"{name}.stl")
        print(f"{name:12s} {p.volume() / 1000:7.1f} cm^3 solid")
    out = check(parts["top"], parts["leg_plus_y"], parts["bolt"])
    report(out)
    import drawing
    drawing.draw(parts, out, HERE / "drawing.png")


if __name__ == "__main__":
    main()
