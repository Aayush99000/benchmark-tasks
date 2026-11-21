"""Visual meshes of an IKEA MARKUS office chair, modelled here from IKEA's published dimensions and product photos (no third-party mesh; no CC0 or CC-BY MARKUS model exists: the only ones found are commercial or proprietary).

Everything here is for the eye. The colliders are the boxes and cylinders in task.py, and each visual below stays inside its collider's box or cylinder, except where `size_check` prints a difference.
Chair frame: origin at the centre of the base on the bench; the chair faces -y; z up. Sizes in metres.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as m

Mesh = m.Mesh

# IKEA MARKUS, US listing: width 70 cm, depth 70 cm, seat 57 x 49, seat height 47 to 58, min. height 129 cm.
SEAT_W, SEAT_D = 0.57, 0.49
SEAT_TOP = 0.47
BACK_Z0, BACK_Z1 = 0.50, 1.29
BACK_W = 0.50
BACK_Y = 0.24
BASE_R = 0.35  # half of the listed 70 cm: the casters' outer edge
CASTER_R = 0.337  # caster axis radius (the collider's is 0.31): the base's widest extent is the listed 70 cm


def solid(top: np.ndarray, bottom: np.ndarray) -> Mesh:
    """A closed solid from two (nu, nv, 3) grids. Its outward side is `cross(d top / du, d top / dv)` on the top grid; the bottom grid has the same layout."""
    nu, nv = top.shape[:2]
    idx = lambda g, i, j: g * nu * nv + i * nv + j
    v = np.vstack([top.reshape(-1, 3), bottom.reshape(-1, 3)])
    f = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b, c, d = idx(0, i, j), idx(0, i + 1, j), idx(0, i + 1, j + 1), idx(0, i, j + 1)
            f += [(a, b, c), (a, c, d)]
            a, b, c, d = idx(1, i, j), idx(1, i + 1, j), idx(1, i + 1, j + 1), idx(1, i, j + 1)
            f += [(a, c, b), (a, d, c)]
    for i in range(nu - 1):  # edges j = 0 and j = nv - 1
        a, b, c, d = idx(0, i, 0), idx(0, i + 1, 0), idx(1, i + 1, 0), idx(1, i, 0)
        f += [(a, c, b), (a, d, c)]
        a, b, c, d = idx(0, i, nv - 1), idx(0, i + 1, nv - 1), idx(1, i + 1, nv - 1), idx(1, i, nv - 1)
        f += [(a, b, c), (a, c, d)]
    for j in range(nv - 1):  # edges i = 0 and i = nu - 1
        a, b, c, d = idx(0, 0, j), idx(0, 0, j + 1), idx(1, 0, j + 1), idx(1, 0, j)
        f += [(a, b, c), (a, c, d)]
        a, b, c, d = idx(0, nu - 1, j), idx(0, nu - 1, j + 1), idx(1, nu - 1, j + 1), idx(1, nu - 1, j)
        f += [(a, c, b), (a, d, c)]
    return Mesh(v, np.array(f))


def _rounded(u: np.ndarray, v: np.ndarray, t: float) -> tuple[np.ndarray, np.ndarray]:
    """Square (u, v in -1..1) toward a disc by t: rounded corners."""
    du = u * np.sqrt(1 - v * v / 2)
    dv = v * np.sqrt(1 - u * u / 2)
    return (1 - t) * u + t * du, (1 - t) * v + t * dv


def slab(cx: float, cy: float, hx: float, hy: float, z0: float, top, n: int = 22, round_t: float = 0.35) -> Mesh:
    """A plate of half sizes hx, hy about (cx, cy) with rounded corners; flat underside at z0; top face z = top(x, y)."""
    g = np.linspace(-1, 1, n)
    uu, vv = np.meshgrid(g, g, indexing="ij")
    ru, rv = _rounded(uu, vv, round_t)
    x, y = cx + hx * ru, cy + hy * rv
    t = np.stack([x, y, top(x, y)], -1)
    b = np.stack([x, y, np.full_like(x, z0)], -1)
    return solid(t, b)  # du = +x, dv = +y: the top's normal is +z


def _edge_drop(x, y, cx, cy, hx, hy, width: float, depth: float):
    """How far the top sinks within `width` of the rim."""
    e = np.minimum(hx - np.abs(x - cx), hy - np.abs(y - cy))
    k = np.clip(1 - e / width, 0, 1)
    return depth * k * k


def seat_cushion() -> Mesh:
    """The cushion: 57 x 49 cm, a shallow bowl, the rim rolled down 8 mm over its last 15 mm. Its highest point is the 47 cm line."""
    hx, hy = SEAT_W / 2, SEAT_D / 2

    def top(x, y):
        bowl = np.exp(-((x / 0.19) ** 2 + ((y - 0.04) / 0.17) ** 2))
        return SEAT_TOP - 0.009 * bowl - _edge_drop(x, y, 0, 0, hx, hy, 0.02, 0.012)

    return slab(0, 0, hx, hy, 0.443, top, n=26, round_t=0.3)


def seat_shell() -> Mesh:
    """The plastic shell under the cushion."""
    hx, hy = SEAT_W / 2 - 0.012, SEAT_D / 2 - 0.012
    return slab(0, 0, hx, hy, 0.432, lambda x, y: np.full_like(x, 0.446), n=14, round_t=0.4)


def mechanism_housing() -> Mesh:
    """The tilt mechanism's plate under the seat: 20 x 20 cm, rounded."""
    return slab(0, 0, 0.10, 0.10, 0.392, lambda x, y: np.full_like(x, 0.43), n=10, round_t=0.6)


def column_shroud() -> Mesh:
    """The black telescoping cover that slides over the gas-lift's outer tube."""
    prof = [(0, 0.26), (0.045, 0.26), (0.047, 0.275), (0.044, 0.285), (0.044, 0.385), (0.056, 0.395), (0.06, 0.402), (0.05, 0.408), (0, 0.408)]
    return m.lathe(prof, 32)


def column_outer() -> Mesh:
    """The gas-lift's outer tube (38 mm radius) rising out of the base's cone."""
    prof = [(0, 0.06), (0.062, 0.06), (0.064, 0.075), (0.052, 0.10), (0.042, 0.13), (0.038, 0.15), (0.038, 0.305), (0, 0.305)]
    return m.lathe(prof, 32)


def _loft(path: np.ndarray, rx: np.ndarray, rz: np.ndarray, up=(0.0, 0.0, 1.0), sides: int = 14) -> Mesh:
    """A closed tube along `path`, an ellipse of half-width rx (sideways) and half-height rz (along `up`) at each point."""
    p = np.asarray(path, float)
    d = np.gradient(p, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    up = np.asarray(up, float)
    th = np.linspace(0, 2 * np.pi, sides, endpoint=False)
    rings = []
    for k in range(len(p)):
        s = np.cross(d[k], up)
        s /= np.linalg.norm(s)
        w = np.cross(s, d[k])
        rings.append(p[k] + np.outer(np.cos(th), s) * rx[k] + np.outer(np.sin(th), w) * rz[k])
    v = np.vstack(rings + [p[0][None], p[-1][None]])
    f = []
    for k in range(len(p) - 1):
        for i in range(sides):
            j = (i + 1) % sides
            a, b, c, e = k * sides + i, k * sides + j, (k + 1) * sides + j, (k + 1) * sides + i
            f += [(a, b, c), (a, c, e)]
    first, last = len(p) * sides, len(p) * sides + 1
    for i in range(sides):
        j = (i + 1) % sides
        f += [(first, j, i), (last, (len(p) - 1) * sides + i, (len(p) - 1) * sides + j)]
    mesh = Mesh(v, np.array(f))
    return mesh if volume(mesh) > 0 else Mesh(mesh.v, mesh.f[:, ::-1])


def star_leg(angle_deg: float) -> Mesh:
    """One leg of the five-star base: thick and high at the hub, tapering and lower to the caster, out to the casters."""
    a = math.radians(angle_deg)
    LEN = CASTER_R + 0.01
    r = np.linspace(0.0, LEN, 14)
    s = r / LEN
    z = 0.067 + 0.022 * (1 - s) ** 2
    path = np.stack([r * math.cos(a), r * math.sin(a), z], -1)
    rx = 0.034 - 0.016 * s
    rz = 0.015 + 0.014 * (1 - s) ** 1.5
    return _loft(path, rx, rz)


def caster(angle_deg: float) -> list[tuple[str, Mesh, str]]:
    """A twin-wheel caster at the end of a leg: stem, fork, two wheels of 25 mm radius on an axle along the tangent."""
    a = math.radians(angle_deg)
    c = np.array([CASTER_R * math.cos(a), CASTER_R * math.sin(a)])
    t = np.array([-math.sin(a), math.cos(a)])
    out = []
    for k, off in enumerate((-0.0105, 0.0105)):
        w = m.cylinder(0.025, 0.012, (0, 0, 0), axis="x", segments=24).turned(angle_deg + 90).moved((c[0] + t[0] * off, c[1] + t[1] * off, 0.025))
        out.append((f"wheel{k}", w, "wheel"))
    out.append(("stem", m.cylinder(0.009, 0.03, (c[0], c[1], 0.062)), "steel"))
    out.append(("fork", m.box((0.012, 0.045, 0.03), (0, 0, 0)).turned(angle_deg).moved((c[0], c[1], 0.045)), "frame"))
    return out


def hub() -> Mesh:
    """The base's centre, where the five legs meet."""
    prof = [(0, 0.045), (0.07, 0.045), (0.075, 0.06), (0.07, 0.095), (0.05, 0.11), (0, 0.112)]
    return m.lathe(prof, 32)


def back_centre(x, z):
    """y of the back panel's midplane: leans back 20 mm over its height, bowed 8 mm at the sides, 12 mm lumbar bulge."""
    lean = -0.008 + 0.02 * (z - BACK_Z0) / (BACK_Z1 - BACK_Z0)
    wrap = 0.008 * (x / (BACK_W / 2)) ** 2
    lumbar = -0.012 * np.exp(-(((z - 0.74) / 0.10) ** 2))
    return BACK_Y + lean + wrap + lumbar


def _half_width(z):
    return BACK_W / 2 - 0.035 * np.clip((z - 1.0) / 0.29, 0, 1) ** 2


def back_panel(z0: float, z1: float, thick: float, nu: int = 22, nv: int = 24, bulge: float = 0.0) -> Mesh:
    """A curved panel across the back, from z0 to z1. `bulge` pushes its sitter side forward, in the middle (a padded headrest)."""
    u = np.linspace(-1, 1, nu)
    z = np.linspace(z0, z1, nv)
    uu, zz = np.meshgrid(u, z, indexing="ij")
    x = uu * _half_width(zz)
    yc = back_centre(x, zz) - bulge * (1 - uu**2) * np.sin(np.pi * (zz - z0) / (z1 - z0)) ** 0.5
    front = np.stack([x, yc - thick / 2, zz], -1)
    rear = np.stack([x, yc + thick / 2, zz], -1)
    # top normal of `front` grid must point to -y (the sitter's side): d/du = +x, d/dz = +z, cross = -y
    return solid(front, rear)


def back_frame() -> Mesh:
    """The black frame round the mesh: a tube along the panel's rim."""
    pts = []
    zs = np.linspace(BACK_Z0 + 0.01, BACK_Z1 - 0.01, 14)
    for z in zs:
        pts.append((-_half_width(z), back_centre(-_half_width(z), z), z))
    for x in np.linspace(-1, 1, 8)[1:-1]:
        hw = _half_width(BACK_Z1 - 0.01)
        pts.append((x * hw, back_centre(x * hw, BACK_Z1 - 0.01), BACK_Z1 - 0.01))
    for z in zs[::-1]:
        pts.append((_half_width(z), back_centre(_half_width(z), z), z))
    for x in np.linspace(1, -1, 8)[1:-1]:
        hw = _half_width(BACK_Z0 + 0.01)
        pts.append((x * hw, back_centre(x * hw, BACK_Z0 + 0.01), BACK_Z0 + 0.01))
    pts.append(pts[0])
    return m.tube(np.array(pts), 0.0085, sides=10)


def back_spine() -> Mesh:
    """The curved black spine behind the back, from the seat's mechanism up to the back's lower third."""
    pts = np.array([(0, 0.17, 0.425), (0, 0.235, 0.44), (0, 0.285, 0.50), (0, 0.292, 0.62), (0, 0.285, 0.76), (0, 0.272, 0.88)])
    return m.tube(pts, 0.016, sides=12)


def armrest(sx: int) -> list[tuple[str, Mesh, str]]:
    """An armrest: a curved post from the seat's side up to a padded rest 7 cm wide and 26 cm long, its top 20 cm above the seat."""
    x = sx * 0.305
    tag = "R" if sx > 0 else "L"
    post = m.tube(np.array([(sx * 0.255, 0.02, 0.425), (sx * 0.28, 0.02, 0.47), (x, 0.03, 0.56), (x, 0.04, 0.655)]), 0.013, sides=12)
    pad = slab(x, 0.07, 0.034, 0.13, 0.662, lambda xx, yy: 0.688 - 0.0006 * ((yy - 0.07) / 0.13) ** 2 - _edge_drop(xx, yy, x, 0.07, 0.034, 0.13, 0.012, 0.008), n=14, round_t=0.5)
    return [(f"armpost{tag}", post, "frame"), (f"armpad{tag}", pad, "seat")]


def lever_paddle(length: float, width: float, thick: float, centre) -> Mesh:
    """The lever's paddle: a rounded bar, `length` along x, whose tip (the -x end) is rounded; inside its collider box. """
    cx, cy, cz = centre
    n = 18
    g = np.linspace(-1, 1, n)
    uu, vv = np.meshgrid(g, np.linspace(-1, 1, 8), indexing="ij")
    x = cx + uu * length / 2
    s = np.clip((-uu - 0.6) / 0.4, 0, 1)
    half = width / 2 * np.sqrt(1 - 0.92 * s * s) * (1 - 0.2 * np.maximum(uu, 0))
    y = cy + vv * half
    z_top = cz + thick / 2 - 0.0015 * vv**2
    t = np.stack([x, y, z_top], -1)
    b = np.stack([x, y, np.full_like(x, cz - thick / 2)], -1)
    return solid(t, b)


def volume(mesh: Mesh) -> float:
    v, f = mesh.v, mesh.f
    return float(np.einsum("ij,ij->i", v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum() / 6.0)


def is_closed(mesh: Mesh) -> bool:
    """Every directed edge appears once and so does its reverse: a closed, consistently turned surface."""
    e = {}
    for a, b, c in mesh.f:
        for p, q in ((a, b), (b, c), (c, a)):
            e[(p, q)] = e.get((p, q), 0) + 1
    return all(n == 1 and e.get((q, p)) == 1 for (p, q), n in e.items())


def parts() -> dict[str, Mesh]:
    d = {"cushion": seat_cushion(), "shell": seat_shell(), "housing": mechanism_housing(), "shroud": column_shroud(), "outer": column_outer(), "hub": hub(), "leg": star_leg(90),
         "back": back_panel(BACK_Z0, BACK_Z1, 0.008), "headrest": back_panel(1.13, BACK_Z1 - 0.004, 0.034, nu=18, nv=12, bulge=0.010), "backframe": back_frame(), "spine": back_spine(),
         "paddle": lever_paddle(0.12, 0.03, 0.008, (-0.255, -0.08, 0.41))}
    for n, mesh, _ in armrest(1) + caster(90):
        d[n] = mesh
    return d


def size_check() -> None:
    """Print the model's key sizes against IKEA's published ones (listing: 70 x 70 cm, seat 57 x 49, seat height 47 to 58, min. height 129)."""
    p = parts()
    lo = lambda k: p[k].v.min(0)
    hi = lambda k: p[k].v.max(0)
    legs = [star_leg(72 * i + 90) for i in range(5)] + [mm for i in range(5) for _, mm, _ in caster(72 * i + 90)]
    base = np.vstack([q.v for q in legs])
    print(f"seat width  {100 * (hi('cushion')[0] - lo('cushion')[0]):.1f} cm   (listed 57)")
    print(f"seat depth  {100 * (hi('cushion')[1] - lo('cushion')[1]):.1f} cm   (listed 49)")
    print(f"seat front edge top at its lowest {100 * hi('cushion')[2]:.1f} cm   (listed 47 to 58 cm)")
    print(f"back top, seat lowest {100 * hi('back')[2]:.1f} cm (headrest {100 * hi('headrest')[2]:.1f})   (listed min. height 129)")
    print(f"base width x {100 * (base[:, 0].max() - base[:, 0].min()):.1f}, y {100 * (base[:, 1].max() - base[:, 1].min()):.1f} cm   (listed 70 x 70; the casters' collider circle is 67 cm across)")
    print(f"armrest outer width {100 * 2 * hi('armpadR')[0]:.1f} cm; armrest top {100 * hi('armpadR')[2]:.1f} cm = {100 * (hi('armpadR')[2] - SEAT_TOP):.1f} cm above the seat")
    print(f"caster wheel diameter {100 * (hi('wheel0')[2] - lo('wheel0')[2]):.1f} cm" if "wheel0" in p else "")
    for k, v in p.items():
        print(f"  {k:10s} closed={is_closed(v)} volume={1e6 * volume(v):.0f} cm3 triangles={len(v.f)}")


if __name__ == "__main__":
    size_check()
