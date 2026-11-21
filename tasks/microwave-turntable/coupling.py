"""Dimensioned three-lobed drive cup and underside tray lug. All coordinates here are mm."""

import argparse
import math
from pathlib import Path
import struct

import numpy as np

COUNT = 60
ANGLES = np.arange(COUNT) * math.tau / COUNT
UNIT = np.column_stack((np.cos(ANGLES), np.sin(ANGLES)))
# The stock part's topology is visible on p. 7 of the Insignia manual.
# These radii and the D bore are the fixture's own manufacturing dimensions.
INNER = (15 + 5 * np.cos(3 * ANGLES))[:, None] * UNIT
OUTER = INNER + 3 * UNIT
POCKET_FLOOR, CUP_TOP = 7, 16
LUG_HEIGHT = 9


def level(xy, z):
    return np.column_stack((xy, np.full(len(xy), z)))


def skin(rings, bottom_center, top_center):
    """Close a sequence of counterclockwise contours into an oriented solid."""
    count = len(rings[0])
    points = np.vstack((bottom_center, *rings, top_center))
    first, last = 1, len(points) - count - 1
    faces = []
    for i in range(count):
        j = (i + 1) % count
        faces.append((0, first + j, first + i))
        for start in range(first, last, count):
            a, b = start + i, start + j
            faces.extend(((a, b, b + count), (a, b + count, a + count)))
        faces.append((last + i, last + j, len(points) - 1))
    return points, np.array(faces)


def cup():
    bore = 3.1 * UNIT.copy()
    forward = UNIT[:, 0] > 2.15 / 3.1
    bore[forward] = UNIT[forward] * (2.15 / UNIT[forward, 0, None])
    rings = [level(bore, 4), level(bore, -5), level(6 * UNIT, -5), level(6 * UNIT, 2),
             level(OUTER, 2), level(OUTER, CUP_TOP), level(INNER, CUP_TOP), level(INNER, POCKET_FLOOR)]
    return skin(rings, (0, 0, 4), (0, 0, POCKET_FLOOR))


def lug():
    """One 7 mm wide lug with semicircular ends centered at radii 9 and 14 mm."""
    half = np.linspace(-math.pi / 2, math.pi / 2, 13)
    far = np.column_stack((14 + 3.5 * np.cos(half), 3.5 * np.sin(half)))
    near = np.column_stack((9 - 3.5 * np.cos(half), -3.5 * np.sin(half)))
    outline = np.vstack((far, near))
    return skin([level(outline, 0), level(outline, LUG_HEIGHT)], (11.5, 0, 0), (11.5, 0, LUG_HEIGHT))


def write_stl(path, shape):
    points, faces = shape
    with path.open("wb") as stream:
        stream.write(b"Turntable fixture, millimetres".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(faces)))
        for a, b, c in points[faces]:
            normal = np.cross(b - a, c - a)
            normal /= np.linalg.norm(normal)
            stream.write(struct.pack("<12fH", *normal, *a, *b, *c, 0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Write millimetre STL files for the unheated fixture")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    write_stl(args.out / "drive-cup.stl", cup())
    write_stl(args.out / "tray-lug.stl", lug())
