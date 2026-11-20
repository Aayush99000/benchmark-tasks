"""Approximate arm clearance from the YAM collision hulls (vertex clouds), for dry runs only."""

import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree
from yam_arena import YAM_URDF
from yam_arena.constants import ALL_JOINTS, FINGER_OPEN
from yam_arena.urdf import forward_kinematics, rpy_matrix


def _stl(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    rows = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return np.unique(rows["v"].reshape(-1, 3).astype(float).round(5), axis=0)


def _hulls():
    root = ET.parse(YAM_URDF).getroot()
    out = {}
    for link in root.iter("link"):
        clouds = []
        for col in link.findall("collision"):
            origin = col.find("origin")
            xyz = np.array(origin.get("xyz", "0 0 0").split(), float) if origin is not None else np.zeros(3)
            rpy = np.array(origin.get("rpy", "0 0 0").split(), float) if origin is not None else np.zeros(3)
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            v = _stl(YAM_URDF.parent / mesh.get("filename"))
            clouds.append(v @ rpy_matrix(rpy).T + xyz)
        if clouds:
            out[link.get("name")] = np.concatenate(clouds)
    return out


HULLS = _hulls()


def arm_cloud(q6, jaw, base, links=None):
    """World vertex cloud of one arm; ``jaw`` is the normalised 0 (closed) .. 1 (open) command."""
    q = dict(zip(ALL_JOINTS[:6], q6, strict=True))
    q["joint7"] = q["joint8"] = float(jaw) * FINGER_OPEN
    poses = forward_kinematics(YAM_URDF, q)
    parts = []
    for name, cloud in HULLS.items():
        if links is not None and name not in links:
            continue
        t = poses[name]
        parts.append(cloud @ t[:3, :3].T + t[:3, 3] + np.asarray(base))
    return np.concatenate(parts)


def arm_gap(left, right):
    return float(cKDTree(left).query(right)[0].min())


def box_gap(cloud, centre, half):
    """Smallest distance from the cloud to an axis-aligned box (negative inside)."""
    d = np.abs(cloud - np.asarray(centre)) - np.asarray(half)
    outside = np.linalg.norm(np.maximum(d, 0.), axis=1)
    inside = np.minimum(d.max(axis=1), 0.)
    return float((outside + inside).min())
