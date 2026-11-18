"""Pure numpy checks for a fixed-label surface through an axis-aligned outlet.

Positions, stub_transform and robot boxes are in simulation world coordinates.
Outlet coordinates are environment-local; env_origin is subtracted exactly once.
Callers own the immutable rest-mesh labels and solver-node mapping. This module
neither selects labels from current geometry nor moves simulation nodes.
"""

from dataclasses import asdict, dataclass
import hashlib
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class Outlet:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z: float


@dataclass(frozen=True)
class CollisionBox:
    """A real collision piece: world centre, positive half extents, xyzw rotation."""

    name: str
    center: Sequence[float]
    half_extents: Sequence[float]
    quat_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class AccessReport:
    valid_input: bool
    topology_ok: bool
    root_attached: bool
    geometry_accessible: bool
    hands_clear: bool
    settled: bool
    accessible: bool
    reasons: tuple[str, ...]
    diagnostics: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def rotation_matrix_xyzw(quaternion: Sequence[float]) -> np.ndarray:
    """Normalize a finite nonzero xyzw quaternion and return its active rotation."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("quat_xyzw must be four finite numbers with nonzero norm")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array((
        (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
        (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
        (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)),
    ))


def topology_hash(node_count: int, triangles: np.ndarray) -> str:
    """Pin exact solver connectivity/order; caller must also pin its label mapping."""
    faces = np.asarray(triangles, dtype="<i8")
    header = np.asarray((node_count, *faces.shape), dtype="<i8")
    return hashlib.sha256(header.tobytes() + faces.tobytes()).hexdigest()


def _indices(values, count: int, name: str, minimum: int = 1) -> np.ndarray:
    raw = np.asarray(values)
    if raw.ndim != 1 or len(raw) < minimum or not np.issubdtype(raw.dtype, np.integer):
        raise ValueError(f"{name} must contain at least {minimum} integer node IDs")
    out = raw.astype(np.int64)
    if np.any(out < 0) or np.any(out >= count) or len(np.unique(out)) != len(out):
        raise ValueError(f"{name} contains duplicate or out-of-range IDs")
    return out


def _component_count(node_ids: np.ndarray, faces: np.ndarray) -> int:
    adjacency = {int(i): set() for i in node_ids}
    for a, b, c in faces:
        for x, y in ((a, b), (b, c), (c, a)):
            adjacency[int(x)].add(int(y))
            adjacency[int(y)].add(int(x))
    remaining, components = set(adjacency), 0
    while remaining:
        components += 1
        todo = [remaining.pop()]
        while todo:
            for other in adjacency[todo.pop()]:
                if other in remaining:
                    remaining.remove(other)
                    todo.append(other)
    return components


def _diameter(points: np.ndarray) -> float:
    return float(np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2).max())


def _plane_segments(nodes: np.ndarray, faces: np.ndarray, z: float, epsilon: float) -> np.ndarray:
    """Actual triangle/plane intersection segments, including edges on the plane.

    Entirely coplanar faces and point touches do not establish passage through an
    opening. Shared segments are counted once.
    """
    segments = {}
    for face in faces:
        tri = nodes[face]
        d = tri[:, 2] - z
        signs = np.where(d > epsilon, 1, np.where(d < -epsilon, -1, 0))
        if np.all(signs == 0) or np.all(signs > 0) or np.all(signs < 0):
            continue
        points = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            if signs[a] == 0:
                points.append(tri[a])
            if signs[a] * signs[b] < 0:
                points.append(tri[a] + (z - tri[a, 2]) / (tri[b, 2] - tri[a, 2]) * (tri[b] - tri[a]))
        unique = []
        for p in points:
            if not any(np.linalg.norm(p - other) <= epsilon for other in unique):
                unique.append(p)
        if len(unique) < 2:
            continue
        p = np.asarray(unique)
        distance = np.linalg.norm(p[:, None] - p[None, :], axis=2)
        i, j = np.unravel_index(distance.argmax(), distance.shape)
        if distance[i, j] <= epsilon:
            continue
        segment = np.stack((p[i], p[j]))
        key = tuple(sorted(tuple(np.round(v, 10)) for v in segment))
        segments[key] = segment
    return np.asarray(list(segments.values()), dtype=float).reshape(-1, 2, 3)


def _exposed_subpatch(nodes, all_faces, patch_faces, free_ids, outlet,
                     minimum_drop, minimum_width, minimum_area, epsilon):
    """Clip material triangles and join only along surviving material edges.

    Coincident world positions do not join unrelated material. Two surviving
    triangle polygons join only if they share an original material edge whose
    clipped length is positive. Each patch must contain one contiguous exposed
    material-boundary free-edge run of sufficient chord span; disconnected
    islands or separated short runs cannot pool area or width.
    """
    planes = ((0, outlet.x_min, 1), (0, outlet.x_max, -1),
              (1, outlet.y_min, 1), (1, outlet.y_max, -1),
              (2, outlet.z, -1))

    def clip_polygon(poly):
        poly = [p.copy() for p in poly]
        for axis, bound, sign in planes:
            result = []
            if not poly:
                return np.empty((0, 3))
            for a, b in zip(poly, poly[1:] + poly[:1]):
                da, db = sign * (a[axis] - bound), sign * (b[axis] - bound)
                ina, inb = da >= -epsilon, db >= -epsilon
                if ina:
                    result.append(a)
                if ina != inb:
                    # Clamp only to counter epsilon-tolerant endpoint tests.
                    t = float(np.clip(da / (da - db), 0., 1.))
                    result.append(a + t * (b-a))
            poly = result
        return np.asarray(poly).reshape(-1, 3)

    def segment(a, b, exposed_edge=False):
        lo, hi = 0., 1.
        for axis, bound, sign in planes:
            if exposed_edge and axis == 2:
                bound -= minimum_drop
            start = sign * (a[axis] - bound)
            delta = sign * (b[axis] - a[axis])
            if abs(delta) <= epsilon:
                if start < -epsilon:
                    return None
                continue
            crossing = -start / delta
            if delta > 0:
                lo = max(lo, crossing)
            else:
                hi = min(hi, crossing)
            if hi < lo:
                return None
        lo, hi = max(0., lo), min(1., hi)
        if hi < lo or np.linalg.norm((hi-lo)*(b-a)) <= epsilon:
            return None
        return a+lo*(b-a), a+hi*(b-a), lo, hi

    edge_incidence = {}
    for face in all_faces:
        for a, b in zip(face, np.roll(face, -1)):
            edge = tuple(sorted((int(a), int(b))))
            edge_incidence[edge] = edge_incidence.get(edge, 0) + 1
    free = set(map(int, free_ids))
    surviving, areas, face_edges = {}, {}, {}
    for i, face in enumerate(patch_faces):
        poly = clip_polygon(nodes[face])
        if len(poly) < 3:
            continue
        area = float(sum(np.linalg.norm(np.cross(poly[k]-poly[0], poly[k+1]-poly[0]))
                         for k in range(1, len(poly)-1)) / 2)
        if area <= epsilon**2:
            continue
        surviving[i], areas[i] = poly, area
        for a, b in zip(face, np.roll(face, -1)):
            edge = tuple(sorted((int(a), int(b))))
            face_edges.setdefault(edge, []).append(i)
    adjacency = {i: set() for i in surviving}
    clipped_edges = {}
    for edge, incident in face_edges.items():
        clipped = segment(nodes[edge[0]], nodes[edge[1]])
        if clipped is None:
            continue
        clipped_edges[edge] = clipped
        for a in incident:
            adjacency[a].update(b for b in incident if b != a)
    remaining, components = set(surviving), []
    while remaining:
        todo = [remaining.pop()]
        component = set(todo)
        while todo:
            for other in adjacency[todo.pop()] - component:
                component.add(other)
                remaining.discard(other)
                todo.append(other)
        components.append(component)
    witnesses = []
    for component in components:
        # Build a free-edge segment graph using original material endpoint IDs.
        # A clipped point inside an edge is unique to that material edge.
        segments, graph = [], {}
        for edge, clipped in clipped_edges.items():
            if edge_incidence.get(edge) != 1 or not set(edge).issubset(free):
                continue
            if not component.intersection(face_edges[edge]):
                continue
            clipped = segment(nodes[edge[0]], nodes[edge[1]], exposed_edge=True)
            if clipped is None:
                continue
            a, b, lo, hi = clipped
            ka = ('vertex', edge[0]) if lo <= 1e-10 else ('cut', *edge, 0)
            kb = ('vertex', edge[1]) if hi >= 1-1e-10 else ('cut', *edge, 1)
            graph.setdefault(ka, []).append((kb, b))
            graph.setdefault(kb, []).append((ka, a))
            segments.append((ka, kb, a, b))
        keys = set(graph)
        spans, runs = [], []
        while keys:
            todo = [keys.pop()]
            run = set(todo)
            while todo:
                for other, _ in graph[todo.pop()]:
                    if other not in run:
                        run.add(other)
                        keys.discard(other)
                        todo.append(other)
            points = np.asarray([p for ka,kb,a,b in segments if ka in run for p in (a,b)])
            span = _diameter(points) if len(points) else 0.
            spans.append(span)
            runs.append(dict(span_m=span, segment_count=sum(ka in run for ka,kb,a,b in segments),
                             endpoint_low=points.min(0).tolist(), endpoint_high=points.max(0).tolist()))
        area = float(sum(areas[i] for i in component))
        span = max(spans, default=0.)
        witnesses.append(dict(area_m2=area, triangle_polygon_count=len(component),
            longest_contiguous_freeedge_span_m=span, contiguous_freeedge_runs=runs,
            qualifying=bool(area + epsilon**2 >= minimum_area and span + epsilon >= minimum_width)))
    return dict(component_count=len(witnesses), total_clipped_area_m2=float(sum(areas.values())),
        clipped_triangle_polygon_count=len(surviving), components=witnesses,
        qualifying_components=[w for w in witnesses if w['qualifying']],
        maximum_component_area_m2=max((w['area_m2'] for w in witnesses), default=0.),
        maximum_contiguous_freeedge_span_m=max((w['longest_contiguous_freeedge_span_m'] for w in witnesses), default=0.))


def evaluate_surface_access(
    *,
    nodes,
    velocities,
    triangles,
    root_ids,
    free_edge_ids,
    terminal_patch_ids,
    stub_transform,
    root_local_anchors,
    outlet: Outlet,
    robot_boxes: Sequence[CollisionBox] = (),
    env_origin=(0.0, 0.0, 0.0),
    expected_topology_hash: str | None = None,
    root_tolerance: float = 0.002,
    minimum_drop: float = 0.020,
    minimum_width: float = 0.020,
    minimum_patch_area: float = 0.0002,
    maximum_node_speed: float = 0.010,
    hands_clearance: float = 0.020,
    minimum_z: float | None = None,
    epsilon: float = 1e-7,
) -> AccessReport:
    """Evaluate a known connected paper surface; all distances are metres.

    ``geometry_accessible`` includes topology/root validity, one sufficiently
    large exposed connected terminal subpatch with a contiguous free edge,
    and passage through the correct rectangle. ``accessible``
    additionally requires hands clear and every node still. Callers can use the
    separate fields for staged partial credit. Thresholds are task criteria,
    not measured tissue material properties. No one-second history is kept here.

    Hand clearance conservatively overlaps each actual surface triangle's box
    in each robot collision box's local frame, expanded by hands_clearance.
    This may reject near passes; it cannot prove exact triangle/mesh contact.
    """
    try:
        p = np.asarray(nodes, dtype=float)
        v = np.asarray(velocities, dtype=float)
        raw_faces = np.asarray(triangles)
        origin = np.asarray(env_origin, dtype=float)
        transform = np.asarray(stub_transform, dtype=float)
        if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or v.shape != p.shape:
            raise ValueError("nodes and velocities must have the same (N,3) shape")
        if not np.isfinite(p).all() or not np.isfinite(v).all():
            raise ValueError("nodes and velocities must be finite")
        if (raw_faces.ndim != 2 or raw_faces.shape[1] != 3 or not len(raw_faces)
                or not np.issubdtype(raw_faces.dtype, np.integer)):
            raise ValueError("triangles must be a nonempty (F,3) integer array")
        faces = raw_faces.astype(np.int64)
        if np.any(faces < 0) or np.any(faces >= len(p)) or np.any(np.diff(np.sort(faces, axis=1), axis=1) == 0):
            raise ValueError("triangle IDs must be in range and distinct in each face")
        if len(np.unique(np.sort(faces, axis=1), axis=0)) != len(faces):
            raise ValueError("duplicate triangles cannot count as additional material area")
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError("env_origin must be three finite numbers")
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError("stub_transform must be finite (4,4)")
        if (not np.allclose(transform[3], (0, 0, 0, 1), atol=1e-8)
                or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-6)):
            raise ValueError("stub_transform must be a rigid transform")
        roots = _indices(root_ids, len(p), "root_ids", 2)
        free = _indices(free_edge_ids, len(p), "free_edge_ids", 2)
        patch = _indices(terminal_patch_ids, len(p), "terminal_patch_ids", 3)
        if set(roots) & set(patch) or not set(free).issubset(set(patch)):
            raise ValueError("terminal patch must contain the free edge and exclude the root")
        anchors = np.asarray(root_local_anchors, dtype=float)
        if anchors.shape != (len(roots), 3) or not np.isfinite(anchors).all():
            raise ValueError("root_local_anchors must match root_ids order and have shape (R,3)")
        o = np.array((outlet.x_min, outlet.x_max, outlet.y_min, outlet.y_max, outlet.z), dtype=float)
        thresholds = np.array((root_tolerance, minimum_drop, minimum_width, minimum_patch_area,
                               maximum_node_speed, hands_clearance, epsilon))
        if (not np.isfinite(o).all() or o[0] >= o[1] or o[2] >= o[3]
                or not np.isfinite(thresholds).all() or np.any(thresholds < 0) or epsilon <= 0):
            raise ValueError("outlet bounds and nonnegative thresholds must be finite and valid")
        if minimum_z is not None and not np.isfinite(minimum_z):
            raise ValueError("minimum_z must be finite")
        parsed_boxes = []
        for box in robot_boxes:
            center, half = np.asarray(box.center, float), np.asarray(box.half_extents, float)
            if (center.shape != (3,) or half.shape != (3,) or not np.isfinite(center).all()
                    or not np.isfinite(half).all() or np.any(half <= 0)):
                raise ValueError("robot collision boxes need finite centres and positive half extents")
            parsed_boxes.append((box.name, center - origin, half, rotation_matrix_xyzw(box.quat_xyzw)))
    except (ValueError, TypeError, AttributeError) as error:
        return AccessReport(False, False, False, False, False, False, False,
                            ("invalid_input",), {"error": str(error)})

    p = p - origin
    reasons = []
    digest = topology_hash(len(p), faces)
    components = _component_count(np.arange(len(p)), faces)
    referenced = len(np.unique(faces)) == len(p)
    if components != 1:
        reasons.append("mesh_disconnected")
    if not referenced:
        reasons.append("mesh_unreferenced_nodes")
    hash_ok = expected_topology_hash is None or digest == expected_topology_hash
    if not hash_ok:
        reasons.append("topology_hash_changed")
    topology_ok = components == 1 and referenced and hash_ok

    expected_roots = anchors @ transform[:3, :3].T + transform[:3, 3] - origin
    root_error = float(np.linalg.norm(p[roots] - expected_roots, axis=1).max())
    root_attached = root_error <= root_tolerance
    if not root_attached:
        reasons.append("root_attachment_error")

    patch_faces = faces[np.isin(faces, patch).all(axis=1)]
    patch_connected = (len(patch_faces) > 0 and len(np.unique(patch_faces)) == len(patch)
                       and _component_count(patch, patch_faces) == 1)
    if not patch_connected:
        reasons.append("terminal_patch_disconnected")
    points = p[patch_faces]
    patch_area = float(np.linalg.norm(np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0]), axis=1).sum() / 2)
    width = _diameter(p[free])
    terminal = p[patch]
    exposed = _exposed_subpatch(p, faces, patch_faces, free, outlet,
                               minimum_drop, minimum_width, minimum_patch_area, epsilon)
    if not exposed['qualifying_components']:
        if exposed['maximum_contiguous_freeedge_span_m'] + epsilon < minimum_width:
            reasons.append("free_edge_too_narrow")
        if exposed['maximum_component_area_m2'] + epsilon**2 < minimum_patch_area:
            reasons.append("terminal_patch_too_small")
        reasons.append("no_qualifying_exposed_terminal_subpatch")
    if minimum_z is not None and terminal[:, 2].min() < minimum_z - epsilon:
        reasons.append("terminal_patch_below_floor")
    if p[roots, 2].min() <= outlet.z + epsilon:
        reasons.append("root_not_above_outlet")

    segments = _plane_segments(p, faces, outlet.z, epsilon)
    cross_length, crossing_width = 0.0, 0.0
    outside = 0
    if not len(segments):
        reasons.append("no_aperture_crossing")
    else:
        endpoints = segments.reshape(-1, 3)
        inside = ((endpoints[:, 0] >= outlet.x_min - epsilon) & (endpoints[:, 0] <= outlet.x_max + epsilon)
                  & (endpoints[:, 1] >= outlet.y_min - epsilon) & (endpoints[:, 1] <= outlet.y_max + epsilon))
        outside = int((~inside).sum())
        if outside:
            reasons.append("crossing_outside_outlet")
        cross_length = float(np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1).sum())
        crossing_width = _diameter(endpoints)
        if crossing_width + epsilon < minimum_width:
            reasons.append("aperture_crossing_too_narrow")

    geometry_reasons = tuple(reasons)
    geometry_accessible = topology_ok and root_attached and not geometry_reasons
    overlapping_boxes = []
    if parsed_boxes:
        # Project the whole surface's world AABB into each collision box's local axes.
        # This contains every triangle's original local AABB, so an axis-separated
        # box cannot pass the original predicate. World-AABB separation alone would
        # remove some of that predicate's conservative positives for rotated boxes.
        paper_low, paper_high = p.min(axis=0), p.max(axis=0)
        paper_center, paper_half = (paper_low + paper_high) / 2, (paper_high - paper_low) / 2
        centers = np.asarray([box[1] for box in parsed_boxes])
        halves = np.asarray([box[2] for box in parsed_boxes])
        rotations = np.asarray([box[3] for box in parsed_boxes])
        local_centers = np.einsum("bi,bij->bj", paper_center - centers, rotations)
        local_radii = np.einsum("i,bij->bj", paper_half, np.abs(rotations))
        # A numerical guard only widens this preliminary candidate set. The final
        # triangle predicate below retains its original thresholds unchanged.
        guard = 64 * np.finfo(float).eps * (1 + np.abs(p).max() + np.abs(centers).max()
                                            + paper_half.max() + halves.max())
        candidates = np.all((local_centers + local_radii >= -halves - hands_clearance - guard)
                            & (local_centers - local_radii <= halves + hands_clearance + guard), axis=1)
        for index in np.flatnonzero(candidates):
            name, center, half, rotation = parsed_boxes[index]
            local_triangles = ((p - center) @ rotation)[faces]
            low, high = local_triangles.min(axis=1), local_triangles.max(axis=1)
            overlap = np.all((high >= -half - hands_clearance) & (low <= half + hands_clearance), axis=1)
            if overlap.any():
                overlapping_boxes.append(str(name))
    hands_clear = not overlapping_boxes
    if not hands_clear:
        reasons.append("robot_overlap")
    speed = float(np.linalg.norm(v, axis=1).max())
    settled = speed <= maximum_node_speed
    if not settled:
        reasons.append("nodes_moving")

    diagnostics = dict(
        topology_hash=digest,
        node_count=len(p), triangle_count=len(faces), component_count=components,
        root_error_max_m=root_error,
        terminal_patch_area_m2=patch_area, terminal_patch_triangle_count=len(patch_faces),
        free_edge_width_m=width,
        free_edge_low=p[free].min(axis=0).tolist(), free_edge_high=p[free].max(axis=0).tolist(),
        terminal_patch_low=terminal.min(axis=0).tolist(), terminal_patch_high=terminal.max(axis=0).tolist(),
        aperture_crossing_count=len(segments), aperture_crossing_length_m=cross_length,
        aperture_crossing_width_m=crossing_width, outside_crossing_endpoint_count=outside,
        max_node_speed_m_s=speed, overlapping_robot_boxes=overlapping_boxes,
        exposed_terminal_subpatch=exposed,
        env_origin=origin.tolist(),
    )
    return AccessReport(True, topology_ok, root_attached, geometry_accessible, hands_clear, settled,
                        geometry_accessible and hands_clear and settled, tuple(reasons), diagnostics)
