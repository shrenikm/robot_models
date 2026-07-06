"""
Numpy-only mesh utilities shared by the description-build scripts: binary STL parsing, vertex welding,
OBJ export, minimum enclosing circles (for cylinder fitting), and primitive mesh generation. No
dependencies beyond numpy.
"""

from __future__ import annotations

import struct

import numpy as np


def parse_binary_stl(path: str) -> np.ndarray:
    """
    Parse a binary STL into an (N, 3, 3) float64 array of triangle vertices.
    """
    with open(path, "rb") as fh:
        fh.read(80)
        (n,) = struct.unpack("<I", fh.read(4))
        data = np.frombuffer(fh.read(n * 50), dtype=np.uint8).reshape(n, 50)
    # Each 50-byte record: 12B normal, 36B vertices (9 float32), 2B attribute.
    tris = data[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
    return tris


def weld(tris: np.ndarray, decimals: int = 7) -> tuple[np.ndarray, np.ndarray]:
    """
    Weld duplicated vertices. Returns (verts (V,3), faces (F,3) int indices).
    """
    flat = tris.reshape(-1, 3)
    keys = np.round(flat, decimals)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    # Use the first occurrence's full-precision coordinates for each unique key.
    first_idx = np.full(len(uniq), -1, dtype=np.int64)
    seen_order = np.argsort(inverse, kind="stable")
    inv_sorted = inverse[seen_order]
    starts = np.searchsorted(inv_sorted, np.arange(len(uniq)))
    first_idx = seen_order[starts]
    verts = flat[first_idx]
    faces = inverse.reshape(-1, 3)
    # Drop degenerate faces (two or more identical vertex indices).
    good = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return verts, faces[good]


def write_obj(path: str, verts: np.ndarray, faces: np.ndarray, header: str = "") -> None:
    """
    Write a plain v/f OBJ (no normals/texcoords), matching the lite6 visual mesh convention.
    """
    lines = []
    if header:
        for h in header.strip().split("\n"):
            lines.append(f"# {h}")
    lines.append("o obj1")
    for v in verts:
        lines.append(f"v {v[0]:.8g} {v[1]:.8g} {v[2]:.8g}")
    for f in faces:
        lines.append(f"f {f[0] + 1} {f[1] + 1} {f[2] + 1}")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def mesh_stats(tris: np.ndarray) -> str:
    flat = tris.reshape(-1, 3)
    lo = flat.min(axis=0)
    hi = flat.max(axis=0)
    return (
        f"tris={len(tris):7d}  "
        f"x[{lo[0]:+.4f},{hi[0]:+.4f}] y[{lo[1]:+.4f},{hi[1]:+.4f}] z[{lo[2]:+.4f},{hi[2]:+.4f}]  "
        f"dims=({hi[0] - lo[0]:.4f},{hi[1] - lo[1]:.4f},{hi[2] - lo[2]:.4f})"
    )


def min_enclosing_circle(points_2d: np.ndarray, grid: float = 3.0e-4) -> tuple[np.ndarray, float]:
    """
    Minimum enclosing circle via Welzl's algorithm on the 2D convex hull of grid-deduplicated
    points. Deduplication moves each point by at most grid * sqrt(2) / 2, which is added back to
    the radius so the returned circle is guaranteed to enclose every original point.
    """
    if grid > 0.0:
        pts_in = np.unique(np.round(points_2d / grid), axis=0) * grid
        margin = grid * np.sqrt(2.0) / 2.0
    else:
        pts_in = points_2d
        margin = 0.0
    hull = _convex_hull_2d(pts_in)
    rng = np.random.default_rng(0)
    pts = hull[rng.permutation(len(hull))]
    center, r2 = _welzl_iterative(pts)
    return center, float(np.sqrt(r2)) + margin


def _akl_toussaint_filter(pts: np.ndarray) -> np.ndarray:
    """
    Discard points strictly inside the octagon spanned by the extreme points in 8 directions;
    the survivors contain the full convex hull. Fully vectorized.
    """
    if len(pts) <= 16:
        return pts
    dirs = np.array(
        [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, 1], [-1, -1], [1, -1]], dtype=np.float64
    )
    extremes = pts[np.argmax(pts @ dirs.T, axis=0)]
    # Order the (deduped) extreme points counterclockwise around their centroid.
    uniq = np.unique(np.round(extremes, 12), axis=0)
    if len(uniq) < 3:
        return pts
    centroid = uniq.mean(axis=0)
    order = np.argsort(np.arctan2(uniq[:, 1] - centroid[1], uniq[:, 0] - centroid[0]))
    poly = uniq[order]
    keep = np.zeros(len(pts), dtype=bool)
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        edge = b - a
        # Points on the outside half-plane (or on the edge) of any polygon edge are kept.
        cross = edge[0] * (pts[:, 1] - a[1]) - edge[1] * (pts[:, 0] - a[0])
        keep |= cross <= 1e-15
    return pts[keep]


def _convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """
    Andrew's monotone chain convex hull; returns hull points (H, 2).
    """
    pts = np.unique(np.round(points, 9), axis=0)
    if len(pts) <= 2:
        return pts
    pts = _akl_toussaint_filter(pts)
    if len(pts) <= 2:
        return pts
    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pt_list = [(float(x), float(y)) for x, y in pts[order]]

    def half(seq):
        out = []
        for px, py in seq:
            while len(out) >= 2:
                ax, ay = out[-2]
                bx, by = out[-1]
                if (bx - ax) * (py - ay) - (by - ay) * (px - ax) <= 0.0:
                    out.pop()
                else:
                    break
            out.append((px, py))
        return out

    lower = half(pt_list)
    upper = half(pt_list[::-1])
    return np.array(lower[:-1] + upper[:-1])


def _circle_from(points: list[np.ndarray]) -> tuple[np.ndarray, float]:
    """
    Smallest circle through 0-3 boundary points; returns (center, radius_squared).
    """
    if len(points) == 0:
        return np.zeros(2), 0.0
    if len(points) == 1:
        return points[0].copy(), 0.0
    if len(points) == 2:
        c = (points[0] + points[1]) / 2.0
        r2 = float(np.sum((points[0] - c) ** 2))
        return c, r2
    (ax, ay), (bx, by), (cx, cy) = points
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-18:
        # Collinear: fall back to the widest pair.
        best = (np.zeros(2), -1.0)
        for i in range(3):
            for j in range(i + 1, 3):
                c, r2 = _circle_from([points[i], points[j]])
                if r2 > best[1]:
                    best = (c, r2)
        return best
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    c = np.array([ux, uy])
    r2 = float(np.sum((points[0] - c) ** 2))
    return c, r2


def _welzl_iterative(points: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Move-to-front Welzl on a small point set (hull points), iterative restart formulation.
    """
    center, r2 = _circle_from([])
    boundary: list[np.ndarray] = []

    def mec(pts: np.ndarray, boundary: list[np.ndarray]) -> tuple[np.ndarray, float]:
        center, r2 = _circle_from(boundary)
        if len(boundary) == 3:
            return center, r2
        for i, p in enumerate(pts):
            if np.sum((p - center) ** 2) > r2 * (1 + 1e-12) + 1e-24:
                center, r2 = mec(pts[:i], boundary + [p])
        return center, r2

    import sys

    old = sys.getrecursionlimit()
    sys.setrecursionlimit(10000)
    try:
        center, r2 = mec(points, [])
    finally:
        sys.setrecursionlimit(old)
    return center, r2


def make_cylinder_obj(
    center_a: np.ndarray, center_b: np.ndarray, radius: float, segments: int = 32
) -> tuple[np.ndarray, np.ndarray]:
    """
    Closed convex cylinder mesh between two cap centers. Returns (verts, faces).
    """
    axis = center_b - center_a
    length = np.linalg.norm(axis)
    if length < 1e-12:
        raise ValueError("Degenerate cylinder")
    z = axis / length
    # Any perpendicular.
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(ref, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    theta = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    ring = np.outer(np.cos(theta), x) + np.outer(np.sin(theta), y)
    bottom = center_a + radius * ring
    top = center_b + radius * ring
    verts = np.vstack([bottom, top])
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        faces.append([i, j, segments + i])
        faces.append([j, segments + j, segments + i])
    # Caps as fans.
    for i in range(1, segments - 1):
        faces.append([0, i + 1, i])  # bottom (faces -z)
        faces.append([segments, segments + i, segments + i + 1])  # top
    return verts, np.array(faces, dtype=np.int64)


def make_box_obj(lo: np.ndarray, hi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Axis-aligned box mesh from min/max corners. Returns (verts, faces).
    """
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    verts = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ]
    )
    faces = np.array(
        [
            [0, 2, 1],
            [0, 3, 2],  # bottom
            [4, 5, 6],
            [4, 6, 7],  # top
            [0, 1, 5],
            [0, 5, 4],  # -y
            [2, 3, 7],
            [2, 7, 6],  # +y
            [1, 2, 6],
            [1, 6, 5],  # +x
            [3, 0, 4],
            [3, 4, 7],  # -x
        ],
        dtype=np.int64,
    )
    return verts, faces


def points_in_cylinder(pts: np.ndarray, center_a: np.ndarray, center_b: np.ndarray, radius: float) -> np.ndarray:
    """
    Boolean mask of points inside the (closed) cylinder.
    """
    axis = center_b - center_a
    length = np.linalg.norm(axis)
    z = axis / length
    rel = pts - center_a
    t = rel @ z
    radial = rel - np.outer(t, z)
    r = np.linalg.norm(radial, axis=1)
    return (t >= -1e-9) & (t <= length + 1e-9) & (r <= radius + 1e-9)


def points_in_box(pts: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.all((pts >= lo - 1e-9) & (pts <= hi + 1e-9), axis=1)
