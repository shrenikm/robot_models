#!/usr/bin/env python3
"""
Fit tight convex collision primitives (boxes / cylinders) to each visual mesh of a robot description.

Per link: a dynamic program chooses split planes along a slab axis; each region is covered by the
minimum-volume primitive among a tight AABB box and minimum-enclosing cylinders along x/y/z. The objective
is total primitive volume (plus a small per-primitive penalty against over-segmentation), which directly
encodes "capture the geometry completely, minimize extra space". The best slab axis of x/y/z wins.

CRITICAL correctness detail: the fit runs on area-weighted SURFACE SAMPLES, not just mesh vertices.
Vertices concentrate at feature edges; fitting on vertices alone lets the DP cheat with thin boxes through
vertex-dense rings while leaving smooth walls uncovered (observed: 3.5 percent of the surface leaking by
2.5 mm). Surface samples make that impossible.

Outputs per link:
  - convex primitive OBJs written to <desc_dir>/meshes/<group>/collision/<link>/<link>_<box|cylinder>N.obj
  - an overlay PNG (mesh points + fitted primitive outlines in 3 orthographic views) under --report-dir
  - coverage stats printed (vertex + surface-sample coverage must be 100 percent, max leak ~0 mm)
  - manifest.json under --report-dir mapping links to primitive specs and mesh-relative OBJ paths

Runtime is a few minutes per link (single core); run links in parallel shells for big robots:
    for m in link1 link2 ...; do fit_collision_primitives.py --desc-dir D --mesh $m & done

Usage:
    fit_collision_primitives.py --desc-dir <repo>/<robot>_description --report-dir /tmp/fits [--mesh link1]
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_tools import (
    make_box_obj,
    make_cylinder_obj,
    min_enclosing_circle,
    parse_binary_stl,
    points_in_box,
    points_in_cylinder,
    weld,
    write_obj,
)

# Adding a primitive must reduce total volume by at least this fraction to be worth it.
K_PENALTY = 0.06
MAX_REGIONS = 5
N_CANDIDATES = 28
SEAM_EPS = 2.0e-4
# Area-weighted surface samples added to the fitting point set (see module docstring).
N_FIT_SAMPLES = 40000
# Uniform safety inflation applied to final primitives so seam-crossing triangles stay enclosed.
INFLATE = 2.0e-4


class Box:
    def __init__(self, lo: np.ndarray, hi: np.ndarray):
        self.lo = np.asarray(lo, dtype=np.float64)
        self.hi = np.asarray(hi, dtype=np.float64)

    @property
    def volume(self) -> float:
        return float(np.prod(self.hi - self.lo))

    def inflate(self, eps: float) -> "Box":
        return Box(self.lo - eps, self.hi + eps)

    def contains(self, pts: np.ndarray) -> np.ndarray:
        return points_in_box(pts, self.lo, self.hi)

    def mesh(self):
        return make_box_obj(self.lo, self.hi)

    def spec(self) -> dict:
        return {"type": "box", "lo": self.lo.tolist(), "hi": self.hi.tolist()}

    def draw_2d(self, ax, i, j) -> None:
        ax.add_patch(
            mpatches.Rectangle(
                (self.lo[i], self.lo[j]),
                self.hi[i] - self.lo[i],
                self.hi[j] - self.lo[j],
                fill=False,
                edgecolor="red",
                linewidth=1.2,
            )
        )

    def __repr__(self) -> str:
        c = (self.lo + self.hi) / 2
        d = self.hi - self.lo
        return f"Box(center=({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}), dims=({d[0]:.4f},{d[1]:.4f},{d[2]:.4f}))"


class Cylinder:
    def __init__(self, axis: int, center2: np.ndarray, radius: float, a_lo: float, a_hi: float):
        self.axis = axis
        self.center2 = np.asarray(center2, dtype=np.float64)
        self.radius = float(radius)
        self.a_lo = float(a_lo)
        self.a_hi = float(a_hi)

    @property
    def other(self) -> list[int]:
        return [i for i in range(3) if i != self.axis]

    @property
    def volume(self) -> float:
        return float(np.pi * self.radius**2 * (self.a_hi - self.a_lo))

    def inflate(self, eps: float) -> "Cylinder":
        return Cylinder(self.axis, self.center2, self.radius + eps, self.a_lo - eps, self.a_hi + eps)

    def _caps(self) -> tuple[np.ndarray, np.ndarray]:
        a = np.zeros(3)
        b = np.zeros(3)
        a[self.axis] = self.a_lo
        b[self.axis] = self.a_hi
        a[self.other[0]] = b[self.other[0]] = self.center2[0]
        a[self.other[1]] = b[self.other[1]] = self.center2[1]
        return a, b

    def contains(self, pts: np.ndarray) -> np.ndarray:
        a, b = self._caps()
        return points_in_cylinder(pts, a, b, self.radius)

    def mesh(self):
        a, b = self._caps()
        return make_cylinder_obj(a, b, self.radius, segments=32)

    def spec(self) -> dict:
        a, b = self._caps()
        return {"type": "cylinder", "axis": "xyz"[self.axis], "cap_a": a.tolist(), "cap_b": b.tolist(), "radius": self.radius}

    def draw_2d(self, ax, i, j) -> None:
        if i != self.axis and j != self.axis:
            ci = self.center2[self.other.index(i)]
            cj = self.center2[self.other.index(j)]
            ax.add_patch(mpatches.Circle((ci, cj), self.radius, fill=False, edgecolor="blue", linewidth=1.2))
        elif i == self.axis:
            cj = self.center2[self.other.index(j)]
            ax.add_patch(
                mpatches.Rectangle(
                    (self.a_lo, cj - self.radius), self.a_hi - self.a_lo, 2 * self.radius,
                    fill=False, edgecolor="blue", linewidth=1.2,
                )
            )
        else:
            ci = self.center2[self.other.index(i)]
            ax.add_patch(
                mpatches.Rectangle(
                    (ci - self.radius, self.a_lo), 2 * self.radius, self.a_hi - self.a_lo,
                    fill=False, edgecolor="blue", linewidth=1.2,
                )
            )

    def __repr__(self) -> str:
        return (
            f"Cyl(axis={'xyz'[self.axis]}, c2=({self.center2[0]:+.4f},{self.center2[1]:+.4f}), "
            f"r={self.radius:.4f}, span[{self.a_lo:+.4f},{self.a_hi:+.4f}])"
        )


def fit_region(verts: np.ndarray) -> tuple[object, float]:
    lo = verts.min(axis=0)
    hi = verts.max(axis=0)
    best: object = Box(lo, hi)
    best_vol = best.volume
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        c2, r = min_enclosing_circle(verts[:, other])
        cyl = Cylinder(axis, c2, r, lo[axis], hi[axis])
        if cyl.volume < best_vol:
            best = cyl
            best_vol = cyl.volume
    return best, best_vol


def candidate_planes(coords: np.ndarray, n: int) -> np.ndarray:
    qs = np.quantile(coords, np.linspace(0.02, 0.98, n))
    s = np.unique(np.round(np.sort(coords), 6))
    if len(s) > 1:
        gaps = np.diff(s)
        top = np.argsort(gaps)[-8:]
        mids = (s[top] + s[top + 1]) / 2.0
        qs = np.concatenate([qs, mids])
    return np.unique(np.round(qs, 5))


def fit_link(verts: np.ndarray, slab_axis: int, max_regions: int) -> tuple[list[object], float, int]:
    coords = verts[:, slab_axis]
    planes = candidate_planes(coords, N_CANDIDATES)
    bounds = np.unique(np.concatenate([[coords.min() - 1e-9], planes, [coords.max() + 1e-9]]))
    nb = len(bounds)
    cache: dict[tuple[int, int], tuple[object, float]] = {}

    def region_fit(i: int, j: int) -> tuple[object, float]:
        key = (i, j)
        if key not in cache:
            m = (coords >= bounds[i] - SEAM_EPS) & (coords <= bounds[j] + SEAM_EPS)
            sub = verts[m]
            cache[key] = (None, 0.0) if len(sub) < 3 else fit_region(sub)
        return cache[key]

    best_total = None
    best_config = None
    for K in range(1, max_regions + 1):
        INF = 1e18
        dp = np.full((K + 1, nb), INF)
        choice = np.full((K + 1, nb), -1, dtype=np.int64)
        dp[0, nb - 1] = 0.0
        for r in range(1, K + 1):
            for i in range(nb - 2, -1, -1):
                for j in range(i + 1, nb):
                    if dp[r - 1, j] >= INF:
                        continue
                    _, vol = region_fit(i, j)
                    c = vol + dp[r - 1, j]
                    if c < dp[r, i]:
                        dp[r, i] = c
                        choice[r, i] = j
        raw = dp[K, 0]
        cost = raw * (1.0 + K_PENALTY * (K - 1))
        if best_total is None or cost < best_total:
            prims = []
            i, r = 0, K
            while r > 0:
                j = choice[r, i]
                prim, _ = region_fit(i, j)
                if prim is not None:
                    prims.append(prim)
                i, r = j, r - 1
            best_total = cost
            best_config = (prims, raw, K)
    return best_config


def surface_samples(tris: np.ndarray, n: int = 200000, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    total = areas.sum()
    if total <= 0:
        return tris.reshape(-1, 3)
    idx = rng.choice(len(tris), size=n, p=areas / total)
    r1 = np.sqrt(rng.random(n))
    r2 = rng.random(n)
    a, b, c = 1 - r1, r1 * (1 - r2), r1 * r2
    return a[:, None] * v0[idx] + b[:, None] * v1[idx] + c[:, None] * v2[idx]


def coverage(prims: list[object], pts: np.ndarray) -> tuple[float, float]:
    inside = np.zeros(len(pts), dtype=bool)
    for p in prims:
        inside |= p.contains(pts)
    frac = float(inside.mean())
    if inside.all():
        return frac, 0.0
    out = pts[~inside]
    dists = np.full(len(out), np.inf)
    for p in prims:
        if isinstance(p, Box):
            d = np.linalg.norm(np.maximum(np.maximum(p.lo - out, out - p.hi), 0.0), axis=1)
        else:
            a, _b = p._caps()
            axis = np.zeros(3)
            axis[p.axis] = 1.0
            rel = out - a
            t = rel @ axis
            length = p.a_hi - p.a_lo
            radial = np.linalg.norm(rel - np.outer(t, axis), axis=1)
            dr = np.maximum(radial - p.radius, 0.0)
            dt = np.maximum(np.maximum(-t, t - length), 0.0)
            d = np.sqrt(dr**2 + dt**2)
        dists = np.minimum(dists, d)
    return frac, float(dists.max())


def render_overlay(name: str, verts: np.ndarray, prims: list[object], out_path: str) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(21, 7))
    for ax, (i, j, label) in zip(axes, [(0, 1, "xy"), (0, 2, "xz"), (1, 2, "yz")]):
        ax.scatter(verts[:, i], verts[:, j], s=0.8, c="black", alpha=0.8, linewidths=0)
        ax.set_xlabel("xyz"[i])
        ax.set_ylabel("xyz"[j])
        ax.set_title(f"{name} {label}")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        for prim in prims:
            prim.draw_2d(ax, i, j)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def find_visual_stls(desc_dir: str, only: str | None) -> list[tuple[str, str, str]]:
    """
    Return (group, link_name, stl_path) for every visual STL. group is the directory level between
    meshes/ and /visual (e.g. links, gripper).
    """
    hits = []
    meshes_dir = os.path.join(desc_dir, "meshes")
    for root, _dirs, files in os.walk(meshes_dir):
        if os.path.basename(root) != "visual":
            continue
        group = os.path.relpath(os.path.dirname(root), meshes_dir)
        for f in sorted(files):
            if f.lower().endswith(".stl"):
                name = os.path.splitext(f)[0]
                if only is None or name == only:
                    hits.append((group, name, os.path.join(root, f)))
    return hits


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--desc-dir", required=True, help="Path to the <robot>_description directory.")
    parser.add_argument("--report-dir", required=True, help="Where overlay PNGs and manifest.json go.")
    parser.add_argument("--mesh", default=None, help="Fit only this mesh basename (e.g. link1).")
    args = parser.parse_args()

    os.makedirs(args.report_dir, exist_ok=True)
    entries = find_visual_stls(args.desc_dir, args.mesh)
    if not entries:
        raise SystemExit(f"no visual STLs found under {args.desc_dir}/meshes/**/visual/")

    manifest: dict[str, dict] = {}
    for group, name, stl_path in entries:
        tris = parse_binary_stl(stl_path)
        verts, _ = weld(tris)
        fit_pts = np.vstack([verts, surface_samples(tris, n=N_FIT_SAMPLES, seed=1)])
        results = []
        for slab_axis in range(3):
            prims, raw, K = fit_link(fit_pts, slab_axis, MAX_REGIONS)
            results.append((raw * (1.0 + K_PENALTY * (K - 1)), raw, slab_axis, K, prims))
        results.sort(key=lambda t: t[0])
        _cost, raw, slab_axis, K, prims = results[0]
        prims = [p.inflate(INFLATE) for p in prims]

        samples = surface_samples(tris)
        frac_v, maxd_v = coverage(prims, verts)
        frac_s, maxd_s = coverage(prims, samples)
        aabb_vol = float(np.prod(verts.max(axis=0) - verts.min(axis=0)))
        print(f"=== {name}: slab_axis={'xyz'[slab_axis]} K={K} ===")
        for p in prims:
            print(f"    {p!r}")
        print(f"    total_prim_vol={raw * 1e6:.1f} cm3  aabb_vol={aabb_vol * 1e6:.1f} cm3 (ratio {raw / aabb_vol:.2f})")
        print(f"    vert coverage={frac_v * 100:.3f}% maxd={maxd_v * 1000:.3f}mm")
        print(f"    surf coverage={frac_s * 100:.3f}% maxd={maxd_s * 1000:.3f}mm")
        if frac_s < 1.0 or maxd_s > 5e-4:
            print(f"WARNING: {name} surface not fully enclosed; do not ship this fit")

        # Write the convex primitive OBJs next to the visual meshes.
        outdir = os.path.join(args.desc_dir, "meshes", group, "collision", name)
        os.makedirs(outdir, exist_ok=True)
        counters = {"box": 0, "cylinder": 0}
        rel_paths = []
        for prim in prims:
            kind = prim.spec()["type"]
            counters[kind] += 1
            fname = f"{name}_{kind}{counters[kind]}.obj"
            pverts, pfaces = prim.mesh()
            write_obj(
                os.path.join(outdir, fname),
                pverts,
                pfaces,
                header=f"Convex collision primitive for {name} ({kind}), fitted to the visual mesh.",
            )
            rel_paths.append(os.path.join("meshes", group, "collision", name, fname))
        manifest[name] = {"group": group, "primitives": [p.spec() for p in prims], "obj_paths": rel_paths}

        render_overlay(name, verts, prims, os.path.join(args.report_dir, f"{name}.png"))

    suffix = "" if args.mesh is None else f"_{args.mesh}"
    manifest_path = os.path.join(args.report_dir, f"manifest{suffix}.json")
    with open(manifest_path, "w") as fh:
        json.dump(manifest, fh, indent=2)
    print(f"manifest -> {manifest_path}")


if __name__ == "__main__":
    main()
