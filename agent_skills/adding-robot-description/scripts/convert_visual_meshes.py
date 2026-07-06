#!/usr/bin/env python3
"""
Convert visual STL meshes to decimated OBJ with vertex normals, for a robot description package.

Scans <desc_dir>/meshes/**/visual/*.stl and writes a sibling .obj for each: quadric-edge-collapse
decimation via meshlabserver (if the mesh exceeds the face target), then computed area-weighted vertex
normals. Drake's VTK render engine REQUIRES vn records in OBJ visuals (meshcat does not) -- without them
RgbdSensor rendering fails with "OBJ has no normals".

meshlabserver note: the Quadric Edge Collapse filter takes an exact parameter set that varies by meshlab
version; passing a partial set aborts with "Assertion parameterSet.size() == required.size()". This script
writes the 13-parameter set matching meshlab 2020-era builds (TargetFaceNum ... Selected, including
PlanarWeight). If your meshlabserver differs, dump the expected list with: meshlabserver -d filters.txt

Usage:
    convert_visual_meshes.py --desc-dir <repo>/<robot>_description [--target-faces 20000] [--mesh link1]

Verification printed per mesh: OBJ-vs-STL AABB deviation (should be < ~0.01 mm) and vertex/face counts.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_tools import parse_binary_stl

_MLX_TEMPLATE = """<!DOCTYPE FilterScript>
<FilterScript>
 <filter name="Simplification: Quadric Edge Collapse Decimation">
  <Param type="RichInt" value="{target_faces}" name="TargetFaceNum"/>
  <Param type="RichFloat" value="0" name="TargetPerc"/>
  <Param type="RichFloat" value="0.5" name="QualityThr"/>
  <Param type="RichBool" value="true" name="PreserveBoundary"/>
  <Param type="RichFloat" value="1" name="BoundaryWeight"/>
  <Param type="RichBool" value="true" name="PreserveNormal"/>
  <Param type="RichBool" value="true" name="PreserveTopology"/>
  <Param type="RichBool" value="true" name="OptimalPlacement"/>
  <Param type="RichBool" value="false" name="PlanarQuadric"/>
  <Param type="RichFloat" value="0.001" name="PlanarWeight"/>
  <Param type="RichBool" value="false" name="QualityWeight"/>
  <Param type="RichBool" value="true" name="AutoClean"/>
  <Param type="RichBool" value="false" name="Selected"/>
 </filter>
</FilterScript>
"""


def find_visual_stls(desc_dir: str, only: str | None) -> list[str]:
    hits = []
    meshes_dir = os.path.join(desc_dir, "meshes")
    for root, _dirs, files in os.walk(meshes_dir):
        if os.path.basename(root) != "visual":
            continue
        for f in sorted(files):
            if f.lower().endswith(".stl") and (only is None or os.path.splitext(f)[0] == only):
                hits.append(os.path.join(root, f))
    return hits


def parse_obj(path: str) -> tuple[np.ndarray, np.ndarray]:
    verts, faces = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                verts.append([float(t) for t in line.split()[1:4]])
            elif line.startswith("f "):
                faces.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
    return np.array(verts), np.array(faces, dtype=np.int64)


def vertex_normals(verts: np.ndarray, faces: np.ndarray) -> np.ndarray:
    v0, v1, v2 = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    fn = np.cross(v1 - v0, v2 - v0)
    vn = np.zeros_like(verts)
    for k in range(3):
        np.add.at(vn, faces[:, k], fn)
    norms = np.linalg.norm(vn, axis=1, keepdims=True)
    norms[norms < 1e-30] = 1.0
    return vn / norms


def write_obj_with_normals(path: str, verts: np.ndarray, faces: np.ndarray) -> None:
    vn = vertex_normals(verts, faces)
    lines = ["# Decimated visual mesh with computed per-vertex normals.", "o obj1"]
    for v in verts:
        lines.append(f"v {v[0]:.8g} {v[1]:.8g} {v[2]:.8g}")
    for n in vn:
        lines.append(f"vn {n[0]:.5g} {n[1]:.5g} {n[2]:.5g}")
    for f in faces:
        lines.append(f"f {f[0] + 1}//{f[0] + 1} {f[1] + 1}//{f[1] + 1} {f[2] + 1}//{f[2] + 1}")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def convert(stl_path: str, target_faces: int) -> None:
    obj_path = os.path.splitext(stl_path)[0] + ".obj"
    with tempfile.TemporaryDirectory() as tmp:
        mlx = os.path.join(tmp, "decimate.mlx")
        with open(mlx, "w") as fh:
            fh.write(_MLX_TEMPLATE.format(target_faces=target_faces))
        raw_obj = os.path.join(tmp, "raw.obj")
        subprocess.run(
            ["meshlabserver", "-i", stl_path, "-o", raw_obj, "-s", mlx],
            check=True,
            capture_output=True,
            timeout=600,
        )
        verts, faces = parse_obj(raw_obj)
    write_obj_with_normals(obj_path, verts, faces)

    # Fidelity check: the decimated OBJ's AABB should match the STL's almost exactly.
    stl_pts = parse_binary_stl(stl_path).reshape(-1, 3)
    dev = max(
        np.abs(stl_pts.min(axis=0) - verts.min(axis=0)).max(),
        np.abs(stl_pts.max(axis=0) - verts.max(axis=0)).max(),
    )
    name = os.path.basename(stl_path)
    print(f"{name:>24s} -> {len(verts):6d} verts {len(faces):6d} faces, AABB dev {dev * 1000:.4f} mm")
    if dev > 1e-4:
        print(f"WARNING: {name} AABB deviation exceeds 0.1 mm; inspect the decimation quality")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--desc-dir", required=True, help="Path to the <robot>_description directory.")
    parser.add_argument("--target-faces", type=int, default=20000, help="Decimation face target per mesh.")
    parser.add_argument("--mesh", default=None, help="Convert only this mesh basename (e.g. link1).")
    args = parser.parse_args()

    stls = find_visual_stls(args.desc_dir, args.mesh)
    if not stls:
        raise SystemExit(f"no visual STLs found under {args.desc_dir}/meshes/**/visual/")
    for stl in stls:
        convert(stl, args.target_faces)


if __name__ == "__main__":
    main()
