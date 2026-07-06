#!/usr/bin/env python3
"""
Capture meshcat screenshots of a robot description for its README: visual meshes, collision geometry, and
inertia representations, each as a separate image (matching the style of the existing model READMEs).

Requires pydrake (run from an env that has it) and google-chrome for the headless capture.

How it works: builds a plant from the Drake URDF, adds AddDefaultVisualization (which installs the
/drake/illustration, /drake/proximity and /drake/inertia meshcat layers), toggles exactly one layer visible
per shot, then snapshots via Meshcat.StaticHtml() rendered by headless chrome from a file:// URL.

DO NOT screenshot the live meshcat URL with headless chrome: --virtual-time-budget fast-forwards virtual
time without letting the websocket deliver the scene, producing a blank page. StaticHtml bakes the scene
into the HTML so the file:// capture is deterministic.

Every *_description directory directly under --repo-root is registered in the package map under its
directory name, so package://<robot>_description/... URIs resolve.

Usage:
    screenshot_model.py --repo-root <repo> --urdf <repo>/x_description/drake_urdf/x.urdf \
        --base-link base_link --out-dir <repo>/x_description/images --prefix x \
        [--positions 0 -0.7 ... ] [--camera 0.6 -0.6 0.5] [--target 0.1 0 0.2]
"""

from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
import time

import numpy as np
from pydrake.geometry import Meshcat, MeshcatParams
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

_MESHCAT_PORT = 7091


def capture(html: str, out_path: str) -> None:
    with tempfile.NamedTemporaryFile("w", suffix=".html", delete=False) as fh:
        fh.write(html)
        html_path = fh.name
    try:
        subprocess.run(
            [
                "google-chrome",
                "--headless=new",
                "--hide-scrollbars",
                "--window-size=1400,1050",
                f"--screenshot={out_path}",
                "--virtual-time-budget=30000",
                f"file://{html_path}",
            ],
            check=True,
            capture_output=True,
            timeout=180,
        )
    finally:
        os.remove(html_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo-root", required=True, help="robot_models repo root (package map source).")
    parser.add_argument("--urdf", required=True, help="Drake URDF to render.")
    parser.add_argument("--base-link", required=True, help="Frame welded to the world.")
    parser.add_argument("--out-dir", required=True, help="Output directory for the PNGs.")
    parser.add_argument("--prefix", required=True, help="Image filename prefix (e.g. the robot name).")
    parser.add_argument(
        "--positions",
        type=float,
        nargs="*",
        default=None,
        help="Full plant positions vector for the pose; defaults to the plant default (usually zeros).",
    )
    parser.add_argument("--camera", type=float, nargs=3, default=[0.6, -0.6, 0.5], help="Camera eye xyz.")
    parser.add_argument("--target", type=float, nargs=3, default=[0.1, 0.0, 0.2], help="Camera target xyz.")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    meshcat = Meshcat(MeshcatParams(port=_MESHCAT_PORT))

    builder = DiagramBuilder()
    plant, _scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    urdf_parser = Parser(plant)
    for dirname in sorted(os.listdir(args.repo_root)):
        dirpath = os.path.join(args.repo_root, dirname)
        if os.path.isdir(dirpath) and dirname.endswith("_description"):
            urdf_parser.package_map().Add(dirname, dirpath)
    urdf_parser.AddModels(args.urdf)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(args.base_link))
    plant.Finalize()
    AddDefaultVisualization(builder=builder, meshcat=meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    if args.positions is not None:
        positions = np.asarray(args.positions, dtype=np.float64)
        if positions.shape[0] != plant.num_positions():
            raise SystemExit(f"--positions needs {plant.num_positions()} values; got {positions.shape[0]}")
        plant.SetPositions(plant.GetMyContextFromRoot(context), positions)
    diagram.ForcedPublish(context)

    meshcat.SetCameraPose(np.asarray(args.camera), np.asarray(args.target))

    layers = {
        "visual": (True, False, False),
        "collision": (False, True, False),
        "inertia": (False, False, True),
    }
    for label, (vis, col, inertia) in layers.items():
        meshcat.SetProperty("/drake/illustration", "visible", vis)
        meshcat.SetProperty("/drake/proximity", "visible", col)
        meshcat.SetProperty("/drake/inertia", "visible", inertia)
        time.sleep(0.5)
        out_path = os.path.join(args.out_dir, f"{args.prefix}_{label}.png")
        capture(meshcat.StaticHtml(), out_path)
        print(f"captured {out_path}")


if __name__ == "__main__":
    main()
