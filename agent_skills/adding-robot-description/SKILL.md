---
name: adding-robot-description
description: Use when adding a new robot model to this repository — porting a URDF/description from a manufacturer repo, ROS package, or vendor SDK into the repo's <robot>_description format with meshes, collision geometry, Drake URDF, README and screenshots.
---

# Adding a Robot Description

## Overview

Port a source description faithfully (kinematics, limits, inertials verbatim), replace its heavy geometry
with decimated visuals + tight convex collision primitives, and prove every step with Drake: parse checks,
FK renders against product photos, coverage numbers, self-collision queries. Follow the structure of the
existing `*_description` directories exactly — read `lite6_description/` and `rebot_b601_dm_description/`
as references before writing anything.

## Inputs

A pointer to an existing description: a manufacturer URDF/xacro, a ROS(2) package, a vendor GitHub repo, or
a wiki page. If given only a product page, search the vendor's GitHub org — description files often live in
the ROS2/controller repo (`*_bringup/description/`), NOT the main hardware repo. Record the exact source
repo/path and its license before modifying anything. If multiple candidate descriptions exist (legacy vs
current product line), stop and ask which is authoritative rather than guessing.

Source-format notes:
- xacro sources: flatten once (`xacro file.urdf.xacro > flat.urdf`, or hand-expand) with the plain
  arm+gripper argument set, and record the args used in the README. The flattened file is the verbatim
  reference; never edit the xacro.
- Non-STL meshes (DAE/PLY): convert to STL first (`meshlabserver -i in.dae -o out.stl`) — the pipeline
  scripts scan `visual/*.stl` only. Colors lost from DAE are reapplied via URDF materials.

## Environment

Needs `pydrake`, `meshlabserver`, `google-chrome`, `numpy`, `matplotlib`. Prefer running from a project env
that already has Drake (e.g. manor's conda env). If none exists, create a disposable env and REMOVE it when
done:

```bash
conda create -y -n drake_tmp python=3.12 && conda activate drake_tmp
pip install drake numpy matplotlib
# ... do the work ...
conda deactivate && conda env remove -y -n drake_tmp
```

## Workflow

Scripts live in `scripts/` next to this file; each documents its flags via `--help`.

1. **Research the source.** Find the URDF + mesh files, joint limits, motor specs, gripper geometry, and
   the vendor's zero pose (render/photos). Fetch any vendor configs (MoveIt limits, SDK yaml) — they
   catch URDF lies (e.g. velocity limits that are motor no-load figures).
2. **Skeleton + meshes.** Create `<robot>_description/{meshes/<group>/{visual,collision}/,urdf/,drake_urdf/}`
   with groups like `links/` and `gripper/`. Directory name: lowercase snake-case of the product name plus
   `_description` (matches how the parser package name is derived). Download source meshes into
   `meshes/<group>/visual/`. Verify each STL: binary format (`84 + 50*n_triangles == filesize`) and
   meter-scale AABBs (a 0.1–1 m robot, not 100–1000 mm). Keep the original STLs; they ship alongside the
   OBJs. One URDF per shipping configuration — only add with/without-gripper flavors (lite6-style
   subdirectories) if the hardware actually ships those trims; a single-configuration robot gets one flat
   file per flavor directory (rebot-style).
3. **Convert visuals:** `scripts/convert_visual_meshes.py --desc-dir <dir>`. Produces decimated OBJs with
   vertex normals and prints AABB fidelity per mesh.
4. **Fit collision primitives:** `scripts/fit_collision_primitives.py --desc-dir <dir> --report-dir <tmp>`.
   Run per-link in parallel shells for speed (`--mesh <link>`). For every link: surface coverage must print
   100% with ~0 mm leak, and you must LOOK at the overlay PNG — tight red/blue outlines hugging the black
   point cloud. Volume ratio (primitives / AABB) around 0.2–0.9 is normal.
5. **Author the URDFs.** Write `urdf/<robot>.urdf` (plain) and `drake_urdf/<robot>.urdf` (Drake tags).
   A small generator script beats hand-editing 30 collision blocks; keep source values verbatim.
   Conventions (mirror the existing robots):
   - Keep the source's link/joint names, origins, axes, limits, inertials unchanged.
   - Mesh URIs: `package://<robot>_description/meshes/...` (package name == directory name).
   - Drake URDF: `xmlns:drake` on `<robot>`, collision meshes wrapped with `<drake:declare_convex/>`,
     named materials approximating product colors, transmissions per actuated joint (with
     `drake:gear_ratio` if motor reductions are known).
   - Add a TCP frame (`link_eef_tip`-style fixed frame) if the source lacks one — measure the grasp point
     from the assembled gripper meshes, don't guess.
   - Mimic-joint grippers (one motor, two fingers): Drake's URDF mimic support is limited — model both
     fingers as independently actuated prismatic joints (the existing robots do this) and let the software
     layer command them symmetrically; document the deviation in the README.
   - Check self-collision at the zero pose and operational poses (`ComputePointPairPenetration`). Pairs
     that overlap at EVERY configuration (permanently nested parts) get a `drake:collision_filter_group`;
     pairs that only touch in folded poses stay unfiltered and get documented in the README.
6. **Verify in Drake.** Parse both URDFs (plain + drake), check `num_positions`/`num_actuators` match the
   spec sheet, and render the robot offscreen (RgbdSensor + RenderEngineVtk) at the vendor zero pose —
   compare against product photos: colors, proportions, pose, gripper orientation. Wrong-looking renders
   mean wrong joint origins or mesh frames; fix before proceeding.
7. **README + LICENSE + screenshots.** Take README images with
   `scripts/screenshot_model.py --repo-root <repo> --urdf <drake_urdf> --base-link <base> --out-dir
   <desc_dir>/images --prefix <robot>` (choose a pose that shows the arm off, via `--positions`). Write
   `<robot>_description/README.md` following the existing ones: Overview (source links), Additional
   modifications (every deviation from source), robot-specific sections (gripper geometry, zero pose),
   Visuals (visual / collision geometry / inertia representations images). Copy the manufacturer's license
   text verbatim into `LICENSE`. Add the robot's row to the repo's top-level `README.md`.

## Gotchas (each one cost real debugging time)

| Symptom | Cause / fix |
|---|---|
| meshlabserver asserts `parameterSet.size() == required.size()` | Filter param set is version-exact; use the 13-param set in `convert_visual_meshes.py`, or dump expected params with `meshlabserver -d` |
| Drake render fails `OBJ has no normals` | VTK render engine requires `vn` records (meshcat doesn't) — the convert script adds them; don't hand-convert |
| Headless-chrome screenshot of live meshcat URL is blank | `--virtual-time-budget` fast-forwards past websocket delivery; snapshot `Meshcat.StaticHtml()` to a file:// URL instead (screenshot script does this) |
| Collision fit reports 100% on vertices but leaks on the surface | Fitting on vertices only lets thin boxes cheat through vertex-dense feature rings; fit on surface samples (the fit script already does) |
| Robot renders folded/tangled at q = 0 | Often correct! Many arms zero at a folded pose; verify against vendor photos before "fixing" joint origins |
| `package://` URIs fail to resolve | Package name must equal the description directory name; register every `*_description` dir in the parser's PackageMap |
| Source URDF velocity/effort limits look absurd | Usually motor no-load figures; keep them verbatim but note reality (vendor planner limits) in the README |

## Done checklist

- [ ] Both URDFs parse in Drake; DOF/actuator counts match the spec sheet
- [ ] Every link: 100% surface coverage, ~0 mm leak, overlay PNG visually tight
- [ ] Offscreen render at vendor zero pose matches product photos
- [ ] Self-collision at zero + operational poses: only documented/filtered pairs
- [ ] README (with images), LICENSE (manufacturer verbatim), top-level README row
- [ ] Temp conda env removed, if one was created
