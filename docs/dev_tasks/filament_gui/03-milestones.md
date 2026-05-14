# Filament GUI Replacement - Milestones

## Phase 0 - Dependency and MVP spike

**Deliverables**

- `examples/filament_gui` builds with an explicit experimental option.
- Filament, GLFW, and Dear ImGui dependency discovery is understood and
  documented.
- MVP scene renders a DART world with camera controls and a minimal ImGui panel.
- MVP scene demonstrates visible shadow casting and receiving.

**Exit criteria**

- MVP acceptance criteria in `02-mvp-example.md` pass on Linux.
- Maintainers agree Filament packaging/tooling risk is manageable enough to
  continue.

## Phase 1 - Scene extraction foundation

**Deliverables**

- Backend-hidden scene extraction for common DART renderables.
- Unit tests for renderable identity, transform propagation, shape descriptors,
  color/material extraction, and cache invalidation.
- Example uses the extraction layer instead of bespoke scene code.

**Exit criteria**

- Common primitive shapes render from a `simulation::World` without OSG.
- Extraction tests do not require a graphics context.

**Current progress**

- `dart-gui-experimental` extraction descriptors exist for visual shape nodes,
  common primitive geometry, material colors, visibility, shadow flags,
  transforms, stable runtime IDs, and version stamps.
- A graphics-free unit test covers descriptor stability, transform propagation,
  property updates, visibility, and common primitive descriptors.
- The Filament example uses the extraction layer for the dynamic box/ground
  fixture, additional primitive visual shapes including `PyramidShape` and
  `MultiSphereConvexHullShape`, visual-only line segments from
  `LineSegmentShape`, a generated TriMesh visual, and a finite checker-textured
  PlaneShape proxy.
- The visual fixture includes an imported WAM Collada mesh. Filament mesh
  rendering now consumes DART's preserved UV metadata, submesh material ranges,
  material colors, emissive color, metallic/roughness factors, and typed
  PNG/JPEG texture paths for base color, metallic, roughness, combined
  metallic-roughness, normal, occlusion, and emissive maps when available.
- The Filament smoke fixture now also loads the full WAM URDF skeleton through
  DART's normal `dart::io` and `dart-utils-urdf` path, so the descriptor and
  rendering path sees a larger articulated robot asset instead of only isolated
  mesh fixtures.
- The Filament smoke fixture also requires a larger Atlas DAE torso mesh to
  load and create a Filament renderable, adding a non-WAM robot mesh fixture to
  the material/rendering path.
- The Filament smoke fixture also requires the full Atlas SDF robot to load
  through DART's normal `dart::io` path and produce at least twenty visible mesh
  descriptors/renderables, adding a larger articulated robot fixture to the
  material/rendering path.
- Checked-in glTF PBR fixtures are loaded through the real Assimp importer in
  `UNIT_dynamics_MeshShape`, covering authored base-color,
  metallic-roughness, normal, occlusion, and emissive texture slots plus UV
  metadata, alpha-bearing material factors, and single-/multi-material submesh
  ranges without requiring a graphics context.
- The Filament smoke fixture also loads those glTF PBR fixtures and routes
  alpha-bearing visual aspects and mesh material alpha through transparent lit
  and textured-lit Filament material variants.
- The Filament smoke fixture also uses the multi-material glTF PBR asset as a
  required four-panel environment layout, so the renderer exercises authored
  PBR maps on more than isolated mesh samples.
- `dart-gui-experimental` now owns backend-hidden debug line descriptors for a
  grid, world/body frames, center-of-mass markers, contact markers, contact
  normals, and contact force vectors. The Filament example converts those
  descriptors to line primitives.
- The extraction layer includes tested local picking bounds and nearest
  ray-hit helpers for visible renderables.
- Phase 1 is not complete yet because broader human visual review with larger
  authored environment and PBR assets beyond the current WAM, Atlas, and PBR
  panel fixtures is still pending.

## Phase 2 - Viewer-quality baseline

**Deliverables**

- Render common primitives, static meshes, and colors with equal or better
  quality than DART's current OSG examples.
- Enable the visual-quality requirements in `06-visual-quality.md`, including
  shadows, PBR materials, lighting, anti-aliasing, and depth cues.
- Add screenshot capture and bounded-frame smoke mode.
- Add basic debug overlays: grid, frames, contacts, and force vectors.

**Exit criteria**

- One maintained example can be demonstrated end-to-end on Filament.
- Screenshots are stable enough for visual smoke checks.
- The shadow gate in `06-visual-quality.md` passes.
- Grid, world/body frame, contact, normal, and force-vector overlays render in
  the MVP; the built-in panel can toggle the current overlay groups. Broader
  scenario coverage is still needed before Phase 2 is complete.
- Alpha-bearing solid, textured, and mesh visual paths render through
  transparent lit material variants in the MVP scene. Broader transparency
  workflows still need visual review before promotion.
- Windowed screenshot capture and headless swap-chain screenshot capture both
  produce local nonblank smoke images; the hosted Ubuntu GCC/Clang Filament
  smoke jobs passed on merged MVP PR #2647. Broader CI coverage remains
  promotion work.

## Phase 3 - Interaction baseline

**Deliverables**

- Backend-hidden input model.
- Selection/picking for frames/bodies/shapes.
- Minimal manipulation workflow or an explicit replacement for the OSG
  drag-and-drop behavior.

**Exit criteria**

- Basic tutorial workflows that need picking or manipulation can be supported
  without OSG.

**Current progress**

- Basic click-to-select is wired in the Filament example through the
  backend-hidden picking helpers. It reports the selected DART shape in the
  ImGui panel, applies a temporary highlight, and renders a
  renderer-independent selection bounds overlay.
- Backend-hidden helpers can translate selected free-joint and `SimpleFrame`
  renderables through a combined frame-renderable API. The Filament example
  uses that shared path for keyboard nudging and camera-plane dragging, while
  C++/Python tests cover the mutable free-joint path, fixed-body rejection, and
  simple-frame translation.
- Backend-hidden plane intersection and plane-drag translation helpers now
  support Ctrl-left camera-plane dragging of selected dynamic bodies or
  `SimpleFrame` visuals in the Filament example, with C++/Python coverage for
  the shared math.
- The Filament example includes a selectable `--scene drag-and-drop` fixture
  that extracts and renders the legacy drag-and-drop example's `SimpleFrame`
  anchor, child frame, and axis markers through the same manipulation path.
- The Filament example includes a selectable `--scene g1` fixture that loads
  the Unitree G1 URDF through DART resource retrievers and exposes colored IK
  targets for both hands and feet. `pixi run ex g1_puppet` routes to this
  Filament scene by default so the in-tree G1 workflow no longer depends on the
  legacy OSG viewer path.
- The same panel provides pause/resume and single-step controls.
- Phase 3 has an MVP interaction baseline through picking, selection bounds,
  selected-body and `SimpleFrame` movement, a first drag-and-drop fixture, and
  a robot IK fixture. Broader interaction-heavy workflows still need migration
  and visual review before promotion.

## Phase 4 - Experimental library target

**Deliverables**

- Move reusable code from the example into an experimental `dart-gui` target or
  subtarget.
- Keep public APIs under `dart::gui::experimental` until the major-version
  replacement point.
- Add `dartpy` bindings only for the constrained experimental API.

**Exit criteria**

- At least two examples use the experimental API.
- The API remains free of Filament/GLFW/ImGui types.

**Current progress**

- `dart-gui-experimental` exists and owns the backend-hidden scene extraction
  and picking helpers used by the Filament example.
- The Filament example links through `dart-gui-experimental` and can be built
  in an OSG-free configure when `DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF`.
- `dart-gui-experimental` now owns the run-option normalization, orbit-camera
  basis/update logic, camera eye computation, and perspective pick-ray
  generation used by the Filament example.
- `dart-gui-experimental` now owns viewer lifecycle state for pause/step
  behavior, screenshot request tracking, rendered/skipped frame counters, and
  bounded-run stop checks. The Filament example uses this state instead of
  keeping those viewer-loop decisions example-local.
- `dart-gui-experimental` now owns reusable RGBA-to-PPM screenshot storage.
  The Filament example keeps only renderer readback and delegates file writing
  to the backend-hidden helper.
- `dartpy.gui.experimental` exposes constrained bindings for renderable
  descriptors, material/geometry descriptors, picking helpers, free-joint
  translation, plane-drag helpers, debug-line descriptor generation, run
  options, viewer lifecycle state, screenshot storage, and orbit-camera helpers.
- `dartpy` can now build the constrained `dartpy.gui.experimental` module with
  `DART_BUILD_GUI=OFF` when `dart-gui-experimental` is present, so the Python
  experimental API is no longer tied to the legacy GUI target.
- `examples/gui_scene_diagnostics` is a second example consumer of the
  experimental API. It exercises descriptors, debug lines, run options, camera
  basis, and picking without introducing another renderer.
- Phase 4 is not complete because remaining interaction-heavy workflow
  migration and cross-platform wheel coverage are still pending.

## Phase 5 - First-class promotion

**Deliverables**

- Promote the new API to the primary `dart::gui` API in the appropriate major
  DART version.
- Update maintained examples/tutorials and documentation.
- Keep OSG only as a temporary legacy component if required.

**Exit criteria**

- Maintained examples/tutorials and `dartpy` have a supported non-OSG
  visualization path.
- Filament dependency handling works in the supported source-build and wheel
  workflows.
- Maintainers accept the visual-quality gate for DART's built-in
  visualization/debugging workflows.

## Phase 6 - Legacy removal

**Deliverables**

- Remove current OSG GUI APIs and implementation after the deprecation window.
- Remove the Raylib smoke experiment and `DART_BUILD_GUI_RAYLIB`.
- Remove migration shims that were only needed during the transition.

**Exit criteria**

- `DART_BUILD_GUI=ON` uses the Filament implementation by default.
- Default GUI builds do not require OSG or Raylib.
