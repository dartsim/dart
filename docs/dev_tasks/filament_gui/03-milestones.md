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
  `LineSegmentShape`, a generated `ConvexMeshShape` tetrahedron, a generated
  `PointCloudShape` box-point cloud with per-point colors, a generated
  `HeightmapShape` terrain mesh, a generated `SoftMeshShape` soft-body surface,
  an OctoMap-backed `VoxelGridShape` cell cluster when available, a generated
  TriMesh visual, and a finite checker-textured PlaneShape proxy.
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
  normal arrows, contact force arrows, support-polygon outlines,
  support-centroid markers, inertia boxes, and collision-shape bounds. The
  Filament example converts those descriptors to line primitives.
- The extraction layer includes tested local picking bounds and nearest
  ray-hit helpers for visible renderables, including bounds hit points and hit
  normals plus primitive
  sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane
  surface points and normals.
- Unsupported shape types now remain visible to non-rendering consumers through
  diagnostic descriptors instead of being silently dropped by extraction.
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
- Grid, world/body frame, contact, normal-arrow, force-arrow, center-of-mass,
  support-polygon, inertia-box, and collision-shape-bound overlays render in
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
  uses that shared path with camera-relative directional nudge math for
  keyboard nudging and camera-plane dragging, while C++/Python tests cover the
  mutable free-joint path, fixed-body rejection, simple-frame translation, and
  nudge-vector calculation.
- Backend-hidden plane intersection and plane-drag translation helpers now
  support Ctrl-left camera-plane dragging of selected dynamic bodies or
  `SimpleFrame` visuals in the Filament example, with C++/Python coverage for
  the shared math.
- Backend-hidden axis-drag translation helpers are also covered by C++/Python
  tests, giving future constrained manipulators a renderer-independent math
  primitive without exposing Filament input types.
- The Filament example includes a selectable `--scene hello-world` fixture that
  extracts and renders the legacy hello-world example's single dynamic box and
  ground plane through the same descriptor-driven Filament path.
  `pixi run ex hello_world` routes to this Filament scene by default so the
  simplest in-tree visual workflow no longer depends on the legacy OSG viewer
  path.
- The Filament example includes a selectable `--scene boxes` fixture that
  extracts and renders the legacy boxes example's dynamic colored box grid and
  ground plane through the same descriptor-driven Filament path.
  `pixi run ex boxes` routes to this Filament scene by default so the in-tree
  multi-box visual workflow no longer depends on the legacy OSG viewer path.
- `pixi run ex rigid_cubes` also routes to the `--scene boxes` Filament fixture
  by default while the standalone source remains as legacy OSG comparison
  material for directional force controls and frame-recording options.
- `pixi run ex box_stacking` also routes to the `--scene boxes` Filament
  fixture by default while the standalone source remains as legacy OSG/ImGui
  comparison material for solver selection, gravity controls, and custom key
  callbacks.
- The Filament example includes a selectable `--scene hardcoded-design` fixture
  that recreates the legacy hardcoded-design example's three-link
  manually-constructed skeleton through descriptor-owned box renderables.
  `pixi run ex hardcoded_design` routes to this Filament scene by default
  while the standalone source remains as legacy OSG comparison material for
  wireframe rendering and direct key-controlled joint motion.
- The Filament example includes a selectable `--scene rigid-chain` fixture
  that loads the legacy rigid-chain SKEL data and renders its ten link boxes
  through descriptor-owned renderables. `pixi run ex rigid_chain` routes to
  this Filament scene by default while the standalone source remains as legacy
  OSG comparison material for the custom per-step damping hook.
- The Filament example includes a selectable `--scene rigid-loop` fixture that
  loads the legacy loop-chain SKEL data, restores the loop-closing pose and
  ball-joint constraint, marks the constrained links red, and renders its ten
  link boxes through descriptor-owned renderables.
  `pixi run ex rigid_loop` routes to this Filament scene by default while the
  standalone source remains as legacy OSG comparison material for damping and
  constraint setup.
- The Filament example includes a selectable `--scene mixed-chain` fixture that
  loads the legacy mixed rigid/soft chain SKEL data and renders its ten box
  links plus five soft meshes through descriptor-owned renderables.
  `pixi run ex mixed_chain` routes to this Filament scene by default while the
  standalone source remains as legacy OSG comparison material for
  keyboard-applied external forces.
- The Filament example includes a selectable `--scene coupler-constraint`
  fixture that builds the legacy paired mimic/coupler rig layout and renders
  four link boxes plus six guide lines through descriptor-owned renderables.
  `pixi run ex coupler_constraint` routes to this Filament scene by default
  while the standalone source remains as legacy OSG/ImGui comparison material
  for the status overlay and reset controls.
- The Filament example includes a selectable `--scene add-delete-skels` fixture
  that loads the legacy ground world and adds five deterministic cube skeletons
  through descriptor-owned renderables. `pixi run ex add_delete_skels` routes
  to this Filament scene by default while the standalone source remains as
  legacy OSG comparison material for live q/w add-delete controls.
- The Filament example includes a selectable `--scene vehicle` fixture that
  loads the legacy vehicle SKEL world and renders the car body, wheel
  cylinders, ground, and obstacle boxes through descriptor-owned renderables.
  `pixi run ex vehicle` routes to this Filament scene by default while the
  standalone source remains as legacy OSG comparison material for live throttle
  and steering controls.
- The Filament example includes a selectable `--scene hybrid-dynamics` fixture
  that loads the legacy fullbody SKEL world, applies the same initial humanoid
  pose, and renders the biped plus ground boxes through descriptor-owned
  renderables. `pixi run ex hybrid_dynamics` and `pixi run ex biped_stand`
  route to this Filament scene by default while the standalone sources remain
  as legacy OSG comparison material for scripted joint commands,
  harness toggling, SPD control, and perturbation controls.
- The Filament example includes a selectable `--scene joint-constraints`
  fixture that loads the legacy fullbody SKEL world, applies the standing pose,
  and runs the SPD balance controller through a private scene pre-step hook.
  `pixi run ex joint_constraints` routes to this Filament scene by default
  while the standalone source remains as legacy OSG comparison material for
  perturbation shortcuts and harness toggling.
- The Filament example includes a selectable `--scene free-joint-cases`
  fixture that builds the legacy zero-gravity free-joint scenarios and renders
  each active body plus its transparent torque-free reference body through
  descriptor-owned box renderables. `pixi run ex free_joint_cases` routes to
  this Filament scene by default while the standalone source remains as legacy
  OSG/ImGui comparison material for numeric checks and reference-model
  controls.
- The Filament example includes a selectable `--scene human-joint-limits`
  fixture that loads the legacy Kima human SKEL world, enables DART
  joint-limit enforcement, and renders the posed humanoid plus ground through
  descriptor-owned mesh, multi-sphere, and box renderables.
  `pixi run ex human_joint_limits` routes to this Filament scene by default
  while the standalone source remains as legacy comparison material for the
  custom TinyDNN-backed arm and leg constraints.
- The Filament example includes a selectable `--scene lcp-physics` fixture
  that builds a deterministic contact benchmark layout with mass-ratio boxes,
  a box stack, dominoes, falling spheres, and ground through descriptor-owned
  renderables. `pixi run ex lcp_physics` routes to this Filament scene by
  default while the standalone source remains as legacy OSG/ImGui comparison
  material for solver controls, plots, scenario switching, and frame recording.
- The Filament example includes a selectable `--scene mimic-pendulums` fixture
  that loads the legacy mimic-pendulums SDF world and renders the three
  pendulum rigs plus ground through descriptor-owned renderables.
  `pixi run ex mimic_pendulums` routes to this Filament scene by default while
  the standalone source remains as legacy OSG comparison material for the ImGui
  solver/debug table.
- The Filament example includes a selectable `--scene atlas-puppet` fixture
  that loads Atlas and renders the robot, ground, and four selectable IK target
  spheres through descriptor-owned renderables. `pixi run ex atlas_puppet`
  routes to this Filament scene by default while the standalone source remains
  as legacy OSG comparison material for teleoperation-widget and
  support-polygon behavior.
- The Filament example includes a selectable `--scene hubo-puppet` fixture
  that loads Hubo and renders the robot, ground, and six selectable hand, foot,
  and wrist-peg IK target spheres through descriptor-owned renderables.
  `pixi run ex hubo_puppet` routes to this Filament scene by default while the
  standalone source remains as legacy OSG comparison material for
  teleoperation-widget, support-polygon, and keyboard-control behavior.
- The Filament example includes a selectable `--scene atlas-simbicon` fixture
  that loads the legacy Atlas SDF and ground in the Simbicon start orientation
  through descriptor-owned renderables. `pixi run ex atlas_simbicon` routes to
  this Filament scene by default while the standalone source remains as legacy
  OSG/ImGui comparison material for the gait controller, perturbation
  shortcuts, and panel controls.
- The Filament example includes a selectable
  `--scene operational-space-control` fixture that loads the WAM arm, renders
  the selectable red target sphere, and runs the task-space controller through
  a private scene pre-step hook. `pixi run ex operational_space_control`
  routes to this Filament scene by default while the standalone source remains
  as legacy OSG comparison material for drag-and-drop axis constraints.
- The Filament example includes a selectable `--scene wam-ikfast` fixture that
  loads the WAM arm, ground, and end-effector target through descriptor-owned
  renderables. `pixi run ex wam_ikfast` routes to this Filament scene by
  default while the standalone source remains as legacy OSG comparison material
  for the IKFast solver, drag modes, keyboard shortcuts, and posture reset.
- The Filament example includes a selectable `--scene fetch` fixture that
  loads the legacy MJCF pick-and-place world, applies the initial Fetch robot
  and object pose, and exposes the target frame through descriptor-owned
  renderables. `pixi run ex fetch` routes to this Filament scene by default
  while the standalone source remains as legacy OSG/ImGui comparison material
  for the panel, drag controls, and mocap target update loop.
- The Filament example includes a selectable `--scene tinkertoy` fixture that
  recreates the legacy builder's initial block assemblies, target marker, force
  line, and reference axes through descriptor-owned renderables. The
  `tinkertoy` runner routes to this Filament scene by default while the
  standalone source remains as legacy OSG/ImGui comparison material for the
  panel, mouse picking, and block-add/delete controls.
- The Filament example includes a selectable `--scene drag-and-drop` fixture
  that extracts and renders the legacy drag-and-drop example's `SimpleFrame`
  anchor, child frame, and axis markers through the same manipulation path.
  `pixi run ex drag_and_drop` routes to this Filament scene by default so the
  in-tree drag-and-drop workflow no longer depends on the legacy OSG viewer
  path.
- `pixi run ex empty` also routes to the `--scene drag-and-drop` Filament
  fixture by default while the standalone source remains as legacy OSG
  comparison material for custom world hooks and raw key-event callbacks.
- The Filament example includes a selectable `--scene simple-frames` fixture
  that extracts and renders the legacy simple-frames example's `SimpleFrame`
  hierarchy, marker ellipsoids, and arrow marker through the same
  descriptor-driven Filament path. `pixi run ex simple_frames` routes to this
  Filament scene by default so the simplest frame-visualization workflow no
  longer depends on the legacy OSG viewer path.
- The Filament example includes a selectable `--scene soft-bodies` fixture
  that loads the legacy `softBodies.skel` data through DART IO and renders its
  soft meshes through the same descriptor-driven Filament path.
  `pixi run ex soft_bodies` routes to this Filament scene by default so the
  in-tree soft-body visual workflow no longer depends on the legacy OSG viewer
  path.
- The Filament example includes a selectable `--scene point-cloud` fixture
  that renders the legacy point-cloud example's visual point cloud and, when
  OctoMap is enabled, a voxel-grid cell cluster through descriptor-owned
  point-cloud and voxel-grid renderables. `pixi run ex point_cloud` routes to
  this Filament scene by default while the standalone source remains as legacy
  OSG/ImGui comparison material for the robot-mesh sampling controls.
- The Filament example includes a selectable `--scene capsule-ground-contact`
  fixture that renders the legacy capsule/ground contact setup through
  descriptor-owned capsule and ground renderables, using ODE collision when
  available. `pixi run ex capsule_ground_contact` routes to this Filament
  scene by default while the standalone source remains as legacy OSG comparison
  material for the pose-reset controls.
- The Filament example includes a selectable `--scene simulation-event-handler`
  fixture that renders the legacy simulation event-handler body's falling
  boxes, sphere, ground, and sensor markers through descriptor-owned
  renderables. `pixi run ex simulation_event_handler` routes to this Filament
  scene by default while the standalone source remains as legacy OSG comparison
  material for force/torque controls and force-arrow visualization.
- `pixi run ex imgui` routes to the Filament example's default viewer and
  private built-in status panel by default while the standalone source remains
  as legacy OSG/ImGui comparison material for custom-widget extension points.
- `pixi run ex rigid_shapes` routes to the Filament example's default MVP scene
  by default, covering broad primitive, mesh, point-cloud, heightmap, soft-mesh,
  and robot visuals while the standalone source remains as legacy OSG
  comparison material for shape spawning, contact toggles, and
  collision-detector controls.
- The Filament example includes a `--scene polyhedron` fixture that renders the
  legacy polyhedron visual example's convex hull and wireframe through
  descriptor-owned convex-mesh and line-segment renderables.
  `pixi run ex polyhedron_visual` routes to this Filament scene by default
  while the standalone source remains as legacy OSG comparison material.
- The Filament example includes a selectable `--scene heightmap` fixture that
  renders a local heightmap surface and reference markers through
  descriptor-owned heightmap renderables. `pixi run ex heightmap` routes to
  this Filament scene by default while the standalone source remains as legacy
  OSG/ImGui comparison material for the panel-driven sculpting and
  contact-alignment controls.
- The Filament example includes a selectable `--scene g1` fixture that loads
  the Unitree G1 URDF through DART resource retrievers and exposes colored IK
  targets for both hands and feet. `pixi run ex g1_puppet` routes to this
  Filament scene by default so the in-tree G1 workflow no longer depends on the
  legacy OSG viewer path. The G1 foot targets also register active support
  geometry so the support-polygon debug overlay can be inspected through the
  Filament line-rendering path.
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
  basis/update logic, camera-controller cursor tracking, camera eye
  computation, perspective pick-ray generation, camera-relative directional
  nudge math, GUI scale normalization, and perspective projection/clipping
  descriptor generation used by the Filament example. These viewer-runtime
  helpers are split into `dart/gui/experimental/viewer.hpp` while `scene.hpp`
  remains an aggregate compatibility include.
- `dart-gui-experimental` now keeps the constrained experimental API in
  focused backend-hidden headers: `renderable.hpp`, `interaction.hpp`,
  `debug.hpp`, `geometry.hpp`, and `viewer.hpp`, while `scene.hpp` remains an
  aggregate compatibility include. Implementation is split by responsibility:
  renderable identity/extraction and resource versioning in `renderable.cpp`,
  shape description extraction in `shape_descriptions.cpp`, interaction and
  renderable update planning in `interaction.cpp`, debug descriptor generation
  in `debug.cpp`, viewer runtime helpers in `viewer.cpp`, and backend-hidden
  mesh builders in `geometry.cpp`.
- Backend-specific GLFW/ImGui input polling and status-panel hit testing live
  in `dart/gui/experimental/detail/filament/input.hpp` and `.cpp`, while
  reusable nudge/orbit/manipulation math remains in
  `dart-gui-experimental`.
- The Filament-native ImGui context setup, style scaling, font loading,
  overlay renderer, and draw-data upload live in
  `dart/gui/experimental/detail/filament/imgui_overlay.hpp` and `.cpp`; panel
  contents remain MVP/example-scoped until a later promotion phase needs
  DART-owned panel/tool APIs.
- Filament PNG/JPEG image decoding, texture-cache ownership, sampler setup, and
  PBR material texture parameter binding now live in
  `dart/gui/experimental/detail/filament/textures.hpp` and `.cpp`, with repeat
  sampler construction private to the `.cpp` implementation.
- `UNIT_gui_FilamentSceneExtraction` now also checks that any remaining
  `examples/filament_gui/*.hpp` files have no direct Filament header includes.
- Filament neutral lighting/color grading, orbiting key-light direction,
  scene environment binding, viewport/camera application, and windowed
  view-quality configuration now live in
  `dart/gui/experimental/detail/filament/render_environment.hpp` and `.cpp`.
- Platform-specific GLFW native-window handle selection lives in
  `dart/gui/experimental/detail/filament/native_window.hpp` and `.cpp`.
- Selection label formatting and robot IK-target translation glue live in
  `dart/gui/experimental/detail/filament/selection.hpp` and `.cpp`.
- Filament renderable state, lit-material configuration, shadow flag
  application, and destruction lifecycle helpers now live in
  `dart/gui/experimental/detail/filament/renderable_resources.hpp` and `.cpp`.
- Filament descriptor-to-scene synchronization, unsupported descriptor logging,
  and renderable transform/selection update helpers now live in
  `dart/gui/experimental/detail/filament/renderable_sync.hpp` and `.cpp`.
- `dart-gui-experimental` now owns viewer lifecycle state for pause/step
  behavior, screenshot request tracking, rendered/skipped frame counters, and
  bounded-run stop checks. The Filament example uses this state instead of
  keeping those viewer-loop decisions example-local.
- `dart-gui-experimental` now owns reusable RGBA-to-PPM screenshot storage.
  The Filament renderer readback and capture synchronization now live under
  `dart/gui/experimental/detail/filament/screenshot.hpp` and `.cpp`, while
  file writing delegates to the backend-hidden helper.
- `dart-gui-experimental` now owns renderer-hidden renderable set update
  planning. The Filament example uses that plan to create and destroy Filament
  resources when descriptor snapshots gain, lose, hide, or reveal renderables
  after startup.
- `dartpy.gui.experimental` exposes constrained bindings for renderable
  descriptors, material/geometry descriptors including mesh material/submesh
  metadata, renderable set update planning, picking helpers with bounds hit
  normals and primitive
  sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane
  hit normals,
  free-joint translation, plane/axis-drag helpers, debug-line descriptor
  generation including support-polygon, inertia-box, and collision-shape-bound
  overlays,
  run options including GUI scale normalization, viewer lifecycle state,
  screenshot storage, orbit-camera helpers, orbit-camera controller helpers,
  directional nudge helpers, and perspective projection/clipping helpers.
- `dartpy` can now build the constrained `dartpy.gui.experimental` module with
  `DART_BUILD_GUI=OFF` when `dart-gui-experimental` is present, so the Python
  experimental API is no longer tied to the legacy GUI target.
- `examples/gui_scene_diagnostics` is a second example consumer of the
  experimental API. It exercises descriptors, debug lines, run options, camera
  basis, and picking without introducing another renderer.
- Scene option parsing and dispatch now live in
  `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp`, while
  reusable DART world fixtures live in
  `dart/gui/experimental/detail/filament/scene_fixtures.hpp` and `.cpp`.
- Filament example frame lifecycle, scene synchronization, capture, built-in
  panel wiring, and top-level orchestration now live in
  `dart/gui/experimental/detail/filament/application.hpp` and `.cpp`, leaving
  `examples/filament_gui/main.cpp` as a minimal entry point.
- Debug-line overlay refresh and cleanup now live in
  `dart/gui/experimental/detail/filament/debug_overlay.hpp` and `.cpp`.
- Scene content requirement counting and MVP/G1/hello-world/boxes/
  hardcoded-design/rigid-chain/rigid-loop/mixed-chain/coupler-constraint/
  add-delete-skels/vehicle/hybrid-dynamics/joint-constraints/
  free-joint-cases/human-joint-limits/lcp-physics/mimic-pendulums/atlas-puppet/
  hubo-puppet/atlas-simbicon/operational-space-control/wam-ikfast/fetch/
  tinkertoy/drag/simple-frames/soft-bodies/point-cloud/
  capsule-ground-contact/simulation-event-handler/polyhedron/heightmap
  validation gates now live in
  `dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`.
- Private built-in status panel rendering now lives in
  `dart/gui/experimental/detail/filament/panel.hpp` and `.cpp`; promoted
  panel/tool API design remains deferred.
- Phase 4 is not complete because remaining interaction-heavy workflow
  migration and cross-platform wheel coverage are still pending.

## Phase 5 - First-class promotion

**Deliverables**

- Promote the new API to the primary `dart::gui` API in the appropriate major
  DART version.
- Update maintained examples/tutorials and documentation.
- Keep OSG only as a temporary legacy component if required.
- Extend the reusable public-header backend-token scan from
  `dart/gui/experimental/*.hpp` to promoted `dart/gui/*.hpp` before first-class
  API promotion. The scan must continue rejecting direct backend tokens such as
  `#include <filament/`, `#include "filament/`, `filament::`, `Filament::`,
  `GLFW`, `ImGui`, `imgui`, `OpenGL`, `Vulkan`, `Metal`, `osg::`, `::osg`,
  `osg/`, `osgGA`, `osgViewer`, `Raylib`, `raylib`, and `rlgl`.
  `UNIT_gui_FilamentSceneExtraction` now has the reusable
  `scanHeadersForBackendTokens` helper and a reserved promoted-header
  directory hook for that extension.
- Add an example-level promotion check that maintained examples, including any
  surviving replacement for `examples/filament_gui`, have zero direct
  `#include <filament/...>` or `#include "filament/..."` usage. Examples should
  consume the promoted DART GUI API, not the renderer implementation.
- Add an example-tree promotion check that any surviving `examples/filament_gui`
  directory contains only a minimal `main.cpp` entry point plus unavoidable
  build/docs files. Renderer setup, frame lifecycle, resources,
  synchronization, capture, overlays, input translation, and fixture logic
  should live in `dart::gui` or private GUI implementation units.
  `UNIT_gui_FilamentSceneExtraction` now enforces the current incremental form:
  only `CMakeLists.txt`, `README.md`, and `main.cpp` may remain in the example
  tree, and `main.cpp` must stay a single-include delegating entry point with no
  direct backend implementation tokens.

**Exit criteria**

- Maintained examples/tutorials and `dartpy` have a supported non-OSG
  visualization path.
- Maintained examples no longer include Filament headers directly.
- Any surviving `examples/filament_gui` tree remains a minimal DART GUI entry
  point rather than owning renderer architecture.
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
