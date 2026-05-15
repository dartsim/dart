# Filament GUI Replacement - Dev Task

## Current status

- The shipped `dart-gui` library is still the OpenSceneGraph (OSG) backend, and
  public headers expose OSG types.
- The existing Raylib work is only a smoke example and is no longer the
  preferred long-term replacement direction.
- Experimental Filament build plumbing and `examples/filament_gui` now exist
  behind `DART_BUILD_GUI_FILAMENT=ON`.
- `dart-gui-experimental` now provides a backend-hidden scene extraction layer
  that converts DART visual shape nodes and world-owned `SimpleFrame` visuals
  into renderable descriptors. Focused unit tests cover descriptor extraction
  without a graphics context.
- The example renderer consumes box, sphere, ellipsoid, cylinder, cone,
  capsule, pyramid, multi-sphere, line-segment, convex-mesh,
  point-cloud, heightmap, soft-mesh, OctoMap-backed voxel-grid,
  TriMesh-backed `MeshShape`, and finite `PlaneShape` descriptors, and applies
  DART visual-aspect shadow flags to Filament renderables. Convex mesh,
  heightmap, soft mesh, and `MeshShape` rendering now consume descriptor-owned
  triangle data instead of reaching back into concrete shape objects, and
  point-cloud rendering now consumes descriptor-owned per-point colors when
  present.
- The example configures Filament view quality features needed by the MVP gate:
  cascaded SUN shadows, contact shadows for windowed runs, deterministic PCF
  shadows for headless screenshots, a neutral skybox, spherical harmonics
  indirect lighting, high-quality color grading, anisotropic texture sampling,
  HDR buffer quality, screen-space ambient occlusion, temporal and multi-sample
  anti-aliasing, FXAA, and temporal dithering. The higher-cost post-process
  options are kept to windowed runs because they prevent frame acquisition on
  the current headless OpenGL software path.
- The fixture includes a procedural checker-textured plane path through a
  Filament sampler material. It also loads an existing textured WAM Collada
  mesh, the full WAM URDF skeleton, a required Atlas DAE torso mesh, and a full
  Atlas SDF robot fixture, and a required four-panel glTF/PBR environment
  layout. The experimental descriptors expose `MeshShape` material base
  colors, metallic/roughness factors, emissive colors, UV coordinate data,
  imported vertex normals, submesh ranges, and typed PNG/JPEG texture image
  paths for base color, metallic, roughness, combined metallic-roughness,
  normal, occlusion, and emissive maps when available, and the Filament example
  consumes that descriptor metadata.
  Checked-in glTF PBR fixtures now validate single- and multi-material authored
  texture slots, alpha-bearing material factors, UV metadata, and submesh
  ranges through the real Assimp importer and are loaded by the Filament smoke
  scene.
  Transparent lit material variants cover alpha-bearing solid, textured, and
  mesh visual paths, and `MeshShape` alpha-mode policy now flows through
  descriptors so Filament can match DART's material-alpha behavior; broader
  robot/environment visual review is still pending.
- `dart-gui-experimental` now exposes backend-hidden debug line descriptors for
  a 3D grid, world/body frames, center-of-mass markers, contact markers,
  inertia boxes, collision-shape bounds, contact normal arrows, contact force
  arrows, support-polygon outlines, and support-centroid markers; the Filament
  example translates those descriptors into line primitives.
- Backend-hidden mesh builders for box, sphere/ellipsoid, cylinder, capsule,
  cone, pyramid, multi-sphere, point boxes, and triangle descriptors now live
  in `dart/gui/experimental/geometry.hpp` and `.cpp`; the Filament example is a
  consumer of those generated mesh buffers instead of owning that tessellation
  code locally.
- The constrained experimental public surface is split into focused
  backend-hidden headers: `renderable.hpp`, `interaction.hpp`, `debug.hpp`,
  `geometry.hpp`, `viewer.hpp`, and `profile.hpp`. `scene.hpp` remains an
  aggregate compatibility include, while implementation now separates shape
  description extraction in `shape_descriptions.cpp` from renderable
  identity/resource extraction in `renderable.cpp`.
- The experimental interaction layer includes tested picking bounds and nearest
  ray-hit selection helpers with hit point, bounds-normal, primitive-surface,
  and triangle-backed mesh surface reporting. The example uses them for basic
  click-to-select highlighting and a renderer-independent selection bounds
  overlay without exposing Filament types.
- The experimental interaction layer includes tested free-joint, simple-frame,
  and combined frame-renderable translation helpers for selected renderables.
  The Filament example uses the combined path with camera-relative nudge math
  for keyboard nudging of selected dynamic bodies and `SimpleFrame` visuals
  without exposing Filament input types.
- The experimental interaction layer includes tested plane intersection and
  plane-drag translation helpers. The Filament example uses them for Ctrl-left
  camera-plane dragging of selected dynamic bodies and `SimpleFrame` visuals
  without exposing Filament input types.
- The Filament example's GLFW/ImGui input bridge now lives in
  `dart/gui/experimental/detail/filament/input.hpp` and `.cpp`, keeping
  backend-specific mouse/controller input, orbit-camera scroll callback
  attachment, application hotkey polling, and pause/single-step/IK target
  key-edge state out of the main viewer loop while the reusable manipulation
  math remains in `dart-gui-experimental`.
- The Filament-native ImGui context setup, style scaling, font loading,
  current IO access, overlay renderer, and draw-data upload now live in
  `dart/gui/experimental/detail/filament/imgui_overlay.hpp` and `.cpp`; the
  built-in panel contents remain MVP/example-scoped and are still not a
  promoted panel/tool API.
- The Filament example's per-frame UI input sync, built-in status panel
  dispatch, debug-overlay refreshes caused by panel toggles, and ImGui overlay
  upload now live in `dart/gui/experimental/detail/filament/ui_frame.hpp` and
  `.cpp`.
- Ordered teardown of the private Filament application resources now lives in
  `dart/gui/experimental/detail/filament/application_teardown.hpp` and `.cpp`.
- The experimental viewer-runtime layer owns backend-hidden run-option
  normalization, viewer lifecycle state, orbit-camera math, orbit-camera
  controller state, viewer profiling accumulation, and perspective
  projection/clipping descriptors used by the example for bounded screenshots,
  camera placement, headless mode, pause/step behavior, frame accounting, GUI
  scale normalization, perspective pick rays, near/far plane policy, and
  profile reporting. Those viewer-runtime helpers now live in
  `dart/gui/experimental/viewer.hpp` and
  `dart/gui/experimental/profile.hpp`, with core viewer helpers re-exported by
  `scene.hpp` for existing experimental consumers.
- Renderer resource synchronization planning now compares active renderable
  render-resource versions, so descriptor-owned geometry changes, including
  dynamic soft-mesh vertex changes, `MeshShape` material-color policy changes,
  and alpha-only visual changes, recreate Filament resources without exposing
  renderer handles.
- Supported descriptor kinds that cannot produce renderer resources, such as
  empty point clouds or meshes without triangle data, now carry diagnostic
  reasons and are logged once by the Filament example instead of failing
  silently.
- Retained Filament renderables now reapply descriptor shadow flags each frame,
  so DART visual-aspect shadow changes are not limited to resource creation.
- The experimental viewer-runtime layer owns reusable RGBA-to-PPM screenshot
  storage, while `dart/gui/experimental/detail/filament/screenshot.hpp` and
  `.cpp` isolate the Filament example's renderer readback, capture
  synchronization, wait/save finalization, and screenshot profile accounting
  behind the Filament render context, so the screenshot helper header does not
  expose Filament renderer types.
  The current Filament readback path writes top-left-origin screenshots so
  headless captures are directly reviewable without a manual vertical flip.
- The Filament example's PNG/JPEG texture loading, Filament texture cache, and
  PBR texture parameter wiring now live in
  `dart/gui/experimental/detail/filament/textures.hpp` and `.cpp`; repeat
  sampler construction stays private to the texture implementation so the
  helper header does not include Filament sampler headers.
- The Filament descriptor-to-Filament renderable factory now lives in
  `dart/gui/experimental/detail/filament/renderable_factory.hpp` and `.cpp`,
  keeping generated mesh buffers, material part assembly, debug-line resource
  creation, and texture-backed mesh binding out of the main viewer loop.
- `UNIT_gui_FilamentSceneExtraction` now checks that any remaining
  `examples/filament_gui/*.hpp` files and the example entry point have no
  direct Filament header includes, while the full north-star metric remains
  zero direct Filament header includes from maintained examples after
  promotion.
- The backend-token scan now uses a reusable
  `scanHeadersForBackendTokens` helper with an explicit future hook for
  promoted `dart/gui/*.hpp` headers once the first-class Filament API replaces
  the legacy OSG-shaped public GUI headers.
- The full north-star also requires any surviving `examples/filament_gui/`
  tree to shrink to a minimal executable entry point: renderer setup, frame
  lifecycle, material and texture resources, scene synchronization, capture,
  overlays, input translation, and reusable fixture logic should live in
  `dart::gui` or private GUI implementation units rather than as example-local
  architecture.
- The Filament example's engine, renderer, swap-chain, main view, scene,
  camera lifecycle, and begin/render/end frame calls now live in
  `dart/gui/experimental/detail/filament/render_context.hpp` and `.cpp`,
  leaving `main.cpp` without direct Filament header includes and moving another
  backend implementation slice out of the example tree.
- The Filament example's neutral lighting, light entity creation and orbit
  update, scene environment binding, color grading, viewport/camera
  configuration helper, and windowed view-quality setup now live in
  `dart/gui/experimental/detail/filament/render_environment.hpp` and `.cpp`,
  moving another backend implementation slice out of the example tree.
- The Filament example's per-frame framebuffer sizing, ImGui display metrics,
  viewport/camera update dispatch, and camera-controller suppression while
  selection or built-in UI consumes the pointer now live in
  `dart/gui/experimental/detail/filament/frame_viewport.hpp` and `.cpp`.
- The Filament example's GLFW initialization/window lifecycle,
  window-close loop predicate, and platform-specific native-window handle
  selection now live in
  `dart/gui/experimental/detail/filament/native_window.hpp` and `.cpp`.
- The Filament example's selection labels, G1 IK-target translation glue, and
  private selection controller state now live in
  `dart/gui/experimental/detail/filament/selection.hpp` and `.cpp`. That
  controller owns keyboard nudging, click selection, and Ctrl-left drag event
  translation while renderer-independent selection and translation primitives
  remain in `dart-gui-experimental`.
- The Filament material bundle, seed texture resources, renderable state,
  material selection and parameter binding helpers, shadow settings, and
  destruction lifecycle now live in
  `dart/gui/experimental/detail/filament/renderable_resources.hpp` and `.cpp`.
- The Filament descriptor-to-scene synchronization, unsupported descriptor
  logging, initial scene renderable creation, scene entity attachment, and
  per-frame renderable transform/selection/shadow update helpers now live in
  `dart/gui/experimental/detail/filament/renderable_sync.hpp` and `.cpp`.
- The Filament debug-line overlay option defaults, static/contact
  refresh/cleanup state, and selected-renderable overlay lookup helpers now
  live in `dart/gui/experimental/detail/filament/debug_overlay.hpp` and `.cpp`.
- The Filament example's scene option parsing and reusable DART world fixtures
  now live in `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp`;
- The Filament example's scene content requirement counting and MVP/G1/drag
  validation gates, including created-renderable content counting, now live in
  `dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`;
- Initial scene extraction, startup validation, first renderable
  synchronization, and startup debug-overlay validation now live in
  `dart/gui/experimental/detail/filament/scene_startup.hpp` and `.cpp`.
- The Filament example's frame lifecycle, scene synchronization, capture,
  and top-level orchestration now live in
  `dart/gui/experimental/detail/filament/application.hpp` and `.cpp`, leaving
  `examples/filament_gui/main.cpp` as a minimal entry point guarded by the
  focused scene-extraction unit. The application unit now depends on the
  private GUI runtime surfaces directly and leaves DART scene fixture/import
  dependencies in `scenes.hpp` and `.cpp`.
- Filament frame rendering, skipped-frame accounting, screenshot request
  dispatch, rendered-frame profiling, and bounded-run stop checks now live in
  `dart/gui/experimental/detail/filament/frame_renderer.hpp` and `.cpp`.
- The Filament example's bounded realtime simulation step-count accumulator,
  world-step application, lifecycle marking, and simulation profile accounting
  now live in `dart/gui/experimental/detail/filament/simulation_stepper.hpp`
  and `.cpp`.
- The Filament example's built-in status panel rendering now lives in
  `dart/gui/experimental/detail/filament/panel.hpp` and `.cpp`; it remains
  private MVP policy rather than a promoted panel/tool API.
- `dartpy.gui.experimental` now exposes the constrained backend-hidden
  descriptor, picking, frame-translation, debug-line, run-option, viewer
  lifecycle, screenshot storage, orbit-camera, orbit-camera controller,
  directional nudge, and projection/clipping APIs used by the current C++
  experiment.
- The constrained `dartpy.gui.experimental` module can build and pass its
  focused test with `DART_BUILD_GUI=OFF` when the experimental GUI target is
  present, so the Python descriptor API no longer depends on the legacy GUI
  target.
- `examples/gui_scene_diagnostics` now provides a second, non-rendering
  consumer of `dart-gui-experimental` for descriptor/debug/camera API
  diagnostics, including a `SimpleFrame` visual, without adding another
  rendering backend.
- The Filament example's built-in panel now provides pause/step controls and
  debug overlay toggles for grid, frames, center-of-mass markers, contacts,
  inertia boxes, collision-shape bounds, support polygons, normal arrows, and
  force arrows.
- The Filament example also includes `--scene drag-and-drop`, a first
  interaction-heavy fixture that carries the legacy `SimpleFrame` anchor, child
  frame, and axis marker layout through the same backend-hidden manipulation
  path. The in-tree `pixi run ex drag_and_drop` runner now routes to that
  Filament scene while the legacy standalone OSG source remains available for
  comparison until the promoted GUI API replaces the old viewer path.
- The Filament example includes `--scene polyhedron`, a convex-hull and
  wireframe fixture that carries the legacy `polyhedron_visual` example through
  descriptor-owned convex-mesh and line-segment renderables. The in-tree
  `pixi run ex polyhedron_visual` runner now routes to that Filament scene
  while the legacy standalone OSG source remains available for comparison.
- The Filament example now also includes `--scene g1`, which loads the remote
  Unitree G1 URDF through DART's normal resource-retriever path and exposes
  colored IK targets for both hands and feet. The in-tree
  `pixi run ex g1_puppet` runner now routes to that Filament scene while the
  legacy standalone source remains available for comparison. The G1 scene is
  intentionally not part of `--scene all` because it fetches remote robot
  assets by default.
- The Filament example can render bounded screenshots through Filament's
  headless swap-chain path without creating a GLFW window.
- `DART_ENABLE_FILAMENT_GUI_SMOKE_TESTS=ON` registers an opt-in CTest smoke
  check for the headless screenshot path. This is a CI hook, not a default test
  yet.
- The headless smoke check validates more than a nonblank image: it also samples
  the shadowed fixture region for dark, mid-tone, and bright pixels plus a
  minimum luminance spread.
- Filament is the preferred renderer candidate for the next experiment because
  it provides modern real-time rendering features while keeping DART's built-in
  visualization scope maintainable.
- The Filament example now links through `dart-gui-experimental` and can build
  without the legacy OSG `dart-gui` target when `DART_BUILD_GUI=OFF` and
  `DART_BUILD_DARTPY=OFF`. The example now includes the
  `dart::gui::experimental` API directly instead of carrying an example-local
  namespace re-export header.
- Checked-in dartpy stubs and Python API docs now list
  `dartpy.gui.experimental` and include a backend-hidden stub surface for
  descriptor, picking, debug-line, run-loop, camera, camera-controller,
  directional nudge, and screenshot helpers.
- Promotion requires explicit visual-quality gates, including dynamic shadows.
- The current Pixi environment does not provide Filament. The upstream Filament
  Linux archive can be discovered with `Filament_ROOT`. It links and runs in
  this checkout when compatible libc++/libc++abi libraries are added, but that
  is still a local workaround rather than a supported packaging path.
- `pixi run test-filament-gui-smoke` now exercises the explicit pinned
  `DART_FETCH_FILAMENT=ON` path on Linux x86_64, builds without the legacy OSG
  GUI target, and runs the default, drag-and-drop, and polyhedron headless
  CTest smokes. When no display server is available, the task runs the smokes
  under Xvfb and prefers Mesa's EGL vendor file for software rendering. The
  Ubuntu CI workflow has a matching opt-in smoke job that gets Xvfb, libc++,
  and libc++abi from
  system packages instead of relying on a Filament package. The MVP PR #2647
  merged with hosted `Filament GUI Smoke (GCC)` and
  `Filament GUI Smoke (Clang)` passing.
- Local Linux CPython 3.12, 3.13, and 3.14 wheel builds now repair with
  `auditwheel` and pass installed-wheel testing. The smoke confirms
  `dartpy.gui.experimental` is present in the wheel and can extract a one-box
  renderable descriptor.
- Full default validation passes locally with
  `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 pixi run test-all`, covering
  lint, Release/Debug builds, C++ tests, simulation-experimental tests, Python
  tests, and docs. The default `UNIT_gui_HeadlessViewer` test was tightened to
  avoid creating a second headless pbuffer in one process after that path
  exposed a local Mesa GLX hang during full-suite validation.
- A ready-for-review conda-forge staged-recipes PR exists for Filament 1.71.3.
  Its current head is still open and behind the target branch, but the latest
  inspected head `6b20da5` is green across staged-recipes linter,
  conda-forge-linter, Check Skip, Azure linux_64, osx_64, win_64, and aggregate
  checks. It has not produced an installable feedstock yet, but its planned
  `filament-static` output is the build-time package DART should consume once
  it is available.
- The north-star migration plan now records the intended complete migration to
  Filament, the no-renderer-backward-compatibility stance, the cleanup of
  multi-backend-only abstraction layers, the OSG and alternative-renderer
  capability gaps that still need DART-owned concepts, and the multi-phase path
  from MVP example to promoted `dart::gui`.
- The legacy surface audit now maps the current public OSG GUI headers, legacy
  dartpy GUI bindings, Raylib build/example support, and maintained examples
  into DART-owned concepts to keep, renderer details to make private, and
  renderer-specific surfaces to remove or explicitly leave unsupported.
- Onboarding docs now identify OSG and Raylib as legacy current-state details,
  route new GUI work to the Filament north-star plan, and warn against public
  APIs or abstractions whose only purpose is supporting multiple renderers.
- The GUI and Python module `AGENTS.md` files now carry the same migration
  guardrails for future local edits: legacy OSG APIs are current-state only,
  experimental Filament concepts must stay backend-hidden, and new public
  bindings should use DART-owned concepts.

## Goal

Evaluate and, if it passes explicit gates, promote a Filament + GLFW + private
Dear ImGui implementation as DART's built-in visualization solution under
`dart::gui`, replacing the current main OSG GUI, the Raylib smoke path, and any
temporary experimental visualization paths in the appropriate major DART
release while deleting or collapsing multi-backend abstraction layers that no
longer express stable DART concepts.

## Non-goals

- Supporting multiple maintained rendering backends.
- Building a general-purpose production renderer or content pipeline as a
  built-in DART feature.
- Exposing Filament, GLFW, Dear ImGui, OpenGL, Vulkan, or Metal types in public
  DART headers.
- Preserving OSG-specific extension points or renderer source compatibility by
  default.

## Key decisions

- Keep the public namespace and component identity as `dart::gui`; do not invent
  a separate `dart::viz` user-facing surface.
- Start with a standalone `examples/filament_gui` MVP before adding a public
  API or library target.
- Use DART-owned scene/debug concepts between simulation objects and Filament
  while the API is experimental. This keeps simulation-side behavior testable
  and prevents renderer details from leaking, but promotion should delete or
  collapse indirection that only exists to preserve multiple renderer backends.
- Prefer conda-forge/Pixi packaged Filament once available. A pinned CMake fetch
  fallback is available for Linux x86_64 experiments only when explicitly
  requested and should not be the primary release path.
- Keep Dear ImGui internal for debug panels and controls.

## Documents

- Renderer selection: `docs/dev_tasks/filament_gui/00-renderer-selection.md`
- Architecture and API boundaries: `docs/dev_tasks/filament_gui/01-architecture.md`
- MVP example plan: `docs/dev_tasks/filament_gui/02-mvp-example.md`
- Milestones and promotion gates: `docs/dev_tasks/filament_gui/03-milestones.md`
- Migration and deprecation plan: `docs/dev_tasks/filament_gui/04-migration.md`
- Testing, CI, and packaging gates: `docs/dev_tasks/filament_gui/05-testing.md`
- Visual quality requirements: `docs/dev_tasks/filament_gui/06-visual-quality.md`
- Completion audit: `docs/dev_tasks/filament_gui/07-completion-audit.md`
- North-star migration plan: `docs/dev_tasks/filament_gui/08-north-star-migration.md`
- Legacy surface audit: `docs/dev_tasks/filament_gui/09-legacy-surface-audit.md`
- Resume prompt: `docs/dev_tasks/filament_gui/RESUME.md`
- Live supervisor steering: `docs/dev_tasks/filament_gui/STEERING.md`

## Immediate next steps

1. Use the north-star migration plan as the gate for any new public GUI API:
   DART-owned concepts only, no public backend types, and no OSG renderer
   source-compatibility promise. Use the legacy surface audit to decide whether
   an existing GUI surface becomes a stable DART concept, a private Filament
   detail, or a removed/unsupported renderer-specific API. Keep the experimental
   public-header leakage test passing; its reusable scan should add promoted
   `dart/gui/*.hpp` headers when APIs move into first-class `dart::gui`.
2. Keep the hosted Ubuntu `Filament GUI Smoke (GCC)` and
   `Filament GUI Smoke (Clang)` jobs green on each follow-up PR when the
   explicit pinned fetch path or Filament example behavior changes.
3. Add broader human visual review and larger authored environment/PBR assets
   for the visual-quality gate. The current screenshot analyzer is a smoke check
   for shadow/lighting contrast, not a replacement for broader visual review.
4. Track the conda-forge staged-recipes PR as the preferred future packaging
   path, but do not block Linux smoke coverage on it. After it merges, add the
   `filament-static` package to the Pixi toolchain and validate that
   `Filament_ROOT=$CONDA_PREFIX` discovers headers, libraries, and `matc`.
5. Expand debug overlay and interaction scenario coverage beyond the MVP,
   drag-and-drop, and G1 IK fixtures.
6. Continue moving reusable renderer resource management out of the Filament
   example once the API boundary is stable enough.
7. Keep the MVP ImGui panel/tool policy example-scoped unless promotion needs
   user extension points; then add DART-owned panel/tool abstractions instead
   of exposing raw ImGui APIs.
8. Complete remaining platform and GUI option-matrix wheel evidence before
   promoting anything to `dart-gui`.
9. Keep onboarding docs aligned with the north-star plan as API names and
   migration gates move from experimental to promoted `dart::gui`.
