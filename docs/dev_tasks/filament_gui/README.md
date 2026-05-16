# Filament GUI Replacement - Dev Task

## Current status

This task is active again on the promotion branch. Filament with GLFW3 and Dear
ImGui is the only official renderer direction, and legacy OpenSceneGraph/Raylib
renderer surfaces have been removed from the branch. The next execution slice is
tracked in `10-active-execution.md`: keep restoring the pre-existing
user-facing examples as `dart::gui` examples, restore the historical `--out

<dir>` image-sequence capture contract, and continue removing stale
`experimental`/backend-named concepts from promoted surfaces. The
backend-named MVP executable has already moved to the application-level
`dartsim` identity because Filament is no longer an alternate backend. `DART`
remains the project/library identity, and `libdart` remains the
packaging/library context name. `dartsim` is the application-level
simulator/viewer brand, analogous to an application product such as Isaac Sim.

The current source-ownership pass is not the same as full example parity.
Maintainer review called out `examples/fetch/` as a concrete example that had a
real source file but was not fully restored to the historical GUI behavior.
Every pre-existing user-facing GUI example remains parity pending until its
legacy source has been inventoried and the migrated `dart::gui` version
preserves the user-visible behavior or records a deliberate follow-up gap.
The Fetch checkpoint has now restored the historical camera framing, made the
visible target cross the draggable target-frame affordance, and restored the
historical work-area grid offset as source-owned public DART geometry because
the example-specific grid is another user-visible behavior from the old OSG
source. The active robot/IK parity work started with
`g1_puppet`, `atlas_puppet`, and `hubo_puppet`, because those examples still
have clear gaps around target handles, target activation/teleoperation, and
whole-body IK behavior compared with the historical OSG sources. The first
robot/IK parity slice restores line-segment target handles and updated control
text for those three examples, while explicitly leaving solver-objective and
full hotkey parity as remaining audit work. The next target-affordance slice
applies the same visible-handle pattern to `operational_space_control`,
`wam_ikfast`, and `tinkertoy` while tracking Tinkertoy's larger builder
workflow gaps separately. That handle slice and the Fetch grid slice have been
pushed. The Tinkertoy builder and keyboard checkpoints now restore the
stateful builder workflow through promoted `dart::gui` concepts: selected
block state, add/delete panel controls, gravity and force-coefficient controls,
target reorientation, force-line updates, external force application,
collision/dynamics block construction, the `1`/`2`/`3`, Backspace, Delete,
Up/Down, and backtick hotkeys, and Tab camera-home. Enter recording remains an
explicit follow-up gap until a public recording API exists.

The next active slice uses the renderer-neutral keyboard action surface for
robot/IK behavior parity. The first target is Atlas/Hubo whole-body puppet
behavior that can be restored through promoted `dart::gui`: continuous IK
solving plus WASD/QE/FZ root teleoperation. That slice is implemented locally
for `atlas_puppet` and `hubo_puppet` and has been pushed. Atlas
relaxed-posture/balance optimization, Hubo analytical IK, and Atlas/Hubo target
activation/deactivation semantics remain tracked parity gaps unless a later
slice adds the narrow public API needed to implement them cleanly. The next
active slice closes Tinkertoy's Tab
camera-home gap by adding a renderer-neutral camera reset callback to
`KeyboardActionContext`; that callback and Tinkertoy Tab binding are
implemented and pushed. The next active slice restores G1's historical
number-key target activation/deactivation semantics and active-target pre-step
solving through source-owned public `dart::gui` actions; it is implemented and
pushed. The next source-owned restoration slice re-audits `examples/fetch/`
against the historical OSG source, because prior Fetch evidence proves
camera/grid/capture smoke parity but does not by itself prove the full example
was restored. That Fetch re-audit found the remaining first-slice gap in the
legacy panel affordances; Exit, Play/Pause, Help, and About controls are now
implemented and pushed through public `dart::gui` lifecycle/panel APIs. The
next source-owned restoration slice is `examples/add_delete_skels/`, restoring
the historical `q`/`w` keyboard controls, Bullet preference, and camera default
through public `dart::gui`; that checkpoint is implemented and pushed. The
next bounded restoration slice is `examples/mixed_chain/`, restoring the
historical six impulse keyboard controls and camera default through public
`dart::gui`; that checkpoint is implemented and pushed. The next bounded
restoration slice is `examples/vehicle/`, restoring the historical
`w`/`s`/`x`/`a`/`d` command keys and camera default through public
`dart::gui`; that checkpoint is implemented and pushed. The next bounded
restoration slice is `examples/soft_bodies/`, restoring the historical
`[`/`]`/`{`/`}`/`r`/`\` playback keys through public `dart::gui`; that
checkpoint is implemented and pushed. Enter recording remains a separate
capture/session API gap. The next bounded restoration slice is
`examples/hybrid_dynamics/`, restoring the historical `h` harness toggle and
camera default through public `dart::gui`; that checkpoint is implemented and
pushed. The next bounded restoration slice is `examples/joint_constraints/`,
restoring the historical `1`-`4` perturbation keys, `h` harness toggle, and
camera default through public `dart::gui`; that checkpoint is implemented and
pushed. The immediate correction is to re-open `examples/fetch/` as a full
historical-source parity audit, not a source-ownership audit. Earlier Fetch
checkpoints restored the promoted source file, camera, target cross,
work-area grid, and panel controls, but Fetch remains the concrete reminder
that screenshots and a real `main.cpp` are not enough evidence. Any remaining
Fetch user-visible behavior gap should be restored through promoted
`dart::gui` APIs or recorded with the public API still needed. The Fetch
re-audit checkpoint now restores the missing example README and adds
renderer-neutral `ApplicationOptions::runDefaults` so the source-owned example
can recover the historical 1280x960 default launch size while preserving
command-line overrides. The next active parity slice is `examples/rigid_shapes`:
restore the historical shape-spawn/delete/contact keyboard controls, convex
mesh spawn path, contact-point visualization, camera home, and 640x480 default
launch size through public `dart::gui`, plus the rigid-shapes-specific
collision-detector, max-contact, and ground-thickness command-line options;
that checkpoint is implemented and pushed. The next active parity slice is
`examples/biped_stand`: restore the historical `1`-`4` perturbation keys,
camera home, 640x480 default launch size, and README through promoted
`dart::gui`. That checkpoint is implemented and locally validated; the next
checkpoint was pushed as
`90b373beb14 Restore biped stand controls`. The next active parity slice is
`examples/free_joint_cases`: restore the historical numeric/reference controls,
local CLI flags, camera/run defaults, and README through promoted `dart::gui`.
That checkpoint is implemented and locally validated through the pre-lint
focused build, CTest, direct/pixi screenshot smokes, Python example-runner
tests, the aggregate `examples` build, `pixi run lint`, and post-lint focused
build/CTest/direct screenshot smoke. The checkpoint was pushed as
`f4963df00cd Restore free joint cases controls`. The next active parity slice
is `examples/lcp_physics`: restore the historical scenario/solver/list CLI,
camera/run defaults, panel context, and README through promoted `dart::gui`.
That checkpoint is implemented and locally validated through the pre-lint
focused build, CTest, `--list`, direct/pixi screenshot smokes, Python
example-runner tests, the aggregate `examples` build, `pixi run lint`, and
post-lint focused build/CTest/direct screenshot smoke. The checkpoint was
pushed as `619af5649bc Restore LCP physics controls`. The next active parity
slice is `examples/mimic_pendulums`: restore baseline retargeting, mimic
diagnostics, local solver/collision flags, camera/run defaults, and README
through promoted `dart::gui`. That checkpoint is implemented and locally
validated through the pre-lint focused build, CTest, direct/pixi screenshot
smokes, Python example-runner tests, the aggregate `examples` build,
`git diff --check`, `pixi run lint`, and post-lint focused
build/CTest/direct screenshot smoke. The checkpoint was committed and pushed
as `2bc6a0e168d Restore mimic pendulums diagnostics`. The next active parity
slice is `examples/box_stacking`: restore solver selection, camera/run
defaults, README, and parity tests through promoted `dart::gui`. That
checkpoint is implemented and locally validated through the pre-lint focused
build, CTest, direct/pixi screenshot smokes, Python example-runner tests, the
aggregate `examples` build, `git diff --check`, `pixi run lint`, and
post-lint focused build/CTest/direct screenshot smoke. The checkpoint was
committed and pushed as `3493d7065c0 Restore box stacking solver controls`.
The next active parity slice is `examples/boxes`: restore the historical
Bullet preference, camera/run defaults, README, and parity tests through
promoted `dart::gui`. That checkpoint is implemented and locally validated
through the pre-lint focused build, CTest, direct/pixi screenshot smokes,
Python example-runner tests, the aggregate `examples` build,
`git diff --check`, `pixi run lint`, and post-lint focused
build/CTest/direct screenshot smoke. The checkpoint was committed and pushed
as `74870cc5cf3 Restore boxes run defaults`. The next active parity slice is
`examples/simple_frames`: restore camera/run defaults, README, and parity tests
through promoted `dart::gui`. That checkpoint is implemented and locally
validated through the pre-lint focused build, CTest, direct/pixi screenshot
smokes, Python example-runner tests, the aggregate `examples` build,
`git diff --check`, `pixi run lint`, and post-lint focused
build/CTest/direct screenshot smoke. The remaining work is commit and push.

The completion audit in `07-completion-audit.md` describes an earlier
promotion checkpoint. The detailed progress notes below are retained as
migration history and may describe intermediate states that no longer exist in
the source tree.

- `DART_BUILD_GUI` now builds the Filament-backed GUI component, and
  `DART_BUILD_GUI_FILAMENT` builds the official Filament visual runner.
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
- The experimental interaction layer includes tested plane intersection,
  plane-drag translation, and axis-drag translation helpers. The Filament
  example uses the plane helpers for Ctrl-left camera-plane dragging of
  selected dynamic bodies and `SimpleFrame` visuals without exposing Filament
  input types, while the axis helper covers the backend-hidden primitive needed
  for future constrained manipulators.
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
  tree to contain only a minimal `main.cpp` executable entry point, plus
  unavoidable build/docs files: renderer setup, frame lifecycle, material and
  texture resources, scene synchronization, capture, overlays, input
  translation, and reusable fixture logic should live in `dart::gui` or
  private GUI implementation units rather than as example-local architecture.
  The Filament example CMake helper now compiles only that `main.cpp` entry
  point and rejects unexpected example-tree regular files during configure,
  leaving only `CMakeLists.txt`, `README.md`, and `main.cpp`. It also rejects
  direct Filament header includes from the entry point, so the build plumbing
  enforces the same boundary as the focused unit guard.
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
- The Filament example's selection labels, robot IK-target translation glue, and
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
- The Filament example's scene option parsing and dispatch now live in
  `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp`, while
  reusable DART world fixtures now live in
  `dart/gui/experimental/detail/filament/scene_fixtures.hpp` and `.cpp`;
- The Filament example's scene content requirement counting and
  MVP/G1/hello-world/boxes/hardcoded-design/rigid-chain/rigid-loop/
  mixed-chain/coupler-constraint/add-delete-skels/vehicle/hybrid-dynamics/
  joint-constraints/free-joint-cases/human-joint-limits/lcp-physics/
  mimic-pendulums/atlas-puppet/hubo-puppet/atlas-simbicon/
  operational-space-control/wam-ikfast/fetch/tinkertoy/drag/
  simple-frames/
  soft-bodies/point-cloud/capsule-ground-contact/simulation-event-handler/
  polyhedron/heightmap
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
  dependencies in `scene_fixtures.hpp` and `.cpp`.
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
  descriptor, picking, frame-translation, plane/axis drag, debug-line,
  run-option, viewer lifecycle, screenshot storage, orbit-camera, orbit-camera
  controller, directional nudge, and projection/clipping APIs used by the
  current C++ experiment.
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
- The Filament example includes `--scene hello-world`, a simplest dynamic box
  and ground-plane fixture that carries the legacy `hello_world` example
  through descriptor-owned box renderables. The in-tree
  `pixi run ex hello_world` runner now routes to that Filament scene while the
  legacy standalone OSG source remains available for comparison.
- The Filament example includes `--scene boxes`, a dynamic colored box-grid
  fixture that carries the legacy `boxes` example through descriptor-owned box
  renderables. The in-tree `pixi run ex boxes` runner now routes to that
  Filament scene while the legacy standalone OSG/Bullet source remains
  available for comparison.
- The in-tree `pixi run ex rigid_cubes` runner now routes to the same
  `--scene boxes` Filament fixture while the legacy standalone OSG source
  remains available for directional force controls and frame-recording
  comparison.
- The in-tree `pixi run ex box_stacking` runner now routes to the same
  `--scene boxes` Filament fixture while the legacy standalone OSG/ImGui source
  remains available for solver selection, gravity controls, and custom-key
  callback comparison.
- The Filament example includes `--scene hardcoded-design`, a hand-built
  skeleton fixture that carries the legacy `hardcoded_design` example through
  descriptor-owned box renderables. The in-tree
  `pixi run ex hardcoded_design` runner now routes to that Filament scene while
  the legacy standalone OSG source remains available for wireframe and
  key-controlled joint comparison.
- The Filament example includes `--scene rigid-chain`, a SKEL-loaded chain
  fixture that carries the legacy `rigid_chain` example through
  descriptor-owned box-link renderables. The in-tree `pixi run ex rigid_chain`
  runner now routes to that Filament scene while the legacy standalone OSG
  source remains available for per-step damping comparison.
- The Filament example includes `--scene rigid-loop`, a constrained
  SKEL-loaded chain fixture that carries the legacy `rigid_loop` example
  through descriptor-owned box-link renderables. The in-tree
  `pixi run ex rigid_loop` runner now routes to that Filament scene while the
  legacy standalone OSG source remains available for damping and constraint
  setup comparison.
- The Filament example includes `--scene mixed-chain`, a mixed rigid/soft
  chain fixture that carries the legacy `mixed_chain` example through
  descriptor-owned box-link and soft-mesh renderables. The in-tree
  `pixi run ex mixed_chain` runner now routes to that Filament scene while the
  legacy standalone OSG source remains available for keyboard-force comparison.
- The Filament example includes `--scene coupler-constraint`, a paired
  mimic/coupler rig fixture that carries the legacy `coupler_constraint`
  example through descriptor-owned box-link and guide-line renderables. The
  in-tree `pixi run ex coupler_constraint` runner now routes to that Filament
  scene while the legacy standalone OSG/ImGui source remains available for
  status-overlay and reset-control comparison.
- The Filament example includes `--scene add-delete-skels`, a deterministic
  add/delete skeleton fixture that carries the legacy `add_delete_skels`
  example through descriptor-owned ground and cube-skeleton renderables. The
  in-tree `pixi run ex add_delete_skels` runner now routes to that Filament
  scene while the legacy standalone OSG source remains available for live q/w
  add-delete control comparison.
- The Filament example includes `--scene vehicle`, a SKEL-loaded vehicle visual
  fixture that carries the legacy `vehicle` example through descriptor-owned
  car body, wheel cylinder, ground, and obstacle renderables. The in-tree
  `pixi run ex vehicle` runner now routes to that Filament scene while the
  legacy standalone OSG source remains available for live throttle and steering
  control comparison.
- The Filament example includes `--scene hybrid-dynamics`, a posed fullbody
  visual fixture that carries the legacy `hybrid_dynamics` and `biped_stand`
  examples through descriptor-owned biped and ground box renderables. The
  in-tree `pixi run ex hybrid_dynamics` and `pixi run ex biped_stand` runners
  now route to that Filament scene while the legacy standalone OSG sources
  remain available for scripted joint-command, harness-toggle, SPD-controller,
  and perturbation-control comparison.
- The Filament example includes `--scene joint-constraints`, a fullbody SPD
  control fixture that carries the legacy `joint_constraints` visual workflow
  through descriptor-owned biped and ground box renderables. The fixture runs
  the standing controller through a private scene pre-step hook before each
  DART step. The in-tree `pixi run ex joint_constraints` runner now routes to
  that Filament scene while the legacy standalone OSG source remains available
  for perturbation-shortcut and harness-toggle comparison.
- The Filament example includes `--scene free-joint-cases`, a zero-gravity
  free-joint fixture that carries the legacy `free_joint_cases` visual
  workflow through descriptor-owned active and transparent reference boxes. The
  fixture advances torque-free reference bodies through a private scene
  pre-step hook before each DART step. The in-tree
  `pixi run ex free_joint_cases` runner now routes to that Filament scene while
  the legacy standalone OSG/ImGui source remains available for numeric-check
  and reference-model control comparison.
- The Filament example includes `--scene human-joint-limits`, a Kima human
  fixture that carries the legacy `human_joint_limits` visual workflow through
  descriptor-owned mesh, multi-sphere, box, and ground renderables with DART
  joint-limit enforcement enabled. The in-tree
  `pixi run ex human_joint_limits` runner now routes to that Filament scene
  while the legacy standalone source remains available for the custom
  TinyDNN-backed arm and leg constraint comparison.
- The Filament example includes `--scene lcp-physics`, a deterministic contact
  benchmark fixture that carries the legacy `lcp_physics` visual workflow
  through descriptor-owned mass-ratio boxes, stack boxes, dominoes, falling
  spheres, and ground renderables. The in-tree `pixi run ex lcp_physics`
  runner now routes to that Filament scene while the legacy standalone source
  remains available for solver controls, plots, scenario switching, and frame
  recording.
- The Filament example includes `--scene mimic-pendulums`, an SDF-loaded
  pendulum-rig fixture that carries the legacy `mimic_pendulums` example
  through descriptor-owned cylinder, box, and ground renderables. The in-tree
  `pixi run ex mimic_pendulums` runner now routes to that Filament scene while
  the legacy standalone OSG source remains available for ImGui solver/debug
  table comparison.
- The Filament example includes `--scene atlas-puppet`, an Atlas IK-target
  fixture that carries the legacy `atlas_puppet` visual workflow through
  descriptor-owned Atlas mesh, ground, and selectable hand/foot target
  renderables. The in-tree `pixi run ex atlas_puppet` runner now routes to
  that Filament scene while the legacy standalone OSG source remains available
  for teleoperation-widget and support-polygon comparison.
- The Filament example includes `--scene hubo-puppet`, a Hubo IK-target
  fixture that carries the legacy `hubo_puppet` visual workflow through
  descriptor-owned Hubo mesh, ground, and selectable hand/foot/wrist-peg target
  renderables. The in-tree `pixi run ex hubo_puppet` runner now routes to that
  Filament scene while the legacy standalone OSG source remains available for
  teleoperation-widget, support-polygon, and keyboard-control comparison.
- The Filament example includes `--scene atlas-simbicon`, an Atlas visual
  fixture that carries the legacy `atlas_simbicon` startup view through
  descriptor-owned Atlas mesh and ground renderables. The in-tree
  `pixi run ex atlas_simbicon` runner now routes to that Filament scene while
  the legacy standalone OSG/ImGui source remains available for the Simbicon
  gait controller, perturbation shortcuts, and panel controls.
- The Filament example includes `--scene operational-space-control`, a WAM
  fixture that carries the legacy `operational_space_control` visual workflow
  through descriptor-owned robot, ground, and selectable target renderables.
  The fixture runs the task-space controller through a private scene pre-step
  hook before each DART step. The in-tree
  `pixi run ex operational_space_control` runner now routes to that Filament
  scene while the legacy standalone OSG source remains available for
  drag-and-drop axis-constraint comparison.
- The Filament example includes `--scene wam-ikfast`, a WAM visual fixture
  that carries the legacy `wam_ikfast` visual workflow through
  descriptor-owned robot, ground, and end-effector target renderables. The
  in-tree `pixi run ex wam_ikfast` runner now routes to that Filament scene
  while the legacy standalone OSG source remains available for IKFast solver,
  drag-mode, keyboard-shortcut, and posture-reset comparison.
- The Filament example includes `--scene fetch`, a Fetch MJCF visual fixture
  that carries the legacy `fetch` workflow through descriptor-owned robot,
  object, and target renderables. The in-tree `pixi run ex fetch` runner now
  routes to that Filament scene while the legacy standalone OSG/ImGui source
  remains available for panel, drag-control, and mocap target update
  comparison.
- The Filament example includes `--scene tinkertoy`, a Tinkertoy visual fixture
  that carries the legacy builder's initial block assemblies, target marker,
  force line, and reference axes through descriptor-owned renderables. The
  in-tree `pixi run ex tinkertoy` runner now routes to that Filament scene
  while the legacy standalone OSG/ImGui source remains available for panel,
  mouse-picking, and block-add/delete comparison.
- The Filament example also includes `--scene drag-and-drop`, a first
  interaction-heavy fixture that carries the legacy `SimpleFrame` anchor, child
  frame, and axis marker layout through the same backend-hidden manipulation
  path. The in-tree `pixi run ex drag_and_drop` runner now routes to that
  Filament scene while the legacy standalone OSG source remains available for
  comparison until the promoted GUI API replaces the old viewer path.
- The in-tree `pixi run ex empty` runner now routes to the same
  `--scene drag-and-drop` Filament fixture while the legacy standalone OSG
  source remains available for custom world hooks and raw key-event callback
  comparison.
- The Filament example includes `--scene simple-frames`, a frame-hierarchy
  fixture that carries the legacy `simple_frames` example through
  descriptor-owned `SimpleFrame` box, ellipsoid, and arrow-marker renderables.
  The in-tree `pixi run ex simple_frames` runner now routes to that Filament
  scene while the legacy standalone OSG source remains available for
  comparison.
- The Filament example includes `--scene soft-bodies`, a SKEL-loaded fixture
  that carries the legacy `soft_bodies` example through descriptor-owned
  soft-mesh renderables. The in-tree `pixi run ex soft_bodies` runner now
  routes to that Filament scene while the legacy standalone OSG recorded
  playback source remains available for comparison.
- The Filament example includes `--scene point-cloud`, a visual point cloud and
  voxel-grid fixture that carries the legacy `point_cloud` example through
  descriptor-owned point-cloud and voxel-grid renderables. The in-tree
  `pixi run ex point_cloud` runner now routes to that Filament scene while the
  legacy standalone OSG/ImGui source remains available for the robot-mesh
  sampling controls.
- The Filament example includes `--scene capsule-ground-contact`, a capsule and
  ground fixture that carries the legacy `capsule_ground_contact` example
  through descriptor-owned capsule and ground renderables, using ODE collision
  when available. The in-tree `pixi run ex capsule_ground_contact` runner now
  routes to that Filament scene while the legacy standalone OSG source remains
  available for pose-reset controls.
- The Filament example includes `--scene simulation-event-handler`, a falling
  body and sensor-marker fixture that carries the legacy
  `simulation_event_handler` example through descriptor-owned box, sphere,
  ground, and sensor marker renderables. The in-tree
  `pixi run ex simulation_event_handler` runner now routes to that Filament
  scene while the legacy standalone OSG source remains available for
  force/torque controls and force-arrow visualization.
- The in-tree `pixi run ex imgui` runner now routes to the Filament example's
  default viewer and private built-in status panel while the legacy standalone
  OSG/ImGui source remains available for custom-widget comparison until a
  promoted DART-owned panel/tool API exists.
- The in-tree `pixi run ex rigid_shapes` runner now routes to the Filament
  example's default MVP scene, giving the recommended broad-shape visual path
  descriptor-owned primitive, mesh, point-cloud, heightmap, soft-mesh, and robot
  coverage while the legacy standalone OSG source remains available for shape
  spawning, contact toggles, and collision-detector comparison.
- The Filament example includes `--scene polyhedron`, a convex-hull and
  wireframe fixture that carries the legacy `polyhedron_visual` example through
  descriptor-owned convex-mesh and line-segment renderables. The in-tree
  `pixi run ex polyhedron_visual` runner now routes to that Filament scene
  while the legacy standalone OSG source remains available for comparison.
- The Filament example now also includes `--scene heightmap`, a local
  heightmap surface and reference-marker fixture that carries the visual side
  of the legacy `heightmap` example through descriptor-owned heightmap
  renderables. The in-tree `pixi run ex heightmap` runner now routes to that
  Filament scene while the legacy standalone OSG/ImGui source remains available
  for the panel-driven sculpting and contact-alignment controls.
- The Filament example now also includes `--scene g1`, which loads the remote
  Unitree G1 URDF through DART's normal resource-retriever path and exposes
  colored IK targets for both hands and feet. The in-tree
  `pixi run ex g1_puppet` runner now routes to that Filament scene while the
  legacy standalone source remains available for comparison. The G1 scene is
  intentionally not part of `--scene all` because it fetches remote robot
  assets by default.
- `dartsim` can render bounded screenshots through Filament's
  headless swap-chain path without creating a GLFW window.
- `DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS=ON` registers an opt-in CTest smoke
  check for the headless screenshot path. This is a CI hook, not a default test
  yet.
- The default headless smoke check validates more than a nonblank image: it
  samples the shadowed fixture region for dark, mid-tone, and bright pixels plus
  a minimum luminance spread. The generated scene-specific smokes also run the
  analyzer in a basic mode that scans each full PPM for nonzero pixels. Focused
  Python unit coverage checks those analyzer modes against synthetic PPMs, and
  the example-runner tests guard the matching CMake analysis-mode registration.
- Filament is the maintained renderer because it provides modern real-time
  rendering features while keeping DART's built-in visualization scope
  maintainable.
- `dartsim` now links through the promoted `dart-gui` component and can build
  without the legacy OSG implementation when `DART_BUILD_GUI=OFF` and
  `DART_BUILD_DARTPY=OFF`. Compatibility shims remain under
  `dart/gui/experimental` for old include paths.
- Checked-in dartpy stubs and Python API docs now list `dartpy.gui` as the
  promoted surface and keep `dartpy.gui.experimental` as a compatibility
  namespace for descriptor, picking, debug-line, run-loop, camera,
  camera-controller, directional nudge, and screenshot helpers.
- Promotion requires explicit visual-quality gates, including dynamic shadows.
- The default source-build GUI path is enabled on Linux x86_64, where CMake can
  fetch the pinned upstream Filament archive. Other platforms should provide
  `Filament_ROOT` or keep `DART_BUILD_GUI=OFF` until packaged Filament is
  available there.
- `pixi run test-dart-gui-smoke` now exercises the explicit pinned
  `DART_FETCH_FILAMENT=ON` path on Linux x86_64, builds without the legacy OSG
  GUI target, and runs the default, hello-world, boxes, hardcoded-design,
  rigid-chain, rigid-loop, mixed-chain, coupler-constraint, add-delete-skels,
  vehicle, hybrid-dynamics, joint-constraints, free-joint-cases,
  human-joint-limits, lcp-physics, mimic-pendulums, atlas-puppet,
  hubo-puppet, atlas-simbicon, operational-space-control, wam-ikfast, fetch,
  tinkertoy,
  drag-and-drop,
  simple-frames, soft-bodies,
  point-cloud, capsule-ground-contact, simulation-event-handler, polyhedron,
  and heightmap headless CTest smokes.
  When no display server is available, the task runs the smokes under Xvfb and
  prefers Mesa's EGL vendor file for software rendering. The Ubuntu CI workflow
  has a matching opt-in smoke job that gets Xvfb, libc++, and libc++abi from
  system packages instead of relying on a Filament package. The MVP PR #2647
  merged with hosted `DART GUI Smoke (GCC)` and
  `DART GUI Smoke (Clang)` passing.
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
- An open non-draft conda-forge staged-recipes PR exists for Filament 1.71.3.
  Its current head is still open and behind the target branch, but the latest
  inspected head `d78834e` is green across staged-recipes linter,
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

1. Continue the legacy-source parity audit for migrated examples instead of
   treating source ownership, buildability, or headless screenshots as full
   restoration. `examples/fetch/` is explicitly still part of that audit
   standard if any missing Fetch-specific behavior is identified.
2. The current implementation slice is `examples/hardcoded_design`: restore
   number-key joint controls, the `-` direction toggle, camera home, README,
   and parity tests through public `dart::gui`. Track the old OSG wireframe
   mode as a future DART-owned render-style/debug API requirement.
3. Add only DART-owned public GUI concepts needed for those examples: no public
   Filament, GLFW, Dear ImGui, OSG, Raylib, OpenGL, Vulkan, or Metal types.
4. Keep checkpoint commits small, run `pixi run lint` before every commit, and
   push the tracked branch without opening a PR so GitHub CI can run.
