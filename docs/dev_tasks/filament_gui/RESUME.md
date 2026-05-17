# Resume: Filament GUI Replacement

This resume file is active handoff material for the current promotion branch.
Read `10-active-execution.md` first; it records the latest maintainer steering,
checkpoint-push policy, CI state, and immediate example/API migration plan.

## Current Resume Override

The branch is `feature/filament-gui-full-execution`, tracking
`origin/feature/filament-gui-full-execution`. The current local branch is ahead
of the tracked remote with the Fetch image-sequence capture checkpoint
`de1873ddab7 Document Fetch image-sequence capture`, and this handoff includes
the R24-1 public gizmo-affordance checkpoint, the R24-2 gizmo axis-handle drag
checkpoint, the R24-3 rotation-ring checkpoint, the R24-4 plane-handle
checkpoint, the R24-5 handle-highlighting checkpoint, the R24-7 Atlas
public-gizmo target checkpoint, the R24-8 remaining robot/IK gizmo rollout, the
R24-9 `imgui` panel target gizmo cleanup, the R24-10 Tinkertoy force-target
gizmo cleanup, the R24-11 drag-and-drop frame-gizmo cleanup, and the R24-12
Fetch target-gizmo cleanup, the R24-13 native window-title cleanup, the R24-14
camera up-vector cleanup, the R24-15 panel camera-inspection cleanup, the
R24-16 key-release cleanup, the R24-17 panel lighting cleanup, the R24-18
application-callback cleanup, the R24-19 render-settings cleanup, and the
R24-20 panel-color cleanup, the R24-21 panel-table/swatch cleanup, the R24-22
Simulation Event Handler help-alias cleanup, the R24-23 `hardcoded_design`
wireframe cleanup, the R24-24 Heightmap/Point Cloud source-owned grid cleanup,
the R24-25 `lcp_physics` metric-diagnostics cleanup, and the R24-26
`hubo_puppet` COM-overlay cleanup, and the R24-27 `atlas_puppet`
target/support cleanup, and the R24-28 `atlas_puppet` whole-body solver
cleanup, and the R24-29 `hubo_puppet` whole-body solver cleanup, and the
R24-30 `hubo_puppet` analytical IK cleanup. R24-16 adds key-release triggers
for promoted keyboard actions and restores keydown/key-release callbacks in the
affected migrated examples. R24-17 adds
`PanelContext::lighting` headlight state and restores historical headlight
checkboxes through public `dart::gui`. R24-18 adds public
`ApplicationOptions::postStep`, `preRender`, and `postRender` callbacks and
restores the historical lifecycle-hook demos in `empty` and `imgui`. R24-19
adds public `RenderSettings::shadowsEnabled` and restores historical shadow
toggles in `atlas_simbicon` and `operational_space_control`. R24-20 adds
public `PanelBuilder::colorEdit` and restores the terrain color editor in
`heightmap` plus point-cloud and voxel-grid color editors in `point_cloud`.
R24-21 adds public panel table and color-swatch primitives and restores the
historical Mimic Pendulums rig legend plus diagnostics table. R24-22 restores
the Simulation Event Handler `?` help alias. R24-23 restores
`hardcoded_design` wireframe link visuals with source-owned DART line
geometry. R24-24 restores Heightmap and Point Cloud fine-grained grid controls
with source-owned DART line geometry. R24-25 restores `lcp_physics` render FPS,
rendered/skipped frame counts, and rolling step-time diagnostics with
source-owned metrics. R24-26 restores the `hubo_puppet` support overlay's
blue/red center-of-mass validity marker with source-owned DART line geometry.
R24-27 restores `atlas_puppet` target activation/deactivation, active-target
solving, X/C support toggles, P/T diagnostics, and source-owned
support-polygon/COM overlays. Its focused build, marker CTest, mandatory lint,
post-lint rebuild/CTest, and Atlas headless smoke analyzer validation pass
locally. R24-28 restores the Atlas source-owned `RelaxedPosture` objective,
public `BalanceConstraint`, whole-body IK solve path, and hold/release R
posture/balance optimization. R24-29 restores the Hubo source-owned
`RelaxedPosture` objective, public `BalanceConstraint`, whole-body IK solve
path, and hold/release R posture/balance optimization. R24-30 restores the
Hubo source-owned `HuboArmIK` and `HuboLegIK` analytical gradient methods. Its
focused build, marker CTest, mandatory lint, post-lint rebuild/CTest, and Hubo
headless smoke analyzer validation pass locally.
Do not push these local checkpoints without explicit maintainer/user approval
in the active session. Leave the pre-existing local
`docs/dev_tasks/filament_gui/STEERING.md` edits unstaged unless the maintainer
explicitly asks to include them.

Immediate next slice after the R24-30 Hubo analytical IK rollout: continue
reducing the named public API gaps in `11-example-parity-audit.md`. Public
`dart::gui::Gizmo` now covers the source-owned manipulation affordances for the
recently re-opened target examples, public `OrbitCamera::up` covers historical
custom camera up-vector defaults, and public `PanelContext::camera` covers the
Eye/Center/Up panel readouts in `box_stacking` and `imgui`. Public
`KeyboardActionTrigger::Release` now covers the historical keyup messages in
`box_stacking`, `empty`, and `imgui`, and public `PanelContext::lighting` now
covers historical headlight checkboxes. Public `ApplicationOptions` callbacks
now cover post-step and pre/post-render lifecycle hooks for `empty` and
`imgui`, public `RenderSettings` now covers shadow toggles, and public
`PanelBuilder::colorEdit` now covers basic RGBA panel color editors, and the
local source-owned grid helper covers the Heightmap and Point Cloud
fine-grained grid controls. Remaining audit gaps are mostly depth render
outputs, exact panel line plotting/backend debug metrics, and example-specific
simulation controls rather than bare source-owned target handles, camera
up-vector defaults, basic camera readouts, simple key-release callbacks,
headlight checkboxes, basic lifecycle hooks, shadow toggles, basic color
editors, source-owned grid controls, source-owned metric summaries,
source-owned target activation, source-owned COM overlays, or Atlas/Hubo
source-owned posture/balance solving.

## Live Supervisor Steering

A parallel evaluator pass (2026-05-14) wrote `STEERING.md` next to this
file. Read it before picking the next slice; it records the live steering
context for the post-MVP branch and should be kept in sync when slices land.

## Last Session Summary

The branch is `feature/filament-gui-full-execution`, tracking
`origin/feature/filament-gui-full-execution`. Recent pushed checkpoints promote
Filament as the maintained GUI renderer, remove legacy OSG/Raylib renderer
surfaces, document Filament as official, and make the default GUI build respect
the platform support matrix. GitHub CI was manually dispatched without opening
a PR; Linux headless rendering exposed a concrete restoration gap because the
old `rigid_cubes` executable was removed while the workflow still invokes it.
Checkpoint `d343c3c64bc` renames the MVP app to `dartsim`, adds
`dart/gui/application.hpp`, restores historical GUI example executable names as
thin `dart::gui` launchers, updates Linux headless CI to validate the promoted
`--screenshot` capture path, and was pushed to the tracked remote branch. CI
was manually dispatched for lint, Linux, macOS, Windows, and CodeQL without
opening a PR. Checkpoint `c9ccedfebe8` adds promoted `dart/gui/*.hpp` wrappers
and explicit `dart::gui` aliases for renderer-independent scene, viewer,
geometry, interaction, debug, and profiling concepts. CI Lint on
`d343c3c64bc` revealed that `examples/simple_frames/CMakeLists.txt` was ignored
by `.gitignore`'s `*_frames/` rule and missing on GitHub; the immediate repair
was force-added in `cd04ba4862ac`, then CI was manually redispatched for lint,
Linux, macOS, Windows, and CodeQL on that branch head. Checkpoint
`858ab55cacd1` promotes existing Python GUI descriptor/helper symbols from
`dartpy.gui.experimental` onto `dartpy.gui`, leaves the old submodule as a
compatibility namespace, and was pushed with a new manual CI dispatch.
Checkpoint `30b879458f8` renames private CMake helpers, smoke-test variables,
and runner environment variables from the old backend-named `filament_gui`
compound to `gui_filament`/`DART_GUI_FILAMENT` wording, then was pushed with a
new manual CI dispatch. Checkpoint `8796ed5ad99` promotes the
renderer-independent C++ GUI declarations and definitions from
`dart::gui::experimental` to stable `dart::gui` headers/names, keeps
`dart/gui/experimental/*.hpp` as compatibility shims, promotes the private
Filament implementation namespace to `dart::gui::filament`, adds a promoted
header guard test, runs the focused C++/Python/smoke verification set, and is
pushed to the tracked remote branch. CI Lint on that checkpoint completed
successfully; Linux, macOS, Windows, and CodeQL were still running at the
latest check.

The latest steering is to continue without waiting for CI to finish, document
scope/design updates in this dev-task folder before acting on them, remove the
remaining `experimental` naming from promoted Filament GUI concepts, restore
all pre-existing user-facing examples by migrating them to the new
`dart::gui`, and rename the MVP executable away from `filament_gui` to a
scope-based name because there is now only one official renderer.
The latest naming direction also distinguishes library and application brands:
`DART` is the project and C++ library family identity, `libdart` is appropriate
for package/library artifacts, and `dartsim` is the application-level
simulator/viewer identity, analogous to Isaac Sim.

Maintainer correction after the source-ownership sweep: examples such as
`examples/fetch/` may be source-owned without being fully restored. The active
docs now distinguish example ownership (`options.world`, no
`options.defaultScene`) from feature parity with the historical source. The
Fetch repair has been committed and pushed: it adds a public
`ApplicationOptions::camera` override, restores the historical camera framing,
and makes the visible target bars one draggable target-frame renderable rather
than child bars that can detach from the mocap target. A follow-up Fetch slice
is now implemented locally because the historical source also added a visible
`GridVisual` offset at the pick-and-place work area. It restores that as public
DART line-segment geometry in `examples/fetch/main.cpp`, not as a private
Filament debug-overlay hook. The robot/IK target-handle parity checkpoint has
also been pushed for `g1_puppet`, `atlas_puppet`, `hubo_puppet`,
`operational_space_control`, `wam_ikfast`, and `tinkertoy`. The Tinkertoy
builder and keyboard checkpoints are pushed: selected-block state, dynamic
add/delete controls, gravity and force-coefficient controls, target
reorientation, force-line updates, external force application,
collision/dynamics block construction, and the `1`/`2`/`3`, Backspace, Delete,
Up/Down, and backtick hotkeys are implemented through public `dart::gui`
panels, `ApplicationOptions::preStep`, and
`ApplicationOptions::keyboardActions`. Later local checkpoints restore
Atlas/Hubo target activation semantics, Atlas/Hubo relaxed-posture/balance,
and Hubo analytical IK; Hubo Shift modifier movement and the legacy Enter
recording toggle remain explicit follow-ups.

## Immediate Next Step

Restore Atlas/Hubo robot puppet behavior through the promoted keyboard action
API: continuous IK solving plus WASD/QE/FZ root teleoperation. This is
implemented and pushed for `atlas_puppet` and `hubo_puppet`. The next slice is
Tinkertoy Tab camera-home: add a renderer-neutral camera reset callback to
`KeyboardActionContext` and register Tab from the Tinkertoy example without
exposing backend camera/window types. This is implemented and pushed. The next
slice is G1 target activation parity: use public `dart::gui` keyboard actions
to make number keys add/remove target frames, reset activated targets to their
end-effector transforms, and solve only active G1 targets from
`ApplicationOptions::preStep`. This is implemented and pushed. The next
source-owned restoration slice re-audits `examples/fetch/` against the
historical OSG source, because prior Fetch camera/grid/capture evidence is not
proof that the full example behavior was restored. That Fetch re-audit found a
remaining first-slice gap in the legacy panel affordances; Exit, Play/Pause,
Help, and About controls are implemented and pushed through public `dart::gui`
lifecycle/panel APIs. The active slice is `examples/add_delete_skels/`:
restore the historical `q`/`w` keyboard controls, Bullet preference, and camera
default through public `dart::gui`; that checkpoint is implemented and pushed.
The active slice is now `examples/mixed_chain/`: restore the historical
`q`/`w`, `e`/`r`, and `t`/`y` impulse keyboard controls plus camera default
through public `dart::gui`; that checkpoint is implemented and pushed. The
active slice is now `examples/vehicle/`: restore the historical
`w`/`s`/`x`/`a`/`d` command keys and camera default through public `dart::gui`.
That checkpoint is implemented and pushed. The active slice is now
`examples/soft_bodies/`: restore the historical `[`/`]`/`{`/`}`/`r`/`\`
playback keys through public `dart::gui`, including shifted-bracket handling in
the private GLFW input bridge; that checkpoint is implemented and pushed. The
active slice is now `examples/hybrid_dynamics/`: restore the historical `h`
harness toggle and camera default through public `dart::gui`; that checkpoint
is implemented and pushed. The active slice is now
`examples/joint_constraints/`: restore the historical `1`-`4` perturbation
keys, `h` harness toggle, and camera default through public `dart::gui`; that
checkpoint is implemented and pushed. The active slice is now to re-open
`examples/fetch/` as a full historical-source parity audit. Earlier Fetch
checkpoints restored the promoted source file, camera, target bars,
work-area grid, panel controls, README, and run defaults, but maintainer
feedback says there are many more examples that are not fully restored and
Fetch remains the concrete reminder. Do not declare Fetch, or any other
source-owned example, complete until `11-example-parity-audit.md` has an
itemized historical-source inventory and every user-visible item is restored
through promoted `dart::gui`, explicitly superseded, or recorded as a public
API gap. The current Fetch re-audit checkpoint restores the historical
`Fetch robot example` panel title and whole-body-motion instructional copy,
updates the audit table, adds source-marker coverage, and has passed focused
build/CTest, direct and pixi screenshot analyzer checks, and the Python
example-runner unit test. After the pushed human joint-limits checkpoint
(`cf0ed62209e`), maintainer steering re-opened Fetch again as the concrete
incomplete-example case. The next active slice should strengthen Fetch target
manipulation/help parity through public `dart::gui`: restore a richer
source-owned target affordance, add any feasible renderer-neutral target
orientation controls, and record exact mouse rotation-ring parity as a public
manipulation API gap if needed. The local implementation now adds the richer
target handle, target rotation/reset keyboard actions, and restored viewer help
text; it was committed and pushed as
`ae175ef6981 Restore Fetch target manipulation`. The active slice is now
`examples/imgui/`: restore the historical panel-extension example through
public `dart::gui`, not the current generic box/slider demo. The first code
pass should restore the target frame, promoted keydown callbacks, gravity
control, panel sections, viewer help, 640x480/camera defaults, README, and
source-marker guards while tracking headlight toggles, camera-inspector text,
keyboard release behavior, and render/post-step hooks for follow-up. Later local
R24-15/R24-16 checkpoints restore camera-inspector text and keyboard release
callbacks through public `dart::gui`; the later local R24-17 checkpoint
restores headlight toggles, and the local R24-18 callback checkpoint restores
render/post-step hooks. The current heightmap checkpoint restores
`--demo interactive|alignment`, a mutable `HeightmapShaped` `SimpleFrame`,
renderer-neutral panel controls, the ODE-backed alignment scene, camera/run
defaults, README, changelog, and source-marker tests. The old OSG grid style
controls and terrain color editor remain documented public API gaps. A later
slice restored `examples/rigid_shapes`: restore the
historical `q`/`w`/`e`/`r` shape-spawn controls, `a` delete-last control, `c`
contact-point toggle, convex mesh spawn path, camera home, and 640x480 default
launch size through public `dart::gui`, plus the rigid-shapes-specific
collision-detector, max-contact, and ground-thickness command-line options;
that checkpoint is implemented and pushed. The active slice is now
`examples/biped_stand`: restore the historical `1`-`4` perturbation keys,
camera home, 640x480 default launch size, and README through promoted
`dart::gui`. That checkpoint is implemented and locally validated; commit and
push it before selecting the next pre-existing example for source-owned parity
restoration. The checkpoint was pushed as
`90b373beb14 Restore biped stand controls`. The active slice is now
`examples/free_joint_cases`: restore the historical numeric/reference controls,
local CLI flags, camera/run defaults, and README through promoted `dart::gui`.
That checkpoint is implemented and locally validated through the pre-lint
focused build, CTest, direct/pixi screenshot smokes, Python example-runner
tests, the aggregate `examples` build, `pixi run lint`, and post-lint focused
build/CTest/direct screenshot smoke. The checkpoint was pushed as
`f4963df00cd Restore free joint cases controls`. The active slice is now
`examples/lcp_physics`: restore the historical scenario/solver/list CLI,
camera/run defaults, panel context, and README through promoted `dart::gui`.
That checkpoint is implemented and locally validated through the pre-lint
focused build, CTest, `--list`, direct/pixi screenshot smokes, Python
example-runner tests, the aggregate `examples` build, `pixi run lint`, and
post-lint focused build/CTest/direct screenshot smoke. The checkpoint was
pushed as `619af5649bc Restore LCP physics controls`. The active slice is now
`examples/mimic_pendulums`: restore baseline retargeting, mimic diagnostics,
local solver/collision flags, camera/run defaults, and README through promoted
`dart::gui`. That checkpoint is implemented and locally validated through the
pre-lint focused build, CTest, direct/pixi screenshot smokes, Python
example-runner tests, the aggregate `examples` build, `git diff --check`,
`pixi run lint`, and post-lint focused build/CTest/direct screenshot smoke.
The checkpoint was committed and pushed as
`2bc6a0e168d Restore mimic pendulums diagnostics`. The active slice is now
`examples/box_stacking`: restore solver selection, camera/run defaults, README,
and parity tests through promoted `dart::gui`. That checkpoint is implemented
and locally validated through the pre-lint focused build, CTest, direct/pixi
screenshot smokes, Python example-runner tests, and the aggregate `examples`
build, `git diff --check`, `pixi run lint`, and post-lint focused
build/CTest/direct screenshot smoke. The checkpoint was committed and pushed as
`3493d7065c0 Restore box stacking solver controls`. The active slice is now
`examples/boxes`: restore the historical Bullet preference, camera/run
defaults, README, and parity tests through promoted `dart::gui`. That
checkpoint is implemented and locally validated through the pre-lint focused
build, CTest, direct/pixi screenshot smokes, Python example-runner tests, and
the aggregate `examples` build, `git diff --check`, `pixi run lint`, and
post-lint focused build/CTest/direct screenshot smoke. The checkpoint was
committed and pushed as `74870cc5cf3 Restore boxes run defaults`. The
`examples/point_cloud` strict audit and restoration checkpoint is committed and
pushed as `9e111631eb5 Restore point cloud example controls`. The
`examples/polyhedron_visual` checkpoint is committed and pushed as
`227c2498a53 Restore polyhedron visual defaults`. The `examples/empty`
checkpoint is committed and pushed as
`158e10d4cc2 Restore empty viewer scaffold`; it restores public keydown
actions, camera/run defaults, README, and marker guards while keeping pre/post
render callbacks as public API gaps after the later local R24-16 key-release
repair. The active slice is now `examples/simulation_event_handler`: the local implementation restores
the historical event-handler controls, selected-body force/torque application,
force-arrow visualization, camera/run defaults, README, and source-marker
guards through public `dart::gui`. This checkpoint has been committed and
pushed as `6ee2b97a20f Restore simulation event handler controls`. The active
slice is now `examples/hello_world`: restore the historical instruction text,
deterministic non-axis-aligned initial box orientation, 640x480 run default,
camera home, profiling scope/dump markers, README, and source-marker guards
through public `dart::gui`. The local implementation is in place and has
passed focused build/CTest, direct and pixi screenshot smokes, Python
example-runner tests, the aggregate `examples` build, and `git diff --check`
before lint. Mandatory `pixi run lint` and post-lint focused
build/CTest/direct screenshot smoke also pass.
This checkpoint has been committed and pushed as
`bcc31d495c8 Restore hello world defaults`. The active slice is now
`examples/capsule_ground_contact`: restore the historical `h`/`v` pose-reset
keys, Space velocity-clear action, 1024x768 run default, camera home,
persistent-manifold instruction text, README, and source-marker guards through
public `dart::gui`. The local implementation is in place and has passed the
pre-lint focused build/CTest, direct and pixi screenshot smokes, Python
example-runner tests, the aggregate `examples` build, and `git diff --check`.
Mandatory `pixi run lint` and post-lint focused build/CTest/direct screenshot
smoke also pass. The checkpoint has been committed and pushed as
`b00e054c45f Restore capsule contact controls`. The active slice is now
`examples/rigid_chain`: restore the historical random initial pose, 640x480
run default, camera home, README, and source-marker guards while preserving
the promoted `ApplicationOptions::preStep` damping behavior. The local
implementation is in place and has passed the pre-lint focused build/CTest,
direct and pixi screenshot smokes, Python example-runner tests, the aggregate
`examples` build, and `git diff --check`. Mandatory `pixi run lint`,
post-lint focused build/CTest/direct screenshot smoke also pass. Commit and
push this checkpoint next. The checkpoint has been committed and pushed as
`b864e5a31d3 Restore rigid chain defaults`. The active slice is now
`examples/rigid_loop`: restore the historical exact red constrained-link
colors, console instructions, 640x480 run default, README, and source-marker
guards while preserving the promoted `ApplicationOptions::preStep` damping
behavior. The local implementation is in place and has passed the pre-lint
focused build/CTest, direct and pixi screenshot smokes, Python example-runner
tests, the aggregate `examples` build, and `git diff --check`. Mandatory
`pixi run lint` and post-lint focused build/CTest/direct screenshot smoke also
pass. The checkpoint has been committed and pushed as
`4a957790bef Restore rigid loop defaults`. The active slice is now
`examples/rigid_cubes`: restore the historical `cubes.skel` world, Y-down
gravity, decaying directional force keys, 640x480 run default, camera home,
instruction text, README, and source-marker guards through public `dart::gui`.
The local implementation is in place and has passed the pre-lint focused
build/CTest, direct and pixi screenshot smokes, Python example-runner tests,
the aggregate `examples` build, and `git diff --check`. Mandatory
`pixi run lint` and post-lint focused build/CTest/direct screenshot smoke also
pass. The checkpoint has been committed and pushed as
`9f4af05ef1c Restore rigid cubes controls`.
Keep Hubo Shift modifier movement and Enter recording as explicit parity gaps
unless a later slice adds the narrow renderer-neutral public API or
source-owned state needed for them.

## Context That Would Be Lost

- Do not open a PR. Push checkpoint commits directly to the tracked remote
  branch so GitHub Actions can run.
- `rigid_cubes --headless --frames 10 --out /tmp/headless_output` failed in
  CI originally because the executable no longer existed and the old OSG-style
  output contract was gone. The executable has been restored, CI now checks
  `--screenshot`, and the local capture checkpoint restores `--out <dir>`
  without bringing back OSG.
- `filament_gui` is now the wrong example name. The executable should describe
  its role or use the application-level `dartsim` brand, not the renderer
  backend.
- The stretch direction is ImGui Docking with the 3D scene shown as a docked
  window/widget, plus offscreen rendering and automated image/video capture.
- The current code still has promoted concepts under `dart::gui::experimental`
  only as compatibility shims; stable includes/names live under `dart::gui`
  while backend objects remain private.
- The private file layout can remain under `dart/gui/experimental/detail` for a
  later sweep; the next slice is capture compatibility, not a full directory
  move.
- Do not treat a restored `examples/<name>/main.cpp` as full parity. Compare
  against the historical source, for example
  `git show 520993d7301^:examples/<name>/main.cpp`, before claiming an example
  is restored.

## Historical Summary

The GUI replacement plan was redirected from Raylib to a Filament + GLFW + Dear
ImGui experiment. The active working docs now describe an MVP example-first
strategy, promotion gates, and eventual replacement of both OSG and Raylib in
the appropriate major DART version. A visual-quality gate was added so
promotion requires visible dynamic shadows and broader rendering-quality
evidence.

The first implementation pass added experimental CMake options, Filament install
tree discovery, material compilation/embedding, and an `examples/filament_gui`
target. The example source initializes GLFW/Filament, builds a DART
two-box/ground-world fixture, extracts DART visual shape nodes into
backend-hidden renderable descriptors, uses lit materials, and configures
PCSS/cascaded sun shadows plus contact-shadow options. The Filament view also
uses a neutral skybox, spherical harmonics indirect lighting, high-quality
color grading, anisotropic texture sampling, HDR buffer quality, screen-space
ambient occlusion, temporal and multi-sample anti-aliasing, FXAA, and temporal
dithering; multi-sample anti-aliasing and the higher-cost post-process options
are kept to windowed runs because they prevent frame acquisition on the current
headless OpenGL software path. It also supports `--frames`, viewport size
options, orbit/pan/zoom camera controls, and
`--screenshot <path>` via Filament readback. It also supports `--headless`,
which uses Filament's headless swap-chain path for bounded screenshot smoke
tests without creating a GLFW window. A minimal Filament-native ImGui overlay
converts ImGui draw vertices/indices into Filament buffers for the built-in
status panel.

The `dart-gui-experimental` extraction layer has a graphics-free unit test
covering stable runtime IDs, transforms, geometry descriptors, colors,
visibility, shadow flags, and version stamps. The renderer consumes box, sphere,
ellipsoid, cylinder, cone, capsule, pyramid, multi-sphere, TriMesh-backed mesh,
line-segment, and finite PlaneShape descriptors from that target. The viewer
also maps DART visual-aspect shadow flags into Filament renderable shadow
settings. The finite PlaneShape proxy uses a procedural checker-textured
Filament material to exercise UVs and sampler binding. The fixture also loads
an imported WAM Collada mesh, the full WAM URDF
skeleton through DART's normal `dart::io` and `dart-utils-urdf` path, a
required Atlas DAE torso mesh, and a full Atlas SDF robot fixture with at least
twenty visible mesh descriptors/renderables. It also includes a required
four-panel glTF/PBR environment layout. The Filament mesh renderer consumes
DART's preserved UV metadata plus submesh material ranges, material colors,
emissive colors, metallic/roughness factors, and typed PNG/JPEG texture images
for base color, metallic, roughness, combined metallic-roughness, normal,
occlusion, and emissive maps when present. Broader authored asset fixtures now
include non-WAM robot meshes and PBR environment panels; broader
robot/environment visual review remains promotion work. The
renderer-independent scene layer now also extracts world-owned `SimpleFrame`
visuals, so drag-and-drop style targets can flow through the same renderable
descriptor path as body shape nodes. It also owns debug line descriptors for the
grid, world/body frames, center-of-mass markers, contact markers, contact
normal arrows, contact force arrows, support-polygon outlines,
support-centroid markers, inertia boxes, and collision-shape bounds, with
graphics-free unit coverage. It also has nearest ray-hit tests
for visible, hidden, hit, and miss cases, including bounds hit points/normals
and primitive
sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane
surface hit points/normals plus triangle-backed mesh hit points/normals. The
example wires those helpers into basic click-to-select highlighting with a
selection bounds overlay, and reports the selected DART shape in the built-in
panel.
Backend-hidden free-joint, simple-frame, and combined
frame-renderable translation helpers are covered by C++ and Python tests, and
the example uses the combined helper for keyboard nudging of selected dynamic
bodies and `SimpleFrame` visuals. Backend-hidden plane intersection,
plane-drag translation, and axis-drag translation helpers are also covered by
C++ and Python tests; the example uses the plane helpers for Ctrl-left
camera-plane dragging and the axis helpers for Ctrl-X/Y/Z-left constrained
dragging of selected dynamic bodies and `SimpleFrame` visuals.
Backend-hidden
run-option normalization, viewer lifecycle state, viewer profiling
accumulation, and orbit-camera helpers now live in `dart-gui-experimental`;
the Filament example uses them for bounded screenshots, camera placement,
headless runs, pause/step behavior, frame accounting, profile reporting, and
perspective pick rays. The constrained experimental API is now
split across focused backend-hidden headers: `renderable.hpp`,
`interaction.hpp`, `debug.hpp`, `geometry.hpp`, `viewer.hpp`, and
`profile.hpp`.
`scene.hpp` remains an aggregate compatibility include for existing
experimental consumers. Viewer-runtime helpers live in
`dart/gui/experimental/viewer.cpp`, profile accumulation lives in
`dart/gui/experimental/profile.cpp`, debug descriptor generation lives in
`dart/gui/experimental/debug.cpp`, backend-hidden mesh builders live in
`dart/gui/experimental/geometry.cpp`, shape description extraction lives in
`dart/gui/experimental/shape_descriptions.cpp`, renderable identity/extraction
and resource versioning live in `dart/gui/experimental/renderable.cpp`, and
picking, plane/axis dragging, frame translation, and renderable set planning
live in `dart/gui/experimental/interaction.cpp`. Backend-hidden renderable set planning
now also compares active render-resource versions so
descriptor-owned geometry changes, including dynamic soft-mesh vertex changes,
recreate Filament resources. Supported descriptor kinds that cannot produce
renderer resources now carry diagnostic reasons that the Filament example logs
once. Backend-hidden RGBA-to-PPM screenshot storage also now lives in
`dart-gui-experimental`; Filament renderer readback, wait/save finalization,
and screenshot profile accounting now live under
`dart/gui/experimental/detail/filament` behind the Filament render context, so
the screenshot helper header does not expose Filament renderer types.
Retained Filament renderables also reapply descriptor shadow flags each frame,
so DART visual-aspect shadow changes are not limited to resource creation.
Filament PNG/JPEG image decoding, texture-cache ownership, sampler setup, and
PBR texture parameter binding live in
`dart/gui/experimental/detail/filament/textures.hpp` and `.cpp`, with repeat
sampler construction private to the `.cpp` implementation so the helper header
avoids a direct Filament sampler include.
`UNIT_gui_FilamentSceneExtraction` also checks that any remaining
`examples/filament_gui/*.hpp` files and the example entry point have no direct
Filament header includes; this is only an incremental guard, not completion of
the full example include metric. The same unit now recursively guards that the
example tree contains no regular files except `CMakeLists.txt`, `README.md`,
and `main.cpp`, and no C++ source/header files except `main.cpp`. It also
checks that the entry point has only the generic application include, a single
`main`, the delegated `runGuiApplication(argc, argv)` call, and no backend
implementation tokens.
The backend-token scan now uses a reusable `scanHeadersForBackendTokens`
helper with an explicit future hook for promoted `dart/gui/*.hpp` headers once
the first-class Filament API replaces the legacy OSG-shaped public GUI headers.
The full north-star also requires any surviving `examples/filament_gui/` tree
to contain only a minimal `main.cpp` executable entry point, plus unavoidable
build/docs files: renderer setup, frame lifecycle, material and texture
resources, scene synchronization, capture, overlays, input translation, and
reusable fixture logic should move into `dart::gui` or private GUI
implementation units rather than remain as example-local architecture.
Filament engine, renderer, swap-chain, main view, scene, camera lifecycle, and
begin/render/end frame calls live in
`dart/gui/experimental/detail/filament/render_context.hpp` and `.cpp`;
`main.cpp` now has zero direct Filament header includes, and this backend
implementation slice has moved out of the example tree.
Debug-line overlay option defaults, static/contact refresh/cleanup state, and
selected-renderable overlay lookup now live in
`dart/gui/experimental/detail/filament/debug_overlay.hpp` and `.cpp`.
Initial scene extraction, startup validation, first renderable synchronization,
and startup debug-overlay validation live in
`dart/gui/experimental/detail/filament/scene_startup.hpp` and `.cpp`.
Filament neutral lighting/color grading, light entity creation and orbit
updates, scene environment binding, viewport/camera configuration helper, and
windowed view-quality setup live in
`dart/gui/experimental/detail/filament/render_environment.hpp` and `.cpp`.
Per-frame framebuffer sizing, ImGui display metrics, viewport/camera update
dispatch, and camera-controller suppression while selection or the built-in UI
consumes the pointer live in
`dart/gui/experimental/detail/filament/frame_viewport.hpp` and `.cpp`.
The GLFW/ImGui input bridge, orbit-camera scroll callback attachment, and
application hotkey polling live in
`dart/gui/experimental/detail/filament/input.hpp` and `.cpp`; reusable
manipulation math remains in `dart-gui-experimental`.
GLFW initialization/window lifecycle, window-close loop predicate, and
platform-specific native-window handle selection live in
`dart/gui/experimental/detail/filament/native_window.hpp` and `.cpp`.
Filament-native ImGui context setup, style scaling, font loading, current IO
access, overlay rendering, and draw-data upload live in
`dart/gui/experimental/detail/filament/imgui_overlay.hpp` and `.cpp`; the
built-in panel policy remains MVP/example-scoped.
Per-frame UI input sync, built-in status panel dispatch, debug-overlay
refreshes caused by panel toggles, and ImGui overlay upload live in
`dart/gui/experimental/detail/filament/ui_frame.hpp` and `.cpp`.
Per-frame simulation stepping, renderable descriptor extraction, scene
synchronization, selected-renderable keyboard/mouse interaction updates,
selection-debug overlay refresh, and orbiting-light updates live in
`dart/gui/experimental/detail/filament/scene_frame.hpp` and `.cpp`.
Ordered teardown of private Filament application resources lives in
`dart/gui/experimental/detail/filament/application_teardown.hpp` and `.cpp`.
Selection label formatting, robot IK-target translation glue, keyboard nudging,
click selection, Ctrl-left camera-plane drag event translation, and
Ctrl-X/Y/Z-left axis-constrained drag dispatch live in
`dart/gui/experimental/detail/filament/selection.hpp` and `.cpp`; reusable
selection math stays in `dart-gui-experimental`.
Filament material bundle creation, seed texture resources, renderable state,
lit-material setup, shadow flag application, and destruction lifecycle helpers
live in `dart/gui/experimental/detail/filament/renderable_resources.hpp` and
`.cpp`.
Filament material shader sources now live under
`dart/gui/experimental/detail/filament/materials`, and the opt-in headless
smoke CMake/Python helpers now live under
`dart/gui/experimental/detail/filament/testing`, leaving the example directory
to carry only its entry-point source plus example-level CMake/README files.
The example entry point now includes
`dart/gui/experimental/detail/application.hpp`, a generic private GUI detail
wrapper, instead of the backend-specific private detail application header.
The private backend source list, dependency wiring, target setup, material
inputs, material header generation, and smoke-test registration are centralized
in
`dart/gui/experimental/detail/filament/filament_sources.cmake`, so the example
CMake file includes private helper plumbing instead of owning the backend file
list, renderer dependency links, or repeated Filament material/test commands
itself. That helper now compiles only `examples/filament_gui/main.cpp` from the
example tree and fails configuration if unexpected example-tree regular files
are added outside `CMakeLists.txt`, `README.md`, and `main.cpp`, or if the entry
point adds a direct Filament header include.
Filament descriptor-to-scene synchronization, unsupported descriptor logging,
initial scene renderable creation, scene entity attachment, and per-frame
renderable transform/selection/shadow update helpers live in
`dart/gui/experimental/detail/filament/renderable_sync.hpp` and `.cpp`.
Filament descriptor-to-renderable construction now lives in
`dart/gui/experimental/detail/filament/renderable_factory.hpp` and `.cpp`,
including generated mesh buffer upload, debug-line renderable creation,
material part assembly, and texture-backed mesh binding. The factory header does
not include Filament headers; the full migration metric remains zero direct
Filament header includes from maintained example files after promotion, not just
zero direct Filament includes from helper headers.
`UNIT_dynamics_MeshShape` also loads the checked-in
`data/gltf/pbr_triangle.gltf` and `data/gltf/pbr_multi_material.gltf` fixtures
through the real Assimp importer and verifies authored glTF PBR texture slots,
material factors, alpha-bearing material color, UV metadata, and
single-/multi-material submesh ranges without requiring a graphics context. The
Filament smoke scene also loads these fixtures and uses transparent lit
material variants for alpha-bearing solid, textured, and mesh visual paths.
The scene descriptor now also carries `MeshShape` alpha-mode policy into
Filament material creation, and alpha-only visual changes participate in the
renderer resource version so opaque/transparent material resources are recreated
when needed.
The Filament point-cloud fixture now exercises `BIND_PER_POINT` colors, and the
renderer creates per-point material ranges from those descriptor colors instead
of collapsing the point cloud to one material color.
`examples/gui_scene_diagnostics` now gives `dart-gui-experimental` a second,
non-rendering example consumer for descriptor/debug/camera diagnostics,
including a `SimpleFrame` visual, without adding another backend. The
Filament example's built-in panel now exposes pause/resume, single-step, and
debug overlay toggles for grid, frames, center-of-mass markers, contacts,
inertia boxes, collision-shape bounds, normal arrows, force arrows, and support
polygons. The same executable now supports `--scene drag-and-drop`, a first
interaction-heavy fixture that carries the
legacy `SimpleFrame` anchor, child frame, and axis markers through the
backend-hidden extraction and manipulation path.

The main remaining packaging risk is durable toolchain compatibility:
conda-forge does not currently provide an installable Filament package, and the
upstream Linux archive links against libc++ while the current DART Pixi
environment uses GCC/libstdc++. A ready-for-review staged-recipes PR exists at
`conda-forge/staged-recipes#33297`; it builds Filament 1.71.3 from source and
splits host tools into `filament` and headers/static libraries into
`filament-static`. That package remains the preferred future Pixi dependency,
but the current Linux smoke path does not wait for it. At latest inspection, the
PR was still open and behind the target branch at head
`d78834e9bcca1c0c5a55f3d2752f39765932c3f2`; staged-recipes linter,
conda-forge-linter, Check Skip, Azure linux_64, osx_64, win_64, and aggregate
staged-recipes checks passed. The previous Linux failure came from using
`source_files: test-cmake`; the current head uses `files: test-cmake` for the
recipe-local CMake consumer test and links that test through CMake OpenGL
package targets. `@conda-forge/help-c-cpp` has been pinged, and the feedstock
is not created yet.

When compatible libc++/libc++abi libraries are supplied, the experimental target
configures, links, and runs locally with Mesa llvmpipe. The explicit
`DART_FETCH_FILAMENT=ON` path also fetched the pinned Linux x86_64 archive and
passed local windowed and headless screenshot smoke checks. The
opt-in headless CTest smoke now also runs a small PPM analyzer that checks the
shadowed fixture region for dark, mid-tone, and bright pixels plus luminance
spread, so a flat-but-nonblank default frame fails the smoke gate. The
scene-specific headless smokes also run the analyzer in a basic full-image
nonzero mode, so each generated scene smoke verifies more than just the sampled
CMake pixel prefix. Focused Python unit coverage checks both analyzer modes and
the default CLI mode against synthetic PPMs, and the example-runner tests check
that the CMake smoke registration keeps those analysis modes wired. The
Filament example also builds through `dart-gui-experimental` with
`DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF`, proving the path does not
require the legacy OSG GUI target. Constrained `dartpy.gui.experimental`
bindings now expose the backend-hidden descriptor, picking, frame-translation,
plane/axis drag, debug-line, run-option, viewer lifecycle, and orbit-camera
APIs. They pass the focused Python smoke test in both the default configure and the
OSG-free Filament configure with `DART_BUILD_DARTPY=ON`; the OSG-free configure
also passes the default and drag-and-drop Filament headless smokes. Local
Linux CPython 3.12, 3.13, and 3.14 wheel builds were repaired with
`auditwheel`, verified, and installed-tested from temporary virtual
environments; those installed-wheel smokes confirmed
`dartpy.gui.experimental` and passed the one-box scene descriptor probe. Full
macOS, Windows, and GUI option-matrix wheel coverage remains promotion work.
After moving viewer lifecycle state into `dart-gui-experimental`, full default
validation now passes locally with
`DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 pixi run test-all`, covering lint,
Release/Debug builds, C++ tests, simulation-experimental tests, Python tests,
and docs. The full-suite run exposed a local Mesa GLX hang when
`UNIT_gui_HeadlessViewer` created two headless pbuffer contexts in one process;
the factory test now covers the factory path without requesting a second
headless context, while the dedicated headless construction test remains.

`pixi run test-dart-gui-smoke` now wraps the explicit pinned fetch fallback
for Linux x86_64. It configures with `DART_BUILD_GUI=OFF` and
`DART_BUILD_DARTPY=OFF`, builds `dart_filament_gui`, and runs the default,
hello-world, boxes, hardcoded-design, rigid-chain, rigid-loop, mixed-chain,
coupler-constraint, add-delete-skels, vehicle, hybrid-dynamics,
joint-constraints, free-joint-cases, human-joint-limits, mimic-pendulums,
atlas-puppet, lcp-physics, hubo-puppet, atlas-simbicon,
operational-space-control, wam-ikfast, fetch, tinkertoy, drag-and-drop,
simple-frames, soft-bodies, point-cloud, capsule-ground-contact,
simulation-event-handler, polyhedron, and heightmap headless CTest smokes.
When `DISPLAY` is absent, the
task uses Xvfb and prefers Mesa's EGL vendor file for software rendering. The
Ubuntu CI
workflow has a matching `filament-gui-smoke` job that installs Mesa, Xvfb, and
libc++/libc++abi development packages from apt and runs that task without
relying on a Filament conda package. The MVP PR #2647 merged with hosted
`DART GUI Smoke (GCC)` and `DART GUI Smoke (Clang)` passing.

`feature/filament-gui-completion` is the follow-up branch for work beyond the
MVP, and `feature/filament-gui-full-execution` continues that work in the
separate north-star completion branch requested after MVP PR #2647. It routes
`pixi run ex hello_world` through the Filament example's selectable
`--scene hello-world` fixture by default while keeping the standalone source as
legacy OSG comparison material. It routes `pixi run ex boxes` through the
Filament example's selectable `--scene boxes` fixture by default while keeping
the standalone source as legacy OSG/Bullet comparison material. It routes
`pixi run ex rigid_cubes` through the same Filament `--scene boxes` fixture
while keeping the standalone source as legacy OSG comparison material for
directional force controls and frame-recording options. It routes
`pixi run ex box_stacking` through the same Filament `--scene boxes` fixture
while keeping the standalone source as legacy OSG/ImGui comparison material for
solver selection, gravity controls, and custom key callbacks. It routes
`pixi run ex hardcoded_design` through the Filament example's
`--scene hardcoded-design` fixture, which renders the legacy hand-built
three-link skeleton through backend-hidden descriptors while keeping the
standalone source as legacy OSG comparison material for wireframe rendering and
direct key-controlled joint motion. It routes
`pixi run ex rigid_chain` through the Filament example's
`--scene rigid-chain` fixture, which loads the legacy chain SKEL data and
renders its box-link descriptors while keeping the standalone source as legacy
OSG comparison material for the custom per-step damping hook. It routes
`pixi run ex rigid_loop` through the Filament example's
`--scene rigid-loop` fixture, which loads the legacy loop-chain SKEL data,
restores the loop-closing pose and constraint, and renders its red constrained
links through backend-hidden descriptors while keeping the standalone source as
legacy OSG comparison material for damping and constraint setup. It routes
`pixi run ex mixed_chain` through the Filament example's
`--scene mixed-chain` fixture, which loads the legacy mixed rigid/soft chain
SKEL data and renders its box-link and soft-mesh descriptors while keeping the
standalone source as legacy OSG comparison material for keyboard-applied
external forces. It routes
`pixi run ex coupler_constraint` through the Filament example's
`--scene coupler-constraint` fixture, which builds the legacy paired
mimic/coupler rig layout and renders its box-link and guide-line descriptors
while keeping the standalone source as legacy OSG/ImGui comparison material
for the status overlay and reset controls. It routes
`pixi run ex add_delete_skels` through the Filament example's
`--scene add-delete-skels` fixture, which loads the legacy ground world and
adds deterministic cube skeletons while keeping the standalone source as legacy
OSG comparison material for live q/w add-delete controls. It routes
`pixi run ex vehicle` through the Filament example's selectable
`--scene vehicle` fixture, which loads the legacy vehicle SKEL world and
renders car body, wheel cylinder, ground, and obstacle descriptors while
keeping the standalone source as legacy OSG comparison material for live
throttle and steering controls. It routes
`pixi run ex hybrid_dynamics` through the Filament example's selectable
`--scene hybrid-dynamics` fixture, which loads the legacy fullbody SKEL world
and renders posed biped and ground descriptors while keeping the standalone
source as legacy OSG comparison material for scripted joint commands and
harness toggling. It routes `pixi run ex biped_stand` through the same
Filament fixture while keeping the standalone source as legacy OSG comparison
material for SPD control and perturbation controls. It routes
`pixi run ex joint_constraints` through the Filament example's selectable
`--scene joint-constraints` fixture, which loads the legacy fullbody SKEL world,
renders standing biped and ground descriptors, and runs the SPD balance
controller through a private scene pre-step hook while keeping the standalone
source as legacy OSG comparison material for perturbation shortcuts and harness
toggling. It routes
`pixi run ex free_joint_cases` through the Filament example's selectable
`--scene free-joint-cases` fixture, which builds the legacy zero-gravity
free-joint cases, renders active and transparent reference box descriptors,
and advances the reference bodies through a private torque-free pre-step hook
while keeping the standalone source as legacy OSG/ImGui comparison material
for numeric checks and reference-model controls. It routes
`pixi run ex human_joint_limits` through the Filament example's selectable
`--scene human-joint-limits` fixture, which loads the legacy Kima human SKEL,
enables DART joint-limit enforcement, and renders mesh, multi-sphere, box, and
ground descriptors while keeping the standalone source as legacy comparison
material for the custom TinyDNN-backed arm and leg constraints. It routes
`pixi run ex lcp_physics` through the Filament example's selectable
`--scene lcp-physics` fixture, which renders the legacy contact benchmark's
mass-ratio boxes, stack boxes, dominoes, falling spheres, and ground
descriptors while keeping the standalone source as legacy comparison material
for solver controls, plots, scenario switching, and frame recording. It routes
`pixi run ex mimic_pendulums` through the Filament example's selectable
`--scene mimic-pendulums` fixture, which loads the legacy SDF pendulum rigs and
renders cylinder, box, and ground descriptors while keeping the standalone
source as legacy OSG comparison material for the ImGui solver/debug table. It
routes `pixi run ex atlas_puppet` through the Filament example's selectable
`--scene atlas-puppet` fixture, which loads Atlas and renders the robot,
ground, and selectable hand/foot IK target descriptors while keeping the
standalone source as legacy OSG comparison material for the teleoperation
widget and support-polygon visual. It routes `pixi run ex hubo_puppet` through
the Filament example's selectable `--scene hubo-puppet` fixture, which loads
Hubo and renders the robot, ground, and selectable hand, foot, and wrist-peg IK
target descriptors while keeping the standalone source as legacy OSG
comparison material for the teleoperation widget, support-polygon visual, and
keyboard controls. It routes `pixi run ex atlas_simbicon` through the Filament
example's selectable `--scene atlas-simbicon` fixture, which loads Atlas and
renders the robot and ground descriptors while keeping the standalone source as
legacy OSG/ImGui comparison material for the Simbicon gait controller,
perturbation shortcuts, and panel controls. It routes
`pixi run ex operational_space_control` through the Filament example's
selectable `--scene operational-space-control` fixture, which loads the WAM
arm, renders the ground and selectable red target descriptor, and runs the
task-space controller through a private scene pre-step hook while keeping the
standalone source as legacy OSG comparison material for drag-and-drop axis
constraints. It routes `pixi run ex wam_ikfast` through the Filament example's
selectable `--scene wam-ikfast` fixture, which loads the WAM arm and renders
the robot, ground, and end-effector target descriptors while keeping the
standalone source as legacy OSG comparison material for IKFast solver,
drag-mode, keyboard-shortcut, and posture-reset comparison. It routes
`pixi run ex fetch` through the Filament example's selectable `--scene fetch`
fixture, which loads the legacy MJCF pick-and-place world and renders the
Fetch robot, object, and mocap target descriptors while keeping the standalone
source as legacy OSG/ImGui comparison material for panel, drag-control, and
mocap target update comparison. It routes `pixi run ex tinkertoy` through the
Filament example's selectable `--scene tinkertoy` fixture, which renders the
builder's initial block assemblies, target marker, force line, and reference
axes while keeping the standalone source as legacy OSG/ImGui comparison
material for panel, mouse-picking, and block-add/delete comparison. It routes
`pixi run ex drag_and_drop` through the Filament example's selectable
`--scene drag-and-drop` fixture by default while keeping the standalone source
as legacy OSG comparison material. It also routes
`pixi run ex empty` through the same Filament `--scene drag-and-drop` fixture
while keeping the standalone source as legacy OSG comparison material for
custom world hooks and raw key-event callbacks. It also routes
`pixi run ex simple_frames` through the Filament example's
`--scene simple-frames` fixture, which renders the legacy `SimpleFrame`
hierarchy, marker ellipsoids, and arrow marker through backend-hidden
descriptors while keeping the standalone source as legacy OSG comparison
material. It also routes
`pixi run ex soft_bodies` through the Filament example's
`--scene soft-bodies` fixture, which loads the legacy `softBodies.skel` data
through DART IO and renders its soft meshes through backend-hidden descriptors
while keeping the standalone recorded-playback source as legacy OSG comparison
material. It also routes `pixi run ex point_cloud` through the Filament
example's `--scene point-cloud` fixture, which renders visual point-cloud and
voxel-grid descriptors while keeping the standalone source as legacy OSG/ImGui
comparison material for the robot-mesh sampling controls. It also routes
`pixi run ex capsule_ground_contact` through the Filament example's
`--scene capsule-ground-contact` fixture, which renders capsule and ground
descriptors while keeping the standalone source as legacy OSG comparison
material for the pose-reset controls. It also routes
`pixi run ex simulation_event_handler` through the Filament example's
`--scene simulation-event-handler` fixture, which renders falling body and
sensor-marker descriptors while keeping the standalone source as legacy OSG
comparison material for force/torque controls and force-arrow visualization.
It also routes
`pixi run ex imgui` through the Filament example's default viewer and private
built-in status panel while keeping the standalone source as legacy OSG/ImGui
comparison material for custom-widget extension points. It also routes
`pixi run ex rigid_shapes` through the Filament example's default MVP scene for
broad primitive, mesh, point-cloud, heightmap, soft-mesh, and robot visual
coverage while keeping the standalone source as legacy OSG comparison material
for shape spawning, contact toggles, and collision-detector controls. It also
routes `pixi run ex polyhedron_visual` through the Filament example's
`--scene polyhedron` fixture, which renders the legacy convex hull and wireframe
through descriptor-owned convex-mesh and line-segment renderables. It also
routes `pixi run ex heightmap` through the Filament example's
`--scene heightmap` fixture, which renders a local heightmap surface and
reference markers through descriptor-owned heightmap renderables while keeping
the standalone source as legacy OSG/ImGui comparison material for the sculpting
and contact-alignment controls. It also includes the Filament version of the G1
puppet example: `--scene g1` loads the
Unitree G1 URDF through DART resource retrievers, exposes colored IK targets
for both hands and feet, registers active support geometry on the foot targets
for support-polygon overlay inspection, and routes `pixi run ex g1_puppet`
through the Filament example by default. This branch is intentionally separate
from the merged MVP PR #2647.

The follow-up branch also extends the backend-hidden shape descriptor and
renderer path to `PyramidShape`, `MultiSphereConvexHullShape`, and
`LineSegmentShape`, plus generated `ConvexMeshShape` and `PointCloudShape`
visuals, generated `HeightmapShape` terrain, and a generated `SoftMeshShape`
soft-body surface, plus an OctoMap-backed `VoxelGridShape` cell cluster when
available. The MVP scene now includes pyramid, multi-sphere, line-segment,
convex-mesh, point-cloud, heightmap, soft-mesh, and voxel-grid fixtures, and
the example startup checks that all enabled descriptors are extracted and
converted into Filament renderables. Convex mesh, heightmap, and soft mesh
rendering now consumes descriptor-owned triangle data instead of concrete shape
dynamic casts. Unsupported shapes now produce diagnostic descriptors instead of
being silently dropped by the extraction layer.
The Filament example scene option parsing and dispatch now live in
`dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp`, while reusable
DART world fixtures now live in
`dart/gui/experimental/detail/filament/scene_fixtures.hpp` and `.cpp`.
Scene content requirement counting and
MVP/G1/hello-world/boxes/hardcoded-design/rigid-chain/rigid-loop/mixed-chain/
coupler-constraint/add-delete-skels/vehicle/hybrid-dynamics/joint-constraints/
free-joint-cases/human-joint-limits/lcp-physics/mimic-pendulums/atlas-puppet/
hubo-puppet/atlas-simbicon/operational-space-control/wam-ikfast/fetch/
tinkertoy/drag/
simple-frames/soft-bodies/
point-cloud/capsule-ground-contact/simulation-event-handler/polyhedron/
heightmap validation gates, including created-renderable content counting, now live in
`dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`.
The Filament example frame lifecycle, scene synchronization, capture, built-in
panel wiring, and top-level orchestration now live in
`dart/gui/experimental/detail/filament/application.hpp` and `.cpp`, leaving
`examples/filament_gui/main.cpp` as a minimal entry point. The application unit
depends on the private GUI runtime surfaces directly and leaves DART scene
fixture/import dependencies in `scene_fixtures.hpp` and `.cpp`.
Frame rendering, skipped-frame accounting, screenshot request dispatch,
rendered-frame profiling, and bounded-run stop checks now live in
`dart/gui/experimental/detail/filament/frame_renderer.hpp` and `.cpp`.
Per-frame framebuffer sizing, ImGui display metrics, viewport/camera updates,
and camera-controller suppression while selection or the built-in UI consumes
the pointer now live in
`dart/gui/experimental/detail/filament/frame_viewport.hpp` and `.cpp`.
The bounded realtime simulation step-count accumulator, world-step application,
lifecycle marking, and simulation profile accounting now live in
`dart/gui/experimental/detail/filament/simulation_stepper.hpp` and `.cpp`.
The built-in status panel rendering now lives in
`dart/gui/experimental/detail/filament/panel.hpp` and `.cpp`; it remains
private MVP policy rather than a promoted panel/tool API.
`MeshShape` triangle geometry, texture coordinates, imported vertex normals,
material, texture-path, and submesh metadata now flow through renderer-hidden
descriptors, and the Filament example consumes that descriptor metadata for
per-part mesh materials without a renderer-side `MeshShape` dynamic cast. The
picking helpers now also use primitive
sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere plus
point-cloud/voxel-grid box-proxy, finite plane-proxy, and triangle-backed
convex-mesh/heightmap/soft-mesh/MeshShape intersections for surface hit points
and normals before falling back to local bounds for other shape descriptors.

`docs/dev_tasks/filament_gui/07-completion-audit.md` maps the current
implementation, verification evidence, and missing promotion gates. Use that
audit before deciding whether the dev task is complete.

Recent follow-ups added render-resource revision tracking, descriptor
diagnostics for supported geometry without renderable payload, retained
renderable shadow-flag reapplication, the Fetch/Tinkertoy/Hubo routed fixtures,
and the recursive example-tree boundary guard for the minimal
`examples/dartsim` shape. The runner now keeps the `--scene all` and
CTest smoke scene set in one `FILAMENT_ALL_SCENES` list, with
the CMake smoke registration using one
`DART_GUI_FILAMENT_SMOKE_SCENE_PAIRS` list and
`python/tests/unit/test_run_cpp_example.py` checking migrated runner defaults,
the smoke regex, and CMake scene pairs for drift.

## Current Branch

`feature/filament-gui-full-execution`, tracking
`origin/feature/filament-gui-full-execution`. Verify with
`git status --short --branch` before editing. The latest pushed checkpoint
before the current working tree is
`0800c54ec18 Restore Hybrid Dynamics defaults`. No rows remain with the exact
`Needs strict audit` state. The current pending checkpoint is
`examples/joint_constraints/` strict re-open. The Hybrid checkpoint was pushed
as `0800c54ec18 Restore Hybrid Dynamics defaults`; compare
`examples/joint_constraints/` against
`520993d7301^:examples/joint_constraints` before coding. The current
implementation restores the missing README, 640x480 default, loaded SKEL
names/visuals, historical perturbation and harness console messages, and
marker coverage. A later local R24-14 checkpoint restores the historical custom
camera up vector through public `dart::gui::OrbitCamera::up`. Local validation
is complete: focused build/CTest, direct and pixi 640x480 headless screenshots, Python
example-runner tests, aggregate `examples` build, `git diff --check`,
mandatory `pixi run lint`, post-lint focused build/CTest, and post-lint direct
screenshot smoke passed.

## Current Immediate Next Step

Audit `examples/joint_constraints/` next: compare the current source and README
to `520993d7301^:examples/joint_constraints`, challenge the recent parity
checkpoint, and verify perturbation controls, harness toggle behavior,
camera/run defaults, loaded SKEL visuals, README, capture, and source-marker
guards. The code/docs edits and local validation are complete; commit and push
the checkpoint without opening a PR, then continue the strict re-open cursor at
`examples/lcp_physics/`. A later broader smoke sweep can still use:

```bash
pixi run test-dart-gui-smoke
```

Keep tracking Filament package availability as a longer packaging concern, but
do not start conda-forge/feedstock work in this branch. Keep the ImGui Docking,
docked 3D scene widget, first-class offscreen API, and video capture work as
follow-up application/capture tasks unless maintainers explicitly reprioritize
them over the required example restoration and capture compatibility work.

## Context That Would Be Lost

- Keep the public namespace as `dart::gui`; the goal is a backend-hidden API,
  not a new `dart::viz` namespace.
- Filament is preferred because it provides the rendering features needed for a
  high-quality built-in visual-debugging workflow while keeping scope
  maintainable. Full production-rendering scope remains out of scope unless
  maintainers explicitly broaden the project scope.
- The MVP must include a shadowed body/ground fixture, and promotion must pass
  `docs/dev_tasks/filament_gui/06-visual-quality.md`.
- Do not make DART a multi-backend rendering project. OSG and Raylib are legacy
  or experimental paths to remove after the Filament path is promoted.
- A concrete full-migration metric is zero direct Filament header includes from
  maintained examples, including `examples/dartsim`; Filament headers should be
  confined to private promoted GUI implementation units.
- A companion API-cleanliness metric is that `examples/dartsim` remains only a
  minimal `main.cpp` entry point plus unavoidable build/docs files. Renderer
  setup, frame lifecycle, resources, synchronization, capture, overlays, input
  translation, and fixture logic should live in `dart::gui` or private GUI
  implementation units.
- Filament package availability and matching material compiler tooling are the
  primary risks. Fetch fallback must be explicit and pinned.
- The current ImGui overlay is deliberately narrow. Its Filament renderer is
  private GUI detail, and the built-in panel policy must stay internal or be
  wrapped in DART-owned panel/tool abstractions before first-class promotion.

## How to Resume

```bash
git status
find docs/dev_tasks/filament_gui -maxdepth 1 -type f -print | sort
```

For the current branch, the MVP implementation already exists and #2647 is
merged. Read `10-active-execution.md` and `STEERING.md`, then continue the
documented capture compatibility slice on the tracked branch without opening a
PR unless maintainers explicitly ask for one.
