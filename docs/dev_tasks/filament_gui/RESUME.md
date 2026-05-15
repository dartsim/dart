# Resume: Filament GUI Replacement

## Live Supervisor Steering

A parallel evaluator pass (2026-05-14) wrote `STEERING.md` next to this
file. Read it before picking the next slice; it records the live steering
context for the post-MVP branch and should be kept in sync when slices land.

## Last Session Summary

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
example tree and fails configuration if extra example-local C++ source/header
files are added or if the entry point adds a direct Filament header include.
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
conda-forge-linter, Check Skip, Azure linux_64, and win_64 passed, while Azure
osx_64 and the aggregate staged-recipes status were still pending. The previous
Linux failure came from using `source_files: test-cmake`; the current head uses
`files: test-cmake` for the recipe-local CMake consumer test and links that test
through CMake OpenGL package targets. `@conda-forge/help-c-cpp` has been pinged,
and the feedstock is not created yet.

When compatible libc++/libc++abi libraries are supplied, the experimental target
configures, links, and runs locally with Mesa llvmpipe. The explicit
`DART_FETCH_FILAMENT=ON` path also fetched the pinned Linux x86_64 archive and
passed local windowed and headless screenshot smoke checks. The
opt-in headless CTest smoke now also runs a small PPM analyzer that checks the
shadowed fixture region for dark, mid-tone, and bright pixels plus luminance
spread, so a flat-but-nonblank frame fails the smoke gate. The Filament example
also builds through `dart-gui-experimental` with
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

`pixi run test-filament-gui-smoke` now wraps the explicit pinned fetch fallback
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
`Filament GUI Smoke (GCC)` and `Filament GUI Smoke (Clang)` passing.

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
`examples/filament_gui` shape. The runner now keeps the `--scene all` and
CTest smoke scene set in one `FILAMENT_ALL_SCENES` list, with
the CMake smoke registration using one
`DART_FILAMENT_GUI_SMOKE_SCENE_PAIRS` list and
`python/tests/unit/test_run_cpp_example.py` checking migrated runner defaults,
the smoke regex, and CMake scene pairs for drift.

## Current Branch

`feature/filament-gui-full-execution`, stacked on the merged MVP PR #2647. Verify
with `git status && git branch --show-current` before editing.

## Immediate Next Step

Use `07-completion-audit.md` and `08-north-star-migration.md` to choose the next
promotion-gate slice. Keep the hosted Ubuntu GCC/Clang Filament smoke jobs green
on every follow-up PR that changes the explicit pinned fetch fallback or
Filament example behavior. Locally, use:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi>
pixi run test-filament-gui-smoke
```

Keep tracking the staged-recipes PR as the preferred future package path. After
it merges, add the `filament-static` output to the Pixi toolchain and validate
`Filament_ROOT=$CONDA_PREFIX`.

After the target links, keep the MVP ImGui panel/tool policy example-scoped
unless promotion needs user extension points; in that case add DART-owned
panel/tool abstractions instead of exposing raw ImGui APIs. Add
interaction-heavy example coverage beyond the current selection/nudging/drag
path and first drag-and-drop, task-space target, and robot IK fixtures, and add
broader human visual review with larger real authored environment/PBR assets for
the shadow/material promotion gate.

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
  maintained examples, including any surviving replacement for
  `examples/filament_gui`; Filament headers should be confined to private
  promoted GUI implementation units.
- A companion API-cleanliness metric is that any surviving
  `examples/filament_gui` tree contains only a minimal `main.cpp` entry point
  plus unavoidable build/docs files. Renderer setup, frame lifecycle,
  resources, synchronization, capture, overlays, input translation, and fixture
  logic should live in `dart::gui` or private GUI implementation units.
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
merged; use `07-completion-audit.md` and `08-north-star-migration.md` to choose
the next promotion-gate work. Keep follow-up PRs separate from the MVP branch.
