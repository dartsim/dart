# Resume: Filament GUI Replacement

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
normals, and contact force vectors, with graphics-free unit coverage. It also
has picking bounds and nearest ray-hit tests for visible, hidden, hit, and miss
cases. The example wires those helpers into basic click-to-select highlighting
with a selection bounds overlay, and reports the selected DART shape in the
built-in panel. Backend-hidden free-joint, simple-frame, and combined
frame-renderable translation helpers are covered by C++ and Python tests, and
the example uses the combined helper for keyboard nudging of selected dynamic
bodies and `SimpleFrame` visuals. Backend-hidden plane intersection and
plane-drag translation helpers are also covered by C++ and Python tests, and the
example uses them for Ctrl-left camera-plane dragging of selected dynamic bodies
and `SimpleFrame` visuals. Backend-hidden
run-option normalization, viewer lifecycle state, and orbit-camera helpers now
live in `dart-gui-experimental`; the Filament example uses them for bounded
screenshots, camera placement, headless runs, pause/step behavior, frame
accounting, and perspective pick rays. Backend-hidden RGBA-to-PPM screenshot
storage also now lives in `dart-gui-experimental`; the Filament example keeps
only renderer readback.
`UNIT_dynamics_MeshShape` also loads the checked-in
`data/gltf/pbr_triangle.gltf` and `data/gltf/pbr_multi_material.gltf` fixtures
through the real Assimp importer and verifies authored glTF PBR texture slots,
material factors, alpha-bearing material color, UV metadata, and
single-/multi-material submesh ranges without requiring a graphics context. The
Filament smoke scene also loads these fixtures and uses transparent lit
material variants for alpha-bearing solid, textured, and mesh visual paths.
`examples/gui_scene_diagnostics` now gives `dart-gui-experimental` a second,
non-rendering example consumer for descriptor/debug/camera diagnostics,
including a `SimpleFrame` visual, without adding another backend. The
Filament example's built-in panel now exposes pause/resume, single-step, and
debug overlay toggles for grid, frames, center-of-mass markers, contacts,
normals, and force vectors. The same executable now supports
`--scene drag-and-drop`, a first interaction-heavy fixture that carries the
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
`6b20da57ce864edb5bb4080a2b4a8e312b4c0a22`; staged-recipes linter,
conda-forge-linter, Check Skip, Azure linux_64, osx_64, win_64, and aggregate
checks passed on that head. The previous Linux failure came from using
`source_files: test-cmake`; the current head uses `files: test-cmake` for the
recipe-local CMake consumer test and links that test through CMake OpenGL
package targets. `@conda-forge/help-c-cpp` has been pinged, and the feedstock is
not created yet.

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
plane-drag, debug-line, run-option, viewer lifecycle, and orbit-camera APIs. They
pass the focused Python smoke test in both the default configure and the
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
`DART_BUILD_DARTPY=OFF`, builds `dart_filament_gui`, and runs the default plus
drag-and-drop headless CTest smokes. When `DISPLAY` is absent, the task uses
Xvfb and prefers Mesa's EGL vendor file for software rendering. The Ubuntu CI
workflow has a matching `filament-gui-smoke` job that installs Mesa, Xvfb, and
libc++/libc++abi development packages from apt and runs that task without
relying on a Filament conda package. The MVP PR #2647 merged with hosted
`Filament GUI Smoke (GCC)` and `Filament GUI Smoke (Clang)` passing.

`feature/filament-gui-completion` is the follow-up branch for work beyond the
MVP. It includes the Filament version of the G1 puppet example: `--scene g1`
loads the Unitree G1 URDF through DART resource retrievers, exposes colored IK
targets for both hands and feet, and routes `pixi run ex g1_puppet` through the
Filament example by default. This branch is intentionally separate from the
merged MVP PR #2647.

The follow-up branch also extends the backend-hidden shape descriptor and
renderer path to `PyramidShape`, `MultiSphereConvexHullShape`, and
`LineSegmentShape`. The MVP scene now includes pyramid, multi-sphere, and
line-segment fixtures, and the example startup checks that all three
descriptors are extracted and converted into Filament renderables.

`docs/dev_tasks/filament_gui/07-completion-audit.md` maps the current
implementation, verification evidence, and missing promotion gates. Use that
audit before deciding whether the dev task is complete.

## Current Branch

`feature/filament-gui-completion`, stacked on the merged MVP PR #2647. Verify with
`git status && git branch --show-current` before editing.

## Immediate Next Step

Keep the hosted Ubuntu GCC/Clang Filament smoke jobs green on every follow-up PR
that changes the explicit pinned fetch fallback or Filament example behavior.
Locally, use:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi>
pixi run test-filament-gui-smoke
```

Keep tracking the staged-recipes PR as the preferred future package path. After
it merges, add the `filament-static` output to the Pixi toolchain and validate
`Filament_ROOT=$CONDA_PREFIX`.

After the target links, keep the MVP ImGui overlay example-local unless
promotion needs user extension points; in that case add DART-owned panel/tool
abstractions instead of exposing raw ImGui APIs. Add interaction-heavy example
coverage beyond the current selection/nudging/drag path and first
drag-and-drop and G1 IK fixtures, and add broader human visual review with
larger real authored environment/PBR assets for the shadow/material promotion
gate.

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
- Filament package availability and matching material compiler tooling are the
  primary risks. Fetch fallback must be explicit and pinned.
- The current ImGui overlay is deliberately narrow and example-local. It
  renders built-in controls through Filament buffers, but must stay internal or
  be wrapped in DART-owned panel/tool abstractions before first-class
  promotion.

## How to Resume

```bash
git status
find docs/dev_tasks/filament_gui -maxdepth 1 -type f -print | sort
```

For the current branch, the MVP implementation already exists and #2647 is
merged; use `07-completion-audit.md` and `08-north-star-migration.md` to choose
the next promotion-gate work. Keep follow-up PRs separate from the MVP branch.
