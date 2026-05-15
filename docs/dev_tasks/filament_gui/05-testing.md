# Filament GUI Replacement - Testing and CI

## Current promotion update

This file began as the MVP testing plan. The live branch has promoted Filament
with GLFW3 and Dear ImGui as the only maintained `dart::gui` renderer, renamed
the application-level runner to `dartsim`, restored historical examples as thin
`dart::gui` launchers, and moved C++ renderer-independent declarations into
stable `dart/gui/*.hpp` headers. Treat old `filament_gui` target/test names in
historical sections below as migration history unless a passage explicitly says
it is current.

Current live names:

- Application runner: `dartsim`
- Main GUI component target: `dart-gui`
- Private backend helper naming: `gui_filament` / `DART_GUI_FILAMENT`
- Smoke-test CTest names: `EXAMPLE_dartsim_headless_smoke` and
  `EXAMPLE_dartsim_<scene>_headless_smoke`
- Pixi shortcut: `pixi run test-filament-gui-smoke`

Current capture contract:

- `--screenshot <path>` writes the single final PPM image and is the current CI
  smoke path.
- `--out <dir>` is the next compatibility slice and should write a numbered PPM
  image sequence without reviving OSG.
- Automated video capture, ImGui Docking, docked 3D scene widgets, and a
  first-class offscreen API are follow-up application/capture work, not blockers
  for the immediate example-restoration checkpoint.

## Goals

- Keep the promoted Filament-backed `dart::gui` path working across local and
  hosted CI.
- Keep non-GUI builds unaffected.
- Keep the pinned Filament fetch path and package-manager transition explicit.
- Catch rendering, resource-lifetime, and platform regressions with targeted
  checks rather than broad visual goldens.

## Required gates before promotion

### Dependency gates

- Filament can be found as a conda-forge/Pixi package in the supported
  development workflow, or a pinned explicit CMake fetch path exists for
  developers.
- Required Filament host tools for material compilation are available and
  version-matched to the runtime.
- GLFW and Dear ImGui discovery works through the existing DART dependency
  patterns.
- PNG and JPEG texture-image decoding dependencies are found only for the
  experimental Filament example.
- No GUI option silently fetches Filament without an explicit opt-in.

### Build gates

- `DART_BUILD_GUI_FILAMENT=ON` builds on Linux.
- Before first-class promotion, it must also build on macOS and Windows.
- `DART_BUILD_GUI_FILAMENT=OFF` keeps existing non-Filament builds unaffected.
- `DART_BUILD_GUI=OFF` remains valid for headless/non-GUI users.

### Runtime gates

- MVP renders a nonblank 3D scene in a window.
- MVP renders visible shadows in the default body/ground fixture.
- `--frames <n>` exits deterministically.
- Screenshot/offscreen capture works before broad example migration.
- CI runtime checks are gated on an environment with a usable graphics context.

### Functional gates

- Scene extraction has unit tests that do not require a graphics context.
- Transform updates, geometry cache invalidation, and visual aspect changes are
  covered by tests.
- Picking/selection math is unit-tested before manipulation tools are migrated.

### Quality gates

- Visual quality satisfies the requirements in
  `docs/dev_tasks/filament_gui/06-visual-quality.md` for normal DART debugging
  scenes.
- Shadow quality passes the non-negotiable gate in
  `docs/dev_tasks/filament_gui/06-visual-quality.md`.
- The scope does not expand to a general-purpose production renderer or content
  pipeline.
- Mesh-heavy and point-cloud-heavy examples have acceptable interactive
  performance before OSG removal.

## Suggested CI stages

1. Build-only check with `DART_BUILD_GUI=ON`.
2. Windowed smoke check on a CI worker with a display or virtual display.
3. Opt-in headless/offscreen final-frame capture through
   `EXAMPLE_dartsim_headless_smoke` on graphics-capable workers.
4. Opt-in scene matrix smokes through
   `EXAMPLE_dartsim_<scene>_headless_smoke` for the shared
   `DART_GUI_FILAMENT_SMOKE_SCENE_PAIRS` scene list.
5. Focused C++ unit tests for scene extraction, promoted-header cleanliness,
   lifecycle/capture helpers, and interaction math.
6. Python import/binding smoke tests for `dartpy.gui`, with
   `dartpy.gui.experimental` kept as compatibility coverage.
7. Restored historical example runner tests, including at least one headless
   image-sequence proof after `--out <dir>` compatibility lands.

## Local commands

Default local configuration uses the current platform GUI defaults:

```bash
pixi run config
```

The promoted GUI target builds `dartsim` when the Filament-backed GUI component
is enabled:

```bash
DART_BUILD_GUI_FILAMENT_OVERRIDE=ON Filament_ROOT=<filament-install> pixi run config
cmake --build build/default/cpp/Release --target dart-gui dartsim
./build/default/cpp/Release/bin/dartsim --frames 10 \
  --screenshot /tmp/dart_gui.ppm
pixi run lint
```

For explicit local fetching on Linux x86_64:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi>
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  pixi run config
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dart-gui dartsim
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  LIBGL_ALWAYS_SOFTWARE=1 \
  MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  ./build/default/cpp/Release/bin/dartsim --frames 10 \
  --headless --screenshot /tmp/dart_gui_fetch.ppm
```

The same headless path can be registered as an opt-in CTest smoke check:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi>
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS_OVERRIDE=ON \
  pixi run config
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dartsim
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ctest --test-dir build/default/cpp/Release \
  -R 'EXAMPLE_dartsim_(headless|hello_world_headless|boxes_headless|hardcoded_design_headless|rigid_chain_headless|rigid_loop_headless|mixed_chain_headless|coupler_constraint_headless|add_delete_skels_headless|vehicle_headless|hybrid_dynamics_headless|joint_constraints_headless|free_joint_cases_headless|human_joint_limits_headless|lcp_physics_headless|mimic_pendulums_headless|atlas_puppet_headless|hubo_puppet_headless|atlas_simbicon_headless|operational_space_control_headless|wam_ikfast_headless|fetch_headless|tinkertoy_headless|drag_and_drop_headless|simple_frames_headless|soft_bodies_headless|point_cloud_headless|capsule_ground_contact_headless|simulation_event_handler_headless|polyhedron_headless|heightmap_headless)_smoke' \
  --output-on-failure
```

The CI-oriented shortcut uses the same explicit fetch path, disables the legacy
legacy renderer targets, and builds the DART GUI application before running the CTest
smokes:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi> \
  pixi run test-filament-gui-smoke
```

On Ubuntu CI, libc++/libc++abi are installed from apt for this smoke job so the
workflow does not depend on a Filament conda package.

To validate that the Filament-backed GUI path is independent from the removed
legacy OSG implementation, configure with the no-dartpy preset argument and
enable the GUI explicitly:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi>
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_OVERRIDE=OFF \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  pixi run config OFF
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dartsim
```

When testing the upstream Linux release archive, add compatible libc++ and
libc++abi libraries until Filament is available in the normal Pixi dependency
set:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi>
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  Filament_ROOT=<filament-install> \
  pixi run config
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dartsim
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  LIBGL_ALWAYS_SOFTWARE=1 \
  MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  ./build/default/cpp/Release/bin/dartsim --frames 10 \
  --screenshot /tmp/dart_gui.ppm
```

Known current result:

- `pixi search filament` does not find a conda-forge Filament package.
- `gh repo view conda-forge/filament-feedstock` does not find a created
  feedstock yet.
- `conda-forge/staged-recipes#33297` is an open non-draft PR for
  Filament 1.71.3, with `@conda-forge/help-c-cpp` pinged for review. The recipe
  builds from the official source tarball, keeps host tools in the `filament`
  output, and places headers/static archives in the `filament-static` output.
  Once merged, DART's build-time Pixi dependency should use `filament-static`,
  which pulls the matching tool output needed for `matc`.
- At the latest inspected head `d78834e9bcca1c0c5a55f3d2752f39765932c3f2`,
  the PR was still open, non-draft, and behind the target branch.
  Staged-recipes linter, conda-forge-linter, Check Skip, Azure linux_64,
  osx_64, win_64, and aggregate staged-recipes checks passed. The previous
  Linux failure came from using `source_files: test-cmake`; the current head
  uses `files: test-cmake` for the recipe-local CMake consumer test and links
  that test through CMake OpenGL package targets. The PR is still not merged,
  so DART still has no installable Filament package to consume.
- Upstream Filament release archives include `matc`, headers, and static
  libraries, so `Filament_ROOT` can point at that layout.
- `DART_FETCH_FILAMENT=ON` explicitly fetches the pinned Linux x86_64
  prebuilt archive for local experiments. It is not the preferred packaging
  path and still inherits the archive's libc++ requirement on Linux.
- The upstream Linux archive tested here was built against libc++. The finder
  detects those `std::__1` symbols and requires libc++/libc++abi or a
  compatible Filament build.
- With compatible libc++/libc++abi libraries supplied through
  `CMAKE_LIBRARY_PATH` and `LD_LIBRARY_PATH`, the `dartsim` target configured,
  linked, and ran locally with Mesa llvmpipe.
- The explicit fetch path configured, linked, and ran the same local screenshot
  smoke check with the pinned upstream archive and Mesa llvmpipe.
- The same fetched target rendered a nonblank screenshot through
  `--headless`, exercising Filament's headless swap-chain path without creating
  a GLFW window.
- `DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS=ON` registers
  `EXAMPLE_dartsim_headless_smoke`, an opt-in CTest that runs the default scene
  through the headless path and verifies the generated PPM header, dimensions,
  expected byte count, nonzero sampled pixel data, full-image nonzero pixels,
  and scene-region luminance contrast from the shadowed fixture. The same
  option also registers scene-specific `EXAMPLE_dartsim_<scene>_headless_smoke`
  tests from `DART_GUI_FILAMENT_SMOKE_SCENE_PAIRS`; those fixture smokes use the
  headless path and verify the generated PPM structure and sampled pixels.
- The opt-in CTest smoke passed locally with the explicit fetch path,
  compatible libc++/libc++abi libraries, and Mesa llvmpipe.
- `pixi run test-filament-gui-smoke` passed locally with the same explicit
  fetch path. The task configures with `DART_BUILD_GUI=OFF`,
  `DART_BUILD_DARTPY=OFF`, `DART_FETCH_FILAMENT=ON`, and
  `DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS=ON`, builds `dartsim`, and runs the
  default, hello-world, boxes, hardcoded-design, rigid-chain,
  rigid-loop, mixed-chain, coupler-constraint, add-delete-skels, vehicle,
  hybrid-dynamics, joint-constraints, free-joint-cases, human-joint-limits,
  lcp-physics, mimic-pendulums, atlas-puppet, hubo-puppet, atlas-simbicon,
  operational-space-control, wam-ikfast, fetch, tinkertoy, drag-and-drop,
  simple-frames, soft-bodies,
  point-cloud, capsule-ground-contact, simulation-event-handler, polyhedron,
  and heightmap headless CTest smokes.
  When `DISPLAY` is unset, the task runs CTest
  under `xvfb-run` and prefers Mesa's EGL vendor file when available so the
  OpenGL backend uses software rendering on headless Linux workers.
- `.github/workflows/ci_ubuntu.yml` now includes a `filament-gui-smoke` job
  that installs Mesa, Xvfb, and libc++/libc++abi development packages and runs
  `pixi run test-filament-gui-smoke` without relying on a Filament conda
  package. The MVP PR #2647 merged with hosted
  `Filament GUI Smoke (GCC)` and `Filament GUI Smoke (Clang)` passing.
- The `dartsim` application also configured, linked, and ran with
  `DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF` using
  `pixi run config OFF`, proving the Filament-backed path no longer needs
  the legacy OSG `dart-gui` target.
- `dartpy` also builds with `DART_BUILD_GUI=OFF`, `DART_BUILD_GUI_FILAMENT=ON`,
  and `DART_BUILD_DARTPY=ON`. In that mode, `dartpy.gui` is the promoted
  surface and `dartpy.gui.experimental` remains available as compatibility.
- The local screenshot smoke command produced a nonblank 1280x720 PPM file from
  the shadow-enabled two-box/ground fixture with a visible ImGui overlay.
- `UNIT_gui_FilamentSceneExtraction` builds and passes. It covers the
  `dart-gui-experimental` extraction layer, including primitive, mesh, plane,
  material, visibility, shadow, unsupported-shape diagnostics, picking hit
  points/normals including primitive
  sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane
  surface hits, renderable set update planning, and debug-line descriptors, without
  requiring Filament or a graphics context. The same focused test covers
  backend-hidden orbit-camera controller state and camera-relative nudge math
  used by the Filament example's mouse/cursor and keyboard manipulation paths.
- After moving viewer lifecycle state into `dart-gui-experimental`,
  `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 pixi run test-all` passed
  locally, covering lint, Release/Debug builds, default C++ tests,
  simulation-experimental tests, Python tests, and documentation.
- The Filament smoke fixture now routes extracted primitive, TriMesh, and
  finite checker-textured plane descriptors into the renderer and writes a
  nonblank screenshot through the explicit fetch path.
- The smoke fixture explicitly enables PCSS/cascaded SUN shadows, contact
  shadows, a neutral skybox, spherical harmonics indirect lighting,
  high-quality color grading, anisotropic texture sampling, HDR buffer quality,
  screen-space ambient occlusion, temporal anti-aliasing, FXAA, temporal
  dithering, and multi-sample anti-aliasing for windowed screenshot runs. The
  higher-cost post-process options are disabled for the current headless OpenGL
  software path because they prevent frame acquisition.
- The fixture now includes an imported WAM Collada mesh. The Filament mesh path
  consumes `MeshShape` UV metadata, submesh material ranges, material colors,
  metallic/roughness factors, emissive color, and typed PNG/JPEG texture images
  for base color, metallic, roughness, combined metallic-roughness, normal,
  occlusion, and emissive maps when those are present. `UNIT_dynamics_MeshShape`
  covers extraction of typed PBR texture paths, material factors, and submesh
  ranges from Assimp metadata.
- The smoke fixture also requires the full WAM URDF skeleton to load through
  DART's normal `dart::io` and `dart-utils-urdf` path, exercising a larger
  multi-link robot asset with existing DAE/JPEG material data in the rendered
  scene. The example fails early if the WAM fixture does not produce at least
  five visible mesh renderable descriptors or if any of those descriptors fail
  to create Filament renderables.
- The smoke fixture also requires a larger Atlas torso DAE mesh to load through
  the same `MeshShape` path and create a Filament renderable. The example fails
  early if the Atlas mesh descriptor is missing or cannot be rendered.
- The smoke fixture also requires the full Atlas SDF robot to load through
  DART's normal `dart::io` path. The example fails early if the Atlas robot
  fixture produces fewer than twenty visible mesh descriptors or if any of
  those descriptors fail to create Filament renderables.
- `UNIT_dynamics_MeshShape` also loads `data/gltf/pbr_triangle.gltf` and
  `data/gltf/pbr_multi_material.gltf` through the real Assimp importer,
  covering authored glTF base-color, metallic-roughness, normal, occlusion, and
  emissive texture slots plus UV metadata, alpha-bearing material factors, and
  single-/multi-material submesh ranges without requiring a graphics context.
- The Filament smoke fixture loads the same glTF fixtures and also includes
  alpha-bearing solid and checker-textured visual aspects, exercising
  transparent lit and textured-lit Filament material variants through the
  runtime path.
- The smoke fixture also uses the multi-material glTF PBR asset as a required
  four-panel environment layout. The example fails early if fewer than four
  visible environment mesh descriptors are extracted or if any extracted
  environment descriptor fails to create a Filament renderable.
- The opt-in headless smoke now runs
  `dart/gui/experimental/detail/filament/testing/analyze_headless_smoke.py` on
  the rendered PPM. The default smoke uses the analyzer's contrast mode, which
  samples the fixture region and requires dark, mid-tone, and bright pixels
  plus a minimum luminance spread. The scene-specific smoke matrix uses the
  analyzer's basic mode, which scans each full image for nonzero pixels without
  imposing shadow-fixture thresholds. This catches flat shadowed-fixture
  regressions while keeping broad scene coverage less brittle than
  golden-image comparisons. Focused Python unit coverage now imports the
  analyzer directly and checks accepted/rejected synthetic PPMs for basic mode,
  contrast mode, and the default CLI mode. The example-runner unit coverage
  also checks that the generated scene-smoke CMake registration stays wired to
  `ANALYZE` plus `ANALYSIS_MODE basic`.
- The same smoke fixture renders Filament line primitives from
  `dart-gui-experimental` grid, world/body frame, center-of-mass, contact,
  inertia-box, normal-arrow, and force-arrow descriptors. The debug-line
  material uses transparent blending so alpha-bearing overlay colors can be
  used for translucent visual-debugging marks. The bounded local run reports
  nonzero final contacts from the intentionally contacting fixture body.
  Support-polygon, support-centroid, and inertia-box descriptors have
  graphics-free C++/Python coverage, and the G1 Filament scene exposes active
  foot support geometry through the same static debug overlay path.
  Collision-shape-bound descriptors also have graphics-free C++/Python
  coverage for collision-only shape nodes that are not visual renderables.
- The Filament example panel exposes controls for pause/resume, single-step, and
  toggling the current debug overlay groups. The bounded smoke run verifies that
  these controls compile and render as part of the ImGui overlay path.
- Renderer-independent nearest-hit picking is unit-tested for visible, hidden,
  hit, miss, bounds-normal, and primitive
  sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane
  surface-normal cases. The example wires that path into basic click-to-select
  highlighting plus a
  selection bounds overlay.
- Renderer-independent free-joint and `SimpleFrame` translation are unit-tested
  for selected mutable bodies, fixed-body rejection, and world-owned
  simple-frame visuals. The Filament example wires the combined frame-renderable
  path into keyboard nudging for selected dynamic bodies and `SimpleFrame`
  visuals.
- Renderer-independent plane intersection, plane-drag translation, and
  axis-drag translation are unit-tested in C++ and Python. The Filament example
  wires the plane path into Ctrl-left camera-plane dragging for selected dynamic
  bodies and `SimpleFrame` visuals, and wires the axis path into
  Ctrl-X/Y/Z-left constrained dragging.
- `python/tests/unit/test_run_cpp_example.py` verifies that the migrated visual
  example runners resolve to their restored executable names with intended
  `--scene` defaults, so stale backend-named runner expectations fail locally.
  It also checks that the runner's `--scene all` list, smoke-test regex, and
  loop-generated CMake Filament smoke scene registrations stay aligned.
- `UNIT_gui_FilamentSceneExtraction` also covers run-option normalization,
  viewer lifecycle state, bounded screenshot/frame gates, RGBA-to-PPM
  screenshot storage, orbit-camera basis/update behavior, and perspective
  pick-ray generation in the experimental viewer-runtime layer without
  requiring a graphics context.
- The same unit target covers center-of-mass markers and equivalent inertia-box
  debug descriptors plus collision-shape-bound descriptors generated by the
  backend-hidden debug descriptor layer.
- `pixi run build-py-dev` builds the promoted `dartpy.gui` bindings, and
  `python/tests/unit/gui/test_gui_scene.py` passes
  against the local build tree, including the camera/run helper,
  viewer lifecycle, renderable set update planning, frame-translation,
  plane/axis-drag helpers, and screenshot storage bindings.
- The same Python test passes against the OSG-free Filament configure with
  `DART_BUILD_GUI=OFF`, `DART_BUILD_GUI_FILAMENT=ON`, and
  `DART_BUILD_DARTPY=ON`, covering the promoted `dartpy.gui` module and its
  experimental compatibility namespace without linking the legacy GUI target.
  In that same OSG-free configure, the default and drag-and-drop Filament
  headless smokes also pass.
- `scripts/test_wheel.py` now checks `dartpy.gui` when the GUI module is
  present in a wheel: it builds a one-box world, extracts renderable
  descriptors, and verifies the box shape descriptor.
- The Linux CPython 3.12, 3.13, and 3.14 wheel workflows now build, repair,
  verify, and installed-test repaired manylinux wheels locally. The
  installed-wheel smoke imports `dartpy`, checks the core modules, confirms the
  promoted `dartpy.gui` APIs are present, and runs the one-box scene descriptor
  smoke. This is local Linux CPython 3.12/3.13/3.14 evidence, not full macOS,
  Windows, or GUI option-matrix evidence.
- Full default validation passes locally with
  `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 pixi run test-all`, including
  lint, Release/Debug builds, default C++ tests, simulation-experimental tests,
  Python tests, and documentation. During that validation,
  `UNIT_gui_HeadlessViewer` exposed a local Mesa GLX hang when the same test
  binary created two headless pbuffer contexts; the factory test now covers the
  factory path without requesting a second headless context, while the dedicated
  headless construction test remains in place.
- `gui_scene_diagnostics --frames 5 --width 640 --height 480` builds and runs
  in the default configure, producing descriptor/debug-line counts, including a
  `SimpleFrame` visual descriptor, plus a center-pick result from the shared
  experimental API.
