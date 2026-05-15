# Filament GUI Replacement - Completion Audit

## Objective

Execute `docs/dev_tasks/filament_gui/` fully: evaluate Filament as DART's
single built-in visualization direction under `dart::gui`, implement the
experimental path far enough to validate the visual-debugging requirements, and
identify whether it is ready for first-class promotion.

## Current Decision

This task is not complete. The experimental Linux implementation has enough
evidence to continue with Filament, but it is not ready to replace the existing
`dart::gui` implementation.

Do not promote the Filament path or remove legacy GUI code until every
promotion gate below is satisfied.

## Prompt-to-Artifact Checklist

| Requirement                                                                              | Evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Status                                                                                |
| ---------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| Keep the public direction as `dart::gui`, not a new user-facing namespace.               | `dart-gui-experimental` lives under `dart::gui::experimental`; docs state the final component remains `dart::gui`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Satisfied for the experimental layer.                                                 |
| Avoid committing to multiple maintained rendering backends.                              | Filament is the preferred candidate in `00-renderer-selection.md`; old Raylib dev-task docs were removed.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Satisfied for this plan; legacy code still exists until replacement.                  |
| Make the real north-star a full Filament replacement, not an MVP-only add-on.            | `08-north-star-migration.md` explicitly says PR #2647 is only MVP evidence and that the separate north-star branch/PR must replace the main OSG GUI, the Raylib smoke path, current experimental visualization paths, and backend-agnostic scaffolding whose only remaining purpose was to support multiple renderer implementations. `09-legacy-surface-audit.md` maps the current public OSG GUI headers, legacy dartpy GUI bindings, Raylib build/example support, and maintained examples into surfaces to keep as DART concepts, make Filament-private, or remove/leave unsupported. GUI, build, testing, and dartpy onboarding docs now identify OSG/Raylib as legacy current-state details and route new visualization work to the Filament north-star plan.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Satisfied as the documented direction; implementation and removal remain gated.       |
| Use Filament only if it can meet high-quality visual-debugging needs, including shadows. | `06-visual-quality.md` lists required gates; the example configures PCSS/cascaded sun shadows, contact shadows, neutral sky/indirect lighting, high-quality color grading, anisotropic texture sampling, lit materials, textures, debug overlays, and screenshot readback.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Partially satisfied; local evidence exists, but promotion gates remain.               |
| Provide experimental build plumbing.                                                     | `DART_BUILD_GUI_FILAMENT`, `DART_USE_SYSTEM_FILAMENT`, `DART_FETCH_FILAMENT`, `cmake/dart_find_filament.cmake`, and material embedding exist.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Satisfied for Linux experiments.                                                      |
| Keep Filament optional and explicit.                                                     | Default configure keeps `DART_BUILD_GUI_FILAMENT=OFF`; fetch requires explicit opt-in.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Satisfied.                                                                            |
| Provide package-manager path or explicit fallback.                                       | Pinned CMake fetch works locally with compatible libc++/libc++abi. `pixi search filament` still reports no installable conda-forge package, but `conda-forge/staged-recipes#33297` is an open ready-for-review PR for Filament 1.71.3 with `filament` and `filament-static` outputs.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Tracked externally; package PR is not merged and does not block Linux smoke.          |
| Build a maintained Filament MVP example.                                                 | `examples/filament_gui` builds and runs with the explicit fetch path.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Satisfied on local Linux.                                                             |
| Render visible dynamic shadows.                                                          | Local windowed and headless screenshot smokes produced nonblank images from the shadow-enabled fixture, and the opt-in headless CTest now verifies scene-region luminance contrast from that fixture. The merged MVP PR #2647 Ubuntu Filament smoke jobs passed under both GCC and Clang.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Satisfied for local and hosted Linux smoke; broader visual review remains.            |
| Render common primitives and DART mesh shapes.                                           | The renderer consumes box, sphere, ellipsoid, cylinder, cone, capsule, pyramid, multi-sphere, line-segment, convex-mesh, point-cloud, heightmap, soft-mesh, OctoMap-backed voxel-grid when available, TriMesh-backed `MeshShape`, and finite `PlaneShape` descriptors. Convex mesh, heightmap, soft mesh, and `MeshShape` renderables now use descriptor-owned triangle data instead of concrete shape dynamic casts, point-cloud renderables consume descriptor-owned per-point colors when present, and retained renderables reapply descriptor shadow flags during scene synchronization.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Satisfied for MVP coverage.                                                           |
| Preserve useful mesh material data.                                                      | `MeshMaterial` now preserves typed base-color, metallic, roughness, combined metallic-roughness, normal, occlusion, and emissive texture paths plus PBR scalar factors; `MeshShape` exposes submesh ranges; the experimental descriptors now carry mesh material, texture-path, texture-coordinate, imported vertex-normal, submesh, and alpha-mode metadata; the example consumes those descriptors for per-part materials with color-space-aware Filament textures, fallback samplers, and DART-compatible material-alpha policy; `data/gltf/pbr_triangle.gltf` and `data/gltf/pbr_multi_material.gltf` cover authored glTF PBR texture slots, alpha-bearing factors, UV metadata, and single-/multi-material submesh ranges through the real Assimp importer; the Filament smoke fixture now requires the glTF fixtures, the full WAM URDF skeleton with a minimum WAM mesh descriptor/renderable count, a larger Atlas DAE torso mesh descriptor/renderable, a full Atlas SDF robot fixture with at least twenty visible mesh descriptors/renderables, and a four-panel glTF/PBR environment fixture; fixture descriptor/renderable requirement counting and validation now live in `scene_requirements.hpp`/`.cpp` rather than the viewer loop. | Satisfied for MVP coverage; broader human visual review remains.                      |
| Provide renderer-hidden scene extraction.                                                | `dart/gui/experimental/*.hpp` exposes renderable, debug, interaction, geometry, viewer-runtime, and profile concepts without Filament types; the constrained public surface is split across `renderable.hpp`, `interaction.hpp`, `debug.hpp`, `geometry.hpp`, `viewer.hpp`, and `profile.hpp`, while `scene.hpp` remains an aggregate compatibility include. Implementation is split by responsibility across `renderable.cpp`, `shape_descriptions.cpp`, `interaction.cpp`, `debug.cpp`, `geometry.cpp`, `viewer.cpp`, and `profile.cpp`. `UNIT_gui_FilamentSceneExtraction` covers the path, including mesh material/submesh/UV/normal/alpha-mode metadata, triangle data for convex-mesh/heightmap/soft-mesh/MeshShape rendering and picking, diagnostic descriptors for unsupported shapes and supported-but-empty geometries, and renderer-hidden renderable set update planning for descriptor add/remove/visibility and active render-resource changes, including dynamic soft-mesh vertex updates and alpha-only visual changes.                                                                                                                                                                                                             | Satisfied for the constrained experimental API.                                       |
| Provide debug overlays for physics state.                                                | Grid, world/body frames, centers of mass, inertia boxes, collision-shape bounds, contacts, normal arrows, force arrows, support-polygon outlines, support-centroid markers, and selection bounds are generated as debug descriptors. The Filament example renders debug descriptors through transparent line materials, exposes inertia-box and collision-bound overlay toggles, and the G1 scene exposes active foot support geometry for the support-polygon overlay. Debug-line overlay refresh and cleanup now live in private Filament GUI detail rather than the application loop.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Satisfied for MVP coverage; broader scenario coverage remains.                        |
| Render transparent debug and visual elements.                                            | Debug-line, solid-lit, and textured-lit transparent material paths exist. The Filament smoke fixture includes alpha-bearing visual aspects and an alpha-bearing glTF mesh material.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Satisfied for MVP coverage; broader visual review remains.                            |
| Decide how to handle the MVP ImGui overlay.                                              | `01-architecture.md` keeps built-in ImGui panel policy MVP-scoped and requires DART-owned panel/tool abstractions, or a strictly internal overlay policy, before first-class promotion. The Filament-native ImGui context setup, renderer, and built-in status panel implementations now live under private GUI detail rather than in the example tree.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Satisfied as a design decision; promoted panel/tool API remains.                      |
| Provide basic interaction.                                                               | Click-to-select, highlight tint, selection bounds, pause/resume, single-step, keyboard nudging, and Ctrl-left camera-plane dragging for selected dynamic bodies and `SimpleFrame` visuals exist in the example; `--scene drag-and-drop` exercises a first interaction-heavy `SimpleFrame` fixture; `--scene g1` loads a Unitree G1 robot and exposes IK targets for both hands and feet through the same selection/drag/nudge controls; viewer lifecycle state, orbit-camera controller state, bounds picking with hit point and hit normal, primitive sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane picking with surface normals, triangle-backed mesh picking with surface normals, free-joint, simple-frame, combined frame-translation, camera-relative nudge, and plane-drag helpers have C++ and Python coverage.                                                                                                                                                                                                                                                                                                                                                                                   | Satisfied for MVP coverage; broader interaction-heavy workflows remain.               |
| Provide screenshot and bounded-run smoke behavior.                                       | `--frames`, `--screenshot`, and `--headless` exist; local smokes produced PPM files; the reusable RGBA-to-PPM writer lives in `dart-gui-experimental` with C++ and Python coverage; an opt-in CTest smoke hook validates the header, size, nonzero samples, and scene-region luminance contrast; merged PR #2647 ran the same smoke path in hosted Ubuntu CI for GCC and Clang.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Satisfied for local and hosted Linux smoke; broader CI coverage remains.              |
| Keep public headers free of Filament/GLFW/ImGui types.                                   | `dart-gui-experimental` descriptors and Python bindings do not expose renderer/windowing types. `UNIT_gui_FilamentSceneExtraction` now also scans installed `dart/gui/experimental/*.hpp` headers for Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, and Raylib implementation tokens through a reusable backend-token helper with a reserved promoted-header hook; the focused Python test also checks the `dartpy.gui.experimental` stub surface for backend-specific names.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Satisfied for the constrained API.                                                    |
| Keep maintained examples free of direct Filament headers after promotion.                | The north-star metric is now explicit: any surviving replacement for `examples/filament_gui` must include zero direct `#include <filament/...>` or `#include "filament/..."` directives. Filament headers should move under private promoted GUI implementation units, with examples using DART-owned viewer, scene, debug, capture, and tool APIs. `UNIT_gui_FilamentSceneExtraction` now guards the incremental state where any remaining `examples/filament_gui/*.hpp` files and `examples/filament_gui/main.cpp` have no direct Filament header includes, and the render-context, screenshot-readback, render-environment, texture, renderable, input, scene-fixture, selection, application, ImGui overlay, and native-window implementation slices have moved under `dart/gui/experimental/detail/filament`. The current branch satisfies the direct-include scan for `examples/filament_gui`, but the full promotion metric still requires examples to stop depending on private detail headers and use a promoted DART GUI API.                                                                                                                                                                                                              | Partially satisfied; this remains a first-class promotion gate.                       |
| Keep maintained examples minimal after promotion.                                        | The north-star metric now also requires any surviving `examples/filament_gui/` tree to shrink to a minimal executable entry point. Renderer setup, frame lifecycle, material/texture resources, scene synchronization, capture, overlays, input translation, and reusable fixture logic should be encapsulated by `dart::gui` or private GUI implementation units, not preserved as example-local architecture. The current branch leaves `examples/filament_gui/main.cpp` as a minimal C++ entry point that delegates to private GUI detail, and `UNIT_gui_FilamentSceneExtraction` now guards that the example tree contains no C++ source/header files other than `main.cpp`. The called implementation is still private experimental API rather than a promoted DART GUI API.                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Satisfied for the current example shape; promoted API remains a gate.                 |
| Add Python access only for constrained backend-hidden APIs.                              | `dartpy.gui.experimental` exposes descriptors, renderable set update planning, picking with bounds, primitive sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere/point-cloud/voxel-grid/plane hit normals, and triangle-backed mesh hit normals, frame-translation helpers, plane-drag helpers, debug lines, run options including GUI scale normalization, viewer lifecycle state, screenshot storage, orbit-camera helpers, orbit-camera controller helpers, directional nudge helpers, and perspective projection/clipping helpers; checked-in stubs and Python API docs now list the same backend-hidden submodule for IDE and RTD fallback consumers; focused Python tests pass in both default and OSG-free Filament configures. `scripts/test_wheel.py` now exercises a small `dartpy.gui.experimental` scene-descriptor smoke when that submodule is present in a wheel, and the repaired Linux CPython 3.12, 3.13, and 3.14 wheels pass that installed-wheel smoke locally.                                                                                                                                                                                                                                                        | Satisfied for constrained bindings; full cross-platform wheel matrix remains pending. |
| Provide at least two experimental API consumers.                                         | `examples/filament_gui` and `examples/gui_scene_diagnostics` both consume `dart-gui-experimental`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Satisfied.                                                                            |
| Validate OSG-free experimental path.                                                     | The Filament example can configure/build with `DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF`; `dartpy.gui.experimental` also builds/tests with `DART_BUILD_GUI=OFF`, `DART_BUILD_GUI_FILAMENT=ON`, and `DART_BUILD_DARTPY=ON`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Satisfied locally.                                                                    |
| Validate platform support.                                                               | Local Linux builds/smokes pass.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Not satisfied for macOS and Windows; requires CI workers or external validation.      |
| Integrate CI.                                                                            | Suggested CI stages are documented; `pixi run test-filament-gui-smoke` exercises the explicit pinned fetch path with `DART_BUILD_GUI=OFF`, builds `dart_filament_gui`, and runs the default plus drag-and-drop headless CTest smokes. The task uses `xvfb-run` when `DISPLAY` is absent and prefers Mesa's EGL vendor file when available so the OpenGL backend can run on headless Linux workers. `.github/workflows/ci_ubuntu.yml` has a `filament-gui-smoke` job that installs Mesa, Xvfb, and libc++/libc++abi development packages and runs that task without relying on a Filament conda package. Merged PR #2647 passed `Filament GUI Smoke (GCC)` and `Filament GUI Smoke (Clang)`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Partially satisfied; hosted Linux smoke passes, broader promotion CI remains.         |
| Promote to first-class `dart::gui`.                                                      | Promotion criteria are documented, and the legacy surface audit identifies which existing APIs must become DART-owned concepts versus private Filament implementation details.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Not satisfied.                                                                        |
| Remove legacy OSG/Raylib paths.                                                          | Legacy removal is documented as a future phase; `09-legacy-surface-audit.md` lists the OSG-shaped public headers, Python bindings, Raylib build option/dependency/example plumbing, and examples that must be ported or removed after promotion.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Not satisfied.                                                                        |

## Verified Commands

Recent local evidence:

```bash
pixi run lint
python -m json.tool data/gltf/pbr_triangle.gltf >/dev/null
python -m json.tool data/gltf/pbr_multi_material.gltf >/dev/null
cmake --build build/default/cpp/Release --target gui_scene_diagnostics UNIT_gui_FilamentSceneExtraction UNIT_dynamics_MeshShape -j2
ctest --test-dir build/default/cpp/Release -R UNIT_gui_FilamentSceneExtraction --output-on-failure
ctest --test-dir build/default/cpp/Release -R UNIT_dynamics_MeshShape --output-on-failure
./build/default/cpp/Release/bin/gui_scene_diagnostics --frames 5 --width 640 --height 480
cmake --build build/default/cpp/Release --target dartpy --parallel 2
pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:${PYTHONPATH:-} pixi run python -m pytest python/tests/unit/gui/test_experimental_scene.py -q
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction dart_filament_gui dartpy --parallel 5
ctest --test-dir build/default/cpp/Release -R UNIT_gui_FilamentSceneExtraction --output-on-failure -j 5
PYTHONPATH=build/default/cpp/Release/python:${PYTHONPATH:-} pixi run python -m pytest python/tests/unit/gui/test_experimental_scene.py -q
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction dartpy --parallel 5
ctest --test-dir build/default/cpp/Release -R UNIT_gui_FilamentSceneExtraction --output-on-failure -j 5
PYTHONPATH=build/default/cpp/Release/python:${PYTHONPATH:-} pixi run python -m pytest python/tests/unit/gui/test_experimental_scene.py -q
pixi run ex filament_gui --headless --frames 1 --width 640 --height 480 --screenshot /tmp/dart_filament_mesh_descriptor.ppm
pixi run python examples/filament_gui/analyze_headless_smoke.py /tmp/dart_filament_mesh_descriptor.ppm --width 640 --height 480
pixi run ex filament_gui --headless --frames 1 --width 640 --height 480 --screenshot /tmp/dart_filament_softmesh.ppm
pixi run python examples/filament_gui/analyze_headless_smoke.py /tmp/dart_filament_softmesh.ppm --width 640 --height 480
LIBCXX_PREFIX=/home/jeongseok/.cache/rattler/cache/cached-envs-v0/f00a128274527791 pixi run test-filament-gui-smoke
LIBCXX_PREFIX=/home/jeongseok/dev/jslee02/dartsim/dart/task_2/.pixi/envs/default pixi run test-filament-gui-smoke
pixi run python -m py_compile scripts/test_wheel.py
PYTHONPATH=build/default/cpp/Release/python:${PYTHONPATH:-} pixi run python -c "import numpy as np; import dartpy; gui = dartpy.gui; world = dartpy.World.create('wheel_gui_experimental'); skel = dartpy.Skeleton('robot'); _, body = skel.create_free_joint_and_body_node_pair(); shape = dartpy.BoxShape(np.array([1.0, 1.0, 1.0])); node = body.create_shape_node(shape); node.create_visual_aspect(); world.add_skeleton(skel); renderables = gui.experimental.extract_renderables(world); assert len(renderables) == 1; assert renderables[0].geometry.kind == gui.experimental.ShapeKind.Box"
pixi run -e py312-wheel wheel-build
pixi run -e py312-wheel wheel-repair
pixi run -e py312-wheel wheel-verify
pixi run -e py312-wheel wheel-test
pixi run -e py313-wheel wheel-build
pixi run -e py313-wheel wheel-repair
pixi run -e py313-wheel wheel-verify
pixi run -e py313-wheel wheel-test
pixi run -e py314-wheel wheel-build
pixi run -e py314-wheel wheel-repair
pixi run -e py314-wheel wheel-verify
pixi run -e py314-wheel wheel-test
DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 pixi run test-all
pixi search filament
gh repo view conda-forge/filament-feedstock --json name,url,description,isArchived
gh pr list --repo conda-forge/staged-recipes --state open --search "filament" \
  --json number,title,state,isDraft,url,headRefName,updatedAt --limit 20
gh pr checks 33297 --repo conda-forge/staged-recipes \
  --json name,state,bucket,workflow,link
```

OSG-free Python evidence:

```bash
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_OVERRIDE=OFF \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  DART_BUILD_DARTPY_OVERRIDE=ON \
  DART_ENABLE_FILAMENT_GUI_SMOKE_TESTS_OVERRIDE=ON \
  pixi run config
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dart_filament_gui dartpy -j2
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ctest --test-dir build/default/cpp/Release \
  -R 'EXAMPLE_filament_gui_headless_smoke|EXAMPLE_filament_gui_drag_and_drop_headless_smoke' \
  --output-on-failure
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  PYTHONPATH=build/default/cpp/Release/python:${PYTHONPATH:-} \
  pixi run python -m pytest python/tests/unit/gui/test_experimental_scene.py -q
```

Explicit fetch smoke evidence:

```bash
LIBCXX_PREFIX=/home/jeongseok/.cache/rattler/cache/cached-envs-v0/f00a128274527791
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  pixi run config
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dart_filament_gui -j2
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  LIBGL_ALWAYS_SOFTWARE=1 \
  MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  ./build/default/cpp/Release/bin/filament_gui --headless --frames 10 \
  --screenshot /tmp/dart_filament_gui_quality_headless.ppm
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  LIBGL_ALWAYS_SOFTWARE=1 \
  MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  ./build/default/cpp/Release/bin/filament_gui --frames 10 \
  --screenshot /tmp/dart_filament_gui_quality_windowed.ppm
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  DART_ENABLE_FILAMENT_GUI_SMOKE_TESTS_OVERRIDE=ON \
  pixi run config
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dart_filament_gui -j2
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ctest --test-dir build/default/cpp/Release \
  -R 'EXAMPLE_filament_gui_headless_smoke|EXAMPLE_filament_gui_drag_and_drop_headless_smoke' \
  --output-on-failure
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_OVERRIDE=OFF \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  pixi run config OFF
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dart_filament_gui -j2
```

CI-oriented explicit fetch smoke evidence:

```bash
LIBCXX_PREFIX=/home/jeongseok/.cache/rattler/cache/cached-envs-v0/f00a128274527791 \
  pixi run test-filament-gui-smoke
```

The headless and windowed smoke outputs were nonblank 1280x720 PPM files.
The opt-in headless CTest smoke passed locally and verified a 640x480 PPM
generated by the same headless path, including a scene-region luminance
contrast check from the shadowed fixture. After adding the neutral skybox,
spherical harmonics indirect lighting, high-quality color grading, anisotropic
texture sampling, the full Atlas SDF robot fixture, and the four-panel
glTF/PBR environment fixture, the analyzer reported
`dark=59741, mid=6841, bright=10229, p05=1.2, p95=200.3, spread=199.1`.
The opt-in smoke was rerun after adding the multi-material glTF fixture, full
WAM URDF robot fixture, required Atlas DAE torso mesh fixture, full Atlas SDF
robot fixture, four-panel glTF/PBR environment fixture, selected-body
manipulation, and the screenshot analyzer to the example. The non-rendering
diagnostics example now also exercises a world-owned `SimpleFrame` visual, and
the Filament example includes a selectable `SimpleFrame` visual that shares the
same frame-translation helpers as selected free-joint bodies. The Filament
example now also has a selectable `--scene drag-and-drop` fixture, and the
opt-in CTest smoke set includes a dedicated headless run for that fixture.
The `pixi run test-filament-gui-smoke` shortcut now runs that same CTest set
after configuring the Linux x86_64 explicit fetch fallback with the legacy OSG
GUI target disabled.
Viewer lifecycle state for pause/resume, single-step, screenshot request
tracking, rendered/skipped frame accounting, and bounded-run stop checks now
lives in `dart-gui-experimental` with C++ and Python coverage. The Filament
example uses that state in its main loop, and the explicit fetch smoke passed
after this refactor.
Renderable set update planning now also lives in `dart-gui-experimental` with
C++ and Python coverage. The Filament example uses that backend-hidden plan to
create and destroy renderer resources when descriptor snapshots gain, lose,
hide, or reveal renderables after startup.
The Filament example now also has a selectable `--scene g1` fixture that loads
the Unitree G1 URDF through DART resource retrievers and exposes colored IK
targets for both hands and feet. `pixi run ex g1_puppet` routes to this
Filament scene by default, with `--package-uri`, `--robot-uri`, and
`--package-name` kept as compatibility aliases for the legacy standalone
example options.

Linux CPython 3.12 wheel evidence:

- `pixi run -e py312-wheel wheel-build` produced
  `dartpy-7.0.0-cp312-cp312-linux_x86_64.whl` with
  `DARTPY_HAS_EXPERIMENTAL_GUI=1`.
- `pixi run -e py312-wheel wheel-repair` repaired it with `auditwheel` into
  `dartpy-7.0.0-cp312-cp312-manylinux_2_39_x86_64.whl`.
- `pixi run -e py312-wheel wheel-verify` confirmed the repaired wheel version.
- `pixi run -e py312-wheel wheel-test` installed the repaired wheel into a
  temporary virtual environment, imported `dartpy`, checked required
  submodules, confirmed `dartpy.gui.Viewer` and `dartpy.gui.experimental`, and
  passed the one-box experimental scene descriptor smoke.

Linux CPython 3.13 wheel evidence:

- `pixi run -e py313-wheel wheel-build` produced
  `dartpy-7.0.0-cp313-cp313-linux_x86_64.whl` with
  `DARTPY_HAS_EXPERIMENTAL_GUI=1`.
- `pixi run -e py313-wheel wheel-repair` repaired it with `auditwheel` into
  `dartpy-7.0.0-cp313-cp313-manylinux_2_39_x86_64.whl`.
- `pixi run -e py313-wheel wheel-verify` confirmed the repaired wheel version.
- `pixi run -e py313-wheel wheel-test` installed the repaired wheel into a
  temporary virtual environment, imported `dartpy`, checked required
  submodules, confirmed `dartpy.gui.Viewer` and `dartpy.gui.experimental`, and
  passed the one-box experimental scene descriptor smoke.

Linux CPython 3.14 wheel evidence:

- `pixi run -e py314-wheel wheel-build` produced
  `dartpy-7.0.0-cp314-cp314-linux_x86_64.whl` with
  `DARTPY_HAS_EXPERIMENTAL_GUI=1`.
- `pixi run -e py314-wheel wheel-repair` repaired it with `auditwheel` into
  `dartpy-7.0.0-cp314-cp314-manylinux_2_39_x86_64.whl`.
- `pixi run -e py314-wheel wheel-verify` confirmed the repaired wheel version.
- `pixi run -e py314-wheel wheel-test` installed the repaired wheel into a
  temporary virtual environment, imported `dartpy`, checked required
  submodules, confirmed `dartpy.gui.Viewer` and `dartpy.gui.experimental`, and
  passed the one-box experimental scene descriptor smoke.

Full default validation evidence:

- After moving viewer lifecycle state into `dart-gui-experimental`,
  `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 pixi run test-all` passed
  locally, including lint, Release/Debug builds, default C++ tests,
  simulation-experimental tests, Python tests, and documentation.
- The first full-suite attempt exposed a local Mesa GLX hang in
  `UNIT_gui_HeadlessViewer` when the same test binary created a second
  headless pbuffer context. The factory test now covers the factory path with a
  window-mode config, while the dedicated headless construction test still
  covers headless viewer creation. The focused `UNIT_gui_HeadlessViewer` binary
  and the subsequent full `pixi run test-all` both passed.

Packaging evidence:

- `conda-forge/filament-feedstock` does not exist yet.
- `conda-forge/staged-recipes#33297` is open and ready for review at head
  `6b20da57ce864edb5bb4080a2b4a8e312b4c0a22`; it builds Filament 1.71.3 from
  the official source tarball and splits tools into `filament` and headers/libs
  into `filament-static`. `@conda-forge/help-c-cpp` has been pinged.
- At latest inspection, the current PR head was still open and behind the target
  branch; staged-recipes linter, conda-forge-linter, Check Skip, Azure linux_64,
  osx_64, win_64, and aggregate checks passed on head `6b20da5`. The previous
  Linux failure came from using `source_files: test-cmake`; the current head
  uses `files: test-cmake` for the recipe-local CMake consumer test and links
  that test through CMake OpenGL package targets. The PR is still not merged,
  so no installable feedstock exists yet.

## Remaining Required Work

- Keep the hosted Ubuntu GCC/Clang Filament smoke jobs green on follow-up PRs
  when the explicit pinned fetch path or Filament example behavior changes.
- Land and consume the staged conda-forge Filament package path when available.
  DART should depend on the `filament-static` output once available because it
  provides the headers/static libraries and pulls the matching `matc` tool
  output, but Linux smoke coverage no longer waits on that package.
- Validate macOS and Windows configure/build/runtime behavior.
- Expand CI beyond the current Ubuntu explicit-fetch smoke once the package
  path and cross-platform dependency story are stable.
- Expand visual material validation to broader robot/environment assets with
  larger authored glTF/PBR environment scenes and human visual review beyond the
  current WAM, Atlas, and PBR panel fixtures.
- Finish interaction-heavy workflow migration beyond the current selected-body,
  `SimpleFrame`, drag-and-drop, and G1 IK fixture baseline.
- Implement any promoted DART-owned panel/tool abstraction if the built-in-only
  overlay is not enough for first-class `dart::gui`.
- Complete macOS, Windows, and GUI option-matrix wheel coverage.
- Promote the stable API to `dart::gui` only after all promotion gates pass.
- Remove OSG/Raylib legacy paths only after the deprecation and replacement
  plan is accepted.
