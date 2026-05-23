# DART GUI Rendering

## Overview

The maintained DART GUI renderer is Filament, with GLFW3 for windowing and
Dear ImGui for built-in overlays. `DART_BUILD_GUI=ON` builds the public
`dart-gui` component, the `dartsim` executable, and the smoke-test target.

The public GUI surface should describe DART concepts rather than renderer
objects. Public headers must not expose Filament, GLFW, raw Dear ImGui,
OpenGL, Vulkan, Metal, OSG, or Raylib types.

## Public Surface

The public `dart/gui` aggregate includes promoted renderer-independent
descriptor APIs:

- scene and renderable descriptors
- geometry and material descriptors
- debug line descriptors
- picking rays and hit records
- orbit camera and controller helpers
- run options and viewer lifecycle state
- application options and renderer-neutral panel controls
- renderer-neutral transform gizmo descriptors
- screenshot and frame-output helpers
- profile/debug helpers

These types are stable DART concepts. They are suitable for C++ and Python
bindings because they do not require applications to include renderer,
windowing, or UI toolkit headers.

## Backend Boundary

The private Filament backend lives under
`dart/gui/detail/` and uses the private
`dart::gui::detail` namespace. It owns:

- Filament engine, renderer, view, scene, camera, swap-chain, and resources
- GLFW native window and input translation
- Dear ImGui overlay state and rendering
- material generation and embedded material packages
- texture and image loading through PNG/JPEG
- renderable synchronization, selection, screenshots, and frame lifecycle

Maintained code should include public headers such as
`dart/gui/scene.hpp`, `dart/gui/viewer.hpp`, and `dart/gui/renderable.hpp`.
`dart-gui` is the official component target that links to the private
implementation.

## Build Options

- `DART_BUILD_GUI=ON`: builds the official Filament-backed GUI library,
  `dartsim` executable, and smoke-test target.
  Defaults to `ON` on Linux x86_64 source builds where DART can fetch the
  pinned Filament archive; other platforms need an explicit Filament install or
  `Filament_ROOT`.
- `DART_USE_SYSTEM_FILAMENT=ON`: discovers an installed Filament tree, usually
  through `Filament_ROOT`.
- `DART_FETCH_FILAMENT=ON`: fetches the pinned Filament archive for supported
  platforms when no packaged Filament install is available.
- `DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS=ON`: registers bounded headless
  screenshot smoke tests for the DART GUI executable.

OpenSceneGraph and Raylib are no longer buildable renderer options.

## Performance, Profiling, and Backend Selection

The renderer aims for real-time interaction by default while leaving room for
higher-fidelity offline rendering later. Three pieces support that:

- **Rendering backend selection.** The Filament graphics API is chosen at
  runtime as a renderer-neutral DART concept — `dart::gui::RunOptions::renderBackend`,
  the `--render-backend` flag, or the `DART_FILAMENT_BACKEND` environment
  variable (`default`/`opengl`/`vulkan` both GPU, `noop` for CPU-only/no-render;
  env takes precedence). Selection, fallback, and the startup log live in
  `dart/gui/detail/render_context.cpp`; no Filament backend types appear in
  public headers. Embedded materials are compiled for both OpenGL and Vulkan
  (`matc -a opengl -a vulkan` in `backend_sources.cmake`) so the choice needs no
  separate build. `noop` renders nothing and is useful for isolating CPU cost.
- **Profiling.** Use the built-in unified profiler (`dart/common/profile.hpp`,
  text + optional Tracy backends, enabled via `DART_BUILD_PROFILE` /
  `DART_PROFILE_TRACY`); the GUI loop already emits frame and per-phase zones,
  and `--profile` prints a per-phase summary at exit (`dart/gui/profile.hpp`).
  `--perf-hud` (toggleable at runtime with `F2`) adds a live in-app ImGui overlay
  with smoothed CPU per-phase timings, GPU frame time from Filament frame-info,
  FPS, a real-time-factor (sim-vs-wall) readout that shows whether playback is
  keeping up, and fixed-scale history plots against the 60 FPS budget — see
  `dart/gui/detail/perf_hud.cpp`.
- **Scene-sync cost.** Per-frame scene extraction is the dominant CPU cost, not
  GPU rendering. `dart::gui::RenderableExtractor` caches each shape's geometry by
  shape version so `describeShape` is not rebuilt every frame for static
  geometry; kinds whose geometry changes without a version bump (soft mesh, point
  cloud, voxel grid) are rebuilt every frame. This reuses the same version-based
  caching assumption the Filament resource sync relies on — see
  `dart/gui/renderable.cpp`.

## Validation

For renderer changes, use the focused scene extraction tests plus at least one
bounded headless render:

```bash
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_gui_FilamentSceneExtraction$'
pixi run ex dartsim --headless --frames 1 --width 1280 --height 720 \
  --screenshot /tmp/dartsim.ppm
pixi run ex rigid_cubes --headless --frames 2 --width 640 --height 480 \
  --out /tmp/rigid_cubes_frames
```

`--screenshot` writes a single final PPM image. `--out <dir>` writes a numbered
PPM image sequence named `frame_000001.ppm`, `frame_000002.ppm`, and so on.
Inspect screenshots or captured frame sequences when judging visual quality.
Command success alone is not enough for material, lighting, transparency,
camera, or UI regressions.

## Native Collision Visual Checks

`examples/collision_sandbox` is the maintained native collision visual
verifier. Keep its pair registry data-driven so the selector, unsupported
placeholders, focused registry tests, and headless smoke captures stay aligned
with the native narrow-phase support table. Broad-phase debug rendering should
consume copied native debug snapshots, not mutable tree internals, and the GUI
path should use public `dart::gui` APIs without adding FCL, Bullet, or ODE as
runtime example dependencies.

Persistent debug geometry attached to a `World` should call
`dart::gui::applyDebugVisualStyle` rather than setting visual aspect color and
shadow state ad hoc. Built-in `DebugLineDescriptor` renderables and
world-backed debug `ShapeFrame`s should share the same no-shadow policy.
Collision object posing should use public `dart::gui::Gizmo` handles so visual
pair checks stay mouse-driven; keep ImGui panels for mode and parameter
controls rather than as the primary transform editor.

## Migration Notes

The removed OSG and Raylib paths are not compatibility targets for new work.
When porting old examples or downstream code, preserve DART concepts such as
world attachment, simulation stepping, camera state, picking, debug drawing,
selection, manipulation, screenshots, and diagnostics; do not preserve public
renderer object ownership.

Historical GUI example parity is guarded by source-marker tests and the
examples themselves, not by long-lived working notes. Restored examples should
own their scene setup, controls, camera defaults, capture behavior, and README
documentation through public `dart::gui` APIs. If an old renderer behavior
does not map directly to Filament, prefer a DART-owned replacement such as
debug lines, gizmos, panel state, or run options rather than exposing backend
types.
