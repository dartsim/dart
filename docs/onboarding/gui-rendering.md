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
- dynamic deformable surface helpers that convert node positions and triangle
  topology into shaded `RenderableDescriptor` meshes without exposing Filament
  resources or depending on a specific solver type

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
  `dart/gui/detail/perf_hud.cpp`. Interpreting the readout: a very high GPU
  frame time (far over the 60 FPS budget) while CPU time is low usually means
  the GPU driver is not active and OpenGL/Vulkan fell back to Mesa software
  rendering (llvmpipe/lavapipe), not that the GPU is slow. Confirm the active
  device (e.g. `nvidia-smi`, `glxinfo`) before optimizing the renderer.
- **GUI scaling.** Built-in ImGui overlays automatically multiply the manual
  `RunOptions::guiScale` / `--gui-scale` user multiplier by GLFW's window
  content scale. `DART_GUI_DPI_SCALE` can override the detected DPI scale for
  remote desktops or window managers that report the wrong value. Interactive
  windows without explicit dimensions use at least a `1600x900` logical window,
  expand toward the primary monitor work area on high-resolution displays, grow
  with the combined user and DPI scale, and cap below the full work area.
  Automatic windows are resized again if GLFW reports a DPI/content-scale change
  while the app is running.
  Explicit `--width` / `--height` values and headless render, screenshot, or
  frame-output dimensions remain authoritative when capture dimensions matter.
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
cmake --build build/default/cpp/Release --target UNIT_gui_DebugVisuals
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_gui_DebugVisuals$'
pixi run ex dartsim --headless --frames 1 --width 1280 --height 720 \
  --screenshot /tmp/dartsim.ppm
pixi run demos -- --scene rigid_body --headless --frames 2 --width 640 \
  --height 480 --out /tmp/rigid_body_frames
```

`--screenshot` writes a single final PPM image. `--out <dir>` writes a numbered
PPM image sequence named `frame_000001.ppm`, `frame_000002.ppm`, and so on.
Inspect screenshots or captured frame sequences when judging visual quality.
Command success alone is not enough for material, lighting, transparency,
camera, or UI regressions.

### Offscreen render-to-texture

`dart/gui/detail/offscreen_parity.cpp` can render the live scene to an offscreen
Filament `RenderTarget` (a color `Texture`, not the swapchain) and verify it
matches the on-screen render. It is the proven seed of the render-to-texture
path used by headless sensor cameras and any future composited/streamed viewer
(see `docs/design/dartsim_gui_toolkit_decisions.md`, Decision 3). It is a
diagnostic — gated by `DART_GUI_OFFSCREEN_PARITY` on the headless path and not in
the default test suite. Run the parity gate (needs a graphics context, e.g.
headless OpenGL/llvmpipe):

```bash
pixi run gui-offscreen-parity
```

It exits non-zero if the offscreen and swapchain renders diverge.

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

## dartsim Editor (DART 7 World)

The standalone GUI simulator lives in the top-level `dartsim/` folder (a runtime
executable distribution, not a library). It is split into a headless editor
engine (`dartsim/engine`, no GUI dependency) that owns a
`dart::simulation::World` plus object/selection/command
(undo/redo)/name managers, an Edit/Run controller, record/replay, and a
human-readable project format; a thin panel layer (`dartsim/ui`) that wires the
engine to `dart::gui`; and the executable entry point (`dartsim/app`). The
engine keeps a SceneModel as the source of truth and rebuilds the DART 7
World from it (that World has no per-object removal), which is also how undo/redo
and Reset work. Design rationale lives in
`docs/design/dartsim_gui_simulator.md`.

The viewport draws the DART 7 scene through
`dart::gui::ApplicationOptions::renderableProvider`, an optional callback that
returns extra `RenderableDescriptor`s appended to the world-derived renderables
each frame. Descriptors only need `id`, `geometry`, `material`, and
`worldTransform`; dynamics pointers stay null. This keeps the renderer
backend-hidden while letting non-legacy-World sources contribute geometry. The
editor hosts an empty legacy world purely as a render canvas (never stepped).
Viewport pick / Scene Tree / Inspector selection now share one engine-owned
selection through `dart::gui::ApplicationOptions::{selectedRenderableProvider,
onRenderableSelected}` and the outliner action seam.

The editor is feature-complete as a workbench. Its key code rule: panel and menu
logic lives in tested, backend-hidden view-model **action seams**
(`dartsim/ui/include/dartsim_ui/*_actions.hpp` — project, history, outliner,
inspector, memory, palette, relationship, simulation, console, watch, viewport),
each with a `UNIT_dartsim_ui_*Actions` test, rather than in anonymous `editor.cpp`
lambdas. Add new behavior as a seam first, then wire it into `editor.cpp` as a
thin view. The filtered `dartsim/engine/*` plus `dartsim/ui/*_actions` surface is
held at ≥95% line coverage (`pixi run coverage-report-dartsim`). The
renderer-backend boundary (no Filament/GLFW/ImGui/OpenGL/Vulkan/Metal in
`dartsim/engine` or `dartsim/ui`, and no `dart/gui` include in the headless
engine) is enforced by `scripts/check_api_boundaries.py`. The native file picker
(`project_file_dialog.cpp`, nativefiledialog-extended) is the one sanctioned
OS/windowing dependency; it crosses the `dart::gui` boundary only as an opaque
`void* parentNativeWindow`. Architecture and the remaining experimental-API-gated
follow-ups live in `docs/design/dartsim_gui_simulator.md`.

## Demos App (World Scenes)

`dart-demos` (`examples/demos/`) is the C++ World demo app: one window that hosts
the current World solver scenes, picked from a categorized sidebar and switched
at runtime via `dart::gui::runDemos`. A scene is a factory returning a
`dart::gui::ApplicationOptions`; add one by writing
`examples/demos/scenes/<name>.cpp`, declaring it in `scenes.hpp`, and registering
it in `registry.cpp`. High-value DART 6 concepts that are not ported yet belong
as lightweight launchable placeholders in `Planned World Ports`, not as retained
legacy scene code. The `dart/gui/detail` `ExampleScene` set is kept as the
renderer's internal test fixtures (renderable-extraction +
`EXAMPLE_dartsim_<scene>` smokes), distinct from the examples. Design rationale:
`docs/design/demos_app.md`.

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
