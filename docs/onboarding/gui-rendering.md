# Filament GUI Rendering

## Overview

The maintained DART GUI renderer is Filament, with GLFW3 for windowing and
Dear ImGui for built-in overlays. `DART_BUILD_GUI=ON` builds the public
`dart-gui` component, and `DART_BUILD_GUI_FILAMENT=ON` builds the
`dartsim` executable and smoke-test target.

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
- screenshot and frame-output helpers
- profile/debug helpers

These types are stable DART concepts. They are suitable for C++ and Python
bindings because they do not require applications to include renderer,
windowing, or UI toolkit headers.

## Backend Boundary

The private Filament backend lives under
`dart/gui/experimental/detail/filament/` and uses the private
`dart::gui::filament` namespace. It owns:

- Filament engine, renderer, view, scene, camera, swap-chain, and resources
- GLFW native window and input translation
- Dear ImGui overlay state and rendering
- material generation and embedded material packages
- texture and image loading through PNG/JPEG
- renderable synchronization, selection, screenshots, and frame lifecycle

The older `dart/gui/experimental/*.hpp` headers are compatibility shims.
Maintained code should include promoted headers such as
`dart/gui/scene.hpp`, `dart/gui/viewer.hpp`, and `dart/gui/renderable.hpp`.
`dart-gui` is the official component target that links to the private
implementation.

## Build Options

- `DART_BUILD_GUI=ON`: builds the official Filament-backed GUI library.
  Defaults to `ON` on Linux x86_64 source builds where DART can fetch the
  pinned Filament archive; other platforms need an explicit Filament install or
  `Filament_ROOT`.
- `DART_BUILD_GUI_FILAMENT=ON`: builds `dartsim` and related smoke-test
  plumbing. This is selected automatically when `DART_BUILD_GUI=ON`.
- `DART_USE_SYSTEM_FILAMENT=ON`: discovers an installed Filament tree, usually
  through `Filament_ROOT`.
- `DART_FETCH_FILAMENT=ON`: fetches the pinned Linux x86_64 Filament archive
  when no packaged Filament install is available.
- `DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS=ON`: registers bounded headless
  screenshot smoke tests for the DART GUI executable.

OpenSceneGraph and Raylib are no longer buildable renderer options.

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

## Migration Notes

The removed OSG and Raylib paths are not compatibility targets for new work.
When porting old examples or downstream code, preserve DART concepts such as
world attachment, simulation stepping, camera state, picking, debug drawing,
selection, manipulation, screenshots, and diagnostics; do not preserve public
renderer object ownership.

Historical planning and audit details live in:

- `docs/dev_tasks/filament_gui/08-north-star-migration.md`
- `docs/dev_tasks/filament_gui/09-legacy-surface-audit.md`
