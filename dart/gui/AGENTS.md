# dart/gui/

Agent guidelines for the GUI/rendering module.

## Overview

Current shipped 3D visualization uses OpenSceneGraph (OSG) and Dear ImGui.
Treat that surface as the legacy implementation while the Filament replacement
work is in progress.

The north-star is one Filament-backed `dart::gui` surface, not multiple
maintained renderers. New GUI work should keep public headers free of Filament,
GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, and Raylib types, and should
delete or collapse abstractions that exist only to choose among renderers once
Filament is promoted.

## Current Legacy Components

| Component         | File                    | Purpose                                  |
| ----------------- | ----------------------- | ---------------------------------------- |
| Viewer            | `Viewer.hpp`            | Main window, camera, lighting            |
| ViewerConfig      | `ViewerConfig.hpp`      | Configuration for headless/windowed mode |
| ImGuiViewer       | `ImGuiViewer.hpp`       | Viewer + ImGui widgets                   |
| WorldNode         | `WorldNode.hpp`         | Bridge World → OSG scene                 |
| RealTimeWorldNode | `RealTimeWorldNode.hpp` | Real-time simulation loop                |
| DragAndDrop       | `DragAndDrop.hpp`       | Interactive manipulation                 |
| InteractiveFrame  | `InteractiveFrame.hpp`  | 3D manipulator handles                   |

## Subdirectories

- `render/` - Shape-specific renderers (BoxShapeNode, MeshShapeNode, etc.)
- `experimental/` - Backend-hidden scene/debug/picking/run-loop descriptors used by the Filament experiment

## Filament Migration Rules

- Use `dart/gui/experimental/scene.hpp` for testable DART-owned concepts while
  the API is experimental.
- Do not add new public OSG, Raylib, Filament, GLFW, or raw ImGui extension
  points.
- Keep renderer resources private: materials, textures, buffers, entities,
  windows, swap chains, and ImGui draw resources.
- Promote concepts only when they are stable DART concepts such as viewer
  options, renderables, debug draws, selections, tools, panels, screenshots,
  cameras, or simulation run control.
- See `docs/dev_tasks/filament_gui/08-north-star-migration.md` and
  `docs/dev_tasks/filament_gui/09-legacy-surface-audit.md` before changing the
  public GUI boundary.

## Code Patterns

- Legacy OSG control hooks: override `customPreStep()`/`customPostStep()`
- Legacy OSG IK manipulation: use `viewer.enableDragAndDrop(bodyNode)`
- Legacy ImGui widgets: inherit `ImGuiWidget`, implement `render()`
- Legacy OSG headless rendering: `ViewerConfig::headless(w, h)` ->
  single-buffered pbuffer
- Legacy OSG capture: use `captureScreen()` for file output, `captureBuffer()`
  for raw RGBA (headless only)

## Testing

Unit tests: `tests/unit/gui/` (legacy OSG coverage plus experimental Filament
descriptor and public-header boundary guards)

Filament smoke: `pixi run test-filament-gui-smoke` when changing the explicit
fetch path, renderer fixture behavior, or screenshot smoke coverage.

Examples: `examples/imgui/`, `examples/drag_and_drop/`,
`examples/rigid_cubes/` (legacy OSG), `examples/filament_gui/`, and
`examples/gui_scene_diagnostics/` (experimental Filament path)

## See Also

- @docs/onboarding/gui-rendering.md - Current OSG details and Filament status
- @docs/dev_tasks/filament_gui/08-north-star-migration.md - Filament replacement plan
- @docs/dev_tasks/filament_gui/09-legacy-surface-audit.md - Legacy surface audit
- @examples/imgui/README.md - ImGui usage example
