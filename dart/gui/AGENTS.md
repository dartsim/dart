# dart/gui/

Agent guidelines for the GUI/rendering module.

## Overview

The maintained GUI is one Filament-backed `dart::gui` surface. Filament,
GLFW3, and Dear ImGui are implementation dependencies; public headers should
expose DART-owned scene, viewer, debug, picking, and run-loop concepts rather
than renderer or windowing types.

## Subdirectories

- `detail/` - Private Filament implementation. Maintained code should include
  the public `dart/gui/*.hpp` headers.

## GUI Rules

- Use `dart/gui/scene.hpp` for testable DART-owned scene concepts and
  `dart/gui/viewer.hpp` for testable viewer-runtime concepts.
- Do not add public OSG, Raylib, Filament, GLFW, or raw ImGui extension points.
- Keep renderer resources private: materials, textures, buffers, entities,
  windows, swap chains, and ImGui draw resources.
- Promote concepts only when they are stable DART concepts such as viewer
  options, renderables, debug draws, selections, tools, panels, screenshots,
  cameras, or simulation run control.
- See `docs/onboarding/gui-rendering.md` before changing the public GUI
  boundary.

## Testing

Unit tests: `tests/unit/gui/` (Filament descriptor, scene extraction, and
public-header boundary guards)

DART GUI smoke: `pixi run test-dart-gui-smoke` when changing the explicit
fetch path, renderer fixture behavior, or screenshot smoke coverage.

Examples: the standalone `dartsim/` application, restored GUI example
launchers, and `examples/gui_scene_diagnostics/`

## See Also

- @docs/onboarding/gui-rendering.md - Filament GUI architecture and workflow
