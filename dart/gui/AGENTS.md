# dart/gui/

Agent guidelines for the GUI/rendering module.

## Overview

The maintained GUI is one Filament-backed `dart::gui` surface. Filament,
GLFW3, and Dear ImGui are implementation dependencies; public headers should
expose DART-owned scene, viewer, debug, picking, and run-loop concepts rather
than renderer or windowing types.

## Subdirectories

- `experimental/` - Backend-hidden scene/debug/picking/run-loop descriptors and
  the private Filament implementation.

## GUI Rules

- Use `dart/gui/experimental/scene.hpp` for testable DART-owned scene concepts
  and `dart/gui/experimental/viewer.hpp` for testable viewer-runtime concepts.
- Do not add public OSG, Raylib, Filament, GLFW, or raw ImGui extension points.
- Keep renderer resources private: materials, textures, buffers, entities,
  windows, swap chains, and ImGui draw resources.
- Promote concepts only when they are stable DART concepts such as viewer
  options, renderables, debug draws, selections, tools, panels, screenshots,
  cameras, or simulation run control.
- See `docs/dev_tasks/filament_gui/08-north-star-migration.md` and
  `docs/dev_tasks/filament_gui/09-legacy-surface-audit.md` before changing the
  public GUI boundary.

## Testing

Unit tests: `tests/unit/gui/` (Filament descriptor, scene extraction, and
public-header boundary guards)

Filament smoke: `pixi run test-filament-gui-smoke` when changing the explicit
fetch path, renderer fixture behavior, or screenshot smoke coverage.

Examples: `examples/dartsim/`, restored GUI example launchers, and
`examples/gui_scene_diagnostics/`

## See Also

- @docs/onboarding/gui-rendering.md - Filament GUI architecture and workflow
- @docs/dev_tasks/filament_gui/08-north-star-migration.md - Filament replacement plan
- @docs/dev_tasks/filament_gui/09-legacy-surface-audit.md - Legacy surface audit
