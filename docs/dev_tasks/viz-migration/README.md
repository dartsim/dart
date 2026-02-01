# Visualization Migration (OSG → Raylib) — Dev Task

## Current Status

- [x] Phase 0: Architecture design and API specification
- [x] Phase 1: Raylib World Viewer MVP (display World, camera, primitives, stepping)
- [x] Phase 2: Interaction parity (input, picking, selection)
- [ ] Phase 3: Manipulation tools parity (drag-and-drop, IK)
- [ ] Phase 4: UI overlay parity (ImGui integration)
- [ ] Phase 5: Migration of shipped content (examples/tutorials)
- [ ] Phase 6: dartpy parity (Python bindings for new viewer)
- [ ] Phase 7: Deprecation, default switch, OSG removal

## Goal

Add a renderer-agnostic scene API to `dart::gui` with a Raylib-based default
backend. The new API coexists with the OSG code in `dart::gui` during migration.

## Non-Goals (for early phases)

- Perfect visual parity with OSG (shadows, advanced materials)
- VSG or Meshcat backends (architecture allows them; not in scope for this epic)
- Preserving OSG-specific extensibility (custom OSG nodes)

## Key Decisions

- **Coexistence in dart::gui**: New code lives alongside OSG code, not in a separate module
- **No backend leakage**: Public headers expose zero Raylib/OSG/OpenGL types
- **Renderer-agnostic scene layer**: `dart::gui::Scene` contains pure data; backends consume it
- **Raylib as default backend**: Zero dependencies, WebAssembly support, excellent debug drawing
- **ImGui kept internal**: Used for UI panels but not exposed in public headers
- **Architecture allows future backends**: VSG, Meshcat, or custom — same scene API
- **SceneViewer naming**: Avoids conflict with existing OSG `dart::gui::Viewer`

## Documents

- Architecture and API: `01-architecture.md`
- Milestones and phases: `02-milestones.md`

## Immediate Next Steps

1. Proceed to Phase 3: manipulation tools parity (drag-and-drop, IK)
2. Or Phase 4: UI overlay parity (ImGui integration)
