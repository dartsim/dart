# Raylib GUI Backend (OSG Replacement) — Dev Task

## Current status

- `DART_BUILD_GUI_RAYLIB` exists to gate the experimental Raylib dependency; no standalone Raylib example is shipped yet.
- The shipped GUI library (`dart-gui`) is still the OpenSceneGraph (OSG) backend and its public headers expose OSG types.

## Goal

Replace the OSG-backed GUI as the default visualization path by introducing a Raylib-based backend that reaches feature parity for DART’s supported examples/tutorials and `dartpy`, then deprecating and eventually removing the OSG dependency.

## Non-goals (for early milestones)

- Perfect visual parity (shadows/material system matching OSG).
- Preserving OSG-specific extensibility as-is (e.g., custom OSG nodes) without an explicit migration story.

## Key constraints / why this is non-trivial

- The current `dart::gui` API is not backend-neutral (e.g., `dart::gui::Viewer` inherits from `osgViewer::Viewer`), so “swap OSG for Raylib” is fundamentally an API + component architecture change, not just a renderer rewrite.

## Decisions (so far)

- Clean-break new viewer API (not backward compatible with `dart::gui`).
- Dear ImGui used internally for UI panels/overlays (no ImGui in public headers).

## Documents

- Proposed architecture and boundaries: `docs/dev_tasks/raylib/01-architecture.md`
- Roadmap / milestone criteria: `docs/dev_tasks/raylib/02-milestones.md`
- Migration + deprecation strategy: `docs/dev_tasks/raylib/03-migration.md`
- Testing and CI strategy: `docs/dev_tasks/raylib/04-testing.md`

## Immediate next steps (recommended)

1. Inventory what the OSG backend is used for in shipped examples/tutorials and `dartpy` (feature list, not file lists).
2. Define and review the new, intentionally constrained public API (clean break; no backward compatibility with `dart::gui`).
3. Build a vertical slice: render a `simulation::World` with camera controls and stepping in Raylib (not just a window).
