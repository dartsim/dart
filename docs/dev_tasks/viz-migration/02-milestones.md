# Visualization Migration — Milestones

## Phase 0 — Architecture and API Design ✅

**Deliverables**

- Renderer-agnostic `dart::viz` API design (scene data, viewer, backend interface)
- Module scaffolding with CMake integration
- Scene extraction from `dart::simulation::World`

**Exit criteria**

- `dart::viz` compiles as a standalone module with no GPU dependencies
- `SceneExtractor` can traverse a World and produce a `Scene` snapshot

## Phase 1 — Raylib World Viewer MVP

**Deliverables**

- Raylib backend implementing `ViewerBackend`
- Render common primitive shapes (box, sphere, cylinder, capsule, cone, ellipsoid)
- Orbit camera controls (rotate, pan, zoom)
- Simulation stepping (pause, step, real-time)
- Ground grid and coordinate axes

**Exit criteria**

- At least one maintained example rendered end-to-end through `dart::viz` + Raylib

## Phase 2 — Interaction Parity

**Deliverables**

- Backend-agnostic input event model (keyboard, mouse)
- Picking/selection support (select bodies/frames via ray cast)
- Highlight selected objects

**Exit criteria**

- Basic interactive workflows from tutorials work without OSG

## Phase 3 — Manipulation Tools

**Deliverables**

- Drag-and-drop body manipulation (or documented replacement UX)
- Backend-neutral IK/constraint logic for manipulation

**Exit criteria**

- Commonly used manipulation features available on Raylib backend

## Phase 4 — UI Overlay (ImGui)

**Deliverables**

- Dear ImGui integrated into Raylib backend (internal, not in public API)
- DART-owned panel/tool abstraction for simulation controls, inspectors
- ImGui widget system for custom panels

**Exit criteria**

- Essential built-in panels run on Raylib using constrained API

## Phase 5 — Example/Tutorial Migration

**Deliverables**

- Maintained examples/tutorials work without OSG by default
- Documentation updated for Raylib-first workflow

**Exit criteria**

- `pixi run build` produces working examples without OSG

## Phase 6 — dartpy Parity

**Deliverables**

- Python bindings for `dart::viz` viewer API (via nanobind)
- Python examples demonstrating visualization

**Exit criteria**

- `dartpy` users have equivalent visualization without OSG

## Phase 7 — Deprecation and Removal

**Deliverables**

- OSG dependency made opt-in (not default)
- Deprecation period with clear messaging
- OSG removal in a major release

**Exit criteria**

- Default builds require zero OSG dependency
