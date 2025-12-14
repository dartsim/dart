# Raylib GUI Backend — Milestones

This document defines a staged plan toward fully replacing the current OSG GUI.

## Phase 0 — Baseline and requirements

**Deliverables**

- A feature inventory of what DART’s maintained examples/tutorials and `dartpy` need from the GUI (camera, stepping, picking, manipulation, overlays, etc.).
- A written, reviewed “minimal public API” for the new viewer surface (see `docs/dev_tasks/raylib/01-architecture.md`).
- Explicit agreement that the new API is **not** backward compatible with `dart::gui` and will stay intentionally constrained.

**Exit criteria**

- Agreement on the minimum “replacement bar” for removing OSG from default builds.

## Phase 1 — Raylib “World Viewer” MVP

**Deliverables**

- A Raylib-backed viewer that can:
  - Display a `simulation::World` with basic camera controls.
  - Render common primitive shapes with correct transforms and colors.
  - Run a deterministic render loop with simulation stepping (pause/step).

**Exit criteria**

- At least one maintained example or tutorial can be demonstrated end-to-end on Raylib using the MVP viewer.

## Phase 2 — Interaction parity (core)

**Deliverables**

- Input/event model that is not tied to OSG types (keyboard/mouse).
- Picking/selection support (at minimum: selecting frames/bodies).

**Exit criteria**

- Basic interactive workflows used by tutorials are possible without OSG.

## Phase 3 — Manipulation tools parity

**Deliverables**

- Interactive manipulation equivalent to the current “drag and drop” workflows (or a documented replacement UX).
- Any required IK/constraint logic moved to backend-neutral layers when possible.

**Exit criteria**

- The most commonly used manipulation features are available on Raylib.

## Phase 4 — UI overlay parity

**Deliverables**

- Integrate Dear ImGui into the Raylib viewer implementation (kept internal; no ImGui types in the public API).
- A small, intentionally constrained DART-owned UI surface (panels/tools) that is sufficient for DART-maintained workflows.

**Exit criteria**

- The essential built-in panels/tools (or replacements) run on the Raylib viewer using only the new constrained API.

## Phase 5 — Migration of shipped content

**Deliverables**

- Maintained examples/tutorials no longer require OSG by default (either migrated to a backend-neutral API or duplicated with a Raylib path).
- Documentation updates so the Raylib path is a first-class workflow.

**Exit criteria**

- Building and running maintained examples/tutorials does not require OSG.

## Phase 6 — `dartpy` parity

**Deliverables**

- Python bindings for the new viewer surface area (or a supported alternative path).

**Exit criteria**

- `dartpy` users have an equivalent supported visualization workflow without OSG.

## Phase 7 — Deprecation, default switch, removal

**Deliverables**

- Deprecation period for the legacy OSG GUI API/components (clear messaging and migration notes).
- Default backend switch (Raylib becomes default where a GUI is enabled).
- Optional build path for legacy OSG during transition (time-bounded).

**Exit criteria**

- OSG is no longer required for default builds; removal proceeds in a major release once downstream migration is complete.
