# Multi Solver World Refactor Plan (02)

## Status

- Draft: align on public ECS object lifecycle shape.

## Goal

- Introduce a minimal public ECS object lifecycle API without exposing EnTT or
  solver pointers, while preserving classic Skeleton behavior.

## Scope

- Define World-facing create/destroy hooks for ECS-backed objects that only
  expose `EcsEntity` or `simulation::object::Object` handles.
- Keep `entt::registry` access internal via `detail::WorldEcsAccess`.
- Route ECS object lifecycle events through the configured object solver.
- Add object-level lifecycle tests (creation, destruction, routing).

## Non-Goals

- No ECS physics pipeline (constraints/collision) yet.
- No breaking changes to classic Skeleton APIs.
- No `dart8/` migration.

## Open Questions

- Ownership model for objects: does World store and return shared handles?
- Naming/indexing strategy for ECS objects in World.
- Minimal public surface: `EcsEntity` only vs. typed object wrappers.

## Test Plan

- `pixi run test`
- `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`
