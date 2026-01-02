# World Split: Migration Plan

## Status

- Draft.

## Phase 0: Cleanup (Done in this branch)

- Revert multi-solver scaffolding and ECS hooks in classic `World`.
- Restore classic world behavior as the only Skeleton pipeline.

## Phase 1: Introduce ECS World Skeleton

- Create `dart/ecs` component with `dart::ecs::World` and minimal lifecycle.
- Port core ECS types from `dart8` (World, Frame, RigidBody, MultiBody, Tags).
- Add a dedicated CMake target and install headers for `dart/ecs`.

## Phase 2: Compatibility Layer

- Keep `dart8` as a thin alias layer:
  - `namespace dart8 = dart::ecs` for types and functions.
  - Deprecation notes in `dart8` headers.
- Maintain existing include paths via forwarding headers for one release.

## Phase 3: Python Binding Split

- Add `dartpy.ecs` module that binds `dart::ecs`.
- Keep classic bindings unchanged.
- Provide `dartpy.dart8` as a temporary alias to `dartpy.ecs`.

## Phase 4: Tests + Docs

- Move ECS tests to `tests_ecs/` or `tests/ecs/` (decide structure).
- Add new ECS world tests for object lifecycle + stepping.
- Update onboarding docs with a short note about the split worlds.

## Phase 5: Deprecation + Removal

- Announce deprecation of `dart8` and the alias modules.
- Remove `dart8` once downstream usage is migrated.

## Open Decisions

- Final namespace choice: `dart::ecs` vs `dart::simulation::ecs`.
- Test layout for ECS coverage.
- Timeline for `dart8` removal (major release boundary?).
