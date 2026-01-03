# World Split: Classic and Next Worlds (Design Note)

## Status

- In progress: moved `dart8` to `dart/simulation/next` and tests to
  `tests/simulation/next`. Namespace is `dart::simulation::next`, classic
  `dart::simulation::World` reverted.

## Goals

- Keep `dart::simulation::World` stable for classic `Skeleton`/`Frame` workflows.
- Provide a next-gen world in `dart::simulation::next` without exposing EnTT in
  classic headers.
- Avoid multi-solver orchestration inside the classic world.

## Non-Goals

- No mixing classic and next objects in a single world.
- No public ECS entity creation through the classic world.
- No full rename of `DART8_*` macros/targets in this phase.

## Proposed Shape

- **Classic world**
  - `dart::simulation::World` stays as-is (Skeleton/Frame + ConstraintSolver).

- **Next world**
  - `dart::simulation::next::World` hosts ECS-backed object types.
  - Built behind `DART_BUILD_SIMULATION_NEXT` (with `DART_BUILD_DART8` alias).

## Namespace / Directory Choice

- `dart/simulation/next` with `namespace dart::simulation::next`.

## Python Binding Direction

- New module `dartpy.simulation_next` (preferred).
- Optional temporary alias `dartpy.dart8` during migration.

## Compatibility Notes

- Keep the `dart8` CMake target/macros as a compatibility layer for now.
