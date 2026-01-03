# World Split: Classic and Experimental Worlds (Design Note)

## Status

- In progress: moved `dart8` to `dart/simulation/experimental` and tests to
  `tests/simulation/experimental`. Namespace is `dart::simulation::experimental`, classic
  `dart::simulation::World` reverted.

## Goals

- Keep `dart::simulation::World` stable for classic `Skeleton`/`Frame` workflows.
- Provide an experimental world in `dart::simulation::experimental` without exposing EnTT in
  classic headers.
- Avoid multi-solver orchestration inside the classic world.

## Non-Goals

- No mixing classic and experimental objects in a single world.
- No public ECS entity creation through the classic world.
- No full rename of `DART8_*` macros/targets in this phase.

## Proposed Shape

- **Classic world**
  - `dart::simulation::World` stays as-is (Skeleton/Frame + ConstraintSolver).

- **Experimental world**
  - `dart::simulation::experimental::World` hosts ECS-backed object types.
  - Built behind `DART_BUILD_SIMULATION_EXPERIMENTAL` (with `DART_BUILD_DART8` alias).

## Namespace / Directory Choice

- `dart/simulation/experimental` with `namespace dart::simulation::experimental`.

## Python Binding Direction

- New module `dartpy.simulation_experimental` (preferred).
- Optional temporary alias `dartpy.dart8` during migration.

## Compatibility Notes

- Keep the `dart8` CMake target/macros as a compatibility layer for now.
