# World Split: Classic and Experimental Worlds (Design Note)

## Status

- In progress: moved the experimental stack to `dart/simulation/experimental` and tests to
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
- No compatibility aliases for removed legacy macros/targets.

## Proposed Shape

- **Classic world**
  - `dart::simulation::World` stays as-is (Skeleton/Frame + ConstraintSolver).

- **Experimental world**
  - `dart::simulation::experimental::World` hosts ECS-backed object types.
  - Built behind `DART_BUILD_SIMULATION_EXPERIMENTAL`.

## Namespace / Directory Choice

- `dart/simulation/experimental` with `namespace dart::simulation::experimental`.

## Python Binding Direction

- New module `dartpy.simulation_experimental` (preferred).
- No legacy `dartpy` compatibility alias.

## Compatibility Notes

- Use `DART_EXPERIMENTAL_*` macros and `dart_experimental_*` CMake helpers.
