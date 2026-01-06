# World Split: Migration Plan

## Status

- In progress.

## Phase 0: Cleanup (Done in this branch)

- Revert multi-solver scaffolding and ECS hooks in classic `World`.
- Restore classic world behavior as the only Skeleton pipeline.

## Phase 1: Move Experimental World (Done in this branch)

- Move the experimental stack to `dart/simulation/experimental/`.
- Move tests to `tests/unit/simulation/experimental/` and `tests/benchmark/simulation/experimental/`.
- Rename namespace to `dart::simulation::experimental`.
- Update build/test plumbing (`DART_BUILD_SIMULATION_EXPERIMENTAL`, CTest labels,
  pixi tasks, docs).
- Rename CMake helpers/macros to `dart_experimental_*` / `DART_EXPERIMENTAL_*`.

## Phase 2: Python Binding Split (Planned)

- Add `dartpy.simulation_experimental` module for the new world.

## Phase 3: Deprecation (Planned)

- Deprecate classic `dart::simulation::World` in DART7 docs.
- Provide migration notes for moving to `dart::simulation::experimental::World`.

## Phase 4: Promotion (Planned)

- Remove classic `dart::simulation::World` in DART 8.
- Promote `dart::simulation::experimental::World` to `dart::simulation::World`.

## Open Decisions

- Python module name (`dartpy.simulation_experimental` vs `dartpy.next`).
- Deprecation timeline for classic world.
