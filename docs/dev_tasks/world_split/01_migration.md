# World Split: Migration Plan

## Status

- In progress.

## Phase 0: Cleanup (Done in this branch)

- Revert multi-solver scaffolding and ECS hooks in classic `World`.
- Restore classic world behavior as the only Skeleton pipeline.

## Phase 1: Move Next-Gen World (In progress)

- Move `dart8/` to `dart/simulation/next/`.
- Rename namespace to `dart::simulation::next`.
- Update build/test plumbing (`DART_BUILD_SIMULATION_NEXT`, CTest labels,
  pixi tasks, docs).

## Phase 2: Python Binding Split (Planned)

- Add `dartpy.simulation_next` module for the new world.
- Optionally keep `dartpy.dart8` as a temporary alias.

## Phase 3: Deprecation (Planned)

- Deprecate classic `dart::simulation::World` in DART7 docs.
- Provide migration notes for moving to `dart::simulation::next::World`.

## Phase 4: Promotion (Planned)

- Remove classic `dart::simulation::World` in DART8.
- Promote `dart::simulation::next::World` to `dart::simulation::World`.

## Open Decisions

- Python module name (`dartpy.simulation_next` vs `dartpy.next`).
- Timeline to rename `dart8` CMake targets/macros.
- Deprecation timeline for classic world.
