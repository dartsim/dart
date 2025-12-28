# Issue 244: Sensor scaffolding progress

## Repro (origin/main)

- `git ls-tree -r --name-only origin/main | rg '^dart/sensor/'` returns nothing (no sensor module).
- `git grep -n "dart::sensor" origin/main -- dart` returns nothing; only MJCF user-sensor fields appear.

## Phase status

- [x] Phase 0: Intake + repro
- [x] Phase 1: Core sensor API
- [x] Phase 2: Tests + GUI example
- [x] Phase 3: Validation + delivery

## Validation + delivery

- `pixi run lint`
- `DART_PARALLEL_JOBS=16 CTEST_PARALLEL_LEVEL=16 pixi run test`
- `DART_PARALLEL_JOBS=16 CTEST_PARALLEL_LEVEL=16 pixi run test-all`
- `DART_PARALLEL_JOBS=16 CTEST_PARALLEL_LEVEL=16 pixi run -e gazebo test-gz`
- PR opened; CI green across Linux/macOS/Windows/gz-physics/publish.
