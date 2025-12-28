# Issue 244: Sensor scaffolding plan

## Phase 0: Intake + repro

- Confirm issue status on origin/main and capture minimal repro steps.
- Identify the missing API surface and target integration points.

## Phase 1: Core sensor API

- Add `dart/sensor` module with `Sensor` base and `SensorManager`.
- Define update context, enable/update-rate gating, and frame attachment.
- Integrate sensors with `simulation::World` (add/remove/update/reset).

## Phase 2: Tests

- Add an integration test in `tests/integration/simulation/test_Sensors.cpp` to cover:
  - name management for duplicate sensors
  - update invocation during `World::step` with expected context
  - enable/disable gating
- Add or update a GUI example to visualize sensor scaffolding.

## Phase 3: Validation + delivery

- Run `pixi run lint`, `pixi run test`, `pixi run test-all`, and
  `pixi run -e gazebo test-gz` with `DART_PARALLEL_JOBS=16` and
  `CTEST_PARALLEL_LEVEL=16`.
- Create branch, commit, push, open PR, and monitor CI.
