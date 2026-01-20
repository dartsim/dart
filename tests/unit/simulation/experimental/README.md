# Simulation Experimental Unit Tests

This directory contains unit tests for `dart::simulation::experimental`.
Tests should be small, deterministic, and focused on a single class or behavior.

## Running Tests

- `pixi run test-simulation-experimental` for just these tests.
- `pixi run test` to run the broader C++ suite.

## Adding Tests

- Add `test_*.cpp` files under the appropriate subdirectory.
- If a new subdirectory is needed, register it in
  `tests/unit/simulation/experimental/CMakeLists.txt`.
- Follow the existing CMake helpers (`dart_experimental_add_unit_test_dir` and
  `dart_add_simulation_test`).

## Patterns

- Use `TEST(ClassName, Method_Condition_Expected)` naming.
- Prefer minimal `World`/`MultiBody` setups; avoid long simulations.
- Use `EXPECT_NEAR` with explicit tolerances for floating point values.
- Keep assertions on deterministic outputs (transforms, state vectors, counts).
