# Summary

- Add DART 6 World-owned simulation memory management with additive
  `WorldConfig` allocator options and optional `World::enterSimulationMode()`.
- Route DART-owned profiler, solver, Dantzig, and World step scratch away from
  steady-state heap growth on the native rigid path.
- Add allocation-counting support and first-post-bake regression gates for
  native DART collision, plus base-allocator compatibility gates for Bullet and
  ODE.

# Evidence

## Allocation

Native DART collision, `boxes_per_side=3`, 100 measured steady-state steps:

| Surface | Baseline / step | Final / step |
| --- | ---: | ---: |
| `operator new` count | 5835.020 | 0.000 |
| raw malloc-family count | 5862.020 | 0.000 |
| World base allocator count | 0.000 | 0.000 |

Strict native first-post-bake gates pass for both explicit
`enterSimulationMode()` and implicit first-step preparation followed by the
second step: zero operator-new, zero raw malloc, zero World base allocator
growth.

Bullet/ODE compatibility gates are intentionally scoped to the World base
allocator because global/raw counters include collision-backend internals.

## Determinism

`boxes_headless 8 2000 500` final checkpoints diff exactly against the recorded
baseline checkpoints.

## Timing

`boxes_headless 8 2000 500` improved from 9325.991 ms to 8147.949 ms
(-12.6%) on the recorded host.

The full Google Benchmark rerun is mixed and should not be described as a broad
speedup without qualification. The final `BM_INTEGRATION_contact_container`
capture is slower in short-lived world cases, and the `BM_RunBoxes` aggregate
is affected by google-benchmark automatic iterations advancing one stateful
world beyond the first 2000-step window. See
`docs/dev_tasks/dart6_simulation_allocation_hardening/03-results.md` for the
full table and caveat.

# Verification

- `WorldSimulationModeMemoryManager.*:StepAllocation.*`: passed, 16 tests.
- `ctest -R '(Profile|StepAllocation)' --output-on-failure`: passed in the
  WP-D6M.6 gate run.
- `boxes_headless 8 2000 500` checkpoint diff: empty.
- `pixi run lint`: passed in the WP-D6M.6 gate run.
- `pixi run test-all`: passed, including 129 C++ tests and 78 Python tests.
- `pixi run -e gazebo test-gz`: pending final WP-D6M.7 gate.

# Changelog Decision

Entry required under DART 6.20.0 / Simulation because this adds public
simulation preparation/configuration API and changes the documented allocator
behavior of DART-owned simulation steps. Draft entry has no PR link yet.
