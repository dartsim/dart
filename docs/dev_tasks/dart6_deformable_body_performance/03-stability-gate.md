# WP-DB.02 stability gate

Captured on 2026-07-07 from `release-6.20` plus local WP-DB.01/WP-DB.02
edits on branch `wp-db-soft-skel-allocation-gates`.

## Command

```bash
cmake --build build/default/cpp/Release --target test_SoftDynamics --parallel
ctest --test-dir build/default/cpp/Release -R 'test_SoftDynamics$' --output-on-failure
```

Result: `test_SoftDynamics` passed locally in 0.09 seconds after the final
state comparison gate was added.

## Coverage

`tests/integration/test_SoftDynamics.cpp` now loads and steps these SKEL soft
scenes for both `World::setNumSimulationThreads(1)` and
`World::setNumSimulationThreads(4)`:

| Scene | Min soft bodies | Min point masses | Steps per thread setting |
| --- | ---: | ---: | ---: |
| `test_drop_box.skel` | 1 | 26 | 30 |
| `test_drop_low_stiffness.skel` | 1 | 26 | 30 |
| `test_double_pendulum.skel` | 2 | 52 | 30 |
| `test_adaptive_deformable.skel` | 1 | 12 | 30 |
| `soft_cubes.skel` | 2 | 52 | 30 |
| `softBodies.skel` | 5 | 290 | 30 |
| `soft_open_chain.skel` | 5 | 120 | 30 |

At initialization, step 10, step 20, and step 30, the gate checks:

- world time is finite and bounded,
- each scene still contains at least the expected soft-body and point-mass
  counts,
- skeleton positions, velocities, accelerations, and generalized forces are
  finite,
- soft-body stiffness and damping parameters are finite and nonnegative,
- point-mass position, velocity, acceleration, force, local/world position, and
  body/world velocity and acceleration are finite,
- point-mass world position and velocity norms stay under broad blow-up guards.

After each scene finishes, the gate compares the ordered final state from the
`threads=1` run against the `threads=4` run with a relative tolerance of
`1e-12`. The compared state includes skeleton positions and velocities plus
each point mass's local position, local velocity, and world position. This
turns the threaded-world coverage into a deterministic final-state regression
check instead of only proving that both runs stayed finite.

## Fixture cleanup

`data/skel/test/test_double_pendulum.skel` now has both intended bodies encoded
as `<soft_shape>` with explicit `<frags>3 3 3</frags>`. This prevents
`SkelParser` from silently treating one link as rigid or throwing when loading
the converted soft box.

## Remaining gaps

- The existing `compareEquationsOfMotion` body is still disabled. Completing
  soft-body matrix/vector aggregation and re-enabling equation checks belongs
  to WP-DB.04. The first active point-mass mass-matrix, gravity, and
  combined-vector aggregation sub-gate is recorded separately in
  `07-equation-correctness.md`.
- The gate does not yet check energy drift, contact force variance, or CoP
  smoothness. It also does not pin historical golden states across revisions.
- Multi-core coverage currently proves finite state under a threaded world
  setting; it does not prove speedup. WP-DB.07 owns scaling.
- SIMD is not exercised by this gate. WP-DB.06 owns `dart/simd/` profiling and
  SIMD-on/off benchmark evidence.
