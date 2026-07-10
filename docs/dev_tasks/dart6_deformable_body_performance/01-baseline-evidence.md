# WP-DB.01 baseline evidence

Captured on 2026-07-05 from `release-6.20` at `d0d8fa1a495` plus local
WP-DB.01 edits. The table below uses the second smoke run after cleaning legacy
soft-body scene markup.

## Command

```bash
pixi run bm-soft-body --benchmark_filter=SoftBodyStep --benchmark_min_time=0.01s
```

The initial run performed a fresh Release configure and built
`BM_INTEGRATION_soft_body`. CMake reported `DART_ENABLE_SIMD=OFF`, so these
rows are scalar CPU baseline rows, not SIMD-enabled rows.

Google Benchmark warned that CPU scaling was enabled and the load average was
high. Treat the rows below as a smoke baseline proving target coverage, not as
final performance numbers for regression thresholds.

## Smoke rows

| Scene | Threads | Soft bodies | Point masses | ms / 200 steps | sim_s/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 1 | 58 | 29.1 | 6.86 |
| `adaptive_deformable` | 16 | 1 | 58 | 34.3 | 5.97 |
| `soft_cubes` | 1 | 2 | 52 | 25.3 | 7.90 |
| `soft_cubes` | 16 | 2 | 52 | 26.7 | 7.60 |
| `soft_bodies` | 1 | 5 | 290 | 181 | 1.10 |
| `soft_bodies` | 16 | 5 | 290 | 175 | 1.14 |
| `soft_open_chain` | 1 | 5 | 130 | 40.5 | 4.94 |
| `soft_open_chain` | 16 | 5 | 130 | 41.7 | 4.86 |

## Findings

- The new target builds, links, loads the existing soft-body `.skel` scenes,
  and emits single-core and multi-core rows.
- Current multi-core behavior is flat or negative for these soft-body scenes.
  That makes WP-DB.07 a real optimization lane, not a proven capability.
- The most representative existing scene, `soft_bodies`, is only about
  real-time in this noisy smoke run and has no paper-scale character DOFs.
- Several tracked soft-body SKEL/SDF assets used `<frags>2 2 2</frags>`,
  which `SoftBodyNodeHelper::makeBoxProperties` clamps to 3 with warnings.
  WP-DB.01 makes the files explicit as `<frags>3 3 3</frags>` so benchmark
  and stability-gate output stays clean without changing the effective loaded
  topology.
- `data/skel/test/test_double_pendulum.skel` used a legacy `<soft>` tag for one
  body while `SkelParser` only creates soft bodies from `<soft_shape>`. WP-DB.01
  updates the tag so future stability coverage sees both intended soft links.

## Next evidence

- Re-run with `--benchmark_min_time=1s --benchmark_repetitions=5` on an idle
  host with CPU governor metadata before setting thresholds.
- Add a SIMD-on A/B once the DART 6 `dart/simd/` contract for point-mass loops
  is chosen.
- Attach stability or checksum evidence to performance rows so timing claims
  cannot hide exploding or divergent soft-body states. The first
  `test_SoftDynamics` one-thread versus four-thread final-state gate is
  recorded in `03-stability-gate.md`.
