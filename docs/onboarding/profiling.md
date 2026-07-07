# Profiling And Benchmarks

Use this page for durable DART 6.20 profiling procedures. Temporary
packet-by-packet findings belong under `docs/dev_tasks/` until the related work
lands; promote only repeatable commands, gates, and current owner surfaces here.

## Dashboard Surface

The release-branch dashboard runner builds and executes the bounded CPU
benchmark surfaces that are safe to publish from GitHub Actions:

```bash
pixi run bm-dashboard-surfaces
pixi run bm-dashboard-merge
pixi run bm-dashboard-preview
```

For a local deformable-body dashboard slice only:

```bash
pixi run bm-dashboard-surfaces -- --surface soft-body \
  --benchmark-min-time 1s \
  --benchmark-repetitions 5
pixi run bm-dashboard-preview
```

The preview writes `build/performance-dashboard/index.html`.

## Soft-Body Benchmarks

`BM_INTEGRATION_soft_body` measures steady-state soft-body world stepping. It
loads each scene, applies the selected collision detector, warms one step
outside the timed loop, and reports scene, detector, thread count, soft-body
count, point-mass count, and simulated seconds per second.

Run the scalar default-detector matrix:

```bash
pixi run bm-soft-body -- \
  --benchmark_filter=BM_SoftBodyStep/.* \
  --benchmark_min_time=1s \
  --benchmark_repetitions=5
```

Run the same benchmark with a specific detector:

```bash
COLLISION_DETECTOR=dart pixi run bm-soft-body -- \
  --benchmark_filter=BM_SoftBodyStep/.* \
  --benchmark_min_time=1s \
  --benchmark_repetitions=5

COLLISION_DETECTOR=fcl pixi run bm-soft-body -- \
  --benchmark_filter=BM_SoftBodyStep/.* \
  --benchmark_min_time=1s \
  --benchmark_repetitions=5
```

The benchmark rows cover `adaptive_deformable`, `soft_cubes`, `soft_bodies`,
and `soft_open_chain` at one and sixteen simulation threads. Use
`COLLISION_DETECTOR=dart` for the native DART detector and
`COLLISION_DETECTOR=fcl` for FCL comparisons. Other registered detectors may
be useful diagnostics, but they are not the apples-to-apples soft-body
performance baseline unless the row proves equivalent soft-shape coverage.

## Soft-Body Headless Profiles

`soft_body_headless` is the repeatable soft-body checksum and text-profiler
driver. It accepts a scene, total steps, and checkpoint interval:

```bash
THREADS=1 COLLISION_DETECTOR=dart \
  pixi run bm-soft-body-headless soft_bodies 200 100

THREADS=16 COLLISION_DETECTOR=fcl \
  pixi run bm-soft-body-headless soft_bodies 200 100
```

Named scenes are `drop_box`, `drop_low_stiffness`, `double_pendulum`,
`adaptive_deformable`, `soft_cubes`, `soft_bodies`, and `soft_open_chain`.
Unknown scene names are treated as custom URIs. The output includes the active
thread count, collision detector, timestep, deterministic checksum rows, elapsed
time, steps per second, and the built-in text profiler dump when profiling is
enabled in the build.

Use headless profiles to validate that a timing improvement preserves the
expected state checksums and that the measured profiler scope is the intended
one.

## Revision Comparisons

Use the soft-body comparison script for PR evidence that must compare the
current commit against both its parent and the release-branch base on the same
host:

```bash
python3 scripts/compare_soft_body_performance.py \
  --current HEAD \
  --parent HEAD^ \
  --base origin/release-6.20 \
  --detectors dart,fcl \
  --threads 1,16 \
  --benchmark-min-time 1s \
  --benchmark-repetitions 5 \
  --benchmark-cycles 2 \
  --benchmark-run-order detector \
  --wait-for-local-dart-builds \
  --idle-max-load-1m 4 \
  --output-dir build/soft-body-comparison
```

The script writes `summary.md`, `summary.json`, raw benchmark JSON, and captured
logs. `summary.md` contains the comparison tables, an ASCII CPU-change graph
for one-shot review, current detector winners, and the evaluator verdict. The
balanced detector-first run order alternates revisions across cycles so host
drift is less likely to look like a detector regression.

Gate strict CPU regressions with:

```bash
python3 scripts/check_soft_body_performance_regressions.py \
  build/soft-body-comparison/summary.json
```

## Remaining Deformable Gates

Do not treat the native detector as the default deformable collision backend
until same-host evidence shows representative soft scenes are correct and at
least as fast as FCL in apples-to-apples rows. The remaining DART 6.20
deformable-body gates are:

- re-enable or replace the disabled soft-body equations-of-motion comparison
  after matrix and vector aggregation paths are complete; the current
  point-mass gravity and combined-vector sub-gate is not full equation parity;
- broaden the current one-thread versus multi-thread final-state check with
  energy, contact-force, CoP, historical-golden, or other invariant checks that
  catch divergent soft-body state;
- complete paper-parity scenes or approved representative substitutes for the
  Kim/Pollard and Jain/Liu soft-body references;
- extend native soft collision beyond the current primitive and retained
  soft-face lanes to fuller triangle/contact-neighborhood coverage;
- continue measured point-mass data-layout work toward contiguous,
  allocation-free, SIMD-eligible phase data before adding `dart/simd/` kernels;
- require one-thread and multi-thread CPU rows for each detector comparison,
  with checksum or equivalence evidence beside timing rows.
