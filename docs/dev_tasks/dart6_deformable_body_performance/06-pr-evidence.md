# WP-DB PR evidence packet

Captured on 2026-07-05 from local branch
`js/dart6-deformable-performance`.

Updated on 2026-07-07 from local branch
`wp-db-soft-skel-allocation-gates` at commits `649926d28dc` and
`221fdf00145`, plus the follow-up native primitive-frame stability slice.

## Baseline setup

The soft-body benchmark harness was used for the baseline and PR rows. Because
`BM_INTEGRATION_soft_body` does not exist on `origin/release-6.20`, the
baseline was measured in a temporary detached worktree with the benchmark
harness applied:

```bash
git worktree add --detach /tmp/dart-release620-baseline.DgUMfX origin/release-6.20
git diff origin/release-6.20...HEAD -- \
  tests/benchmark/integration/CMakeLists.txt \
  tests/benchmark/integration/bm_soft_body.cpp \
  tests/benchmark/integration/soft_body_headless.cpp \
  | git -C /tmp/dart-release620-baseline.DgUMfX apply
```

Baseline commit for the benchmark matrix: `fd6919a9893`
(`origin/release-6.20` at capture time).

PR branch commit under benchmark test: `c33efb45e176`, including the benchmark
harness update that adds one warmup step outside the measured loop, supports
`COLLISION_DETECTOR=<factory-name>`, adds the native soft-stack and
`softBodies.skel` allocation gates described below, and reuses all-resting
snapshot storage to avoid steady-state soft-SKEL heap churn.

Current post-merge validation head: `420065b8f57a`, after merging the latest
`origin/release-6.20` into `js/dart6-deformable-performance`.

The build used the default Release Pixi environment with `DART_ENABLE_SIMD=OFF`
and `DART_BUILD_PROFILE=ON`. Google Benchmark reported CPU scaling enabled and
high load averages for all captures, so the rows below are same-host smoke
evidence, not final threshold-quality benchmark results. The benchmark now runs
one warmup `World::step()` while timing is paused so the measured loop reflects
steady-state soft-body stepping rather than one-time collision/simulation
preparation.

## 2026-07-07 correction benchmark

The latest WP-DB.06 slice restores the prior two-pass
`SoftBodyNode::updateArtInertia()` scalar timing after benchmark experiments
showed the simplified point-mass `Pi` formula and direct cached-position
variants could move FCL rows in the wrong direction. The retained change keeps
the span-backed phase view and adds only a conservative
`updateBiasForce()` connection-loop cleanup.

Command:

```bash
pixi run python scripts/compare_soft_body_performance.py \
  --current HEAD \
  --parent HEAD^1 \
  --base origin/release-6.20 \
  --detectors fcl,dart \
  --threads 1,16 \
  --benchmark-min-time 0.2s \
  --benchmark-repetitions 7 \
  --benchmark-cycles 2 \
  --benchmark-run-order detector \
  --correctness-scenes soft_cubes,soft_bodies \
  --correctness-steps 200 \
  --wait-for-local-dart-builds \
  --idle-max-load-1m 4 \
  --idle-cooldown 5 \
  --output-dir .benchmark_results/wp-db06-inertia-conn-649926-parent-43347c-base
```

Results:

- Artifact:
  `.benchmark_results/wp-db06-inertia-conn-649926-parent-43347c-base/summary.md`.
- Revisions: current `649926d28dc`, parent `43347c78514`,
  base `2d898081931`.
- Evaluator verdict: `PASS`.
- Detector equivalence: native `dart` is the reference detector; `fcl` is
  checksum-equivalent on the correctness scenes.
- Current detector ranking: native `dart` is the winner for every tracked
  current scene/thread row.

| Scene | Threads | Native CPU ms | FCL CPU ms |
| --- | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 2.326 | 15.927 |
| `adaptive_deformable` | 16 | 2.501 | 16.351 |
| `soft_bodies` | 1 | 13.706 | 66.629 |
| `soft_bodies` | 16 | 13.546 | 69.737 |
| `soft_cubes` | 1 | 3.738 | 17.931 |
| `soft_cubes` | 16 | 3.872 | 18.527 |
| `soft_open_chain` | 1 | 5.417 | 31.537 |
| `soft_open_chain` | 16 | 5.710 | 32.604 |

This comparison is strong enough to keep the correction and to show native is
the best current detector choice on these apples-to-apples soft-body rows. It
is still not final all-threshold evidence: the shared workstation had other
DART workloads during the run, and a few current-vs-parent/base CPU mean rows
remain noise-sensitive. Before a PR claims every CPU row beats both parent and
base, rerun the same comparison on an exclusive idle host with longer timing
windows and refresh this evidence packet.

## 2026-07-07 aggregation-temporary comparison

The follow-up `221fdf00145` slice removes avoidable soft-body aggregation
temporaries and no-op point dispatch:

- dynamic diagonal `K`/`D` matrices in `aggregateAugMassMatrix()`,
- the temporary gravity vector in `aggregateGravityForceVector()`,
- no-op point dispatch in `aggregateInvMassMatrix()`, and
- point-mass pointer/helper calls in mass summation and force clearing.

Command:

```bash
pixi run python scripts/compare_soft_body_performance.py \
  --current HEAD \
  --parent HEAD^1 \
  --base origin/release-6.20 \
  --detectors fcl,dart \
  --threads 1,16 \
  --benchmark-min-time 0.1s \
  --benchmark-repetitions 5 \
  --benchmark-cycles 2 \
  --benchmark-run-order detector \
  --correctness-scenes soft_cubes,soft_bodies \
  --correctness-steps 200 \
  --wait-for-local-dart-builds \
  --idle-max-load-1m 4 \
  --idle-cooldown 5 \
  --output-dir .benchmark_results/wp-db06-aggregation-temp-221fdf-parent-423f926-base
```

Results:

- Artifact:
  `.benchmark_results/wp-db06-aggregation-temp-221fdf-parent-423f926-base/summary.md`.
- Revisions: current `221fdf00145`, parent `423f926e030`,
  base `2d898081931`.
- Evaluator verdict: `PASS`.
- Detector equivalence: native `dart` is the reference detector; `fcl` is
  checksum-equivalent on the correctness scenes.
- Current detector ranking: native `dart` is the winner for every tracked
  current scene/thread row.

| Scene | Threads | Native CPU ms | FCL CPU ms |
| --- | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 2.039 | 15.738 |
| `adaptive_deformable` | 16 | 2.375 | 16.177 |
| `soft_bodies` | 1 | 13.538 | 66.086 |
| `soft_bodies` | 16 | 13.724 | 65.266 |
| `soft_cubes` | 1 | 3.680 | 17.785 |
| `soft_cubes` | 16 | 3.839 | 17.599 |
| `soft_open_chain` | 1 | 5.905 | 30.072 |
| `soft_open_chain` | 16 | 5.449 | 29.989 |

The broad comparison improved every current-vs-base FCL row and every native
row except `dart/soft_open_chain/1`, where the mean was +1.3% against base and
+7.8% against parent while the median was -0.9% against base. Targeted reruns
of that exact row using the already-built parent/current binaries measured
current faster than parent:

| Order | Parent CPU ms | Current CPU ms | Current change |
| --- | ---: | ---: | ---: |
| parent then current | 4.851 | 4.795 | -1.2% |
| current then parent, higher load | 7.662 | 7.420 | -3.2% |

Treat the broad positive as workstation noise, not a code regression. Still,
before claiming all CPU rows beat parent and base, rerun the full matrix on an
exclusive idle host with longer benchmark windows.

## 2026-07-07 native primitive-frame smoke

The follow-up native collision slice classifies soft-vs-plane,
soft-vs-sphere, and soft-vs-box point contacts in primitive-local coordinates,
then computes the world contact point only for vertices that actually collide.
The same slice extends `test_SoftDynamics` so the representative finite-state
and one-thread versus four-thread final-state gate runs under both the default
detector and `CollisionDetectorType::Dart`.

Focused validation:

```bash
pixi run cmake --build build/default/cpp/Release \
  --target test_DARTCollisionDetector test_SoftDynamics \
    INTEGRATION_StepAllocation soft_body_headless BM_INTEGRATION_soft_body \
  --parallel 8
pixi run ctest --test-dir build/default/cpp/Release \
  -R 'test_DARTCollisionDetector$|test_SoftDynamics$|INTEGRATION_StepAllocation$' \
  --output-on-failure
```

Result: all three focused CTest gates passed. FCL/native `drop_box` 200-step
checksums remained identical; the native row measured 1.431 ms elapsed versus
7.045 ms for FCL in this busy-host smoke. Native `soft_cubes`, `soft_bodies`,
and `soft_open_chain` 200-step checksum rows matched exactly between
`THREADS=1` and `THREADS=16`.

Quick current-only detector-ranking smoke:

```bash
for detector in dart fcl; do
  COLLISION_DETECTOR=$detector \
    ./build/default/cpp/Release/bin/BM_INTEGRATION_soft_body \
      --benchmark_filter='BM_SoftBodyStep/(0|1|2|3)/(1|16)/200$' \
      --benchmark_min_time=0.05s \
      --benchmark_repetitions=3 \
      --benchmark_report_aggregates_only=true
done
```

Google Benchmark reported CPU scaling enabled and load averages of
2.27 / 6.96 / 9.50, so treat these rows as quick detector-ranking evidence,
not threshold-quality parent/base evidence.

| Scene | Threads | Native CPU ms | FCL CPU ms |
| --- | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 2.01 | 16.4 |
| `adaptive_deformable` | 16 | 2.41 | 17.0 |
| `soft_cubes` | 1 | 3.69 | 17.8 |
| `soft_cubes` | 16 | 3.68 | 17.9 |
| `soft_bodies` | 1 | 13.8 | 68.1 |
| `soft_bodies` | 16 | 13.6 | 63.4 |
| `soft_open_chain` | 1 | 5.38 | 29.6 |
| `soft_open_chain` | 16 | 5.06 | 29.8 |

Native remained the fastest detector for every tracked current row in this
smoke. The formal current/parent/base matrix still needs a committed-revision
rerun on an idle host before this slice can support all-threshold claims.

## 2026-07-08 direct-native fallback and broadphase smoke

After merging `origin/release-6.20`, the direct
`COLLISION_DETECTOR=native` factory path initially skipped `SoftMeshShape` and
`EllipsoidShape` rows. The follow-up slice keeps those shapes in the native
broadphase and routes their pairs through cached DART-native fallback collision
objects. It also reuses a fallback scratch result without colliding-object
lookup-cache bookkeeping, uses the cached plane fallback for plane/soft pairs,
and stores brute-force native broadphase entries contiguously instead of
fetching AABBs through a hash table in the pair loop.

Focused validation:

```bash
pixi run cmake --build build/default/cpp/Release \
  --target UNIT_collision_native_brute_force \
    UNIT_collision_native_detector_adapter BM_INTEGRATION_soft_body \
    soft_body_headless --parallel 4
pixi run ctest --test-dir build/default/cpp/Release \
  -R 'UNIT_collision_native_brute_force$|UNIT_collision_native_detector_adapter$' \
  --output-on-failure
```

Result: both focused unit tests passed. Direct native `soft_cubes` and
`soft_bodies` 200-step headless checksums matched `COLLISION_DETECTOR=dart`,
and the representative direct-native soft-scene benchmark emitted no
unsupported-shape warnings.

Quick detector-ranking smoke:

```bash
for detector in native dart fcl; do
  COLLISION_DETECTOR=$detector \
    ./build/default/cpp/Release/bin/BM_INTEGRATION_soft_body \
      --benchmark_filter='BM_SoftBodyStep/(0|1|2|3)/(1|16)/200$' \
      --benchmark_min_time=0.2s \
      --benchmark_repetitions=7 \
      --benchmark_report_aggregates_only=true
done
```

Google Benchmark still reported CPU scaling enabled, and the FCL run had a
noisy `soft_open_chain/16` row, so these rows remain smoke evidence rather than
threshold-quality parent/base evidence.

| Scene | Threads | Native CPU ms | DART CPU ms | FCL CPU ms |
| --- | ---: | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 1.85 | 1.85 | 15.0 |
| `adaptive_deformable` | 16 | 2.21 | 2.26 | 15.5 |
| `soft_cubes` | 1 | 3.30 | 3.41 | 17.2 |
| `soft_cubes` | 16 | 3.48 | 3.52 | 18.2 |
| `soft_bodies` | 1 | 12.2 | 12.1 | 66.0 |
| `soft_bodies` | 16 | 12.5 | 12.4 | 63.6 |
| `soft_open_chain` | 1 | 4.84 | 4.73 | 28.9 |
| `soft_open_chain` | 16 | 5.06 | 5.08 | 30.6 |

The direct `native` and `dart` rows are close because direct native currently
bridges to the same soft contact kernels for soft/ellipsoid pairs; FCL remains
far slower on every tracked row. The formal matrix script now treats
`native` as the expected fastest checksum-equivalent detector while keeping
`dart` as the checksum reference, and its generated `summary.md` includes both
the CPU-change graphs and a detector-winner graph. The next committed-revision
comparison will fail if direct native is missing, checksum-ineligible, or not
the winner.

## Benchmark commands

Baseline:

```bash
pixi run bash -lc '
cd /tmp/dart-release620-baseline.DgUMfX
./build/default/cpp/Release/bin/BM_INTEGRATION_soft_body \
  --benchmark_filter=BM_SoftBodyStep \
  --benchmark_min_time=0.05s \
  --benchmark_repetitions=3 \
  --benchmark_format=json \
    --benchmark_out=/tmp/dart-soft-bench-baseline-warm.json \
  --benchmark_out_format=json
'
```

PR branch:

```bash
./build/default/cpp/Release/bin/BM_INTEGRATION_soft_body \
  --benchmark_filter=BM_SoftBodyStep \
  --benchmark_min_time=0.05s \
  --benchmark_repetitions=3 \
  --benchmark_format=json \
    --benchmark_out=/tmp/dart-soft-bench-pr-warm.json \
  --benchmark_out_format=json
```

PR branch, native detector:

```bash
COLLISION_DETECTOR=dart \
  ./build/default/cpp/Release/bin/BM_INTEGRATION_soft_body \
  --benchmark_filter=BM_SoftBodyStep \
  --benchmark_min_time=0.05s \
  --benchmark_repetitions=3 \
  --benchmark_format=json \
  --benchmark_out=/tmp/dart-soft-bench-pr-native-warm.json \
  --benchmark_out_format=json
```

## Default-backend benchmark rows

Mean CPU time and CPU-time-derived `sim_s/s` from the JSON outputs:

| Scene | Threads | Baseline CPU ms | PR CPU ms | CPU change | Baseline sim_s/s | PR sim_s/s | sim_s/s change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 91.033 | 82.015 | -9.9% | 2.197 | 2.443 | +11.2% |
| `adaptive_deformable` | 16 | 74.701 | 84.648 | +13.3% | 2.681 | 2.363 | -11.8% |
| `soft_cubes` | 1 | 59.635 | 52.855 | -11.4% | 3.357 | 3.797 | +13.1% |
| `soft_cubes` | 16 | 63.866 | 53.983 | -15.5% | 3.133 | 3.708 | +18.4% |
| `soft_bodies` | 1 | 417.115 | 235.957 | -43.4% | 0.480 | 0.848 | +76.9% |
| `soft_bodies` | 16 | 396.372 | 240.094 | -39.4% | 0.505 | 0.833 | +65.1% |
| `soft_open_chain` | 1 | 99.594 | 128.822 | +29.3% | 2.010 | 1.553 | -22.7% |
| `soft_open_chain` | 16 | 108.258 | 124.234 | +14.8% | 1.851 | 1.612 | -12.9% |

Interpretation:

- The default-backend rows improve the representative `soft_bodies` and
  `soft_cubes` scenes under the same warmup harness.
- The default-backend rows are mixed for `adaptive_deformable` and regress on
  `soft_open_chain` in CPU time. A direct headless `soft_open_chain` FCL profile
  on the same checkout showed lower branch wall-clock and lower collision and
  dynamics profile totals than the temporary baseline, so this remains noisy
  CPU-time benchmark evidence rather than a settled default-backend regression.

## Native-backend benchmark rows

These rows compare the release-branch FCL baseline with the PR branch using
`COLLISION_DETECTOR=dart`, which matches the native-first deformable-collision
direction of WP-DB.08:

| Scene | Threads | Baseline FCL CPU ms | PR native CPU ms | CPU change | Baseline sim_s/s | PR native sim_s/s | sim_s/s change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 91.033 | 9.351 | -89.7% | 2.197 | 21.389 | +873.4% |
| `adaptive_deformable` | 16 | 74.701 | 12.113 | -83.8% | 2.681 | 16.525 | +516.5% |
| `soft_cubes` | 1 | 59.635 | 18.694 | -68.7% | 3.357 | 10.701 | +218.7% |
| `soft_cubes` | 16 | 63.866 | 18.695 | -70.7% | 3.133 | 10.721 | +242.2% |
| `soft_bodies` | 1 | 417.115 | 96.822 | -76.8% | 0.480 | 2.076 | +332.9% |
| `soft_bodies` | 16 | 396.372 | 86.974 | -78.1% | 0.505 | 2.302 | +356.2% |
| `soft_open_chain` | 1 | 99.594 | 19.935 | -80.0% | 2.010 | 10.034 | +399.3% |
| `soft_open_chain` | 16 | 108.258 | 21.853 | -79.8% | 1.851 | 9.158 | +394.8% |

The native-backend rows show large wins across every tracked benchmark scene in
this smoke run. They are still not a complete native-default acceptance claim:
the broader native soft-soft lane remains a vertex-face approximation, and
full triangle/contact-neighborhood coverage is still tracked in
`05-native-collision-deformable-lane.md`.

## Native collision evidence

Soft-box FCL/native parity and timing smoke:

```bash
COLLISION_DETECTOR=fcl THREADS=1 \
  ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless \
  drop_box 200 200
COLLISION_DETECTOR=dart THREADS=1 \
  ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless \
  drop_box 200 200
```

Both detectors produced the same step-200 checksum:

```text
skelPosL1 0.33013838249780042
skelPosSq 0.038880346760999968
skelVelL1 1.9620000000000026
skelVelSq 3.8494440000000059
pointPosL1 5.2745885180896149e-17
pointPosSq 6.1836217223150731e-35
pointVelL1 2.1297127260119868e-16
pointVelSq 1.2599100820381264e-33
pointWorldPosL1 8.3885820947800553
pointWorldPosSq 1.3492978157859996
```

| Detector | Elapsed ms | Steps/s |
| --- | ---: | ---: |
| FCL | 50.319 | 3974.6 |
| Native | 8.879 | 22523.8 |

The native row is about 5.7x faster for this limited soft-box contact lane on
this host. Treat the exact ratio as load-sensitive.

Threaded native soft-scene smoke:

```bash
COLLISION_DETECTOR=dart THREADS=4 \
  ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless \
  soft_bodies 20 20
```

The step-20 checksum matched the previous native diagnostic row:

```text
skelPosL1 0.014467820943960557
skelPosSq 2.1220060049998933e-05
skelVelL1 0.98100000000000276
skelVelSq 0.1924722000000004
pointPosL1 6.0628137954490416e-17
pointPosSq 3.8824696766450367e-35
pointVelL1 1.5423940823156441e-15
pointVelSq 4.2365456422608738e-32
pointWorldPosL1 213.75594489383639
pointWorldPosSq 77.599935310371123
```

The profile contained `DART native finite-finite soft workers`, confirming that
the threaded native soft-soft path was exercised. This run is worker-activation
evidence only; the host load was too high for a scaling claim.

## Local validation gates

Commands run on 2026-07-05 for the implementation and post-merge validation:

```bash
pixi run lint
pixi run cmake --build build/default/cpp/Release \
  --target test_ConstraintSolver test_IslandDeactivation test_SoftDynamics \
    test_DARTCollisionDetector INTEGRATION_StepAllocation soft_body_headless \
  --parallel
pixi run ctest --test-dir build/default/cpp/Release \
  -R 'INTEGRATION_StepAllocation$|test_ConstraintSolver$|test_IslandDeactivation$|test_DARTCollisionDetector$|test_SoftDynamics$|UNIT_common_(FrameAllocator|FreeListAllocator|MemoryManager|Profile)$' \
  --output-on-failure
pixi run build
pixi run test
pixi run test-py
```

Results:

- `pixi run lint` passed before the implementation and evidence commits.
- The focused build, focused CTest set, `pixi run build`, `pixi run test`
  (136 CTest entries), and `pixi run test-py` (78 pytest entries) passed.
- The initial downstream Gazebo pass on the pre-merge implementation built and
  tested gz-physics successfully: 199/199 gz-physics tests passed, followed by
  4/4 gz-physics performance checks passing. A first fresh gz-sim run in that
  pass hit a downstream `INTEGRATION_entity_system` diff-drive pose
  monotonicity flake once; immediate focused reruns passed.

Additional allocation-gate slice committed after the benchmark/evidence
capture:

```bash
pixi run lint
pixi run cmake --build build/default/cpp/Release \
  --target INTEGRATION_StepAllocation --parallel
pixi run ctest --test-dir build/default/cpp/Release \
  -R 'INTEGRATION_StepAllocation$|UNIT_common_(FrameAllocator|FreeListAllocator|MemoryManager|Profile)$|test_DARTCollisionDetector$|test_ConstraintSolver$' \
  --output-on-failure
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation \
  --gtest_filter='StepAllocation.*Soft*'
```

Results:

- The focused allocation and native-collision CTest set passed: 7/7 tests.
- The soft-only allocation filter passed 6/6 tests.
- The two-soft-box native stack measured 100 steady-state steps after warmup
  with 27 contacts, including 18 soft-soft contacts, and zero `operator new`,
  zero raw `malloc`, and zero counted base-allocator growth.

Follow-up SKEL soft-dynamics allocation slice:

```bash
pixi run lint
pixi run cmake --build build/default/cpp/Release \
  --target INTEGRATION_StepAllocation test_World test_ConstraintSolver \
    test_DARTCollisionDetector UNIT_common_FrameAllocator \
    UNIT_common_FreeListAllocator UNIT_common_MemoryManager \
    UNIT_common_Profile --parallel
pixi run ctest --test-dir build/default/cpp/Release \
  -R 'INTEGRATION_StepAllocation$|test_World$|test_ConstraintSolver$|test_DARTCollisionDetector$|UNIT_common_(FrameAllocator|FreeListAllocator|MemoryManager|Profile)$' \
  --output-on-failure
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation \
  --gtest_filter='StepAllocation.*Soft*'
```

Results:

- The focused allocation, World, ConstraintSolver, memory-manager, and native
  collision CTest set passed: 8/8 tests.
- The soft-only allocation filter passed 8/8 tests.
- The `softBodies.skel` gate transferred the parsed soft skeletons into a
  counted native world and measured 100 no-contact steady-state steps after
  warmup with zero `operator new`, zero raw `malloc`, and zero counted
  base-allocator growth. This is soft-dynamics allocation evidence, not
  contact-solving evidence.

Post-merge validation after merging latest `origin/release-6.20` into the PR
branch:

```bash
pixi run cmake --build build/default/cpp/Release \
  --target INTEGRATION_StepAllocation test_DARTCollisionDetector \
    UNIT_collision_native_sphere_sphere test_ConstraintSolver test_World \
    UNIT_common_FrameAllocator UNIT_common_FreeListAllocator \
    UNIT_common_MemoryManager UNIT_common_Profile --parallel
pixi run ctest --test-dir build/default/cpp/Release \
  -R 'INTEGRATION_StepAllocation$|test_DARTCollisionDetector$|UNIT_collision_native_sphere_sphere$|test_ConstraintSolver$|test_World$|UNIT_common_(FrameAllocator|FreeListAllocator|MemoryManager|Profile)$' \
  --output-on-failure
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation \
  --gtest_filter='StepAllocation.*Soft*'
pixi run build
pixi run test
pixi run test-py
pixi run -e gazebo test-gz
```

Results:

- The focused post-merge CTest set passed: 9/9 tests.
- The soft-only allocation filter passed: 8/8 tests. The native soft-stack gate
  again measured 100 steady-state steps with 27 contacts, including 18
  soft-soft contacts, and zero global or counted allocator growth.
- `pixi run build` passed.
- `pixi run test` passed: 136/136 CTest entries, including the merged
  DART-collision SIMD broadphase tests.
- `pixi run test-py` passed: 78/78 pytest entries.
- `pixi run -e gazebo test-gz` passed cleanly. It built and tested gz-physics
  with 199/199 tests passing plus 4/4 gz-physics performance checks, linked the
  source-built DART physics plugin, then built gz-sim and passed its selected
  headless `INTEGRATION_entity_system` smoke test 1/1. Display-dependent
  gz-sim tests were skipped because the environment did not expose valid DRI.

## Changelog decision

This branch includes user-visible performance and allocation improvements, so
`CHANGELOG.md` carries a DART 6.20.0 Simulation entry for the World-owned
simulation memory-management and no-steady-state-allocation preparation path.
If maintainers require the entry to include the final PR number, add that link
in a follow-up commit after PR publication.

## GUI examples and visual debugging

After rebasing this branch onto PR #3304, the existing C++ `soft_bodies` GUI
example includes headless capture and a soft-body display widget. This branch
extends that widget with performance stats so visual debugging can inspect scene
state and measured step cost in the same screenshot.

Interactive examples:

```bash
pixi run ex soft_bodies
pixi run ex mixed_chain
```

Local visual debugging captures inspected:

```bash
pixi run ex soft_bodies -- --gui-scale 2 --headless \
  --shot /tmp/soft_bodies-stats-widget.png \
  --steps 120 --width 896 --height 768
./build/default/cpp/Release/bin/soft_bodies --gui-scale 2 --headless \
  --shot /tmp/soft_bodies-stats-widget-v2.png \
  --steps 120 --width 896 --height 768
./build/default/cpp/Release/bin/soft_bodies --gui-scale 1 --headless \
  --hide-widget --shot /tmp/soft_bodies-clean-scene.png \
  --steps 30 --width 640 --height 480
```

The inspected widget capture shows the framed soft-body scene plus frame/time,
render FPS, target RTF, measured physics step time, measured physics steps/s,
measured physics RTF, sample count, step-time range, soft-body/point-mass
counts, skeleton/body-node counts, and current contact count. The clean capture
keeps the same camera framing with `--hide-widget`.

No new standalone GUI example or local video artifact was added by this branch.
