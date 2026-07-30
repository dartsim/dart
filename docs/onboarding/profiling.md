# Profiling DART 6.20

Use profiling to find the hot path before changing performance-sensitive code,
then use the benchmark and determinism gates to prove the change. Profiling
output is explanatory evidence; it does not replace a before/after benchmark
matrix with matching guard hashes.

Temporary packet-by-packet findings belong under `docs/dev_tasks/` until the
related work lands. Promote only repeatable commands, gates, and current owner
surfaces here.

## Built-in Text Profiler

DART 6.20 has the `dart/common/Profile.hpp` front end. The default Pixi
configure task builds it in:

```bash
pixi run config
```

That task passes `-DDART_BUILD_PROFILE=ON`; the built-in text backend is ON by
default through `DART_PROFILE_BUILTIN=ON`. When `DART_BUILD_PROFILE=OFF`, the
`DART_PROFILE_*` macros compile to no-ops.

For contact-workload profiling, build the benchmark target and run it with
`--profile`:

```bash
pixi run cmake --build build/default/cpp/Release \
  --target contact_benchmark --parallel 8

CB=./build/default/cpp/Release/bin/contact_benchmark
pixi run $CB --generate-container 120 --steps 200 --checkpoint 0 \
  --collision dart --disable-deactivation --world-threads 1 \
  --max-contacts 20000 --max-contacts-per-pair 4 --quiet --profile
```

`--profile` resets the profiler before the measured run and prints the text
summary at exit. Use the same scene, solver, detector, contact caps, thread
count, and commit scope as the benchmark row you are investigating.

## Adding Scopes

Use named scopes where the summary would otherwise be ambiguous:

```cpp
#include <dart/common/Profile.hpp>

void stepSomething()
{
  DART_PROFILE_SCOPED_N("stepSomething");
  // Work to measure.
}
```

Use `DART_PROFILE_FRAME` once per simulation frame when frame counts matter.
Use `DART_PROFILE_COUNTER_N` for integer census data that complements timing,
such as island counts, row counts, or queue depths:

```cpp
DART_PROFILE_COUNTER_N("solver island rows", totalRows);
```

The text summary reports counter samples with sum, mean, min, max, and last
values per thread. Counter labels should be fixed string literals so Tracy builds
do not depend on temporary label storage.

For instrumentation inside allocation-sensitive runtime paths, use the
conditional macros and enable recording only from the profiling tool or command:

```cpp
const bool profileRecording
    = dart::common::profile::isProfileRecordingEnabled();
DART_PROFILE_SCOPED_IF_N(profileRecording, "solver stage");
DART_PROFILE_COUNTER_IF_N(profileRecording, "solver island rows", totalRows);
```

`contact_benchmark --profile` enables this runtime recording flag around the
measured run. Normal Release builds may still compile the profiler in, but
conditional scopes and counters should stay off unless a tool explicitly calls
`setProfileRecordingEnabled(true)`.

For tools that need to control the text backend directly, use:

```cpp
const bool previousRecording
    = dart::common::profile::setProfileRecordingEnabled(true);
dart::common::profile::resetProfile();
dart::common::profile::markProfileFrame();
dart::common::profile::printProfileSummary(std::cout);
const auto text = dart::common::profile::getProfileSummaryText();
dart::common::profile::setProfileRecordingEnabled(previousRecording);
```

Keep scopes coarse enough to answer a packet question. If a scope is only useful
during local diagnosis and would add noise to normal summaries, remove it before
the PR.

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
`COLLISION_DETECTOR=fcl` for FCL comparisons. Other registered detectors may be
useful diagnostics, but they are not the apples-to-apples soft-body performance
baseline unless the row proves equivalent soft-shape coverage.

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
logs. `summary.md` contains the comparison tables, an ASCII CPU-change graph for
one-shot review, current detector winners, and the evaluator verdict. The
balanced detector-first run order alternates revisions across cycles so host
drift is less likely to look like a detector regression.

Gate strict CPU regressions with:

```bash
python3 scripts/check_soft_body_performance_regressions.py \
  build/soft-body-comparison/summary.json
```

## Tracy

The Tracy backend is opt-in. Configure a Tracy-enabled build from the profile
environment:

```bash
pixi run -e profile config-tracy
pixi run -e profile cmake --build build/profile/cpp/Release \
  --target contact_benchmark --parallel 8
```

`config-tracy` enables `DART_BUILD_PROFILE=ON`, keeps the text backend enabled,
sets `DART_PROFILE_TRACY=ON`, and uses the system Tracy package from the Pixi
profile environment. The build directory is `build/profile/cpp/Release`.

Start the viewer in a separate terminal:

```bash
pixi run -e profile tracy
```

For short-lived command-line runs, keep the process alive long enough for Tracy
to connect:

```bash
TRACY_NO_EXIT=1 pixi run -e profile ./build/profile/cpp/Release/bin/contact_benchmark \
  --generate-container 120 --steps 200 --checkpoint 0 --collision dart \
  --disable-deactivation --world-threads 1 --max-contacts 20000 \
  --max-contacts-per-pair 4 --quiet --profile
```

Use Tracy to locate and understand hot regions. Do not use a Tracy-connected run
as the acceptance timing for a performance PR; run the normal Release
before/after matrix separately.

## Reporting

Every performance PR should match the #3307-style comparison and report bar,
not just a timing snippet. Record:

- exact commit SHAs for the recorded baseline, parent/current base, and PR head;
- exact configure, build, benchmark, and profile commands;
- compiler, CPU model, governor/scaling state, Pixi environment, and optional
  detector availability;
- profile summary lines that explain the chosen optimization target;
- the full benchmark evidence table with contacts, pairs, resting counts,
  cap-hit status, and final-state hashes where applicable.

For `contact_benchmark` rows using ODE, keep `--max-contacts-per-pair 4`; larger
caps measure a different detector behavior on this branch.

## Remaining Deformable Gates

Do not treat the native detector as the default deformable collision backend
until same-host evidence shows representative soft scenes are correct and at
least as fast as FCL in apples-to-apples rows. The remaining DART 6.20
deformable-body gates are:

- re-enable or replace the disabled soft-body equations-of-motion comparison
  after matrix and vector aggregation paths are complete; the current point-mass
  mass-matrix, augmented-mass, gravity, and combined-vector sub-gate is not full
  equation parity;
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

## Troubleshooting

- Empty text summaries usually mean the binary was not rebuilt after enabling
  `DART_BUILD_PROFILE`, or the tool did not print the summary. For
  `contact_benchmark`, include `--profile`.
- If Tracy headers or packages are missing, use the profile environment:
  `pixi run -e profile config-tracy`.
- If a short run exits before the Tracy viewer captures it, set
  `TRACY_NO_EXIT=1`.
- Reconfigure when switching between default and Tracy builds. They use separate
  Pixi environment build directories, so stale binaries are easy to spot by
  path.
