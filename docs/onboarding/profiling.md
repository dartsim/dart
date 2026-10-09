# Profiling DART 6.20

Use profiling to find the hot path before changing performance-sensitive code,
then use the benchmark and determinism gates to prove the change. Profiling
output is explanatory evidence; it does not replace a before/after benchmark
matrix with matching guard hashes.

Temporary packet-by-packet findings belong under `docs/dev_tasks/` until the
related work lands. Promote only repeatable commands, gates, and current owner
surfaces here.

## Performance Methodology

Use this method for performance investigations and claims. Agents load
`dart-perf`; contributors follow the same standard here.

### Reproduce the User's Scenario

Before optimizing, reproduce the reported world/model, collision detector,
solver and settings, timestep, thread counts, and simulated duration. Record
initial state, seeds, contact caps, and sleeping/deactivation settings. State
every deviation and its effect on the claim; a convenient microbenchmark does
not establish an improvement in the user's workload.

### Choose the Right Numbers

Use two complementary measurements:

- **Exact-count regression gates:** [`scripts/perf_regression.py`](../../scripts/perf_regression.py)
  compares DART base and head revisions using Callgrind instruction counts,
  allocation counts/requested bytes, and state/behavior guard hashes. Follow
  [Revision Comparisons](#revision-comparisons) for A/B commands, matching
  inputs/toolchain/environment fingerprints, qualification, and rationale
  policy. Its wall time and hosted dashboard timings are advisory; count
  reductions alone do not establish a user-facing speed-up.
- **Wall-time evidence:** measure normal Release builds on the same controlled
  machine. Fix CPU affinity to one hardware thread per physical core and keep
  its SMT siblings idle; keep the solver and world single-threaded unless
  threading is the measured variable. Record the CPU governor/scaling state
  and background load.
  Run serially, including builds and other benchmarks, and alternate baseline
  and candidate runs to expose drift. Run at least three independent repeats
  per revision; report the median and min-max range, retaining all samples.
  Exclude and document warm-up, then measure the same fixed simulated duration
  and timestep in both arms. Measure settling and resting windows separately;
  warm-up must not erase the transient the user reported. Keep profiler and
  diagnostic overhead out of acceptance timings.

### Guard Behavior

Every performance claim needs a correctness check on both revisions over the
same workload and measurement windows. Choose guards that catch the reported
failure: sinking/penetration census and depth, sleeping/resting counts, finite
state and state hashes, contact/pair counts, or contact-cap hits. Exact hashes
prove equality only for the recorded samples; use tolerances and physical
invariants when exact equality is inappropriate. Report behavior regressions
beside timing, even when speed improves. Changed physics or workload must be
classified explicitly, with its compatibility impact; never present omitted
contacts or incorrect sleeping as a performance win.

### Attribute Before Optimizing

Separate engine time (for example, `World::step`) from host/integration work
such as transport, callbacks, logging, and rendering. Report the timed boundary
and end-to-end time separately. Profile settling transients and resting steady
state in distinct windows, using Callgrind or `perf` with symbols and resolved
stacks. Rank candidate changes by measured share of the relevant window, then
remeasure the full scenario after each change. Pair cache, SIMD, or threading
claims with hardware-counter evidence as described in
[the performance CI owner](ci-cd.md#performance-records-and-guards).

### Report the Effect

Follow [PR Descriptions](contributing.md#pr-descriptions): lead with what users
get, the speed-up versus a named DART baseline revision, real-time factor
(simulated seconds / measured wall seconds), and behavior results. State
whether the speed-up uses engine or end-to-end time. Show time-series plots for
transients and comparison plots with repeat ranges; keep raw samples/tables
available behind the plots. Record CPU/hardware, affinity and SMT policy,
governor, threads, compiler/dependency versions, build flags, exact baseline
and candidate revisions, reproduction commands, windows, and caveats. Bound
the claim to measured workloads and explain unavailable or noisy evidence.

Public comparisons use DART revisions only (old versus new). Cross-engine
tooling, artifacts, and results stay in private workspaces, outside the public
DART repository and its GitHub content. Model-format compatibility support
remains in scope. Use the existing
[visual-evidence selection and publication flow](../ai/verification.md#visual-verification-headless-capture)
for plots and claim-tied simulation highlights; publish images through its
prerelease backend after upload approval, and keep transient media out of Git.
For visible physics changes, follow the
[simulation verification route](../ai/verification.md#simulation-verification-route).

### Preserve Downstream Compatibility

Keep downstream simulator behavior acceptable against the last released DART
version, naming the release tag and recording remaining differences. Use the
existing [Gazebo compatibility lanes](../../tools/gazebo/README.md#unpatched-compatibility-lanes)
and [failure comparisons](../../tools/gazebo/README.md#comparing-failures),
alongside `pixi run -e gazebo test-gz` when affected. The stored lane baselines
are DART 6.19.4; if that is not the last release, compare that release explicitly.
The patched forward lane alone does not prove released-simulator compatibility.
Refactors are acceptable when these behavior and compatibility checks hold;
explain inherited failures and report new regressions rather than trading
physics correctness for speed.

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

The DART 6 dashboard runner builds and executes the bounded CPU
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
`COLLISION_DETECTOR=dart` for the built-in DART detector and
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

Use `pixi run perf-compare --base origin/main --head HEAD` for deterministic
instruction and allocation deltas and behavior-guard checks, measured with the
system Valgrind; wall time is advisory only.
The default `build/perf-compare` directory is reset only when it contains the
`.perf-compare-owned` marker written by the harness; custom output directories
must be empty.
Each run repeats every row under seven heap-layout perturbations, and a row
gates only when its guards and allocation counts stay identical under all of
them in that run; `--no-perturb` skips the checks and leaves every row
diagnostic. The report names each row's qualification and thread count.
All tiers stage each arm at `/tmp/dart-perf/arm`, including binaries, libraries,
revision inputs, shims, dependency paths and measurement outputs. Measurements
run from `/tmp/dart-perf` with a fixed minimal environment: system `PATH`,
`LC_ALL=C`, the existing FMA mask in `GLIBC_TUNABLES`, and staged library paths;
the harness preloads a small shim that disables OSG's implicit platform plugin
discovery before its static initializers run. These headless workloads need no
OSG plugins; otherwise OSG allocates a conda-embedded environment path even
when `OSG_LIBRARY_PATH` is set. The harness adds its measurement and
perturbation controls and checks the shim's lookup ABI against the active Pixi
OSG library. A host lock serializes staging, measurement and copying artifacts
to the requested output directory. Subprocesses inherit the lock, so an orphan
must finish before a later run can replace the slot. The staging root must be
private, owned by the current user, and on its parent's filesystem; mount points
are refused.
Set `DART_PERF_STAGING_ROOT` to an absolute directory on an executable
filesystem when another account owns the default root or `/tmp` is mounted
`noexec`. Its value is recorded in the environment fingerprint and backfill
run identity; comparisons and resumes across staging roots are refused.
Records expose the generic default root as `default` and a custom root as
`sha256:<digest>`; the actual value remains in the fingerprint input. The
publisher also normalizes older records containing the default `/tmp/dart-perf`
or a custom root in `run.env` or a row's `head_env`, without allowing paths
elsewhere in the record.
Installs also record their build staging root and must be rebuilt when it
changes, because DART resource paths are compiled into the libraries.
Read-only installs are copied into writable staging directories without
changing the originals.
Harness builds also use fixed source, build, dependency and install aliases,
because resource retrievers embed source and install data paths. Caller-owned
build trees remain reusable. Harness builds disable RPATHs and rely on the
measurement environment's `LD_LIBRARY_PATH`; their installs are intended to
run only through the harness. Normalization is part of `harness_sha`, so earlier
records have a different environment fingerprint and require new measurements.
Comparisons require matching environment fingerprints. Exit status 1 means a
policy failure; status 2 means an infrastructure error. When measuring an
existing install with `scripts/perf_regression.py run`, supply `--commit` for
its installed revision. Without `--shim` or `--heappad`, `run` uses
`<prefix>/../shims/allocshim.so` and `<prefix>/../shims/heappad.so` when present,
matching the `local` layout (`<output-dir>/a` and `<output-dir>/b` installs).
Otherwise it falls back to `build/perf/liballocshim.so` and
`build/perf/libheappad.so`; missing files report the corresponding option to
supply. `--no-perturb` does not require heappad. The install must also contain
`share/dart/perf-build.json`, written by `local` with the CMake compiler
ID/version, build preset, Pixi lock hash, commit, and installed libdart and
driver hashes, plus per-driver workload source hashes from the archived revision
and a digest of the installed sample data.
`contact_benchmark` hashes its CMake-globbed `.cpp`/`.hpp` files;
`BM_INTEGRATION_kinematics` hashes `bm_kinematics.cpp` and `PerfGuard.hpp`;
`BM_UNIT_dantzig_lcp` hashes `bm_dantzig_lcp.cpp`, `PerfGuard.hpp`, and
`tests/unit/lcpsolver/DantzigProblemCases.hpp`, which defines its generated cases.
DART library sources are excluded because they are the code being measured.
Each row's `input_sha` combines its scene/model input with its driver's workload
hash, also recorded as `workload_sha`. A workload or input change between arms is
a `behaviour-change`: performance deltas are withheld and a matching
`Rebaseline-Rationale` is required, just as for changed behavior guards.
`portable_step_bench` rows retain their existing input hashes because both arms
use the common driver source from this checkout, covered by the environment's
harness hash. `run` reads workload hashes from the install stamp, rather than
from the current checkout. `run` checks the stamp against the artifacts and
commit and installed sample data; missing or stale provenance, including missing
workload or sample-data hashes, is an infrastructure error. Saved records
without compiler provenance cannot pass comparison. Gated rows fail on increases
in either allocations per step or requested bytes per step unless acknowledged with a
matching `Perf-Regression-Rationale`. The report includes requested-byte deltas;
missing or invalid byte measurements are handled like allocation counts.
`dart://sample` resources resolve from the install's verified revision data,
including the `dyn` scenes. `--source-dir` supplies only explicitly named
file/model inputs, such as `pend` and `robot`; it cannot replace installed
`dart://sample` data. Rebuild earlier installs to obtain the sample-data stamp.
Gated rows with gate failures, allocation-count or requested-byte increases, or
Ir deltas at or above the +0.30% warning threshold count as regressed in the
summary. Otherwise, allocation-count or requested-byte decreases, or Ir
improvements of at least 1%, count as improved.

Use `pixi run perf-backfill --revs <file>` for a resumable local history run.
The file contains one commit, ref or `v6.x.y` tag per line, with blank lines
and `#` comments ignored. Put tags after commits. For the historical window,
generate the commit list with
`git rev-list --first-parent --reverse --since=2026-07-01T00:00:00Z 789d3662c59 -- dart`,
then append `v6.19.0` through `v6.19.5`. The command adds each commit's latest
first-parent measured-path base and each tag's previous `v6` tag, including
`v6.18.0` for `v6.19.0`: 85 listed commits, 13 additional commit bases and
seven tag revisions, for 105 unique measurements and 91 assembled records.
`--plan-only` prints the revision inventory without building or measuring.
The default rows are the quick tier plus S6, with perturbation always enabled.
Revisions without `examples/contact_benchmark`, such as the 6.18 and 6.19 tags,
run only the portable rows (`gzb,robot`). S6 remains in the saved arm records but
is omitted from published comparisons, along with advisory wall time and RSS.

Run `git fetch origin gh-pages` before starting. The harness checkout must be
clean for the script, `tools/perf` and `pixi.lock`, and publication later
requires its harness commit to be on `main`. The run uses one detached source
checkout, persistent Ninja build trees and an install prefix emptied before
each install under `build/perf-backfill`. A lock prevents concurrent runs.
Rerunning resumes completed revisions; do not delete the source checkout
between runs, because its unchanged file timestamps allow object reuse.
The run identity pins the toolchain, glibc libraries, harness commit and rows,
checks the initial toolchain against the newest hosted merge record, and
requires one fingerprint throughout. Drift stops the run. For a long campaign,
hold `libc6`, `libc6-dev`, `valgrind`, `gcc-13` and `g++-13` and stop
`apt-daily-upgrade.timer`; restore the previous package holds and timer state
afterward. A build failure receives a clean retry and then a saved broken arm;
persistent infrastructure errors stop the run. If the run has never completed
a measurement, assembly stops because no verified environment fingerprint is
available; the build-failure markers remain available for resume.
After review and publication,
remove the retained checkout with
`git worktree remove --force build/perf-backfill/src`.

A maintainer publishes the reviewed set once with
`python scripts/perf_regression.py publish --tier backfill --record build/perf-backfill/records --pages-dir <pages-dir>`,
using a clean dedicated `gh-pages` checkout and their own git credentials.
This command refuses GitHub Actions and validates the entire publication set
before writing publication files to the checkout; a refusal leaves it clean.
It never updates the chart or nightly guard table. Repeating an identical
publication adds no commit. Hosted records take precedence over local records
even when their environment fingerprints match; identical-or-refuse checks
apply only within the same `runner.environment`.

For repeatability checks, require identical deterministic counts and common
guards with matching toolchain, inputs and environment fingerprints. Earlier
pilots used arbitrary absolute paths and inherited environments: even local
runs of one revision could differ by over 0.26% Ir. Fixed staging removes those
allocation-layout inputs, including OSG's implicit conda plugin path. The
runtime shim hash and staging root are fingerprinted; matching fingerprints
still require the same runtime environment and toolchain. Wall time and RSS
remain advisory.

Inspect history with
`python scripts/perf_regression.py ledger --records build/perf-backfill/records <pages-records-dir> --since <base-sha> --until <head-sha>`.
`--json <file>` and `--markdown <file>` save the deterministic report.
`--intent <file>` reads a TSV with a SHA prefix of at least seven characters
or `#PR`, an intent (`perf`, `behaviour` or `unrelated`), and a one-line reason.
The ledger attributes failures to the head, separates inherited failures and
broken rows from rationale friction, and lists improvements as well as
regressions. Path groups summarize affected modules, collision detectors and
CMake inputs. Its headline counts unrelated merges needing a rationale against
the bar of at most one in ten;
unlabelled changes are counted separately so they can be reviewed.
Completeness requires an intent for every non-PASS entry and every entry with
listed rows or rationale lines, including PASS entries.

Use the soft-body comparison script for PR evidence that must compare the
current commit against both its parent and the `main` base on the same host:

```bash
python3 scripts/compare_soft_body_performance.py \
  --current HEAD \
  --parent HEAD^ \
  --base origin/main \
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

## Interactive Memory Inspection

Memory inspection is compiled out of `dart-demos` by default. After the normal
`pixi run config`, opt in by reconfiguring the same build with
`-DDART_BUILD_DEMOS_MEMORY_DIAGNOSTICS=ON`. The default-OFF build excludes the
diagnostics sources, link dependencies, host state, environment probe, Memory
tab, and frame-path calls; its post-link check rejects any diagnostics marker in
the demo executable.

When compiled in, the consolidated `dart-demos` Memory tab is a diagnostic lead
generator, not a heap or cache profiler. Its shared
`examples/demos/memory_diagnostics_model.*` owns opt-in cadence, process probes,
bounded history, reset, and compatible snapshot comparisons. The branch-local
`memory_diagnostics.*` collector/view owns DART 6 World traversal and raw-ImGui
presentation inside `DemoHost`.

Within a diagnostics-enabled build, keep the collection-disabled path before
every OS probe, World walk, address collection, history mutation, and
value-formatting pass. This runtime checkbox prevents collection work but does
not make the compiled-in UI integration zero-cost. Each metric needs an exact
scope, source, limitation, unit, and measured/estimate/proxy classification;
use an absent value for unavailable instrumentation rather than zero.

The exact allocator map and the classic-object atlas answer different
questions. Free-list/frame region maps can show allocator metadata, allocated,
free, reserved, and padding byte ranges because their backing allocations and
bookkeeping are observed directly. The classic graph is separately allocated,
so its typed atlas uses exact object addresses plus explicitly shallow
`sizeof` lower bounds, grouped only into host-page runs from a runtime page-size
query. Atlas gaps are unobserved address space, not fragmentation or free
memory. Raw addresses stay inside collection; the UI uses relative offsets.

Never describe virtual-address adjacency, gaps, or page buckets as physical
placement, cache misses, or a performance improvement without separate
hardware-counter/access-sampling and benchmark evidence. Hue identifies data
category, while hatch, border, opacity, and text also identify storage state so
the map is not color-only. Region maps retain up to 48 logical rows in an inner
scroll area, but render only the roughly eight visible rows into a bounded
per-region draw list so the 16-bit-index OpenGL2 backend stays below its vertex
limit. The current runtime page-size observation groups the object atlas into
virtual-page runs but does not place page or cache-line separators: the snapshot
intentionally lacks raw bases and does not yet capture the scrubbed base
remainders and host cache-line size required for those guides.

The DART 6 World `MemoryManager` rows and exact maps cover reservation arenas.
The World reserves and resets the frame arena, but current classic solver paths
do not allocate their scratch through it. Do not present those rows as legacy
solver scratch usage or color the arena as though it owns the classic object
graph; use a solver-specific profiler or instrumentation point for that
question.

## Reporting

Follow [Performance Methodology](#performance-methodology) for measurement,
behavior guards, attribution, and Effect-first evidence.

For `contact_benchmark` rows using ODE, keep `--max-contacts-per-pair 4`; larger
caps measure a different detector behavior on this branch.

## Remaining Deformable Gates

Do not treat the built-in `dart` detector as the default deformable collision backend
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
- extend DART soft collision beyond the current primitive and retained
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
