# Profiling DART 6.20

Use profiling to find the hot path before changing performance-sensitive code,
then use the benchmark and determinism gates to prove the change. Profiling
output is explanatory evidence; it does not replace a before/after benchmark
matrix with matching guard hashes.

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

Keep scopes coarse enough to answer a packet question. If a scope is only
useful during local diagnosis and would add noise to normal summaries, remove
it before the PR.

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

Use Tracy to locate and understand hot regions. Do not use a Tracy-connected
run as the acceptance timing for a performance PR; run the normal Release
before/after matrix separately.

## Interactive Memory Inspection

The consolidated `dart-demos` Memory tab is a diagnostic lead generator, not a
heap or cache profiler. Its shared `examples/demos/memory_diagnostics_model.*`
owns opt-in cadence, process probes, bounded history, reset, and compatible
snapshot comparisons. The branch-local `memory_diagnostics.*` collector/view
owns DART 6 World traversal and raw-ImGui presentation inside `DemoHost`.

Keep the disabled path before every OS probe, World walk, address collection,
history mutation, and value-formatting pass. Each metric needs an exact scope,
source, limitation, unit, and measured/estimate/proxy classification; use an
absent value for unavailable instrumentation rather than zero.

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
the map is not color-only. The current runtime page-size observation groups the
object atlas into virtual-page runs but does not place page or cache-line
separators: the snapshot intentionally lacks raw bases and does not yet capture
the scrubbed base remainders and host cache-line size required for those guides.

The DART 6 World `MemoryManager` rows and exact maps cover reservation arenas.
The World reserves and resets the frame arena, but current classic solver paths
do not allocate their scratch through it. Do not present those rows as legacy
solver scratch usage or color the arena as though it owns the classic object
graph; use a solver-specific profiler or instrumentation point for that
question.

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

## Troubleshooting

- Empty text summaries usually mean the binary was not rebuilt after enabling
  `DART_BUILD_PROFILE`, or the tool did not print the summary. For
  `contact_benchmark`, include `--profile`.
- If Tracy headers or packages are missing, use the profile environment:
  `pixi run -e profile config-tracy`.
- If a short run exits before the Tracy viewer captures it, set
  `TRACY_NO_EXIT=1`.
- Reconfigure when switching between default and Tracy builds. They use
  separate Pixi environment build directories, so stale binaries are easy to
  spot by path.
