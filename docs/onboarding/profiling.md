# Profiling the DART 7 World (text-first)

DART 7 is built so an AI agent (or a human) can find where simulation time goes
**without a GUI profiler** — by reading a plain-text breakdown of a step. This
page is the how-to for that workflow. It maps to real, tracked APIs and uses the
same build switch as the rest of DART's profiling support.

There are two complementary text profilers:

1. **World step profile** (recommended starting point) — a per-stage wall-clock
   breakdown of one `World::step`, surfaced directly on the experimental
   `World` and in dartpy. Use this to answer "which stage of the step is slow?"
2. **Built-in scope profiler** (`dart::common::profile`) — a finer-grained,
   hierarchical scope timer for the rest of the library. Use this when you need
   timing _inside_ a stage or in non-experimental code.

## 1. World step profile

The World step profile requires a build configured with `DART_BUILD_PROFILE=ON`.
The standard `pixi` build and test tasks enable that option for developer
workflows. When `DART_BUILD_PROFILE=OFF`, the runtime toggle is a no-op,
`last_step_profile` stays empty, and `World` carries no profiling cache fields;
the normal `World::step` path has no compiled profiling branch or clock reads.

The DART 7 World step runs an ordered pipeline of named stages
(`rigid_body_velocity`, `rigid_body_contact`, `multibody_forward_dynamics`,
`deformable_dynamics`, `rigid_body_position`, `kinematics`, ...). Step profiling
times each stage and keeps the most recent breakdown.

It is **opt-in and off by default** inside profiling-enabled builds: when the
runtime toggle is disabled the step path uses the same unprofiled pipeline
execution as before. Enable it, take one or more steps, then read the profile.

### Python (dartpy)

```python
import dartpy as dart

world = dart.World()
# ... build your scene ...

world.step_profiling_enabled = True
world.step()                       # or world.step(50) to profile the last of 50

# Use the parallel backend when you want nested graph worker/parallelism data.
executor = sx.ParallelExecutor(8)
world.step(executor)

profile = world.last_step_profile
print(profile.summary())           # compact, sorted, agent-readable table

# Or inspect programmatically:
for stage in profile.stages:       # pipeline order
    print(stage.name, stage.domain, stage.duration_us, stage.acceleration)
    for graph in stage.graph_profiles:
        print(graph.worker_count, graph.max_parallelism, graph.average_parallelism)

contact = profile.get_stage("rigid_body_contact")
if contact is not None:
    print("contact stage:", contact.duration_ms, "ms")
```

`profile.summary()` prints something like:

```
=== World Step Profile ===
steps=1  wall=2.413 ms

Stage                             Domain             Time (ms)    % wall
--------------------------------------------------------------------------
deformable_dynamics               deformable_body        1.802     74.7%
rigid_body_contact                constraint             0.402     16.7%
multibody_forward_dynamics        articulated_body       0.101      4.2%
rigid_body_velocity               rigid_body             0.041      1.7%
rigid_body_position               rigid_body             0.030      1.2%
kinematics                        kinematics             0.018      0.7%
--------------------------------------------------------------------------
(unattributed overhead)                                  0.019      0.8%

Execution details:
- deformable_dynamics: acceleration=task_parallel|data_locality|gpu
- kinematics: acceleration=task_parallel|data_locality, graph_profiles=1, graph_wall=0.018 ms, max_workers=8, max_parallelism=4
```

Fields available on `WorldStepProfile`: `step_count`, `wall_time_us`,
`wall_time_ms`, `total_stage_time_us`, `stages`, `is_empty()`,
`get_stage(name)`, and `summary()`. Each `WorldStepStageProfile` has `name`,
`domain`, `acceleration`, `accelerated_backend_enabled`, `duration_us`,
`duration_ms`, `graph_profiles`, `total_graph_wall_time_us`,
`max_graph_worker_count`, and `max_graph_parallelism`.

Nested `graph_profiles` are `ComputeExecutionProfile` snapshots captured when a
stage runs compute graphs through the injected executor. They expose
`worker_count`, `max_parallelism`, `average_parallelism`, per-node records, and
their own `summary()` text. This is the multi-threading view: stage wall time
answers which stage is slow, while graph profiles answer how much parallel work
the executor actually ran inside that stage. Direct calls to
`ComputeExecutor::executeProfiled()` also require `DART_BUILD_PROFILE=ON`; in
no-profile builds they execute the graph normally and return an empty profile.

`acceleration` is backend-neutral stage metadata. A stage can advertise `gpu`
without exposing CUDA, streams, or device handles; when an accelerated backend
is active for that stage, `accelerated_backend_enabled` is true. For the
deformable solve, this reflects the World-baked PSD projector selected by that
World's `ComputeAcceleratorPolicy`; the legacy flat PSD toggle still affects
direct backend calls outside `World::step()`.

### C++

```cpp
#include <dart/simulation/world.hpp>

dart::simulation::World world;
world.setStepProfilingEnabled(true);
world.step();

const auto& profile = world.getLastStepProfile();
std::cout << profile.toSummaryText();
const auto* contact = profile.getStage("rigid_body_contact");
```

For nested graph timing, inspect `WorldStepStageProfile::graphProfiles` or call
`ComputeExecutionProfile::toSummaryText()` on an individual graph profile.

### Semantics

- The snapshot reflects the **last** built-in pipeline step. For `step(count)`
  it is the final step (matching `getLastDeformableSolverDiagnostics()`); to
  average noise away, step one at a time and aggregate the readings yourself.
- `wall_time` encloses all stages; `wall_time - total_stage_time` is reported as
  the unattributed step overhead (validation, scratch reset, bookkeeping).
- Pair this with `world.last_deformable_solver_diagnostics` (solver iteration
  counts, contact activity) and `world.get_memory_diagnostics()` (frame-scratch
  usage) for a fuller per-step performance picture.

## 2. Built-in scope profiler (`dart::common::profile`)

For finer-grained timing of arbitrary scopes, the core library ships a
text/Tracy profiler controlled by CMake options (`DART_BUILD_PROFILE`,
`DART_PROFILE_BUILTIN`, `DART_PROFILE_TRACY`). The `pixi` tasks enable the text
backend by default. Instrument a scope and dump the aggregated table:

```cpp
#include <dart/common/profile.hpp>

void myHotFunction() {
  DART_PROFILE_SCOPED_N("myHotFunction");
  // ...
}

// somewhere after running:
DART_PROFILE_TEXT_DUMP();   // prints the aggregated per-scope summary

const std::string summary = DART_PROFILE_TEXT_SUMMARY();
// In no-profile or no-text-backend builds, summary is empty.
```

The same text backend is also available through ordinary C++ functions when a
callable API is easier to pass through tools or bindings:

```cpp
if (dart::common::profile::isTextProfilingEnabled()) {
  dart::common::profile::resetProfile();
  // Run instrumented code...
  dart::common::profile::markProfileFrame();
  std::cout << dart::common::profile::getProfileSummaryText();
}
```

These helper calls compile to constants/no-ops (and an empty summary string) in
`DART_BUILD_PROFILE=OFF` or no-text-backend builds.

From dartpy, the same helper surface is promoted to the DART 7 flat namespace:

```python
import dartpy as dart

dart.reset_profile()
# Run instrumented DART code...
print(dart.get_profile_summary_text())
```

`DART_PROFILE_TRACY=ON` additionally streams to the Tracy GUI for local
developer profiling; the text backend stays the portable, agent-friendly path.
For library code that already has access to the text backend, use
`dart::common::profile::Profiler::instance().toSummaryText()` to retrieve the
same text without printing to `std::cout`.

## Where this is implemented

- World step profile type:
  `dart/simulation/compute/world_step_profile.{hpp,cpp}`
- Nested compute graph profile type:
  `dart/simulation/compute/execution_profile.{hpp,cpp}`
- Pipeline timing seam: `WorldStepPipeline::executeProfiled`
  (`dart/simulation/compute/world_step_stage.cpp`)
- World surface: `World::setStepProfilingEnabled` /
  `World::getLastStepProfile` (`dart/simulation/world.hpp`)
- dartpy bindings: `python/dartpy/simulation/module.cpp`
- Built-in scope profiler: `dart/common/profile.hpp`
