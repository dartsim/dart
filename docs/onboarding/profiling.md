# Profiling the experimental World (text-first)

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
`last_step_profile` stays empty, and the normal `World::step` path has no
compiled profiling branch or clock reads.

The experimental World step runs an ordered pipeline of named stages
(`rigid_body_velocity`, `rigid_body_contact`, `multibody_forward_dynamics`,
`deformable_dynamics`, `rigid_body_position`, `kinematics`, ...). Step profiling
times each stage and keeps the most recent breakdown.

It is **opt-in and off by default** inside profiling-enabled builds: when the
runtime toggle is disabled the step path uses the same unprofiled pipeline
execution as before. Enable it, take one or more steps, then read the profile.

### Python (dartpy)

```python
import dartpy.simulation_experimental as sx

world = sx.World()
# ... build your scene ...

world.step_profiling_enabled = True
world.step()                       # or world.step(50) to profile the last of 50

profile = world.last_step_profile
print(profile.summary())           # compact, sorted, agent-readable table

# Or inspect programmatically:
for stage in profile.stages:       # pipeline order
    print(stage.name, stage.domain, stage.duration_us)

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
```

Fields available on `WorldStepProfile`: `step_count`, `wall_time_us`,
`wall_time_ms`, `total_stage_time_us`, `stages`, `is_empty()`,
`get_stage(name)`, and `summary()`. Each `WorldStepStageProfile` has `name`,
`domain`, `duration_us`, and `duration_ms`.

### C++

```cpp
#include <dart/simulation/experimental/world.hpp>

dart::simulation::experimental::World world;
world.setStepProfilingEnabled(true);
world.step();

const auto& profile = world.getLastStepProfile();
std::cout << profile.toSummaryText();
const auto* contact = profile.getStage("rigid_body_contact");
```

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
```

`DART_PROFILE_TRACY=ON` additionally streams to the Tracy GUI for local
developer profiling; the text backend stays the portable, agent-friendly path.

## Where this is implemented

- World step profile type:
  `dart/simulation/experimental/compute/world_step_profile.{hpp,cpp}`
- Pipeline timing seam: `WorldStepPipeline::executeProfiled`
  (`dart/simulation/experimental/compute/world_step_stage.cpp`)
- World surface: `World::setStepProfilingEnabled` /
  `World::getLastStepProfile` (`dart/simulation/experimental/world.hpp`)
- dartpy bindings: `python/dartpy/simulation_experimental/module.cpp`
- Built-in scope profiler: `dart/common/profile.hpp`
