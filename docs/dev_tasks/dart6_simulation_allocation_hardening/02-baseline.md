# DART 6 Simulation Allocation Baseline

## Allocation baseline (pre-change)

This is the allocation baseline after adding the reporting harness, before any
runtime allocation-hardening changes. The reporting test is non-asserting for
`World::step()` allocation counts; only the counter self-tests assert.

### Commands run

```bash
XDG_CACHE_HOME=/home/js/dev/dartsim/dart/task_9-mem620/build/gersemi-cache pixi run lint
pixi run cmake --build build/default/cpp/Release --target INTEGRATION_StepAllocation --parallel
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation '--gtest_filter=StepAllocation.*'
(cd build/default/cpp/Release && ctest -R StepAllocation --output-on-failure)
pixi run build-tests
```

`pixi run lint` used a workspace-local `XDG_CACHE_HOME` because the default
home-directory gersemi cache was stale/corrupt in this sandbox; the same Pixi
lint task completed cleanly with that cache location.

### Reporting test output

Native DART collision detector scene (`native_dart_boxes`,
`boxes_per_side=3`, `warmup_steps=50`, `measured_steps=100`):

```text
operator_new_count=583502 operator_new_bytes=53762172
operator_new_count_per_step=5835.020 operator_new_bytes_per_step=537621.720
raw_malloc_count=586202 raw_malloc_bytes=54539772
raw_malloc_count_per_step=5862.020 raw_malloc_bytes_per_step=545397.720
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0
counting_allocator_deallocate_count=0 counting_allocator_deallocate_bytes=0
counting_allocator_allocate_count_per_step=0.000
```

Bullet collision detector comparison scene (`bullet_boxes`,
`boxes_per_side=3`, `warmup_steps=50`, `measured_steps=100`; includes
collision-backend-internal allocations):

```text
operator_new_count=583169 operator_new_bytes=53960891
operator_new_count_per_step=5831.690 operator_new_bytes_per_step=539608.910
raw_malloc_count=585953 raw_malloc_bytes=54872027
raw_malloc_count_per_step=5859.530 raw_malloc_bytes_per_step=548720.270
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0
counting_allocator_deallocate_count=0 counting_allocator_deallocate_bytes=0
counting_allocator_allocate_count_per_step=0.000
```

## WP-D6M.3 evidence

### Commands run

```bash
pixi run lint
pixi run build
pixi run cmake --build build/default/cpp/Release --target UNIT_common_FrameAllocator UNIT_common_FreeListAllocator UNIT_common_MemoryManager --parallel
(cd build/default/cpp/Release && ctest -R "(FrameAllocator|MemoryManager|FreeListAllocator)" --output-on-failure)
pixi run build-tests
```

### Focused ctest summary

```text
Test #70: UNIT_common_FrameAllocator ....... Passed
Test #71: UNIT_common_FreeListAllocator .... Passed
Test #73: UNIT_common_MemoryManager ........ Passed
100% tests passed, 0 tests failed out of 3
```

### API surface note

Additive only: new `dart::common::FrameAllocator` and `FrameStlAllocator<T>`;
new `MemoryManager::Type::Frame` role with `getFrameAllocator()` and
allocate/deallocate dispatch; new `FreeListAllocator::GrowthPolicy` with
default `Expand` and opt-in `FixedCapacity`.

## Allocation attribution (backtrace sampling, 2026-07-05)

Command: `DART_TEST_ALLOCATION_BACKTRACE=1 ./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation --gtest_filter='StepAllocation.ReportsWorldStepAllocationBaseline'`

Top steady-state operator-new sources on the measured windows (both scenes,
top-25 sites per scene, 199 distinct sites on the native scene):

| Source | Approx. allocs/step | Share |
| --- | --- | --- |
| Built-in text profiler (`Profiler::pushScope`/`findOrCreateChild` string churn; `DART_BUILD_PROFILE=ON` in the default pixi config) | ~5,393 | ~92% |
| Dantzig LCP `LCP<double>::transfer_i_from_C_to_N` (pivot path) | ~26 | <1% |
| `World::step` direct (free-root snapshot vectors) | ~29 | <1% |
| Long tail (sites beyond top-25 dump) | ~385 | ~7% |

Consequences recorded in `README.md`: new packet WP-D6M.9 (allocation-free
profiler steady state; DART 7 `main` shares the same pattern, so this is
novel work here, forward-port candidate later), WP-D6M.5 updated with the
Dantzig pivot site, and a deep-dump pass required to attribute the long
tail before WP-D6M.5 implementation.

## WP-D6M.9 evidence

### Commands run

```bash
DART_PROFILE_COLOR=0 ./build/default/cpp/Release/tests/benchmark/integration/boxes_headless 3 50 25 > /tmp/dart9_boxes_before.txt
pixi run cmake --build build/default/cpp/Release --target UNIT_common_Profile boxes_headless INTEGRATION_StepAllocation --parallel
ctest --test-dir build/default/cpp/Release -R "(Profile|StepAllocation)" --output-on-failure
DART_PROFILE_COLOR=0 ./build/default/cpp/Release/tests/benchmark/integration/boxes_headless 3 50 25 > /tmp/dart9_boxes_after_3.txt
DART_TEST_ALLOCATION_BACKTRACE=1 ./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation --gtest_filter='StepAllocation.ReportsWorldStepAllocationBaseline'
```

`/tmp/dart9_boxes_before.txt` was captured immediately before the profiler
storage change; `/tmp/dart9_boxes_after_3.txt` was captured after rebuilding
the same target.

### Allocation reporting output

Native DART collision detector scene (`native_dart_boxes`,
`boxes_per_side=3`, `warmup_steps=50`, `measured_steps=100`):

```text
operator_new_count=5602 operator_new_bytes=1780672
operator_new_count_per_step=56.020 operator_new_bytes_per_step=17806.720
raw_malloc_count=8302 raw_malloc_bytes=2558272
raw_malloc_count_per_step=83.020 raw_malloc_bytes_per_step=25582.720
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0
counting_allocator_deallocate_count=0 counting_allocator_deallocate_bytes=0
counting_allocator_allocate_count_per_step=0.000
```

Bullet collision detector comparison scene (`bullet_boxes`,
`boxes_per_side=3`, `warmup_steps=50`, `measured_steps=100`; includes
collision-backend-internal allocations):

```text
operator_new_count=6019 operator_new_bytes=2043872
operator_new_count_per_step=60.190 operator_new_bytes_per_step=20438.720
raw_malloc_count=8803 raw_malloc_bytes=2955008
raw_malloc_count_per_step=88.030 raw_malloc_bytes_per_step=29550.080
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0
counting_allocator_deallocate_count=0 counting_allocator_deallocate_bytes=0
counting_allocator_allocate_count_per_step=0.000
```

The `operator_new_count_per_step` values dropped from 5835.020 to 56.020 for
`native_dart_boxes` and from 5831.690 to 60.190 for `bullet_boxes`.

### Backtrace attribution

The post-change backtrace dump reported 6 distinct sampled sites for
`native_dart_boxes` and 7 for `bullet_boxes`. The former top profiler sites
(`Profiler::pushScope`, `Profiler::findOrCreateChild`, and
`std::string::_M_append` under those paths) no longer appeared in the top
backtrace list. The largest remaining sites were direct `World::step` scratch
work and Dantzig LCP pivot-path allocations.

### Text-profiler report parity

The before/after `boxes_headless 3 50 25` dumps preserved the same simulation
checksums at steps 25 and 50 and the same profiler hierarchy for the stable
per-step scopes: labels, sources, and call counts stayed aligned for the
printed `World::step`, `ConstraintSolver::solve`, `updateConstraints`,
collision, Bullet update, and contact-constraint scopes. A sorted tree-label
comparison showed every pre-change printed label still present after the
change; one very small existing scope
(`World::step - Wake resting after world change`, 29 calls, 13.94 us total)
crossed the profiler's pre-existing timing-based print threshold after the
profiler overhead was removed, so the tiny-leaf portion of the text dump is not
byte-identical.

## Residual attribution after WP-D6M.9 (profiler fix, 2026-07-05)

With the profiler allocation-free, the native-DART-collision scene drops to
~60 operator-new/step, from exactly two DART-owned sources (backtrace dump,
100 measured steps):

| Source | allocs/step | bytes/step | Target packet |
| --- | --- | --- | --- |
| `World::step` free-root snapshot vectors (`World.cpp` shallow-support feature) | ~29 (+~2 one-shot vectors) | ~2.5 KB | WP-D6M.5 |
| Dantzig `LCP<double>::transfer_i_from_C_to_N` pivot buffer | ~28 | ~17.6 KB (dominant) | WP-D6M.5 |

No profiler frames remain in the top sites; there is no significant long
tail. WP-D6M.5 now has a fully attributed, two-source target and can route
both through the World-owned MemoryManager (WP-D6M.4).

## WP-D6M.4 evidence

### Commands run

```bash
DART_PROFILE_COLOR=0 ./build/default/cpp/Release/tests/benchmark/integration/boxes_headless 8 200 100 > /tmp/dart_d6m4_boxes_before.txt
pixi run cmake --build build/default/cpp/Release --target INTEGRATION_StepAllocation --parallel
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation --gtest_filter='WorldSimulationModeMemoryManager.*'
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation --gtest_filter='StepAllocation.*'
DART_PROFILE_COLOR=0 ./build/default/cpp/Release/tests/benchmark/integration/boxes_headless 8 200 100 > /tmp/dart_d6m4_boxes_after.txt
diff -u <(rg '^step' /tmp/dart_d6m4_boxes_before.txt) <(rg '^step' /tmp/dart_d6m4_boxes_after.txt)
ctest --test-dir build/default/cpp/Release -R StepAllocation --output-on-failure
pixi run lint
pixi run cmake --build build/default/cpp/Release --target INTEGRATION_StepAllocation --parallel
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation --gtest_filter='WorldSimulationModeMemoryManager.*'
ctest --test-dir build/default/cpp/Release -R StepAllocation --output-on-failure
pixi run build
pixi run build-tests
```

### New test results

```text
WorldSimulationModeMemoryManager.ExplicitEnterMatchesImplicitSteps PASSED
WorldSimulationModeMemoryManager.ImplicitFirstStepMatchesExplicitEnter PASSED
WorldSimulationModeMemoryManager.ShapeChangeInvalidatesAndRebakes PASSED
WorldSimulationModeMemoryManager.FrameArenaResetsEachStep PASSED
WorldSimulationModeMemoryManager.BakedWorldBaseAllocatorDoesNotGrow PASSED
```

The injected-base-allocator test constructs a `WorldConfig` with
`CountingMemoryAllocator` as the World MemoryManager base allocator, calls
`enterSimulationMode()`, then runs 10 same-shape steps under
`ScopedCountingMemoryAllocatorCounter`. The observed and asserted World base
allocator growth was zero allocations and zero bytes.

### Allocation reporting check

`StepAllocation.ReportsWorldStepAllocationBaseline` still reports the same
post-WP-D6M.9 steady-state values:

```text
native_dart_boxes operator_new_count_per_step=56.020 counting_allocator_allocate_count=0
bullet_boxes operator_new_count_per_step=60.190 counting_allocator_allocate_count=0
```

### Bit-exactness check

The `boxes_headless 8 200 100` step-checkpoint diff was empty. Both before and
after runs reported:

```text
step    100  dofs  3072  posL1 25.377535997306794  posSq 1.2565833038073266  velL1 502.39999999999804  velSq 492.72899199999836
step    200  dofs  3072  posL1 100.9822719973081  posSq 19.906743941630715  velL1 1004.6719999999947  velSq 1970.9154880000115
```

### Gate results

```text
pixi run lint PASSED
pixi run build PASSED
pixi run build-tests PASSED
pixi run cmake --build build/default/cpp/Release --target INTEGRATION_StepAllocation --parallel PASSED
ctest --test-dir build/default/cpp/Release -R StepAllocation --output-on-failure PASSED
```

## WP-D6M.5 evidence

### Commands run

```bash
pixi run cmake --build build/default/cpp/Release --target INTEGRATION_StepAllocation UNIT_common_Profile --parallel
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation '--gtest_filter=WorldSimulationModeMemoryManager.*:StepAllocation.*'
DART_TEST_ALLOCATION_BACKTRACE=1 ./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation --gtest_filter='StepAllocation.ReportsWorldStepAllocationBaseline'
DART_PROFILE_COLOR=0 ./build/default/cpp/Release/tests/benchmark/integration/boxes_headless 8 2000 500 > /tmp/dart_d6m5_boxes_headless.log
diff -u <(rg '^step' docs/dev_tasks/dart6_simulation_allocation_hardening/baseline/boxes_headless.log) <(rg '^step' /tmp/dart_d6m5_boxes_headless.log)
```

### Allocation reporting output

Native DART collision detector scene (`native_dart_boxes`,
`boxes_per_side=3`, `warmup_steps=50`, `measured_steps=100`,
`last_step_contacts=108`):

```text
operator_new_count=0 operator_new_bytes=0
operator_new_count_per_step=0.000 operator_new_bytes_per_step=0.000
raw_malloc_count=0 raw_malloc_bytes=0
raw_malloc_count_per_step=0.000 raw_malloc_bytes_per_step=0.000
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0
counting_allocator_deallocate_count=0 counting_allocator_deallocate_bytes=0
counting_allocator_allocate_count_per_step=0.000
```

Bullet comparison scene (`bullet_boxes`, same shape; external-backend
comparison, not the strict native gate):

```text
operator_new_count=0 operator_new_bytes=0
operator_new_count_per_step=0.000 operator_new_bytes_per_step=0.000
raw_malloc_count=84 raw_malloc_bytes=133536
raw_malloc_count_per_step=0.840 raw_malloc_bytes_per_step=1335.360
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0
counting_allocator_deallocate_count=0 counting_allocator_deallocate_bytes=0
counting_allocator_allocate_count_per_step=0.000
```

### Backtrace attribution

With raw malloc-family sampling enabled in the allocation harness, the native
DART collision scene reported `distinct_sites=0` and `dropped_samples=0`.
The Bullet comparison retained one raw-malloc site
(`ConstraintSolver::solveConstrainedGroups`, 84 allocations / 133536 bytes
over 100 measured steps). Operator-new and World base/counting allocator
surfaces were zero for both scenes.

### Bit-exactness check

`boxes_headless 8 2000 500` step checkpoints matched
`baseline/boxes_headless.log` exactly (`diff` output empty).

### Implementation note

The closure removed the measured native residuals by retaining the Dantzig
LDLT-removal tmp buffer in `DantzigLcpScratch`, reusing World free-root and
support-detection scratch storage, replacing a per-step support-detection map
with allocation-free lookup, scanning body external wrenches directly in
`Skeleton::checkExternalDisturbanceAndReset()`, and bounding profiler frame
samples so `Profiler::markFrame()` no longer grows during steady-state steps.

## WP-D6M.6 evidence

### Commands run

```bash
pixi run cmake --build build/default/cpp/Release --target INTEGRATION_StepAllocation --parallel
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation '--gtest_filter=StepAllocation.Native*:*ExternalBackends*:StepAllocation.AllocationGateRejectsInjectedAllocationMeasurement'
pixi run cmake --build build/default/cpp/Release --target INTEGRATION_StepAllocation UNIT_common_Profile boxes_headless --parallel
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation '--gtest_filter=WorldSimulationModeMemoryManager.*:StepAllocation.*'
ctest --test-dir build/default/cpp/Release -R '(Profile|StepAllocation)' --output-on-failure
DART_PROFILE_COLOR=0 ./build/default/cpp/Release/tests/benchmark/integration/boxes_headless 8 2000 500 > /tmp/dart_d6m6_boxes_headless.log
diff -u <(rg '^step' docs/dev_tasks/dart6_simulation_allocation_hardening/baseline/boxes_headless.log) <(rg '^step' /tmp/dart_d6m6_boxes_headless.log)
pixi run lint
```

### First-post-bake gate output

Native DART collision strict gates (`boxes_per_side=3`, one measured step,
`last_step_contacts=108`) assert all three surfaces:

```text
native_dart_explicit_first_post_bake_gate:
operator_new_count=0 operator_new_bytes=0
raw_malloc_count=0 raw_malloc_bytes=0
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0

native_dart_implicit_second_step_gate:
operator_new_count=0 operator_new_bytes=0
raw_malloc_count=0 raw_malloc_bytes=0
counting_allocator_allocate_count=0 counting_allocator_allocate_bytes=0
```

The raw-malloc-specific native tests reported the same zero values for
explicit first post-bake and implicit second-step paths on this Linux/glibc
host. On ASan, codecov, or non-glibc hosts, the raw-malloc gate records a
GTest skip instead of a pass.

### External-backend compatibility gates

Bullet and ODE are intentionally gated only on the World base allocator
surface, because the global counters include backend-internal allocation:

```text
bullet_explicit_first_post_bake_base_gate counting_allocator_allocate_count=0
ode_explicit_first_post_bake_base_gate counting_allocator_allocate_count=0
bullet_implicit_second_step_base_gate counting_allocator_allocate_count=0
ode_implicit_second_step_base_gate counting_allocator_allocate_count=0
```

The same run recorded nonzero global/raw counts for Bullet and ODE, as
expected for external-backend compatibility reporting rather than native strict
gate evidence.

### Verification summary

`WorldSimulationModeMemoryManager.*:StepAllocation.*` passed all 16 tests.
Focused ctest passed `INTEGRATION_StepAllocation` and `UNIT_common_Profile`.
The `boxes_headless 8 2000 500` checkpoint diff against
`baseline/boxes_headless.log` was empty. `pixi run lint` passed.

### Implementation note

The gates initially exposed allocations that the warm reporting test could not
see: first-step contact/vector double-buffer growth, Dantzig LDLT-removal
scratch under-reservation, deactivation group scratch, and built-in profiler
first-visit node allocation. WP-D6M.6 closes those by preparing solver contact
and group scratch from `World::enterSimulationMode()`, mirroring reusable
contact-vector capacity, reserving Dantzig scratch to the LDLT removal bound,
reserving deactivation group scratch, and changing the built-in profiler to use
a per-thread pre-reserved node arena with `string_view` labels/sources.
