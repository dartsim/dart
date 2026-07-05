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
