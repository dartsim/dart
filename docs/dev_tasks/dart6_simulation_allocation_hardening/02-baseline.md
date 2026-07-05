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
