# Contact Patch Cache Benchmarking Plan (04)

## Status

- Extended `tests/benchmark/collision/bm_boxes.cpp` with cache ON/OFF modes
- Results not captured yet (pending local runs)

## Existing Benchmark Harness

- Benchmarks live under `tests/benchmark/`
- Benchmarks are standalone executables, not ctests
- Example runner: `tests/benchmark/collision/bm_boxes.cpp`

## Proposed Benchmarks

1. Extend collision benchmark
   - Updated `tests/benchmark/collision/bm_boxes.cpp`
   - Runs two modes: feature OFF and feature ON
   - Record wall time per simulation step

2. Contact update micro-benchmark
   - Follow-up: add `tests/benchmark/collision/bm_contact_patch_cache.cpp`
   - Feed synthetic contact lists with varying sizes and pairs
   - Measure cache update cost only

3. Allocation and memory
   - Track allocations per step (optional, if instrumentation exists)
   - Ensure no per-frame heap churn when cache is enabled

## Metrics to Capture

- Contact update time per step
- Constraint build time per step
- Total simulation step time
- Peak pair count and patch count

## How to Run

Suggested (Unverified):

```bash
cmake --build build/default/cpp/Release --target bm_boxes
./build/default/cpp/Release/tests/benchmark/bm_boxes
```

For custom runs (Unverified):

```bash
./build/default/cpp/Release/tests/benchmark/bm_boxes \
  --benchmark_min_time=1s --benchmark_repetitions=5
```
