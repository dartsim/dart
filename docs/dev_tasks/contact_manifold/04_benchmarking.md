# Contact Patch Cache Benchmarking Plan (04)

## Status

- Extended `tests/benchmark/collision/bm_boxes.cpp` with cache ON/OFF modes
- Added `tests/benchmark/collision/bm_contact_patch_cache.cpp` micro-benchmark
- Results captured (see Results)

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
   - Added `tests/benchmark/collision/bm_contact_patch_cache.cpp`
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

Commands used:

```bash
cmake --build build/default/cpp/Release --target bm_boxes
cmake --build build/default/cpp/Release --target bm_contact_patch_cache
./build/default/cpp/Release/bin/bm_boxes
./build/default/cpp/Release/bin/bm_contact_patch_cache
```

For custom runs:

```bash
./build/default/cpp/Release/bin/bm_boxes \
  --benchmark_min_time=1s --benchmark_repetitions=5
```

## Results

Run info:

- Date: 2025-12-31
- Build: `build/default/cpp/Release` (pixi default environment)
- CPU scaling enabled (benchmark warning about noisy real-time)

### Collision benchmark (bm_boxes)

Times in milliseconds (lower is better):

| Case | Legacy | Cache ON | Delta |
| --- | --- | --- | --- |
| 2x2x2 | 150 | 149 | -1% |
| 4x4x4 | 1944 | 3292 | +69% |
| 8x8x8 | 21840 | 22843 | +5% |

### Cache update micro-benchmark (bm_contact_patch_cache)

Times in microseconds:

| Pairs | Contacts/Pair | Time |
| --- | --- | --- |
| 1 | 4 | 0.852 |
| 1 | 16 | 2.34 |
| 10 | 4 | 10.6 |
| 10 | 16 | 21.4 |
| 100 | 4 | 92.1 |
| 100 | 16 | 237 |
