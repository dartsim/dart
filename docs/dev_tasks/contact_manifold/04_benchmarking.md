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
| 4x4x4 | 1944 | 3270 | +68% |
| 8x8x8 | 21840 | 22843 | +5% |

Note: 4x4x4 numbers use a repeated run (mean of 3) with:

```bash
./build/default/cpp/Release/bin/bm_boxes \
  --benchmark_filter="BM_RunBoxes/4|BM_RunBoxesPersistentContacts/4" \
  --benchmark_min_time=1s --benchmark_repetitions=3
```

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

### Post-optimization run (vector grouping)

Run info:

- Date: 2025-12-31
- CPU scaling enabled and higher load variance (see `bm_boxes` CV)

#### Cache update micro-benchmark (bm_contact_patch_cache)

Times in microseconds:

| Pairs | Contacts/Pair | Time |
| --- | --- | --- |
| 1 | 4 | 0.731 |
| 1 | 16 | 2.04 |
| 10 | 4 | 7.16 |
| 10 | 16 | 22.1 |
| 100 | 4 | 83.6 |
| 100 | 16 | 235 |

#### Collision benchmark (bm_boxes, 4x4x4 only)

Times in milliseconds (mean of 3):

| Case | Legacy | Cache ON | Delta |
| --- | --- | --- | --- |
| 4x4x4 | 2763 | 3871 | +40% |

Note: this run had higher variance (18% CV for legacy), so compare with care.

### Post-optimization run (output scratch)

Run info:

- Date: 2025-12-31
- CPU scaling enabled (benchmark warning about noisy real-time)

#### Cache update micro-benchmark (bm_contact_patch_cache)

Times in microseconds:

| Pairs | Contacts/Pair | Time |
| --- | --- | --- |
| 1 | 4 | 0.691 |
| 1 | 16 | 1.91 |
| 10 | 4 | 6.56 |
| 10 | 16 | 20.6 |
| 100 | 4 | 66.7 |
| 100 | 16 | 207 |

#### Collision benchmark (bm_boxes, 4x4x4 only)

Times in milliseconds (mean of 3):

| Case | Legacy | Cache ON | Delta |
| --- | --- | --- | --- |
| 4x4x4 | 1958 | 3200 | +63% |

### Post-optimization run (direct output)

Run info:

- Date: 2025-12-31
- CPU scaling enabled (benchmark warning about noisy real-time)

#### Cache update micro-benchmark (bm_contact_patch_cache)

Times in microseconds:

| Pairs | Contacts/Pair | Time |
| --- | --- | --- |
| 1 | 4 | 0.625 |
| 1 | 16 | 1.77 |
| 10 | 4 | 5.94 |
| 10 | 16 | 18.7 |
| 100 | 4 | 61.9 |
| 100 | 16 | 193 |

#### Collision benchmark (bm_boxes, 4x4x4 only)

Times in milliseconds (mean of 3):

| Case | Legacy | Cache ON | Delta |
| --- | --- | --- | --- |
| 4x4x4 | 1934 | 3242 | +68% |

### Post-optimization run (scratch reuse)

Run info:

- Date: 2025-12-31
- CPU scaling enabled (benchmark warning about noisy real-time)

#### Cache update micro-benchmark (bm_contact_patch_cache)

Times in microseconds:

| Pairs | Contacts/Pair | Time |
| --- | --- | --- |
| 1 | 4 | 0.585 |
| 1 | 16 | 1.65 |
| 10 | 4 | 5.81 |
| 10 | 16 | 18.0 |
| 100 | 4 | 60.8 |
| 100 | 16 | 181 |

#### Collision benchmark (bm_boxes, 4x4x4 only)

Times in milliseconds (mean of 3):

| Case | Legacy | Cache ON | Delta |
| --- | --- | --- | --- |
| 4x4x4 | 2015 | 3313 | +64% |

### Post-optimization run (candidate reuse)

Run info:

- Date: 2025-12-31
- CPU scaling enabled (benchmark warning about noisy real-time)

#### Cache update micro-benchmark (bm_contact_patch_cache)

Times in microseconds:

| Pairs | Contacts/Pair | Time |
| --- | --- | --- |
| 1 | 4 | 0.618 |
| 1 | 16 | 1.68 |
| 10 | 4 | 5.72 |
| 10 | 16 | 17.3 |
| 100 | 4 | 57.4 |
| 100 | 16 | 176 |

#### Collision benchmark (bm_boxes, 4x4x4 only)

Times in milliseconds (mean of 3):

| Case | Legacy | Cache ON | Delta |
| --- | --- | --- | --- |
| 4x4x4 | 1975 | 3165 | +60% |
