# libccd Parity and Benchmarking

## Purpose

- Validate DART convex-only kernels (GJK/EPA/MPR) against libccd.
- Provide runtime comparisons for support-function based narrow-phase.
- Capture edge-case regressions early, especially for MPR and EPA.

## Local reference

- libccd root (local clone): `/home/js/dev/physics/libccd`
- CMake option: `-DDART_TESTS_LIBCCD_ROOT=/path/to/libccd`
- Fallback: `pkg-config ccd` if a system libccd is installed.

## Unit tests (optional)

`tests/unit/collision/experimental/test_libccd_algorithms.cpp` compares:

- GJK sphere-sphere separation vs analytic solution.
- EPA sphere-sphere penetration vs libccd.
- MPR sphere-sphere penetration vs libccd.
- GJK box-box intersect vs libccd.

Build and run:

```bash
cmake --build build/default/cpp/Release --target test_libccd_algorithms
ctest --test-dir build/default/cpp/Release -R test_libccd_algorithms --output-on-failure
```

## Benchmarks (optional)

`tests/benchmark/collision/experimental/bm_libccd.cpp` includes:

- DART vs libccd GJK intersect (sphere-sphere, box-box).
- DART vs libccd GJK+EPA penetration (sphere-sphere).
- DART vs libccd MPR penetration (sphere-sphere).

Build and run:

```bash
cmake --build build/default/cpp/Release --target bm_experimental_libccd
./build/default/cpp/Release/bin/bm_experimental_libccd
```

## Latest run (2026-01-20, commit db9b3f36dc2)

- Command:
  - `build/default/cpp/Release/bin/bm_experimental_libccd --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_035253.json --benchmark_out_format=json`
- Output:
  - `build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_035253.json`
- Summary (speedup = libccd time / experimental time):
  - GJK sphere-sphere: 0.43x (experimental ~2.3x slower).
  - GJK+EPA sphere-sphere: 12,427x (experimental faster).
  - MPR sphere-sphere: 0.49x (experimental ~2.0x slower).
  - GJK box-box: 0.56x (experimental ~1.8x slower).

## Caution

libccd is a useful reference but not battle-tested across all degeneracies.
Keep parity tests exhaustive and add targeted cases for numeric edge paths.
