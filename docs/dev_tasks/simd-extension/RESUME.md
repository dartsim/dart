# Resume: dart::simd Extension

## Last Session Summary (2026-01-21)

**ALL 8 SPRINTS COMPLETE.** Naming convention refactor finished - all snake_case functions renamed to camelCase. Build passes (529 targets), all 8 SIMD unit tests pass, lint passes, benchmarks verified.

## Current Branch

`feature/simd` — has uncommitted changes (significant new functionality)

## Immediate Next Step

**Commit current work** or **integrate into DART core** (collision detection, dynamics).

## What's Done

| Sprint | Description                       | Status |
| ------ | --------------------------------- | ------ |
| 1      | drjit benchmark integration       | ✅     |
| 2      | Pure AVX backend                  | ✅     |
| 3      | Vector3/Vector4 geometric types   | ✅     |
| 4      | Matrix3x3/Matrix4x4               | ✅     |
| 5      | ECS batch operations (EigenSoA)   | ✅     |
| 6      | FMA optimization + loop unrolling | ✅     |
| 7      | Quaternion + Isometry3            | ✅     |
| 8      | DynamicVector + DynamicMatrix     | ✅     |
| -      | Naming refactor (camelCase)       | ✅     |

## Benchmark Performance (65536 elements, AVX2+FMA)

| Operation        | DART       | drjit     | Advantage |
| ---------------- | ---------- | --------- | --------- |
| FMA f32 (unroll) | 134.6 Gi/s | 84.8 Gi/s | **+59%**  |
| Dot f32 (unroll) | 157.1 Gi/s | 36.5 Gi/s | **+330%** |
| Mul f32          | 85.7 Gi/s  | 80.8 Gi/s | **+6%**   |
| Add f32          | 85.6 Gi/s  | 90.4 Gi/s | -5%       |

## Context That Would Be Lost

- All naming now uses camelCase: `fromAxisAngle`, `transformPoint`, `toEigen`, `simdChunks`
- Previous snake_case in tests caused build failures until fixed
- `toVec3Padded` (not `toVec3_padded`) for padded vector conversion
- Benchmarks use `preferred_width_v<T>` for native SIMD width

## Uncommitted Changes

```
Modified:
  .github/workflows/ci_simd.yml
  dart/simd/CMakeLists.txt, config.hpp, simd.hpp
  dart/simd/detail/scalar/vec.hpp
  dart/simd/eigen/interop.hpp
  tests/benchmark/simd/bm_simd.cpp
  tests/unit/simd/test_eigen_interop.cpp, test_geometry.cpp, test_dynamic.cpp

New:
  dart/simd/detail/avx/          # Pure AVX backend
  dart/simd/geometry/            # Vector3, Vector4, Matrix3x3, Matrix4x4, Quaternion, Isometry3
  dart/simd/dynamic/             # DynamicVector, DynamicMatrix
  dart/simd/eigen/iterator.hpp   # simdChunks streaming iterator
  docs/dev_tasks/simd-extension/

Deleted:
  tests/benchmark/simd/bm_simd_vs_drjit.cpp
```

## How to Resume

```bash
git checkout feature/simd
git status  # Verify uncommitted changes
pixi run test-unit simd  # Should pass (8/8 tests)
pixi run bm-simd         # Verify benchmarks work
```

Then: Commit the work, or continue with integration into DART core.

## Key Commands

```bash
pixi run test-unit simd    # SIMD unit tests (8 suites)
pixi run bm-simd           # DART-only benchmarks
pixi run bm-simd-drjit     # With drjit comparison
pixi run lint              # Format before commit
```

## Potential Next Steps

1. **Commit current work** — All sprints complete, ready for review
2. **Integration into DART core** — Apply to collision detection, dynamics
3. **ARM SVE backend** — Deferred research spike (detection macros ready)
4. **Additional optimizations** — Matrix inverse, SVD, advanced operations
