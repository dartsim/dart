# Resume: dart::simd Extension

## Last Session Summary (2026-01-22)

**ALL 8 SPRINTS + SVE BACKEND COMPLETE.** Added ARM SVE backend with fixed-width Vec/VecMask implementation. All work committed and pushed to `feature/simd`. CI running.

## Current Branch

`feature/simd` — clean, pushed to origin

## Immediate Next Step

**Wait for CI** to verify ARM64 NEON build passes, then **integrate into DART core** (collision detection, dynamics).

## What's Done

| Sprint | Description                       | Status |
| ------ | --------------------------------- | ------ |
| 1      | Benchmark infrastructure          | ✅     |
| 2      | Pure AVX backend                  | ✅     |
| 3      | Vector3/Vector4 geometric types   | ✅     |
| 4      | Matrix3x3/Matrix4x4               | ✅     |
| 5      | ECS batch operations (EigenSoA)   | ✅     |
| 6      | FMA optimization + loop unrolling | ✅     |
| 7      | Quaternion + Isometry3            | ✅     |
| 8      | DynamicVector + DynamicMatrix     | ✅     |
| -      | Naming refactor (camelCase)       | ✅     |
| -      | ARM SVE backend                   | ✅     |

## Benchmark Performance (65536 elements, AVX2+FMA)

| Operation          | DART     | vs drjit |
| ------------------ | -------- | -------- |
| Dot f32 (unrolled) | 184 Gi/s | +359%    |
| FMA f32            | 103 Gi/s | +34%     |
| MatVec 180         | 141 Gi/s | +136%    |
| TransformPoints 1K | 255 Gi/s | +128%    |

## Context That Would Be Lost

- SVE uses scalar fallback for VecMask (SVE predicates are variable-length)
- macOS CI uses NEON (Apple Silicon), not SVE (SVE requires Graviton3/Ampere)
- drjit comparison benchmarks moved to `/tmp/dart-simd-benchmark` (private repo)
- All naming uses camelCase: `fromAxisAngle`, `transformPoint`, `toEigen`, `simdChunks`

## How to Resume

```bash
git checkout feature/simd
git status  # Should be clean
gh run list --branch feature/simd  # Check CI status
pixi run test-unit simd  # Local verification (8/8 tests)
```

## Key Commands

```bash
pixi run test-unit simd    # SIMD unit tests (8 suites)
pixi run bm-simd           # SIMD benchmarks
pixi run lint              # Format before commit
gh run view <run-id>       # Check CI status
```

## Potential Next Steps

1. **Wait for CI** — Verify ARM64 NEON build passes (SVE compiles but can't execute on macOS)
2. **Integration into DART core** — Apply to collision detection, dynamics
3. **Test SVE on real hardware** — Requires AWS Graviton3 or Ampere Altra
4. **Additional optimizations** — Matrix inverse, SVD, advanced operations
