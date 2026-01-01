# Progress Tracker

## Stage Checklist

- [x] Stage 0: Recon and requirements capture
- [x] Stage 1: Plan docs
- [x] Stage 2: Skeleton implementation (feature gated, compiles)
- [x] Stage 3: Full behavior wiring (use persistent contacts)
- [x] Stage 4: Tests and regression protection
- [x] Stage 4.5: GUI demo
- [x] Stage 5: Benchmarks and documentation polish

## Current Status

- Stage 5 ongoing: perf tuning and benchmark tracking
- Workflow: run `pixi run lint` before every commit
- Benchmark rebuild pending (last `bm_boxes`/`bm_contact_manifold` build was
  interrupted)

## Stage 5 Notes

- Renamed ContactPatchCache to ContactManifoldCache (API, files, docs)
- Benchmark coverage added in `tests/benchmark/collision/bm_boxes.cpp`
- Added micro-benchmark `tests/benchmark/collision/bm_contact_manifold_cache.cpp`
- Results captured (see `04_benchmarking.md`)
- Cache now emits persisted contacts for unseen pairs up to
  `maxSeparationFrames`
- Backend contact history can be disabled via `CollisionOption`
- Added `ConstraintSolver::getContactsUsedForConstraints()` and updated
  `World::bake()` to use constraint contacts when recording
- Repeated 4x4x4 benchmark run confirms ~68% overhead with cache enabled
- Added `World::getContactsUsedForConstraints()` convenience API
- Switched cache update grouping to sorted vectors (removed maps)
- Post-change micro-benchmark shows faster small/medium cases; 4x4x4
  `bm_boxes` run still shows overhead but with higher variance
- Added output scratch buffer to reduce per-pair allocations
- Latest 4x4x4 run shows ~63% overhead with cache enabled (stable CV)
- Direct output merge (no extra copy) further improved micro-benchmark numbers
  but overall 4x4x4 overhead remains ~68%
- Reused scratch buffers inside the cache to cut per-frame allocations
- Latest micro-benchmark shows continued gains; 4x4x4 overhead ~64%
- Reused per-pair candidate vectors inside update loop
- Latest 4x4x4 run shows ~60% overhead with cache enabled
- Added fast path when only fresh contacts fit within max points
- Latest micro-benchmark shows large gains for small contact sets; 4x4x4
  overhead remains ~64%
- Added thread-local scratch reuse for per-pair candidate buffers
- Latest 4x4x4 run shows ~65% overhead with cache enabled
- Added empty-manifold fast path for first-frame pairs with <= max points
- Latest 4x4x4 run shows ~65% overhead with cache enabled (low CV)
- Synced persistent contact forces back into raw collision results
- Added integration coverage for force sync when cache is enabled
- Latest 4x4x4 run after force sync shows ~67% overhead with cache enabled

## Verification

- Benchmarks:
  - `pixi run -- cmake --build build/default/cpp/Release --target bm_boxes`
  - `pixi run -- cmake --build build/default/cpp/Release --target bm_contact_manifold_cache`
  - `pixi run -- ./build/default/cpp/Release/bin/bm_boxes`
  - `pixi run -- ./build/default/cpp/Release/bin/bm_contact_manifold_cache`
- Tests:
  - `pixi run -- cmake --build build/default/cpp/Release --target UNIT_constraint_ContactManifoldCache`
  - `pixi run -- cmake --build build/default/cpp/Release --target INTEGRATION_simulation_ContactManifoldCache`
  - `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R ContactManifoldCache`

## Decisions

- Name: ContactManifoldCache
- Default: feature OFF
- Keep CollisionResult semantics unchanged
- Soft contacts stay on legacy path initially
- Allow short TTL output when raw contacts drop out

## Open Questions

- TTL value for inactive pairs
- Uniqueness thresholds and normal-angle cutoff
- Final data layout (AoS vs SoA)
