# Progress Tracker

## Stage Checklist

- [x] Stage 0: Recon and requirements capture
- [x] Stage 1: Plan docs
- [x] Stage 2: Skeleton implementation (feature gated, compiles)
- [x] Stage 3: Full behavior wiring (use persistent contacts)
- [x] Stage 4: Tests and regression protection
- [x] Stage 4.5: GUI demo
- [ ] Stage 5: Benchmarks and documentation polish

## Current Status

- Stage 5 in progress: benchmarks extended, results pending
- Update pending: record benchmark results and finalize docs

## Stage 5 Notes

- Benchmark coverage added in `tests/benchmark/collision/bm_boxes.cpp`
- Added micro-benchmark `tests/benchmark/collision/bm_contact_patch_cache.cpp`
- Results not captured yet (pending local runs)
- Cache now emits persisted contacts for unseen pairs up to
  `maxSeparationFrames`

## Decisions

- Name: ContactPatchCache
- Default: feature OFF
- Keep CollisionResult semantics unchanged
- Soft contacts stay on legacy path initially
- Allow short TTL output when raw contacts drop out

## Open Questions

- TTL value for inactive pairs
- Uniqueness thresholds and normal-angle cutoff
- Whether to disable backend contact history when cache is enabled
- Final data layout (AoS vs SoA)
