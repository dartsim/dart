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

- Stage 4.5 GUI demo complete
- Next: Stage 5 benchmarks and documentation polish

## Stage 5 Notes

- Benchmark coverage added in `tests/benchmark/collision/bm_boxes.cpp`
- Results not captured yet (pending local runs)

## Decisions

- Name: ContactPatchCache
- Default: feature OFF
- Keep CollisionResult semantics unchanged
- Soft contacts stay on legacy path initially

## Open Questions

- TTL value for inactive pairs
- Uniqueness thresholds and normal-angle cutoff
- Whether to disable backend contact history when cache is enabled
- Final data layout (AoS vs SoA)
