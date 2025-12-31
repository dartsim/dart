# Progress Tracker

## Stage Checklist

- [x] Stage 0: Recon and requirements capture
- [x] Stage 1: Plan docs
- [x] Stage 2: Skeleton implementation (feature gated, compiles)
- [x] Stage 3: Full behavior wiring (use persistent contacts)
- [ ] Stage 4: Tests and regression protection
- [ ] Stage 4.5: GUI demo
- [ ] Stage 5: Benchmarks and documentation polish

## Current Status

- Stage 3 wiring complete (persistent contacts used for constraints)
- Next: Stage 4 tests and regression coverage

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
