# Compute Resource Access — Dev Task

## Current Status

- [ ] Phase 1: Define resource access metadata for compute nodes
- [ ] Phase 2: Add debug validation for obvious read/write hazards
- [ ] Phase 3: Extend DOT export with resource access labels or data edges
- [ ] Phase 4: Add focused tests for sharing, conflicts, reductions, and
      stage propagation

## Goal

Add a descriptive resource-access layer to the experimental compute graph so
developers can inspect which data each compute node reads, writes, mutates, or
reduces. The first milestone should improve validation, visualization, and
future optimization inputs without replacing explicit graph dependencies.

## Non-Goals

- No replacement of explicit node dependencies as the correctness source of
  truth.
- No full automatic dependency inference in the first resource-access PR.
- No stable public resource registry.
- No GPU residency, transfer, stream, or device-memory API.
- No collision, contact, or constraint solver implementation in this PR.

## Key Decisions

- Treat resource access as metadata first. Use it for diagnostics, validation,
  DOT output, and profiling context before making it drive scheduling.
- Keep identifiers simple in the first pass. Stable resource names are enough
  to validate the model; typed ECS/component IDs can follow if the string model
  proves too weak.
- Keep explicit dependencies available for semantic ordering that cannot be
  inferred from data hazards, such as deterministic reductions, solver
  iteration order, frame hierarchy order, and side effects.
- Model reductions explicitly instead of treating all multi-writer patterns as
  races.

## Immediate Next Steps

1. Add `ComputeAccessMode` and a small resource-access value type in
   `dart/simulation/experimental/compute/`.
2. Attach optional resource accesses to `ComputeNode`, and make stage builders
   pass useful defaults for kinematics and rigid-body integration.
3. Add validation helpers that report obvious read/write and write/write
   hazards without mutating graph dependencies.
4. Extend DOT output and tests so resource access is visible alongside domains,
   acceleration tags, and profile data.
