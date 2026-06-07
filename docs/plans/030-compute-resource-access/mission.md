# Compute Resource Access Mission

This mission is the objective-specific evidence contract for the PLAN-030
resource-access milestone. The dashboard remains the operating-state owner; this
bundle defines what a resource-access implementation must prove.

## Objective

Land descriptive resource access metadata for the simulation compute graph so
developers can inspect which resources graph nodes read, write, mutate, reduce,
or use as scratch space without replacing explicit graph dependencies as the
correctness source of truth.

## Scope

- Add a small resource access mode/value model under
  `dart::simulation::compute`.
- Attach optional resource access metadata to compute nodes.
- Add conservative validation for obvious same-resource read/write,
  write/write, and mutation hazards.
- Keep reductions explicit and separate from ordinary multi-writer access.
- Surface resource access metadata in DOT output without adding GUI
  dependencies.
- Add focused unit coverage for sharing, conflicts, reductions, and stage
  propagation.

## Non-Goals

- No automatic dependency inference in this milestone.
- No stable public resource registry.
- No GPU residency, stream, transfer, or device-memory API.
- No dependency on the DART 6 `World` behavior; parity evidence belongs on
  `release-6.*` branches.
- No collision, contact, constraint, or rendering backend implementation.

## Expected Deliverable

A bounded local change that keeps the simulation compute executor boundary
backend-neutral, improves graph inspection and validation, and can be reported
against the evaluator contract in the completion report or active dev-task
handoff.

## Source Evidence

- `docs/plans/dashboard.md` owns PLAN-030 operating state.
- `docs/design/scalable_compute_decisions.md` owns durable scalable-compute
  rationale.
- `docs/plans/dashboard.md` (PLAN-030) and
  `docs/design/scalable_compute_decisions.md` own the durable scalable-compute
  state now that the consolidated dev-task folder has been retired;
  resource-access was Phase 1 of that plan.
- `tests/unit/simulation/compute/` and `tests/unit/simulation/world/` own
  focused regression coverage.
- `tests/benchmark/simulation/bm_compute_graph.cpp` owns
  compute-graph benchmark coverage.

## Handoff

Use [`evaluator.md`](evaluator.md) before claiming the milestone is complete.
If any required proof cannot run locally, record the blocker and the strongest
next-best evidence in the completion report.
