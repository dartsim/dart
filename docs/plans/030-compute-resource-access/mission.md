# Compute Resource Access Mission

This mission is the objective-specific evidence contract for the PLAN-030
resource-access milestone. The dashboard remains the operating-state owner; this
bundle defines what a resource-access implementation must prove.

## Objective

Land descriptive resource access metadata for the experimental compute graph so
developers can inspect which resources graph nodes read, write, mutate, reduce,
or use as scratch space without replacing explicit graph dependencies as the
correctness source of truth.

## Scope

- Add a small resource access mode/value model under
  `dart::simulation::experimental::compute`.
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
- No changes to classic `dart::simulation::World` behavior.
- No collision, contact, constraint, or rendering backend implementation.

## Expected Deliverable

A bounded local change that keeps the experimental compute executor boundary
backend-neutral, improves graph inspection and validation, and can be reported
against the evaluator contract in the completion report or active dev-task
handoff.

## Source Evidence

- `docs/plans/dashboard.md` owns PLAN-030 operating state.
- `docs/design/scalable_compute_decisions.md` owns durable scalable-compute
  rationale.
- `docs/dev_tasks/experimental_world_scalable_compute/` owns active multi-session
  task state; resource-access is Phase 1 of that consolidated plan.
- `tests/unit/simulation/experimental/compute/` and
  `tests/unit/simulation/experimental/world/` own focused regression coverage.
- `tests/benchmark/simulation/experimental/bm_compute_graph.cpp` owns
  compute-graph benchmark coverage.

## Handoff

Use [`evaluator.md`](evaluator.md) before claiming the milestone is complete.
If any required proof cannot run locally, record the blocker and the strongest
next-best evidence in the completion report.
