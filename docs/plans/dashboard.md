# Plan Dashboard

This is the one-screen operating view for DART's living plans.

This file is the single source of truth for plan priority, status, horizon,
north-star dimension, next step, and gate. Each entry may link to a detailed
numbered initiative file or to the authoritative owner document for initiatives
that are tracked elsewhere until they warrant a dedicated plan file. Detailed
plan files own workstreams and acceptance criteria. External owner documents
own the referenced scope until a detailed initiative file is added.
`north-star-roadmap.md` owns strategic framing.

## Operating View

Priority order follows document order. Keep each frequently changed field on
its own line so status updates remain git-history friendly.

### PLAN-001: Living Plan System

- Owner doc: [`README.md#plan-update-workflow`](README.md#plan-update-workflow)
- Status: Complete
- Horizon: Later
- Dimension: AI-native execution
- Next step: Use the recorded structural checks whenever plan workflows,
  generated adapters, required-reading paths, or dev-task shape rules change.
- Gate: `check-ai-commands`, `check-docs-policy`, and `sync-ai-commands` cover
  adapter sync, required-reading coverage, and dev-task shape.

### PLAN-010: Easy-Start API And Package Readiness

- Owner doc: [`../../README.md#quick-start`](../../README.md#quick-start)
- Status: Complete
- Horizon: Later
- Dimension: Easy start
- Next step: During DART 7 package publication, rerun the artifact recheck and
  DART 7 README first-success verification against the published artifacts,
  then remove or demote the current-package fallback snippets.
- Gate: README first-success snippets pass for current published packages,
  source checkout, and local DART 7 package installs; DART 7 public package
  publication remains tracked by `check-dart7-artifacts`.

### PLAN-020: Algorithm Extension Contracts

- Owner doc:
  [`../design/algorithm_extension_contracts.md`](../design/algorithm_extension_contracts.md)
- Status: Complete
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Use the LCP v0 contract, tests, benchmark, and `lcp_physics`
  example as the template when the next algorithm family is selected.
- Gate: LCP contract docs, focused tests, smoke benchmark, API-boundary
  exclusions, and baseline example evidence are recorded.

### PLAN-030: Compute Scalability Roadmap

- Owner doc:
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
- Status: Active
- Horizon: Now
- Dimension: Scalable compute
- Next step: Land resource-access metadata for compute nodes before dependency
  inference, contact scheduling, or GPU/rendering backend work; use the
  resource-access evaluator mission to keep objective-specific proof explicit.
- Gate:
  [`030-compute-resource-access/evaluator.md`](030-compute-resource-access/evaluator.md)
  records the focused proof: graph/world tests and the compute-graph benchmark
  stay green; Taskflow remains behind the experimental executor boundary; graph
  metadata/profiling/DOT output remain backend-neutral; classic World behavior
  stays untouched.

### PLAN-035: Native Collision Feature Dashboard

- Owner doc:
  [`035-native-collision-dashboard.md`](035-native-collision-dashboard.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Keep native collision feature/use-case/benchmark coverage,
  runtime-isolation proof, and DART 7/DART 8 abstraction cleanup planning
  current through the completing local packet and final PR evidence transfer.
- Gate:
  [`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)
  has no active GAP/PARTIAL rows, runtime reference engines remain
  test/benchmark-only, final
  benchmark/CI evidence is attached to the completing review surface, and
  temporary dev-task folders are removed.

### PLAN-040: DART 7 Release Hardening

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Complete
- Horizon: Later
- Dimension: Release transition
- Next step: Use the DART 7 gate table and packaging checklist during release
  packaging or release-hardening passes.
- Gate: Release gate evidence names the relevant local or CI command, blocker,
  or external compatibility check for each touched release surface.

### PLAN-050: Experimental World Split

- Owner doc:
  [`../onboarding/python-bindings.md#experimental-world-bindings-and-transition`](../onboarding/python-bindings.md#experimental-world-bindings-and-transition)
- Status: Complete
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Track any future DART 8 promotion work under the release roadmap
  once the experimental world has parity gates.
- Gate: `dartpy.simulation_experimental` is separate from legacy
  `dartpy.simulation`, has focused import/API coverage, and the DART 7/8
  transition path is documented in onboarding docs.

### PLAN-060: Backend-Hidden GUI Roadmap

- Owner doc:
  [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md)
- Status: Complete
- Horizon: Later
- Dimension: Easy start
- Next step: Keep future public GUI API promotion aligned with the durable
  backend-hidden renderer guidance.
- Gate: GUI promotion stays backend-hidden and aligned with the maintained
  Filament renderer onboarding guidance.

### PLAN-070: DART 8 Compatibility Cleanup

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Complete
- Horizon: Later
- Dimension: Release transition
- Next step: Use the compatibility-debt inventory and cleanup review checklist
  during DART 7 packaging passes and DART 8 removal planning.
- Gate: DART 8 cleanup decisions cite migration notes, changelog entries,
  package/export status, and gz-physics compatibility where relevant.
