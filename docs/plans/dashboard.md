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
- Status: Active
- Horizon: Next
- Dimension: AI-native execution
- Next step: Keep structural checks covering workflow adapters, required-reading
  paths, and dev-task shape.
- Gate: AI docs/adapters gates stay green after each workflow change.

### PLAN-010: Easy-Start API And Package Readiness

- Owner doc:
  [`010-easy-start-api-package-readiness.md`](010-easy-start-api-package-readiness.md)
- Status: Proposed
- Horizon: Now
- Dimension: Easy start
- Next step: Build the first-success workflow matrix.
- Gate: First-success commands are documented for Python, C++, packages, and
  Pixi source builds.

### PLAN-020: Algorithm Extension Contracts

- Owner doc:
  [`020-algorithm-extension-contracts.md`](020-algorithm-extension-contracts.md)
- Status: Proposed
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Choose the first algorithm family to formalize.
- Gate: The selected family has an inventoried extension surface, baseline, and
  benchmark command.

### PLAN-030: Compute Scalability Roadmap

- Owner doc:
  [`030-compute-scalability-roadmap.md`](030-compute-scalability-roadmap.md)
- Status: Active
- Horizon: Now
- Dimension: Scalable compute
- Next step: Land resource-access metadata for compute nodes before dependency
  inference, contact scheduling, or GPU/rendering backend work.
- Gate: Focused graph/world tests and the compute-graph benchmark stay green;
  Taskflow remains behind the experimental executor boundary; graph
  metadata/profiling/DOT output remain backend-neutral; classic World behavior
  stays untouched.

### PLAN-040: DART 7 Release Hardening

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Proposed
- Horizon: Next
- Dimension: Release transition
- Next step: Convert release roadmap into checkable gates.
- Gate: Release roadmap gates are mapped to local or CI evidence.

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
  [`../dev_tasks/filament_gui/08-north-star-migration.md`](../dev_tasks/filament_gui/08-north-star-migration.md)
- Status: Active
- Horizon: Next
- Dimension: Easy start
- Next step: Keep GUI API promotion aligned with Filament gates.
- Gate: GUI promotion stays aligned with the Filament dev-task migration plan.

### PLAN-070: DART 8 Compatibility Cleanup

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Proposed
- Horizon: Later
- Dimension: Release transition
- Next step: Maintain compatibility-debt inventory during DART 7.
- Gate: Compatibility cleanup stays tied to the DART 7/8 release roadmap.
