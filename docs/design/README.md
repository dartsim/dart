# DART Design Docs

This directory holds durable technical design proposals and decision rationale.
Use it for architecture, API shape, tradeoffs, constraints, and design rules
that should outlive a particular roadmap sequence.

Design docs may be revised as evidence changes, but they should not own active
priority, horizon, next step, gate, or implementation handoff state. Keep those
fields in `docs/plans/dashboard.md` or `docs/dev_tasks/<task>/`.

## Files

| File                                                                   | Purpose                                                                       |
| ---------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| [`algorithm_extension_contracts.md`](algorithm_extension_contracts.md) | Durable rules for research-facing algorithm extension and baseline contracts  |
| [`hierarchical_allocator.md`](hierarchical_allocator.md)               | World-level memory management proposal and allocator ownership rationale      |
| [`scalable_compute_decisions.md`](scalable_compute_decisions.md)       | Workload-first CPU, SIMD, and GPU decision framework for scalable computation |

## Placement Rules

- Put roadmap priority, status, horizon, next step, and gate in
  `docs/plans/dashboard.md`.
- Put active multi-session implementation state in `docs/dev_tasks/<task>/`.
- Put landed developer explanations in `docs/onboarding/`.
- Put user-facing instructions in `docs/readthedocs/` or `README.md`.
- Link to `docs/onboarding/api-boundaries.md` for public/internal API policy
  instead of duplicating it.

When a design becomes part of active roadmap work, keep the design doc as the
rationale owner and link it from the relevant plan.
