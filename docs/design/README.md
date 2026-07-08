# DART Design Docs

This directory holds durable DART 6.20 technical design proposals and decision
rationale. Use it for architecture, API shape, compatibility constraints,
tradeoffs, and design rules that should outlive a particular roadmap sequence.

Design docs may be revised as evidence changes, but they should not own active
priority, horizon, next step, gate, or implementation handoff state. Keep those
fields in `docs/plans/dashboard.md` or `docs/dev_tasks/<task>/`.

For cross-bucket placement decisions, use
[`../information-architecture.md`](../information-architecture.md).

## Files

| File | Purpose |
| --- | --- |
| [`dart6_ai_infra.md`](dart6_ai_infra.md) | Release-branch decision for adopting DART 7-inspired AI documentation buckets while preserving DART 6 compatibility constraints. |

## Placement Rules

- Put roadmap priority, status, horizon, next step, and gate in
  `docs/plans/dashboard.md`.
- Put active multi-session implementation state in `docs/dev_tasks/<task>/`.
- Put landed contributor and maintainer guidance in `docs/onboarding/`.
- Put theory and source-reference material in `docs/background/`.
- Put user-facing instructions in `docs/readthedocs/` or `README.md`.

When a design becomes part of active roadmap work, keep this directory as the
rationale owner and link it from the relevant plan or dev-task folder.
