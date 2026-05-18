# docs/design/

Agent rules for durable DART design documents.

## Purpose

`docs/design/` owns technical rationale that should survive after a plan or
dev task completes. Design docs explain architecture, API shape, constraints,
tradeoffs, and non-goals. They are not roadmap dashboards or implementation
handoff logs.

## Rules

- Start with `docs/README.md`, then this file, then the relevant plan,
  onboarding, background, or module docs.
- Keep design docs durable and current-state oriented. Use Git history for old
  alternatives that are no longer relevant.
- Do not put priority, horizon, next step, gate, or active implementation
  handoff state here. Use `docs/plans/dashboard.md` and
  `docs/dev_tasks/<task>/` for that state.
- A design doc may have a short proposal/accepted/superseded status, but it
  should not become a phase tracker.
- Link to `docs/onboarding/api-boundaries.md` for public/internal API policy
  and apply that policy to the specific design.
- Keep each direct `docs/design/*.md` file listed in `docs/design/README.md`.

## Verification

Use `docs/ai/verification.md` to select gates for design-doc edits. The docs
policy check verifies that direct design docs stay indexed and avoid
dashboard-owned fields.
