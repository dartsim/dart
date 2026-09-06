# docs/design/

Agent rules for durable DART design documents.

- Read `docs/README.md`, then `README.md` here, then the relevant plan,
  handbook, background, or module docs.
- Design docs own architecture, API shape, constraints, tradeoffs, and
  non-goals that outlive a roadmap sequence. Keep them current-state oriented;
  rejected alternatives that no longer inform a decision go to git history.
- Do not record priority, horizon, next step, gate, or implementation handoff
  state here; those belong to `docs/plans/dashboard.md` and
  `docs/dev_tasks/<task>/`. A short proposal/accepted/superseded status is
  fine; a phase tracker is not.
- Link to `docs/onboarding/api-boundaries.md` for public/internal API policy
  instead of restating it.
- List every direct `docs/design/*.md` file in `README.md` (checked by
  `pixi run check-docs-policy`).

Select gates with `docs/ai/verification.md`.
