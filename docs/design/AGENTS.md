# docs/design/

Agent rules for durable DART 6.20 design documents.

## Purpose

`docs/design/` owns technical rationale that should survive after a plan or
dev task completes. Design docs explain architecture, API shape,
compatibility constraints, tradeoffs, decisions, and non-goals.

They are not roadmap dashboards or implementation handoff logs.

## Rules

- Start with `docs/README.md`, `docs/information-architecture.md`, then this
  file.
- Keep design docs durable and current-state oriented. Use Git history for old
  alternatives that are no longer relevant.
- Do not put priority, horizon, next step, gate, or active implementation
  handoff state here. Use `docs/plans/dashboard.md` and
  `docs/dev_tasks/<task>/` for that state.
- Preserve the DART 6 compatibility envelope: public headers, installed
  package components, ABI-sensitive class layouts, and Gazebo/gz-physics
  behavior are release-branch constraints.
- Link every direct `docs/design/*.md` file from `docs/design/README.md`.

## Verification

Use `docs/ai/verification.md` to select gates. For design-only Markdown edits,
run `pixi run lint`.
