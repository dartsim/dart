# docs/design/

Agent rules for durable DART design documents.

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
- Preserve the compatibility envelope in `AGENTS.md`: public headers,
  installed package components, and the Gazebo-facing C++ API and behavior.
  ABI-sensitive class layouts become release-branch constraints once a
  release line is first packaged.
- Link every direct `docs/design/*.md` file from `docs/design/README.md`.

## Verification

Use `docs/ai/verification.md` to select gates. For design-only Markdown edits,
run `pixi run lint`.
