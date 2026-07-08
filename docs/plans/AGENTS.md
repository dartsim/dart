# docs/plans/

Agent rules for DART 6.20 living plan documents.

## Purpose

`docs/plans/` connects the DART 6.20 north star to executable roadmap choices.
These files are living planning docs, not archival status dumps and not
implementation trackers.

## Rules

- Start with `docs/ai/north-star.md`, then `docs/plans/README.md`.
- Keep plans concise and evidence-based; link deeper design rationale to
  `docs/design/` and active implementation detail to `docs/dev_tasks/`.
- Keep `dashboard.md` as the single source of truth for priority, status,
  horizon, dimension, next step, and gate of operating plans.
- Prefer linking existing active dev-task owners over duplicating their packet
  boards in `docs/plans/`.
- Put durable architecture/API tradeoffs in `docs/design/`, landed maintainer
  guidance in `docs/onboarding/`, theory/reference material in
  `docs/background/`, user instructions in `docs/readthedocs/`, and active
  handoff state in `docs/dev_tasks/`.
- Do not use DART 7 clean-break plan language as a release-branch decision.
  Every DART 6 plan needs compatibility evidence for the release branch.

## Verification

Use `docs/ai/verification.md` to select gates. For plan-only Markdown edits,
run `pixi run lint`.
