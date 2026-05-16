# docs/plans/

Agent rules for DART's living plan documents.

## Purpose

`docs/plans/` connects the AI-native north star to executable roadmap choices.
These files are living planning docs, not archival status dumps and not
implementation trackers.

## Rules

- Start with `docs/ai/north-star.md`, then `docs/plans/README.md`.
- Keep plans concise and evidence-based; link deeper design to onboarding docs
  or `docs/dev_tasks/`.
- Prefer revising existing initiatives over adding duplicate plan entries.
- Use stable initiative IDs when renaming, reprioritizing, splitting, or
  consolidating work.
- Keep `dashboard.md` as the single source of truth for priority, status,
  horizon, north-star dimension, next step, and gate. Do not repeat those fields
  in detailed plan files.
- Keep dashboard entries git-history friendly: priority comes from document
  order, and frequently changed fields each get their own line.
- Keep `docs/plans/` flat unless one initiative needs multiple durable planning
  artifacts. Prefer a stable `.md` owner file with sidecar artifacts before
  moving the owner file into a subdirectory. Do not split plans by status,
  horizon, or north-star dimension.
- Do not use plan docs as a running changelog. Git history records past plan
  shapes; the files should describe the current intended path.

## Verification

Use `docs/ai/verification.md` to select gates for plan edits. Use
`docs/ai/components.md` for AI command and generated-adapter check coverage.
