# docs/plans/

Agent rules for DART's living plan documents.

## Purpose

`docs/plans/` connects the AI-native north star to executable roadmap choices.
These files are living planning docs, not archival status dumps and not
implementation trackers.

## Rules

- Start with `docs/ai/north-star.md`, then `docs/plans/README.md`.
- Keep plans concise and evidence-based; link deeper design to developer docs
  or `docs/dev_tasks/`.
- Keep plans lifecycle-oriented: they should name current open gaps,
  sequencing, gates, and acceptance criteria, then be removed, consolidated, or
  retargeted after durable output lands elsewhere.
- Put durable architecture/API tradeoffs in `docs/design/`, landed developer
  knowledge in `docs/onboarding/`, user instructions in `docs/readthedocs/` or
  `README.md`, and active multi-session handoff state in `docs/dev_tasks/`.
- Prefer revising existing initiatives over adding duplicate plan entries.
- Use stable initiative IDs when renaming, reprioritizing, splitting, or
  consolidating work.
- Keep `dashboard.md` as the single source of truth for the priority, status,
  horizon, north-star dimension, next step, and gate of operating
  (non-`Complete`) plans. Do not repeat those fields in detailed plan files.
- Keep dashboard entries git-history friendly: priority comes from document
  order, and frequently changed fields each get their own line.
- Keep each dashboard `### PLAN-` entry at or under 40 lines and its
  `- Next step:` field at or under 15 lines (enforced by
  `pixi run check-docs-policy`). Relocate overflow history to the owner plan
  file's `## Progress log` section, leaving only the current next action plus a
  `History:` pointer.
- Keep `docs/plans/` flat unless one initiative needs multiple durable planning
  artifacts. Prefer a stable `.md` owner file with sidecar artifacts before
  moving the owner file into a subdirectory. Do not split plans by status,
  horizon, or north-star dimension.
- Do not use plan docs as a running changelog. Git history records past plan
  shapes; the files should describe the current intended path.
- When a plan completes, move its entry from `dashboard.md` to
  [`archive.md`](archive.md) in the same PR, converting it to the
  `**Final status:** Complete` shape. Archived entries point to durable owner
  docs outside numbered plan files. If a numbered plan file no longer guides
  current work, remove it or fold its durable content into the appropriate
  owner doc.

## Verification

Use `docs/ai/verification.md` to select gates for plan edits. Use
`docs/ai/components.md` for AI command and generated-adapter check coverage.
