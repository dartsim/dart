# Session State

Use `docs/dev_tasks/<task>/` for multi-session implementation work.

Each active task should keep:

- `README.md` for scope, evidence, decisions, and remaining work.
- `RESUME.md` when another session needs exact next steps.
- `HANDOFF.md` only when the task is being transferred across agents or
  worktrees.

Keep these notes current with branch name, PR number, exact validation commands,
known blockers, and next action. Do not treat a dev-task note as durable
documentation after the work completes; promote durable content first.

## Autonomous Project Sessions

`dart-ultrawork` is the DART 6 entrypoint for large, multi-session,
team-scale, or explicitly autonomous release-branch work. It still uses
`docs/dev_tasks/<task>/` as the project home; do not create a parallel generic
project directory.

For autonomous projects, add sidecars when they make resumability or evidence
clearer:

- `decisions.md`: dated decisions, alternatives, evidence, tradeoffs, and
  revisit triggers.
- `verification.md`: chunk or milestone, checks run, results, evidence, known
  gaps, and follow-up.
- `progress-log.md`: chronological record of meaningful completed work.

At session start, read the task `README.md`, `RESUME.md`, and any sidecars,
then verify current reality with `git status --short --branch` plus branch, PR,
or plan evidence named by the docs. If the docs are stale, add a
current-reality note before acting on them.

At session end, update status, next action, blockers, risks, verification
results, and handoff notes so a fresh session can continue without hidden chat
history. Do not mark an autonomous project complete while acceptance criteria,
required evidence, current docs, or dev-task cleanup are still missing.
