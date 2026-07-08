---
type: ai-session-policy
owner: self
---

# AI Sessions

Multi-session AI work uses the existing `docs/dev_tasks/` lifecycle. Do not add
a separate permanent session system until repeated failures justify it.

## When To Create A Dev Task

Use `docs/dev_tasks/<task>/` when the work is multi-phase, crosses sessions,
needs design decisions, or has dependencies between steps. Follow
`docs/dev_tasks/README.md`.

## Required Files

| File        | Purpose                                                       |
| ----------- | ------------------------------------------------------------- |
| `README.md` | Current status, goal, key decisions, and immediate next steps |
| `RESUME.md` | Handoff prompt for a fresh AI or human session                |

A separate `TODO.md` is optional and should only be introduced with a checker or
clear cleanup rule. Otherwise, status belongs in `README.md`.

## Autonomous Project Sessions

`dart-new-team-task` is the DART entrypoint for large, multi-session, team-scale,
or explicitly autonomous work. It still uses `docs/dev_tasks/<task>/` as the
project home for both DART 7 and DART 6 maintenance work; do not create a
parallel project-home tree unless a task-specific owner doc requires it.

For autonomous projects, the dev-task folder may add these sidecars:

| File              | Purpose                                                                  |
| ----------------- | ------------------------------------------------------------------------ |
| `decisions.md`    | Dated decisions, alternatives considered, evidence, tradeoffs, revisit   |
| `verification.md` | Checks, results, evidence, known gaps, and follow-up per chunk/milestone |
| `progress-log.md` | Chronological record of meaningful work completed                        |

Keep `README.md` as the combined overview and plan: north star, final
deliverable, acceptance criteria, scope, non-goals, constraints, risks, current
milestone, blockers, next actions, and gates. Keep `RESUME.md` as the handoff
surface: current branch/worktree state, immediate next step, risks, recovery
notes, and commands to verify reality.

### Session Start

At the start of every autonomous or multi-session continuation:

1. Locate `docs/dev_tasks/<task>/` from the prompt, branch, active plan, or
   current worktree. If none exists and the work needs one, create it before
   implementation.
2. Read `README.md`, `RESUME.md`, and the autonomous sidecars that exist.
3. Inspect current repository state with `git status --short --branch` and any
   branch, PR, or plan evidence named by the docs.
4. If the docs are stale, add a current-reality note before acting on them.
5. Identify the current milestone, next action, blockers, acceptance evidence,
   and escalation risks.

If existing relevant work lives outside the project home, first absorb or
summarize it into the dev-task docs before extending it.

### Session End

Before pausing, handing off, or claiming a chunk complete:

1. Update `README.md` with current status, blockers, risks, and next actions.
2. Update `RESUME.md` with the exact next step, branch/worktree state, and
   recovery commands for a fresh session.
3. Update `decisions.md` if an assumption or decision changed.
4. Update `verification.md` for checks run, results, evidence, known gaps, and
   follow-up.
5. Update `progress-log.md` for meaningful completed work when that sidecar is
   present.

Do not mark an autonomous project complete while acceptance criteria, required
evidence, current docs, or dev-task cleanup are still missing.

## Shared Checkouts

When more than one agent or human session works in the same clone, branch
state can change between commands: a parallel session may move the checkout to
a different branch or retire local branches mid-flow. Re-verify
`git branch --show-current` and the expected HEAD before creating a topic
branch, committing, or any other step that depends on branch state, and
confirm again after any pause or handoff. Prefer a separate `git worktree` per concurrent lane when
sessions run simultaneously.

## Resume Discipline

Update `RESUME.md` after significant progress and before ending a session. It
must contain:

- last session summary;
- current branch and worktree state;
- one immediate next step;
- context that would otherwise be lost;
- commands to verify the state.

## Completion Discipline

When a dev task completes:

1. Move durable lessons to the owner selected by
   `docs/information-architecture.md`.
2. Move any dashboard, evidence matrix, API inventory, migration map, or other
   artifact that must survive task cleanup to the matching durable bucket, such
   as `docs/plans/`, `docs/design/`, `docs/onboarding/`, or
   `docs/readthedocs/`.
3. Replace dev-task links from durable docs with persistent links, or delete
   the temporary references.
4. Delete the task folder in the same PR.
5. Run the verification gates listed in `docs/ai/verification.md`.
6. Include the cleanup in the completion PR.

A dev task is not complete while its `docs/dev_tasks/<task>/` folder remains in
the worktree. If the implementation is done but the folder still exists, treat
folder deletion plus durable-doc promotion as remaining work, not as optional
post-completion cleanup.

Never treat a dashboard or matrix under `docs/dev_tasks/` as durable. If users
or future agents will need it after the implementation lands, promote it before
the dev-task folder is removed.
