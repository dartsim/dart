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

1. Move durable lessons to the relevant developer doc.
2. Move any dashboard, evidence matrix, API inventory, migration map, or other
   artifact that must survive task cleanup to `docs/plans/` or
   `docs/onboarding/`.
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
