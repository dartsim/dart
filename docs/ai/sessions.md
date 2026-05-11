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

1. Move durable lessons to the relevant onboarding doc.
2. Delete the task folder in the same PR.
3. Run the verification gates listed in `docs/ai/verification.md`.
4. Include the cleanup in the completion PR.
