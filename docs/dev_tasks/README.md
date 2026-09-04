# Development Tasks

This file owns active-task tracking, session handoff, and task retirement.
Use `docs/dev_tasks/<task>/` for multi-phase or multi-session work, consequential
design decisions, or dependencies between steps. A small, linear task needs no
folder; documentation work follows the same criteria.

Task folders are temporary. Promote durable output to the owner selected by
[`docs/information-architecture.md`](../information-architecture.md) and delete
the folder in the completing change.

## Project Home

Keep one project home for a task. `dart-ultrawork` uses this same structure for
autonomous DART 7 and DART 6 work; do not add a parallel tracking tree.

| File                         | Contents                                                                                                                                     |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `README.md` (required)       | Goal, current status, scope/non-goals, decisions, acceptance evidence, gates, blockers, and next action                                      |
| `RESUME.md` (required)       | Current snapshot, branch/worktree state, next-action owner, context at risk, authorization/model constraints, and commands to verify reality |
| `decisions.md` (optional)    | Decision, evidence, tradeoff, date, and revisit trigger                                                                                      |
| `verification.md` (optional) | Change, checks, review passes, artifact links, results, limitations, and follow-up                                                           |
| `progress-log.md` (optional) | Chronological evidence when the current snapshot alone is insufficient                                                                       |

The README records the specification intake owned by
`docs/ai/orchestration.md`, or links to its plan packet. Include traceability to
the issue, plan, design, or user request. For autonomous work, also state the
north star, final deliverable, constraints, risks, and current milestone.
Add other files only for a distinct need. A separate `TODO.md` needs a checker
or explicit cleanup rule; otherwise status stays in the README.

When a dashboard entry owns the next action, its `- Next step:` field is the
single source of truth. README and RESUME point there instead of copying it.
Link to code and evidence instead of reproducing API inventories, mutable
versions, or full implementation history.

## Session Start

1. Locate the named task from the prompt, branch, plan, or current worktree.
   Inspect the current subdirectories rather than maintaining a task inventory.
2. Read the README and RESUME current snapshot before historical logs. For long
   handoffs, inspect headings and current-state sections first; load historical
   decisions or verification only for the active action or an unresolved fact.
3. Verify `git status --short --branch`, HEAD, and any relevant live PR or plan
   evidence. Handoff branch/status sections are snapshots, not checkout commands
   to execute blindly.
4. Reconcile stale instructions with current reality before acting. Keep a
   compact current-reality note above useful historical evidence.
5. Continue the active objective, milestone, and next action with its recorded
   constraints. Summarize relevant work found elsewhere into this project home
   before extending it.

## Session End

Update the current snapshot after meaningful progress and before pausing or
handing off. Preserve the user's objective, decisions, remaining work, blockers,
verification gaps, and exact recovery commands. Keep the immediate next action
ahead of history, and update optional sidecars only when their evidence changed.

Record the selected model/effort, current phase, and approved research, review,
and implementation delegation scopes from `docs/ai/orchestration.md`.
Carry authorization forward for its stated action, target, and scope; a handoff
does not expand it. A fresh session must be able to continue without hidden chat
history. For solver/paper work, retain the full completion target and next
missing parity gap from `docs/ai/verification.md`.

## Shared Checkouts

Concurrent sessions can change branch state between commands. Re-verify
`git branch --show-current` and expected HEAD before branching, committing, or
other branch-dependent actions, and after a pause or handoff. Prefer a separate
`git worktree` per concurrent lane. Preserve other sessions' edits and claims.

## Task Completion Checklist (MANDATORY)

Complete these steps in the same change that finishes the task:

1. Inspect remaining work against the requested outcome and the completion
   audit in `docs/ai/verification.md`, including required reviews and
   physics/GUI evidence. Checkpoint success does not complete a broader task.
2. Complete still-doable work. Move genuinely deferred or blocked work to its
   durable plan/design owner. Ask the human before retiring a folder with real
   unfinished work unless prior maintainer direction already covers it. An
   open-ended expansion needs an explicit scope cap or stopping condition.
3. Promote durable decisions, dashboards, evidence matrices, API inventories,
   and migration maps to existing owners. Replace durable links into the
   temporary folder with persistent links. Keep only conclusions that future
   work needs; avoid creating a handbook page for every completed task.
4. Delete the task folder in the completing change, not after merge. A task is
   not complete while its folder remains.
5. Run the applicable gates in `docs/ai/verification.md` and update owning
   plan/dashboard state. That file owns pre-commit and pre-PR requirements;
   the quick Git hook does not replace full lint.

## Related

- [Documentation placement](../information-architecture.md)
- [Agent policy and workflow index](../ai/README.md)
- [Developer handbook](../onboarding/README.md)
