---
description: continue work from a previous session
argument-hint: "[branch-or-task]"
agent: build
---

Resume unfinished work: $ARGUMENTS

## Objective

`dart-resume` is a completion-oriented task manager, not a status lookup or a
single-slice helper by default. Resume the named or reconstructed work, build or
refresh the execution plan, track progress in the repo-owned task surface, split
independent work into verifiable packets, use subagents/sub-sessions only when
the user explicitly requested delegation and the current AI surface permits it,
verify every result, and keep going until the whole task is complete or a real
blocker or approval boundary remains.

Decisions must be evidence-based. Before choosing between meaningful options,
first improve or define the verification/debugging method so it can catch false
positives and false negatives. Use repository inspection, focused tests,
benchmarks, A/B comparisons, GUI or visual evidence, logs, and external resource
searches as needed to decide from evidence instead of preference.

For a `docs/dev_tasks/<task>` target, full completion means all feasible task
work is finished, durable decisions and deferred work are promoted, and the
temporary dev-task folder is removed in the completing change. Do not stop after
one successful slice unless the user explicitly requested a limited mode.

## Argument Handling

Use `$ARGUMENTS` to identify the target, explicit scope limits, and execution
modifiers. Interpret arguments in this order:

1. **Explicit scope-limited mode**: if arguments include `status`,
   `audit-only`, `plan-only`, `slice`, or `next-slice`, honor that limit and
   state it in the plan. Without one of these limiters, do not downscope the
   session to a single slice.
2. **Execution modifiers**: if arguments include `no-subagents`, run serially
   but keep the same whole-task completion objective. If arguments explicitly
   request `subagents`, `parallel`, `delegation`, or `workers`, use available
   AI-native delegation when the current surface permits it. Tool-use modifiers
   do not limit scope unless paired with an explicit scope-limited mode.
3. **Explicit target path**: if an argument names `docs/dev_tasks/<task>` or a
   file under that folder, treat that dev task as the target and read its
   current `README.md` and `RESUME.md` snapshots after recon, following the
   selective-reading protocol in `docs/dev_tasks/README.md`. Resolve relative paths from the repo
   root discovered by `git rev-parse --show-toplevel`, not from the launch
   directory.
4. **Closeout wording**: words such as `complete`, `finish`, `retire`,
   `close out`, or `cleanup` are accepted but not required for dev-task
   completion. Treat `retire` / `close out` as emphasis that the likely end
   state is durable-doc promotion plus folder removal.
5. **Branch/PR/issue hint**: otherwise, treat arguments as a branch, PR, issue,
   topic, or free-form resume hint and reconstruct the task from live repo state.

Examples:

```text
$dart-resume docs/dev_tasks/rigid_ipc_solver
$dart-resume docs/dev_tasks/rigid_ipc_solver --slice
$dart-resume PR 2991
```

## Required Reading

@AGENTS.md
@docs/dev_tasks/README.md
@docs/ai/verification.md

Load `docs/onboarding/ci-cd.md` when CI is part of the current action,
`docs/onboarding/contributing.md` before branch/PR work, and
`docs/onboarding/changelog.md` for the closeout changelog decision.

## Workflow

### Recon (no changes)

```bash
git rev-parse --show-toplevel
git status -sb && git branch -vv && git log -10 --oneline --decorate
git diff --stat && git stash list
gh pr list --head "$(git branch --show-current)"
gh pr status
```

### Reconstruct

If `$ARGUMENTS` names a `docs/dev_tasks/<task>` path, use that folder as the
target and inspect its current `README.md` and `RESUME.md` snapshots before
choosing work. Follow current owner pointers; retrieve historical entries only
for relevant uncertainty, per `docs/dev_tasks/README.md`. If the folder is already absent, verify whether
it was retired on current `main`, identify the durable owner doc that replaced
it, and report that no folder cleanup remains.

Otherwise infer the task from branch name, commits, diffs, issue/PR
description, and any `docs/dev_tasks/<task>/` state. If the goal is still
unclear after recon, stop and ask.

Before editing, write explicit pass/fail criteria for the resumed task:

- what exact dev-task folder, branch, PR, issue, or topic is being completed;
- which remaining checklist items, decisions, or evidence must be resolved;
- how progress will be tracked during this session and across any handoff;
- which independent subtasks, if any, can be delegated when explicitly
  authorized or run as separate serial sessions without creating conflicting
  edits;
- which verification/debugging method will make false positives and false
  negatives unlikely before implementation decisions are made;
- which A/B tests, benchmarks, resource searches, GUI checks, or textual checks
  are needed to support consequential decisions;
- which durable docs will own any surviving decision or deferred work;
- whether the resumed change may need a changelog entry and how the
  `dart-changelog` decision will be recorded;
- which verification commands prove the result.

### Continue

Continue the selected action within the reconstructed scope and supplied
decisions. Use the task lifecycle in `docs/dev_tasks/README.md`, delegation
contract in `docs/ai/orchestration.md`, and claim-specific gates/completion
audit in `docs/ai/verification.md`. For 3D claims, use `dart-verify-sim`;
retain the full solver/paper target across checkpoints.

Before closeout, run `dart-changelog` in `decide` or `finalize` mode.
Promote durable dashboards/matrices and remaining-work decisions, then remove
the completed task folder under its owner's retirement rules. If a real blocker
remains, preserve exact current state and next action instead of claiming done.

For publication, load `docs/onboarding/ai-reviews.md`: merge the latest base
before a PR push and reuse explicit authorization only within its recorded
scope. Run `pixi run lint` before committing and the applicable pre-PR gates.

## Safety

No destructive git commands (`reset --hard`, dropping stashes, deleting
branches) without explicit maintainer/user approval.

## Output

- Reconstructed task and current branch/PR state
- Plan followed and files changed
- Verification commands run and their results
- Completion-audit result: dev-task promotion/cleanup done or the remaining
  blocker
- PR readiness, noting any external mutation that was explicitly approved
