---
description: continue work from a previous session
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
   `README.md` and `RESUME.md` after recon. Resolve relative paths from the repo
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
$dart-resume docs/dev_tasks/skeleton_simulation_mode
$dart-resume docs/dev_tasks/dart7_core_dynamics_perf_forwardport
$dart-resume docs/dev_tasks/dart7_core_dynamics_perf_forwardport --slice
$dart-resume PR 2991
```

## Required Reading

@AGENTS.md
@docs/dev_tasks/README.md
@docs/ai/sessions.md
@docs/ai/verification.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md

## Step 1: Recon (no changes)

```bash
git rev-parse --show-toplevel
git status -sb && git branch -vv && git log -10 --oneline --decorate
git diff --stat && git stash list
gh pr list --head "$(git branch --show-current)"
gh pr status
```

## Step 2: Reconstruct

If `$ARGUMENTS` names a `docs/dev_tasks/<task>` path, use that folder as the
target and inspect its `README.md`, `RESUME.md`, and any directly referenced
owner docs before choosing work. If the folder is already absent, verify whether
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
- which verification commands prove the result.

## Step 3: Continue

- Propose a 3-6 step plan before editing.
- Continue with minimal scope and preserve existing user changes.
- Verification comes first when the route is uncertain. Define the observable
  signal, build or select the smallest reproducer/check, run a control vs
  candidate comparison when possible, and only then choose the implementation
  path. If the existing verification is weak, improve the check or debugging
  surface before trusting the result.
- For text-based behavior, prefer concrete command output, failing/passing
  tests, focused scripts, logs, structured packets, assertions, and diffable
  artifacts. For GUI or visual behavior, capture screenshots, rendered assets,
  pixel/visual comparisons, smoke runs, or videos where the surface supports
  them; verify both the expected pass path and at least one failure-sensitive
  condition when practical.
- When the decision depends on behavior outside the repository, search the
  relevant upstream docs, papers, issues, standards, or release notes and record
  the source-backed conclusion in the task docs, durable owner doc, or PR
  evidence. Prefer primary sources.
- If the task is large, decompose it into independently verifiable work items.
  Use available AI-native subagent, sub-session, or parallel-worker support only
  when the user explicitly requested delegation and the current surface permits
  it. Otherwise, keep the decomposition as a serial plan or explicit handoff
  packets. Validate and integrate every delegated result yourself; repo-tracked
  docs remain the source of truth even when agent-specific orchestration is
  available.
- Keep progress tracking current in the task's `README.md`, `RESUME.md`, or
  durable owner doc after meaningful progress. If the current session cannot
  finish because of context, environment, approval, or a real blocker, leave the
  next session with exact current reality, remaining work, blockers, and gates.
- For active solver/paper implementations, keep the plan or dev-task resume
  surface explicit about the completed slice, the next missing paper-parity
  gap, and why focused green tests are not a full paper-completion claim.
- Unless an explicit limited mode was requested, do not stop at "one more
  slice." Continue the plan/execute/verify/update loop until all feasible work
  is complete or a blocker or approval boundary remains.
- Run a completion audit before finalizing a dev-task target:
  identify the exact `docs/dev_tasks/<task>/` folder, inspect it for remaining
  plans/evidence/decisions, promote any durable dashboard, evidence matrix, API
  inventory, migration map, long-lived decision, or deferred-but-real work into
  `docs/plans/`, `docs/design/`, or `docs/onboarding/`, update dashboard/plan
  progress when the task changes roadmap state, then remove the dev-task folder
  completely in the completing change.
- In completion mode, do not leave the long-form resume prompt to carry
  decisions. Record the final decision or parked follow-up in the durable owner
  doc, then delete or update the dev-task folder according to
  `docs/dev_tasks/README.md`.
- If remaining work is real but blocked by a substantial design decision,
  maintainer direction, external dependency, or scope boundary that should not
  be resolved in the current session, ask the human before retiring the folder
  unless prior maintainer direction is already recorded. Record the parked or
  blocked work in the durable owner doc before deletion.
- Do not call a dev task complete while `docs/dev_tasks/<task>/` still exists.
  If implementation is done but the folder remains, the remaining work is the
  durable-doc promotion plus folder cleanup.
- Run `pixi run lint` before committing.
- Run relevant tests; use `pixi run test-all` before done when feasible, and
  also `pixi run -e cuda test-all` on Linux hosts with a visible NVIDIA CUDA
  runtime.
- Push with `git push -u origin HEAD` and create/update the PR only after
  explicit maintainer/user approval.
- For already-published PRs, prefer additive follow-up commits. Amend or
  force-push only after explicit maintainer/user approval and only when the user
  explicitly requests it or when there is a clear reason such as removing
  sensitive content or repairing broken branch history.
- If an already-published PR needs the latest target branch, use explicit
  maintainer/user approval to update that published branch by merging the
  target branch and pushing normally. Do not rebase published PR branches by
  default because that invalidates existing CI runs and makes PR review/comment
  history harder to follow. Rebase or force-push only when the maintainer
  explicitly requests it.

## Safety

No destructive git commands (`reset --hard`, dropping stashes, deleting branches)
without explicit maintainer/user approval.
