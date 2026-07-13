---
name: dart-resume
description: "DART Resume: continue work from a previous session"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-resume.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-resume

Use this skill in Codex to run the DART `dart-resume` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/dart-resume <arguments>`
- Codex: `$dart-resume <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

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
@docs/onboarding/changelog.md

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
- whether the resumed change may need a changelog entry and how the
  `dart-changelog` decision will be recorded;
- which verification commands prove the result.

### Continue

- Propose a 3-6 step plan before editing.
- Continue with minimal scope and preserve existing user changes.
- Verification comes first when the route is uncertain. Define the observable
  signal, build or select the smallest reproducer/check, run a control vs
  candidate comparison when possible, and only then choose the implementation
  path. If the existing verification is weak, improve the check or debugging
  surface before trusting the result.
- For model/scene, simulation, dynamics, collision/contact, GUI, or visual
  behavior, route through `dart-verify-sim`: establish a text correctness
  oracle first, then capture assessed claim-tied images/debug layers or record
  an unavailable/not-applicable reason. Verify the expected pass path and a
  failure-sensitive condition when practical; a screenshot alone is not proof.
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
- Before finalizing a resumed task that changes behavior, public API, packaging,
  CI, docs workflow, AI-infra workflow, release process, or user-visible docs,
  invoke the `dart-changelog` routine in `decide` or `finalize` mode. If no
  entry is needed, keep the reason in the local PR body/checklist draft or task
  evidence; if an entry is needed, prepare the local `CHANGELOG.md` edit and PR
  link when available. Pushing the changelog edit or updating the PR body still
  requires explicit maintainer/user approval.
- If remaining work is real but blocked by a substantial design decision,
  maintainer direction, external dependency, or scope boundary that should not
  be resolved in the current session, ask the human before retiring the folder
  unless prior maintainer direction is already recorded. Record the parked or
  blocked work in the durable owner doc before deletion.
- Do not call a dev task complete while `docs/dev_tasks/<task>/` still exists.
  If implementation is done but the folder remains, the remaining work is the
  durable-doc promotion plus folder cleanup.
- Run `pixi run lint` before committing. Run relevant tests; use
  `pixi run test-all` before done when feasible, and also
  `pixi run -e cuda test-all` on Linux hosts with a visible NVIDIA CUDA runtime.
- Merge the latest base branch into any published PR branch before pushing, and
  follow the base-merge rules in `docs/onboarding/ai-tools.md`. Push with
  `git push -u origin HEAD` and create or update the PR only after explicit
  maintainer/user approval, preferring additive follow-up commits.

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
