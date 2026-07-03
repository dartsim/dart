---
description: continue work from a previous session
agent: build
---

Resume unfinished work: $ARGUMENTS

## Argument Handling

Use `$ARGUMENTS` to avoid long free-form resume prompts. Interpret arguments in
this order:

1. **Explicit target path**: if an argument names `docs/dev_tasks/<task>` or a
   file under that folder, treat that dev task as the target and read its
   `README.md` and `RESUME.md` after recon. Resolve relative paths from the repo
   root discovered by `git rev-parse --show-toplevel`, not from the launch
   directory.
2. **Completion intent**: if arguments include words such as `complete`,
   `finish`, `retire`, `close out`, or `cleanup`, run the completion-audit path
   for the target dev task. Completion means durable work is promoted and the
   dev-task folder is removed in the same change; it does not mean "advance one
   more slice."
3. **Branch/PR/issue hint**: otherwise, treat arguments as a branch, PR, issue,
   topic, or free-form resume hint and reconstruct the task from live repo state.

Examples:

```text
$dart-resume docs/dev_tasks/skeleton_simulation_mode --complete
$dart-resume docs/dev_tasks/dart7_core_dynamics_perf_forwardport retire
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

For completion-intent arguments, write explicit pass/fail criteria before
editing:

- what exact dev-task folder is being completed;
- which remaining checklist items, decisions, or evidence must be resolved;
- which durable docs will own any surviving decision or deferred work;
- which verification commands prove the result.

## Step 3: Continue

- Propose a 3-6 step plan before editing.
- Continue with minimal scope and preserve existing user changes.
- For active solver/paper implementations, keep the plan or dev-task resume
  surface explicit about the completed slice, the next missing paper-parity
  gap, and why focused green tests are not a full paper-completion claim.
- If the task is being completed, run a completion audit before finalizing:
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
