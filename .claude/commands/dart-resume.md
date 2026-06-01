---
description: continue work from a previous session
agent: build
---

Resume unfinished work: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/dev_tasks/README.md
@docs/ai/sessions.md
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

Infer the task from branch name, commits, diffs, issue/PR description, and any
`docs/dev_tasks/<task>/` state. If the goal is still unclear after recon, stop
and ask.

## Step 3: Continue

- Propose a 3-6 step plan before editing.
- Continue with minimal scope and preserve existing user changes.
- If the task is being completed, promote any durable dashboard, evidence
  matrix, API inventory, migration map, or long-lived decision out of
  `docs/dev_tasks/<task>/` into `docs/plans/` or `docs/onboarding/`, then
  remove the dev-task folder completely in the completing change.
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
