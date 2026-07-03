---
description: continue work from a previous session
argument-hint: "[branch-or-task]"
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-resume.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Resume unfinished work: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/dev_tasks/README.md
@docs/ai/sessions.md
@docs/ai/verification.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md

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

Infer the task from branch name, commits, diffs, issue/PR description, and any
`docs/dev_tasks/<task>/` state. If the goal is still unclear after recon, stop
and ask.

### Continue

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
