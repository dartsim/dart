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

### Step 1: Recon (no changes)

```bash
git rev-parse --show-toplevel
git status -sb && git branch -vv && git log -10 --oneline --decorate
git diff --stat && git stash list
gh pr list --head "$(git branch --show-current)"
gh pr status
```

### Step 2: Reconstruct

Infer the task from branch name, commits, diffs, issue/PR description, and any
`docs/dev_tasks/<task>/` state. If the goal is still unclear after recon, stop
and ask.

### Step 3: Continue

- Propose a 3-6 step plan before editing.
- Continue with minimal scope and preserve existing user changes.
- For active solver/paper implementations, keep the plan or dev-task resume
  surface explicit about the completed slice, the next missing paper-parity
  gap, and why focused green tests are not a full paper-completion claim.
- If the task is being completed, run a completion audit before finalizing:
  identify the exact `docs/dev_tasks/<task>/` folder, inspect it for remaining
  plans/evidence/decisions, promote any durable dashboard, evidence matrix, API
  inventory, migration map, long-lived decision, or deferred-but-real work into
  release/onboarding/AI docs, update any durable status surface when the task
  changes roadmap state, then remove the dev-task folder
  completely in the completing change.
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
  `pixi run -e gazebo test-gz` when package or downstream Gazebo/gz-physics
  compatibility could be affected.
- For model/scene, simulation, dynamics, collision/contact, OSG, or visual
  behavior, route through `dart-verify-sim`: establish a text correctness
  oracle first, then capture assessed claim-tied images/debug layers or record
  a DISPLAY/Xvfb unavailable/not-applicable reason. A screenshot alone is not
  proof.
- Push with the same local and remote topic-branch name only after explicit
  maintainer/user approval:
  `branch=$(git branch --show-current); git push -u origin "HEAD:${branch}"`.
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

## Output

- Reconstructed task and current branch state
- Plan followed and files changed
- Verification commands run and their results
- Remaining work and anything held for explicit maintainer/user approval before push/PR update
