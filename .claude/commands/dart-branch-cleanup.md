---
description: analyze or clean stale repository branches
argument-hint: "[analyze|clean]"
agent: build
---

Analyze or clean branches: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md

## Modes

- `analyze`: inspect only, no deletions
- `action`: prepare follow-up, or delete only after ownership/safety are clear
  and the maintainer/user gives explicit approval for each local or remote
  branch deletion

Default to `analyze` if the requested mode is ambiguous.

## Workflow

1. `git fetch --all --no-prune`
2. List stale remote-tracking refs without deleting them:
   ```bash
   git remote prune origin --dry-run
   ```
   If `origin` uses SSH and port 22 is unavailable, keep the check read-only
   and use a temporary HTTPS rewrite instead of changing repository config:
   ```bash
   git -c url.https://github.com/.insteadOf=git@github.com: \
     fetch --all --no-prune
   git -c url.https://github.com/.insteadOf=git@github.com: \
     remote prune origin --dry-run
   ```
   To confirm that a remote PR branch was deleted without updating refs, query
   GitHub directly over HTTPS:
   ```bash
   git ls-remote --heads https://github.com/dartsim/dart.git <BRANCH>
   ```
3. Determine the target/base branch from the PR or task, usually
   `origin/release-6.20` for this release lane.
4. For each branch:
   ```bash
   git rev-list --left-right --count <TARGET>...<BRANCH>
   git log --oneline <TARGET>..<BRANCH>
   git diff --stat <TARGET>..<BRANCH>
   git cherry -v <TARGET> <BRANCH>
   ```
5. Before any explicitly approved local branch deletion, check whether the
   branch is checked out by a linked worktree:
   ```bash
   git worktree list --porcelain
   git -C <WORKTREE> status --short --branch
   ```
   Dirty linked worktrees must not be removed, detached, or reset during branch
   cleanup. Only after explicit maintainer/user approval, switch that worktree
   to a preservation branch at the same commit to free the obsolete branch name
   before deleting the merged branch.
6. The command `git branch --merged` may not prove ancestry for PR branches
   that landed through a squash or merge commit. Use the merged PR state plus
   an empty tree diff or equivalent `git cherry -v` output as the deletion
   signal; if the history relationship is unclear, keep the branch.
7. Classify recommendations only; deleting a candidate requires explicit
   maintainer/user approval:
   - `ahead=0`: safe deletion candidate
   - equivalent commits already landed: deletion candidate
   - small, current, useful diff: keep or rebase into PR
   - large or unclear diff: document follow-up before action
8. Ask for explicit maintainer/user approval before pruning refs or deleting any
   local or remote branch, even when ownership, branch purpose, and remote
   impact look clear.

## Output

- Branch summary with ahead/behind count and last commit date
- Useful commits or risks
- Recommendation: delete, keep, rebase, or needs follow-up
- Actions taken after explicit maintainer/user approval, if action mode was
  explicitly requested
