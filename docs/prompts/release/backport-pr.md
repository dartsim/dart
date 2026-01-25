# DART: Release Branch Backport

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## When to Use

Use this template to **port a fix from main to a release branch**. This is the opposite of [merge-into-main.md](merge-into-main.md).

| Scenario                                | Use This? | Alternative                              |
| --------------------------------------- | --------- | ---------------------------------------- |
| Port a bug fix from main to release-X.Y | ✅ Yes    | —                                        |
| Sync main with release after a release  | ❌ No     | [merge-into-main.md](merge-into-main.md) |
| Fix CI on release branch                | ❌ No     | [branch-ci-fix.md](branch-ci-fix.md)     |

## Key Principle

**Bug fixes require PRs to BOTH branches** (see [CONTRIBUTING.md](../../../CONTRIBUTING.md)):

1. First: Fix on main (primary development branch)
2. Then: Backport to release-X.Y using this template

## Prompt

````text
# DART: Release Branch Backport

Port fix from main to <RELEASE_BRANCH>.

## Context
- Release branch: <RELEASE_BRANCH>
- Release version: <RELEASE_VERSION>
- Source PR or commits: <SOURCE_PR_OR_COMMITS>
- Source branch (if available): <SOURCE_BRANCH or origin/main>
- Target milestone: <MILESTONE_NAME or "none">
- Notes: <optional>

## Workflow

### 1. Verify fix exists on main
```bash
gh pr view <SOURCE_PR> --json state,mergedAt
```

### 2. Check if already backported

```bash
git cherry -v origin/<RELEASE_BRANCH> origin/main | grep <COMMIT_HASH>
# Empty output = not backported yet
```

### 3. Create backport branch

```bash
git fetch origin <RELEASE_BRANCH>
git checkout -B backport/<SOURCE_PR>-to-<RELEASE_BRANCH> origin/<RELEASE_BRANCH>
```

### 4. Cherry-pick with -x (records source)

```bash
git cherry-pick -x <COMMIT_HASH>
# For merge commits: git cherry-pick -x -m 1 <MERGE_COMMIT_HASH>
```

### 5. Resolve conflicts (if any)

- Keep changes minimal; avoid refactors
- If conflicts are large, stop and ask before proceeding

### 6. Run minimal tests

```bash
pixi run build && pixi run test-unit
```

### 7. Push and create PR

```bash
git push -u origin HEAD
gh pr create --base <RELEASE_BRANCH> --title "Backport: <ORIGINAL_TITLE> (<RELEASE_BRANCH>)"
```

### 8. Set milestone (if applicable)

```bash
gh pr edit <PR_NUMBER> --milestone "<MILESTONE_NAME>"
```

## Output

- PR URL and CI status
- Short summary of changes and any conflicts resolved
- Link to original PR on main

```

```
````
