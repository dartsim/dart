# DART: Release Branch CI Fix

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## When to Use

Use this template when **CI is failing on the release branch** and you need to fix it.

| Scenario                         | Use This? | Alternative                      |
| -------------------------------- | --------- | -------------------------------- |
| CI failing on release-X.Y branch | ✅ Yes    | —                                |
| CI failing on main branch        | ❌ No     | Use `/dart-fix-ci` command       |
| Port a fix from main to release  | ❌ No     | [backport-pr.md](backport-pr.md) |

## Key Principle

**Prefer fixes already in main** to reduce future merge conflicts. Check if the fix exists on main before creating a new one.

## Prompt

````text
# DART: Release Branch CI Fix

Fix CI failure on <RELEASE_BRANCH>.

## Context
- Release branch: <RELEASE_BRANCH>
- Release version: <RELEASE_VERSION>
- Failing run: <RUN_URL_OR_ID>
- Existing PR: <PR_URL or "create new">
- Milestone: <MILESTONE_NAME or blank>

## Workflow

### 1. Inspect the failure
```bash
gh run view <RUN_ID> --log-failed
# Or for specific job:
gh run view <RUN_ID> --job <JOB_ID> --log
````

### 2. Check if fix exists on main

```bash
git log origin/main --oneline --grep="<ERROR_KEYWORD>" -10
```

### 3. Create or continue on branch

If existing PR:

```bash
git fetch origin && git checkout <EXISTING_BRANCH>
```

If creating new:

```bash
git fetch origin <RELEASE_BRANCH>
git checkout -B fix/<ISSUE>-<RELEASE_BRANCH> origin/<RELEASE_BRANCH>
```

### 4. Apply minimal fix

- Prefer cherry-picking from main if fix exists there
- If new fix needed, keep it minimal

### 5. Analyze root cause

- Why wasn't this caught earlier?
- Should workflows be updated to catch similar issues?

### 6. Test locally

```bash
pixi run build && pixi run test-unit
```

### 7. Push and create/update PR

```bash
git push -u origin HEAD
# If new PR:
gh pr create --base <RELEASE_BRANCH> --title "ci: fix <ISSUE> on <RELEASE_BRANCH>"
```

### 8. Monitor CI

```bash
gh run watch <RUN_ID> --interval 30
```

## Output

- Root cause and why not caught earlier
- Fix summary and CI status
- PR URL (if created)
- Recommendations for preventing similar issues

```

```
