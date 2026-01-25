# DART: Release Packaging

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## When to Use

Use this template to **create a new release version**. This bumps version numbers and prepares the CHANGELOG.

| Scenario                      | Use This? | Alternative                              |
| ----------------------------- | --------- | ---------------------------------------- |
| Creating new X.Y.Z release    | ✅ Yes    | —                                        |
| Syncing main after release    | ❌ No     | [merge-into-main.md](merge-into-main.md) |
| Porting fix to release branch | ❌ No     | [backport-pr.md](backport-pr.md)         |

## Prompt

````text
# DART: Release Packaging

Create release packaging PR for <NEW_VERSION>.

## Context
- New version: <NEW_VERSION> (e.g., 6.16.6)

## Auto-derive these values
```bash
# Previous version
prev_version=$(grep -oP '(?<=<version>)[^<]+' package.xml)

# Base branch (release-X.Y from NEW_VERSION)
base_branch="release-$(echo <NEW_VERSION> | cut -d. -f1-2)"

# Release date
release_date=$(date +%Y-%m-%d)

# Milestone
gh api repos/{owner}/{repo}/milestones --jq '.[] | select(.title | contains("<NEW_VERSION>"))'
````

## Workflow

### 1. Sync base branch

```bash
git fetch origin $base_branch
git checkout -B release/<NEW_VERSION>-version-bump origin/$base_branch
```

### 2. Bump version in package.xml

```xml
<version><NEW_VERSION></version>
```

### 3. Bump version in pixi.toml

```toml
version = "<NEW_VERSION>"
```

### 4. Update find_package in examples/tutorials

```bash
grep -r "find_package(DART" examples/ tutorials/ | grep -v ".pyc"
# Update version requirements if needed
```

### 5. Add CHANGELOG.md section

```markdown
### [DART <NEW_VERSION> (<RELEASE_DATE>)](https://github.com/dartsim/dart/milestone/XX?closed=1)

- Category
  - Description: [#PR](https://github.com/dartsim/dart/pull/PR)
```

### 6. Commit and push

```bash
git add -A
git commit -m "Packaging <NEW_VERSION>"
git push -u origin HEAD
```

### 7. Create PR

```bash
gh pr create --base $base_branch --title "Packaging <NEW_VERSION>" \
  --body "Version bump and CHANGELOG for <NEW_VERSION> release."
```

### 8. Set milestone

```bash
gh pr edit <PR_NUMBER> --milestone "DART <NEW_VERSION>"
```

## Output

- PR URL
- Confirmation of version bumps
- Ask if any derived info is ambiguous

```

```
