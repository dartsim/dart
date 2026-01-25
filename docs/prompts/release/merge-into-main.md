# DART: Merge Release Branch Into Main

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## When to Use

Use this template **after a release** to align main branch history with the release branch. This is a **forward merge** (release → main), not a backport.

| Scenario                              | Use This? | Alternative                                |
| ------------------------------------- | --------- | ------------------------------------------ |
| After publishing X.Y.Z release        | ✅ Yes    | —                                          |
| Port a specific fix to release branch | ❌ No     | [backport-pr.md](backport-pr.md)           |
| Fix CI on release branch              | ❌ No     | [branch-ci-fix.md](branch-ci-fix.md)       |
| Create release version bump           | ❌ No     | [packaging-branch.md](packaging-branch.md) |

## Reference PRs

Study these as examples of well-executed merges:

- [#2487 - Merge release-6.16 into main (v6.16.5)](https://github.com/dartsim/dart/pull/2487)
- [#2411 - Merge release-6.16 into main (v6.16.4)](https://github.com/dartsim/dart/pull/2411)

## Prompt

````text
# DART: Merge Release Branch Into Main

Merge the active release branch into main to align history.

## Auto-Detection

Run these commands to automatically determine all required values:

```bash
# 1. Find the active release branch (highest release-X.Y)
git fetch origin
RELEASE_BRANCH=$(git branch -r | grep -oE 'origin/release-[0-9]+\.[0-9]+' | sort -V | tail -1 | sed 's|origin/||')
echo "Release branch: $RELEASE_BRANCH"

# 2. Get latest release version from that branch
RELEASE_VERSION=$(git show origin/$RELEASE_BRANCH:package.xml | grep -oP '(?<=<version>)[^<]+')
echo "Release version: $RELEASE_VERSION"

# 3. Determine target milestone (main = DART 7.0)
TARGET_MILESTONE="DART 7.0"
echo "Target milestone: $TARGET_MILESTONE"

# 4. Verify tag exists
git tag | grep "v$RELEASE_VERSION" && echo "Tag found" || echo "Warning: Tag v$RELEASE_VERSION not found"

# 5. Show commits to be merged
echo "New commits on $RELEASE_BRANCH since last merge:"
git log origin/main..origin/$RELEASE_BRANCH --oneline | head -10
````

## Prerequisites

```bash
# Ensure clone is not shallow (required for merge)
if [ "$(git rev-parse --is-shallow-repository)" = "true" ]; then
  git fetch --unshallow origin main $RELEASE_BRANCH
fi

# Verify merge base exists
git merge-base origin/main origin/$RELEASE_BRANCH || echo "ERROR: No common ancestor"
```

## Steps

### 1. Create merge branch

```bash
git checkout -B merge/$RELEASE_BRANCH-into-main origin/main
```

### 2. Merge release branch

```bash
git merge origin/$RELEASE_BRANCH -m "Merge $RELEASE_BRANCH into main (v$RELEASE_VERSION)"
```

### 3. Resolve conflicts

Three types of conflicts will occur:

**DU conflicts (Deleted in main, Updated in release):**

- Files removed as part of DART 7 cleanup
- Resolution: Keep deleted

```bash
git status --porcelain | grep "^DU" | sed 's/^DU //' | xargs -r git rm -f
echo "Resolved $(git status --porcelain | grep -c '^DU' || echo 0) DU conflicts"
```

**AA conflicts (Added in both branches):**

- CI/Docker/script files added independently
- Resolution: Prefer main's version

```bash
git status --porcelain | grep "^AA" | sed 's/^AA //' | while read file; do
  git checkout --ours "$file" && git add "$file"
done
echo "Resolved $(git status --porcelain | grep -c '^AA' || echo 0) AA conflicts"
```

**UU conflicts (Content conflicts):**

- Resolution: Prefer main's C++20 modernization
- **Exception**: CHANGELOG.md requires manual merge (see below)

```bash
# Resolve all UU conflicts EXCEPT CHANGELOG.md
git status --porcelain | grep "^UU" | sed 's/^UU //' | grep -v CHANGELOG.md | while read file; do
  git checkout --ours "$file" && git add "$file"
done
echo "Resolved UU conflicts (excluding CHANGELOG.md)"
```

**CHANGELOG.md (Special Handling Required):**

CHANGELOG.md is the trickiest conflict because:
- Release branch has version-specific sections (e.g., "DART 6.16.5")
- Main branch has DART 7.0 section with possibly the same fixes
- Blind `--ours` would lose release version documentation

Resolution strategy:
1. Keep main's version as base (has DART 7.0 structure)
2. Check if release entries are already in DART 7.0 section (usually yes, via individual PRs)
3. If fixes are missing from DART 7.0, add them (rare - fixes should already be forward-ported)

```bash
# Check CHANGELOG.md conflict
if git status --porcelain | grep -q "^UU CHANGELOG.md"; then
  echo "CHANGELOG.md has conflicts - manual review needed"
  
  # Show what's different
  echo "=== Release branch changelog entries ==="
  git show origin/$RELEASE_BRANCH:CHANGELOG.md | head -100
  
  echo "=== Main branch changelog entries ==="
  git show origin/main:CHANGELOG.md | head -100
  
  # Usually safe to prefer main's version if fixes were forward-ported individually
  # The DART 6.16.x section in release branch is for that branch only
  git checkout --ours CHANGELOG.md && git add CHANGELOG.md
  echo "Used main's CHANGELOG.md (fixes should already be in DART 7.0 section)"
fi
```

Common CHANGELOG scenarios:

| Scenario | Action |
|----------|--------|
| Fix in both DART 7.0 and 6.16.x sections | Keep main's version (no duplication needed) |
| Fix only in 6.16.x section | Add to DART 7.0 section if not already there |
| Version bump entries (Packaging X.Y.Z) | Keep in release section only (not for main) |

**New files only in release (check after resolving above):**

```bash
# Files that failed --ours because they only exist in release
git diff --name-only --diff-filter=U | while read file; do
  git checkout --theirs "$file" && git add "$file"
  echo "Kept from release: $file"
done
```

### 4. Verify all conflicts resolved

```bash
REMAINING=$(git diff --name-only --diff-filter=U | wc -l)
[ "$REMAINING" -eq 0 ] && echo "All conflicts resolved" || echo "ERROR: $REMAINING conflicts remain"
```

### 5. Commit the merge

```bash
git commit -m "Merge $RELEASE_BRANCH into main (v$RELEASE_VERSION)"
```

### 6. Push and create PR with milestone

```bash
git push -u origin merge/$RELEASE_BRANCH-into-main

gh pr create --base main \
  --title "Merge $RELEASE_BRANCH into main (v$RELEASE_VERSION)" \
  --body "$(cat <<EOF
## Summary
- Merge $RELEASE_BRANCH into main to align history after v$RELEASE_VERSION release
- Forward-port release changelog entries (fixes already in main via individual PRs)

## Conflict Resolution
- **DU conflicts**: Kept deleted (DART 7 cleanup)
- **AA conflicts**: Preferred main's version (DART 7 compatibility)
- **UU conflicts**: Preferred main's C++20 modernization

## Testing
- CI checks pending
EOF
)"

# Set milestone to DART 7.0 (target version for main branch)
PR_NUMBER=$(gh pr list --head merge/$RELEASE_BRANCH-into-main --json number -q '.[0].number')
gh pr edit $PR_NUMBER --milestone "$TARGET_MILESTONE"
echo "Set milestone to: $TARGET_MILESTONE"
```

### 7. Monitor CI

```bash
gh run list --branch merge/$RELEASE_BRANCH-into-main --limit 3
# Watch specific run:
# gh run watch <RUN_ID> --interval 30
```

## Output

- PR URL and CI status
- Summary of conflicts resolved (by type and count)
- Milestone set to DART 7.0

## Fully Automatic Mode

This prompt is designed to work with **zero user input**:

1. Auto-detects `$RELEASE_BRANCH` (highest release-X.Y branch)
2. Auto-detects `$RELEASE_VERSION` (from package.xml on that branch)
3. Auto-sets milestone to `DART 7.0` (main branch target)
4. Resolves all conflicts using documented strategies
5. Only stops if: shallow clone, no merge base, or CI fails

```

## Conflict Resolution Quick Reference

| Conflict Type        | Count (typical) | Resolution         | Reason                      |
| -------------------- | --------------- | ------------------ | --------------------------- |
| DU (delete/update)   | 200-300         | Keep deleted       | DART 7 removed these files  |
| AA (add/add)         | 5-15            | Prefer main        | DART 7 compatibility        |
| UU (content)         | 150-250         | Prefer main        | C++20 modernization         |
| New files in release | 1-5             | Keep from release  | New tests/fixes             |

## Milestone Reference

| Target Branch | Milestone   | Notes                           |
| ------------- | ----------- | ------------------------------- |
| main          | DART 7.0    | Next major release              |
| release-6.16  | DART 6.16.x | Patch releases (set minor too)  |
```
