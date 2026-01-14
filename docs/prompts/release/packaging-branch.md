# DART: Release Packaging

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Release Packaging

Create release packaging PR for <NEW_VERSION>.

Auto-derive:
- prev_version: from package.xml
- base_branch: release-<major.minor> from <NEW_VERSION>
- release_date: today
- milestone: "DART <NEW_VERSION>" via `gh api repos/{owner}/{repo}/milestones`

Steps:
1. Sync base branch with origin.
2. Create branch: `release/<NEW_VERSION>-version-bump`.
3. Bump version in package.xml and pixi.toml.
4. Update find_package(DART ...) in examples/ and tutorials/.
5. Add CHANGELOG.md section for <NEW_VERSION>.
6. Commit: "Packaging <NEW_VERSION>".
7. Push and create PR to base branch with `gh pr create`.

Output
- PR URL.
- Ask if any derived info is ambiguous.
```
