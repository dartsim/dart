# DART: PR Title + Description

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: PR Title + Description

Goal
- Provide an updated PR title and a concise PR description for the current branch.

Inputs
- Repo: dartsim/dart
- Branch: <BRANCH>
- Summary of changes: <3-8 bullets or "infer from git diff --stat">
- Why this is needed: <1-3 sentences or "infer, then confirm">
- Long-term goal: <1-2 sentences or "unknown">
- Related issue/PRs: <links or "none">
- Change type: <feature|bugfix|refactor|api-change|perf|docs|build|test|chore>
- Constraints: <optional>

Output
- PR Title: <single line>
- PR Description (use the repo's PR template if one exists):
  - Why: <why this change is needed>
  - Long-term goal: <how this change supports the bigger direction>
  - Changes: <3-6 bullets grouped by area>
  - User impact (only if refactor or API change):
    - Before: <old usage or behavior>
    - After: <new usage or behavior>
  - Developer impact (only if refactor or API change):
    - Before: <old API/surface or internal flow>
    - After: <new API/surface or internal flow>
  - Testing: <what was run or "not run">
  - Risks/Notes: <compat or migration notes, if any>

Rules
- Title reflects the overall branch goal, not a single commit.
- Be specific; avoid vague words like "misc", "updates", "cleanup".
- Keep the description short and skimmable (1-3 short paragraphs + bullets).
- If a PR template exists, follow its sections and add this content inside them.
- If critical inputs are missing, ask for them instead of guessing.
- If you paste this into `gh pr create` and the body includes backticks, use `--body-file` or a here-doc to avoid shell expansion.
- Use ASCII only.
```
