# DART: Pre-PR Branch Cleanup

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: Pre-PR Branch Cleanup

Goal
- Prepare the current branch for a PR by removing unnecessary or intermediate changes.
- Prefer the smallest viable change set unless explicitly asked to expand scope.

Inputs
- Repo: dartsim/dart
- Branch: <BRANCH>
- Target branch: <TARGET_BRANCH or origin/main>
- Summary of changes: <3-8 bullets or "infer from git diff --stat">
- Must-keep changes: <bullets or "none">
- Known experimental edits: <bullets or "unknown">
- Constraints: <optional>

Workflow
1) Compare against the target branch and summarize what changed.
2) Identify and propose removal of:
   - debug artifacts
   - dead code or temporary scaffolding
   - redundant formatting-only diffs (unless required)
   - reverted experiments or unused files
3) If any change is ambiguous, ask before deleting.
4) Keep changes minimal and scoped to the goal; do not refactor unless asked.

Output
- Cleanup plan: <3-6 bullets>
- Proposed removals: <bullets with file paths>
- Actions taken: <bullets with commands or edits>
- Remaining diffs to review: <bullets or "none">

Rules
- Do not discard work without confirmation.
- Avoid drive-by refactors or style-only edits.
- If a PR template exists, ensure the final diff aligns with its expectations.
- Use ASCII only.
```
