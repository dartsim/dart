# DART: Issue Triage

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Issue Triage

Context
- Issue: <ISSUE_URL_OR_NUMBER>
- Notes: <optional>

Task
- Review the issue using `gh issue view <ISSUE_NUMBER>` and determine whether it is still valid on the latest origin/main.
- If still valid, summarize the next action (repro, fix, or request info).
- If not valid, explain why (already fixed, obsolete, or not reproducible) and suggest closing.

Output
- Status: <still valid|already fixed|needs info|out of scope>
- Rationale: <1-3 bullets>
- Recommended next step: <short>
```
