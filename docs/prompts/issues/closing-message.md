# DART: Issue Closing Message

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Issue Closing Message

Context
- Issue: <ISSUE_URL_OR_NUMBER>
- Resolution summary: <1-3 bullets or "no longer reproducible on main">
- Extra context: <optional>

Output
- A concise, friendly closing reply (2-5 sentences) that thanks the reporter and explains why the issue is being closed.
- Use `gh issue comment <ISSUE_NUMBER> --body "<message>"` to post the comment.
- Use `gh issue close <ISSUE_NUMBER>` to close the issue.
```
