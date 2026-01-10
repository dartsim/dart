# After Finishing a Task - Initial Prompt Improvement

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# After Finishing a Task - Initial Prompt Improvement

We have just completed a task together in this session, and it is now finished (no further product/code changes are needed). Based on the full context, propose the best initial prompt that would have produced equal or better results than the prompts used here. Then update the prompt library at `docs/prompts/` to capture that improved prompt as a reusable template (add, improve, remove, or reorganize prompts as needed).

Rules
- Only change prompt templates and category README indexes in `docs/prompts/`.
- Follow `docs/prompts/README.md`, `docs/prompts/CONTRIBUTING.md`.
- Keep prompts short, task-scoped, reusable, and ASCII unless a symbol is required.
- Each prompt must use a `## Prompt` section with a fenced `text` code block and placeholders like `<PLACEHOLDER>`.
- Avoid task-specific or ephemeral identifiers (branches, PRs, commit hashes, timestamps, usernames, machine paths).

Process
1) Draft the improved initial prompt (concise and reusable).
2) Implement it in the prompt library (add/update/remove).
3) Update the relevant category `README.md` links.

Deliverable
- The improved initial prompt text.
- Summary of changes with rationale.
- Edited file list (paths).
```
