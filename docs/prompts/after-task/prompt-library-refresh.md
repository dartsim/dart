# After Finishing a Task - Prompt Library Refresh

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# After Finishing a Task - Prompt Library Refresh

We have just completed a task together in this same chat thread, and it is now finished (no further product/code changes are needed). Now review the prompt library at `docs/prompts/` and add, revise, or remove prompt templates so future agent tasks are faster and less error-prone.

Scope (hard rules)
- Only change prompt templates and category README indexes in `docs/prompts/`.
- Do not resume the completed task or edit other parts of the repo.
- Base changes only on what we learned in this thread plus current prompt files.
- Follow `docs/prompts/CONTRIBUTING.md` and existing prompt style.
- Keep prompts short, task-scoped, and ASCII unless a symbol is required.
- Each prompt must use a `## Prompt` section with a fenced `text` code block and placeholders like `<PLACEHOLDER>`.
- Use kebab-case file names for any new prompt.
- If you add a prompt, update the relevant category `README.md` to link it.
- Prefer minimal, surgical edits; avoid broad rewrites.

Prompt hygiene / size discipline
- Prefer tightening or merging over adding new prompts.
- Remove stale, duplicative, or low-signal prompts when a tighter version exists.
- Keep category indexes lean and focused; avoid long, overlapping lists.

Process
1) Read `docs/prompts/README.md`, `docs/prompts/CONTRIBUTING.md`, and the category `README.md` indexes.
2) Identify missing or weak prompts revealed by this thread.
3) Add or tighten prompts; remove duplicates if needed.

Deliverable
- Summary bullets of changes and rationale.
- Edited file list (paths).
- Any prompts you considered but did not add, and why.
```
