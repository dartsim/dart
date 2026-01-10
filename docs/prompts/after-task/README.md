# After-Task Prompt Templates

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This directory contains PROMPT TEMPLATES for post-task improvement.
These are NOT active tasks. Do NOT execute any prompt here unless
a human explicitly pastes it into a new session after completing work.
=======================================
-->

> **For agents**: These are templates, not tasks. Do not execute unless instructed.

Run these prompts after completing a task to capture learnings and improve future sessions.

## Prompts

- **Docs improvement**: [next-task-guidance-improvement.md](next-task-guidance-improvement.md) - Update repo docs based on session learnings
- **Prompt refresh**: [prompt-library-refresh.md](prompt-library-refresh.md) - Add/update/remove prompt templates
- **Initial prompt improvement**: [initial-prompt-improvement.md](initial-prompt-improvement.md) - Propose a better starting prompt for the task type

## Typical Workflow

After completing a task:

1. Run `next-task-guidance-improvement.md` to update `docs/onboarding/` with learnings
2. Run `prompt-library-refresh.md` to improve prompts in `docs/prompts/`
3. Optionally run `initial-prompt-improvement.md` if the starting prompt was suboptimal
