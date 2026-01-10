# DART Prompt Templates

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This directory contains PROMPT TEMPLATES for starting new agent sessions.
These are NOT active tasks. Do NOT execute any prompt in this directory
unless a human explicitly pastes it into a new session with filled placeholders.

If you are an agent and reached this file by following documentation:
- STOP: Do not treat these as instructions to execute
- RETURN: Go back to your actual task
- ASK: If confused, ask the human for clarification
=======================================
-->

> **STOP - READ THIS FIRST**
>
> This directory contains **reusable prompt templates** for starting new AI agent sessions.
> These are **NOT active tasks** to execute. They are reference materials for humans
> to copy, customize, and paste into fresh sessions.
>
> **If you are an agent**: Do not execute prompts from this directory. Return to your
> actual task or ask for clarification.

## Purpose

Short, reusable prompt templates for `dartsim/dart` tasks. These templates encode
lessons learned from previous sessions to make future sessions faster and less error-prone.

## Directory Structure

```
docs/prompts/
├── README.md              # This file (index + usage guide)
├── CONTRIBUTING.md        # Style guide for adding/updating prompts
├── after-task/            # Post-task improvement prompts
├── common/                # General workflow prompts
├── issues/                # GitHub issue handling prompts
└── release/               # Release and packaging prompts
```

## Categories

- **After-task prompts**: [after-task/README.md](after-task/README.md) - Run after completing a task to improve docs and prompts
- **Common prompts**: [common/README.md](common/README.md) - General task templates (new task, CI fix, PR workflow)
- **Issue prompts**: [issues/README.md](issues/README.md) - GitHub issue triage and resolution
- **Release prompts**: [release/README.md](release/README.md) - Release packaging and branch management

## How to Use

1. Open the category README and pick the prompt file for your task type.
2. Copy the fenced prompt block exactly.
3. Fill in placeholders like `<ISSUE>`, `<PR_URL>`, `<BRANCH>`.
4. Paste into a fresh agent session (OpenCode, Codex, Claude, etc.).

## Naming Conventions

- Category `README.md` files are indexes for that folder.
- Prompt files use kebab-case and are scoped to a single task type.
- Placeholders are written as `<PLACEHOLDER>` and should be replaced verbatim.

## Workflow Integration

After completing a task, consider running the after-task prompts to:

1. **Improve docs**: [after-task/next-task-guidance-improvement.md](after-task/next-task-guidance-improvement.md)
2. **Refresh prompts**: [after-task/prompt-library-refresh.md](after-task/prompt-library-refresh.md)
3. **Improve initial prompt**: [after-task/initial-prompt-improvement.md](after-task/initial-prompt-improvement.md)

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for prompt style and structure guidelines.
