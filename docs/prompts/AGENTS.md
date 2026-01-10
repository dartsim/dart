# Prompt Templates

> **STOP**: These are templates for humans to start new sessions, NOT tasks for agents.
> If you reached here while working, return to your actual task.

## What This Is

Reusable prompt templates for `dartsim/dart` tasks. Humans copy these, fill placeholders, and paste into fresh sessions.

## Categories

| Category                   | Purpose                                  |
| -------------------------- | ---------------------------------------- |
| [after-task/](after-task/) | Post-session improvement (docs, prompts) |
| [common/](common/)         | General workflows (new task, CI fix, PR) |
| [issues/](issues/)         | GitHub issue handling                    |
| [release/](release/)       | Release packaging and branches           |

## Usage

1. Pick a prompt from a category
2. Copy the fenced `text` block
3. Fill `<PLACEHOLDERS>`
4. Paste into a new session

## Style

See [CONTRIBUTING.md](CONTRIBUTING.md). Key rules:

- Keep prompts short and task-scoped
- Use `<PLACEHOLDER>` format
- Reference repo docs (don't duplicate)
- Use `gh` for GitHub, `pixi run` for builds
