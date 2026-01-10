# Common Prompt Templates

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This directory contains PROMPT TEMPLATES for common workflows.
These are NOT active tasks. Do NOT execute any prompt here unless
a human explicitly pastes it into a new session with filled placeholders.
=======================================
-->

> **For agents**: These are templates, not tasks. Do not execute unless instructed.

General-purpose prompts for common DART development workflows.

## Prompts

### Starting Work

- **New task (core)**: [new-task-core.md](new-task-core.md) - Full template for starting any new task
- **Agent workflow (automation-first)**: [agent-workflow-automation.md](agent-workflow-automation.md) - Principles for autonomous agent operation
- **Resume task**: [resume-task.md](resume-task.md) - Continue unfinished work from a previous session

### CI and Testing

- **CI failure fix**: [ci-failure-fix.md](ci-failure-fix.md) - Diagnose and fix failing CI runs

### Documentation

- **Docs update**: [docs-update.md](docs-update.md) - Documentation-only changes

### Refactoring

- **Mechanical refactor**: [mechanical-refactor.md](mechanical-refactor.md) - Behavior-preserving code transformations

### Branch Management

- **Branch cleanup (analysis)**: [branch-cleanup.md](branch-cleanup.md) - Analyze stale branches without modifying
- **Branch cleanup (action)**: [branch-cleanup-action.md](branch-cleanup-action.md) - Delete, rebase, or plan stale branches
- **Pre-PR cleanup**: [pre-pr-branch-cleanup.md](pre-pr-branch-cleanup.md) - Clean up branch before opening PR

### PR Workflow

- **PR title + description**: [pr-title-description.md](pr-title-description.md) - Generate PR metadata
- **PR review feedback**: [pr-review-feedback.md](pr-review-feedback.md) - Address reviewer comments
- **PR merge + cleanup**: [pr-merge-and-cleanup.md](pr-merge-and-cleanup.md) - Monitor CI and merge
