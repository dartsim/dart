# Prompt Templates

> **STOP**: These are templates for humans to start new sessions, NOT tasks for agents.
> If you reached here while working, return to your actual task.

## Prefer Slash Commands

**Use slash commands instead of manual prompts.** They auto-load context and enforce workflows:

| Command             | Replaces                                                     | Defined In                                |
| ------------------- | ------------------------------------------------------------ | ----------------------------------------- |
| `/dart-new-task`    | [common/new-task-core.md](common/new-task-core.md)           | `.claude/commands/`, `.opencode/command/` |
| `/dart-resume`      | [common/resume-task.md](common/resume-task.md)               | `.claude/commands/`, `.opencode/command/` |
| `/dart-fix-issue`   | [issues/new.md](issues/new.md)                               | `.claude/commands/`, `.opencode/command/` |
| `/dart-fix-ci`      | [common/ci-failure-fix.md](common/ci-failure-fix.md)         | `.claude/commands/`, `.opencode/command/` |
| `/dart-review-pr`   | [common/pr-review-feedback.md](common/pr-review-feedback.md) | `.claude/commands/`, `.opencode/command/` |
| `/dart-docs-update` | [common/docs-update.md](common/docs-update.md)               | `.claude/commands/`, `.opencode/command/` |

Works with: Claude Code (`.claude/`), OpenCode (`.opencode/`).

---

## Prompt-Only Templates (No Command Equivalent)

These templates don't have slash command equivalents. Use them manually:

| Prompt                                                           | Purpose                        |
| ---------------------------------------------------------------- | ------------------------------ |
| [common/mechanical-refactor.md](common/mechanical-refactor.md)   | Behavior-preserving transforms |
| [common/branch-cleanup.md](common/branch-cleanup.md)             | Analyze/delete stale branches  |
| [common/pr-merge-and-cleanup.md](common/pr-merge-and-cleanup.md) | Monitor CI and merge           |
| [issues/discussion.md](issues/discussion.md)                     | Triage / validity check        |
| [issues/closing-message.md](issues/closing-message.md)           | Generate closing reply         |
| [release/packaging-branch.md](release/packaging-branch.md)       | Create release packaging PR    |
| [release/backport-pr.md](release/backport-pr.md)                 | Cherry-pick to release branch  |
| [release/branch-ci-fix.md](release/branch-ci-fix.md)             | Fix CI on release branch       |
| [release/merge-into-main.md](release/merge-into-main.md)         | Merge release branch to main   |
| [after-task/improve-docs.md](after-task/improve-docs.md)         | Capture learnings → route to correct docs |
| [after-task/improve-prompts.md](after-task/improve-prompts.md)   | Update `docs/prompts/`         |

---

## Usage (Fallback)

For tools without slash command support:

1. Copy the `text` block from the template
2. Fill `<PLACEHOLDERS>`
3. Paste into new session

Style guide: [CONTRIBUTING.md](CONTRIBUTING.md)
