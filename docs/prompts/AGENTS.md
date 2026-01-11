# Prompt Templates

> **STOP**: These are templates for humans to start new sessions, NOT tasks for agents.
> If you reached here while working, return to your actual task.

Reusable prompts for `dartsim/dart`. Copy the `text` block, fill `<PLACEHOLDERS>`, paste into new session.

Style guide: [CONTRIBUTING.md](CONTRIBUTING.md)

## Common

| Prompt                                                                     | Purpose                        |
| -------------------------------------------------------------------------- | ------------------------------ |
| [common/new-task-core.md](common/new-task-core.md)                         | Full new task template         |
| [common/resume-task.md](common/resume-task.md)                             | Continue previous session      |
| [common/ci-failure-fix.md](common/ci-failure-fix.md)                       | Fix failing CI                 |
| [common/docs-update.md](common/docs-update.md)                             | Docs-only changes              |
| [common/mechanical-refactor.md](common/mechanical-refactor.md)             | Behavior-preserving transforms |
| [common/agent-workflow-automation.md](common/agent-workflow-automation.md) | Automation-first principles    |

## Branch & PR

| Prompt                                                             | Purpose                      |
| ------------------------------------------------------------------ | ---------------------------- |
| [common/branch-cleanup.md](common/branch-cleanup.md)               | Analyze stale branches       |
| [common/branch-cleanup-action.md](common/branch-cleanup-action.md) | Delete/rebase stale branches |
| [common/pre-pr-branch-cleanup.md](common/pre-pr-branch-cleanup.md) | Clean branch before PR       |
| [common/pr-title-description.md](common/pr-title-description.md)   | Generate PR metadata         |
| [common/pr-review-feedback.md](common/pr-review-feedback.md)       | Address review comments      |
| [common/pr-merge-and-cleanup.md](common/pr-merge-and-cleanup.md)   | Monitor CI and merge         |

## Issues

| Prompt                                                 | Purpose                 |
| ------------------------------------------------------ | ----------------------- |
| [issues/new.md](issues/new.md)                         | Resolve an issue        |
| [issues/resume.md](issues/resume.md)                   | Continue issue work     |
| [issues/discussion.md](issues/discussion.md)           | Triage / validity check |
| [issues/closing-message.md](issues/closing-message.md) | Generate closing reply  |

## Release

| Prompt                                                     | Purpose                       |
| ---------------------------------------------------------- | ----------------------------- |
| [release/packaging-branch.md](release/packaging-branch.md) | Create release packaging PR   |
| [release/backport-pr.md](release/backport-pr.md)           | Cherry-pick to release branch |
| [release/branch-ci-fix.md](release/branch-ci-fix.md)       | Fix CI on existing PR         |
| [release/branch-ci-fix-pr.md](release/branch-ci-fix-pr.md) | New PR to fix release CI      |
| [release/merge-into-main.md](release/merge-into-main.md)   | Merge release branch to main  |

## After-Task

Run after completing a session to capture learnings.

| Prompt                                                                                       | Purpose                   |
| -------------------------------------------------------------------------------------------- | ------------------------- |
| [after-task/next-task-guidance-improvement.md](after-task/next-task-guidance-improvement.md) | Update `docs/onboarding/` |
| [after-task/prompt-library-refresh.md](after-task/prompt-library-refresh.md)                 | Update `docs/prompts/`    |
| [after-task/initial-prompt-improvement.md](after-task/initial-prompt-improvement.md)         | Improve starting prompt   |
