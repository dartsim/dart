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

## Release Workflow Decision Tree

**Use this to pick the right release prompt:**

```
What are you doing?
│
├─ "We just released X.Y.Z, need to sync main"
│   └─→ [release/merge-into-main.md](release/merge-into-main.md)
│       Forward-merges release branch into main to align history
│
├─ "I need to port a fix from main to release branch"
│   └─→ [release/backport-pr.md](release/backport-pr.md)
│       Cherry-picks commits from main to release-X.Y
│
├─ "CI is failing on the release branch"
│   └─→ [release/branch-ci-fix.md](release/branch-ci-fix.md)
│       Fixes CI issues specific to release branch
│
├─ "I need to create a new release (version bump)"
│   └─→ [release/packaging-branch.md](release/packaging-branch.md)
│       Creates version bump PR for new release
│
└─ "I need to fix a bug reported downstream (gz-physics/Gazebo)"
    └─→ [issues/downstream-fix.md](issues/downstream-fix.md)
        Fixes DART bug reported via downstream projects
```

### Release Prompt Quick Reference

| Prompt                                             | Direction      | When to Use                              |
| -------------------------------------------------- | -------------- | ---------------------------------------- |
| [merge-into-main.md](release/merge-into-main.md)   | release → main | After publishing a release, sync history |
| [backport-pr.md](release/backport-pr.md)           | main → release | Port specific fix to release branch      |
| [branch-ci-fix.md](release/branch-ci-fix.md)       | on release     | Fix CI failures on release branch        |
| [packaging-branch.md](release/packaging-branch.md) | on release     | Create version bump for new release      |

---

## Prompt-Only Templates (No Command Equivalent)

These templates don't have slash command equivalents. Use them manually:

### Common Tasks

| Prompt                                                           | Purpose                        |
| ---------------------------------------------------------------- | ------------------------------ |
| [common/mechanical-refactor.md](common/mechanical-refactor.md)   | Behavior-preserving transforms |
| [common/branch-cleanup.md](common/branch-cleanup.md)             | Analyze/delete stale branches  |
| [common/pr-merge-and-cleanup.md](common/pr-merge-and-cleanup.md) | Monitor CI and merge           |

### Issue Handling

| Prompt                                                 | Purpose                                   |
| ------------------------------------------------------ | ----------------------------------------- |
| [issues/discussion.md](issues/discussion.md)           | Triage / validity check                   |
| [issues/closing-message.md](issues/closing-message.md) | Generate closing reply                    |
| [issues/downstream-fix.md](issues/downstream-fix.md)   | Fix DART bug from gz-physics/Gazebo issue |

### Release Management

| Prompt                                                     | Purpose                       |
| ---------------------------------------------------------- | ----------------------------- |
| [release/packaging-branch.md](release/packaging-branch.md) | Create release packaging PR   |
| [release/backport-pr.md](release/backport-pr.md)           | Cherry-pick to release branch |
| [release/branch-ci-fix.md](release/branch-ci-fix.md)       | Fix CI on release branch      |
| [release/merge-into-main.md](release/merge-into-main.md)   | Merge release branch to main  |

### After Task

| Prompt                                                                       | Purpose                                   |
| ---------------------------------------------------------------------------- | ----------------------------------------- |
| [after-task/improve-docs.md](after-task/improve-docs.md)                     | Capture learnings → route to correct docs |
| [after-task/audit-agent-compliance.md](after-task/audit-agent-compliance.md) | Analyze agent rule violations → fix docs  |

---

## Usage (Fallback)

For tools without slash command support:

1. Copy the `text` block from the template
2. Fill `<PLACEHOLDERS>`
3. Paste into new session

Style guide: [CONTRIBUTING.md](CONTRIBUTING.md)
