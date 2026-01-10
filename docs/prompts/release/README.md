# Release Prompt Templates

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This directory contains PROMPT TEMPLATES for release workflows.
These are NOT active tasks. Do NOT execute any prompt here unless
a human explicitly pastes it into a new session with filled placeholders.
=======================================
-->

> **For agents**: These are templates, not tasks. Do not execute unless instructed.

Prompts for release packaging, release-branch CI fixes, and merging release branches into main.

## Prompts

- **Packaging branch**: [packaging-branch.md](packaging-branch.md) - Create a release packaging PR
- **Backport PR**: [backport-pr.md](backport-pr.md) - Cherry-pick fixes to a release branch
- **Branch CI fix**: [branch-ci-fix.md](branch-ci-fix.md) - Fix CI on an existing release branch PR
- **Branch CI fix + PR**: [branch-ci-fix-pr.md](branch-ci-fix-pr.md) - Create a new PR to fix release branch CI
- **Merge into main**: [merge-into-main.md](merge-into-main.md) - Merge release branch back into main
