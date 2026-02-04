# DART: Docs Update

> **Prefer command**: Use `/dart-docs-update` in Claude Code or OpenCode.
> This template is for tools without command support.

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Docs Update

Goal
- Update documentation in dartsim/dart with no product code changes.

Context
- Docs scope: <FILES_OR_TOPICS>
- Audience: <users|developers|contributors|mixed>
- Target branch: <TARGET_BRANCH or origin/main>
- Notes: <optional>

Workflow
- Read `AGENTS.md` and `CONTRIBUTING.md` to follow repo doc conventions.
- Create a new branch from the latest <TARGET_BRANCH>.
- Edit only docs (e.g., `docs/**`, README files, templates) unless explicitly asked otherwise.
- Keep changes concise and reusable; avoid task-specific IDs or temporary examples.
- If adding commands, label them as suggested/unverified unless you ran them.
- Update any indexes or references that point to the changed docs.
- Use `.github/PULL_REQUEST_TEMPLATE.md` for the PR body and set the milestone (`DART 7.0` for `main`, `DART 6.16.x` for `release-6.16`).
- Commit, push, and open a PR if requested.

Output
- Summary of changes and intent.
- Files touched.
- PR URL (if created).
```
