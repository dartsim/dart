# DART: Release Branch CI Fix

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Release Branch CI Fix

Context
- Release branch: <RELEASE_BRANCH>
- Release version: <RELEASE_VERSION>
- Failing run: <RUN_URL_OR_ID>
- Existing PR: <PR_URL or "create new">
- Milestone: <MILESTONE_NAME or blank>

Workflow
- If existing PR: continue on that branch. If "create new": branch from latest <RELEASE_BRANCH>.
- Read `AGENTS.md`, `CONTRIBUTING.md`, `docs/onboarding/ci-cd.md`.
- Inspect failures: `gh run view <RUN_ID> --log-failed` or `gh run view <RUN_ID> --job <JOB_ID> --log`.
- Apply minimal fixes; prefer fixes already in `origin/main` to reduce merge conflicts.
- Identify why failure wasn't caught earlier; update workflows if needed.
- Commit and push.
- If new PR needed: `gh pr create` into <RELEASE_BRANCH>, set milestone.
- Monitor CI: `gh run watch <RUN_ID> --interval 30` until green.

Output
- Root cause and why not caught earlier.
- Fix summary and CI status.
- PR URL (if created).
```
