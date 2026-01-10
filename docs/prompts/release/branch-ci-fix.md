# Release Branch CI Fix Prompt

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: Release Branch CI Fix

Context
- PR: <PR_URL>
- Release branch: <RELEASE_BRANCH>
- Release version: <RELEASE_VERSION>
- Failing CI job: <CI_JOB_URL>
- Notes: <optional>

Workflow
- Continue working on the current branch unless told otherwise.
- Read `AGENTS.md` and `CONTRIBUTING.md` (and `docs/onboarding/ci-cd.md` if present).
- Identify why the failure was not caught earlier; decide whether release-branch workflows need updates.
- Apply minimal fixes for <CI_JOB_URL>.
- Prefer fixes already in origin/main to reduce merge conflicts when this release branch merges back.
- If adding a new CI job, add a short note on why it is needed.
- Commit and push with `git push`, then monitor CI with `gh run watch <RUN_ID> --interval 30` until green.

Output
- Root cause and why it was not caught.
- Workflow changes (if any) and rationale.
- Fix summary and CI status.
```
