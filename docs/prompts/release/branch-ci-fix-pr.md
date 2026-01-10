# Release Branch CI Fix + PR

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: Release Branch CI Fix + PR

Context
- Release branch: <RELEASE_BRANCH>
- Release version: <RELEASE_VERSION>
- Failing run URL or ID: <RUN_URL_OR_ID>
- Target milestone: <MILESTONE_NAME>

Workflow
- Create a new branch from the latest <RELEASE_BRANCH>.
- Read `AGENTS.md` and `CONTRIBUTING.md` (and `docs/onboarding/ci-cd.md` if present).
- If a run URL does not work with `gh run view` or `gh run watch`, extract the run ID.
- Inspect failures with `gh run view <RUN_URL_OR_ID> --log-failed` or `gh run view <RUN_URL_OR_ID> --job <JOB_ID> --log-failed`.
- Apply minimal fixes; prefer existing fixes from `origin/main` to reduce merge conflicts.
- Commit and push with `git push -u origin HEAD`; monitor CI until green using `gh run watch <RUN_ID> --interval 30`.

PR
- Open a PR into <RELEASE_BRANCH> using `gh pr create` with `.github/PULL_REQUEST_TEMPLATE.md` if it exists.
- Set milestone to <MILESTONE_NAME>.
- Include links to the failing run(s) and a brief change summary.

Output
- PR URL and CI status.
```
