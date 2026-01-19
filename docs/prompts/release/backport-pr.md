# DART: Release Branch Backport

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Release Branch Backport

Context
- Release branch: <RELEASE_BRANCH>
- Release version: <RELEASE_VERSION>
- Source PR or commits: <SOURCE_PR_OR_COMMITS>
- Source branch (if available): <SOURCE_BRANCH or origin/main>
- Target milestone: <MILESTONE_NAME or "none">
- Notes: <optional>

Workflow
- Create a new branch from the latest <RELEASE_BRANCH>.
- Read `AGENTS.md` and `CONTRIBUTING.md` (and `docs/onboarding/ci-cd.md` if present).
- Identify the exact commits to backport (prefer fixes already merged into origin/main).
- Check if the commits already exist on <RELEASE_BRANCH> (use `git cherry -v` or compare commit hashes).
- Cherry-pick with `-x` and keep changes minimal; resolve conflicts without refactors.
- Run the smallest relevant tests per repo docs; avoid expanding scope.
- If conflicts are large or the backport seems risky, stop and ask before proceeding.
- Commit and push with `git push -u origin HEAD`, then monitor CI until green.

PR
- Open a PR into <RELEASE_BRANCH> using `gh pr create` with the PR template if it exists.
- Link the source PR/commits and mention any conflicts/resolutions.
- Set the milestone if provided.

Output
- PR URL and CI status.
- Short summary of changes and conflicts.
```
