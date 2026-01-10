# Release Branch Merge Into Main Prompt

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: Merge Release Branch Into Main

Context
- Release branch: <RELEASE_BRANCH>
- Release version: <RELEASE_VERSION>
- Release series: <RELEASE_SERIES>
- Latest release: <LATEST_RELEASE>
- Next major: <NEXT_MAJOR>
- Reference PR: <REFERENCE_PR_URL>

Workflow
- Create a new branch from the latest origin/main.
- Merge the latest origin/<RELEASE_BRANCH>.
- Resolve conflicts and update CHANGELOG.md to include release-branch changes.
- Prefer fixes already in origin/main to minimize conflicts.
- Push with `git push -u origin HEAD` and open a PR using `gh pr create` modeled after <REFERENCE_PR_URL>.

Output
- PR URL and CI status.
- Summary of conflicts and how they were resolved.
```
