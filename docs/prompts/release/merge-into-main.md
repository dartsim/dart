# DART: Merge Release Branch Into Main

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Merge Release Branch Into Main

Merge <RELEASE_BRANCH> into main.

Context
- Release branch: <RELEASE_BRANCH>
- Reference PR (optional): <REFERENCE_PR_URL>

Steps:
1. Create branch from latest origin/main.
2. Merge origin/<RELEASE_BRANCH>.
3. Resolve conflicts; update CHANGELOG.md.
4. Push and create PR with `gh pr create`.
5. Monitor CI until green.

Output
- PR URL and CI status.
- Summary of conflicts resolved.
```
