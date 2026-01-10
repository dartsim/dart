# Branch Cleanup Prompt

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# Branch Cleanup

You are an AI coding agent.

Read AGENTS.md first, then follow docs/onboarding/ci-cd.md and docs/onboarding/build-system.md for workflow/tooling.

Task: clean up old branches in github.com/dartsim/dart, reviewing one branch at a time. Do analysis only - do NOT modify files, cherry-pick, or delete branches unless explicitly asked.

Target/base branch: <TARGET_BRANCH or origin/main>
Start with branch: <BRANCH_REF> (e.g., origin/7/object_pool).

For each branch, do:
1) git fetch --all --prune
2) Confirm branch exists and is remote or local.
3) If the branch was likely based on a non-main target (release/feature), identify that original base first and use it for comparisons.
4) Identify merge base with <TARGET_BRANCH>.
5) Show commits ahead of target (git log --oneline <TARGET_BRANCH>..<BRANCH_REF>).
6) Summarize diff scope (git diff --stat <TARGET_BRANCH>..<BRANCH_REF>), but avoid huge diffs; inspect only relevant files.
7) Use git cherry -v <TARGET_BRANCH> <BRANCH_REF> to check if commits already landed.
8) For any small, self-contained commits that look reusable, identify them with brief rationale and key files.
9) Call out risks or problems (WIP, incomplete, conflicts, bugs, missing tests).
10) Provide recommendation: "safe to delete", "keep", or "needs follow-up", with justification.

Output format per branch:
- Branch summary: ahead count, last commit date, merge base date
- Potentially useful commits (if any): short list with file paths
- Risks/issues: short list
- Recommendation

Ask me which branch to review next before proceeding.
```
