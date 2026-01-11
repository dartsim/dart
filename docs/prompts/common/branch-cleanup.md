# DART: Branch Cleanup

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Branch Cleanup

Context
- Branches: <BRANCH_LIST or "list stale branches">
- Target branch: <TARGET_BRANCH or origin/main>
- Mode: <analyze|action>
  - analyze: review only, no deletions
  - action: triage and delete/rebase as appropriate

Workflow
- `git fetch --all --prune`
- Read `AGENTS.md`, `docs/onboarding/ci-cd.md`.

For each branch:
1) Confirm exists on origin.
2) Get ahead/behind: `git rev-list --left-right --count <TARGET>..<BRANCH>`.
3) If ahead=0: safe to delete (if action mode: `git push origin --delete <BRANCH_NAME>`).
4) If ahead>0:
   - `git log --oneline <TARGET>..<BRANCH>` (commits)
   - `git diff --stat <TARGET>..<BRANCH>` (scope)
   - `git cherry -v <TARGET> <BRANCH>` (already landed?)
5) Decide:
   - Outdated/invalid → delete
   - Small, no conflicts → rebase + PR (action mode)
   - Large/conflicts → document in `docs/dev_tasks/` for later

Output per branch:
- Summary: ahead count, last commit date
- Useful commits (if any)
- Risks/issues
- Recommendation: delete | keep | needs follow-up
- Actions taken (if action mode)

Rules
- Ask before deleting if ownership unclear.
- Keep changes minimal; no drive-by refactors.
```
