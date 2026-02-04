# DART: Monitor CI and Merge PR

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Monitor CI and Merge PR

Context
- PR: <PR_URL>
- PR number: <PR_NUMBER or "infer from PR">
- Target branch: <TARGET_BRANCH or origin/main>
- Merge method: <merge|squash|rebase|infer>
- Delete branch after merge: <yes|no>
- Constraints: <optional>

Workflow
- Start at repo root; read `AGENTS.md`, `CONTRIBUTING.md`, and `docs/onboarding/ci-cd.md` if present.
- Confirm required checks and review status; ask if anything is ambiguous.
- Monitor CI in real time: `gh pr checks <PR_NUMBER> --watch --fail-fast` (or `gh run watch <RUN_ID> --interval 30`).
- If any job fails, stop and fix the failure (or ask if out of scope).
- Once checks pass and reviews are satisfied, merge using the requested method via `gh pr merge <PR_NUMBER> --<method>`.
- If requested, delete the remote branch after merge with `gh pr merge --delete-branch` or `git push origin --delete <BRANCH>`.

Output
- CI result and merge status.
- Merge method used and whether the branch was deleted.
- Follow-up tasks or risks (if any).
```
