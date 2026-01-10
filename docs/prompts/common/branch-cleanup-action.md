# DART: Old Branch Cleanup (Action)

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: Old Branch Cleanup (Action)

Goal
- Triage stale branches in dartsim/dart and either delete, rebase + PR, or research + plan.

Inputs
- Branches to review: <BRANCH_REF_LIST> (each <BRANCH_REF> is a full ref like origin/foo or a local branch name)
- Branch name for delete: <BRANCH_NAME> (branch without `origin/`)
- Target/base branch: <TARGET_BRANCH or origin/main (DART 7)>
- Original base branch (if known): <ORIGINAL_BASE or unknown>
- Notes: <optional>

Workflow
- Read `AGENTS.md` and `docs/onboarding/`.
- `git fetch --all --prune`.
- For each branch:
  1) Confirm it exists on origin.
  2) If the branch was likely based on a non-main target (release/feature), identify that original base first and use it for the initial comparison.
  3) Get ahead/behind vs target: `git rev-list --left-right --count <TARGET_BRANCH>...<BRANCH_REF>`.
  4) If ahead count is 0: delete the remote branch (`git push origin --delete <BRANCH_NAME>`; use the branch name without `origin/`).
  5) If ahead count > 0: inspect commits (`git log --oneline <TARGET_BRANCH>..<BRANCH_REF>`), diff scope (`git diff --stat <TARGET_BRANCH>..<BRANCH_REF>`), and check if commits already landed (`git cherry -v <TARGET_BRANCH> <BRANCH_REF>`).
  6) Decide and act:
     - Not relevant, outdated, or invalid: delete the remote branch.
     - Small change with no conflicts vs target: rebase onto latest <TARGET_BRANCH>, revise as needed, then open a PR.
     - Large change or conflicts: research intent and current validity; if still worth it, create a plan in `docs/dev_tasks/<TASK_NAME>/` and execute phase-by-phase before PR.
  7) If you need to make changes, work in a new branch off the latest <TARGET_BRANCH> (use origin/main when targeting main), e.g., cleanup/<task> or cleanup/branch/<task>.

Output
- Per-branch decision and rationale.
- Branches deleted (remote).
- PR URLs and CI status.
- Open questions or risks.

Rules
- Ask before deleting if ownership is unclear.
- Keep changes minimal; avoid drive-by refactors.
- Use ASCII only.
```
