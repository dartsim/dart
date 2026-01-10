# Resume Issue Prompt

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
Resuming Issue
Known info (fill what you know; leave the rest blank)
- Issue: <url or #>
- PR: <url or #>
- Expected branch: <name>
- Keywords for gh search: <terms>
- Extra constraints: <bullets>


We need to resume an unfinished task in `dartsim/dart`. Prior chat history is NOT available. Use the current repo state + GitHub metadata as the source of truth; do not assume what was previously intended.

Goal
- Safely reconstruct what was in progress, identify the target issue/PR, and continue to completion using the repo's standard workflows (`pixi run ...`).

Safety/constraints
- Do not discard work: no `git reset --hard`, no `git clean -fdx`, no dropping stashes unless you explain and I explicitly confirm.
- Work in the correct directory (confirm with `git rev-parse --show-toplevel`).
- Follow repo conventions (`CONTRIBUTING.md`, onboarding docs, `AGENTS.md` pointers).
- Must not break `pixi run -e gazebo test-gz` without any further Gazebo patching (do not modify `patch_gz_physics`, except the existing narrow exception for newer DART CMake config version).

Step 1: Recon (no code changes yet)
- Collect and summarize:
  - `git status -sb`
  - `git branch -vv`
  - `git remote -v`
  - `git log -20 --oneline --decorate`
  - `git diff --stat` (and `git diff` if needed)
  - `git stash list`
- Identify what this WIP corresponds to:
  - Try `gh pr list --head "$(git branch --show-current)"` and `gh pr status`
  - If unclear, search by branch name / recent commit messages: `gh pr list --search "<keywords>"`, `gh issue list --search "<keywords>"`
- If you cannot confidently identify the target issue/PR from repo+GitHub, STOP and ask me for the issue/PR link/number.

Step 2: Reconstruct intent
- Read the identified issue/PR fully (`gh issue view ...` / `gh pr view ...`) and extract acceptance criteria + constraints.
- Map "what's already done" vs "what remains" by inspecting changed files and any TODO/FIXME notes.
- Check `docs/dev_tasks/<TASK_NAME>/` for plan/progress notes and any resume prompt file; use it as a source of truth if present.

Step 3: Plan and continue
- Propose a short plan (3-6 steps) to finish the remaining work.
- Implement the remaining changes with minimal scope.
- Add/revise unit tests appropriate to the changes.
- If `docs/dev_tasks/<TASK_NAME>/` exists, update plan/progress there and refresh the resume prompt file so a fresh agent can continue.
- Use the repo's existing entry points (`pixi run ...`); don't invent new ones.

Step 4: When ready, run the standard checks (use repo guidance for parallelism)
- `pixi run test` (quick)
- `pixi run test-all` (final)
- `pixi run -e gazebo test-gz` (must pass under the constraint above)
- `pixi run lint` before committing
(If parallelism requires specific flags/env vars, inspect the `pixi` task definitions and apply the correct mechanism for the underlying runner. Use ~2/3 of logical cores if the repo provides no guidance.)

Step 5: Git + CI loop
- Keep commits focused and readable.
- Sync with the latest `origin/main` per repo convention (merge/rebase), push the branch with `git push -u origin HEAD`, and open/update the PR with `gh pr create` or `gh pr edit`.
- Monitor CI fail-fast: get the run id via `gh run list -b <branch>`, then `gh run watch <id> --interval 30`; fix the first failure and repeat until green.

Reporting expectations
- After recon: summarize current branch, uncommitted changes, stashes, likely target issue/PR.
- After finishing: summarize changes, files touched, commands run, PR link, and any remaining risks.
```
