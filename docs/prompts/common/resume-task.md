# DART: Resume Task

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
Resuming Task
We need to resume unfinished work in `dartsim/dart` on the CURRENT branch. Prior chat history is NOT available. Use the current repo state as the source of truth; do not assume what was previously intended.

Known info
- Branch: infer via `git branch --show-current` (do not switch branches unless needed and explained)
- Goal hint (optional): <1 sentence or leave blank>
- Extra constraints (optional): <bullets or leave blank>

Safety/constraints
- Do not discard work: no `git reset --hard`, no `git clean -fdx`, no dropping stashes unless you explain and I explicitly confirm.
- Work in the correct directory (confirm with `git rev-parse --show-toplevel`).
- Follow repo conventions (`CONTRIBUTING.md`, onboarding docs, `AGENTS.md` pointers).
- Use repo entry points (`pixi run ...`); don't invent new ones.

Step 1: Recon (no code changes yet)
- Collect and summarize:
  - `git status -sb`
  - `git branch -vv`
  - `git remote -v`
  - `git log -20 --oneline --decorate`
  - `git diff --stat` (and `git diff` if needed)
  - `git stash list`
- Check whether this branch already has a PR:
  - `gh pr list --head "$(git branch --show-current)"`
  - `gh pr status`

Step 2: Reconstruct intent (from repo state)
- Infer the intended goal from:
  - branch name + commit messages
  - diff contents and touched subsystems
  - TODO/FIXME notes in changed files
  - any repo notes/docs that look like WIP guidance
  - `docs/dev_tasks/<TASK_NAME>/` notes and any resume prompt file
- If you cannot confidently infer the goal/acceptance criteria, STOP and ask me for:
  - intended outcome, constraints, and whether a PR should exist.

Step 3: Plan and continue
- Propose a short plan (3-6 steps) to finish the remaining work.
- Implement remaining changes with minimal scope.
- Add/revise unit tests when behavior changes.
- If `docs/dev_tasks/<TASK_NAME>/` exists, update plan/progress there and refresh the resume prompt file so a fresh agent can continue.
- Run the standard `pixi run ...` checks appropriate for the change (per repo docs), then lint before committing.

Step 4: Git + CI loop (if we're pushing)
- Keep commits focused.
- Sync with latest `origin/main` per repo convention, push with upstream tracking (`git push -u origin HEAD`).
- Create/update a PR with `gh pr create` or `gh pr edit` if needed, then monitor CI (`gh run list ...`, `gh run watch <id> --interval 30`) and fix failures until green.

Reporting expectations
- After recon: summarize branch, uncommitted changes, stashes, and inferred goal (or ask me to clarify).
- After finishing: summarize changes, files touched, commands run, and PR link (if any).
```
