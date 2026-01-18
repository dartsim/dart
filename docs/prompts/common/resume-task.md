# DART: Resume Task

> **Prefer command**: Use `/dart-resume` in Claude Code or OpenCode.
> This template is for tools without command support.

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Resume Task

We need to resume unfinished work in `dartsim/dart`. Prior session history is NOT available. Use repo state + GitHub metadata as the source of truth.

Known info (fill what you know; leave blank if unknown)
- Branch: <BRANCH or "infer from current">
- Issue/PR: <URL or # or blank>
- Goal hint: <1 sentence or blank>
- Constraints: <bullets or blank>

Safety
- No destructive git commands (`reset --hard`, `clean -fdx`, dropping stashes) without confirmation.
- Confirm repo root with `git rev-parse --show-toplevel`.
- Follow `AGENTS.md`, `CONTRIBUTING.md`, `docs/onboarding/`.

Step 1: Recon (no changes yet)
- `git status -sb`, `git branch -vv`, `git log -10 --oneline --decorate`
- `git diff --stat`, `git stash list`
- `gh pr list --head "$(git branch --show-current)"`, `gh pr status`
- If issue/PR unknown, search: `gh pr list --search "<keywords>"`, `gh issue list --search "<keywords>"`
- If goal unclear, STOP and ask.

Step 2: Reconstruct intent
- From branch name, commits, diffs, TODO/FIXME notes, issue/PR description.
- Check `docs/dev_tasks/<TASK>/` if this is a tracked multi-session task.
- Map "done" vs "remaining".

Step 3: Plan and continue
- Propose 3-6 step plan.
- Implement with minimal scope.
- Add/revise tests as needed.

Step 4: Verify
- `pixi run test` (quick), `pixi run test-all` (final).
- `pixi run lint` before committing.

Step 5: Git + CI
- Keep commits focused.
- Sync with `origin/main`, push with `git push -u origin HEAD`.
- Create/update PR with `gh pr create` or `gh pr edit`.
- Monitor CI: `gh run watch <id> --interval 30`, fix failures until green.

Output
- After recon: branch, changes, stashes, inferred goal.
- After done: summary, files touched, PR link.
```
