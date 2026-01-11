# DART: Resolve Issue

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Resolve Issue

Resolve GitHub issue <ISSUE>.

Workflow
1. Validate: `gh issue view <ISSUE>`, confirm still reproduces on `origin/main`.
   - If already fixed, explain and stop.

2. Plan: Identify root cause, smallest fix, and where regression test should live.

3. Implement:
   - Create branch from `origin/main`.
   - Fix the issue.
   - Add regression test.

4. Verify:
   - `pixi run test` (quick), `pixi run test-all` (final).
   - `pixi run lint` before committing.

5. PR:
   - Push: `git push -u origin HEAD`.
   - Create PR: `gh pr create`.
   - CI loop: `gh run watch <id> --interval 30`, fix failures until green.

Rules
- Confirm repo root: `git rev-parse --show-toplevel`.
- Follow `AGENTS.md`, `CONTRIBUTING.md`.
- Keep fix minimal; add tests for the bug.

Output
- Summary of fix.
- Files touched.
- PR link and CI status.
```
