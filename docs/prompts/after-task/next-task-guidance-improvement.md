# DART: Next-Task Guidance Improvement

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Next-Task Guidance Improvement

We just completed a task in this session (no further code changes needed). Update repo docs so future agents are faster.

Scope
- Docs only; no product/code changes.
- Source of truth: this session + current repo.
- Follow `CONTRIBUTING.md` style.
- Keep `AGENTS.md` concise (pointers only); details go in `docs/`.

What to extract
1. Task recap (2-5 sentences, general terms)
2. Repeatable playbook
3. Iteration commands + success signals
4. Gotchas (general failure modes)
5. Accelerators (high-leverage tips)

Where to update
- Prefer existing docs: `docs/onboarding/ci-cd.md`, `docs/onboarding/building.md`, etc.
- Add "Start here" pointers if discoverability is poor.
- Update `AGENTS.md` only if needed (1-3 bullets max).

Rules
- No ephemeral IDs (branches, PRs, commits, timestamps, usernames).
- Commands must be labeled "Suggested (Unverified)" unless verified.
- Prefer condensing over appending; keep docs lean.
- No fixed core counts; use `$(nproc)` or "~2/3 of logical cores".

Output
- Summary bullets of changes.
- Files edited.
- What was pruned/consolidated.
```
