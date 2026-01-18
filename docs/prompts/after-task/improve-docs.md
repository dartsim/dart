# DART: Improve Docs

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Improve Docs

Task completed. Now update repo docs so future agents are faster.

Scope
- Docs only (`docs/`, `AGENTS.md`); no product/code changes.
- Source of truth: this session + current repo.
- Follow `CONTRIBUTING.md` style.

What to capture
1. Task recap (2-5 sentences, general terms)
2. Repeatable playbook
3. Commands + success signals
4. Gotchas and failure modes
5. High-leverage tips

Where to update
- Prefer existing: `docs/onboarding/ci-cd.md`, `docs/onboarding/building.md`, etc.
- Keep `AGENTS.md` concise (pointers only).

Improvement modes (pick what fits)
- **Verify**: Check existing docs are still accurate after code changes.
- **Add**: New pattern not yet documented.
- **Remove**: Outdated, redundant, or obsolete content.
- **Consolidate**: Merge scattered related content.
- **Restructure**: Improve organization or navigation.
- **Condense**: Shorten verbose sections.

Bias toward verification/removal/consolidation over adding.

Rules
- No ephemeral IDs (branches, PRs, commits, usernames).
- Unverified commands must be labeled "Suggested (Unverified)".
- Write general patterns, not task-specific chronicles.
- 1-3 sentences per topic is ideal; avoid detailed symptom lists.
- Code examples only when the fix pattern is non-obvious.
- Check if code changes invalidated existing docs (e.g., API behavior changed).

Output
- Summary of changes.
- Files edited.
```
