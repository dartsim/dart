---
description: review a PR or address review feedback
agent: build
---

Review or respond to PR: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/code-style.md
@docs/onboarding/ai-tools.md (for AI-generated review handling)

## To Review

```bash
gh pr view $1 && gh pr diff $1
```

Check: code style, tests, docs, focused commits

## To Address Feedback

```bash
gh pr view $1 --comments
```

Apply minimal fixes locally and verify. Do not push, comment, resolve threads,
or re-trigger review without explicit maintainer/user approval for that
external mutation.

## AI-Generated Reviews (Codex, Copilot, etc.)

1. Make the local fix silently (no reply)
2. Run the relevant local gates, including `pixi run lint` before any commit
3. Ask for explicit maintainer/user approval before push, thread resolution,
   PR comment, or review re-trigger
4. If approved, push the fix silently
5. If approved, resolve only reviewed and addressed thread IDs via GraphQL (see
   ai-tools.md)
6. If approved, re-trigger: `gh pr comment $1 --body "@codex review"`
7. Monitor CI: `gh pr checks $1`
8. Check for new review, repeat until no comments + CI green

Full iterative loop: `docs/onboarding/ai-tools.md` § "Autonomous Review-Fix-Monitor Loop"
