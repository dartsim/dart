---
description: Review PR or address review feedback
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

Apply minimal fixes, verify, push.

## AI-Generated Reviews (Codex, Copilot, etc.)

1. Push fix silently (no reply)
2. Resolve thread via GraphQL (see ai-tools.md)
3. Re-trigger: `gh pr comment $1 --body "@codex review"`
4. Monitor CI: `gh pr checks $1`
5. Check for new review, repeat until no comments + CI green

Full iterative loop: `docs/onboarding/ai-tools.md` § "Autonomous Review-Fix-Monitor Loop"
