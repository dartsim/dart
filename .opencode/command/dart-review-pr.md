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

## AI-Generated Reviews

See `docs/onboarding/ai-tools.md` § "Handling Automated Reviews" — do NOT reply directly.
