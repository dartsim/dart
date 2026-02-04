---
description: Review PR or address review feedback
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-review-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

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

Full details: `docs/onboarding/ai-tools.md` § "Handling Automated Reviews"
