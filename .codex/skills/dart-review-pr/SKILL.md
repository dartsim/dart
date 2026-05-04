---
name: dart-review-pr
description: "DART Review PR: review a PR or address review feedback"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-review-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-review-pr

Use this skill in Codex when you want the same workflow that Claude Code and
OpenCode expose as `/dart-review-pr`.

## Invocation

- Claude Code/OpenCode: `/dart-review-pr <arguments>`
- Codex: `$dart-review-pr <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

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
