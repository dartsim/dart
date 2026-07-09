# AI Tooling And Review Rules

This release branch supports Claude Code, OpenCode, and Codex workflow
entrypoints generated from `.claude/` sources.

## Source And Generated Files

- Edit workflow commands in `.claude/commands/`.
- Edit domain skills in `.claude/skills/`.
- Do not hand-edit `.codex/skills/` or `.opencode/command/`; run
  `pixi run sync-ai-commands` instead.
- Check parity with `pixi run check-ai-commands`.
- Use canonical DART capability names in prompt-driven goal modes: Claude goal
  text should name `/dart-ultrawork`, and Codex goal text can name
  `$dart-ultrawork`. If Claude goal text starts with `ulw:` or the common typo
  `ultrawok:`, normalize it to the canonical `/dart-ultrawork` workflow. These
  are prompt-level shorthands, not separate shared capabilities.

## Approval Boundaries

The following actions require explicit maintainer/user approval:

- pushing commits;
- opening, editing, marking ready, or merging PRs;
- posting PR or issue comments;
- rerunning CI;
- resolving review threads;
- deleting local or remote branches after explicit maintainer/user approval.

## AI Review Comments

Never reply to AI-generated review comments from bot users such as
`chatgpt-codex-connector[bot]`, `github-code-quality[bot]`,
`github-actions[bot]`, or `copilot[bot]`.

Make fixes silently. After an approved follow-up push, request a new top-level
review only when explicit approval covers the PR comment.

## PR Branches

Before every approved push to a published PR branch, fetch and merge the latest
target base branch into the topic branch. Use merge, not rebase, unless a
maintainer explicitly requests history rewriting.

Never push directly to `release-*` branches. Create a topic branch from the
release base without tracking the release ref:

```bash
git fetch origin release-6.20
git switch --no-track -c <type>/<topic> origin/release-6.20
```

After explicit maintainer/user approval, push the topic branch with the same
local and remote branch name:

```bash
branch=$(git branch --show-current)
# Requires explicit maintainer/user approval.
git push -u origin "HEAD:${branch}"
```
