# AI Tooling And Review Rules

This release branch supports Claude Code, OpenCode, and Codex workflow
entrypoints generated from `.claude/` sources.

## Source And Generated Files

- Edit workflow commands in `.claude/commands/`.
- Edit domain skills in `.claude/skills/`.
- Do not hand-edit `.agents/skills/` or `.opencode/command/`; run
  `pixi run sync-ai-commands` instead.
- Check parity with `pixi run check-ai-commands`.
- Use canonical DART capability names in prompt-driven goal modes: Claude goal
  text should name `/dart-ultrawork`, and Codex goal text can name
  `$dart-ultrawork`. If Claude goal text starts with `ulw:` or the common typo
  `ultrawok:`, normalize it to the canonical `/dart-ultrawork` workflow. These
  are prompt-level shorthands, not separate shared capabilities.

## Codex Project Setup

Current Codex discovers project skills under `.agents/skills/`. It reads
bounded agent profiles from `.codex/agents/`, project defaults from
`.codex/config.toml`, and the advisory PreToolUse hook from
`.codex/hooks.json` after the repository is trusted. Use:

```bash
pixi run python scripts/setup_ai.py
pixi run python scripts/check_ai_infrastructure.py --doctor
```

For large, high-value autonomous work, select Codex 5.6 Sol Ultra in the
session. The project does not pin a model so ordinary maintenance and future
clients inherit the maintainer's choice. The three project agents are
read-only: `dart_scout` gathers evidence, `dart_reviewer` audits the current
diff, and `dart_release_auditor` compares `main` with the DART 6.20 compatibility
surface.

Inspect project hooks with `/hooks`. Project hooks are advisory and may be
skipped in an untrusted repository. `pixi run install-hooks` installs the
cross-tool git hook.

On native Windows, `.claude/hooks/pre-commit-guard.ps1` launches
`scripts/pretool_guard_bridge.py`, which forwards the unchanged hook payload to
the same Git Bash guard used on POSIX; commit classification has one shared
implementation. The manual fallback is:

```bash
pixi run python scripts/check_agent_hook.py --profile staged
```

Both paths are fast safety checks, not a substitute for `pixi run lint` before
commits.

## Other Clients And Manual Fallback

Claude Code uses the editable `.claude/` commands and skills. OpenCode uses the
generated `.opencode/command/` adapters. Codex uses `.agents/skills/` plus the
trusted `.codex/` runtime layer. Gemini and other clients that read
`AGENTS.md` can follow the same owner docs and `pixi run ...` gates without a
tool-specific command surface. Never make correctness depend only on a project
hook or one client's private state; the public docs, direct commands, and
installed git hook remain the fallback contract.

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
