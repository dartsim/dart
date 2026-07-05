# DART 6.20 AI Workflows

This directory contains the release-branch AI workflow map used by Codex,
Claude Code, and OpenCode. It is intentionally smaller than the DART 7 `main`
workflow surface and focuses on the workflows needed to maintain the DART 6 LTS
line.

Start with:

- [`principles.md`](principles.md)
- [`workflows.md`](workflows.md)
- [`verification.md`](verification.md)
- [`sessions.md`](sessions.md)
- [`components.md`](components.md)
- [`capabilities.json`](capabilities.json)

Editable workflow sources live in `.claude/commands/` and `.claude/skills/`.
Generated Codex and OpenCode surfaces live in `.codex/skills/` and
`.opencode/command/`. Run `python scripts/sync_ai_commands.py --check` to
verify parity.

For documentation placement, use
[`docs/information-architecture.md`](../information-architecture.md).
