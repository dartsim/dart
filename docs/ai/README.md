# DART 6.20 AI Workflows

This directory contains the release-branch AI workflow map used by Codex,
Claude Code, and OpenCode. It is intentionally smaller than the DART 7 `main`
workflow surface and focuses on the workflows needed to maintain the DART 6 LTS
line.

## Visibility And Context Budget

`AGENTS.md`, this start list, and workflow `Required Reading` blocks are the
visibility contract for release-branch agents. Do not copy the same rule into
every workflow; move it to the owner doc, then add only the pointer needed for
the workflow to load it.

Always-loaded surfaces such as `AGENTS.md`, `docs/ai/principles.md`, and
`docs/ai/verification.md` must stay compact. Put procedures, compatibility
details, and examples in the owner docs named by those entrypoints.

When a documented rule is missed, diagnose whether the issue is owner
placement, weak wording, missing required reading, a workflow description, or
generated-adapter sync.

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
