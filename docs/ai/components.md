# AI Components

The DART 6.20 release branch uses a small cross-agent workflow surface:

- `.claude/commands/`: editable workflow command sources.
- `.claude/skills/`: editable domain-skill sources.
- `.opencode/command/`: generated OpenCode command files.
- `.codex/skills/`: generated Codex workflow and domain skills.
- `scripts/sync_ai_commands.py`: sync and validation tool.
- `docs/ai/capabilities.json`: machine-readable workflow inventory.
- `docs/ai/workflows.md`: human-readable workflow map and gates.
- `docs/information-architecture.md`: release-branch docs placement owner.

AI docs are agent context, not a dumping ground. Keep always-loaded entrypoints
compact and make rules visible through owner placement, read-order pointers,
workflow required reading, and generated-adapter sync instead of duplication.

Use `.claude/` as the editable source. Do not hand-edit generated `.codex/` or
`.opencode/` files; rerun the sync script instead.

Use `dart-retro` after a completed release-branch session only when the
learning is general enough to improve future DART 6.20 maintenance work. Skip
routine work, one-off local choices, and review-only narrative.
