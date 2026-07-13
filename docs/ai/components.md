# AI Components

The DART 6.20 release branch uses a small cross-agent workflow surface:

- `.claude/commands/`: editable workflow command sources.
- `.claude/skills/`: editable domain-skill sources.
- `.opencode/command/`: generated OpenCode command files.
- `.agents/skills/`: generated Codex workflow and domain skills, with a
  DART-owned manifest that preserves unrelated skills.
- `.codex/config.toml`: bounded project-local Codex concurrency defaults.
- `.codex/agents/`: maintained read-only specialist profiles.
- `.codex/hooks.json`: fast advisory Codex commit-command hook.
- `.claude/hooks/pre-commit-guard.ps1` and
  `scripts/pretool_guard_bridge.py`: native-Windows forwarding into the shared
  Git Bash guard.
- `scripts/sync_ai_commands.py`: sync and validation tool.
- `scripts/check_ai_infrastructure.py`: doctor, drift, and scenario checker.
- `docs/ai/capabilities.json`: machine-readable workflow inventory.
- `docs/ai/branch-profile.json`: machine-readable DART 6.20 facts, required
  surfaces, and DART 7 exclusions.
- `docs/ai/agent-scenarios.json`: seven deterministic contracts covering
  orientation, small changes, failure diagnosis, docs, components, simulation
  verification, and release maintenance. The simulation route machine-checks
  the text-first plus claim-tied visual evidence policy.
- `docs/ai/workflows.md`: human-readable workflow map and gates.
- `docs/ai/terminology.md`: canonical AI-facing vocabulary.
- `docs/ai/orchestration.md`: work-packet and orchestrator/executor contract.
- `docs/plans/dashboard.md`: release-branch operating plan state.
- `docs/information-architecture.md`: release-branch docs placement owner.
- `docs/onboarding/architecture.md`: release component layering and
  compatibility-boundary owner.

AI docs are agent context, not a dumping ground. Keep always-loaded entrypoints
compact and make rules visible through owner placement, read-order pointers,
workflow required reading, and generated-adapter sync instead of duplication.

Use `.claude/` as the editable source. Do not hand-edit generated `.agents/` or
`.opencode/` files; rerun the sync script instead. `.codex/` is maintained
runtime configuration, not generated adapter output.

Use `dart-retro` after a completed release-branch session only when the
learning is general enough to improve future DART 6.20 maintenance work. Skip
routine work, one-off local choices, and review-only narrative.
