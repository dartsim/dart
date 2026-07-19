# .codex/

Maintained Codex runtime configuration for DART 6.20.

- Do not pin a model in project config; inherit the maintainer's session model.
- Keep custom agents few, bounded, read-only, and explicit about inputs/output.
- Keep hooks deterministic, non-interactive, offline, and below 30 seconds.
- Treat the Codex PreToolUse hook as advisory; `pixi run install-hooks` provides
  the cross-tool git enforcement path.
- Run `pixi run python scripts/check_ai_infrastructure.py --check` after
  changing config, agents, or hooks.
