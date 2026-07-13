# AI Terminology

This file owns DART 6.20's canonical language for AI-facing repository
surfaces. Use it when editing `AGENTS.md`, `docs/ai/`,
`docs/onboarding/ai-tools.md`, `.claude/commands/`, `.claude/skills/`, or
generated adapter docs.

## Canonical Terms

| Term                         | Use for                                                                                           | Current owner                                       |
| ---------------------------- | ------------------------------------------------------------------------------------------------- | --------------------------------------------------- |
| Agent                        | A coding-assistant worker session that reasons over context, calls tools, and completes a task.   | `AGENTS.md`, `docs/onboarding/ai-tools.md`          |
| Instruction file             | Persistent project guidance loaded into agent context, especially `AGENTS.md` and tool redirects. | `AGENTS.md`, module `AGENTS.md` files               |
| Capability                   | A DART `dart-*` workflow or domain skill as seen across supported tools.                          | `docs/ai/workflows.md`, `docs/ai/capabilities.json` |
| Workflow                     | A repeatable DART procedure with required reading, steps, output, and gates.                      | `.claude/commands/`, `docs/ai/workflows.md`         |
| Workflow source              | The editable source file for a user-invoked DART workflow.                                        | `.claude/commands/`                                 |
| Domain skill                 | On-demand DART domain knowledge or procedure, usually a lightweight pointer to full docs.         | `.claude/skills/`                                   |
| Generated adapter entrypoint | A tool-specific file generated from an editable DART source.                                      | `.agents/skills/`, `.opencode/command/`             |
| Agent profile                | A bounded Codex subagent contract with explicit input, output, and sandbox limits.                | `.codex/agents/`                                    |
| Project hook                 | Fast advisory automation loaded by a trusted agent client.                                        | `.codex/hooks.json`, `.claude/settings.json`        |
| Orchestrator                 | The role that decomposes, sequences, and reviews work packets.                                    | `docs/ai/orchestration.md`                          |
| Executor                     | The role that implements one packet and returns evidence.                                         | `docs/ai/orchestration.md`                          |
| Work packet                  | A bounded handoff unit in a plan or dev-task surface.                                             | `docs/ai/orchestration.md`                          |
| Gate                         | A command or check that can fail and directly supports the completion claim.                      | `docs/ai/verification.md`                           |
| Evidence                     | File state, command output, review state, benchmark data, or artifact proving a requirement.      | `docs/ai/verification.md`                           |

## Preferred Phrasing

| Avoid when ambiguous  | Prefer                                                                                  |
| --------------------- | --------------------------------------------------------------------------------------- |
| command/skill surface | capability surface                                                                      |
| Codex slash command   | Codex `$dart-*` skill adapter                                                           |
| Claude command source | workflow source in `.claude/commands/`                                                  |
| Codex skill source    | generated Codex adapter, unless the file is an editable domain skill source             |
| AI tool files         | AI adapter entrypoints, workflow sources, or domain skill sources                       |
| tool, for everything  | command for shell/Pixi, capability for DART workflows                                   |
| task, for every unit  | task for user objective, work packet for plan handoff, capability for reusable workflow |

## Compatibility Contract

DART 6.20 keeps `.claude/commands/` as the editable workflow source and
`.claude/skills/` as the editable domain-skill source. Generated
`.agents/skills/` and `.opencode/command/` files are entrypoints, not sources of
truth. `.codex/` contains maintained runtime configuration. Change the editable
source, then run the AI adapter sync checks.
