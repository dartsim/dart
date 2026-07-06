---
type: ai-terminology
owner: self
---

# AI Terminology

This file owns DART's canonical language for AI-facing repository surfaces.
Use it when editing `AGENTS.md`, `docs/ai/`, `docs/onboarding/ai-tools.md`,
`.claude/commands/`, `.claude/skills/`, generated adapter docs, or AI sync
checks.

DART uses one extra internal abstraction: **capability**. A capability is the
tool-neutral DART unit named `dart-*`. Each capability must have an equivalent
path in every supported agent surface, even when those tools expose it as a
slash command, a skill, or a manual recipe.

## Canonical Terms

| Term                         | Use for                                                                                                                              | Do not use for                                                                       | Current DART owner                                     |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------ | ------------------------------------------------------ |
| Agent                        | A coding-assistant worker session that reasons over context, calls tools, and completes a task.                                      | GitHub bot comments or generated files.                                              | `AGENTS.md`, `docs/onboarding/ai-tools.md`             |
| Instruction file             | Persistent project guidance loaded into agent context, especially `AGENTS.md` and tool redirects such as `CLAUDE.md` or `GEMINI.md`. | Reusable procedures that should load on demand.                                      | `AGENTS.md`, module `AGENTS.md` files                  |
| Capability                   | A DART `dart-*` workflow or domain skill as seen across supported tools.                                                             | A specific file format, command syntax, or runtime tool call.                        | `docs/ai/workflows.md`, `docs/ai/capabilities.json`    |
| Workflow                     | A repeatable DART procedure with required reading, steps, output, and gates.                                                         | General domain background or one-off task state.                                     | `.claude/commands/`, `docs/ai/workflows.md`            |
| Workflow source              | The editable source file for a user-invoked DART workflow.                                                                           | Generated tool adapters.                                                             | `.claude/commands/`                                    |
| Domain skill                 | On-demand DART domain knowledge or procedure, usually a lightweight pointer to full docs.                                            | Workflow commands with task lifecycle steps.                                         | `.claude/skills/`                                      |
| Agent Skill                  | A portable `SKILL.md` package with metadata, instructions, and optional scripts or references.                                       | MCP tools, slash commands, or always-loaded instructions.                            | `.claude/skills/`, generated Codex skill adapters      |
| Slash command                | A leading-slash UI invocation, such as `/dart-next` in Claude Code or OpenCode.                                                      | Codex project workflows, because Codex slash commands are built-in session controls. | `.claude/commands/`, `.opencode/command/`              |
| Generated adapter entrypoint | A tool-specific file generated from an editable DART source so a supported agent can invoke the same capability.                     | An editable source of truth.                                                         | `.codex/skills/`, `.opencode/command/`                 |
| MCP server                   | External provider of tools, resources, prompts, or context through Model Context Protocol.                                           | Local docs, workflow files, or generated adapters.                                   | Tool configuration outside this repo unless checked in |
| MCP tool                     | Callable action exposed by an MCP server.                                                                                            | Shell commands, Pixi tasks, DART workflow capabilities, or skill instructions.       | MCP server metadata                                    |
| Hook                         | Deterministic lifecycle automation that runs at configured agent events.                                                             | Prompt-based reusable workflows.                                                     | `.claude/hooks/`, tool settings                        |
| Subagent                     | A separate worker context used for isolated or delegated work.                                                                       | A DART capability, workflow, or skill.                                               | `docs/ai/orchestration.md`                             |
| Orchestrator                 | The role that decomposes, sequences, and reviews work packets.                                                                       | A specific AI product.                                                               | `docs/ai/orchestration.md`                             |
| Executor                     | The role that implements one packet and returns evidence.                                                                            | A specific AI product.                                                               | `docs/ai/orchestration.md`                             |
| Work packet                  | A bounded handoff unit in a numbered plan or dev-task surface.                                                                       | A GitHub issue, PR, branch, or entire roadmap item.                                  | `docs/ai/orchestration.md`, numbered plans             |
| Gate                         | A command or check that can fail and directly supports the completion claim.                                                         | Informal confidence.                                                                 | `docs/ai/verification.md`                              |
| Evidence                     | File state, command output, review state, benchmark data, or artifact proving a requirement.                                         | Intent, plausible reasoning, or an uninspected broad check.                          | `docs/ai/verification.md`                              |

## Preferred Phrasing

Use these replacements when editing AI-facing docs:

| Avoid when ambiguous      | Prefer                                                                                  |
| ------------------------- | --------------------------------------------------------------------------------------- |
| command/skill surface     | capability surface                                                                      |
| Codex slash command       | Codex `$dart-*` skill adapter                                                           |
| Claude command source     | workflow source in `.claude/commands/`                                                  |
| Codex skill source        | generated Codex adapter, unless the file is an editable domain skill source             |
| AI tool files             | AI adapter entrypoints, workflow sources, or domain skill sources                       |
| bot review                | automated review bot comment                                                            |
| tool, for everything      | command for shell/Pixi, MCP tool for MCP actions, capability for DART workflows         |
| memory, for project rules | instruction file or persistent guidance                                                 |
| task, for every unit      | task for user objective, work packet for plan handoff, capability for reusable workflow |

## Standards Alignment

The terminology above is intentionally close to current public agent
conventions:

- `AGENTS.md` is the project instruction file for coding agents, matching the
  public AGENTS.md convention and Codex's documented project-instruction
  discovery.
- `SKILL.md` packages are Agent Skills: on-demand, progressively loaded
  capabilities with `name` and `description` metadata, optional scripts, and
  optional references.
- Claude Code now treats custom command files and skills as slash-invoked
  skills, while keeping `.claude/commands/` compatible.
- OpenCode keeps custom commands and Agent Skills as separate surfaces.
- Codex uses skills for repo workflows and reserves slash commands for built-in
  session controls.
- MCP is the integration standard for external tools and context. Do not rename
  local DART workflows or Pixi commands as MCP tools unless they are actually
  exposed by an MCP server.

## Current Compatibility Contract

DART currently keeps `.claude/commands/` as the editable workflow source because
Claude Code and OpenCode expose those workflows through `/dart-*` command
entrypoints. The sync script renders equivalent generated adapters for Codex.

DART keeps `.claude/skills/` as the editable domain-skill source. The sync
script renders equivalent generated Codex skill adapters.

Generated `.codex/` and `.opencode/` files are first-class entrypoints for
their tools, but not sources of truth. Change the editable source, then run the
AI adapter sync checks.

## Migration Candidates

Do not perform these migrations opportunistically during ordinary docs work.
Track them as explicit AI-infra changes because they affect discovery,
generated files, and CI checks:

1. Evaluate moving Codex repo-skill output from `.codex/skills/` to the current
   `.agents/skills/` convention. Avoid checking in duplicate skill names under
   both directories unless the active Codex version proves it de-duplicates or
   scopes them clearly.
2. Evaluate moving reusable workflow authoring from `.claude/commands/` to an
   Agent Skills source once Claude Code, OpenCode, and Codex all expose the
   same behavior from that source without weakening slash-command ergonomics.
3. Add MCP only for external systems or deterministic local services that need
   structured callable actions or resources. Do not add MCP as a wrapper around
   plain docs links or Pixi commands.
