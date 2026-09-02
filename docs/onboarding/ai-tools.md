# AI Tools Compatibility

This document tracks AI coding assistant compatibility with DART's documentation structure.

> **Last Verified**: 2026-09-01. Command/skill/adapter surfaces are
> continuously machine-verified by `pixi run check-ai-commands` in CI. Claude
> Code notes were behaviorally verified on the tested version recorded in
> [Claude Code](#claude-code); OpenCode terminology notes were checked against
> current public docs; Codex notes were checked against the locally tested
> version recorded in [OpenAI Codex](#openai-codex) plus current OpenAI Codex
> docs. Gemini notes remain a manual-reference path.
> **Review Cadence**: Verify when updating tool versions or experiencing unexpected behavior.

## For Collaborators: Tool Selection

| Tool            | Best For                                | Limitations                     |
| --------------- | --------------------------------------- | ------------------------------- |
| **Claude Code** | Full workflow automation, complex tasks | Requires Anthropic subscription |
| **OpenCode**    | Multi-model flexibility, open source    | Commands require separate sync  |
| **Codex**       | Full workflow automation, OpenAI native | Requires OpenAI subscription    |
| **Gemini CLI**  | Quick queries, large context            | No command/skill support        |

### Quick Start by Tool

| Tool            | Setup                                                                        |
| --------------- | ---------------------------------------------------------------------------- |
| **Claude Code** | `CLAUDE.md` auto-loaded; use `/dart-*` commands                              |
| **OpenCode**    | `AGENTS.md` auto-loaded; use `/dart-*` commands; skills in `.claude/skills/` |
| **Codex**       | Trust checkout; `pixi run ai-setup`; use generated `$dart-*` skills          |
| **Gemini CLI**  | Read `GEMINI.md` or `AGENTS.md`; read `.claude/commands/` manually if needed |

---

## Quick Reference

| Tool            | Instructions               | Commands                     | Skills                     |
| --------------- | -------------------------- | ---------------------------- | -------------------------- |
| **Claude Code** | `CLAUDE.md` -> `AGENTS.md` | `.claude/commands/`          | `.claude/skills/`          |
| **OpenCode**    | `AGENTS.md`                | `.opencode/command/`         | `.claude/skills/`          |
| **Codex**       | `AGENTS.md`                | `.agents/skills/`            | `.agents/skills/`          |
| **Gemini CLI**  | `GEMINI.md` -> `AGENTS.md` | `.claude/commands/` manually | `.claude/skills/` manually |

---

## Maintaining AI Docs

### Conventions

| Convention                      | Rule                                                                                                                  |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| **Pointer board**               | `AGENTS.md` stays concise and points to durable docs                                                                  |
| **Terminology owner**           | `docs/ai/terminology.md` owns canonical terms such as capability, workflow source, skill, adapter, MCP tool, and hook |
| **AI-native policy**            | `docs/ai/` owns AI-infra principles, terminology, workflow maps, verification, sessions, and component ownership      |
| **Capability naming**           | `dart-` prefix for the cross-tool capability name (for example `dart-new-task`)                                       |
| **Workflow source naming**      | `.claude/commands/dart-<name>.md` while that directory remains the editable workflow source                           |
| **Domain skill naming**         | `.claude/skills/dart-<name>/SKILL.md` for reusable on-demand Agent Skills                                             |
| **Skill descriptions**          | Start with display name and quote colon values (e.g., `"DART Build: ..."`)                                            |
| **Tool-specific language**      | Use generic terms except in compatibility or routing docs where tool behavior is the subject                          |
| **Placeholders**                | Use `$ARGUMENTS`, `$1`, `$2` for command args                                                                         |
| **Tracked file references**     | Use repo-relative `@file` syntax; home-directory references are only for untracked personal files                     |
| **Generated adapters**          | `.agents/skills/` and `.opencode/command/` are generated entrypoints, not editable sources                            |
| **Manual public path required** | Every AI workflow must map back to public docs and `pixi run ...` commands for non-AI contributors                    |

### @file Import Syntax

The `@path/to/file` syntax tells agents to automatically load referenced files into context.

**Usage in AGENTS.md**:

```markdown
| Task Type | Load These Files             |
| --------- | ---------------------------- |
| Building  | @docs/onboarding/building.md |
```

**Usage in commands/skills**:

```markdown
@AGENTS.md
@docs/onboarding/contributing.md

## Workflow

...
```

**Notes**:

- Paths are relative to repository root
- Imports are NOT evaluated inside code blocks (use backticks to escape)
- Recursive imports are supported (imported files can import other files)
- Tracked project instructions must use repo-relative paths. Home-directory
  references are only for untracked personal files.

### File Ownership

| File/Directory                | Purpose                             | When to Update                                  |
| ----------------------------- | ----------------------------------- | ----------------------------------------------- |
| `AGENTS.md`                   | Root pointer board                  | When workflows or gates change                  |
| `docs/ai/`                    | Durable AI-native policy            | When AI workflow policy changes                 |
| `docs/ai/terminology.md`      | Canonical AI-facing terms           | When names, roles, or adapter structure change  |
| `docs/ai/capabilities.json`   | Capability status/category          | When workflows or skills change                 |
| `CLAUDE.md`, `GEMINI.md`      | Redirects only                      | Rarely (keep minimal)                           |
| `.claude/commands/`           | Editable workflow source            | When adding workflows                           |
| `.opencode/command/`          | Generated OpenCode command adapters | Auto-synced from `.claude/`                     |
| `.claude/skills/`             | Editable domain-skill source        | When adding domain knowledge                    |
| `.agents/skills/`             | Generated Codex skill adapters      | Auto-generated from `.claude/`                  |
| `.codex/config.toml`          | Codex project delegation policy     | When concurrency or delegation depth changes    |
| `.codex/agents/`              | Codex project agent profiles        | When a stable specialist responsibility changes |
| `.codex/hooks.json`           | Codex project command hooks         | When a lifecycle trigger or hook gate changes   |
| `docs/onboarding/ai-tools.md` | Tool compatibility details          | When tool compatibility changes                 |

### Adding a Command or Skill

`docs/ai/components.md` owns the steps: add the `.claude/commands/dart-<name>.md`
or `.claude/skills/dart-<name>/SKILL.md` source, add the catalog rows in
`docs/ai/workflows.md` and `docs/ai/capabilities.json`, then run
`pixi run sync-ai-commands` and `pixi run check-ai-commands` (both are part of
`pixi run lint`). Keep command files concise and action-oriented and put
background material in `docs/onboarding/*.md`. Codex does not use project
slash-command files directly; the sync script generates a `$dart-*` skill
adapter from each workflow source, so `/dart-fix-ci` becomes `$dart-fix-ci`.

### Skill Design Principles

Skills and subfolder `AGENTS.md` files serve different purposes. Understanding this prevents duplication.

#### Skills vs Subfolder AGENTS.md

| Type                    | Purpose                   | When Loaded                   | Content Style                       |
| ----------------------- | ------------------------- | ----------------------------- | ----------------------------------- |
| **SKILL.md**            | On-demand quick reference | Agent explicitly loads skill  | Lightweight, commands, pointers     |
| **Subfolder AGENTS.md** | Module-specific context   | Auto-loaded when in directory | Module architecture, file locations |

#### Design Rules

1. **Skills are lightweight** — Quick commands and common patterns only
2. **Skills point to full docs** — "For complete guide: `docs/onboarding/X.md`"
3. **Subfolder AGENTS.md is source of truth** — Module-specific details live there
4. **No duplication** — Skills reference docs, don't copy content

#### Skill Template (Follow Existing Pattern)

Follow the pattern of `dart-build` and `dart-test`:

```markdown
---
name: dart-<name>
description: "DART <Name>: brief description for skill discovery"
---

# Skill Title

Load this skill when [trigger condition].

## Quick Commands

\`\`\`bash
pixi run <relevant-command>
\`\`\`

## Full Documentation

For complete guide: `docs/onboarding/<relevant>.md`

For module details: `<module>/AGENTS.md`

## Common Issues

| Issue | Solution |
| ----- | -------- |
| ...   | ...      |

## Key Files

- `path/to/main/file`
- `path/to/tests`
```

#### Cross-Agent Compatibility

Skills work across multiple AI tools through automatic syncing:

| Source              | Target                   | Tool                          |
| ------------------- | ------------------------ | ----------------------------- |
| `.claude/skills/`   | (native)                 | Claude Code, OpenCode         |
| `.claude/skills/`   | `.agents/skills/`        | Codex                         |
| `.claude/commands/` | `.agents/skills/dart-*/` | Codex workflow skill adapters |

**Sync is automatic** via `pixi run lint` (includes `sync-ai-commands`).

**CI verification**: `pixi run check-ai-commands` is the non-mutating sync and
AI-component check. `docs/ai/components.md` owns the exact check coverage.

#### Acknowledgment

The SKILL.md format is inspired by [OpenSkills](https://github.com/numman-ali/openskills) (Apache 2.0).
DART skills are original content under BSD 2-Clause license.

### Keeping Commands and Skills in Sync

Workflow sources, domain skills, and generated adapters exist in multiple
directories because tools don't share one invocation surface. The `.claude/`
directory is the current editable source for workflow sources and domain
skills. Generated Codex/OpenCode files are first-class adapter entrypoints for
their tools, but they are overwritten by sync.

**Automated sync** (included in `pixi run lint`):

```bash
pixi run lint               # Includes sync-ai-commands (recommended)
pixi run sync-ai-commands   # Sync commands + skills to all tool directories
pixi run check-ai-commands  # Check adapter parity (CI mode, no changes)
pixi run check-ai-infra     # Check runtime, references, and scenarios
```

**What gets synced**:

| Source              | Target                   | Purpose                       |
| ------------------- | ------------------------ | ----------------------------- |
| `.claude/commands/` | `.opencode/command/`     | OpenCode command adapters     |
| `.claude/commands/` | `.agents/skills/dart-*/` | Codex workflow skill adapters |
| `.claude/skills/`   | `.agents/skills/`        | Codex domain-skill adapters   |

Generated Codex skill adapters are adapter entrypoints even when
their editable workflow source currently lives in `.claude/commands/`.

**Durable AI-native decisions**:

- Keep `AGENTS.md` as the concise pointer board, `docs/ai/` as shared agent
  policy, and this document as the tool-compatibility reference.
- Keep canonical terms in `docs/ai/terminology.md`; do not re-define
  capability, workflow source, skill, adapter, MCP tool, hook, or subagent in
  each workflow.
- Keep AI-infra axioms and the manual principle audit in
  `docs/ai/principles.md`; link to it from entrypoints and workflows instead of
  restating it.
- Keep public contributor paths available through tracked docs and
  `pixi run ...` tasks; AI workflows can route work, but must not be the only
  way to complete it.
- Treat `docs/dev_tasks/<task>/` folders as temporary working state. When the
  task completes, move only durable decisions into the owner selected by
  `docs/information-architecture.md` and delete the task folder in the same
  PR.

**Effective capability parity**:

Different tools expose the same workflows differently:

- Claude Code: `.claude/commands/` plus `.claude/skills/`
- OpenCode: `.opencode/command/` plus `.claude/skills/`
- Codex: `.agents/skills/` for both domain-skill and workflow skill adapters

`pixi run check-ai-commands` compares those effective sets and runs the
structural AI-component checks owned by `docs/ai/components.md`.

**Sync details**:

- Synced files get an auto-generated header (placed AFTER frontmatter to preserve tool parsing)
- Generated target directories (`.agents/skills/`, `.opencode/command/`) are
  excluded from prettier to prevent re-sync loops. Maintained `.codex/` TOML
  and JSON remain linted.
- Edit source files only; synced files are overwritten on each sync
- NEW generated files can be silently skipped by plain `git add` when a
  personal global gitignore excludes a parent directory (for example
  `.agents`): git cannot re-include files under an excluded directory, and
  `check-ai-commands` compares disk state, so the local check stays green
  while CI fails on the missing adapter in a fresh clone. After adding a
  command or skill, stage new adapter directories with `git add -f` and
  confirm with `git ls-files .agents/skills/<name>`

**Manual fallback**:

If adapter sync is unavailable, fix or run the generator instead of manually
maintaining generated files. `.agents/skills/` and `.opencode/command/` outputs
are overwritten by `pixi run sync-ai-commands`; edit `.claude/` source files,
then regenerate and verify with `pixi run check-ai-infra`. Maintained `.codex/`
configuration is never overwritten by adapter sync.

Manual-only tools should read the `.claude/commands/dart-*.md` source files
directly. There is no separate prompt-template folder.

### Review Cadence

| Check                    | Frequency                      |
| ------------------------ | ------------------------------ |
| Tool compatibility       | When updating tool versions    |
| Capability functionality | After creating new ones        |
| Doc accuracy             | Quarterly or when issues arise |

---

## Detailed Compatibility

### Claude Code

**Tested Versions**: Claude Code CLI 2.1.257 on Claude Fable 5.1
(`claude-fable-5-1`), 2026-09-01 (earlier exercised on 2.1.252 with Fable 5,
2026-08-31, and on 2.1.220, 2026-07-31) — `/dart-model-upgrade` exercised end
to end: command and skill loading, the PreToolUse hook, native image review of
`agent-capture` output, and a controlled Fable 5 versus Fable 5.1 comparison
through fresh `claude -p` sessions (the CLI auto-updated to 2.1.258 during
that session; the update itself was not separately exercised). Opus 5 routing
and image guidance is sourced from current official docs; that lane was not
separately exercised.
Capability and adapter surfaces stay continuously machine-verified via
`pixi run check-ai-commands` in CI.

| Feature      | Location                    | Status                             |
| ------------ | --------------------------- | ---------------------------------- |
| Instructions | `CLAUDE.md`                 | ✅ Reads, redirects to `AGENTS.md` |
| Commands     | `.claude/commands/*.md`     | ✅ `/dart-*` commands available    |
| Skills       | `.claude/skills/*/SKILL.md` | ✅ On-demand loading               |

**Notes**:

- Does NOT read `.opencode/` directory
- Commands use `$ARGUMENTS` for user input
- Skills require YAML frontmatter with `name` and `description`
- Claude Fable 5.1 requires Claude Code 2.1.255 or newer; Fable 5 needs
  2.1.170 or newer
- For `/goal`, put the canonical command in the goal text, such as
  `/goal Run /dart-ultrawork with: <task>; done when: ...`. If goal text starts
  with `ulw:` or the common typo `ultrawok:`, normalize it to the canonical
  `/dart-ultrawork` workflow. These are prompt-level shorthands, not separate
  shared capabilities.
- Model and reasoning routing for current Claude models lives in
  `docs/ai/README.md` § "Model Routing"; do not duplicate or pin it here.
- For controlled model comparisons, `claude -p --model <id> [--effort <level>]`
  with the prompt on stdin runs a fresh non-interactive session that still
  loads the repository instructions and hooks. `--bare` drops hooks,
  `CLAUDE.md` discovery, and auto-memory and authenticates only with an API
  key or `apiKeyHelper`, so it strips exactly the harness a lane must
  exercise; do not use it for comparison lanes. Constrain lanes with
  `--allowedTools`/`--disallowedTools` and keep every lane on the same prompt,
  tools, and checkout state. Nested sessions also load the project's Claude
  Code auto-memory (`~/.claude/projects/<project>/memory/`), so quarantine
  audit notes written during the run before launching lanes and check each
  transcript for reads of them; a lane that saw the expected answer is not a
  control.

Current references:
[Introducing Claude Fable 5.1 and Claude Mythos 5.1](https://www.anthropic.com/claude-fable-and-mythos-5-1),
[What's new in Claude Fable 5.1](https://platform.claude.com/docs/en/models/fable-5-1/whats-new-fable-5-1),
[Migrating to Claude Fable 5.1](https://platform.claude.com/docs/en/models/fable-5-1/migration-guide),
[Prompting Claude Fable 5.1](https://platform.claude.com/docs/en/build-with-claude/prompt-engineering/prompting-claude-fable-5-1),
[Claude Code model configuration](https://code.claude.com/docs/en/model-config),
[Claude Fable 5 (legacy, still served)](https://platform.claude.com/docs/en/models/fable-5/overview), and
[Claude models overview](https://platform.claude.com/docs/en/models/overview).

### OpenCode

**Verified**: generated command adapters continuously via `pixi run
check-ai-commands` in CI; behavior notes checked against current OpenCode docs
on 2026-09-01 with OpenCode 1.18.21 installed

| Feature      | Location                    | Status                              |
| ------------ | --------------------------- | ----------------------------------- |
| Instructions | `AGENTS.md`                 | ✅ Primary entry point              |
| Commands     | `.opencode/command/*.md`    | ✅ `/dart-*` commands available     |
| Skills       | `.claude/skills/*/SKILL.md` | ✅ Claude-compatible path supported |

**Notes**:

- Discovers skills from `.opencode/skills/`, `.claude/skills/`, and
  `.agents/skills/` (project and home directories)
- Does NOT read `.claude/commands/`. Current OpenCode docs name
  `.opencode/commands/`; the installed 1.18.21 client also accepts DART's
  generated `.opencode/command/` (both spellings are in the binary). Move the
  generated path when the singular form stops loading.
- Commands support frontmatter: `description`, `agent`, `model`

### Gemini CLI

**Verified**: manual-reference path only (no generated adapter); behavior notes
hand-checked 2026-07

| Feature      | Location                    | Status                             |
| ------------ | --------------------------- | ---------------------------------- |
| Instructions | `GEMINI.md`                 | ✅ Reads, redirects to `AGENTS.md` |
| Commands     | `.claude/commands/*.md`     | Manual reference only              |
| Skills       | `.claude/skills/*/SKILL.md` | Manual reference only              |

**Notes**:

- Use `GEMINI.md` or `AGENTS.md` as context
- No slash command support; read `.claude/commands/dart-*.md` manually when a workflow recipe is needed

### OpenAI Codex

**Tested Versions**: Codex CLI 0.151.0 (strict-config compatibility and local
discovery/config/hook checks), 2026-08-31 (earlier exercised on 0.144.1 and
0.146.0, 2026-07-29). Codex 0.152.0 is installed as of 2026-09-01 but was not
re-exercised (weekly usage limit); its `--help` lists a `doctor` subcommand.

| Feature           | Location                    | Status                                |
| ----------------- | --------------------------- | ------------------------------------- |
| Instructions      | `AGENTS.md`                 | ✅ Root-to-CWD scoped discovery       |
| Skills            | `.agents/skills/*/SKILL.md` | ✅ `$dart-*` skills available         |
| Workflow adapters | `.agents/skills/dart-*/`    | ✅ Generated from `.claude/commands/` |
| Project agents    | `.codex/agents/*.toml`      | ✅ Bounded read-only specialists      |
| Project hooks     | `.codex/hooks.json`         | ✅ Fast Bash pre-tool guard           |

**Setup and diagnosis**:

1. Trust the checkout so project `.codex/` layers may load; run `codex doctor`
   when the installation, config, auth, or runtime itself looks unhealthy.
2. Run `pixi run ai-setup` to synchronize generated adapters and install the
   cross-tool Git hook.
3. Run `pixi run ai-doctor`. Resolve every reported missing or stale surface.
4. Open `/hooks`, review the exact project hook definition, and trust it if it
   matches the tracked file. Changed definitions require review again.
5. Use `$dart-*` skills; use `dart_scout`, `dart_reviewer`, or
   `dart_release_auditor` only for the bounded read-only contracts documented
   in `docs/ai/orchestration.md`.

Codex walks instruction files from repository root to the current directory;
the closest `AGENTS.md` augments or overrides broader guidance. Skills use
`$skill-name` syntax, including workflow-derived `$dart-*` adapters. CLI slash
commands are built-in session controls, not repository workflows. For goal
mode, put the generated adapter in the goal text, such as
`/goal $dart-ultrawork <task>`.

Use the current model and reasoning guidance in `docs/ai/README.md`; do not
duplicate or pin it here. Project agents inherit the active parent model.
`.codex/config.toml` bounds concurrency with `agents.max_threads` (Codex's
documented alias for `agents.max_concurrent_threads_per_session`; both spellings
load on the tested client and `check-ai-infra` accepts either, exactly one) and
delegation depth with `agents.max_depth`, while progressively loaded skills and
owner docs supply task procedures.

Project hooks are trusted-project automation, not complete enforcement.
`PreToolUse` does not intercept every possible mutation path, and a hook may be
skipped until trusted. The Codex hook therefore runs only the bounded,
noninteractive `check-agent-hook`; the installed Git hook and explicit
pre-commit/full gates remain authoritative.

On native Windows, `.claude/hooks/pre-commit-guard.ps1` launches
`scripts/pretool_guard_bridge.py`, which forwards the unchanged hook payload to
the same Git Bash guard used on POSIX; commit classification has one shared
implementation.
The launcher must consume both script pipeline input (`$input`) and direct
console input: `commandWindows` already reads the hook JSON before invoking the
`.ps1` script, so relying only on `[Console]::In` can turn valid payloads into
empty or invalid JSON. Windows smoke tests should assert bridge exit semantics
for valid, malformed, and raw-stdin payloads; nested PowerShell command output
capture is not a reliable oracle for the child hook's stderr.

Current references:
[GPT-5.6 model guidance](https://developers.openai.com/api/docs/guides/model-guidance?model=gpt-5.6),
[GPT-5.6 prompt guidance](https://developers.openai.com/api/docs/guides/prompt-guidance-gpt-5p6),
[Codex models and reasoning](https://learn.chatgpt.com/docs/models),
[Agent Skills](https://learn.chatgpt.com/docs/build-skills),
[subagents](https://learn.chatgpt.com/docs/agent-configuration/subagents),
[project configuration](https://learn.chatgpt.com/docs/config-file/config-advanced#project-config-files-codexconfigtoml),
[configuration reference](https://learn.chatgpt.com/docs/config-file/config-reference#configtoml), and
[hooks](https://learn.chatgpt.com/docs/hooks).

---

## Directory Structure

```
.claude/                   # Editable source for workflows and domain skills
├── commands/              # Claude Code commands
│   └── dart-*.md
└── skills/                # Claude + OpenCode skills
    └── dart-*/SKILL.md

.opencode/                 # Generated OpenCode command adapters
└── command/               # OpenCode command adapter entrypoints
    └── dart-*.md

.agents/                   # Portable generated Agent Skills
└── skills/                # Codex domain + workflow skill adapters
    └── dart-*/SKILL.md    # Includes workflow-derived $dart-* adapters

.codex/                    # Maintained Codex project runtime
├── config.toml            # Bounded project agent settings
├── agents/                # Read-only specialist profiles
└── hooks.json             # Trusted-project fast command guard
```

---

## Verification Checklist

When verifying compatibility:

1. Run `pixi run ai-doctor` and inspect every reported surface.
2. Run `pixi run check-ai-infra` and `pixi run test-ai-infra`.
3. Run `pixi run exercise-agent-scenarios` for routing/profile changes.
4. Test one generated workflow and domain skill in each supported tool.
5. Verify current tool documentation for discovery/config/hook changes.
6. Update the tested version, evidence, and date in this document.

---

## Failure Recovery And Branch Differences

| Symptom                                | Recovery                                                                                                                        |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `$dart-*` skill missing or stale       | Run `pixi run ai-doctor`, then `pixi run sync-ai-commands` and `pixi run check-ai-infra`                                        |
| Project agents or hooks do not appear  | Confirm the checkout is trusted; inspect `/hooks`; validate `.codex/` with `pixi run check-ai-infra`                            |
| Frequent hook blocks unexpectedly      | Run `pixi run check-agent-hook` directly; inspect JSON/input diagnostics; use the documented emergency bypass only if necessary |
| Full validation fails after quick gate | Select the task-specific focused/full gates in `docs/ai/verification.md`; the fast hook is not completion evidence              |
| A documented command/path is absent    | Confirm the current branch; run `pixi run ai-doctor`; fix the source owner rather than adding an unverified alias               |
| Generated file differs                 | Edit `.claude/` source, regenerate, and never patch `.agents/skills/` or `.opencode/command/` directly                          |

`main` is DART 7: C++23, nanobind, `dart::io`, the clean-break architecture,
CUDA validation, planning packets, benchmark packets, and DART 7 verification
skills belong there. `release-6.20` is DART 6: C++17, pybind11,
`dart::utils`, OSG, Gazebo compatibility, and release-maintenance workflows
belong there. The release catalog is intentionally smaller. Common AI-infra
changes use an apply/adapt/omit audit and branch-local gates; never copy a task,
path, command, or toolchain fact merely because it exists on the other branch.

---

## Personal Preferences (CLAUDE.local.md)

Contributors can create personal instruction files that are gitignored:

| File                  | Scope        | Purpose                               |
| --------------------- | ------------ | ------------------------------------- |
| `CLAUDE.local.md`     | Project root | Project-specific personal preferences |
| `~/.claude/CLAUDE.md` | All projects | Global personal preferences           |

**Example `CLAUDE.local.md`**:

```markdown
# Personal Preferences

- I prefer verbose explanations
- Always show full file paths in responses
- Use tabs, not spaces (personal editor setting)
```

**Notes**:

- These files are NOT checked into git
- Use for IDE preferences, debug verbosity, personal aliases
- Project settings in `AGENTS.md` take precedence for shared conventions

---

## Handling Automated Reviews

Automated-review handling (bot detection, the review-fix loop, Codex
re-trigger cadence, draft-ready criteria, base-branch merges, and thread
resolution) lives in [ai-reviews.md](ai-reviews.md).

## Known Limitations

- **Generated adapter copies**: Claude Code, OpenCode, and Codex read different
  directories, so generated adapter copies exist for tool compatibility.
- **Editable source of truth**: Maintain `.claude/` sources, then run
  `pixi run sync-ai-commands` and `pixi run check-ai-infra`.
- **Skill sharing works for manual tools**: Claude Code and OpenCode can read
  `.claude/skills/` directly.

---

## Changelog

| Date     | Change                                                                                                                                                               |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jan 2025 | Initial setup with Claude Code, OpenCode, Gemini CLI, Codex support                                                                                                  |
| Jan 2025 | Added collaborator guide and maintenance conventions                                                                                                                 |
| Jul 2026 | Refreshed verification metadata to CI-checked adapter sync; added independent review lane                                                                            |
| Jul 2026 | Migrated Codex skills, added project agents/hooks, diagnosis, scenarios, and branch profiles                                                                         |
| Jul 2026 | Fable 5 audit: Claude 5 lane added to the model-routing owner, capability-based image-review wording, refreshed Claude Code verification                             |
| Sep 2026 | Fable 5.1 audit: Claude lane advanced to Fable 5.1 in the routing owner, model names consolidated into that owner, frontmatter pin check, `claude -p` lane mechanics |
