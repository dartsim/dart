# AI Tools Compatibility

This document tracks AI coding assistant compatibility with DART's documentation structure.

> **Last Verified**: 2026-07-11. Command/skill/adapter surfaces are
> continuously machine-verified by `pixi run check-ai-commands` in CI. Claude
> Code and OpenCode terminology notes were checked against current public docs;
> Codex notes were checked against local `codex-cli 0.144.1` discovery plus
> current OpenAI Codex docs. Gemini notes remain a manual-reference path.
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

### Adding a New Command

1. Create `.claude/commands/dart-<name>.md`:

   ```markdown
   ---
   description: brief description
   argument-hint: "<topic>"
   ---

   ## Required Reading

   @AGENTS.md
   @docs/onboarding/relevant-doc.md

   ## Workflow

   1. Step one
   2. Step two

   ## Output

   - Summarize the outcome and validation for `$ARGUMENTS`.
   ```

2. Sync to OpenCode and Codex: `pixi run lint` (includes `sync-ai-commands`)

3. Update `docs/ai/workflows.md` so the command is discoverable from the workflow catalog

4. Update `docs/ai/capabilities.json` so the command is represented in the
   machine-readable capability manifest

5. Put long background material in `docs/onboarding/*.md`; keep command files concise and action-oriented

Codex does not use project slash-command files directly. The sync script
generates a Codex skill adapter from each workflow source, so `/dart-fix-ci`
becomes `$dart-fix-ci`.

### Adding a New Skill

1. Create `.claude/skills/dart-<name>/SKILL.md`:

   ```markdown
   ---
   name: dart-<name>
   description: When to use this skill
   ---

   # Skill Title

   ## Quick Reference

   ...

   ## Full Documentation

   See: `docs/onboarding/relevant-doc.md`
   ```

2. Sync to Codex: `pixi run lint` (includes `sync-ai-commands`)

3. Update `docs/ai/workflows.md` so the skill is discoverable from the workflow catalog

4. Update `docs/ai/capabilities.json` so the skill is represented in the
   machine-readable capability manifest

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

#### Creating New Skills

To add a new DART-specific skill:

1. Create skill directory and file:

   ```bash
   mkdir -p .claude/skills/dart-<name>
   # Create SKILL.md following the template above
   ```

2. Sync to all tool directories:
   ```bash
   pixi run lint  # Includes sync-ai-commands
   ```

**Skill candidates** (create when needed): `dart-dynamics` (articulated bodies), `dart-collision` (collision detection), `dart-architecture` (core design).

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

**Verified**: capability and adapter surfaces continuously via `pixi run
check-ai-commands` in CI; behavior notes hand-checked 2026-07

| Feature      | Location                    | Status                             |
| ------------ | --------------------------- | ---------------------------------- |
| Instructions | `CLAUDE.md`                 | ✅ Reads, redirects to `AGENTS.md` |
| Commands     | `.claude/commands/*.md`     | ✅ `/dart-*` commands available    |
| Skills       | `.claude/skills/*/SKILL.md` | ✅ On-demand loading               |

**Notes**:

- Does NOT read `.opencode/` directory
- Commands use `$ARGUMENTS` for user input
- Skills require YAML frontmatter with `name` and `description`
- For `/goal`, put the canonical command in the goal text, such as
  `/goal Run /dart-ultrawork with: <task>; done when: ...`. If goal text starts
  with `ulw:` or the common typo `ultrawok:`, normalize it to the canonical
  `/dart-ultrawork` workflow. These are prompt-level shorthands, not separate
  shared capabilities.

### OpenCode

**Verified**: generated command adapters continuously via `pixi run
check-ai-commands` in CI; behavior notes hand-checked 2026-07

| Feature      | Location                    | Status                              |
| ------------ | --------------------------- | ----------------------------------- |
| Instructions | `AGENTS.md`                 | ✅ Primary entry point              |
| Commands     | `.opencode/command/*.md`    | ✅ `/dart-*` commands available     |
| Skills       | `.claude/skills/*/SKILL.md` | ✅ Claude-compatible path supported |

**Notes**:

- Reads `.claude/skills/` for Claude compatibility
- Does NOT read `.claude/commands/` (use `.opencode/command/` instead)
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

**Tested Version**: Codex CLI 0.144.1 (local discovery/config/hook checks,
2026-07-11)

| Feature           | Location                    | Status                                |
| ----------------- | --------------------------- | ------------------------------------- |
| Instructions      | `AGENTS.md`                 | ✅ Root-to-CWD scoped discovery       |
| Skills            | `.agents/skills/*/SKILL.md` | ✅ `$dart-*` skills available         |
| Workflow adapters | `.agents/skills/dart-*/`    | ✅ Generated from `.claude/commands/` |
| Project agents    | `.codex/agents/*.toml`      | ✅ Bounded read-only specialists      |
| Project hooks     | `.codex/hooks.json`         | ✅ Fast Bash pre-tool guard           |

**Setup and diagnosis**:

1. Trust the checkout so project `.codex/` layers may load.
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

For complex, high-value autonomous work, prefer Codex 5.6 Sol with Ultra
reasoning when available. The repository does not pin a model: project agents
inherit the active parent model, and smaller work should use the lightest
capable choice. Keep prompts lean, state the objective/constraints/done-when
once, and let progressively loaded skills and owner docs supply procedures.

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

Current references: [models and reasoning](https://learn.chatgpt.com/docs/models),
[Agent Skills](https://learn.chatgpt.com/docs/build-skills),
[subagents](https://learn.chatgpt.com/docs/agent-configuration/subagents),
[project configuration](https://learn.chatgpt.com/docs/config-file/config-advanced#project-config-files-codexconfigtoml), and
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

When AI agents (Claude Code, OpenCode, etc.) work on PRs, they may encounter review comments from other AI systems (e.g., Codex bot, GitHub Copilot).

### Independent Review Lane

For substantive code PRs, an independent reviewer session — a human, or a
separate agent session running `/dart-review-pr` that did not author the
change — records findings before merge approval. Docs-only and mechanical
changes are exempt. This complements `@codex review`; it does not replace it.
`dart-manage-pr` checks this gate in `mode=merge`.

### Detecting AI-Generated Reviews

**Bot usernames always end with `[bot]`:**

- `chatgpt-codex-connector[bot]` — Codex automated reviews
- `github-actions[bot]` — GitHub Actions automated comments
- `github-code-quality[bot]` — GitHub code-quality review comments
- `copilot[bot]` — GitHub Copilot suggestions

**If the reviewer username ends in `[bot]`, treat it as an automated review.**

### Rules for AI Agents (CRITICAL)

**NEVER reply to AI-generated review comments. This means:**

- ❌ **NO** `gh pr comment` commands responding to bot feedback
- ❌ **NO** PR comment replies acknowledging or addressing bot feedback
- ❌ **NO** comments like "Addressed the Codex review feedback"
- ✅ **YES** Make the local code fix silently
- ✅ **YES** Ask for explicit maintainer/user approval before any push, PR
  comment, thread resolution, reviewer request, merge, or review re-trigger

**The code change IS the response. No acknowledgment needed.**

**Guidance for AI agents addressing automated reviews**:

- Address the feedback in code locally, then ask before external mutations
- If the feedback is valid, implement the fix without commenting
- If the feedback appears incorrect (false positive):
  1. **Verify** the claim is false by running standalone tests or examining the code
  2. **Add a test** that explicitly documents the correct behavior AND refutes the claim
  3. Example: If Codex claims `hprod([2,3,5,7])` returns 294 instead of 210:
     ```cpp
     EXPECT_FLOAT_EQ(result, 210.0f);  // Verify correct behavior
     EXPECT_NE(result, 294.0f);        // Explicitly refute false claim
     ```
  4. Add the test locally (no comment needed) - the test serves as permanent documentation
- **Follow the review-fix loop** (see workflow below)

This avoids noisy bot-to-bot conversations while still leveraging automated verification.

> **Note**: False positives can recur across reviews. Tests that explicitly refute incorrect claims prevent future confusion and document the verification.

### Codex Review For Draft PRs

For fast-moving work, trigger the first Codex review while the PR is still a
draft when explicit maintainer/user approval covers PR comments. Use a top-level
comment:

```bash
gh pr comment <PR> --body "@codex review"
```

This keeps the PR draft for human readiness while getting automated feedback
early. If a Codex activity signal or submitted review already appears, do not
post a duplicate trigger. After posting, wait for a submitted review, a no-issues
comment, a thumbs-up reaction, or an eyes reaction before treating the trigger as
accepted; do not re-trigger unless there is a concrete timeout/blocker or a
follow-up push addressed Codex findings.

After an approved follow-up push that addresses Codex review comments, request a
fresh top-level Codex review with `@codex review`. This is the normal completion
step for a Codex review-fix round, not an inline reply. A manual trigger is a PR
comment and still requires explicit maintainer/user approval.

### Draft Ready Fast Path

To move quickly without bypassing branch protection, a draft PR can be marked
ready for review once all of these are true on the current head:

- Codex review has no unresolved actionable threads, or the latest Codex result
  is a no-issues comment/reaction.
- Local validation passed after the last pushed change, and the worktree is
  clean: default `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux
  hosts with a visible NVIDIA CUDA runtime.
- PR metadata is correct: base, milestone, title, template, and testing
  evidence match the current branch.

Hosted CI may still be pending when the draft is marked ready. Merge still waits
for branch protection and required checks unless a maintainer explicitly
approves a policy bypass.

### Codex Re-Trigger Cadence And Throttling

After explicit maintainer/user approval for the PR comment, re-trigger Codex at
most once per review-fix round, and only after an approved push that addressed
its comments. Rapid, repeated `@codex review` requests across many quick rounds
can slow or suspend Codex: observed review latency grows round over round and a
later re-trigger can receive no review at all. If Codex stays silent well beyond
its usual turnaround after a re-trigger, treat it as a throttle/timeout blocker,
not a reason to re-request. Record the converged state as evidence instead — all
surfaced findings fixed and their threads resolved — and report the throttle
rather than re-spamming the PR with more triggers.

### Updating Published PRs

Prefer additive follow-up commits for updates to already-published PRs. This
keeps review history inspectable and makes each review round clear. Pushing any
such update is an external mutation that requires explicit maintainer/user
approval.

#### Merge The Base Branch Before Every Push (MANDATORY)

**Before every push, first merge the latest base branch (usually `main`) into
the working branch.** Do this on every push, not just the first, so each
pushed/CI-tested state reflects current `main` and conflicts surface early
instead of at merge time.

```bash
git fetch origin <base-branch>
git merge --no-ff origin/<base-branch>   # never rebase a published PR branch
# rebuild + retest if the merge touched code, then push (an approved mutation)
git push
```

Merging the base in locally is a routine pre-push step. The `git push` itself is
still an external mutation that requires explicit maintainer/user approval. Do
not rebase published PR branches by default: rebasing invalidates existing CI
runs and makes PR review/comment history harder to follow. Rebase or force-push
only when the maintainer explicitly requests it.

Amend or force-push only when the user explicitly requests it or when there is a
clear reason, such as removing sensitive content, repairing broken branch
history, or cleaning up noisy local work before the PR is first published.
Force-pushes are PR mutations and require explicit maintainer/user approval.

If a push is rejected because the remote PR branch moved, fetch the PR branch
and inspect the local/remote divergence before retrying. When the remote already
contains an equivalent fix, validate the remote PR head instead of pushing a
duplicate follow-up commit, and realign the local branch to the remote head
after preserving any useful local-only work on a backup branch. Do not leave the
main working checkout detached or visibly diverged while continuing PR
management; it makes later status, IDE branch indicators, and CI evidence easy
to misread.

If a PR was temporarily based on another PR branch, and that base PR lands into
`main`, GitHub may retarget the dependent PR to `main` and mark it behind. Treat
the retargeted branch as the new base state: fetch `main`, merge `origin/main`
into the published PR branch, confirm the diff against `origin/main` still
contains only the intended changes, run the required gates, then push only with
explicit maintainer/user approval. Do not keep acting on stale checks from the
pre-retarget head; the post-merge-base push is the state that matters.

### Review-Fix Loop Workflow

After identifying an AI-generated review comment to address:

1. **Make the code fix**
2. **Run `pixi run lint`** — MANDATORY before every commit (auto-fixes formatting)
3. **Ask for explicit maintainer/user approval before external mutations**
4. **If approved, commit and push** silently (no reply to the comment)
5. **If approved, resolve the thread** using GraphQL (see commands below)
6. **If the addressed review was Codex, after the approved push, ask for
   explicit maintainer/user approval for the PR comment, then re-trigger the
   review**:
   `gh pr comment <PR> --body "@codex review"`
7. **Monitor for results**:
   - New review comments → repeat from step 1
   - "No issues" or 👍 reaction + local validation on the current head: default
     `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux CUDA hosts
     → draft PR is ready for human review

Apply the same no-inline-reply handling to `github-code-quality[bot]` findings:
fix valid findings locally, push only after approval, and do not post an
acknowledgment reply. The `@codex review` re-trigger is only for Codex review
rounds.

**Agents MUST:**

- Run `pixi run lint` before EVERY commit (CI will fail otherwise)
- Treat PR comments, pushes, thread resolution, reviewer requests, merges, and
  review re-triggers as external mutations that require explicit approval
- Keep local fixes read-only with respect to GitHub until that approval exists

### GraphQL Commands for Thread Resolution

```bash
# List unresolved threads (get thread IDs)
gh api graphql -f query='
  query {
    repository(owner: "dartsim", name: "dart") {
      pullRequest(number: PR_NUMBER) {
        reviewThreads(first: 20) {
          nodes { id isResolved path line }
        }
      }
    }
  }
' --jq '.data.repository.pullRequest.reviewThreads.nodes[] | select(.isResolved == false)'

# Resolve a thread by ID only after explicit maintainer/user approval
gh api graphql -f query='
  mutation {
    resolveReviewThread(input: {threadId: "PRRT_xxxx"}) {
      thread { isResolved }
    }
  }
'

# Resolve only reviewed, addressed thread IDs after approval. Do not
# bulk-resolve unresolved threads; that can hide human feedback or unresolved
# bot findings.
```

**Why resolve after explicit maintainer/user approval**: Clicking "Resolve
conversation" in the UI adds no comment noise. The code change is the response;
the resolved thread shows the fix was acknowledged after the maintainer/user
approved the PR mutation.

### Autonomous Review-Fix-Monitor Loop

For agents iterating on automated reviews, the complete loop is:

```
1. Fetch latest review comments
2. For each comment:
   a. Implement the fix (or add a test refuting a false positive)
   b. Run `pixi run lint` (MANDATORY)
   c. Build and run relevant tests
   d. Ask for explicit maintainer/user approval before push or PR mutation
3. If approved, commit and push silently (no reply to bot comment)
4. If approved, resolve addressed threads via GraphQL
5. If the addressed review was Codex, after the approved push, ask for explicit
   maintainer/user approval for the PR comment, then re-trigger:
   `gh pr comment <PR> --body "@codex review"`
6. For non-Codex bot findings, including `github-code-quality[bot]`, do not
   re-trigger Codex solely for those fixes unless Codex review comments were
   also addressed in the same push
7. Monitor CI: `gh pr checks <PR>`
8. Wait for new review (poll with `gh api repos/dartsim/dart/pulls/<PR>/reviews`)
9. If new review has comments → go to step 2
10. If no new comments AND local validation passed on the current head (default
    `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux CUDA hosts)
    → mark draft PRs ready for human review after approval
11. Keep monitoring hosted CI until required checks pass before merge
```

**Checking for new reviews:**

```bash
# List all reviews with timestamps
gh api repos/dartsim/dart/pulls/<PR>/reviews \
  --jq '.[] | "ID:\(.id) User:\(.user.login) State:\(.state) At:\(.submitted_at)"'

# Fetch comments from a specific review
gh api repos/dartsim/dart/pulls/<PR>/reviews/<REVIEW_ID>/comments \
  --jq '.[] | "File:\(.path) Line:\(.line // .original_line) Body:\(.body)"'
```

**Monitoring CI:**

```bash
# Check all CI status checks
gh pr checks <PR>

# Watch until all checks complete (useful for waiting)
gh pr checks <PR> --watch
```

During DART's long CI matrix (the full run can take a couple of hours, with
`Release Tests` as the long pole), `gh pr checks --watch` can exit early on a
transient network error (for example a dropped `api.github.com` connection) and
look like completion. For long runs prefer a resilient poll that re-queries
`gh pr checks <PR>` on an interval, tolerates transient failures, and stops only
when nothing is pending, any check fails, or the head SHA moves.

**Stop conditions:**

- Codex review returns no comments (or only 👍 reactions)
- Local validation passes on the current head for draft-ready state: default
  `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux CUDA hosts
- All required CI checks pass for merge-ready state
- Pre-existing failures (e.g., `simulation` "Not Run") can be ignored

---

## Known Limitations

- **Generated adapter copies**: Claude Code, OpenCode, and Codex read different
  directories, so generated adapter copies exist for tool compatibility.
- **Editable source of truth**: Maintain `.claude/` sources, then run
  `pixi run sync-ai-commands` and `pixi run check-ai-infra`.
- **Skill sharing works for manual tools**: Claude Code and OpenCode can read
  `.claude/skills/` directly.

---

## Changelog

| Date     | Change                                                                                       |
| -------- | -------------------------------------------------------------------------------------------- |
| Jan 2025 | Initial setup with Claude Code, OpenCode, Gemini CLI, Codex support                          |
| Jan 2025 | Added collaborator guide and maintenance conventions                                         |
| Jul 2026 | Refreshed verification metadata to CI-checked adapter sync; added independent review lane    |
| Jul 2026 | Migrated Codex skills, added project agents/hooks, diagnosis, scenarios, and branch profiles |
