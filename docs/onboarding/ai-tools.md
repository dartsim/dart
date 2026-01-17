# AI Tools Compatibility

This document tracks AI coding assistant compatibility with DART's documentation structure.

> **Last Verified**: January 2025
> **Review Cadence**: Verify when updating tool versions or experiencing unexpected behavior.

## For Collaborators: Tool Selection

| Tool            | Best For                                | Limitations                      |
| --------------- | --------------------------------------- | -------------------------------- |
| **Claude Code** | Full workflow automation, complex tasks | Requires Anthropic subscription  |
| **OpenCode**    | Multi-model flexibility, open source    | Commands require separate config |
| **Gemini CLI**  | Quick queries, large context            | No command/skill support         |
| **Codex**       | Code generation, OpenAI integration     | No command/skill support         |

### Quick Start by Tool

| Tool            | Setup                                                                        |
| --------------- | ---------------------------------------------------------------------------- |
| **Claude Code** | `CLAUDE.md` auto-loaded; use `/dart-*` commands                              |
| **OpenCode**    | `AGENTS.md` auto-loaded; use `/dart-*` commands; skills in `.claude/skills/` |
| **Gemini CLI**  | Read `GEMINI.md` or `AGENTS.md`; copy prompts from `docs/prompts/`           |
| **Codex**       | Read `AGENTS.md`; copy prompts from `docs/prompts/`                          |

---

## Quick Reference

| Tool            | Instructions               | Commands             | Skills            |
| --------------- | -------------------------- | -------------------- | ----------------- |
| **Claude Code** | `CLAUDE.md` -> `AGENTS.md` | `.claude/commands/`  | `.claude/skills/` |
| **OpenCode**    | `AGENTS.md`                | `.opencode/command/` | `.claude/skills/` |
| **Gemini CLI**  | `GEMINI.md` -> `AGENTS.md` | Manual               | Manual            |
| **Codex**       | `AGENTS.md`                | Manual               | Manual            |

---

## Maintaining AI Docs

### Conventions

| Convention                    | Rule                                                           |
| ----------------------------- | -------------------------------------------------------------- |
| **Single source of truth**    | `AGENTS.md` contains all instructions; other files redirect    |
| **Command naming**            | `dart-` prefix (e.g., `dart-new-task.md`)                      |
| **Skill naming**              | `dart-` prefix (e.g., `dart-build`)                            |
| **No tool-specific language** | Use generic terms; avoid "Claude will..." or "Codex should..." |
| **Placeholders**              | Use `$ARGUMENTS`, `$1`, `$2` for command args                  |
| **File references**           | Use `@file` syntax for auto-loading context                    |

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
- Use `@~/.claude/file.md` for home directory references

### File Ownership

| File/Directory                | Purpose              | When to Update                              |
| ----------------------------- | -------------------- | ------------------------------------------- |
| `AGENTS.md`                   | Primary instructions | When workflows change                       |
| `CLAUDE.md`, `GEMINI.md`      | Redirects only       | Rarely (keep minimal)                       |
| `.claude/commands/`           | Claude Code commands | When adding workflows                       |
| `.opencode/command/`          | OpenCode commands    | Mirror `.claude/commands/` changes          |
| `.claude/skills/`             | Shared skills        | When adding domain knowledge                |
| `docs/prompts/`               | Fallback templates   | When adding workflows for non-command tools |
| `docs/onboarding/ai-tools.md` | This file            | When tool compatibility changes             |

### Adding a New Command

1. Create `.claude/commands/dart-<name>.md`:

   ```markdown
   ---
   description: Brief description
   agent: build
   ---

   Task: $ARGUMENTS

   @AGENTS.md
   @docs/onboarding/relevant-doc.md

   ## Workflow

   1. Step one
   2. Step two
   ```

2. Copy to `.opencode/command/dart-<name>.md` (required for OpenCode)

3. Update `docs/prompts/AGENTS.md` command table

4. Optionally add fallback to `docs/prompts/` for non-command tools

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

2. Update `AGENTS.md` skills table

### Keeping Commands in Sync

Commands exist in both `.claude/commands/` and `.opencode/command/` because tools don't share directories. When updating:

1. Edit `.claude/commands/dart-<name>.md`
2. Copy changes to `.opencode/command/dart-<name>.md`
3. Verify both match

### Review Cadence

| Check                       | Frequency                      |
| --------------------------- | ------------------------------ |
| Tool compatibility          | When updating tool versions    |
| Command/skill functionality | After creating new ones        |
| Doc accuracy                | Quarterly or when issues arise |

---

## Detailed Compatibility

### Claude Code

**Tested Version**: Claude Code (Jan 2025)

| Feature      | Location                    | Status                             |
| ------------ | --------------------------- | ---------------------------------- |
| Instructions | `CLAUDE.md`                 | ✅ Reads, redirects to `AGENTS.md` |
| Commands     | `.claude/commands/*.md`     | ✅ `/dart-*` commands available    |
| Skills       | `.claude/skills/*/SKILL.md` | ✅ On-demand loading               |

**Notes**:

- Does NOT read `.opencode/` directory
- Commands use `$ARGUMENTS` for user input
- Skills require YAML frontmatter with `name` and `description`

### OpenCode

**Tested Version**: OpenCode 1.x (Jan 2025)

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

**Tested Version**: Gemini CLI (Jan 2025)

| Feature      | Location    | Status                             |
| ------------ | ----------- | ---------------------------------- |
| Instructions | `GEMINI.md` | ✅ Reads, redirects to `AGENTS.md` |
| Commands     | N/A         | ❌ No native support               |
| Skills       | N/A         | ❌ No native support               |

**Notes**:

- Use `GEMINI.md` or `AGENTS.md` as context
- No slash command support; use `docs/prompts/` templates manually

### OpenAI Codex

**Tested Version**: Codex (Jan 2025)

| Feature      | Location    | Status                 |
| ------------ | ----------- | ---------------------- |
| Instructions | `AGENTS.md` | ✅ Primary entry point |
| Commands     | N/A         | ❌ No native support   |
| Skills       | N/A         | ❌ No native support   |

**Notes**:

- `AGENTS.md` is the standard for Codex
- No slash command support; use `docs/prompts/` templates manually

---

## Directory Structure

```
.claude/
├── commands/              # Claude Code commands
│   └── dart-*.md
└── skills/                # Shared skills (Claude + OpenCode)
    └── dart-*/SKILL.md

.opencode/
└── command/               # OpenCode commands (mirrors .claude/commands/)
    └── dart-*.md

docs/prompts/              # Fallback templates (for tools without commands)
└── *.md
```

---

## Verification Checklist

When verifying compatibility:

1. **Check tool documentation** for changes to config file locations
2. **Test commands** - Run `/dart-new-task test` and verify it works
3. **Test skills** - Check if skills appear in tool's skill list
4. **Test file references** - Verify `@file` syntax loads files correctly
5. **Update this document** with findings and date

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

## Known Limitations

- **No cross-tool command sharing**: Claude Code and OpenCode read different directories
- **Skill sharing works**: Both tools read `.claude/skills/`
- **Command duplication required**: Must maintain both `.claude/commands/` and `.opencode/command/`

---

## Changelog

| Date     | Change                                                              |
| -------- | ------------------------------------------------------------------- |
| Jan 2025 | Initial setup with Claude Code, OpenCode, Gemini CLI, Codex support |
| Jan 2025 | Added collaborator guide and maintenance conventions                |
