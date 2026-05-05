# AI Tools Compatibility

This document tracks AI coding assistant compatibility with DART's documentation structure.

> **Last Verified**: May 2026 for Codex command/skill sync; January 2025 for other tool assumptions
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
| **Codex**       | `AGENTS.md` auto-loaded; use `$dart-*` skills in `.codex/skills/`            |
| **Gemini CLI**  | Read `GEMINI.md` or `AGENTS.md`; copy prompts from `docs/prompts/`           |

---

## Quick Reference

| Tool            | Instructions               | Commands             | Skills            |
| --------------- | -------------------------- | -------------------- | ----------------- |
| **Claude Code** | `CLAUDE.md` -> `AGENTS.md` | `.claude/commands/`  | `.claude/skills/` |
| **OpenCode**    | `AGENTS.md`                | `.opencode/command/` | `.claude/skills/` |
| **Codex**       | `AGENTS.md`                | `.codex/skills/`     | `.codex/skills/`  |
| **Gemini CLI**  | `GEMINI.md` -> `AGENTS.md` | Manual               | Manual            |

---

## Maintaining AI Docs

### Conventions

| Convention                    | Rule                                                                       |
| ----------------------------- | -------------------------------------------------------------------------- |
| **Single source of truth**    | `AGENTS.md` contains all instructions; other files redirect                |
| **Command naming**            | `dart-` prefix (e.g., `dart-new-task.md`)                                  |
| **Skill naming**              | `dart-` prefix (e.g., `dart-build`)                                        |
| **Skill descriptions**        | Start with display name and quote colon values (e.g., `"DART Build: ..."`) |
| **No tool-specific language** | Use generic terms; avoid "Claude will..." or "Codex should..."             |
| **Placeholders**              | Use `$ARGUMENTS`, `$1`, `$2` for command args                              |
| **File references**           | Use `@file` syntax for auto-loading context                                |

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

| File/Directory                | Purpose              | When to Update                  |
| ----------------------------- | -------------------- | ------------------------------- |
| `AGENTS.md`                   | Primary instructions | When workflows change           |
| `CLAUDE.md`, `GEMINI.md`      | Redirects only       | Rarely (keep minimal)           |
| `.claude/commands/`           | Claude Code commands | When adding workflows           |
| `.opencode/command/`          | OpenCode commands    | Auto-synced from `.claude/`     |
| `.claude/skills/`             | Claude/OpenCode      | When adding domain knowledge    |
| `.codex/skills/`              | Codex skills         | Auto-generated from `.claude/`  |
| `docs/prompts/`               | Fallback templates   | For non-command tools           |
| `docs/onboarding/ai-tools.md` | This file            | When tool compatibility changes |

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

2. Sync to OpenCode and Codex: `pixi run lint` (includes `sync-ai-commands`)

3. Update `docs/prompts/AGENTS.md` command table

4. Optionally add fallback to `docs/prompts/` for non-command tools

Codex does not use project slash-command files directly. The sync script
generates an equivalent Codex skill from each command, so `/dart-fix-ci` becomes
`$dart-fix-ci`.

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

3. Update `AGENTS.md` skills table

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

**Skill candidates** (create when needed): `dart-dynamics` (articulated bodies), `dart-collision` (collision backends), `dart-architecture` (core design).

#### Cross-Agent Compatibility

Skills work across multiple AI tools through automatic syncing:

| Source              | Target                  | Tool                  |
| ------------------- | ----------------------- | --------------------- |
| `.claude/skills/`   | (native)                | Claude Code, OpenCode |
| `.claude/skills/`   | `.codex/skills/`        | Codex                 |
| `.claude/commands/` | `.codex/skills/dart-*/` | Codex workflow skills |

**Sync is automatic** via `pixi run lint` (includes `sync-ai-commands`).

**CI verification**: `pixi run check-ai-commands` ensures sync in CI, checks
that Claude Code, OpenCode, and Codex expose the same effective DART capability
set, and validates command/skill description style plus context-budget limits.

#### Acknowledgment

The SKILL.md format is inspired by [OpenSkills](https://github.com/numman-ali/openskills) (Apache 2.0).
DART skills are original content under BSD 2-Clause license.

### Keeping Commands and Skills in Sync

Commands and skills exist in multiple directories because tools don't share paths.
The `.claude/` directory is the **source of truth**.

**Automated sync** (included in `pixi run lint`):

```bash
pixi run lint               # Includes sync-ai-commands (recommended)
pixi run sync-ai-commands   # Sync commands + skills to all tool directories
pixi run check-ai-commands  # Check if in sync (CI mode, no changes)
```

**What gets synced**:

| Source              | Target                  | Purpose                       |
| ------------------- | ----------------------- | ----------------------------- |
| `.claude/commands/` | `.opencode/command/`    | OpenCode commands             |
| `.claude/commands/` | `.codex/skills/dart-*/` | Codex command workflow skills |
| `.claude/skills/`   | `.codex/skills/`        | Codex domain skills           |

**Effective capability parity**:

Different tools expose the same workflows differently:

- Claude Code: `.claude/commands/` plus `.claude/skills/`
- OpenCode: `.opencode/command/` plus `.claude/skills/`
- Codex: `.codex/skills/` for both domain skills and workflow skills

`pixi run check-ai-commands` compares those effective sets and fails if any
supported agent is missing a command/skill or has an extra one. It also checks
that skill descriptions begin with the display name, command descriptions stay
short and lowercase, and command/skill files remain within context-budget line
limits.

**Sync details**:

- Synced files get an auto-generated header (placed AFTER frontmatter to preserve tool parsing)
- Target directories (`.codex/`, `.opencode/`) are excluded from prettier to prevent re-sync loops
- Edit source files only; synced files are overwritten on each sync

**Manual workflow** (if not using sync script):

1. Edit files in `.claude/` directory
2. Copy changes to corresponding tool directories
3. Verify all match

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

**Tested Version**: Codex CLI 0.128.x (May 2026)

| Feature           | Location                   | Status                                |
| ----------------- | -------------------------- | ------------------------------------- |
| Instructions      | `AGENTS.md`                | ✅ Primary entry point                |
| Skills            | `.codex/skills/*/SKILL.md` | ✅ `$dart-*` skills available         |
| Workflow commands | `.codex/skills/dart-*/`    | ✅ Generated from `.claude/commands/` |

**Notes**:

- `AGENTS.md` is the standard for Codex (same as Claude Code convention)
- Supports subdirectory `AGENTS.md` files (walks from root to CWD)
- Skills use `$skill-name` syntax (e.g., `$dart-build`)
- Command workflows use the same syntax (e.g., `$dart-fix-ci <arguments>`)
- Codex CLI slash commands are built-in controls; project workflows are exposed
  through skills instead of repo-local `/dart-*` slash files
- Full documentation: https://developers.openai.com/codex/

---

## Directory Structure

```
.claude/                   # SOURCE OF TRUTH
├── commands/              # Claude Code commands
│   └── dart-*.md
└── skills/                # Claude + OpenCode skills
    └── dart-*/SKILL.md

.opencode/                 # Auto-synced from .claude/
└── command/               # OpenCode commands
    └── dart-*.md

.codex/                    # Auto-synced from .claude/
└── skills/                # Codex domain + workflow skills
    └── dart-*/SKILL.md    # Includes command-derived $dart-* workflows

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

## Handling Automated Reviews

When AI agents (Claude Code, OpenCode, etc.) work on PRs, they may encounter review comments from other AI systems (e.g., Codex bot, GitHub Copilot).

### Detecting AI-Generated Reviews

**Bot usernames always end with `[bot]`:**

- `chatgpt-codex-connector[bot]` — Codex automated reviews
- `github-actions[bot]` — GitHub Actions automated comments
- `copilot[bot]` — GitHub Copilot suggestions

**If the reviewer username ends in `[bot]`, it is an AI-generated review.**

### Rules for AI Agents (CRITICAL)

**NEVER reply to AI-generated review comments. This means:**

- ❌ **NO** `gh pr comment` commands responding to bot feedback
- ❌ **NO** PR comment replies acknowledging or addressing bot feedback
- ❌ **NO** comments like "Addressed the Codex review feedback"
- ✅ **YES** Push the code fix silently
- ✅ **YES** Re-trigger review with `@codex review` after pushing

**The code change IS the response. No acknowledgment needed.**

**Guidance for AI agents addressing automated reviews**:

- Address the feedback in code, then push the fix silently
- If the feedback is valid, implement the fix without commenting
- If the feedback appears incorrect (false positive):
  1. **Verify** the claim is false by running standalone tests or examining the code
  2. **Add a test** that explicitly documents the correct behavior AND refutes the claim
  3. Example: If Codex claims `hprod([2,3,5,7])` returns 294 instead of 210:
     ```cpp
     EXPECT_FLOAT_EQ(result, 210.0f);  // Verify correct behavior
     EXPECT_NE(result, 294.0f);        // Explicitly refute false claim
     ```
  4. Push the test (no comment needed) - the test serves as permanent documentation
- **Follow the review-fix loop** (see workflow below)

This avoids noisy bot-to-bot conversations while still leveraging automated verification.

> **Note**: False positives can recur across reviews. Tests that explicitly refute incorrect claims prevent future confusion and document the verification.

### Review-Fix Loop Workflow

After identifying an AI-generated review comment to address:

1. **Make the code fix**
2. **Run `pixi run lint`** — MANDATORY before every commit (auto-fixes formatting)
3. **Commit and push** silently (no reply to the comment)
4. **Resolve the thread immediately** using GraphQL (see commands below)
5. **Re-trigger the review**: `gh pr comment <PR> --body "@codex review"`
6. **Monitor for results**:
   - New review comments → repeat from step 1
   - "No issues" or 👍 reaction → done, PR is ready for human review

**Agents MUST:**

- Run `pixi run lint` before EVERY commit (CI will fail otherwise)
- Resolve threads automatically after pushing fixes (keeps PR clean)

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

# Resolve a thread by ID
gh api graphql -f query='
  mutation {
    resolveReviewThread(input: {threadId: "PRRT_xxxx"}) {
      thread { isResolved }
    }
  }
'

# One-liner: resolve all unresolved threads for a PR
PR=2458; gh api graphql -f query="query { repository(owner: \"dartsim\", name: \"dart\") { pullRequest(number: $PR) { reviewThreads(first: 50) { nodes { id isResolved } } } } }" --jq '.data.repository.pullRequest.reviewThreads.nodes[] | select(.isResolved == false) | .id' | while read -r tid; do gh api graphql -f query="mutation { resolveReviewThread(input: {threadId: \"$tid\"}) { thread { isResolved } } }" > /dev/null && echo "Resolved: $tid"; done
```

**Why resolve immediately**: Clicking "Resolve conversation" in the UI adds no comment noise. The code change is the response—the resolved thread shows the fix was acknowledged.

### Autonomous Review-Fix-Monitor Loop

For agents iterating on automated reviews, the complete loop is:

```
1. Fetch latest review comments
2. For each comment:
   a. Implement the fix (or add a test refuting a false positive)
   b. Run `pixi run lint` (MANDATORY)
   c. Build and run relevant tests
   d. Commit and push silently (no reply to bot comment)
3. Resolve addressed threads via GraphQL
4. Re-trigger: `gh pr comment <PR> --body "@codex review"`
5. Monitor CI: `gh pr checks <PR>`
6. Wait for new review (poll with `gh api repos/dartsim/dart/pulls/<PR>/reviews`)
7. If new review has comments → go to step 2
8. If no new comments AND CI is green → done
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

**Stop conditions:**

- Codex review returns no comments (or only 👍 reactions)
- All CI checks pass (green)
- Pre-existing failures (e.g., `simulation-experimental` "Not Run") can be ignored

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
