---
name: dart-docs-update
description: "DART Docs Update: update documentation without code changes"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-docs-update.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-docs-update

Use this skill in Codex to run the DART `dart-docs-update` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-docs-update <arguments>`
- Codex: `$dart-docs-update <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Update documentation: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/README.md
@docs/ai/principles.md
@docs/ai/verification.md
@docs/onboarding/ai-tools.md
@docs/onboarding/changelog.md

## Workflow

1. Create a branch from the target branch: `git checkout -b docs/<topic> origin/main`
2. Edit docs and AI workflow sources only:
   - Regular docs: `docs/**`, `README.md`, `AGENTS.md`, `CONTRIBUTING.md`,
     and `CHANGELOG.md` when `dart-changelog` requires a release-note entry
   - AI source files: `.claude/commands/**`, `.claude/skills/**`
3. For AI workflow changes, run `pixi run sync-ai-commands`; do not hand-edit generated `.opencode/` or `.codex/` files
4. Update indexes and cross-references that point to changed docs
5. Use `docs/ai/verification.md` to select the docs-only or AI docs/adapters
   gate set, then run `pixi run lint` before committing
6. Invoke the `dart-changelog` routine for the `CHANGELOG.md` decision and any
   required entry.
7. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, use `.github/PULL_REQUEST_TEMPLATE.md` and the proper
   milestone.
