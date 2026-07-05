---
name: dart-docs-update
description: "DART Docs Update: update docs or AI instruction visibility without code changes"
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
@docs/AGENTS.md
@docs/information-architecture.md
@docs/ai/principles.md
@docs/ai/verification.md
@docs/onboarding/ai-tools.md
@docs/onboarding/changelog.md

## Workflow

1. Create a branch from the target branch, for example:
   `git switch --no-track -c docs/<topic> origin/release-6.20`
2. Edit docs and AI workflow sources only:
   - Regular docs: `docs/**`, `README.md`, `AGENTS.md`, `CONTRIBUTING.md`
   - AI source files: `.claude/commands/**`, `.claude/skills/**`
3. For AI workflow changes, run `pixi run sync-ai-commands`; do not hand-edit generated `.opencode/` or `.codex/` files
4. Classify new or moved docs by lifecycle first, then audience, then topic,
   using `docs/information-architecture.md`
5. Update indexes and cross-references that point to changed docs. For AI docs,
   keep always-loaded entrypoints compact: improve owner placement or pointers
   instead of duplicating procedures.
6. Use `docs/ai/verification.md` to select the docs-only or AI docs/adapters
   gate set, then run `pixi run lint` before committing
7. Record the `CHANGELOG.md` decision using `docs/onboarding/changelog.md`.
8. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, push with the same local and remote topic-branch name, use
   `.github/PULL_REQUEST_TEMPLATE.md`, and set the proper milestone.

## Output

- Docs changed and why
- Verification commands run (docs/AI checks) and their results
- Any external mutation left pending explicit approval
