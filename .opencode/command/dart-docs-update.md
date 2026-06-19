---
description: update documentation without code changes
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-docs-update.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Update documentation: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/README.md
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
4. Update indexes and cross-references that point to changed docs
5. Use `docs/ai/verification.md` to select the docs-only or AI docs/adapters
   gate set, then run `pixi run lint` before committing
6. Record the `CHANGELOG.md` decision using `docs/onboarding/changelog.md`.
7. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, push with the same local and remote topic-branch name, use
   `.github/PULL_REQUEST_TEMPLATE.md`, and set the proper milestone.
