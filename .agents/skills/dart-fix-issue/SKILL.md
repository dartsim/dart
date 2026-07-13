---
name: dart-fix-issue
description: "DART Fix Issue: resolve a GitHub issue with a fix and regression test"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-fix-issue.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-fix-issue

Use this skill in Codex to run the DART `dart-fix-issue` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/dart-fix-issue <arguments>`
- Codex: `$dart-fix-issue <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Fix GitHub issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/changelog.md
@docs/ai/verification.md

## Workflow

If the issue's fix depends on model/scene structure, simulation, dynamics,
collision/contact/constraints, or GUI output, route through `dart-verify-sim`:
pair the focused behavioral oracle with assessed, claim-tied visual evidence,
or document a visual exception when rendering is unavailable or not
applicable.

1. `gh issue view $1` - Validate issue
2. Classify whether the issue is a bug fix that applies to the active DART 6
   LTS branch. For applicable bug fixes, start from the highest maintained
   `origin/release-6.*` branch; otherwise start from `origin/main`.
3. Fix with minimal changes + add regression test. For dual-PR bug fixes, fix
   the active DART 6 LTS branch first, then cherry-pick or reapply to `main`.
4. `pixi run lint`, then run the smallest relevant tests; use
   `pixi run test-all` before finalizing when feasible, and also
   `pixi run -e cuda test-all` on Linux hosts with a visible NVIDIA CUDA runtime
5. Before PR creation, invoke the `dart-changelog` routine to decide whether
   `CHANGELOG.md` needs an entry, then fill `.github/PULL_REQUEST_TEMPLATE.md`.
6. After explicit maintainer/user approval, `git push -u origin HEAD && gh pr create --base <target-branch> --milestone "<milestone>"`
7. **Before PR**: If task used `docs/dev_tasks/<task>/`, remove the folder (include in this PR, not after merge)

## CRITICAL: Dual-PR for Bug Fixes

PRs to BOTH the active DART 6 LTS branch AND `main`. See
`docs/onboarding/contributing.md`.

## Output

- Issue number and classification (bug fix, dual-PR, or main-only)
- Fix summary and regression test added
- Gates run (lint, focused tests, `pixi run test-all`)
- PR readiness for each target branch, noting any explicitly approved mutation
