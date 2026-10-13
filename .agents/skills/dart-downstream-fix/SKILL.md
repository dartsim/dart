---
name: dart-downstream-fix
description: "DART Downstream Fix: fix a DART bug reported through gz-physics or Gazebo"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-downstream-fix.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-downstream-fix

Use this skill to run the DART `dart-downstream-fix` workflow. The editable workflow
source lives in `.claude/commands/`; this generated skill is its entrypoint for
Codex, Muse, and OpenCode's skill loader.

## Invocation

- Claude Code/OpenCode: `/dart-downstream-fix <arguments>`
- Codex: `$dart-downstream-fix <arguments>`
- Muse: load the `dart-downstream-fix` skill

Treat the text after the skill name, or the user's request when the skill is
loaded on demand, as `$ARGUMENTS`. When the workflow references `$1`, `$2`,
etc., map those to the positional values supplied by the user.

## Command Body

Fix downstream-reported DART issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/ci-cd.md

## When To Use

Use for downstream issues in gz-physics, Gazebo, or gz-sim that trace back to DART behavior: crashes, assertions, NaN/Inf propagation, missing validation, or DART performance regressions.

## Workflow

If the downstream symptom depends on scene/model structure, simulation,
collision/contact, or Filament output, route through `dart-verify-sim`: establish a
text oracle, add assessed claim-tied visual evidence, or document a
visual exception when graphics support is unavailable or not applicable.

1. Read the downstream issue, logs, stack traces, and reproduction steps.
2. Identify the DART API, component, and invalid usage pattern involved.
3. Search for related validation and recovery patterns in DART.
4. Plan the smallest fix and the regression test location.
5. Decide whether the bug applies to the active release line. For applicable
   bug fixes, implement on the development branch, `main`:
   - branch:
     `fix/<downstream-project>-<issue-number>-<brief-description>`
   - add a regression test that reproduces the downstream symptom
   - keep the fix minimal; no unrelated refactors
6. Run `pixi run lint` and relevant tests; use `pixi run test-all` when
   feasible, and run `pixi run -e gazebo test-gz` when Gazebo/gz-physics
   compatibility could be affected.
7. Ask for explicit maintainer/user approval before pushing or creating PRs.
   After approval, create the PR against `main` with the milestone resolved
   from the target branch's `docs/onboarding/release-management.md`, and
   reference the downstream issue.

## Release-Line Conventions

- DART 6 code (`release-6.20`) may use `dtwarn << ...`, `<dart/dart.hpp>`, and older test CMake patterns.

## Output

- Root cause and fix summary
- PR URL, if one was opened
- Tests run and CI status
- Link back to the downstream issue
