---
description: fix a DART bug reported through gz-physics or Gazebo
argument-hint: "<downstream-issue-url> [issue-number]"
agent: build
---

Fix downstream-reported DART issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/ci-cd.md

## When To Use

Use for downstream issues in gz-physics, Gazebo, or gz-sim that trace back to DART behavior: crashes, assertions, NaN/Inf propagation, missing validation, or DART performance regressions.

## Workflow

If the downstream symptom depends on scene/model structure, simulation,
collision/contact, or GUI output, route through `dart-verify-sim`: establish a
text oracle, add assessed claim-tied visual evidence, or document a
visual exception when rendering is unavailable or not applicable.

1. Read the downstream issue, logs, stack traces, and reproduction steps.
2. Identify the DART API, component, and invalid usage pattern involved.
3. Search for related validation and recovery patterns in DART.
4. Plan the smallest fix and the regression test location.
5. Decide whether the bug applies to the active release line. For applicable
   bug fixes, implement on the active DART 6 LTS branch first, then cherry-pick
   or reapply to `main` for DART 7:
   - branch:
     `fix/<downstream-project>-<issue-number>-<brief-description>-6-lts`
   - add a regression test that reproduces the downstream symptom
   - keep the fix minimal; no unrelated refactors
6. Run `pixi run lint` and relevant tests; use `pixi run test-all` when
   feasible, and also `pixi run -e cuda test-all` on Linux hosts with a visible
   NVIDIA CUDA runtime.
7. Ask for explicit maintainer/user approval before pushing or creating PRs.
   After approval, create the release-branch PR with the branch-matching DART
   6.x patch milestone and reference the downstream issue.
8. Create the matching `main` PR with milestone `DART 7.0`; adapt API
   differences if needed.

## Release-Line Differences

- DART 7 commonly uses `DART_WARN()` and `<dart/All.hpp>`.
- DART 6 LTS may use `dtwarn << ...`, `<dart/dart.hpp>`, and older test CMake patterns.

## Output

- Root cause and fix summary
- Main PR URL and release PR URL, if applicable
- Tests run and CI status
- Link back to the downstream issue
