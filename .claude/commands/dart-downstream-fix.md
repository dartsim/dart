---
description: fix a DART bug reported through gz-physics or Gazebo
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

1. Read the downstream issue, logs, stack traces, and reproduction steps.
2. Identify the DART API, component, and invalid usage pattern involved.
3. Search for related validation and recovery patterns in DART.
4. Plan the smallest fix and the regression test location.
5. Implement on `main` first for DART 7:
   - branch: `fix/<downstream-project>-<issue-number>-<brief-description>`
   - add a regression test that reproduces the downstream symptom
   - keep the fix minimal; no unrelated refactors
6. Run `pixi run lint` and relevant tests; use `pixi run test-all` when feasible.
7. Create a `main` PR with milestone `DART 7.0` and reference the downstream issue.
8. If this is a bug fix applicable to the release line, create the release-branch PR too:
   - branch from `release-6.16`
   - adapt API differences if needed
   - milestone `DART 6.16.x`

## Release-Line Differences

- DART 7 commonly uses `DART_WARN()` and `<dart/All.hpp>`.
- DART 6.16 may use `dtwarn << ...`, `<dart/dart.hpp>`, and older test CMake patterns.

## Output

- Root cause and fix summary
- Main PR URL and release PR URL, if applicable
- Tests run and CI status
- Link back to the downstream issue
