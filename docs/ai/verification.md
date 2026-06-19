# Verification Guide

Use the narrowest gate that proves the change, then broaden when the touched
surface affects shared behavior.

## Baseline Checks

- Always inspect `git status --short --branch`.
- Before every commit, run `pixi run lint`.
- For C++ or Python code changes, run `pixi run build` and the focused tests for
  the touched module.
- For package, exported target, installed-header, collision, constraint, or
  default-solver changes that can affect Gazebo/gz-physics, run the Gazebo gate:

  ```bash
  N=${DART_SAFE_JOBS:-$(python3 scripts/parallel_jobs.py)}
  DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
  ```

## Docs And Workflow Changes

- For AI workflow changes, run:

  ```bash
  pixi run sync-ai-commands
  pixi run check-ai-commands
  pixi run lint
  ```

- For ordinary docs-only changes on this release branch, run `pixi run lint`.
  If the docs affect Read the Docs pages, run `pixi run docs-build` when
  practical.

## Completion Audit

Before calling work complete, verify every explicit requirement against current
evidence: files, command output, tests, PR state, and downstream gates where
applicable. If evidence is indirect or missing, keep working.
