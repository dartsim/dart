# Multi Solver World Refactor Plan (01)

## Status

- Draft: follow-up milestone after PR 00.

## Goal

- Establish an internal ECS object onboarding path while preserving classic
  solver behavior.

## Scope

- Define internal APIs for ECS object creation/registration in World
  (non-public, non-installed headers).
- Extend the ECS-backed rigid solver to consume components beyond legacy
  skeleton mirrors (still no real physics step).
- Provide minimal lifecycle hooks for add/remove to keep solvers in sync.
- Add targeted tests for solver enable/disable and ECS object registration
  lifecycles.

## Non-Goals

- No public EnTT exposure or raw solver pointers.
- No classic solver deprecation.
- No `dart8/` removal or migration in this milestone.
- No full ECS physics pipeline (constraints/collision) yet.

## Open Questions

- Ownership model for ECS objects (World-owned vs solver-owned).
- Placement of ECS component definitions to keep public installs clean.

## Test Plan

- `pixi run test`
- `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`
- `pixi run test-all` before PR
