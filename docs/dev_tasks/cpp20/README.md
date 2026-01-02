# C++20 Modernization - Dev Task

## Status

- Phase 4 in progress: consolidation and validation.

## Goal

Modernize the codebase to idiomatic C++20 while preserving behavior. Public API
breaks are acceptable for DART 7 if `pixi run -e gazebo test-gz` passes without
changing Gazebo code.

## Non-goals

- Behavior changes or algorithmic refactors
- Gazebo code changes to accommodate DART API updates
- New dependencies or build entry points

## Constraints

- Use existing tooling (`pixi run ...`) and follow `CONTRIBUTING.md`.
- Keep changes mechanical and reviewable; prefer internal-only refactors.
- Update progress and plan docs after each phase.

## Documents

- Plan: `docs/dev_tasks/cpp20/00_plan.md`
- Progress: `docs/dev_tasks/cpp20/01_progress.md`
- Resume prompt: `docs/dev_tasks/cpp20/resume_prompt.md`
