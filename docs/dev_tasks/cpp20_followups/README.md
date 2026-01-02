# C++20 Modernization Follow-ups

## Status

- Phase 8 in progress: container membership cleanup.

## Goal

Continue C++20 modernization with no behavior changes after the initial cpp20
initiative. Focus on internal-only refactors where possible.

## Constraints

- No functional changes, no new dependencies.
- Prefer internal-only refactors; avoid public API changes unless necessary.
- Use existing tooling (`pixi run ...`) and follow `CONTRIBUTING.md`.
- Keep docs concise; remove this folder when the task is complete.

## Documents

- Plan: `docs/dev_tasks/cpp20_followups/00_plan.md`
- Progress: `docs/dev_tasks/cpp20_followups/01_progress.md`
- Resume prompt: `docs/dev_tasks/cpp20_followups/resume_prompt.md`
