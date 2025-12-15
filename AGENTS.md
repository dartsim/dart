# Agent Guidelines for DART

This file is a pointer board for agents working in this repository. Keep it concise and expand other documents instead.

## Read First

- Architectural, build, and workflow expectations live in `docs/onboarding` (start with `docs/onboarding/ci-cd.md` and `docs/onboarding/build-system.md`).
- The day-to-day pixi workflow (install, config, build, test) is documented in `docs/onboarding/building.md`.
- Coding standards, formatting, and contribution flow are in `CONTRIBUTING.md`.
- Feature‑specific notes belong beside the code (e.g., README in the component directory) or in `docs/`.
- Unified model loading API (`dart::io`) is documented in `docs/onboarding/io-parsing.md`.

## Daily Reminders

- Use the existing tooling (`pixi run …`) described in the onboarding docs; do not invent new entry points.
- When you learn something new, update the relevant document (usually under `docs/onboarding/` or the component README) and then add a short pointer here only if discoverability is still lacking.
- Treat AGENTS.md as a TODO list for missing documentation: if this file grows beyond pointers, migrate the details to the appropriate doc and shrink this file again before you finish the task.
- Gazebo / gz-physics integration notes (including patching policy and deprecated `collision-*` compatibility components) live in `docs/onboarding/build-system.md` under “Gazebo Integration Feature”.
