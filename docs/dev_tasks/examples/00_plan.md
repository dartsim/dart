# DART Examples Restructure Plan (00)

## Status

- Draft.

## Desired outcomes

- User-first path from "hello world" to advanced topics in a logical order.
- Example taxonomy aligns with core DART capabilities and common user journeys.
- Coverage includes performance and integration examples where appropriate.
- Examples follow consistent conventions (naming, CLI args, assets, and build notes).

## References

- `docs/onboarding/building.md`
- `docs/onboarding/testing.md`
- `docs/onboarding/io-parsing.md`
- `docs/onboarding/gui-rendering.md`
- `docs/onboarding/code-style.md`
- `CONTRIBUTING.md`
- `/home/js/dev/physics_engine/newton/newton/examples`
- `/home/js/dev/physics_engine/Genesis/examples`

## Phases

1. Discovery and taxonomy
   - Inventory existing examples and tutorials by capability (no file lists in docs).
   - Compare with Newton and Genesis patterns (domain folders, assets, shared runner, tests).
   - Draft a user-journey ordering and domain taxonomy.

2. Structure and navigation
   - Propose a numbered folder layout (for example: getting-started, core-dynamics, collisions, constraints, control, soft bodies, IO, visualization, performance).
   - Define naming conventions and minimal metadata per example (goal, APIs touched, expected output).
   - Decide where assets live and how examples locate them.

3. Shared harness and ergonomics
   - Evaluate a small shared helper library for common tasks (CLI parsing, viewer selection, logging, deterministic setup, asset lookup).
   - Ensure consistent build and run notes using `pixi run` entry points.

4. Content migration and gap fill
   - Move or rename existing examples into the new taxonomy with minimal code changes.
   - Add missing examples prioritized by onboarding flow and high-demand use cases.

5. Validation and polish
   - Add smoke tests or scripted runs for non-GUI examples when feasible.
   - Update `examples/README.md` and cross-link to tutorials.

## Milestones and sequencing

- Keep changes incremental and reviewable; avoid giant renames in one PR.
- Each phase should end with updated progress notes in `docs/dev_tasks/examples/01_progress.md`.
