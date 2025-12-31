# DART Examples Restructure Plan (00)

## Status

- Draft.

## Desired outcomes

- User-first path from "hello world" to advanced topics in a logical order.
- Example taxonomy aligns with core DART capabilities and common user journeys.
- Coverage includes performance and integration examples where appropriate.
- Examples follow consistent conventions (naming, CLI args, assets, and build notes).

## Draft taxonomy and ordering (user-first)

1. `00_getting_started`: build/run, minimal world + skeleton, first visualization
2. `01_rigid_bodies_and_frames`: shapes, frames, transforms, basic dynamics
3. `02_joints_and_constraints`: joint types, limits, mimic/coupler, constraints
4. `03_collisions_and_contacts`: collision setup, contact tuning, terrains
5. `04_control_and_ik`: IK, operational space, controllers, motion tasks
6. `05_io_and_models`: `dart::io` loading, URDF/SDF/MJCF, resource retrieval
7. `06_soft_and_hybrid`: soft bodies, hybrid dynamics, mixed simulations
8. `07_visualization_and_interaction`: viewer usage, UI, interaction hooks
9. `08_performance_and_scaling`: headless runs, profiling, scale tests
10. `09_integration_and_tools`: point clouds, external tooling, logging

Notes:
- Category names are draft; keep them short and ordered with numeric prefixes.
- Tutorials remain separate but should be cross-linked from relevant categories.

## Example metadata template (draft)

Each example should include a short header block in its README:
- Goal (user story)
- Concepts/APIs touched
- Expected output (visual or numeric)
- Run command (pixi entry point)
- Variants (headless, alternate viewer, data files)

## Shared harness sketch (draft)

- Provide a small `examples/common` helper with a minimal `ExampleOptions` and
  CLI parsing for headless runs, step counts, and viewer selection.
- Ensure viewer selection respects build flags (OSG vs Raylib) with clear
  fallback behavior.
- Include helpers for asset lookup via `dart::common::Uri` and
  `dart::io::ReadOptions` where relevant.

## Asset layout proposal (draft)

- Prefer shared assets already shipped with DART via `dart://` URIs.
- New example-specific assets live under `examples/assets/` or the example
  directory when they are not reused.
- Add a small helper to resolve assets for in-source and install builds.

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

## Success criteria (Phase 1/2)

- Each category has a short index describing the user goal and prerequisites.
- Each example has minimal metadata: purpose, primary APIs, expected output.
- `dart::io` unified loading is demonstrated in the IO category.
- A shared harness exists (or an explicit decision not to) with consistent CLI flags.
- Assets are discoverable without hardcoded absolute paths.

## Pilot migration (proposal)

- Pilot category: getting started (minimal world + basic visualization).
- Scope: reorganize a small set of beginner examples, update their metadata
  headers, and add a short category index.
- Harness: optional for the pilot; avoid code changes unless required.

## Milestones and sequencing

- Keep changes incremental and reviewable; avoid giant renames in one PR.
- Each phase should end with updated progress notes in `docs/dev_tasks/examples/01_progress.md`.
