# DART Examples Restructure Progress (01)

## Status

- Discovery completed; taxonomy draft captured in plan.

## Completed

- Created the plan and Codex prompt.
- Reviewed DART examples/tutorials structure and build notes.
- Reviewed Newton and Genesis example organization for patterns.
- Drafted a user-first taxonomy and ordering in `docs/dev_tasks/examples/00_plan.md`.
- Drafted a minimal metadata template and shared harness sketch.
- Added an asset layout proposal to the plan.
- Validated the metadata template on a small set of representative examples.
- Selected a pilot category and recorded the harness decision.

## Discovery findings

- DART examples are a flat list of subdirectories, with only coarse grouping in
  `examples/CMakeLists.txt` (non-GUI, OSG, MJCF). There is no user-first
  ordering or onboarding index.
- Most per-example READMEs are boilerplate build/run steps; the top-level
  `examples/README.md` is build-focused and does not explain purpose or scope.
- Tutorials are separate and use a “skeleton vs finished” pattern, but are not
  cross-linked from the examples index.
- Newton organizes examples by domain folders, shares an `assets` folder, and
  provides a unified runner with common CLI flags and a test mode.
- Genesis organizes by domain folders plus a tutorials area, with per-folder
  READMEs for complex domains and a mix of performance, rendering, sensors, and
  coupling topics.

## Coverage by category (draft)

- Getting started: basic entry points exist, but there is no single guided path
  that shows build/run via pixi and a first visualization.
- Rigid bodies and frames: strong coverage of shapes, frames, and simple
  rigid-body scenes.
- Joints and constraints: good coverage of joint types, limits, and constraint
  behaviors; fewer examples explain when to choose each model.
- Collisions and contacts: partial coverage (stacking, ground contact, terrain),
  but little on tuning contact parameters or broad-phase choices.
- Control and IK: strong coverage of IK and higher-level control, but missing a
  minimal, reusable controller baseline.
- IO and models: partial coverage via format-specific loaders; `dart::io`
  unified loading is not showcased directly.
- Soft and hybrid: a small set of examples exist, but breadth is limited.
- Visualization and interaction: OSG-based UI and interaction exist, Raylib is
  isolated, and viewer selection is inconsistent.
- Performance and scaling: only a single speed benchmark is prominent; no
  headless batch, determinism, or profiling workflow.
- Integration and tools: point cloud and tooling touchpoints exist, but logging
  and external tooling examples are sparse.

## Next steps

1. Draft the pilot category index text and migration checklist.
2. Expand metadata headers to the rest of the pilot category.
3. Decide whether to add a top-level `examples/README.md` onboarding index now.

## Open questions

- Should tutorials and examples share a common harness, or stay separate?
- Which viewer and backend options must be supported in the baseline examples?
