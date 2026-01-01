# DART Examples Restructure Progress (01)

## Status

- Taxonomy migration complete; remaining work is gap analysis and polish.

## Completed

- Created the plan and Codex prompt.
- Reviewed DART examples/tutorials structure and build notes.
- Reviewed Newton and Genesis example organization for patterns.
- Drafted a user-first taxonomy and ordering in `docs/dev_tasks/examples/00_plan.md`.
- Drafted a minimal metadata template and shared harness sketch.
- Added an asset layout proposal to the plan.
- Validated the metadata template on a small set of representative examples.
- Selected a pilot category and recorded the harness decision.
- Drafted a pilot category index and migration checklist.
- Expanded metadata headers to more beginner-focused examples.
- Deferred updating `examples/README.md` until the new category layout exists.
- Decided the pilot migration does not require a shared harness initially.
- Executed the pilot migration: added a getting-started category folder and
  moved initial examples into it.
- Updated `examples/README.md` with a brief onboarding note.
- Created the rigid-bodies category and moved initial examples with refreshed
  metadata headers.
- Created the joints-and-constraints category and moved initial examples with
  refreshed metadata headers.
- Created the collisions-and-contacts category and moved initial examples with
  refreshed metadata headers.
- Created the control-and-IK category and moved initial examples with refreshed
  metadata headers.
- Created the IO-and-models category and moved initial examples with refreshed
  metadata headers.
- Created the soft-and-hybrid category and moved initial examples with refreshed
  metadata headers.
- Created the visualization-and-interaction category and moved initial examples
  with refreshed metadata headers.
- Created the performance-and-scaling category and moved initial examples with
  refreshed metadata headers.
- Created the integration-and-tools category and moved remaining uncategorized
  examples into the numbered layout.

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

1. Identify top gaps per category (especially IO, performance, and tools) and
   decide whether to add or defer missing examples.
2. Cross-link tutorials from the category READMEs where the learning path
   continues.
3. Keep `examples/CMakeLists.txt` grouping comments aligned with the new
   category layout.

## Resume checklist

- Read `docs/dev_tasks/examples/00_plan.md` and this progress log.
- Review the current category layout in `examples/` and the build wiring in
  `examples/CMakeLists.txt`.
- Scan for missing metadata headers and category index tweaks.
- Create a commit per checkpoint so progress is resumable.

## Open questions

- Should tutorials and examples share a common harness, or stay separate?
- Which viewer and backend options must be supported in the baseline examples?
