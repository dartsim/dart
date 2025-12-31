# DART Examples Restructure Progress (01)

## Status

- Discovery completed; taxonomy draft captured in plan.

## Completed

- Created the plan and Codex prompt.
- Reviewed DART examples/tutorials structure and build notes.
- Reviewed Newton and Genesis example organization for patterns.
- Drafted a user-first taxonomy and ordering in `docs/dev_tasks/examples/00_plan.md`.
- Drafted a minimal metadata template and shared harness sketch.

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

## Gap analysis (high level)

- No structured learning path or capability-based grouping.
- No shared example harness (CLI args, viewer selection, headless/test mode).
- Limited visibility of `dart::io` unified loading in the examples.
- Mixed viewer backends without consistent guidance (OSG, Raylib, headless).
- Performance and scalability coverage is minimal.

## Next steps

1. Map current examples into the new taxonomy (no file lists in docs).
2. Validate the metadata template against a small subset of examples.
3. Decide whether the shared harness is required for the initial migration.

## Open questions

- Should tutorials and examples share a common harness, or stay separate?
- Which viewer and backend options must be supported in the baseline examples?
