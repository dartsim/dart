# Phase 4 USD Coordination

## Decision

The SKEL format evolution task does not own USD implementation. It coordinates
with the existing OpenUSD scene-loader task and treats that task as the owner
for USD loader, viewer, dartpy, dependency, and OpenUSD-enabled validation work.

## Evidence Refreshed 2026-07-04 UTC

- PR [#3109](https://github.com/dartsim/dart/pull/3109) is merged into `main`
  at `fab38a336f7`, adding the OFF-by-default `DART_BUILD_IO_USD` scaffold,
  `dart::io::ModelFormat::Usd`, `.usda` inference, `dart/io/usd/`, and gated
  tests.
- [`docs/dev_tasks/usd_scene_loader/`](../usd_scene_loader/) remains active and
  owns the follow-up work:
  - Phase 2: map USD prims to DART 7 World shapes/joints.
  - Phase 3: add a Filament-backed viewer example and headless smoke coverage.
  - Phase 4: add dartpy bindings and resolve Linux/macOS pytest stability.
- The USD task's non-goals explicitly keep USD export out of the early loader
  phases. Export remains part of this task's Phase 5 round-trip planning, but
  USD write-back should not be added here before the read-side loader matures.

## Boundary

This task may use USD as design pressure for `dart::io`, DART 7 World loading,
and export round-trip planning. It must not duplicate:

- `DART_BUILD_IO_USD` dependency policy;
- `dart/io/usd/` parser implementation;
- OpenUSD/pxr environment setup;
- USD viewer examples;
- dartpy USD bindings;
- OpenUSD-specific CI gates.

## Consequences

- Phase 4 is complete for this SKEL task when the coordination boundary is
  recorded and the README points to `usd_scene_loader/` as owner.
- Remaining USD implementation stays in `docs/dev_tasks/usd_scene_loader/`.
- Phase 5 export planning must account for USD as an eventual peer format, but
  should start from accepted read-side formats and avoid promising USD write
  support before the USD loader's own phases are further along.
