# Resume: OpenUSD Scene Loader

## Last Session Summary

Captured the OpenUSD scene-loader prototype (formerly on
`origin/feature/usd-viewer`) as a tracked dev task. The prototype branch was
retired because it predated the snake_case migration, used the retired OSG
viewer, exposed a parallel `dart::utils::UsdParser` namespace instead of the
unified `dart::io` front door, and left macOS pytest aborts unresolved.

## Current Branch

`main` — no implementation work on the loader has started after the prototype.

## Immediate Next Step

Pick a feature branch off `main`, then port the prototype's parser core (see
`README.md` Prototype Reference for SHAs) into `dart/io/usd/usd_parser.{cpp,hpp}`
behind `dart::io::ModelFormat::Usd`. Keep dependencies behind a CMake toggle
parallel to `DART_BUILD_GUI_FILAMENT`.

## Context That Would Be Lost

- The prototype's macOS pytest aborts (see tail commit
  `b7a7ac09823 Collect lldb backtrace on mac pytest aborts`) were never root-
  caused; likely an OpenUSD plugin path / fork-safety issue. Treat as a
  Phase 4 blocker, not a parser bug.
- The viewer example must target Filament + dartsim engine, not OSG
  (PLAN-060 / PLAN-090 retired OSG).
- `dart::simulation::experimental` (PLAN-050) is the World target; the
  legacy `dart::simulation::World` does not need a USD path.

## How to Resume

```bash
git checkout main
git pull --ff-only origin main
git checkout -b feature/usd-loader-phase1

# Inspect prototype (while reflog still holds it):
git show 28bad2773d1 --stat
git show 28bad2773d1:dart/utils/usd/UsdParser.hpp
```

Then start Phase 1 from `docs/dev_tasks/usd_scene_loader/README.md`.
