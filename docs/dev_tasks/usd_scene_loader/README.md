# OpenUSD Scene Loader — Dev Task

## Current Status

- [x] Phase 0: Captured prototype reference and migration requirements.
- [x] Phase 1: Add `dart::io::ModelFormat::Usd` and a snake_case `dart/io/usd/`
      implementation scaffold behind the DART 7 model-loading front door. PR
      [#3109](https://github.com/dartsim/dart/pull/3109) landed the
      OFF-by-default `DART_BUILD_IO_USD` toggle, `cmake/dart_find_pxr.cmake`,
      `dart/io/usd/usd_parser.{hpp,cpp}`, `.usda` extension inference, the
      default OFF-path guard, `data/usd/simple_chain.usda`, and gated ON-path
      integration coverage.
- [ ] Phase 2: Map USD prims to DART 7 World shapes/joints (PLAN-050);
      keep the loader behind the API-boundary policy
- [ ] Phase 3: Filament-backed viewer example (replace the OSG prototype) plus
      headless smoke coverage under the dartsim engine
- [ ] Phase 4: dartpy bindings against the snake_case API and Linux/macOS
      pytest stability (the prototype's macOS pytest aborts must be resolved
      before the loader is enabled by default)

## Goal

Make `.usda` / `.usd` scenes loadable through DART's unified IO front door so a
researcher can drop a USD asset into a DART 7 World and visualize it
through the maintained Filament renderer, with parity to the existing URDF /
SDF / MJCF paths.

## Non-Goals (for early phases)

- Writing USD back out — read-only loader first.
- Authoring USD physics schema (USDPhysics) features beyond the minimum needed
  to land a working rigid-body chain plus the Unitree H1 minimal sample.
- Re-introducing the OSG viewer that the prototype example used; the project
  has standardized on Filament (see PLAN-060 / PLAN-090).

## Key Decisions

- **Front door**: extend `dart::io::ModelFormat` with `Usd` and route reads
  through DART 7 loaders instead of restoring the retired public
  `dart::io::readWorld` API. Do not create a parallel `dart::utils::UsdParser`
  namespace as the prototype did.
- **Naming**: snake_case headers under `dart/io/usd/` (e.g.
  `usd_parser.{cpp,hpp}`), per `docs/onboarding/code-style.md` and the
  completed snake_case migration. The prototype's PascalCase paths
  (`dart/utils/usd/UsdParser.{cpp,hpp}`) predate the migration.
- **World target**: bind first against the DART 7 World
  (`dart/simulation/`, PLAN-050) so the loader co-evolves with
  the new shape/loader APIs. The legacy `dart::simulation::World` does not
  need a USD path.
- **Viewer example**: build the example against the dartsim engine + Filament
  (see PLAN-101 and `docs/onboarding/gui-rendering.md`). The prototype's
  OSG-based `examples/usd_viewer/main.cpp` is incompatible with the
  retired-OSG policy.
- **Dependency**: pxr (OpenUSD). Discovery lives in
  `cmake/dart_find_pxr.cmake` (snake_case) and is opt-in via a CMake toggle
  mirroring the real `DART_BUILD_GUI` style. The prototype's
  `cmake/DARTFindpxr.cmake` informed the fresh snake_case finder, but the
  current scaffold was authored under `dart::io`.
- **macOS stability**: the prototype's six diagnostic commits chased a pytest
  abort on macOS without resolving it. Phase 4 must address the root cause
  (likely an OpenUSD plugin-path or fork-safety issue) before the loader is
  enabled in default Linux + macOS Pixi environments.

## Prototype Reference

The original prototype lived on the now-deleted `feature/usd-viewer` branch.
These SHAs are historical references; the prior prototype is no longer reachable
from this worktree's reflog/fsck state, so do not rely on `git show` succeeding
unless another clone still has the objects:

- Feature commit: `28bad2773d18ce47bbaef98508d385abe1a8aead`
  ("Add OpenUSD loader and viewer support") — the substantive prototype.
- macOS diagnostics tail: `82f9a261361..b7a7ac09823` — five commits chasing
  pytest aborts; useful as a clue trail when Phase 4 picks up macOS stability.

To inspect the prototype contents during Phase 1:

```bash
git fetch origin
# If still in reflog window
git show 28bad2773d1 --stat
git show 28bad2773d1:dart/utils/usd/UsdParser.hpp
git show 28bad2773d1:examples/usd_viewer/main.cpp
```

Files of interest in the prototype: `dart/utils/usd/UsdParser.{cpp,hpp}`,
`cmake/DARTFindpxr.cmake`, `data/usd/{simple_chain,unitree_h1_minimal}.usda`,
`python/dartpy/utils/UsdParser.cpp`,
`tests/integration/io/test_UsdParser.cpp`,
`python/tests/unit/test_usd_parser.py`.

## Immediate Next Steps

1. Start Phase 2 from current `main` in an OpenUSD-enabled environment: build
   with `DART_BUILD_IO_USD=ON` and flesh out `usd::UsdParser` beyond the single
   default-prim link scaffold.
2. Keep the loader behind the `dart::io` front door and API-boundary policy; do
   not enable OpenUSD by default or add `pxr` to Pixi as part of the parser
   semantics slice.

## Verification Gates

- Phase 1 (landed via PR #3109): `pixi run lint`, `pixi run test-unit`,
  focused USD integration test when `DART_BUILD_IO_USD=ON`,
  `pixi run check-api-boundaries`.
- Phase 3: GUI smoke runs the USD sample headlessly under the dartsim engine.
- Phase 4: dartpy unit tests pass on Linux + macOS Pixi environments without
  pytest aborts.
