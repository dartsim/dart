# OpenUSD Scene Loader — Dev Task

## Current Status

- [x] Phase 0: Captured prototype reference and migration requirements.
- [x] Phase 1: Add `dart::io::ModelFormat::Usd` and a snake_case `dart/io/usd/`
      implementation scaffold behind the DART 7 model-loading front door. PR
      [#3109](https://github.com/dartsim/dart/pull/3109) landed (merged to `main`
      at `fab38a336f7`) the OFF-by-default `DART_BUILD_IO_USD` toggle,
      `cmake/dart_find_pxr.cmake`, `dart/io/usd/usd_parser.{hpp,cpp}`, `.usda`
      extension inference, the default OFF-path guard,
      `data/usd/simple_chain.usda`, and gated ON-path integration coverage.
- [ ] Phase 2: Map USD prims to DART 7 `World` shapes/joints and keep the loader
      behind the API-boundary policy. **Blocked** — needs OpenUSD in the build
      env (see [Blocker](#blocker--approval-boundary)).
- [ ] Phase 3: Filament-backed viewer example (replace the OSG prototype) plus
      headless smoke coverage under the dartsim engine. **Blocked** on Phase 2.
- [ ] Phase 4: dartpy bindings against the snake_case API and Linux/macOS
      pytest stability (the prototype's macOS pytest aborts must be resolved
      before the loader is enabled by default). **Blocked** — needs OpenUSD in
      the build env plus a root-cause fix for the macOS abort.

## Goal

Make `.usda` / `.usd` scenes loadable through DART's unified IO front door so a
researcher can drop a USD asset into a DART 7 `World` and visualize it through
the maintained Filament renderer, with parity to the existing URDF / SDF / MJCF
paths.

## Specification Intake

- **Value:** researchers can load OpenUSD scenes through the same
  `dart::io` / `dart::simulation::io` front door as URDF/SDF/MJCF.
- **Scope:** `dart/io/usd/`, `dart/io/read.{hpp,cpp}`, `cmake/dart_find_pxr.cmake`,
  `data/usd/`, `tests/{unit,integration}/io/`, plus (later phases) an
  `examples/` Filament viewer and `python/dartpy` bindings. Enabling OpenUSD by
  default additionally touches `pixi.toml` (a separate infrastructure lane).
- **Assumptions:** the loader stays OFF by default until the OpenUSD build-env
  slice lands; only textual `.usda` is read in the current slice (binary `.usdc`
  / packaged `.usdz` deferred).
- **Traceability:** this folder; PR #3109 (Phase 1); DART 7 IO front door owner
  doc `docs/onboarding/io-parsing.md`; renderer owner doc
  `docs/onboarding/gui-rendering.md`; active viewer-app plan PLAN-101
  (`docs/plans/101-dartsim-gui-simulator.md`).

## Blocker / Approval boundary

Phases 2–4 are all gated on **OpenUSD (pxr) being available in the build
environment**, which the current DART build does not have:

- `pixi.toml` has no `pxr`/`openusd` dependency, and the default build keeps
  `DART_BUILD_IO_USD=OFF`. Enabling it requires adding OpenUSD to the build env.
  `openusd` is available on conda-forge (e.g. `openusd 26.05`), so this is
  feasible, but the Phase 1 session recorded that adding `pxr` to `pixi.toml`
  **collides with the `pixi.toml` lane and is a separate follow-up slice** — it
  was intentionally kept out of the loader-semantics work.
- Phase 4 additionally carries an **unresolved macOS blocker**: the prototype's
  dartpy pytest aborts on macOS (likely an OpenUSD plugin-path / fork-safety
  issue) were never root-caused. This must be resolved before the loader can be
  enabled by default in the Linux + macOS Pixi environments.

Because the remaining phases cannot be built or verified without the OpenUSD
build-env slice, and that slice is a recorded separate lane, a maintainer
decision is needed before proceeding (see [Open Decisions](#open-decisions)).

## Non-Goals (for early phases)

- Writing USD back out — read-only loader first.
- Authoring USD physics schema (USDPhysics) features beyond the minimum needed
  to land a working rigid-body chain plus the Unitree H1 minimal sample.
- Re-introducing the OSG viewer that the prototype example used; the project has
  standardized on the Filament renderer (owner doc
  `docs/onboarding/gui-rendering.md`; the backend-hidden GUI and renderer
  performance plans PLAN-060 / PLAN-090 completed and were archived 2026-07-03).

## Acceptance Evidence

- Phase 1 (landed): OFF-path guard covered by
  `tests/unit/io/test_read.cpp::ReadUnit.InfersFormatFromUsdExtension`; ON-path
  end-to-end covered by `tests/integration/io/test_usd_parser.cpp` (compiled only
  when `DART_BUILD_IO_USD=ON`). `#3109` passed CI's clean OFF-path build at merge.
- Phase 2+: a `DART_BUILD_IO_USD=ON` build that maps a multi-prim USD stage to a
  DART 7 multibody with the expected links/joints, exercised by the (currently
  gated) integration test extended beyond the single-link case.

## Gates

- OFF-path (default, no OpenUSD): `pixi run lint`, `pixi run build`,
  `pixi run test-unit` (`InfersFormatFromUsdExtension`),
  `pixi run check-api-boundaries`, `pixi run test-py`.
- ON-path (requires an OpenUSD/pxr-enabled environment):
  `cmake -DDART_BUILD_IO_USD=ON ...` then run `INTEGRATION_io_Usd`.
- Phase 3: GUI smoke runs the USD sample headlessly under the dartsim engine.
- Phase 4: dartpy unit tests pass on Linux + macOS Pixi environments without
  pytest aborts.

## Open Decisions

**Decision (2026-07-04): keep this folder parked.** A maintainer chose to leave
the refreshed dev-task folder as the active-but-blocked task surface and resume
Phase 2 when the OpenUSD build-env lane lands, rather than opening the deferred
`pixi.toml` lane now or retiring the folder into a plan. The alternatives below
are kept for rationale/history; do not re-litigate them without new direction.

1. **Stand up the OpenUSD build-env slice now** — add `openusd` to a Pixi
   feature/environment (or a dedicated CI environment) so `DART_BUILD_IO_USD=ON`
   can be built and the ON-path verified, then implement Phase 2. Cost: touches
   the `pixi.toml` lane the Phase 1 session deferred; heavy dependency.
2. **Promote + retire** — move the durable USD front-door decision into
   `docs/onboarding/io-parsing.md`, convert Phases 2–4 into a durable plan entry
   (e.g. under `docs/plans/`), and remove this dev-task folder, tracking the
   OpenUSD-gated work there instead of in a temporary folder.
3. **Keep parked** — leave this folder as the active-but-blocked task surface
   until the OpenUSD build-env lane lands, then resume Phase 2.

## Key Decisions

- **Front door**: extend `dart::io::ModelFormat` with `Usd` and route reads
  through DART 7 loaders instead of restoring the retired public
  `dart::io::readWorld` API. Do not create a parallel `dart::utils::UsdParser`
  namespace as the prototype did.
- **Naming**: snake_case headers under `dart/io/usd/` (e.g.
  `usd_parser.{cpp,hpp}`), per `docs/onboarding/code-style.md` and the completed
  snake_case migration. The prototype's PascalCase paths
  (`dart/utils/usd/UsdParser.{cpp,hpp}`) predate the migration.
- **World target**: bind against the DART 7 `World` (`dart/simulation/`) through
  the `dart::simulation::io::addSkeleton` front door. PLAN-050 (DART 7 World
  binding transition) completed and was archived 2026-07-03, so this API is now
  stable — the loader no longer needs to "co-evolve" with an in-flight binding.
  The legacy `dart::simulation::World` does not need a USD path.
- **Viewer example**: build the example against the dartsim engine + Filament
  (active owner PLAN-101 `docs/plans/101-dartsim-gui-simulator.md`; renderer
  guidance in `docs/onboarding/gui-rendering.md`). The prototype's OSG-based
  `examples/usd_viewer/main.cpp` is incompatible with the retired-OSG policy.
- **Dependency**: pxr (OpenUSD). Discovery lives in `cmake/dart_find_pxr.cmake`
  (snake_case) and is opt-in via the `DART_BUILD_IO_USD` CMake toggle, mirroring
  the real `DART_BUILD_GUI` style. (`DART_BUILD_GUI_FILAMENT` referenced in older
  notes does not exist; the real Filament toggle is `DART_BUILD_GUI`.)
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

Files of interest in the prototype: `dart/utils/usd/UsdParser.{cpp,hpp}`,
`cmake/DARTFindpxr.cmake`, `data/usd/{simple_chain,unitree_h1_minimal}.usda`,
`python/dartpy/utils/UsdParser.cpp`,
`tests/integration/io/test_UsdParser.cpp`,
`python/tests/unit/test_usd_parser.py`.

## Immediate Next Steps

1. Resolve the [Open Decisions](#open-decisions) with a maintainer: decide
   whether to stand up the OpenUSD build-env slice, promote-and-retire, or keep
   the folder parked.
2. If proceeding with implementation: in an OpenUSD-enabled environment, build
   with `DART_BUILD_IO_USD=ON` and flesh out `usd::UsdParser` beyond the single
   default-prim link scaffold (map child prims to links/joints, then shapes and
   inertia), extending `tests/integration/io/test_usd_parser.cpp` past the
   single-link assertion. Keep the loader behind the `dart::io` front door and
   the API-boundary policy.
