# Resume: OpenUSD Scene Loader

## Last Session Summary

Landed the Phase 1 **scaffold**: a snake_case `dart/io/usd/` module behind a new
OFF-by-default `DART_BUILD_IO_USD` CMake toggle. `dart::io::ModelFormat::Usd`,
auto-detection of textual USD (`.usda` only — the Phase 1 loader imports the
layer from text, so binary Crate `.usdc` and packaged `.usdz` are deferred to a
later slice), and a guarded `readSkeleton` switch case now exist behind the
DART 7 model-loading front door.
With the toggle OFF (default, no OpenUSD in the build env) the dormant USD code
compiles and the front door returns a clean "USD support is not available"
diagnostic instead of a link error — mirroring the existing SDF/URDF
unavailable paths. The loader itself (`usd::UsdParser`) and its OpenUSD
dependency are compiled only when the toggle is ON.

## Current Branch

`feature/usd-loader-phase1` — branched off `main` (clean). Implementation
committed for PR to milestone "DART 7.0".

## Current Reality (verified 2026-06-20)

- OpenUSD/pxr is **not** in the build env (no `pxr` in `pixi.toml`, no pxr CMake
  module before this work), so the toggle must stay OFF by default. Enabling it
  by default / adding `pxr` to `pixi.toml` is a **separate follow-up slice** (it
  collides with the `pixi.toml` lane); do not do it here.
- The owner doc (`README.md`) and earlier resume notes referenced a
  `DART_BUILD_GUI_FILAMENT` toggle to model on — that option does not exist. The
  real Filament toggle is `DART_BUILD_GUI`; the USD toggle mirrors it.
- The prior `feature/usd-viewer` prototype (SHA `28bad2773d1`) is no longer
  reachable from this worktree (reflog/fsck expired). Phase 1 artifacts were
  authored fresh as snake_case under `dart::io`, not ported from the PascalCase
  `dart::utils::UsdParser` prototype.

## What landed this PR (Phase 1 scaffold)

- `dart_option(DART_BUILD_IO_USD ... OFF)` in the root `CMakeLists.txt`.
- `cmake/dart_find_pxr.cmake` — snake_case OpenUSD (pxr) finder.
- `dart/io/usd/usd_parser.{hpp,cpp}` + `dart/io/usd/CMakeLists.txt` — compiled
  into `dart-io` only when the toggle is ON; sets `DART_IO_HAS_USD=1`.
- `dart/io/read.{hpp,cpp}` — `ModelFormat::Usd`, extension inference, and the
  `#if DART_IO_HAS_USD … #else DART_ERROR … #endif` guard.
- `data/usd/simple_chain.usda` — minimal single-link sample.
- `tests/unit/io/test_read.cpp` — `InfersFormatFromUsdExtension` exercises the
  OFF-path guard (always compiled).
- `tests/integration/io/test_usd_parser.cpp` (+ gated registration in
  `tests/integration/CMakeLists.txt`) — ON-path end-to-end test, compiled only
  when `DART_BUILD_IO_USD=ON`.

## Immediate Next Step

Phase 2 (PLAN-050): in an OpenUSD-enabled environment, build with
`DART_BUILD_IO_USD=ON` and flesh out `usd::UsdParser` to map USD prims to DART 7
`World` shapes/joints (currently it creates a single free-floating link for the
stage default prim). Keep the loader behind the `dart::io` front door and the
API-boundary policy.

## Context That Would Be Lost

- The OFF-path guard is verified by `tests/unit/io/test_read.cpp`
  (`InfersFormatFromUsdExtension`); the ON-path test cannot run locally because
  pxr is absent, so it is correctly gated to compile out when OFF.
- The viewer example must target Filament + the dartsim engine, not OSG
  (PLAN-060 / PLAN-090 retired OSG). That is Phase 3.
- The prototype's macOS pytest aborts were never root-caused; treat as a Phase 4
  blocker before the loader is enabled by default.

## How to Resume

```bash
git checkout feature/usd-loader-phase1   # or main, once this PR has merged
git status && git log -3 --oneline

# Default (toggle OFF) gates — must stay green:
pixi run lint
pixi run build
pixi run test-unit
pixi run check-api-boundaries
pixi run test-py

# ON-path (requires an OpenUSD/pxr-enabled environment):
#   cmake -DDART_BUILD_IO_USD=ON ...   # then run INTEGRATION_io_Usd
```

Then start Phase 2 from `docs/dev_tasks/usd_scene_loader/README.md`.
