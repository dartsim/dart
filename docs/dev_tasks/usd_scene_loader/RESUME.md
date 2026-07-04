# Resume: OpenUSD Scene Loader

## Current Reality (verified 2026-07-04)

A resume session re-audited this task against current `main`
(`0f628d15665`). Findings:

- **Phase 1 is confirmed landed on `main`** (`fab38a336f7`, PR #3109). All
  scaffold artifacts are present: `dart/io/usd/usd_parser.{hpp,cpp}`,
  `cmake/dart_find_pxr.cmake`, the `DART_BUILD_IO_USD` toggle, the `read.cpp`
  front-door guard, `data/usd/simple_chain.usda`, and the unit/integration tests.
- **The plans this task referenced as in-flight are now complete/archived**
  (all archived 2026-07-03): PLAN-050 (DART 7 World binding transition),
  PLAN-060 (backend-hidden GUI), PLAN-090 (Filament renderer performance). The
  README was refreshed to stop treating them as active: Phase 2 now targets the
  **stable** `dart::simulation::io::addSkeleton` front door directly, and the
  viewer work (Phase 3) aligns with the active PLAN-101 dartsim GUI simulator
  plus `docs/onboarding/gui-rendering.md`.
- **Phases 2–4 remain blocked** on OpenUSD being in the build env — a recorded
  separate `pixi.toml` lane — plus the unresolved macOS pytest abort for Phase 4.
  See the README "Blocker / Approval boundary" and "Open Decisions" sections;
  a maintainer decision is needed before implementation continues.
- **Local build-tree caveat:** `build/default/cpp/Release` is stale (configured
  2026-06-19 with system `/usr/bin/c++`) and its incremental relink now fails
  against the updated conda toolchain (`@GLIBC_PRIVATE` symbol mismatches and a
  stale libcurl link interface). This is an environment-drift issue unrelated to
  the USD code. The OFF-path guard was therefore verified by code inspection plus
  the fact that #3109 passed CI's clean OFF-path build at merge; a clean local
  reconfigure (`rm -rf build/default/cpp/Release && pixi run config && pixi run
build`) is needed before any local build/test will pass again.

## Immediate Next Step

**Decision recorded (2026-07-04): parked.** The maintainer chose to keep this
folder as the active-but-blocked task surface (see the README "Open Decisions").
The next actionable step is external to this task: land the OpenUSD build-env
lane (add `openusd` to a Pixi feature/environment so `DART_BUILD_IO_USD=ON` can
build). Once that exists, resume Phase 2 — flesh out `usd::UsdParser` beyond the
single default-prim link and extend `tests/integration/io/test_usd_parser.cpp`.
Do not write Phase 2 parser code until an OpenUSD-enabled build exists to verify
it.

---

## Last Session Summary (Phase 1, 2026-06-20)

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

## Current Branch / PR

PR [#3109](https://github.com/dartsim/dart/pull/3109)
(`feature/usd-loader-phase1`) merged to `main` on 2026-06-20 at
`fab38a336f7`. Start Phase 2 from current `main`.

## What landed in PR #3109 (Phase 1 scaffold)

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

## Context That Would Be Lost

- The OFF-path guard is verified by `tests/unit/io/test_read.cpp`
  (`InfersFormatFromUsdExtension`); the ON-path test cannot run locally because
  pxr is absent, so it is correctly gated to compile out when OFF.
- Enabling OpenUSD by default / adding `pxr` to `pixi.toml` is a **separate
  follow-up slice** (it collides with the `pixi.toml` lane); it was intentionally
  not done in the Phase 1 PR.
- The viewer example must target Filament + the dartsim engine, not OSG
  (PLAN-060 / PLAN-090, both now archived). That is Phase 3.
- The prototype's macOS pytest aborts were never root-caused; treat as a Phase 4
  blocker before the loader is enabled by default.

## How to Resume

```bash
git checkout main
git pull --ff-only origin main
git status && git log -3 --oneline

# Default (toggle OFF) gates — must stay green (needs a clean local reconfigure
# first; see Current Reality):
pixi run lint
pixi run build
pixi run test-unit
pixi run check-api-boundaries
pixi run test-py

# ON-path (requires an OpenUSD/pxr-enabled environment):
#   cmake -DDART_BUILD_IO_USD=ON ...   # then run INTEGRATION_io_Usd
```

Then read `docs/dev_tasks/usd_scene_loader/README.md` — resolve the Open
Decisions before starting Phase 2 implementation.
