# CI Optimization - Dev Task

Temporary working surface for the CI optimization initiative on `main`
(DART 7). The durable owner is
[`../../onboarding/ci-cd.md`](../../onboarding/ci-cd.md); this folder is
deleted by the completing PR. There is no numbered plan file: the packets
below are the plan.

## Goal

Keep every CI coverage dimension somewhere in the tiers (PR / main push /
schedule / manual) while cutting PR wall-clock, job count, and configuration
duplication, and making the silent regressions below impossible to repeat.

## Evidence (2026-09-05)

- Compiler cache silently off on every GitHub-hosted runner since the Actions
  cache-service v2 migration: `configure-compiler-cache` gated sccache on the
  removed `ACTIONS_CACHE_URL`, and `scripts/cmake_config.py` turns the whole
  cache off when `SCCACHE_GHA_ENABLED=false`. Builds are >95% of job time
  (Linux Release: 53 min build / 21 s ctest; Windows: ~100 min build / 32 s
  ctest).
- The shared doc-only skip never fired: `.github/filters/ci-code.yml` relied
  on `!` exclusions that `dorny/paths-filter` OR-ed under its default
  quantifier, so `code` was always true (since #2655, 2026-05-16).
- The 10 GB Actions cache was full: CodeQL C++ TRAP caches (2 x 3 GB) and
  pixi environment caches (~3.3 GB) left nothing for sccache.
- The Windows `Tests (Release)` job cannot use a compiler launcher at all:
  the pixi Windows tasks configure a multi-config Visual Studio tree and
  MSBuild ignores `CMAKE_<LANG>_COMPILER_LAUNCHER`.
- Baseline for a normal code PR (#3455, 2026-08-29): 27 jobs, ~735
  job-minutes, ~107 min wall-clock; the free-plan cap of 20 concurrent jobs
  adds up to 35 min of queue wait per job.

## Decisions (maintainer, 2026-09-05)

- D1: fold redundant jobs and move Eigen 64-byte + Linux Clang GUI smoke to
  the continuous tier (no required-check change).
- D2: keep ubuntu, macos-15, and windows wheels on every PR.
- D3: drop the feature-branch push tier.
- D4: two sequenced PRs; PR-2 is sized against PR-1's warm-cache numbers.

## Packets

### PR-1 `ci/fix-compiler-cache-and-path-filter` (this branch)

Defects and hygiene, no job added to or removed from the PR tier:

- Re-enable sccache with a fail-loud guard and a backend assertion
  (`configure-compiler-cache`); drop `SCCACHE_NO_DAEMON`; Windows keeps the
  Visual Studio generator uncached (see the follow-up packet).
- Cache budget: pixi environment cache and CodeQL TRAP cache off.
- Fix and widen the code filter (`predicate-quantifier: some-with-excludes`,
  AI-tool and docs exclusions) with `tests/test_ci_code_filter.py`; Windows
  hook smoke moved to its own path-gated job.
- `permissions`/`timeout-minutes` everywhere, static wheel matrix, Dependabot
  coverage for `.github/actions/*`, first `ci-cd.md` sync.

### PR-2 `ci/consolidate-tiers-and-dedupe` (after PR-1 numbers)

- Drop the feature-branch push tier (`ci_ubuntu.yml` `push: "**"`, the
  `build-core` job, the open-PR probe, `.github/actions/ci-scope`);
  `ci_gz_physics.yml` on `main` becomes dispatch-only.
- Delete `Native Collision (no FCL/Bullet/ODE)`: the default env has no
  FCL/Bullet/ODE and the CMake defaults are OFF, so it configures exactly what
  `Release Tests` configures and runs a subset of its tests
  (`collision-native` labels, full Python suite).
- Fold the `dart-demos --headless` PPM check and the GCC `dartsim` GUI smoke
  into `Release Tests` (examples are already built there); rename the
  headless job to a visual-verification job that builds only the
  `Release-docking` dartpy tree (`DART_USE_SYSTEM_IMGUI=OFF`); delete the GCC
  GUI-smoke leg. Update `scripts/ai_infrastructure.py::check_ci_wiring`
  (section boundary is the `filament-gui-smoke:` job key) and its tests.
- Move `Eigen 64-byte alignment` and the Clang GUI smoke to
  `github.event_name != 'pull_request'` like `build-asan`.
- Weekly schedule (Sunday only) for `ci_ubuntu`, `ci_macos`, `ci_windows`,
  `publish_dartpy`, `performance_dashboard`.
- YAML anchors for the pre-checkout clean and disk-cleanup steps (GitHub
  supports anchors/aliases, not merge keys) and one Linux setup composite
  (setup-pixi-ci + apt + compiler cache) used by `ci_ubuntu`, `codeql`,
  `performance_dashboard`, `benchmark_pr_comparison`, `ci_simd`.
- `ci_simd.yml` paths per the `ci-cd.md` guardrail (root `CMakeLists.txt`,
  `pixi.toml`, `pixi.lock`); Coverage allowed on `workflow_dispatch`; stale
  comments; `ci-cd.md` tiering table and Expected CI Times refreshed; delete
  this folder.

### Follow-up: Windows compiler cache

Either configure the Windows test job with Ninja under a Visual Studio
developer environment (`DART_WINDOWS_CMAKE_GENERATOR=Ninja` plus
`-DCMAKE_BUILD_TYPE`, which the win-64 pixi tasks do not pass today, and a
`vcvars` activation step) or keep the Visual Studio generator and pass
sccache's MSBuild integration
(`-DCMAKE_VS_GLOBALS=CLToolExe=sccache.exe;CLToolPath=<dir>;TrackFileAccess=false;UseMultiToolTask=true`),
which needs an extra-arguments passthrough in `scripts/cmake_config.py`.
Windows is the PR long pole (~100 min uncached), so this is the largest
remaining wall-clock win; it changes what the Windows check exercises, so it
gets its own PR.

## Maintainer checklist (GitHub settings, not automated)

- Required checks on `main` stay: `Lint`, `Coverage (Debug)`, `Debug Tests`,
  `Release Tests`, `Asserts enabled (no -DNDEBUG)`, `Release Tests (arm64)`,
  `Debug Tests (arm64)`, `Tests (Release)`, `Wheels`, the two Read the Docs
  builds. Nothing in PR-1 or PR-2 renames them.
- Consider disabling the GitHub default "CodeQL - Code Quality" dynamic
  workflow if it duplicates `codeql.yml`.
- Decide whether `ci_macos.yml` should pin `macos-15` (wheel parity) or stay
  on `macos-latest` (currently macOS 26); record the choice in `ci-cd.md`.

## Non-goals

- No external sccache backend unless the GHA cache measurably thrashes.
- No `release-6.*` workflow changes.

## Acceptance evidence

- PR-1: a second push of a code change shows non-zero sccache hits in the
  sccache-action post-step summary on Linux and macOS jobs and on the macOS
  and Windows wheels, with build steps >= 40% shorter than the cold run; a
  docs-only push reports `Filter code = false` and skips every platform and
  wheel job with required checks green;
  `gh api repos/dartsim/dart/actions/cache/usage` shows the CodeQL/pixi
  entries gone.
- PR-2: PR job count <= 20 with every coverage dimension mapped to a tier in
  `ci-cd.md`; `ci_ubuntu.yml` shrinks by >= 400 lines; all guard scripts pass.

## Gates

`pixi run lint`, `pixi run check-lint`, `pixi run test-ai-infra`,
`pixi run check-ai-infra`, `pixi run check-diff-workflow`,
`pixi run check-phase5-cuda-workflow`, `pixi run check-docs-policy`, YAML
parse of every workflow, and the live PR runs listed above.

## Status

See [`RESUME.md`](RESUME.md) for the current stop point.
