# LCP Solver Interface And Demos — Dev Task

## Current Status

- [x] Consolidated the current work on one branch:
      `feature/lcp-solver-interface-demos`.
- [x] Merged the latest `origin/main` into the branch before publishing the
      active checkpoint stack.
- [x] Added representative LCP demo/benchmark packets for active
      friction-index contact cases.
- [x] Refined Dantzig friction-index solves so tangent bounds are re-solved
      after final normal-scaled friction bounds shift.
- [x] Exposed shared LCP problem validation diagnostics through C++ and dartpy:
      `LcpProblem::isValid()`, `LcpProblem::getValidationMessage()`,
      `LcpProblem.is_valid()`, and `LcpProblem.get_validation_message()`.
- [x] Tightened per-problem native support reporting so `DirectSolver` no
      longer reports large standard packets as native when it will delegate to
      Dantzig.
- [x] Captured the current hand-off state after the user explicitly requested
      no further verification or implementation work.
- [x] Exposed the DART 7 contact-pipeline comparison sweeps in py-demo LCP
      benchmark metadata.
- [x] Added a live billiard symmetry-error metric alongside momentum and energy
      error in the LCP py-demo.
- [x] Pointed the high-mass-ratio stack demo metadata at solver-manifest stack
      contact benchmarks as well as the boxed world-step benchmark.
- [x] Added GUI plots for billiard momentum, energy, and symmetry invariant
      histories in the LCP py-demo.
- [x] Added a representative benchmark-suite filter and command to the LCP
      py-demo metadata, derived from the benchmark packet table while keeping
      the smoke command intact.
- [x] Realigned LCP background docs with DART 7 snake_case solver header paths
      and extended the LCP roster lint gate to catch stale documented paths.
- [x] Captured the critical stop-and-handoff state without running further
      verification, including the interrupted MPRGP support-predicate audit.
- [x] Tightened MPRGP per-problem native support reporting so non-symmetric and
      non-positive-definite standard packets are marked delegated instead of
      native.
- [x] Tightened Baraff per-problem native support reporting so non-symmetric and
      indefinite standard packets delegate to Dantzig instead of entering the
      native active-set loop.
- [x] Refreshed the branch against `origin/main`, captured the final hand-off
      state, and stopped without running any further verification.
- [x] Filtered singular-degenerate standard benchmark registrations through
      concrete `supportsProblem(problem)` checks so MPRGP fallback rows are not
      listed as native while Baraff PSD rows remain.
- [x] Filtered contact-normal standard benchmark registrations through
      concrete normal-only contact packets, preserving MPRGP/Baraff native rows
      and replacing Direct's size special case with concrete support checks.
- [x] Captured the 2026-06-11 critical hand-off after the contact-normal
      checkpoint with no further lint, tests, benchmark listing, solver
      execution, or implementation work.
- [x] Added manifest-driven active friction-index contact benchmark rows to
      `lcp_compare` and retargeted the py-demo representative metadata from the
      older two-solver benchmark surface to that main comparison filter.
- [x] Filtered active-set transition benchmark registrations through concrete
      `supportsProblem(problem)` checks, replacing the Direct-only standard
      packet special case with the shared native-route gate.
- [ ] Continue the remaining DART 7 audit of LCP solver/problem interfaces and
      py-demo coverage from a fresh session.

## Goal

Keep the DART 7 standalone LCP surface coherent across C++, dartpy, tests,
benchmarks, and py-demos. A fresh session should be able to compare solver
families on standard, boxed, and friction-index packets, understand solver
capability limits, and extend representative challenging examples without
rediscovering the current branch state.

## Non-Goals

- Do not rebase or split the published branch unless the maintainer explicitly
  asks for history surgery.
- Do not treat the broad LCP goal as complete just because the validation API
  checkpoint is pushed.
- Do not retire this dev-task folder until remaining follow-up work is either
  completed or moved to durable planning/design docs.

## Key Decisions

- Use one additive feature branch,
  `feature/lcp-solver-interface-demos`, for the current LCP interface/demo
  continuation.
- Keep `detail::validateProblem(const LcpProblem&)` routed through the public
  `LcpProblem` validation diagnostic so C++ solvers, C++ tests, and Python
  demos share the same first-failure message.
- Treat `LcpSolver::supportsProblem(problem)` as per-problem native support,
  not only surface-family support. Solver-specific native limits should be
  reflected there when the public `solve()` path delegates internally.
- Leave the raw matrix/vector validation overload in `lcp_validation.hpp` in
  place for low-level call sites that validate temporary raw data without
  constructing another `LcpProblem`.
- Treat the active-bound two-contact friction-index packet as a regression and
  demo benchmark seed for Dantzig and friction-index-capable iterative solvers.

## Latest Code Checkpoint

The latest implementation checkpoint is the active-set transition benchmark
routing slice.

## Active-Set Transition Benchmark Routing Checkpoint

The latest implementation checkpoint aligns active-set transition benchmark
registration with concrete native solver support:

- `RegisterActiveSetTransitionBenchmarks()` now builds the concrete generated
  packet for each standard, boxed, and friction-index active-set family and
  registers only solvers whose `supportsProblem(problem)` accepts that packet.
- The previous Direct-only special case for the 16-row standard packet is gone;
  Direct is excluded by its own concrete native-support predicate.
- Baraff and MPRGP remain registered for the standard active-set transition
  packet because the generated matrix is symmetric positive-definite and
  satisfies their native predicates.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpActiveSetTransition/Standard/(Direct|Baraff|MPRGP)'
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE` and listed Baraff and MPRGP
  standard active-set transition rows, with no Direct row.
- `pixi run lint`: passed.

## Active Friction-Index Benchmark Routing Checkpoint

The latest implementation checkpoint moves the demo's active friction-index
contact benchmark metadata onto the main manifest-driven `lcp_compare` surface:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers
  `BM_LcpActiveFrictionIndexContact/FrictionIndex/<solver>` rows for every
  friction-index-capable manifest solver whose concrete
  `supportsProblem(problem)` accepts
  `LcpProblemFactory::activeFrictionIndexContact().problem`.
- The original narrower
  `BM_DantzigSolver_ActiveFrictionIndexContact` and
  `BM_PgsSolver_ActiveFrictionIndexContact` microbenchmark rows remain in
  `bm_lcpsolver_solvers.cpp`.
- `python/examples/demos/scenes/lcp_physics.py` now points the
  `active_friction_index_contact` representative filter at
  `BM_LcpActiveFrictionIndexContact`, so
  `representative_benchmark_command` runs rows that exist in `lcp_compare`.
- `python/tests/unit/test_py_demo_panels.py` now asserts that metadata.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpActiveFrictionIndexContact'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpActiveFrictionIndexContact' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE` and listed 16
  friction-index-capable rows: Dantzig, Pgs, SymmetricPsor, Jacobi,
  RedBlackGaussSeidel, BlockedJacobi, BGS, NNCG, SubspaceMinimization, Apgd,
  Tgs, ShockPropagation, Staggering, Admm, Sap, and BoxedSemiSmoothNewton.
- The short benchmark execution reported `contract_ok=1` for all 16 rows.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `pixi run lint`: passed.

## 2026-06-11 Critical Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only
with no further verification. No lint, build, tests, benchmark listing, or
solver execution was run after that instruction.

Branch state before this docs-only hand-off update:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `7ef5b79e602 Filter contact normal benchmark native rows`.
- Remote tracking branch: `origin/feature/lcp-solver-interface-demos` at
  `f86a095ce9a Document final LCP handoff state`.
- Local branch was two commits ahead of the tracking branch:
  - `fdc9a0c13fd Filter singular LCP benchmark native rows`
  - `7ef5b79e602 Filter contact normal benchmark native rows`
- `origin/main` was refreshed over HTTPS to `7d05d7b9ea72`, then
  `git merge --no-edit origin/main` reported `Already up to date.`
- No PR was associated with the branch when checked earlier in the session.

Interrupted next-slice audit, with no code changes made after the stop:

- `python/examples/demos/scenes/lcp_physics.py` still points the
  `active_friction_index_contact` benchmark packet at the older two-solver
  filter:
  `BM_DantzigSolver_ActiveFrictionIndexContact|BM_PgsSolver_ActiveFrictionIndexContact`.
- Those rows live in `tests/benchmark/lcpsolver/bm_lcpsolver_solvers.cpp`,
  while the representative command runs `pixi run bm lcp_compare`.
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` does not yet expose an
  equivalent manifest-driven `BM_LcpActiveFrictionIndexContact` row.
- A likely next bounded continuation is to add a concrete
  `LcpProblemFactory::activeFrictionIndexContact()` registration to
  `bm_lcp_compare.cpp`, gate participating solvers with
  `supportsProblem(problem)`, then update the py-demo metadata and panel test
  so the representative command points at the `lcp_compare` row.
- Re-inspect the generated problem and solver options before implementing; the
  note above is hand-off reconnaissance, not verified design.

## Contact-Normal Benchmark Routing Checkpoint

The latest implementation checkpoint aligns contact-normal standard benchmark
registration with concrete native solver support:

- `RegisterContactNormalStandardSweepBenchmarks()` now builds each concrete
  normal-only contact packet once and registers only solvers whose
  `supportsProblem(problem)` accepts that packet.
- Current MPRGP and Baraff rows remain registered for all contact-normal sweep
  cases because the generated packets satisfy their native predicates.
- Direct's previous `contactOrShapeCount > 3` registration special case is gone;
  its rows are now limited by the same concrete support predicate and remain
  only for 1-, 2-, and 3-row normal problems.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpContactNormalStandardSweep/(MPRGP|Baraff|Direct)'
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE`, listed all current Baraff
  and MPRGP contact-normal standard rows, and listed Direct only for the
  concrete 1-, 2-, and 3-row normal packets.
- `pixi run lint`: passed.

## Singular-Degenerate Benchmark Routing Checkpoint

The previous implementation checkpoint aligns singular-degenerate benchmark
registration with concrete native solver support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now has a benchmark-local
  `SolverSupportsConcreteProblem(...)` helper that instantiates the manifest
  solver and asks `supportsProblem(problem)` on the generated packet.
- `SolverShouldRunSingularDegenerateBenchmark(...)` still applies the scoped
  solver allowlists, but now intersects them with concrete support for the
  actual singular-degenerate packet.
- The singular-degenerate standard packets are symmetric positive-semidefinite
  but rank-deficient. Baraff remains registered as native; MPRGP no longer gets
  native singular-degenerate standard rows that would run through its Dantzig
  fallback.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp.*SingularDegenerate.*/(MPRGP|Baraff)'
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE` and listed Baraff
  singular-degenerate standard and standard-batch rows, with no MPRGP rows.
- `pixi run lint`: passed.

## Baraff Support Predicate Checkpoint

The latest implementation checkpoint makes Baraff's per-problem support
predicate match its documented symmetric-PSD native path:

- `dart/math/lcp/pivoting/baraff_solver.hpp` and `.cpp` now override
  `supportsProblem(problem, standardTolerance)` while preserving the base
  overloads.
- Baraff now reports native support only for standard problems with symmetric
  positive-semidefinite matrices.
- Boxed, friction-indexed, non-symmetric, and indefinite packets now delegate to
  Dantzig through the unified `solve()` path before entering the native Baraff
  active-set loop.
- C++ and dartpy tests cover the SPD native case, PSD native case, boxed false
  case, non-symmetric false case, indefinite false case, and fallback solve
  success for an indefinite standard packet.

Verification for this checkpoint:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=$N cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy --parallel "$N"
CTEST_PARALLEL_LEVEL=1 ctest --test-dir build/default/cpp/Release \
  --output-on-failure -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$' -j 1
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
```

Observed results:

- Targeted C++ build: passed.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke`: passed.
- `python/tests/unit/math/test_lcp.py`: 61 tests passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `scripts/check_lcp_solver_roster.py`: passed.

## MPRGP Support Predicate Checkpoint

The MPRGP support predicate checkpoint makes MPRGP's per-problem support
predicate match its actual native path:

- `dart/math/lcp/other/mprgp_solver.hpp` and `.cpp` now override
  `supportsProblem(problem, standardTolerance)` while preserving the base
  overloads.
- MPRGP now reports native support only for standard problems with symmetric
  matrices and, by default, a successful positive-definite factorization check.
- Boxed, friction-indexed, non-symmetric, and non-positive-definite packets can
  still be solved through fallback delegation, but no longer appear as native
  MPRGP rows in Python/demo capability reporting.
- C++ and dartpy tests now cover the SPD native case, boxed false case,
  non-symmetric false case, indefinite false default case, and the C++ member
  parameter case where `checkPositiveDefinite = false`.

Verification for this checkpoint:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=$N cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy --parallel "$N"
CTEST_PARALLEL_LEVEL=1 ctest --test-dir build/default/cpp/Release \
  --output-on-failure -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$' -j 1
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
```

Observed results:

- Targeted C++ build: passed.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke`: passed.
- `python/tests/unit/math/test_lcp.py`: 60 tests passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `scripts/check_lcp_solver_roster.py`: passed.

## Source-Layout Docs Checkpoint

The source-layout docs checkpoint, `7f4b0227eaf Align LCP docs with snake case
headers`, realigns the LCP background docs with the actual DART 7 source
layout:

- `docs/background/lcp/02_overview.md` now lists snake_case `dart/math/lcp`
  solver header/source paths in the implementation table, repository layout,
  section headings, and completion checklist.
- `docs/background/lcp/03_pivoting-methods.md`,
  `docs/background/lcp/05_newton-methods.md`, and
  `docs/background/lcp/06_other-methods.md` now use snake_case include paths in
  C++ snippets.
- `scripts/check_lcp_solver_roster.py` now scans LCP background docs for
  documented LCP header/source paths and fails if those paths do not exist.

Verification for this checkpoint:

```bash
pixi run python scripts/check_lcp_solver_roster.py
pixi run lint
```

Observed results:

- `pixi run python scripts/check_lcp_solver_roster.py`: passed.
- `pixi run lint`: passed.

## Critical Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only
with no further verification. No lint, build, tests, benchmark listing, or
solver execution was run after that final instruction.

Branch state at the start of this hand-off update:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD before this docs-only hand-off update:
  `b553a8ca444 Report Baraff native support precisely`.
- Remote tracking branch before this docs-only hand-off update:
  `origin/feature/lcp-solver-interface-demos` at
  `f3436654bbd Document critical LCP handoff state`.
- Local branch was two commits ahead of the tracking branch before this
  docs-only hand-off update.
- `origin/main` was refreshed over HTTPS and `git merge --no-edit origin/main`
  reported `Already up to date.`
- This docs-only hand-off update is intended to be committed and pushed to the
  same consolidated branch after capture.

Benchmark-routing audit notes:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` still has registrations that
  gate rows with manifest-level family support such as
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::Standard)`.
- Manifest-level gates are acceptable for synthetic packet families whose
  generated packets are already known to match the scoped solvers' native
  domains, but concrete generated packets should use
  `supportsProblem(problem)` before being exposed as native benchmark rows.
- The singular-degenerate standard registration gap was addressed by filtering
  rows through `supportsProblem(problem)` for the concrete generated packet.
- The contact-normal standard registration gap was addressed by filtering rows
  through `supportsProblem(problem)` for each concrete generated normal-only
  contact packet.
- Do not refactor every manifest support gate mechanically. Generic standard
  rows produced by `MakeBenchmarkProblem(Standard, size)`,
  `MakeStandardSpdProblem`, MPRGP SPD-check sweeps, interior-point path sweeps,
  and mild/near-singular SPD builders may already be correct.

## Representative Benchmark Command Checkpoint

The latest implementation checkpoint exposes a practical benchmark-suite
command in the LCP py-demo metadata:

- `benchmark_command` remains the quick smoke command for
  `BM_LCP_COMPARE_SMOKE`.
- `representative_benchmark_filter` is derived from every
  `_BENCHMARK_PACKET_ROWS[*]["benchmark_filter"]` token in table order.
- `representative_benchmark_command` wraps that filter as a runnable
  `pixi run bm lcp_compare` command.
- `python/tests/unit/test_py_demo_panels.py` now asserts the representative
  filter exactly matches the union of the benchmark packet table filters.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `pixi run lint`: passed.

## Billiard Invariant Plot Checkpoint

The latest implementation checkpoint adds GUI plots for billiard momentum,
kinetic-energy, and symmetry invariant histories in the LCP py-demo:

- `python/examples/demos/scenes/lcp_physics.py` now emits live plot streams for
  sequential impulse and boxed-LCP billiards:
  - `Sequential billiard momentum error`
  - `Boxed LCP billiard momentum error`
  - `Sequential billiard energy error`
  - `Boxed LCP billiard energy error`
  - `Sequential billiard symmetry error`
  - `Boxed LCP billiard symmetry error`
- `python/tests/unit/test_py_demo_panels.py` asserts those plot events are
  exposed by the demo panel.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `pixi run lint`: passed.

## Direct Support Checkpoint

This checkpoint tightened Direct solver native support reporting:

- `LcpSolver::supportsProblem(problem)` becomes virtual.
- `DirectSolver::supportsProblem(problem, standardTolerance)` reports native
  support only for standard packets with `problem.size() <= 3`.
- `DirectSolver::solve()` remains unchanged and can still delegate larger
  standard packets to Dantzig.
- The LCP py-demo representative-suite metadata counts Direct as delegated for
  the 4-row near-singular standard case and the 12-row moderate-scale standard
  case.

Files changed in this checkpoint:

- `CHANGELOG.md`
- `dart/math/lcp/lcp_solver.hpp`
- `dart/math/lcp/pivoting/direct_solver.cpp`
- `dart/math/lcp/pivoting/direct_solver.hpp`
- `docs/background/lcp/02_overview.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `docs/onboarding/python-bindings.md`
- `python/tests/unit/math/test_lcp.py`
- `python/tests/unit/test_py_demo_panels.py`
- `tests/unit/math/lcp/test_all_solvers_smoke.cpp`

Verification for the Direct support checkpoint:

```bash
pixi run lint
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_generated_coverage \
  --clean-first --parallel "$JOBS"
ninja -C build/default/cpp/Release -j "$JOBS" \
  UNIT_math_lcp_math_lcp_additional_solvers \
  UNIT_math_lcp_math_lcp_all_solvers_smoke \
  UNIT_math_lcp_math_lcp_dantzig_misc \
  UNIT_math_lcp_math_lcp_dantzig_solver \
  UNIT_math_lcp_math_lcp_dantzig_vs_ode \
  UNIT_math_lcp_math_lcp_lcp_comparison_harness \
  UNIT_math_lcp_math_lcp_lcp_edge_cases \
  UNIT_math_lcp_math_lcp_lcp_generated_coverage \
  UNIT_math_lcp_math_lcp_lcp_newton_solvers \
  UNIT_math_lcp_math_lcp_lcp_projection_solvers \
  UNIT_math_lcp_math_lcp_lcp_solvers_stress \
  UNIT_math_lcp_math_lcp_lcp_problems \
  UNIT_math_lcp_math_lcp_lcp_types \
  UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  UNIT_math_lcp_math_lcp_lemke \
  UNIT_math_lcp_math_lcp_pgs \
  UNIT_math_lcp_math_lcp_pivot_matrix
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_math_lcp_' -j "$JOBS"
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed results:

- `pixi run lint`: passed.
- `UNIT_math_lcp_math_lcp_lcp_generated_coverage`: passed after a clean rebuild
  removed stale vtable state from the local incremental build.
- `ctest -R '^UNIT_math_lcp_'`: 17/17 tests passed.
- `python/tests/unit/math/test_lcp.py`: 59 tests passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

## Previous Hand-Off Snapshot

The current session was stopped for hand-off only after the billiard invariant
plot checkpoint. No implementation work and no verification were run after the
user requested: "stop and focus on hand-off for all the current work without any
further verification."

Branch state at hand-off start:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD before this docs-only hand-off update:
  `cae4efcce30 Plot LCP billiard invariants`.
- Remote tracking branch before the hand-off docs update:
  `origin/feature/lcp-solver-interface-demos` at
  `b2f5632b277 Expose LCP problem validation diagnostics`.
- Local branch was six commits ahead of the tracking branch before this
  docs-only hand-off update.
- The earlier SSH fetch/push path failed on `github.com:22`, but a later HTTPS
  fetch of `origin/main` succeeded, and `origin/main` was confirmed to be an
  ancestor of `HEAD` before the contact-pipeline metadata checkpoint.

Interrupted next-slice reconnaissance, with no code changes made:

- The next bounded gap had shifted from solver coverage metadata to a practical
  representative benchmark command.
- `python/examples/demos/scenes/lcp_physics.py` still has a smoke-only command:
  `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`.
- The benchmark packet table already contains the representative per-packet
  filters, including active-set transitions, active friction-index contact,
  contact-pipeline sweeps, normal-only contact sweeps, degenerate and
  near-singular packets, batch scaling, billiards, stack, card-pile, and
  articulated-contact rows.
- A likely next bounded patch is to add separate metadata such as
  `representative_benchmark_filter` and `representative_benchmark_command`
  built from `_BENCHMARK_PACKET_ROWS`, while keeping `benchmark_command` as the
  quick smoke command for compatibility.
- Suggested test shape in `python/tests/unit/test_py_demo_panels.py`: split the
  representative filter on `|` and compare it with the union of every
  benchmark packet row filter token. Keep the smoke assertion for
  `BM_LCP_COMPARE_SMOKE`.

## Contact-Pipeline Metadata Checkpoint

The continuation after the hand-off snapshot kept the exact
`active_friction_index_contact` row pointed at its two-solver regression
benchmark, then added separate py-demo benchmark packet rows for the broader
DART 7 contact-pipeline comparisons registered in `bm_lcp_compare.cpp`:

- `contact_solver_comparison_sweep`:
  `BM_LcpContactSolverComparisonSweep|BM_LcpStaggeringContactPipelineSweep`
  for Staggering, ADMM, SAP, and BoxedSemiSmoothNewton contact-pipeline
  fixtures.
- `contact_normal_standard_sweep`: `BM_LcpContactNormalStandardSweep` for
  normal-only standard contact subproblems across standard-capable solvers.

Verification for this checkpoint:

```bash
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git merge-base --is-ancestor origin/main HEAD
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run bm lcp_compare -- --benchmark_list_tests \
  --benchmark_filter='BM_LcpContactSolverComparisonSweep|BM_LcpStaggeringContactPipelineSweep|BM_LcpContactNormalStandardSweep'
pixi run lint
```

Observed results:

- `origin/main` fetched successfully over HTTPS and is an ancestor of `HEAD`.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- Benchmark listing built `BM_LCP_COMPARE` and listed the referenced sweep
  benchmarks.
- `pixi run lint`: passed.

## Billiard Symmetry Metric Checkpoint

The LCP py-demo already tracked billiard momentum and kinetic-energy error.
This checkpoint adds `billiard_symmetry_error`, measured as the maximum lateral
drift of either billiard ball from its initial collision line, so the live
headless and GUI surfaces directly expose the symmetry invariant called out in
the LCP representative-example goal.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed result:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

## Stack Benchmark Filter Checkpoint

The high-mass-ratio stack live packet previously pointed only at
`BM_LcpWorldStackStep_BoxedLcp`. This checkpoint updates the demo benchmark row
to include `BM_LcpWorldStackContact/`, which is the manifest-driven
friction-index stack contact benchmark registered for the solver roster, while
keeping the boxed world-step benchmark for the live rollout analog.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run bm lcp_compare -- --benchmark_list_tests \
  --benchmark_filter='BM_LcpWorldStackContact/|BM_LcpWorldStackStep_BoxedLcp'
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- Benchmark listing showed `BM_LcpWorldStackContact/FrictionIndex/<solver>`
  rows and `BM_LcpWorldStackStep_BoxedLcp` rows.

## Billiard Invariant Plot Checkpoint

The live LCP panel now plots billiard momentum error, kinetic-energy error, and
symmetry error histories for both the sequential impulse and boxed-LCP worlds.
This keeps the table values and GUI time histories aligned for the billiards
dealbreaker example.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed result:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

## Verification Snapshot

Latest validation API checkpoint was verified with:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_types dartpy --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_types
pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Observed results:

- `UNIT_math_lcp_math_lcp_lcp_types`: 13 tests passed.
- `pixi run test-lcpsolver`: 17/17 tests passed.
- `python/tests/unit/math/test_lcp.py`: 58 tests passed.
- `pixi run lint`: passed.

Earlier branch checkpoints also ran the LCP demo panel tests and active
friction benchmark smoke rows; re-run those if the next change touches demo
metadata, packet generation, or benchmark rows.

## Immediate Next Steps

1. Resume on `feature/lcp-solver-interface-demos`.
2. Check that `origin/feature/lcp-solver-interface-demos` contains the final
   hand-off commit from this session. If it does not, push only after explicit
   maintainer/user approval is still in force.
3. Fetch `origin/main`; if it moved, merge it into this branch before the next
   push.
4. Continue the LCP interface/demo audit from the next concrete gap. Good
   starting points are remaining manifest-level benchmark gates in
   `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, the older
   `active_friction_index_contact` demo benchmark filter, and any solver whose
   native mathematical domain is still broader in docs than in
   `supportsProblem(problem)`.
5. Prefer a bounded checkpoint: one benchmark/demo routing gap, focused tests,
   `pixi run lint`, then an additive commit.
