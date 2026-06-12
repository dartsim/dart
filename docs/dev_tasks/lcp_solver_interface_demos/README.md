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

## Latest Checkpoint

The latest checkpoint tightens Direct solver native support reporting:

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

## Hand-Off Snapshot

The current session was stopped for hand-off only after the Direct support
checkpoint. No implementation work and no verification were run after the user
requested: "stop and focus on hand-off for all the current work without any
further verification."

Branch state at hand-off start:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `80a31ddb2b5 Report Direct LCP native support precisely`.
- Remote tracking branch before the hand-off docs update:
  `origin/feature/lcp-solver-interface-demos` at
  `b2f5632b277 Expose LCP problem validation diagnostics`.
- `git fetch origin main` failed in this environment with
  `ssh: connect to host github.com port 22: Network is unreachable`, so the
  latest remote `main` could not be refreshed before this hand-off checkpoint.

Interrupted next-slice reconnaissance, with no code changes made:

- The next bounded gap was going to audit whether demo/benchmark comparison
  metadata reflects all implemented LCP solver families.
- `python/examples/demos/scenes/lcp_physics.py` currently maps the
  `active_friction_index_contact` benchmark packet to the older two-solver
  filter:
  `BM_DantzigSolver_ActiveFrictionIndexContact|BM_PgsSolver_ActiveFrictionIndexContact`.
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` appears to contain broader
  manifest-driven contact comparison registrations. Inspect
  `kContactComparisonSolverNames`, `kContactNormalStandardSolverNames`, the
  world/contact benchmark functions, and the final registration block before
  changing demo metadata.
- `tests/benchmark/lcpsolver/bm_lcpsolver_solvers.cpp` looks like an older,
  narrower benchmark surface for Dantzig, Lemke, and PGS. Confirm whether it is
  secondary before pointing demos at it.
- A likely next bounded patch is to update the demo benchmark filter and
  `python/tests/unit/test_py_demo_panels.py` expectations to use the broader
  manifest-driven benchmark, if source inspection confirms that is the intended
  comparison surface.

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
2. Check whether the local hand-off commits are already on
   `origin/feature/lcp-solver-interface-demos`; if not, push them after network
   access is available and maintainer/user approval is still in force.
3. Fetch `origin/main`; if it moved, merge it into this branch before the next
   push.
4. Audit the next smallest LCP interface/demo gap. Good starting points are
   `dart/math/lcp`, `python/examples/demos`, `python/tests/unit/math/test_lcp.py`,
   `python/tests/unit/test_py_demo_panels.py`,
   `tests/common/lcpsolver`, and `tests/benchmark/lcpsolver`.
5. Prefer a bounded checkpoint: one solver/interface/demo gap, focused tests,
   `pixi run lint`, then an additive commit.
