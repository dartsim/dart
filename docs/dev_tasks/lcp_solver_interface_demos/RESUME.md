# Resume: LCP Solver Interface And Demos

## Last Session Summary

The current work is consolidated on `feature/lcp-solver-interface-demos`.
Recent checkpoints added active friction-index demo/benchmark coverage, refined
Dantzig friction-index fixed-bound re-solves, and exposed direct LCP problem
validation diagnostics in C++ and dartpy. The latest checkpoint tightens
`supportsProblem(problem)` so solver-specific native limits are visible to
py-demos, starting with `DirectSolver` treating only n <= 3 standard packets as
native and marking larger standard packets as delegated.

The latest continuation exposed the broader DART 7 contact-pipeline benchmark
comparison sweeps in the LCP py-demo metadata while keeping the exact
`active_friction_index_contact` row pointed at its two-solver regression
benchmark.

## Current Branch

`feature/lcp-solver-interface-demos` — before this hand-off docs update, local
HEAD was `80a31ddb2b5 Report Direct LCP native support precisely` and the
tracking branch was `origin/feature/lcp-solver-interface-demos` at
`b2f5632b277 Expose LCP problem validation diagnostics`.

The earlier SSH fetch/push failed on `github.com:22`, but HTTPS fetch later
succeeded. `origin/main` was confirmed to be an ancestor of `HEAD` before the
contact-pipeline metadata checkpoint.

## Immediate Next Step

Resume by inspecting branch state. If local commits are not on
`origin/feature/lcp-solver-interface-demos`, fetch `origin/main`, merge it if it
moved, then push the consolidated branch once network access is available and
maintainer/user approval is still in force. Continue with one bounded LCP
solver/interface/demo gap at a time.

## Context That Would Be Lost

- Public callers can now ask `LcpProblem::isValid()` /
  `getValidationMessage()` and Python `problem.is_valid()` /
  `problem.get_validation_message()` before solving.
- `detail::validateProblem(const LcpProblem&)` now delegates to the public
  diagnostic path, so solver rejection messages and demo diagnostics stay
  aligned for `LcpProblem` inputs.
- The raw validation overload in `lcp_validation.hpp` intentionally remains for
  raw matrix/vector validation call sites.
- The active friction-index contact packet is meant to exercise moving tangent
  bounds where a single Dantzig pass could leave stale fixed bounds.
- Current Direct support change: `DirectSolver` still solves larger standard
  packets through Dantzig fallback, but `supportsProblem()` returns false for
  those packets so demo native vs delegated rows match the actual route.
- `LcpSolver::supportsProblem(problem)` is virtual in the working tree so
  dartpy calls through the base binding can dispatch to `DirectSolver`'s
  override.
- `python/tests/unit/test_py_demo_panels.py` now expects Direct to be the only
  non-native solver for the 4-row near-singular standard case and the 12-row
  moderate-scale standard case.
- Verification for the Direct checkpoint passed: `pixi run lint`;
  clean rebuild of `UNIT_math_lcp_math_lcp_lcp_generated_coverage`; direct
  Ninja rebuild of all `UNIT_math_lcp_*` binaries after the clean; CTest
  `-R '^UNIT_math_lcp_'` passed 17/17; `python/tests/unit/math/test_lcp.py`
  passed 59 tests; and `python/tests/unit/test_py_demo_panels.py` passed
  43 tests.
- The first `pixi run test-lcpsolver` attempt exposed stale local vtable state
  after adding virtual methods to `LcpSolver`; a clean rebuild of the generated
  coverage binary removed the segfault.
- The branch should remain one additive published branch. Merge latest `main`
  before future pushes; avoid rebasing unless explicitly requested.
- No verification was run after the user's stop-and-handoff instruction. The
  last verification remains the Direct checkpoint verification listed below.
- The interrupted next-slice audit found that
  `python/examples/demos/scenes/lcp_physics.py` still points
  `active_friction_index_contact` at the older two-solver benchmark filter
  `BM_DantzigSolver_ActiveFrictionIndexContact|BM_PgsSolver_ActiveFrictionIndexContact`.
  Inspect the broader manifest-driven registrations in
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` before changing this metadata.
- Good source locations for that next audit are `kContactComparisonSolverNames`,
  `kContactNormalStandardSolverNames`, the world/contact benchmark functions,
  and the registration block near the end of `bm_lcp_compare.cpp`.
- `tests/benchmark/lcpsolver/bm_lcpsolver_solvers.cpp` appears to be a narrower
  older benchmark surface; confirm whether it is secondary before wiring demos
  to it.
- The contact-pipeline metadata checkpoint resolved the immediate missing
  py-demo rows for `BM_LcpContactSolverComparisonSweep`,
  `BM_LcpStaggeringContactPipelineSweep`, and
  `BM_LcpContactNormalStandardSweep`.
- Verification for that checkpoint passed:
  `python/tests/unit/test_py_demo_panels.py` passed 43 tests;
  `pixi run bm lcp_compare -- --benchmark_list_tests` listed the referenced
  sweep benchmarks; and `pixi run lint` passed.

## How to Resume

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git log --oneline --decorate --max-count=12
```

Expected files in the Direct support checkpoint:

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

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/background/lcp/02_overview.md`
- `docs/background/lcp/07_selection-guide.md`
- `docs/onboarding/python-bindings.md`
- `docs/onboarding/testing.md`

If the fetch succeeds and `origin/main` is not an ancestor of `HEAD`, merge it
before pushing or continuing:

```bash
git merge origin/main
```

If the Direct support checkpoint is still uncommitted, confirm the working tree
matches the expected files above and commit it with:

```text
Report Direct LCP native support precisely
```

Verification already run for that checkpoint:

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

Also run relevant LCP benchmark smoke rows if touching demo metadata, benchmark
manifests, or generated representative packets.
