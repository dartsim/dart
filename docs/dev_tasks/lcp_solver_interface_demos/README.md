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
2. Fetch `origin/main`; if it moved, merge it into this branch before the next
   push.
3. Push only after explicit maintainer/user approval.
4. Audit the next smallest LCP interface/demo gap. Good starting points are
   `dart/math/lcp`, `python/examples/demos`, `python/tests/unit/math/test_lcp.py`,
   `python/tests/unit/test_py_demo_panels.py`,
   `tests/common/lcpsolver`, and `tests/benchmark/lcpsolver`.
5. Prefer a bounded checkpoint: one solver/interface/demo gap, focused tests,
   `pixi run lint`, then an additive commit.
