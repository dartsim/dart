# Resume: LCP Solver Interface And Demos

## Current Reality — 2026-06-12 Staggering Native Support Reporting

After the stop-only hand-off, work resumed on the broad LCP solver/interface
goal. The active slice tightens `StaggeringSolver::supportsProblem()` so native
support reporting matches the actual Staggering friction-block route rather
than its standard/boxed Dantzig fallback routes.

Latest implementation slice:

- `StaggeringSolver` now reports native support only for friction-index packets
  containing both normal and friction rows.
- Standard, boxed no-friction, near-standard, and large standard packets report
  false for Staggering native support, while `solve()` can still succeed
  through fallback delegation.
- C++ smoke tests cover the updated support predicates and an explicit
  standard-packet fallback solve.
- dartpy LCP tests mirror those expectations and keep a fallback solve check.
- `CHANGELOG.md` and the dev-task README describe the native-support-reporting
  behavior.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke \
  --gtest_filter='AllSolversSmokeTest.SolverCapabilityPredicatesClassifyProblemForms:AllSolversSmokeTest.SolverCapabilityPredicatesUseDefaultToleranceForNearStandardForm:AllSolversSmokeTest.StaggeringReportsOnlyFrictionBlockProblemsAsNative:AllSolversSmokeTest.NearSingularStandardProblemProducesExpectedIterates'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- The C++ smoke target and dartpy rebuilt successfully.
- Focused C++ smoke coverage passed 4/4 tests.
- `python/tests/unit/math/test_lcp.py`: 78 passed.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the LCP solver roster gate.

## Immediate Next Step

After the checkpoint commit, continue the broad DART 7 LCP objective with the
next concrete solver/interface/demo/performance gap. Do not mark the overall
goal complete from this focused support-reporting slice, and do not push
without explicit approval.

## Current Reality — 2026-06-12 Remaining Parameter Validation

The user resumed the broad LCP solver/interface/demo goal after the stop-only
hand-off. The remaining parameter-validation audit has been implemented and
targeted verification has passed locally.

Latest implementation slice:

- `PgsSolver`, `ApgdSolver`, and `TgsSolver` reject invalid
  `epsilonForDivision`; APGD also rejects negative `restartCheckInterval`
  while keeping `0` valid as the every-iteration restart-check mode.
- `MinimumMapNewtonSolver`, `FischerBurmeisterNewtonSolver`, and
  `PenalizedFischerBurmeisterNewtonSolver` reject invalid exposed line-search,
  gradient-descent warm-start, PGS warm-start, smoothing, and penalty
  parameters before numerical iteration.
- `InteriorPointSolver` rejects invalid `sigma` and `stepScale` instead of
  silently clamping them.
- C++ validation tests and dartpy LCP tests cover invalid custom parameter
  structs/objects across the affected solvers.
- Existing Newton warm-start tests now use valid line-search parameters, and
  invalid line-search counts are tested as `InvalidProblem` diagnostics.
- `CHANGELOG.md` and the dev-task README describe the new validation behavior.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
           UNIT_math_lcp_math_lcp_lcp_newton_solvers \
           dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='PgsSolverCoverage.RejectsInvalidDivisionEpsilon:ApgdSolverCoverage.RejectsInvalidParameters:TgsSolverCoverage.RejectsInvalidDivisionEpsilon:MinimumMapNewtonCoverage.RejectsInvalidParameters:FischerBurmeisterNewtonCoverage.RejectsInvalidParameters:PenalizedFischerBurmeisterNewtonCoverage.RejectsInvalidParameters:InteriorPointSolverCoverage.RejectsInvalidParameters'
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_newton_solvers \
  --gtest_filter='MinimumMapNewtonSolver.RejectsInvalidLineSearchStepCount:MinimumMapNewtonSolver.GradientDescentWarmStartReducesMerit:MinimumMapNewtonSolver.PgsWarmStartReducesMerit:FischerBurmeisterNewtonSolver.RejectsInvalidLineSearchStepCount:FischerBurmeisterNewtonSolver.GradientDescentWarmStartReducesMerit:FischerBurmeisterNewtonSolver.PgsWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.RejectsInvalidLineSearchStepCount:PenalizedFischerBurmeisterNewtonSolver.GradientDescentWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.PgsWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.RejectsInvalidLambda'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- Focused C++ validation executable: 7/7 tests passed.
- Focused C++ Newton executable: 10/10 tests passed.
- `python/tests/unit/math/test_lcp.py`: 77 passed.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step:

- After the checkpoint, continue the broad DART 7 LCP objective with the next
  concrete solver/interface/demo/performance gap. Do not mark the overall goal
  complete from this focused validation slice.

## Current Reality — 2026-06-12 Stop-Only Hand-Off

The user explicitly requested a full stop and asked only to ensure the
hand-off docs, with no further verification. After that instruction, do not
infer that lint, tests, builds, commits, pushes, or implementation edits were
performed; this hand-off only updates
`docs/dev_tasks/lcp_solver_interface_demos/README.md` and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `e3535141c5d Show all LCP solver parameters in py demo`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 22]`.
- Worktree: clean before this docs-only hand-off edit.
- Stash list: empty.
- No PR was associated with this branch when checked earlier in the session.

Latest completed local checkpoints:

- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`
- `748cef40ea0 Validate advanced LCP solver parameters`
- `06ac009b27c Show LCP solver parameters in py demo`
- `8cd98e2e553 Expose advanced LCP parameters in dartpy`

## Previous Stop-Only Immediate Next Step

Do nothing unless the user explicitly resumes implementation. If resumed, first
inspect the current branch state:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file

## Interrupted Audit Slice

No implementation files were edited in the interrupted slice. The current work
was an audit for remaining solve-time parameter validation gaps in LCP solvers
whose parameter surfaces are now exposed through C++ and dartpy.

Audit findings to resume from:

- `dart/math/lcp/projection/pgs_solver.cpp`: validate
  `PgsSolverParameters::epsilonForDivision` before iteration; reject
  non-finite or non-positive values.
- `dart/math/lcp/projection/apgd_solver.cpp`: validate
  `ApgdSolverParameters::epsilonForDivision > 0` and
  `restartCheckInterval >= 0`; keep `0` valid as the every-iteration restart
  check mode.
- `dart/math/lcp/projection/tgs_solver.cpp`: validate
  `TgsSolverParameters::epsilonForDivision` before iteration; reject
  non-finite or non-positive values.
- `dart/math/lcp/newton/minimum_map_newton_solver.cpp`: validate line-search
  and warm-start parameters before iteration. Suggested checks:
  `maxLineSearchSteps > 0`, `stepReduction` finite in `(0, 1)`,
  `sufficientDecrease` finite in `[0, 1)`, `minStep > 0`, non-negative
  gradient-descent and PGS warm-start iteration counts, positive
  gradient-descent line-search steps, finite gradient-descent factors in the
  same ranges, and finite `pgsWarmStartRelaxation` in `(0, 2]`.
- `dart/math/lcp/newton/fischer_burmeister_newton_solver.cpp`: add the same
  Newton line-search and warm-start validation, plus positive finite
  `smoothingEpsilon`.
- `dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.cpp`: keep
  existing `lambda` validation in `(0, 1]`, and add the same Newton
  line-search, warm-start, and positive finite `smoothingEpsilon` validation.
- `dart/math/lcp/other/interior_point_solver.cpp`: stop silently clamping
  user-facing `sigma` and `stepScale`; validate finite `sigma` in `(0, 1)`
  and finite `stepScale` in `(0, 1]`, then use the validated values directly.

Test cautions for the next session:

- `tests/unit/math/lcp/test_lcp_newton_solvers.cpp` has coverage that may set
  `maxLineSearchSteps = 0` to drive numerical failure branches. If validation
  changes this behavior, update the expectations to `InvalidProblem` with a
  parameter-specific message.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` has
  interior-point coverage that sets `sigma = 1.5` and `stepScale = 1.5`; this
  likely reflects the old silent-clamping behavior.
- APGD tests that use `restartCheckInterval = 0` should stay valid.

Potential focused verification, only if implementation is explicitly resumed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
           dartpy \
  --parallel "$JOBS"
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Do not push or commit without a fresh explicit user instruction and the normal
DART pre-commit checks.

## Current Reality — 2026-06-11 Full Parameter Metadata In Py-Demo

Checkpoint `b60e20a8dc8 Expose LCP solver parameters in dartpy` is complete
locally. The next in-progress slice updates the Python LCP demo's
advanced-parameter table so it no longer reports only ADMM, SAP, and boxed
semi-smooth Newton.

Latest implementation slice:

- Generated `advanced_solver_parameter_rows` from the full dartpy parameter
  surface for projection, Newton, MPRGP, interior-point, shock propagation,
  ADMM, SAP, and boxed semi-smooth Newton solvers.
- Kept rows linked to representative benchmark filters from the LCP comparison
  benchmark metadata.
- Updated `python/tests/unit/test_py_demo_panels.py`, `CHANGELOG.md`, and this
  dev-task state.

Verification completed:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step:

- Commit the py-demo metadata checkpoint.

## Current Reality — 2026-06-11 Full dartpy Parameter Surface

Checkpoint `748cef40ea0 Validate advanced LCP solver parameters` is complete
locally. The next in-progress slice expands dartpy from the three advanced
solver parameter objects to the full set of C++ LCP solvers that already expose
`setParameters()` / `getParameters()`.

Latest implementation slice:

- Added parameter classes and solver `parameters` properties for PGS, symmetric
  PSOR, Jacobi, red-black Gauss-Seidel, blocked Jacobi, BGS, NNCG, subspace
  minimization, APGD, TGS, minimum-map Newton, Fischer-Burmeister Newton,
  penalized Fischer-Burmeister Newton, interior-point, MPRGP, and shock
  propagation.
- Updated `python/stubs/dartpy/math.pyi`, `python/tests/unit/math/test_lcp.py`,
  `CHANGELOG.md`, and this dev-task state.
- Used `snake_case` Python field names; the penalized Fischer-Burmeister
  `lambda` field is exposed as `lambda_`.

Verification completed:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Observed results:

- `pixi run build-py-dev` rebuilt and linked `dartpy`.
- `python/tests/unit/math/test_lcp.py`: 70 passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step:

- Commit the full dartpy parameter-surface slice.
- Then continue with the next concrete solver/interface/demo/performance gap;
  the broad LCP objective is still open.

## Current Reality — 2026-06-11 Advanced Parameter Validation Resumed

The user resumed the broad LCP solver/interface/demo goal after the previous
stop-only hand-off. The interrupted advanced solver parameter validation slice
has now been rebuilt and verified locally.

Latest implementation slice:

- `AdmmSolver` returns `InvalidProblem` before iteration for invalid
  `rhoInit`, `muProx`, or `adaptiveRhoTolerance` values.
- `SapSolver` returns `InvalidProblem` before iteration for invalid
  `regularization`, Armijo, backtracking, or line-search iteration settings.
- `BoxedSemiSmoothNewtonSolver` returns `InvalidProblem` before iteration for
  invalid line-search, sufficient-decrease, minimum-step, or Jacobian
  regularization settings.
- C++ coverage in the LCP smoke and validation tests exercises invalid custom
  parameter structs.
- dartpy coverage exercises invalid Python parameter objects through the solver
  `parameters` properties.
- `CHANGELOG.md` and the dev-task README describe the new validation behavior.

Verification completed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='BoxedSemiSmoothNewtonSolverCoverage.*:AdmmSolverCoverage.*'
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
```

Observed results:

- Focused validation executable passed 4 tests after an explicit rebuild.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `python/tests/unit/math/test_lcp.py`: 69 passed.

Immediate next step:

- Run `pixi run lint`, then commit the validation slice if lint is clean.
- After that checkpoint, continue the broad DART 7 LCP audit with the next
  concrete solver/interface/demo/performance gap. Do not mark the overall goal
  complete from this focused validation slice.

## Current Reality — 2026-06-11 Stop-Only Hand-Off

The user explicitly requested a full stop and asked only to ensure the
hand-off docs, with no further verification. After that instruction, do not
infer that tests, lint, commits, or pushes were performed; this hand-off only
updates `docs/dev_tasks/lcp_solver_interface_demos/README.md` and this file.

Observed repository state before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `06ac009b27c Show LCP solver parameters in py demo`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 19]`.
- Worktree had uncommitted implementation edits in:
  - `CHANGELOG.md`
  - `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp`
  - `dart/math/lcp/other/admm_solver.cpp`
  - `dart/math/lcp/other/sap_solver.cpp`
  - `python/tests/unit/math/test_lcp.py`
  - `tests/unit/math/lcp/test_all_solvers_smoke.cpp`
  - `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`

## Immediate Next Step

Do nothing unless the user explicitly resumes implementation. If resumed, first
inspect the current worktree and the uncommitted validation slice:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
git diff --stat
```

Then inspect:

- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file
- the uncommitted files listed above

## Interrupted Implementation Slice

The interrupted, uncommitted slice adds solve-time validation for advanced
solver parameter structs:

- `AdmmSolver`: rejects non-positive `rhoInit`, negative `muProx`, and
  `adaptiveRhoTolerance <= 1` or non-finite values with `InvalidProblem`.
- `SapSolver`: rejects non-positive `regularization`, invalid Armijo and
  backtracking factors outside `(0, 1)`, non-positive line-search iteration
  counts, and non-finite values with `InvalidProblem`.
- `BoxedSemiSmoothNewtonSolver`: rejects invalid line-search steps,
  `stepReduction`, `sufficientDecrease`, `minStep`, and
  `jacobianRegularization` values with `InvalidProblem`.
- C++ and dartpy tests were added or adjusted to assert invalid advanced
  parameters are rejected before solver iteration.
- `CHANGELOG.md` was updated for the new validation behavior.

Verification that happened before the stop-only instruction:

- Explicit build of `UNIT_math_lcp_math_lcp_all_solvers_smoke` plus `dartpy`
  succeeded after fixing a const-qualification issue in the smoke test.
- `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke`
  passed 17 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest
python/tests/unit/math/test_lcp.py -q` passed 69 tests.
- `pixi run test-lcpsolver` failed only
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`; the failure output still
  appeared to use the old test name and old expectations after the source was
  edited, so a fresh session should first rebuild that exact target and rerun
  the focused executable before drawing conclusions.

Suggested first verification only if implementation is resumed:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='BoxedSemiSmoothNewtonSolverCoverage.*:AdmmSolverCoverage.*'
```

Do not push or commit without a fresh explicit user instruction and the normal
DART pre-commit checks.

This stop-only state is historical after the current continuation.

## Current Reality — 2026-06-11 Advanced Solver Parameters

The user later explicitly resumed the broad LCP solver/interface/demo goal, so
the prior stop-only hand-off is now historical. This continuation resumes from
`17a994e3772 Reject negative LCP friction coefficients` on
`feature/lcp-solver-interface-demos` and adds a bounded dartpy interface slice.

Current implementation slice:

- Exposes `AdmmSolverParameters`, `SapSolverParameters`, and
  `BoxedSemiSmoothNewtonSolverParameters` through dartpy with DART 7 snake_case
  field names.
- Adds `parameters` properties to dartpy `AdmmSolver`, `SapSolver`, and
  `BoxedSemiSmoothNewtonSolver`, forwarding to the existing C++
  `setParameters()` / `getParameters()` APIs.
- Updates the manual dartpy type stubs and Python LCP tests so Python demos can
  tune the same advanced boxed and friction-index solver knobs used by C++
  benchmark sweeps such as `BM_LcpAdmmRhoSweep`,
  `BM_LcpSapRegularizationSweep`, and
  `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep`.
- Adds a Python LCP demo parameter table that reports those advanced solver
  knobs, defaults, and matching benchmark sweep filters in `setup.info` and
  the GUI panel.
- Updates `CHANGELOG.md`, focused py-demo tests, the LCP roster lint guard, the
  generated API boundary inventory, and these dev-task docs.

Verification completed for this slice:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
pixi run lint
```

Observed results:

- `pixi run build-py-dev` rebuilt and linked `dartpy`.
- `python/tests/unit/math/test_lcp.py`: `66 passed`.
- `python/tests/unit/test_py_demo_panels.py`: `43 passed`.
- LCP solver roster check: `24 solvers, 24 standard, 16 boxed/findex`.
- `pixi run lint` passed.

Observed repository state before this slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD:
  `17a994e3772 Reject negative LCP friction coefficients`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 17]`.
- Recent local checkpoints:
  - `17a994e3772 Reject negative LCP friction coefficients`
  - `e3353bf04b7 Expose LCP solver sweep metadata`
  - `46700198d80 Expose LCP scale benchmark metadata`
  - `197b55c335a Use concrete LCP contact registration gates`
- No PR was associated with this branch when checked before the stop-only
  hand-off.

Fresh-session resume sequence:

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git log --oneline --decorate --max-count=15
```

Then read:

- `docs/ai/principles.md`
- `docs/dev_tasks/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- this file

Do not treat this slice as completion of the broad LCP solver/interface/demo
objective. Continue one bounded DART 7 LCP interface, solver completeness,
demo, benchmark, or performance gap at a time.

## Latest Completed Implementation — 2026-06-11 Friction Coefficient Validation

The latest committed implementation slice:

- Rejects friction-index rows with negative stored upper coefficients `hi[i]`
  in shared problem validation. DART's public LCP docs define this slot as the
  non-negative `+mu` coefficient for effective tangent bounds.
- Makes `computeEffectiveBounds(...)` use `hi[i]` directly after validation
  instead of silently accepting a negative coefficient through `abs(hi[i])`.
- Makes `LcpProblem::hasFrictionIndex()` reject the same invalid metadata so
  classification, `isValid()`, `supportsProblem(problem)`, and solver dispatch
  agree.
- Adds C++ coverage in `test_lcp_types.cpp`,
  `test_lcp_validation_and_solvers.cpp`, and generated invalid-problem solver
  coverage.
- Adds dartpy coverage in `python/tests/unit/math/test_lcp.py`.

Verification completed for this slice:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
```

Observed results:

- C++ LCP suite: `100% tests passed, 0 tests failed out of 17`.
- The focused dartpy LCP test reported `62 passed`.
- `pixi run lint` passed before commit.

## Historical Hand-Off — 2026-06-11 Stop-Only Point

The previous user instruction was to stop all implementation work, do not run
further verification, ensure only the hand-off docs were current, and then
fully stop. After that instruction, the only edits were this file and
`docs/dev_tasks/lcp_solver_interface_demos/README.md`.

No lint, build, tests, benchmark-list commands, benchmark execution, solver
execution, code edits, commit, or push was run after that stop-only instruction.
Historical verification below belongs only to the named implementation
checkpoints that explicitly list it.

That stop-only hand-off recorded:

- Consolidated branch: `feature/lcp-solver-interface-demos`.
- Local HEAD before that docs-only hand-off edit:
  `e3353bf04b7 Expose LCP solver sweep metadata`.
- Tracking state before that docs-only hand-off edit:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 16]`.
- Recent local checkpoints:
  - `e3353bf04b7 Expose LCP solver sweep metadata`
  - `46700198d80 Expose LCP scale benchmark metadata`
  - `197b55c335a Use concrete LCP contact registration gates`
- The worktree was clean before that final docs-only hand-off edit.

The suspected non-finite lower-bound follow-up from that stop-only note was
rechecked during the current continuation. Existing tests intentionally use
`lo = -inf` or `lo = 0` storage on some friction-index rows while deriving
effective tangent bounds from `hi`; do not tighten that behavior without a
separate design decision.

## Previous Completed Implementation — 2026-06-11 Py-Demo Metadata

The latest completed implementation slices extend the Python LCP demo's
representative benchmark packet metadata:

- `active_set_scale` now points at the larger, stress, extreme, production,
  and production-batch active-set transition benchmark rows.
- `singular_degenerate_scale` now points at larger, stress, extreme, and
  serial/parallel batch singular-degenerate benchmark rows.
- `solver_parameter_sweeps` now points at solver-specific relaxation,
  line-search, pivoting-scale, block-partition, restart-policy,
  iteration-budget, shock-layer, SPD-check, path-following, ADMM rho, and SAP
  regularization benchmark sweeps.
- The existing representative filter union continues to derive
  `representative_benchmark_filter` and `representative_benchmark_command`
  from every packet row, so the demo now exposes the current scalability and
  hard-degeneracy benchmark surfaces without duplicating another command owner.
- `python/tests/unit/test_py_demo_panels.py`, `CHANGELOG.md`, and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe and cover the
  slice.

Verification completed for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(Larger|Stress|Extreme|Production)ActiveSetTransition|BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)|BM_Lcp(Larger|Stress|Extreme)SingularDegenerate|BM_LcpSingularDegenerate(FrictionIndexBatch|StandardBoxedBatch)(Serial|Parallel)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(PgsRelaxationSweep|SymmetricPsorRelaxationSweep|RedBlackGaussSeidelRelaxationSweep|BoxedSemiSmoothNewtonLineSearchSweep|PivotingScaleSweep|BlockPartitionSweep|ApgdRestartSweep|TgsIterationBudgetSweep|NncgPgsIterationsSweep|SubspaceMinimizationPgsIterationsSweep|ShockPropagationLayerSweep|MprgpSpdCheckSweep|InteriorPointPathSweep|AdmmRhoSweep|SapRegularizationSweep)'
pixi run lint
```

Observed results:

- The Python demo-panel test reported `43 passed`.
- The benchmark-list check rebuilt/linked `BM_LCP_COMPARE` and listed the
  newly exposed active-set scale/production and singular-degenerate
  scale/batch benchmark rows.
- A second benchmark-list check listed the newly exposed solver-specific
  tuning and robustness sweep rows.
- `pixi run lint` passed.

The latest implementation slice removes redundant manifest-family prechecks
from contact benchmark registration paths:

- `RegisterActiveFrictionIndexContactBenchmarks()`,
  `RegisterWorldContactBenchmarks()`, `RegisterWorldBoxContactBenchmarks()`,
  `RegisterWorldStackContactBenchmarks()`,
  `RegisterArticulatedUnifiedContactBenchmarks()`,
  `RegisterContactSolverComparisonSweepBenchmarks()`,
  `RegisterContactNormalStandardSweepBenchmarks()`,
  `RegisterWorldContactBatchBenchmarks()`, and
  `RegisterWorldBoxContactBatchBenchmarks()` now rely on concrete generated
  contact packets or support probes instead of static
  `LcpProblemSupport` prechecks.
- Dense world-box contact registrations still keep the explicit
  `SupportsDenseWorldBoxContactPatch(...)` solver scope before the concrete
  probe.
- The audited LCP benchmark/test/demo surface no longer has
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::...)` prechecks.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(ActiveFrictionIndexContact|WorldContact|WorldBoxContact|WorldStackContact|ArticulatedUnifiedContact|ContactSolverComparisonSweep|ContactNormalStandardSweep|WorldContactBatchSerial|WorldContactStressBatchSerial|WorldContactPipeline32BatchSerial|WorldBoxContactBatchSerial).*(Dantzig|Pgs|MPRGP|Baraff|BoxedSemiSmoothNewton)'
pixi run lint
```

Observed results:

- The benchmark-list check rebuilt and linked `BM_LCP_COMPARE`.
- It listed representative concrete rows for active friction-index contact,
  world contact, dense world-box contact, world stack contact, articulated
  unified contact, contact solver comparison, normal-standard contact, mixed
  world-contact batches, and dense world-box contact batches.
- `pixi run lint` passed, including the LCP solver roster check.

Resume status:

- The broader LCP solver/interface/demo objective is not complete. Resume from
  the next concrete support-routing, solver-domain, demo, benchmark, or
  performance gap after this slice lands.

## Previous Reality — 2026-06-11 Concrete Benchmark Helper Cleanup

That continuation built on
`38266f484e2 Route LCP smoke tests through concrete support` on
`feature/lcp-solver-interface-demos`.

The implementation slice removed redundant manifest-family prechecks from
concrete benchmark-routing helpers:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` no longer keeps
  `getMildIllConditionedProblemSupport(...)`,
  `getNearSingularProblemSupport(...)`,
  `getLargerActiveSetTransitionProblemSupport(...)`, or
  `getSingularDegenerateProblemSupport(...)`.
- The corresponding `SolverShouldRun...Benchmark(...)` helpers now rely on
  explicit solver scopes plus `SolverSupportsConcreteProblem(...)` for exact
  generated problems.
- The production active-set transition batch helper now uses the same simplified
  concrete-problem helper signature.

Verification completed for that slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(MildIllConditioned|NearSingular|LargerActiveSetTransition|ProductionActiveSetTransitionBatchSerial|SingularDegenerate|SingularDegenerateFrictionIndexBatchSerial|SingularDegenerateStandardBoxedBatchSerial)/(Standard32|Boxed16|FrictionIndex8|Standard8|Boxed8|CoupledFrictionIndex3|Standard16|CoupledFrictionIndex6|CoupledFrictionIndex8)/(Direct|MPRGP|Baraff|Dantzig|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run lint
```

Observed results:

- The first benchmark-list attempt caught an unused `testCase` parameter after
  the precheck removal; the helper signature and callers were simplified.
- The rerun rebuilt and linked `BM_LCP_COMPARE`, then listed concrete active
  set, production batch, mild/near-singular, and singular-degenerate rows for
  the scoped solver/problem combinations.
- `pixi run lint` passed, including the LCP solver roster check.

## Previous Reality — 2026-06-11 All-Solvers Smoke Checkpoint

That continuation built on
`29ece871105 Filter pivoting LCP sweeps concretely` on
`feature/lcp-solver-interface-demos`.

The implementation slice routed the generated all-solvers smoke-test skip
helper through concrete per-problem support:

- `tests/unit/math/lcp/test_all_solvers_smoke.cpp` removes the local
  `ProblemCategory` to `LcpProblemSupport` manifest-family mapping.
- `canSolve(...)` now constructs the solver instance and calls
  `instance->supportsProblem(problem.problem)`, returning false when solver
  creation fails.

Verification completed for that slice:

```bash
pixi run test-lcpsolver
pixi run lint
```

Observed results:

- The focused LCP test task rebuilt as needed and reported
  `100% tests passed, 0 tests failed out of 17`.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke` passed.
- `pixi run lint` passed, including the LCP solver roster check.

## Previous Reality — 2026-06-11 Pivoting Checkpoint

That continuation resumed from
`e86dbf2efa5 Filter singular LCP batches concretely` on
`feature/lcp-solver-interface-demos`.

The latest implementation slice filters pivoting scale sweep benchmark
registration through concrete generated-problem support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the exact
  generated problem for each `BM_LcpPivotingScaleSweep` row and publishes it
  only when the selected solver's concrete `supportsProblem(problem)` predicate
  accepts the packet.
- Runtime pivoting scale sweep execution now uses the same concrete guard.
- The now-unused manifest-family `getProblemSupport(...)` helper was removed.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpPivotingScaleSweep/(Standard|Boxed|FrictionIndex)/(Direct|Lemke|Baraff|Dantzig)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpPivotingScaleSweep/Standard/Direct/Rows3|BM_LcpPivotingScaleSweep/Standard/Baraff/Rows8|BM_LcpPivotingScaleSweep/Boxed/Dantzig/Rows12|BM_LcpPivotingScaleSweep/FrictionIndex/Dantzig/Contacts4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The first benchmark-list attempt caught the now-unused
  `getProblemSupport(...)` helper under `-Werror`; that helper was removed.
- The rerun rebuilt `BM_LCP_COMPARE` and listed Direct standard rows 2 and 3,
  Lemke/Baraff/Dantzig standard rows, Dantzig boxed rows, and Dantzig
  friction-index rows.
- The short benchmark execution reported `contract_ok=1` for sampled Direct
  standard, Baraff standard, Dantzig boxed, and Dantzig friction-index rows.

The broader LCP solver/interface/demo objective is not complete. Resume from
the next concrete support-routing, solver-domain, demo, benchmark, or
performance gap after this slice lands.

## Previous Reality — 2026-06-11 Singular Batch Checkpoint

The current continuation resumes from
`911e9530c41 Document latest LCP handoff checkpoint` on
`feature/lcp-solver-interface-demos`.

The latest implementation slice filters singular-degenerate batch benchmark
registrations through exact generated-batch support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the exact
  four-problem singular-degenerate friction-index batch for each batch row and
  requires every concrete problem to pass `supportsProblem(problem)` before
  publishing serial or parallel rows.
- Singular-degenerate standard/boxed serial and parallel batch rows now use the
  same exact generated-batch support check.
- Single singular-degenerate registrations now precompute the generated problem
  once per case and pass it to the shared concrete helper.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpSingularDegenerate(FrictionIndexBatch|StandardBoxedBatch)(Serial|Parallel)/(Standard16|Boxed16|CoupledFrictionIndex6|Standard128|Boxed128|CoupledFrictionIndex16|CoupledFrictionIndex256)/(Direct|MPRGP|Baraff|Admm|Sap|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpSingularDegenerateFrictionIndexBatchSerial/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateFrictionIndexBatchParallel/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Standard16/Baraff|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Boxed16/BoxedSemiSmoothNewton' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The benchmark-list check rebuilt `BM_LCP_COMPARE` and listed exact supported
  singular-degenerate batch rows for Admm, Sap, BoxedSemiSmoothNewton, and
  Baraff where their concrete generated batches are supported; Direct and MPRGP
  were absent from the scoped filter.
- The short benchmark execution reported `contract_ok=1` for sampled Sap
  friction-index serial and parallel batch rows, Baraff standard-batch rows,
  and BoxedSemiSmoothNewton boxed-batch rows.

The broader LCP solver/interface/demo objective is not complete. Resume from
the next concrete support-routing, solver-domain, demo, benchmark, or
performance gap after this slice lands.

## Critical Hand-Off — 2026-06-11 Final Stop Point

The latest user instruction was critical: stop implementation work and focus on
hand-off only, with no further verification. After that instruction, no lint,
build, tests, benchmark-list commands, benchmark execution, or solver execution
were run. Historical verification below belongs only to the implementation
checkpoints that explicitly list it.

Use this as the fresh-session entrypoint:

- Consolidated branch: `feature/lcp-solver-interface-demos`.
- Latest implementation checkpoint:
  `9a17ba85aa5 Filter conditioning LCP benchmarks concretely`.
- Previous checkpoint:
  `559cda91ace Filter active-set scale LCP benchmarks concretely`.
- `main` was fetched from `https://github.com/dartsim/dart.git` at
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`.
- This checkout's `origin` remote is SSH. Previous successful publishes used
  HTTPS, so a fresh session should fetch the branch from GitHub before relying
  on local remote-tracking metadata.

Current branch contents to preserve:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` filters mildly
  ill-conditioned and near-singular benchmark registrations through concrete
  generated-problem support, including exact serial/parallel batch problem-list
  checks.
- `CHANGELOG.md` records the conditioning benchmark-routing checkpoint.
- `docs/dev_tasks/lcp_solver_interface_demos/README.md` and this resume file
  capture the full hand-off context.

Resume guidance:

1. Fetch and check out `feature/lcp-solver-interface-demos` from GitHub.
2. Read `docs/ai/principles.md`, `docs/dev_tasks/README.md`, this file, and
   the dev-task README before editing.
3. Inspect the current branch tip and continue one bounded remaining LCP
   interface/demo audit gap.
4. Do not treat this hand-off as completion of the broad LCP objective, and do
   not retire `docs/dev_tasks/lcp_solver_interface_demos/` from this checkpoint
   alone.

Likely next gaps:

- Continue auditing benchmark/demo/test surfaces that still summarize native
  solver support without checking concrete generated problems.
- Review remaining solver-domain predicates against the actual native paths and
  fallback/delegation behavior.
- Continue py-demo and benchmark apples-to-apples coverage work after the next
  session re-establishes local verification.

## Current Reality — 2026-06-11 Active Continuation

The current continuation resumes from
`c564b6b2ba2 Report concrete LCP demo native coverage` on
`feature/lcp-solver-interface-demos`.

The latest implementation slice filters mildly ill-conditioned and near-singular
benchmark registrations through concrete generated-problem support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the generated
  problem for each `BM_LcpMildIllConditioned` and `BM_LcpNearSingular` row and
  only publishes solver rows whose concrete `supportsProblem(problem)`
  predicate accepts the packet.
- Mildly ill-conditioned and near-singular batch rows now precompute the exact
  four-problem batch and require every concrete batch problem to be supported
  before publishing serial or parallel rows.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the latest
  slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpMildIllConditioned/(Standard32|Boxed16|FrictionIndex8|ExtremeCoupledFrictionIndex256)/(Dantzig|Baraff|MPRGP|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpNearSingular/(Standard8|Boxed8|CoupledFrictionIndex3|CoupledFrictionIndex256)/(Dantzig|Baraff|MPRGP|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(MildIllConditioned|NearSingular)Batch(Serial|Parallel)/(Standard32|Boxed16|FrictionIndex8|CoupledFrictionIndex8|ExtremeCoupledFrictionIndex256|Standard8|Boxed8|CoupledFrictionIndex3)/(Dantzig|Baraff|MPRGP|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpMildIllConditioned/Standard32/Baraff|BM_LcpMildIllConditionedBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton|BM_LcpNearSingular/Boxed8/BoxedSemiSmoothNewton|BM_LcpNearSingularBatchSerial/CoupledFrictionIndex3/ShockPropagation' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The mild row-list check rebuilt `BM_LCP_COMPARE` and listed scoped concrete
  rows for Standard32, Boxed16, FrictionIndex8, and
  ExtremeCoupledFrictionIndex256; MPRGP and Direct were absent.
- The near-singular row-list check listed scoped concrete rows for Standard8,
  Boxed8, CoupledFrictionIndex3, and CoupledFrictionIndex256; MPRGP, PGS, and
  Direct were absent.
- The batch row-list check listed serial and parallel mild/near-singular rows
  for the exact supported four-problem batches.
- The short benchmark execution reported `contract_ok=1` for the sampled
  mild/near-singular single and serial-batch rows.
- `pixi run lint`: passed.

Previous implementation slice: larger active-set transition benchmark
registrations were filtered through concrete generated-problem support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now precomputes the generated
  problem for larger, stress, extreme, and production active-set transition
  rows and only publishes solver rows whose concrete
  `supportsProblem(problem)` predicate accepts the packet.
- Production active-set transition batch rows now precompute the exact
  four-problem batch and require every concrete batch problem to be supported
  before publishing serial or parallel rows.
- `CHANGELOG.md` and
  `docs/dev_tasks/lcp_solver_interface_demos/README.md` describe the slice.

Verification completed for this slice:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(Larger|Stress|Extreme|Production)ActiveSetTransition/(Standard32|Boxed32|CoupledFrictionIndex8|Standard64|Boxed64|CoupledFrictionIndex12|Standard128|Boxed128|CoupledFrictionIndex16|CoupledFrictionIndex24)/(MPRGP|Baraff|Direct|Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)/(Standard32|Boxed32|CoupledFrictionIndex8)/(MPRGP|Baraff|Direct|Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpLargerActiveSetTransition/Standard32/MPRGP|BM_LcpStressActiveSetTransition/Boxed64/Pgs|BM_LcpProductionActiveSetTransitionBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The single-row benchmark-list check rebuilt `BM_LCP_COMPARE` and listed
  concrete larger/stress/extreme/production active-set transition rows for
  Dantzig, PGS, MPRGP, and BoxedSemiSmoothNewton where supported; Direct and
  Baraff were absent from the filtered row set.
- The production-batch benchmark-list check listed serial and parallel rows for
  Standard32, Boxed32, and CoupledFrictionIndex8 for the supported solver set.
- The short benchmark execution reported `contract_ok=1` for the sampled
  larger active-set, stress active-set, and production batch rows.
- `pixi run lint`: passed.

The broader LCP solver/interface/demo objective is not complete. Resume from
the next concrete support-routing, solver-domain, demo, benchmark, or
performance gap after this slice lands.

## Previous Reality — 2026-06-11 Critical Hand-Off

The latest user instruction was to stop implementation and focus on hand-off
only, with no further verification. No lint, build, tests, benchmark listing,
solver execution, or additional implementation work was run after that
instruction. Historical verification notes below belong only to their named
implementation checkpoints.

This hand-off consolidates the current work on
`feature/lcp-solver-interface-demos`. Before the final hand-off checkpoint
commit, local implementation HEAD was:

```text
2cd337aa0cf Use concrete gates for LCP benchmark registration
```

The working tree at the stop point contained the Python demo concrete
native-case profile slice:

- `CHANGELOG.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

That slice makes the LCP py-demo solver profile report native coverage from
the concrete representative problem rows instead of only static manifest
surfaces. The panel now exposes partial native coverage such as Direct's
`standard 2/4` case count, while full-surface solvers report labels such as
`standard 4/4, boxed 3/3, findex 2/2`.

Before this hand-off checkpoint, local status showed this branch ahead of the
stale SSH tracking ref by five commits plus the uncommitted Python slice:

```text
## feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 5]
 M CHANGELOG.md
 M docs/dev_tasks/lcp_solver_interface_demos/README.md
 M docs/dev_tasks/lcp_solver_interface_demos/RESUME.md
 M python/examples/demos/scenes/lcp_physics.py
 M python/tests/unit/test_py_demo_panels.py
```

Do not treat `origin/feature/lcp-solver-interface-demos` in this checkout as
authoritative: `origin` is configured for SSH, while previous successful
publishes used HTTPS and did not refresh the local SSH remote-tracking ref. A
fresh session should fetch the remote branch directly from GitHub.

For this final hand-off, `main` was fetched over HTTPS from
`https://github.com/dartsim/dart.git` to `7d05d7b9ea72`, and
`git merge --no-edit FETCH_HEAD` reported `Already up to date.`

Base first-parent stack before this final hand-off checkpoint commit:

```text
2cd337aa0cf Use concrete gates for LCP benchmark registration
6c0763927a5 Filter grouped LCP batch rows concretely
20aaa23d0fe Document final LCP no-verification handoff
fe5b70b32cc Route generated LCP coverage by concrete support
f86e353df2f Gate heavyweight LCP contact benchmarks concretely
737b9c95c11 Document final LCP handoff checkpoint
8f0242c2442 Filter LCP contact benchmark rows concretely
4c63db30bd7 Filter LCP benchmark args concretely
be4643d1743 Document consolidated LCP handoff state
```

Resume from the next bounded gap in the broader LCP interface/demo audit
rather than repeating the completed benchmark-routing, generated-coverage, or
Python solver-profile slices. The broad LCP objective is not complete.

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
benchmark. It then added `billiard_symmetry_error` to the live LCP py-demo so
billiard symmetry is tracked alongside momentum and kinetic-energy error. The
high-mass-ratio stack benchmark metadata now points at
`BM_LcpWorldStackContact/|BM_LcpWorldStackStep_BoxedLcp`. The live LCP panel
also plots billiard momentum, energy, and symmetry invariant histories for both
comparison worlds. The next checkpoint added
`representative_benchmark_filter` and `representative_benchmark_command` to the
LCP py-demo metadata, derived from the benchmark packet table while preserving
the existing smoke command. The current slice realigns LCP background docs with
DART 7 snake_case solver header paths and extends the LCP roster lint gate to
reject stale documented LCP header/source paths.

The user then gave a critical stop instruction: focus on hand-off only and run
no further verification. No code changes were made after that instruction. The
interrupted next-slice audit found a likely bounded follow-up in
`MprgpSolver::supportsProblem(...)`: MPRGP currently inherits the base
form-level predicate even though its native standard-LCP path rejects
non-symmetric matrices and, by default, matrices that fail an LLT positive
definite check before delegating to Dantzig.

The current continuation implements that MPRGP follow-up: MPRGP now overrides
`supportsProblem(problem, standardTolerance)` and reports native support only
for standard problems that satisfy its configured symmetry and
positive-definite native-path checks. Focused C++ and dartpy tests cover native
SPD support, boxed false support, non-symmetric false support, default
indefinite false support, and the C++ parameter case where
`checkPositiveDefinite = false`.

The next continuation applies the same native-route audit to Baraff: Baraff now
reports native support only for symmetric positive-semidefinite standard
packets, and non-standard, non-symmetric, or indefinite packets delegate to
Dantzig before the active-set loop. Focused C++ and dartpy tests cover SPD,
PSD, boxed, non-symmetric, indefinite, and fallback solve behavior.

The final update in this session is hand-off only. The user explicitly asked
to stop implementation and run no further verification. The branch was
refreshed against `origin/main`, was already up to date, and this document now
captures the next interrupted audit: benchmark/demo routing should be checked
against the new concrete `supportsProblem(problem)` predicates for MPRGP and
Baraff before any more solver-comparison rows are exposed as native.

The next continuation completed the first benchmark-routing slice:
singular-degenerate standard benchmark registration now intersects the scoped
solver allowlist with each solver's concrete `supportsProblem(problem)`
predicate. The generated standard packets are symmetric PSD but rank-deficient,
so Baraff remains listed as native while MPRGP's default positive-definite
native gate removes its fallback rows from the native benchmark list.

The next continuation completed the following benchmark-routing slice:
contact-normal standard sweep registration now builds each concrete normal-only
contact packet once and registers only solvers whose `supportsProblem(problem)`
accepts that packet. Current MPRGP and Baraff contact-normal rows remain
registered because the generated packets satisfy their native predicates, and
Direct remains limited to concrete 1-, 2-, and 3-row normal packets through the
same support predicate rather than a size special case.

The latest user instruction on 2026-06-11 was critical: stop implementation
and focus on hand-off only with no further verification. No code, lint, build,
test, benchmark-list, or solver-execution work was performed after that
instruction. This resume document and the dev-task README are the only intended
changes in the final hand-off checkpoint.

The current continuation resumes implementation from that hand-off and closes
the active-friction-index benchmark routing gap: `lcp_compare` now registers
manifest-driven `BM_LcpActiveFrictionIndexContact/FrictionIndex/<solver>` rows
for every friction-index-capable solver that accepts the concrete
`LcpProblemFactory::activeFrictionIndexContact()` packet, and the Python LCP
demo metadata now points its representative active-friction filter at that
main comparison row family instead of the older two-solver microbenchmark
surface.

The follow-up continuation also replaces the active-set transition benchmark's
hand-coded Direct skip with the same concrete `supportsProblem(problem)` gate.
For the standard 16-row active-set transition packet, Baraff and MPRGP remain
registered as native rows while Direct is excluded by its own native-support
predicate instead of a benchmark-local size special case.

The latest user instruction is another critical stop: do not continue
implementation and do not run further verification. The only intended final
work is this hand-off documentation checkpoint, followed by publishing the
single consolidated branch so a fresh session can resume there.

The current continuation resumed implementation from that pushed hand-off and
completed the next benchmark-routing slice: manifest-generated `BM_LcpCompare`
and serial/parallel `BM_LcpBatch` registrations now filter their candidate
argument rows through concrete generated-problem support instead of only the
manifest-level problem family. The existing row shape is preserved while Direct,
MPRGP, Baraff, and other solvers stay aligned with their
`supportsProblem(problem)` native-route predicates.

The latest continuation completed the next contact-routing slice: separated
world-contact, world stack-contact, and contact-solver comparison sweep
registrations now filter through concrete generated contact packets before
registering rows. Dense world-box contact, articulated unified contact, and
contact batch rows still use manifest-level friction-index gates because
eagerly constructing the largest 256-contact fixtures during benchmark
registration needs a separate strategy.

The final requested work is this no-verification hand-off checkpoint. It is
documentation-only and is intended to preserve the current branch state for a
fresh AI session.

The current continuation resumes implementation from that hand-off and closes
the remaining heavyweight contact benchmark registration gap. Dense box-contact
single rows, articulated unified-contact single rows, and dense box-contact
batch rows now use small concrete support probes before publishing their large
argument sets, while mixed world-contact batch rows check the exact generated
batch problem lists for the baseline, stress-stack, and contact-pipeline-32
families.

The follow-up continuation aligns generated LCP correctness coverage with that
same concrete support model: generated cases now skip by each solver's
`supportsProblem(testCase.problem)` result instead of a manifest-family check
plus a Direct-only size special case.

## Current Branch

`feature/lcp-solver-interface-demos` — consolidated branch for this work.

Current reality after the Python demo concrete native-case profile, benchmark
concrete-gate cleanup, grouped batch benchmark, heavyweight contact benchmark,
and generated coverage support-routing slices:

- The latest implementation slice changed `CHANGELOG.md`,
  `python/examples/demos/scenes/lcp_physics.py`,
  `python/tests/unit/test_py_demo_panels.py`, and these dev-task docs.
- The benchmark concrete-gate cleanup slice changed `CHANGELOG.md`,
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, and these dev-task docs.
- The grouped batch implementation slice changed `CHANGELOG.md`,
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, and these dev-task docs.
- The generated coverage implementation slice changed `CHANGELOG.md`,
  `tests/unit/math/lcp/test_lcp_generated_coverage.cpp`, and these dev-task
  docs.
- The heavyweight contact benchmark implementation slice changed `CHANGELOG.md`,
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, and these dev-task docs.
- Local implementation HEAD before this hand-off checkpoint was
  `2cd337aa0cf Use concrete gates for LCP benchmark registration`; the Python
  demo native-case profile slice was still uncommitted when the final
  stop-and-handoff instruction arrived and is included in this checkpoint.
- The configured SSH remote-tracking ref still showed
  `origin/feature/lcp-solver-interface-demos` at
  `737b9c95c11 Document final LCP handoff checkpoint` before this hand-off
  commit; treat it as stale and fetch the branch directly from GitHub.
- `main` was fetched over HTTPS to `7d05d7b9ea72`; merging it with
  `git merge --no-edit FETCH_HEAD` reported `Already up to date.`
- No PR was associated with the branch when checked earlier in the session.

After this checkpoint is pushed, the single consolidated branch should remain
the source of truth for a fresh session. Merge latest `main` before any later
push.

## Immediate Next Step

Continue the broader LCP interface/demo audit from the next concrete gap. Do
not push without explicit approval. Do not treat the broad LCP objective as
complete.

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
- Previous handoff note: no lint, build, test, benchmark listing, or solver
  execution was run after the earlier user instruction to stop and focus on
  hand-off.
- Current benchmark-routing audit: `bm_lcp_compare.cpp` still contains
  registrations
  that gate rows with manifest-level form support, for example
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::Standard)`.
  That is now too coarse for at least some MPRGP/Baraff rows because their
  concrete `supportsProblem(problem)` predicates report narrower native
  mathematical domains.
- Completed benchmark follow-up: singular-degenerate standard registration now
  calls a benchmark-local `SolverSupportsConcreteProblem(...)` helper after the
  scoped solver allowlist. The benchmark-list check for
  `BM_Lcp.*SingularDegenerate.*/(MPRGP|Baraff)` lists Baraff rows and no MPRGP
  rows.
- Completed benchmark follow-up: contact-normal standard registration now
  generates each concrete normal-only contact packet once and checks
  `supportsProblem(problem)` before registering rows. The benchmark-list check
  for `BM_LcpContactNormalStandardSweep/(MPRGP|Baraff|Direct)` keeps all
  Baraff/MPRGP rows and Direct only on concrete 1-, 2-, and 3-row normal
  packets.
- Completed benchmark follow-up: active friction-index contact registration now
  uses the shared active-contact factory packet in `bm_lcp_compare.cpp` and
  filters participating manifest solvers through concrete
  `supportsProblem(problem)` checks. The benchmark-list check for
  `BM_LcpActiveFrictionIndexContact` lists 16 friction-index-capable rows, and
  the short execution check reports `contract_ok=1` for all 16.
- Completed benchmark follow-up: active-set transition registration now checks
  concrete generated packets before registering native rows. The list check for
  `BM_LcpActiveSetTransition/Standard/(Direct|Baraff|MPRGP)` keeps Baraff and
  MPRGP and excludes Direct through its concrete support predicate.
- Completed benchmark follow-up: `BM_LcpCompare` and serial/parallel
  `BM_LcpBatch` registrations now compute candidate argument rows and keep only
  rows whose generated concrete problem or full generated batch is accepted by
  `supportsProblem(problem)`. Focused row-list checks covered Standard,
  Boxed, and FrictionIndex families, and a short execution check reported
  `contract_ok=1` on representative affected single, serial-batch, and
  parallel-batch rows.
- Completed benchmark follow-up: separated `BM_LcpWorldContact/FrictionIndex`,
  `BM_LcpWorldStackContact/FrictionIndex`, and
  `BM_LcpContactSolverComparisonSweep` registrations now prebuild the relevant
  concrete contact packets and register only solver/arg rows accepted by
  `supportsProblem(problem)`. Focused row-list checks covered representative
  Dantzig, PGS, BoxedSemiSmoothNewton rows and the scoped comparison solvers
  `Admm`, `Sap`, `BoxedSemiSmoothNewton`; a short execution check reported
  `contract_ok=1` on targeted rows and on two larger rows matched by the regex.
- Superseded contact registration gap: dense world-box contact, articulated
  unified contact, and contact batch rows previously used manifest-level
  friction-index gates because concrete filtering them naively would construct
  very large 256-contact fixtures during benchmark registration/listing.
- Completed benchmark follow-up: the heavyweight dense/contact registrations
  now use concrete solver support gates. `BM_LcpWorldBoxContact`,
  `BM_LcpArticulatedUnifiedContact`, and `BM_LcpWorldBoxContactBatch` use small
  concrete support probes to avoid registration-time construction of their
  largest fixtures; `BM_LcpWorldContactBatch` baseline, stress-stack, and
  contact-pipeline-32 families check the exact generated batch problem lists.
- Completed generated-coverage follow-up: `solverShouldRun(...)` in
  `tests/unit/math/lcp/test_lcp_generated_coverage.cpp` now uses each solver's
  concrete `supportsProblem(problem)` result, so Direct, Baraff, MPRGP, and
  future narrow native routes stay aligned with benchmark/demo support
  reporting. Focused verification rebuilt and passed
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage`; `pixi run lint` passed.
- Historical hand-off note: after the earlier 2026-06-11 stop instruction, no
  additional verification was run in that hand-off-only checkpoint, including
  `pixi run lint`. The later heavyweight contact support-gating continuation
  resumed implementation and ran focused benchmark checks plus `pixi run lint`.
- Generic standard rows that come from `MakeBenchmarkProblem(Standard, size)`,
  `MakeStandardSpdProblem`, MPRGP SPD-check sweeps, interior-point path sweeps,
  and mild/near-singular SPD builders may already be fine. Do not refactor all
  benchmark registration gates mechanically without inspecting the generated
  concrete packet.
- That active-friction-index metadata gap is now closed: the demo filter is
  `BM_LcpActiveFrictionIndexContact`, which matches the main `lcp_compare`
  benchmark row family added in this slice. The old two-solver rows remain in
  `bm_lcpsolver_solvers.cpp` as a narrower microbenchmark surface.
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
- `billiard_symmetry_error` is the maximum lateral drift of either billiard
  ball from its initial collision line. The focused panel test passed 43 tests
  after adding the metric.
- The stack benchmark metadata now includes the manifest-driven stack contact
  benchmark across friction-index-capable solvers. The filter keeps the slash in
  `BM_LcpWorldStackContact/` to avoid matching the separate boxed contact
  assembly benchmark unintentionally.
- The billiard invariant plot checkpoint added GUI plots for sequential and
  boxed-LCP momentum error, energy error, and symmetry error histories. The
  focused panel test passed 43 tests after this update.
- Current smoke command metadata in
  `python/examples/demos/scenes/lcp_physics.py` is still
  `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`.
- The current slice adds `benchmark_smoke_filter`,
  `representative_benchmark_filter`, and `representative_benchmark_command` to
  `SceneSetup.info` from `lcp_physics.build()`.
- The representative filter is assembled from the union of every
  `_BENCHMARK_PACKET_ROWS[*]["benchmark_filter"]` token in row order. This keeps
  fresh sessions from having to manually concatenate active-set, contact,
  degeneracy, scaling, billiards, stack, card-pile, and articulated-contact
  benchmark filters.
- `python/tests/unit/test_py_demo_panels.py` should prove that splitting the
  representative filter on `|` equals the union of every benchmark packet row
  filter token, and keep the existing smoke assertion.
- Verification for the current slice passed:
  `python/tests/unit/test_py_demo_panels.py` passed 43 tests and
  `pixi run lint` passed.
- The source-layout docs slice committed as
  `7f4b0227eaf Align LCP docs with snake case headers` updates
  `docs/background/lcp/02_overview.md`,
  `docs/background/lcp/03_pivoting-methods.md`,
  `docs/background/lcp/05_newton-methods.md`, and
  `docs/background/lcp/06_other-methods.md` from legacy PascalCase-style header
  paths to DART 7 snake_case paths.
- `scripts/check_lcp_solver_roster.py` now scans LCP background docs for
  documented `dart/math/lcp` and LCP subdirectory header/source paths and fails
  if the referenced files do not exist.
- Focused verification for the current source-layout docs slice passed:
  `pixi run python scripts/check_lcp_solver_roster.py`.
- Final verification for the current source-layout docs slice passed:
  `pixi run lint`.
- Completed MPRGP predicate checkpoint:
  - `dart/math/lcp/other/mprgp_solver.hpp` now preserves the base overloads with
    `using LcpSolver::supportsProblem` and overrides the tolerance-aware
    predicate.
  - `dart/math/lcp/other/mprgp_solver.cpp` now checks base form support,
    empty-problem success, solver parameter validity, symmetry, and the
    configured LLT positive-definite gate.
  - `tests/unit/math/lcp/test_all_solvers_smoke.cpp` covers SPD, boxed,
    non-symmetric, indefinite, disabled positive-definite check, and invalid
    symmetry-tolerance predicate cases.
  - `python/tests/unit/math/test_lcp.py` covers the default dartpy-visible SPD,
    non-symmetric, indefinite, and boxed cases.
  - `docs/background/lcp/02_overview.md`,
    `docs/background/lcp/06_other-methods.md`,
    `docs/onboarding/python-bindings.md`, and `CHANGELOG.md` document the
    native-support behavior.
  - Focused verification passed:
    `cmake --build build/default/cpp/Release --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy`;
    `ctest -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$'`;
    `python/tests/unit/math/test_lcp.py` with 60 passed;
    `python/tests/unit/test_py_demo_panels.py` with 43 passed; and
    `pixi run python scripts/check_lcp_solver_roster.py`.
- Completed Baraff predicate checkpoint:
  - `dart/math/lcp/pivoting/baraff_solver.hpp` now preserves the base overloads
    with `using LcpSolver::supportsProblem` and overrides the tolerance-aware
    predicate.
  - `dart/math/lcp/pivoting/baraff_solver.cpp` now checks symmetric
    positive-semidefinite matrix support and delegates unsupported standard
    packets to Dantzig before the active-set loop.
  - `tests/unit/math/lcp/test_all_solvers_smoke.cpp` covers SPD, PSD, boxed,
    non-symmetric, indefinite, and fallback solve behavior.
  - `python/tests/unit/math/test_lcp.py` covers the same dartpy-visible support
    and fallback behavior.
  - `docs/background/lcp/02_overview.md`,
    `docs/background/lcp/03_pivoting-methods.md`,
    `docs/onboarding/python-bindings.md`, and `CHANGELOG.md` document the
    native-support behavior.
  - Focused verification passed:
    `cmake --build build/default/cpp/Release --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy`;
    `ctest -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$'`;
    `python/tests/unit/math/test_lcp.py` with 61 passed;
    `python/tests/unit/test_py_demo_panels.py` with 43 passed; and
    `pixi run python scripts/check_lcp_solver_roster.py`.

## How to Resume

```bash
git checkout feature/lcp-solver-interface-demos
git status --short --branch
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git log --oneline --decorate --max-count=12
```

Expected files in the latest final hand-off checkpoint:

- `CHANGELOG.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Expected files in the older Direct support checkpoint:

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

Expected files in the older source-layout docs checkpoint:

- `CHANGELOG.md`
- `docs/background/lcp/02_overview.md`
- `docs/background/lcp/03_pivoting-methods.md`
- `docs/background/lcp/05_newton-methods.md`
- `docs/background/lcp/06_other-methods.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `scripts/check_lcp_solver_roster.py`

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

Historical note: if a fresh checkout somehow lands before the Direct support
checkpoint, confirm the working tree matches the older Direct files above and
commit it with:

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
