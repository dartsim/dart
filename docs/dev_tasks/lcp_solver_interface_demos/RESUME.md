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

## Current Branch

`feature/lcp-solver-interface-demos` — consolidated branch for this work. At
the start of this docs-only hand-off update, the latest code checkpoint was
`7f4b0227eaf Align LCP docs with snake case headers`; the branch was two
commits ahead of `origin/feature/lcp-solver-interface-demos`, which was at
`eb2147d9e6b Document current LCP handoff state`.

At the start of the MPRGP predicate slice, the branch was clean at
`f3436654bbd Document critical LCP handoff state` and matched
`origin/feature/lcp-solver-interface-demos`. After committing this slice, the
local branch may be one commit ahead unless the maintainer explicitly requests a
push.

After the MPRGP checkpoint commit, the local branch was clean at
`ee08bbe8ce0 Report MPRGP native support precisely` and one commit ahead of
`origin/feature/lcp-solver-interface-demos`. After committing the Baraff slice,
the local branch may be two commits ahead unless the maintainer explicitly
requests a push.

The earlier SSH fetch/push failed on `github.com:22`, but HTTPS fetch later
succeeded. `origin/main` was refreshed over HTTPS and confirmed to be an
ancestor of `HEAD` before this representative benchmark-command slice.

## Immediate Next Step

Continue the next smallest LCP solver/interface/demo audit gap. Good next
targets are standard-only solvers whose native mathematical domain may be
narrower than form-level support, or representative demo/benchmark rows that
still do not expose a solver-specific strength or failure mode. Do not push
unless the maintainer/user gives explicit approval in the current turn.

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
  `f09ea880ea8 Align LCP docs with snake case headers` updates
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
- No verification was run after the user's critical stop-and-handoff
  instruction.
- Interrupted audit evidence for MPRGP:
  - `dart/math/lcp/lcp_solver.hpp` currently provides the base virtual
    `supportsProblem(problem, standardTolerance)` as a form-level predicate:
    standard, friction-index, or boxed.
  - `dart/math/lcp/other/mprgp_solver.hpp` documents MPRGP as a standard-LCP
    solver for symmetric positive definite matrices, but it does not override
    `supportsProblem(...)`.
  - `dart/math/lcp/other/mprgp_solver.cpp` has `isSymmetric(...)`, delegates
    non-standard problems to Dantzig, delegates non-symmetric standard problems
    to Dantzig, and delegates matrices that fail `Eigen::LLT` when
    `checkPositiveDefinite` is true.
  - A likely patch is to add `using LcpSolver::supportsProblem;` and override
    `bool supportsProblem(const LcpProblem&, double standardTolerance) const`
    in `MprgpSolver`, returning true only for standard problems that satisfy the
    configured native symmetry and positive-definite checks.
  - Focused C++ coverage should include an SPD standard packet, a boxed packet,
    a non-symmetric standard packet, a symmetric indefinite packet, and the
    `checkPositiveDefinite = false` parameter case.
  - Python/demo counts are expected to remain unchanged because the current
    representative standard demo packets appear symmetric positive definite.
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

Expected files in the latest source-layout docs checkpoint:

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
