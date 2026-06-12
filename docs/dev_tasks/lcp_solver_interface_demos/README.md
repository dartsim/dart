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
- [x] Filtered pivoting scale sweep benchmark registrations through concrete
      generated problem support and removed the now-unused manifest-family
      helper from `lcp_compare`.
- [x] Captured the final 2026-06-11 consolidated hand-off after the latest
      two local benchmark-routing commits. No lint, build, tests, benchmark
      listing, solver execution, or further implementation work was run after
      the user's critical stop instruction.
- [x] Filtered the manifest-generated `BM_LcpCompare` and serial/parallel
      `BM_LcpBatch` argument rows through concrete generated-problem support,
      keeping benchmark problem sizes aligned with solver native-route
      predicates.
- [x] Filtered separated world-contact, world stack-contact, and contact-solver
      comparison benchmark rows through concrete generated contact packets
      while avoiding registration-time generation of the largest dense
      box/articulated fixtures.
- [x] Captured the critical no-verification hand-off after local implementation
      commit `8f0242c2442` so a fresh session can resume from the consolidated
      branch without relying on chat context.
- [x] Added concrete support gates to the remaining heavyweight contact
      benchmark registrations: exact generated-batch checks for mixed
      world-contact batches and representative small contact probes for dense
      box-contact, articulated unified-contact, and dense box-contact batch rows.
- [x] Aligned generated LCP correctness coverage with concrete
      `supportsProblem(problem)` predicates instead of manifest-family support.
- [x] Captured the final no-verification hand-off after the latest two local
      implementation commits, refreshed against `main`, so a fresh session can
      resume from the single consolidated branch.
- [x] Filtered grouped serial/parallel batch benchmark registrations through
      concrete generated grouped-batch support for their exact published
      variants.
- [x] Removed residual manifest-family prechecks from `lcp_compare`
      registration paths that already use concrete generated problem filters.
- [x] Updated the Python LCP demo solver profile so native coverage is derived
      from concrete representative problem cases instead of only static
      manifest surfaces.
- [x] Captured the latest critical no-verification hand-off after the Python
      demo concrete native-case profile slice, with the branch refreshed
      against `main` and ready for publication as one consolidated branch.
- [x] Filtered larger, stress, extreme, and production active-set transition
      benchmark registrations through concrete generated-problem support,
      including exact production-batch problem-list checks.
- [x] Filtered mildly ill-conditioned and near-singular benchmark
      registrations through concrete generated-problem support, including exact
      serial/parallel batch problem-list checks.
- [x] Filtered singular-degenerate friction-index and standard/boxed batch
      benchmark registrations through exact generated batch support.
- [x] Captured the latest 2026-06-11 critical hand-off after the conditioning
      benchmark-routing checkpoint, refreshed the branch against current
      `main`, and stopped without running further verification.
- [x] Routed the all-solvers LCP smoke-test skip helper through concrete
      `supportsProblem(problem)` predicates and verified the focused LCP test
      suite.
- [x] Removed redundant manifest-family prechecks from concrete
      benchmark-routing helpers for active-set transition, mildly
      ill-conditioned, near-singular, and singular-degenerate packets.
- [x] Removed redundant manifest-family prechecks from contact benchmark
      registration paths that already use concrete contact support probes.
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

The latest implementation checkpoint removes redundant manifest-family
prechecks from contact benchmark registration paths that already gate rows
through concrete support probes, following the concrete benchmark helper
cleanup, all-solvers smoke-test concrete support-routing, pivoting scale sweep
concrete support-routing, singular-degenerate batch, conditioning, active set
scale, Python demo concrete native-case profile, benchmark concrete-gate
cleanup, grouped batch support-routing, generated coverage support-routing, and
heavyweight contact benchmark support-gating slices.

## Contact Benchmark Registration Cleanup Checkpoint

The latest implementation checkpoint removes redundant manifest-family
prechecks from contact benchmark registration paths:

- `RegisterActiveFrictionIndexContactBenchmarks()`,
  `RegisterWorldContactBenchmarks()`, `RegisterWorldBoxContactBenchmarks()`,
  `RegisterWorldStackContactBenchmarks()`,
  `RegisterArticulatedUnifiedContactBenchmarks()`,
  `RegisterContactSolverComparisonSweepBenchmarks()`,
  `RegisterContactNormalStandardSweepBenchmarks()`,
  `RegisterWorldContactBatchBenchmarks()`, and
  `RegisterWorldBoxContactBatchBenchmarks()` now rely on their concrete
  generated contact packets or support probes.
- Dense world-box contact registrations still keep their explicit
  `SupportsDenseWorldBoxContactPatch(...)` solver scope before the concrete
  probe, avoiding unrelated solver/performance expansion.
- The representative LCP benchmark/test/demo surface now has no remaining
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::...)` prechecks
  in these audited paths.

Verification for this checkpoint:

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

## Concrete Benchmark Helper Cleanup Checkpoint

The latest implementation checkpoint removes redundant manifest-family gates
from helper predicates that already have concrete generated problems:

- `SolverShouldRunMildIllConditionedBenchmark(...)`,
  `SolverShouldRunNearSingularBenchmark(...)`,
  `SolverShouldRunLargerActiveSetTransitionBenchmark(...)`, and
  `SolverShouldRunSingularDegenerateBenchmark(...)` now rely on their explicit
  solver scopes plus `SolverSupportsConcreteProblem(...)`.
- The now-unused
  `getMildIllConditionedProblemSupport(...)`,
  `getNearSingularProblemSupport(...)`,
  `getLargerActiveSetTransitionProblemSupport(...)`, and
  `getSingularDegenerateProblemSupport(...)` helpers were removed.
- The production active-set transition batch helper now shares the simpler
  concrete-problem helper signature.

Verification for this checkpoint:

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

## All-Solvers Smoke Test Support-Routing Checkpoint

The latest implementation checkpoint aligns all-solvers smoke-test skip logic
with concrete solver support:

- `tests/unit/math/lcp/test_all_solvers_smoke.cpp` no longer maps
  `ProblemCategory` to manifest-family `LcpProblemSupport` for the local
  `canSolve(...)` helper.
- `canSolve(...)` now constructs the solver and asks its concrete
  `supportsProblem(problem.problem)` predicate whether the exact factory
  problem is native-supported.
- This keeps generated smoke coverage aligned with solver-specific native-route
  predicates such as Direct's small standard-LCP window and MPRGP/Baraff
  matrix-domain limits.

Verification for this checkpoint:

```bash
pixi run test-lcpsolver
pixi run lint
```

Observed results:

- The focused LCP test task rebuilt as needed and reported
  `100% tests passed, 0 tests failed out of 17`.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke` passed.
- `pixi run lint` passed, including the LCP solver roster check.

## Pivoting Scale Sweep Support-Routing Checkpoint

The latest implementation checkpoint aligns pivoting scale sweep registration
with concrete solver support:

- `BM_LcpPivotingScaleSweep` registration now builds each exact generated
  problem once and publishes a row only when the selected solver's concrete
  `supportsProblem(problem)` predicate accepts it.
- `RunPivotingScaleSweepBenchmark` now uses the same concrete guard before
  execution, so future additions cannot silently run fallback/delegated rows as
  native pivoting comparisons.
- The now-unused manifest-family `getProblemSupport(...)` helper was removed
  from `lcp_compare`.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpPivotingScaleSweep/(Standard|Boxed|FrictionIndex)/(Direct|Lemke|Baraff|Dantzig)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpPivotingScaleSweep/Standard/Direct/Rows3|BM_LcpPivotingScaleSweep/Standard/Baraff/Rows8|BM_LcpPivotingScaleSweep/Boxed/Dantzig/Rows12|BM_LcpPivotingScaleSweep/FrictionIndex/Dantzig/Contacts4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The first benchmark-list attempt caught that `getProblemSupport(...)` had
  become unused under `-Werror`; the helper was removed.
- The rerun rebuilt `BM_LCP_COMPARE` and listed the expected concrete pivoting
  rows: Direct standard rows 2 and 3, Lemke/Baraff/Dantzig standard rows,
  Dantzig boxed rows, and Dantzig friction-index rows.
- The short benchmark execution reported `contract_ok=1` for sampled Direct
  standard, Baraff standard, Dantzig boxed, and Dantzig friction-index rows.

## Singular-Degenerate Batch Support-Routing Checkpoint

The latest implementation checkpoint aligns singular-degenerate batch
registration with concrete solver support:

- Singular-degenerate friction-index serial/parallel batch rows now build the
  exact four-problem generated batch used by each published row and require
  every problem to pass the solver's concrete `supportsProblem(problem)` route.
- Singular-degenerate standard/boxed serial/parallel batch rows now use the same
  exact-batch check instead of relying only on the single-problem helper.
- Single singular-degenerate registration now precomputes the generated problem
  once per case and passes it through the shared concrete helper.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpSingularDegenerate(FrictionIndexBatch|StandardBoxedBatch)(Serial|Parallel)/(Standard16|Boxed16|CoupledFrictionIndex6|Standard128|Boxed128|CoupledFrictionIndex16|CoupledFrictionIndex256)/(Direct|MPRGP|Baraff|Admm|Sap|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpSingularDegenerateFrictionIndexBatchSerial/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateFrictionIndexBatchParallel/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Standard16/Baraff|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Boxed16/BoxedSemiSmoothNewton' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The benchmark-list check rebuilt `BM_LCP_COMPARE` and listed exact supported
  serial/parallel singular-degenerate batch rows for Admm, Sap, and
  BoxedSemiSmoothNewton on coupled friction-index batches; Baraff, Admm, and
  Sap on standard batches; and Admm, Sap, and BoxedSemiSmoothNewton on boxed
  batches. Direct and MPRGP rows were absent from the scoped filter.
- The short benchmark execution reported `contract_ok=1` for sampled Sap
  friction-index serial and parallel batch rows, Baraff standard-batch rows,
  and BoxedSemiSmoothNewton boxed-batch rows.

## 2026-06-11 Latest Critical Hand-Off Snapshot

The latest user instruction was to stop implementation work and focus on
hand-off only, with no further verification. After that instruction, no lint,
build, tests, benchmark-list commands, benchmark execution, or solver execution
were run. The only remaining work in this checkpoint is repository
housekeeping: record this hand-off state and publish the consolidated branch.

Branch and base state:

- Consolidated branch: `feature/lcp-solver-interface-demos`.
- Latest implementation commit:
  `9a17ba85aa5 Filter conditioning LCP benchmarks concretely`.
- Current `main` was fetched from `https://github.com/dartsim/dart.git` at
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`.
- This checkout's `origin` remote is configured as SSH. Earlier successful
  publishes used HTTPS, so a fresh session should fetch the branch directly
  from GitHub before trusting stale local remote-tracking metadata.

Fresh-session resume checklist:

1. Fetch and check out `feature/lcp-solver-interface-demos` from GitHub.
2. Read this file and
   `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.
3. Inspect the branch tip and choose one bounded remaining LCP interface/demo
   audit gap.
4. Do not mark the broad LCP objective complete or retire this dev-task folder
   from this hand-off alone.

Likely remaining follow-ups:

- Continue auditing any benchmark/demo/test surfaces that still summarize
  solver native support without a concrete `supportsProblem(problem)` check.
- Review remaining solver-domain predicates against actual native solve paths,
  especially where public `solve()` delegates internally.
- Continue py-demo and benchmark apples-to-apples coverage work after the next
  session has re-established local verification.

## Mild/Near-Singular Benchmark Support-Routing Checkpoint

The latest implementation checkpoint aligns conditioning benchmark registration
with concrete solver support:

- `BM_LcpMildIllConditioned` and `BM_LcpNearSingular` registrations now build
  the generated problem for each published case once and keep only solver rows
  whose concrete `supportsProblem(problem)` predicate accepts that packet.
- Mildly ill-conditioned and near-singular serial/parallel batch registrations
  now build the exact four-problem batch used by each row and require every
  concrete batch problem to be supported before publishing the row.

Verification for this checkpoint:

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

## Larger Active-Set Transition Benchmark Support-Routing Checkpoint

The latest implementation checkpoint aligns active-set scaling benchmark
registration with concrete solver support:

- Larger, stress, extreme, and production active-set transition benchmark
  registrations now build the generated problem for each published case once
  and keep only solver rows whose concrete `supportsProblem(problem)` predicate
  accepts that packet.
- Production active-set transition batch registrations now build the exact
  four-problem batch used by the benchmark row and require every concrete batch
  problem to be supported before publishing serial or parallel rows.

Verification for this checkpoint:

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
  Dantzig, PGS, MPRGP, and BoxedSemiSmoothNewton where the generated packets
  are supported; Direct and Baraff were absent from the filtered row set.
- The production-batch benchmark-list check listed serial and parallel rows for
  Standard32, Boxed32, and CoupledFrictionIndex8 for the supported solver set.
- The short benchmark execution reported `contract_ok=1` for
  `BM_LcpLargerActiveSetTransition/Standard32/MPRGP`,
  `BM_LcpStressActiveSetTransition/Boxed64/Pgs`, and
  `BM_LcpProductionActiveSetTransitionBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton/4`.
- `pixi run lint`: passed.

## Python Demo Concrete Native-Case Profile Checkpoint

The latest implementation checkpoint improves the LCP py-demo's apples-to-apples
solver profile:

- The representative solver profile now derives its native coverage label from
  the actual concrete representative problem rows instead of the static
  standard/boxed/findex manifest flags.
- The panel column now reports concrete case coverage such as `standard 2/4`
  for Direct and `standard 4/4, boxed 3/3, findex 2/2` for full-surface
  solvers, making partial native support visible without opening the
  per-problem detail table.

Verification already completed before the latest critical hand-off instruction:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

No lint, build, test, benchmark-list, or solver-execution verification was run
after the final stop-and-handoff instruction. The hand-off checkpoint is
intentionally committed without further validation per that instruction.

## 2026-06-11 Critical Consolidated Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only,
with no further verification. This is the latest snapshot a fresh Claude/Codex
session should trust before resuming.

Branch state before the final hand-off checkpoint commit:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local implementation HEAD:
  `2cd337aa0cf Use concrete gates for LCP benchmark registration`.
- Working tree contained the Python demo concrete native-case profile slice in
  `CHANGELOG.md`, `python/examples/demos/scenes/lcp_physics.py`,
  `python/tests/unit/test_py_demo_panels.py`, and these dev-task docs.
- Remote-tracking branch in this checkout still showed
  `origin/feature/lcp-solver-interface-demos` at
  `737b9c95c11 Document final LCP handoff checkpoint`; treat that as stale
  because previous pushes used HTTPS while `origin` is configured for SSH.
- Local first-parent stack ahead of that stale tracking ref:
  - `2cd337aa0cf Use concrete gates for LCP benchmark registration`
  - `6c0763927a5 Filter grouped LCP batch rows concretely`
  - `20aaa23d0fe Document final LCP no-verification handoff`
  - `fe5b70b32cc Route generated LCP coverage by concrete support`
  - `f86e353df2f Gate heavyweight LCP contact benchmarks concretely`
- `main` was fetched over HTTPS from `https://github.com/dartsim/dart.git` to
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`

This final checkpoint should be pushed to the same consolidated branch,
`feature/lcp-solver-interface-demos`. The broad LCP objective is still open;
do not retire this dev-task folder or mark the work complete from this
checkpoint alone.

Recommended resume entrypoint:

1. Fetch `feature/lcp-solver-interface-demos` directly from GitHub and inspect
   the branch tip; do not rely on the stale SSH tracking ref recorded above.
2. Read this file and
   `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.
3. Continue one bounded LCP interface/demo audit gap at a time, starting with
   remaining demos/tests that summarize native rows without a concrete
   `supportsProblem(problem)` check.

## Benchmark Concrete-Gate Cleanup Checkpoint

The latest implementation checkpoint removes the last coarse manifest-family
prechecks from benchmark registration paths that already had concrete generated
problem filters:

- `BM_LcpCompare`, `BM_LcpActiveSetTransition`, `BM_LcpBatchSerial`, and
  `BM_LcpBatchParallel` now rely on concrete generated packet support checks to
  decide whether rows should be published.
- `bm_lcp_compare.cpp` no longer contains registration-time checks of the form
  `supportsProblem(solver, getProblemSupport(...))`; the remaining benchmark
  support gates instantiate solvers and call `supportsProblem(problem)` on the
  generated packet or problem list.

Verification for this checkpoint:

```bash
rg -n "supportsProblem\\(solver, getProblemSupport|supportsProblem\\(solverEntry|supportsProblem\\(solver,|supportsProblem\\(\\*solverEntry" \
  tests/benchmark/lcpsolver/bm_lcp_compare.cpp
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpCompare/Standard/(Direct|MPRGP|Baraff)|BM_LcpActiveSetTransition/Standard/(Direct|MPRGP|Baraff)|BM_LcpBatch(Serial|Parallel)/Standard/(Direct|MPRGP|Baraff)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpCompare/Standard/MPRGP/12|BM_LcpActiveSetTransition/Standard/Baraff|BM_LcpBatchSerial/Standard/Direct/3/4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The `rg` check found no remaining `supportsProblem(solver,
getProblemSupport(...))` registration patterns in `bm_lcp_compare.cpp`.
- The benchmark-list check rebuilt `BM_LCP_COMPARE` and preserved concrete
  standard rows for Direct, MPRGP, and Baraff across single, active-set, and
  serial/parallel batch families; Direct remained absent from the active-set
  transition row.
- The short benchmark execution reported `contract_ok=1` for
  `BM_LcpCompare/Standard/MPRGP/12`,
  `BM_LcpActiveSetTransition/Standard/Baraff`, and
  `BM_LcpBatchSerial/Standard/Direct/3/4`.
- `pixi run lint`: passed.

## Grouped Batch Benchmark Support-Routing Checkpoint

The latest implementation checkpoint aligns grouped batch benchmark
registration with concrete solver support:

- `BM_LcpGroupedBatchSerial` and `BM_LcpGroupedBatchParallel` now build the
  exact generated grouped batches for the published two- and three-variant rows
  and register only rows whose concrete problem list is accepted by the solver's
  `supportsProblem(problem)` predicate.
- The benchmark remains scoped to Jacobi and PGS to preserve the existing
  CPU/CUDA-comparable surface, but the published row set is no longer gated by
  only the manifest-level problem family.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpGroupedBatch(Serial|Parallel)/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpGroupedBatchSerial/Standard/Jacobi/2|BM_LcpGroupedBatchParallel/FrictionIndex/Pgs/2' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The benchmark-list check rebuilt `BM_LCP_COMPARE` and listed 24 grouped
  serial/parallel rows: Jacobi and PGS for standard, boxed, and friction-index
  families, with two- and three-variant grouped rows for each.
- The short benchmark execution reported `contract_ok=1` for
  `BM_LcpGroupedBatchSerial/Standard/Jacobi/2` and
  `BM_LcpGroupedBatchParallel/FrictionIndex/Pgs/2`.
- `pixi run lint`: passed.

## 2026-06-11 Final No-Verification Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only,
with no further verification. This section captures the branch state for a
fresh Claude/Codex session.

Branch state before this docs-only hand-off checkpoint:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local implementation HEAD:
  `fe5b70b32cc1 Route generated LCP coverage by concrete support`.
- Previous implementation checkpoint:
  `f86e353df2f Gate heavyweight LCP contact benchmarks concretely`.
- Remote-tracking branch in this checkout:
  `origin/feature/lcp-solver-interface-demos` at
  `737b9c95c11 Document final LCP handoff checkpoint`.
- Local status before this docs-only edit:
  `feature/lcp-solver-interface-demos` was ahead of the remote-tracking branch
  by two commits.
- `main` was fetched over HTTPS from `https://github.com/dartsim/dart.git` to
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`
- An SSH fetch from `origin` failed in this environment because GitHub port 22
  was unreachable; HTTPS fetch succeeded.
- No PR was associated with the branch when checked earlier in the session.

Current first-parent implementation stack before this docs-only checkpoint:

```text
fe5b70b32cc Route generated LCP coverage by concrete support
f86e353df2f Gate heavyweight LCP contact benchmarks concretely
737b9c95c11 Document final LCP handoff checkpoint
8f0242c2442 Filter LCP contact benchmark rows concretely
4c63db30bd7 Filter LCP benchmark args concretely
be4643d1743 Document consolidated LCP handoff state
```

Important hand-off constraints:

- The broad LCP objective is still open. Do not mark this dev task complete
  only because the branch is consolidated and published.
- Treat verification notes below as historical evidence for their named
  implementation checkpoints. They are not evidence for this docs-only
  hand-off checkpoint.
- A fresh session should fetch
  `feature/lcp-solver-interface-demos`, read this file and
  `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`, then continue from one
  bounded remaining LCP support-routing, solver-domain, benchmark, or py-demo
  gap.

Likely next bounded continuations:

- Audit `tests/unit/math/lcp/test_all_solvers_smoke.cpp` before changing it:
  some manifest-level checks there may intentionally test solver manifest
  categories rather than concrete native routing.
- Continue the solver documentation and py-demo coverage audit only after
  identifying a concrete mismatch between documented native domains,
  `supportsProblem(problem)`, benchmark rows, and demo metadata.

## Generated Coverage Support-Routing Checkpoint

The latest implementation checkpoint aligns generated LCP correctness coverage
with the concrete solver support predicate used by demos and benchmarks:

- `tests/unit/math/lcp/test_lcp_generated_coverage.cpp` now creates the solver
  and calls `supportsProblem(testCase.problem)` before running each generated
  case.
- The older manifest-family gate and Direct-only size special case were removed
  from the shared `solverShouldRun(...)` path. Direct size support, Baraff
  PSD-only native support, and MPRGP SPD-only native support are now all routed
  through the same solver predicates as public callers.

Verification for this checkpoint:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_generated_coverage --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_math_lcp_math_lcp_lcp_generated_coverage$'
pixi run lint
```

Observed results:

- `UNIT_math_lcp_math_lcp_lcp_generated_coverage` rebuilt successfully.
- CTest `^UNIT_math_lcp_math_lcp_lcp_generated_coverage$`: passed.
- `pixi run lint`: passed.

## Heavyweight Contact Benchmark Support-Gating Checkpoint

The latest implementation checkpoint replaces the remaining manifest-only
contact benchmark gates without making benchmark registration construct the
largest dense contact fixtures:

- `BM_LcpWorldBoxContact/FrictionIndex` now builds a one-box concrete support
  probe and registers dense box-count rows only for dense-box-scoped solvers
  whose `supportsProblem(problem)` accepts that concrete contact packet.
- `BM_LcpArticulatedUnifiedContact/FrictionIndex` now builds one-contact
  support probes for the ground, rigid-impact, and cross-link articulated cases
  and registers each case only for solvers that accept the concrete probe.
- `BM_LcpWorldContactBatchSerial/FrictionIndex` and
  `BM_LcpWorldContactBatchParallel/FrictionIndex`, including the stress-stack
  and contact-pipeline-32 families, now check each generated batch's concrete
  problem list through `supportsProblem(problem)` before publishing serial or
  parallel rows.
- `BM_LcpWorldBoxContactBatchSerial/FrictionIndex` and
  `BM_LcpWorldBoxContactBatchParallel/FrictionIndex` now build a one-box
  batch support probe and publish dense box batch rows only for dense-box-scoped
  solvers that accept that concrete batch.

Scope note: the dense box-contact and articulated unified-contact families
still avoid exact per-argument generation of the 256-contact fixtures during
benchmark listing. The registration gate is concrete and representative for
the contact family; the benchmark body remains the source of truth for each
large fixture.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpWorldBoxContact/FrictionIndex/(Pgs|Admm)|BM_LcpWorldContact(Batch|StressBatch|Pipeline32Batch)(Serial|Parallel)/FrictionIndex/(Pgs|Admm)|BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/(Pgs|Admm)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpArticulatedUnifiedContact/FrictionIndex/(Ground|RigidImpact|CrossLinkImpact)/(Pgs|Admm)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpWorldBoxContact/FrictionIndex/Pgs/1|BM_LcpArticulatedUnifiedContact/FrictionIndex/Ground/Admm/1|BM_LcpWorldContactBatchSerial/FrictionIndex/Pgs$|BM_LcpWorldBoxContactBatchSerial/FrictionIndex/Pgs/1/4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The first benchmark-list check built `BM_LCP_COMPARE` and listed PGS and
  ADMM rows for dense world-box contact, baseline/stress/contact-pipeline mixed
  world-contact batches, and dense world-box contact batches.
- The articulated benchmark-list check listed PGS and ADMM rows for ground,
  rigid-impact, and cross-link articulated unified-contact cases.
- The short benchmark execution reported `contract_ok=1` for the targeted
  dense box-contact, articulated unified-contact, mixed world-contact batch,
  and dense box-contact batch rows; the regex also matched larger dense
  box-contact and articulated rows, which also reported `contract_ok=1`.

## Critical No-Verification Hand-Off

Current branch:

- `feature/lcp-solver-interface-demos`

Local first-parent stack at the hand-off point, before this docs-only
checkpoint:

- `8f0242c2442 Filter LCP contact benchmark rows concretely`
- `4c63db30bd7 Filter LCP benchmark args concretely`
- `be4643d1743 Document consolidated LCP handoff state`
- `02c6d0acb4b Filter active-set LCP benchmark rows concretely`
- `b2e212db5c4 Add active friction-index LCP benchmark rows`
- `d143d0dc355 Document latest LCP handoff state`

Important state:

- The working tree was clean before this docs-only hand-off edit.
- Local tracking showed the branch ahead of
  `origin/feature/lcp-solver-interface-demos` by five commits because previous
  HTTPS pushes did not refresh the SSH remote-tracking ref in this checkout.
  A fresh session should inspect the remote branch directly before assuming the
  tracking ref is authoritative.
- The latest user instruction explicitly prohibited further verification. Any
  future session should treat verification data below as historical evidence
  for the checkpoint that recorded it, not as evidence for this final docs-only
  checkpoint.
- The broad LCP objective is not complete. This dev-task folder should remain
  active until remaining follow-up work is completed or moved into durable
  planning/design docs.

Resume from:

1. Fetch the branch and inspect `git status --short --branch` and
   `git log --oneline --decorate --max-count=12 --first-parent`.
2. Read this file and `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.
3. Continue with one bounded checkpoint, preferably a remaining concrete
   support-routing gap in `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`.

## World/Contact Benchmark Routing Checkpoint

The latest implementation checkpoint aligns low-cost contact-derived benchmark
registration with concrete native solver support:

- `BM_LcpWorldContact/FrictionIndex` now precomputes the 1-, 2-, and 4-contact
  separated world packets once and registers only solver/arg rows whose concrete
  packet is accepted by `supportsProblem(problem)`.
- `BM_LcpWorldStackContact/FrictionIndex` now precomputes the existing
  2- through 16-, 24-, and 32-sphere stack packets once and filters those args
  through the same concrete support predicate.
- `BM_LcpContactSolverComparisonSweep` now builds each DART 7 contact-pipeline
  comparison packet once per case and filters the scoped comparison solvers
  (`Admm`, `Sap`, and `BoxedSemiSmoothNewton`) through concrete support.
- Dense world-box contact, articulated unified contact, and batch contact rows
  were completed by the later heavyweight contact support-gating checkpoint.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpWorldContact/FrictionIndex/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpWorldStackContact/FrictionIndex/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpContactSolverComparisonSweep/(Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpContactSolverComparisonSweep/(Admm|Sap|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpWorldContact/FrictionIndex/Dantzig/1|BM_LcpWorldStackContact/FrictionIndex/Pgs/2|BM_LcpContactSolverComparisonSweep/Admm/WorldSeparated1' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The separated world-contact and stack-contact row-list check built
  `BM_LCP_COMPARE` and preserved representative Dantzig, PGS, and
  BoxedSemiSmoothNewton rows.
- The contact-solver comparison row-list check confirmed the scoped solver set
  (`Admm`, `Sap`, `BoxedSemiSmoothNewton`) remains registered for the current
  world, stack, and articulated contact-pipeline cases.
- The short benchmark execution reported `contract_ok=1` on the targeted rows;
  the regex also matched larger `Pgs/24` and `Admm/WorldSeparated16` rows, which
  also reported `contract_ok=1`.

## Manifest And Batch Benchmark Argument Routing Checkpoint

The latest implementation checkpoint aligns manifest-generated benchmark
argument rows with concrete native solver support:

- `BM_LcpCompare` registration now precomputes candidate problem sizes and
  keeps only sizes whose generated `MakeBenchmarkProblem(...)` packet is
  accepted by the solver's concrete `supportsProblem(problem)` predicate.
- `BM_LcpBatchSerial` and `BM_LcpBatchParallel` now apply the same filter to
  every generated problem in each `MakeBenchmarkProblemBatch(...)` row.
- The existing benchmark suite shape is preserved: Direct remains limited to
  its tiny standard-LCP rows, Dantzig/Baraff/MPRGP keep their standard SPD
  rows, and boxed/friction-index rows stay available for the solvers that
  accept the generated concrete packets.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpCompare/Standard/(Direct|Dantzig|MPRGP|Baraff)|BM_LcpBatchSerial/Standard/(Direct|Dantzig|MPRGP|Baraff)|BM_LcpBatchParallel/Standard/(Direct|Dantzig|MPRGP|Baraff)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpCompare/(Boxed|FrictionIndex)/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpBatchSerial/(Boxed|FrictionIndex)/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpBatchParallel/(Boxed|FrictionIndex)/(Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpCompare/Standard/Direct|BM_LcpBatchSerial/Standard/Direct|BM_LcpCompare/FrictionIndex/BoxedSemiSmoothNewton|BM_LcpBatchParallel/Boxed/Pgs/24/4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The standard row-list check built `BM_LCP_COMPARE` and listed Direct only for
  `2`/`3` single packets and `3/4` batch packets, while Dantzig, Baraff, and
  MPRGP kept their existing standard SPD rows.
- The boxed/friction-index row-list check kept representative Dantzig, PGS, and
  BoxedSemiSmoothNewton rows for single, serial batch, and parallel batch
  benchmark families.
- The short benchmark execution reported `contract_ok=1` for all representative
  affected rows that were run.

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

## 2026-06-11 Final Consolidated Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only
with no further verification. No lint, build, tests, benchmark listing, solver
execution, or implementation work was run after that instruction.

Branch state before this docs-only hand-off update:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `02c6d0acb4b4 Filter active-set LCP benchmark rows concretely`.
- Remote tracking branch: `origin/feature/lcp-solver-interface-demos` at
  `d143d0dc355c Document latest LCP handoff state`.
- Local branch was two commits ahead of the tracking branch:
  - `b2e212db5c4 Add active friction-index LCP benchmark rows`
  - `02c6d0acb4b Filter active-set LCP benchmark rows concretely`
- `main` was refreshed over HTTPS to `7d05d7b9ea72`, then
  `git merge --no-edit FETCH_HEAD` reported `Already up to date.`
- The working tree was clean before this docs-only hand-off update.
- No PR was associated with the branch when checked earlier in the session.

This final hand-off checkpoint should remain on the same consolidated branch,
`feature/lcp-solver-interface-demos`, together with the two latest
benchmark-routing commits. A fresh session should resume from that branch tip
and continue the remaining LCP interface/demo audit; the broad task is not
complete.

Likely next bounded continuations:

- Inspect remaining manifest-level benchmark gates in
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` and only convert them to
  concrete `supportsProblem(problem)` gates when the generated packet actually
  exercises a narrower native domain.
- Re-check solver documentation against concrete per-problem support predicates
  for any solver that still delegates internally from its public `solve()` path.
- Improve py-demo benchmark packets only when the referenced rows exist on the
  same benchmark executable surface used by
  `representative_benchmark_command`.

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
2. Check that the remote `feature/lcp-solver-interface-demos` branch contains
   the final hand-off checkpoint from this session; do not trust the stale
   local tracking ref without fetching.
3. Fetch `origin/main`; if it moved, merge it into this branch before any later
   push.
4. Continue the LCP interface/demo audit from the next concrete gap. Good
   starting points are any remaining manifest-level benchmark gates in
   `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` that still publish native
   rows without generated-problem support checks or representative
   contact-family probes, plus any solver whose native mathematical domain is
   still broader in docs than in `supportsProblem(problem)`.
5. Prefer a bounded checkpoint: one benchmark/demo routing gap, focused tests,
   `pixi run lint`, then an additive commit.
