# Resume: LCP Solvers

## Last Session Summary

The worktree is on `main` with uncommitted changes adding solver-agnostic
correctness coverage for advanced LCP solvers, a shared DART 7 LCP solver
manifest for tests, manifest-generated comparison benchmark registration, a
solver-by-solver implementation audit, manifest-driven generated correctness
coverage, generated invalid-problem rejection checks, generated batch-shaped
state-reuse checks, pivoting-solver near-singular standard checks, and
manifest-generated serial and DART 7 `ParallelExecutor` batch benchmarks. The
completed slices do not finish the full objective; they close evidence gaps in
known-solution coverage, manifest-backed coverage drift checks, benchmark
registration, generated standard/boxed/friction-index coverage including
coupled friction-index cases, invalid/failure reporting, scoped larger
well-conditioned, production-scale, and near-singular generated grids, backend
build-state benchmark counters, focused local SIMD-enabled CPU evidence,
focused CUDA-enabled build/runtime evidence, and documented implementation
status. They also add opt-in solver-internal CPU worker threads for
`JacobiSolver` with generated correctness and benchmark evidence, plus
experimental CUDA fixed-iteration Jacobi and PGS batch paths for homogeneous
dense standard, boxed, friction-index, grouped variable-size synthetic
standard/boxed/friction-index, 4-/8-/16-contact world-contact, and homogeneous
5-sphere coupled stack-contact LCP packets, with grouped articulated
unified-contact CUDA evidence later expanded to cross-multibody link-vs-link
packets in this task log.
They also fix boxed semi-smooth Newton's `findex` moving-bound Jacobian and add
the previously failing coupled mildly ill-conditioned 4-contact friction-index
case to the generated all-solver grid.
The work now also adds a scoped larger mildly ill-conditioned generated
known-solution slice for standard 32/64-row, boxed 16/32-row, friction-index
8-contact, and coupled friction-index 6-, 8-, 12-, 16-, and 24-contact
packets, plus stronger-coupled 16-/24-contact packets with 4x and 8x
cross-contact coupling and 170 matching `BM_LcpMildIllConditioned` benchmark
rows verified in default, SIMD-enabled, and CUDA-enabled build trees. It also
adds 56 `BM_LcpMildIllConditionedBatch(Serial|Parallel)` rows for batch-size-4
serial and DART 7 `ParallelExecutor` runs over the 8x-coupled 16-/24-contact
mildly ill-conditioned packets, verified in default, SIMD-enabled, and
CUDA-enabled build trees.
It now also adds DART 7 per-contact block-structure evidence for BGS and
Blocked Jacobi on a real two-contact boxed-LCP world-contact snapshot: the
focused tests pass when blocks are derived from non-contiguous `findex`
dependencies and reject explicit block sizes that split tangent rows from their
owning normal rows. A focused `BM_LCP_COMPARE` gate on the merged default tree
also verifies 22 BGS/Blocked Jacobi world-contact, stack-contact, and
serial/parallel batch rows with `bad_contract=0`.
It now also extends the robust near-singular generated slice to coupled
friction-index 12-contact packets and adds 13 matching `BM_LcpNearSingular`
benchmark rows for standard 8-row, boxed 8-row, and coupled friction-index
3-, 6-, 9-, and 12-contact packets verified in default, SIMD-enabled, and
CUDA-enabled build trees. The CUDA-enabled rows are CPU solver rows in a
CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds exact rank-deficient singular-degenerate generated coverage
for standard 16-row, boxed 16-row, and coupled friction-index 6-contact
packets, plus 27 matching `BM_LcpSingularDegenerate` benchmark rows verified in
the default, SIMD-enabled, and CUDA-enabled build trees. The standard
singular-degenerate packet covers 21 standard-capable solvers. The boxed and
friction-index singular-degenerate packets are scoped to `Admm`, `Sap`, and
`BoxedSemiSmoothNewton` after broader trials exposed contract failures for the
other tried boxed/findex paths.
It now also extends that slice to larger exact rank-deficient standard 32-row,
boxed 32-row, and coupled friction-index 8-contact packets, plus 27 matching
`BM_LcpLargerSingularDegenerate` benchmark rows verified in the default,
SIMD-enabled, and CUDA-enabled build trees. The CUDA-enabled rows are CPU solver
rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also extends that exact rank-deficient coverage to a stress slice:
standard 64-row, boxed 64-row, and coupled friction-index 12-contact packets,
plus 27 matching `BM_LcpStressSingularDegenerate` benchmark rows verified in
the default, SIMD-enabled, and CUDA-enabled build trees. The CUDA-enabled rows
are CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also extends that exact rank-deficient coverage to an extreme slice:
standard 128-row, boxed 128-row, and coupled friction-index 16-contact
packets, plus 27 matching `BM_LcpExtremeSingularDegenerate` benchmark rows
verified in the default, SIMD-enabled, and CUDA-enabled build trees. The
CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
kernel execution.
It now also extends active-set transition coverage to a stress slice: standard
64-row, boxed 64-row, and coupled friction-index 12-contact packets, plus 49
matching `BM_LcpStressActiveSetTransition` benchmark rows verified in the
default, SIMD-enabled, and CUDA-enabled build trees. The CUDA-enabled rows are
CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also extends active-set transition coverage to an extreme slice: standard
128-row, boxed 128-row, and coupled friction-index 16-contact packets, plus 49
matching `BM_LcpExtremeActiveSetTransition` benchmark rows verified in
the default, SIMD-enabled, and CUDA-enabled build trees. The CUDA-enabled rows
are CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also extends active-set transition coverage to a production
friction-index slice: stronger-coupled 24-contact, 72-row and 32-contact,
96-row packets, plus 32 matching `BM_LcpProductionActiveSetTransition`
benchmark rows verified in the default, SIMD-enabled, and CUDA-enabled build
trees. It now also adds 64
`BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
batch-size-4 serial and DART 7 `ParallelExecutor` runs over those same
production packets, verified in the default, SIMD-enabled, and CUDA-enabled
build trees. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled build,
not CUDA LCP kernel execution.
The work adds DART 7 experimental `World` boxed-LCP contact snapshot tests that
validate real LCPs assembled from one sphere-ground friction contact and from
two separated sphere-ground friction contacts, plus a 200-step two-sphere
boxed-LCP `World::step()` invariant test. It now also adds contact-derived
benchmark rows that compare all friction-index-capable solvers on identical
1/2/4 separated sphere-ground boxed/findex contact snapshots, plus boxed-LCP
contact assembly/solve benchmark rows for the same contact counts. It also adds
2/3-sphere vertical-stack benchmark rows where sphere-ground and sphere-sphere
contacts share dynamic bodies, plus stack assembly/solve benchmark rows and a
3-sphere stack snapshot unit test that validates nonzero normal-contact
coupling. It now extends that snapshot evidence to a 4-sphere, 4-contact
stack, and adds 4-sphere stack benchmark rows for every friction-index-capable
solver except `NNCG`; a focused `NNCG` 4-sphere benchmark trial did not satisfy
the contract, so that row is not claimed. It now also extends the direct
snapshot/assembly evidence to a 5-sphere, 5-contact, 15-row coupled stack,
verified in default, SIMD-enabled, and CUDA-enabled build trees. It also adds
3-sphere 200-step, 3-sphere 500-step, and 4-sphere 200-step public
`World::step()` invariant tests and benchmark rows for the boxed-LCP stack path
after adding the boxed-LCP contact solve's timestep-driven Baumgarte velocity
bias, plus 4- and 16-sphere separated sphere-ground `World::step()` invariant
tests and 4-/8-/16-contact separated sphere-ground step benchmark rows. It now
also adds a fixed-base prismatic
articulated link-ground `World::step()` invariant tests for one-link and
four-link scenes and 1-/4-/8-/16-link articulated ground-step benchmark rows
through the public boxed-LCP unified constraint path. It now also adds connected
fixed-base three-axis prismatic Cartesian-chain `World::step()` invariant
coverage and 1-/4-/8-/16-chain articulated Cartesian ground-step benchmark rows
through the same public unified path. It now also adds a
fixed-base prismatic link-vs-dynamic-rigid `World::step()` invariant test and
1-/4-/8-/16-pair articulated rigid-impact benchmark rows through the same
public unified path. It now also adds a cross-multibody fixed-base prismatic
link-vs-link `World::step()` invariant test and 1-/4-/8-/16-pair articulated
link-impact benchmark rows through the same public unified path. It now also
adds
all-solver articulated unified-contact benchmark rows for
manually assembled fixed-base three-axis prismatic link-ground and
link-vs-dynamic-rigid LCP snapshots, now extended to cross-multibody
link-vs-link LCP snapshots. It now also adds mixed
world-contact batch benchmark rows that compare all friction-index-capable
solvers over the same five separated-contact and stacked-contact snapshots,
both serially and through the DART 7 experimental
`ParallelExecutor`. The CUDA LCP batch evidence now also includes homogeneous
4-/8-/16-contact DART 7 world-contact packets, homogeneous 5-sphere coupled
stack-contact packets, grouped variable-size synthetic standard/boxed/findex
packets, grouped variable-size 1/2/4/8/16-contact separated sphere-ground
packets, plus grouped variable-size 2/3/4/5-sphere coupled stack-contact
packets, plus grouped variable-size manually assembled 1-/4-contact
articulated unified-contact packets covering link-ground, link-vs-dynamic-rigid,
and cross-multibody link-vs-link cases, plus mixed grouped contact batches that
combine separated, stack, and articulated fixture families for fixed-iteration
Jacobi and PGS.
The work now also adds 49 `BM_LcpLargerActiveSetTransition` rows for the
scoped scalable active-set transition packets: standard 32-row, boxed 32-row,
and coupled friction-index 8-contact. Focused default, SIMD-enabled, and
CUDA-enabled runs pass with `contract_ok=1`; the CUDA-enabled rows are CPU
solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.

## Current Branch

`main` - uncommitted local changes.

## Immediate Next Step

Move from the now-verified extreme 128-row/16-contact exact rank-deficient,
production active-set transition 32-contact, coupled mildly ill-conditioned
24-contact, and near-singular 12-contact CPU solver rows toward broader
solver-specific conditioning grids and direct backend execution evidence beyond
SIMD/CUDA-enabled build-state counters.
Broaden SIMD benchmark gates, solver-internal threaded benchmark evidence, and
the CUDA Jacobi/PGS batch slices into broader/general CUDA LCP execution
separately from the current serial, task-parallel independent-problem, simple
world-contact, mixed world-contact batch, small coupled-stack, 16-contact
separated CUDA world-contact, and 5-sphere homogeneous/grouped CUDA
coupled-stack, and grouped manually assembled articulated unified-contact
including cross-multibody link-vs-link CUDA rows, plus mixed
separated/stack/articulated CUDA benchmark rows, plus the current
PGS-only homogeneous and grouped variable-size dense box-face CUDA batch rows.
After that, extend
DART 7 boxed-LCP world-contact
evidence beyond the current separated sphere-ground, fixed-base prismatic
articulated end-to-end coverage, connected Cartesian-chain articulated
end-to-end coverage, cross-multibody articulated link-vs-link impact coverage,
manually assembled three-axis articulated LCP snapshots, 4-sphere coupled-stack
end-to-end, 5-sphere vertical stack snapshots,
and dense box face-contact slice to broader articulated, longer-running coupled,
and broader dense/robot-like contact scenes.

## Context That Would Be Lost

- The task is DART 7 only. The default local configure used this session
  reported DART 7.0.0 and `DART_BUILD_SIMULATION_EXPERIMENTAL=ON`.
- Current proof is intentionally narrow: 10 all-solver smoke/advanced tests, 44
  `LcpComparisonHarness` tests, and 21 default-build `LcpGeneratedCoverage`
  tests pass,
  covering known solutions for all comparison-harness solvers after adding
  `Apgd`, `Tgs`, `Admm`, `Sap`, boxed semi-smooth Newton, and active-set
  transition cases. The active-set transition correctness grid now includes
  scoped scalable-solver cases for standard 32-row, 64-row, and 128-row, boxed
  32-row, 64-row, and 128-row, and coupled friction-index 8-contact,
  12-contact, and 16-contact packets, plus a stronger-coupled production
  coupled friction-index 24-contact packet and a stronger 32-contact packet. The
  larger mildly ill-conditioned
  correctness grid now includes standard 32/64-row, boxed 16/32-row,
  friction-index 8-contact, and coupled friction-index 6-, 8-, 12-, 16-, and
  24-contact packets over a scoped solver set; `MPRGP` is excluded after a
  standard 32-row trial satisfied the LCP contract but missed the selected
  expected-solution tolerance. A short `BM_LCP_COMPARE_SMOKE` benchmark run also
  passes.
  The singular-degenerate correctness grid includes exact rank-deficient
  standard 16-row, boxed 16-row, and coupled friction-index 6-contact packets.
  It covers 21 standard-capable solvers on the standard packet and `Admm`,
  `Sap`, and `BoxedSemiSmoothNewton` on the boxed and friction-index packets.
  The larger singular-degenerate correctness grid covers exact rank-deficient
  standard 32-row, boxed 32-row, and coupled friction-index 8-contact packets
  over the same scoped robust solver set.
  The stress singular-degenerate correctness grid covers exact rank-deficient
  standard 64-row, boxed 64-row, and coupled friction-index 12-contact packets
  over the same scoped robust solver set.
  The extreme singular-degenerate correctness grid covers exact
  rank-deficient standard 128-row, boxed 128-row, and coupled friction-index
  16-contact packets over the same scoped robust solver set.
  `BM_LCP_COMPARE` now lists 55 `BM_LcpActiveSetTransition` rows and the
  focused active-set benchmark slice passes with `contract_ok=1` for standard
  16-row, boxed 16-row, and coupled friction-index 6-contact packets.
  `DirectSolver` is intentionally omitted from the standard active-set rows
  because the 16-row direct path dispatches to the Dantzig fallback. Focused
  standard/boxed/friction-index rows also pass in the SIMD and CUDA-enabled
  build trees with the expected backend counters.
  `BM_LCP_COMPARE` also lists 49 `BM_LcpLargerActiveSetTransition` rows for
  standard 32-row, boxed 32-row, and coupled friction-index 8-contact packets
  over the scoped scalable solver set already proven by generated correctness
  coverage; the focused default, SIMD-enabled, and CUDA-enabled runs pass with
  `contract_ok=1` on all rows and report `active_set_transition=1`,
  `larger_active_set_transition=1`, backend build-state counters, and
  `contact_count=8`/`coupled=1` for the coupled friction-index packet. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
  `BM_LCP_COMPARE` also lists 49 `BM_LcpStressActiveSetTransition` rows for
  standard 64-row, boxed 64-row, and coupled friction-index 12-contact packets
  over the scoped scalable solver set already proven by generated correctness
  coverage; the focused default, SIMD-enabled, and CUDA-enabled runs pass with
  `contract_ok=1` on all rows and report `active_set_transition=1`,
  `stress_active_set_transition=1`, backend build-state counters, and
  `contact_count=12`/`coupled=1` for the coupled friction-index packet. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
  `BM_LCP_COMPARE` also lists 49 `BM_LcpExtremeActiveSetTransition` rows for
  standard 128-row, boxed 128-row, and coupled friction-index 16-contact
  packets over the scoped scalable solver set already proven by generated
  correctness coverage; the focused default run passes with `contract_ok=1` on
  all rows, and the focused SIMD-enabled and CUDA-enabled runs also pass with
  `contract_ok=1` on all rows. The rows report `active_set_transition=1`,
  `extreme_active_set_transition=1`, backend build-state counters, and
  `contact_count=16`/`coupled=1` for the coupled friction-index packet. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
  `BM_LCP_COMPARE` also lists 32
  `BM_LcpProductionActiveSetTransition` rows for stronger-coupled 24-contact,
  72-row and 32-contact, 96-row friction-index active-set packets over all 16
  friction-index-capable manifest solvers; the focused default, SIMD-enabled,
  and CUDA-enabled runs pass with `contract_ok=1` on all rows and report
  `active_set_transition=1`, `production_active_set_transition=1`,
  `contact_count=24/32`, `problem_size=72/96`, `coupling_scale=2/4`, and
  `coupled=1`. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
  `BM_LCP_COMPARE` also lists 64
  `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over those same
  stronger-coupled production active-set packets. Focused default,
  SIMD-enabled, and CUDA-enabled JSON checks report 64 rows with
  `contract_ok=1`; the SIMD run reports `build_simd_enabled=1`, and the
  CUDA-enabled run reports `build_cuda_enabled=1`. These are CPU solver batch
  rows in a CUDA-enabled build, not CUDA LCP kernel execution.
  `BM_LCP_COMPARE` also lists 27 `BM_LcpStressSingularDegenerate` rows for
  exact rank-deficient standard 64-row, boxed 64-row, and coupled
  friction-index 12-contact packets over the generated stress
  singular-degenerate solver scope; the focused default, SIMD-enabled, and
  CUDA-enabled runs pass with `contract_ok=1` on all rows and report
  `singular_degenerate=1`, `rank_deficient=1`, backend build-state counters,
  and `contact_count=12`/`coupled=1` for the coupled friction-index packet. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
  `BM_LCP_COMPARE` also lists 27 `BM_LcpExtremeSingularDegenerate` rows for
  exact rank-deficient standard 128-row, boxed 128-row, and coupled
  friction-index 16-contact packets over the generated extreme
  singular-degenerate solver scope; the focused default run passes with
  `contract_ok=1` on all rows, and the focused SIMD-enabled and CUDA-enabled
  runs also pass with `contract_ok=1` on all rows. The rows report
  `singular_degenerate=1`,
  `rank_deficient=1`, backend build-state counters, and
  `contact_count=16`/`coupled=1` for the coupled friction-index packet. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
  `BM_LCP_COMPARE` also lists 170 `BM_LcpMildIllConditioned` rows for standard
  32-row, boxed 16-row, friction-index 8-contact, and coupled friction-index
  6-, 8-, 12-, 16-, and 24-contact packets, plus stronger-coupled 16-/24-contact
  packets with 4x and 8x cross-contact coupling, over the scoped solver set; the
  focused default, SIMD-enabled, and CUDA-enabled runs pass with
  `contract_ok=1` on all rows and report `mildly_ill_conditioned=1` plus
  contact/coupling counters where applicable.
  `BM_LCP_COMPARE` also lists 56
  `BM_LcpMildIllConditionedBatch(Serial|Parallel)` rows for batch-size-4 serial
  and DART 7 `ParallelExecutor` runs over the 8x-coupled 16-/24-contact mildly
  ill-conditioned packets; focused default, SIMD-enabled, and CUDA-enabled JSON
  checks pass with `contract_ok=1` on all rows and report
  `mildly_ill_conditioned_batch=1`, `coupling_scale=8`, contact/total-contact
  counters, backend build-state counters, and parallel execution counters on
  `ParallelExecutor` rows.
  `BM_LCP_COMPARE` also lists 13 `BM_LcpNearSingular` rows for standard
  8-row, boxed 8-row, and coupled friction-index 3-, 6-, 9-, and 12-contact
  packets over the scoped robust solver set; the focused default,
  SIMD-enabled, and CUDA-enabled runs pass with `contract_ok=1` on all rows and
  report `near_singular=1`, backend build-state counters, and contact/coupling
  counters where applicable. The CUDA-enabled rows are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
  `BM_LCP_COMPARE` also lists 27 `BM_LcpSingularDegenerate` rows for exact
  rank-deficient standard 16-row, boxed 16-row, and coupled friction-index
  6-contact packets over the generated singular-degenerate solver scope; the
  focused default, SIMD-enabled, and CUDA-enabled runs pass with
  `contract_ok=1` on all rows and report `singular_degenerate=1`,
  `rank_deficient=1`, backend build-state counters, and contact/coupling
  counters where applicable.
  `BM_LCP_COMPARE` also lists 27 `BM_LcpLargerSingularDegenerate` rows for
  exact rank-deficient standard 32-row, boxed 32-row, and coupled
  friction-index 8-contact packets over the generated larger
  singular-degenerate solver scope; the focused default, SIMD-enabled, and
  CUDA-enabled runs pass with `contract_ok=1` on all rows and report
  `singular_degenerate=1`, `rank_deficient=1`, backend build-state counters,
  and `contact_count=8`/`coupled=1` for the coupled friction-index packet. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
- Standard, boxed, and friction-index benchmark registration now comes from
  `tests/common/lcpsolver/lcp_solver_manifest.hpp`; each family/size uses one
  deterministic generated problem across supported solvers. `DirectSolver` is
  intentionally registered only for dimensions 2 and 3 because larger standard
  problems fall back to Dantzig in the implementation.
- `BM_LcpBatchSerial/<family>/<solver>` now registers one deterministic
  independent-problem batch per supported solver/family: 24 standard, 16 boxed,
  and 16 friction-index entries. This is serial batch evidence only; do not
  count it as SIMD, threaded, or CUDA batch execution.
- `BM_LcpBatchParallel/<family>/<solver>` now registers the same deterministic
  independent-problem batches through the DART 7 experimental
  `ParallelExecutor`: 24 standard, 16 boxed, and 16 friction-index entries. This
  is task-parallel batch evidence for independent LCP problems; do not count it
  as SIMD, CUDA, or solver-internal parallelism.
- `BM_LCP_COMPARE` rows now report `build_simd_enabled`,
  `build_simd_force_scalar`, `build_cuda_enabled`, and
  `has_simulation_experimental`. The default build slice reported
  `build_simd_enabled=0`, `build_cuda_enabled=0`, and `contract_ok=1`; a
  separate `build/simd/cpp/Release` build configured with `DART_ENABLE_SIMD=ON`
  now passes 21 `LcpGeneratedCoverage.*` tests and reports
  `build_simd_enabled=1`, `build_cuda_enabled=0`, and `contract_ok=1` on all
  170 focused `BM_LcpMildIllConditioned` rows, all 49 focused
  `BM_LcpStressActiveSetTransition` rows, all 27 focused
  `BM_LcpSingularDegenerate` rows, and all 27 focused
  `BM_LcpLargerSingularDegenerate` rows, and all 27 focused
  `BM_LcpStressSingularDegenerate` rows, plus all 49 focused
  `BM_LcpExtremeActiveSetTransition` rows, all 16 focused
  `BM_LcpProductionActiveSetTransition` rows, and all 27 focused
  `BM_LcpExtremeSingularDegenerate` rows.
  A separate `build/cuda/cpp/Release` build configured by
  `pixi run -e cuda build-cuda Release` reported DART 7.0.0 with
  `DART_ENABLE_EXPERIMENTAL_CUDA=ON` and `DART_CUDA_ARCHITECTURES=89`; the
  documented `pixi run -e cuda test-cuda Release` smoke passed, and 21-test LCP
  generated coverage plus the focused `BM_LcpMildIllConditioned` slice passed
  with `build_cuda_enabled=1`, `build_simd_enabled=0`, and `contract_ok=1` on
  all 170 rows. The focused `BM_LcpStressActiveSetTransition` slice also passes
  with `build_cuda_enabled=1`, `build_simd_enabled=0`, and `contract_ok=1` on
  all 49 rows; those rows are CPU solver rows in a CUDA-enabled build, not CUDA
  LCP kernel execution. The focused `BM_LcpSingularDegenerate` slice also
  passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`, and `contract_ok=1` on all 27
  rows. The focused `BM_LcpLargerSingularDegenerate` slice also passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all 27 rows. The focused
  `BM_LcpStressSingularDegenerate` slice also passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all 27 rows. The focused
  `BM_LcpExtremeActiveSetTransition` slice passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`, and `contract_ok=1` on all
  49 rows, the focused `BM_LcpProductionActiveSetTransition` slice passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `contact_count=24/32`,
  `problem_size=72/96`, `coupling_scale=2/4`, and `contract_ok=1` on all 32
  rows, the focused `BM_LcpProductionActiveSetTransitionBatch` slice passes
  with `build_cuda_enabled=1`, `build_simd_enabled=0`,
  `production_active_set_transition_batch=1`, and `contract_ok=1` on all 64
  rows, and the focused
  `BM_LcpExtremeSingularDegenerate` slice passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all 27 rows. The same CUDA tree now also builds
  `test_lcp_jacobi_batch_cuda`,
  `BM_LcpCudaJacobiBatch_*`, `BM_LcpCudaPgsBatch_*`, grouped synthetic CUDA
  batch rows, homogeneous world-contact CUDA batch rows, and grouped
  variable-size world-contact CUDA batch rows,
  proving narrow CUDA fixed-iteration projected-Jacobi and PGS batch paths for
  standard, boxed, friction-index, grouped variable-size synthetic
  standard/boxed/findex, 4-/8-/16-contact world-contact packets, and grouped
  1/2/4/8/16-contact separated sphere-ground packets. General CUDA execution for
  the full solver manifest remains a gap.
- `docs/dev_tasks/lcp_solvers/01-implementation-audit.md` maps the background
  docs, current exports, manifest support, correctness tests, benchmark
  registration, backend evidence, and remaining DART 7 gaps solver by solver.
  `ApgdSolver`, `TgsSolver`, `AdmmSolver`, `SapSolver`, and
  `BoxedSemiSmoothNewtonSolver` are now described in `docs/background/lcp/` and
  remain tied back to the shared manifest.
- The manifest intentionally uses solver-reported names for drift checks:
  `BGS`, `NNCG`, and `MPRGP` are uppercase because `getName()` reports them
  that way.
- `tests/unit/math/lcp/test_lcp_generated_coverage.cpp` is the current
  manifest-driven generated correctness grid. It covers standard sizes
  1/2/4/8/16 plus mildly ill-conditioned 4D and 8D standard cases, boxed sizes
  2/4/8/12 plus mildly ill-conditioned 4D and 8D boxed cases and a
  near-singular 4D boxed case with lower, upper, interior, and unbounded-upper
  variables, well-conditioned friction-index contact counts 1/2/4/8 with mixed
  cone activity, coupled well-conditioned 2-contact, 4-contact, and 6-contact
  friction-index cases, coupled mildly ill-conditioned 2-contact and 4-contact
  friction-index cases, batch-shaped solver reuse across families,
  pivoting-solver near-singular standard cases, and invalid
  dimension/bounds/findex/NaN rejection across the full manifest. Separate
  larger generated tests cover standard 32-row and 64-row, boxed 32-row,
  friction-index 16-contact, coupled friction-index 8-contact,
  production-scale standard 128-row, boxed 64-row, friction-index 24-contact,
  coupled friction-index 12-contact, larger mildly ill-conditioned standard
  32/64-row, boxed 16/32-row, friction-index 8-contact, coupled friction-index
  6-, 8-, 12-, 16-, and 24-contact, stronger-coupled mildly ill-conditioned
  16-/24-contact, near-singular standard 8-row, boxed 8-row,
  coupled friction-index 3-, 6-, 9-, and 12-contact, and exact rank-deficient
  singular-degenerate standard 16-row, boxed 16-row, and coupled friction-index
  6-contact
  known-solution cases, plus
  larger exact rank-deficient singular-degenerate standard 32-row, boxed
  32-row, and coupled friction-index 8-contact known-solution cases, plus
  stress exact rank-deficient singular-degenerate standard 64-row, boxed
  64-row, and coupled friction-index 12-contact known-solution cases, plus
  extreme exact rank-deficient singular-degenerate standard 128-row, boxed
  128-row, and coupled friction-index 16-contact known-solution cases, plus
  stress and extreme active-set transition standard 64/128-row, boxed
  64/128-row, coupled friction-index 12/16-contact, and production
  stronger-coupled friction-index 24/32-contact known-solution cases for
  scoped solver sets. Trial
  evidence keeps the near-singular test narrow: Lemke
  produced a valid complementary solution but not the selected generated
  solution for the 8-row singular standard case, and boxed semi-smooth Newton
  failed line search on the near-singular coupled friction-index cases.
  Trial evidence also keeps the singular-degenerate boxed/findex slice scoped:
  `Dantzig`, `ShockPropagation`, and projection-style boxed/findex paths did
  not satisfy the selected exact rank-deficient boxed/findex contracts.
- `JacobiSolver::Parameters::workerThreads` enables an opt-in solver-internal
  CPU threaded update path. `LcpGeneratedCoverage.ThreadedJacobiStandardKnownSolution`
  passes on a 128-row generated standard LCP. `BM_LCP_COMPARE` has
  `BM_LcpJacobiSolverThreading_Standard` rows for 128-row and 512-row standard
  problems with 1 and 8 worker threads. Default, SIMD, and CUDA-enabled focused
  runs report `contract_ok=1`, backend build-state counters,
  `solver_internal_threads`, and `worker_count`; the focused timings do not show
  a speedup for these dense cases, so treat this as correctness/comparison
  evidence rather than a performance recommendation.
- `BoxedSemiSmoothNewton` now includes the derivative of moving `findex`
  friction bounds in its natural-residual Jacobian. The previously failing
  coupled mildly ill-conditioned 4-contact friction-index generated case passes
  the focused generated coverage test without allowing `MaxIterations` for the
  boxed semi-smooth Newton path.
- `tests/unit/simulation/experimental/contact/test_boxed_lcp_contact.cpp` now
  validates two real DART 7 boxed-LCP contact snapshots:
  `BoxedLcpContact.WorldContactSnapshotSatisfiesLcpContract` assembles contacts
  from `World::collide()`, calls `detail::solveBoxedLcpContacts`, checks the
  boxed/findex LCP with `tests/common/lcpsolver/lcp_test_harness.hpp`, and
  verifies the impulse cancels downward approach while reducing slip.
  `BoxedLcpContact.TwoSphereWorldContactSnapshotSatisfiesLcpContract` uses the
  same contract for two separated sphere-ground contacts assembled into one LCP
  snapshot. `BoxedLcpContact.BlockSolversUseFindexContactBlocksOnWorldSnapshot`
  proves BGS and Blocked Jacobi solve that real two-contact snapshot through
  `findex`-derived non-contiguous contact blocks, and
  `BoxedLcpContact.BlockSolversRejectExplicitWorldContactFindexSplit` proves
  explicit block sizes that split normal/tangent dependencies fail with
  `InvalidProblem`. `BoxedLcpContact.TwoSphereWorldStepMaintainsContactInvariants`
  advances two independent sphere-ground contacts through `World::step(200)`
  using the public boxed-LCP solver and checks finite state, non-penetration,
  near-rest normal velocity, tangential-speed reduction, and static-ground
  invariants. `BoxedLcpContact.FourSphereWorldStepMaintainsContactInvariants`
  advances four independent sphere-ground contacts through `World::step(200)`
  and checks finite state, contact height, bounded vertical velocity,
  tangential-speed reduction, and static-ground invariants.
  `BoxedLcpContact.SixteenSphereWorldStepMaintainsContactInvariants` advances
  16 independent sphere-ground contacts through `World::step(200)` and checks
  the same denser separated-contact invariants. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.BlockSolversUseFindexContactBlocksOnWorldSnapshot:BoxedLcpContact.BlockSolversRejectExplicitWorldContactFindexSplit'`
  run passes both per-contact block tests, the focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorld(Contact|StackContact|ContactBatchSerial|ContactBatchParallel|ContactStressBatchSerial|ContactStressBatchParallel)/FrictionIndex/(BGS|BlockedJacobi)'`
  JSON gate reports 22 rows, `bad_contract=0`, `bad_problem=0`,
  `stack_rows=8`, and `batch_rows=8`, the focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*WorldContactSnapshot*'`
  run passes both snapshot tests, the focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.TwoSphereWorldStepMaintainsContactInvariants'`
  and
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.FourSphereWorldStepMaintainsContactInvariants:BoxedLcpContact.SixteenSphereWorldStepMaintainsContactInvariants'`
  runs pass, and the full `test_boxed_lcp_contact --gtest_brief=1` binary
  passes 40 tests. The full run still emits the existing
  `StaticFrictionHoldsSmallPush` degenerate-pivot warning.
- `BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants`
  advances a fixed-base prismatic articulated link in light ground contact
  through `World::step(400)` with `ContactSolverMethod::BoxedLcp`, confirms
  the active contact touches a `comps::Link`, and checks finite link height,
  near-rest contact height, bounded joint velocity, and parity with the
  sequential articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants' --gtest_brief=1`
  run passes. Treat this as fixed-base prismatic link-ground evidence, not
  broad articulated robot contact coverage.
- `BoxedLcpContact.FourArticulatedPrismaticLinksGroundStepMaintainsInvariants`
  advances four independent fixed-base prismatic articulated links in
  simultaneous ground contact through `World::step(200)` with
  `ContactSolverMethod::BoxedLcp`, confirms all four contacts touch links, and
  checks finite state, bounded height error, bounded joint velocity, and parity
  with the sequential articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants:BoxedLcpContact.FourArticulatedPrismaticLinksGroundStepMaintainsInvariants' --gtest_brief=1`
  run passes both articulated link-ground tests. Treat this as a broader
  fixed-base prismatic link-ground scene, not broad articulated robot contact
  coverage.
- `BoxedLcpContact.CartesianPrismaticChainGroundStepMaintainsInvariants`
  advances two connected fixed-base three-axis prismatic Cartesian chains in
  simultaneous tip-ground contact through `World::step(200)` with
  `ContactSolverMethod::BoxedLcp`, confirms both contacts touch links, and
  checks finite state, six total generalized coordinates, bounded tip height
  error, bounded joint velocities, bounded planar joint speed, and parity with
  the sequential articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.CartesianPrismaticChainGroundStepMaintainsInvariants' --gtest_brief=1`
  run passes. Treat this as connected multi-DOF fixed-base articulated contact
  evidence, not broad articulated robot contact coverage.
- `BoxedLcpContact.ArticulatedPrismaticLinkPushesDynamicRigidBody` advances a
  fixed-base prismatic articulated striker link in contact with a dynamic rigid
  sphere through one boxed-LCP `World::step()`, confirms the contact touches
  both a `comps::Link` and a rigid body, and checks finite velocities, target
  motion, striker slowdown, X-momentum conservation, and parity with the
  sequential articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesDynamicRigidBody' --gtest_brief=1`
  run passes. Treat this as fixed-base prismatic link-vs-rigid evidence, not
  broad articulated robot contact coverage.
- `BoxedLcpContact.ArticulatedPrismaticLinkPushesArticulatedPrismaticLink`
  advances a fixed-base prismatic articulated striker link in contact with a
  prismatic target link owned by a separate multibody through one boxed-LCP
  `World::step()`, confirms the contact touches two `comps::Link` entities, and
  checks finite velocities, target motion, striker slowdown, nonnegative
  post-step separation velocity, X-momentum conservation, and parity with the
  sequential cross-multibody articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesArticulatedPrismaticLink' --gtest_brief=1`
  run passes. Treat this as fixed-base cross-multibody link-vs-link evidence,
  not broad articulated robot contact coverage.
- `BoxedLcpContact.SphereStackWorldContactSnapshotSatisfiesLcpContract`,
  `BoxedLcpContact.LargerSphereStackWorldContactSnapshotSatisfiesLcpContract`,
  and
  `BoxedLcpContact.StressSphereStackWorldContactSnapshotSatisfiesLcpContract`
  validate 3-sphere, 4-sphere, and 5-sphere vertical stacks assembled from
  `World::collide()` and check that the normal-contact block has nonzero
  off-diagonal coupling. The 4-sphere snapshot has 4 contacts and 12 LCP rows;
  the 5-sphere snapshot has 5 contacts and 15 LCP rows.
  The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*SphereStack*'` run
  passes these snapshot tests plus the 3-sphere 200-step, 3-sphere 500-step,
  and 4-sphere 200-step invariant tests. The full
  `test_boxed_lcp_contact --gtest_brief=1` binary now passes 40 tests.
- `BoxedLcpContact.SphereStackWorldStepMaintainsContactInvariants` advances the
  same 3-sphere vertical stack through 200 public boxed-LCP `World::step()`
  iterations and checks finite state, non-penetration, preserved sphere spacing,
  near-rest vertical velocity, bounded lateral drift, and static-ground
  invariants. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.SphereStackWorldStepMaintainsContactInvariants'`
  run passes.
- `BoxedLcpContact.LongRunningSphereStackWorldStepMaintainsContactInvariants`
  advances the same 3-sphere vertical stack through 500 public boxed-LCP
  `World::step()` iterations with the same finite-state, non-penetration,
  spacing, vertical-rest, lateral-drift, and static-ground invariants. The
  focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.LongRunningSphereStackWorldStepMaintainsContactInvariants'`
  run passes.
- A resting box multi-contact friction snapshot probe did not satisfy the LCP
  contract cleanly; keep dense-degenerate multi-contact boxed-LCP snapshots as
  a separate evidence slice rather than claiming them from the sphere-ground
  tests.
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers 48
  `BM_LcpWorldContact/FrictionIndex/<solver>/{1,2,4}` rows: all 16
  friction-index-capable manifest solvers over the same DART 7 `World::collide()`
  sphere-ground boxed/findex LCP snapshots. It also registers
  `BM_LcpWorldContactAssembly_BoxedLcp/{1,2,4}`, which rebuilds the world,
  collides, assembles through `detail::solveBoxedLcpContacts`, solves, and
  validates the contact snapshot. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContact|BM_LcpWorldContactAssembly' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  run passed with `contract_ok=1` for all 51 rows. `ShockPropagation` uses
  default `findex`-based grouping on these non-contiguous contact snapshots and
  reports one 3-row block per contact. Treat this as simple separated-contact
  benchmark evidence, not evidence for articulated or dense-degenerate contact
  scenes.
- The benchmark target also registers 32
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{2,3}` rows for all 16
  friction-index-capable solvers over DART 7 `World::collide()` snapshots from
  2- and 3-sphere vertical stacks. It also registers 30
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{4,5}` rows for the same
  solver set except `NNCG`; a focused `NNCG` 4-sphere trial reached the
  benchmark cap with `contract_ok=0`, so the NNCG 4-/5-sphere rows are not
  claimed. The matching
  `BM_LcpWorldStackContactAssembly_BoxedLcp/{2,3,4,5}` rows rebuild the world,
  collide, assemble through `detail::solveBoxedLcpContacts`, solve, and
  validate the coupled stack snapshot. The focused 4-sphere benchmark run
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/4|BM_LcpWorldStackContactAssembly_BoxedLcp/4' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed with `contract_ok=1` for all 16 registered rows. Focused default,
  SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/5$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs also pass with `contract_ok=1` for all 15 registered 5-sphere solver
  rows, with `sphere_count=5`, `contact_count=5`, and `problem_size=15`; the
  CUDA-enabled rows are CPU solver rows in that build tree, not CUDA LCP kernel
  execution. Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LcpWorldStackContactAssembly_BoxedLcp/5` runs also pass with
  `contract_ok=1`, `sphere_count=5`, `contact_count=5`, and `problem_size=15`.
  Treat this as small coupled-stack benchmark evidence, not evidence for
  articulated or dense-degenerate contact scenes.
- The benchmark target now also registers 32
  `BM_LcpWorldContactBatch(Serial|Parallel)/FrictionIndex/<solver>` rows: all
  16 friction-index-capable manifest solvers over one mixed 5-problem DART 7
  contact batch made from the 1/2/4 separated sphere-ground snapshots and the
  2/3-sphere vertical-stack snapshots. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContactBatch(Serial|Parallel)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  run passed with `contract_ok=1` for all 32 rows. Each row reports
  `batch_size=5`, `total_problem_size=36`, `total_contact_count=12`, and
  `total_body_count=12`; parallel rows also report `profile_enabled=1`,
  `worker_count=20`, `parallel_units=5`, and observed `max_parallelism` up to 5. Treat this as serial and DART 7 `ParallelExecutor` batch evidence over
  contact-derived snapshots, not CUDA batch execution.
- The benchmark target now also registers 64
  `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows: all 16
  friction-index-capable manifest solvers over batch-size-4 serial and DART 7
  `ParallelExecutor` runs for the stronger-coupled 24-contact/72-row and
  32-contact/96-row production active-set packets. Focused JSON benchmark
  checks in default, SIMD-enabled, and CUDA-enabled build trees report 64 rows
  with `contract_ok=1`. Each row reports
  `production_active_set_transition_batch=1`, `batch_size=4`,
  `contact_count=24/32`, `total_contact_count=96/128`,
  `problem_size=72/96`, `total_problem_size=288/384`, and
  `coupling_scale=2/4`; parallel rows also report `profile_enabled=1`,
  `worker_count=20`, `parallel_units=4`, and observed `max_parallelism`. Treat
  this as serial and DART 7 `ParallelExecutor` batch evidence over synthetic
  production active-set packets, not CUDA kernel execution.
- The benchmark target now also registers 30
  `BM_LcpWorldContactStressBatch(Serial|Parallel)/FrictionIndex/<solver>` rows
  for all friction-index-capable manifest solvers except `NNCG`. The stress
  batch uses 1/2/4 separated sphere-ground snapshots plus 2/3/4/5-sphere
  vertical-stack snapshots. Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContactStressBatch(Serial|Parallel)/FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs pass with `contract_ok=1` for all 30 rows. Each row reports
  `batch_size=7`, `total_problem_size=63`, `total_contact_count=21`,
  `total_body_count=21`, `separated_contact_shape_count=3`,
  `stack_contact_shape_count=4`, and `stress_stack_contact_batch=1`; parallel
  rows also report `profile_enabled=1`, `worker_count=20`, and
  `parallel_units=7`. The CUDA-enabled rows are CPU solver rows in that build
  tree, not CUDA LCP kernel execution.
- `DenseBoxWorldContactSnapshotSatisfiesLcpContract` assembles a 4-contact,
  12-row DART 7 box-face ground patch from `World::collide()`, checks the
  boxed/findex shape, and verifies the same dense single-body LCP with APGD.
  The sliding/static-friction box end-to-end tests now assert at least four
  contacts before stepping. `FourBoxWorldStepMaintainsDenseContactInvariants`
  extends the unit coverage to a separated 4-box, 16-contact dense face-contact
  scene advanced for 200 public boxed-LCP `World::step()` iterations. The full
  `test_boxed_lcp_contact --gtest_brief=1` run passes 40 tests while still
  emitting the dense-patch Dantzig warning.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldBoxContact/FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passes in default, SIMD-enabled, and CUDA-enabled build trees for 18 scoped
  dense box rows: `Pgs`, `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and
  `Admm` on 1/2/4-box snapshots. Each row reports `contract_ok=1`,
  `dense_box_contact=1`, `box_count=1`, `2`, and `4`, `contact_count=4`, `8`,
  and `16`, and `problem_size=12`, `24`, and `48`; CUDA-enabled rows are CPU
  solver rows in that build tree, not CUDA LCP kernel execution.
- `BM_LcpWorldBoxStep_BoxedLcp/{1,2,4}/200` rebuilds separated dense box-face
  worlds, confirms 4/8/16 contacts before stepping, enters simulation mode,
  advances 200 public boxed-LCP `World::step()` iterations, and reports
  end-to-end invariant counters. Focused default, SIMD-enabled, and
  CUDA-enabled build-tree
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldBoxStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `invariant_ok=1`, `dense_box_contact=1`, and
  `box_count=1`, `2`, and `4`; the CUDA-enabled rows are CPU public-step rows
  in that build tree, not CUDA LCP kernel execution. The runs still emit the
  dense-patch Dantzig warning, so treat this as public-step invariant evidence,
  not a direct Dantzig dense box solve claim.
- `BM_LcpWorldStackStep_BoxedLcp/3/200`,
  `BM_LcpWorldStackStep_BoxedLcp/3/500`, and
  `BM_LcpWorldStackStep_BoxedLcp/4/200` rebuild coupled stack worlds, enter
  simulation mode, advance public boxed-LCP `World::step()` iterations, and
  report end-to-end invariant counters. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in default, SIMD-enabled, and CUDA-enabled build trees reported
  `invariant_ok=1` for the existing 3-sphere rows and the new
  `BM_LcpWorldStackStep_BoxedLcp/4/200` row. The SIMD run reported
  `build_simd_enabled=1`, and the CUDA-enabled run reported
  `build_cuda_enabled=1`. The default 3-sphere rows reported
  `time_step=0.005`, `min_spacing=0.9999`, and
  `max_vertical_speed=4.31e-14`; the default 4-sphere row reported
  `time_step=0.001`, `min_spacing=0.9999`, and
  `max_vertical_speed=1.72e-8`.
- `BM_LcpWorldSeparatedStep_BoxedLcp/4/200` and
  `BM_LcpWorldSeparatedStep_BoxedLcp/8/200` rebuild separated sphere-ground
  worlds, enter simulation mode, advance 200 public boxed-LCP `World::step()`
  iterations, and report end-to-end invariant counters. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldSeparatedStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  run reported `invariant_ok=1` for the 4-, 8-, and 16-contact rows; the
  16-contact row reported `max_height_error=0`,
  `max_vertical_speed=0`, and `min_tangential_speed_drop=0.23816`.
- `BM_LcpWorldArticulatedGroundStep_BoxedLcp/{1,4,8,16}/200` rebuild fixed-base
  prismatic-link worlds, enters simulation mode inside the world factory so
  link kinematics are current before `World::collide()`, advances 200 public
  boxed-LCP `World::step()` iterations, and reports end-to-end invariant
  counters. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldArticulated(Ground|RigidImpact)Step_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  default, SIMD-enabled, and CUDA-enabled build-tree runs reported
  `invariant_ok=1` for all eight articulated rows, including ground rows with
  `articulated_link_count=1`, `4`, `8`, and `16`.
- `BM_LcpWorldArticulatedRigidImpactStep_BoxedLcp/{1,4,8,16}/1` rebuild
  fixed-base prismatic-link striker worlds with dynamic rigid targets, enters
  simulation mode inside the world factory so link kinematics are current
  before `World::collide()`, advances one public boxed-LCP `World::step()`,
  and reports end-to-end invariant counters. The same default, SIMD-enabled,
  and CUDA-enabled runs reported `invariant_ok=1` for all four rigid-impact
  rows with `articulated_link_count=1`, `4`, `8`, and `16`,
  `dynamic_rigid_body_count=1`, `4`, `8`, and `16`,
  `max_momentum_error=0`, `max_striker_velocity=0.606667`, and
  `min_target_velocity=0.786667`.
- `BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp/{1,4,8,16}/1` rebuild
  cross-multibody fixed-base prismatic striker/target link worlds, enters
  simulation mode inside the world factory so link kinematics are current
  before `World::collide()`, advances one public boxed-LCP `World::step()`,
  and reports end-to-end invariant counters. The focused default,
  SIMD-enabled, and CUDA-enabled build-tree runs reported `invariant_ok=1` for
  all four link-impact rows with `articulated_pair_count=1`, `4`, `8`, and
  `16`, `articulated_dof_count=2`, `8`, `16`, and `32`,
  `cross_multibody_link_contact=1`, `max_momentum_error=0`,
  `max_striker_velocity=0.606667`, `min_target_velocity=0.786667`, and
  `min_relative_velocity=0.18`.
- `BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp/{1,4,8,16}/200`
  rebuild connected fixed-base three-axis prismatic Cartesian-chain worlds,
  enters simulation mode inside the world factory so link kinematics are
  current before `World::collide()`, advances 200 public boxed-LCP
  `World::step()` iterations, and reports end-to-end invariant counters. The
  focused default, SIMD-enabled, and CUDA-enabled build-tree runs reported
  `invariant_ok=1` for all four rows with `cartesian_chain_count=1`, `4`, `8`,
  and `16`, `articulated_dof_count=3`, `12`, `24`, and `48`, and
  `serial_prismatic_chain=1`.
- `BM_LcpArticulatedUnifiedContact/FrictionIndex/{Ground,RigidImpact,CrossLinkImpact}/<solver>/{1,4}`
  manually assemble fixed-base three-axis prismatic `LinkContact` snapshots
  through `assembleMultibodyLinkContactProblem` and
  `assembleUnifiedConstraintProblem`, then compare all 16
  friction-index-capable solvers on identical 3-row and 12-row LCPs. The
  cross-link rows complete a second articulated endpoint for a separate
  multibody, so they exercise the unified contact matrix's cross-multibody
  block. The
  focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpArticulatedUnifiedContact' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  run reported `contract_ok=1` for all 96 rows, with
  `articulated_unified_contact=1`, `contact_count=1` and `4`, and
  `dynamic_rigid_body_count=1` and `4` for the rigid-impact rows. A focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpArticulatedUnifiedContact/FrictionIndex/CrossLinkImpact' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  default/SIMD/CUDA build-tree run reported `contract_ok=1` for all 32
  cross-link rows, with `multibody_count=2` and `8`,
  `articulated_cross_link_contact=1`, and the expected backend build counters.
  This is manual articulated unified-contact LCP assembly evidence, not
  collision-discovered or end-to-end stepping evidence. Focused SIMD-enabled
  and CUDA-enabled build-tree runs over the Dantzig/Jacobi/SAP ground and
  rigid-impact rows also reported `contract_ok=1` with `build_simd_enabled=1`
  and `build_cuda_enabled=1`, respectively; those are CPU solver rows in those
  build trees, not CUDA kernel execution.
- CUDA was enabled and validated through the documented DART 7
  `simulation-experimental-cuda` smoke path on a visible RTX 4080 Laptop GPU,
  and the LCP generated coverage plus selected LCP benchmark rows pass in that
  CUDA-enabled build. `test_lcp_jacobi_batch_cuda` passes 31 tests, including
  standard, boxed, friction-index, grouped variable-size synthetic
  standard/boxed/friction-index, contact-derived world-contact, and homogeneous
  5-sphere coupled stack-contact batches, plus grouped manually assembled
  1-/4-contact articulated unified-contact batches, plus mixed grouped contact
  batches executed on CUDA for Jacobi and PGS.
  `BM_LcpCudaJacobiBatch_*`, `BM_LcpCudaPgsBatch_*`,
  `BM_LcpCudaJacobiGroupedBatch_*`, `BM_LcpCudaPgsGroupedBatch_*`,
  `BM_LcpCudaJacobiWorldContactBatch_FrictionIndex`,
  `BM_LcpCudaPgsWorldContactBatch_FrictionIndex`,
  `BM_LcpCudaJacobiWorldStackContactBatch_FrictionIndex`,
  `BM_LcpCudaPgsWorldStackContactBatch_FrictionIndex`,
  `BM_LcpCudaJacobiWorldContactGroupedBatch_FrictionIndex`,
  `BM_LcpCudaPgsWorldContactGroupedBatch_FrictionIndex`,
  `BM_LcpCudaJacobiWorldStackContactGroupedBatch_FrictionIndex`, and
  `BM_LcpCudaPgsWorldStackContactGroupedBatch_FrictionIndex`,
  `BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch_FrictionIndex`, and
  `BM_LcpCudaPgsArticulatedUnifiedContactGroupedBatch_FrictionIndex`,
  `BM_LcpCudaJacobiMixedContactGroupedBatch_FrictionIndex`, and
  `BM_LcpCudaPgsMixedContactGroupedBatch_FrictionIndex` rows report
  `build_cuda_enabled=1`, `cuda_lcp_execution=1`, `cuda_batch_execution=1`,
  fixed iteration counters, and `contract_ok=1`. The grouped synthetic rows use
  three groups and two variants per group (`batch_size=6`,
  `cuda_group_count=3`). The standard and boxed grouped rows cover
  16/32/48-row packets with `total_problem_size=192`; the friction-index
  grouped rows cover 4/8/16-contact packets with `min_problem_size=12`,
  `max_problem_size=48`, `total_contact_count=56`, and
  `total_problem_size=168`. The homogeneous world-contact CUDA rows are
  4-, 8-, and 16-contact packets; the denser rows report
  `batch_size=4`, `contact_count=16`, `problem_size=48`,
  `total_contact_count=64`, and `total_problem_size=192`. The homogeneous stack
  CUDA rows are 5-sphere coupled stack packets (`batch_size=4`,
  `contact_count=5`, `problem_size=15`, `total_contact_count=20`, and
  `total_problem_size=60`). The separated grouped variable-size rows are
  sphere-ground packets with 1/2/4/8/16 contacts
  (`batch_size=10`, `cuda_group_count=5`, `contact_shape_count=5`,
  `min_problem_size=3`, `max_problem_size=48`, `total_contact_count=62`,
  `total_problem_size=186`).
  The coupled stack grouped variable-size rows are 2/3/4/5-sphere stack packets
  (`batch_size=8`, `cuda_group_count=4`, `contact_shape_count=4`,
  `min_problem_size=6`, `max_problem_size=15`, `total_contact_count=28`,
  `total_problem_size=84`).
  The articulated unified-contact grouped variable-size rows are manually
  assembled fixed-base three-axis prismatic link-ground,
  link-vs-dynamic-rigid, and cross-multibody link-vs-link packets with 1 and 4
  contacts (`batch_size=12`, `cuda_group_count=2`, `contact_shape_count=2`,
  `articulated_contact_case_count=3`, `articulated_cross_link_contact=1`,
  `min_problem_size=3`, `max_problem_size=12`, `total_contact_count=30`,
  `total_problem_size=90`).
  The mixed grouped rows combine separated sphere-ground, coupled stack, and
  manually assembled articulated unified-contact packets including
  cross-multibody link-vs-link packets (`batch_size=20`,
  `contact_fixture_family_count=3`, `cuda_group_count=4`,
  `contact_shape_count=4`, `articulated_contact_case_count=3`,
  `articulated_cross_link_contact=1`, `min_problem_size=3`,
  `max_problem_size=15`, `total_contact_count=54`,
  `total_problem_size=162`).
  The dense box-face CUDA rows are PGS-only:
  `CudaLcpPgsBatch.DenseBoxWorldContactBatchSatisfiesLcpContract` and
  `CudaLcpPgsBatch.DenseBoxWorldContactGroupedBatchSatisfiesLcpContract` pass
  on homogeneous and grouped variable-size dense box-face `World::collide()`
  snapshots. `BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex/4` reports
  `cuda_dense_box_contact_batch=1`, `batch_size=4`,
  `total_contact_count=16`, `total_body_count=4`, and
  `total_problem_size=48`. `BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex/2`
  reports `cuda_group_count=3`, `box_count_shape_count=3`,
  `min_problem_size=12`, `max_problem_size=48`, `total_contact_count=56`,
  `total_body_count=14`, and `total_problem_size=168`. A fixed-iteration CUDA
  Jacobi dense-box trial
  failed the LCP contract, so Jacobi dense-contact CUDA execution remains
  unclaimed. The failed probe used the same homogeneous 4-problem and grouped
  1/2/4-box dense box-face fixtures: at 4096 iterations with relaxation 1.0 it
  failed with residual/complementarity/bound violations of about 3.3e-2 to
  5.0e-2 (`w must be non-negative at lo`), and at 8192 iterations with
  relaxation 0.25 it reached near-zero residual/bounds but failed
  fixed-variable complementarity at about 0.34 to 0.435 (`fixed variable
residual`).
  Treat this as narrow CUDA LCP batch evidence only; general CUDA execution for
  the full solver manifest, CUDA Jacobi dense-contact batches, and end-to-end
  articulated world-step CUDA execution are still missing.
- `pixi run test-lcpsolver` currently builds the full test suite before the LCP
  filter. For focused iteration, use the exact CMake target once the build tree
  is configured.

## How to Resume

```bash
git status --short
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           BM_LCP_COMPARE
./build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke \
  "--gtest_filter=AllSolversSmokeTest.*:AdvancedBoxedSolvers.*"
./build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_comparison_harness \
  "--gtest_filter=LcpComparisonHarness.*"
./build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_generated_coverage \
  "--gtest_filter=LcpGeneratedCoverage.*"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LCP_COMPARE_SMOKE" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpActiveSetTransition" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpLargerActiveSetTransition" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpStressActiveSetTransition" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpExtremeActiveSetTransition" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpProductionActiveSetTransition" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpMildIllConditioned" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpSingularDegenerate" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpLargerSingularDegenerate" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpStressSingularDegenerate" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpExtremeSingularDegenerate" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCompare/(Standard/Dantzig|Boxed/ShockPropagation|FrictionIndex/NNCG)" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpBatch(Serial|Parallel)/(Standard/Dantzig|Boxed/BoxedSemiSmoothNewton|FrictionIndex/BGS)" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpWorldContact|BM_LcpWorldContactAssembly" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpWorldStackContact|BM_LcpWorldStackContactAssembly" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpWorldContactBatch(Serial|Parallel)" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpWorldContactStressBatch(Serial|Parallel)/FrictionIndex" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpWorldBoxContact/FrictionIndex" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpWorldBoxStep_BoxedLcp" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpArticulatedUnifiedContact" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpWorldStackStep_BoxedLcp" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/cuda/cpp/Release/bin/test_lcp_jacobi_batch_cuda \
  "--gtest_filter=CudaLcpJacobiBatch.StackedWorldContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.StackedWorldContactGroupedBatchSatisfiesLcpContract" \
  "--gtest_brief=1"
./build/cuda/cpp/Release/bin/test_lcp_jacobi_batch_cuda \
  "--gtest_filter=CudaLcpJacobiBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract" \
  "--gtest_brief=1"
./build/cuda/cpp/Release/bin/test_lcp_jacobi_batch_cuda \
  "--gtest_filter=CudaLcpJacobiBatch.MixedContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.MixedContactGroupedBatchSatisfiesLcpContract" \
  "--gtest_brief=1"
./build/cuda/cpp/Release/bin/test_lcp_jacobi_batch_cuda \
  "--gtest_filter=CudaLcpPgsBatch.DenseBoxWorldContactBatchSatisfiesLcpContract:CudaLcpPgsBatch.DenseBoxWorldContactGroupedBatchSatisfiesLcpContract" \
  "--gtest_brief=1"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)WorldStackContactGroupedBatch_FrictionIndex" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)ArticulatedUnifiedContactGroupedBatch_FrictionIndex" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)MixedContactGroupedBatch_FrictionIndex" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCudaPgsWorldBoxContact(Batch|GroupedBatch)_FrictionIndex" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
```

Then continue with harder generated-size coverage, broader DART 7 boxed-LCP
contact coverage, and backend-specific SIMD/CUDA/solver-internal threaded
benchmark evidence described in `README.md` and
`01-implementation-audit.md`.
