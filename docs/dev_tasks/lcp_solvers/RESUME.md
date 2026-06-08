# Resume: LCP Solvers

## Last Session Summary

The worktree is on local branch `feature/dart7-lcp-solver-evidence` with
checkpoint commits adding solver-agnostic correctness coverage for advanced LCP
solvers, a shared DART 7 LCP solver
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
5-/6-/7-/8-sphere coupled stack-contact LCP packets, with grouped articulated
unified-contact CUDA evidence later expanded to cross-multibody link-vs-link
packets in this task log.
They also fix boxed semi-smooth Newton's `findex` moving-bound Jacobian and add
the previously failing coupled mildly ill-conditioned 4-contact friction-index
case to the generated all-solver grid.
The work now also adds a scoped larger mildly ill-conditioned generated
known-solution slice for standard 32/64-row, boxed 16/32-row, friction-index
8-contact, 1x-/4x-/8x-coupled friction-index
6-/8-/12-/16-/24-/32-/48-/64-/96-contact packets, and 16x-coupled
6-/8-/12-/16-/24-/32-/48-/64-/96-/128-contact packets with Boxed
Semi-Smooth Newton included across those coupled rows, and 599 matching
`BM_LcpMildIllConditioned` benchmark rows verified in
default, SIMD-enabled, and CUDA-enabled build trees. It also
adds 1198 `BM_LcpMildIllConditionedBatch(Serial|Parallel)` rows for batch-size-4
serial and DART 7 `ParallelExecutor` runs over the full scoped mildly
ill-conditioned packet set: standard 32-row, boxed 16-row, friction-index
8-contact, coupled friction-index 6-/8-/12-/16-/24-/32-/48-/64-/96-contact,
4x-coupled 6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 8x-coupled
6-/8-/12-/16-/24-/32-/48-/64-/96-contact, and 15-solver 16x-coupled
6-/8-/12-/16-/24-/32-/48-/64-/96-/128-contact rows. Boxed Semi-Smooth Newton reports
tuned line-search settings on the 16x rows. The slice is verified in default,
SIMD-enabled, and CUDA-enabled build trees.
The robust near-singular generated and benchmark coverage now reaches coupled
friction-index 128-contact packets. It also adds 54
`BM_LcpNearSingularBatch(Serial|Parallel)` rows for batch-size-4 serial and
DART 7 `ParallelExecutor` runs over near-singular standard 8-row, boxed 8-row,
and coupled friction-index 3-/6-/9-/12-/16-/24-/32-/48-/64-/96-/128-contact
packets, verified in default, SIMD-enabled, and CUDA-enabled build trees.
It also adds 66
`BM_LcpSingularDegenerateFrictionIndexBatch(Serial|Parallel)` rows for
batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
rank-deficient coupled friction-index
6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact packets, verified in
default, SIMD-enabled, and CUDA-enabled build trees.
It also adds 192
`BM_LcpSingularDegenerateStandardBoxedBatch(Serial|Parallel)` rows for
batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
rank-deficient standard and boxed 16-/32-/64-/128-row packets, verified in
default, SIMD-enabled, and CUDA-enabled build trees.
It now also adds DART 7 per-contact block-structure evidence for BGS and
Blocked Jacobi on a real two-contact boxed-LCP world-contact snapshot: the
focused tests pass when blocks are derived from non-contiguous `findex`
dependencies and reject explicit block sizes that split tangent rows from their
owning normal rows. A focused `BM_LCP_COMPARE` gate on the merged default tree
also verifies 22 BGS/Blocked Jacobi world-contact, stack-contact, and
serial/parallel batch rows with `bad_contract=0`.
It now also adds opt-in projected gradient-descent warm starts for the native
standard-LCP paths of Minimum Map, Fischer-Burmeister, and Penalized
Fischer-Burmeister Newton. Focused unit tests prove each initializer reduces its
solver-specific merit before Newton line search, while boxed/findex problems
remain delegated for those three solvers.
It now also adds opt-in PGS warm starts for those same standard Newton paths,
accepting the PGS seed only when it reduces the solver-specific merit before
Newton line search.
It now also adds 24 `BM_LcpNewtonWarmStart` benchmark rows for the three native
standard-LCP Newton solvers, comparing no seed, PGS, projected gradient
descent, and PGS-then-gradient modes on identical 32-row and 64-row active-set
transition packets. Focused default, SIMD-enabled, and CUDA-enabled build-tree
runs passed with `contract_ok=1` on every row and recorded mode/iteration
counters in the JSON output. The CUDA-enabled rows are CPU solver rows in a
CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 48
`BM_LcpNewtonWarmStartBatch(Serial|Parallel)` benchmark rows for batch-size-4
serial and DART 7 `ParallelExecutor` runs over the same standard active-set
transition packets and the same no-seed, PGS-only, gradient-only, and
PGS-then-gradient mode matrix. Focused default, SIMD-enabled, and CUDA-enabled
build-tree runs passed with `contract_ok=1` on every row. The rows record
`newton_warm_start_batch=1`, serial/parallel execution counters,
`parallel_units=4`, `worker_count=20`, observed `max_parallelism` up to 4, and
backend build-state counters. The CUDA-enabled rows are CPU solver batch rows
in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 9 `BM_LcpPgsRelaxationSweep` benchmark rows for the
`PgsSolver`/PSOR relaxation path, covering standard 48-row, boxed 24-row, and
friction-index 8-contact fixtures at relaxation 0.5, 1.0, and 1.3. Focused
default, SIMD-enabled, and CUDA-enabled build-tree runs passed with
`contract_ok=1` on every row and recorded under/plain/over-relaxation counters.
The CUDA-enabled rows are CPU PGS solver rows in a CUDA-enabled build, not CUDA
LCP kernel execution.
It now also adds 9 `BM_LcpSymmetricPsorRelaxationSweep` benchmark rows for the
`SymmetricPsorSolver` relaxation path, covering standard 48-row, boxed 24-row,
and friction-index 8-contact fixtures at relaxation 0.5, 1.0, and 1.3. Focused
default, SIMD-enabled, and CUDA-enabled build-tree runs passed with
`contract_ok=1` on every row and recorded symmetric under/plain/over-relaxation
counters. The CUDA-enabled rows are CPU symmetric PSOR solver rows in a
CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 9 `BM_LcpRedBlackGaussSeidelRelaxationSweep` benchmark rows
for the `RedBlackGaussSeidelSolver` relaxation and two-color partition path,
covering standard 48-row, boxed 24-row, and friction-index 8-contact fixtures
at relaxation 0.5, 1.0, and 1.3. Focused default, SIMD-enabled, and
CUDA-enabled build-tree runs passed with `contract_ok=1` on every row and
recorded under/plain/over-relaxation counters plus `red_black_color_count=2`
with the expected 12/24 red and black row counts. The CUDA-enabled rows are CPU
solver rows in a CUDA-enabled build, not CUDA LCP kernel execution; this is not
evidence for solver-internal threaded Red-Black execution.
It now also adds 9 `BM_LcpApgdRestartSweep` benchmark rows for the
`ApgdSolver` adaptive-restart path, covering standard 48-row, boxed 24-row,
and friction-index 8-contact fixtures for adaptive restart every iteration,
adaptive restart every 5 iterations, and no restart. Focused default,
SIMD-enabled, and CUDA-enabled build-tree runs passed with `contract_ok=1` on
every row and recorded `apgd_restart_sweep`, `apgd_adaptive_restart`,
`apgd_restart_check_interval`, `apgd_relaxation=1`, backend build-state
counters, and `contact_count=8` for the friction-index rows. The CUDA-enabled
rows are CPU APGD solver rows in a CUDA-enabled build, not CUDA LCP kernel
execution.
It now also adds 9 `BM_LcpTgsIterationBudgetSweep` benchmark rows for the
`TgsSolver` standalone boxed-LCP sweep, covering standard 48-row, boxed
24-row, and friction-index 8-contact fixtures at 10-, 50-, and
100-iteration budgets. Focused default, SIMD-enabled, and CUDA-enabled
build-tree runs passed with `contract_ok=1` on every row and recorded
`tgs_iteration_budget_sweep`, `tgs_max_iterations`, `tgs_relaxation=1`,
observed `iterations=5/6`, backend build-state counters, and
`contact_count=8` for the friction-index rows. The CUDA-enabled rows are CPU
TGS solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 9 `BM_LcpNncgPgsIterationsSweep` benchmark rows for the
`NncgSolver` PGS-preconditioner path, covering standard 48-row, boxed 24-row,
and friction-index 8-contact fixtures at 1, 2, and 5 PGS preconditioner
iterations while holding restart interval 10 and restart threshold 1.0.
Focused default, SIMD-enabled, and CUDA-enabled build-tree runs passed with
`contract_ok=1` on every row and recorded `nncg_pgs_iterations_sweep`,
`nncg_pgs_iterations`, `nncg_restart_interval=10`,
`nncg_restart_threshold=1`, observed outer `iterations=0/2/4/5`, backend
build-state counters, and `contact_count=8` for the friction-index rows. The
CUDA-enabled rows are CPU NNCG solver rows in a CUDA-enabled build, not CUDA
LCP kernel execution.
It now also adds 9 `BM_LcpSubspaceMinimizationPgsIterationsSweep` benchmark
rows for the `SubspaceMinimizationSolver` active-set-estimation path, covering
standard 48-row, boxed 24-row, and friction-index 8-contact fixtures at 1, 3,
and 5 PGS iterations while holding active-set tolerance 0.0. Focused default,
SIMD-enabled, and CUDA-enabled build-tree runs passed with `contract_ok=1` on
every row and recorded `subspace_pgs_iterations_sweep`,
`subspace_pgs_iterations`, `subspace_active_set_tolerance=0`, observed outer
`iterations=1/2`, backend build-state counters, and `contact_count=8` for the
friction-index rows. The CUDA-enabled rows are CPU SubspaceMinimization solver
rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 18 `BM_LcpAdmmRhoSweep` benchmark rows for the `AdmmSolver`
rho/adaptive-rho path, covering standard 48-row, boxed 24-row, and
friction-index 8-contact fixtures at `rhoInit` 0.5, 1.0, and 4.0 with fixed
and adaptive rho policies. Focused default, SIMD-enabled, and CUDA-enabled
build-tree runs passed with `contract_ok=1` on every row and recorded
`admm_rho_sweep`, `admm_rho_init`, fixed/adaptive policy counters,
`admm_adaptive_rho_tolerance`, `admm_mu_prox`, backend build-state counters,
and `contact_count=8` for the friction-index rows. The CUDA-enabled rows are
CPU ADMM solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 9 `BM_LcpSapRegularizationSweep` benchmark rows for the
`SapSolver` regularized objective path, covering standard 48-row, boxed
24-row, and friction-index 8-contact fixtures at regularization values `1e-6`,
`1e-5`, and `1e-4`. Focused default, SIMD-enabled, and CUDA-enabled build-tree
runs passed with `contract_ok=1` on every row and recorded
`sap_regularization_sweep`, `sap_regularization`, Armijo/backtracking
parameters, `sap_max_line_search_iterations`, backend build-state counters,
and `contact_count=8` for the friction-index rows. The CUDA-enabled rows are
CPU SAP solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also broadens the 1x-/4x-/8x-coupled mildly ill-conditioned
friction-index slice to the scoped iterative solvers plus Boxed Semi-Smooth Newton: the
generated known-solution coverage and
`BM_LcpMildIllConditioned/*CoupledFrictionIndex*` benchmark rows cover
6-, 8-, 12-, 16-, 24-, 32-, 48-, 64-, and 96-contact packets across coupling
scales 1, 4, and 8, plus 16x-coupled packets through 128 contacts. Focused default,
SIMD-enabled, and CUDA-enabled build-tree runs passed
`LcpGeneratedCoverage.LargerMildlyIllConditionedKnownSolutionsForScopedSolvers`
and previously passed with `contract_ok=1` on all 1752 mild single and batch
rows; focused new 128-contact PGS single, serial-batch, and parallel-batch
smokes also pass in default, SIMD-enabled, and CUDA-enabled build trees. The
benchmark rows record
`coupling_scale=1/4/8/16`, `contact_count=6/8/12/16/24/32/48/64/96/128`,
`problem_size=18/24/36/48/72/96/144/192/288/384`,
`total_contact_count=24/32/48/64/96/128/192/256/384/512`,
`total_problem_size=72/96/144/192/288/384/576/768/1152/1536`, and backend
build-state counters. The Boxed Semi-Smooth Newton 16x rows report tuned
line-search settings. The
CUDA-enabled rows are CPU solver rows in a
CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 27 `BM_LcpContactSolverComparisonSweep` benchmark rows for
`Admm`, `Sap`, and `BoxedSemiSmoothNewton` over the same DART 7 contact-derived
fixtures used by the Staggering contact-pipeline sweep: separated world-contact
1/2/4-contact snapshots, coupled vertical-stack 2/3/5-contact snapshots, and
articulated unified ground, rigid-impact, and cross-link-impact 4-contact
snapshots. Focused default, SIMD-enabled, and CUDA-enabled build-tree runs
passed with `contract_ok=1` on every row and recorded solver identity,
fixture-family, normal/friction row-count, contact-count, problem-size, and
backend build-state counters. The CUDA-enabled rows are CPU solver rows in a
CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 76 `BM_LcpContactNormalStandardSweep` benchmark rows that
extract normal-only standard LCP subproblems from those same DART 7
contact-derived fixtures. The rows compare `Dantzig`, `Lemke`, `Baraff`,
`Direct`, `MinimumMapNewton`, `FischerBurmeisterNewton`,
`PenalizedFischerBurmeisterNewton`, `InteriorPoint`, and `MPRGP`; `Direct` is
limited to four 1-, 2-, and 3-row no-fallback rows. Focused default,
SIMD-enabled, and CUDA-enabled build-tree runs passed with `contract_ok=1` on
every row and recorded normal-row, source-problem-size, standard-problem-size,
coupled-fixture, Direct no-fallback, and backend build-state counters. This is
contact-normal standard-LCP evidence, not native boxed or friction-index
support for standard-only solvers.
It now also adds 9 `BM_LcpShockPropagationLayerSweep` benchmark rows for the
`ShockPropagationSolver` layer schedule path, covering standard 48-row, boxed
24-row, and friction-index 8-contact fixtures with single-layer, two-layer, and
serial layer schedules built from 3-row blocks. Focused default, SIMD-enabled,
and CUDA-enabled build-tree runs passed with `contract_ok=1` on every row and
recorded `shock_propagation_layer_sweep`, schedule counters,
`layer_count=1/2/8/16`, `block_count=8/16`, `max_block_size=3`,
`max_blocks_per_layer=1/4/8/16`, backend build-state counters, and
`contact_count=8` for the friction-index rows. The CUDA-enabled rows are CPU
ShockPropagation solver rows in a CUDA-enabled build, not CUDA LCP kernel
execution.
It now also adds 9 `BM_LcpMprgpSpdCheckSweep` benchmark rows for the
`MprgpSolver` standard-SPD path, covering dense SPD 32/64-row, banded SPD
64-row, mildly ill-conditioned SPD 32-row, and near-singular SPD 8-row
standard-LCP fixtures with `checkPositiveDefinite` enabled and disabled where
paired. Focused default, SIMD-enabled, and CUDA-enabled build-tree runs passed
with `contract_ok=1` on every row and recorded `mprgp_spd_check_sweep`,
`mprgp_positive_definite_check`, SPD-kind counters, `problem_size=8/32/64`,
observed solver `iterations=3/5/15`, `mprgp_symmetry_tolerance=1e-9`,
`mprgp_epsilon_for_division=1e-12`, and backend build-state counters. The
CUDA-enabled rows are CPU MPRGP solver rows in a CUDA-enabled build, not CUDA
LCP kernel execution.
It now also adds 9 `BM_LcpInteriorPointPathSweep` benchmark rows for the
`InteriorPointSolver` standard-LCP path, covering dense SPD 32/64-row, banded
SPD 64-row, mildly ill-conditioned SPD 32-row, and near-singular SPD 8-row
standard-LCP fixtures with `sigma=0.1/0.3` and `stepScale=0.75/0.99`. Focused
default, SIMD-enabled, and CUDA-enabled build-tree runs passed with
`contract_ok=1` on every row and recorded `interior_point_path_sweep`,
`interior_point_sigma`, `interior_point_step_scale`, SPD-kind counters,
`problem_size=8/32/64`, observed solver `iterations=14/16/27/31/32/41/51/53`,
and backend build-state counters. The CUDA-enabled rows are CPU Interior Point
solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 9 `BM_LcpStaggeringContactPipelineSweep` benchmark rows for
the `StaggeringSolver` normal/friction split on DART 7 contact-pipeline
fixtures. The rows cover separated sphere-ground 1/2/4-contact fixtures,
coupled vertical-stack 2/3/5-contact fixtures, and articulated unified ground,
rigid-impact, and cross-link-impact 4-contact fixtures. Focused default,
SIMD-enabled, and CUDA-enabled build-tree runs passed with `contract_ok=1` on
every row and recorded `staggering_contact_pipeline_sweep`,
`staggering_normal_friction_split`, normal-row counts `1/2/3/4/5`,
friction-row counts `2/4/6/8/10`, coupled-contact flags, contact counts
`1/2/3/4/5`, and backend build-state counters. The CUDA-enabled rows are CPU
Staggering solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
It now also adds 9 `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep` benchmark rows
for the `BoxedSemiSmoothNewtonSolver` line-search path, covering standard
48-row, boxed 24-row, and friction-index 8-contact fixtures with default line
search, an expanded line-search step budget, and a gentler step-reduction
policy. Focused default, SIMD-enabled, and CUDA-enabled build-tree runs passed
with `contract_ok=1` on every row and recorded
`boxed_ssn_line_search_sweep`, `boxed_ssn_max_line_search_steps=10/20`,
`boxed_ssn_step_reduction=0.5/0.8`, default/more-step/gentle-reduction policy
counters, observed solver `iterations=2/7/8/9`, `contact_count=8` for the
friction-index rows, and backend build-state counters. The CUDA-enabled rows
are CPU BoxedSemiSmoothNewton solver rows in a CUDA-enabled build, not CUDA LCP
kernel execution.
It now also adds 12 `BM_LcpPivotingScaleSweep` benchmark rows for the pivoting
family, separating DirectSolver's 2-row and 3-row enumeration path from its
larger-problem Dantzig fallback. The rows cover Direct 2/3-row standard
fixtures, Lemke and Baraff 8/16-row standard fixtures, and Dantzig 8/16-row
standard, 12/24-row boxed, and 4/8-contact friction-index fixtures. Focused
default, SIMD-enabled, and CUDA-enabled build-tree runs passed with
`contract_ok=1` on every row and recorded `pivoting_scale_sweep`, Direct
no-fallback counters, four Dantzig boxed-or-findex rows,
`contact_count=4/8`, observed solver `iterations=1/4/8/16`, and backend
build-state counters. Focused pivoting unit coverage passed 42 tests. The
CUDA-enabled rows are CPU pivoting solver rows in a CUDA-enabled build, not
CUDA LCP kernel execution.
It now also adds 12 `BM_LcpBlockPartitionSweep` benchmark rows for BGS and
Blocked Jacobi, covering standard 12-row, boxed 12-row, and friction-index
4-contact fixtures with full-block, 3-row block, auto `findex`, and explicit
contact-block partitions. Focused default, SIMD-enabled, and CUDA-enabled
build-tree runs passed with `contract_ok=1` on every row and recorded
`block_partition_sweep`, block counts `1/4`, block sizes `3/12`,
`contact_count=4`, observed solver `iterations=1/4/5/6/10`, and backend
build-state counters. Focused block-solver unit coverage passed 15 tests. The
CUDA-enabled rows are CPU BGS/Blocked Jacobi solver rows in a CUDA-enabled
build, not CUDA LCP kernel execution.
It now also extends the robust near-singular generated slice to coupled
friction-index 128-contact packets and adds 27 `BM_LcpNearSingular` benchmark
rows for standard 8-row, boxed 8-row, and coupled friction-index 3-, 6-, 9-,
12-, 16-, 24-, 32-, 48-, 64-, 96-, and 128-contact packets verified in
default, SIMD-enabled, and CUDA-enabled build trees. The 96- and 128-contact
generated packets use the contract-verified capped normal ramp and `1e6`
diagonal spread; focused generated coverage and focused 128-contact benchmark
rows pass in default, SIMD-enabled, and CUDA-enabled build trees. The
CUDA-enabled rows/tests are CPU solver rows in a CUDA-enabled build, not CUDA
LCP kernel execution.
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
standard 128-row, boxed 128-row, and coupled friction-index
16-/24-/32-/48-/64-/96-/128-/192-contact packets, plus 48 matching
`BM_LcpExtremeSingularDegenerate` benchmark rows
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
friction-index slice: stronger-coupled 24-contact, 72-row, 32-contact, 96-row,
48-contact, 144-row, 64-contact, 192-row, 96-contact, 288-row, and
128-contact, 384-row, and 192-contact, 576-row packets, plus 112 matching
`BM_LcpProductionActiveSetTransition`
benchmark rows verified in the default, SIMD-enabled, and CUDA-enabled build
trees. The generated coverage test now includes the same 192-contact, 576-row,
`coupling_scale=32` production active-set packet and passes focused default,
SIMD-enabled, and CUDA-enabled runs. It now also adds 518
`BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
batch-size-4 serial and DART 7 `ParallelExecutor` runs over standard
32/64/128-row, boxed 32/64/128-row, and coupled friction-index
8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact active-set packets, verified in the default,
SIMD-enabled, and CUDA-enabled build trees. The CUDA-enabled rows are CPU
solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
The work adds DART 7 `dart::simulation::World` boxed-LCP contact snapshot tests
that validate real LCPs assembled from one sphere-ground friction contact and
from two separated sphere-ground friction contacts, plus a 200-step two-sphere
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
3-sphere 200-step, 3-sphere 500-step, 4-sphere 200-step, and 5-sphere 500-step
public `World::step()` invariant tests and benchmark rows for the boxed-LCP
stack path after adding the boxed-LCP contact solve's timestep-driven Baumgarte
velocity bias and preserving the kinematic-contact static-obstacle
compatibility path, plus 4- and 16-sphere separated sphere-ground `World::step()`
invariant tests and 4-/8-/16-contact separated sphere-ground step benchmark
rows. It now also adds a fixed-base prismatic
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
link-vs-link LCP snapshots and 24-contact packets. It now also adds mixed
world-contact batch benchmark rows that compare all friction-index-capable
solvers over the same five separated-contact and stacked-contact snapshots,
both serially and through the DART 7 experimental
`ParallelExecutor`. The CUDA LCP batch evidence now also includes direct
synthetic standard/boxed/findex packets through 256-row and 96-contact sizes,
homogeneous 4-/8-/16-contact DART 7 world-contact packets, homogeneous 5-/6-/7-/8-sphere coupled
stack-contact packets, grouped variable-size synthetic standard/boxed/findex
through 192-row and 64-contact groups, grouped variable-size
1/2/4/8/16-contact separated sphere-ground packets with two- and three-variant
grouped benchmark rows, plus grouped variable-size
2/3/4/5/6/7/8-sphere coupled stack-contact packets with two- and three-variant
grouped benchmark rows, plus grouped variable-size manually assembled
1-/4-/8-/16-contact articulated unified-contact packets with two- and
three-variant grouped benchmark rows covering link-ground, link-vs-dynamic-rigid,
and cross-multibody link-vs-link cases, plus mixed grouped contact batches that
combine separated, stack, and 1-/4-/8-/16-contact articulated fixture families for fixed-iteration
Jacobi and PGS. The generic CPU serial and DART 7 `ParallelExecutor` batch
benchmark registrations now also expose Jacobi and PGS rows at the same
grouped synthetic packet sizes plus the same standard/boxed
24-/48-/96-/128-/192-/256-row and friction-index
8-/16-/32-/48-/64-/96-contact packet sizes used
by the CUDA batch rows; the focused evidence covers default, SIMD-enabled, and
CUDA-enabled build trees.
The work now also adds 49 `BM_LcpLargerActiveSetTransition` rows for the
scoped scalable active-set transition packets: standard 32-row, boxed 32-row,
and coupled friction-index 8-contact. Focused default, SIMD-enabled, and
CUDA-enabled runs pass with `contract_ok=1`; the CUDA-enabled rows are CPU
solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.

## Current Branch

`feature/dart7-lcp-solver-evidence` - local checkpoint branch with DART 7 LCP
solver evidence commits, including native standard-Newton warm-start tests and
default/SIMD/CUDA-enabled warm-start single-problem and batch benchmark rows,
PGS/PSOR, symmetric PSOR, Red-Black Gauss-Seidel relaxation sweep benchmark
rows, APGD restart-policy benchmark rows, ADMM rho/adaptive-rho benchmark
rows, TGS iteration-budget benchmark rows, NNCG PGS-preconditioner iteration
benchmark rows, SubspaceMinimization PGS-iteration benchmark rows,
ShockPropagation layer-layout benchmark rows, MPRGP SPD/check benchmark rows,
Interior Point path-parameter benchmark rows, Staggering contact-pipeline
benchmark rows, Boxed Semi-Smooth Newton line-search benchmark rows, Pivoting
scale benchmark rows, BGS/Blocked Jacobi block-partition benchmark rows, SAP
regularization benchmark rows, Jacobi threading benchmark rows, and
singular-degenerate standard/boxed batch benchmark rows.
The latest checkpoints extend direct synthetic CUDA Jacobi/PGS evidence through
standard/boxed 256-row and friction-index 96-contact packets, extend grouped
synthetic CPU/CUDA Jacobi/PGS rows through 192-row standard/boxed and
64-contact friction-index groups with matching DART 7 `ParallelExecutor` rows,
and broaden solver-internal Jacobi threading benchmark evidence through
8192-row banded SPD rows. The grouped synthetic CPU/CUDA rows, new direct
256-row/96-contact CPU and CUDA rows, and the 8192-row Jacobi threading rows
pass focused default, SIMD-enabled, and CUDA-enabled build-tree benchmark gates
with `contract_ok=1`.
Push/PR work still requires explicit maintainer/user approval.

## Immediate Next Step

Move from the now-verified direct 256-row/96-contact CUDA batch rows, grouped
synthetic CPU/CUDA batch rows, 8192-row banded Jacobi solver-internal threading
rows, extreme 128-row/192-contact exact rank-deficient,
production active-set transition 192-contact, coupled mildly ill-conditioned
24-contact, near-singular 128-contact generated CPU solver coverage, near-singular
serial/parallel batch rows, exact rank-deficient singular-degenerate
friction-index and standard/boxed batch rows, and default/SIMD/CUDA Newton warm-start
single-problem and batch benchmark rows, plus focused
PGS/PSOR, symmetric PSOR, and Red-Black Gauss-Seidel relaxation sweep rows,
APGD restart-policy sweep rows, TGS iteration-budget sweep rows, NNCG
PGS-preconditioner iteration sweep rows, SubspaceMinimization PGS-iteration
sweep rows, ShockPropagation layer-layout sweep rows, MPRGP SPD/check sweep
rows, Interior Point path-parameter sweep rows, Staggering contact-pipeline
sweep rows, Boxed Semi-Smooth Newton line-search sweep rows, Pivoting scale
sweep rows, BGS/Blocked Jacobi block-partition sweep rows, ADMM
rho/adaptive-rho sweep rows, SAP regularization sweep rows, and ADMM/SAP/Boxed
Semi-Smooth Newton contact comparison sweep rows, plus contact-normal
standard-LCP sweep rows, plus the 1x-/4x-/8x-coupled and 16x-through-128 mildly
ill-conditioned friction-index slice, toward broader solver-specific
conditioning grids and direct backend execution evidence beyond SIMD/CUDA-enabled
build-state counters.
Broaden SIMD benchmark gates, solver-internal threaded benchmark evidence, and
the CUDA Jacobi/PGS batch slices into broader/general CUDA LCP execution
separately from the current serial, task-parallel independent-problem, simple
world-contact, mixed world-contact batch, small coupled-stack, 16-contact
separated CUDA world-contact, and 8-sphere homogeneous/grouped CUDA
coupled-stack, and grouped manually assembled articulated unified-contact
including cross-multibody link-vs-link CUDA rows, plus mixed
separated/stack/articulated CUDA benchmark rows, plus the current
PGS-only homogeneous and grouped variable-size dense box-face CUDA batch rows.
Do not claim a 7-sphere public-step stack invariant yet: local temporary probes
failed at both 1000 and 2000 public `World::step()` iterations under the
existing motion-invariant contract, with benchmark probes reporting
`invariant_ok=0`. After that, extend
DART 7 boxed-LCP world-contact
evidence beyond the current separated sphere-ground, fixed-base prismatic
articulated end-to-end coverage, connected Cartesian-chain articulated
end-to-end coverage, cross-multibody articulated link-vs-link impact coverage,
manually assembled three-axis articulated LCP snapshots, 4-/5-/6-sphere
coupled-stack end-to-end, 7-sphere vertical stack snapshots,
and the 48-box unit/benchmark dense face-contact public-step slice to
broader articulated, longer-running coupled, and broader dense/robot-like
contact scenes.

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
  12-contact, and 16-contact packets, plus stronger-coupled production coupled
  friction-index 24-, 32-, and 48-contact packets. The larger mildly
  ill-conditioned
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
  16-/24-/32-/48-/64-/96-contact packets over the same scoped robust solver set.
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
  `BM_LCP_COMPARE` also lists 112
  `BM_LcpProductionActiveSetTransition` rows for stronger-coupled 24-contact,
  72-row, 32-contact, 96-row, 48-contact, 144-row, 64-contact, 192-row,
  96-contact, 288-row, 128-contact, 384-row, and 192-contact, 576-row
  friction-index active-set packets over all 16 friction-index-capable manifest
  solvers; focused 192-contact follow-up runs in the default, SIMD-enabled,
  and CUDA-enabled build trees report the 16 added rows with `contract_ok=1` and report
  `active_set_transition=1`, `production_active_set_transition=1`,
  `contact_count=192`, `problem_size=576`, `coupling_scale=32`, and
  `coupled=1`. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
  `BM_LCP_COMPARE` also lists 518
  `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over standard
  32/64/128-row, boxed 32/64/128-row, and coupled friction-index
  8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact active-set packets. Focused default,
  SIMD-enabled, and CUDA-enabled JSON checks through the 96-contact packet
  report 454 rows with `contract_ok=1`, 227 serial rows, and 227 parallel rows;
  focused 128-contact follow-up runs in those same build trees report 32 rows
  with `contract_ok=1`; focused 192-contact follow-up runs in those same build
  trees report 32 added rows with `contract_ok=1`, `contact_count=192`,
  `total_contact_count=768`, `problem_size=576`, and
  `total_problem_size=2304`. The SIMD run reports `build_simd_enabled=1`, and the
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
  `BM_LCP_COMPARE` also lists 48 `BM_LcpExtremeSingularDegenerate` rows for
  exact rank-deficient standard 128-row, boxed 128-row, and coupled
  friction-index 16-/24-/32-/48-/64-/96-/128-/192-contact packets over the generated
  extreme singular-degenerate solver scope; previous focused default,
  SIMD-enabled, and CUDA-enabled full-slice runs through the 96-contact packet
  passed with `contract_ok=1`, and focused 128-contact follow-up runs in those
  same build trees report 3 added rows with `contract_ok=1`; focused
  192-contact follow-up runs in those same build trees report another 3 added
  rows with `contract_ok=1`. The rows report
  `singular_degenerate=1`,
  `rank_deficient=1`, backend build-state counters, and
  contact counts `16/24/32/48/64/96/128/192`, problem sizes
  `48/72/96/144/192/288/384/576`, and `coupled=1` for the coupled friction-index
  packets. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
  `BM_LCP_COMPARE` also lists 599 `BM_LcpMildIllConditioned` rows for standard
  32-row, boxed 16-row, friction-index 8-contact, and coupled friction-index
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact packets, plus 4x- and 8x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact packets and 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-contact packets; the
  focused default, SIMD-enabled, and CUDA-enabled new 128-contact PGS rows
  pass with `contract_ok=1` and report `mildly_ill_conditioned=1` plus
  contact/coupling counters where applicable. The `BoxedSemiSmoothNewton`
  coupled single-problem rows span coupling scales `1/4/8/16`; the 16x scale
  now includes the 128-contact packet, and the 16x rows report the
  same tuned line-search counters as the batch rows.
  `BM_LCP_COMPARE` also lists 1198
  `BM_LcpMildIllConditionedBatch(Serial|Parallel)` rows for batch-size-4 serial
  and DART 7 `ParallelExecutor` runs over the full scoped mildly
  ill-conditioned
  packet set: standard 32-row, boxed 16-row, friction-index 8-contact, coupled
  friction-index 6-/8-/12-/16-/24-/32-/48-/64-/96-contact,
  4x-coupled 6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 8x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact, and 15-solver 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-contact rows; focused default,
  SIMD-enabled, and CUDA-enabled new 128-contact PGS batch smokes pass with
  `contract_ok=1` and report
  `mildly_ill_conditioned_batch=1`, problem/total-problem-size counters,
  contact/total-contact counters where applicable, coupling-scale counters for
  coupled rows, backend build-state counters, and parallel execution counters
  on `ParallelExecutor` rows. The `BoxedSemiSmoothNewton` coupled batch rows
  span coupling scales `1/4/8/16`; the 16x scale now includes the
  128-contact packet, and the 16x rows report `boxed_ssn_max_line_search_steps=50`,
  `boxed_ssn_step_reduction=0.8`, and
  `boxed_ssn_jacobian_regularization=1e-8`.
  `BM_LCP_COMPARE` also lists 27 `BM_LcpNearSingular` rows for standard
  8-row, boxed 8-row, and coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-,
  32-, 48-, 64-, 96-, and 128-contact
  packets over the scoped robust solver set; the focused default,
  SIMD-enabled, and CUDA-enabled runs pass with `contract_ok=1` on all rows and
  report `near_singular=1`, backend build-state counters, and contact/coupling
  counters where applicable. The CUDA-enabled rows are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
  `BM_LCP_COMPARE` also lists 54
  `BM_LcpNearSingularBatch(Serial|Parallel)` rows for batch-size-4 serial and
  DART 7 `ParallelExecutor` runs over near-singular standard 8-row, boxed
  8-row, and coupled friction-index 3-/6-/9-/12-/16-/24-/32-/48-/64-/96-/128-contact
  packets; focused default, SIMD-enabled, and CUDA-enabled checks pass
  with `contract_ok=1` on all rows and report `near_singular_batch=1`,
  problem/total-problem-size counters, contact/total-contact counters for the
  friction-index rows, backend build-state counters, and parallel execution
  counters on `ParallelExecutor` rows. The CUDA-enabled rows are CPU solver
  batch rows in a CUDA-enabled build, not CUDA LCP kernel execution.
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
  `BM_LCP_COMPARE` also lists 66
  `BM_LcpSingularDegenerateFrictionIndexBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
  rank-deficient coupled friction-index
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact packets; previous focused
  default, SIMD-enabled, and CUDA-enabled JSON checks through the 96-contact
  packet pass with `contract_ok=1`, and focused 128-contact follow-up runs in
  those same build trees report 6 added batch rows with `contract_ok=1`;
  focused 192-contact follow-up runs in those same build trees report another
  6 added batch rows with `contract_ok=1`.
  The rows report `singular_degenerate_batch=1`, `rank_deficient=1`,
  contact/total-contact counters, backend build-state counters, and parallel
  execution counters on `ParallelExecutor` rows. The
  CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
  `BM_LCP_COMPARE` also lists 192
  `BM_LcpSingularDegenerateStandardBoxedBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
  rank-deficient standard and boxed 16-/32-/64-/128-row packets; focused
  default, SIMD-enabled, and CUDA-enabled JSON checks pass with `contract_ok=1`
  on all rows and report `singular_degenerate_batch=1`,
  `singular_degenerate_standard_boxed_batch=1`, `rank_deficient=1`,
  `problem_size=16/32/64/128`, `total_problem_size=64/128/256/512`, backend
  build-state counters, and parallel execution counters on `ParallelExecutor`
  rows. The CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
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
  the previous 1752 focused `BM_LcpMildIllConditioned` single and batch rows
  plus the new 128-contact PGS single and batch rows, all 49 focused
  `BM_LcpStressActiveSetTransition` rows, all 27 focused
  `BM_LcpSingularDegenerate` rows, and all 27 focused
  `BM_LcpLargerSingularDegenerate` rows, and all 27 focused
  `BM_LcpStressSingularDegenerate` rows, plus all 49 focused
  `BM_LcpExtremeActiveSetTransition` rows, all 48 focused
  `BM_LcpProductionActiveSetTransition` rows, the focused
  `BM_LcpExtremeSingularDegenerate` rows through the 96-contact packet, and
  the focused 128-contact `BM_LcpExtremeSingularDegenerate` follow-up rows.
  A separate `build/cuda/cpp/Release` build configured by
  `pixi run -e cuda build-cuda Release` reported DART 7.0.0 with
  `DART_ENABLE_EXPERIMENTAL_CUDA=ON` and `DART_CUDA_ARCHITECTURES=89`; the
  documented `pixi run -e cuda test-cuda Release` smoke passed, and 21-test LCP
  generated coverage plus the focused `BM_LcpMildIllConditioned` slice passed
  with `build_cuda_enabled=1`, `build_simd_enabled=0`, and `contract_ok=1` on
  the previous 1752 single and batch rows plus the new 128-contact PGS single
  and batch rows. The focused `BM_LcpStressActiveSetTransition` slice also passes
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
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `contact_count=24/32/48/64/96`,
  `problem_size=72/96/144/192/288`, `coupling_scale=2/4/8/16/32`, and
  `contract_ok=1` on all 80 rows; the focused 128-contact and 192-contact
  production active-set follow-ups pass in default, SIMD-enabled, and
  CUDA-enabled build trees with 16 single rows and 32 batch rows per tree. The
  focused generated coverage test now also passes the
  24-/32-/48-/64-/96-/128-/192-contact production
  active-set packet set in default, SIMD-enabled, and CUDA-enabled build trees,
  and the previous full default generated coverage suite passes 21 tests
  through the 128-contact packet. The focused
  `BM_LcpProductionActiveSetTransitionBatch` slice passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`,
  `production_active_set_transition_batch=1`, and `contract_ok=1` on all 454
  rows through the 96-contact packet, with focused 128-contact and 192-contact
  follow-up rows passing in the same build trees, and the focused
  `BM_LcpExtremeSingularDegenerate` slice passes with
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all rows through the 96-contact packet, and the focused
  128-contact follow-up passes with `contract_ok=1` on the added rows. The same
  CUDA tree now also builds
  `test_lcp_jacobi_batch_cuda`,
  `BM_LcpCudaJacobiBatch_*`, `BM_LcpCudaPgsBatch_*`, grouped synthetic CUDA
  batch rows, homogeneous world-contact CUDA batch rows, and grouped
  variable-size world-contact CUDA batch rows,
  proving narrow CUDA fixed-iteration projected-Jacobi and PGS batch paths for
  standard, boxed, friction-index, grouped variable-size synthetic
  standard/boxed/findex through 192-row and 64-contact groups,
  CPU serial/`ParallelExecutor` Jacobi/PGS batch rows at the same
  standard/boxed 24-/48-/96-/128-/192-/256-row and friction-index
  8-/16-/32-/48-/64-/96-contact sizes,
  4-/8-/16-contact world-contact packets, and grouped
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
  32/64-row, boxed 16/32-row, friction-index 8-contact, and
  1x-/4x-/8x-coupled mildly ill-conditioned
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact and 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-contact,
  near-singular standard 8-row,
  boxed 8-row, coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, 48-,
  64-, 96-, and 128-contact, and exact rank-deficient
  singular-degenerate standard 16-row, boxed 16-row, and coupled friction-index
  6-contact
  known-solution cases, plus
  larger exact rank-deficient singular-degenerate standard 32-row, boxed
  32-row, and coupled friction-index 8-contact known-solution cases, plus
  stress exact rank-deficient singular-degenerate standard 64-row, boxed
  64-row, and coupled friction-index 12-contact known-solution cases, plus
  extreme exact rank-deficient singular-degenerate standard 128-row, boxed
  128-row, and coupled friction-index
  16-/24-/32-/48-/64-/96-/128-/192-contact known-solution cases, plus
  singular-degenerate standard/boxed batch benchmark packets over
  16-/32-/64-/128-row exact rank-deficient cases, plus
  stress and extreme active-set transition standard 64/128-row, boxed
  64/128-row, coupled friction-index 12/16-contact, and production
  stronger-coupled friction-index 24-/32-/48-/64-/96-contact known-solution cases for
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
  passes on a 128-row generated standard LCP. `BM_LCP_COMPARE` now lists 21
  focused Jacobi threading rows: 4 dense rows for 128-row and 512-row standard
  problems with 1 and 8 worker threads, plus 17 banded rows for 512-row
  standard problems with 1, 4, and 8 worker threads, 1024-row standard problems
  with 1, 4, 8, and 16 worker threads, and 2048-row standard problems with 1,
  8, 16, and 32 worker threads, plus 4096-row standard problems with 1, 8,
  16, and 32 worker threads, plus 8192-row standard problems with 1 and 32
  worker threads. Default, SIMD,
  and CUDA-enabled focused runs report `contract_ok=1`, backend build-state
  counters, `solver_internal_threads`, `worker_count`,
  `jacobi_threading_banded_spd`, `band_half_width`,
  `matrix_nonzero_entries`, and `matrix_density`; the banded rows are
  sparse-structured dense-storage inputs with densities of about 0.00974,
  0.00488, 0.00244, 0.00122, and 0.000610. Focused default, SIMD-enabled, and
  CUDA-enabled checks for the new 8192-row rows passed with `contract_ok=1`,
  `solver_internal_threads=1/32`, `matrix_nonzero_entries=40954`, and
  `matrix_density=0.000610262`. Treat
  this as correctness/comparison evidence rather than a
  performance recommendation. CUDA-enabled rows are CPU Jacobi rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE` now lists 9 focused PGS/PSOR relaxation sweep rows for
  standard 48-row, boxed 24-row, and friction-index 8-contact fixtures at
  relaxation 0.5, 1.0, and 1.3. Default, SIMD, and CUDA-enabled focused runs
  report `contract_ok=1`, `pgs_relaxation_sweep=1`,
  under/plain/over-relaxation counters, backend build-state counters, and
  `contact_count=8` for the friction-index rows. Treat the CUDA-enabled rows as
  CPU PGS solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE` now lists 9 focused symmetric PSOR relaxation sweep rows for
  the same standard 48-row, boxed 24-row, and friction-index 8-contact
  fixtures at relaxation 0.5, 1.0, and 1.3. Default, SIMD, and CUDA-enabled
  focused runs report `contract_ok=1`, `symmetric_psor_relaxation_sweep=1`,
  under/plain/over-relaxation counters, backend build-state counters, and
  `contact_count=8` for the friction-index rows. Treat the CUDA-enabled rows as
  CPU symmetric PSOR solver rows in a CUDA-enabled build, not CUDA LCP kernel
  execution.
- `BM_LCP_COMPARE` now lists 9 focused Red-Black Gauss-Seidel relaxation sweep
  rows for the same standard 48-row, boxed 24-row, and friction-index 8-contact
  fixtures at relaxation 0.5, 1.0, and 1.3. Default, SIMD, and CUDA-enabled
  focused runs report `contract_ok=1`,
  `red_black_gauss_seidel_relaxation_sweep=1`, two-color counters, backend
  build-state counters, and `contact_count=8` for the friction-index rows.
  Treat the CUDA-enabled rows as CPU solver rows in a CUDA-enabled build, not
  CUDA LCP kernel execution; these rows do not prove solver-internal threaded
  Red-Black execution.
- `BM_LCP_COMPARE` now lists 18 focused ADMM rho/adaptive-rho sweep rows for
  the same standard 48-row, boxed 24-row, and friction-index 8-contact
  fixtures at `rhoInit` 0.5, 1.0, and 4.0 under fixed and adaptive rho
  policies. Default, SIMD, and CUDA-enabled focused runs report
  `contract_ok=1`, `admm_rho_sweep=1`, rho/policy counters, backend
  build-state counters, and `contact_count=8` for the friction-index rows.
  Treat the CUDA-enabled rows as CPU ADMM solver rows in a CUDA-enabled build,
  not CUDA LCP kernel execution.
- `BM_LCP_COMPARE` now lists 27 focused contact comparison rows for `Admm`,
  `Sap`, and `BoxedSemiSmoothNewton` on the same DART 7 contact-derived
  separated world-contact, coupled stack-contact, and articulated unified-contact
  fixtures. Default, SIMD, and CUDA-enabled focused runs report
  `contract_ok=1`, solver identity counters, contact counts `1/2/3/4/5`,
  problem sizes `3/6/9/12/15`, normal/friction row-count counters, and backend
  build-state counters. Treat the CUDA-enabled rows as CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE` now lists 76 focused contact-normal standard-LCP rows for
  `Dantzig`, `Lemke`, `Baraff`, `Direct`, `MinimumMapNewton`,
  `FischerBurmeisterNewton`, `PenalizedFischerBurmeisterNewton`,
  `InteriorPoint`, and `MPRGP` on normal-only subproblems extracted from DART 7
  separated world-contact, coupled stack-contact, and articulated
  unified-contact fixtures. Default, SIMD, and CUDA-enabled focused runs report
  `contract_ok=1`, normal-row counts `1/2/3/4/5`, source problem sizes
  `3/6/9/12/15`, standard problem sizes `1/2/3/4/5`, four Direct rows with
  `contact_normal_direct_no_fallback=1`, coupled-fixture counters, and backend
  build-state counters. Treat the CUDA-enabled rows as CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution; this does not claim native
  boxed or friction-index support for standard-only solvers.
- The generated LCP coverage now includes the scoped iterative solvers plus
  Boxed Semi-Smooth Newton on 1x-/4x-/8x-coupled mildly ill-conditioned
  friction-index packets at 6, 8, 12, 16, 24, 32, 48, 64, and 96 contacts plus
  16x-coupled packets through 128 contacts, and `BM_LCP_COMPARE` now lists 599
  `BM_LcpMildIllConditioned` single-problem rows.
  Default, SIMD, and CUDA-enabled focused benchmark runs report
  `contract_ok=1`,
  `coupling_scale=1/4/8/16`, contact counts `6/8/12/16/24/32/48/64/96/128`, problem
  sizes `18/24/36/48/72/96/144/192/288/384`, and backend build-state counters. The
  expanded single and batch focused run previously covered 1752 rows and all rows
  reported `contract_ok=1`; focused new 128-contact PGS smokes also pass.
  Boxed Semi-Smooth Newton 16x rows report tuned line-search
  settings. Treat the CUDA-enabled rows
  as CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- `BoxedSemiSmoothNewton` now includes the derivative of moving `findex`
  friction bounds in its natural-residual Jacobian. The previously failing
  coupled mildly ill-conditioned 4-contact friction-index generated case passes
  the focused generated coverage test without allowing `MaxIterations` for the
  boxed semi-smooth Newton path.
- `tests/unit/simulation/contact/test_boxed_lcp_contact.cpp` now
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
  runs pass, and `test_boxed_lcp_contact --gtest_list_tests` now lists 51
  tests. The full `--gtest_brief=1` run still emits the existing
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
  `BoxedLcpContact.StressSphereStackWorldContactSnapshotSatisfiesLcpContract`,
  and
  `BoxedLcpContact.LargerStressSphereStackWorldContactSnapshotSatisfiesLcpContract`
  and `BoxedLcpContact.SevenSphereStackWorldContactSnapshotSatisfiesLcpContract`
  validate 3-sphere, 4-sphere, 5-sphere, 6-sphere, and 7-sphere vertical stacks assembled
  from `World::collide()` and check that the normal-contact block has nonzero
  off-diagonal coupling. The 4-sphere snapshot has 4 contacts and 12 LCP rows,
  the 5-sphere snapshot has 5 contacts and 15 LCP rows, and the 6-sphere
  snapshot has 6 contacts and 18 LCP rows; the 7-sphere snapshot has 7 contacts
  and 21 LCP rows.
  The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*SphereStack*'` run
  passes these snapshot tests plus the 3-sphere 200-step, 3-sphere 500-step,
  4-sphere 200-step, 5-sphere 500-step, and 6-sphere 1000-step invariant tests.
  Temporary 7-sphere public-step probes were removed because the 1000-step unit
  path failed the near-rest vertical-velocity invariant and
  `BM_LcpWorldStackStep_BoxedLcp/7/1000` reported `invariant_ok=0`,
  `min_spacing=0.999225`, and `max_vertical_speed=5.90648`; the 2000-step
  unit path still failed height, spacing, and near-rest checks, and the
  matching benchmark probe reported `invariant_ok=0` with
  `max_vertical_speed=4.599`.
  Focused default, SIMD-enabled, and CUDA-enabled
  `test_boxed_lcp_contact --gtest_filter=BoxedLcpContact.LargerStressSphereStackWorldContactSnapshotSatisfiesLcpContract:BoxedLcpContact.LargerStressSphereStackWorldStepMaintainsContactInvariants`
  runs pass the new 6-sphere tests. The full
  `test_boxed_lcp_contact --gtest_list_tests` inventory now lists 51 tests.
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
  2- and 3-sphere vertical stacks. It also registers 45
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{4,5,6}` rows for the same
  solver set except `NNCG`; a focused `NNCG` 4-sphere trial reached the
  benchmark cap with `contract_ok=0`, so the NNCG 4-/5-/6-sphere rows are not
  claimed. It also registers 14
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/7` rows for that set except
  `NNCG` and `RedBlackGaussSeidel`; a focused `RedBlackGaussSeidel` 7-sphere
  probe reported `contract_ok=0`, so that row is not claimed. The matching
  `BM_LcpWorldStackContactAssembly_BoxedLcp/{2,3,4,5,6}` rows rebuild the world,
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
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/6$|BM_LcpWorldStackContactAssembly_BoxedLcp/6|BM_LcpWorldStackStep_BoxedLcp/6/1000$' --benchmark_min_time=0.001s`
  runs pass with `contract_ok=1` for all 15 registered 6-sphere solver rows,
  `contract_ok=1` for the 6-sphere assembly row, and `invariant_ok=1` for the
  6-sphere 1000-step public-step row.
  A focused default run of `BM_LcpWorldStackContactAssembly_BoxedLcp/7`
  reports `contract_ok=1`, `sphere_count=7`, `contact_count=7`, and
  `problem_size=21`.
  A focused default
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/7$|BM_LcpWorldStackContactAssembly_BoxedLcp/7$' --benchmark_min_time=0.001s`
  run reports `contract_ok=1` for all 14 registered 7-sphere solver rows and
  the 7-sphere assembly row.
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
- The benchmark target now also registers 518
  `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows over
  batch-size-4 serial and DART 7 `ParallelExecutor` runs for standard
  32/64/128-row, boxed 32/64/128-row, and coupled friction-index
  8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact active-set packets. Focused JSON benchmark
  checks through the 96-contact packet in default, SIMD-enabled, and
  CUDA-enabled build trees report 454 rows with `contract_ok=1`; focused
  128-contact follow-up runs in those same build trees report 32 rows with
  `contract_ok=1`; focused 192-contact follow-up runs report 32 added rows with
  `contract_ok=1`. Each row reports
  `production_active_set_transition_batch=1`, `batch_size=4`,
  `problem_size=24/32/36/48/64/72/96/128/144/192/288/384/576`, and
  `total_problem_size=96/128/144/192/256/288/384/512/576/768/1152/1536/2304`; friction-index rows
  also report `contact_count=8/12/16/24/32/48/64/96/128/192`,
  `total_contact_count=32/48/64/96/128/192/256/384/512/768`, and `coupling_scale=1/2/4/8/16/32`.
  Parallel rows also report `profile_enabled=1`, `worker_count=20`,
  `parallel_units=4`, and observed `max_parallelism`. Treat this as serial and
  DART 7 `ParallelExecutor` batch evidence over synthetic active-set packets,
  not CUDA kernel execution.
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
  and `EightBoxWorldStepMaintainsDenseContactInvariants` extend the unit
  coverage to separated 4-box and 8-box, 16-contact and 32-contact dense
  face-contact scenes advanced for 200 public boxed-LCP `World::step()`
  iterations. `SixteenBoxWorldStepMaintainsDenseContactInvariants` extends the
  same scene family to 16 boxes and 64 dense face contacts over 500 public
  boxed-LCP `World::step()` iterations.
  `TwentyFourBoxWorldStepMaintainsDenseContactInvariants` extends unit coverage
  to 24 boxes and 96 dense face contacts over 2000 small public boxed-LCP
  `World::step()` iterations.
  `ThirtyTwoBoxWorldStepMaintainsDenseContactInvariants` extends unit coverage
  to 32 boxes and 128 dense face contacts over 4000 small public boxed-LCP
  `World::step()` iterations.
  `FortyEightBoxWorldStepMaintainsDenseContactInvariants` extends unit coverage
  to 48 boxes and 192 dense face contacts over 4000 small public boxed-LCP
  `World::step()` iterations; the focused default run passed in 84992 ms. The
  full `test_boxed_lcp_contact --gtest_list_tests` inventory lists 51 tests; the
  earlier `--gtest_brief=1` run still
  emitting the dense-patch Dantzig warning.
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` registers 72 scoped dense box
  rows: `Pgs`, `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and `Admm` on
  1/2/4/8/16/24/32/48/64/96/128/192-box snapshots. Focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContact/FrictionIndex/.+/192$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs report
  `contract_ok=1`, `dense_box_contact=1`, `box_count=192`,
  `contact_count=768`, and `problem_size=2304` in default, SIMD-enabled, and
  CUDA-enabled build trees; CUDA-enabled rows are CPU solver rows in that build
  tree, not CUDA LCP kernel execution.
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` also registers 72 dense
  box-face serial and DART 7 `ParallelExecutor` batch rows:
  `BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex` rows cover
  `Pgs`, `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and `Admm` over
  24/64/96/128/192-box snapshots at batch size 4; `Pgs` additionally covers
  1/4/8/16/32/48-box snapshots so CPU rows match the homogeneous CUDA PGS
  packet sizes through 96 boxes. Focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/.+/192/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in default, SIMD-enabled, and CUDA-enabled build trees report 12 rows
  with `contract_ok=1`, `dense_box_contact_batch=1`, `box_count=192`,
  `contact_count=768`, `problem_size=2304`, `batch_size=4`,
  `total_contact_count=3072`, `total_problem_size=9216`, and
  `parallel_units=4` on parallel rows; CUDA-enabled rows are CPU solver batch
  rows in that build tree, not CUDA LCP kernel execution. Focused matching-size
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/Pgs/(1|4|8|16|24|32|48|64|96)/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in default, SIMD-enabled, and CUDA-enabled build trees report 18 rows
  with `contract_ok=1`, `dense_box_contact_batch=1`,
  `box_count=1/4/8/16/24/32/48/64/96`,
  `contact_count=4/16/32/64/96/128/192/256/384`,
  `problem_size=12/48/96/192/288/384/576/768/1152`, `batch_size=4`,
  `total_problem_size` up to 4608, and `parallel_units=4` on parallel rows.
- `BM_LcpWorldBoxStep_BoxedLcp/{1,2,4,8}/200` plus
  `BM_LcpWorldBoxStep_BoxedLcp/16/500`,
  `BM_LcpWorldBoxStep_BoxedLcp/24/2000`, and
  `BM_LcpWorldBoxStep_BoxedLcp/{32,48}/4000` rebuild separated
  dense box-face worlds, confirm 4/8/16/32/64/96/128/192 contacts before stepping,
  enter simulation mode, advance public boxed-LCP `World::step()` iterations,
  and report end-to-end invariant counters. Focused default, SIMD-enabled, and
  CUDA-enabled build-tree runs over the 24-/32-box rows reported
  `invariant_ok=1`, `dense_box_contact=1`, `box_count=24/32`,
  `contact_count=96/128`, and `step_count=2000/4000`; the default 32-box row
  reported `max_height_error=1.46e-4` and `max_vertical_speed=4.38e-2`.
  `BoxedLcpContact.ThirtyTwoBoxWorldStepMaintainsDenseContactInvariants`
  and `BoxedLcpContact.FortyEightBoxWorldStepMaintainsDenseContactInvariants`
  cover the same 32-box/128-contact and 48-box/192-contact scenes in unit
  tests. A
  focused default 48-box row reported `invariant_ok=1`, `contact_count=192`,
  `step_count=4000`, `max_height_error=9.80e-5`, and
  `max_vertical_speed=1.08e-2`. Focused SIMD-enabled and CUDA-enabled 48-box
  rows also reported `invariant_ok=1`: the SIMD row reported
  `build_simd_enabled=1`, `max_height_error=99.597u`, and
  `max_vertical_speed=0.0288169`, and the CUDA-enabled row reported
  `build_cuda_enabled=1`, `max_height_error=98.038u`, and
  `max_vertical_speed=0.0108071`. The
  CUDA-enabled rows are CPU public-step rows in that build tree, not CUDA LCP
  kernel execution. The runs still emit the dense-patch Dantzig warning, so
  treat this as public-step invariant evidence,
  not a direct Dantzig dense box solve claim.
- `BM_LcpWorldStackStep_BoxedLcp/3/200`,
  `BM_LcpWorldStackStep_BoxedLcp/3/500`,
  `BM_LcpWorldStackStep_BoxedLcp/4/200`,
  `BM_LcpWorldStackStep_BoxedLcp/5/500`, and
  `BM_LcpWorldStackStep_BoxedLcp/6/1000` rebuild coupled stack worlds, enter
  simulation mode, advance public boxed-LCP `World::step()` iterations, and
  report end-to-end invariant counters. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in default, SIMD-enabled, and CUDA-enabled build trees reported
  `invariant_ok=1` for the existing 3-sphere rows and the 4-/5-/6-sphere rows.
  The SIMD run reported
  `build_simd_enabled=1`, and the CUDA-enabled run reported
  `build_cuda_enabled=1`. The default 3-sphere rows reported
  `time_step=0.005`, `min_spacing=0.9999`, and
  `max_vertical_speed=4.31e-14`; the default 4-sphere row reported
  `time_step=0.001`, `min_spacing=0.9999`, and
  `max_vertical_speed=1.72e-8`; the default 5-sphere row reported
  `time_step=0.001`, `min_spacing=0.9999`, and
  `max_vertical_speed=1.26e-5`; the default 6-sphere row reported
  `time_step=0.001`, `min_spacing=0.9999`, and
  `max_vertical_speed=2.52e-6`.
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
- `BM_LcpArticulatedUnifiedContact/FrictionIndex/{Ground,RigidImpact,CrossLinkImpact}/<solver>/{1,4,8,16,24}`
  manually assemble fixed-base three-axis prismatic `LinkContact` snapshots
  through `assembleMultibodyLinkContactProblem` and
  `assembleUnifiedConstraintProblem`, then compare all 16
  friction-index-capable solvers on identical 3-row, 12-row, 24-row, 48-row, and
  72-row LCPs. The cross-link rows complete a second articulated endpoint for a separate
  multibody, so they exercise the unified contact matrix's cross-multibody
  block. Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpArticulatedUnifiedContact/FrictionIndex/.+/24$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `contract_ok=1` for all 48 new 24-contact rows, with
  `articulated_unified_contact=1`, `contact_count=24`, `problem_size=72`,
  `multibody_count=24` for ground/rigid-impact rows, and `multibody_count=48`
  plus `articulated_cross_link_contact=1` for cross-link rows. The full
  articulated unified-contact registration now lists 240 rows.
  The SAP rows use the same robust benchmark parameters as generated coverage
  (`sap_regularization=1e-6`, `sap_max_line_search_iterations=32`,
  `maxIterations=5000`) and report those SAP counters in the articulated
  unified-contact benchmark output; the focused 24-contact runs reported SAP
  iteration counts of 2 and 97 across the three articulated contact cases.
  This is manual articulated unified-contact LCP assembly evidence, not
  collision-discovered or end-to-end stepping evidence. The CUDA-enabled
  all-solver rows are CPU solver rows in that build tree, not CUDA kernel
  execution.
- CUDA was enabled and validated through the documented DART 7
  `simulation-experimental-cuda` smoke path on a visible RTX 4080 Laptop GPU,
  and the LCP generated coverage plus selected LCP benchmark rows pass in that
  CUDA-enabled build. `test_lcp_jacobi_batch_cuda` passes 33 tests, including
  standard, boxed, friction-index, grouped variable-size synthetic
  standard/boxed/friction-index, contact-derived world-contact, homogeneous
  5-/6-/7-/8-sphere coupled stack-contact batches, and grouped variable-size
  1/2/4/8/16-contact separated world-contact and 2/3/4/5/6/7/8-sphere coupled
  stack-contact batches with two and three variants per contact or sphere
  count, plus grouped manually assembled
  1-/4-/8-/16-contact articulated unified-contact batches, plus mixed grouped contact
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
  fixed iteration counters, and `contract_ok=1`. The direct/grouped synthetic
  CUDA follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)(Batch|GroupedBatch)_(Standard|Boxed|FrictionIndex)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reports 36 CUDA rows with `contract_ok=1`. The homogeneous rows cover
  standard and boxed 24-/48-/96-/128-row packets and friction-index
  8-/16-/32-/48-contact
  packets at batch size 4. A focused grouped synthetic follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)GroupedBatch_(Standard|Boxed|FrictionIndex)/(2|3)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reports 12 CUDA rows with `contract_ok=1` and `cuda_group_count=6`. The
  `/2` rows report `batch_size=12`, standard/boxed
  16/32/48/96/128/192-row packets with `total_problem_size=1024`, and
  friction-index 4/8/16/32/48/64-contact packets with `min_problem_size=12`,
  `max_problem_size=192`, `total_contact_count=344`, and
  `total_problem_size=1032`. The `/3` rows report `batch_size=18`,
  standard/boxed `total_problem_size=1536`, and friction-index
  `total_contact_count=516` and `total_problem_size=1548` across the same size
  groups. The grouped CPU serial/`ParallelExecutor` follow-up reports 24 rows
  in each default, SIMD-enabled, and CUDA-enabled build tree with
  `contract_ok=1`, `batch_group_count=6`, `batch_size=12/18`,
  standard/boxed `total_problem_size=1024/1536`, friction-index
  `total_contact_count=344/516`, friction-index
  `total_problem_size=1032/1548`, and `parallel_units=12/18` plus
  `profile_enabled=1` on `ParallelExecutor`
  rows. The apples-to-apples CPU/CUDA batch follow-up
  reports 36 CPU serial/`ParallelExecutor` rows with `contract_ok=1`,
  `batch_size=4`, standard/boxed `problem_size=24/48/96`,
  friction-index `contact_count=8/16/32`, `parallel_units=4` on
  `ParallelExecutor` rows, and `total_problem_size` up to 384; the
  SIMD-enabled build reports the same 36 CPU rows with `contract_ok=1`,
  `build_simd_enabled=1`, and the same size counters. A focused larger-size
  CPU follow-up in both default and SIMD builds reports the 12 new
  serial/`ParallelExecutor` rows with `contract_ok=1` for standard/boxed
  128-row and friction-index 48-contact packets, `batch_size=4`,
  `parallel_units=4` on parallel rows, standard/boxed `total_problem_size=512`,
  and friction-index `total_problem_size=576`. A focused 192-row/64-contact CPU
  follow-up in default, SIMD-enabled, and CUDA-enabled build trees reports 12
  serial/`ParallelExecutor` rows with `contract_ok=1`, `batch_size=4`,
  standard/boxed and friction-index `problem_size=192`,
  `total_problem_size=768`, friction-index `contact_count=64`, and
  `parallel_units=4` on parallel rows. The direct CUDA rows now cover
  standard/boxed 24-/48-/96-/128-/192-/256-row and friction-index
  8-/16-/32-/48-/64-/96-contact packets. A focused 256-row/96-contact CPU
  follow-up in default, SIMD-enabled, and CUDA-enabled build trees reports 12
  serial/`ParallelExecutor` rows with `contract_ok=1`, `batch_size=4`,
  standard/boxed `problem_size=256`, friction-index `problem_size=288`,
  friction-index `contact_count=96`, `total_problem_size=1024/1152`, and
  `parallel_units=4` on parallel rows. The focused CUDA unit run
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.LargerSyntheticBatchSatisfiesLcpContract:CudaLcpPgsBatch.LargerSyntheticBatchSatisfiesLcpContract' --gtest_brief=1`
  passes both larger synthetic batch tests with those cases included; the
  focused 192-row/64-contact CUDA
  follow-up reports six rows with `cuda_batch_execution=1`, `contract_ok=1`,
  `problem_size=192`, and `total_problem_size=768`, and the focused
  256-row/96-contact CUDA follow-up reports six rows with `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `contract_ok=1`, `problem_size=256/288`,
  `contact_count=96` on friction-index rows, and `total_problem_size=1024/1152`.
  The homogeneous world-contact
  CUDA rows are
  4-, 8-, and 16-contact packets; the denser rows report
  `batch_size=4`, `contact_count=16`, `problem_size=48`,
  `total_contact_count=64`, and `total_problem_size=192`. The homogeneous stack
  CUDA rows now include 5-, 6-, 7-, and 8-sphere coupled stack packets; the
  8-sphere rows report `batch_size=4`, `contact_count=8`, `problem_size=24`,
  `total_contact_count=32`, and `total_problem_size=96`. The separated grouped
  variable-size rows are
  sphere-ground packets with 1/2/4/8/16 contacts. The `/2` rows report
  `batch_size=10`, `cuda_group_count=5`, `contact_shape_count=5`,
  `min_problem_size=3`, `max_problem_size=48`, `total_contact_count=62`, and
  `total_problem_size=186`; the `/3` rows report
  `problem_variants_per_shape=3`, `batch_size=15`, `cuda_group_count=5`,
  `contact_shape_count=5`, `min_problem_size=3`, `max_problem_size=48`,
  `total_contact_count=93`, and `total_problem_size=279`.
  The coupled stack grouped variable-size rows are 2/3/4/5/6/7/8-sphere stack
  packets. The `/2` rows report `batch_size=14`, `cuda_group_count=7`,
  `contact_shape_count=7`, `min_problem_size=6`, `max_problem_size=24`,
  `total_contact_count=70`, and `total_problem_size=210`; the `/3` rows
  report `problem_variants_per_shape=3`, `batch_size=21`,
  `cuda_group_count=7`, `contact_shape_count=7`, `min_problem_size=6`,
  `max_problem_size=24`, `total_contact_count=105`, and
  `total_problem_size=315`.
  The articulated unified-contact grouped variable-size rows are manually
  assembled fixed-base three-axis prismatic link-ground,
  link-vs-dynamic-rigid, and cross-multibody link-vs-link packets with 1, 4,
  8, and 16 contacts. The `/2` rows report `batch_size=24`,
  `cuda_group_count=4`, `contact_shape_count=4`,
  `articulated_contact_case_count=3`, `articulated_cross_link_contact=1`,
  `min_problem_size=3`, `max_problem_size=48`, `total_contact_count=174`, and
  `total_problem_size=522`; the `/3` rows report
  `problem_variants_per_shape=3`, `batch_size=36`, `cuda_group_count=4`,
  `contact_shape_count=4`, `articulated_contact_case_count=3`,
  `articulated_cross_link_contact=1`, `min_problem_size=3`,
  `max_problem_size=48`, `total_contact_count=261`, and
  `total_problem_size=783`.
  The mixed grouped rows combine separated sphere-ground, coupled stack, and
  manually assembled articulated unified-contact packets including
  cross-multibody link-vs-link packets. The latest focused unit test covers two
  and three variants per mixed scenario, and the focused `/3` benchmark rows
  report `batch_size=48`, `contact_fixture_family_count=3`,
  `cuda_group_count=6`, `contact_shape_count=6`,
  `problem_variants_per_shape=3`, `articulated_contact_case_count=3`,
  `articulated_cross_link_contact=1`, `min_problem_size=3`,
  `max_problem_size=48`, `total_contact_count=297`,
  `total_problem_size=891`.
  The dense box-face CUDA rows are PGS-only:
  `CudaLcpPgsBatch.DenseBoxWorldContactBatchSatisfiesLcpContract` and
  `CudaLcpPgsBatch.DenseBoxWorldContactGroupedBatchSatisfiesLcpContract` pass
  on homogeneous and grouped variable-size dense box-face `World::collide()`
  snapshots. `BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex/{1,4,8,16,24,32,48,64,96}/4`
  reports 9 homogeneous rows with `cuda_dense_box_contact_batch=1`,
  `cuda_lcp_execution=1`, `cuda_batch_execution=1`, `contract_ok=1`,
  `box_count=1/4/8/16/24/32/48/64/96`,
  `contact_count=4/16/32/64/96/128/192/256/384`,
  `problem_size=12/48/96/192/288/384/576/768/1152`, `batch_size=4`, and
  `total_problem_size` up to 4608. `BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex/2`
  reports `cuda_group_count=7`, `box_count_shape_count=7`,
  `min_problem_size=12`, `max_problem_size=384`, `total_contact_count=696`,
  `total_body_count=174`, and `total_problem_size=2088`. Grouped dense-box CUDA
  remains scoped to 1/2/4/8/16/24/32-box packets; a grouped 48-box CUDA PGS
  probe failed two 48-box variants at 1024 and 2048 fixed iterations with
  fixed-variable residual/complementarity around 0.606/0.625. A homogeneous
  128-box CUDA PGS probe also remains unclaimed because `World::collide()`
  returned 472 box contacts while the fixture contract requires 512. A
  fixed-iteration CUDA Jacobi dense-box trial failed the LCP contract, so
  Jacobi dense-contact CUDA execution remains unclaimed. The earlier failed probe used the previous
  homogeneous 4-problem and grouped 1/2/4-box dense box-face fixtures: at 4096
  iterations with relaxation 1.0 it failed with
  residual/complementarity/bound violations of about 3.3e-2 to 5.0e-2
  (`w must be non-negative at lo`), and at 8192 iterations with
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
  "--benchmark_filter=^BM_LcpCuda(Jacobi|Pgs)(Batch|GroupedBatch)_(Standard|Boxed|FrictionIndex)" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1" \
  "--benchmark_format=json"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)GroupedBatch_(Standard|Boxed|FrictionIndex)/3$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)/(24|48|96|8|16|32)/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1" \
  "--benchmark_format=json"
./build/simd/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)/(24|48|96|8|16|32)/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1" \
  "--benchmark_format=json"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpCuda(Jacobi|Pgs)Batch_(Standard|Boxed|FrictionIndex)" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1" \
  "--benchmark_format=json"
python scripts/run_benchmark_smoke.py build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed)/(Jacobi|Pgs)/128/4$|^BM_LcpBatch(Serial|Parallel)/FrictionIndex/(Jacobi|Pgs)/48/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/simd/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed)/(Jacobi|Pgs)/128/4$|^BM_LcpBatch(Serial|Parallel)/FrictionIndex/(Jacobi|Pgs)/48/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed)/(Jacobi|Pgs)/192/4$|^BM_LcpBatch(Serial|Parallel)/FrictionIndex/(Jacobi|Pgs)/64/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/simd/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed)/(Jacobi|Pgs)/192/4$|^BM_LcpBatch(Serial|Parallel)/FrictionIndex/(Jacobi|Pgs)/64/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed)/(Jacobi|Pgs)/192/4$|^BM_LcpBatch(Serial|Parallel)/FrictionIndex/(Jacobi|Pgs)/64/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)Batch_(Standard|Boxed)/128/4$|BM_LcpCuda(Jacobi|Pgs)Batch_FrictionIndex/48/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)Batch_(Standard|Boxed)/192/4$|BM_LcpCuda(Jacobi|Pgs)Batch_FrictionIndex/64/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)WorldContactGroupedBatch_FrictionIndex/3$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
python scripts/run_benchmark_smoke.py build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)WorldStackContact(Batch_FrictionIndex/8/4|GroupedBatch_FrictionIndex/2$)" \
  "--benchmark_min_time=0.001s"
python scripts/run_benchmark_smoke.py build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)WorldStackContactGroupedBatch_FrictionIndex/3$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=BM_LcpCuda(Jacobi|Pgs)ArticulatedUnifiedContactGroupedBatch_FrictionIndex/3$" \
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
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/.+/192/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/default/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/Pgs/(1|4|8|16|24|32|48|64|96)/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/simd/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/Pgs/(1|4|8|16|24|32|48|64|96)/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/Pgs/(1|4|8|16|24|32|48|64|96)/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
./build/cuda/cpp/Release/bin/BM_LCP_COMPARE \
  "--benchmark_filter=^BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex/(1|4|8|16|24|32|48|64|96)/4$" \
  "--benchmark_min_time=0.001s" \
  "--benchmark_repetitions=1"
```

Then continue with harder generated-size coverage, broader DART 7 boxed-LCP
contact coverage beyond the current 48-box dense box-face snapshot/step slices,
and backend-specific SIMD/CUDA/solver-internal threaded benchmark evidence
described in `README.md` and `01-implementation-audit.md`.
