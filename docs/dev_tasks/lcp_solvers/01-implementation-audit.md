# LCP Solver Implementation Audit

## Scope

This audit is for the DART 7 LCP solver work only. It maps the solver methods
described in `docs/background/lcp/` to the current DART 7 implementation
surface, shared test manifest, comparison tests, and benchmark registration.

Evidence used:

- Background taxonomy: `docs/background/lcp/02_overview.md`,
  `03_pivoting-methods.md`, `04_projection-methods.md`,
  `05_newton-methods.md`, `06_other-methods.md`, and
  `07_selection-guide.md`.
- Exported implementation surface: `dart/math/lcp/all.hpp`.
- Shared support manifest:
  `tests/common/lcpsolver/lcp_solver_manifest.hpp`.
- Correctness coverage:
  `tests/unit/math/lcp/test_all_solvers_smoke.cpp` and
  `tests/unit/math/lcp/test_lcp_comparison_harness.cpp`.
- Generated coverage:
  `tests/unit/math/lcp/test_lcp_generated_coverage.cpp`.
- Benchmark coverage: `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`.

Support abbreviations:

- `S`: standard LCP.
- `B`: boxed LCP.
- `F`: friction-index LCP.

## Summary

- The current DART 7 implementation surface exports 24 LCP solvers through
  `dart/math/lcp/all.hpp`.
- The shared manifest records 24 standard-capable solvers, 16 boxed-capable
  solvers, and 16 friction-index-capable solvers.
- The background documents now include the exported DART 7 APGD, TGS, ADMM,
  SAP, and boxed semi-smooth Newton variants alongside the original solver
  taxonomy.
- Current evidence is mostly math-layer evidence, with DART 7
  `dart::simulation::World` boxed-LCP contact snapshot tests for
  single-contact and two-contact sphere-ground friction cases, 200-step
  two-sphere and four-sphere boxed-LCP
  `World::step()` invariant tests, 16-sphere separated-contact boxed-LCP
  `World::step()` invariant tests, 3-, 4-, and 5-sphere coupled-stack boxed-LCP
  snapshot tests, 3-sphere 200-step, 3-sphere 500-step, 4-sphere 200-step, and
  5-sphere 500-step boxed-LCP `World::step()` invariant tests and benchmark
  rows, 4-/8-/16-contact separated sphere-ground step benchmark rows,
  1-/2-/4-/8-/16-/24-/32-/48-box dense box-face step
  benchmark rows, a
  fixed-base prismatic articulated link-ground
  boxed-LCP `World::step()` invariant tests for one-link and four-link scenes,
  1-/4-/8-/16-link articulated ground-step benchmark rows, connected
  fixed-base three-axis prismatic Cartesian-chain boxed-LCP `World::step()`
  invariant coverage, 1-/4-/8-/16-chain articulated Cartesian benchmark rows, a fixed-base
  prismatic link-vs-dynamic-rigid boxed-LCP `World::step()` invariant test, and
  1-/4-/8-/16-pair articulated
  rigid-impact benchmark rows, a cross-multibody fixed-base prismatic
  link-vs-link boxed-LCP `World::step()` invariant test, and
  1-/4-/8-/16-pair articulated link-impact benchmark rows, plus all-solver
  articulated unified-contact
  benchmark rows for manually assembled fixed-base three-axis prismatic
  link-ground, link-vs-dynamic-rigid, and cross-multibody link-vs-link through
  16-contact
  snapshots,
  serial and DART 7 `ParallelExecutor` independent-problem batch benchmark
  packets, and world-contact benchmark rows that compare all
  friction-index-capable
  solvers on the same 1/2/4 separated sphere-ground boxed/findex contact
  snapshots, all-solver 2/3-sphere vertical-stack boxed/findex snapshots,
  scoped 4-sphere vertical-stack rows excluding `NNCG`, 5-sphere stack assembly
  rows, and a mixed 5-problem
  serial/`ParallelExecutor` batch over the 1/2/4 separated-contact and 2/3
  stack snapshots. A focused local SIMD-enabled CPU run now exists for
  generated LCP correctness and selected benchmark rows. Jacobi also has an
  opt-in solver-internal CPU worker-thread correctness and comparison slice,
  plus experimental CUDA fixed-iteration Jacobi and PGS batch paths for
  homogeneous dense standard/boxed/friction-index LCP packets. The CUDA batch
  evidence now also includes grouped synthetic, separated sphere-ground,
  coupled stack-contact, manually assembled articulated unified-contact
  packets, PGS-only homogeneous and grouped variable-size dense box-face
  contact batches, plus mixed
  grouped batches spanning those contact fixture families.
  A focused CUDA-enabled build/runtime run also exists and the LCP harness
  passes in that CUDA-enabled build, but broader robot-like denser contact
  pipeline coverage, full SIMD benchmark gates, broader solver-internal
  multi-threaded runs, general CUDA LCP solver execution, and broader
  vectorized/CUDA LCP batch-processing paths are still missing. The narrow CUDA Jacobi/PGS batch
  evidence now includes grouped variable-size synthetic standard/boxed/findex
  packets, homogeneous 4-/8-/16-contact and grouped variable-size
  1/2/4/8/16-contact DART 7 sphere-ground world-contact packets, homogeneous
  5-sphere and grouped variable-size 2/3/4/5-sphere coupled stack-contact
  packets, grouped variable-size 1-/4-/8-/16-contact articulated unified-contact
  packets including cross-multibody link-vs-link cases, and PGS-only
  homogeneous plus grouped variable-size dense box-face contact packets, plus
  mixed grouped batches that combine separated, stack, and
  1-/4-/8-/16-contact articulated fixture families including cross-multibody link-vs-link packets,
  in addition to homogeneous synthetic
  standard/boxed/findex packets. The generated synthetic grid now
  also includes a scoped larger mildly ill-conditioned slice for standard
  32/64-row, boxed 16/32-row, friction-index 8-contact, and coupled
  friction-index 6-, 8-, 12-, 16-, and 24-contact packets, plus
  4x-coupled 6-/8-/12-/16-/24-contact packets, 8x-coupled
  6-/8-/12-/16-/24-/32-contact packets with Boxed Semi-Smooth Newton included
  in single rows plus non-16x and green 16x batch rows,
  ADMM/SAP/Boxed Semi-Smooth Newton
  single-problem
  16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-contact packets, and 308 matching benchmark rows verified in default,
  SIMD-enabled, and CUDA-enabled build trees. It now
  also includes a scoped robust near-singular slice for standard 8-row, boxed
  8-row, and coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, and
  48-, and 64-contact packets, plus 23 matching benchmark rows and 46 matching
  batch benchmark rows verified in default, SIMD-enabled, and CUDA-enabled
  build trees. It now
  also includes an exact rank-deficient singular-degenerate slice for standard
  16-row, boxed 16-row, and coupled friction-index 6-contact packets, plus 27
  matching benchmark rows verified in default, SIMD-enabled, and CUDA-enabled
  build trees. It now also includes a larger exact rank-deficient
  singular-degenerate slice for standard 32-row, boxed 32-row, and coupled
  friction-index 8-contact packets, plus 27 matching benchmark rows verified
  in default, SIMD-enabled, and CUDA-enabled build trees. It now also includes
  a stress exact rank-deficient singular-degenerate slice for standard 64-row,
  boxed 64-row, and coupled friction-index 12-contact packets, plus 27
  matching benchmark rows verified in default, SIMD-enabled, and CUDA-enabled
  build trees. It now also includes an extreme exact rank-deficient
  singular-degenerate slice for standard 128-row, boxed 128-row, and coupled
  friction-index 16-/24-/32-/48-/64-contact packets, plus 39 matching benchmark rows verified
  in default, SIMD-enabled, and CUDA-enabled build trees. It also includes 48
  exact rank-deficient singular-degenerate friction-index batch rows verified
  in default, SIMD-enabled, and CUDA-enabled build trees. It also includes 192
  exact rank-deficient singular-degenerate standard/boxed batch rows verified
  in default, SIMD-enabled, and CUDA-enabled build trees. It now also adds 49 larger active-set transition benchmark rows
  for standard 32-row, boxed 32-row, and coupled friction-index 8-contact
  packets, verified in default, SIMD-enabled, and CUDA-enabled build trees. It
  now also adds a stress active-set transition slice for standard 64-row, boxed
  64-row, and coupled friction-index 12-contact packets, plus 49 matching
  benchmark rows verified in default, SIMD-enabled, and CUDA-enabled build
  trees. It now also includes an extreme active-set transition slice for
  standard 128-row, boxed 128-row, and coupled friction-index 16-contact
  packets, plus 49 matching benchmark rows verified in default, SIMD-enabled,
  and CUDA-enabled build trees. It now also includes a production active-set
  transition slice for stronger-coupled 24-contact, 72-row, 32-contact,
  96-row, 48-contact, 144-row, and 64-contact, 192-row friction-index packets,
  plus 64 matching
  benchmark rows verified in default, SIMD-enabled, and CUDA-enabled build
  trees. It now also includes 422 serial and DART 7 `ParallelExecutor` batch
  rows over the standard 32/64/128-row, boxed 32/64/128-row, and coupled
  friction-index 8-/12-/16-/24-/32-/48-/64-contact active-set packets, verified in
  default, SIMD-enabled, and CUDA-enabled build trees.

## Pivoting Methods

| Method                       | Background source                                                   | Implementation evidence                                                                                                        | Manifest support | Correctness evidence                                                                                                                                                   | Benchmark evidence                                                                                                                                                                                                                                                                                                                                                                                                                | Remaining DART 7 gap                                                                                                                   |
| ---------------------------- | ------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ | ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| Dantzig principal pivoting   | `02_overview.md`, `03_pivoting-methods.md`, `07_selection-guide.md` | `dart/math/lcp/pivoting/dantzig_solver.hpp` exported by `all.hpp`; legacy low-level Dantzig remains exported for compatibility | S, B, F          | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures; focused pivoting solver coverage passes Dantzig matrix and solver tests | Manifest registers `BM_LcpCompare/Standard/Dantzig/*`, `BM_LcpCompare/Boxed/Dantzig/*`, and `BM_LcpCompare/FrictionIndex/Dantzig/*`; focused `BM_LcpPivotingScaleSweep` rows cover standard, boxed, and friction-index Dantzig fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactNormalStandardSweep/Dantzig/*` rows provide an exact baseline on DART 7 contact-normal standard subproblems | Need broader DART 7 end-to-end contact evidence and direct backend execution evidence beyond build-state counters                      |
| Lemke complementary pivoting | `02_overview.md`, `03_pivoting-methods.md`, `07_selection-guide.md` | `dart/math/lcp/pivoting/lemke_solver.hpp` exported by `all.hpp`                                                                | S                | Smoke manifest checks; comparison harness covers standard fixtures; focused pivoting solver coverage passes Lemke solver tests                                         | Manifest registers `BM_LcpCompare/Standard/Lemke/*`; focused `BM_LcpPivotingScaleSweep` rows cover 8-row and 16-row standard Lemke fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactNormalStandardSweep/Lemke/*` rows cover DART 7 contact-normal standard subproblems in those build trees                                                                                                 | Native boxed and friction-index support is not claimed by the manifest; still needs full-contact and direct backend execution evidence |
| Baraff incremental pivoting  | `02_overview.md`, `03_pivoting-methods.md`, `07_selection-guide.md` | `dart/math/lcp/pivoting/baraff_solver.hpp` exported by `all.hpp`                                                               | S                | Smoke manifest checks; comparison harness covers standard fixtures; focused pivoting solver coverage passes Baraff solver tests                                        | Manifest registers `BM_LcpCompare/Standard/Baraff/*`; focused `BM_LcpPivotingScaleSweep` rows cover 8-row and 16-row standard Baraff fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactNormalStandardSweep/Baraff/*` rows cover DART 7 contact-normal standard subproblems in those build trees                                                                                              | Background docs still list incremental factorization as future work; need full-contact and direct backend execution evidence           |
| Direct 2D/3D enumeration     | `02_overview.md`, `03_pivoting-methods.md`, `07_selection-guide.md` | `dart/math/lcp/pivoting/direct_solver.hpp` exported by `all.hpp`                                                               | S                | Smoke manifest checks; comparison harness covers tiny standard fixtures; focused pivoting solver coverage passes Direct solver tests                                   | Manifest registers only dimensions 2 and 3 because larger direct solves fall back to Dantzig; focused `BM_LcpPivotingScaleSweep` rows cover Direct 2-row and 3-row standard fixtures with no-fallback counters in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactNormalStandardSweep/Direct/*` rows cover four DART 7 contact-normal 1-, 2-, and 3-row subproblems with no-fallback counters          | Keep benchmark reporting limited to actual direct enumeration; need full-contact and direct backend execution evidence                 |

## Projection Methods

| Method                         | Background source                                                     | Implementation evidence                                                                                                                                                                                                                                  | Manifest support           | Correctness evidence                                                                                                                                                                                                                                                                                                                          | Benchmark evidence                                                                                                                                                                                                                                                                                                                   | Remaining DART 7 gap                                                                                                                                                                              |
| ------------------------------ | --------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Projected Jacobi               | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/jacobi_solver.hpp` exported by `all.hpp`; optional `JacobiSolver::Parameters::workerThreads` CPU update path; experimental CUDA fixed-iteration batch path in `simulation/experimental/compute/cuda/lcp_jacobi_batch_cuda.cuh` | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures; generated coverage includes 4-worker standard 128-row known-solution case; CUDA unit test covers standard, boxed, friction-index, grouped world-contact, coupled stack-contact, articulated unified-contact, and mixed grouped contact batches | Manifest registers all three benchmark families plus focused serial-vs-worker Jacobi threading rows for dense 128/512-row and banded 512/1024/2048-row standard packets, CUDA standard/boxed/friction-index batch rows, and CUDA grouped contact batch rows                                                                          | Threaded CPU rows currently show correctness/comparison evidence, not speedup; Jacobi's CUDA batch path remains fixed-iteration only                                                              |
| PGS                            | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/pgs_solver.hpp` exported by `all.hpp`; experimental CUDA fixed-iteration batch path in `simulation/experimental/compute/cuda/lcp_jacobi_batch_cuda.cuh`                                                                        | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures; CUDA unit test covers standard, boxed, friction-index, grouped world-contact, coupled stack-contact, articulated unified-contact, and mixed grouped contact PGS batches                                                                        | Manifest registers all three benchmark families; scaled PGS benchmark remains; CUDA standard/boxed/friction-index batch rows and CUDA grouped contact batch rows                                                                                                                                                                     | Need broader end-to-end contact evidence and broader backend-specific packets                                                                                                                     |
| PSOR                           | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | Implemented through `PgsSolver` relaxation via `LcpOptions::relaxation`                                                                                                                                                                                  | Same solver support as PGS | Covered by PGS path and solver-specific tests that configure relaxation                                                                                                                                                                                                                                                                       | Focused `BM_LcpPgsRelaxationSweep` rows cover standard, boxed, and friction-index fixtures at relaxation 0.5, 1.0, and 1.3 in default, SIMD-enabled, and CUDA-enabled build trees; CUDA-enabled rows are CPU solver rows, not CUDA LCP kernel execution                                                                              | Need broader end-to-end contact evidence and workload-specific relaxation tuning before making general PSOR performance claims                                                                    |
| Symmetric PSOR                 | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/symmetric_psor_solver.hpp` exported by `all.hpp`                                                                                                                                                                               | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures; solver-specific tests cover valid solves and invalid relaxation                                                                                                                                                                                | Manifest registers all three benchmark families; focused `BM_LcpSymmetricPsorRelaxationSweep` rows cover standard, boxed, and friction-index fixtures at relaxation 0.5, 1.0, and 1.3 in default, SIMD-enabled, and CUDA-enabled build trees                                                                                         | Need broader end-to-end contact evidence and workload-specific relaxation tuning before making general symmetric PSOR performance claims                                                          |
| Blocked Gauss-Seidel           | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/bgs_solver.hpp` exported by `all.hpp`                                                                                                                                                                                          | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, friction-index, and explicit-block fixtures; DART 7 world-contact tests cover `findex`-derived non-contiguous contact blocks and explicit split rejection; focused block-solver coverage passes BGS tests                                                                   | Manifest registers all three benchmark families; focused world-contact, stack-contact, and serial/parallel batch rows pass with `contract_ok=1`; focused `BM_LcpBlockPartitionSweep` rows cover full-block, 3-row block, auto `findex`, and explicit contact-block partitions in default, SIMD-enabled, and CUDA-enabled build trees | Need broader contact-pipeline performance/backend evidence before claiming solver-specific speedups; CUDA-enabled block-partition rows are CPU solver rows, not CUDA LCP kernel execution         |
| NNCG                           | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/nncg_solver.hpp` exported by `all.hpp`                                                                                                                                                                                         | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures                                                                                                                                                                                                                                                 | Manifest registers all three benchmark families; focused `BM_LcpNncgPgsIterationsSweep` rows cover 1, 2, and 5 PGS preconditioner iterations on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees                                                                                  | Still needs large generated, end-to-end contact, and direct backend execution evidence; CUDA-enabled PGS-preconditioner rows are CPU solver rows, not CUDA LCP kernel execution                   |
| Subspace minimization / PGS-SM | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/subspace_minimization_solver.hpp` exported by `all.hpp`                                                                                                                                                                        | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures                                                                                                                                                                                                                                                 | Manifest registers all three benchmark families; focused `BM_LcpSubspaceMinimizationPgsIterationsSweep` rows cover 1, 3, and 5 PGS active-set-estimation iterations on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees                                                           | Still needs broader size-grid, end-to-end contact, and direct backend execution evidence; CUDA-enabled PGS-iteration rows are CPU solver rows, not CUDA LCP kernel execution                      |
| Red-Black Gauss-Seidel         | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/red_black_gauss_seidel_solver.hpp` exported by `all.hpp`; uses an even/odd two-color partition                                                                                                                                 | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures; solver-specific tests cover invalid relaxation and relaxed friction solves                                                                                                                                                                     | Manifest registers all three benchmark families; focused `BM_LcpRedBlackGaussSeidelRelaxationSweep` rows cover standard, boxed, and friction-index fixtures at relaxation 0.5, 1.0, and 1.3 with two-color counters in default, SIMD-enabled, and CUDA-enabled build trees                                                           | The docs describe parallel-style updates, but current evidence is two-color CPU solver execution only; no solver-internal threaded Red-Black path is implemented                                  |
| Blocked Jacobi                 | `02_overview.md`, `06_other-methods.md`, `07_selection-guide.md`      | `dart/math/lcp/projection/blocked_jacobi_solver.hpp` exported by `all.hpp`                                                                                                                                                                               | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, friction-index, and explicit-block fixtures; DART 7 world-contact tests cover `findex`-derived non-contiguous contact blocks and explicit split rejection; focused block-solver coverage passes Blocked Jacobi tests                                                        | Manifest registers all three benchmark families; focused world-contact, stack-contact, and serial/parallel batch rows pass with `contract_ok=1`; focused `BM_LcpBlockPartitionSweep` rows cover full-block, 3-row block, auto `findex`, and explicit contact-block partitions in default, SIMD-enabled, and CUDA-enabled build trees | Background docs place this in both projection/other context; need implemented parallel/backend execution before claiming parallel speedups; CUDA-enabled block-partition rows are CPU solver rows |
| APGD                           | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/apgd_solver.hpp` exported by `all.hpp`                                                                                                                                                                                         | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures                                                                                                                                                                                                                                                 | Manifest registers all three benchmark families; focused `BM_LcpApgdRestartSweep` rows cover adaptive restart every iteration, adaptive restart every 5 iterations, and no restart on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees                                            | Still needs end-to-end contact evidence and direct backend execution evidence; CUDA-enabled restart rows are CPU solver rows, not CUDA LCP kernel execution                                       |
| TGS                            | `02_overview.md`, `04_projection-methods.md`, `07_selection-guide.md` | `dart/math/lcp/projection/tgs_solver.hpp` exported by `all.hpp`                                                                                                                                                                                          | S, B, F                    | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures                                                                                                                                                                                                                                                 | Manifest registers all three benchmark families; focused `BM_LcpTgsIterationBudgetSweep` rows cover 10-, 50-, and 100-iteration budgets on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees                                                                                       | Still needs end-to-end contact evidence and direct backend execution evidence; CUDA-enabled iteration-budget rows are CPU solver rows, not CUDA LCP kernel execution                              |

## Newton Methods

| Method                              | Background source                                                 | Implementation evidence                                                                     | Manifest support | Correctness evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Benchmark evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Remaining DART 7 gap                                                                                                                                                                                                                                                                    |
| ----------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------------------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Minimum Map Newton                  | `02_overview.md`, `05_newton-methods.md`, `07_selection-guide.md` | `dart/math/lcp/newton/minimum_map_newton_solver.hpp` exported by `all.hpp`                  | S                | Smoke manifest checks; comparison harness covers standard fixtures; focused unit coverage proves the opt-in PGS and gradient-descent warm starts reduce the minimum-map merit before Newton line search                                                                                                                                                                                                                                                                                                 | Manifest registers `BM_LcpCompare/Standard/MinimumMapNewton/*`; warm-start rows compare no seed, PGS, gradient descent, and PGS-then-gradient on standard active-set packets, including serial and `ParallelExecutor` batch rows; focused `BM_LcpContactNormalStandardSweep/MinimumMapNewton/*` rows cover DART 7 contact-normal standard subproblems in default, SIMD-enabled, and CUDA-enabled build trees                                                                                                                                                                                                                                                                                                                                                                                                           | Manifest does not claim native boxed or friction-index support; need full-contact and backend evidence                                                                                                                                                                                  |
| Fischer-Burmeister Newton           | `02_overview.md`, `05_newton-methods.md`, `07_selection-guide.md` | `dart/math/lcp/newton/fischer_burmeister_newton_solver.hpp` exported by `all.hpp`           | S                | Smoke manifest checks; comparison harness covers standard fixtures; focused unit coverage proves the opt-in PGS and gradient-descent warm starts reduce the FB merit before Newton line search                                                                                                                                                                                                                                                                                                          | Manifest registers `BM_LcpCompare/Standard/FischerBurmeisterNewton/*`; warm-start rows compare no seed, PGS, gradient descent, and PGS-then-gradient on standard active-set packets, including serial and `ParallelExecutor` batch rows; focused `BM_LcpContactNormalStandardSweep/FischerBurmeisterNewton/*` rows cover DART 7 contact-normal standard subproblems in default, SIMD-enabled, and CUDA-enabled build trees                                                                                                                                                                                                                                                                                                                                                                                             | Manifest does not claim native boxed or friction-index support; need full-contact and backend evidence                                                                                                                                                                                  |
| Penalized Fischer-Burmeister Newton | `02_overview.md`, `05_newton-methods.md`, `07_selection-guide.md` | `dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.hpp` exported by `all.hpp` | S                | Smoke manifest checks; comparison harness covers standard fixtures with custom lambda; focused unit coverage proves the opt-in PGS and gradient-descent warm starts reduce the penalized FB merit before Newton line search                                                                                                                                                                                                                                                                             | Manifest registers `BM_LcpCompare/Standard/PenalizedFischerBurmeisterNewton/*`; warm-start rows compare no seed, PGS, gradient descent, and PGS-then-gradient on standard active-set packets, including serial and `ParallelExecutor` batch rows; focused `BM_LcpContactNormalStandardSweep/PenalizedFischerBurmeisterNewton/*` rows cover DART 7 contact-normal standard subproblems in default, SIMD-enabled, and CUDA-enabled build trees                                                                                                                                                                                                                                                                                                                                                                           | Manifest does not claim native boxed or friction-index support; need parameter-sweep, full-contact, and backend evidence                                                                                                                                                                |
| Boxed semi-smooth Newton            | `02_overview.md`, `05_newton-methods.md`, `07_selection-guide.md` | `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.hpp` exported by `all.hpp`            | S, B, F          | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures; smoke tests include line-search failure reporting; generated coverage passes coupled mildly ill-conditioned 2-contact and 4-contact friction-index cases after the moving-bound Jacobian fix; generated coverage now also passes 1x-/4x-coupled mildly ill-conditioned friction-index packets through 24 contacts, 8x-coupled packets through 32 contacts, and the 16x-coupled slice through 64 contacts | Manifest registers all three benchmark families; focused `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep` rows cover default line search, expanded line-search step budget, and gentle step-reduction settings on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactSolverComparisonSweep/BoxedSemiSmoothNewton/*` rows cover separated world-contact, coupled stack-contact, and articulated unified-contact fixtures in those build trees; focused `BM_LcpMildIllConditioned/*/BoxedSemiSmoothNewton` single rows cover coupled 1x/4x/8x/16x mildly ill-conditioned friction-index packets; batch rows cover coupled 1x/4x/8x packets plus 16x packets at 6/8/12/16/32/48 contacts while 16x 24/64 remain excluded after failed contract trials | Still needs batch contract evidence for the 16x-coupled 24-/64-contact packets, longer-running/denser end-to-end contact, and direct backend execution evidence; CUDA-enabled line-search, contact-comparison, and coupled mild rows are CPU solver rows, not CUDA LCP kernel execution |

## Other Methods

| Method            | Background source                                                | Implementation evidence                                                  | Manifest support | Correctness evidence                                                                                                 | Benchmark evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Remaining DART 7 gap                                                                                                                                                                                                      |
| ----------------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------ | ---------------- | -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Interior Point    | `02_overview.md`, `06_other-methods.md`, `07_selection-guide.md` | `dart/math/lcp/other/interior_point_solver.hpp` exported by `all.hpp`    | S                | Smoke manifest checks; comparison harness covers standard fixtures                                                   | Manifest registers `BM_LcpCompare/Standard/InteriorPoint/*`; focused `BM_LcpInteriorPointPathSweep` rows cover dense SPD, banded SPD, mildly ill-conditioned SPD, and near-singular SPD standard-LCP fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactNormalStandardSweep/InteriorPoint/*` rows cover DART 7 contact-normal standard subproblems in those build trees                                                                                                                                                                             | Manifest does not claim native boxed or friction-index support; need full-contact and direct backend execution evidence                                                                                                   |
| MPRGP             | `02_overview.md`, `06_other-methods.md`, `07_selection-guide.md` | `dart/math/lcp/other/mprgp_solver.hpp` exported by `all.hpp`             | S                | Smoke manifest checks; comparison harness covers standard fixtures                                                   | Manifest registers `BM_LcpCompare/Standard/MPRGP/*`; focused `BM_LcpMprgpSpdCheckSweep` rows cover dense SPD, banded SPD, mildly ill-conditioned SPD, and near-singular SPD standard-LCP fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactNormalStandardSweep/MPRGP/*` rows cover DART 7 contact-normal standard subproblems in those build trees                                                                                                                                                                                                 | Docs narrow this to standard SPD LCPs; need full-contact and direct backend execution evidence                                                                                                                            |
| Shock Propagation | `02_overview.md`, `06_other-methods.md`, `07_selection-guide.md` | `dart/math/lcp/other/shock_propagation_solver.hpp` exported by `all.hpp` | S, B, F          | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures                        | Manifest registers all three benchmark families; focused `BM_LcpShockPropagationLayerSweep` rows compare single-layer, two-layer, and serial schedules on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees                                                                                                                                                                                                                                                                                                                           | Need end-to-end stacked-contact scenes that prove layered contact behavior against DART 7 simulation invariants; CUDA-enabled layer rows are CPU solver rows, not CUDA LCP kernel execution                               |
| Staggering        | `02_overview.md`, `06_other-methods.md`, `07_selection-guide.md` | `dart/math/lcp/other/staggering_solver.hpp` exported by `all.hpp`        | S, B, F          | Smoke manifest checks; comparison harness covers standard, boxed, friction-index, and normal/friction pass behavior  | Manifest registers all three benchmark families; focused `BM_LcpStaggeringContactPipelineSweep` rows cover separated world contacts, coupled vertical stacks, and articulated unified ground/rigid-impact/cross-link contact fixtures in default, SIMD-enabled, and CUDA-enabled build trees                                                                                                                                                                                                                                                                                            | Need longer-running and denser contact-pipeline scenes plus direct backend execution evidence; CUDA-enabled contact-pipeline rows are CPU solver rows, not CUDA LCP kernel execution                                      |
| ADMM              | `02_overview.md`, `06_other-methods.md`, `07_selection-guide.md` | `dart/math/lcp/other/admm_solver.hpp` exported by `all.hpp`              | S, B, F          | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures                        | Manifest registers all three benchmark families; focused `BM_LcpAdmmRhoSweep` rows cover fixed and adaptive rho policies at `rhoInit` 0.5, 1.0, and 4.0 on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactSolverComparisonSweep/Admm/*` rows cover separated world-contact, coupled stack-contact, and articulated unified-contact fixtures in those build trees; focused `BM_LcpMildIllConditioned/ExtremeCoupledFrictionIndex*/Admm` rows cover 16x-coupled mildly ill-conditioned friction-index packets | Still needs longer-running/denser end-to-end contact and direct backend execution evidence; CUDA-enabled rho-sweep, contact-comparison, and extreme-coupling rows are CPU solver rows, not CUDA LCP kernel execution      |
| SAP               | `02_overview.md`, `06_other-methods.md`, `07_selection-guide.md` | `dart/math/lcp/other/sap_solver.hpp` exported by `all.hpp`               | S, B, F          | Smoke manifest checks; comparison harness covers standard, boxed, and friction-index fixtures with custom parameters | Manifest registers all three benchmark families; focused `BM_LcpSapRegularizationSweep` rows cover regularization values `1e-6`, `1e-5`, and `1e-4` on standard, boxed, and friction-index fixtures in default, SIMD-enabled, and CUDA-enabled build trees; focused `BM_LcpContactSolverComparisonSweep/Sap/*` rows cover separated world-contact, coupled stack-contact, and articulated unified-contact fixtures in those build trees; focused `BM_LcpMildIllConditioned/ExtremeCoupledFrictionIndex*/Sap` rows cover 16x-coupled mildly ill-conditioned friction-index packets       | Still needs longer-running/denser end-to-end contact and direct backend execution evidence; CUDA-enabled regularization, contact-comparison, and extreme-coupling rows are CPU solver rows, not CUDA LCP kernel execution |

## Current Verification Evidence

The current local evidence for this task is:

- `pixi run lint` completed successfully after the manifest and benchmark
  changes, after this audit was added, and after the generated coverage and
  dev-task evidence updates.
- `cmake --build build/default/cpp/Release --parallel 5` completed
  successfully for `BM_LCP_COMPARE`,
  `UNIT_math_lcp_math_lcp_all_solvers_smoke`, and
  `UNIT_math_lcp_math_lcp_lcp_comparison_harness`.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke` passed 10 focused smoke and
  advanced-boxed tests.
- `UNIT_math_lcp_math_lcp_lcp_comparison_harness` passed 44
  `LcpComparisonHarness.*` tests.
- `UNIT_math_lcp_math_lcp_lcp_newton_solvers` passed 50 tests after adding
  focused standard-LCP PGS and gradient-descent warm-start merit-reduction
  coverage for Minimum Map, Fischer-Burmeister, and Penalized
  Fischer-Burmeister Newton.
- `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` passed 162 tests after
  the Newton warm-start updates.
- `UNIT_math_lcp_math_lcp_lcp_generated_coverage` passed 21
  `LcpGeneratedCoverage.*` tests in the default, SIMD-enabled, and
  CUDA-enabled build trees, covering
  manifest-supported solvers on
  deterministic known-solution standard, boxed, and friction-index generated
  cases, batch-shaped state reuse, pivoting-solver near-singular standard
  cases, near-singular boxed cases, coupled well-conditioned and mildly
  ill-conditioned friction-index cases, active-set transition cases, and
  invalid-problem rejection with diagnostic messages. The generated size grid
  currently reaches standard
  16-row, boxed 12-row, friction-index 8-contact, coupled friction-index
  6-contact, and mildly ill-conditioned standard/boxed 8-row cases across all
  supporting solvers, plus scoped scalable-solver slices for standard 32-row
  and 64-row, boxed 32-row, friction-index 16-contact, coupled friction-index
  8-contact, production-scale standard 128-row, boxed 64-row, friction-index
  24-contact, coupled friction-index 12-contact, near-singular standard 8-row,
  boxed 8-row, coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, and
  48-, and 64-contact,
  exact rank-deficient
  singular-degenerate standard 16-row, boxed 16-row, and coupled
  friction-index 6-contact known-solution cases, and larger exact
  rank-deficient singular-degenerate standard 32-row, boxed 32-row, and coupled
  friction-index 8-contact known-solution cases, and stress exact
  rank-deficient singular-degenerate standard 64-row, boxed 64-row, and coupled
  friction-index 12-contact known-solution cases, and extreme exact
  rank-deficient singular-degenerate standard 128-row, boxed 128-row, and
  coupled friction-index 16-contact known-solution cases. The active-set
  transition grid now also includes scoped scalable-solver cases for standard
  32-row, 64-row, and 128-row, boxed 32-row, 64-row, and 128-row, and coupled
  friction-index 8-contact, 12-contact, and 16-contact packets, plus a
  stronger-coupled production coupled friction-index 24-contact packet and a
  stronger 32-contact packet. The new larger
  mildly ill-conditioned slice covers standard 32-row and 64-row, boxed 16-row
  and 32-row, friction-index 8-contact, and coupled friction-index 6-, 8-, 12-,
  16-, and 24-contact packets, plus 4x-coupled 6-/8-/12-/16-/24-contact
  packets and 8x-coupled 6-/8-/12-/16-/24-/32-contact packets with Boxed
  Semi-Smooth Newton included across those coupled rows,
  over a scoped solver set. `MPRGP` is intentionally
  excluded from this stricter known-solution slice because a focused standard
  32-row trial satisfied the
  LCP contract but missed the selected expected-solution tolerance.
  The singular-degenerate standard packet covers 21 standard-capable solvers
  and intentionally excludes `Direct` because 16-row solves use the Dantzig
  fallback and excludes `PenalizedFischerBurmeisterNewton` after a contract
  trial. The singular-degenerate boxed and friction-index packets are scoped to
  `Admm`, `Sap`, and `BoxedSemiSmoothNewton` after broader trials exposed
  contract failures for the other tried boxed/findex paths.
  Trial evidence keeps the near-singular slice narrow: Lemke produced a valid
  complementary solution but not the selected generated solution for the 8-row
  singular standard case, and boxed semi-smooth Newton failed line search on
  the near-singular coupled friction-index cases.
- `BM_LCP_COMPARE` now lists 55 `BM_LcpActiveSetTransition` rows and the
  focused default benchmark run passed all of them with `contract_ok=1`. These
  rows compare standard 16-row, boxed 16-row, and coupled friction-index
  6-contact active-set transition packets across manifest-supporting solvers.
  `DirectSolver` is intentionally omitted from the standard rows because the
  16-row direct path dispatches to Dantzig fallback. Focused
  standard/boxed/friction-index rows also passed in the SIMD build with
  `build_simd_enabled=1` and in the CUDA-enabled build with
  `build_cuda_enabled=1`.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpLargerActiveSetTransition/' | wc -l`
  reported 49 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpLargerActiveSetTransition' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows compare standard 32-row, boxed 32-row,
  and coupled friction-index 8-contact active-set transition packets across the
  scoped scalable solver set already proven by generated correctness coverage.
  The rows report `active_set_transition=1`,
  `larger_active_set_transition=1`, backend build-state counters,
  `contact_count=8` for the friction-index packet, and `coupled=1` for the
  coupled packet. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpStressActiveSetTransition/' | wc -l`
  reported 49 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpStressActiveSetTransition' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows compare standard 64-row, boxed 64-row,
  and coupled friction-index 12-contact active-set transition packets across
  the scoped scalable solver set already proven by generated correctness
  coverage. The rows report `active_set_transition=1`,
  `stress_active_set_transition=1`, backend build-state counters,
  `contact_count=12` for the friction-index packet, and `coupled=1` for the
  coupled packet. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpExtremeActiveSetTransition/' | wc -l`
  reported 49 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpExtreme(ActiveSetTransition|SingularDegenerate)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all extreme active-set rows with `contract_ok=1` in the default,
  SIMD-enabled, and CUDA-enabled build trees. These rows compare standard
  128-row, boxed 128-row, and coupled
  friction-index 16-contact active-set transition packets across the scoped
  scalable solver set already proven by generated correctness coverage. The
  rows report `active_set_transition=1`,
  `extreme_active_set_transition=1`, backend build-state counters,
  `contact_count=16` for the friction-index packet, and `coupled=1` for the
  coupled packet. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpProductionActiveSetTransition/' | wc -l`
  reported 64 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpProductionActiveSetTransition' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all production active-set rows with `contract_ok=1` in the default,
  SIMD-enabled, and CUDA-enabled build trees. These rows compare all 16
  friction-index-capable manifest solvers on stronger-coupled 24-contact,
  72-row, 32-contact, 96-row, 48-contact, 144-row, and 64-contact, 192-row friction-index
  active-set transition packets.
  The rows report
  `active_set_transition=1`, `production_active_set_transition=1`,
  `contact_count=24/32/48/64`, `problem_size=72/96/144/192`,
  `coupling_scale=2/4/8/16`, backend
  build-state counters, and `coupled=1`. The CUDA-enabled rows are CPU solver
  rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpProductionActiveSetTransitionBatch' | wc -l`
  reported 422 rows, and JSON benchmark checks for
  `BM_LcpProductionActiveSetTransitionBatch` reported 422 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows compare manifest-supporting solvers on batch-size-4 serial and
  DART 7 `ParallelExecutor` runs over standard 32/64/128-row, boxed
  32/64/128-row, and coupled friction-index 8-/12-/16-/24-/32-/48-/64-contact
  active-set packets. The rows
  report `production_active_set_transition_batch=1`, `batch_size=4`,
  `problem_size=24/32/36/48/64/72/96/128/144`,
  `total_problem_size=96/128/144/192/256/288/384/512/576/768`, backend build-state
  counters, and parallel execution counters on the `ParallelExecutor` rows.
  Friction-index rows also report `contact_count=8/12/16/24/32/48/64`,
  `total_contact_count=32/48/64/96/128/192/256`, and `coupling_scale=1/2/4/8/16`.
  The CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not
  CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditioned/' | wc -l`
  reported 308 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpMildIllConditioned' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 32-row, boxed 16-row,
  friction-index 8-contact, and coupled friction-index 6-, 8-, 12-, 16-, and
  24-contact larger mildly ill-conditioned packets, plus 4x-coupled
  6-/8-/12-/16-/24-contact packets and 8x-coupled
  6-/8-/12-/16-/24-/32-contact packets with Boxed Semi-Smooth Newton included
  across those coupled rows, plus
  ADMM/SAP/Boxed Semi-Smooth Newton single-problem 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-contact packets over the scoped solver set. The
  rows report `mildly_ill_conditioned=1`, backend build-state
  counters, `contact_count` for friction-index packets, `coupled=1` for the
  coupled packets, and `coupling_scale=1`, `4`, `8`, or `16` for the
  stronger/extreme-coupled packets. The `BoxedSemiSmoothNewton` coupled
  single-problem rows contribute 24 rows over coupling scales `1/4/8/16` and
  contact counts `6/8/12/16/24/32/48/64`.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditionedBatch' | wc -l`
  reported 612 rows, and JSON benchmark checks for
  `BM_LcpMildIllConditionedBatch(Serial|Parallel)` reported 612 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows compare the batch-scoped solver set over batch-size-4 serial and
  DART 7 `ParallelExecutor` runs on standard
  32-row, boxed 16-row, friction-index 8-contact, coupled friction-index
  6-/8-/12-/16-/24-contact, 4x-coupled 6-/8-/12-/16-/24-contact,
  8x-coupled 6-/8-/12-/16-/24-/32-contact with Boxed Semi-Smooth Newton
  included across those non-16x coupled rows and the 16x-coupled
  6-/8-/12-/16-/32-/48-contact rows; the 16x-coupled 24-/64-contact rows remain
  scoped to ADMM/SAP. The rows report
  `mildly_ill_conditioned_batch=1`, `batch_size=4`, problem sizes
  `16/18/24/32/36/48/72/96/144/192`, total problem sizes
  `64/72/96/128/144/192/288/384/576/768`, backend build-state counters, and
  parallel execution counters on the `ParallelExecutor` rows. The
  `BoxedSemiSmoothNewton` coupled batch rows contribute 44 serial and parallel
  rows over coupling scales `1/4/8/16` and contact counts
  `6/8/12/16/24/32/48`; the 16x-coupled 24-/64-contact batch rows remain
  ADMM/SAP-only after Boxed Semi-Smooth Newton contract trials failed. The
  friction-index rows additionally report `contact_count=6/8/12/16/24/32/48/64`
  and `total_contact_count=24/32/48/64/96/128/192/256`; coupled rows report
  `coupled=1` and `coupling_scale=1/4/8/16`. The CUDA-enabled rows are CPU
  solver batch rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNearSingular/' | wc -l`
  reported 23 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpNearSingular' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover near-singular standard 8-row,
  boxed 8-row, and coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, and
  48-, and 64-contact packets over the scoped robust solver set. The rows report
  `near_singular=1`, backend build-state counters, contact counts
  `3/6/9/12/16/24/32/48/64`, problem sizes `9/18/27/36/48/72/96/144/192`, and
  `coupled=1` for the coupled packets. The CUDA-enabled rows are CPU solver
  rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNearSingularBatch' | wc -l`
  reported 46 rows, and JSON benchmark checks for
  `BM_LcpNearSingularBatch(Serial|Parallel)` reported 46 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows compare `Dantzig` and `Baraff` on standard 8-row packets,
  `Dantzig`, `ShockPropagation`, and `BoxedSemiSmoothNewton` on boxed 8-row
  packets, and `Dantzig` and `ShockPropagation` on coupled friction-index 3-,
  6-, 9-, 12-, 16-, 24-, 32-, 48-, and 64-contact packets. They cover batch-size-4
  serial and DART 7 `ParallelExecutor` runs on near-singular packets. The rows
  report `near_singular_batch=1`, `batch_size=4`, problem sizes
  `8/9/18/27/36/48/72/96/144/192`, total problem sizes
  `32/36/72/108/144/192/288/384/576/768`, backend build-state counters, and
  parallel execution counters on the `ParallelExecutor` rows. The
  friction-index rows additionally report
  `contact_count=3/6/9/12/16/24/32/48/64`,
  `total_contact_count=12/24/36/48/64/96/128/192/256`,
  and `coupled=1`. The CUDA-enabled rows are CPU solver batch rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSingularDegenerate/' | wc -l`
  reported 27 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpSingularDegenerate' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover exact rank-deficient standard
  16-row, boxed 16-row, and coupled friction-index 6-contact packets over the
  same solver scope proven by generated coverage. The rows report
  `singular_degenerate=1`, `rank_deficient=1`, backend build-state counters,
  `contact_count` for friction-index packets, and `coupled=1` for the coupled
  packet.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpLargerSingularDegenerate/' | wc -l`
  reported 27 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpLargerSingularDegenerate' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover exact rank-deficient standard
  32-row, boxed 32-row, and coupled friction-index 8-contact packets over the
  same scoped robust solver set as the generated larger singular-degenerate
  correctness slice. The rows report `singular_degenerate=1`,
  `rank_deficient=1`, backend build-state counters, `contact_count=8`, and
  `coupled=1` for the coupled packet. The CUDA-enabled rows are CPU solver rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpStressSingularDegenerate/' | wc -l`
  reported 27 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpStressSingularDegenerate' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover exact rank-deficient standard
  64-row, boxed 64-row, and coupled friction-index 12-contact packets over the
  same scoped robust solver set as the generated stress singular-degenerate
  correctness slice. The rows report `singular_degenerate=1`,
  `rank_deficient=1`, backend build-state counters, `contact_count=12`, and
  `coupled=1` for the coupled packet. The CUDA-enabled rows are CPU solver rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpExtremeSingularDegenerate/' | wc -l`
  reported 39 rows, and
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpExtremeSingularDegenerate/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all extreme singular-degenerate rows with `contract_ok=1` in the default,
  SIMD-enabled, and CUDA-enabled build trees. These rows cover exact
  rank-deficient standard 128-row, boxed
  128-row, and coupled friction-index 16-/24-/32-/48-/64-contact packets over the same scoped
  robust solver set as the generated extreme singular-degenerate correctness
  slice. The rows report `singular_degenerate=1`, `rank_deficient=1`, backend
  build-state counters, contact counts `16/24/32/48/64`, problem sizes `48/72/96/144/192`, and
  `coupled=1` for the coupled packets. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled build,
  not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSingularDegenerateFrictionIndexBatch' | wc -l`
  reported 48 rows, and JSON benchmark checks for
  `BM_LcpSingularDegenerateFrictionIndexBatch(Serial|Parallel)` reported 48
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows compare `Admm`, `Sap`, and
  `BoxedSemiSmoothNewton` over batch-size-4 serial and DART 7
  `ParallelExecutor` runs on exact rank-deficient coupled friction-index 6-,
  8-, 12-, 16-, 24-, 32-, 48-, and 64-contact packets. The rows report
  `singular_degenerate_batch=1`, `rank_deficient=1`, `batch_size=4`,
  `contact_count=6/8/12/16/24/32/48/64`,
  `total_contact_count=24/32/48/64/96/128/192/256`,
  `problem_size=18/24/36/48/72/96/144/192`,
  `total_problem_size=72/96/144/192/288/384/576/768`, backend build-state
  counters, and parallel execution counters on the `ParallelExecutor` rows.
  The CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not
  CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSingularDegenerateStandardBoxedBatch' | wc -l`
  reported 192 rows, and JSON benchmark checks for
  `BM_LcpSingularDegenerateStandardBoxedBatch(Serial|Parallel)` reported 192
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows compare the standard-capable robust solver set on
  standard 16-/32-/64-/128-row packets and `Admm`, `Sap`, and
  `BoxedSemiSmoothNewton` on boxed 16-/32-/64-/128-row packets. They cover
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
  rank-deficient standard and boxed packets. The rows report
  `singular_degenerate_batch=1`,
  `singular_degenerate_standard_boxed_batch=1`, `rank_deficient=1`,
  `batch_size=4`, `problem_size=16/32/64/128`,
  `total_problem_size=64/128/256/512`, backend build-state counters, and
  parallel execution counters on the `ParallelExecutor` rows. The CUDA-enabled
  rows are CPU solver batch rows in a CUDA-enabled build, not CUDA LCP kernel
  execution.
- `LcpGeneratedCoverage.ThreadedJacobiStandardKnownSolution` passed with
  `JacobiSolver::Parameters::workerThreads = 4` on the 128-row deterministic
  standard generated case. This is solver-internal CPU threading correctness
  evidence for Jacobi's update loop, not a performance claim.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpJacobiSolverThreadingBanded_Standard/(512|1024|2048)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed in the default build for all 9 banded Jacobi threading rows, including
  new 2048-row rows with 1, 8, and 16 worker threads. Focused SIMD-enabled and
  CUDA-enabled runs passed the 2048-row rows with `contract_ok=1`,
  `solver_internal_threads=1/8/16`, `matrix_nonzero_entries=10234`, and
  `matrix_density=0.00244`. The CUDA-enabled rows are CPU Jacobi rows in that
  build tree, not CUDA LCP kernel execution.
- `BoxedSemiSmoothNewton` now includes the derivative of moving `findex`
  friction bounds in its natural-residual Jacobian. With that fix,
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage
--gtest_filter='LcpGeneratedCoverage.CoupledFrictionIndexKnownSolutionsAcrossContactCounts'`
  passes the previously failing coupled mildly ill-conditioned 4-contact
  friction-index case across the full manifest-supported friction-index solver
  set. The boxed semi-smooth Newton generated path keeps `MaxIterations`
  disallowed for these coupled cases.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*WorldContactSnapshot*'`
  passed 2 tests,
  including `BoxedLcpContact.WorldContactSnapshotSatisfiesLcpContract`, which
  validates a real DART 7 `World::collide()` single sphere-ground friction
  contact, and
  `BoxedLcpContact.TwoSphereWorldContactSnapshotSatisfiesLcpContract`, which
  validates two separated sphere-ground contacts assembled into one boxed/findex
  LCP snapshot by `detail::solveBoxedLcpContacts`.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.BlockSolversUseFindexContactBlocksOnWorldSnapshot:BoxedLcpContact.BlockSolversRejectExplicitWorldContactFindexSplit'`
  passed 2 tests, proving BGS and Blocked Jacobi solve a real DART 7
  two-contact boxed/findex snapshot through default `findex`-derived
  non-contiguous contact blocks, and reject explicit block partitions that split
  tangent rows from their owning normal row.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorld(Contact|StackContact|ContactBatchSerial|ContactBatchParallel|ContactStressBatchSerial|ContactStressBatchParallel)/FrictionIndex/(BGS|BlockedJacobi)'`
  on the merged default tree produced 22 rows with `bad_contract=0`,
  `bad_problem=0`, 8 stack rows, and 8 serial/parallel batch rows.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpBlockPartitionSweep' | wc -l`
  listed 12 BGS/Blocked Jacobi block-partition rows. Focused JSON runs for
  `BM_LcpBlockPartitionSweep` in default, SIMD-enabled, and CUDA-enabled build
  trees reported 12 rows with zero `contract_ok` failures across standard
  12-row, boxed 12-row, and friction-index 4-contact fixtures; the rows
  recorded full-block, 3-row block, auto `findex`, and explicit contact-block
  partitions, block counts `1/4`, block sizes `3/12`, friction-index
  `contact_count=4`, observed solver `iterations=1/4/5/6/10`, and backend
  build-state counters.
- Focused block-solver unit coverage
  `BlockedJacobiSolverCoverage.*:BgsSolverCoverage.*` passed 15 tests.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.TwoSphereWorldStepMaintainsContactInvariants'`
  passed 1 test, advancing two independent sphere-ground contacts for 200
  boxed-LCP `World::step()` iterations and checking finite state,
  non-penetration, near-rest normal velocity, tangential-speed reduction, and
  static-ground invariants.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.FourSphereWorldStepMaintainsContactInvariants'`
  passed 1 test, advancing four independent sphere-ground contacts for 200
  boxed-LCP `World::step()` iterations and checking finite state, contact
  height, bounded vertical velocity, tangential-speed reduction, and
  static-ground invariants.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.SixteenSphereWorldStepMaintainsContactInvariants'`
  passed 1 test, advancing 16 independent sphere-ground contacts for 200
  boxed-LCP `World::step()` iterations and checking the same finite state,
  contact-height, bounded vertical velocity, tangential-speed reduction, and
  static-ground invariants.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants' --gtest_brief=1`
  passed 1 test, advancing a fixed-base prismatic articulated link in light
  ground contact for 400 boxed-LCP `World::step()` iterations. The test first
  confirms the active contact touches a `comps::Link`, then checks finite
  link height, near-rest ground-contact height, bounded joint velocity, and
  parity with the sequential articulated shortcut. This is fixed-base
  prismatic link-ground evidence, not general articulated robot contact
  coverage.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants:BoxedLcpContact.FourArticulatedPrismaticLinksGroundStepMaintainsInvariants' --gtest_brief=1`
  passed 2 tests, adding a four-link scene that advances four independent
  fixed-base prismatic articulated links in simultaneous ground contact for
  200 boxed-LCP `World::step()` iterations. The multi-link test confirms four
  link-touching contacts, finite state, bounded height error, bounded joint
  velocity, and parity with the sequential articulated shortcut. This broadens
  fixed-base link-ground evidence, but it is still not general articulated
  robot contact coverage.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.CartesianPrismaticChainGroundStepMaintainsInvariants' --gtest_brief=1`
  passed 1 test, adding a connected fixed-base three-axis prismatic Cartesian
  chain scene. The test advances two serial-chain multibodies in simultaneous
  tip-ground contact for 200 boxed-LCP `World::step()` iterations, confirms two
  link-touching contacts and six generalized coordinates, and checks finite
  state, bounded tip height error, bounded joint velocities, bounded planar
  joint speed, and parity with the sequential articulated shortcut. This is
  connected multi-DOF fixed-base articulated contact evidence, not general
  articulated robot contact coverage.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesDynamicRigidBody' --gtest_brief=1`
  passed 1 test, advancing a fixed-base prismatic articulated striker link in
  contact with a dynamic rigid sphere for one boxed-LCP `World::step()`. The
  test confirms the contact touches both a `comps::Link` and a rigid body, then
  checks finite velocities, target motion, striker slowdown, X-momentum
  conservation, and parity with the sequential articulated shortcut. This is
  two-sided articulated link-vs-rigid evidence, not general robot contact
  coverage.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesArticulatedPrismaticLink' --gtest_brief=1`
  passed 1 test, advancing a fixed-base prismatic articulated striker link in
  contact with a prismatic target link owned by a separate multibody for one
  boxed-LCP `World::step()`. The test confirms the contact touches two
  `comps::Link` entities, then checks finite velocities, target motion,
  striker slowdown, nonnegative post-step separation velocity, X-momentum
  conservation, and parity with the sequential cross-multibody articulated
  shortcut. This is cross-multibody link-vs-link evidence, not general robot
  contact coverage.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*SphereStack*'`
  passed 7 tests, validating the boxed/findex LCP contract for 3-sphere,
  4-sphere, and 5-sphere vertical stacks assembled from DART 7
  `World::collide()`, plus the 3-sphere, 4-sphere, and 5-sphere public step
  invariants below. The
  4-sphere snapshot contains 4 contacts and 12 LCP rows; the 5-sphere snapshot
  contains 5 contacts and 15 LCP rows. The snapshot tests also check that the
  normal-contact block has nonzero off-diagonal coupling, so this evidence is
  not just a set of independent contact rows. The 4-sphere public-step path is
  enabled by the boxed-LCP contact solve's timestep-driven Baumgarte velocity
  bias.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.StressSphereStackWorldStepMaintainsContactInvariants'`
  passed 1 test, advancing the same 5-sphere vertical stack through 500 public
  boxed-LCP `World::step()` iterations and checking finite state,
  non-penetration, preserved sphere spacing, near-rest vertical velocity,
  bounded lateral drift, and static-ground invariants.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.SphereStackWorldStepMaintainsContactInvariants'`
  passed 1 test, advancing the same 3-sphere vertical stack through 200 public
  boxed-LCP `World::step()` iterations and checking finite state,
  non-penetration, preserved sphere spacing, near-rest vertical velocity,
  bounded lateral drift, and static-ground invariants.
- `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.LongRunningSphereStackWorldStepMaintainsContactInvariants'`
  passed 1 test, advancing the same 3-sphere vertical stack through 500 public
  boxed-LCP `World::step()` iterations with the same motion invariants.
- The full `test_boxed_lcp_contact --gtest_brief=1` binary passed 45 tests. The
  run still emits the existing `StaticFrictionHoldsSmallPush` degenerate-pivot
  warning, so this should not be counted as clean evidence for
  dense-degenerate multi-contact systems.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContact|BM_LcpWorldContactAssembly' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally. It ran 48 manifest-generated
  `BM_LcpWorldContact/FrictionIndex/<solver>/{1,2,4}` rows, covering all 16
  friction-index-capable solvers on identical boxed/findex LCP snapshots
  assembled from DART 7 `World::collide()` sphere-ground contacts, plus
  `BM_LcpWorldContactAssembly_BoxedLcp/{1,2,4}` rows that rebuild, collide,
  assemble, solve, and validate the boxed-LCP world contact path. All 51 rows
  reported `contract_ok=1`. `ShockPropagation` rows now rely on the solver's
  `findex`-derived contact grouping for non-contiguous world-contact snapshots
  and report one 3-row block per contact. This is simple separated-contact
  benchmark evidence, not articulated, robot-like, or dense-degenerate contact
  evidence.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact|BM_LcpWorldStackContactAssembly' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally. It ran 32 manifest-generated
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{2,3}` rows, covering all 16
  friction-index-capable solvers on DART 7 `World::collide()` snapshots from
  2- and 3-sphere vertical stacks. It also registers 30
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{4,5}` rows for the same
  solver set except `NNCG`; a focused `NNCG` 4-sphere trial reached the
  benchmark cap with `contract_ok=0`, so the NNCG 4-/5-sphere rows are not
  claimed. The matching
  `BM_LcpWorldStackContactAssembly_BoxedLcp/{2,3,4,5}` rows rebuild, collide,
  assemble through `detail::solveBoxedLcpContacts`, solve, and validate the
  boxed-LCP stack contact path. The focused 4-sphere benchmark run
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/4|BM_LcpWorldStackContactAssembly_BoxedLcp/4' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed with `contract_ok=1` for all 16 registered rows. Focused default,
  SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/5$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs also passed with `contract_ok=1` for all 15 registered 5-sphere solver
  rows, with `sphere_count=5`, `contact_count=5`, and `problem_size=15`; the
  CUDA-enabled rows are CPU solver benchmark rows in that build tree, not CUDA
  LCP kernel execution. Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LcpWorldStackContactAssembly_BoxedLcp/5` runs also passed with
  `contract_ok=1`, `sphere_count=5`, `contact_count=5`, and `problem_size=15`.
  These stacks include sphere-ground and sphere-sphere contacts coupled through
  shared dynamic bodies. This is small coupled-stack benchmark evidence, not
  articulated, robot-like, or dense-degenerate contact evidence.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContactBatch(Serial|Parallel)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally. A focused benchmark-list check returned 32 registered
  world-contact batch rows, covering all 16 friction-index-capable solvers on
  serial and DART 7 `ParallelExecutor` rows over the same mixed 5-problem
  world-contact batch. Each row reported `batch_size=5`,
  `total_problem_size=36`, `total_contact_count=12`, `total_body_count=12`, and
  `contract_ok=1`; the parallel rows also reported `profile_enabled=1`,
  `worker_count=20`, and `parallel_units=5`. Observed `max_parallelism`
  reached 5. This is contact-derived batch-processing evidence for separated
  sphere-ground and small coupled-stack snapshots, not CUDA batch execution or
  dense end-to-end contact evidence.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContactStressBatch(Serial|Parallel)/FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed in default, SIMD-enabled, and CUDA-enabled build trees. It runs 30
  registered stress world-contact batch rows for all friction-index-capable
  solvers except `NNCG` over one 7-problem batch: 1/2/4 separated
  sphere-ground snapshots plus 2/3/4/5-sphere vertical-stack snapshots. Each
  row reported `contract_ok=1`, `batch_size=7`, `total_problem_size=63`,
  `total_contact_count=21`, `total_body_count=21`,
  `separated_contact_shape_count=3`, `stack_contact_shape_count=4`, and
  `stress_stack_contact_batch=1`; parallel rows also reported
  `profile_enabled=1`, `worker_count=20`, and `parallel_units=7`. These are
  CPU solver rows in those build trees; the CUDA-enabled rows are not CUDA LCP
  kernel execution.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally in default, SIMD-enabled, and CUDA-enabled build trees for
  `BM_LcpWorldStackStep_BoxedLcp/3/200`,
  `BM_LcpWorldStackStep_BoxedLcp/3/500`, and
  `BM_LcpWorldStackStep_BoxedLcp/4/200`, plus
  `BM_LcpWorldStackStep_BoxedLcp/5/500`. The rows rebuild the stack worlds,
  enter simulation mode, advance public boxed-LCP `World::step()` iterations,
  and report `invariant_ok=1`. The SIMD run reported `build_simd_enabled=1`,
  and the CUDA-enabled run reported `build_cuda_enabled=1`. The default
  3-sphere rows reported `time_step=0.005`, `min_spacing=0.9999`, and
  `max_vertical_speed=4.31e-14`; the default 4-sphere row reported
  `time_step=0.001`, `min_spacing=0.9999`, and `max_vertical_speed=1.72e-8`;
  the default 5-sphere row reported `time_step=0.001`, `min_spacing=0.9999`,
  and `max_vertical_speed=1.26e-5`.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldSeparatedStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally for `BM_LcpWorldSeparatedStep_BoxedLcp/4/200`,
  `BM_LcpWorldSeparatedStep_BoxedLcp/8/200`, and
  `BM_LcpWorldSeparatedStep_BoxedLcp/16/200`. The rows rebuild separated
  sphere-ground worlds, enter simulation mode, advance 200 public boxed-LCP
  `World::step()` iterations, and report `invariant_ok=1`. The 16-contact row
  reported `max_height_error=0`, `max_vertical_speed=0`, and
  `min_tangential_speed_drop=0.23816`.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldArticulated(Ground|RigidImpact)Step_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally in default, SIMD-enabled, and CUDA-enabled build trees for
  `BM_LcpWorldArticulatedGroundStep_BoxedLcp/{1,4,8,16}/200`. The rows rebuild
  fixed-base prismatic-link worlds,
  enter simulation mode inside the world factory so link kinematics are current
  before `World::collide()`, advance 200 public boxed-LCP `World::step()`
  iterations, and report `invariant_ok=1` with
  `articulated_link_count=1`, `4`, `8`, and `16`. This is articulated
  link-ground benchmark evidence for the public unified constraint path, not
  broad robot-like contact coverage.
- The same default, SIMD-enabled, and CUDA-enabled benchmark runs passed for
  `BM_LcpWorldArticulatedRigidImpactStep_BoxedLcp/{1,4,8,16}/1`. The rows
  rebuild fixed-base prismatic striker worlds
  with dynamic rigid targets, enter simulation mode inside the world factory so
  link kinematics are current before `World::collide()`, advance one public
  boxed-LCP `World::step()`, and report `invariant_ok=1` with
  `articulated_link_count=1`, `4`, `8`, and `16`,
  `dynamic_rigid_body_count=1`, `4`, `8`, and `16`,
  `max_momentum_error=0`, `max_striker_velocity=0.606667`, and
  `min_target_velocity=0.786667`. This is two-sided articulated link-vs-rigid
  benchmark evidence for the public unified constraint path, not broad
  robot-like contact coverage.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally in default, SIMD-enabled, and CUDA-enabled build trees for
  `BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp/{1,4,8,16}/1`. The rows
  rebuild cross-multibody fixed-base prismatic striker/target link worlds,
  enter simulation mode inside the world factory so link kinematics are current
  before `World::collide()`, advance one public boxed-LCP `World::step()`, and
  report `invariant_ok=1` with `articulated_pair_count=1`, `4`, `8`, and `16`,
  `articulated_dof_count=2`, `8`, `16`, and `32`,
  `cross_multibody_link_contact=1`, `max_momentum_error=0`,
  `max_striker_velocity=0.606667`, `min_target_velocity=0.786667`, and
  `min_relative_velocity=0.18`. This is cross-multibody articulated
  link-vs-link benchmark evidence for the public unified constraint path, not
  broad robot-like contact coverage.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed locally in default, SIMD-enabled, and CUDA-enabled build trees for
  `BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp/{1,4,8,16}/200`. The rows
  rebuild connected fixed-base three-axis prismatic Cartesian-chain worlds,
  enter simulation mode inside the world factory so link kinematics are current
  before `World::collide()`, advance 200 public boxed-LCP `World::step()`
  iterations, and report `invariant_ok=1` with `cartesian_chain_count=1`, `4`,
  `8`, and `16`, `articulated_dof_count=3`, `12`, `24`, and `48`, and
  `serial_prismatic_chain=1`. This is connected multi-DOF fixed-base
  articulated contact evidence for the public unified constraint path, not
  broad robot-like contact coverage.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpArticulatedUnifiedContact' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  previously passed locally for all 96
  `BM_LcpArticulatedUnifiedContact/FrictionIndex/{Ground,RigidImpact,CrossLinkImpact}/<solver>/{1,4}`
  rows. The benchmark now also registers 8-contact, 16-contact, and 24-contact rows, so these rows manually
  assemble fixed-base three-axis prismatic `LinkContact` snapshots through
  `assembleMultibodyLinkContactProblem` and `assembleUnifiedConstraintProblem`,
  then compare all 16 friction-index-capable solvers on identical 3-row,
  12-row, 24-row, 48-row, and 72-row LCPs. The cross-link rows complete a second articulated
  endpoint for a separate multibody, so they exercise the unified contact
  matrix's cross-multibody block. Focused default, SIMD-enabled, and
  CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpArticulatedUnifiedContact/FrictionIndex/.+/24$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `contract_ok=1` for all 48 new 24-contact rows, with
  `articulated_unified_contact=1`, `contact_count=24`, `problem_size=72`,
  `multibody_count=24` for ground/rigid-impact rows,
  `dynamic_rigid_body_count=24` for rigid-impact rows, and `multibody_count=48` plus
  `articulated_cross_link_contact=1` for cross-link rows.
  The SAP rows use the same robust benchmark parameters as generated coverage
  (`sap_regularization=1e-6`, `sap_max_line_search_iterations=32`,
  `maxIterations=5000`) and report those SAP counters in the articulated
  unified-contact benchmark output; the focused 24-contact runs reported SAP
  iteration counts of 2 and 97 across the three articulated contact cases.
  This is articulated unified-contact LCP assembly evidence, not
  collision-discovered or end-to-end stepping evidence. Focused
  SIMD-enabled and CUDA-enabled build-tree runs over the Dantzig/Jacobi/SAP
  ground and rigid-impact rows also reported `contract_ok=1` with
  `build_simd_enabled=1` and `build_cuda_enabled=1`, respectively; those are CPU
  solver rows in those build trees, not CUDA kernel execution.
- `test_boxed_lcp_contact --gtest_brief=1` now passes 46 tests. The dense box
  face-contact test assembles a 4-contact, 12-row boxed/findex LCP from
  `World::collide()`, checks the single-dynamic-body dense patch shape, and
  verifies the problem with APGD; the sliding and static-friction box
  end-to-end tests also assert at least four contacts before stepping.
  `FourBoxWorldStepMaintainsDenseContactInvariants` and
  `EightBoxWorldStepMaintainsDenseContactInvariants` extend this to separated
  4-box and 8-box, 16-contact and 32-contact dense face-contact scenes advanced
  for 200 public boxed-LCP `World::step()` iterations.
  `SixteenBoxWorldStepMaintainsDenseContactInvariants` extends this to a
  16-box, 64-contact scene over 500 public boxed-LCP `World::step()`
  iterations.
  `TwentyFourBoxWorldStepMaintainsDenseContactInvariants` extends this to a
  24-box, 96-contact scene over 2000 small public boxed-LCP `World::step()`
  iterations. The run still emits the dense-patch Dantzig warning, so
  Dantzig's direct dense box solve is not claimed.
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers 48 scoped dense
  box rows: `Pgs`, `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and `Admm` on
  1/2/4/8/16/24/32/48-box snapshots. Focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContact/FrictionIndex/.+/48$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported
  `contract_ok=1`, `dense_box_contact=1`, `box_count=48`,
  `contact_count=192`, and `problem_size=576` in default, SIMD-enabled, and
  CUDA-enabled build trees; the CUDA-enabled rows are CPU solver rows in that
  build tree, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldBoxStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  previously passed in default, SIMD-enabled, and CUDA-enabled build trees for
  `BM_LcpWorldBoxStep_BoxedLcp/{1,2,4,8}/200`,
  `BM_LcpWorldBoxStep_BoxedLcp/16/500`,
  `BM_LcpWorldBoxStep_BoxedLcp/24/2000`, and
  `BM_LcpWorldBoxStep_BoxedLcp/32/4000`; the benchmark now also registers
  `BM_LcpWorldBoxStep_BoxedLcp/48/4000`. The eight-row
  registry covers 4/8/16/32/64/96/128/192 face contacts. Focused 24-/32-box runs
  reported `invariant_ok=1` and `dense_box_contact=1` in default, SIMD-enabled,
  and CUDA-enabled build trees; the default 32-box row reported
  `max_height_error=1.46e-4` and `max_vertical_speed=4.38e-2`.
  A focused default 48-box run reported `invariant_ok=1`,
  `contact_count=192`, `step_count=4000`, `max_height_error=9.80e-5`, and
  `max_vertical_speed=1.08e-2`. The CUDA-enabled rows are CPU public-step rows
  in that build tree, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests` listed 190 manifest-generated
  `BM_LcpCompare/<family>/<solver>/<size>` benchmarks.
- `BM_LCP_COMPARE --benchmark_list_tests` listed 56 manifest-generated
  `BM_LcpBatchSerial/<family>/<solver>` benchmarks: 24 standard, 16 boxed, and
  16 friction-index entries.
- `BM_LCP_COMPARE --benchmark_list_tests` listed 56 manifest-generated
  `BM_LcpBatchParallel/<family>/<solver>` benchmarks using the DART 7
  experimental `ParallelExecutor`: 24 standard, 16 boxed, and 16 friction-index
  entries.
- `BM_LCP_COMPARE --benchmark_list_tests` listed 32 manifest-generated
  `BM_LcpWorldContactBatch(Serial|Parallel)/FrictionIndex/<solver>`
  benchmarks: 16 serial and 16 DART 7 `ParallelExecutor` rows over the mixed
  contact-derived batch.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpShockPropagationLayerSweep' | wc -l`
  listed 9 ShockPropagation layer-layout rows. Focused JSON runs for
  `BM_LcpShockPropagationLayerSweep` in default, SIMD-enabled, and CUDA-enabled
  build trees reported 9 rows with zero `contract_ok` failures across
  standard 48-row, boxed 24-row, and friction-index 8-contact fixtures; the
  rows recorded `layer_count=1/2/8/16`, `block_count=8/16`,
  `max_block_size=3`, `max_blocks_per_layer=1/4/8/16`, and backend build-state
  counters.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMprgpSpdCheckSweep' | wc -l`
  listed 9 MPRGP SPD/check rows. Focused JSON runs for
  `BM_LcpMprgpSpdCheckSweep` in default, SIMD-enabled, and CUDA-enabled build
  trees reported 9 rows with zero `contract_ok` failures across dense SPD
  32/64-row, banded SPD 64-row, mildly ill-conditioned SPD 32-row, and
  near-singular SPD 8-row standard-LCP fixtures; the rows recorded
  `mprgp_positive_definite_check=0/1`, problem sizes `8/32/64`, observed solver
  `iterations=3/5/15`, and backend build-state counters.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpInteriorPointPathSweep' | wc -l`
  listed 9 Interior Point path-parameter rows. Focused JSON runs for
  `BM_LcpInteriorPointPathSweep` in default, SIMD-enabled, and CUDA-enabled
  build trees reported 9 rows with zero `contract_ok` failures across dense SPD
  32/64-row, banded SPD 64-row, mildly ill-conditioned SPD 32-row, and
  near-singular SPD 8-row standard-LCP fixtures; the rows recorded
  `interior_point_sigma=0.1/0.3`, `interior_point_step_scale=0.75/0.99`,
  observed solver `iterations=14/16/27/31/32/41/51/53`, and backend build-state
  counters.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpStaggeringContactPipelineSweep' | wc -l`
  listed 9 Staggering contact-pipeline rows. Focused JSON runs for
  `BM_LcpStaggeringContactPipelineSweep` in default, SIMD-enabled, and
  CUDA-enabled build trees reported 9 rows with zero `contract_ok` failures
  across separated world-contact 1/2/4-contact fixtures, coupled vertical-stack
  2/3/5-contact fixtures, and articulated unified ground, rigid-impact, and
  cross-link-impact 4-contact fixtures; the rows recorded
  `staggering_normal_friction_split=1`, normal-row counts `1/2/3/4/5`,
  friction-row counts `2/4/6/8/10`, coupled-contact flags, contact counts
  `1/2/3/4/5`, and backend build-state counters.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpBoxedSemiSmoothNewtonLineSearchSweep' | wc -l`
  listed 9 Boxed Semi-Smooth Newton line-search rows. Focused JSON runs for
  `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep` in default, SIMD-enabled, and
  CUDA-enabled build trees reported 9 rows with zero `contract_ok` failures
  across standard 48-row, boxed 24-row, and friction-index 8-contact fixtures;
  the rows recorded `boxed_ssn_max_line_search_steps=10/20`,
  `boxed_ssn_step_reduction=0.5/0.8`, default/more-step/gentle-reduction
  policy counters, observed solver `iterations=2/7/8/9`, and backend
  build-state counters.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpContactSolverComparisonSweep' | wc -l`
  listed 27 ADMM/SAP/Boxed Semi-Smooth Newton contact comparison rows. Focused
  JSON runs for `BM_LcpContactSolverComparisonSweep` in default, SIMD-enabled,
  and CUDA-enabled build trees reported 27 rows with zero `contract_ok`
  failures across separated world-contact 1/2/4-contact fixtures, coupled
  vertical-stack 2/3/5-contact fixtures, and articulated unified ground,
  rigid-impact, and cross-link-impact 4-contact fixtures. The rows recorded
  solver identity counters, normal-row counts `1/2/3/4/5`, friction-row counts
  `2/4/6/8/10`, contact counts `1/2/3/4/5`, problem sizes `3/6/9/12/15`, and
  backend build-state counters. The CUDA-enabled rows are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditioned/ExtremeCoupledFrictionIndex' | wc -l`
  lists 24 ADMM/SAP/Boxed Semi-Smooth Newton extreme-coupling single-problem
  rows. Focused runs for
  `BM_LcpMildIllConditioned(BatchSerial|BatchParallel)?/ExtremeCoupledFrictionIndex64`
  in default, SIMD-enabled, and CUDA-enabled build trees reported 7 rows with zero
  `contract_ok` failures across 16x-coupled mildly ill-conditioned
  friction-index packets at 64 contacts/192 rows: three single-problem rows
  for `Admm`, `Sap`, and `BoxedSemiSmoothNewton`, plus ADMM/SAP serial and
  parallel batch rows. The 64-contact packet extends the existing 16
  contacts/48 rows, 24 contacts/72 rows, 32 contacts/96 rows, and
  48 contacts/144 rows. The generated coverage test now covers the
  16-/24-/32-/48-/64-contact ADMM/SAP/Boxed Semi-Smooth Newton packets, and the focused
  `LcpGeneratedCoverage.LargerMildlyIllConditionedKnownSolutionsForScopedSolvers`
  unit test passes in the default, SIMD-enabled, and CUDA-enabled build trees.
  Boxed Semi-Smooth Newton serial and parallel batch rows are now registered
  for 6/8/12/16/32/48 contacts after focused checks passed; the 24-/64-contact
  rows remain unregistered after default-tree trials reported `contract_ok=0`.
  The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpContactNormalStandardSweep' | wc -l`
  listed 76 contact-normal standard-LCP rows. Focused JSON runs for
  `BM_LcpContactNormalStandardSweep` in default, SIMD-enabled, and CUDA-enabled
  build trees reported 76 rows with zero `contract_ok` failures across
  normal-only standard LCPs extracted from separated world-contact 1/2/4-contact
  fixtures, coupled vertical-stack 2/3/5-contact fixtures, and articulated
  unified ground, rigid-impact, and cross-link-impact 4-contact fixtures. The
  rows compare `Dantzig`, `Lemke`, `Baraff`, `Direct`, `MinimumMapNewton`,
  `FischerBurmeisterNewton`, `PenalizedFischerBurmeisterNewton`,
  `InteriorPoint`, and `MPRGP`; `Direct` is intentionally limited to four
  1-, 2-, and 3-row subproblems so every Direct row reports
  `contact_normal_direct_no_fallback=1`. The rows recorded normal-row counts
  `1/2/3/4/5`, source contact problem sizes `3/6/9/12/15`, standard problem
  sizes `1/2/3/4/5`, coupled-fixture counters, and backend build-state
  counters. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled build,
  not CUDA LCP kernel execution. This is contact-normal standard-LCP evidence,
  not native boxed or friction-index support for standard-only solvers.
- `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpPivotingScaleSweep' | wc -l`
  listed 12 pivoting scale rows. Focused JSON runs for
  `BM_LcpPivotingScaleSweep` in default, SIMD-enabled, and CUDA-enabled build
  trees reported 12 rows with zero `contract_ok` failures across Direct 2/3-row
  standard fixtures, Lemke and Baraff 8/16-row standard fixtures, and Dantzig
  8/16-row standard, 12/24-row boxed, and 4/8-contact friction-index fixtures;
  the rows recorded Direct no-fallback counters, four Dantzig
  boxed-or-findex rows, friction-index `contact_count=4/8`, observed solver
  `iterations=1/4/8/16`, and backend build-state counters.
- Focused pivoting unit coverage
  `DantzigMatrixCoverage.*:LemkeSolverCoverage.*:DantzigSolverCoverage.*:DirectSolverCoverage.*:BaraffSolverCoverage.*`
  passed 42 tests.
- A focused `BM_LCP_COMPARE` slice covering standard Dantzig, boxed
  ShockPropagation, and friction-index NNCG ran successfully with residual,
  complementarity, bound-violation, `contract_ok`, iteration, and problem-size
  counters.
- A focused `BM_LCP_COMPARE` batch slice covering serial and parallel-batch
  standard Dantzig, boxed boxed-semi-smooth Newton, and friction-index BGS ran
  successfully with `batch_size`, `total_problem_size`, maximum residual,
  maximum complementarity, maximum bound-violation, average-iteration, and
  `contract_ok` counters. The parallel rows also reported `worker_count=20`,
  `parallel_units=4`, `profile_enabled=1`, and observed `max_parallelism=4`.
- `BM_LCP_COMPARE` rows now report `build_simd_enabled`,
  `build_simd_force_scalar`, `build_cuda_enabled`, and
  `has_simulation_experimental` counters. The default DART 7 build slice
  reported `build_simd_enabled=0`, `build_cuda_enabled=0`,
  `has_simulation_experimental=1`, and `contract_ok=1` for the focused serial
  batch, task-parallel batch, threaded validation, and smoke rows.
- A separate focused SIMD-enabled DART 7 build configured with
  `DART_ENABLE_SIMD=ON`, `DART_ENABLE_EXPERIMENTAL_CUDA=OFF`, and
  `DART_BUILD_SIMULATION_EXPERIMENTAL=ON`. In that build,
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.*'`
  passed 21 tests after adding the production active-set slice, the focused
  `BM_LcpMildIllConditioned` slice reported
  `build_simd_enabled=1`, `build_cuda_enabled=0`,
  `has_simulation_experimental=1`, and `contract_ok=1` on all 308 rows, and the
  focused `BM_LcpStressActiveSetTransition` slice reported
  `build_simd_enabled=1`, `build_cuda_enabled=0`,
  `active_set_transition=1`, `stress_active_set_transition=1`, and
  `contract_ok=1` on all 49 rows. The
  focused `BM_LcpSingularDegenerate` slice reported `build_simd_enabled=1`,
  `build_cuda_enabled=0`, `rank_deficient=1`, and `contract_ok=1` on all 27
  rows. The focused `BM_LcpLargerSingularDegenerate` slice also reported
  `build_simd_enabled=1`, `build_cuda_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all 27 rows. The focused
  `BM_LcpStressSingularDegenerate` slice also reported
  `build_simd_enabled=1`, `build_cuda_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all 27 rows. The focused
  `BM_LcpProductionActiveSetTransition` slice reported
  `build_simd_enabled=1`, `build_cuda_enabled=0`, `contact_count=24/32/48/64`,
  `problem_size=72/96/144/192`, `coupling_scale=2/4/8/16`, and `contract_ok=1` on all 64
  rows. Earlier
  focused `BM_LcpJacobiSolverThreading_Standard` rows also passed in that SIMD
  build with `contract_ok=1` and `solver_internal_threads=8` on worker-count 8
  rows, and the focused 2048-row banded Jacobi rows passed with
  `solver_internal_threads=1/8/16`; the local rows did not show a general
  speedup.
- A separate focused CUDA-enabled DART 7 build configured through
  `pixi run -e cuda build-cuda Release` reported
  `DART_ENABLE_EXPERIMENTAL_CUDA=ON`, `DART_CUDA_ARCHITECTURES=89`,
  `DART_ENABLE_SIMD=OFF`, and `DART_BUILD_SIMULATION_EXPERIMENTAL=ON`.
  `pixi run -e cuda test-cuda Release` passed 3
  `simulation-experimental-cuda` CTest targets and ran the documented CUDA
  benchmark smoke filters on the visible NVIDIA GeForce RTX 4080 Laptop GPU.
  In that same CUDA-enabled build,
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.*'`
  passed 21 tests after adding the production active-set slice, the focused
  `BM_LcpMildIllConditioned` slice reported
  `build_cuda_enabled=1`, `build_simd_enabled=0`,
  `has_simulation_experimental=1`, and `contract_ok=1` on all 308 rows, and the
  focused `BM_LcpStressActiveSetTransition` slice reported
  `build_cuda_enabled=1`, `build_simd_enabled=0`,
  `active_set_transition=1`, `stress_active_set_transition=1`, and
  `contract_ok=1` on all 49 rows. Those rows are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution. The
  focused `BM_LcpSingularDegenerate` slice reported `build_cuda_enabled=1`,
  `build_simd_enabled=0`, `rank_deficient=1`, and `contract_ok=1` on all 27
  rows. The focused `BM_LcpLargerSingularDegenerate` slice also reported
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all 27 rows. The focused
  `BM_LcpStressSingularDegenerate` slice also reported
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `rank_deficient=1`, and
  `contract_ok=1` on all 27 rows. The focused
  `BM_LcpProductionActiveSetTransition` slice reported
  `build_cuda_enabled=1`, `build_simd_enabled=0`, `contact_count=24/32/48/64`,
  `problem_size=72/96/144/192`, `coupling_scale=2/4/8/16`, and `contract_ok=1` on all 64
  rows. These are CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel
  execution. Earlier
  focused `BM_LcpJacobiSolverThreading_Standard` rows also passed in that CUDA
  build with `contract_ok=1` and `solver_internal_threads=8` on worker-count 8
  rows, and the focused 2048-row banded Jacobi rows passed with
  `solver_internal_threads=1/8/16`; this remains CPU Jacobi execution in a
  CUDA-enabled build.
- `test_lcp_jacobi_batch_cuda --gtest_brief=1` passed 31 tests in the
  CUDA-enabled build, including standard, boxed, friction-index, and
  grouped variable-size synthetic standard/boxed/friction-index plus
  contact-derived world-contact, coupled stack-contact, and articulated
  unified-contact batches including cross-multibody link-vs-link packets, plus
  mixed grouped contact batches executed on CUDA for fixed-iteration Jacobi and
  PGS.
  Focused `BM_LcpCudaJacobiBatch_*`,
  `BM_LcpCudaPgsBatch_*`, `BM_LcpCudaJacobiGroupedBatch_*`,
  `BM_LcpCudaPgsGroupedBatch_*`,
  `BM_LcpCudaJacobiWorldContactBatch_FrictionIndex`,
  `BM_LcpCudaPgsWorldContactBatch_FrictionIndex`,
  `BM_LcpCudaJacobiWorldStackContactBatch_FrictionIndex`,
  `BM_LcpCudaPgsWorldStackContactBatch_FrictionIndex`,
  `BM_LcpCudaJacobiWorldContactGroupedBatch_FrictionIndex`, and
  `BM_LcpCudaPgsWorldContactGroupedBatch_FrictionIndex`,
  `BM_LcpCudaJacobiWorldStackContactGroupedBatch_FrictionIndex`, and
  `BM_LcpCudaPgsWorldStackContactGroupedBatch_FrictionIndex`, plus
  `BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch_FrictionIndex` and
  `BM_LcpCudaPgsArticulatedUnifiedContactGroupedBatch_FrictionIndex`,
  `BM_LcpCudaJacobiMixedContactGroupedBatch_FrictionIndex`,
  `BM_LcpCudaPgsMixedContactGroupedBatch_FrictionIndex`, and the PGS-only
  `BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex` and
  `BM_LcpCudaPgsWorldBoxContactGroupedBatch_FrictionIndex` rows ran
  successfully and reported `build_cuda_enabled=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, fixed iteration counters, and `contract_ok=1`. The
  grouped synthetic CUDA rows use three problem-size groups and two variants
  per group (`batch_size=6`, `cuda_group_count=3`). The standard and boxed rows
  cover 16/32/48-row packets with `total_problem_size=192`; the friction-index
  rows cover 4/8/16-contact packets with `min_problem_size=12`,
  `max_problem_size=48`, `total_contact_count=56`, and
  `total_problem_size=168`. The
  homogeneous world-contact CUDA rows used DART 7 `World::collide()`
  snapshots up to 16 contacts with `batch_size=4`, `contact_count=16`,
  `problem_size=48`, `total_contact_count=64`, and `total_problem_size=192`
  on the denser rows. The
  homogeneous coupled stack CUDA rows use 5-sphere DART 7 `World::collide()`
  snapshots with `batch_size=4`, `sphere_count=5`, `contact_count=5`,
  `problem_size=15`, `total_contact_count=20`, and `total_problem_size=60`.
  The separated grouped variable-size rows now
  use 1/2/4/8/16-contact sphere-ground packets with `batch_size=10`,
  `cuda_group_count=5`, `contact_shape_count=5`, `min_problem_size=3`,
  `max_problem_size=48`, `total_contact_count=62`, and
  `total_problem_size=186`. The coupled stack
  grouped variable-size rows now use 2/3/4/5-sphere stack packets with
  `batch_size=8`, `cuda_group_count=4`, `contact_shape_count=4`,
  `min_problem_size=6`, `max_problem_size=15`,
  `total_contact_count=28`, and `total_problem_size=84`. The articulated
  grouped variable-size rows use manually assembled 1-/4-/8-/16-contact fixed-base
  three-axis prismatic unified-contact packets covering link-ground,
  link-vs-dynamic-rigid, and cross-multibody link-vs-link cases with
  `batch_size=24`, `cuda_group_count=4`, `contact_shape_count=4`,
  `articulated_contact_case_count=3`, `articulated_cross_link_contact=1`,
  `min_problem_size=3`, `max_problem_size=48`, `total_contact_count=174`, and
  `total_problem_size=522`. The mixed grouped rows combine separated
  sphere-ground, coupled stack, and manually assembled 1-/4-/8-/16-contact
  articulated unified-contact packets, including cross-multibody link-vs-link
  packets, with
  `batch_size=32`, `contact_fixture_family_count=3`, `cuda_group_count=6`,
  `contact_shape_count=6`, `articulated_contact_case_count=3`,
  `articulated_cross_link_contact=1`, `min_problem_size=3`,
  `max_problem_size=48`, `total_contact_count=198`, and
  `total_problem_size=594`. The dense box PGS
  rows include homogeneous 4-problem batches for 1-/4-/8-/16-/24-/32-/48-box dense
  face-contact `World::collide()` snapshots and a grouped variable-size
  1/2/4/8/16/24/32-box batch. All report
  `cuda_dense_box_contact_batch=1` and `dense_box_contact=1`; the 48-box
  homogeneous row reports `batch_size=4`, `box_count=48`, `contact_count=192`,
  `problem_size=576`, `total_contact_count=768`, `total_body_count=192`, and
  `total_problem_size=2304`, while the grouped row remains scoped to
  1/2/4/8/16/24/32-box packets and reports `batch_size=14`,
  `cuda_group_count=7`, `box_count_shape_count=7`, `min_problem_size=12`,
  `max_problem_size=384`, `total_contact_count=696`, `total_body_count=174`, and
  `total_problem_size=2088`. A grouped 48-box CUDA PGS extension was probed and
  is not claimed: at 1024 fixed iterations the grouped validation failed two
  48-box variants with fixed-variable residual/complementarity around
  0.606/0.625. A fixed-iteration CUDA Jacobi dense-box trial
  failed the LCP contract and is not claimed. The earlier failed probe used the
  previous homogeneous 4-problem and grouped 1/2/4-box dense box-face fixtures:
  at 4096 iterations with relaxation 1.0 it failed with
  residual/complementarity/bound violations of about 3.3e-2 to 5.0e-2
  (`w must be non-negative at lo`), and at 8192 iterations with relaxation
  0.25 it reached near-zero residual/bounds but failed fixed-variable
  complementarity at about 0.34 to 0.435 (`fixed variable residual`). This is
  real CUDA LCP batch execution evidence for fixed-iteration projected Jacobi
  and PGS on homogeneous dense standard/boxed/friction-index packets plus
  separated sphere-ground and small coupled stack world-contact packets plus
  manually assembled articulated unified-contact packets and mixed grouped
  contact packets, plus PGS-only dense box-face contact packets up to 16 boxes
  and 64 contacts; it is not evidence for CUDA execution across the full solver
  manifest, CUDA Jacobi dense-contact execution, or end-to-end articulated
  world-step CUDA execution.

## Cross-Cutting Gaps

- Background taxonomy: APGD, TGS, ADMM, SAP, and boxed semi-smooth Newton are
  now described in `docs/background/lcp/` and remain covered by the manifest.
  Future solver additions should update the background docs and manifest
  together.
- Support semantics: the manifest is the current source of truth for native
  standard/boxed/friction-index support. Some background text says
  boxed/findex fallback for standard-only methods; that should not be counted
  as native support without solver-specific evidence.
- DART 7 integration: current tests now include single-contact and two-contact
  `dart::simulation::World` boxed-LCP contact snapshots from `World::collide()`
  plus two-contact, four-contact, 16-contact, and 3-sphere-stack 200-step and
  500-step, plus 4-sphere-stack 200-step and 5-sphere-stack 500-step boxed-LCP
  `World::step()` invariant tests and world-contact
  benchmark rows for 1/2/4 separated sphere-ground contacts, separated
  4-/8-/16-contact step rows,
  2/3-sphere vertical stacks, 4-/5-sphere vertical-stack rows for all of those
  solvers except `NNCG`, mixed contact-derived serial/parallel batches, the
  3-sphere stack public step path, fixed-base prismatic articulated
  link-ground one-link and four-link `World::step()` paths, connected
  fixed-base three-axis Cartesian-chain `World::step()` paths, fixed-base
  link-vs-rigid `World::step()` paths, cross-multibody fixed-base
  link-vs-link `World::step()` paths, 1-/2-/4-/8-/16-box dense box-face
  all-solver snapshot and `World::step()` benchmark rows, plus manually assembled three-axis
  articulated unified-contact LCP snapshots including cross-multibody
  link-vs-link rows, but broader solver
  selection evidence is still missing for richer articulated, longer-running
  denser coupled multi-contact scenes.
- Correctness breadth: the comparison fixtures prove known small solutions, and
  generated coverage now adds batch-shaped state reuse, invalid/failure
  reporting, near-singular standard pivoting cases, and near-singular boxed
  cases. Generated friction-index coverage now includes coupled
  well-conditioned 2-contact, 4-contact, and 6-contact cases plus mildly
  ill-conditioned 2-contact and 4-contact cases. Separate scalable-solver
  slices now reach standard 128-row, boxed 64-row, friction-index 24-contact,
  coupled friction-index 12-contact, mildly ill-conditioned coupled
  friction-index 16-contact and 24-contact, stronger-coupled mildly
  ill-conditioned 16-/24-contact cases plus a 32-contact 8x case,
  near-singular standard 8-row, boxed 8-row, coupled friction-index 3-, 6-,
  9-, 12-, 16-, 24-, 32-, 48-, and 64-contact, and larger exact
  rank-deficient
  singular-degenerate standard 32-row, boxed 32-row, and coupled
  friction-index 8-contact cases, plus stress exact rank-deficient
  singular-degenerate standard 64-row, boxed 64-row, and coupled
  friction-index 12-contact cases, plus extreme exact rank-deficient
  singular-degenerate standard 128-row, boxed 128-row, and coupled
  friction-index 16-/24-/32-/48-/64-contact cases. Active-set transition coverage now includes
  standard 16-row, boxed 16-row, coupled friction-index 6-contact, scoped
  standard 32-row, 64-row, and 128-row, boxed 32-row, 64-row, and 128-row, and
  coupled friction-index 8-contact, 12-contact, and 16-contact cases near
  lower, upper, and friction-cone boundaries, plus stronger-coupled production
  coupled friction-index 24-, 32-, 48-, and 64-contact active-set packets, plus
  4x/8x-coupled mildly ill-conditioned 6-/8-/12-/16-/24-contact plus 8x
  32-contact packets with Boxed Semi-Smooth Newton included in single rows plus
  non-16x and green 16x batch rows, ADMM/SAP/Boxed Semi-Smooth Newton
  single-problem
  16x-coupled
  mildly ill-conditioned
  6-/8-/12-/16-/24-/32-/48-/64-contact
  friction-index slice. Coverage is still incomplete for broader hard
  solver-specific conditioning/coupling grids, direct backend execution beyond
  CPU solver rows in SIMD/CUDA-enabled builds, and real contact systems.
- Performance breadth: manifest benchmarks are apples-to-apples math-layer
  comparisons for generated single problems, active-set transition packets,
  larger mildly ill-conditioned packets, coupled/4x/8x/16x mildly ill-conditioned
  batch packets, plus serial and task-parallel independent-problem batches.
  World-contact
  benchmark rows now cover simple
  separated boxed-LCP contact snapshots, small coupled vertical stacks through
  5-sphere all-solver rows except `NNCG`, mixed serial/task-parallel batches
  over those snapshots, stress mixed serial/task-parallel batches that include
  4-/5-sphere stack snapshots for all of those solvers except `NNCG`,
  200-step/500-step
  3-sphere, 200-step 4-sphere, and 500-step 5-sphere end-to-end stack step
  rows, and 4-/8-/16-contact separated end-to-end step rows, plus
  1-/2-/4-/8-/16-/24-/32-/48-box dense box-face end-to-end step rows, fixed-base
  prismatic articulated link-ground, link-vs-rigid impact, and
  cross-multibody link-vs-link impact step rows up to 16-link or 16-pair
  benchmark scenes, plus
  manually assembled three-axis articulated unified-contact all-solver rows
  through 16 contacts including cross-multibody link-vs-link rows,
  plus scoped dense box face-contact rows through 48 boxes for six solvers and
  PGS-only dense
  box-face CUDA batch rows through homogeneous 1/4/8/16/24/32/48-box and grouped
  1/2/4/8/16/24/32-box packets, but not broad
  robot-like or general dense CUDA contact systems.
- Backend evidence: benchmark rows now identify scalar/SIMD/CUDA build state,
  a focused SIMD-enabled CPU slice passes, a focused CUDA-enabled build/runtime
  slice passes, Jacobi has opt-in CPU worker-thread correctness/comparison
  evidence, and narrow CUDA projected-Jacobi and PGS
  standard/boxed/friction-index plus grouped variable-size synthetic
  standard/boxed/friction-index, homogeneous 4-/8-/16-contact, homogeneous
  5-sphere coupled stack, and grouped variable-size 1/2/4/8/16-contact separated
  and 2/3/4/5-sphere coupled stack world-contact batch paths, plus manually
  assembled 1-/4-/8-/16-contact articulated unified-contact batch paths including
  cross-multibody link-vs-link packets, and mixed
  separated/stack/articulated grouped contact batch paths, and PGS-only dense
  box-face CUDA batch paths through homogeneous 1/4/8/16/24/32/48-box and grouped
  1/2/4/8/16/24/32-box packets pass.
  This still does
  not prove broader solver-internal multi-threaded speedups, general CUDA LCP
  solver execution, CUDA Jacobi dense-contact execution, end-to-end articulated
  world-step CUDA execution, or broader vectorized/CUDA LCP batch-processing
  coverage.

## Next Evidence Slices

1. Expand the manifest-generated correctness matrix across larger problem
   sizes, conditioning classes, active-set patterns, and friction-index
   coupling patterns.
2. Extend DART 7 boxed-LCP `World` contact fixtures from the current
   separated sphere-ground, fixed-base prismatic articulated, cross-multibody
   fixed-base articulated link-vs-link, manually assembled three-axis
   articulated, 5-sphere end-to-end vertical-stack, and
   small vertical-stack boxed-LCP
   snapshots and contact-derived benchmark rows to richer articulated,
   longer-running, and denser coupled multi-contact scenes beyond the current
   24-box unit / 48-box benchmark dense face-contact public-step and
   all-solver snapshot slices
   that validate solver outputs against motion/contact invariants.
3. Add benchmark packets that broaden scalar CPU and SIMD-enabled CPU evidence,
   larger and sparser solver-internal multi-threaded CPU cases,
   general CUDA LCP solver execution, and vectorized/CUDA LCP batch-processing
   evidence.
4. Keep `docs/background/lcp/`, `tests/common/lcpsolver/lcp_solver_manifest.hpp`,
   and benchmark registration synchronized when solver support changes.
