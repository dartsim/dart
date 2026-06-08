# LCP Solvers - Dev Task

## Current Status

- [x] Confirmed this task targets the DART 7 line, with the default configured
      tree reporting DART 7.0.0 and `DART_BUILD_SIMULATION_EXPERIMENTAL=ON`.
- [x] Expanded the solver-agnostic comparison harness so advanced solvers that
      previously had smoke coverage also prove known-solution correctness on
      standard, boxed, and friction-index fixtures.
- [x] Added a shared test manifest for the DART 7 LCP solver set so smoke tests,
      comparison coverage checks, and benchmark compile checks use one solver
      capability source of truth.
- [x] Migrated standard, boxed, and friction-index comparison benchmark
      registration to that manifest so supported solvers use one deterministic
      problem per family/size.
- [x] Added manifest-generated serial batch benchmark registration so the same
      independent-problem batch is run across every supporting solver in each
      LCP family.
- [x] Added manifest-generated DART 7 `ParallelExecutor` batch benchmark
      registration so the same independent-problem batches also exercise
      task-parallel CPU execution through the experimental compute graph.
- [x] Added backend build-state counters to LCP benchmark rows so scalar,
      SIMD-enabled, CUDA-enabled, and simulation-experimental builds are
      distinguishable in benchmark output.
- [x] Added opt-in solver-internal CPU worker threads for `JacobiSolver` plus
      generated correctness and dense/banded serial-vs-threaded benchmark
      evidence.
- [x] Added focused PGS/PSOR and symmetric PSOR relaxation sweep benchmark rows
      covering under-relaxation, plain relaxation, and over-relaxation on
      standard, boxed, and friction-index fixtures.
- [x] Added focused Red-Black Gauss-Seidel relaxation sweep benchmark rows with
      two-color partition counters on standard, boxed, and friction-index
      fixtures.
- [x] Added focused APGD restart-policy sweep benchmark rows on standard,
      boxed, and friction-index fixtures.
- [x] Added focused TGS iteration-budget sweep benchmark rows on standard,
      boxed, and friction-index fixtures.
- [x] Added focused NNCG PGS-preconditioner iteration sweep benchmark rows on
      standard, boxed, and friction-index fixtures.
- [x] Added focused SubspaceMinimization PGS-iteration sweep benchmark rows on
      standard, boxed, and friction-index fixtures.
- [x] Added focused ShockPropagation layer-layout sweep benchmark rows on
      standard, boxed, and friction-index fixtures.
- [x] Added focused MPRGP SPD/positive-definite-check sweep benchmark rows on
      standard SPD fixtures.
- [x] Added focused Interior Point path-parameter sweep benchmark rows on
      standard SPD fixtures.
- [x] Added focused Staggering contact-pipeline sweep benchmark rows on DART 7
      separated world-contact, coupled stack-contact, and articulated unified
      contact fixtures.
- [x] Added focused Boxed Semi-Smooth Newton line-search sweep benchmark rows
      on standard, boxed, and friction-index fixtures.
- [x] Added focused ADMM rho/adaptive-rho sweep benchmark rows on standard,
      boxed, and friction-index fixtures.
- [x] Added focused SAP regularization sweep benchmark rows on standard, boxed,
      and friction-index fixtures.
- [x] Added focused ADMM, SAP, and Boxed Semi-Smooth Newton contact comparison
      benchmark rows on DART 7 separated world-contact, coupled stack-contact,
      and articulated unified-contact fixtures.
- [x] Added solver-specific ADMM/SAP generated correctness and benchmark rows
      for 16x-coupled mildly ill-conditioned DART 7 friction-index packets at
      16, 24, 32, and 48 contacts.
- [x] Added focused contact-normal standard-LCP benchmark rows for standard-only
      pivoting, Newton, Interior Point, and MPRGP solvers over DART 7
      separated world-contact, coupled stack-contact, and articulated
      unified-contact snapshots, with Dantzig as an exact baseline.
- [x] Verified a focused DART 7 SIMD-enabled CPU build slice for generated LCP
      correctness and scalar/parallel batch benchmark rows.
- [x] Verified a focused DART 7 CUDA-enabled build/runtime slice and ran the
      LCP generated coverage plus selected LCP benchmark rows in that
      CUDA-enabled build.
- [x] Added experimental CUDA LCP execution paths: fixed-iteration projected
      Jacobi and PGS batch solves for homogeneous dense standard, boxed, and
      friction-index LCP packets, grouped variable-size synthetic standard,
      boxed, and friction-index packets, homogeneous 4-, 8-, and 16-contact DART 7
      world-contact packets, homogeneous 5-sphere coupled stack-contact
      packets, grouped variable-size 1/2/4/8/16-contact sphere-ground packets,
      grouped variable-size 2/3/4/5-sphere coupled stack-contact packets, and
      grouped variable-size manually assembled 1-/4-/8-contact articulated
      unified-contact packets including cross-multibody link-vs-link cases,
      plus mixed grouped contact packets combining those separated, stack, and
      articulated fixture families, with CUDA unit coverage and benchmark rows.
      This does not yet cover a general CUDA backend for every solver.
- [x] Completed a solver-by-solver implementation audit against
      `docs/background/lcp/`.
- [x] Added manifest-driven generated known-solution coverage for all supporting
      solvers on standard, boxed, and friction-index problem families.
- [x] Added generated coverage for batch-shaped state reuse, invalid-problem
      failure reporting, near-singular standard pivoting cases, and
      near-singular boxed cases.
- [x] Added focused pivoting scale benchmark rows that separate Direct
      2D/3D enumeration from larger-problem Dantzig fallback and cover Dantzig
      standard, boxed, and friction-index fixtures.
- [x] Added coupled friction-index generated coverage for well-conditioned
      2-contact, 4-contact, and 6-contact cases plus mildly ill-conditioned
      2-contact and 4-contact cases.
- [x] Extended all-supporting-solver generated size/conditioning coverage to
      standard 16-row, boxed 12-row, friction-index 8-contact, and mildly
      ill-conditioned standard/boxed 8-row cases.
- [x] Added a larger generated known-solution slice for scalable/iterative-capable
      solvers covering standard 32-row and 64-row, boxed 32-row,
      friction-index 16-contact, and coupled friction-index 8-contact cases.
- [x] Added a production-scale generated known-solution slice for scoped
      scalable solvers covering standard 128-row, boxed 64-row,
      friction-index 24-contact, and coupled friction-index 12-contact cases.
- [x] Added a near-singular generated known-solution slice for a robust scoped
      solver set covering standard 8-row, boxed 8-row, and coupled
      friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, and 48-contact cases.
- [x] Added a singular-degenerate generated known-solution slice with exact
      rank-deficient matrices covering standard 16-row, boxed 16-row, and
      coupled friction-index 6-contact cases. The standard packet covers 21
      standard-capable solvers. The boxed and friction-index packets cover
      `Admm`, `Sap`, and `BoxedSemiSmoothNewton`; trials excluded
      `Dantzig`, `ShockPropagation`, and the projection-style boxed/findex
      paths because they did not satisfy the selected singular packets'
      contract.
- [x] Extended the singular-degenerate generated known-solution slice to larger
      exact rank-deficient packets covering standard 32-row, boxed 32-row, and
      coupled friction-index 8-contact cases over the same robust solver scope.
- [x] Added a larger mildly ill-conditioned generated known-solution slice for
      scoped solvers covering standard 32-row and 64-row, boxed 16-row and
      32-row, friction-index 8-contact, and coupled friction-index 6-, 8-, and
      12-, 16-, and 24-contact cases, plus stronger-coupled 16- and
      24-contact cases with 4x and 8x cross-contact coupling, a
      stronger-coupled 32-contact case with 8x cross-contact coupling, plus an
      ADMM/SAP-only 16x-coupled 16-/24-/32-/48-contact slice. `MPRGP` is intentionally
      excluded from the stricter standard known-solution slice after the 32-row
      trial satisfied the LCP contract but missed the selected expected-solution
      tolerance.
- [x] Added generated active-set transition known-solution coverage for
      standard 16-row, boxed 16-row, and coupled friction-index 6-contact cases
      across manifest-supporting solvers, verified in default, SIMD, and CUDA
      build trees.
- [x] Extended generated active-set transition coverage for scoped
      scalable solvers to standard 32-row, boxed 32-row, and coupled
      friction-index 8-contact cases, verified in default, SIMD, and CUDA build
      trees.
- [x] Added a stress generated active-set transition slice for scoped scalable
      solvers covering standard 64-row, boxed 64-row, and coupled
      friction-index 12-contact cases, verified in default, SIMD, and CUDA
      build trees.
- [x] Added active-set transition `BM_LCP_COMPARE` rows for the same standard,
      boxed, and coupled friction-index generated packets across
      manifest-supporting solvers. The standard rows intentionally skip
      `DirectSolver` because 16-row direct solves dispatch to the Dantzig
      fallback.
- [x] Added 49 `BM_LcpLargerActiveSetTransition` benchmark rows for the scoped
      scalable active-set transition packets covering standard 32-row, boxed
      32-row, and coupled friction-index 8-contact cases, verified in default,
      SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 49 `BM_LcpStressActiveSetTransition` benchmark rows for the scoped
      scalable active-set transition packets covering standard 64-row, boxed
      64-row, and coupled friction-index 12-contact cases, verified in default,
      SIMD-enabled, and CUDA-enabled build trees.
- [x] Added extreme generated active-set transition coverage and 49
      `BM_LcpExtremeActiveSetTransition` rows for scoped standard 128-row,
      boxed 128-row, and coupled friction-index 16-contact packets, verified in
      default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added production active-set transition coverage and 48
      `BM_LcpProductionActiveSetTransition` rows for stronger-coupled
      24-contact/72-row, 32-contact/96-row, and 48-contact/144-row
      friction-index packets, verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
- [x] Added 96
      `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
      batch-size-4 runs over the same stronger-coupled 24-contact/72-row and
      32-contact/96-row and 48-contact/144-row friction-index packets, verified
      in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 192 `BM_LcpMildIllConditioned` benchmark rows for the scoped larger
      mildly ill-conditioned standard 32-row, boxed 16-row, friction-index
      8-contact, coupled friction-index 6-, 8-, 12-, 16-, and 24-contact
      packets, stronger-coupled 16-/24-contact packets with 4x and 8x
      cross-contact coupling, a stronger-coupled 32-contact packet with 8x
      cross-contact coupling, and ADMM/SAP-only 16x-coupled 16-/24-/32-/48-contact
      packets, verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 156 `BM_LcpMildIllConditionedBatch(Serial|Parallel)` benchmark rows
      for batch-size-4 runs over the 4x-coupled 16-/24-contact, 8x-coupled
      16-/24-/32-contact, and ADMM/SAP-only 16x-coupled
      16-/24-/32-/48-contact mildly ill-conditioned friction-index packets,
      verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 21 `BM_LcpNearSingular` benchmark rows for the scoped robust
      near-singular standard 8-row, boxed 8-row, and coupled friction-index
      3-, 6-, 9-, 12-, 16-, 24-, 32-, and 48-contact packets, verified in
      default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 32 `BM_LcpNearSingularBatch(Serial|Parallel)` benchmark rows for
      batch-size-4 runs over the coupled friction-index 3-, 6-, 9-, 12-, 16-,
      24-, 32-, and 48-contact near-singular packets, verified in default,
      SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 27 `BM_LcpSingularDegenerate` benchmark rows for the exact
      rank-deficient standard 16-row, boxed 16-row, and coupled friction-index
      6-contact packets over the same solver scope proven by generated
      coverage.
- [x] Added 27 `BM_LcpLargerSingularDegenerate` benchmark rows for the larger
      exact rank-deficient standard 32-row, boxed 32-row, and coupled
      friction-index 8-contact packets, verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
- [x] Added a stress singular-degenerate generated known-solution slice and 27
      matching `BM_LcpStressSingularDegenerate` benchmark rows for exact
      rank-deficient standard 64-row, boxed 64-row, and coupled
      friction-index 12-contact packets, verified in default, SIMD-enabled,
      and CUDA-enabled build trees.
- [x] Added extreme exact rank-deficient singular-degenerate coverage and 36
      `BM_LcpExtremeSingularDegenerate` rows for scoped standard 128-row,
      boxed 128-row, and coupled friction-index 16-/24-/32-/48-contact packets,
      verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Fixed boxed semi-smooth Newton's `findex` moving-bound Jacobian so the
      coupled mildly ill-conditioned 4-contact generated case satisfies the
      all-solver contract.
- [x] Added a DART 7 `dart::simulation::World` boxed-LCP contact snapshot test that
      validates a real sphere-ground friction contact LCP assembled from
      `World::collide()`.
- [x] Extended the DART 7 boxed-LCP contact snapshot evidence to a two-contact
      world with separated sphere-ground contacts assembled into one
      boxed/friction-index LCP.
- [x] Added a DART 7 boxed-LCP end-to-end `World::step()` invariant test for two
      independent sphere-ground contacts advanced over 200 steps.
- [x] Added a DART 7 boxed-LCP end-to-end `World::step()` invariant test for
      four independent sphere-ground contacts advanced over 200 steps, plus
      4-/8-contact separated sphere-ground step benchmark rows.
- [x] Extended denser separated-contact DART 7 boxed-LCP end-to-end evidence to
      16 independent sphere-ground contacts advanced over 200 steps, plus a
      16-contact separated sphere-ground step benchmark row.
- [x] Added DART 7 boxed-LCP articulated contact evidence for fixed-base
      prismatic links in one-link and four-link ground-contact scenes, plus
      1-/4-/8-/16-link articulated ground-step benchmark rows through the
      public unified constraint path.
- [x] Added DART 7 boxed-LCP connected multi-DOF articulated contact evidence
      for fixed-base three-axis prismatic Cartesian chains in ground contact,
      plus 1-/4-/8-/16-chain articulated Cartesian ground-step benchmark rows
      through the public unified constraint path.
- [x] Added DART 7 boxed-LCP two-sided articulated contact evidence for a
      fixed-base prismatic link pushing a dynamic rigid body, plus
      1-/4-/8-/16-pair articulated rigid-impact benchmark rows.
- [x] Added DART 7 boxed-LCP cross-multibody articulated contact evidence for a
      fixed-base prismatic link pushing a prismatic link owned by another
      multibody, plus 1-/4-/8-/16-pair articulated link-impact benchmark rows.
- [x] Added DART 7 articulated unified-contact benchmark rows that compare all
      friction-index-capable solvers on the same manually assembled fixed-base
      three-axis prismatic link-ground, link-vs-dynamic-rigid, and
      cross-multibody link-vs-link LCP snapshots.
- [x] Added a DART 7 boxed-LCP coupled stack snapshot test for a 3-sphere
      vertical stack, validating the LCP contract and nonzero normal-contact
      coupling from shared dynamic bodies.
- [x] Extended coupled DART 7 boxed-LCP stack snapshot evidence to a 4-sphere
      vertical stack, validating a 4-contact, 12-row boxed/findex LCP assembled
      from shared dynamic bodies.
- [x] Extended coupled DART 7 boxed-LCP stack snapshot evidence to a 5-sphere
      vertical stack, validating a 5-contact, 15-row boxed/findex LCP assembled
      from shared dynamic bodies.
- [x] Added a DART 7 boxed-LCP end-to-end `World::step()` invariant test and
      benchmark row for a 3-sphere vertical stack advanced through the public
      contact pipeline for 200 steps.
- [x] Extended the same DART 7 boxed-LCP coupled stack end-to-end evidence to a
      500-step 3-sphere vertical-stack run and matching benchmark row.
- [x] Added timestep-driven Baumgarte velocity bias to the DART 7 boxed-LCP
      rigid contact solve and verified a 4-sphere coupled-stack public
      `World::step(200)` invariant test plus a matching benchmark row.
- [x] Extended DART 7 boxed-LCP coupled stack public-step evidence to a
      5-sphere stack that satisfies the same finite-state, spacing, near-rest,
      lateral-drift, and static-ground invariants after `World::step(500)`,
      with a matching benchmark row in default, SIMD-enabled, and CUDA-enabled
      build trees.
- [x] Added DART 7 world-contact `BM_LCP_COMPARE` rows that run every
      friction-index-capable solver on the same boxed-LCP snapshots assembled
      from 1, 2, and 4 separated sphere-ground contacts, plus a
      contact assembly/solve benchmark for the boxed-LCP world path.
- [x] Added coupled DART 7 world-contact stack benchmark rows for 2- and
      3-sphere vertical stacks across every friction-index-capable solver, plus
      4- and 5-sphere vertical-stack rows for the same solver set except
      `NNCG`, so the comparison now includes larger boxed/findex snapshots
      where contacts share dynamic bodies.
- [x] Added mixed DART 7 world-contact batch benchmark rows that compare every
      friction-index-capable solver over the same 5-problem batch of separated
      sphere-ground and coupled stack snapshots, both serially and through the
      experimental `ParallelExecutor`.
- [x] Added stress mixed DART 7 world-contact batch rows over 1/2/4 separated
      sphere-ground plus 2/3/4/5-sphere coupled stack snapshots, both serially
      and through `ParallelExecutor`, for all friction-index-capable solvers
      except `NNCG`.
- [x] Added DART 7 dense box-face contact evidence: a 4-contact, 12-row,
      single-dynamic-body boxed/findex snapshot assembled from
      `World::collide()`, APGD-verified in the unit test, plus 48 scoped
      `BM_LcpWorldBoxContact/FrictionIndex` benchmark rows over
      1/2/4/8/16/24/32/48-box snapshots verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
- [x] Added contact-derived CUDA batch evidence for homogeneous 4-, 8-, and
      16-contact DART 7 world-contact packets, covering fixed-iteration CUDA
      Jacobi and PGS unit tests and benchmark rows on the visible GPU.
- [x] Added contact-derived CUDA batch evidence for homogeneous 5-sphere
      DART 7 coupled stack-contact packets, covering fixed-iteration CUDA
      Jacobi and PGS unit tests and benchmark rows on the visible GPU.
- [x] Added grouped variable-size synthetic CUDA batch evidence for DART 7
      standard 16/32/48-row, boxed 16/32/48-row, and friction-index
      4/8/16-contact packets, covering fixed-iteration CUDA Jacobi and PGS
      unit tests and benchmark rows on the visible GPU.
- [x] Added grouped variable-size CUDA contact-batch evidence for DART 7
      1/2/4/8/16-contact separated sphere-ground packets, covering
      fixed-iteration CUDA Jacobi and PGS unit tests and benchmark rows on the
      visible GPU.
- [x] Added grouped variable-size CUDA contact-batch evidence for DART 7
      2/3/4/5-sphere coupled stack-contact packets, covering fixed-iteration
      CUDA Jacobi and PGS unit tests and benchmark rows on the visible GPU.
- [x] Added grouped variable-size CUDA contact-batch evidence for DART 7
      manually assembled fixed-base three-axis prismatic articulated
      unified-contact packets with 1, 4, and 8 contacts, covering fixed-iteration
      CUDA Jacobi and PGS unit tests and benchmark rows on the visible GPU.
- [x] Added mixed grouped CUDA contact-batch evidence for DART 7 separated
      sphere-ground, coupled stack-contact, and manually assembled articulated
      unified-contact packets including cross-multibody link-vs-link cases in
      one size-grouped batch, covering fixed-iteration CUDA Jacobi and PGS unit
      tests and benchmark rows on the visible GPU.
- [x] Added dense box-face CUDA contact-batch evidence for DART 7:
      homogeneous 4-problem 1-/4-/8-/16-/24-/32-/48-box and grouped variable-size
      1/2/4/8/16/24/32-box batches of box-face `World::collide()` snapshots pass
      fixed-iteration CUDA PGS benchmark coverage on the visible GPU; CUDA PGS
      unit coverage now includes homogeneous 1-/16-/24-/32-/48-box packets and
      the grouped 1/2/4/8/16/24/32-box packet.
      Fixed-iteration CUDA Jacobi was tried on the dense 4-contact patch and is
      not claimed: 4096 iterations with relaxation 1.0 failed the LCP contract with
      residual/complementarity/bound violations of about 3.3e-2 to 5.0e-2
      (`w must be non-negative at lo`), while 8192 iterations with relaxation
      0.25 drove residual and bounds near zero but still failed fixed-variable
      complementarity at about 0.34 to 0.435.
- [x] Added dense box-face DART 7 end-to-end unit and benchmark evidence:
      `FourBoxWorldStepMaintainsDenseContactInvariants` and
      `EightBoxWorldStepMaintainsDenseContactInvariants` advance 4-box and
      8-box dense face-contact scenes through public boxed-LCP `World::step()`;
      `SixteenBoxWorldStepMaintainsDenseContactInvariants` extends the same
      scene family to 16 boxes with a longer 500-step horizon, and
      `TwentyFourBoxWorldStepMaintainsDenseContactInvariants` extends unit
      coverage to a 24-box/96-contact small-timestep scene. The
      `BM_LcpWorldBoxStep_BoxedLcp` rows report matching invariant counters for
      4/8/16/32-contact 200-step scenes, the 64-contact 500-step scene, the
      96-contact 2000-step scene, and the 128-/192-contact 4000-step scenes.
- [x] Added DART 7 per-contact block-structure evidence for BGS and
      Blocked Jacobi on real two-contact boxed-LCP world-contact snapshots:
      the tests prove `findex`-derived non-contiguous contact blocks solve the
      snapshot and explicit block partitions that split normal/tangent rows are
      rejected. A focused `BM_LCP_COMPARE` gate also verifies 22 BGS/Blocked
      Jacobi world-contact, stack-contact, and serial/parallel batch rows with
      `contract_ok=1`.
- [x] Added focused BGS/Blocked Jacobi block-partition sweep benchmark rows for
      full-block, 3-row block, auto `findex`, and explicit contact-block
      partitions on standard, boxed, and friction-index fixtures.
- [x] Added opt-in projected gradient-descent warm starts for the native
      standard-LCP paths of Minimum Map, Fischer-Burmeister, and Penalized
      Fischer-Burmeister Newton, with focused tests proving each initializer
      reduces its solver-specific merit before Newton line search.
- [x] Added opt-in PGS warm starts for the same native standard-LCP Newton
      paths, with focused tests proving each accepted seed reduces its
      solver-specific merit before Newton line search.
- [x] Added 24 `BM_LcpNewtonWarmStart` benchmark rows that compare no seed,
      PGS, projected gradient descent, and PGS-then-gradient modes for the
      three native standard-LCP Newton solvers on identical 32-row and 64-row
      active-set transition packets, verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
- [x] Added 48
      `BM_LcpNewtonWarmStartBatch(Serial|Parallel)` benchmark rows that run the
      same Newton warm-start mode matrix over batch-size-4 standard active-set
      transition packets, both serially and through DART 7 `ParallelExecutor`,
      verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [ ] Continue expanding synthetic coverage beyond the current production-scale,
      larger mildly ill-conditioned, extreme singular-degenerate, and extreme
      active-set transition benchmark slices into harder solver-specific
      friction-index conditioning/coupling cases and broader backend execution
      evidence beyond CPU solver rows in SIMD/CUDA-enabled builds.
- [ ] Continue extending DART 7 end-to-end contact cases through the public
      simulation pipeline beyond current separated contacts, fixed-base
      articulated contacts, 5-sphere coupled stacks, and 24-box unit /
      48-box benchmark dense face-contact step coverage.
- [ ] Broaden apples-to-apples benchmark packets from generated problems and
      simple world-contact snapshots to broader dense/robot-like end-to-end
      contact systems, with scalar, SIMD, threaded, CUDA, and broader batch
      evidence separated.

## Goal

Fully implement the LCP solver suite described in `docs/background/lcp/` for
DART 7, with correctness tests and performance benchmarks that make solver
tradeoffs evidence based.

## Current Evidence

- Source taxonomy: `docs/background/lcp/02_overview.md`,
  `03_pivoting-methods.md`, `04_projection-methods.md`,
  `05_newton-methods.md`, `06_other-methods.md`, and
  `07_selection-guide.md`.
- Implementation surface: `dart/math/lcp/all.hpp` exports pivoting,
  projection, Newton, and other solver families used by the comparison harness.
- Correctness harness: `tests/unit/math/lcp/test_lcp_comparison_harness.cpp`
  checks residuals, effective bounds, complementarity, validation, and expected
  solutions through `tests/common/lcpsolver/lcp_test_harness.hpp`.
- Verified initial comparison expansion:
  `cmake --build build/default/cpp/Release --target UNIT_math_lcp_math_lcp_lcp_comparison_harness`,
  then
  `./build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_comparison_harness '--gtest_filter=LcpComparisonHarness.*'`
  passed 43 tests.
- Verified manifest and coverage slice:
  `cmake --build build/default/cpp/Release --parallel 5 --target UNIT_math_lcp_math_lcp_all_solvers_smoke UNIT_math_lcp_math_lcp_lcp_comparison_harness BM_LCP_COMPARE`,
  `./build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke --gtest_filter='AllSolversSmokeTest.*:AdvancedBoxedSolvers.*'`
  passed 10 tests,
  `./build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_comparison_harness --gtest_filter='LcpComparisonHarness.*'`
  passed 44 tests, and
  `./build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='BM_LCP_COMPARE_SMOKE' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran successfully.
- Verified manifest benchmark registration slice:
  `cmake --build build/default/cpp/Release --parallel 4 --target BM_LCP_COMPARE`
  passed,
  `./build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_list_tests`
  listed manifest-generated `BM_LcpCompare/<family>/<solver>/<size>` entries
  for 24 standard-capable solvers, 16 boxed-capable solvers, and 16
  friction-index-capable solvers, and
  `./build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='BM_LcpCompare/(Standard/Dantzig|Boxed/ShockPropagation|FrictionIndex/NNCG)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran successfully with residual, complementarity, bound-violation,
  `contract_ok`, iteration, and problem-size counters.
- Verified pivoting scale benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpPivotingScaleSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpPivotingScaleSweep` reported 12
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover Direct 2-row and 3-row standard fixtures,
  Lemke and Baraff 8-row and 16-row standard fixtures, and Dantzig 8-row and
  16-row standard, 12-row and 24-row boxed, and 4-contact and 8-contact
  friction-index fixtures. The rows report `pivoting_scale_sweep=1`, Direct
  no-fallback counters, four Dantzig boxed-or-findex rows,
  `contact_count=4/8`, observed solver `iterations=1/4/8/16`, and backend
  build-state counters. The CUDA-enabled rows report `build_cuda_enabled=1`
  but are CPU pivoting solver rows in a CUDA-enabled build, not CUDA LCP kernel
  execution. Focused pivoting unit coverage
  `DantzigMatrixCoverage.*:LemkeSolverCoverage.*:DantzigSolverCoverage.*:DirectSolverCoverage.*:BaraffSolverCoverage.*`
  passed 42 tests.
- Verified batch benchmark registration slice:
  `./build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_list_tests`
  listed 56 manifest-generated `BM_LcpBatchSerial/<family>/<solver>` entries
  (24 standard, 16 boxed, 16 friction-index) and 56 manifest-generated
  `BM_LcpBatchParallel/<family>/<solver>` entries with the same family counts.
  `./build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='BM_LcpBatch(Serial|Parallel)/(Standard/Dantzig|Boxed/BoxedSemiSmoothNewton|FrictionIndex/BGS)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran successfully with `batch_size`, `total_problem_size`, maximum residual,
  maximum complementarity, maximum bound-violation, average-iteration, and
  `contract_ok` counters. The parallel rows also reported `worker_count=20`,
  `parallel_units=4`, `profile_enabled=1`, and observed `max_parallelism=4` for
  the focused standard, boxed, and friction-index rows.
- Verified backend self-reporting counters:
  `BM_LCP_COMPARE` rows now report `build_simd_enabled`,
  `build_simd_force_scalar`, `build_cuda_enabled`, and
  `has_simulation_experimental`. The default DART 7 build slice reported
  `build_simd_enabled=0`, `build_cuda_enabled=0`,
  `has_simulation_experimental=1`, and `contract_ok=1` for the focused serial
  batch, task-parallel batch, threaded validation, and smoke rows.
- Verified focused SIMD-enabled CPU slice:
  `pixi run cmake -G Ninja -S . -B build/simd/cpp/Release ... -DDART_ENABLE_SIMD=ON -DDART_ENABLE_EXPERIMENTAL_CUDA=OFF -DDART_BUILD_SIMULATION_EXPERIMENTAL=ON`
  configured DART 7.0.0 with SIMD enabled, then
  `cmake --build build/simd/cpp/Release --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_generated_coverage`
  passed. After adding the stress, extreme, and production active-set slices,
  `build/simd/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.*'`
  passed 21 tests. The focused `BM_LCP_COMPARE` mild slice reported
  `build_simd_enabled=1`, `build_cuda_enabled=0`,
  `has_simulation_experimental=1`, and `contract_ok=1` for all 170
  `BM_LcpMildIllConditioned` rows, and the focused
  `BM_LcpStressActiveSetTransition` slice reported
  `build_simd_enabled=1`, `build_cuda_enabled=0`,
  `active_set_transition=1`, `stress_active_set_transition=1`, and
  `contract_ok=1` for all 49 rows. The focused
  `BM_LcpSingularDegenerate`, `BM_LcpLargerSingularDegenerate`, and
  `BM_LcpStressSingularDegenerate` slices
  reported `build_simd_enabled=1`, `build_cuda_enabled=0`,
  `rank_deficient=1`, and `contract_ok=1` for all 27 rows in each slice.
  Earlier focused SIMD benchmark rows also
  reported the same backend counters and `contract_ok=1` for the selected
  standard, boxed, friction-index, serial batch, task-parallel batch, threaded
  validation, and smoke rows.
- Verified focused CUDA-enabled DART 7 build/runtime slice:
  `pixi run -e cuda build-cuda Release` configured DART 7.0.0 with
  `DART_ENABLE_EXPERIMENTAL_CUDA=ON`, `DART_CUDA_ARCHITECTURES=89`,
  `DART_ENABLE_SIMD=OFF`, and `DART_BUILD_SIMULATION_EXPERIMENTAL=ON`, then
  built the documented CUDA smoke targets. `pixi run -e cuda test-cuda Release`
  passed 3 `simulation-experimental-cuda` CTest targets and ran the
  `bm_cuda_rigid_body_state_batch` and `bm_vbd_cuda` smoke filters on the
  visible NVIDIA GeForce RTX 4080 Laptop GPU. In the same CUDA-enabled build,
  `cmake --build build/cuda/cpp/Release --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_generated_coverage`
  passed,
  `build/cuda/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.*'`
  passed 21 tests after adding the stress, extreme, and production active-set
  slices. The
  focused `BM_LCP_COMPARE` mild slice reported
  `build_cuda_enabled=1`, `build_simd_enabled=0`,
  `has_simulation_experimental=1`, and `contract_ok=1` for all 170
  `BM_LcpMildIllConditioned` rows, and the focused
  `BM_LcpStressActiveSetTransition` slice reported
  `build_cuda_enabled=1`, `build_simd_enabled=0`,
  `active_set_transition=1`, `stress_active_set_transition=1`, and
  `contract_ok=1` for all 49 rows. These are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution. The focused
  `BM_LcpSingularDegenerate`, `BM_LcpLargerSingularDegenerate`, and
  `BM_LcpStressSingularDegenerate` slices
  reported `build_cuda_enabled=1`, `build_simd_enabled=0`,
  `rank_deficient=1`, and `contract_ok=1` for all 27 rows in each slice.
  Earlier focused CUDA-enabled benchmark rows
  also reported the same backend counters and `contract_ok=1` for the selected
  standard, boxed, friction-index, serial batch, task-parallel batch, threaded
  validation, and smoke rows.
- Verified CUDA LCP execution slice:
  `cmake --build build/cuda/cpp/Release --parallel 5 --target dart-simulation-experimental-cuda test_lcp_jacobi_batch_cuda BM_LCP_COMPARE`
  passed. `build/cuda/cpp/Release/bin/test_lcp_jacobi_batch_cuda --gtest_brief=1`
  passed 31 tests, including standard, boxed, friction-index, grouped
  variable-size synthetic standard/boxed/friction-index CUDA batches,
  homogeneous contact-derived world-contact CUDA batches, homogeneous 5-sphere
  coupled stack-contact CUDA batches, and grouped variable-size 1/2/4/8/16-contact
  separated, 2/3/4/5-sphere coupled stack world-contact, and manually assembled
  1-/4-/8-contact articulated unified-contact CUDA batches covering link-ground,
  link-vs-dynamic-rigid, and cross-multibody link-vs-link packets, plus mixed
  grouped contact batches combining the separated, stack, and articulated
  fixture families for Jacobi and PGS on the visible GPU.
  `build/cuda/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='BM_LcpBatchSerial/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)|BM_LcpBatchParallel/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)|BM_LcpCuda(Jacobi|Pgs)Batch|BM_LcpCuda(Jacobi|Pgs)GroupedBatch|BM_LcpCuda(Jacobi|Pgs)WorldContactBatch|BM_LcpCuda(Jacobi|Pgs)WorldContactGroupedBatch' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran successfully. The CUDA rows reported `build_cuda_enabled=1`,
  `cuda_lcp_execution=1`, `cuda_batch_execution=1`, fixed iteration counters,
  and `contract_ok=1` for standard, boxed, friction-index, grouped synthetic
  standard/boxed/friction-index, and homogeneous 4-, 8-, and 16-contact world-contact
  Jacobi and PGS batches, plus grouped variable-size 1/2/4/8/16-contact
  separated sphere-ground and 2/3/4/5-sphere coupled stack CUDA batches. A
  focused
  grouped synthetic follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)GroupedBatch_(Standard|Boxed|FrictionIndex)/2' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported six grouped CUDA rows with `contract_ok=1`, `cuda_group_count=3`,
  and `batch_size=6`. The standard and boxed rows reported
  `min_problem_size=16`, `max_problem_size=48`, and `total_problem_size=192`;
  the friction-index rows reported `min_contact_count=4`,
  `max_contact_count=16`, `total_contact_count=56`, `min_problem_size=12`,
  `max_problem_size=48`, and `total_problem_size=168`. A focused homogeneous
  16-contact follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)WorldContactBatch_FrictionIndex/16/4' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two homogeneous CUDA rows with `contract_ok=1`, `contact_count=16`,
  `problem_size=48`, `batch_size=4`, `total_contact_count=64`, and
  `total_problem_size=192`. A focused homogeneous stack follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)WorldStackContactBatch_FrictionIndex/5/4' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two stack CUDA rows with `contract_ok=1`, `sphere_count=5`,
  `contact_count=5`, `problem_size=15`, `batch_size=4`,
  `total_contact_count=20`, and `total_problem_size=60`. A focused
  separated-contact follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)WorldContactGroupedBatch_FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two separated CUDA rows with `contract_ok=1`, `cuda_group_count=5`,
  `contact_shape_count=5`, `max_problem_size=48`, `total_contact_count=62`,
  and `total_problem_size=186`. A focused stack follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)WorldStackContactGroupedBatch_FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two stack CUDA rows with `contract_ok=1`, `cuda_group_count=4`,
  `contact_shape_count=4`, `max_problem_size=15`, `total_contact_count=28`,
  and `total_problem_size=84`. A focused articulated unified-contact follow-up
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract' --gtest_brief=1`
  passed both CUDA grouped-batch tests, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)ArticulatedUnifiedContactGroupedBatch_FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two articulated CUDA rows with `contract_ok=1`, `batch_size=18`,
  `cuda_group_count=3`, `contact_shape_count=3`,
  `articulated_contact_case_count=3`, `articulated_cross_link_contact=1`,
  `min_problem_size=3`, `max_problem_size=24`, `total_contact_count=78`, and
  `total_problem_size=234`. A focused mixed-contact follow-up
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.MixedContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.MixedContactGroupedBatchSatisfiesLcpContract' --gtest_brief=1`
  passed both CUDA grouped-batch tests, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)MixedContactGroupedBatch_FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two mixed CUDA rows with `contract_ok=1`,
  `batch_size=20`, `contact_fixture_family_count=3`, `cuda_group_count=4`,
  `contact_shape_count=4`, `articulated_contact_case_count=3`,
  `articulated_cross_link_contact=1`, `min_problem_size=3`,
  `max_problem_size=15`, `total_contact_count=54`, and
  `total_problem_size=162`. This proves narrow
  CUDA projected-Jacobi and
  PGS batch LCP paths; it does not prove CUDA execution for the full solver
  manifest or for dense contact CUDA batches, or end-to-end articulated
  world-step CUDA execution.
- Formatting evidence this session: `pixi run lint` completed successfully
  after the C++ harness change, after the implementation audit was added, and
  after the generated coverage, batch benchmark, and dev-task evidence updates.
- Implementation audit:
  `docs/dev_tasks/lcp_solvers/01-implementation-audit.md` maps each
  documented/background solver and each currently exported DART 7 LCP solver to
  implementation, manifest support, correctness, benchmark, backend, and
  remaining-gap evidence.
- Generated correctness coverage:
  `tests/unit/math/lcp/test_lcp_generated_coverage.cpp` constructs deterministic
  known-solution standard, boxed, and friction-index LCPs and runs every
  manifest-supported solver against the matching family. It covers standard
  sizes 1/2/4/8/16 plus mildly ill-conditioned 4D and 8D standard cases, boxed
  sizes 2/4/8/12 plus mildly ill-conditioned 4D and 8D boxed cases and a
  near-singular 4D boxed case with lower, upper, interior, and unbounded-upper
  variables, friction-index contact counts 1/2/4/8 with mixed cone activity,
  coupled friction-index 2-contact, 4-contact, and 6-contact well-conditioned
  cases, coupled mildly ill-conditioned 2-contact and 4-contact friction-index
  cases, batch-shaped solver reuse across families, pivoting-solver
  near-singular standard cases, and invalid-problem rejection with diagnostic
  messages across the full manifest. Separate scoped larger-case tests cover
  standard 32-row and 64-row, boxed 32-row, friction-index 16-contact, and
  coupled friction-index 8-contact known-solution cases; production-scale
  standard 128-row, boxed 64-row, friction-index 24-contact, and coupled
  friction-index 12-contact known-solution cases; larger mildly
  ill-conditioned standard 32-row and 64-row, boxed 16-row and 32-row,
  friction-index 8-contact, and coupled friction-index 6-, 8-, 12-, 16-, and
  24-contact known-solution cases, plus an ADMM/SAP-only 16x-coupled
  16-/24-/32-/48-contact mildly ill-conditioned friction-index slice;
  near-singular standard 8-row, boxed 8-row, and coupled friction-index 3-, 6-,
  9-, 12-, 16-, 24-, 32-, and 48-contact known-solution cases;
  exact rank-deficient
  singular-degenerate standard 16-row, boxed 16-row, and coupled friction-index
  6-contact known-solution cases; and larger exact rank-deficient
  singular-degenerate standard 32-row, boxed 32-row, and coupled friction-index
  8-contact known-solution cases; and stress exact rank-deficient
  singular-degenerate standard 64-row, boxed 64-row, and coupled friction-index
  12-contact known-solution cases for solver sets selected by observed
  robustness. `MPRGP` is not included in the larger
  mildly ill-conditioned standard slice because a focused 32-row trial
  satisfied the LCP contract but missed the selected expected solution
  tolerance. The singular-degenerate slice excludes `Direct` from the 16-row
  standard packet because that path would measure its Dantzig fallback, excludes
  `PenalizedFischerBurmeisterNewton` after a standard packet contract trial,
  and keeps boxed/findex evidence to `Admm`, `Sap`, and
  `BoxedSemiSmoothNewton` after the other tried boxed/findex paths did not
  satisfy the generated contract. Active-set transition tests cover standard
  16-row, boxed 16-row, and coupled friction-index 6-contact known-solution
  cases near lower, upper, and friction-cone boundaries across
  manifest-supporting solvers, plus scoped scalable-solver cases for standard
  32-row, 64-row, and 128-row, boxed 32-row, 64-row, and 128-row, coupled
  friction-index 8-contact, 12-contact, and 16-contact packets, plus a
  stronger-coupled production coupled friction-index 24-contact packet and a
  stronger 32-contact packet.
- Verified generated coverage slice:
  `cmake --build build/default/cpp/Release --parallel "$JOBS" --target UNIT_math_lcp_math_lcp_all_solvers_smoke UNIT_math_lcp_math_lcp_lcp_comparison_harness UNIT_math_lcp_math_lcp_lcp_generated_coverage`,
  then
  `UNIT_math_lcp_math_lcp_all_solvers_smoke --gtest_filter='AllSolversSmokeTest.*:AdvancedBoxedSolvers.*'`
  passed 10 tests,
  `UNIT_math_lcp_math_lcp_lcp_comparison_harness --gtest_filter='LcpComparisonHarness.*'`
  passed 44 tests, and
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.*'`
  passed 21 tests in the default, SIMD-enabled, and CUDA-enabled build trees
  after adding the stress, extreme, and production active-set slices.
- Verified active-set transition benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpActiveSetTransition/' | wc -l`
  reported 55 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpActiveSetTransition' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1`. The rows cover standard 16-row, boxed
  16-row, and coupled friction-index 6-contact packets and set
  `active_set_transition=1`; the friction-index rows also report
  `contact_count=6`. `DirectSolver` is omitted from the standard rows because
  the 16-row case would measure its Dantzig fallback rather than the direct
  solver. Focused standard/boxed/friction-index active-set rows also passed in
  `build/simd/cpp/Release` with `build_simd_enabled=1` and in
  `build/cuda/cpp/Release` with `build_cuda_enabled=1`.
- Verified Newton warm-start benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNewtonWarmStart/' | wc -l`
  reported 24 rows, and
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpNewtonWarmStart/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover `MinimumMapNewton`,
  `FischerBurmeisterNewton`, and
  `PenalizedFischerBurmeisterNewton` on the same standard active-set transition
  packets at 32 and 64 rows. The rows report `active_set_transition=1`,
  `newton_pgs_warm_start`, `newton_gradient_warm_start`,
  `newton_pgs_warm_start_iterations`, and
  `newton_gradient_warm_start_iterations`, so no-seed, PGS-only,
  gradient-only, and PGS-then-gradient modes are distinguishable in the JSON
  output. The SIMD-enabled rows report `build_simd_enabled=1`; the
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified Newton warm-start batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNewtonWarmStartBatch' | wc -l`
  reported 48 rows, and
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpNewtonWarmStartBatch' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover batch-size-4 serial and DART 7
  `ParallelExecutor` runs for `MinimumMapNewton`, `FischerBurmeisterNewton`,
  and `PenalizedFischerBurmeisterNewton` on the same 32-row and 64-row
  standard active-set transition packets and the same no-seed, PGS-only,
  gradient-only, and PGS-then-gradient mode matrix. The rows report
  `newton_warm_start_batch=1`, `batch_size=4`, `total_problem_size=128/256`,
  warm-start mode/iteration counters, serial or parallel execution counters,
  and backend build-state counters. Parallel rows report `parallel_units=4`,
  `worker_count=20`, and observed `max_parallelism` values up to 4. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU solver batch rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified PGS/PSOR relaxation sweep benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpPgsRelaxationSweep' | wc -l`
  reported 9 rows, and JSON checks for
  `BM_LcpPgsRelaxationSweep` reported 9 rows with `contract_ok=1` in the
  default, SIMD-enabled, and CUDA-enabled build trees. These rows cover the
  standard 48-row, boxed 24-row, and friction-index 8-contact benchmark
  fixtures at relaxation 0.5, 1.0, and 1.3. The rows report
  `pgs_relaxation_sweep=1`, `psor_relaxation`,
  `psor_under_relaxation`, `psor_plain_pgs`, `psor_over_relaxation`,
  `problem_size=24/48`, `contact_count=8` for the friction-index rows, and
  backend build-state counters. The CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU PGS solver rows in a CUDA-enabled build,
  not CUDA LCP kernel execution.
- Verified symmetric PSOR relaxation sweep benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSymmetricPsorRelaxationSweep' | wc -l`
  reported 9 rows, and JSON checks for
  `BM_LcpSymmetricPsorRelaxationSweep` reported 9 rows with `contract_ok=1` in
  the default, SIMD-enabled, and CUDA-enabled build trees. These rows cover the
  standard 48-row, boxed 24-row, and friction-index 8-contact benchmark
  fixtures at relaxation 0.5, 1.0, and 1.3. The rows report
  `symmetric_psor_relaxation_sweep=1`, `psor_relaxation`,
  `psor_under_relaxation`, `psor_plain_symmetric_psor`,
  `psor_over_relaxation`, `problem_size=24/48`, `contact_count=8` for the
  friction-index rows, and backend build-state counters. The CUDA-enabled rows
  report `build_cuda_enabled=1` but are CPU symmetric PSOR solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified Red-Black Gauss-Seidel relaxation sweep benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpRedBlackGaussSeidelRelaxationSweep' | wc -l`
  reported 9 rows, and JSON checks for
  `BM_LcpRedBlackGaussSeidelRelaxationSweep` reported 9 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover the standard 48-row, boxed 24-row, and friction-index
  8-contact benchmark fixtures at relaxation 0.5, 1.0, and 1.3. The rows
  report `red_black_gauss_seidel_relaxation_sweep=1`,
  `red_black_color_count=2`, `red_black_red_rows=12/24`,
  `red_black_black_rows=12/24`, `psor_relaxation`,
  `psor_under_relaxation`, `psor_plain_red_black_gauss_seidel`,
  `psor_over_relaxation`, `problem_size=24/48`, `contact_count=8` for the
  friction-index rows, and backend build-state counters. The CUDA-enabled rows
  report `build_cuda_enabled=1` but are CPU Red-Black Gauss-Seidel solver rows
  in a CUDA-enabled build, not CUDA LCP kernel execution. These rows expose the
  even/odd two-color partition; they do not prove solver-internal threaded
  Red-Black execution.
- Verified APGD restart-policy benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpApgdRestartSweep' | wc -l`
  reported 9 rows, and JSON checks for `BM_LcpApgdRestartSweep` reported 9
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-contact fixtures for adaptive restart every iteration,
  adaptive restart every 5 iterations, and no restart. The rows report
  `apgd_restart_sweep=1`, `apgd_adaptive_restart`,
  `apgd_restart_check_interval`, `apgd_relaxation=1`, `problem_size=24/48`,
  `contact_count=8` for the friction-index rows, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU
  APGD solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified TGS iteration-budget benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpTgsIterationBudgetSweep' | wc -l`
  reported 9 rows, and JSON checks for `BM_LcpTgsIterationBudgetSweep`
  reported 9 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-contact fixtures at 10-, 50-, and 100-iteration budgets.
  The rows report `tgs_iteration_budget_sweep=1`, `tgs_max_iterations`,
  `tgs_relaxation=1`, `problem_size=24/48`, `contact_count=8` for the
  friction-index rows, observed `iterations=5/6`, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU TGS
  solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified NNCG PGS-preconditioner iteration benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNncgPgsIterationsSweep' | wc -l`
  reported 9 rows, and JSON checks for `BM_LcpNncgPgsIterationsSweep`
  reported 9 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-contact fixtures at 1, 2, and 5 PGS preconditioner
  iterations while holding NNCG restart interval 10 and restart threshold 1.0.
  The rows report `nncg_pgs_iterations_sweep=1`, `nncg_pgs_iterations`,
  `nncg_restart_interval=10`, `nncg_restart_threshold=1`, `problem_size=24/48`,
  `contact_count=8` for the friction-index rows, observed outer
  `iterations=0/2/4/5`, and backend build-state counters. The CUDA-enabled
  rows report `build_cuda_enabled=1` but are CPU NNCG solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified SubspaceMinimization PGS-iteration benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSubspaceMinimizationPgsIterationsSweep' | wc -l`
  reported 9 rows, and JSON checks for
  `BM_LcpSubspaceMinimizationPgsIterationsSweep` reported 9 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover standard 48-row, boxed 24-row, and friction-index 8-contact
  fixtures at 1, 3, and 5 PGS active-set-estimation iterations while holding
  active-set tolerance 0.0. The rows report
  `subspace_pgs_iterations_sweep=1`, `subspace_pgs_iterations`,
  `subspace_active_set_tolerance=0`, `problem_size=24/48`, `contact_count=8`
  for the friction-index rows, observed outer `iterations=1/2`, and backend
  build-state counters. The CUDA-enabled rows report `build_cuda_enabled=1`
  but are CPU SubspaceMinimization solver rows in a CUDA-enabled build, not
  CUDA LCP kernel execution.
- Verified ShockPropagation layer-layout benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpShockPropagationLayerSweep' | wc -l`
  reported 9 rows, and JSON checks for `BM_LcpShockPropagationLayerSweep`
  reported 9 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-contact fixtures with single-layer, two-layer, and serial
  layer schedules built from 3-row blocks. The rows report
  `shock_propagation_layer_sweep=1`, schedule counters, `layer_count=1/2/8/16`,
  `block_count=8/16`, `max_block_size=3`, `max_blocks_per_layer=1/4/8/16`,
  `problem_size=24/48`, `contact_count=8` for the friction-index rows, observed
  solver `iterations=3/4`, and backend build-state counters. The CUDA-enabled
  rows report `build_cuda_enabled=1` but are CPU ShockPropagation solver rows in
  a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified MPRGP SPD/positive-definite-check benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMprgpSpdCheckSweep' | wc -l`
  reported 9 rows, and JSON checks for `BM_LcpMprgpSpdCheckSweep` reported 9
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build
  trees. These rows cover dense SPD 32/64-row, banded SPD 64-row, mildly
  ill-conditioned SPD 32-row, and near-singular SPD 8-row standard-LCP fixtures
  with `MprgpSolver::Parameters::checkPositiveDefinite` enabled and disabled
  where paired. The rows report `mprgp_spd_check_sweep=1`,
  `mprgp_positive_definite_check`, SPD-kind counters, `problem_size=8/32/64`,
  observed solver `iterations=3/5/15`, `mprgp_symmetry_tolerance=1e-9`,
  `mprgp_epsilon_for_division=1e-12`, and backend build-state counters. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU MPRGP solver rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified Interior Point path-parameter benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpInteriorPointPathSweep' | wc -l`
  reported 9 rows, and JSON checks for `BM_LcpInteriorPointPathSweep` reported 9
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build
  trees. These rows cover dense SPD 32/64-row, banded SPD 64-row, mildly
  ill-conditioned SPD 32-row, and near-singular SPD 8-row standard-LCP fixtures
  with centering parameter `sigma=0.1/0.3` and step scale `0.75/0.99`. The rows
  report `interior_point_path_sweep=1`, `interior_point_sigma`,
  `interior_point_step_scale`, SPD-kind counters, `problem_size=8/32/64`,
  observed solver `iterations=14/16/27/31/32/41/51/53`, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU
  Interior Point solver rows in a CUDA-enabled build, not CUDA LCP kernel
  execution.
- Verified Staggering contact-pipeline benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpStaggeringContactPipelineSweep' | wc -l`
  reported 9 rows, and JSON checks for
  `BM_LcpStaggeringContactPipelineSweep` reported 9 rows with `contract_ok=1`
  in the default, SIMD-enabled, and CUDA-enabled build trees. These rows cover
  separated sphere-ground 1/2/4-contact fixtures, coupled vertical-stack
  2/3/5-contact fixtures, and articulated unified ground, rigid-impact, and
  cross-link-impact 4-contact fixtures. The rows report
  `staggering_contact_pipeline_sweep=1`,
  `staggering_normal_friction_split=1`, normal-row counts `1/2/3/4/5`,
  friction-row counts `2/4/6/8/10`, coupled-contact flags, contact counts
  `1/2/3/4/5`, and backend build-state counters. The CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU Staggering solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
- Verified Boxed Semi-Smooth Newton line-search benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpBoxedSemiSmoothNewtonLineSearchSweep' | wc -l`
  reported 9 rows, and JSON checks for
  `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep` reported 9 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover standard 48-row, boxed 24-row, and friction-index 8-contact
  fixtures with default line search, an expanded line-search step budget, and a
  gentler step-reduction policy. The rows report
  `boxed_ssn_line_search_sweep=1`,
  `boxed_ssn_max_line_search_steps=10/20`,
  `boxed_ssn_step_reduction=0.5/0.8`, default/more-step/gentle-reduction
  policy counters, observed solver `iterations=2/7/8/9`, `contact_count=8` for
  the friction-index rows, and backend build-state counters. The CUDA-enabled
  rows report `build_cuda_enabled=1` but are CPU BoxedSemiSmoothNewton solver
  rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified ADMM rho/adaptive-rho benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpAdmmRhoSweep' | wc -l`
  reported 18 rows, and JSON checks for `BM_LcpAdmmRhoSweep` reported 18 rows
  with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build
  trees. These rows cover standard 48-row, boxed 24-row, and friction-index
  8-contact fixtures at `rhoInit` 0.5, 1.0, and 4.0 with fixed and adaptive
  rho policies. The rows report `admm_rho_sweep=1`, `admm_rho_init`,
  `admm_fixed_rho`, `admm_adaptive_rho`,
  `admm_adaptive_rho_tolerance`, `admm_mu_prox`, `problem_size=24/48`,
  `contact_count=8` for the friction-index rows, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU
  ADMM solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified SAP regularization benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSapRegularizationSweep' | wc -l`
  reported 9 rows, and JSON checks for `BM_LcpSapRegularizationSweep` reported
  9 rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-contact fixtures at regularization values `1e-6`, `1e-5`,
  and `1e-4`. The rows report `sap_regularization_sweep=1`,
  `sap_regularization`, `sap_armijo_parameter`, `sap_backtracking_factor`,
  `sap_max_line_search_iterations`, `problem_size=24/48`, `contact_count=8`
  for the friction-index rows, and backend build-state counters. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU SAP solver rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified ADMM/SAP extreme-coupling benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditioned/ExtremeCoupledFrictionIndex' | wc -l`
  now reports 8 rows, and focused checks for
  `BM_LcpMildIllConditioned/ExtremeCoupledFrictionIndex48` reported 2 new rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover only `Admm` and `Sap` on 16x-coupled mildly ill-conditioned
  friction-index packets, now including 48 contacts/144 rows in addition to the
  existing 16 contacts/48 rows, 24 contacts/72 rows, and 32 contacts/96 rows. The
  rows report `mildly_ill_conditioned=1`, `coupled=1`,
  `coupling_scale=16`, `contact_count=16/24/32/48`,
  `problem_size=48/72/96/144`, and backend build-state counters. The
  CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU ADMM/SAP solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
- Verified ADMM/SAP/Boxed Semi-Smooth Newton contact comparison benchmark
  slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpContactSolverComparisonSweep' | wc -l`
  reported 27 rows, and JSON checks for
  `BM_LcpContactSolverComparisonSweep` reported 27 rows with `contract_ok=1`
  in the default, SIMD-enabled, and CUDA-enabled build trees. These rows run
  `Admm`, `Sap`, and `BoxedSemiSmoothNewton` over the same DART 7 separated
  sphere-ground 1/2/4-contact fixtures, coupled vertical-stack 2/3/5-contact
  fixtures, and articulated unified ground, rigid-impact, and cross-link-impact
  4-contact fixtures. The rows report
  `contact_solver_comparison_sweep=1`, solver identity counters,
  fixture-family counters, `contact_count=1/2/3/4/5`,
  `normal_row_count=1/2/3/4/5`, `friction_row_count=2/4/6/8/10`,
  `problem_size=3/6/9/12/15`, and backend build-state counters. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified contact-normal standard-LCP benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpContactNormalStandardSweep' | wc -l`
  reported 76 rows, and JSON checks for
  `BM_LcpContactNormalStandardSweep` reported 76 rows with `contract_ok=1` in
  the default, SIMD-enabled, and CUDA-enabled build trees. These rows extract
  only the normal rows from the same DART 7 separated sphere-ground 1/2/4-contact
  fixtures, coupled vertical-stack 2/3/5-contact fixtures, and articulated
  unified ground, rigid-impact, and cross-link-impact 4-contact fixtures, then
  run them as standard LCPs. The rows compare `Dantzig`, `Lemke`, `Baraff`,
  `Direct`, `MinimumMapNewton`, `FischerBurmeisterNewton`,
  `PenalizedFischerBurmeisterNewton`, `InteriorPoint`, and `MPRGP`; `Direct`
  is intentionally limited to four 1-, 2-, and 3-row contact-normal rows so the
  benchmark measures its enumeration path rather than Dantzig fallback. The
  rows report `contact_normal_standard_sweep=1`, `normal_row_count=1/2/3/4/5`,
  `source_problem_size=3/6/9/12/15`, `problem_size=1/2/3/4/5`,
  `contact_normal_direct_no_fallback=1` for all Direct rows, coupled-fixture
  counters, and backend build-state counters. The CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU solver rows in a CUDA-enabled build, not
  CUDA LCP kernel execution. This is contact-normal standard-LCP evidence, not
  native boxed or friction-index support for standard-only solvers.
- Verified larger active-set transition benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpLargerActiveSetTransition/' | wc -l`
  reported 49 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpLargerActiveSetTransition' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 32-row, boxed 32-row, and
  coupled friction-index 8-contact packets over the same scoped scalable solver
  set proven by generated active-set coverage. The rows report
  `active_set_transition=1`,
  `larger_active_set_transition=1`, backend build-state counters, and
  `contact_count=8` plus `coupled=1` for the coupled friction-index packet.
  The CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
- Verified stress active-set transition benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpStressActiveSetTransition/' | wc -l`
  reported 49 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpStressActiveSetTransition' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 64-row, boxed 64-row, and
  coupled friction-index 12-contact packets over the same scoped scalable
  solver set as the generated stress active-set correctness slice. The rows
  report `active_set_transition=1`, `stress_active_set_transition=1`, backend
  build-state counters, and `contact_count=12` plus `coupled=1` for the coupled
  friction-index packet. The CUDA-enabled rows are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified production active-set transition benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpProductionActiveSetTransition/' | wc -l`
  reported 48 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpProductionActiveSetTransition' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover stronger-coupled 24-contact,
  72-row, 32-contact, 96-row, and 48-contact, 144-row friction-index
  active-set transition packets over all 16 friction-index-capable manifest
  solvers. The rows report
  `active_set_transition=1`, `production_active_set_transition=1`, backend
  build-state counters, `contact_count=24/32/48`,
  `problem_size=72/96/144`, `coupling_scale=2/4/8`, and `coupled=1`. The CUDA-enabled rows are CPU solver
  rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified production active-set transition batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpProductionActiveSetTransitionBatch' | wc -l`
  reported 96 rows, and JSON benchmark checks for
  `BM_LcpProductionActiveSetTransitionBatch` reported 96 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover batch-size-4 serial and DART 7 `ParallelExecutor` runs over
  the same stronger-coupled 24-contact/72-row, 32-contact/96-row, and
  48-contact/144-row packets.
  The rows report `production_active_set_transition_batch=1`,
  `batch_size=4`, `contact_count=24/32/48`,
  `total_contact_count=96/128/192`, `problem_size=72/96/144`,
  `total_problem_size=288/384/576`, `coupling_scale=2/4/8`, and backend build-state counters. Parallel rows also
  report `profile_enabled=1`, `parallel_units=4`, `worker_count=20`, and
  observed `max_parallelism`. The CUDA-enabled rows are CPU solver batch rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified larger mildly ill-conditioned benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditioned/' | wc -l`
  reported 192 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpMildIllConditioned' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. The rows cover standard 32-row, boxed 16-row,
  friction-index 8-contact, and coupled friction-index 6-, 8-, 12-, 16-, and
  24-contact packets, plus stronger-coupled 16-/24-contact packets with 4x and
  8x cross-contact coupling and a stronger-coupled 32-contact packet with 8x
  cross-contact coupling, plus ADMM/SAP-only 16x-coupled
  16-/24-/32-/48-contact packets over the scoped solver set. The friction-index rows
  report `contact_count`, the coupled rows report `coupled=1`, the
  stronger-coupled rows report `coupling_scale=4` or `coupling_scale=8`, and
  all rows report `mildly_ill_conditioned=1` plus backend build-state counters.
- Verified larger mildly ill-conditioned batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditionedBatch' | wc -l`
  reported 156 rows, and JSON benchmark checks for
  `BM_LcpMildIllConditionedBatch(Serial|Parallel)` reported 156 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover batch-size-4 serial and DART 7 `ParallelExecutor` runs over
  the 4x-coupled 16-/24-contact, 8x-coupled 16-/24-/32-contact, and
  ADMM/SAP-only 16x-coupled 16-/24-/32-/48-contact mildly ill-conditioned
  packets. The rows report `mildly_ill_conditioned_batch=1`, `batch_size=4`,
  `contact_count=16/24/32/48`, `total_contact_count=64/96/128/192`,
  `problem_size=48/72/96/144`, `total_problem_size=192/288/384/576`,
  `coupling_scale=4/8/16`, and backend build-state counters. Parallel rows also
  report `profile_enabled=1`, `parallel_units=4`, `worker_count=20`, and
  observed `max_parallelism`. The CUDA-enabled rows are CPU solver batch rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified near-singular batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNearSingularBatch' | wc -l`
  reported 32 rows, and JSON benchmark checks for
  `BM_LcpNearSingularBatch(Serial|Parallel)` reported 32 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover batch-size-4 serial and DART 7 `ParallelExecutor` runs over
  coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, and 48-contact
  near-singular packets. The rows report `near_singular_batch=1`,
  `batch_size=4`, `contact_count=3/6/9/12/16/24/32/48`,
  `total_contact_count=12/24/36/48/64/96/128/192`,
  `problem_size=9/18/27/36/48/72/96/144`,
  `total_problem_size=36/72/108/144/192/288/384/576`, and backend
  build-state counters. Parallel rows also report `profile_enabled=1`,
  `parallel_units=4`, `worker_count=20`, and observed `max_parallelism`. The
  CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
- Verified singular-degenerate benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSingularDegenerate/' | wc -l`
  reported 27 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpSingularDegenerate' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover exact rank-deficient standard
  16-row, boxed 16-row, and coupled friction-index 6-contact packets over the
  same scoped solver set as the generated singular-degenerate correctness
  slice. The rows report
  `singular_degenerate=1`, `rank_deficient=1`, backend build-state counters,
  and contact/coupling counters where applicable.
- Verified larger singular-degenerate benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpLargerSingularDegenerate/' | wc -l`
  reported 27 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpLargerSingularDegenerate' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover exact rank-deficient standard
  32-row, boxed 32-row, and coupled friction-index 8-contact packets over the
  same scoped robust solver set as the generated larger singular-degenerate
  correctness slice. The rows report `singular_degenerate=1`,
  `rank_deficient=1`, backend build-state counters, and `contact_count=8` plus
  `coupled=1` for the coupled friction-index packet. The CUDA-enabled rows are
  CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified stress singular-degenerate benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpStressSingularDegenerate/' | wc -l`
  reported 27 rows, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpStressSingularDegenerate' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover exact rank-deficient standard
  64-row, boxed 64-row, and coupled friction-index 12-contact packets over the
  same scoped robust solver set as the generated stress singular-degenerate
  correctness slice. The rows report `singular_degenerate=1`,
  `rank_deficient=1`, backend build-state counters, and `contact_count=12` plus
  `coupled=1` for the coupled friction-index packet. The CUDA-enabled rows are
  CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified extreme singular-degenerate benchmark slice:
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpExtremeSingularDegenerate/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran 36 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover exact rank-deficient standard
  128-row, boxed 128-row, and coupled friction-index 16-/24-/32-/48-contact packets
  over the same scoped robust solver set as the generated extreme
  singular-degenerate correctness slice. The rows report
  `singular_degenerate=1`, `rank_deficient=1`, backend build-state counters,
  contact counts `16/24/32/48`, problem sizes `48/72/96/144`, and `coupled=1`
  for the coupled packets. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
- Verified solver-internal Jacobi threading slice:
  `JacobiSolver::Parameters::workerThreads` now enables an opt-in CPU threaded
  update path. `LcpGeneratedCoverage.ThreadedJacobiStandardKnownSolution`
  passed on a 128-row generated standard LCP. `BM_LCP_COMPARE` now lists 13
  Jacobi threading rows: 4 dense `BM_LcpJacobiSolverThreading_Standard` rows
  for 128-row and 512-row standard problems with 1 and 8 worker threads, plus 6
  banded `BM_LcpJacobiSolverThreadingBanded_Standard` rows for 512-row and
  1024-row standard problems with 1, 4, and 8 worker threads, plus 3
  larger banded rows for 2048-row standard problems with 1, 8, and 16 worker
  threads. The focused
  default, SIMD-enabled, and CUDA-enabled benchmark runs all reported
  `contract_ok=1`, `solver_internal_threads`, `worker_count`,
  `jacobi_threading_banded_spd`, `band_half_width`, `matrix_nonzero_entries`,
  `matrix_density`, and backend build-state counters. The banded rows use
  sparse-structured matrices in dense storage and report densities of about
  0.00974 for 512 rows, 0.00488 for 1024 rows, and 0.00244 for 2048 rows. The
  focused local timings are benchmark-comparison evidence only; they do not
  establish a general worker-thread speedup. CUDA-enabled rows are CPU Jacobi
  rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Boxed semi-smooth Newton friction-index fix:
  `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp` now includes the
  derivative of moving `findex` friction bounds in the natural-residual
  Jacobian. The previously failing coupled mildly ill-conditioned 4-contact
  generated case is now included in the passing all-solver grid, and the
  boxed semi-smooth Newton generated path no longer needs a `MaxIterations`
  allowance for coupled friction-index cases.
- DART 7 world-contact evidence:
  `tests/unit/simulation/contact/test_boxed_lcp_contact.cpp` now
  checks `BoxedLcpContact.WorldContactSnapshotSatisfiesLcpContract`, which
  takes a real sphere-ground friction contact from `World::collide()`, routes it
  through `detail::solveBoxedLcpContacts`, validates the assembled boxed/findex
  LCP via `tests/common/lcpsolver/lcp_test_harness.hpp`, and verifies the
  applied impulse cancels downward approach while reducing tangential slip.
  It also checks
  `BoxedLcpContact.TwoSphereWorldContactSnapshotSatisfiesLcpContract`, which
  assembles two separated sphere-ground contacts into one boxed/findex LCP and
  validates the same contract and impulse invariants.
  `BoxedLcpContact.SphereStackWorldContactSnapshotSatisfiesLcpContract` now
  validates a 3-sphere vertical stack with sphere-ground and sphere-sphere
  contacts coupled through shared dynamic bodies; it also checks that the
  normal-contact block has nonzero off-diagonal coupling.
  `BoxedLcpContact.LargerSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extends that snapshot evidence to a 4-sphere stack, validating a 4-contact,
  12-row boxed/findex LCP assembled from shared dynamic bodies.
  `BoxedLcpContact.StressSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extends the same snapshot path to a 5-sphere stack, validating a 5-contact,
  15-row boxed/findex LCP assembled from shared dynamic bodies. The
  `BoxedLcpContact.TwoSphereWorldStepMaintainsContactInvariants` test advances
  two independent sphere-ground contacts through `World::step(200)` using the
  public boxed-LCP solver and verifies finite state, non-penetration, near-rest
  normal velocity, tangential-speed reduction, and static-ground invariants.
  `BoxedLcpContact.SphereStackWorldStepMaintainsContactInvariants` advances the
  3-sphere vertical stack through `World::step(200)` and verifies finite state,
  non-penetration, preserved sphere spacing, near-rest vertical velocity,
  bounded lateral drift, and static-ground invariants.
  `BoxedLcpContact.LongRunningSphereStackWorldStepMaintainsContactInvariants`
  extends that same coupled topology to `World::step(500)` with the same
  invariants.
  `BoxedLcpContact.LargerSphereStackWorldStepMaintainsContactInvariants`
  advances a 4-sphere coupled stack through `World::step(200)` using the
  timestep-driven boxed-LCP Baumgarte velocity bias and verifies the same
  finite-state, non-penetration, spacing, near-rest, lateral-drift, and
  static-ground invariants.
  `BoxedLcpContact.StressSphereStackWorldStepMaintainsContactInvariants`
  advances a 5-sphere coupled stack through `World::step(500)` and verifies the
  same invariants; the longer horizon is required before the taller stack
  satisfies the near-rest vertical-velocity gate. The
  focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*WorldContactSnapshot*'`
  run passed the single-contact and two-contact sphere-ground snapshot tests,
  the focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*SphereStack*'`
  run passed the 3-sphere snapshot, 4-sphere snapshot, 5-sphere snapshot,
  3-sphere 200-step, 3-sphere 500-step, 4-sphere 200-step, and 5-sphere
  500-step stack tests,
  the focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.SphereStackWorldStepMaintainsContactInvariants:BoxedLcpContact.LongRunningSphereStackWorldStepMaintainsContactInvariants:BoxedLcpContact.LargerSphereStackWorldStepMaintainsContactInvariants:BoxedLcpContact.StressSphereStackWorldStepMaintainsContactInvariants'`
  run passed all four coupled stack step tests, focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.TwoSphereWorldStepMaintainsContactInvariants'`
  and
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.FourSphereWorldStepMaintainsContactInvariants:BoxedLcpContact.SixteenSphereWorldStepMaintainsContactInvariants'`
  runs passed.
  `BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants`
  advances a fixed-base prismatic link in light ground contact through
  `World::step(400)` with `ContactSolverMethod::BoxedLcp`, confirms the
  active contact touches a `comps::Link`, and checks finite state, near-rest
  link height, bounded joint velocity, and parity with the sequential
  articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants' --gtest_brief=1`
  run passed.
  `BoxedLcpContact.FourArticulatedPrismaticLinksGroundStepMaintainsInvariants`
  advances four independent fixed-base prismatic links in simultaneous ground
  contact through `World::step(200)` with `ContactSolverMethod::BoxedLcp`,
  confirms all four contacts touch links, and checks finite state, bounded
  height error, bounded joint velocity, and parity with the sequential
  articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkGroundStepMaintainsInvariants:BoxedLcpContact.FourArticulatedPrismaticLinksGroundStepMaintainsInvariants' --gtest_brief=1`
  run passed both articulated link-ground tests.
  `BoxedLcpContact.CartesianPrismaticChainGroundStepMaintainsInvariants`
  advances two connected three-axis prismatic Cartesian chains in simultaneous
  tip-ground contact through `World::step(200)` with
  `ContactSolverMethod::BoxedLcp`, confirms both contacts touch links, and
  checks finite state, 6 total generalized coordinates, bounded tip height
  error, bounded joint velocities, bounded planar joint speed, and parity with
  the sequential articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.CartesianPrismaticChainGroundStepMaintainsInvariants' --gtest_brief=1`
  run passed.
  `BoxedLcpContact.ArticulatedPrismaticLinkPushesDynamicRigidBody` advances a
  fixed-base prismatic striker link in contact with a dynamic rigid sphere
  through one boxed-LCP `World::step()`, confirms the contact touches both a
  `comps::Link` and a rigid body, and checks finite velocities, target motion,
  striker slowdown, X-momentum conservation, and parity with the sequential
  articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesDynamicRigidBody' --gtest_brief=1`
  run passed. The full `test_boxed_lcp_contact --gtest_brief=1` binary now
  passes 46 tests.
  `BoxedLcpContact.ArticulatedPrismaticLinkPushesArticulatedPrismaticLink`
  advances a fixed-base prismatic striker link in contact with a prismatic
  target link owned by a separate multibody through one boxed-LCP
  `World::step()`, confirms the contact touches two `comps::Link` entities,
  and checks finite velocities, target motion, striker slowdown,
  nonnegative post-step separation velocity, X-momentum conservation, and parity
  with the sequential cross-multibody articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesArticulatedPrismaticLink' --gtest_brief=1`
  run passed. `DenseBoxWorldContactSnapshotSatisfiesLcpContract`
  assembles a 4-contact, 12-row box-face ground patch from `World::collide()`,
  checks the boxed/findex shape, and verifies the same LCP with APGD. The
  sliding and static-friction end-to-end box tests now also assert at least
  four contacts before stepping, so they are dense face-contact public-step
  evidence. `FourBoxWorldStepMaintainsDenseContactInvariants` and
  `EightBoxWorldStepMaintainsDenseContactInvariants` extend that to separated
  4-box and 8-box, 16-contact and 32-contact dense face-contact scenes advanced
  for 200 public boxed-LCP `World::step()` iterations.
  `SixteenBoxWorldStepMaintainsDenseContactInvariants` advances the same scene
  family to 16 boxes and 64 dense face contacts over 500 public boxed-LCP
  `World::step()` iterations.
  `TwentyFourBoxWorldStepMaintainsDenseContactInvariants` extends this to 24
  boxes and 96 dense face contacts over 2000 small public boxed-LCP
  `World::step()` iterations. The full
  `test_boxed_lcp_contact --gtest_brief=1` run passes 46 tests while still
  emitting the dense-patch Dantzig warning; Dantzig's direct dense box solve is
  not claimed.
  warning; Dantzig's direct dense box solve is not claimed.
- DART 7 world-contact benchmark evidence:
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers 48
  manifest-generated
  `BM_LcpWorldContact/FrictionIndex/<solver>/{1,2,4}` rows, covering all 16
  friction-index-capable solvers on the same boxed/findex LCP snapshots
  assembled from separated sphere-ground contacts. It also registers
  `BM_LcpWorldContactAssembly_BoxedLcp/{1,2,4}`, which rebuilds
  `dart::simulation::World`, calls `World::collide()`, assembles the contact
  LCP through `detail::solveBoxedLcpContacts`, and validates the solved
  snapshot. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContact|BM_LcpWorldContactAssembly' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  run passed with `contract_ok=1` for all 51 rows. This is contact-derived
  benchmark evidence for 1/2/4 separated sphere-ground contacts, not evidence
  for articulated, robot-like, or denser degenerate contact scenes.
- DART 7 dense box-contact benchmark evidence:
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers 48 scoped
  `BM_LcpWorldBoxContact/FrictionIndex/<solver>/{1,2,4,8,16,24,32,48}` rows for
  `Pgs`, `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and `Admm`. Each solver
  uses the same 1/2/4/8/16/24/32/48-box dense face-contact snapshots, covering
  4/8/16/32/64/96/128/192 contacts, 12/24/48/96/192/288/384/576 rows, and
  1/2/4/8/16/24/32/48 dynamic bodies. Focused default, SIMD-enabled, and
  CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContact/FrictionIndex/.+/48$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs each reported `contract_ok=1`, `dense_box_contact=1`, `box_count=48`,
  `contact_count=192`, and `problem_size=576` for the six new 48-box rows, with
  `build_simd_enabled=1` in the SIMD build tree and `build_cuda_enabled=1` in
  the CUDA-enabled build tree.
  The CUDA-enabled rows are CPU solver benchmark rows in that build tree, not
  CUDA LCP kernel execution. This is dense contact-snapshot evidence, not
  broad robot-like or CUDA dense-contact execution evidence.
- DART 7 dense box-contact end-to-end benchmark evidence:
  `BM_LcpWorldBoxStep_BoxedLcp/{1,2,4,8}/200` and
  `BM_LcpWorldBoxStep_BoxedLcp/16/500`,
  `BM_LcpWorldBoxStep_BoxedLcp/24/2000`, and
  `BM_LcpWorldBoxStep_BoxedLcp/{32,48}/4000` rebuild separated box-on-ground scenes,
  confirm each box contributes a 4-contact face patch before stepping, enter
  simulation mode, advance the public boxed-LCP `World::step()` path, and check
  finite-state, contact-height, vertical-rest, and tangential-slowing
  invariants. Focused default, SIMD-enabled, and CUDA-enabled build-tree
  runs over the 24-/32-box rows reported `invariant_ok=1` with
  `dense_box_contact=1`, `box_count=24/32`, `contact_count=96/128`, and
  `step_count=2000/4000`. The default 32-box row reported
  `max_height_error=1.46e-4` and `max_vertical_speed=4.38e-2`; the SIMD and
  CUDA-enabled 32-box rows also reported `invariant_ok=1`. The focused default
  48-box row reported `invariant_ok=1`, `dense_box_contact=1`,
  `contact_count=192`, `step_count=4000`, `max_height_error=9.80e-5`, and
  `max_vertical_speed=1.08e-2`. The
  CUDA-enabled rows are CPU public-step rows in that build tree, not CUDA LCP
  kernel execution. The runs still emit the dense-patch Dantzig warning, so
  this is public-step invariant evidence for dense face-contact scenes, not a
  direct Dantzig dense box solve claim.
- DART 7 dense box-contact CUDA batch evidence:
  `CudaLcpPgsBatch.DenseBoxWorldContactBatchSatisfiesLcpContract` builds
  homogeneous batches for 1-/16-/24-/32-/48-box dense face-contact snapshots and
  verifies fixed-iteration CUDA PGS against the LCP contract.
  `CudaLcpPgsBatch.DenseBoxWorldContactGroupedBatchSatisfiesLcpContract` extends
  that to grouped variable-size 1/2/4/8/16/24/32-box packets. The
  focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaPgsWorldBoxContact(Batch_FrictionIndex/48/4|GroupedBatch_FrictionIndex/2)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  CUDA run reported `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_dense_box_contact_batch=1`, and
  `dense_box_contact=1` for the 48-box homogeneous row and the grouped row. The
  48-box homogeneous row reported `batch_size=4`, `box_count=48`,
  `contact_count=192`, `problem_size=576`, `total_contact_count=768`,
  `total_body_count=192`, and `total_problem_size=2304`; the grouped row
  remains scoped to 1/2/4/8/16/24/32-box packets and reported `batch_size=14`,
  `cuda_group_count=7`, `box_count_shape_count=7`, `min_problem_size=12`,
  `max_problem_size=384`, `total_contact_count=696`, `total_body_count=174`, and
  `total_problem_size=2088`. A grouped 48-box CUDA PGS extension was probed and
  is not claimed: at 1024 fixed iterations the grouped validation failed two
  48-box variants with fixed-variable residual/complementarity around
  0.606/0.625. A fixed-iteration CUDA Jacobi dense-box trial failed the LCP
  contract, so Jacobi dense-box CUDA execution remains unclaimed.
  The earlier failed Jacobi probe covered the previous homogeneous 4-problem and
  grouped 1/2/4-box fixtures: 4096 iterations with relaxation 1.0 failed with
  residual/complementarity/bound violations of about 3.3e-2 to 5.0e-2
  (`w must be non-negative at lo`), and 8192 iterations with relaxation 0.25
  reached near-zero residual/bound violation but failed fixed-variable
  complementarity at about 0.34 to 0.435 (`fixed variable residual`).
- DART 7 coupled world-contact stack benchmark evidence:
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now also registers 32
  manifest-generated
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{2,3}` rows, covering all 16
  friction-index-capable solvers on boxed/findex LCP snapshots assembled from
  2- and 3-sphere vertical stacks. It also registers 30
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{4,5}` rows for the same
  solver set except `NNCG`; a focused NNCG 4-sphere trial reached the benchmark
  iteration cap with `contract_ok=0`, so the NNCG 4-/5-sphere rows are not
  claimed. These snapshots include a ground contact and sphere-sphere contacts,
  so the Delassus system couples multiple contacts through shared dynamic
  bodies. The target also registers
  `BM_LcpWorldStackContactAssembly_BoxedLcp/{2,3,4,5}`, which
  rebuilds the stack world, calls `World::collide()`, assembles/solves through
  `detail::solveBoxedLcpContacts`, and validates the solved snapshot. The
  focused 4-sphere run
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/4|BM_LcpWorldStackContactAssembly_BoxedLcp/4' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed with `contract_ok=1` for all 16 registered rows. This is coupled stack
  benchmark evidence for small vertical sphere stacks, not evidence for
  articulated, robot-like, or dense-degenerate contact scenes.
  The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/5$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in default, SIMD-enabled, and CUDA-enabled build trees each reported
  `contract_ok=1` for all 15 registered 5-sphere solver rows with
  `sphere_count=5`, `contact_count=5`, and `problem_size=15`; the
  CUDA-enabled rows are CPU solver benchmark rows in that build tree, not CUDA
  LCP kernel execution.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LcpWorldStackContactAssembly_BoxedLcp/5` runs also passed with
  `contract_ok=1`, `sphere_count=5`, `contact_count=5`, and `problem_size=15`;
  the CUDA-enabled row is a CPU boxed-LCP contact assembly row in a CUDA-enabled
  build, not CUDA LCP kernel execution.
- DART 7 real-world contact batch benchmark evidence:
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers 32
  manifest-generated `BM_LcpWorldContactBatch(Serial|Parallel)/FrictionIndex`
  rows: all 16 friction-index-capable solvers over the same mixed 5-problem
  batch of 1/2/4 separated sphere-ground snapshots and 2/3-sphere vertical
  stack snapshots. A focused benchmark-list check returned 32 registered
  world-contact batch rows. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContactBatch(Serial|Parallel)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  run passed with `contract_ok=1` for all 32 rows. Each row reported
  `batch_size=5`, `total_problem_size=36`, `total_contact_count=12`, and
  `total_body_count=12`; the parallel rows also reported
  `profile_enabled=1`, `worker_count=20`, `parallel_units=5`, and observed
  `max_parallelism` up to 5. This proves serial and DART 7
  `ParallelExecutor` batch processing over contact-derived LCP snapshots; it
  does not prove CUDA batch execution for these variable-size world-contact
  packets.
  The benchmark target also registers 30
  `BM_LcpWorldContactStressBatch(Serial|Parallel)/FrictionIndex/<solver>` rows
  for the same solver set except `NNCG`, using one 7-problem batch made from
  1/2/4 separated sphere-ground snapshots and 2/3/4/5-sphere vertical-stack
  snapshots. Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldContactStressBatch(Serial|Parallel)/FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs each reported `contract_ok=1` for all 30 stress rows with
  `batch_size=7`, `total_problem_size=63`, `total_contact_count=21`,
  `total_body_count=21`, `separated_contact_shape_count=3`,
  `stack_contact_shape_count=4`, and `stress_stack_contact_batch=1`; the
  stress parallel rows reported `profile_enabled=1`, `parallel_units=7`, and
  `worker_count=20`. The CUDA-enabled rows are CPU solver benchmark rows in
  that build tree, not CUDA LCP kernel execution.
- DART 7 coupled stack end-to-end benchmark evidence:
  `BM_LcpWorldStackStep_BoxedLcp/3/200` and
  `BM_LcpWorldStackStep_BoxedLcp/3/500` rebuild the 3-sphere stack world, while
  `BM_LcpWorldStackStep_BoxedLcp/4/200` rebuilds the 4-sphere stack world with
  a smaller 0.001 s step and `BM_LcpWorldStackStep_BoxedLcp/5/500` rebuilds the
  5-sphere stack world with the same 0.001 s step and a longer horizon. All four
  rows enter simulation mode, advance the
  public boxed-LCP `World::step()` path, and check the same finite-state,
  non-penetration, spacing, vertical-rest, lateral-drift, and static-ground
  invariants as the unit tests.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs each reported four rows with `invariant_ok=1`; the SIMD run reported
  `build_simd_enabled=1`, and the CUDA-enabled run reported
  `build_cuda_enabled=1`. The default 3-sphere rows reported
  `time_step=0.005`, `min_spacing=0.9999`, and
  `max_vertical_speed=4.31e-14`; the default 4-sphere row reported
  `time_step=0.001`, `min_spacing=0.9999`, and
  `max_vertical_speed=1.72e-8`; the default 5-sphere row reported
  `time_step=0.001`, `min_spacing=0.9999`, and
  `max_vertical_speed=1.26e-5`.
- DART 7 separated multi-contact end-to-end benchmark evidence:
  `BM_LcpWorldSeparatedStep_BoxedLcp/4/200`,
  `BM_LcpWorldSeparatedStep_BoxedLcp/8/200`, and
  `BM_LcpWorldSeparatedStep_BoxedLcp/16/200` rebuild separated sphere-ground
  worlds, enter simulation mode, advance the public boxed-LCP `World::step()`
  path for 200 steps, and check finite-state, contact-height, vertical-rest,
  and tangential-slowing invariants. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldSeparatedStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  run reported `invariant_ok=1` for all three rows, including
  `contact_count=16`; the 16-contact row reported `max_height_error=0`,
  `max_vertical_speed=0`, and `min_tangential_speed_drop=0.23816`.
- DART 7 dense box-face end-to-end benchmark evidence:
  `BM_LcpWorldBoxStep_BoxedLcp/{1,2,4,8}/200` and
  `BM_LcpWorldBoxStep_BoxedLcp/16/500`,
  `BM_LcpWorldBoxStep_BoxedLcp/24/2000`, and
  `BM_LcpWorldBoxStep_BoxedLcp/{32,48}/4000` rebuild separated
  dense box-face worlds,
  enter simulation mode, advance the public boxed-LCP `World::step()` path, and
  check finite-state, contact-height, vertical-rest, and tangential-slowing
  invariants. Focused default, SIMD-enabled, and CUDA-enabled build-tree
  runs over the 24-/32-box rows reported `invariant_ok=1`, with
  `dense_box_contact=1`, `box_count=24/32`, and `contact_count=96/128`; the
  focused default 48-box row reported `invariant_ok=1`, `box_count=48`,
  `contact_count=192`, and `step_count=4000`. The
  CUDA-enabled rows are CPU public-step rows in that build tree, not CUDA LCP
  kernel execution.
- DART 7 articulated contact end-to-end benchmark evidence:
  `BM_LcpWorldArticulatedGroundStep_BoxedLcp/{1,4,8,16}/200` rebuilds fixed-base
  prismatic-link worlds, enters simulation mode in the world factory, advances
  the public boxed-LCP `World::step()` path for 200 steps, and checks finite
  link height plus near-zero joint velocity after ground contact. The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldArticulated(Ground|RigidImpact)Step_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  default-build run reported `invariant_ok=1` for all eight articulated rows,
  with ground-step `articulated_link_count=1`, `4`, `8`, and `16`; matching
  SIMD-enabled and CUDA-enabled build-tree runs also reported
  `invariant_ok=1` for all eight rows, with the backend build counters set for
  their respective build trees. This is fixed-base prismatic-link
  ground-contact evidence, not broad articulated robot contact coverage.
  `BM_LcpWorldArticulatedRigidImpactStep_BoxedLcp/{1,4,8,16}/1` rebuilds
  fixed-base prismatic striker worlds with dynamic rigid targets, enters
  simulation mode in the world factory, advances one public boxed-LCP
  `World::step()`, and checks target motion, striker slowdown, and X-momentum
  conservation. The same focused default/SIMD/CUDA runs reported
  `invariant_ok=1` for all four rigid-impact rows, with
  `articulated_link_count=1`, `4`, `8`, and `16`,
  `dynamic_rigid_body_count=1`, `4`, `8`, and `16`,
  `max_momentum_error=0`, `max_striker_velocity=0.606667`, and
  `min_target_velocity=0.786667`.
  `BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp/{1,4,8,16}/1` rebuilds
  cross-multibody fixed-base prismatic striker/target link worlds, enters
  simulation mode in the world factory, advances one public boxed-LCP
  `World::step()`, and checks target-link motion, striker-link slowdown,
  nonnegative post-step separation velocity, and X-momentum conservation. The
  focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  default/SIMD/CUDA build-tree runs reported `invariant_ok=1` for all four
  link-impact rows, with `articulated_pair_count=1`, `4`, `8`, and `16`,
  `articulated_dof_count=2`, `8`, `16`, and `32`,
  `cross_multibody_link_contact=1`, `max_momentum_error=0`,
  `max_striker_velocity=0.606667`, `min_target_velocity=0.786667`, and
  `min_relative_velocity=0.18`.
  `BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp/{1,4,8,16}/200`
  rebuilds connected fixed-base three-axis prismatic Cartesian-chain worlds,
  enters simulation mode in the world factory, advances the public boxed-LCP
  `World::step()` path for 200 steps, and checks finite tip height, bounded
  joint velocities, and bounded planar joint speed after ground contact. The
  focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  default/SIMD/CUDA build-tree runs reported `invariant_ok=1` for all four
  rows, with `cartesian_chain_count=1`, `4`, `8`, and `16`,
  `articulated_dof_count=3`, `12`, `24`, and `48`, and backend build counters
  set for the respective build trees. This is connected fixed-base
  multi-DOF articulated contact evidence, not broad articulated robot contact
  coverage.
- DART 7 articulated unified-contact solver benchmark evidence:
  `BM_LcpArticulatedUnifiedContact/FrictionIndex/{Ground,RigidImpact,CrossLinkImpact}/<solver>/{1,4,8}`
  manually assembles fixed-base three-axis prismatic `LinkContact` snapshots
  through `assembleMultibodyLinkContactProblem` and
  `assembleUnifiedConstraintProblem`, then compares all 16
  friction-index-capable solvers on the same 3-row, 12-row, and 24-row LCPs.
  The cross-link rows complete a second articulated endpoint for a separate
  multibody, so they exercise the unified contact matrix's cross-multibody
  block. Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpArticulatedUnifiedContact/FrictionIndex/.+/8$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `contract_ok=1` for all 48 new 8-contact rows, with
  `articulated_unified_contact=1`, `contact_count=8`, `problem_size=24`,
  `multibody_count=8` for ground/rigid-impact rows, and `multibody_count=16`
  plus `articulated_cross_link_contact=1` for cross-link rows.
  This is manual articulated link-contact assembly evidence, not
  `World::collide()` or end-to-end stepping evidence. The CUDA-enabled
  all-solver rows are CPU solver execution
  in those build trees, not CUDA kernel execution.

## Completed This Session

- Added comparison-harness coverage for `ApgdSolver`, `TgsSolver`,
  `AdmmSolver`, `SapSolver`, and `BoxedSemiSmoothNewtonSolver`.
- Covered those solvers on standard and boxed fixtures, plus friction-index
  fixtures for the boxed-capable solvers.
- Added `tests/common/lcpsolver/lcp_solver_manifest.hpp` and migrated the
  all-solver smoke test to instantiate solvers through it.
- Added comparison coverage drift checks against the manifest and expanded
  `ShockPropagationSolver` comparison coverage from standard-only to standard
  plus boxed fixtures.
- Replaced per-solver benchmark registration for standard, boxed, and
  friction-index families with manifest-generated benchmarks. These benchmark
  families now use the same deterministic generated problem for every solver at
  the same size; `DirectSolver` remains limited to dimensions 2 and 3 to avoid
  benchmarking its Dantzig fallback as direct-solver work.
- Added `BM_LcpBatchSerial/<family>/<solver>` registrations for one
  deterministic independent-problem batch per supported solver/family. This is
  serial batch evidence only; it does not claim SIMD, threaded, or CUDA batch
  execution.
- Added `BM_LcpBatchParallel/<family>/<solver>` registrations for the same
  deterministic independent-problem batches using the DART 7 experimental
  `ParallelExecutor`. This is task-parallel batch evidence for independent LCP
  problems, not evidence for SIMD, CUDA, or intra-solver parallelism.
- Added backend build-state counters to `BM_LCP_COMPARE` so benchmark rows
  distinguish default scalar, SIMD-enabled, CUDA-enabled, and
  simulation-experimental build configurations.
- Added opt-in solver-internal CPU worker threads to `JacobiSolver` and
  benchmark rows comparing serial and threaded Jacobi on the same generated
  standard problems. Current focused timings show correctness with overhead for
  the tested dense 128/512-row and larger sparse-structured 2048-row banded
  cases.
- Added `docs/dev_tasks/lcp_solvers/01-implementation-audit.md`. The audit
  records the current DART 7 evidence and now maps the exported APGD, TGS,
  ADMM, SAP, and boxed semi-smooth Newton solvers back to
  `docs/background/lcp/`.
- Added `tests/unit/math/lcp/test_lcp_generated_coverage.cpp` and registered it
  in `tests/unit/CMakeLists.txt`. The new target is manifest-driven, so solver
  additions or support changes must be reflected in the manifest before they
  participate in the generated correctness grid.
- Extended generated coverage with batch-shaped state-reuse checks, invalid
  dimension/bounds/findex/NaN rejection checks, pivoting-solver near-singular
  standard known-solution cases, and near-singular boxed known-solution cases.
- Added
  `LcpGeneratedCoverage.LargerWellConditionedKnownSolutionsForScalableSolvers`
  for standard 32-row and 64-row, boxed 32-row, friction-index 16-contact, and
  coupled friction-index 8-contact known-solution cases over a scoped
  scalable/iterative-capable solver set.
- Added
  `LcpGeneratedCoverage.ProductionScaleWellConditionedKnownSolutionsForScalableSolvers`
  for standard 128-row, boxed 64-row, friction-index 24-contact, and coupled
  friction-index 12-contact known-solution cases over scoped scalable solvers.
- Added `LcpGeneratedCoverage.NearSingularKnownSolutionsForRobustSolverSlice`
  for standard 8-row, boxed 8-row, and coupled friction-index 3-, 6-, 9-, 12-,
  16-, 24-, 32-, and 48-contact near-singular known-solution cases over a
  scoped robust solver set. Trial
  evidence kept this intentionally narrow: Lemke produced a valid complementary
  solution but not the selected generated solution for the 8-row singular
  standard case, and boxed semi-smooth Newton failed line search on the
  near-singular coupled friction-index cases.
- Added
  `LcpGeneratedCoverage.LargerMildlyIllConditionedKnownSolutionsForScopedSolvers`
  for standard 32-row and 64-row, boxed 16-row and 32-row, friction-index
  8-contact, and coupled friction-index 6-, 8-, 12-, 16-, and 24-contact
  known-solution cases, plus stronger-coupled 16-/24-contact cases with 4x and
  8x cross-contact coupling and a stronger-coupled 32-contact case with 8x
  cross-contact coupling, over a scoped solver set. `MPRGP` is excluded from
  this stricter known-solution slice after a focused 32-row standard trial
  satisfied the LCP contract but missed the selected expected-solution
  tolerance by a small margin.
- Added
  `LcpGeneratedCoverage.SingularDegenerateKnownSolutionsForRobustSolverSlice`
  for exact rank-deficient standard 16-row, boxed 16-row, and coupled
  friction-index 6-contact known-solution cases. The standard packet covers 21
  standard-capable solvers; boxed and friction-index packets are scoped to
  `Admm`, `Sap`, and `BoxedSemiSmoothNewton` after broader trials exposed
  contract failures for the other tried boxed/findex paths.
- Added
  `LcpGeneratedCoverage.LargerSingularDegenerateKnownSolutionsForRobustSolverSlice`
  for larger exact rank-deficient standard 32-row, boxed 32-row, and coupled
  friction-index 8-contact known-solution cases over the same observed-robust
  singular-degenerate solver scope.
- Added
  `LcpGeneratedCoverage.StressSingularDegenerateKnownSolutionsForRobustSolverSlice`
  for stress exact rank-deficient standard 64-row, boxed 64-row, and coupled
  friction-index 12-contact known-solution cases over the same observed-robust
  singular-degenerate solver scope.
- Added
  `LcpGeneratedCoverage.ExtremeSingularDegenerateKnownSolutionsForRobustSolverSlice`
  for extreme exact rank-deficient standard 128-row, boxed 128-row, and coupled
  friction-index 16-/24-/32-/48-contact known-solution cases over the same
  observed-robust singular-degenerate solver scope.
- Added `LcpGeneratedCoverage.ActiveSetTransitionKnownSolutions` for standard
  16-row, boxed 16-row, and coupled friction-index 6-contact known-solution
  cases near lower, upper, interior, and friction-cone boundary transitions.
- Added
  `LcpGeneratedCoverage.StressActiveSetTransitionKnownSolutionsForScalableSolvers`
  for stress standard 64-row, boxed 64-row, and coupled friction-index
  12-contact known-solution cases near active-set boundary transitions.
- Added
  `LcpGeneratedCoverage.ExtremeActiveSetTransitionKnownSolutionsForScalableSolvers`
  for extreme standard 128-row, boxed 128-row, and coupled friction-index
  16-contact known-solution cases near active-set boundary transitions.
- Added `BM_LcpActiveSetTransition` rows for the same standard, boxed, and
  coupled friction-index packets so the active-set boundary cases are also
  benchmarked apples-to-apples across manifest-supporting solvers.
- Added 192 `BM_LcpMildIllConditioned` rows for larger mildly ill-conditioned
  standard 32-row, boxed 16-row, friction-index 8-contact, and coupled
  friction-index 6-, 8-, 12-, 16-, and 24-contact packets, plus
  stronger-coupled 16-/24-contact packets with 4x and 8x cross-contact
  coupling and a stronger-coupled 32-contact packet with 8x cross-contact
  coupling, plus ADMM/SAP-only 16x-coupled 16-/24-/32-/48-contact packets over the
  same scoped solver set. These rows are solve-to-tolerance benchmark
  evidence and report `contract_ok=1`, `mildly_ill_conditioned=1`, backend
  build-state counters, and contact/coupling counters where applicable.
- Added 156 `BM_LcpMildIllConditionedBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over the 4x-coupled
  16-/24-contact, 8x-coupled 16-/24-/32-contact, and ADMM/SAP-only 16x-coupled
  16-/24-/32-/48-contact mildly ill-conditioned friction-index packets. These rows
  report `contract_ok=1`, `mildly_ill_conditioned_batch=1`, backend build-state
  counters, and contact/coupling counters.
- Added 21 `BM_LcpNearSingular` rows for near-singular standard 8-row, boxed
  8-row, and coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, and
  48-contact packets over the generated robust near-singular solver scope.
  These rows report `contract_ok=1`, `near_singular=1`, backend build-state
  counters, and contact/coupling counters where applicable. The CUDA-enabled
  rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel
  execution.
- Added 32 `BM_LcpNearSingularBatch(Serial|Parallel)` rows for batch-size-4
  serial and DART 7 `ParallelExecutor` runs over the coupled friction-index 3-,
  6-, 9-, 12-, 16-, 24-, 32-, and 48-contact near-singular packets. These rows
  report `contract_ok=1`, `near_singular_batch=1`, backend build-state
  counters, and contact/coupling counters. The CUDA-enabled rows are CPU solver
  batch rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Added 27 `BM_LcpSingularDegenerate` rows for exact rank-deficient standard
  16-row, boxed 16-row, and coupled friction-index 6-contact packets over the
  generated singular-degenerate solver scope. These rows report `contract_ok=1`,
  `singular_degenerate=1`, `rank_deficient=1`, backend build-state counters,
  and contact/coupling counters where applicable.
- Added 27 `BM_LcpLargerSingularDegenerate` rows for exact rank-deficient
  standard 32-row, boxed 32-row, and coupled friction-index 8-contact packets
  over the generated larger singular-degenerate solver scope. These rows report
  `contract_ok=1`, `singular_degenerate=1`, `rank_deficient=1`, backend
  build-state counters, and contact/coupling counters where applicable. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
- Added 27 `BM_LcpStressSingularDegenerate` rows for exact rank-deficient
  standard 64-row, boxed 64-row, and coupled friction-index 12-contact packets
  over the generated stress singular-degenerate solver scope. These rows report
  `contract_ok=1`, `singular_degenerate=1`, `rank_deficient=1`, backend
  build-state counters, and contact/coupling counters where applicable. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
- Added 36 `BM_LcpExtremeSingularDegenerate` rows for exact rank-deficient
  standard 128-row, boxed 128-row, and coupled friction-index 16-/24-/32-/48-contact
  packets over the generated extreme singular-degenerate solver scope. These
  rows report `contract_ok=1`, `singular_degenerate=1`, `rank_deficient=1`,
  backend build-state counters, and contact/coupling counters where applicable.
  The CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
- Added 49 `BM_LcpLargerActiveSetTransition` rows for the scoped scalable
  active-set transition packets: standard 32-row, boxed 32-row, and coupled
  friction-index 8-contact. These rows report `contract_ok=1`,
  `active_set_transition=1`, `larger_active_set_transition=1`, backend
  build-state counters, and contact/coupling counters where applicable.
- Added 49 `BM_LcpStressActiveSetTransition` rows for the scoped scalable
  active-set transition packets: standard 64-row, boxed 64-row, and coupled
  friction-index 12-contact. These rows report `contract_ok=1`,
  `active_set_transition=1`, `stress_active_set_transition=1`, backend
  build-state counters, and contact/coupling counters where applicable. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
- Added 49 `BM_LcpExtremeActiveSetTransition` rows for the scoped scalable
  active-set transition packets: standard 128-row, boxed 128-row, and coupled
  friction-index 16-contact. These rows report `contract_ok=1`,
  `active_set_transition=1`, `extreme_active_set_transition=1`, backend
  build-state counters, and contact/coupling counters where applicable. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
- Added 48 `BM_LcpProductionActiveSetTransition` rows for stronger-coupled
  24-contact, 72-row, 32-contact, 96-row, and 48-contact, 144-row
  friction-index active-set transition packets over every friction-index-capable manifest solver. These
  rows report `contract_ok=1`, `active_set_transition=1`,
  `production_active_set_transition=1`, `contact_count=24/32/48`,
  `problem_size=72/96/144`, `coupling_scale=2/4/8`, backend build-state counters,
  and `coupled=1`. The CUDA-enabled rows are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Added 96
  `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over the same
  stronger-coupled 24-contact/72-row, 32-contact/96-row, and
  48-contact/144-row packets. These rows
  report `contract_ok=1`, `production_active_set_transition_batch=1`,
  `batch_size=4`, `contact_count=24/32/48`,
  `total_contact_count=96/128/192`, `problem_size=72/96/144`,
  `total_problem_size=288/384/576`, `coupling_scale=2/4/8`, backend build-state counters, and parallel execution
  counters on the `ParallelExecutor` rows. The CUDA-enabled rows are CPU solver
  batch rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Extended generated friction-index coverage with coupled contact matrices:
  well-conditioned 2-contact, 4-contact, and 6-contact cases plus mildly
  ill-conditioned 2-contact and 4-contact cases pass across all
  manifest-supported friction-index solvers.
- Added DART 7 `World` contact evidence for the boxed-LCP path by validating the
  real contact LCP snapshot assembled from a single sphere-ground friction
  contact.
- Extended that DART 7 `World` contact evidence to a two-contact snapshot with
  separated sphere-ground contacts assembled into one boxed/findex LCP.
- Added BGS and Blocked Jacobi per-contact block evidence on that DART 7
  two-contact world snapshot: the solver tests pass with default `findex`
  grouping over the normal-first/tangent-later row layout and reject explicit
  block sizes that split tangent rows from their owning normal row. A focused
  default `BM_LCP_COMPARE` JSON gate covers 22 BGS/Blocked Jacobi
  world-contact rows (`BM_LcpWorldContact`, `BM_LcpWorldStackContact`,
  `BM_LcpWorldContactBatch(Serial|Parallel)`, and
  `BM_LcpWorldContactStressBatch(Serial|Parallel)`) with zero bad contracts.
- Added BGS and Blocked Jacobi block-partition sweep evidence:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpBlockPartitionSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpBlockPartitionSweep` reported 12
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover standard 12-row, boxed 12-row, and
  friction-index 4-contact fixtures with full-block, 3-row block, auto
  `findex`, and explicit contact-block partitions for both solvers. The rows
  report `block_partition_sweep=1`, block counts `1/4`, block sizes `3/12`,
  `contact_count=4`, observed solver `iterations=1/4/5/6/10`, and backend
  build-state counters. Focused
  `BlockedJacobiSolverCoverage.*:BgsSolverCoverage.*` unit coverage passed 15
  tests. The CUDA-enabled rows are CPU BGS/Blocked Jacobi rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Added end-to-end DART 7 `World::step()` evidence for the boxed-LCP path with
  two independent sphere-ground contacts advanced for 200 steps and checked
  against non-penetration, near-rest normal velocity, tangential-speed
  reduction, finite-state, and static-ground invariants.
- Added DART 7 coupled stack snapshot correctness evidence for a 3-sphere
  vertical stack, including a nonzero normal-contact coupling check.
- Extended DART 7 coupled stack snapshot correctness evidence to a 4-sphere
  vertical stack, covering a 4-contact, 12-row boxed/findex LCP assembled from
  shared dynamic bodies.
- Extended DART 7 coupled stack snapshot correctness evidence to a 5-sphere
  vertical stack, covering a 5-contact, 15-row boxed/findex LCP assembled from
  shared dynamic bodies.
- Added DART 7 coupled stack end-to-end `World::step()` evidence for the same
  3-sphere topology over 200 and 500 public boxed-LCP steps.
- Added boxed-LCP Baumgarte velocity-bias stabilization and DART 7 coupled
  stack end-to-end `World::step()` evidence for a 4-sphere stack over 200
  public boxed-LCP steps.
- Added DART 7 articulated boxed-LCP `World::step()` evidence for fixed-base
  prismatic links in one-link and four-link ground-contact scenes, plus
  1-/4-/8-/16-link articulated ground-step benchmark rows.
- Added DART 7 connected multi-DOF articulated boxed-LCP `World::step()`
  evidence for fixed-base three-axis prismatic Cartesian chains in ground
  contact, plus 1-/4-/8-/16-chain articulated Cartesian ground-step benchmark
  rows.
- Added DART 7 two-sided articulated boxed-LCP `World::step()` evidence for a
  fixed-base prismatic link pushing a dynamic rigid body, plus
  1-/4-/8-/16-pair articulated rigid-impact benchmark rows.
- Added DART 7 cross-multibody articulated boxed-LCP `World::step()` evidence
  for a fixed-base prismatic link pushing a separate multibody's prismatic link,
  plus 1-/4-/8-/16-pair articulated link-impact benchmark rows.
- Added DART 7 articulated unified-contact all-solver benchmark evidence for
  manually assembled fixed-base three-axis prismatic link-ground and
  link-vs-dynamic-rigid LCP snapshots, now extended to cross-multibody
  link-vs-link snapshots.
- Added DART 7 world-contact benchmark rows that compare all
  friction-index-capable solvers on the same real boxed/findex contact
  snapshots from 1, 2, and 4 separated sphere-ground contacts, plus a benchmark
  for rebuilding/colliding/assembling/solving those boxed-LCP contact snapshots.
- Added coupled DART 7 world-contact stack benchmark rows that compare all
  friction-index-capable solvers on 2- and 3-sphere vertical stacks with shared
  dynamic bodies, plus 4- and 5-sphere rows for all of those solvers except
  `NNCG`. Stack assembly/solve benchmark rows cover 2-, 3-, 4-, and 5-sphere
  scenes.
- Added mixed DART 7 world-contact batch benchmark rows that compare all
  friction-index-capable solvers over the same 5-problem separated-contact and
  stacked-contact snapshot batch, both serially and through the experimental
  `ParallelExecutor`.
- Added focused local SIMD-enabled CPU evidence for generated LCP correctness
  and selected serial/task-parallel batch benchmark rows.
- Added focused local CUDA-enabled build/runtime evidence and ran generated LCP
  correctness plus selected LCP benchmark rows in that CUDA-enabled build.
- Added CUDA LCP execution evidence through
  `dart::simulation::experimental::compute::cuda::solveBoxedLcpJacobiBatchCuda`
  and `solveBoxedLcpPgsBatchCuda`, fixed-iteration projected Jacobi and PGS
  kernels for homogeneous dense standard, boxed, and friction-index LCP batches.
  Added grouped variable-size synthetic standard/boxed/friction-index CUDA unit
  and benchmark evidence for 16/32/48-row standard and boxed packets plus
  4/8/16-contact friction-index packets. Added homogeneous 4-/8-/16-contact
  and grouped variable-size 1/2/4/8/16-contact DART 7 separated world-contact
  CUDA unit and benchmark evidence, plus homogeneous 5-sphere and grouped
  variable-size 2/3/4/5-sphere coupled stack-contact CUDA unit and benchmark
  evidence and manually assembled 1-/4-/8-contact articulated unified-contact CUDA
  unit and benchmark evidence for link-ground, link-vs-dynamic-rigid, and
  cross-multibody link-vs-link packets for the same kernels. Added mixed grouped CUDA
  unit and benchmark evidence that combines those separated, stack, and
  articulated fixture families, including cross-multibody link-vs-link packets,
  in one size-grouped batch.
  The current CUDA path intentionally excludes other solvers and dense,
  contact batches, and end-to-end articulated world-step CUDA execution.

## Remaining Gaps

- DART 7 integration: decide how these math solvers are selected and exercised
  from the `dart::simulation::World` contact pipeline without exposing internal
  solver/backend registry types.
- Coverage breadth: extend deterministic generated fixtures beyond the current
  production-scale well-conditioned, larger mildly ill-conditioned,
  singular-degenerate through the current 128-row/48-contact slice, and
  active-set transition through the current stronger-coupled 48-contact slice into
  harder solver-specific friction-index coupling edge cases and direct backend
  execution evidence beyond CPU solver rows in SIMD/CUDA-enabled builds.
- Harder friction-index conditioning: the current all-solver generated
  friction-index grid now includes coupled well-conditioned 2-contact and
  4-contact and 6-contact cases plus mildly ill-conditioned 2-contact and
  4-contact cases. Scoped generated evidence now reaches coupled
  well-conditioned 12-contact, mildly ill-conditioned 24-contact,
  stronger-coupled mildly ill-conditioned 16-, 24-, and 32-contact cases,
  ADMM/SAP-only 16x-coupled mildly ill-conditioned 16-, 24-, 32-, and
  48-contact cases, near-singular 48-contact cases, singular-degenerate
  48-contact cases, and production active-set transition 24-, 32-, and
  48-contact cases. Broader
  harder-conditioned coupled friction-index grids beyond the narrow ADMM/SAP
  16x slice still need solver-specific evidence.
- Real-world cases: the current DART 7 world-contact evidence includes
  single-contact and two-contact sphere-ground boxed-LCP snapshots, a
  two-contact, four-contact, and 16-contact 200-step boxed-LCP `World::step()`
  invariant tests, separated 4-/8-/16-contact 200-step boxed-LCP
  `World::step()` benchmark rows, 3-sphere 200-step, 3-sphere 500-step,
  4-sphere 200-step, and 5-sphere 500-step boxed-LCP `World::step()` invariant tests and benchmark
  rows, fixed-base prismatic articulated link-ground boxed-LCP
  `World::step()` invariant tests for one-link and four-link scenes and
  1-/4-/8-/16-link articulated ground-step benchmark rows, fixed-base
  three-axis prismatic Cartesian-chain boxed-LCP `World::step()` invariant and
  1-/4-/8-/16-chain benchmark rows, a fixed-base
  prismatic link-vs-dynamic-rigid boxed-LCP `World::step()` invariant test and
  1-/4-/8-/16-pair articulated rigid-impact benchmark rows, a cross-multibody
  fixed-base prismatic link-vs-link boxed-LCP `World::step()` invariant test
  and 1-/4-/8-/16-pair articulated link-impact benchmark rows,
  1-/2-/4-/8-/16-/24-/32-/48-box dense face-contact boxed-LCP `World::step()`
  benchmark rows,
  manually assembled fixed-base three-axis prismatic articulated unified-contact
  all-solver benchmark rows for link-ground, link-vs-dynamic-rigid, and
  cross-multibody link-vs-link
  snapshots,
  contact-derived benchmark rows for 1/2/4 separated sphere-ground contacts,
  coupled benchmark rows for 2- and 3-sphere vertical stacks across all
  friction-index-capable solvers, 4-/5-sphere stack snapshot and benchmark rows
  for all of those solvers except `NNCG`, mixed serial and `ParallelExecutor`
  batch rows over the 1/2/4 separated-contact and 2/3 stack snapshots, stress
  mixed serial and `ParallelExecutor` batch rows over the same separated
  snapshots plus 2/3/4/5 stack snapshots for all of those solvers except
  `NNCG`, plus existing boxed-contact parity tests. Broader articulated
  contacts beyond fixed-base prismatic link-ground, connected Cartesian-chain
  ground contact, link-vs-rigid impact, and cross-multibody link-vs-link impact,
  longer denser coupled scenes, larger
  coupled multi-contact systems beyond the current 5-sphere snapshot, and
  robot-like contact systems still need LCP-contract, invariant, and benchmark
  evidence.
- Multi-contact boxed-LCP snapshots: a resting box multi-contact friction
  snapshot probe did not satisfy the LCP contract cleanly, consistent with the
  existing static-friction test's degenerate-pivot warning. Keep this as a
  separate solver/contact-assembly evidence slice.
- Benchmarks: extend `tests/benchmark/lcpsolver/` beyond generated math
  fixtures, active-set, larger/stress/extreme/production active-set packets,
  production active-set serial and `ParallelExecutor` batch packets,
  mildly ill-conditioned single-problem and 4x/8x/16x-coupled batch packets,
  near-singular single-problem and batch packets, singular-degenerate packets,
  PGS/PSOR, symmetric PSOR, and Red-Black Gauss-Seidel relaxation sweep rows, APGD restart-policy sweep
  rows, TGS iteration-budget sweep rows, NNCG PGS-preconditioner iteration
  sweep rows, SubspaceMinimization PGS-iteration sweep rows, ShockPropagation
  layer-layout sweep rows, MPRGP SPD/check sweep rows, Interior Point
  path-parameter sweep rows, Staggering contact-pipeline sweep rows, Boxed
  Semi-Smooth Newton line-search sweep rows, Pivoting scale sweep rows, ADMM
  rho/adaptive-rho sweep rows, SAP regularization sweep rows, BGS/Blocked
  Jacobi block-partition sweep rows, ADMM/SAP/Boxed Semi-Smooth Newton
  contact comparison sweep rows, contact-normal standard-LCP sweep rows,
  independent-problem batches, simple world-contact snapshots, small coupled
  stack snapshot batches, and dense box-face snapshot and step rows to broader
  dense and robot-like end-to-end contact systems, broader SIMD benchmark packets, larger threaded
  configurations, broader CUDA LCP solver execution, and
  vectorized/threaded/CUDA batch-processing paths.
- Compute backends: current benchmark rows now self-report scalar/SIMD/CUDA
  build state, a focused local SIMD-enabled CPU slice passes, a focused
  CUDA-enabled build/runtime slice passes, and narrow CUDA projected-Jacobi and
  PGS standard/boxed/friction-index plus grouped variable-size synthetic
  standard/boxed/friction-index, homogeneous 4-/8-/16-contact, homogeneous
  5-sphere coupled stack, and grouped variable-size 1/2/4/8/16-contact separated
  and 2/3/4/5-sphere coupled stack world-contact batch paths, plus manually
  assembled 1-/4-/8-contact articulated unified-contact batch paths including
  cross-multibody link-vs-link packets, mixed
  separated/stack/articulated grouped contact batch paths, and PGS-only
  homogeneous dense box-face CUDA batches through 48 boxes, pass.
  Jacobi has opt-in solver-internal CPU
  worker-thread correctness and benchmark evidence, including larger 2048-row
  banded rows, but the focused local rows did not show a general speedup. Other intra-solver multi-threaded
  CPU paths, general CUDA LCP solver execution, CUDA Jacobi dense-contact
  execution, end-to-end articulated world-step CUDA execution, and broader
  vectorized/CUDA LCP batch-processing paths still need separate evidence.
- Background taxonomy upkeep: keep `docs/background/lcp/`, the solver manifest,
  and benchmark registration synchronized whenever solver support changes.

## Immediate Next Steps

1. Extend solver-specific friction-index conditioning/coupling grids beyond the
   current exact rank-deficient 128-row/16-contact and production active-set
   transition 48-contact slices.
2. Extend DART 7 boxed-LCP world-contact evidence from current separated
   sphere-ground, current fixed-base prismatic articulated end-to-end coverage,
   current connected Cartesian-chain articulated end-to-end coverage,
   current cross-multibody articulated link-vs-link impact coverage,
   current manually assembled three-axis articulated LCP snapshots, and
   current 5-sphere vertical-stack snapshots and dense box face-contact
   evidence to broader articulated, longer-running, and denser coupled contact
   scenes.
3. Add broader benchmark gates for SIMD-enabled CPU, intra-solver
   multi-threaded CPU, general CUDA LCP solver execution, and vectorized/CUDA
   batch-processing paths.
