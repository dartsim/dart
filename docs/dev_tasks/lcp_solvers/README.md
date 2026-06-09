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
      evidence, including larger 8192-row banded packets.
- [x] Added focused PGS/PSOR and symmetric PSOR relaxation sweep benchmark rows
      covering under-relaxation, plain relaxation, and over-relaxation on
      standard, boxed, and 8-/16-contact friction-index fixtures.
- [x] Added focused Red-Black Gauss-Seidel relaxation sweep benchmark rows with
      two-color partition counters on standard, boxed, and 8-/16-contact
      friction-index fixtures, plus focused solver-internal threaded
      banded-standard rows through 8192 rows.
- [x] Added opt-in solver-internal CPU worker threads for `BlockedJacobiSolver`
      independent block solves, with focused generated correctness and
      banded-standard benchmark rows through 8192 rows.
- [x] Added focused APGD restart-policy sweep benchmark rows on standard,
      boxed, and friction-index fixtures.
- [x] Added focused TGS iteration-budget sweep benchmark rows on standard,
      boxed, and friction-index fixtures.
- [x] Added focused NNCG PGS-preconditioner iteration sweep benchmark rows on
      standard, boxed, and friction-index fixtures.
- [x] Added focused SubspaceMinimization PGS-iteration sweep benchmark rows on
      standard, boxed, and friction-index fixtures.
- [x] Added focused ShockPropagation layer-layout sweep benchmark rows on
      standard, boxed, and 8-/16-contact friction-index fixtures.
- [x] Added focused MPRGP SPD/positive-definite-check sweep benchmark rows on
      standard SPD fixtures.
- [x] Added focused Interior Point path-parameter sweep benchmark rows on
      standard SPD fixtures.
- [x] Added focused Staggering contact-pipeline sweep benchmark rows on DART 7
      separated world-contact, coupled stack-contact, and articulated unified
      contact fixtures up to 8 contacts.
- [x] Added focused Boxed Semi-Smooth Newton line-search sweep benchmark rows
      on standard, boxed, and 8-/16-contact friction-index fixtures.
- [x] Added focused ADMM rho/adaptive-rho sweep benchmark rows on standard,
      boxed, and 8-/16-contact friction-index fixtures.
- [x] Added focused SAP regularization sweep benchmark rows on standard, boxed,
      and 8-/16-contact friction-index fixtures.
- [x] Added focused ADMM, SAP, and Boxed Semi-Smooth Newton contact comparison
      benchmark rows on DART 7 separated world-contact, coupled stack-contact,
      and articulated unified-contact fixtures.
- [x] Added scoped solver generated correctness and single/batch benchmark rows
      for 1x-/4x-/8x-coupled mildly ill-conditioned DART 7 friction-index
      packets at 6, 8, 12, 16, 24, 32, 48, 64, and 96 contacts, plus
      16x-coupled generated correctness through 192 contacts, single benchmark
      rows through 256 contacts, and default batch rows through 256 contacts.
      Boxed
      Semi-Smooth Newton reports tuned line-search settings on the 16x batch
      rows; focused SIMD/CUDA all-solver serial/parallel 192-contact batch
      rows now pass as contract gates.
- [x] Broadened the scoped larger mildly ill-conditioned DART 7 friction-index
      evidence to 1x-, 4x-, and 8x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-contact packets plus 16x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-/128-contact packets.
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
      boxed, and friction-index packets, homogeneous 4-, 8-, 16-, 24-, and 32-contact
      DART 7 world-contact packets, homogeneous 5-/6-/7-/8-/9-/10-/11-/12-/13-/14-/15-/16-sphere coupled
      stack-contact packets, grouped variable-size 1/2/4/8/16/24/32-contact
      sphere-ground packets, grouped variable-size 2/3/4/5/6/7/8/9/10/11/12/13/14/15/16-sphere
      coupled stack-contact packets with two- and three-variant stack grouped
      benchmark rows, and
      grouped variable-size manually assembled 1-/4-/8-/16-/24-/32-contact articulated
      unified-contact packets with two- and three-variant grouped benchmark
      rows including cross-multibody link-vs-link cases,
      plus mixed grouped contact packets combining those separated, stack, and
      1-/4-/8-/16-/24-/32-contact articulated fixture families, with CUDA unit
      coverage and benchmark rows.
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
      standard through 32 rows, boxed through 48 rows, and friction-index
      through 16 contacts.
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
      friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, 48-, 64-, and
      96-, 128-, 192-, and 256-contact cases. The 96-, 128-, 192-, and
      256-contact packets keep the coupled friction-index topology but use the
      contract-verified capped normal ramp and `1e6` diagonal spread. The
      friction-index known-solution slice is Dantzig-only after
      `ShockPropagation` contract-succeeded but missed the selected generated
      solution tolerance on the near-singular coupled packets.
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
      exact-solution scoped solvers covering standard 32-row and 64-row, boxed
      16-row and 32-row, friction-index 8-contact, and coupled friction-index
      6-, 8-, 12-, 16-, 24-, 32-, 48-, and 64-contact cases, plus
      1x-/4x-/8x-coupled 6-/8-/12-/16-/24-/32-/48-/64-/96-contact cases and
      16x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact cases with Boxed
      Semi-Smooth Newton included across those coupled rows. Projection-like
      solvers that satisfy the contract but return alternate valid solutions
      remain covered by benchmark contract rows. `MPRGP` is intentionally
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
- [x] Added production active-set transition generated coverage through a
      256-contact/768-row packet and 128 `BM_LcpProductionActiveSetTransition`
      rows for stronger-coupled
      24-contact/72-row, 32-contact/96-row, 48-contact/144-row, and
      64-contact/192-row, 96-contact/288-row, 128-contact/384-row, and
      192-contact/576-row and 256-contact/768-row friction-index packets,
      verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 550
      `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
      batch-size-4 runs over standard 32/64/128-row, boxed 32/64/128-row, and
      coupled friction-index
      8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact active-set packets,
      verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 629 `BM_LcpMildIllConditioned` benchmark rows for the scoped larger
      mildly ill-conditioned standard 32-row, boxed 16-row, friction-index
      8-contact, 1x-/4x-/8x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-contact single-problem packets, and
      16x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets.
      Focused default, SIMD-enabled, and CUDA-enabled runs verify the
      256-contact single rows with `contract_ok=1`. The SIMD-enabled SAP
      192-contact row is contract-correct but slow and reports 20k iterations,
      so no speedup is claimed.
- [x] Added 1258 `BM_LcpMildIllConditionedBatch(Serial|Parallel)` benchmark rows
      for batch-size-4 runs over the full scoped mildly ill-conditioned packet
      set: standard 32-row, boxed 16-row, friction-index 8-contact, coupled
      friction-index 6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 4x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 8x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-contact, and 15-solver 16x-coupled
      6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact rows. Boxed
      Semi-Smooth Newton reports tuned line-search settings on the 16x rows.
      The default build verifies the retained 192- and 256-contact batch rows;
      prior rows through the 128-contact 16x packet were verified in default,
      SIMD-enabled, and CUDA-enabled build trees. Focused SIMD/CUDA all-solver
      serial/parallel 192-contact batch rows pass as contract gates. Focused
      SIMD/CUDA selected-solver serial/parallel 256-contact batch rows pass for
      14 of the 15 registered solvers: all except `Sap`. A focused
      SIMD-enabled `Sap` batch probe was stopped after 180s before producing a
      benchmark row, so the full all-solver 256-contact SIMD/CUDA batch gate
      remains unclaimed.
- [x] Added 31 `BM_LcpNearSingular` benchmark rows for the scoped robust
      near-singular standard 8-row, boxed 8-row, and coupled friction-index
      3-, 6-, 9-, 12-, 16-, 24-, 32-, 48-, 64-, 96-, 128-, 192-, and
      256-contact packets, verified in default, SIMD-enabled, and CUDA-enabled
      build trees.
- [x] Added 62 `BM_LcpNearSingularBatch(Serial|Parallel)` benchmark rows for
      batch-size-4 runs over near-singular standard 8-row, boxed 8-row, and
      coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, 48-, 64-, 96-, 128-,
      192-, and 256-contact packets, verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
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
- [x] Added extreme exact rank-deficient singular-degenerate coverage and 51
      `BM_LcpExtremeSingularDegenerate` rows for scoped standard 128-row,
      boxed 128-row, and coupled friction-index
      16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets,
      verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 72
      `BM_LcpSingularDegenerateFrictionIndexBatch(Serial|Parallel)` benchmark
      rows for batch-size-4 runs over exact rank-deficient coupled
      friction-index
      6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets,
      verified in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 192
      `BM_LcpSingularDegenerateStandardBoxedBatch(Serial|Parallel)` benchmark
      rows for batch-size-4 runs over exact rank-deficient standard and boxed
      16-/32-/64-/128-row packets, verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
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
      for two-, four-, eight-, and sixteen-chain fixed-base three-axis
      prismatic Cartesian chains in ground contact, plus 1-/4-/8-/16-chain articulated
      Cartesian
      ground-step benchmark rows through the public unified constraint path.
- [x] Added DART 7 boxed-LCP two-sided articulated contact evidence for
      one-link, four-pair, eight-pair, and sixteen-pair fixed-base prismatic
      link-vs-dynamic-rigid scenes, including a sixteen-pair 200-step scene,
      plus 1-/4-/8-/16-pair one-step and 16-pair 200-step articulated
      rigid-impact benchmark rows.
- [x] Added DART 7 boxed-LCP cross-multibody articulated contact evidence for
      one-pair, four-pair, eight-pair, and sixteen-pair fixed-base prismatic
      link-vs-link scenes, including a sixteen-pair 200-step scene, plus
      1-/4-/8-/16-pair one-step and 16-pair 200-step articulated link-impact
      benchmark rows.
- [x] Added DART 7 articulated unified-contact benchmark rows that compare all
      friction-index-capable solvers on the same manually assembled fixed-base
      three-axis prismatic link-ground, link-vs-dynamic-rigid, and
      cross-multibody link-vs-link LCP snapshots through 64-contact packets.
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
- [x] Preserved the boxed-LCP kinematic-contact compatibility contract by
      skipping Baumgarte velocity bias when either contact body is kinematic;
      focused default and CUDA runs of
      `World.BoxedLcpContactTreatsKinematicBodyAsStaticObstacle` leave the
      dynamic body velocity unchanged.
- [x] Extended DART 7 boxed-LCP coupled stack public-step evidence to a
      5-sphere stack that satisfies the same finite-state, spacing, near-rest,
      lateral-drift, and static-ground invariants after `World::step(500)`,
      with a matching benchmark row in default, SIMD-enabled, and CUDA-enabled
      build trees.
- [x] Extended DART 7 boxed-LCP coupled stack evidence to a 6-sphere, 6-contact,
      18-row vertical stack, with unit snapshot-contract coverage, a 1000-step
      public `World::step()` invariant test, all-solver stack benchmark rows
      through 7 spheres and scoped 8-/9-/10-/11-/12-/13-sphere all-solver stack
      evidence. The PGS, Jacobi, Blocked Jacobi, Red-Black Gauss-Seidel, and
      ShockPropagation stack rows use a 512-iteration cap; Symmetric PSOR, BGS,
      and TGS use that cap on the 11-/12-/13-sphere rows; and the NNCG stack rows
      use 20 PGS preconditioner iterations through 11 spheres and 40 at 12
      spheres. Focused default, SIMD-enabled, and CUDA-enabled build-tree rows
      pass for the 11-/12-/13-sphere all-solver stack slice; the CUDA-enabled rows
      are CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel
      execution.
- [x] Extended DART 7 boxed-LCP coupled stack snapshot evidence to a 7-sphere,
      7-contact, 21-row vertical stack and matching boxed-LCP assembly
      benchmark row, plus 16 friction-index solver benchmark rows. `NNCG` uses
      a 20-iteration PGS preconditioner for the coupled stack family; the
      `RedBlackGaussSeidel` row uses a 512-iteration stack-contact cap after
      focused 100- and 128-iteration probes did not satisfy the LCP contract at
      the current larger stack sizes. A 7-sphere public-step invariant is not
      claimed.
- [x] Extended DART 7 boxed-LCP coupled stack snapshot evidence to an 8-sphere,
      8-contact, 24-row vertical stack, with a matching assembly benchmark row
      and 16 friction-index solver benchmark rows. The `Pgs`, `Jacobi`,
      `BlockedJacobi`, `RedBlackGaussSeidel`, and `ShockPropagation` rows use a
      512-iteration cap, and the `NNCG` row uses the 20-iteration PGS
      preconditioner after a 10-iteration probe failed. No 8-sphere public-step
      invariant is claimed.
- [x] Extended DART 7 boxed-LCP coupled stack snapshot evidence to a 9-sphere,
      9-contact, 27-row vertical stack, with a matching assembly benchmark row
      and 16 friction-index solver benchmark rows. Focused default,
      SIMD-enabled, and CUDA-enabled runs satisfy the LCP contract; no 9-sphere
      public-step invariant is claimed. The PGS, Jacobi, Blocked Jacobi, and
      Red-Black Gauss-Seidel, and ShockPropagation rows use the same
      512-iteration cap, and the NNCG row uses the same 20-iteration PGS
      preconditioner as the smaller coupled-stack rows.
- [x] Extended DART 7 boxed-LCP coupled stack snapshot evidence to a 10-sphere,
      10-contact, 30-row vertical stack, with a matching assembly benchmark row
      and 16 friction-index solver benchmark rows. Focused default,
      SIMD-enabled, and CUDA-enabled runs satisfy the LCP contract; no
      10-sphere public-step invariant is claimed. The PGS, Jacobi, Blocked
      Jacobi, Red-Black Gauss-Seidel, and ShockPropagation rows use the same
      512-iteration cap, and the NNCG row uses the same 20-iteration PGS
      preconditioner as the smaller coupled-stack rows.
- [x] Extended DART 7 boxed-LCP coupled stack snapshot and solver-comparison
      evidence to 11- and 12-sphere vertical stacks, validating 11-/12-contact
      and 33-/36-row boxed/findex LCP snapshots assembled from shared dynamic
      bodies. Focused default build rows satisfy the LCP contract for all 16
      friction-index solvers plus the assembly rows; no 11-/12-sphere
      public-step invariant is claimed.
- [x] Extended DART 7 boxed-LCP coupled stack solver-comparison evidence to the
      13-sphere vertical stack, validating 13-contact and 39-row boxed/findex
      LCP rows across all 16 friction-index solvers in default, SIMD-enabled,
      and CUDA-enabled build trees.
- [x] Extended DART 7 boxed-LCP coupled stack snapshot and assembly evidence to
      14-, 15-, and 16-sphere vertical stacks, validating 14-/15-/16-contact
      and 42-/45-/48-row boxed/findex LCP snapshots assembled from shared
      dynamic bodies. Focused default build assembly rows satisfy the LCP
      contract; no 14+-sphere solver-comparison or public-step invariant is
      claimed.
- [x] Added DART 7 world-contact `BM_LCP_COMPARE` rows that run every
      friction-index-capable solver on the same boxed-LCP snapshots assembled
      from 1, 2, and 4 separated sphere-ground contacts, plus a
      contact assembly/solve benchmark for the boxed-LCP world path.
- [x] Added coupled DART 7 world-contact stack benchmark rows for 2- and
      3-sphere vertical stacks across every friction-index-capable solver, plus
      4-, 5-, and 6-sphere vertical-stack rows for the same solver set
      (`NNCG` reports a 20-iteration PGS preconditioner for this coupled
      contact family), plus 7-sphere rows for that solver set, an 8-sphere
      `RedBlackGaussSeidel` row, and 8-/9-/10-/11-/12-/13-sphere rows for the full
      solver set, so the comparison now includes larger boxed/findex snapshots
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
      `World::collide()`, APGD-verified in the unit test, plus 72 scoped
      `BM_LcpWorldBoxContact/FrictionIndex` benchmark rows over
      1/2/4/8/16/24/32/48/64/96/128/192-box snapshots verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
- [x] Added DART 7 dense box-face serial and `ParallelExecutor` batch benchmark
      rows: 72 `BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex`
      rows cover `Pgs`, `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and
      `Admm` over 24/64/96/128/192-box snapshots with batch size 4. `Pgs` also
      covers 1/4/8/16/32/48-box snapshots, so the CPU serial and
      `ParallelExecutor` rows match the homogeneous CUDA PGS packet sizes
      through 96 boxes in default, SIMD-enabled, and CUDA-enabled build trees.
- [x] Added contact-derived CUDA batch evidence for homogeneous 4-, 8-, 16-,
      24-, and 32-contact DART 7 world-contact packets, covering fixed-iteration CUDA
      Jacobi and PGS unit tests and benchmark rows on the visible GPU.
- [x] Added contact-derived CUDA batch evidence for homogeneous
      5-/6-/7-/8-/9-/10-/11-/12-/13-/14-/15-/16-sphere DART 7 coupled stack-contact packets, covering
      fixed-iteration CUDA Jacobi and PGS unit tests and benchmark rows on the
      visible GPU.
- [x] Extended contact-derived CUDA batch evidence to homogeneous and grouped
      variable-size 16-sphere DART 7 coupled stack-contact packets. Focused
      CUDA benchmark rows report `contract_ok=1`, `cuda_lcp_execution=1`, and
      `cuda_world_stack_contact_batch=1` for both fixed-iteration Jacobi and
      PGS.
- [x] Added grouped variable-size synthetic CUDA batch evidence for DART 7
      standard and boxed packets through 192 rows and friction-index packets
      through 64 contacts. The benchmark rows cover
      16/32/48/96/128/192-row standard/boxed groups and
      4/8/16/32/48/64-contact friction-index groups, covering
      fixed-iteration CUDA Jacobi and PGS unit tests and two-/three-variant
      benchmark rows on the visible GPU.
- [x] Added apples-to-apples grouped variable-size DART 7 CPU serial and
      DART 7 `ParallelExecutor` batch benchmark rows for Jacobi and PGS on the
      same grouped synthetic packets, verified in default, SIMD-enabled, and
      CUDA-enabled build trees.
- [x] Added apples-to-apples DART 7 CPU serial, DART 7 `ParallelExecutor`, and
      fixed-iteration CUDA batch benchmark rows for Jacobi and PGS on the same
      standard/boxed 24-/48-/96-/128-/192-/256-row and friction-index
      8-/16-/32-/48-/64-/96-contact packets at batch size 4, verified in default,
      SIMD-enabled, and CUDA-enabled build trees.
- [x] Added grouped variable-size CUDA contact-batch evidence for DART 7
      1/2/4/8/16/24/32-contact separated sphere-ground packets, covering
      fixed-iteration CUDA Jacobi and PGS unit tests and benchmark rows on the
      visible GPU.
      The separated grouped tests now run two and three velocity variants per
      contact count, matching the `/2` and `/3` benchmark rows.
- [x] Added grouped variable-size CUDA contact-batch evidence for DART 7
      2/3/4/5/6/7/8/9/10/11/12/13/14/15/16-sphere coupled stack-contact packets, covering fixed-iteration
      CUDA Jacobi and PGS unit tests and benchmark rows on the visible GPU.
      The stack grouped tests now run two and three velocity variants per sphere
      count, matching the `/2` and `/3` benchmark rows.
- [x] Added grouped variable-size CUDA contact-batch evidence for DART 7
      manually assembled fixed-base three-axis prismatic articulated
      unified-contact packets with 1, 4, 8, 16, 24, and 32 contacts, covering fixed-iteration
      CUDA Jacobi and PGS unit tests and benchmark rows on the visible GPU.
- [x] Added mixed grouped CUDA contact-batch evidence for DART 7 separated
      sphere-ground, coupled stack-contact, and manually assembled 1-/4-/8-/16-/24-/32-contact
      articulated unified-contact packets including cross-multibody link-vs-link
      cases in one size-grouped batch, covering fixed-iteration CUDA Jacobi and PGS unit
      tests and benchmark rows on the visible GPU.
- [x] Added dense box-face CUDA contact-batch evidence for DART 7:
      homogeneous 4-problem 1-/4-/8-/16-/24-/32-/48-/64-/96-/128-box CUDA Jacobi
      rows, homogeneous 4-problem 1-/4-/8-/16-/24-/32-/48-/64-/96-/128-box CUDA PGS rows,
      and grouped
      variable-size 1/2/4/8/16/24/32/48/64/96-box CUDA Jacobi and PGS batches with two and three
      velocity variants per box-count shape of box-face `World::collide()`
      snapshots pass fixed-iteration CUDA benchmark coverage on the visible
      GPU; CUDA unit coverage now includes homogeneous Jacobi
      1-/4-/8-/16-/24-/32-/48-/64-/96-/128-box packets, homogeneous PGS
      1-/16-/24-/32-/48-/64-/96-box packets, the grouped
      1/2/4/8/16/24/32/48/64/96-box Jacobi and PGS packets, and bounded
      homogeneous 128-box single-problem CUDA Jacobi and PGS packets.
      A focused fixture boundary test now verifies that the same dense box-face
      construction keeps the 128-box grid at 512 contacts and 1536 LCP rows.
      The focused 128-box batch-size-4 CUDA PGS benchmark row now reports
      `contract_ok=1` with 512 contacts, 1536 LCP rows per problem, and 6144
      total rows.
- [x] Added dense box-face DART 7 end-to-end unit and benchmark evidence:
      `FourBoxWorldStepMaintainsDenseContactInvariants` and
      `EightBoxWorldStepMaintainsDenseContactInvariants` advance 4-box and
      8-box dense face-contact scenes through public boxed-LCP `World::step()`;
      `SixteenBoxWorldStepMaintainsDenseContactInvariants` extends the same
      scene family to 16 boxes with a longer 500-step horizon, and
      `TwentyFourBoxWorldStepMaintainsDenseContactInvariants` extends unit
      coverage to a 24-box/96-contact small-timestep scene.
      `ThirtyTwoBoxWorldStepMaintainsDenseContactInvariants` extends unit
      coverage again to a 32-box/128-contact small-timestep scene, and
      `FortyEightBoxWorldStepMaintainsDenseContactInvariants` extends unit
      coverage to the 48-box/192-contact benchmark boundary. The
      `SixtyFourBoxWorldStepPreservesDenseContactShape` smoke test covers one
      public boxed-LCP `World::step()` on a 64-box/256-contact dense face
      scene, and
      `SixtyFourBoxWorldShortHorizonMaintainsDenseContactInvariants` covers
      the same 64-box/256-contact scene for 75 strict invariant-checked public
      steps without claiming the longer settling invariant. The
      `BM_LcpWorldBoxStep_BoxedLcp` rows report matching invariant counters for
      4/8/16/32-contact 200-step scenes, the 64-contact 500-step scene, the
      96-contact 2000-step scene, the 128-/192-contact 4000-step scenes, and
      the 256-contact one-step and 75-step scenes. The 75-step 64-box row now
      has focused default, SIMD-enabled, and CUDA-enabled build-tree evidence.
- [x] Added DART 7 per-contact block-structure evidence for BGS and
      Blocked Jacobi on real two-contact boxed-LCP world-contact snapshots:
      the tests prove `findex`-derived non-contiguous contact blocks solve the
      snapshot and explicit block partitions that split normal/tangent rows are
      rejected. A focused `BM_LCP_COMPARE` gate also verifies 22 BGS/Blocked
      Jacobi world-contact, stack-contact, and serial/parallel batch rows with
      `contract_ok=1`.
- [x] Added focused BGS/Blocked Jacobi block-partition sweep benchmark rows for
      full-block, 3-row block, auto `findex`, and explicit contact-block
      partitions on standard, boxed, and 4-/8-contact friction-index fixtures.
- [x] Added opt-in projected gradient-descent warm starts for the native
      standard-LCP paths of Minimum Map, Fischer-Burmeister, and Penalized
      Fischer-Burmeister Newton, with focused tests proving each initializer
      reduces its solver-specific merit before Newton line search.
- [x] Added opt-in PGS warm starts for the same native standard-LCP Newton
      paths, with focused tests proving each accepted seed reduces its
      solver-specific merit before Newton line search.
- [x] Added 36 `BM_LcpNewtonWarmStart` benchmark rows that compare no seed,
      PGS, projected gradient descent, and PGS-then-gradient modes for the
      three native standard-LCP Newton solvers on identical 32-row, 64-row,
      and 128-row active-set transition packets, verified in default,
      SIMD-enabled, and CUDA-enabled build trees.
- [x] Added 72
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
      articulated contacts, 7-sphere coupled snapshots, 6-sphere coupled-stack
      public-step rows, 48-box unit/benchmark dense face-contact long-horizon
      step coverage, and bounded 64-box dense face-contact one-step shape and
      75-step strict-invariant checks.
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
  reported 15 rows, and JSON checks for `BM_LcpPivotingScaleSweep` reported 15
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover Direct 2-row and 3-row standard fixtures,
  Lemke and Baraff 8-row and 16-row standard fixtures, and Dantzig 8-row and
  16-row and 32-row standard, 12-row, 24-row, and 48-row boxed, and
  4-contact, 8-contact, and 16-contact friction-index fixtures. The rows
  report `pivoting_scale_sweep=1`, Direct no-fallback counters, six Dantzig
  boxed-or-findex rows, `contact_count=4/8/16`,
  `problem_size=2/3/8/12/16/24/32/48`, observed solver
  `iterations=1/4/8/16`, and backend
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
  `has_simulation_experimental=1`, and `contract_ok=1` for the previous 1752
  `BM_LcpMildIllConditioned` single and batch rows; the new 192-contact single
  rows also pass in that build tree, with the SAP row taking 20k iterations.
  The focused
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
  `has_simulation_experimental=1`, and `contract_ok=1` for the previous 1752
  `BM_LcpMildIllConditioned` single and batch rows; the new 192-contact single
  rows also pass in that build tree, and focused CUDA-enabled all-solver
  serial/parallel 192-contact batch rows now report 30 rows with
  `contract_ok=1`. The focused
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
  passed 33 tests, including standard, boxed, friction-index, grouped
  variable-size synthetic standard/boxed/friction-index CUDA batches,
  homogeneous contact-derived world-contact CUDA batches, homogeneous
  5-/6-/7-/8-sphere coupled stack-contact CUDA batches, and grouped
  variable-size 1/2/4/8/16/24/32-contact separated world-contact and
  2/3/4/5/6/7/8-sphere coupled stack world-contact
  packets with two and three variants per contact or sphere count, and manually assembled
  1-/4-/8-/16-contact articulated unified-contact CUDA batches covering link-ground,
  link-vs-dynamic-rigid, and cross-multibody link-vs-link packets, plus mixed
  grouped contact batches combining the separated, stack, and articulated
  fixture families for Jacobi and PGS on the visible GPU.
  A focused current follow-up
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.HomogeneousStackedWorldContactBatchSatisfiesLcpContract:CudaLcpJacobiBatch.StackedWorldContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.HomogeneousStackedWorldContactBatchSatisfiesLcpContract:CudaLcpPgsBatch.StackedWorldContactGroupedBatchSatisfiesLcpContract' --gtest_brief=1`
  passed the four stack CUDA tests after adding the 10-sphere homogeneous and
  grouped coupled-stack packets.
  `build/cuda/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='BM_LcpBatchSerial/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)|BM_LcpBatchParallel/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)|BM_LcpCuda(Jacobi|Pgs)Batch|BM_LcpCuda(Jacobi|Pgs)GroupedBatch|BM_LcpCuda(Jacobi|Pgs)WorldContactBatch|BM_LcpCuda(Jacobi|Pgs)WorldContactGroupedBatch' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran successfully. The CUDA rows reported `build_cuda_enabled=1`,
  `cuda_lcp_execution=1`, `cuda_batch_execution=1`, fixed iteration counters,
  and `contract_ok=1` for standard, boxed, friction-index, grouped synthetic
  standard/boxed/friction-index, and homogeneous 4-, 8-, 16-, 24-, and 32-contact world-contact
  Jacobi and PGS batches, plus grouped variable-size 1/2/4/8/16/24/32-contact
  separated sphere-ground and 2/3/4/5/6/7/8-sphere coupled stack CUDA batches. An
  earlier focused direct/grouped synthetic follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)(Batch|GroupedBatch)_(Standard|Boxed|FrictionIndex)' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported 36 CUDA rows with `contract_ok=1` at the then-registered sizes. The
  direct homogeneous rows now cover standard and boxed
  24-/48-/96-/128-/192-/256-row packets and
  friction-index 8-/16-/32-/48-/64-/96-contact packets at batch size 4. A
  focused grouped synthetic follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)GroupedBatch_(Standard|Boxed|FrictionIndex)/(2|3)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported 12 CUDA rows with `contract_ok=1` and `cuda_group_count=7`. The
  `/2` rows report `batch_size=14`; the standard and boxed grouped rows report
  `min_problem_size=16`, `max_problem_size=256`, and `total_problem_size=1536`,
  while the friction-index grouped rows report `min_contact_count=4`,
  `max_contact_count=96`, `total_contact_count=536`, `min_problem_size=12`,
  `max_problem_size=288`, and `total_problem_size=1608`. The `/3` rows report
  `batch_size=21`, standard/boxed `total_problem_size=2304`, and
  friction-index `total_contact_count=804` and `total_problem_size=2412`. A
  focused grouped CPU follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpGroupedBatch(Serial|Parallel)/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)/(2|3)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported 24 serial/`ParallelExecutor` rows with `contract_ok=1` in default,
  SIMD-enabled, and CUDA-enabled build trees. Those rows report
  `batch_group_count=7`; `/2` rows report `batch_size=14`,
  standard/boxed `max_problem_size=256` and `total_problem_size=1536`, and
  friction-index `max_contact_count=96`, `max_problem_size=288`,
  `total_contact_count=536`, and `total_problem_size=1608`. The `/3` rows
  report `batch_size=21`, standard/boxed `total_problem_size=2304`,
  friction-index `total_contact_count=804` and `total_problem_size=2412`, and
  `ParallelExecutor` rows report `parallel_units=14/21` with
  `profile_enabled=1`. A
  focused homogeneous
  apples-to-apples CPU/CUDA follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpBatch(Serial|Parallel)/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)/(24|48|96|8|16|32)/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  in the default build reported 36 CPU rows with `contract_ok=1`,
  `batch_size=4`, standard/boxed `problem_size=24/48/96`,
  friction-index `contact_count=8/16/32`, `parallel_units=4` on
  `ParallelExecutor` rows, and `total_problem_size` up to 384. The matching
  SIMD-enabled command reported the same 36 CPU rows with `contract_ok=1`,
  `build_simd_enabled=1`, and the same size, batch, and `parallel_units`
  counters. A focused larger-size CPU follow-up in both default and SIMD builds
  reported the 12 serial/`ParallelExecutor` rows with `contract_ok=1` for
  standard/boxed 128-row and friction-index 48-contact packets, `batch_size=4`,
  `parallel_units=4` on parallel rows, standard/boxed `total_problem_size=512`,
  and friction-index `total_problem_size=576`. A focused 192-row/64-contact CPU
  follow-up in default, SIMD-enabled, and CUDA-enabled build trees reported 12
  serial/`ParallelExecutor` rows with `contract_ok=1`, `batch_size=4`,
  standard/boxed and friction-index `problem_size=192`, `total_problem_size=768`,
  friction-index `contact_count=64`, and `parallel_units=4` on parallel rows.
  A focused 256-row/96-contact CPU follow-up in default, SIMD-enabled, and
  CUDA-enabled build trees reported the next 12 serial/`ParallelExecutor` rows
  with `contract_ok=1`, `batch_size=4`, standard/boxed `problem_size=256`,
  friction-index `problem_size=288`, friction-index `contact_count=96`,
  `total_problem_size=1024/1152`, and `parallel_units=4` on parallel rows.
  The focused CUDA unit run
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.LargerSyntheticBatchSatisfiesLcpContract:CudaLcpPgsBatch.LargerSyntheticBatchSatisfiesLcpContract' --gtest_brief=1`
  passed both larger synthetic batch tests with the 256-row and 96-contact cases
  included.
  The matching CUDA command
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)Batch_(Standard|Boxed|FrictionIndex)' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  reported 24 direct CUDA rows through the 128-row/48-contact boundary with `cuda_batch_execution=1`,
  `contract_ok=1`, the same batch size, standard/boxed `problem_size` through
  128, friction-index `contact_count` through 48, and maximum
  `total_problem_size=576`; the focused 192-row/64-contact CUDA follow-up
  reported six additional direct CUDA rows with `contract_ok=1`,
  `cuda_batch_execution=1`, standard/boxed and friction-index `problem_size=192`,
  friction-index `contact_count=64`, and `total_problem_size=768`. The focused
  256-row/96-contact CUDA follow-up reported six more direct CUDA rows with
  `contract_ok=1`, `cuda_lcp_execution=1`, `cuda_batch_execution=1`,
  standard/boxed `problem_size=256`, friction-index `problem_size=288`,
  friction-index `contact_count=96`, and `total_problem_size=1024/1152`. A focused homogeneous
  16-/24-/32-contact follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)WorldContactBatch_FrictionIndex/(16|24|32)/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported six homogeneous CUDA rows with `contract_ok=1`,
  `cuda_lcp_execution=1`, and `cuda_batch_execution=1`; the rows report
  `contact_count=16/24/32`, `problem_size=48/72/96`, `batch_size=4`,
  `total_contact_count=64/96/128`, `total_problem_size=192/288/384`, and
  maximum residual and complementarity `1.3877787807814457e-17` with no bound
  violation. A focused homogeneous stack follow-up
  `scripts/run_benchmark_smoke.py build/cuda/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)WorldStackContact(Batch_FrictionIndex/8/4|GroupedBatch_FrictionIndex/2$)' --benchmark_min_time=0.001s`
  reported four stack CUDA rows with `contract_ok=1`; the homogeneous 8-sphere
  rows reported `sphere_count=8`, `contact_count=8`, `problem_size=24`,
  `batch_size=4`, `total_contact_count=32`, and `total_problem_size=96`.
  The focused current 16-sphere follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)WorldStackContactBatch_FrictionIndex/(13|14|15|16)/4$|^BM_LcpCuda(Jacobi|Pgs)WorldStackContactGroupedBatch_FrictionIndex/[23]$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  reported 12 CUDA execution rows with `contract_ok=1` and
  `cuda_lcp_execution=1`; the homogeneous 13-/14-/15-/16-sphere rows reported
  `contact_count=13/14/15/16`, `problem_size=39/42/45/48`, `batch_size=4`,
  `total_contact_count=52/56/60/64`, and `total_problem_size=156/168/180/192`,
  while the grouped `/2` and `/3` rows reported `max_problem_size=48`,
  `batch_size=30/45`, `cuda_group_count=15`, `total_contact_count=270/405`,
  and `total_problem_size=810/1215`.
  A focused
  separated-contact follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)WorldContactGroupedBatch_FrictionIndex/[23]$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported four separated CUDA rows with `contract_ok=1`, `cuda_group_count=7`,
  `contact_shape_count=7`, `min_problem_size=3`, `max_problem_size=96`,
  `batch_size=14/21`, `total_contact_count=174/261`, and
  `total_problem_size=522/783`. A focused stack follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)WorldStackContactGroupedBatch_FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two stack CUDA rows with `contract_ok=1`, `batch_size=30`,
  `cuda_group_count=15`, `contact_shape_count=15`, `min_problem_size=6`,
  `max_problem_size=48`, `total_contact_count=270`, and
  `total_problem_size=810`. A focused three-variant stack follow-up
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)WorldStackContactGroupedBatch_FrictionIndex/3$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  reported two stack CUDA rows with `contract_ok=1`,
  `problem_variants_per_shape=3`, `batch_size=45`, `cuda_group_count=15`,
  `contact_shape_count=15`, `min_problem_size=6`, `max_problem_size=48`,
  `total_contact_count=405`, and `total_problem_size=1215`. A focused
  articulated unified-contact follow-up
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract' --gtest_brief=1`
  previously passed both CUDA grouped-batch tests, and
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpCuda(Jacobi|Pgs)ArticulatedUnifiedContactGroupedBatch_FrictionIndex' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  previously reported two articulated CUDA rows with `contract_ok=1`, `batch_size=24`,
  `cuda_group_count=4`, `contact_shape_count=4`,
  `articulated_contact_case_count=3`, `articulated_cross_link_contact=1`,
  `min_problem_size=3`, `max_problem_size=48`, `total_contact_count=174`, and
  `total_problem_size=522`. A focused current articulated follow-up
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract' --gtest_brief=1`
  passed both CUDA grouped-batch tests after adding 24- and 32-contact packets.
  The matching
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)ArticulatedUnifiedContactGroupedBatch_FrictionIndex/[23]$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  run reported four articulated CUDA rows with `contract_ok=1`,
  `cuda_lcp_execution=1`, `cuda_grouped_batch_execution=1`,
  `cuda_group_count=6`, `contact_shape_count=6`, `min_problem_size=3`,
  `max_problem_size=96`, `batch_size=36/54`,
  `total_contact_count=510/765`, `total_body_count=170/255`,
  `total_problem_size=1530/2295`, `cuda_fixed_iterations=512/1024`,
  `max_bound_violation=0`, and
  `max_residual=max_complementarity=2.7755575615628914e-17`. A focused
  mixed-contact follow-up
  `test_lcp_jacobi_batch_cuda --gtest_filter='CudaLcpJacobiBatch.MixedContactGroupedBatchSatisfiesLcpContract:CudaLcpPgsBatch.MixedContactGroupedBatchSatisfiesLcpContract' --gtest_brief=1`
  passed both CUDA grouped-batch tests for two and three variants per mixed
  scenario, and
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCuda(Jacobi|Pgs)MixedContactGroupedBatch_FrictionIndex/[23]$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  reported four mixed CUDA rows with `contract_ok=1`,
  `batch_size=44/66`, `contact_fixture_family_count=3`, `cuda_group_count=8`,
  `contact_shape_count=8`, `problem_variants_per_shape=2/3`,
  `articulated_contact_case_count=3`, `articulated_cross_link_contact=1`,
  `min_problem_size=3`, `max_problem_size=96`, `total_contact_count=534/801`,
  `total_problem_size=1602/2403`, and
  `max_residual=2.2204460492503131e-16`. This proves narrow
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
  24-contact known-solution cases over exact-solution scoped solvers, plus
  1x-/4x-/8x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact and 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact mildly ill-conditioned
  friction-index slice. Projection-like solvers that satisfy the contract but
  return alternate valid solutions are kept in benchmark contract coverage
  instead;
  near-singular standard 8-row, boxed 8-row, and coupled friction-index 3-, 6-,
  9-, 12-, 16-, 24-, 32-, 48-, 64-, 96-, 128-, 192-, and 256-contact
  known-solution cases;
  exact rank-deficient
  singular-degenerate standard 16-row, boxed 16-row, and coupled friction-index
  6-contact known-solution cases; and larger exact rank-deficient
  singular-degenerate standard 32-row, boxed 32-row, and coupled friction-index
  8-contact known-solution cases; and stress exact rank-deficient
  singular-degenerate standard 64-row, boxed 64-row, and coupled friction-index
  12-contact known-solution cases; and extreme exact rank-deficient
  singular-degenerate standard 128-row, boxed 128-row, and coupled
  friction-index 16-/24-/32-/48-/64-/96-/128-/192-/256-contact known-solution cases for
  solver sets selected by observed robustness. `MPRGP` is not included in the larger
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
- Verified robust near-singular generated coverage follow-up:
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.NearSingularKnownSolutionsForRobustSolverSlice' --gtest_brief=1`
  passes in the default, SIMD-enabled, and CUDA-enabled build trees after
  adding the coupled friction-index 256-contact, 768-row packet and limiting
  friction-index known-solution checks to Dantzig. `ShockPropagation`
  contract-succeeded on the coupled near-singular packets but missed the
  selected exact generated solution tolerance by 0.95 to 20.98 in the focused
  probe, so it remains covered by near-singular benchmark contract rows rather
  than this exact-solution slice. The full default generated coverage suite previously
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_brief=1` also passes
  21 tests through the 128-contact packet. The CUDA-enabled run is CPU generated
  solver coverage in a CUDA-enabled build, not CUDA LCP kernel execution.
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
  reported 36 rows, and
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpNewtonWarmStart/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover `MinimumMapNewton`,
  `FischerBurmeisterNewton`, and
  `PenalizedFischerBurmeisterNewton` on the same standard active-set transition
  packets at 32, 64, and 128 rows. The rows report `active_set_transition=1`,
  `newton_pgs_warm_start`, `newton_gradient_warm_start`,
  `newton_pgs_warm_start_iterations`, and
  `newton_gradient_warm_start_iterations`, so no-seed, PGS-only,
  gradient-only, and PGS-then-gradient modes are distinguishable in the JSON
  output. The focused filter reports 12 `problem_size=128` single rows. The
  SIMD-enabled rows report `build_simd_enabled=1`; the
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified Newton warm-start batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNewtonWarmStartBatch' | wc -l`
  reported 72 rows, and
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpNewtonWarmStartBatch' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran all rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover batch-size-4 serial and DART 7
  `ParallelExecutor` runs for `MinimumMapNewton`, `FischerBurmeisterNewton`,
  and `PenalizedFischerBurmeisterNewton` on the same 32-row, 64-row, and
  128-row
  standard active-set transition packets and the same no-seed, PGS-only,
  gradient-only, and PGS-then-gradient mode matrix. The rows report
  `newton_warm_start_batch=1`, `batch_size=4`,
  `total_problem_size=128/256/512`,
  warm-start mode/iteration counters, serial or parallel execution counters,
  and backend build-state counters. Parallel rows report `parallel_units=4`,
  `worker_count=20`, and observed `max_parallelism` values up to 4. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU solver batch rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified PGS/PSOR relaxation sweep benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpPgsRelaxationSweep' | wc -l`
  reported 12 rows, and JSON checks for
  `BM_LcpPgsRelaxationSweep` reported 12 rows with `contract_ok=1` in the
  default, SIMD-enabled, and CUDA-enabled build trees. These rows cover the
  standard 48-row, boxed 24-row, and friction-index 8-/16-contact benchmark
  fixtures at relaxation 0.5, 1.0, and 1.3. The rows report
  `pgs_relaxation_sweep=1`, `psor_relaxation`,
  `psor_under_relaxation`, `psor_plain_pgs`, `psor_over_relaxation`,
  `problem_size=24/48`, `contact_count=8/16` for the friction-index rows, and
  backend build-state counters. The CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU PGS solver rows in a CUDA-enabled build,
  not CUDA LCP kernel execution.
- Verified symmetric PSOR relaxation sweep benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSymmetricPsorRelaxationSweep' | wc -l`
  reported 12 rows, and JSON checks for
  `BM_LcpSymmetricPsorRelaxationSweep` reported 12 rows with `contract_ok=1` in
  the default, SIMD-enabled, and CUDA-enabled build trees. These rows cover the
  standard 48-row, boxed 24-row, and friction-index 8-/16-contact benchmark
  fixtures at relaxation 0.5, 1.0, and 1.3. The rows report
  `symmetric_psor_relaxation_sweep=1`, `psor_relaxation`,
  `psor_under_relaxation`, `psor_plain_symmetric_psor`,
  `psor_over_relaxation`, `problem_size=24/48`, `contact_count=8/16` for the
  friction-index rows, and backend build-state counters. The CUDA-enabled rows
  report `build_cuda_enabled=1` but are CPU symmetric PSOR solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified Red-Black Gauss-Seidel relaxation sweep benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpRedBlackGaussSeidelRelaxationSweep' | wc -l`
  reported 12 rows, and JSON checks for
  `BM_LcpRedBlackGaussSeidelRelaxationSweep` reported 12 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover the standard 48-row, boxed 24-row, and friction-index
  8-/16-contact benchmark fixtures at relaxation 0.5, 1.0, and 1.3. The rows
  report `red_black_gauss_seidel_relaxation_sweep=1`,
  `red_black_color_count=2`, `red_black_red_rows=12/24`,
  `red_black_black_rows=12/24`, `psor_relaxation`,
  `psor_under_relaxation`, `psor_plain_red_black_gauss_seidel`,
  `psor_over_relaxation`, `problem_size=24/48`, `contact_count=8/16` for the
  friction-index rows, and backend build-state counters. The CUDA-enabled rows
  report `build_cuda_enabled=1` but are CPU Red-Black Gauss-Seidel solver rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
  The combined current relaxation-sweep filter for PGS, symmetric PSOR, and
  Red-Black Gauss-Seidel reports `rows=36`, `failures=0`, 12 rows per solver,
  and 9 `contact_count=16` rows in default, SIMD-enabled, and CUDA-enabled
  build trees; the SIMD and CUDA-enabled runs report `simd_rows=36` and
  `cuda_rows=36`, respectively.
- Verified Red-Black Gauss-Seidel solver-internal threading slice:
  `UNIT_math_lcp_math_lcp_lcp_projection_solvers --gtest_filter='RedBlackGaussSeidelSolver.InvalidWorkerThreadCount:RedBlackGaussSeidelSolver.ThreadedPathMatchesSerial' --gtest_brief=1`
  passed 2 focused tests, and
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.ThreadedRedBlackGaussSeidelStandardKnownSolution' --gtest_brief=1`
  passed the 4-worker 128-row generated known-solution test. Focused
  `BM_LcpRedBlackGaussSeidelSolverThreadingBanded_Standard` rows passed in
  default, SIMD-enabled, and CUDA-enabled build trees for 128-row serial/4-worker
  and 512-/1024-/2048-row serial/4-/8-worker banded packets plus
  4096-/8192-row serial/32-worker banded packets with
  `contract_ok=1`,
  `red_black_color_count=2`, `band_half_width=2`,
  `solver_internal_threads=1/4/8/32`, and
  `red_black_threaded_color_updates=0/1`. This proves the opt-in CPU threaded
  color-update path over larger banded packets, not a solver speedup or
  CUDA-kernel claim.
  The focused 4096-/8192-row follow-up reported 8 rows per build tree with
  `problem_size=4096/8192`, `matrix_nonzero_entries=20474/40954`,
  `matrix_density=0.001220345/0.000610262`, `solver_internal_threads=1/32`,
  residual/complementarity about `4.82e-4/4.72e-4`, `contract_ok=1`, and the
  expected backend build-state counters.
- Verified APGD restart-policy benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpApgdRestartSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpApgdRestartSweep` reported 12
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-/16-contact fixtures for adaptive restart every iteration,
  adaptive restart every 5 iterations, and no restart. The rows report
  `apgd_restart_sweep=1`, `apgd_adaptive_restart`,
  `apgd_restart_check_interval`, `apgd_relaxation=1`, `problem_size=24/48`,
  `contact_count=8/16` for the friction-index rows, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU
  APGD solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified TGS iteration-budget benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpTgsIterationBudgetSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpTgsIterationBudgetSweep`
  reported 12 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-/16-contact fixtures at 10-, 50-, and 100-iteration budgets.
  The rows report `tgs_iteration_budget_sweep=1`, `tgs_max_iterations`,
  `tgs_relaxation=1`, `problem_size=24/48`, `contact_count=8/16` for the
  friction-index rows, observed `iterations=5/6`, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU TGS
  solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified NNCG PGS-preconditioner iteration benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNncgPgsIterationsSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpNncgPgsIterationsSweep`
  reported 12 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-/16-contact fixtures at 1, 2, and 5 PGS preconditioner
  iterations while holding NNCG restart interval 10 and restart threshold 1.0.
  The rows report `nncg_pgs_iterations_sweep=1`, `nncg_pgs_iterations`,
  `nncg_restart_interval=10`, `nncg_restart_threshold=1`, `problem_size=24/48`,
  `contact_count=8/16` for the friction-index rows, observed outer
  `iterations=0/2/4/5`, and backend build-state counters. The CUDA-enabled
  rows report `build_cuda_enabled=1` but are CPU NNCG solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified SubspaceMinimization PGS-iteration benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSubspaceMinimizationPgsIterationsSweep' | wc -l`
  reported 12 rows, and JSON checks for
  `BM_LcpSubspaceMinimizationPgsIterationsSweep` reported 12 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover standard 48-row, boxed 24-row, and friction-index 8-/16-contact
  fixtures at 1, 3, and 5 PGS active-set-estimation iterations while holding
  active-set tolerance 0.0. The rows report
  `subspace_pgs_iterations_sweep=1`, `subspace_pgs_iterations`,
  `subspace_active_set_tolerance=0`, `problem_size=24/48`, `contact_count=8/16`
  for the friction-index rows, observed outer `iterations=1/2`, and backend
  build-state counters. The CUDA-enabled rows report `build_cuda_enabled=1`
  but are CPU SubspaceMinimization solver rows in a CUDA-enabled build, not
  CUDA LCP kernel execution.
  The combined current projection-sweep filter for APGD, TGS, NNCG, and
  SubspaceMinimization reports `rows=48`, `failures=0`, 12 rows per solver, and
  12 `contact_count=16` rows in default, SIMD-enabled, and CUDA-enabled build
  trees; the SIMD and CUDA-enabled runs report `simd_rows=48` and
  `cuda_rows=48`, respectively.
- Verified ShockPropagation layer-layout benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpShockPropagationLayerSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpShockPropagationLayerSweep`
  reported 12 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-/16-contact fixtures with single-layer, two-layer, and
  serial layer schedules built from 3-row blocks. The rows report
  `shock_propagation_layer_sweep=1`, schedule counters, `layer_count=1/2/8/16`,
  `block_count=8/16`, `max_block_size=3`, `max_blocks_per_layer=1/4/8/16`,
  `problem_size=24/48`, `contact_count=8/16` for the friction-index rows,
  observed solver `iterations=3/4`, and backend build-state counters. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU ShockPropagation
  solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified MPRGP SPD/positive-definite-check benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMprgpSpdCheckSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpMprgpSpdCheckSweep` reported 12
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build
  trees. These rows cover dense SPD 32/64/128-row, banded SPD 64/128-row,
  mildly ill-conditioned SPD 32-row, and near-singular SPD 8/16-row
  standard-LCP fixtures with `MprgpSolver::Parameters::checkPositiveDefinite`
  enabled and disabled where paired. The rows report
  `mprgp_spd_check_sweep=1`, `mprgp_positive_definite_check`, SPD-kind
  counters, `problem_size=8/16/32/64/128`, observed solver `iterations=3/4/5/15`,
  `mprgp_symmetry_tolerance=1e-9`, `mprgp_epsilon_for_division=1e-12`, and
  backend build-state counters. The CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU MPRGP solver rows in a CUDA-enabled build,
  not CUDA LCP kernel execution.
- Verified Interior Point path-parameter benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpInteriorPointPathSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpInteriorPointPathSweep` reported
  12 rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover dense SPD 32/64/128-row, banded SPD 64/128-row,
  mildly ill-conditioned SPD 32-row, and near-singular SPD 8/16-row
  standard-LCP fixtures with centering parameter `sigma=0.1/0.3` and step scale
  `0.75/0.99`. The rows report `interior_point_path_sweep=1`,
  `interior_point_sigma`, `interior_point_step_scale`, SPD-kind counters,
  `problem_size=8/16/32/64/128`, observed solver
  `iterations=14/16/27/31/32/33/41/51/53/58`, and backend build-state counters.
  The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU Interior Point
  solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
  The combined current MPRGP/Interior Point sweep filter reports `rows=24`,
  `failures=0`, 12 rows per solver, and 6 large or 16-row near-singular rows in
  default, SIMD-enabled, and CUDA-enabled build trees; the SIMD and CUDA-enabled
  runs report `simd_rows=24` and `cuda_rows=24`, respectively.
- Verified Staggering contact-pipeline benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpStaggeringContactPipelineSweep' | wc -l`
  reported 14 rows, and JSON checks for
  `BM_LcpStaggeringContactPipelineSweep` reported 14 rows with `contract_ok=1`
  in the default, SIMD-enabled, and CUDA-enabled build trees. These rows cover
  separated sphere-ground 1/2/4/8-contact fixtures, coupled vertical-stack
  2/3/5/8-contact fixtures, and articulated unified ground, rigid-impact, and
  cross-link-impact 4-/8-contact fixtures. The rows report
  `staggering_contact_pipeline_sweep=1`,
  `staggering_normal_friction_split=1`, normal-row counts `1/2/3/4/5/8`,
  friction-row counts `2/4/6/8/10/16`, coupled-contact flags, contact counts
  `1/2/3/4/5/8`, and backend build-state counters. The CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU Staggering solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution. The current focused filter reports
  `rows=14`, `failures=0`, 5 `contact_count=8` rows, and `simd_rows=14` or
  `cuda_rows=14` in the backend build-tree runs.
- Verified Boxed Semi-Smooth Newton line-search benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpBoxedSemiSmoothNewtonLineSearchSweep' | wc -l`
  reported 12 rows, and JSON checks for
  `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep` reported 12 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover standard 48-row, boxed 24-row, and friction-index
  8-/16-contact fixtures with default line search, an expanded line-search step
  budget, and a gentler step-reduction policy. The rows report
  `boxed_ssn_line_search_sweep=1`,
  `boxed_ssn_max_line_search_steps=10/20`,
  `boxed_ssn_step_reduction=0.5/0.8`, default/more-step/gentle-reduction
  policy counters, observed solver `iterations=2/7/8/9/15/16`,
  `contact_count=8/16` for the friction-index rows, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU
  BoxedSemiSmoothNewton solver rows in a CUDA-enabled build, not CUDA LCP kernel
  execution.
  The combined current ShockPropagation/BoxedSemiSmoothNewton sweep filter
  reports `rows=24`, `failures=0`, 12 rows per solver, and 6
  `contact_count=16` rows in default, SIMD-enabled, and CUDA-enabled build
  trees; the SIMD and CUDA-enabled runs report `simd_rows=24` and
  `cuda_rows=24`, respectively.
- Verified ADMM rho/adaptive-rho benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpAdmmRhoSweep' | wc -l`
  reported 24 rows, and JSON checks for `BM_LcpAdmmRhoSweep` reported 24 rows
  with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build
  trees. These rows cover standard 48-row, boxed 24-row, and friction-index
  8-/16-contact fixtures at `rhoInit` 0.5, 1.0, and 4.0 with fixed and adaptive
  rho policies. The rows report `admm_rho_sweep=1`, `admm_rho_init`,
  `admm_fixed_rho`, `admm_adaptive_rho`,
  `admm_adaptive_rho_tolerance`, `admm_mu_prox`, `problem_size=24/48`,
  `contact_count=8/16` for the friction-index rows, and backend build-state
  counters. The CUDA-enabled rows report `build_cuda_enabled=1` but are CPU
  ADMM solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified SAP regularization benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSapRegularizationSweep' | wc -l`
  reported 12 rows, and JSON checks for `BM_LcpSapRegularizationSweep` reported
  12 rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover standard 48-row, boxed 24-row, and
  friction-index 8-/16-contact fixtures at regularization values `1e-6`,
  `1e-5`, and `1e-4`. The rows report `sap_regularization_sweep=1`,
  `sap_regularization`, `sap_armijo_parameter`, `sap_backtracking_factor`,
  `sap_max_line_search_iterations`, `problem_size=24/48`, `contact_count=8/16`
  for the friction-index rows, and backend build-state counters. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU SAP solver rows
  in a CUDA-enabled build, not CUDA LCP kernel execution.
  The combined current ADMM/SAP sweep filter reports `rows=36`, `failures=0`,
  24 ADMM rows, 12 SAP rows, and 9 `contact_count=16` rows in default,
  SIMD-enabled, and CUDA-enabled build trees; the SIMD and CUDA-enabled runs
  report `simd_rows=36` and `cuda_rows=36`, respectively.
- Verified scoped-solver extreme-coupling slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditioned/ExtremeCoupledFrictionIndex' | wc -l`
  now reports 180 single-problem rows, and focused checks for
  `BM_LcpMildIllConditioned(BatchSerial|BatchParallel)?/ExtremeCoupledFrictionIndex(6|8|12|16|24|32|48|64|96)`
  reported 405 single and batch rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  The single-problem rows cover `Pgs`, `SymmetricPsor`, `Jacobi`,
  `RedBlackGaussSeidel`, `BlockedJacobi`, `BGS`, `NNCG`,
  `SubspaceMinimization`, `Apgd`, `Tgs`, `ShockPropagation`, `Staggering`,
  `Admm`, `Sap`, and `BoxedSemiSmoothNewton`. These rows cover 16x-coupled
  mildly ill-conditioned friction-index packets at 6, 8, 12, 16, 24, 32, 48,
  64, 96, 128, 192, and 256 contacts. The historical all-build 405-row focused
  gate covers the 6- through 96-contact single/batch rows; focused 128-, 192-,
  and 256-contact follow-up evidence is listed in the larger mildly
  ill-conditioned benchmark section below. The
  rows report `mildly_ill_conditioned=1`, `coupled=1`,
  `coupling_scale=16`,
  `contact_count=6/8/12/16/24/32/48/64/96/128/192/256`,
  `problem_size=18/24/36/48/72/96/144/192/288/384/576/768`,
  `total_contact_count=24/32/48/64/96/128/192/256/384/512/768/1024`,
  `total_problem_size=72/96/144/192/288/384/576/768/1152/1536/2304/3072`,
  and backend build-state counters for the batch rows. The Boxed Semi-Smooth
  Newton 16x rows report the tuned line-search counters.
  Focused
  `LcpGeneratedCoverage.LargerMildlyIllConditionedKnownSolutionsForScopedSolvers`
  unit test also passes in the default, SIMD-enabled, and CUDA-enabled build
  trees after narrowing this exact-solution gate to solvers that reproduce the
  selected generated solution; projection-like solvers that satisfy the LCP
  contract with alternate solutions remain covered by benchmark contract rows.
  The exact-solution 16x coupled cases reach 192 contacts. The
  CUDA-enabled rows report
  `build_cuda_enabled=1` but are CPU solver rows in a CUDA-enabled build, not
  CUDA LCP kernel execution.
- Verified ADMM/SAP/Boxed Semi-Smooth Newton contact comparison benchmark
  slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpContactSolverComparisonSweep' | wc -l`
  reported 42 rows, and JSON checks for
  `BM_LcpContactSolverComparisonSweep` reported 42 rows with `contract_ok=1`
  in the default, SIMD-enabled, and CUDA-enabled build trees. These rows run
  `Admm`, `Sap`, and `BoxedSemiSmoothNewton` over the same DART 7 separated
  sphere-ground 1/2/4/8-contact fixtures, coupled vertical-stack
  2/3/5/8-contact fixtures, and articulated unified ground, rigid-impact, and
  cross-link-impact 4-/8-contact fixtures. The list-test evidence includes
  15 `contact_count=8` rows. The rows report
  `contact_solver_comparison_sweep=1`, solver identity counters,
  fixture-family counters, `contact_count=1/2/3/4/5/8`,
  `normal_row_count=1/2/3/4/5/8`, `friction_row_count=2/4/6/8/10/16`,
  `problem_size=3/6/9/12/15/24`, and backend build-state counters. The
  CUDA-enabled rows report `build_cuda_enabled=1` but are CPU solver rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Verified contact-normal standard-LCP benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpContactNormalStandardSweep' | wc -l`
  reported 116 rows, and JSON checks for
  `BM_LcpContactNormalStandardSweep` reported 116 rows with `contract_ok=1` in
  the default, SIMD-enabled, and CUDA-enabled build trees. These rows extract
  only the normal rows from the same DART 7 separated sphere-ground
  1/2/4/8-contact fixtures, coupled vertical-stack 2/3/5/8-contact fixtures,
  and articulated unified ground, rigid-impact, and cross-link-impact
  4-/8-contact fixtures, then run them as standard LCPs. The list-test
  evidence includes 40 rows sourced from the five 8-contact fixtures. The rows
  compare `Dantzig`, `Lemke`, `Baraff`, `Direct`, `MinimumMapNewton`,
  `FischerBurmeisterNewton`,
  `PenalizedFischerBurmeisterNewton`, `InteriorPoint`, and `MPRGP`; `Direct`
  is intentionally limited to four 1-, 2-, and 3-row contact-normal rows so the
  benchmark measures its enumeration path rather than Dantzig fallback. The
  rows report `contact_normal_standard_sweep=1`, `normal_row_count=1/2/3/4/5/8`,
  `source_problem_size=3/6/9/12/15/24`, `problem_size=1/2/3/4/5/8`,
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
  reports 128 rows. Previous full-slice runs through the 96-contact packet
  passed in default, SIMD-enabled, and CUDA-enabled build trees; focused
  follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpProductionActiveSetTransition/CoupledFrictionIndex128/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in those same three build trees each reported 16 rows with
  `contract_ok=1`,
  `active_set_transition=1`, `production_active_set_transition=1`,
  `contact_count=128`, `problem_size=384`, `coupling_scale=32`, and
  `coupled=1`. Focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpProductionActiveSetTransition/CoupledFrictionIndex192/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in the default, SIMD-enabled, and CUDA-enabled build trees report the
  16 added rows with `contract_ok=1`, `contact_count=192`,
  `problem_size=576`, `coupling_scale=32`, and `coupled=1`. Focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpProductionActiveSetTransition/CoupledFrictionIndex256/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in those same three build trees report another 16 rows with
  `contract_ok=1`, zero `failures` and `contract_failures`,
  `active_set_transition=1`, `production_active_set_transition=1`,
  `contact_count=256`, `problem_size=768`, `coupling_scale=32`, and
  `coupled=1`; the SIMD-enabled rows report `build_simd_enabled=1`, and the
  CUDA-enabled rows report `build_cuda_enabled=1`. The previous
  full-slice
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpProductionActiveSetTransition(BatchSerial|BatchParallel)?/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  ran 534 production single and batch rows with `contract_ok=1` in the default,
  SIMD-enabled, and CUDA-enabled build trees. These rows now cover
  stronger-coupled 24-contact, 72-row, 32-contact, 96-row, 48-contact,
  144-row, 64-contact, 192-row, 96-contact, 288-row, 128-contact, 384-row, and
  192-contact, 576-row and 256-contact, 768-row friction-index active-set
  transition packets over all 16 friction-index-capable manifest solvers. The
  rows report
  `active_set_transition=1`, `production_active_set_transition=1`, backend
  build-state counters, `contact_count=24/32/48/64/96/128/192/256`,
  `problem_size=72/96/144/192/288/384/576/768`,
  `coupling_scale=2/4/8/16/32`, and `coupled=1`. The CUDA-enabled rows are CPU
  solver rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified production active-set transition generated coverage slice:
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.ProductionActiveSetTransitionFrictionIndexKnownSolutionsForScalableSolvers' --gtest_brief=1`
  passes in the default, SIMD-enabled, and CUDA-enabled build trees after
  adding the 256-contact, 768-row, `coupling_scale=32` packet. The previous
  full default generated coverage suite
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_brief=1` also passes
  21 tests through the 128-contact expanded production active-set packet, and
  earlier focused production active-set runs covered the 192-contact packet in
  the same three build trees. The CUDA-enabled run is CPU generated solver
  coverage in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified production active-set transition batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpProductionActiveSetTransitionBatch' | wc -l`
  reports 550 rows. Previous JSON benchmark checks through the 96-contact packet
  reported 454 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees. Focused follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)/CoupledFrictionIndex128/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in those same three build trees each reported 32 rows with
  `contract_ok=1`, `production_active_set_transition_batch=1`,
  `contact_count=128`, `total_contact_count=512`, `problem_size=384`,
  `total_problem_size=1536`, `batch_size=4`, `coupling_scale=32`, and
  `parallel_units=4` on parallel rows. Focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)/CoupledFrictionIndex192/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in those same three build trees each report the 32 added rows with
  `contract_ok=1`, `contact_count=192`, `total_contact_count=768`,
  `problem_size=576`, `total_problem_size=2304`, `batch_size=4`,
  `coupling_scale=32`, and `parallel_units=4` on parallel rows. Focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)/CoupledFrictionIndex256/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in those same three build trees each report another 32 rows with
  `contract_ok=1`, zero `failures` and `contract_failures`,
  `production_active_set_transition_batch=1`, `contact_count=256`,
  `total_contact_count=1024`, `problem_size=768`,
  `total_problem_size=3072`, `batch_size=4`, `coupling_scale=32`,
  16 serial rows, 16 parallel rows, and `parallel_units=4` on parallel rows.
  These rows cover batch-size-4 serial and DART 7 `ParallelExecutor` runs over standard
  32/64/128-row, boxed 32/64/128-row, and coupled friction-index
  8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact active-set packets.
  The rows report `production_active_set_transition_batch=1`,
  `batch_size=4`, `problem_size=24/32/36/48/64/72/96/128/144/192/288/384/576/768`,
  `total_problem_size=96/128/144/192/256/288/384/512/576/768/1152/1536/2304/3072`, and backend
  build-state counters. Friction-index rows also report
  `contact_count=8/12/16/24/32/48/64/96/128/192/256`,
  `total_contact_count=32/48/64/96/128/192/256/384/512/768/1024`,
  and `coupling_scale=1/2/4/8/16/32`. Parallel rows also report `profile_enabled=1`,
  `parallel_units=4`, `worker_count=20`, and observed `max_parallelism`. The
  CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
- Verified larger mildly ill-conditioned benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditioned/' | wc -l`
  reports 629 rows. The combined single/batch check
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpMildIllConditioned(BatchSerial|BatchParallel)?/' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  previously ran the pre-256 1752 rows with `contract_ok=1` in the default,
  SIMD-enabled, and
  CUDA-enabled build trees. The rows cover standard 32-row, boxed 16-row,
  friction-index 8-contact, and coupled friction-index 6-, 8-, 12-, 16-, 24-,
  32-, 48-, 64-, and 96-contact packets, plus 4x- and 8x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact packets and 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-contact packets over the scoped solver
  set. A focused default/SIMD-enabled/CUDA-enabled smoke also passes the
  192-contact single-problem row family with `contract_ok=1`,
  `contact_count=192`, `problem_size=576`, and `coupling_scale=16`; the
  SIMD-enabled SAP row is contract-correct but slow and reports 20k iterations.
  Focused 256-contact single-problem follow-ups in default, SIMD-enabled, and
  CUDA-enabled build trees each report 15 rows with `contract_ok=1`,
  `contact_count=256`, `problem_size=768`, and `coupling_scale=16`. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
  The friction-index rows
  report `contact_count`, the coupled rows report `coupled=1`, the
  stronger/extreme-coupled rows report `coupling_scale=4`, `8`, or `16`, and
  all rows report `mildly_ill_conditioned=1` plus backend build-state
  counters. The `BoxedSemiSmoothNewton` coupled single-problem rows span
  coupling scales `1/4/8/16`; the 16x scale now includes the 128-contact,
  192-contact, and 256-contact packets and reports the same tuned line-search
  counters as the batch rows.
- Verified larger mildly ill-conditioned batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpMildIllConditionedBatch' | wc -l`
  reports 1258 rows, and JSON benchmark checks for
  `BM_LcpMildIllConditionedBatch(Serial|Parallel)` previously reported 1168 rows with
  `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled build trees.
  These rows cover batch-size-4 serial and DART 7 `ParallelExecutor` runs over
  the full scoped mildly ill-conditioned packet set: standard 32-row, boxed
  16-row, friction-index 8-contact, coupled friction-index
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 4x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 8x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact, and 15-solver 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact rows. The focused default
  check for `ExtremeCoupledFrictionIndex192` single, serial-batch, and
  parallel-batch rows reports 45 rows with `contract_ok=1`,
  `total_contact_count=768`, `total_problem_size=2304`, and `parallel_units=4`
  on parallel rows. Focused SIMD-enabled and CUDA-enabled all-solver follow-ups
  for serial/parallel `ExtremeCoupledFrictionIndex192/4` rows each report
  30 rows with `contract_ok=1`, covering `Pgs`, `SymmetricPsor`, `Jacobi`,
  `RedBlackGaussSeidel`, `BlockedJacobi`, `BGS`, `NNCG`,
  `SubspaceMinimization`, `Apgd`, `Tgs`, `ShockPropagation`, `Staggering`,
  `Admm`, `Sap`, and `BoxedSemiSmoothNewton`, with `contact_count=192`,
  `total_contact_count=768`, `problem_size=576`,
  `total_problem_size=2304`, `coupling_scale=16`, and `batch_size=4`; the
  SIMD run reports `build_simd_enabled=1`, the CUDA-enabled run reports
  `build_cuda_enabled=1`, and parallel rows report `max_parallelism=4`.
  Focused default 256-contact single, serial-batch, and parallel-batch rows
  report 45 rows with `contract_ok=1`, `contact_count=256`,
  `total_contact_count=1024`, `problem_size=768`,
  `total_problem_size=3072`, `coupling_scale=16`, `batch_size=4`,
  `parallel_units=4`, and `worker_count=20`. Focused SIMD/CUDA-enabled
  selected-solver 256-contact serial/parallel batch gates now report 28 rows in
  each build with `contract_ok=1` for every registered solver except `Sap`; the
  CUDA-enabled rows are CPU solver rows in that build tree, not CUDA LCP kernel
  execution. The focused SIMD-enabled `Sap` 256-contact batch probe was stopped
  after 180s before producing a benchmark row, so all-solver SIMD/CUDA
  256-contact batch contract evidence is not claimed.
  These focused rows are contract gates only: several projection-like rows
  still report large residual counters, no speedup is claimed, and the prior
  SIMD-enabled `Sap` rows are slow enough that this is not a routine checkpoint
  gate. The rows report
  `mildly_ill_conditioned_batch=1`, `batch_size=4`, problem sizes
  `16/18/24/32/36/48/72/96/144/192/288/384/576/768`, total problem sizes
  `64/72/96/128/144/192/288/384/576/768/1152/1536/2304/3072`, and backend
  build-state counters. The friction-index rows additionally report
  `contact_count=6/8/12/16/24/32/48/64/96/128/192/256`
  and `total_contact_count=24/32/48/64/96/128/192/256/384/512/768/1024`;
  coupled rows report
  `coupled=1` and `coupling_scale=1/4/8/16`. Parallel rows also report
  `profile_enabled=1`, `parallel_units=4`, `worker_count=20`, and observed
  `max_parallelism`. The CUDA-enabled rows are CPU solver batch rows in a
  CUDA-enabled build, not CUDA LCP kernel execution. The
  `BoxedSemiSmoothNewton` coupled batch rows span coupling scales `1/4/8/16`;
  all four coupling scales include the 96-contact packet, the 16x rows now
  include the 256-contact packet, and the 16x rows report
  `boxed_ssn_max_line_search_steps=50`,
  `boxed_ssn_step_reduction=0.8`, and
  `boxed_ssn_jacobian_regularization=1e-8`.
- Verified near-singular batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpNearSingularBatch' | wc -l`
  reports 62 rows. Previous JSON benchmark checks through the 96-contact packet
  reported 50 rows with `contract_ok=1` in the default, SIMD-enabled, and
  CUDA-enabled build trees; focused 128-contact follow-up runs in those same
  build trees report 2 `BM_LcpNearSingular` rows and 4
  `BM_LcpNearSingularBatch(Serial|Parallel)` rows with `contract_ok=1`, and
  focused 192-contact follow-up runs in those same build trees report another 2
  `BM_LcpNearSingular` rows and 4 batch rows with `contract_ok=1`.
  Focused 256-contact follow-up runs in those same build trees report another
  2 `BM_LcpNearSingular` rows and 4 batch rows with `contract_ok=1`, zero
  `failures` and `contract_failures`, `near_singular=1`, `contact_count=256`,
  `problem_size=768`, `total_contact_count=1024`, `total_problem_size=3072`,
  2 single rows, 2 serial-batch rows, 2 parallel-batch rows, and
  `parallel_units=4` on parallel rows.
  These rows cover batch-size-4 serial and DART 7 `ParallelExecutor` runs over
  near-singular standard 8-row, boxed 8-row, and coupled friction-index 3-, 6-,
  9-, 12-, 16-, 24-, 32-, 48-, 64-, 96-, 128-, 192-, and 256-contact packets. The rows report
  `near_singular_batch=1`, `batch_size=4`, problem sizes
  `8/9/18/27/36/48/72/96/144/192/288/384/576/768`, total problem sizes
  `32/36/72/108/144/192/288/384/576/768/1152/1536/2304/3072`, and backend build-state counters. The
  friction-index rows additionally report
  `contact_count=3/6/9/12/16/24/32/48/64/96/128/192/256`,
  `total_contact_count=12/24/36/48/64/96/128/192/256/384/512/768/1024`,
  and `coupled=1`. Parallel rows also report `profile_enabled=1`,
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
  now reports 51 rows. Previous full-slice runs through the 96-contact packet
  passed with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. Focused 128-contact follow-up runs in those same build trees
  report 3 added rows with `contract_ok=1`; focused 192-contact follow-up runs
  in those same build trees report another 3 added rows with `contract_ok=1`.
  Focused 256-contact follow-up runs in those same build trees report another
  3 added rows with `contract_ok=1`, `contact_count=256`, and
  `problem_size=768`.
  The matching generated-coverage filter
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter=LcpGeneratedCoverage.ExtremeSingularDegenerateKnownSolutionsForRobustSolverSlice`
  passed in the default, SIMD-enabled, and CUDA-enabled build trees after
  adding the 256-contact known-solution packet.
  These rows cover exact
  rank-deficient standard 128-row, boxed 128-row, and coupled friction-index
  16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets over the same scoped robust
  solver set as the generated extreme singular-degenerate correctness slice,
  which now includes the same 256-contact packet.
  The rows report `singular_degenerate=1`, `rank_deficient=1`, backend
  build-state counters, contact counts `16/24/32/48/64/96/128/192/256`, problem
  sizes `48/72/96/144/192/288/384/576/768`, and `coupled=1` for the coupled
  packets. The
  CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
  kernel execution.
- Verified singular-degenerate friction-index batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSingularDegenerateFrictionIndexBatch' | wc -l`
  now reports 72 rows. Previous JSON benchmark checks through the 96-contact
  packet reported 54 rows with `contract_ok=1` in the default, SIMD-enabled,
  and CUDA-enabled build trees. Focused 128-contact follow-up runs in those
  same build trees report 6 added batch rows with `contract_ok=1`; focused
  192-contact follow-up runs in those same build trees report another 6 added
  batch rows with `contract_ok=1`. Focused 256-contact follow-up runs in those
  same build trees report another 6 added batch rows with `contract_ok=1`,
  `contact_count=256`, `total_contact_count=1024`, `problem_size=768`, and
  `total_problem_size=3072`. These
  rows cover batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
  rank-deficient coupled friction-index 6-, 8-, 12-, 16-, 24-, 32-, 48-, 64-,
  96-, 128-, 192-, and 256-contact packets. The rows report `singular_degenerate_batch=1`,
  `rank_deficient=1`, `batch_size=4`,
  `contact_count=6/8/12/16/24/32/48/64/96/128/192/256`,
  `total_contact_count=24/32/48/64/96/128/192/256/384/512/768/1024`,
  `problem_size=18/24/36/48/72/96/144/192/288/384/576/768`,
  `total_problem_size=72/96/144/192/288/384/576/768/1152/1536/2304/3072`, and backend
  build-state counters. Parallel rows also report `profile_enabled=1`,
  `parallel_units=4`, `worker_count=20`, and observed `max_parallelism`. The
  CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
- Verified singular-degenerate standard/boxed batch benchmark slice:
  `BM_LCP_COMPARE --benchmark_list_tests | rg '^BM_LcpSingularDegenerateStandardBoxedBatch' | wc -l`
  reported 192 rows, and JSON benchmark checks for
  `BM_LcpSingularDegenerateStandardBoxedBatch(Serial|Parallel)` reported 192
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover batch-size-4 serial and DART 7
  `ParallelExecutor` runs over exact rank-deficient standard and boxed
  16-/32-/64-/128-row packets. The rows report
  `singular_degenerate_batch=1`,
  `singular_degenerate_standard_boxed_batch=1`, `rank_deficient=1`,
  `batch_size=4`, `problem_size=16/32/64/128`,
  `total_problem_size=64/128/256/512`, and backend build-state counters.
  Parallel rows also report `profile_enabled=1`, `parallel_units=4`,
  `worker_count=20`, and observed `max_parallelism`. The CUDA-enabled rows are
  CPU solver batch rows in a CUDA-enabled build, not CUDA LCP kernel execution.
- Verified solver-internal Jacobi threading slice:
  `JacobiSolver::Parameters::workerThreads` now enables an opt-in CPU threaded
  update path. `LcpGeneratedCoverage.ThreadedJacobiStandardKnownSolution`
  passed on a 128-row generated standard LCP. `BM_LCP_COMPARE` now lists 21
  Jacobi threading rows: 4 dense `BM_LcpJacobiSolverThreading_Standard` rows
  for 128-row and 512-row standard problems with 1 and 8 worker threads, plus
  17 banded `BM_LcpJacobiSolverThreadingBanded_Standard` rows for 512-row
  standard problems with 1, 4, and 8 worker threads, 1024-row standard problems
  with 1, 4, 8, and 16 worker threads, and 2048-row standard problems with 1,
  8, 16, and 32 worker threads, plus 4096-row standard problems with 1, 8,
  16, and 32 worker threads, plus 8192-row standard problems with 1 and 32
  worker threads. The focused
  default, SIMD-enabled, and CUDA-enabled benchmark runs all reported
  `contract_ok=1`, `solver_internal_threads`, `worker_count`,
  `jacobi_threading_banded_spd`, `band_half_width`, `matrix_nonzero_entries`,
  `matrix_density`, and backend build-state counters. The banded rows use
  sparse-structured matrices in dense storage and report densities of about
  0.00974 for 512 rows, 0.00488 for 1024 rows, 0.00244 for 2048 rows, and
  0.00122 for 4096 rows, and 0.000610 for 8192 rows. The focused default,
  SIMD-enabled, and CUDA-enabled checks for the new 8192-row rows passed with
  `contract_ok=1`, `solver_internal_threads=1/32`,
  `matrix_nonzero_entries=40954`, and `matrix_density=0.000610262`. The
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
  `BoxedLcpContact.LargerStressSphereStackWorldContactSnapshotSatisfiesLcpContract`
  test extends the same real DART 7 boxed/findex snapshot path to a 6-sphere
  stack with 6 contacts and 18 LCP rows, and
  `BoxedLcpContact.LargerStressSphereStackWorldStepMaintainsContactInvariants`
  advances that taller coupled stack through `World::step(1000)` while checking
  the same finite-state, spacing, near-rest, lateral-drift, and static-ground
  invariants. Focused default, SIMD-enabled, and CUDA-enabled build-tree runs
  passed both new 6-sphere tests.
  `BoxedLcpContact.SevenSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extends the same real DART 7 boxed/findex snapshot path to a 7-sphere stack
  with 7 contacts and 21 LCP rows. The focused default
  `test_boxed_lcp_contact --gtest_filter=BoxedLcpContact.SevenSphereStackWorldContactSnapshotSatisfiesLcpContract:BoxedLcpContact.ThirtyTwoBoxWorldStepMaintainsDenseContactInvariants --gtest_brief=1`
  run passed both tests. This is 7-sphere snapshot evidence only; a 7-sphere
  public-step invariant is not claimed. A temporary
  `SevenSphereStackWorldStepMaintainsContactInvariants` probe failed the
  existing near-rest vertical-velocity invariant after 1000 public
  `World::step()` iterations, with a matching
  `BM_LcpWorldStackStep_BoxedLcp/7/1000` probe reporting `invariant_ok=0`,
  `min_spacing=0.999225`, and `max_vertical_speed=5.90648`. A follow-up
  2000-step probe still failed height, spacing, and near-rest checks, while
  `BM_LcpWorldStackStep_BoxedLcp/7/2000` reported `invariant_ok=0` and
  `max_vertical_speed=4.599`. No 7-sphere public-step row is registered from
  those probes.
  `BoxedLcpContact.EightSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extends the direct boxed/findex snapshot path to an 8-sphere stack with
  8 contacts and 24 LCP rows. This is snapshot evidence only; no 8-sphere
  public-step invariant is claimed.
  `BoxedLcpContact.NineSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extends the same direct snapshot path to a 9-sphere stack with 9 contacts and
  27 LCP rows. This is snapshot evidence only; no 9-sphere public-step
  invariant is claimed.
  `BoxedLcpContact.TenSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extends the same direct snapshot path to a 10-sphere stack with 10 contacts
  and 30 LCP rows. This is snapshot evidence only; no 10-sphere public-step
  invariant is claimed.
  `BoxedLcpContact.ElevenSphereStackWorldContactSnapshotSatisfiesLcpContract`
  and
  `BoxedLcpContact.TwelveSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extend the same direct snapshot path to 11- and 12-sphere stacks with
  11/12 contacts and 33/36 LCP rows. This is snapshot evidence only; no
  11-/12-sphere public-step invariant is claimed. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ElevenSphereStackWorldContactSnapshotSatisfiesLcpContract:BoxedLcpContact.TwelveSphereStackWorldContactSnapshotSatisfiesLcpContract' --gtest_brief=1`
  run passed both tests in the default build.
  `BoxedLcpContact.ThirteenSphereStackWorldContactSnapshotSatisfiesLcpContract`,
  `BoxedLcpContact.FourteenSphereStackWorldContactSnapshotSatisfiesLcpContract`,
  `BoxedLcpContact.FifteenSphereStackWorldContactSnapshotSatisfiesLcpContract`,
  and
  `BoxedLcpContact.SixteenSphereStackWorldContactSnapshotSatisfiesLcpContract`
  extend the same direct snapshot path to 13-, 14-, 15-, and 16-sphere stacks
  with 13/14/15/16 contacts and 39/42/45/48 LCP rows. This is snapshot
  evidence only; no 13-/14-/15-/16-sphere public-step invariant is claimed.
  The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ThirteenSphereStackWorldContactSnapshotSatisfiesLcpContract:BoxedLcpContact.FourteenSphereStackWorldContactSnapshotSatisfiesLcpContract:BoxedLcpContact.FifteenSphereStackWorldContactSnapshotSatisfiesLcpContract:BoxedLcpContact.SixteenSphereStackWorldContactSnapshotSatisfiesLcpContract' --gtest_brief=1`
  run passed all four tests in the default build.
  The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*WorldContactSnapshot*'`
  run passed the single-contact and two-contact sphere-ground snapshot tests,
  the focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.*SphereStack*'`
  run passed 13 stack tests, including the 3-/4-/5-/6-/7-/8-/9-/10-sphere
  snapshots and the 3-sphere 200-/500-step, 4-sphere 200-step, 5-sphere
  500-step, and 6-sphere 1000-step public-step invariants,
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
  `BoxedLcpContact.CartesianPrismaticChainGroundStepMaintainsInvariants`,
  `BoxedLcpContact.FourCartesianPrismaticChainsGroundStepMaintainsInvariants`,
  `BoxedLcpContact.EightCartesianPrismaticChainsGroundStepMaintainsInvariants`,
  and
  `BoxedLcpContact.SixteenCartesianPrismaticChainsGroundStepMaintainsInvariants`
  advance two, four, eight, and sixteen connected three-axis prismatic
  Cartesian chains in simultaneous tip-ground contact through `World::step(200)`
  with `ContactSolverMethod::BoxedLcp`, confirm all contacts touch links, and
  check finite state, 6, 12, 24, and 48 total generalized coordinates, bounded
  tip height error, bounded joint velocities, bounded planar joint speed, and
  parity with the sequential articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.CartesianPrismaticChainGroundStepMaintainsInvariants:BoxedLcpContact.FourCartesianPrismaticChainsGroundStepMaintainsInvariants:BoxedLcpContact.EightCartesianPrismaticChainsGroundStepMaintainsInvariants:BoxedLcpContact.SixteenCartesianPrismaticChainsGroundStepMaintainsInvariants' --gtest_brief=1`
  run passed all four tests. The focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldArticulatedCartesianGroundStep_BoxedLcp/16/200$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  row reported `invariant_ok=1`, `cartesian_chain_count=16`,
  `articulated_dof_count=48`, `contact_count=16`, and `step_count=200`.
  `BoxedLcpContact.ArticulatedPrismaticLinkPushesDynamicRigidBody` and
  `BoxedLcpContact.FourArticulatedPrismaticLinksPushDynamicRigidBodies` and
  `BoxedLcpContact.EightArticulatedPrismaticLinksPushDynamicRigidBodies` and
  `BoxedLcpContact.SixteenArticulatedPrismaticLinksPushDynamicRigidBodies`,
  plus
  `BoxedLcpContact.SixteenArticulatedPrismaticLinksPushDynamicRigidBodiesForManySteps`,
  advance one, four, eight, and sixteen fixed-base prismatic striker links in
  contact with dynamic rigid spheres through one and 200 boxed-LCP
  `World::step()` iterations, confirm all contacts touch both `comps::Link`
  entities and rigid bodies, and check finite velocities, target motion,
  striker slowdown, X-momentum conservation, and parity with the sequential
  articulated shortcut. The focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesDynamicRigidBody:BoxedLcpContact.FourArticulatedPrismaticLinksPushDynamicRigidBodies:BoxedLcpContact.EightArticulatedPrismaticLinksPushDynamicRigidBodies:BoxedLcpContact.SixteenArticulatedPrismaticLinksPushDynamicRigidBodies:BoxedLcpContact.SixteenArticulatedPrismaticLinksPushDynamicRigidBodiesForManySteps' --gtest_brief=1`
  run passed all five tests. The focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldArticulatedRigidImpactStep_BoxedLcp/16/(1|200)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  rows reported `invariant_ok=1`, `articulated_link_count=16`,
  `dynamic_rigid_body_count=16`, `contact_count=16`, and `step_count` values 1
  and 200. The full `test_boxed_lcp_contact --gtest_list_tests` inventory now
  lists 63 tests.
  `BoxedLcpContact.ArticulatedPrismaticLinkPushesArticulatedPrismaticLink` and
  `BoxedLcpContact.FourArticulatedPrismaticLinksPushArticulatedPrismaticLinks`
  and
  `BoxedLcpContact.EightArticulatedPrismaticLinksPushArticulatedPrismaticLinks`
  and
  `BoxedLcpContact.SixteenArticulatedPrismaticLinksPushArticulatedPrismaticLinks`,
  plus
  `BoxedLcpContact.SixteenArticulatedPrismaticLinksPushArticulatedPrismaticLinksForManySteps`,
  advance one, four, eight, and sixteen fixed-base prismatic striker links in
  contact with prismatic target links owned by separate multibodies through one
  and 200 boxed-LCP `World::step()` iterations, confirm all contacts touch two
  `comps::Link` entities, and check finite velocities, target motion, striker
  slowdown, nonnegative post-step separation velocity, X-momentum conservation,
  and parity with the sequential cross-multibody articulated shortcut. The
  focused
  `test_boxed_lcp_contact --gtest_filter='BoxedLcpContact.ArticulatedPrismaticLinkPushesArticulatedPrismaticLink:BoxedLcpContact.FourArticulatedPrismaticLinksPushArticulatedPrismaticLinks:BoxedLcpContact.EightArticulatedPrismaticLinksPushArticulatedPrismaticLinks:BoxedLcpContact.SixteenArticulatedPrismaticLinksPushArticulatedPrismaticLinks:BoxedLcpContact.SixteenArticulatedPrismaticLinksPushArticulatedPrismaticLinksForManySteps' --gtest_brief=1`
  run passed all five tests. The focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldArticulatedLinkImpactStep_BoxedLcp/16/(1|200)$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  rows reported `invariant_ok=1`, `articulated_pair_count=16`,
  `articulated_link_count=32`, `articulated_dof_count=32`, `contact_count=16`,
  and `cross_multibody_link_contact=1` with `step_count` values 1 and 200.
  `DenseBoxWorldContactSnapshotSatisfiesLcpContract`
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
  `World::step()` iterations.
  `ThirtyTwoBoxWorldStepMaintainsDenseContactInvariants` extends this to 32
  boxes and 128 dense face contacts over 4000 small public boxed-LCP
  `World::step()` iterations.
  `FortyEightBoxWorldStepMaintainsDenseContactInvariants` extends this to 48
  boxes and 192 dense face contacts over 4000 small public boxed-LCP
  `World::step()` iterations; the focused default run passed in 84992 ms. The
  `SixtyFourBoxWorldStepPreservesDenseContactShape` test covers one public
  boxed-LCP `World::step()` on a 64-box, 256-contact dense face scene, then
  checks that the contact shape, finite state, and contact height envelope are
  preserved; the focused default run passed in 60 ms.
  `SixtyFourBoxWorldShortHorizonMaintainsDenseContactInvariants` covers the
  same 64-box, 256-contact scene for 75 public boxed-LCP `World::step()`
  iterations under the existing strict settling invariant; the focused default
  two-test filter passed in 3346 ms. This is intentionally bounded evidence:
  temporary 64-box benchmark probes passed at 90 steps with
  `max_vertical_speed=9.80e-2` but failed at 100 steps with
  `invariant_ok=0` and `max_vertical_speed=0.196`, and longer 1000-/4000-step
  probes also failed, so a 64-box long-horizon settling result is not claimed.
  The full `test_boxed_lcp_contact --gtest_list_tests` inventory lists 73 tests; the
  earlier full `--gtest_brief=1` run still emitted the dense-patch Dantzig
  warning, so Dantzig's direct dense box solve is not claimed.
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
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers 72 scoped
  `BM_LcpWorldBoxContact/FrictionIndex/<solver>/{1,2,4,8,16,24,32,48,64,96,128,192}` rows for
  `Pgs`, `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and `Admm`. Each solver
  uses the same 1/2/4/8/16/24/32/48/64/96/128/192-box dense face-contact snapshots,
  covering 4/8/16/32/64/96/128/192/256/384/512/768 contacts,
  12/24/48/96/192/288/384/576/768/1152/1536/2304 rows, and
  1/2/4/8/16/24/32/48/64/96/128/192 dynamic bodies. Focused default, SIMD-enabled, and
  CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContact/FrictionIndex/.+/192$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs each reported `contract_ok=1`, `dense_box_contact=1`, `box_count=192`,
  `contact_count=768`, and `problem_size=2304` for the six new 192-box rows, with
  `build_simd_enabled=1` in the SIMD build tree and `build_cuda_enabled=1` in
  the CUDA-enabled build tree.
  The CUDA-enabled rows are CPU solver benchmark rows in that build tree, not
  CUDA LCP kernel execution. This is dense contact-snapshot evidence, not
  broad robot-like or CUDA dense-contact execution evidence.
- DART 7 dense box-contact serial/parallel batch benchmark evidence:
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now also registers 72
  `BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex` rows. `Pgs`,
  `RedBlackGaussSeidel`, `NNCG`, `Apgd`, `Tgs`, and `Admm` cover
  24/64/96/128/192-box snapshots at batch size 4; `Pgs` additionally covers
  1/4/8/16/32/48-box snapshots so the CPU serial and DART 7
  `ParallelExecutor` rows match the homogeneous CUDA PGS packet sizes through
  96 boxes.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/.+/192/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs each reported 12 rows with `contract_ok=1`, `dense_box_contact=1`,
  `dense_box_contact_batch=1`, `box_count=192`, `contact_count=768`,
  `problem_size=2304`, `batch_size=4`, `total_contact_count=3072`, and
  `total_problem_size=9216`; parallel rows reported `parallel_units=4`, while
  build counters reported `build_simd_enabled=1` in the SIMD build tree and
  `build_cuda_enabled=1` in the CUDA-enabled build tree. The CUDA-enabled rows
  are CPU solver batch rows in that build tree, not CUDA LCP kernel execution.
  Focused matching-size
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/Pgs/(1|4|8|16|24|32|48|64|96)/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in default, SIMD-enabled, and CUDA-enabled build trees each reported 18
  rows with `contract_ok=1`, `dense_box_contact=1`,
  `dense_box_contact_batch=1`, `box_count=1/4/8/16/24/32/48/64/96`,
  `contact_count=4/16/32/64/96/128/192/256/384`,
  `problem_size=12/48/96/192/288/384/576/768/1152`, `batch_size=4`,
  `total_problem_size` up to 4608, and `parallel_units=4` on parallel rows.
- DART 7 dense box-contact end-to-end benchmark evidence:
  `BM_LcpWorldBoxStep_BoxedLcp/{1,2,4,8}/200` and
  `BM_LcpWorldBoxStep_BoxedLcp/16/500`,
  `BM_LcpWorldBoxStep_BoxedLcp/24/2000`, and
  `BM_LcpWorldBoxStep_BoxedLcp/{32,48}/4000`, plus
  `BM_LcpWorldBoxStep_BoxedLcp/64/{1,75}`, rebuild separated box-on-ground scenes,
  confirm each box contributes a 4-contact face patch before stepping, enter
  simulation mode, advance the public boxed-LCP `World::step()` path, and check
  finite-state, contact-height, vertical-rest, and tangential-slowing
  invariants on the registered horizons. Focused default, SIMD-enabled, and
  CUDA-enabled build-tree runs over the 24-/32-box rows reported
  `invariant_ok=1` with
  `dense_box_contact=1`, `box_count=24/32`, `contact_count=96/128`, and
  `step_count=2000/4000`. The default 32-box row reported
  `max_height_error=1.46e-4` and `max_vertical_speed=4.38e-2`; the SIMD and
  CUDA-enabled 32-box rows also reported `invariant_ok=1`. A focused default
  run of the 32-box row in this slice reported `invariant_ok=1`,
  `dense_box_contact=1`, `contact_count=128`, `step_count=4000`,
  `max_height_error=1.46e-4`, and `max_vertical_speed=4.38e-2`. The focused
  default 48-box row reported `invariant_ok=1`, `dense_box_contact=1`,
  `contact_count=192`, `step_count=4000`, `max_height_error=9.80e-5`, and
  `max_vertical_speed=1.08e-2`. The focused default 64-box one-step row
  reported `invariant_ok=1`,
  `dense_box_contact=1`, `contact_count=256`, `step_count=1`,
  `max_height_error=0`, and `max_vertical_speed=6.94e-18`. The focused default
  64-box 75-step row reported `invariant_ok=1`, `dense_box_contact=1`,
  `contact_count=256`, `step_count=75`, `max_height_error=2.00e-4`, and
  `max_vertical_speed=8.28e-2`. Focused SIMD-enabled and CUDA-enabled 64-box
  75-step rows also reported `invariant_ok=1`: the SIMD row reported
  `build_simd_enabled=1`, `max_height_error=1.08e-4`, and
  `max_vertical_speed=8.23e-3`, and the CUDA-enabled row reported
  `build_cuda_enabled=1`, `max_height_error=2.00e-4`, and
  `max_vertical_speed=8.28e-2`. The CUDA-enabled rows are CPU public-step rows
  in that build tree, not CUDA LCP
  kernel execution. The runs still emit the dense-patch Dantzig warning, so
  this is public-step invariant evidence for dense face-contact scenes, not a
  direct Dantzig dense box solve claim.
- DART 7 dense box-contact CUDA batch evidence:
  `CudaLcpJacobiBatch.DenseBoxWorldContactBatchSatisfiesLcpContract` builds
  homogeneous 1-/4-/8-/16-/24-/32-/48-/64-/96-/128-box dense face-contact batches
  with four variants per box count and verifies fixed-iteration CUDA Jacobi
  against the LCP contract with 8192 iterations and relaxation 0.25.
  `CudaLcpJacobiBatch.DenseBoxWorldContactLargestFixtureSatisfiesLcpContract`
  executes the 128-box, 512-contact, 1536-row dense face-contact fixture as a
  homogeneous batch-size-1 CUDA Jacobi packet with the same fixed-iteration
  contract.
  `CudaLcpPgsBatch.DenseBoxWorldContactBatchSatisfiesLcpContract` builds
  homogeneous batches for 1-/16-/24-/32-/48-/64-/96-box dense face-contact snapshots and
  verifies fixed-iteration CUDA PGS against the LCP contract.
  `CudaLcpPgsBatch.DenseBoxWorldContactLargestFixtureSatisfiesLcpContract`
  executes the 128-box, 512-contact, 1536-row dense face-contact fixture as a
  homogeneous batch-size-1 CUDA PGS packet with the same fixed-iteration
  contract.
  `CudaLcpPgsBatch.DenseBoxWorldContactGroupedBatchSatisfiesLcpContract` extends
  that to grouped variable-size 1/2/4/8/16/24/32/48/64/96-box packets; the matching
  `CudaLcpJacobiBatch.DenseBoxWorldContactGroupedBatchSatisfiesLcpContract`
  covers the same grouped shape with 8192 iterations and relaxation 0.25. The
  focused homogeneous Jacobi
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaJacobiWorldBoxContactBatch_FrictionIndex/(1|4|8|16|24|32|48|64|96)/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  CUDA run reported 9 rows with `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_dense_box_contact_batch=1`,
  `dense_box_contact=1`, `cuda_fixed_iterations=8192`,
  `cuda_relaxation=0.25`, `box_count=1/4/8/16/24/32/48/64/96`,
  `contact_count=4/16/32/64/96/128/192/256/384`,
  `problem_size=12/48/96/192/288/384/576/768/1152`,
  `batch_size=4`, `total_contact_count=16/64/128/256/384/512/768/1024/1536`,
  `total_problem_size=48/192/384/768/1152/1536/2304/3072/4608`, and
  `max_residual=max_complementarity` up to `6.94e-18`. The focused 128-box
  Jacobi
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaJacobiWorldBoxContactBatch_FrictionIndex/128/1$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  CUDA row reported `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_dense_box_contact_batch=1`,
  `dense_box_contact=1`, `cuda_fixed_iterations=8192`,
  `cuda_relaxation=0.25`, `box_count=128`, `contact_count=512`,
  `problem_size=1536`, `batch_size=1`, `total_problem_size=1536`, and
  `max_residual=max_complementarity=3.4694469519536142e-18`. The focused
  follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaJacobiWorldBoxContactBatch_FrictionIndex/128/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  CUDA row reported `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_dense_box_contact_batch=1`,
  `dense_box_contact=1`, `cuda_fixed_iterations=8192`,
  `cuda_relaxation=0.25`, `box_count=128`, `contact_count=512`,
  `problem_size=1536`, `batch_size=4`, `total_body_count=512`,
  `total_contact_count=2048`, `total_problem_size=6144`,
  `max_bound_violation=4.3368086899420177e-19`,
  `max_residual=max_complementarity=6.9388939039072284e-18`, and about
  8.096s real time / 8.065s CPU time. The focused grouped
  Jacobi
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaJacobiWorldBoxContactGroupedBatch_FrictionIndex/(2|3)$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  CUDA run reported 2 rows with `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_grouped_batch_execution=1`,
  `cuda_variable_problem_size_batch=1`, `cuda_dense_box_contact_batch=1`,
  `dense_box_contact=1`, `cuda_fixed_iterations=8192`,
  `cuda_relaxation=0.25`, `batch_size=20/30`, `cuda_group_count=10`,
  `box_count_shape_count=10`, `min_problem_size=12`, `max_problem_size=1152`,
  `total_contact_count=2360/3540`, `total_body_count=590/885`,
  `total_problem_size=7080/10620`, and
  `max_residual=max_complementarity=3.4890495707873281e-08`. The focused
  homogeneous PGS
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex/(1|4|8|16|24|32|48|64|96)/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  CUDA run reported 9 rows with `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_dense_box_contact_batch=1`,
  `dense_box_contact=1`, `box_count=1/4/8/16/24/32/48/64/96`,
  `contact_count=4/16/32/64/96/128/192/256/384`,
  `problem_size=12/48/96/192/288/384/576/768/1152`, `batch_size=4`, and
  `total_problem_size` up to 4608. The grouped rows now cover
  1/2/4/8/16/24/32/48/64/96-box packets. The two-variant row reported
  `batch_size=20`, `cuda_group_count=10`, `box_count_shape_count=10`,
  `min_problem_size=12`, `max_problem_size=1152`, `total_contact_count=2360`,
  `total_body_count=590`, `total_problem_size=7080`, and
  `max_residual=max_complementarity=1.5667044491254889e-07`; the
  three-variant row reported `batch_size=30`, the same ten shape groups,
  `total_contact_count=3540`, `total_body_count=885`, `total_problem_size=10620`,
  and `max_residual=max_complementarity=1.5667044491254889e-07`. The PGS
  grouped benchmark rows are contract-correct but expensive, with the `/2` and
  `/3` rows reporting about 140.8s and 142.1s real time respectively, so these
  are not routine checkpoint gates. The old
  fixed-ground homogeneous 128-box fixture loss is now separated from CUDA
  execution:
  `CudaLcpDenseBoxFixture.LargerGridKeepsFaceContactShape` verifies that dynamic
  dense-ground sizing preserves 512 box-face contacts and a 1536-row LCP. The
  focused
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex/128/1$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  CUDA row reports `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_dense_box_contact_batch=1`,
  `dense_box_contact=1`, `box_count=128`, `contact_count=512`,
  `problem_size=1536`, `batch_size=1`, and `total_problem_size=1536`. The
  focused follow-up
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpCudaPgsWorldBoxContactBatch_FrictionIndex/128/4$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  CUDA row reports `contract_ok=1`, `cuda_lcp_execution=1`,
  `cuda_batch_execution=1`, `cuda_dense_box_contact_batch=1`,
  `dense_box_contact=1`, `cuda_fixed_iterations=1024`,
  `cuda_relaxation=1`, `box_count=128`, `contact_count=512`,
  `problem_size=1536`, `batch_size=4`, `total_body_count=512`,
  `total_contact_count=2048`, `total_problem_size=6144`,
  `max_bound_violation=0`,
  `max_residual=max_complementarity=3.4694469519536142e-18`, and about
  232.9s real time / 231.9s CPU time. The earlier dense-box Jacobi probe
  failed under the prior collapsed-interval validation because fixed rows
  required zero residual; after the fixed-bound validation correction, the
  bounded 1-/4-/8-/16-/24-/32-/48-/64-/96-box homogeneous CUDA Jacobi rows,
  1/2/4/8/16/24/32/48/64/96-box grouped CUDA Jacobi rows, and the 128-box
  batch-size-1 and batch-size-4 CUDA Jacobi rows pass.
- DART 7 coupled world-contact stack benchmark evidence:
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now also registers 32
  manifest-generated
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{2,3}` rows, covering all 16
  friction-index-capable solvers on boxed/findex LCP snapshots assembled from
  2- and 3-sphere vertical stacks. It also registers 48
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{4,5,6}` rows for the same
  solver set; the NNCG stack rows report `nncg_pgs_iterations=20` because a
  focused 2-PGS-iteration NNCG 4-sphere trial reached the benchmark iteration
  cap with `contract_ok=0`, and a focused 10-PGS-iteration NNCG 8-sphere trial
  reached the benchmark iteration cap with `contract_ok=0`, residual
  `1.1324957688903847e-02`, and complementarity
  `1.1324957688903792e-02`. A focused NNCG
  2-/3-/4-/5-/6-/7-/8-/9-/10-sphere follow-up reported `contract_ok=1` for all
  nine rows, with residuals from `1.2207031250072164e-05` through
  `2.0448595218027776e-03` and solver iterations from 0 through 101; the
  8-/9-/10-sphere rows report `nncg_pgs_iterations=20`. It also registers 16
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/7` rows for that set. A
  focused default 100-iteration `RedBlackGaussSeidel` 7-sphere
  probe reported `contract_ok=0`, residual
  `1.5380710201222669e-03`, and complementarity
  `1.5380710201222114e-03`; a focused 128-iteration
  `RedBlackGaussSeidel` 8-sphere probe reported `contract_ok=0`, residual
  `1.8185475652150451e-03`, and complementarity
  `1.8185475652149619e-03`. With the stack-contact cap raised to 512
  iterations, focused `RedBlackGaussSeidel`
  2-/3-/4-/5-/6-/7-/8-/9-/10-sphere rows reported `contract_ok=1`; the
  8-/9-/10-sphere rows reported residuals `1.3859891024932125e-03`,
  `1.7007052647963761e-03`, and `2.0442842170353970e-03`, matching
  complementarity residuals, and 135, 162, and 194 solver iterations.
  A focused default 100-iteration `Pgs` 8-sphere probe reported
  `contract_ok=0`, residual `2.7059852530237904e-03`, complementarity
  `2.7059852530237349e-03`, and 100 solver iterations. With the
  stack-contact cap raised to 512 iterations, focused `Pgs`
  8-/9-/10-sphere rows reported `contract_ok=1`; the rows reported residuals
  `3.0803965763981367e-04`, `3.9036340220555132e-04`, and
  `4.7141997313282502e-04`, matching complementarity residuals, and 156, 189,
  and 225 solver iterations. With the same 512-iteration stack-contact cap,
  focused `Jacobi` 8-/9-/10-sphere rows reported `contract_ok=1`; the rows
  reported `jacobi_max_iterations=512`, residuals
  `1.3555336200852253e-03`, `1.6746823956568235e-03`, and
  `2.0486417140315183e-03`, matching complementarity residuals, and 242, 292,
  and 342 solver iterations. With the same 512-iteration stack-contact cap,
  focused `BlockedJacobi` 8-/9-/10-sphere rows reported `contract_ok=1`; the
  rows reported `blocked_jacobi_max_iterations=512`, residuals
  `1.3555336200852253e-03`, `1.6746823956577117e-03`, and
  `2.0486417140306301e-03`, matching complementarity residuals, and 242, 292,
  and 342 solver iterations. With the same 512-iteration stack-contact cap,
  focused `ShockPropagation` 8-/9-/10-sphere rows reported `contract_ok=1`; the
  rows reported `shock_propagation_max_iterations=512`, residuals
  `1.3458208177095088e-03`, `1.6971844082345200e-03`, and
  `2.0335581005710424e-03`, matching complementarity residuals, 118, 141, and
  166 solver iterations, `layer_count=1`, and `block_count=8/9/10`. It also
  registers 16
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/8` rows for `Pgs`, `Jacobi`,
  `Dantzig`, `SymmetricPsor`, `BGS`, `BlockedJacobi`, `RedBlackGaussSeidel`,
  `NNCG`, `SubspaceMinimization`, `Apgd`, `Tgs`, `Staggering`, `Admm`, `Sap`,
  `ShockPropagation`, and `BoxedSemiSmoothNewton`, plus 16
  `BM_LcpWorldStackContact/FrictionIndex/<solver>/{9,10}` rows for the same set
  including `Jacobi`, `BlockedJacobi`, `RedBlackGaussSeidel`, and
  `ShockPropagation`.
  These snapshots include a ground contact and sphere-sphere contacts,
  so the Delassus system couples multiple contacts through shared dynamic
  bodies. The target also registers
  `BM_LcpWorldStackContactAssembly_BoxedLcp/{2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}`, which
  rebuilds the stack world, calls `World::collide()`, assembles/solves through
  `detail::solveBoxedLcpContacts`, and validates the solved snapshot. The
  focused 4-sphere run
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/4|BM_LcpWorldStackContactAssembly_BoxedLcp/4' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  passed with `contract_ok=1` for the original 15 non-NNCG solver rows plus
  the 4-sphere assembly row. This is coupled stack benchmark evidence for small
  vertical sphere stacks, not evidence for
  articulated, robot-like, or dense-degenerate contact scenes.
  The focused
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/5$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs in default, SIMD-enabled, and CUDA-enabled build trees each reported
  `contract_ok=1` for the original 15 non-NNCG 5-sphere solver rows with
  `sphere_count=5`, `contact_count=5`, and `problem_size=15`; focused
  default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/6$|BM_LcpWorldStackContactAssembly_BoxedLcp/6|BM_LcpWorldStackStep_BoxedLcp/6/1000$' --benchmark_min_time=0.001s`
  runs reported `contract_ok=1` for the original 15 non-NNCG 6-sphere solver
  rows, `contract_ok=1` for the 6-sphere assembly row, and
  `invariant_ok=1` for the 6-sphere 1000-step public `World::step()` row. A
  focused default run of `BM_LcpWorldStackContactAssembly_BoxedLcp/7` reported
  `contract_ok=1`, `sphere_count=7`, `contact_count=7`, and `problem_size=21`.
  A focused default
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/7$|BM_LcpWorldStackContactAssembly_BoxedLcp/7$' --benchmark_min_time=0.001s`
  run reported `contract_ok=1` for all 16 registered 7-sphere solver rows and
  the 7-sphere assembly row; the `RedBlackGaussSeidel` 7-sphere row reported
  107 solver iterations.
  Earlier focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/8$|BM_LcpWorldStackContactAssembly_BoxedLcp/8$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `contract_ok=1` for the original 10 non-NNCG registered
  8-sphere solver rows and the 8-sphere assembly row. A current focused default
  run of that same filter reports `contract_ok=1` for all 16 registered
  8-sphere solver rows plus the assembly row; the PGS 8-sphere row reports
  `pgs_max_iterations=512`, `pgs_relaxation=1`,
  `residual=3.0803965763981367e-04`,
  `complementarity=3.0803965763975816e-04`, and 156 solver iterations, the
  NNCG 8-sphere row reports `nncg_pgs_iterations=20`,
  `residual=1.1967153422114407e-03`,
  `complementarity=1.1967153422113852e-03`, and 27 solver iterations, and the
  `RedBlackGaussSeidel` 8-sphere row reports
  `red_black_gauss_seidel_max_iterations=512`,
  `residual=1.3859891024932125e-03`,
  `complementarity=1.3859891024931292e-03`, and 135 solver iterations, and the
  `Jacobi` 8-sphere row reports `jacobi_max_iterations=512`,
  `residual=1.3555336200852253e-03`,
  `complementarity=1.3555336200851698e-03`, and 242 solver iterations, and the
  `BlockedJacobi` 8-sphere row reports `blocked_jacobi_max_iterations=512`,
  `residual=1.3555336200852253e-03`,
  `complementarity=1.3555336200851698e-03`, and 242 solver iterations, and the
  `ShockPropagation` 8-sphere row reports
  `shock_propagation_max_iterations=512`, `residual=1.3458208177095088e-03`,
  `complementarity=1.3458208177094533e-03`, and 118 solver iterations.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/9$|BM_LcpWorldStackContactAssembly_BoxedLcp/9$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `contract_ok=1` for the original 10 non-NNCG registered
  9-sphere solver rows and the 9-sphere assembly row. A current focused default
  run of that same filter reports `contract_ok=1` for all 16 registered
  9-sphere solver rows plus the assembly row; the PGS 9-sphere row reports
  `pgs_max_iterations=512`, `pgs_relaxation=1`,
  `residual=3.9036340220555132e-04`,
  `complementarity=3.9036340220549581e-04`, and 189 solver iterations, while
  the NNCG 9-sphere row reports `nncg_pgs_iterations=20`,
  `residual=1.5222762445965543e-03`,
  `complementarity=1.5222762445964988e-03`, and 101 solver iterations, and the
  `RedBlackGaussSeidel` 9-sphere row reports
  `red_black_gauss_seidel_max_iterations=512`,
  `residual=1.7007052647963761e-03`,
  `complementarity=1.7007052647963206e-03`, and 162 solver iterations, and the
  `Jacobi` 9-sphere row reports `jacobi_max_iterations=512`,
  `residual=1.6746823956568235e-03`,
  `complementarity=1.6746823956567680e-03`, and 292 solver iterations, and the
  `BlockedJacobi` 9-sphere row reports `blocked_jacobi_max_iterations=512`,
  `residual=1.6746823956577117e-03`,
  `complementarity=1.6746823956576562e-03`, and 292 solver iterations, and the
  `ShockPropagation` 9-sphere row reports
  `shock_propagation_max_iterations=512`, `residual=1.6971844082345200e-03`,
  `complementarity=1.6971844082344645e-03`, and 141 solver iterations.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/10$|BM_LcpWorldStackContactAssembly_BoxedLcp/10$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `contract_ok=1` for the original 10 non-NNCG registered
  10-sphere solver rows and the 10-sphere assembly row. A current focused
  default run of that same filter reports `contract_ok=1` for all 16 registered
  10-sphere solver rows plus the assembly row; the PGS 10-sphere row reports
  `pgs_max_iterations=512`, `pgs_relaxation=1`,
  `residual=4.7141997313282502e-04`,
  `complementarity=4.7141997313276951e-04`, and 225 solver iterations, while
  the NNCG 10-sphere row reports `nncg_pgs_iterations=20`,
  `residual=2.0448595218027776e-03`,
  `complementarity=2.0448595218027221e-03`, and 61 solver iterations, and the
  `RedBlackGaussSeidel` 10-sphere row reports
  `red_black_gauss_seidel_max_iterations=512`,
  `residual=2.0442842170353970e-03`,
  `complementarity=2.0442842170353137e-03`, and 194 solver iterations, and the
  `Jacobi` 10-sphere row reports `jacobi_max_iterations=512`,
  `residual=2.0486417140315183e-03`,
  `complementarity=2.0486417140314628e-03`, and 342 solver iterations, and the
  `BlockedJacobi` 10-sphere row reports `blocked_jacobi_max_iterations=512`,
  `residual=2.0486417140306301e-03`,
  `complementarity=2.0486417140305746e-03`, and 342 solver iterations, and the
  `ShockPropagation` 10-sphere row reports
  `shock_propagation_max_iterations=512`, `residual=2.0335581005710424e-03`,
  `complementarity=2.0335581005709868e-03`, and 166 solver iterations.
  The focused default
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackContact/FrictionIndex/.*/(11|12)$|BM_LcpWorldStackContactAssembly_BoxedLcp/(11|12)$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  run reported `rows=34` and `failures=0` for all 16 registered solver
  families at both 11 and 12 spheres plus the 11-/12-sphere assembly rows, with
  `sphere_count=11/12`, `contact_count=11/12`, and `problem_size=33/36`. The 11-/12-sphere PGS rows
  report `pgs_max_iterations=512`, residuals `5.6120473527432324e-04` and
  `6.6932749492387700e-04`, and 263/302 solver iterations; the Jacobi and
  Blocked Jacobi rows report 512-iteration caps, residuals near
  `2.3853672623e-03` and `2.8080594569e-03`, and 398/452 iterations.
  Symmetric PSOR reports `symmetric_psor_max_iterations=512`, residuals
  `2.3669321495569662e-03` and `2.7337675689045327e-03`, and 106/120
  iterations; BGS reports `bgs_max_iterations=512`, residuals
  `2.3990956999897506e-03` and `2.7864848088574590e-03`, and 192/219
  iterations; TGS reports `tgs_max_iterations=512`, residuals
  `5.6120473527432324e-04` and `6.6932749492387700e-04`, and 263/302
  iterations. NNCG reports `nncg_pgs_iterations=20` at 11 spheres and `40` at
  12 spheres, residuals `2.3888996475616153e-03` and
  `2.7209092579925098e-03`, and 183/37 solver iterations. ShockPropagation
  reports `shock_propagation_max_iterations=512`, residuals
  `2.3990956999897506e-03` and `2.7864848088574590e-03`, and 192/219 sweeps.
  The same focused 11-/12-sphere stack/assembly filter passes in the
  SIMD-enabled build tree with `rows=34`, `failures=0`, and `simd_rows=34`, and
  in the CUDA-enabled build tree with `rows=34`, `failures=0`, and
  `cuda_rows=34`. The CUDA-enabled rows are CPU solver rows in a CUDA-enabled
  build, not CUDA LCP kernel execution.
  No 11-/12-sphere public-step rows are claimed.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldStackContact/FrictionIndex/.*/13$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  runs reported 16 all-solver 13-sphere rows with `contract_ok=1`,
  `sphere_count=13`, `contact_count=13`, `problem_size=39`, and the expected
  backend build-state counters. These CUDA-enabled rows are CPU solver rows in
  a CUDA-enabled build, not CUDA LCP kernel execution.
  The focused default
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpWorldStackContactAssembly_BoxedLcp/(13|14|15|16)$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  run reported `contract_ok=1` for the 13-, 14-, 15-, and 16-sphere assembly
  rows with `sphere_count=13/14/15/16`, `contact_count=13/14/15/16`, and
  `problem_size=39/42/45/48`; no 14-/15-/16-sphere solver-comparison or
  public-step rows are claimed.
  The CUDA-enabled solver-comparison rows above are CPU solver benchmark rows
  in that build tree, not CUDA LCP kernel execution; the 11+-sphere assembly
  rows cited here are default-build CPU assembly rows.
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
  a smaller 0.001 s step, `BM_LcpWorldStackStep_BoxedLcp/5/500` rebuilds the
  5-sphere stack world with the same 0.001 s step and a longer horizon, and
  `BM_LcpWorldStackStep_BoxedLcp/6/1000` extends the public step path to the
  6-sphere stack. All five
  rows enter simulation mode, advance the
  public boxed-LCP `World::step()` path, and check the same finite-state,
  non-penetration, spacing, vertical-rest, lateral-drift, and static-ground
  invariants as the unit tests.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='BM_LcpWorldStackStep_BoxedLcp' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `invariant_ok=1` for all stack-step rows; the SIMD run reported
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
  `BM_LcpWorldBoxStep_BoxedLcp/{32,48}/4000`, plus
  `BM_LcpWorldBoxStep_BoxedLcp/64/{1,75}`, rebuild separated
  dense box-face worlds,
  enter simulation mode, advance the public boxed-LCP `World::step()` path, and
  check finite-state, contact-height, vertical-rest, and tangential-slowing
  invariants on the registered horizons. Focused default, SIMD-enabled, and
  CUDA-enabled build-tree
  runs over the 24-/32-box rows reported `invariant_ok=1`, with
  `dense_box_contact=1`, `box_count=24/32`, and `contact_count=96/128`; focused
  default, SIMD-enabled, and CUDA-enabled 48-box rows also reported
  `invariant_ok=1`, `box_count=48`, `contact_count=192`, and `step_count=4000`.
  The focused default 64-box one-step row reported `invariant_ok=1`,
  `box_count=64`, `contact_count=256`, `step_count=1`,
  `max_height_error=0`, and `max_vertical_speed=6.94e-18`.
  The focused default 64-box 75-step row reported `invariant_ok=1`,
  `box_count=64`, `contact_count=256`, `step_count=75`,
  `max_height_error=2.00e-4`, and `max_vertical_speed=8.28e-2`.
  Focused SIMD-enabled and CUDA-enabled 64-box 75-step rows also reported
  `invariant_ok=1`, `box_count=64`, `contact_count=256`, and `step_count=75`;
  the SIMD row reported `build_simd_enabled=1`,
  `max_vertical_speed=8.23e-3`, and the CUDA-enabled row reported
  `build_cuda_enabled=1`, `max_vertical_speed=8.28e-2`.
  The SIMD-enabled 48-box row reported `build_simd_enabled=1`,
  `max_height_error=99.597u`, and `max_vertical_speed=0.0288169`; the
  CUDA-enabled 48-box row reported `build_cuda_enabled=1`,
  `max_height_error=98.038u`, and `max_vertical_speed=0.0108071`. The
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
  `BM_LcpArticulatedUnifiedContact/FrictionIndex/{Ground,RigidImpact,CrossLinkImpact}/<solver>/{1,4,8,16,24,32,48,64}`
  manually assembles fixed-base three-axis prismatic `LinkContact` snapshots
  through `assembleMultibodyLinkContactProblem` and
  `assembleUnifiedConstraintProblem`, then compares all 16
  friction-index-capable solvers on the same 3-row, 12-row, 24-row, 48-row,
  72-row, 96-row, 144-row, and 192-row LCPs.
  The cross-link rows complete a second articulated endpoint for a separate
  multibody, so they exercise the unified contact matrix's cross-multibody
  block. Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpArticulatedUnifiedContact/FrictionIndex/.+/24$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported `contract_ok=1` for all 48 new 24-contact rows, with
  `articulated_unified_contact=1`, `contact_count=24`, `problem_size=72`,
  `multibody_count=24` for ground/rigid-impact rows, and `multibody_count=48`
  plus `articulated_cross_link_contact=1` for cross-link rows. The full
  articulated unified-contact registration now lists 384 rows. Focused
  default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpArticulatedUnifiedContact/FrictionIndex/.*/.*/32$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_format=json`
  runs reported 48 rows with `contract_ok=1`, `contact_count=32`,
  `problem_size=96`, 16 ground rows, 16 rigid-impact rows, 16 cross-link rows,
  and the expected backend build-state counters.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpArticulatedUnifiedContact/FrictionIndex/.+/.+/48$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported 48 rows with `contract_ok=1`, `contact_count=48`,
  `problem_size=144`, 16 rows per articulated contact case,
  `multibody_count=48/96`, and the expected backend build-state counters; the
  maximum reported residual and complementarity counters were
  `8.9363740896075683e-7`.
  Focused default, SIMD-enabled, and CUDA-enabled
  `BM_LCP_COMPARE --benchmark_filter='^BM_LcpArticulatedUnifiedContact/FrictionIndex/.+/.+/64$' --benchmark_min_time=0.001s --benchmark_repetitions=1`
  runs reported 48 rows with `contract_ok=1`, `contact_count=64`,
  `problem_size=192`, 16 rows per articulated contact case,
  `multibody_count=64/128`, and the expected backend build-state counters; the
  maximum reported residual and complementarity counters were
  `8.9363740896075683e-7`.
  The SAP rows use the same robust benchmark parameters as generated coverage
  (`sap_regularization=1e-6`, `sap_max_line_search_iterations=32`,
  `maxIterations=5000`) and report those SAP counters in the articulated
  unified-contact benchmark output; the focused 24-contact runs reported SAP
  iteration counts of 2 and 97 across the three articulated contact cases.
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
- Added a documentation-manifest drift guard: the selection guide's DART 7
  available-solver block now lists exact `LcpSolver::getName()` values between
  machine-readable markers, and
  `AllSolversSmokeTest.DocumentedSolverAvailabilityMatchesManifest` reads that
  block from `docs/background/lcp/07_selection-guide.md` and compares it with
  `kLcpSolverManifest` in order.
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
  16-, 24-, 32-, 48-, 64-, 96-, 128-, 192-, and 256-contact near-singular
  known-solution cases over a scoped robust solver set. The 96-, 128-, 192-,
  and 256-contact packets keep the coupled friction-index topology but use a
  capped normal ramp and `1e6` diagonal spread. Trial
  evidence kept this intentionally narrow: Lemke produced a valid complementary
  solution but not the selected generated solution for the 8-row singular
  standard case, and boxed semi-smooth Newton failed line search on the
  near-singular coupled friction-index cases. `ShockPropagation` is also
  excluded from the coupled friction-index known-solution slice after focused
  default/SIMD/CUDA probes showed contract success but selected-solution errors
  from 0.95 to 20.98 on the coupled packets.
- Added
  `LcpGeneratedCoverage.LargerMildlyIllConditionedKnownSolutionsForScopedSolvers`
  for standard 32-row and 64-row, boxed 16-row and 32-row, friction-index
  8-contact, 1x-/4x-/8x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact cases, and 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-contact cases over solvers that
  reproduce the selected generated solution. Projection-like solvers that
  satisfy the LCP contract but return alternate valid solutions stay covered by
  benchmark contract rows. `MPRGP` is excluded from
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
  friction-index 16-/24-/32-/48-/64-/96-/128-/192-/256-contact known-solution cases over
  the same observed-robust singular-degenerate solver scope.
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
- Added 629 `BM_LcpMildIllConditioned` rows for larger mildly ill-conditioned
  standard 32-row, boxed 16-row, friction-index 8-contact, and coupled
  friction-index 6-, 8-, 12-, 16-, 24-, 32-, 48-, 64-, and 96-contact packets, plus
  4x- and 8x-coupled 6-/8-/12-/16-/24-/32-/48-/64-/96-contact
  single-problem packets and 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact single-problem packets.
  These rows are solve-to-tolerance benchmark
  evidence and report `contract_ok=1`, `mildly_ill_conditioned=1`, backend
  build-state counters, and contact/coupling counters where applicable.
- Added 1258 `BM_LcpMildIllConditionedBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over the full scoped
  mildly ill-conditioned packet set: standard 32-row, boxed 16-row,
  friction-index 8-contact, coupled friction-index
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 4x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact, 8x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-contact, and 15-solver 16x-coupled
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact rows. Boxed Semi-Smooth Newton reports
  tuned line-search settings on the 16x rows. These rows report
  `contract_ok=1`, `mildly_ill_conditioned_batch=1`, backend build-state
  counters, problem/total-problem-size counters, and contact/coupling counters
  where applicable.
- Added 31 `BM_LcpNearSingular` rows for near-singular standard 8-row, boxed
  8-row, and coupled friction-index 3-, 6-, 9-, 12-, 16-, 24-, 32-, 48-, 64-,
  96-, 128-, 192-, and 256-contact packets over the generated robust
  near-singular solver scope.
  These rows report `contract_ok=1`, `near_singular=1`, backend build-state
  counters, and contact/coupling counters where applicable. The CUDA-enabled
  rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP kernel
  execution.
- Added 62 `BM_LcpNearSingularBatch(Serial|Parallel)` rows for batch-size-4
  serial and DART 7 `ParallelExecutor` runs over near-singular standard 8-row,
  boxed 8-row, and coupled friction-index
  3-/6-/9-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets. These rows
  report `contract_ok=1`, `near_singular_batch=1`, backend
  build-state counters, problem/total-problem-size counters, and
  contact/coupling counters where applicable. The CUDA-enabled rows are CPU
  solver batch rows in a CUDA-enabled build, not CUDA LCP kernel execution.
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
- Added 51 `BM_LcpExtremeSingularDegenerate` rows for exact rank-deficient
  standard 128-row, boxed 128-row, and coupled friction-index
  16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets over the generated extreme
  singular-degenerate solver scope. These
  rows report `contract_ok=1`, `singular_degenerate=1`, `rank_deficient=1`,
  backend build-state counters, and contact/coupling counters where applicable.
  The CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
- Added 72 `BM_LcpSingularDegenerateFrictionIndexBatch(Serial|Parallel)` rows
  for batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
  rank-deficient coupled friction-index
  6-/8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact packets. These rows report
  `contract_ok=1`, `singular_degenerate_batch=1`, `rank_deficient=1`, backend
  build-state counters, and contact/coupling counters. The CUDA-enabled rows
  are CPU solver batch rows in a CUDA-enabled build, not CUDA LCP kernel
  execution.
- Added 192 `BM_LcpSingularDegenerateStandardBoxedBatch(Serial|Parallel)`
  rows for batch-size-4 serial and DART 7 `ParallelExecutor` runs over exact
  rank-deficient standard and boxed 16-/32-/64-/128-row packets. These rows
  report `contract_ok=1`, `singular_degenerate_batch=1`,
  `singular_degenerate_standard_boxed_batch=1`, `rank_deficient=1`, backend
  build-state counters, and problem/total-problem-size counters. The
  CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not CUDA
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
- Added 128 `BM_LcpProductionActiveSetTransition` rows for stronger-coupled
  24-contact, 72-row, 32-contact, 96-row, 48-contact, 144-row, and
  64-contact, 192-row, 96-contact, 288-row, 128-contact, 384-row, and
  192-contact, 576-row and 256-contact, 768-row friction-index active-set
  transition packets over every friction-index-capable manifest solver. These
  rows report `contract_ok=1`,
  `active_set_transition=1`,
  `production_active_set_transition=1`, `contact_count=24/32/48/64/96/128/192/256`,
  `problem_size=72/96/144/192/288/384/576/768`,
  `coupling_scale=2/4/8/16/32`, backend build-state counters, and `coupled=1`.
  The CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA
  LCP kernel execution.
- Added production active-set transition generated coverage for the
  256-contact, 768-row, `coupling_scale=32` packet. The focused
  `LcpGeneratedCoverage.ProductionActiveSetTransitionFrictionIndexKnownSolutionsForScalableSolvers`
  filter passes in the default, SIMD-enabled, and CUDA-enabled build trees.
- Added 550
  `BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)` rows for
  batch-size-4 serial and DART 7 `ParallelExecutor` runs over standard
  32/64/128-row, boxed 32/64/128-row, and coupled friction-index
  8-/12-/16-/24-/32-/48-/64-/96-/128-/192-/256-contact active-set packets. These rows
  report `contract_ok=1`, `production_active_set_transition_batch=1`,
  `batch_size=4`, `problem_size=24/32/36/48/64/72/96/128/144/192/288/384/576/768`,
  `total_problem_size=96/128/144/192/256/288/384/512/576/768/1152/1536/2304/3072`,
  backend build-state counters, and parallel execution counters on the
  `ParallelExecutor` rows. Friction-index rows also report
  `contact_count=8/12/16/24/32/48/64/96/128/192/256`,
  `total_contact_count=32/48/64/96/128/192/256/384/512/768/1024`, and
  `coupling_scale=1/2/4/8/16/32`.
  The CUDA-enabled rows are CPU solver batch rows in a CUDA-enabled build, not
  CUDA LCP kernel execution.
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
  reported 16 rows, and JSON checks for `BM_LcpBlockPartitionSweep` reported 16
  rows with `contract_ok=1` in the default, SIMD-enabled, and CUDA-enabled
  build trees. These rows cover standard 12-row, boxed 12-row, and
  friction-index 4-/8-contact fixtures with full-block, 3-row block, auto
  `findex`, and explicit contact-block partitions for both solvers. The four
  8-contact rows report `problem_size=24` and 8 three-row contact blocks. The
  rows report `block_partition_sweep=1`, block counts `1/4/8`, block sizes
  `3/12`, `contact_count=4/8`, observed solver `iterations=1/4/5/6/10/19`,
  and backend build-state counters. Focused
  `BlockedJacobiSolverCoverage.*:BgsSolverCoverage.*` unit coverage passed 15
  tests. The CUDA-enabled rows are CPU BGS/Blocked Jacobi rows in a
  CUDA-enabled build, not CUDA LCP kernel execution.
- Added Blocked Jacobi solver-internal threading evidence:
  `UNIT_math_lcp_math_lcp_lcp_projection_solvers --gtest_filter='BlockedJacobiSolver.InvalidWorkerThreadCount:BlockedJacobiSolver.ThreadedPathMatchesSerial' --gtest_brief=1`
  passed 2 focused tests, and
  `UNIT_math_lcp_math_lcp_lcp_generated_coverage --gtest_filter='LcpGeneratedCoverage.ThreadedBlockedJacobiStandardKnownSolution' --gtest_brief=1`
  passed the 4-worker 128-row generated known-solution test. Focused
  `BM_LcpBlockedJacobiSolverThreadingBanded_Standard` rows passed in default,
  SIMD-enabled, and CUDA-enabled build trees for 128-row serial/4-worker and
  512-/1024-/2048-row serial/4-/8-worker banded packets plus
  4096-/8192-row serial/32-worker banded packets with `contract_ok=1`,
  `blocked_jacobi_auto_singleton_blocks=1`,
  `solver_internal_threads=1/4/8/32`, and
  `blocked_jacobi_threaded_block_updates=0/1`. This proves the opt-in CPU
  threaded independent-block update path over larger banded packets, not a
  speedup or CUDA-kernel claim.
  The focused 4096-/8192-row follow-up reported 8 rows per build tree with
  `problem_size=4096/8192`, `block_count=4096/8192`,
  `matrix_nonzero_entries=20474/40954`,
  `matrix_density=0.001220345/0.000610262`, `solver_internal_threads=1/32`,
  residual/complementarity about `5.68e-4/5.38e-4`, `contract_ok=1`, and the
  expected backend build-state counters.
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
- Extended DART 7 coupled stack snapshot correctness evidence to a 6-sphere
  vertical stack, covering a 6-contact, 18-row boxed/findex LCP assembled from
  shared dynamic bodies.
- Added DART 7 coupled stack end-to-end `World::step()` evidence for the same
  3-sphere topology over 200 and 500 public boxed-LCP steps.
- Added boxed-LCP Baumgarte velocity-bias stabilization and DART 7 coupled
  stack end-to-end `World::step()` evidence for a 4-sphere stack over 200
  public boxed-LCP steps.
- Extended DART 7 coupled stack end-to-end `World::step()` evidence to a
  5-sphere stack over 500 public boxed-LCP steps and a 6-sphere stack over
  1000 public boxed-LCP steps.
- Added DART 7 articulated boxed-LCP `World::step()` evidence for fixed-base
  prismatic links in one-link and four-link ground-contact scenes, plus
  1-/4-/8-/16-link articulated ground-step benchmark rows.
- Added DART 7 connected multi-DOF articulated boxed-LCP `World::step()`
  evidence for fixed-base three-axis prismatic Cartesian chains in ground
  contact, plus 1-/4-/8-/16-chain articulated Cartesian ground-step benchmark
  rows.
- Added DART 7 two-sided articulated boxed-LCP `World::step()` evidence for
  one-pair, four-pair, eight-pair, and sixteen-pair fixed-base prismatic
  link-vs-dynamic-rigid scenes, including a sixteen-pair 200-step scene, plus
  1-/4-/8-/16-pair one-step and 16-pair 200-step articulated rigid-impact
  benchmark rows.
- Added DART 7 cross-multibody articulated boxed-LCP `World::step()` evidence
  for one-pair, four-pair, eight-pair, and sixteen-pair fixed-base prismatic
  link-vs-link scenes, including a sixteen-pair 200-step scene, plus
  1-/4-/8-/16-pair one-step and 16-pair 200-step articulated link-impact
  benchmark rows.
- Added DART 7 articulated unified-contact all-solver benchmark evidence for
  manually assembled fixed-base three-axis prismatic link-ground and
  link-vs-dynamic-rigid LCP snapshots, now extended to cross-multibody
  link-vs-link snapshots and 64-contact packets.
- Added DART 7 world-contact benchmark rows that compare all
  friction-index-capable solvers on the same real boxed/findex contact
  snapshots from 1, 2, and 4 separated sphere-ground contacts, plus a benchmark
  for rebuilding/colliding/assembling/solving those boxed-LCP contact snapshots.
- Added coupled DART 7 world-contact stack benchmark rows that compare all
  friction-index-capable solvers on 2- and 3-sphere vertical stacks with shared
  dynamic bodies, plus 4-, 5-, and 6-sphere rows for all of those solvers,
  plus 7-sphere rows for all of those solvers, plus 8-/9-/10-/11-/12-/13-sphere
  rows for the full solver set. Stack assembly/solve benchmark rows cover
  2-, 3-, 4-, 5-, 6-, 7-,
  8-, 9-, 10-, 11-, 12-, 13-, 14-, 15-, and 16-sphere scenes.
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
  and benchmark evidence for direct 24/48/96/128/192/256-row standard and boxed
  packets, direct 8/16/32/48/64/96-contact friction-index packets, grouped
  16/32/48/96/128/192-row standard and boxed packets, and grouped
  4/8/16/32/48/64-contact friction-index packets, with two- and three-variant
  grouped synthetic rows. Added homogeneous 4-/8-/16-/24-/32-contact
  and grouped variable-size 1/2/4/8/16/24/32-contact DART 7 separated world-contact
  CUDA unit and benchmark evidence, plus homogeneous 5-/6-/7-/8-/9-/10-/11-/12-/13-/14-/15-/16-sphere and
  grouped variable-size 2/3/4/5/6/7/8/9/10/11/12/13/14/15/16-sphere coupled stack-contact CUDA unit and
  benchmark evidence and manually assembled 1-/4-/8-/16-/24-/32-contact articulated
  unified-contact CUDA unit and benchmark evidence for link-ground,
  link-vs-dynamic-rigid, and cross-multibody link-vs-link packets for the same
  kernels. Added mixed grouped CUDA unit and benchmark evidence that combines
  those separated, stack, and 1-/4-/8-/16-/24-/32-contact articulated fixture families,
  including cross-multibody link-vs-link packets, in one size-grouped batch.
  The current CUDA path intentionally excludes other solvers, CUDA Jacobi
  dense-contact batches, broader dense-contact CUDA execution, and end-to-end
  articulated world-step CUDA execution.

## Remaining Gaps

- DART 7 integration: decide how these math solvers are selected and exercised
  from the `dart::simulation::World` contact pipeline without exposing internal
  solver/backend registry types.
- Coverage breadth: extend deterministic generated fixtures beyond the current
  production-scale well-conditioned, larger mildly ill-conditioned,
  singular-degenerate through the current 128-row/256-contact correctness and
  benchmark slice,
  and
  active-set transition through the current stronger-coupled 256-contact
  correctness, single-benchmark, and batch-benchmark slices
  into harder solver-specific friction-index coupling edge cases and direct
  backend execution evidence beyond CPU solver rows in SIMD/CUDA-enabled
  builds. Standard/boxed exact rank-deficient singular-degenerate batch
  evidence now reaches 16-/32-/64-/128-row packets for the scoped robust solver
  set.
- Harder friction-index conditioning: the current all-solver generated
  friction-index grid now includes coupled well-conditioned 2-contact and
  4-contact and 6-contact cases plus mildly ill-conditioned 2-contact and
  4-contact cases. Scoped generated evidence now reaches coupled
  well-conditioned 12-contact, mildly ill-conditioned 64-contact, 4x-coupled
  mildly ill-conditioned 6-, 8-, 12-, 16-, 24-, 32-, 48-, and 64-contact cases, 8x-coupled
  mildly ill-conditioned 6-, 8-, 12-, 16-, 24-, 32-, 48-, and 64-contact cases with
  Boxed Semi-Smooth Newton included in single rows and coupled batch rows,
  15-solver single-problem and batch rows for 1x-/4x-/8x-coupled mildly
  ill-conditioned 6-, 8-, 12-, 16-, 24-, 32-, 48-, 64-, and 96-contact cases
  and 16x-coupled single rows plus default batch rows through 256 contacts,
  near-singular 256-contact generated and benchmark cases,
  singular-degenerate 192-contact cases, and production active-set
  transition 24-, 32-, 48-, 64-, 96-, 128-, 192-, and 256-contact
  single-problem and batch cases. Denser DART 7 contact-derived
  and direct backend execution evidence still need solver-specific expansion.
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
  prismatic link-vs-dynamic-rigid boxed-LCP `World::step()` invariant tests and
  1-/4-/8-/16-pair one-step plus 16-pair 200-step articulated rigid-impact
  benchmark rows, a cross-multibody fixed-base prismatic link-vs-link boxed-LCP
  `World::step()` invariant tests and 1-/4-/8-/16-pair one-step plus 16-pair
  200-step articulated link-impact benchmark rows,
  1-/2-/4-/8-/16-/24-/32-/48-box dense face-contact boxed-LCP `World::step()`
  benchmark rows plus 64-box one-step and 75-step dense face-contact
  public-step rows,
  manually assembled fixed-base three-axis prismatic articulated unified-contact
  all-solver benchmark rows through 64 contacts for link-ground,
  link-vs-dynamic-rigid, and cross-multibody link-vs-link
  snapshots,
  contact-derived benchmark rows for 1/2/4 separated sphere-ground contacts,
  coupled benchmark rows for 2-/3-/4-/5-/6-sphere vertical stacks across all
  friction-index-capable solvers, 7-sphere stack rows for all of those
  solvers, 8-/9-/10-/11-/12-/13-sphere stack rows for the full solver set, mixed
  serial and `ParallelExecutor` batch rows
  over the 1/2/4 separated-contact and 2/3 stack snapshots, stress
  mixed serial and `ParallelExecutor` batch rows over the same separated
  snapshots plus 2/3/4/5 stack snapshots for all of those solvers except
  `NNCG`, 11-/12-/13-/14-/15-/16-sphere stack snapshot and assembly rows, plus existing
  boxed-contact parity tests. Broader articulated
  contacts beyond fixed-base prismatic link-ground, connected Cartesian-chain
  ground contact, link-vs-rigid impact, and cross-multibody link-vs-link impact,
  longer denser coupled scenes, larger
  coupled multi-contact systems beyond the current 16-sphere snapshot, and
  robot-like contact systems still need LCP-contract, invariant, and benchmark
  evidence.
- Multi-contact boxed-LCP snapshots: a resting box multi-contact friction
  snapshot probe did not satisfy the LCP contract cleanly, consistent with the
  existing static-friction test's degenerate-pivot warning. Keep this as a
  separate solver/contact-assembly evidence slice.
- Benchmarks: extend `tests/benchmark/lcpsolver/` beyond generated math
  fixtures, active-set, larger/stress/extreme/production active-set packets,
  production active-set serial and `ParallelExecutor` batch packets,
  mildly ill-conditioned single-problem and coupled/4x/8x/16x batch packets,
  near-singular single-problem and batch packets, singular-degenerate
  single-problem and friction-index batch packets, PGS/PSOR, symmetric PSOR,
  and Red-Black Gauss-Seidel relaxation/threading rows, APGD restart-policy
  sweep rows, TGS iteration-budget sweep rows, NNCG PGS-preconditioner
  iteration sweep rows, SubspaceMinimization PGS-iteration sweep rows,
  ShockPropagation
  layer-layout sweep rows, MPRGP SPD/check sweep rows, Interior Point
  path-parameter sweep rows, Staggering contact-pipeline sweep rows, Boxed
  Semi-Smooth Newton line-search sweep rows, Pivoting scale sweep rows, ADMM
  rho/adaptive-rho sweep rows, SAP regularization sweep rows, BGS/Blocked
  Jacobi block-partition sweep rows, Blocked Jacobi solver-internal threading
  rows, ADMM/SAP/Boxed Semi-Smooth Newton
  contact comparison sweep rows, contact-normal standard-LCP sweep rows,
  independent-problem batches, simple world-contact snapshots, small coupled
  stack snapshot batches, dense box-face snapshot/step rows, and scoped dense
  box-face serial/parallel batch rows to broader
  dense and robot-like end-to-end contact systems, broader SIMD benchmark packets, larger threaded
  configurations, broader CUDA LCP solver execution, and
  vectorized/threaded/CUDA batch-processing paths.
- Compute backends: current benchmark rows now self-report scalar/SIMD/CUDA
  build state, a focused local SIMD-enabled CPU slice passes, a focused
  CUDA-enabled build/runtime slice passes, and narrow CUDA projected-Jacobi and
  PGS standard/boxed/friction-index through direct 256-row and 96-contact
  packets plus grouped variable-size synthetic standard/boxed/friction-index
  through 256-row and 96-contact groups with matching CPU
  serial/`ParallelExecutor` rows and fixed-iteration CUDA rows for the two- and
  three-variant grouped synthetic
  benchmark rows,
  homogeneous 4-/8-/16-/24-/32-contact, homogeneous
  5-/6-/7-/8-/9-/10-/11-/12-/13-/14-/15-/16-sphere coupled stack, grouped variable-size
  1/2/4/8/16/24/32-contact separated sphere-ground and
  2/3/4/5/6/7/8/9/10/11/12/13/14/15/16-sphere coupled stack world-contact batch paths with two- and
  three-variant grouped benchmark rows, plus manually
  assembled 1-/4-/8-/16-/24-/32-contact articulated unified-contact batch paths with
  two- and three-variant grouped benchmark rows including cross-multibody
  link-vs-link packets, mixed
  separated/stack/articulated grouped contact batch paths, homogeneous Jacobi
  dense box-face CUDA batches through 128 boxes, homogeneous PGS dense box-face
  CUDA batches through 128 boxes, two-/three-variant grouped Jacobi and PGS
  dense box-face CUDA batches through 96 boxes, plus bounded 128-box
  homogeneous batch-size-1 and batch-size-4 CUDA Jacobi and PGS packet passes.
  Jacobi has opt-in solver-internal CPU
  worker-thread correctness and benchmark evidence, including larger 8192-row
  banded Jacobi, Red-Black Gauss-Seidel, and Blocked Jacobi rows, but the
  focused local rows do not establish a general speedup.
  Other intra-solver multi-threaded
  CPU paths, general CUDA LCP solver execution, broader dense-contact CUDA
  execution, end-to-end articulated world-step CUDA execution, and broader
  vectorized/CUDA LCP batch-processing paths still need separate evidence.
- Background taxonomy upkeep: keep `docs/background/lcp/`, the solver manifest,
  and benchmark registration synchronized whenever solver support changes. The
  selection guide's available-solver block is now mechanically checked against
  `kLcpSolverManifest`.

## Immediate Next Steps

1. Extend solver-specific friction-index conditioning/coupling grids beyond the
   current exact rank-deficient 128-row/256-contact correctness/benchmark,
   and production active-set transition 256-contact correctness, single-benchmark,
   and batch benchmark slices.
2. Extend DART 7 boxed-LCP world-contact evidence from current separated
   sphere-ground, current fixed-base prismatic articulated end-to-end coverage,
   current connected Cartesian-chain articulated end-to-end coverage,
   current cross-multibody articulated link-vs-link impact coverage,
   current manually assembled three-axis articulated LCP snapshots, and
   current 13-sphere all-solver vertical-stack solver rows, current 16-sphere
   vertical-stack snapshots, and dense box face-contact
   evidence to broader articulated, longer-running, and denser coupled contact
   scenes.
3. Add broader benchmark gates for SIMD-enabled CPU, intra-solver
   multi-threaded CPU, general CUDA LCP solver execution, and vectorized/CUDA
   batch-processing paths.
