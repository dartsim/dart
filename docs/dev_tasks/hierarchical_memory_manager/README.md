# Hierarchical Memory Manager — Dev Task

## Critical Stop Handoff

Maintainer requested on 2026-06-11: stop implementation work and focus only on
handoff, with no further verification. This docs-only handoff must not be read
as a new validation point; the next agent should start fresh and verify before
making code changes.

Use exactly one branch for continuation:
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean`. PR #2955 and PR #2956 are merged.
Other HMM follow-up branches are historical/no-resume targets unless a
maintainer explicitly says otherwise.

Latest validated source checkpoint before this handoff:
`e9b2014f3db` (`Prewarm rigid IPC stack solve scratch`). It was pushed and
validated with lint, build, focused rigid IPC world/barrier tests, and full
`pixi run test-unit`. The handoff-only docs commit after it intentionally skips
`pixi run lint`, build, tests, and benchmarks.

Current completed Phase 4/5 slice:

- Rigid IPC projected-Newton solve scratch now persists assembly and
  line-search buffers on the World allocator.
- `RigidIpcContactStage::prepare()` prewarms same-shape solve/result surface
  buffers, articulation rows, active constraint rows, and assembly scratch.
- The dynamic rigid IPC no-heap gate now covers contact-free dynamics, active
  static/dynamic mesh barrier, fixed-joint and revolute-joint IPC constraints,
  and the two-box stack.
- The measured two-box stack gap was 1 global allocation / 96 bytes over four
  baked steps from free-list growth during objective assembly; that gap is now
  closed at the latest validated source checkpoint.

Fresh agent entry point:

```bash
git fetch origin
git checkout pr/hmm-phase45-follow-up-clean
git pull --ff-only
git status -sb
git log --oneline --decorate -8
```

Then read `RESUME.md` and the remaining Phase 4/5 follow-up items in this
file. Continue only after measuring a real remaining allocator/no-heap gap.
Do not add more work to PR #2956; it is already merged.

## Current Handoff

Stop state for fresh handoff: use exactly one branch,
`pr/hmm-phase45-follow-up-clean`, tracking
`origin/pr/hmm-phase45-follow-up-clean` and based on `origin/main` after PR
#2956 landed. Other similarly named HMM follow-up branches are historical and
should not be used by fresh sessions unless a maintainer explicitly redirects
the work.

The final 2026-06-11 handoff snapshot is the pushed branch head after the
critical "handoff only, no further verification" request. It includes the
active dynamic rigid IPC barrier cleanup plus these handoff notes, but it was
committed without any fresh lint/build/test run after the stop request. The
previous fully validated code checkpoint was `57cb751eef9` (`Avoid heap
allocation in dynamic rigid IPC no-contact steps`). The docs-only stop commit
`91c3d83dd35` incorrectly described the
`WorldStorage` allocator-root slice as unapplied. Current source inspection
shows the branch already contains the equivalent, broader persistent-World
root-routing work: `WorldStorage`, the private built-in step-pipeline cache,
built-in stage-owned scratch/cache objects, the lazy collision query cache, and
the optional replay controller object are all constructed through the World
free-list allocator. Treat `git log` on `pr/hmm-phase45-follow-up-clean` as the
source of truth for the exact pushed branch head, and resume only from a fresh
session after reading this task state.

Current continuation after the final stop request: the rigid IPC two-box stack
WIP was completed and the temporary diagnostic patch was removed. The measured
gap was `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failing for
`dynamic rigid IPC two-box stack` with 1 global allocation / 96 bytes over the
counted baked steps. The allocation came from free-list growth during
projected-Newton objective assembly after a large aligned triplet-scratch
request. `RigidIpcProjectedNewtonSolveScratch` now owns persistent
assembly/line-search scratch constructed from the World allocator, and
`RigidIpcContactStage::prepare()` prewarms same-shape solve/result surface
buffers, articulation rows, and assembly scratch before counted steps.

The focused baked dynamic rigid IPC no-heap gate now also covers fixed-joint
and revolute-joint articulated IPC constraints plus the two-box stack. The
focused gate passed locally after the debug-only heap backtrace and free-list
growth probes were removed. `pixi run lint`, `pixi run build`, focused rigid
IPC world/barrier tests, and `pixi run test-unit` also pass for this slice.

Fresh-session agents should start with
`docs/dev_tasks/hierarchical_memory_manager/RESUME.md`, then verify the live
checkout with:

```bash
git checkout pr/hmm-phase45-follow-up-clean
git status -sb
git log --oneline --decorate -5
```

The latest allocator-root slice closes a compute-graph
scratch gap: allocator-aware `ComputeGraph` traversal used to allocate from the
global heap while checking cycles, rebuilding topological order, validating,
and scanning resource hazards. The focused regression failed before the fix
with 38 global allocations / 880 bytes during allocator-aware traversal and now
passes with zero global heap allocation. This is pipeline scratch cleanup, not
a new deformable production-scene coverage claim.

The current continuation extends dynamic rigid IPC no-heap coverage in three
bounded steps. First, the single supported dynamic body with no possible
contact/articulation pairs takes the exact diagonal inertial quadratic
minimizer instead of constructing the full projected-Newton system; that baked
gate failed before the slice with 156 global allocations / 7920 bytes over four
steps. Second, an active static/dynamic mesh barrier shape no longer rebuilds a
single-node internal solve graph per step, prewarms the reusable projected
Newton step/result row storage during `prepare()`, and reuses those buffers
through the counted solve; that shape failed before the slice with 157 global
allocations / 7968 bytes over four baked steps. Third, the projected-Newton
solve scratch now owns persistent assembly/line-search buffers so stack and
articulated-contact shapes can prewarm their objective assembly scratch during
`prepare()`; the two-box stack failed before this slice with 1 global
allocation / 96 bytes over four baked steps. The covered dynamic rigid IPC
shapes now pass with zero global heap allocation.

The previous proven Phase 4/5 slice keeps the semi-implicit external-force
multibody path inside both World-base no-growth and global-heap no-allocation
gates after bake. The body-Jacobian container now reuses
`MultibodyDynamicsScratch::bodyJacobian`, and the remaining pure
external-force global-heap allocations were removed by skipping split
semi-implicit contact/unified collision queries when no relevant collision
shapes exist. A follow-up query-pruning slice makes the rigid contact stage
treat empty `CollisionGeometry` components like no collision geometry when
deciding whether prepare/execute needs a contact query; this is a scoped
performance guard, not a new claimed heap-gap closure.

The last full local validation for the code checkpoint was:

- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- Focused `test_world` no-growth/no-heap gates covering the forced-slider,
  multibody/deformable, and boxed-LCP fallback shapes.

Additional checks completed before the 2026-06-11 stop-and-handoff request for
the rigid empty-geometry query-pruning slice:

- Focused `test_world` contact gates:
  `World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap`,
  `World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator`, and
  `World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap`.
- `git diff --check`
- `pixi run lint`
- `pixi run build`

No additional verification was run after the stop request; in particular,
`pixi run test-unit` was not re-run for the latest rigid empty-geometry slice.

Additional checks completed for the compute-graph traversal slice:

- Before the fix:
  `SimulationComputeGraph.AllocatorAwareTraversalAvoidsGlobalHeap` failed with
  38 global allocations / 880 bytes.
- After the fix:
  `build/default/cpp/Release/bin/test_compute_graph --gtest_filter='SimulationComputeGraph.AllocatorAwareTraversalAvoidsGlobalHeap' --gtest_color=no`
  passed.
- Full `build/default/cpp/Release/bin/test_compute_graph --gtest_color=no`
  passed.
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit` passed all 161 tests.

Additional checks completed for the dynamic rigid IPC dynamics-only slice:

- Before the fix:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failed with 156
  global allocations / 7920 bytes over four baked steps.
- After the fix:
  `build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap:World.RigidIpcContactStageAdvancesMeshBodyFromRuntimeDynamics:World.RigidIpcContactStageAdvancesSphereBodyFromRuntimeDynamics' --gtest_color=no`
  passed.
- A broader focused IPC `test_world` filter also passed after preserving the
  one-body IPC diagnostic counters.
- `git diff --check` and `pixi run lint` passed before the final handoff-only
  doc edits.

Additional check completed for the persistent-World root-routing status
correction:

- `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
&& build/default/cpp/Release/bin/test_world --gtest_filter='World.WorldPersistentStorageUsesWorldFreeAllocator' --gtest_color=no`
  passed, confirming the current branch's existing `WorldStorage`,
  built-in-cache, collision-cache, replay-controller, and `World::clear()`
  allocator-root coverage.

Additional checks completed for the active dynamic rigid IPC barrier slice:

- Before the fix:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failed for the
  active static/dynamic mesh barrier subcase with 157 global allocations / 7968
  bytes over four baked steps.
- After the fix:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
&& build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap' --gtest_color=no`
  passed.
- A broader focused rigid IPC `test_world` filter passed:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap`,
  `World.RigidIpcContactStageSeparatesActivatedMeshBarrier`,
  `World.RigidIpcContactStageAdvancesMeshBodyFromRuntimeDynamics`,
  `World.RigidIpcContactStageAdvancesSphereBodyFromRuntimeDynamics`, and
  `World.RigidIpcContactStageTwoBoxStackSettlesWithoutPenetration`.
- `git diff --check`, `pixi run lint`, and `pixi run build` passed before the
  critical final handoff request.

Additional checks completed for the dynamic rigid IPC stack/articulation
scratch slice:

- Before the fix:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap` failed for the
  `dynamic rigid IPC two-box stack` subcase with 1 global allocation / 96 bytes
  over four baked steps.
- The temporary allocation trace identified free-list growth during
  `assembleRigidIpcObjectiveSystemWithScratch()` through
  `solveRigidIpcProjectedNewtonBarrierSystem()` after a large aligned
  triplet-scratch request.
- After the fix:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"
&& build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap' --gtest_color=no`
  passed with the debug-only heap/free-list logging removed.
- A broader focused rigid IPC `test_world` filter passed:
  `World.BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap`,
  `World.RigidIpcContactStageSeparatesActivatedMeshBarrier`,
  `World.RigidIpcContactStageAdvancesMeshBodyFromRuntimeDynamics`,
  `World.RigidIpcContactStageAdvancesSphereBodyFromRuntimeDynamics`,
  `World.RigidIpcContactStageTwoBoxStackSettlesWithoutPenetration`,
  `World.RigidIpcContactStageProjectsFixedJointPose`, and
  `World.RigidIpcContactStageProjectsRevoluteJointHingeAxis`.
- Full `build/default/cpp/Release/bin/test_rigid_ipc_barrier --gtest_color=no`
  passed.
- `git diff --check`, `pixi run lint`, `pixi run build`, and
  `pixi run test-unit` passed. `pixi run test-unit` passed all 161 tests.

The final handoff edits after the critical stop request intentionally skipped
`pixi run lint`, build, and tests. A `pixi run test-unit` job was in progress
and was terminated to honor the stop request, so no full `pixi run test-unit`
result should be inferred for the active dynamic rigid IPC barrier slice after
the listed focused checks. Later continuation slices should list their own
verification here, as the dynamic rigid IPC slice does above.

Continue only with evidence-first Phase 4/5 work from the remaining follow-up
items below. Do not add more scenes or scratch-reuse commits to PR #2956; that
PR is already merged.

## Current Status

- [x] Phase 1: Experimental `World` owns a memory manager and exposes
      frame-scratch diagnostics.
- [ ] Phase 2: Allocator correctness and performance gates cover DART
      allocators against standard C++ allocators and foonathan/memory.
      Alignment-aware allocation is implemented; fixed-size pool comparison now
      has a DART `FixedPoolAllocator` path that beats foonathan/memory and
      `std::pmr` locally. The comparative benchmark binary now honors the
      checker-requested repetition count instead of forcing five repetitions.
      The strict checker now rejects high-CV rows before treating ratios as
      evidence. `StlAllocator` keeps allocator-backed STL storage
      alignment-aware, including fixed-pool-backed max-aligned values. Focused
      EnTT registry probes now cover foonathan/memory's array-capable pool
      baseline and the standard registry, including separate build/growth rows
      for bake-time registry storage allocation. The focused EnTT checker
      caches known component storage handles, uses free-list-backed
      world-lifetime DART storage for persistent no-growth churn, and uses
      a resettable frame-backed DART bake arena for one-shot registry
      build/growth storage construction. The foonathan build/growth row uses
      `memory_stack` marker/unwind storage, matching the same bulk-lifetime
      role instead of comparing against a persistent pool collection. The
      no-growth row reports post-prewarm allocator-call counters and fails if
      reserved churn asks the configured allocator for storage after prewarm;
      the build/growth row times the uninstrumented
      construction/destruction path. The
      strict checker prints DART benchmark counters
      alongside
      pass/fail ratios, so EnTT misses distinguish timing loss, allocator
      traffic, growth, and noisy benchmark evidence. It also requires the
      expected benchmark keys for the selected mode, so missing
      foonathan/memory coverage is a failure instead of a skipped comparison.
      The checker rejects high-CV rows unless the saved mean/stddev/repetition
      aggregates still show DART's normal-approximation 95% confidence interval
      strictly below the selected baseline's confidence interval.
      The default comparative matrix now also covers foonathan/memory static
      fixed-storage stacks, scoped temporary allocators, and two-iteration frame
      allocators against the DART frame allocator's 32-byte fast path and
      `std::pmr` monotonic baselines. It also requires raw heap/malloc/new
      allocator rows plus aligned, fallback, segregator, tracked, and deeply
      tracked foonathan adapter rows mapped to DART frame/pool HMM roles and
      standard allocator baselines.
      `FreeListAllocator` now has a fixed-capacity mode for
      deterministic bounded failure after preallocation, and `MemoryManager` /
      DART 7 `WorldOptions` can construct the World free-list hierarchy
      with a fixed-capacity policy. Fixed-capacity free-list arenas can also
      satisfy over-aligned pool chunks from reserved bytes without growing from
      the base allocator. The current allocator correctness slice hardens
      count/size overflow guards for `MemoryAllocator::allocateAs`,
      `FrameAllocator`, `FrameStlAllocator`, and `StlAllocator`, adds focused
      pool/free-list/fixed-pool/frame/STL allocator coverage for invalid sizes,
      reuse, bounded failure, diagnostics, debug misuse paths, and
      allocator-root isolation across independent `MemoryManager` and
      experimental `World` instances, keeps aligned debug allocations tied to
      the matching aligned deallocation contract, releases debugger-tracked
      leaks with their recorded size/alignment during debugger destruction, and
      keeps fixed-capacity free-list aligned-allocation diagnostics on
      user-requested bytes instead of internal header padding. `FrameStlAllocator` blocks are cache-line
      aligned without per-block cache coloring, so frame-backed STL pages used by
      allocator-aware EnTT storage keep the alignment benefit without avoidable
      arena padding, and the frame allocator now has cheaper no-overflow
      reset plus 32-byte and cache-line-aligned fast paths for the comparative
      frame/STL rows. The STL-vector adapter benchmark is batched reserve-only
      allocator-adapter work so it measures allocator throughput instead of
      identical vector element writes. `PoolAllocator` now has an explicit
      diagnostics policy; release `MemoryManager` pool allocation and the
      comparative DART pool rows use the non-diagnostic hot path while direct
      `PoolAllocator` construction keeps live/peak counters enabled by default.
      The stack, frame-bulk, fallback-stack, small-pool, STL-vector,
      iteration, tracked-stack, and deeply tracked pool comparative rows now
      batch repeated allocator cycles so the strict CV gate measures sustained
      allocator work. `FixedPoolAllocator` also uses a cache-friendly stride
      for medium power-of-two slots, which removes the fixed-pool cache-set
      conflict that previously let foonathan/memory win `BM_Pool/256/256`.
      `PoolAllocator` default-size requests now use a constexpr heap-index
      lookup table for the same rounded/skewed size classes, removing repeated
      size-class arithmetic from mixed-size allocation/deallocation hot paths.
      A 2026-06-06 CPU-affined foonathan-plus-standard-plus-EnTT matrix,
      merged with focused replacement rows for strict-CV stability, passed all
      94 DART-vs-baseline comparisons, including EnTT no-growth and
      build/growth rows. Phase 2 remains open for broader HMM production
      no-growth coverage and any future allocator baselines, but the current
      DART allocator implementations now beat every required standard C++ and
      foonathan/memory row in the comparative matrix with non-noisy aggregate
      evidence. A 2026-06-07 continuation added allocator overflow and
      `construct`/`destroy` hooks to the STL adapters, cache-line colored
      frame-backed STL storage, cheaper no-overflow frame resets, and a
      fixed-pool DART steady-state churn row. The current foonathan-only
      matrix, merged with focused strict-CV replacement rows for the loaded
      host, passes all 47 DART-vs-foonathan checks, including EnTT no-growth,
      EnTT build/growth, steady-state, stack, frame, raw, adapter, and tracked
      rows. A later 2026-06-07/08 continuation restored explicit STL
      construct/destroy hooks, added a stateless `DefaultStlAllocator`, and
      refined the checker to require confidence-separated evidence for any
      accepted high-CV rows; the merged current foonathan broad-plus-EnTT
      result in
      `.benchmark_results/allocator_foonathan_broad_entt_current_check.json`
      passes all 47 foonathan comparisons. Re-run the standard-baseline half
      before making a fresh post-policy-change 94-row claim. Later
      random-interleaved EnTT diagnostics showed the pool-backed no-growth row
      is still too close to foonathan/memory at small sizes to treat the
      sequential artifact as the final "beats every foonathan allocator" proof;
      keep EnTT steady-state optimization open and use random interleaving for
      follow-up allocator-policy evidence. A follow-up switched persistent EnTT
      no-growth storage to the world free-list arena, which is the better DART
      HMM match for reserved variable-size registry arrays; the merged focused
      result in
      `.benchmark_results/allocator_entt_freelist_nogrowth_frame_build_current_merged_check.json`
      passes all 12 EnTT no-growth/build comparisons against both
      foonathan/memory and standard baselines with strict CV checks. The same
      continuation fixed the remaining mixed-size pool misses with the
      heap-index table; the current merged broad-plus-focused result
      `.benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json`
      passes all 94 foonathan/memory and standard comparisons, including
      `BM_MultiPool`, `BM_Realistic`, EnTT no-growth, and EnTT build/growth.
      A 2026-06-08 follow-up restored cache-line alignment for large
      allocator-backed STL storage pages, switched the EnTT build/growth DART
      row to the stateless default C-heap STL adapter, and switched the
      foonathan build/growth baseline to `memory_stack` marker/unwind bulk
      lifetime storage. The full foonathan matrix in
      `.benchmark_results/allocator_comparative_foonathan_sustained_entt_stack_cpu13_check.json`
      passes all 47 DART-vs-foonathan comparisons, including EnTT no-growth
      and build/growth. The focused EnTT run in
      `.benchmark_results/allocator_entt_sustained_default_stack_cpu13_check.json`
      passes all 12 EnTT comparisons against both foonathan/memory and standard
      baselines; the full standard-registry half remains a separate
      post-policy-change gap before making a fresh 94-row
      standard-plus-foonathan claim.
      A later continuation corrected the build/growth DART row back to
      `StlAllocator` over `FrameAllocator`, matching foonathan/memory's
      `memory_stack` marker/unwind lifetime. Focused CPU6 probes show this
      role-correct build/growth path can beat foonathan and standard rows, but
      the persistent no-growth timing proof remains open because current runs
      are dominated by unrelated host load and the no-growth row still reports
      zero allocator calls after prewarm. A subsequent follow-up added
      runner-side CPU prewarm immediately before affinity-pinned benchmark
      launch and cache-line coloring for large 64-byte-aligned
      `FreeListAllocator` array allocations, then added shape-aware coloring
      for large 64-byte-aligned `FrameAllocator` storage. The coloring is based
      on the EnTT/World registry access pattern: consecutive page-sized
      component arrays can otherwise start on identical cache sets when their
      allocation sizes are multiples of the cache-index period. Persistent
      free-list-backed registry storage uses eight 64-byte colors with a
      shifted initial phase, while one-shot frame-backed build storage stays
      dense for scalar arrays and uses cache-line alignment only for
      cache-line-sized or over-aligned value pages.
      Focused probes show the 512-entity no-growth row can close the foonathan
      gap with zero post-prewarm allocator calls; a stricter focused check then
      reduced the remaining evidence gap to the `BM_EnttRegistryBuild/512`
      frame-backed build/unwind row. Frame allocator coloring reduced that row
      from ratio 1.043 to roughly 1.007 in a focused probe; the remaining
      adapter overhead is addressed by using DART's frame-native
      `FrameStlAllocator` for the build/growth row and by routing scalar
      frame-backed STL arrays directly to the inline default frame fast path
      instead of cache-line-aligning every large `std::vector` payload.
- [ ] Phase 3: EnTT registry/component storage allocation is configurable from
      the World memory hierarchy and covered by no-growth ECS tests.
      Allocator-aware EnTT storage now has focused `StlAllocator` and
      `FrameStlAllocator` unit tests showing that reserved
      create/emplace/read/destroy churn makes no configured allocator calls or
      arena growth after the prewarm pass. The DART comparative no-growth EnTT
      row now uses free-list-backed world-lifetime storage for reserved
      variable-size registry arrays and fails if reserved churn asks the
      configured allocator for storage after prewarm. The
      benchmark hot path caches known component storage handles so the timing
      surface matches optimized World systems rather than repeated registry
      type lookup. The focused no-growth benchmark now uses free-list-backed
      persistent array storage rather than the small-object pool path. Separate
      EnTT build/growth rows measure the bake-time
      storage allocation phase instead of conflating that cost with the
      no-growth simulation loop. The comparative benchmark now discovers
      installed EnTT package metadata for these rows without invoking DART's
      FetchContent-backed dependency helper, and the checker reports a clear
      error if a requested result file is empty.
      Initial registry/component-storage wiring and `enterSimulationMode()`
      reservation/no-growth tests for current World-owned ECS storage and the
      first multibody/deformable private step-scratch components are
      implemented. Direct EnTT create/emplace/clear/destroy/reuse storage
      cycling is covered after explicit reserve. `World::clear()` now recreates
      the internal allocator-backed registry storage so ECS capacities and
      debug-tracked registry allocations reset at the rebuild boundary while
      preserving the World memory hierarchy. Contact-heavy variational
      ground-contact dual state and compliant contact-point scratch are now
      sized at `enterSimulationMode()` and covered by World-base allocator
      no-growth tests that step multiple variational contact sliders without
      registry capacity growth. Pure compliant variational contact now
      evaluates directly from baked ground-contact scratch and reusable
      contact-evaluation vectors, so it no longer constructs the
      augmented-Lagrangian solver's dual vector; AL contact still owns solver
      dual scratch only when a positive dual-update cadence requires it. The
      compliant contact hook no longer copies contact-point storage in the
      step loop, and the single-prismatic World-surface contact path now
      evaluates ground contact without constructing the general residual/contact
      work vectors.
      Broader solver scratch coverage remains open.
- [ ] Phase 4: Built-in simulation stages borrow world memory for transient
      buffers and avoid growth after simulation is baked. The default
      `WorldStepPipeline` now stores built-in non-owning stage pointers inline,
      removing its per-build `std::vector` allocation from the normal step path.
      Longer custom pipelines keep the previous arbitrary-stage behavior through
      an overflow path. The legacy graph-backed `RigidBodyIntegrationStage` now
      reuses stage-owned rigid-body entity and dependency-node scratch instead
      of allocating per execute. The batched SoA rigid-body integration stage
      now reuses stage-owned force, state, model, initial-state, and
      parent-before-child frame-order scratch, and runs its single SoA kernel
      directly instead of rebuilding a one-node compute graph every execute.
      Custom batched SoA stages can now borrow a `MemoryManager` so their
      force and frame-order scratch vectors reserve from the World allocator
      root.
      The default rigid-body velocity/contact stages and semi-implicit
      multibody velocity/contact path now reuse baked scratch for the covered
      rigid and articulated resting-contact scenes, and the semi-implicit
      external-force branch now reuses baked body-Jacobian scratch instead of
      building a step-local Jacobian vector. The
      variational multibody stage now owns cache-only reusable contact/constraint
      scratch, bakes loop-closure and hard AVBD point-joint constraint capacity,
      evaluates pure compliant ground contact from baked ground-contact scratch,
      reserves augmented-Lagrangian solver scratch only for positive dual-update
      cadence contact, and bakes those vectors before contact-heavy steps. Its
      baked tree topology/link-index/child-list scratch now also borrows the
      World free allocator. It now routes the single-prismatic compliant
      ground-contact stage path through a scalar scratch-free solver fast path.
      A stronger compliant-contact global-heap
      probe now covers the baked multi-slider World-surface path without
      step-loop heap allocations. The boxed-LCP
      unified constraint stage now reuses stage-owned assembly containers and
      unified problem storage, and its shared/cross-row assembly no longer
      allocates per-step row-direction, rigid/articulated row-end, or
      shared-body inertia lookup containers. The stage path also assembles
      per-multibody link-contact rows through reusable
      `MultibodyDynamicsScratch` instead of the public return-by-value
      assembler, and cross-multibody row completion reuses the same scratch
      for other-link point Jacobians and joint-space denominator work instead
      of allocating local lookup/context/Jacobian temporaries. The Dantzig
      boxed-LCP solver now has caller-owned reusable scratch, a matrix/vector
      overload that avoids `LcpProblem` copies for already assembled systems,
      and a same-shape no-heap regression; `UnifiedConstraintStage` owns and
      reuses unified solve scratch that carries the Dantzig scratch plus island
      remapping/sub-problem buffers, successful link impulse application
      buffers, normal-only fallback buffers, and fallback tangent accumulators.
      Same-shape no-heap coverage now includes unified island solves, scratch-
      backed link impulse application, and rigid plus cross-multibody fallback
      friction sweeps.
      The unified assembler now reuses same-shape link-block row storage
      without per-row Eigen matrix-vector temporaries; same-shape no-heap
      coverage now also includes mixed rigid plus borrowed-link unified
      assembly. The boxed-LCP stage borrows per-multibody contact problems from
      persistent `MultibodyDynamicsScratch` instead of copying them into
      staging containers first. The rigid-contact assembler now has an in-place
      scratch overload, so same-shape fallback steps reuse the stage-owned
      rigid contact problem instead of building a by-value temporary each step.
      `UnifiedConstraintStage::prepare()` now primes the initial boxed-LCP
      contact shape during `enterSimulationMode()`, moving the current World
      fallback scenes' first active solve allocation out of the step loop. The
      same bake path now also primes normal-only fallback scratch
      unconditionally, so a same-shape solve that switches from the full
      boxed-LCP to the rank-deficient fallback does not allocate fallback
      matrices, tangent accumulators, or joint-space impulse buffers in the
      counted step loop.
      Public multibody link-contact assembly now has reusable scratch storage
      that can be borrowed by the in-place unified assembler without same-shape
      heap growth. The no-scratch public boxed-LCP solve wrapper now moves the
      solved lambda vector out of its local scratch instead of allocating a
      second result copy, and callers that want a `UnifiedConstraintSolution`
      object can now pass caller-owned result storage alongside solve scratch
      for same-shape no-growth solves. Rigid IPC accepted and rejected
      writeback now reuses the stage-owned blocked/writeback/entity-order
      scratch prepared with the rigid body capacity instead of allocating local
      traversal vectors, and the rigid IPC resting-contact no-op predicate now
      reuses stage-owned per-body contact-power and stationary-flag scratch.
      The rigid IPC projected-Newton loop now reuses solve-local surface buffers
      across line-search and sufficient-decrease backtracking candidates, and
      repeated solve-internal barrier assembly and line-search calls reuse
      surface-pair/triplet scratch, including the lagged-friction barrier pass.
      The rigid IPC contact stage now calls the projected-Newton solve through a
      caller-owned result/scratch overload so per-solve surface candidate
      buffers persist in stage scratch across steps.
      `DeformableDynamicsStage` now owns reusable
      obstacle-list, deformable surface-snapshot, and rigid
      surface-CCD snapshot scratch, and `prepare()` primes per-body
      surface-contact candidate buffers plus inter-body/rigid surface-CCD sweep
      buffers for baked steps. Scripted deformable boundary processing now
      reuses per-body Dirichlet/Neumann count masks instead of allocating local
      per-node vectors each step. Default deformable projected-Newton friction
      now reuses per-body normal-force, normal-direction, and self-contact
      friction-contact buffers instead of allocating local `std::vector`
      scratch in the step loop, and static-ground box CCD footprint clipping now
      uses fixed-size stack storage instead of allocating a tiny footprint
      vector on every non-vertical sweep. Surface-contact candidate and sweep
      buffers now get topology-scaled bake-time reserve capacity, so the covered
      frictional self-contact patch, 5x5 two-layer grid, 7x7 two-layer large
      grid, 9x9 two-layer production grid, and 11x11 two-layer extended
      production grid reuse candidate and friction-contact storage through
      projected-Newton line-search CCD. The direct-sparse default solver now
      carries that same no-growth coverage through 13x13, 15x15, 17x17, and
      dense 13x19 rectangular two-layer production grids, and the matrix-free
      default solver now carries the 17x17 larger, 13x19 rectangular, 7x17 wide,
      and 17x7 tall production guards through its CG scratch path. A mixed
      two-body production gate now steps one direct-sparse rectangular grid and
      one matrix-free wide grid in the same baked default-solver loop. A mixed
      dense production gate now combines a notched, jittered direct-sparse 13x17
      grid with a matrix-free 13x19 dense rectangular grid, and a mixed
      late-active production gate now steps direct-sparse square and matrix-free
      rectangular self-contact grids whose active contacts enter during the
      counted baked steps. These cover
      per-body solver/contact scratch storage for independent deformable bodies
      with different topology shapes and linear-solver modes. A default
      projected-Newton FEM ground-friction block now covers multi-tetrahedron
      rest-shape, Hessian-block, and multi-node ground-friction storage in both
      baked no-growth guards; compact and production mixed default-solver
      storage gates now combine direct-sparse self-contact, matrix-free
      self-contact, and FEM ground-friction bodies in one baked World memory
      root. The default projected-Newton path now also has a baked
      sphere/box/capsule static obstacle barrier gate, covering those radial and
      oriented-obstacle Hessian paths separately from the ground height-field
      barrier and surface-CCD snapshot gates. A production static-obstacle
      friction patch now slides independent node patches near sphere, box, and
      capsule barriers in one default projected-Newton solve, covering the
      shared static-obstacle normal/friction scratch for both sparse and
      matrix-free projected-Newton paths separately from self-contact friction.
      A mixed production gate now steps sparse static-obstacle friction and
      matrix-free self-contact bodies in the same baked World root, covering
      simultaneous obstacle and self-contact scratch without shared-root growth;
      a complementary mixed gate pairs matrix-free static-obstacle friction
      with a direct-sparse irregular self-contact grid to cover the opposite
      solver pairing in one root. A complementary contact-family production
      gate now combines that matrix-free obstacle/direct-irregular
      self-contact pairing with inter-body surface CCD in one baked root.
      The moving rigid-surface CCD path now has baked
      swept-box point-crossing gates for both free predicted motion and
      kinematic trace-backed motion, including a multi-kinematic traced-obstacle
      scene that reuses combined swept snapshot capacity across independent
      deformable bodies. `prepare()` primes the kinematic swept-box
      snapshot buffers before a current-frame trace exists, so stage-owned
      moving rigid snapshot storage and surface-sweep candidate scratch are
      covered separately from static rigid snapshots. Projected-Newton sparse
      assembly now reserves self-contact barrier block storage from baked
      candidate
      capacity instead of only the
      bake-active candidate count, so same-topology active set variation does
      not grow DART-owned barrier vectors. Motion-aware
      self-contact candidate buffers now reserve the swept late-activation
      envelope during bake, and both the direct sparse and matrix-free
      projected-Newton paths now cover square and rectangular late-activating
      self-contact without growing step-loop candidate storage. A notched,
      jittered 13x17 two-layer production self-contact mesh now exercises the
      same direct-sparse default solver path through topology-scaled scratch
      rather than square-grid assumptions, and the same irregular mesh now
      covers the matrix-free projected-Newton path. The matrix-free path also
      reuses solver-owned Hessian block plus CG vector scratch instead of
      allocating local solve temporaries. Per-body deformable solver vectors
      for inertial targets, trial/gradient/direction/candidate state,
      previous-step positions, external accelerations, and fixed/boundary
      masks now construct from the World free allocator.
      AVBD ground contact/friction rows and
      self-contact normal/friction rows now reuse row-inventory and
      self-contact adjacency storage, including previous friction warm-start
      rows, and `prepare()` bakes row/candidate capacity for the covered AVBD
      ground-contact and two-surface contact scenes. AVBD row-inventory and
      friction-projection warm-start lookup keep rows in descriptor order while
      indexing the previous frame through sorted reserved storage, avoiding
      both map allocation and quadratic previous-row scans. Rigid AVBD contact
      projection now reuses stage-owned snapshot, point-joint, row-counter,
      row-inventory, inertial-target, and solve-row scratch for covered active
      rigid contacts and no-contact fixed-joint rows; the public
      return-by-value AVBD helpers remain allocation-boundary conveniences. The
      follow-up VBD path now also reuses stage-owned Chebyshev history vectors
      and bakes self-contact candidate sweep plus adjacency capacity for all
      enabled VBD surface bodies, not only AVBD self-contact row bodies. The
      current
      production boxed-LCP stage
      uses the in-place unified assembler and solve scratch; the public
      return-by-value unified problem and solution wrappers remain
      allocation-boundary API conveniences rather than step-loop hot paths.
- [ ] Phase 5: Add allocation/debug accounting gates for "no dynamic allocation
      during the step loop" on representative rigid, multibody, contact, and
      deformable scenes. World base-allocator no-growth guards now cover baked
      kinematic IPC rigid-body, multibody variational, deformable ECS,
      rigid-body resting-contact, semi-implicit external-force multibody,
      non-cross articulated resting-contact, and same-DOF cross-articulated
      link-contact paths after contact prewarm; a
      global heap guard covers the same baked kinematic IPC, rigid-body,
      multibody variational, compliant variational contact, deformable,
      rigid-body resting-contact, semi-implicit external-force multibody,
      non-cross articulated resting-contact, and same-DOF cross-articulated
      contact paths. Mixed/different-DOF, stacked, and coupled multi-row
      cross-contact boxed-LCP fallback scenes now have base-allocator
      no-growth gates and first baked-step global heap no-allocation gates.
      A focused same-shape guard also covers the prewarmed batched SoA
      rigid-body integration stage on a frame-coupled parent/child body pair,
      proving its batch arrays and frame-order scratch do not allocate after
      prewarm.
      Five-multibody, eight-multibody, 12-multibody, 16-multibody,
      24-multibody, and 32-multibody stacked contact sets extend the boxed-LCP
      fallback gate beyond the original small scenes, and a production
      multi-island mixed scene now covers independent articulated and rigid
      contact islands with 12+ initial contacts; the stress multi-island gate
      extends that shape to 30+ initial contacts across independent
      articulated and rigid islands. A mixed stress boxed-LCP scene now combines
      the 32-multibody stacked fallback and stress multi-island shape under one
      baked World root with 60+ initial contacts. Broader solver coverage,
      including default-solver deformable storage and any newly exposed
      production contact shapes, remains open before making a full
      zero-allocation claim.
      The
      global heap guard now also covers a baked deformable surface-snapshot
      scene with a static rigid surface-CCD obstacle and first-baked-step active
      VBD static rigid surface-CCD point crossing. Default projected-Newton
      deformable scratch now reuses its RHS, sparse Hessian assembly, PSD block
      batches, sparse-pattern cache, and solution storage for the covered
      mass-spring path; default static rigid surface-CCD point crossing and an
      active inter-body deformable surface-CCD crossing are also covered by
      baked no-growth guards. FEM rest-shape caches are primed during
      `enterSimulationMode()`, and a one-tetrahedron FEM projected-Newton path
      is covered by the same guard; motion-aware surface-contact candidate
      scratch now reserves the wider swept-AABB envelope needed by non-square
      self-contact grids while staying linear in topology size; broader
      projected-Newton self-contact barrier scratch is sized from bake-primed
      contact candidates and covered for the two-triangle no-friction
      self-contact path. The global heap guard now also covers a baked
      default-solver deformable ground-friction projected-Newton scene, a
      multi-tetrahedron FEM ground-friction block, plus a multi-triangle
      frictional self-contact patch, a 5x5 two-layer frictional self-contact
      grid, a 7x7 two-layer large grid, a 9x9 two-layer production grid, an
      11x11 two-layer extended production grid, a 13x13 two-layer dense
      production grid, a 15x15 extra-dense two-layer production grid,
      direct-sparse and matrix-free 17x17 larger two-layer production grids, a
      9x13 non-square two-layer production grid, direct-sparse and matrix-free
      13x19 dense rectangular production grids, direct-sparse and matrix-free
      7x17 wide non-square production grids, direct-sparse and matrix-free 17x7
      tall non-square production grids, a notched and jittered 13x17 irregular
      direct-sparse production grid, a notched and jittered 13x17 irregular
      matrix-free production grid, and an 11x11 late-active two-layer
      direct-sparse plus matrix-free grid that starts outside the self-contact
      barrier band and enters it during the counted baked steps, with the same
      late-active direct-sparse plus matrix-free coverage for a 9x13
      rectangular grid. The same base and global-heap guards now also include
      a mixed two-body production scene with independent direct-sparse
      rectangular and matrix-free wide self-contact grids, a mixed late-active
      production scene with independent direct-sparse square and matrix-free
      rectangular self-contact grids, a mixed dense production scene with
      notched direct-sparse and dense matrix-free grids, a production mixed
      default-solver storage scene with direct-sparse self-contact,
      matrix-free self-contact, and FEM ground-friction bodies, a mixed
      production FEM ground-friction scene with direct and matrix-free
      4x4x4-node blocks, plus a production rectangular inter-body deformable
      surface-CCD crossing, a
      barrier-only static sphere/box/capsule obstacle scene, and a production
      static sphere/box/capsule obstacle friction patch for sparse and
      matrix-free default projected Newton, plus a mixed static-obstacle and
      self-contact production scene and the complementary matrix-free
      static-obstacle plus direct irregular self-contact scene that exercise
      both scratch families under one World memory root. A further mixed
      default-solver contact-family production scene now combines direct static
      obstacle friction, matrix-free self-contact friction, and inter-body
      surface CCD under one baked World root; the complementary contact-family
      scene now flips the static-obstacle path to matrix-free and the
      self-contact path to direct irregular topology while keeping production
      inter-body surface CCD in the same baked root.
      The base and global-heap guards now also include default moving
      rigid-surface CCD swept-box crossings for free, single-kinematic, and
      multi-kinematic rigid obstacles, closing the previously static-only rigid
      surface snapshot coverage and the first-traced-kinematic allocation gap.
      The
      larger-grid guards also assert
      non-vacuous solver activity through public deformable diagnostics: active
      self-contact barriers, converged active contacts, and positive friction
      dissipation. The global heap and World-base no-growth guards now also
      cover the compact and production rectangular inter-body deformable
      surface-CCD crossings, active AVBD ground contact/friction rows, AVBD
      self-contact normal/friction rows including 5x9 and 9x13 rectangular grid
      row workloads, and an active rigid AVBD penetrating contact plus
      no-contact fixed-joint rows.
      Additional broader or differently shaped production-scale frictional
      deformable scenarios still need no-growth gates before making the full
      deformable claim.
      The follow-up branch adds an active VBD Chebyshev self-contact grid to
      both the World-base no-growth and global-heap no-allocation guards,
      proving the baked VBD Chebyshev/self-contact path does not allocate after
      prewarm for that covered shape.
- [ ] Phase 6: Add memory-layout profiler/debugger surfaces and GUI
      visualization. `MemoryAllocatorDebugger` now exposes structured live
      bytes, peak live bytes, and live allocation count; `MemoryManager` and
      DART 7 `World` diagnostics now surface direct free/pool allocator
      debug counters. `WorldMemoryDiagnostics` also reports aggregate and
      per-storage ECS registry layout counters without exposing EnTT types, and
      dartpy exposes the same read-only snapshot through
      `World.memory_diagnostics`. The standalone `dartsim` editor now has a
      tested read-only Memory panel action seam that surfaces frame scratch,
      allocator debug counters, and largest ECS storage capacities. Broader
      profiler overlays and workflow-specific visualizations remain future
      work.

## Goal

Move the DART 7 simulation stack toward zero dynamic memory allocation in
the simulation loop by giving each `World` one hierarchical allocator root and
routing persistent and per-frame data through that root. Long term, the memory
hierarchy should map cleanly to world objects/components so it can drive memory
debugging, profiling, optimization experiments, and ImGui visualization.

## Non-Goals For The First Slice

- Do not claim zero allocations for every experimental solver path yet.
- Do not replace Eigen's internal heap behavior in one pass.
- Do not expose raw EnTT storage, solver registries, or backend resource types
  in public memory diagnostics; surface only DART-owned counters and diagnostic
  IDs.

## Key Decisions

- Reuse `dart::common::MemoryManager` as the allocator hierarchy root instead
  of introducing a simulation-only allocator stack for the first slice, but do
  not assume the current allocator implementations are correct or fast enough.
- Follow the existing common-module rule: `World` owns the memory manager;
  stages and components borrow allocators from the world.
- Treat frame scratch as valid for the current simulation frame and reset it at
  the start of the next `World::step()` call.
- Treat `entt::registry` allocation as a first-class integration target. The
  ECS storage layer is a dominant owner of world/component memory, so future
  work must either instantiate the registry/storage with a DART allocator or
  provide an equivalent EnTT storage integration that preserves public API
  boundaries.
- The active EnTT version supports stateful allocator propagation through
  `entt::basic_registry`: the DART 7 World now constructs its internal
  registry and component storages with a `dart::common::StlAllocator` borrowing
  the World's active free allocator.
- Free-list allocations must preserve at least `std::max_align_t` alignment
  after every split. EnTT component storage surfaced this as an Eigen
  `FrameCache` alignment failure when a max-aligned component allocation
  followed an odd-sized allocation.
- Broad hot-loop adoption is blocked until allocator correctness tests and
  benchmarks prove DART's allocators beat both standard C++ allocators and every
  required foonathan/memory allocator baseline on DART-relevant workloads. A
  missing foonathan/memory baseline is incomplete evidence, not a pass. If DART
  cannot beat foonathan/memory for required features and workloads, prefer an
  explicit dependency decision over shipping a weaker in-house allocator.
- Use `FixedPoolAllocator` for fixed-size node/slot workloads and keep
  `PoolAllocator` focused on mixed size-classed small-object workloads. The
  comparative benchmark must not compare DART's generic size-classed pool
  against foonathan's fixed-size pool when a DART fixed-size allocator exists.
- Use fixed-capacity `FreeListAllocator` instances when the allocator budget was
  established during world creation or bake/build and runtime growth would
  violate the no-dynamic-allocation contract. The default free-list policy
  remains expandable for general heap-like use. Route the policy through
  `MemoryManager::Options` and `WorldOptions` rather than exposing EnTT or
  solver storage types.

## Required Allocator Evidence

- Correctness tests: alignment (including over-aligned Eigen-like types),
  zero-size behavior, reuse-after-free, double-free/leak debug paths, frame
  reset semantics, overflow/growth accounting, deterministic failure handling
  for bounded/static storage, and multi-world isolation.
- Benchmarks: allocation/deallocation latency, container workloads, per-frame
  arena reset, pool reuse under churn, mixed-size free-list fragmentation,
  multibody/contact/deformable step-loop scratch patterns, and no-growth
  baked-world repeated simulation loops.
- Comparison targets: `std::allocator`/`std::pmr` where applicable, current
  DART allocators, improved DART allocators, and foonathan/memory's relevant
  raw allocators, pools, static storage, temporary allocator, adapters, and
  tracking/debug wrappers.
- Completion gate: the required comparative benchmark/checker matrix must cover
  every foonathan/memory allocator family that maps to a DART allocator role
  used by the HMM plan, and every DART implementation row must beat the matching
  foonathan/memory row with non-noisy aggregate evidence or
  confidence-separated high-CV evidence. Rows that are missing, noisy without
  separated confidence intervals, or slower keep this dev task open.

## Required EnTT Integration Evidence

- Registry construction: prove whether the active EnTT version can use a
  stateful allocator through `entt::basic_registry` and component
  `basic_storage`, or document the exact adapter/storage work needed.
- Component storage: cover create/emplace/destroy/clear/reuse patterns for the
  current DART 7 components, including sparse-set growth and component
  array capacity behavior. Initial direct coverage now reserves entity,
  `RigidBodyTag`, `Transform`, `Velocity`, and `Force` storages and verifies
  repeated create/emplace/clear/re-emplace/destroy cycles do not grow World
  base-allocator traffic after reserve.
- Bake/build reserve path: reserve registry entities and component pools before
  simulation so repeated `World::step()` calls do not grow ECS storage.
- No-growth proof: after the reserve/prewarm path, representative
  create/emplace/read/destroy churn must make zero calls through the configured
  DART allocator or grow the configured arena; benchmark timing alone is not
  sufficient evidence.
- Boundary rule: keep EnTT allocator/storage types out of the promoted public
  facade; expose only DART-owned memory options and diagnostics.

## PR #2956 Phase 4/5 Wrap-Up

PR #2956 stops at the proven coverage below. Remaining Phase 4/5 work moves to
a follow-up branch so this PR can ship the current allocation gates without
adding more scene surface.

Current Phase 4 scratch-reuse coverage shipped by this PR includes:

- Inline built-in stage storage for the default `WorldStepPipeline`, legacy
  graph-backed rigid integration scratch, and batched SoA rigid integration
  scratch for force/state/model/initial-state/frame-order buffers.
- Rigid IPC stage scratch for accepted/rejected writeback, resting-contact
  no-op detection, projected-Newton surface buffers, barrier assembly, line
  search, equality change-of-variable indices, and lagged-friction passes.
- Unified/boxed-LCP scratch for in-place rigid and multibody assembly, Dantzig
  solve reuse, island remapping, link impulse application, normal-only
  fallback, tangent accumulators, multibody staging vectors, and same-shape
  fallback friction sweeps.
- Variational multibody scratch for baked loop-closure, hard AVBD point-joint,
  finite-stiffness AVBD point-joint compliant-loop constraints,
  velocity-actuator projection rows, compliant/augmented-Lagrangian ground
  contact, and the scalar single-prismatic compliant-contact fast path.
- Default projected-Newton deformable scratch for obstacle lists, static and
  moving rigid surface snapshots, surface-contact candidates, boundary masks,
  friction buffers, sparse/matrix-free solver storage, FEM blocks, static
  obstacle friction, self-contact, and inter-body surface CCD.
- AVBD scratch for ground contact/friction rows, self-contact adjacency and
  warm-start lookup, rigid AVBD contact snapshot/row-counter setup, contact
  projection solve rows, warm-start inventories, inertial targets, and
  no-contact fixed-joint rows.

Current Phase 5 no-growth/no-heap gates shipped by this PR include:

- Baked rigid, kinematic IPC, resting-contact, batched SoA rigid integration,
  multibody variational, compliant-contact, same-DOF cross-articulated, and
  mixed/different-DOF boxed-LCP fallback scenes.
- Production boxed-LCP contact sets up to 32 stacked multibodies, independent
  multi-island rigid/articulated contacts, stress multi-island contacts, and a
  mixed stress stack plus multi-island scene with 60+ initial contacts.
- Default-solver deformable guards for frictional self-contact patches,
  square/rectangular/irregular direct-sparse and matrix-free two-layer grids,
  late-active contact patterns, mixed direct/matrix-free roots, FEM
  ground-friction blocks, static sphere/box/capsule obstacle friction, moving
  rigid-surface CCD, and inter-body surface CCD.
- Mixed contact-family default-solver scenes that combine static-obstacle
  friction, self-contact friction, and production inter-body surface CCD under
  one baked World memory root for both direct-sparse and matrix-free pairings.
- AVBD ground-contact, self-contact, rectangular-row, rigid-contact, and
  no-contact fixed-joint guards with World-base no-growth and global-heap
  no-allocation coverage after bake.

Follow-up progress after PR #2956:

- The VBD Chebyshev self-contact path now borrows stage-owned Chebyshev history
  vectors, bakes self-contact sweep/adjacency capacity for all enabled
  VBD surface bodies, and has active-scene plus World-base/global-heap gates for
  a 5x9 two-layer self-contact grid.
- The VBD topology element lists for springs and tetrahedra now live in
  `DeformableVbdScratch` with the borrowed World allocator. The internal VBD
  kernels consume those read-only lists through `std::span`, so standalone
  one-shot callers keep ordinary vectors while baked World-stage steps avoid
  heap-owned spring/tet topology storage.
- Contact-heavy `WorldRegistry` rebuild coverage now reuses the existing
  compliant variational contact slider setup to bake/step solver-owned ECS
  scratch, clear the `World`, verify registry capacities release to zero while
  preserving the world free-list allocator wiring, rebuild the same contact
  shape, and re-step with identical baked storage capacities and no base
  allocator activity in the baked step loop.
- Scripted deformable Dirichlet/Neumann boundary processing now has a focused
  active-scene check plus World-base/global-heap baked-step allocation guards,
  proving the per-body boundary masks and external-acceleration scratch stay
  inside the baked deformable solver storage after prewarm.
- Variational multibody manifold Anderson acceleration now uses the baked
  `MultibodyVariationalScratch` component for its step/iterate history,
  per-joint tangent temporary, least-squares work matrices, and trial
  positions. The World-stage path pre-sizes this storage during
  `enterSimulationMode()`, while the public one-shot integration helper keeps a
  local fallback scratch. Focused coverage extends the existing loop-closure
  scratch bake test and keeps the manifold Anderson spherical/floating
  regression tests passing.
- Variational multibody articulated inverse-mass and exact recursive-Newton
  solves now share baked linear-solve scratch for articulated operators, spatial
  bias/twist buffers, joint projectors, factored joint blocks, right-hand sides,
  and result vectors. The World-stage bake path pre-sizes this storage with the
  multibody shape, the public helper keeps a local fallback, and focused
  coverage checks the baked scratch shape while preserving dense inverse-mass,
  long-chain exact-Newton, loop-closure, and manifold Anderson regressions.
- Variational multibody step state now reuses baked scratch for generalized
  position, velocity, applied force, next-position trial, bootstrap spatial
  velocities, and forced-DEL residual storage. The hot-loop residual evaluation
  writes into caller-owned storage instead of returning a fresh vector per
  root-find or line-search trial, and the initial-guess retraction reuses
  existing per-joint tangent storage. The stage also routes its initial-guess
  inverse-dynamics query through a reusable scratch-backed overload, so
  same-shape baked steps no longer use the public return-by-value dynamics
  helper or allocate a temporary zero-acceleration vector for that bias query.
  Loop-closure projection now also reuses baked Jacobian, inverse-mass
  transpose, constraint-mass, factorization, lambda, and correction storage
  plus projection row bounds/row partitions instead of allocating dense
  projection work matrices or temporary row lists on every projection
  iteration. The projection loop refreshes the step tree's configuration in
  place for each candidate instead of rebuilding the tree topology, index map,
  and child vectors from the registry on every projection iteration. The
  initial per-step variational tree build now also writes into baked
  `MultibodyVariationalScratch` storage, so same-shape steps reuse the tree
  topology vector, link-index map, per-link child lists, and link-frame
  subspace matrices instead of constructing fresh containers; the same-shape
  map nodes stay alive rather than being cleared/reallocated in the step loop.
  The follow-up branch now also constructs the baked tree scratch pimpl, link
  vector, per-link child lists, and link-index map from the World free
  allocator. The same follow-up line routes the nested inverse-dynamics scratch
  pimpl and dynamics-tree vector payloads through that allocator for baked
  variational stages. Baked variational ground-contact point scratch and
  augmented-Lagrangian solver dual scratch now also allocate their reusable
  vector storage from the World free allocator, as do contact-evaluation
  transform/Jacobian scratch vectors for the same baked contact scenes.
  Existing variational loop-closure baked storage now also routes the step
  constraint staging vector, step spatial-velocity list, articulated
  linear-solve vector lists, projection Jacobian/row-index lists, and Anderson
  history lists through the same World allocator root. The AL contact
  dual-update post-transform list now borrows that allocator too.
  Velocity-actuator projection now follows the same baked projection path:
  bake-time sizing counts actuator target rows, the projection loop writes those
  rows directly into the reusable residual/Jacobian, and per-joint projection
  retractions write through existing scratch instead of allocating a temporary
  constraint list or return-by-value retract result.
  The existing World baked-step global-heap gate now includes an active
  loop-closure chain, covering the variational tree/projection scratch path at
  World level without broadening production scene scope.
- The existing compact and production mixed default-deformable storage scenes
  now have World registry clear/rebuild gates. They prove direct-sparse
  self-contact, matrix-free self-contact, and FEM ground-friction solver
  storage can be baked, stepped without base-allocator growth, cleared back to
  zero registry capacity, and rebuilt with the same storage capacities under
  the world free-list allocator.
- The production mixed default contact-family scene and its complementary
  matrix-free/static-obstacle plus direct-irregular/self-contact pairing now
  have the same clear/rebuild coverage, adding static-obstacle friction,
  self-contact friction, and inter-body surface CCD solver storage to the
  rebuild-boundary proof across both default-solver storage pairings.
- Existing AVBD/VBD scenes now have the same `World::clear()` rebuild-boundary
  proof without adding new scene definitions: AVBD self-contact friction, VBD
  Chebyshev self-contact, and AVBD ground-friction storage bake, step without
  World-base allocator growth or ECS capacity changes, clear to zero registry
  capacity, and rebuild with the same storage shape.
- The existing variational loop-closure chain now has a matching clear/rebuild
  gate that asserts the baked `MultibodyVariationalScratch` tree, step,
  inverse-dynamics, linear-solve, projection, and Anderson storage dimensions,
  then rebuilds the same shape with identical registry capacities under the
  World allocator root.
- Existing boxed-LCP multibody contact scenes now have clear/rebuild coverage
  for semi-implicit multibody contact storage using stacked and multi-island
  fallback shapes. The gate bakes active contact rows, verifies same-shape
  steps do not grow the World base allocator or ECS capacities, clears the
  registry to zero capacity, and rebuilds with identical storage capacities.
- The existing kinematic IPC surface-CCD crossing scene now has a matching
  clear/rebuild gate for `KinematicBodyStepTrace` storage. It bakes the active
  kinematic trace path, verifies same-shape steps keep ECS capacities stable
  without World-base allocator growth, clears the registry to zero capacity,
  and rebuilds the same trace storage shape under the World free-list
  allocator.
- Existing rigid AVBD contact and fixed-joint scenes now cover the remaining
  rigid AVBD registry storage rebuild boundary. The gate proves
  `RigidAvbdContactConfig` and generated `AvbdRigidWorldPointJointConfig`
  storage can be baked, stepped without World-base allocator growth or ECS
  capacity changes, cleared to zero registry capacity, and rebuilt with the
  same storage capacities under the World free-list allocator.
- The existing contact-heavy variational dual-state setup now has matching
  clear/rebuild coverage for `VariationalContactDualState` storage. The gate
  bakes six sliders with four persistent contact duals each, verifies
  same-shape steps keep dual vector capacity and ECS storage capacity stable
  without World-base allocator growth, clears the registry to zero capacity,
  and rebuilds the same dual-state shape. The dual vector payload now borrows
  the World free allocator when baked through `enterSimulationMode()` or first
  created by the variational World stage, pre-existing/default-constructed
  dual-state components are rebound to that allocator before sizing, and the
  existing binary state serialization path accepts allocator-aware trivial
  vectors.
- The sibling `VariationalContact` persistent contact-point configuration now
  uses allocator-aware point-index and local-position vectors. Public
  `Multibody::setGroundContact()`/`addGroundContactPoint()` construction uses
  the World free allocator, and the variational World stage rebinds
  loaded/pre-existing contact configs before building baked contact scratch.
- `MultibodyVariationalState` two-step history now uses allocator-aware
  transform and momentum vectors. Bake-time and lazy World-stage creation use
  the World free allocator, loaded/pre-existing state is rebound without losing
  history, and binary state serialization now handles allocator-aware SE(3)
  transform and 6D momentum lists.
- Finite-stiffness AVBD point-joint compliant-loop scratch now follows the same
  variational multibody route. The private World-stage component constructs its
  compliant constraint list, axis-row vectors, row-descriptor staging lists, and
  scalar-row inventories from the World free allocator; `enterSimulationMode()`
  pre-sizes and synchronizes the row inventories for baked point-joint shapes;
  and the compliant contact hook reads that baked scratch directly instead of
  copying constraints into a default-heap vector.
- Pure compliant variational ground contact now bypasses
  `VariationalGroundContactSolver` and its default-heap dual vector entirely.
  Baked `MultibodyVariationalScratch` still owns the normalized ground-contact
  points plus contact-evaluation force/forcing vectors, while the
  augmented-Lagrangian path keeps solver-owned dual scratch only for positive
  `dualUpdateCadence` configurations. Existing compliant-contact World gates now
  assert the no-solver pure-compliant path, and the contact-heavy dual-state gate
  still asserts solver dual scratch for AL contact.
- `MultibodyVariationalTreeScratch` is now allocator-aware. The World stage sets
  it to the World free allocator before bake/step tree construction, so the
  pimpl object, link vector, per-link child lists, and link-index map belong to
  the same hierarchy as the surrounding variational scratch; one-shot helpers
  keep the default allocator fallback. The existing loop-closure clear/rebuild
  gate now asserts the baked scratch uses the World allocator.
- The semi-implicit one-slider multibody path now has the same clear/rebuild
  proof for baked private multibody dynamics storage. The gate covers the
  all-storage capacity map created by `reserveMultibodyDynamicsRegistryStorage`
  for the active `MultibodyDynamicsScratch`/`PendingMultibodyVelocity` path,
  then verifies same-shape steps do not grow World-base allocator counts or ECS
  capacities, clears to zero registry capacity, and rebuilds the same storage
  shape.
- The semi-implicit `MultibodyDynamicsScratch` follow-up now constructs its
  DART-owned dynamics-tree, link-index, RNEA, link-contact, constrained-DOF,
  constrained-target, contact-row, and body-Jacobian containers from the World
  free allocator when the component is bake-created or first needed by a World
  stage. Its external-force body-Jacobian branch now fills the baked
  body-Jacobian scratch instead of constructing a default-allocated local
  Jacobian vector during `world.step()`, and a focused semi-implicit
  external-force multibody gate covers both World-base no-growth and
  global-heap no-allocation after bake. The remaining heap allocation was the
  unconditional contact query in the split semi-implicit contact/unified stages
  for a world with no collision shapes, not the body-Jacobian multiply itself.
  Eigen-owned matrices/vectors remain tracked as the solver-private storage
  limitation called out below.
- `MemoryAllocatorDebugger` now records the requested alignment alongside live
  byte counts for aligned allocations and rejects mismatched aligned
  deallocations without forwarding them to the wrapped allocator, closing one
  remaining debug-accounting misuse gap. Its destructor also releases any
  still-tracked leaked allocations with the recorded size/alignment after
  reporting the leak.
- `docs/design/hierarchical_allocator.md` now reflects the implemented
  experimental DART 7 `World` hierarchy instead of the original proposal: the
  durable design note describes `WorldOptions`, the World-owned
  `MemoryManager`, allocator lifetime roles, registry bake/rebuild boundaries,
  and the direct evidence expected before making broader zero-allocation
  claims.
- The opaque `WorldStorage` object, private built-in step-pipeline cache,
  built-in stage-owned scratch/cache objects, lazy collision query cache, and
  optional replay controller object use the same World free-list allocator as
  the EnTT registry and differentiable-parameter storage. The focused
  `WorldPersistentStorageUsesWorldFreeAllocator` test verifies initial
  construction, built-in stage scratch construction, lazy collision-cache
  construction, lazy replay-controller construction, and `World::clear()`
  rebuilds keep persistent World state under the World memory hierarchy while
  dropping cached collision query state at the rebuild boundary. Replay frame
  payload vectors and nested stage scratch payload vectors remain governed by
  the existing same-shape no-growth/no-heap gates, not by this allocator-root
  ownership check.
- The first nested stage-scratch payload route covers
  `RigidBodyVelocityStage` force-batch vectors. When that stage borrows the
  World `MemoryManager`, its entity, force, and torque reserve/growth traffic
  uses the World free allocator; a focused heap-counter test verifies first
  `prepare()` does not allocate from the global heap. Other nested stage
  scratch payloads remain evidence-first follow-up work.
- The legacy graph-backed `RigidBodyIntegrationStage` can now borrow a
  `MemoryManager` as well. Its stage-owned rigid-body entity and dependency-node
  scratch vectors reserve from the provided free allocator, and its transient
  `ComputeGraph` uses the same allocator root for owned graph storage. A
  focused parent/child custom-stage test verifies the persistent scratch vector
  reserves use and release the provided free allocator.
- The batched SoA `BatchedRigidBodyIntegrationStage` can now borrow a
  `MemoryManager` for custom stage use. Its force-batch entity/force/torque
  vectors and parent-before-child frame-order/visit-state vectors reserve from
  the provided free allocator while the existing same-shape heap guard still
  covers the prewarmed frame-coupled path.
- `WorldStepPipeline` overflow stage-pointer storage can now borrow a provided
  allocator. The built-in World pipeline cache uses the World free allocator
  for that spillover path, and a focused custom-pipeline test verifies inline
  stages stay allocation-free while the first overflow reserve is charged to
  the provided allocator and released with the pipeline.
- The next nested route covers `RigidBodyContactStage`'s sequential-impulse
  constraint vector. A focused compact contact prepare verifies the vector's
  first reserve increases the World free-list allocation count. The AVBD
  contact scratch bundle is covered by the follow-up bullets below.
- The AVBD contact scratch follow-up routes that broader bundle's stage-owned
  private contact snapshot vectors, row-counter scratch, solve scratch vectors,
  warm-start inventories, and point-joint input vector through the borrowed
  allocator. A focused fixed-joint prepare verifies those first reserves
  increase an isolated provided free-list allocation count and release when the
  custom stage is destroyed.
- The large-row AVBD contact follow-up now routes generated scalar-row
  descriptors, motor active-row pointer lists, and distance-spring active-row
  pointer lists through allocator-backed reusable scratch instead of local
  default-heap vectors. `RigidBodyContactStage::AvbdScratch` also constructs
  distance-spring row inventory with the World allocator, and thresholded
  point-joint fracture-index result storage borrows the solve scratch allocator.
  Focused large-row/fracture builder tests verify the provided scratch allocator
  is used, and the existing rigid AVBD plus baked World no-heap gates still pass
  without adding new production scenes.
- The rigid IPC follow-up routes the stage's top-level runtime-body,
  solver-body, surface, dynamics-term, projected-Newton result,
  kinematic-trace, writeback-order, and resting-contact scratch vectors through
  the borrowed World free allocator. A focused IPC prepare test verifies those
  top-level and nested surface mesh vector reserves increase the provided
  free-list allocation count and release when the custom stage is destroyed.
  The projected-Newton solve scratch now also has allocator-aware construction
  for its surface work vectors, and the stage passes the same allocator into
  that nested solver scratch; a focused detail-solver test verifies those
  reserves use and release the provided free allocator. The same detail-solver
  path now constructs projected-Newton result assembly body-offset, active
  constraint, and active friction-constraint vectors with that allocator,
  preserves the destination allocator across repeated result assignments, and
  routes solve-internal barrier-assembly plus line-search surface-pair,
  sweep-item, candidate-pair, triplet, articulation equality-row, and equality
  change-of-variable index scratch
  through the same borrowed allocator. After
  the post-#2956 main merge added rigid/deformable mixed-domain candidate
  diagnostics, the same stage scratch now reuses allocator-backed BDF2 history,
  articulation-input, mixed-domain surface payload, candidate, edge, and AABB
  scratch; the focused custom-stage prepare gate covers a mixed
  rigid/deformable surface shape with zero global-heap allocation.
- The deformable stage scratch follow-up routes the stage-owned static-ground
  barrier, sphere/box/capsule obstacle, deformable surface-snapshot, static
  rigid surface-CCD snapshot, and moving rigid surface-CCD snapshot vectors
  through the borrowed World free allocator. Each snapshot's nested position,
  topology, contact-mask, and edge payload vectors now use that allocator as
  well. A focused custom-stage test covers those top-level and nested payload
  vectors against an isolated provided free allocator and verifies release when
  the stage is destroyed.
- Deformable self-contact and inter-body surface-contact candidate storage now
  has allocator-aware `ContactCandidateSet` and sweep scratch constructors.
  The World-stage `DeformableContactSolverScratch` and `DeformableVbdScratch`
  components build those candidate/sweep buffers from the World free allocator
  when the component is first created, while standalone reusable builders keep
  default construction for one-shot callers. The same
  `DeformableContactSolverScratch` route now covers the per-body surface
  topology/contact-mask storage and inter-body surface-CCD edge, sweep-item,
  and sweep-link buffers primed during bake. Focused contact-candidate tests
  verify provided allocator reserve/release, and existing World-base/global-heap
  baked-step guards still pass without adding a new production scene.
- `DeformableVbdScratch` now also constructs AVBD scalar-row inventories,
  descriptor metadata vectors, static-contact feature ID buffers, and friction
  warm-start lookup vectors from the World free allocator. The AVBD solve row
  arrays now use the same allocator-backed scratch after the mass-spring,
  half-space contact, attachment, self-contact friction, and tet finite-stiffness
  row kernels accepted allocator-aware vector contracts. The follow-up branch
  also routes the static contact-plane buffer and AVBD attachment fixed-mask
  scratch through the World allocator by
  narrowing the mixed deformable block-descent contact-plane contract to a
  read-only span and making the AVBD mass-spring row fixed-mask argument
  allocator-agnostic. Chebyshev history scratch for the mixed VBD driver now
  follows the same route: the driver preserves caller-provided scratch
  allocators while standalone one-shot callers keep default local storage. The
  spring/tet topology element lists now use the same allocator-backed
  `DeformableVbdScratch` route after narrowing read-only VBD topology inputs to
  spans. The cached VBD coloring plus spring, tetrahedron, and self-contact
  incident-adjacency builders now preserve caller-provided allocators for their
  nested vectors, so World-owned VBD topology scratch no longer falls back to
  default heap storage for those cached structures.
- `DeformableSolverScratch` now constructs inertial targets, iterate, gradient,
  direction, candidate, previous-step, external-acceleration, and active
  fixed/Dirichlet/Neumann mask buffers from the World free allocator. The
  default projected-Newton and VBD/AVBD driver contracts accept those
  allocator-backed vectors through spans or allocator-agnostic vector templates,
  so baked World-stage steps no longer fall back to default heap storage for
  those per-body solver buffers.
- Default deformable projected-Newton assembly scratch now borrows that same
  World free allocator for sparse-pattern arrays, triplet assembly, PSD
  edge/tet/barrier block batches, and matrix-free block/diagonal storage. The
  FEM rest-shape cache and lagged friction normal/contact arrays now use the
  same allocator-backed scratch component. The existing deformable stage
  allocator test now expects projected-Newton vectors to reserve from the
  provided allocator, and existing friction behavior plus baked
  World-base/global-heap gates still pass without adding scene coverage to this
  follow-up.
- The kinematics cache follow-up routes `WorldKinematicsGraph`'s entity-node
  lookup vector through the World free allocator when the graph is constructed
  by the built-in kinematics stage. A focused stack-constructed graph test
  verifies that entity-node cache storage increases the World free-list live
  allocation count and releases it when the graph is destroyed.
- The `ComputeGraph` follow-up adds allocator-aware graph construction and
  routes owned `ComputeNode` objects, the node-name lookup table, the
  dependency-edge vector, and the topological-order cache through the supplied
  allocator. Read-only edge/order accessors now return span views so those
  allocator-backed containers stay private while existing range-iteration call
  sites remain source-compatible. A focused compute-graph test verifies node,
  lookup, edge, and order storage use the provided World free allocator and
  release it on graph destruction. The current continuation extends that
  allocator contract to traversal scratch: cycle detection, topological-order
  rebuild, validation, and no-hazard resource scans no longer allocate from the
  global heap for allocator-aware graphs.
- The unified constraint stage now constructs its private multibody entity,
  link-contact bucket, required-block marker, staged-contact, dynamics-scratch
  pointer, and staged-velocity vectors with the borrowed World free allocator.
  The follow-up also routes the stage-owned rigid contact problem, unified row
  owner/rigid-constraint/multibody-block containers, nested multibody block row
  storage, and boxed-LCP solve scratch traversal/fallback/tangent vectors
  through the same allocator root.
  A focused stacked boxed-LCP prepare test verifies those stage-owned reserves
  use and release a provided free-list allocator.
- The pure rigid `ContactSolverMethod::BoxedLcp` branch now keeps reusable
  stage-owned scratch for contact normals, body-column lookup, dense Delassus
  work matrices/vectors, and Dantzig solver buffers. `RigidBodyContactStage`
  prewarms that scratch during `prepare()`, and the baked rigid sphere-ground
  boxed-LCP gate now covers both World-base no-growth and global-heap
  no-allocation behavior.
- The rigid contact stage now skips contact-query prepare/execute work when the
  only rigid/link collision-geometry components are empty. A baked global-heap
  regression covers that loaded/pre-existing ECS-storage shape without changing
  the existing shape-backed rigid, link, AVBD, and boxed-LCP contact gates.
- The dynamic rigid IPC dynamics-only path bypasses the full projected-Newton
  assembly for the exact single supported dynamic body with no possible contact
  or articulation pairs. The active rigid IPC mesh-barrier path also executes
  the one-node projected-Newton solve directly instead of rebuilding an
  internal `ComputeGraph` each step, and `prepare()` prewarms reusable
  projected-Newton step/result row storage from the current surface topology.
  The latest stack/articulation slice makes the projected-Newton solve scratch
  own persistent assembly/line-search buffers from the World allocator, and the
  focused baked gate now covers the contact-free IPC update, active
  static/dynamic mesh barrier, fixed/revolute articulated constraints, and a
  two-box stack with zero global heap allocation after bake. Broader rigid IPC
  contact shapes remain evidence-first follow-up work.

Remaining Phase 4/5 follow-up items for the next PR:

- Do not add more production scenes or scratch-reuse work to PR #2956; continue
  any remaining no-growth and scratch work on a new follow-up branch.
- Continue projected-Newton deformable scratch reuse only where profiling or a
  no-growth gate exposes a real allocation path, especially solver-private
  storage that still cannot borrow the World allocator directly
  (`Eigen::SparseMatrix`/`VectorXd` internals), and differently shaped
  frictional self-contact, static-obstacle, or inter-body CCD mixes not
  represented by the current gates.
- Continue broader rigid IPC projected-Newton scratch reuse only from measured
  failing shapes beyond the current contact-free and active two-mesh-barrier
  gates, such as larger mesh contact sets, articulated IPC constraints, or
  mixed rigid/deformable barrier solves that expose new per-step allocator
  growth. The dynamic rigid IPC two-box stack gap is now covered by the baked
  no-heap gate, so the next rigid IPC expansion should start from a newly
  measured failing shape rather than reusing that stack case as open work.
- Add any remaining default-solver deformable storage/no-heap gates for
  solver-private paths not exercised by the current direct-sparse,
  matrix-free, FEM, obstacle, surface-CCD, and compact/production
  clear/rebuild mixed/contact-family scenes.
- Expand production contact-set coverage only for newly exposed boxed-LCP or
  unified-assembly shapes that are not covered by the current stacked,
  multi-island, mixed-stress, and contact-family gates.
- Continue production `WorldRegistry` bake/build sizing guidance beyond the
  current compliant-contact allocator-aware config/dual-state/history
  clear/rebuild gate and the newly allocator-backed point-joint loop-scratch
  bake route, especially for differently shaped solver-owned ECS storages and
  rebuild boundaries.
- Re-run allocator comparative evidence when allocator, STL, or frame policy
  changes; keep the current foonathan/memory and standard-baseline evidence
  green instead of adding allocator-policy work to this PR.

## Immediate Next Steps

1. Continue promoting the benchmark-only EnTT storage policies toward
   production `WorldRegistry` bake/build guidance and integration: free-list
   backed world-lifetime storage for reserved no-growth arrays, and resettable
   rebuild-lifetime frame storage for bake/growth. Production wiring now resets
   registry storage on `World::clear()` rebuild boundaries, and the follow-up
   branch adds compliant-contact dual-state, compact, production, mixed and
   complementary contact-family default-deformable, AVBD/VBD, boxed-LCP
   multibody contact, semi-implicit multibody dynamics, kinematic IPC
   surface-CCD trace, rigid AVBD contact/joint, and variational loop-closure
   clear/rebuild no-growth gates for baked solver scratch plus baked
   variational Anderson, step-state, and
   linear-solve/inverse-dynamics/projection work storage. Broader bake/build
   sizing guidance for other solver-owned ECS storage shapes remains open; it
   must not use the per-step frame scratch allocator that resets inside
   `World::step()`.
2. Continue extending allocator correctness tests for remaining
   leak/debug-accounting gaps and production workload cases after the new
   count/size overflow, over-alignment, aligned debug-deallocation,
   debug-leak release, reuse, diagnostics, and bounded-failure coverage for
   `MemoryAllocator`, `MemoryAllocatorDebugger`, `FixedPoolAllocator`,
   `PoolAllocator`, `FreeListAllocator`, `FrameAllocator`,
   `FrameStlAllocator`, `StlAllocator`, `MemoryManager`, and experimental
   `World` allocator-root isolation.
3. Keep the strict allocator comparative checker green as allocator policy
   changes land, and extend it to any remaining foonathan/memory baselines that
   map to HMM allocator roles. The current foonathan-plus-standard-plus-EnTT
   matrix covers pools, stack/frame, static/temporary/iteration, raw
   heap/malloc/new, aligned/fallback/segregator adapters, tracked/deeply tracked
   wrappers, and EnTT no-growth/build rows; a CPU-affined full run plus focused
   replacement rows passed all 94 foonathan and standard comparisons on
   2026-06-06. After the latest frame/STL allocator and steady-state benchmark
   policy changes, a 2026-06-07 foonathan-only run plus focused strict-CV row
   replacements passed all 47 foonathan comparisons in
   `.benchmark_results/allocator_comparative_current_foonathan_entt_cpu16_merged_check.json`.
   After the latest STL allocator hook/default-adapter change, the merged
   foonathan broad-plus-EnTT input
   `.benchmark_results/allocator_foonathan_broad_entt_current_check.json`
   passes all 47 foonathan comparisons with either non-noisy rows or
   confidence-separated high-CV rows.
   Later random-interleaved EnTT diagnostics on the same branch found that the
   pool-backed no-growth row could miss foonathan/memory at 256/512 entities,
   while frame-backed and default-backed probes did not robustly close the gap.
   A follow-up switched the no-growth row to free-list-backed world-lifetime
   storage, matching reserved variable-size registry arrays, and passed all 12
   EnTT comparisons in
   `.benchmark_results/allocator_entt_freelist_nogrowth_frame_build_current_merged_check.json`.
   A later 2026-06-07/08 probe kept `StlAllocator` storage at natural alignment
   for non-overaligned values and added cache-friendly default `PoolAllocator`
   strides for medium power-of-two slots. This is scoped to allocator
   contracts: STL storage only requests natural alignment, while default pool
   requests carry no over-alignment promise and can use non-power-of-two size
   classes to reduce cache-set conflicts. The same continuation moved the EnTT
   build/growth DART row back to the resettable frame-backed bake arena; the
   CPU-pinned build/growth probe in
   `.benchmark_results/allocator_entt_build_frame_bake_current_cpuauto_probe.json`
   beat foonathan/memory clearly and beat the standard registry by median, but
   the 256/512 standard rows were not confidence-separated because the standard
   rows were slightly above the strict CV gate. Earlier no-growth probes,
   including
   `.benchmark_results/allocator_entt_nogrowth_pool_stride_current_cpu12_probe.json`
   and
   `.benchmark_results/allocator_entt_nogrowth_pool_stride_current_cpu12_warm_probe.json`,
   were rejected as noisy and are superseded by the free-list storage result.
   The default `PoolAllocator` heap-index lookup table then fixed the remaining
   mixed-size `BM_MultiPool` and `BM_Realistic` foonathan misses. The current
   merged broad-plus-focused result
   `.benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json`
   passes all 94 foonathan/memory and standard comparisons. Keep this combined
   gate green after allocator or benchmark policy changes:

   A 2026-06-08 follow-up added runner-side CPU prewarm and cache-line coloring
   for large 64-byte-aligned `FreeListAllocator` storage allocations. Direct
   focused probes improved the weak EnTT no-growth 512 row; the current
   persistent policy keeps free-list registry storage on eight 64-byte colors.
   Later frame-bake experiments showed that coloring ordinary one-shot
   `FrameStlAllocator` scalar arrays can move wins between foonathan and
   standard rows without a stable theoretical advantage. The retained frame
   policy keeps scalar arrays on the dense 32-byte frame fast path and reserves
   cache-line alignment for cache-line-sized or over-aligned value pages. The
   best current focused checker evidence for this retained policy is the merged
   CPU12 run in
   `.benchmark_results/allocator_entt_frame_dense_retained_cpu12_reps9_merged.json`:
   all 12 EnTT no-growth/build comparisons pass against foonathan/memory and
   standard baselines, and the no-growth DART rows report zero post-prewarm
   allocator calls. The base run had two noisy baseline rows, so
   `.benchmark_results/allocator_entt_frame_dense_retained_replacements_cpu12_reps9.json`
   replaces only `BM_EnttRegistryBuild_Foonathan/512` and
   `BM_EnttRegistryBuild_Std/256` before the strict checker is re-run over the
   merged artifact.
   Follow-up free-list color stride experiments are intentionally discarded:
   they improved selected 256/512 rows but failed different foonathan or
   standard rows across CPUs, which is cache-layout overfitting rather than
   evidence for a general allocator policy.

   ```bash
   pixi run bm-allocator-comparative-check \
     --include-entt-registry --baseline foonathan --baseline std --verbose \
     --cpu-affinity auto
   ```

4. Repeat the focused EnTT allocator benchmark when registry allocation policy
   changes, then promote the cached-storage policy into production
   `WorldRegistry` bake/build guidance only after the production integration
   has matching no-growth tests. Keep using the registry-only checker for
   focused allocator-policy loops:

   ```bash
   pixi run bm-allocator-comparative-check --only-entt-registry \
     --baseline foonathan --baseline std
   ```

   The current benchmark policy uses cached component storage handles and the
   free-list-backed `StlAllocator` role used by production `WorldRegistry`
   storage for persistent no-growth churn. Build/growth is a separate one-shot
   storage-construction role: DART uses `StlAllocator` over a resettable
   `FrameAllocator` bake arena, and foonathan/memory uses `memory_stack` with
   marker/unwind bulk lifetime. This is benchmark evidence; production
   integration still needs matching no-growth tests and lifetime diagnostics for
   broader rebuild paths.

5. Extend bake-time registry/component storage reservation and no-growth
   allocation tests to remaining solver scratch step paths. The rigid-body,
   non-cross articulated, same-DOF sequential cross-articulated, and boxed-LCP
   mixed/different-DOF, stacked, coupled multi-row, larger stacked, extended,
   dense production, extra-dense production, and stress production stacked
   cross-articulated guards plus disconnected multi-island mixed
   rigid/articulated contact guards, including a mixed stress stack plus
   multi-island scene with 60+ initial contacts, now cover World base-allocator
   growth and first baked-step global heap allocation by priming unified
   constraint scratch at `enterSimulationMode()`. The current
   deformable friction guard
   scales the same topology-reserved candidate/friction scratch, including
   swept-AABB line-search CCD capacity, from patch, 5x5, 7x7, and 9x9 grids to
   active 11x11, 13x13, and 15x15 square grids, a matrix-free 17x17 square grid,
   plus direct-sparse and matrix-free 13x19, 9x13, 7x17, and 17x7 non-square
   two-layer grids.
   Late-active 11x11 square and 9x13 rectangular
   direct-sparse and matrix-free grids now cover dynamic contact-pattern cases
   without World-base or global-heap growth, and a mixed late-active
   production scene now covers independent direct-sparse square and matrix-free
   rectangular self-contact grids that activate during the same baked loop. A
   mixed dense production scene also covers a notched direct-sparse 13x17 grid
   and matrix-free 13x19 dense rectangular grid sharing one baked World memory
   root; a notched, jittered matrix-free 13x17 irregular grid now covers the CG
   scratch path on non-grid topology; compact and production mixed
   default-solver storage scenes now combine direct-sparse, matrix-free, and
   FEM ground-friction deformables under the same root; a mixed production FEM
   scene now combines direct and matrix-free 4x4x4-node ground-friction blocks
   under one baked root; a production rectangular inter-body deformable
   surface-CCD crossing now exercises
   inter-body sweep/candidate scratch beyond the tiny two-triangle crossing;
   and a production static-obstacle friction patch now covers shared
   sphere/box/capsule normal-force, normal-direction, and Hessian scratch under
   sparse and matrix-free no-growth guards. A mixed production scene now
   combines sparse static-obstacle friction and matrix-free self-contact bodies
   under one baked World root; the complementary matrix-free static-obstacle
   plus direct irregular self-contact scene covers the opposite solver pairing.
   A mixed default contact-family production scene now combines direct static
   obstacle friction, matrix-free self-contact friction, and inter-body surface
   CCD under the same baked World root, with a complementary contact-family
   scene covering matrix-free static-obstacle friction, direct irregular
   self-contact, and production inter-body surface CCD in one root.
   The AVBD self-contact row guard now also covers 5x9 and 9x13 rectangular
   grid row workloads with replay-backed activity assertions.
   Continue broadening boxed-LCP unified problem assembly only for newly
   exposed contact shapes, and keep moving any newly exposed deformable/contact
   candidate buffers to backed storage before making the full zero-allocation
   claim.
6. Start replacing per-step `std::vector`/`Eigen` temporaries in hot stages with
   world-frame or world-pool backed storage only after the allocator evidence
   gate proves the DART allocator path is better for that workload. The
   non-owning `WorldStepPipeline` stage list is already inline; focus next on
   solver-owned scratch and contact/deformable candidate buffers.
