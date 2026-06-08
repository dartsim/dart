# Hierarchical Memory Manager — Dev Task

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
      world-lifetime DART storage for persistent no-growth churn, and uses a
      resettable frame-backed bake arena for build/growth. The no-growth row
      reports post-prewarm allocator-call counters and fails if reserved churn
      asks the configured allocator for storage after prewarm; the
      build/growth row times the uninstrumented frame-backed path and reports
      configured-allocator call counters from a matching untimed probe. The
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
      experimental `World` instances, and keeps fixed-capacity free-list
      aligned-allocation diagnostics on user-requested bytes instead of internal
      header padding. `FrameStlAllocator` blocks are cache-line aligned without
      per-block cache coloring, so frame-backed STL pages used by
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
      preserving the World memory hierarchy. Broader solver scratch coverage
      remains open.
- [ ] Phase 4: Built-in simulation stages borrow world memory for transient
      buffers and avoid growth after simulation is baked. The default
      `WorldStepPipeline` now stores built-in non-owning stage pointers inline,
      removing its per-build `std::vector` allocation from the normal step path.
      Longer custom pipelines keep the previous arbitrary-stage behavior through
      an overflow path. The default rigid-body velocity/contact stages and
      semi-implicit multibody velocity/contact path now reuse baked scratch for
      the covered rigid and articulated resting-contact scenes. The boxed-LCP
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
      remapping/sub-problem buffers, normal-only fallback buffers, and fallback
      tangent accumulators. Same-shape no-heap coverage now includes unified
      island solves. The unified assembler now reuses same-shape link-block row
      storage without per-row Eigen matrix-vector temporaries; same-shape
      no-heap coverage now also includes mixed rigid plus borrowed-link unified
      assembly. The boxed-LCP stage borrows per-multibody contact problems from
      persistent `MultibodyDynamicsScratch` instead of copying them into
      staging containers first. The rigid-contact assembler now has an in-place
      scratch overload, so same-shape fallback steps reuse the stage-owned
      rigid contact problem instead of building a by-value temporary each step.
      `UnifiedConstraintStage::prepare()` now primes the initial boxed-LCP
      contact shape during `enterSimulationMode()`, moving the current World
      fallback scenes' first active solve allocation out of the step loop.
      Public multibody link-contact assembly now has reusable scratch storage
      that can be borrowed by the in-place unified assembler without same-shape
      heap growth. The no-scratch public boxed-LCP solve wrapper now moves the
      solved lambda vector out of its local scratch instead of allocating a
      second result copy, and callers that want a `UnifiedConstraintSolution`
      object can now pass caller-owned result storage alongside solve scratch
      for same-shape no-growth solves. `DeformableDynamicsStage` now owns
      reusable obstacle-list, deformable surface-snapshot, and rigid
      surface-CCD snapshot scratch, and `prepare()` primes per-body
      surface-contact candidate buffers plus inter-body/rigid surface-CCD sweep
      buffers for baked steps. Default deformable projected-Newton friction
      now reuses per-body normal-force, normal-direction, and self-contact
      friction-contact buffers instead of allocating local `std::vector`
      scratch in the step loop, and static-ground box CCD footprint clipping now
      uses fixed-size stack storage instead of allocating a tiny footprint
      vector on every non-vertical sweep. Surface-contact candidate and sweep
      buffers now get topology-scaled bake-time reserve capacity, so the covered
      frictional self-contact patch, 5x5 two-layer grid, and 7x7 two-layer
      large grid reuse candidate and friction-contact storage through
      projected-Newton line-search CCD. The current production boxed-LCP stage
      uses the in-place unified assembler and solve scratch; the public
      return-by-value unified problem and solution wrappers remain
      allocation-boundary API conveniences rather than step-loop hot paths.
- [ ] Phase 5: Add allocation/debug accounting gates for "no dynamic allocation
      during the step loop" on representative rigid, multibody, contact, and
      deformable scenes. World base-allocator no-growth guards now cover baked
      kinematic IPC rigid-body, multibody variational, deformable ECS,
      rigid-body resting-contact, non-cross articulated resting-contact, and
      same-DOF cross-articulated link-contact paths after contact prewarm; a
      global heap guard covers the same baked kinematic IPC, rigid-body,
      multibody variational, deformable, rigid-body resting-contact,
      non-cross articulated resting-contact, and same-DOF cross-articulated
      contact paths. Mixed/different-DOF, stacked, and coupled multi-row
      cross-contact boxed-LCP fallback scenes now have base-allocator
      no-growth gates and first baked-step global heap no-allocation gates.
      Five-multibody and eight-multibody stacked contact sets extend the
      boxed-LCP fallback gate beyond the original small scenes. Broader solver
      coverage, including still-larger production contact sets and
      default-solver deformable storage, remains open before making a full
      zero-allocation claim. The
      global heap guard now also covers a baked deformable surface-snapshot
      scene with a static rigid surface-CCD obstacle and first-baked-step active
      VBD static rigid surface-CCD point crossing. Default projected-Newton
      deformable scratch now reuses its RHS, sparse Hessian assembly, PSD block
      batches, sparse-pattern cache, and solution storage for the covered
      mass-spring path, and default static rigid surface-CCD point crossing is
      also covered by the first-baked-step global heap guard. FEM rest-shape
      caches are primed during `enterSimulationMode()`, and a one-tetrahedron
      FEM projected-Newton path is covered by the same guard; broader
      projected-Newton self-contact barrier scratch is sized from bake-primed
      contact candidates and covered for the two-triangle no-friction
      self-contact path. The global heap guard now also covers a baked
      default-solver deformable ground-friction projected-Newton scene plus a
      multi-triangle frictional self-contact patch, a 5x5 two-layer frictional
      self-contact grid, and a 7x7 two-layer large grid. Still-larger
      production-scale frictional deformable sets still need no-growth gates
      before making the full deformable claim.
- [ ] Phase 6: Add memory-layout profiler/debugger surfaces and GUI
      visualization. `MemoryAllocatorDebugger` now exposes structured live
      bytes, peak live bytes, and live allocation count; `MemoryManager` and
      DART 7 `World` diagnostics now surface direct free/pool allocator
      debug counters. `WorldMemoryDiagnostics` also reports aggregate and
      per-storage ECS registry layout counters without exposing EnTT types, and
      dartpy exposes the same read-only snapshot through
      `World.memory_diagnostics`. GUI visualization remains future work.

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

## Immediate Next Steps

1. Continue promoting the benchmark-only EnTT storage policies toward
   production `WorldRegistry` bake/build guidance and integration: free-list
   backed world-lifetime storage for reserved no-growth arrays, and resettable
   rebuild-lifetime frame storage for bake/growth. Production wiring now resets
   registry storage on `World::clear()` rebuild boundaries, but still needs
   broader bake/build sizing guidance and contact-heavy no-growth tests; it
   must not use the per-step frame scratch allocator that resets inside
   `World::step()`.
2. Continue extending allocator correctness tests for remaining
   leak/debug-accounting gaps and production workload cases after the new
   count/size overflow, over-alignment, reuse, diagnostics, and bounded-failure
   coverage for `MemoryAllocator`, `FixedPoolAllocator`, `PoolAllocator`,
   `FreeListAllocator`, `FrameAllocator`, `FrameStlAllocator`, `StlAllocator`,
   `MemoryManager`, and experimental `World` allocator-root isolation.
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

   The current benchmark policy uses cached component storage handles,
   free-list-backed world-lifetime DART storage for persistent no-growth churn,
   and a resettable frame-backed DART bake arena for build/growth churn. This is
   benchmark evidence rather than production `WorldRegistry` wiring; production
   integration needs matching no-growth tests and lifetime diagnostics.

5. Extend bake-time registry/component storage reservation and no-growth
   allocation tests to remaining solver scratch step paths. The rigid-body,
   non-cross articulated, same-DOF sequential cross-articulated, and boxed-LCP
   mixed/different-DOF, stacked, coupled multi-row, larger stacked, and
   extended stacked cross-articulated guards now cover World base-allocator
   growth and first baked-step global heap allocation by priming unified
   constraint scratch at `enterSimulationMode()`. Continue broadening
   boxed-LCP unified problem assembly and production-scale contact sets while
   moving still-larger frictional deformable/contact candidate buffers to
   backed storage before making the full zero-allocation claim.
6. Start replacing per-step `std::vector`/`Eigen` temporaries in hot stages with
   world-frame or world-pool backed storage only after the allocator evidence
   gate proves the DART allocator path is better for that workload. The
   non-owning `WorldStepPipeline` stage list is already inline; focus next on
   solver-owned scratch and contact/deformable candidate buffers.
