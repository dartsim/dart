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
      alignment-aware, including fixed-pool-backed max-aligned values. The
      broader correctness matrix and standard-library registry allocator
      evidence still need to land before this phase is complete. Focused EnTT
      registry probes now cover foonathan/memory's array-capable pool baseline
      and the standard registry, including separate build/growth rows for
      bake-time registry storage allocation. The focused EnTT checker caches
      known component storage handles, uses free-list-backed DART storage for
      persistent no-growth churn, and uses pool-backed DART storage for
      bake/build growth. The build/growth row times the uninstrumented
      pool-backed path and reports configured-allocator call counters from a
      matching untimed probe. The strict checker prints DART benchmark counters
      alongside pass/fail ratios, so EnTT misses distinguish timing loss,
      allocator traffic, and noisy benchmark evidence. A fresh focused strict
      run on 2026-06-04 still fails the foonathan/memory and standard-registry
      timing gate even though the prewarmed DART rows report zero configured
      allocator calls. Phase 2 remains open for allocator-policy optimization,
      the broader correctness matrix, and production workload gates.
- [ ] Phase 3: EnTT registry/component storage allocation is configurable from
      the World memory hierarchy and covered by no-growth ECS tests.
      Allocator-aware EnTT storage now has focused `StlAllocator` and
      `FrameStlAllocator` unit tests showing that reserved
      create/emplace/read/destroy churn makes no configured allocator calls or
      arena growth after the prewarm pass. The DART comparative EnTT benchmark
      row also reports configured-allocator allocation/deallocation counters
      and fails if reserved churn calls that allocator after prewarm. The
      benchmark hot path caches known component storage handles so the timing
      surface matches optimized World systems rather than repeated registry
      type lookup. Separate EnTT build/growth rows measure the bake-time
      storage allocation phase instead of conflating that cost with the
      no-growth simulation loop. The comparative benchmark now discovers
      installed EnTT package metadata for these rows without invoking DART's
      FetchContent-backed dependency helper, and the checker reports a clear
      error if a requested result file is empty.
- [ ] Phase 4: Built-in simulation stages borrow world memory for transient
      buffers and avoid growth after simulation is baked.
- [ ] Phase 5: Add allocation/debug accounting gates for "no dynamic allocation
      during the step loop" on representative rigid, multibody, contact, and
      deformable scenes.
- [ ] Phase 6: Add memory-layout profiler/debugger surfaces and GUI
      visualization.

## Goal

Move the experimental simulation stack toward zero dynamic memory allocation in
the simulation loop by giving each `World` one hierarchical allocator root and
routing persistent and per-frame data through that root. Long term, the memory
hierarchy should map cleanly to world objects/components so it can drive memory
debugging, profiling, optimization experiments, and ImGui visualization.

## Non-Goals For The First Slice

- Do not claim zero allocations for every experimental solver path yet.
- Do not replace Eigen's internal heap behavior in one pass.
- Do not expose ECS storage, solver registries, or backend resource types in
  public memory diagnostics.

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
- Broad hot-loop adoption is blocked until allocator correctness tests and
  benchmarks prove DART's allocators beat both standard C++ allocators and
  foonathan/memory on DART-relevant workloads. If DART cannot beat
  foonathan/memory for required features and workloads, prefer an explicit
  dependency decision over shipping a weaker in-house allocator.
- Use `FixedPoolAllocator` for fixed-size node/slot workloads and keep
  `PoolAllocator` focused on mixed size-classed small-object workloads. The
  comparative benchmark must not compare DART's generic size-classed pool
  against foonathan's fixed-size pool when a DART fixed-size allocator exists.

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

## Required EnTT Integration Evidence

- Registry construction: prove whether the active EnTT version can use a
  stateful allocator through `entt::basic_registry` and component
  `basic_storage`, or document the exact adapter/storage work needed.
- Component storage: cover create/emplace/destroy/clear/reuse patterns for the
  current experimental components, including sparse-set growth and component
  array capacity behavior.
- Bake/build reserve path: reserve registry entities and component pools before
  simulation so repeated `World::step()` calls do not grow ECS storage.
- No-growth proof: after the reserve/prewarm path, representative
  create/emplace/read/destroy churn must make zero calls through the configured
  DART allocator; benchmark timing alone is not sufficient evidence.
- Boundary rule: keep EnTT allocator/storage types out of the promoted public
  facade; expose only DART-owned memory options and diagnostics.

## Immediate Next Steps

1. Optimize the allocator-aware EnTT registry policy until the focused strict
   gate beats both foonathan/memory and the standard registry. The latest local
   strict run still loses the no-growth registry rows at 256, 512, and 2048
   entities despite zero configured allocator calls after prewarm, and the
   build/growth row loses the standard-registry baseline at 256 and 512
   entities.
2. Extend allocator correctness tests for `FixedPoolAllocator` and the existing
   pool/free-list/frame allocators across invalid sizes, over-alignment,
   overflow, reuse-after-free, leak/debug accounting, and bounded failure.
3. Repeat the focused EnTT allocator benchmark on low-load machines as needed,
   then promote the cached-storage policy into production `WorldRegistry`
   bake/build guidance only after the production integration has matching
   no-growth tests. Keep using the registry-only checker for focused
   allocator-policy loops:

   ```bash
   pixi run bm-allocator-comparative-check --only-entt-registry \
     --baseline foonathan --baseline std
   ```

   The current benchmark policy uses cached component storage handles,
   free-list-backed DART storage for persistent no-growth churn, and pool-backed
   DART storage for build/growth churn. This is benchmark evidence rather than
   production `WorldRegistry` wiring. A separate `FrameStlAllocator` no-growth
   unit test models persistent world registry storage allocated during
   bake/build and reset only on rebuild/destruction; use it as correctness
   evidence for an arena policy, not as a passing timing claim.

4. Start replacing per-step `std::vector`/`Eigen` temporaries in hot stages with
   world-frame or world-pool backed storage only after the allocator evidence
   gate proves the DART allocator path is better for that workload.
