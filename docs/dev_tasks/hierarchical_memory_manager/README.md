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
      caches known component storage handles, uses frame-backed DART storage
      for persistent no-growth churn, and uses pool-backed DART storage for
      bake/build growth. The no-growth row reports frame usage and overflow
      counters; the build/growth row times the uninstrumented pool-backed path
      and reports configured-allocator call counters from a matching untimed
      probe. The strict checker prints DART benchmark counters alongside
      pass/fail ratios, so EnTT misses distinguish timing loss, allocator
      traffic, growth, and noisy benchmark evidence.
      `FreeListAllocator` now has a fixed-capacity mode for
      deterministic bounded failure after preallocation, and `MemoryManager` /
      experimental `WorldOptions` can construct the World free-list hierarchy
      with a fixed-capacity policy. Fixed-capacity free-list arenas can also
      satisfy over-aligned pool chunks from reserved bytes without growing from
      the base allocator. A 2026-06-04 focused EnTT run passed the
      foonathan/memory and standard-registry timing gate, but the current pushed
      head needs a clean low-load rerun or optimization before #2890 is
      merge-ready. Phase 2 remains open for the broader correctness matrix and
      production workload gates.
- [ ] Phase 3: EnTT registry/component storage allocation is configurable from
      the World memory hierarchy and covered by no-growth ECS tests.
      Allocator-aware EnTT storage now has focused `StlAllocator` and
      `FrameStlAllocator` unit tests showing that reserved
      create/emplace/read/destroy churn makes no configured allocator calls or
      arena growth after the prewarm pass. The DART comparative no-growth EnTT
      row now uses the same world-lifetime arena policy and fails if reserved
      churn grows frame-backed storage or spills to overflow after prewarm. The
      benchmark hot path caches known component storage handles so the timing
      surface matches optimized World systems rather than repeated registry
      type lookup. Separate EnTT build/growth rows measure the bake-time
      storage allocation phase instead of conflating that cost with the
      no-growth simulation loop. The comparative benchmark now discovers
      installed EnTT package metadata for these rows without invoking DART's
      FetchContent-backed dependency helper, and the checker reports a clear
      error if a requested result file is empty.
      Initial registry/component-storage wiring and `enterSimulationMode()`
      reservation/no-growth tests for current World-owned ECS storage and the
      first multibody/deformable private step-scratch components are
      implemented. Direct EnTT create/emplace/clear/destroy/reuse storage
      cycling is covered after explicit reserve; broader solver scratch
      coverage remains open.
- [ ] Phase 4: Built-in simulation stages borrow world memory for transient
      buffers and avoid growth after simulation is baked. The default
      `WorldStepPipeline` now stores built-in non-owning stage pointers inline,
      removing its per-build `std::vector` allocation from the normal step path.
      Longer custom pipelines keep the previous arbitrary-stage behavior through
      an overflow path. The default rigid-body velocity/contact stages and
      semi-implicit multibody velocity/contact path now reuse baked scratch for
      the covered rigid and articulated resting-contact scenes; broader
      solver-owned transient buffers still need allocator-backed storage.
- [ ] Phase 5: Add allocation/debug accounting gates for "no dynamic allocation
      during the step loop" on representative rigid, multibody, contact, and
      deformable scenes. Initial World base-allocator no-growth guards now
      cover baked kinematic IPC rigid-body, multibody variational, and
      deformable ECS paths; a first global heap guard now covers baked
      kinematic IPC rigid-body, box-obstacle, multibody variational,
      deformable, rigid-body resting-contact, and non-cross articulated
      resting-contact steps. The default sequential articulated contact path
      also covers a same-DOF cross-multibody link contact scene without global
      heap allocation, while mixed/different-DOF and stacked cross-contact
      scenes stay on the boxed-LCP fallback. Broader solver coverage, including
      boxed-LCP unified contact assembly, larger contact sets, and remaining
      solver-owned scratch, remains open before making a full zero-allocation
      claim.
- [ ] Phase 6: Add memory-layout profiler/debugger surfaces and GUI
      visualization. `MemoryAllocatorDebugger` now exposes structured live
      bytes, peak live bytes, and live allocation count; `MemoryManager` and
      experimental `World` diagnostics now surface direct free/pool allocator
      debug counters. `WorldMemoryDiagnostics` also reports aggregate and
      per-storage ECS registry layout counters without exposing EnTT types, and
      dartpy exposes the same read-only snapshot through
      `World.memory_diagnostics`. GUI visualization remains future work.

## Goal

Move the experimental simulation stack toward zero dynamic memory allocation in
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
  `entt::basic_registry`: the experimental World now constructs its internal
  registry and component storages with a `dart::common::StlAllocator` borrowing
  the World's active free allocator.
- Free-list allocations must preserve at least `std::max_align_t` alignment
  after every split. EnTT component storage surfaced this as an Eigen
  `FrameCache` alignment failure when a max-aligned component allocation
  followed an odd-sized allocation.
- Broad hot-loop adoption is blocked until allocator correctness tests and
  benchmarks prove DART's allocators beat both standard C++ allocators and
  foonathan/memory on DART-relevant workloads. If DART cannot beat
  foonathan/memory for required features and workloads, prefer an explicit
  dependency decision over shipping a weaker in-house allocator.
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

## Required EnTT Integration Evidence

- Registry construction: prove whether the active EnTT version can use a
  stateful allocator through `entt::basic_registry` and component
  `basic_storage`, or document the exact adapter/storage work needed.
- Component storage: cover create/emplace/destroy/clear/reuse patterns for the
  current experimental components, including sparse-set growth and component
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

1. Promote the frame-backed no-growth EnTT registry policy toward production
   `WorldRegistry` bake/build guidance and integration. Production wiring must
   use a persistent world-registry arena or bake allocator that resets on
   rebuild/destruction, not the per-step frame scratch allocator that resets
   inside `World::step()`.
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
   frame-backed DART storage for persistent no-growth churn, and pool-backed
   DART storage for build/growth churn. This is benchmark evidence rather than
   production `WorldRegistry` wiring; production integration needs matching
   no-growth tests and lifetime diagnostics.

4. Extend bake-time registry/component storage reservation and no-growth
   allocation tests to contact-heavy scenes and remaining solver scratch step
   paths. The initial rigid-body, non-cross articulated, and same-DOF
   sequential cross-articulated resting-contact global heap guards are in
   place; mixed/different-DOF and stacked cross-articulated contacts stay on
   the boxed-LCP fallback. Continue broadening boxed-LCP unified contact
   assembly, larger contact sets, and remaining
   solver/deformable candidate buffers before making the full zero-allocation
   claim.
5. Start replacing per-step `std::vector`/`Eigen` temporaries in hot stages with
   world-frame or world-pool backed storage only after the allocator evidence
   gate proves the DART allocator path is better for that workload. The
   non-owning `WorldStepPipeline` stage list is already inline; focus next on
   solver-owned scratch and contact/deformable candidate buffers.
