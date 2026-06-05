# Hierarchical Memory Manager — Dev Task

## Current Status

- [x] Phase 1: Experimental `World` owns a memory manager and exposes
      frame-scratch diagnostics.
- [ ] Phase 2: Allocator correctness and performance gates cover DART
      allocators against standard C++ allocators and foonathan/memory.
      Alignment-aware allocation is implemented; fixed-size pool comparison now
      has a DART `FixedPoolAllocator` path that beats foonathan/memory and
      `std::pmr` locally. The strict benchmark gate and broader correctness
      matrix still need to land before this phase is complete.
- [ ] Phase 3: EnTT registry/component storage allocation is configurable from
      the World memory hierarchy and covered by no-growth ECS tests.
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
      resting-contact steps. Broader solver coverage, including boxed-LCP/cross
      articulated contact assembly, remains open before making a full
      zero-allocation claim.
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
- Boundary rule: keep EnTT allocator/storage types out of the promoted public
  facade; expose only DART-owned memory options and diagnostics.

## Immediate Next Steps

1. Land the strict allocator comparative benchmark gate and keep the
   fixed-size, mixed-pool, frame, STL, and realistic workloads below
   foonathan/memory and standard allocator baselines.
2. Extend allocator correctness tests for `FixedPoolAllocator` and the existing
   pool/free-list/frame allocators across invalid sizes, over-alignment,
   overflow, reuse-after-free, leak/debug accounting, and bounded failure.
3. Extend bake-time registry/component storage reservation and no-growth
   allocation tests to contact-heavy scenes and remaining solver scratch step
   paths. The initial rigid-body and non-cross articulated resting-contact
   global heap guards are in place; continue broadening to larger stacks,
   boxed-LCP/cross articulated contacts, and remaining solver/deformable
   candidate buffers before making the full zero-allocation claim.
4. Benchmark the allocator-backed EnTT registry/component-storage path against
   standard C++ allocators and foonathan/memory on DART-relevant workloads.
5. Start replacing per-step `std::vector`/`Eigen` temporaries in hot stages with
   world-frame or world-pool backed storage only after the allocator evidence
   gate proves the DART allocator path is better for that workload. The
   non-owning `WorldStepPipeline` stage list is already inline; focus next on
   solver-owned scratch and contact/deformable candidate buffers.
