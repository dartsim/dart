# Resume: Hierarchical Memory Manager

## Last Session Summary

The first memory-manager slice has landed: experimental `World` owns a
`dart::common::MemoryManager`, accepts root allocator/frame-scratch options,
exposes memory diagnostics, and resets frame scratch at step boundaries. The
allocator-quality gate remains active: DART allocators must beat standard C++
allocators and foonathan/memory on DART-relevant workloads before broad
hot-loop adoption.

Follow-up allocator work added alignment-aware `MemoryAllocator`/`StlAllocator`
paths for over-aligned objects and allocator-aware EnTT registries, fixed
free-list split alignment and overflow edge cases, allowed EnTT view internals
to default-construct `StlAllocator` under Clang, and added `FixedPoolAllocator`
for fixed-size slot workloads. The local comparative allocator benchmark now
passes all DART/Foonathan and DART/StdPmr median ratios when fixed-size pool
rows use `FixedPoolAllocator`, while mixed size-classed workloads remain on
`PoolAllocator`.

The stacked registry branch wires the experimental World's internal EnTT
registry, component storage, and differentiable-parameter list through the
World's active free allocator. It also adds initial
`enterSimulationMode()` reservation/no-growth tests for current World-owned ECS
storage and first private step-scratch component paths. Broader solver scratch
coverage and EnTT allocator benchmarks remain open.

The next stacked no-growth guard adds a counting base allocator test proving
that baked kinematic IPC rigid-body, multibody variational, and single
deformable paths do not request more World base-allocator allocations or
deallocations during repeated `World::step()` calls. This is not the final
global zero-allocation proof; it covers the World-owned memory hierarchy only.

The current stacked no-allocation slice pre-bakes the default step stage bundle
and kinematics graph cache at `enterSimulationMode()`, reuses rigid IPC
kinematic scratch storage, and adds a global `operator new` guard proving baked
kinematic IPC rigid-body and box-obstacle steps do not allocate from the global
heap. The broader follow-up extends that guard to a baked rigid-body
resting-contact scene by reusing collision-query/contact result storage and
default rigid-body velocity/contact stage scratch.

## Current Branch

`feature/world-step-global-heap-guard-broader` - stacked on PR #2888
(`feature/world-step-global-heap-guard`), which is stacked on PR #2872
(`feature/world-registry-allocator`). PRs #2879 and #2880 have merged to
`main`; this branch has merged the current #2872/#2888 heads.

## Immediate Next Step

Monitor #2872/#2888/#2899 CI and resolve any failures. Next allocator work
should land the strict comparative benchmark gate, broaden
`FixedPoolAllocator` correctness coverage, benchmark allocator-backed EnTT
registry/component storage, extend no-growth ECS tests to articulated contacts
and remaining solver scratch step paths, and broaden the global heap allocation
guard before claiming zero dynamic allocation for the full simulation loop.

## Latest Local Validation

- `pixi run lint`
- `cmake --build build/default/cpp/Release --target UNIT_common_stl_allocator -j2 && ctest --test-dir build/default/cpp/Release -R '^UNIT_common_stl_allocator$' --output-on-failure`
- `clang++ --gcc-toolchain=/usr -std=gnu++20 -I. -Ibuild/default/cpp/Release -I.pixi/envs/default/include -fsyntax-only` with an allocator-aware `entt::basic_registry` multi-component `view` instantiation.
- `cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_pool_allocator -j2`
- `ctest --test-dir build/default/cpp/Release -R '^UNIT_common_pool_allocator$' --output-on-failure`
- `build/default/cpp/Release/bin/bm_allocators_comparative --benchmark_min_time=0.1s --benchmark_out=.benchmark_results/allocator-comparative-current-head.json --benchmark_out_format=json`
- Local parser over `.benchmark_results/allocator-comparative-current-head.json`:
  all DART/Foonathan and DART/StdPmr median ratios passed (`< 1.0`),
  including fixed pool, mixed pool, frame, realistic, steady-state, and STL
  vector workloads.
- `cmake --build build/default/cpp/Release --target dartsim -j2`
- `pixi run test-simulation-experimental` (61/61 passed)
- `cmake --build build/default/cpp/Release --target test_world -j2`
- `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure`
- On `feature/world-step-pipeline-inline-storage` after the inline pipeline
  storage change: `cmake --build build/default/cpp/Release --target test_world -j2`
  and `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure`
- On `feature/world-step-global-heap-guard` before this handoff:
  `cmake --build build/default/cpp/Release --target test_world -j2` and
  `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure`
- On `feature/world-step-global-heap-guard-broader` after the rigid contact
  heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j2`
- On `feature/world-step-global-heap-guard-broader`:
  `./build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap'`
- On `feature/world-step-global-heap-guard-broader`:
  `./build/default/cpp/Release/bin/test_world --gtest_filter='World.Baked*DoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths'`
- On `feature/world-step-global-heap-guard-broader`:
  `cmake --build build/default/cpp/Release --target test_world test_collision_world test_collision_filter_core test_world_contact_parity -j2 && ./build/default/cpp/Release/bin/test_world --gtest_filter='World.Baked*DoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths' && ./build/default/cpp/Release/bin/test_world_contact_parity && ./build/default/cpp/Release/bin/test_collision_world && ./build/default/cpp/Release/bin/test_collision_filter_core`
- On `feature/world-step-global-heap-guard-broader`: `pixi run lint`

## Context That Would Be Lost

- The classic `dart::simulation::World` already owns a `common::MemoryManager`;
  experimental `World` did not.
- `dart/common/AGENTS.md` says World owns the memory manager and components
  borrow allocators from it.
- Frame scratch should reset at the start of each simulation step, leaving the
  previous step's scratch usage visible until the next step.
- The full user goal remains broader than this slice: route simulation data
  through allocator styles, prove no dynamic allocations during the loop, add
  memory debugging/profiling, and eventually visualize the hierarchy in GUI.
- The existing allocator implementations are not assumed to be good enough.
  Compare against `std::allocator`/`std::pmr` and foonathan/memory; if DART
  cannot beat foonathan/memory for required DART workloads, record a dependency
  decision instead of forcing an inferior in-house allocator.
- Use `FixedPoolAllocator` for fixed-size node/slot workloads. Keep
  `PoolAllocator` as the size-classed small-object allocator for mixed
  workloads.
- EnTT registry/storage allocation is first-class scope. Future work should
  inspect the active EnTT version's `basic_registry` and `basic_storage`
  allocator hooks and keep EnTT allocator types hidden from public World API.
- The active EnTT version's registry allocator propagates to component storage;
  the remaining Phase 3 gap is reserving/baking storage and proving no ECS
  growth during repeated simulation steps.

## How to Resume

```bash
git status -sb
git diff --stat
```

Then continue from the open PR stack: #2872 allocator-backed experimental World
registry, #2888 initial global heap guard, #2899 broader global heap guard, and
the comparative benchmark gate branch.
