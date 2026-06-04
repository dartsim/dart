# Resume: Hierarchical Memory Manager

## Last Session Summary

The memory-manager work is now split across allocator correctness,
allocator-performance evidence, allocator-backed EnTT registry integration, and
focused no-growth world-step guards. The current fixed-capacity free-list slice
adds an explicit `FreeListAllocator::GrowthPolicy::FixedCapacity` mode so a
preallocated free-list arena can fail deterministically instead of growing from
its base allocator after world creation or bake/build. The same slice now wires
free-list initial capacity and growth policy through `MemoryManager::Options`
and experimental `WorldOptions`, making the policy reachable from the World
memory hierarchy. The current review-fix slice also lets fixed-capacity
free-list arenas satisfy over-aligned allocations from reserved bytes, which is
required for `PoolAllocator` size-class chunks backed by a fixed-capacity
`MemoryManager`.

## Current Branch

`feature/free-list-fixed-capacity` - local branch for fixed-capacity
`FreeListAllocator` behavior and construction-time `MemoryManager` /
experimental `WorldOptions` policy wiring. It is published as draft PR #2892.

## Immediate Next Step

Keep PR #2892 draft until CI and Codex review are clean after the fixed-capacity
aligned-allocation review fix. The next broader allocator-policy work remains
optimizing or replacing the allocator-aware EnTT registry path until it beats
both foonathan/memory and standard-registry baselines, and expanding no-growth
world-step guards beyond the current baked kinematic IPC slice.

## Latest Local Validation

- Pre-lint review fix: `cmake --build build/default/cpp/Release --target UNIT_common_free_list_allocator UNIT_common_memory_manager -j2`
- Pre-lint review fix: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_free_list_allocator|UNIT_common_memory_manager)$' --output-on-failure`
- Pre-lint: `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
- Pre-lint: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`
- `pixi run lint`
- Post-lint review fix: `cmake --build build/default/cpp/Release --target UNIT_common_free_list_allocator UNIT_common_memory_manager -j2`
- Post-lint review fix: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_free_list_allocator|UNIT_common_memory_manager)$' --output-on-failure`
- Post-lint: `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
- Post-lint: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`
- `git diff --check`

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
- Use fixed-capacity `FreeListAllocator` when runtime growth is prohibited by a
  precomputed memory budget; keep the default expandable policy for heap-like
  use. The policy now flows through `MemoryManager::Options` and experimental
  `WorldOptions`.
- EnTT registry/storage allocation is first-class scope. Future work should
  inspect the active EnTT version's `basic_registry` and `basic_storage`
  allocator hooks and keep EnTT allocator types hidden from public World API.

## How to Resume

```bash
git status -sb
git diff --stat
```

Then continue from the open PR stack: #2871 allocator correctness/performance,
#2872 allocator-backed experimental World registry, #2888 baked-step
no-global-heap guard, #2889 fixed-pool correctness, and #2890 comparative
allocator benchmark evidence.
