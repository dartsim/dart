# Resume: Hierarchical Memory Manager

## Last Session Summary

The first memory-manager slice has landed: experimental `World` owns a
`dart::common::MemoryManager`, accepts root allocator/frame-scratch options,
exposes memory diagnostics, and resets frame scratch at step boundaries. The
allocator-quality gate remains active: DART allocators must beat standard C++
allocators and foonathan/memory on DART-relevant workloads before broad hot-loop
adoption.

Follow-up allocator work added alignment-aware `MemoryAllocator`/`StlAllocator`
paths for over-aligned objects and allocator-aware EnTT registries, fixed
free-list split alignment and overflow edge cases, allowed EnTT view internals
to default-construct `StlAllocator` under Clang, and added `FixedPoolAllocator`
for fixed-size slot workloads. The comparative allocator benchmark passes local
DART/Foonathan and DART/StdPmr median ratios when fixed-size pool rows use
`FixedPoolAllocator`, while mixed size-classed workloads remain on
`PoolAllocator`.

The fixed-capacity free-list slice adds
`FreeListAllocator::GrowthPolicy::FixedCapacity` so a preallocated free-list
arena can fail deterministically instead of growing from its base allocator
after world creation or bake/build. The same slice wires free-list initial
capacity and growth policy through `MemoryManager::Options` and experimental
`WorldOptions`, making the policy reachable from the World memory hierarchy.
Fixed-capacity free-list arenas also satisfy over-aligned allocations from
reserved bytes, which is required for `PoolAllocator` size-class chunks backed
by a fixed-capacity `MemoryManager`.

The memory-debugger correctness slice exposes structured live-byte, peak-byte,
and allocation-count queries from the allocator debug path and from the
manager-owned free-list/pool allocators. `MemoryManager::DebugDiagnostics` and
experimental `WorldMemoryDiagnostics` include typed borrowed allocator use, so
diagnostics cover callers that borrow `getFreeListAllocator()` or
`getPoolAllocator()` directly.

The registry/no-growth slices wire the experimental World's internal EnTT
registry, component storage, differentiable-parameter list, and first
World-owned ECS scratch paths through the World memory hierarchy. They also add
base-allocator no-growth guards for baked kinematic IPC rigid-body, multibody
variational, and single-deformable step loops, plus inline default step-pipeline
storage. These are not the final global zero-allocation proof.

## Current Branch

`feature/free-list-fixed-capacity` - PR #2892 branch for fixed-capacity
`FreeListAllocator` behavior and construction-time `MemoryManager` /
experimental `WorldOptions` policy wiring. It is published and should merge
latest `origin/main` before every push.

## Immediate Next Step

Keep PR #2892 focused on fixed-capacity free-list correctness, current-main
merge cleanup, and review/CI fixes. Next allocator work should land the strict
comparative benchmark gate, broaden allocator correctness coverage, benchmark
allocator-backed EnTT registry/component storage, extend no-growth tests to
contact-heavy scenes and remaining solver scratch paths, and continue
optimizing allocator paths until DART beats standard C++ allocators and
foonathan/memory on required workloads.

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
- On `test/memory-allocator-debugger-correctness` after borrowed-allocator
  diagnostics: `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
  and `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`

## Context That Would Be Lost

- The classic `dart::simulation::World` already owns a `common::MemoryManager`;
  experimental `World` did not.
- `dart/common/AGENTS.md` says World owns the memory manager and components
  borrow allocators from it.
- Frame scratch should reset at the start of each simulation step, leaving the
  previous step's scratch usage visible until the next step.
- The full user goal remains broader than this slice: route simulation data
  through allocator styles, prove zero dynamic allocation in representative
  simulation loops, add memory debugging/profiling, and eventually visualize the
  hierarchy in GUI.
- The existing allocator implementations are not assumed to be good enough.
  Compare against standard C++ allocators and foonathan/memory; if DART cannot
  beat foonathan/memory for required DART workloads, record a dependency
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
- The active EnTT version's registry allocator propagates to component storage;
  the remaining Phase 3 gap is broadening bake-time reservation coverage for
  contact-heavy and solver-private scratch paths and benchmarking the
  allocator-backed EnTT path.

## How to Resume

```bash
git status -sb
git diff --stat
gh pr view 2892 --repo dartsim/dart --json state,isDraft,mergeStateStatus,headRefOid,statusCheckRollup
```

Then continue from PR #2892 and the remaining allocator dirty PR cleanup.
