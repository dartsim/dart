# Resume: Hierarchical Memory Manager

## Last Session Summary

The first memory-manager slice has landed: experimental `World` owns a
`dart::common::MemoryManager`, accepts root allocator/frame-scratch options,
exposes memory diagnostics, and resets frame scratch at step boundaries. The
allocator-quality gate is active: DART allocators must beat standard C++
allocators and foonathan/memory on DART-relevant workloads before broad
hot-loop adoption. Follow-up allocator work added alignment-aware
`MemoryAllocator`/`StlAllocator` paths for over-aligned objects and
allocator-aware EnTT registries, fixed free-list split alignment and overflow
edge cases, and added `FixedPoolAllocator` for fixed-size slot workloads. The
local comparative allocator benchmark now passes all DART/Foonathan median
ratios when the fixed-size pool row uses `FixedPoolAllocator`, while mixed
size-classed workloads remain on `PoolAllocator`.
The memory-debugger correctness slice now also exposes structured live-byte,
peak-byte, and allocation-count queries from `MemoryAllocatorDebugger`, giving
later World diagnostics/profiling code an API surface instead of requiring
debug-text parsing. The same branch now routes those counters through
`MemoryManager::DebugDiagnostics` and experimental `WorldMemoryDiagnostics` for
direct free/pool allocator accounting.

## Current Branch

`test/memory-allocator-debugger-correctness` - PR #2893 branch for allocator
debugger accounting coverage and structured debugger counters.

## Immediate Next Step

Keep PR #2893 draft until hosted CI and review are clean after the structured
debugger-counter and MemoryManager/World diagnostic updates. Next allocator work
should continue landing the strict comparative benchmark gate,
fixed-capacity/free-list correctness, and EnTT registry/storage allocator
evidence before replacing more per-step hot-loop temporaries.

## Latest Local Validation

- `pixi run lint`
- `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
- `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`
- `cmake --build build/default/cpp/Release --target UNIT_common_memory_allocator -j2`
- `./build/default/cpp/Release/bin/UNIT_common_memory_allocator`
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
- EnTT registry/storage allocation is first-class scope. Future work should
  inspect the active EnTT version's `basic_registry` and `basic_storage`
  allocator hooks and keep EnTT allocator types hidden from public World API.

## How to Resume

```bash
git status -sb
git diff --stat
```

Then continue from the open PR stack: #2871 allocator correctness/performance,
#2872 allocator-backed experimental World registry, and the comparative
benchmark gate branch.
