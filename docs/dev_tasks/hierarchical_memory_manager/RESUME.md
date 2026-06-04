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

## Current Branch

`feature/aligned-memory-allocator` - PR #2871 branch for allocator correctness
and fixed-size pool performance. The stacked
`feature/world-registry-allocator` branch for PR #2872 builds on this branch.

## Immediate Next Step

Finish review/CI for PR #2871, then merge it into the stacked World registry
branch and keep PR #2872's EnTT allocator/no-growth tests current. Next
allocator work should land the strict comparative benchmark gate, broaden
`FixedPoolAllocator` correctness coverage, and add EnTT registry/storage
allocator benchmarks before replacing more per-step hot-loop temporaries.

## Latest Local Validation

- `pixi run lint`
- `cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_pool_allocator -j2`
- `ctest --test-dir build/default/cpp/Release -R '^UNIT_common_pool_allocator$' --output-on-failure`
- `build/default/cpp/Release/bin/bm_allocators_comparative --benchmark_min_time=0.1s --benchmark_out=.benchmark_results/allocator-comparative-current-head.json --benchmark_out_format=json`
- Local parser over `.benchmark_results/allocator-comparative-current-head.json`:
  all DART/Foonathan and DART/StdPmr median ratios passed (`< 1.0`),
  including fixed pool, mixed pool, frame, realistic, steady-state, and STL
  vector workloads.

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
