# Resume: Hierarchical Memory Manager

## Last Session Summary

Started the zero-allocation simulation-loop work by creating a dev-task tracker
and landing the first implementation slice: experimental `World` now owns a
`dart::common::MemoryManager`, accepts root allocator/frame-scratch options,
exposes memory diagnostics, and resets frame scratch at step boundaries. The
tracker records the hard allocator-quality gate: DART allocators need
correctness tests and benchmarks proving they beat standard C++ allocators and
foonathan/memory on DART-relevant workloads before broad hot-loop adoption. It
also records EnTT registry/component-storage allocation as a required
integration workstream. A follow-up on PR #2869 added frame allocator overflow
byte accounting so World diagnostics report total frame-scratch demand rather
than primary-arena usage alone. A second review follow-up made diagnostic
capacity report the usable aligned arena budget so sizing from diagnostics does
not hide alignment padding. The current follow-up branch wires the experimental
World's internal EnTT registry, component storage, and differentiable-parameter
list through the World's active free allocator. That exposed and fixed a
FreeListAllocator split-alignment bug: odd-sized free-list allocations must be
rounded so subsequent max-aligned Eigen-backed component storage remains
properly aligned.

## Current Branch

`feature/world-registry-allocator` - stacked follow-up branch for the
allocator-backed experimental World registry slice.

## Immediate Next Step

Finish review/CI for the stacked registry allocator slice, then add bake-time
registry/component storage reservation and no-growth ECS tests before claiming
zero allocations for ECS-backed world data.

## Latest Local Validation

- `pixi run lint`
- `pixi run build`
- `cmake --build build/default/cpp/Release --target UNIT_common_free_list_allocator UNIT_common_memory_manager test_world`
- `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_free_list_allocator|UNIT_common_memory_manager|test_world)$' --output-on-failure`

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

Then continue the first slice: inspect `dart/simulation/experimental/world.*`,
`dart/simulation/experimental/world_options.hpp`, and the focused world tests.
