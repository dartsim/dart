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
free-list split alignment and overflow edge cases, default-constructible EnTT
view allocator support under Clang, and `FixedPoolAllocator` for fixed-size slot
workloads. Fixed-capacity free-list arenas now fail deterministically instead of
growing after world creation or bake/build, and the policy flows through
`MemoryManager::Options` and experimental `WorldOptions`.

The registry/no-growth slices wire the experimental World's internal EnTT
registry, component storage, differentiable-parameter list, and first
World-owned ECS scratch paths through the World memory hierarchy. They add
base-allocator no-growth guards for baked kinematic IPC rigid-body, multibody
variational, and single-deformable step loops, plus inline default step-pipeline
storage. These are not the final global zero-allocation proof.

The first global heap guard branch, PR #2888, pre-bakes the default step stage
bundle and kinematics graph cache at `enterSimulationMode()`, reuses rigid IPC
kinematic scratch storage, and adds a global `operator new` guard proving baked
kinematic IPC rigid-body and box-obstacle steps do not allocate from the global
heap. The branch was refreshed on the current `main` after the allocator
debugger diagnostics and fixed-capacity free-list slices merged.

The current broader no-allocation slice, PR #2899, extends that guard to baked
rigid-body and non-cross articulated resting-contact scenes by reusing
collision-query/contact result storage, default rigid-body velocity/contact
stage scratch, and semi-implicit multibody dynamics/contact/staged-velocity
scratch.

## Current Branch

`feature/world-step-global-heap-guard-broader` - PR #2899, stacked on PR #2888
(`feature/world-step-global-heap-guard`). This branch has merged the updated
#2888 base after #2892 and #2893 landed on `main`.

## Immediate Next Step

Resolve PR #2899's hosted Coverage (Debug) failure. The failing job reported a
`test_world` segfault after most of the CTest suite had completed, so first
rerun the updated branch locally with the focused `test_world` and broader
coverage/debug path before deciding whether this is a stale CI artifact or a
real scratch/lifetime bug.

After #2899 is clean, continue the allocator work by landing the strict
comparative benchmark gate, broadening `FixedPoolAllocator` correctness
coverage, benchmarking allocator-backed EnTT registry/component storage,
extending no-growth ECS tests to larger contact stacks and remaining solver
scratch step paths, and broadening the global heap allocation guard before
claiming zero dynamic allocation for the full simulation loop.

## Latest Local Validation

- On `feature/world-step-global-heap-guard` after the latest `origin/main`
  merge:
  `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
- On `feature/world-step-global-heap-guard` after the latest `origin/main`
  merge:
  `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`
- On `feature/world-step-global-heap-guard`: `pixi run lint`
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
- On `feature/world-step-global-heap-guard-broader` after the articulated
  contact heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j2 && build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedArticulatedContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
- On `feature/world-step-global-heap-guard-broader` after the semi-implicit
  multibody scratch update:
  `cmake --build build/default/cpp/Release --target test_multibody_constraint test_multibody_link_contact -j2 && build/default/cpp/Release/bin/test_multibody_constraint --gtest_color=no && build/default/cpp/Release/bin/test_multibody_link_contact --gtest_color=no`
- On `feature/world-step-global-heap-guard-broader` after the semi-implicit
  multibody scratch update:
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.Multibody*'`

Run fresh #2899 validation after this base merge before pushing.

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
  use.
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
gh pr view 2899 --repo dartsim/dart --json state,isDraft,mergeStateStatus,headRefOid,statusCheckRollup
```

Then continue from PR #2899 and the remaining allocator dirty PR cleanup.
