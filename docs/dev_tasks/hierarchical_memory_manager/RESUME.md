# Resume: Hierarchical Memory Manager

## Current Reality (2026-06-06)

Use this folder's `README.md`, `docs/plans/dashboard.md`, and the current code as
the live status. The branch-local "Current Branch", validation-command, and PR
handoff sections below are historical session notes, not current checkout
instructions. Memory/scalability work should continue through World-owned
allocator and diagnostics contracts (`MemoryManager`, `WorldOptions`,
`WorldMemoryDiagnostics`) rather than exposing EnTT storage, allocator internals,
or backend resources on the public simulation facade.

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
to default-construct `StlAllocator` under Clang, added `FixedPoolAllocator` for
fixed-size slot workloads, and added fixed-capacity `FreeListAllocator` policy
wiring through `MemoryManager::Options` and experimental `WorldOptions`.
Fixed-capacity free-list arenas can satisfy over-aligned `PoolAllocator` chunks
from reserved bytes without growing from the base allocator.

The memory-debugger correctness slice exposes structured live-byte, peak-byte,
and allocation-count queries from the allocator debug path and from the
manager-owned free-list/pool allocators. `MemoryManager::DebugDiagnostics` and
experimental `WorldMemoryDiagnostics` include typed borrowed allocator use, so
diagnostics cover callers that borrow `getFreeListAllocator()` or
`getPoolAllocator()` directly.

The registry/no-growth slices wire the experimental World's internal EnTT
registry, component storage, differentiable-parameter list, and first
World-owned ECS scratch paths through the World memory hierarchy. They add
base-allocator no-growth guards for baked kinematic IPC rigid-body, multibody
variational, and single-deformable step loops, plus inline default step-pipeline
storage. These are not the final global zero-allocation proof.

The global heap guard branches pre-bake the default step stage bundle and
kinematics graph cache at `enterSimulationMode()`, reuse rigid IPC kinematic
scratch storage, and add global `operator new` guards proving baked kinematic
IPC rigid-body, box-obstacle, rigid-body resting-contact, non-cross articulated
resting-contact, and same-DOF sequential cross-articulated link-contact steps do
not allocate from the global heap. Mixed/different-DOF, stacked, and coupled
multi-row cross-articulated contacts stay on the boxed-LCP fallback; boxed-LCP
unified contact assembly, larger contact sets, and remaining solver-owned
scratch remain open.

The EnTT benchmark slice (`bench/entt-registry-allocator`, PR #2890) adds
comparative EnTT registry/component-storage rows against foonathan/memory and
standard-registry baselines. It distinguishes no-growth/prewarmed churn from
build/growth, caches component storage handles, uses frame-backed DART storage
for persistent no-growth churn and pool-backed DART storage for build/growth,
reports DART counters, and keeps EnTT rows opt-in. PR #2890 has merged to
`main`; keep its benchmark evidence as the baseline for future allocator-policy
loops.

The broader no-allocation slice, PR #2899, has merged to `main`. It extends the
merged #2888 guard to baked rigid-body and non-cross articulated resting-contact
scenes by
reusing collision-query/contact result storage, default rigid-body
velocity/contact stage scratch, and semi-implicit multibody
dynamics/contact/staged-velocity scratch.

The current boxed-LCP scratch slice starts reusing
`UnifiedConstraintStage`-owned assembly containers and unified problem storage
for cross-body articulated contact paths. It reduces transient container churn
but does not claim the boxed-LCP solve is globally allocation-free; solver-local
and remaining assembler-local scratch are still open.

## Current Branch

`feature/world-unified-constraint-scratch` - starts boxed-LCP unified scratch
reuse from the post-#2906 `main` state.

## Immediate Next Step

Finish the current branch, run `pixi run lint`, open the PR, and request a
fresh Codex review. The next allocator slice should move remaining boxed-LCP
assembler-local vectors/matrices and solver-local scratch toward reusable
world-owned storage.

Do not treat the benchmark-only frame-backed no-growth policy as production
`WorldRegistry` bake/build allocation yet. Production integration needs a
persistent world-registry arena or bake allocator that resets on
rebuild/destruction, not the existing per-step frame allocator that resets
inside `World::step()`.

Rerun the focused gate on a quiet host after any allocator-policy or benchmark
change:

```bash
pixi run bm-allocator-comparative-check --only-entt-registry \
  --baseline foonathan --baseline std --verbose
```

Next allocator work should broaden allocator correctness coverage, extend
no-growth tests to contact-heavy scenes and remaining solver scratch paths, and
continue optimizing allocator paths until DART beats standard C++ allocators and
foonathan/memory on required workloads. The active zero-allocation guard work
should broaden beyond the covered rigid-body, non-cross articulated, and
same-DOF sequential cross-articulated resting-contact scenes before making a
full zero-dynamic-allocation claim.

## Latest Local Validation

- On `feature/world-unified-constraint-scratch`:
  `cmake --build build/default/cpp/Release --target test_unified_constraint test_unified_constraint_stage test_world -j8 && build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no && build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no && build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.ZeroDofMultibodyLinkContactStopsRigidBody:World.ZeroDofMultibodyFallbackDoesNotDoubleApplyShortcutImpulses'`
  (17/17 unified constraint tests, 2/2 unified constraint stage tests, and 2/2
  focused World zero-DOF contact tests passed).
- On `feature/world-step-global-heap-guard` after merging #2890-updated
  `origin/main`: `pixi run lint`,
  `cmake --build build/default/cpp/Release --target test_world -j8`, and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.ReplayRestoreRebuildsCachedKinematicsAfterFrameParentRestore:World.ReplayRecordingRestoresPublicFrameState:World.StepRebuildsCachedKinematicsAfterFrameReparenting'`
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
- On `feature/world-cross-contact-heap-guard` after the sequential
  cross-multibody contact heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j8 && build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedArticulatedContactStepsDoNotAllocateGlobalHeap:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Multibody*'`
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
- Post-lint review fix: `cmake --build build/default/cpp/Release --target UNIT_common_free_list_allocator UNIT_common_memory_manager -j2`
- Post-lint review fix: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_free_list_allocator|UNIT_common_memory_manager)$' --output-on-failure`
- Post-lint: `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
- Post-lint: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`
- `git diff --check`
- Current #2890 focused strict command passed on 2026-06-04 at local timestamp
  `20:20:48-07:00`:
  `pixi run bm-allocator-comparative-check --only-entt-registry --baseline foonathan --baseline std --verbose --output .benchmark_results/allocator_comparative_entt_frame_final.json`.
  All 12 EnTT comparisons passed. No-growth DART rows reported
  `dart_frame_overflow_count=0` and `dart_frame_overflow_bytes=0` after
  prewarm. DART/foonathan ratios were approximately `0.891`, `0.993`, and
  `0.961` for `BM_EnttRegistry/{256,512,2048}`; DART/std ratios were
  approximately `0.762`, `0.737`, and `0.767`. Build/growth DART rows beat both
  foonathan/memory and the standard registry at 256, 512, and 2048 entities
  while reporting configured-allocator calls per iteration of 37, 38, and 43.
- Current-head high-load rerun on head `750cd83b74c1` failed:
  `taskset -c 8-15 pixi run bm-allocator-comparative-check --only-entt-registry --baseline foonathan --baseline std --verbose --output .benchmark_results/allocator_comparative_entt_current_750cd83.json`.
  DART still reported no frame overflow, but warmed EnTT rows lost against
  foonathan/memory at 512 and 2048 entities and against Std at 256 entities;
  several baseline rows also exceeded the CV/noise limit.
- On `feature/free-list-fixed-capacity` after the fixed-capacity slice:
  `cmake --build build/default/cpp/Release --target UNIT_common_free_list_allocator UNIT_common_memory_manager test_world -j6`,
  `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_free_list_allocator|UNIT_common_memory_manager|test_world)$' --output-on-failure`,
  `pixi run lint`, and `git diff --check && git diff --cached --check`.
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
- Treat EnTT registry/storage allocation as first-class scope. The ECS storage
  layer is a dominant owner of world/component memory, but EnTT allocator types
  must remain hidden from promoted public World APIs.
- The active EnTT version's registry allocator propagates to component storage;
  the remaining Phase 3 gap is broadening bake-time reservation coverage for
  contact-heavy and solver-private scratch paths and benchmarking the
  allocator-backed EnTT path.
- The steady-state EnTT registry benchmark prewarms storage before timing. A
  standard-registry miss there points at allocator-aware registry/storage
  overhead, not one-time pool growth.
- The EnTT build/growth benchmark creates a fresh registry, reserves component
  storage, runs one churn pass, and destroys the registry inside each measured
  iteration. A miss there points at bake/build allocator and storage setup cost.
- The frame-backed EnTT benchmark policy is not yet production `WorldRegistry`
  wiring. Production integration needs a persistent world-registry arena or bake
  allocator that is reset on rebuild/destruction, not the existing per-step
  frame allocator that resets inside `World::step()`.

## How to Resume

```bash
git status -sb
git diff --stat
gh pr view 2899 --repo dartsim/dart --json state,isDraft,mergeStateStatus,headRefOid,statusCheckRollup
```

Then continue from PR #2899 and merged-branch cleanup candidates.
