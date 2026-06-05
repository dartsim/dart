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

The stacked registry branch wires the experimental World's internal EnTT
registry, component storage, and differentiable-parameter list through the
World's active free allocator. It also adds initial `enterSimulationMode()`
reservation/no-growth tests for current World-owned ECS storage and first
private step-scratch component paths. Direct EnTT
create/emplace/clear/re-emplace/destroy/reuse storage cycling is covered after
explicit entity/component reserve.

The no-growth and inline-pipeline slices add World base-allocator guards for
baked kinematic IPC rigid-body, multibody variational, and single-deformable
step loops, then remove the default `WorldStepPipeline` heap-backed stage
pointer vector. These are not the final global zero-allocation proof; they cover
World-owned memory hierarchy paths and default pipeline storage.

The current benchmark slice (`bench/entt-registry-allocator`, PR #2890) adds
comparative EnTT registry/component-storage rows against foonathan/memory and
standard-registry baselines. It distinguishes no-growth/prewarmed churn from
build/growth, caches component storage handles, uses frame-backed DART storage
for persistent no-growth churn and pool-backed DART storage for build/growth,
reports DART counters, and keeps EnTT rows opt-in. Current local evidence passes
strict EnTT comparisons against foonathan/memory and the standard registry;
production `WorldRegistry` wiring still requires a persistent world-registry
arena or bake allocator with matching no-growth tests and lifetime diagnostics.

## Current Branch

`bench/entt-registry-allocator` - PR #2890 branch for allocator benchmark
evidence and EnTT registry benchmark loops. The branch is published and should
merge latest `origin/main` before every push.

## Immediate Next Step

Keep #2890 focused on benchmark evidence and review/CI cleanup. Do not treat
the benchmark-only frame-backed no-growth policy as production `WorldRegistry`
bake/build allocation yet. Rerun the focused gate after any policy or benchmark
change:

```bash
pixi run bm-allocator-comparative-check --only-entt-registry \
  --baseline foonathan --baseline std --verbose
```

Next allocator work should land the strict comparative gate, broaden allocator
correctness coverage, benchmark EnTT registry/component storage, extend
no-growth tests to contact-heavy scenes and remaining solver scratch paths, and
continue optimizing allocator paths until DART beats standard C++ allocators and
foonathan/memory on required workloads.

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
- The common comparative benchmark discovers installed EnTT package metadata
  before configuring `bm_allocators_comparative`; the PR branch should require
  the supported EnTT version when enabling those optional rows.
- Historical broad and focused benchmark outputs include both passes and
  failures. Treat older files as evidence for why the gate needs repeated
  low-load runs, not as current proof.

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
gh pr view 2890 --repo dartsim/dart --json state,isDraft,mergeStateStatus,headRefOid,statusCheckRollup
```

Then continue from PR #2890 and the focused EnTT registry benchmark gate,
alongside remaining allocator dirty PR cleanup.
