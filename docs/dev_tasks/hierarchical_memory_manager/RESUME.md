# Resume: Hierarchical Memory Manager

## Last Session Summary

The memory-manager work is now in a PR stack rather than a single allocator
branch. PR #2872 routes experimental `World` registry/storage through the World
memory hierarchy and has focused no-growth coverage. PR #2890 adds comparative
allocator benchmark coverage, a CV/noise guard, and focused EnTT registry
benchmark modes for allocator-policy work.

## Current Branch

`bench/entt-registry-allocator` - PR #2890 draft branch for allocator
benchmark evidence and EnTT registry benchmark loops. The branch is published
and should merge latest `origin/main` before every push.

## Immediate Next Step

Continue PR #2890 by optimizing the allocator-aware EnTT registry path until the
default focused registry gate beats both foonathan/memory and the standard
registry:

```bash
pixi run bm-allocator-comparative-check --only-entt-registry \
  --baseline foonathan --baseline std --verbose
```

The current policy caches known EnTT component storage handles in the hot path,
uses a free-list-backed DART allocator for persistent no-growth registry churn,
and uses a pool-backed DART allocator for bake/build growth churn. The
build/growth timing row uses an uninstrumented pool-backed allocator and reports
configured-allocator call counters from a matching untimed probe. It is not yet
a passing claim: the default focused checker has favorable medians against both
foonathan/memory and the standard registry, but still fails the 10% CV stability
gate. `StlAllocator` now keeps allocator-backed STL storage alignment-aware,
including fixed-pool-backed max-aligned values. A separate world-lifetime
arena-backed registry probe via `FrameStlAllocator` proved the no-growth
invariant, but repeated timing did not consistently beat both baselines. Do not
confuse that probe with the per-step `World` frame allocator, which is reset at
step boundaries and cannot hold persistent registry storage. The
reserved-registry unit tests now prove the prewarmed churn loop makes no
configured allocator calls and does not consume additional arena bytes after
prewarm. The DART EnTT benchmark row reports allocator-call counters and fails
if reserved churn calls the configured allocator after prewarm. Separate EnTT
build/growth rows now measure bake-time registry storage allocation directly
instead of conflating that cost with the no-growth simulation loop.

## Latest Local Validation

- The default focused command still failed the strict gate on 2026-06-04 after
  switching the benchmark hot path to cached component storage handles,
  `storage<entt::entity>().generate()`, free-list-backed persistent no-growth
  churn, and pool-backed build/growth churn. The current default run failed only
  on CV stability, not on median timing ratios. DART steady-state rows reported
  `dart_allocator_allocations=0` and `dart_allocator_deallocations=0`. Median
  DART/std ratios were `BM_EnttRegistry/{256,512,2048}` = `0.529`, `0.540`,
  `0.522`; `BM_EnttRegistryBuild/{256,512,2048}` = `0.865`, `0.922`, `0.591`.
  Median DART/foonathan ratios were `BM_EnttRegistry/{256,512,2048}` =
  `0.781`, `0.862`, `0.984`; `BM_EnttRegistryBuild/{256,512,2048}` =
  `0.411`, `0.648`, `0.477`. Build/growth rows reported DART
  configured-allocator calls per iteration of 37, 38, and 43 for 256, 512, and
  2048 entities. A follow-up longer-sample attempt was stopped because it began
  under high local load and was not useful evidence.
- A refreshed world-lifetime arena-backed benchmark experiment over
  `.benchmark_results/entt_registry_arena_policy_probe.json` also failed the
  strict gate. It drove backing allocator calls to zero for the build/growth
  rows, but still lost all no-growth rows and all standard-registry rows, so do
  not replace the current free-list-backed DART EnTT benchmark policy with
  `FrameStlAllocator` timing evidence.
- The common comparative benchmark now discovers installed EnTT package
  metadata before configuring `bm_allocators_comparative`. Local
  `bm_allocators_comparative --benchmark_list_tests` lists all DART,
  foonathan/memory, and standard EnTT registry/build rows at 256, 512, and 2048
  entities.
- `pixi run bm-allocator-comparative-check --verbose` passed the default
  foonathan/memory gate on the current benchmark branch.
- The focused STL-vector checker over
  `.benchmark_results/stlvector_focused_serial_1s_9.json` passed against both
  foonathan and std, so the earlier 10k vector miss was not stable enough to
  justify an allocator-code change.
- The focused STL-vector checker over
  `.benchmark_results/stlvector_default_align_fastpath_probe.json` passed
  against both foonathan and `std::pmr` before the fixed-pool max-aligned
  correctness fix; rerun this probe before treating it as current performance
  evidence.
- The focused EnTT checker over
  `.benchmark_results/entt_registry_stl_default_align_fastpath_probe.json`
  failed against the standard registry and one foonathan row before the
  cached-storage and pool-backed build/growth policy landed. Treat that file as
  historical evidence for why the generic STL adapter route was not enough.
- The focused `UNIT_common_stl_allocator` ctest passes with
  `StlAllocatorTest.SupportsFixedPoolBackedMaxAlignedStorage`, which preserves
  max-aligned STL values on fixed-pool-backed storage, and
  `StlAllocatorTest.ReservedEnttRegistryChurnDoesNotAllocate`, which asserts no
  DART allocator calls during reserved EnTT create/emplace/read/destroy churn
  after the prewarm pass.
- The focused benchmark probe
  `.benchmark_results/entt_registry_dart_alignment_fix_counter_probe.json`
  reports
  `dart_allocator_allocations=0` and `dart_allocator_deallocations=0` for the
  DART EnTT rows at 256, 512, and 2048 entities.
- The relaxed focused registry checker over
  `.benchmark_results/entt_registry_alignment_fix_checker_probe.json`
  passed with those counters present before the strict standard-registry gap was
  closed by the current benchmark policy.
- The relaxed focused checker over
  `.benchmark_results/entt_registry_with_build_growth_post_lint_probe.json`
  included both no-growth and build/growth EnTT rows. The new DART build/growth
  rows beat the foonathan/memory build/growth rows at all measured sizes, but
  that JSON predates the current cached-storage and uninstrumented build/growth
  timing policy and should not be treated as the current gate result.
- A world-lifetime arena probe using `FrameStlAllocator` showed zero arena
  growth after prewarm and zero backing allocator calls, but repeated focused
  strict checks were not stable enough to claim it beats both foonathan/memory
  and the standard registry. Keep the arena result as correctness evidence and
  continue policy work before changing production `WorldRegistry` storage.

## Context That Would Be Lost

- The full user goal remains broader than the current benchmark PR: route
  simulation data through allocator styles, prove zero dynamic allocation in
  representative simulation loops, add memory debugging/profiling, and
  eventually visualize the hierarchy in GUI.
- The existing allocator implementations are not assumed to be good enough.
  Compare against standard C++ allocators and foonathan/memory; if DART cannot
  beat foonathan/memory for required DART workloads, record a dependency
  decision instead of forcing an inferior in-house allocator.
- Treat EnTT registry/storage allocation as first-class scope. The ECS storage
  layer is a dominant owner of world/component memory, but EnTT allocator types
  must remain hidden from promoted public World APIs.
- The steady-state EnTT registry benchmark prewarms storage before timing.
  A std-registry miss there points at allocator-aware registry/storage overhead,
  not one-time pool growth.
- The EnTT build/growth benchmark creates a fresh registry, reserves component
  storage, runs one churn pass, and destroys the registry inside each measured
  iteration. A miss there points at bake/build allocator and storage setup cost.
- The arena-backed EnTT probe is a policy candidate, not yet production
  `WorldRegistry` wiring and not a passing performance claim. Production
  integration would need a persistent world-registry arena or bake allocator
  that is reset on rebuild/destruction, not the existing per-step frame
  allocator that resets inside `World::step()`.

## How to Resume

```bash
git status -sb
git diff --stat
gh pr view 2890 --repo dartsim/dart --json state,isDraft,mergeStateStatus,headRefOid,statusCheckRollup
```

Then continue from PR #2890 and the focused EnTT registry benchmark gate.
