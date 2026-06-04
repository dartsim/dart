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

Optimize the allocator-aware EnTT registry path until the focused registry gate
beats both foonathan/memory and the standard registry:

```bash
pixi run bm-allocator-comparative-check --only-entt-registry \
  --baseline foonathan --baseline std --verbose
```

The normal-aligned `StlAllocator` fast path now makes the focused STL-vector
probe pass against both foonathan/memory and `std::pmr`, but the
free-list-backed registry route still does not consistently beat both
foonathan/memory and the standard registry on steady-state
create/emplace/read/destroy churn. Keep the PR draft until that registry
baseline gap is resolved or a documented dependency/policy decision replaces the
in-house route. The reserved-registry unit test now proves the prewarmed churn
loop makes zero calls through the configured DART allocator, so the remaining
timing gap is allocator-aware EnTT registry/storage overhead or benchmark noise
rather than one-time pool growth.

## Latest Local Validation

- `pixi run bm-allocator-comparative-check --verbose` passed the default
  foonathan/memory gate on the current benchmark branch.
- The full opt-in checker with EnTT registry coverage and both foonathan and std
  baselines showed DART passing the foonathan EnTT registry rows but failing
  standard-registry rows, with one unrelated noisy std stack row.
- The focused registry checker over
  `.benchmark_results/entt_registry_focused_serial_1s_9.json` confirmed the
  current gap: DART beats foonathan but trails the standard registry at 256,
  512, and 2048 entities.
- The focused STL-vector checker over
  `.benchmark_results/stlvector_focused_serial_1s_9.json` passed against both
  foonathan and std, so the earlier 10k vector miss was not stable enough to
  justify an allocator-code change.
- The focused STL-vector checker over
  `.benchmark_results/stlvector_default_align_fastpath_probe.json` passed
  against both foonathan and `std::pmr` after the normal-aligned
  `StlAllocator` fast path.
- The focused EnTT checker over
  `.benchmark_results/entt_registry_stl_default_align_fastpath_probe.json`
  still failed against the standard registry and one foonathan row, confirming
  that the registry hot loop needs allocator-policy work beyond the generic
  STL adapter fast path.
- The focused `UNIT_common_stl_allocator` ctest passes with
  `StlAllocatorTest.ReservedEnttRegistryChurnDoesNotAllocate`, which asserts no
  DART allocator calls during reserved EnTT create/emplace/read/destroy churn
  after the prewarm pass.

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

## How to Resume

```bash
git status -sb
git diff --stat
gh pr view 2890 --repo dartsim/dart --json state,isDraft,mergeStateStatus,headRefOid,statusCheckRollup
```

Then continue from PR #2890 and the focused EnTT registry benchmark gate.
