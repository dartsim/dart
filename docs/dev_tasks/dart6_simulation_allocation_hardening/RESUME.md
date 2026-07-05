# Resume: DART 6 Simulation-Loop Allocation Hardening

## Last Session Summary

Territory scouted (DART 6 allocation map, DART 7 PLAN-122 porting map,
benchmark surface), maintainer interview completed and recorded in
`README.md`, work packets WP-D6M.1–8 authored. Baseline benchmarks were
being built. No runtime code changed yet.

## Current Branch

`dart6-memory-hardening` (worktree `/home/js/dev/dartsim/dart/task_9-mem620`,
based on `origin/release-6.20`) — dev-task docs only, uncommitted or first
commit pending.

## Immediate Next Step

Execute WP-D6M.1 (allocation-count test harness port) via a Codex executor;
in parallel record the WP-D6M.2 timing baseline
(`BM_INTEGRATION_{empty,boxes,contact_container}`, repetitions, JSON into
this folder).

## Context That Would Be Lost

- Maintainer constraints: gz-physics/gz-sim backward compatibility is
  mandatory; `World::bake()` must be optional with implicit bake in the
  first `step()`; additive public API OK; first-post-bake strict gate.
- release-6.20 hot path is already largely reuse-optimized; the five
  remaining DART-owned allocation sites are listed in WP-D6M.5.
- DART 7 harness reference: `main:tests/unit/simulation/world/test_world.cpp`
  (operator-new override at 112-527, CountingMemoryAllocator at 772-829).
- Strict gates must use native DART collision scenes (global counters see
  backend-internal allocations otherwise).

## How to Resume

```bash
cd /home/js/dev/dartsim/dart/task_9-mem620
git status && git log -3 --oneline
```

Then: continue with the first unfinished packet in `README.md` (packet list
under "Work Packets"), keeping authoring and review lanes separate.
