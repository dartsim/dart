# Collision Detector Overhaul Plan

## Status

- Phase 0 (research and inventory): complete
- Phase 1 (architecture proposal): complete
- Phase 2 (core implementation): in progress
- Raycast MVP runs in DartCollisionEngine for supported primitives; distance MVP covers primitives.

## Scope

- Replace reliance on external collision backends with a new or revised in-house detector.
- Preserve current public API behavior for collision, distance, and raycast queries.
- Keep contact conventions stable (normal direction, penetration depth semantics, filtering).
- Provide a standalone build and installable library with clear CMake targets.
- Maintain integration with the constraint solver and simulation pipeline.
- Keep the existing gazebo integration workflow passing without additional patching.

## Non-goals

- No redesign of the constraint solver algorithms beyond required integration hooks.
- No broad refactor of dynamics or IO APIs unless required for collision coupling.
- No removal of legacy backends without a staged deprecation path.
- No new build entry points outside existing pixi workflows.

## Decisions

- Core namespace: `dart::collision`.
- Implementation lives under `dart/collision/dart` during this effort.
- Core builds into the `dart` target for now (no separate CMake target).
- Default switch only after feature parity and acceptable performance.
- Engine class name: `DartCollisionEngine`.

## Milestones

1. Research and inventory (complete): current API surface, integration points, tests, benchmarks, build and packaging touchpoints.
2. Architecture proposal: data model, ownership, broadphase and narrowphase approach, shape adaptation, and factory integration.
3. Core implementation: new detector core, adapters, and compatibility layers.
4. Feature parity: collision, distance, raycast, filtering, contact generation, and self-collision semantics.
5. Performance parity: profiling, targeted optimizations, and benchmark coverage.
6. Migration and dependency removal: deprecation messaging, downstream guidance, and cleanup.

## Deprecation and Migration Strategy

- Introduce the new detector behind existing factories and configuration keys.
- Keep legacy backends available during transition with clear deprecation warnings.
- Maintain compatibility stubs until the removal milestone is reached.
- Provide a concise migration guide focused on behavior changes and build toggles.

## Test Plan

- Run existing unit and integration collision tests plus constraint and simulation suites.
- Expand tests to cover new detector behaviors and edge cases.
- Maintain and extend benchmarks to track performance trends.
- Add CI checks to prevent regressions and track parity.

## Performance and Benchmark Plan

- Capture a baseline from existing collision benchmarks before enabling the new path.
- Add scenarios that stress broadphase pair generation, narrowphase contact generation, and group-group queries.
- Track timing, candidate pair counts, contact counts, and allocation hot spots.
- Define acceptable regression thresholds and enforce them before switching defaults.

## Risks

- Contact generation differences affecting solver stability.
- Missing feature coverage (distance, raycast, mesh contact fidelity).
- Performance regressions in broadphase or narrowphase.
- Standalone packaging and ABI surface complexity.
- Downstream migration friction.

## Acceptance Criteria

- Existing collision tests pass with the new detector as default.
- Performance meets defined parity thresholds on representative benchmarks.
- Contact conventions remain consistent with current expectations.
- Standalone library builds and installs cleanly, and integrates with DART.
- Gazebo integration workflows remain green with existing task chains.

## Open Questions

- When should the core split into a standalone target and what packaging constraints apply?
- Which broadphase and narrowphase strategies best fit parity and performance goals?
