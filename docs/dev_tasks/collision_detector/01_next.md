# Collision Detector Task - Next

## Current Status
- Phase 1 complete with an initial architecture proposal in 02_architecture.md.
- Research covered the collision API surface, contact conventions, solver and world integration, test and benchmark coverage, and build and packaging touchpoints.
- Phase 2 scaffolding started: DARTCollisionDetector now delegates pair iteration and contact merging to DartCollisionEngine.
- Added axis-aligned bounding box prechecks in DartCollisionEngine to skip narrowphase on separated pairs.
- Added a sweep-based candidate pair pass for single-group queries to reduce pair checks.
- Cached per-object world AABBs in DARTCollisionObject, with DARTCollisionDetector calling updateEngineData before queries.
- Added a sweep-based candidate pair pass for group-group queries using combined membership flags.
- Respect negative penetration filtering in DartCollisionEngine when requested by CollisionOption.
- Added initial core data structs and wired DARTCollisionObject to populate them on update.
- Added cylinder-sphere and cylinder-plane pair support in the DART narrowphase, with new DARTCollisionDetector tests.
- Drafted a lightweight performance and benchmark plan for the new engine path.
- Routed broadphase pair generation through core entries and sweep helpers.
- Added plane-sphere and plane-box support in the DART narrowphase and enabled DART plane tests.
- Skip narrowphase for core entries marked as missing or unsupported.
- Added core-shape dispatch in DartCollisionEngine to avoid per-pair shape lookups.
- Added a DART collision benchmark variant mirroring the box stack scenario.
- Added initial core query result and option types for future standalone use.
- Added DART raycast support for core shapes with DART-focused unit coverage and AABB pruning.
- Added a resume prompt and progress tracking doc for the task.
- Added box-cylinder and cylinder-cylinder narrowphase support with DART-only unit coverage.
- Added distance query MVP for primitives with DART-only unit coverage.
- Expanded DART raycast tests to cover box, cylinder, and plane hits.
- Added DART distance edge-case coverage and a DART distance benchmark.

## Next Actions
- Clarify the core/adapter boundary and record decisions in the architecture doc.
- Extend raycast coverage to edge cases and future shape types.
- Expand distance coverage to additional primitives and refine nearest-point accuracy.
- Add broadphase pruning for distance queries.

## Questions and Decisions Needed
- Confirm the boundary between core engine code and DART-specific adapters.

## Decisions Captured
- Core namespace: `dart::collision`.
- Implementation lives under `dart/collision/dart`.
- Core builds into the `dart` target for now (no separate CMake target).
- Switch the default detector only after feature parity and performance are acceptable.
- No additional downstream constraints beyond keeping the gazebo integration workflow green.
- Engine class name: `DartCollisionEngine`.
