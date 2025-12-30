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

## Next Actions
- Define the standalone library boundary and the DART adapter layer.
- Prototype the core data model and query pipeline (broadphase, narrowphase, contact generation).
- Evaluate whether to keep sweep-based broadphase for group-group or move to a spatial structure.
- Prepare parity test and benchmark updates to compare against existing backends.

## Questions and Decisions Needed
- Confirm the boundary between core engine code and DART-specific adapters.

## Decisions Captured
- Core namespace: `dart::collision`.
- Implementation lives under `dart/collision/dart`.
- Core builds into the `dart` target for now (no separate CMake target).
- Switch the default detector only after feature parity and performance are acceptable.
- No additional downstream constraints beyond keeping the gazebo integration workflow green.
