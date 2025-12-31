# Collision Detector Task - Progress

## Status
- Phase 2 (core implementation) is in progress.
- Core data and broadphase scaffolding are in place, with AABB caching.
- Narrowphase supports sphere, box, cylinder, and plane primitives.
- Raycast is available for the supported primitives with AABB pruning.
- Distance query MVP is implemented for supported primitives with sweep and AABB pruning.
- Raycast tests now cover box, cylinder, and plane hits.
- Core/adapter boundary decisions are captured in the architecture proposal.
- Distance tests now cover sphere-plane, sphere-box, sphere-cylinder, and box-cylinder nearest points.
- Raycast tests now include inside-hit cases for sphere, box, and cylinder.
- Distance tests include rotated box coverage, and raycast tests include a rotated plane hit.
- Raycast tests include tangent sphere hits and parallel plane misses.
- Distance tests now check cylinder-cylinder nearest points; raycast tests include tangent cylinder hits.
- Distance tests include a rotated plane case for sphere-plane queries.

## Completed Checkpoints
- Core engine path wired into the DART detector with AABB prechecks.
- Sweep-based broadphase candidate generation for single and group queries.
- Primitive narrowphase coverage for sphere, box, cylinder, and plane pairs.
- Added box-cylinder and cylinder-cylinder narrowphase support with DART-only unit tests.
- Added a DART-focused distance unit test suite for primitives.
- Added a DART distance benchmark and expanded distance edge-case tests.
- Raycast support for primitives plus DART-only unit coverage.
- Added a DART collision benchmark variant for baseline tracking.
- Added sweep-based pruning for distance queries with AABB lower bounds.
- Documented the core/adapter boundary and raycast placement in the architecture proposal.
- Expanded distance tests for sphere-plane, sphere-box, sphere-cylinder, and box-cylinder.
- Added inside-hit raycast coverage for sphere, box, and cylinder.
- Added rotated box distance coverage and a rotated plane raycast check.
- Added tangent sphere and parallel plane raycast coverage.
- Added cylinder-cylinder nearest-point checks and tangent cylinder raycast coverage.
- Added rotated plane distance coverage for sphere-plane.

## Next Steps
- Expand distance coverage for additional rotated or oblique configurations and refine nearest-point accuracy.
- Extend raycast coverage to additional edge cases and future shape types.
- Decide when to move raycast into the core engine once query types stabilize.
