# Phase 5 Export Writers Plan

## Current State

Before Phase 5 started, DART 7 `dart::io` was read-only:

- `dart::io::readSkeleton()` is the public front door for Skeleton imports.
- `dart::utils::UrdfParser` exposes file/string parsing, package resolution,
  and conversion from URDF data into a `Skeleton`.
- `dart::utils::SdfParser` exposes SDF model parsing into a `Skeleton`.
- `dart::utils::MjcfParser` is present, but `dart::io::readSkeleton()` does not
  expose direct MJCF skeleton loading yet.
- USD read-side work is active in `docs/dev_tasks/usd_scene_loader/`; early USD
  phases are explicitly read-only.

The first implementation slice adds
`dart::utils::SdfParser::tryWriteSkeletonToString()`, a parser-specific SDF
string writer for a conservative `Skeleton` subset. It writes BodyNode links,
root FreeJoint/WeldJoint placement, revolute/prismatic/weld/screw/universal
child joints, passive joint dynamics metadata (damping, Coulomb friction,
spring reference, and spring stiffness), screw thread pitch, topology-only ball
child joints, SDF 1.11+ mimic metadata for axis/axis2 follower joints with
motor enforcement, link gravity mode, inertial parameters, local joint/shape
poses, box/sphere/cylinder/mesh visual or collision geometry, and explicit
visual material colors as SDF `<diffuse>` values. Absolute non-file mesh URI
preservation is covered through a custom retriever-backed write/read test.
Targetless relative mesh references and relative or host-qualified `file` mesh
URIs are rejected because the string writer has no destination SDF URI for
resource resolution or generated asset placement. `WriteOptions`
visual/collision filtering is covered by focused tests. Unsupported constructs,
missing mesh URIs, pre-SDF-1.11 mimic output, coupler-style mimic enforcement,
non-finite material colors, non-finite screw pitch, unsupported ball-joint
metadata, and non-finite joint dynamics return `common::Result` errors instead
of being silently dropped.

The SDF writer integration test now uses
`tests/helpers/io_round_trip_helpers.hpp` for reusable body, joint, DoF,
inertia, and shape assertions. Future writer tests should extend that helper
instead of copying SDF-specific ad hoc checks.

The remaining export gap is still real implementation work. This planning note
and the first SDF writer slice do not complete Phase 5.

## Decision

Phase 5 should be implemented as a separate round-trip effort, not as a SKEL
replacement. The accepted writer scope is:

1. **SDF writer** for portable simulation interchange, because SDF can
   represent worlds/models and already has version/conversion semantics through
   libsdformat. The first parser-specific writer slice is in place; broader SDF
   coverage remains open.
2. **Project/scene save-load** for DART-owned editor state, owned by PLAN-101
   and the dartsim scene model. Current durable GUI docs lean toward JSON for
   tooling simplicity; YAML remains rejected unless a later durable owner
   accepts a versioned schema and round-trip contract.
3. **URDF writer** for robot-link trees that fit URDF's model constraints.
4. **MJCF/USD writers** only after their read-side semantics are mature enough
   in DART to define a truthful round-trip contract.

Writer API home is decided for the current DART 7 slice: keep export APIs
format-owned until more than one accepted writer contract exists. The SDF writer
therefore stays on `dart::utils::SdfParser`, `dart::io` remains the read-side
skeleton front door, and project/editor save-load belongs to the scene/project
layer. The durable rule lives in `docs/onboarding/io-parsing.md`.

## Implementation Shape

Phase 5 should start with a narrow writer API instead of a single mega-exporter:

- Add writer options separately from `ReadOptions`; do not overload the import
  API with write-only policy.
- Make unsupported DART constructs explicit diagnostics, not silent drops.
- Preserve resource URI strategy: targetless string writes should keep rejecting
  relative/generated mesh references unless a future file/project writer defines
  a destination URI, copy/rewrite policy, and round-trip tests.
- Keep project save/load separate from interchange export. Project files may
  include editor IDs, UI/editor metadata, command history seeds, or scene-model
  descriptors that should not appear in URDF/SDF.
- Start with read -> write -> read tests on small scenes that exercise links,
  joints, inertial data, visuals, collisions, mesh references, and root-joint
  policy.
- Add negative tests for unsupported constructs so the writer fails loudly when
  a scene cannot be faithfully represented.

## Acceptance Gates

Phase 5 is complete only when code and tests prove the writer contract:

1. A public or intentionally-internal writer API exists for each accepted format.
2. Focused C++ tests load a representative DART model, write it, re-load it, and
   compare the re-parsed structure.
3. The comparison checks names, topology, joint types, transforms, limits,
   inertial data, visual geometry, collision geometry, and resource URIs where
   the target format supports them.
4. Unsupported features return structured errors or clear diagnostics.
5. Project save/load, if included, has headless PLAN-101 tests for command-safe
   scene round-trip and does not conflate editor metadata with interchange
   export.
6. Documentation explains which DART constructs are representable in each
   format and which are intentionally not.
7. `CHANGELOG.md` records the user-visible export capability before the
   implementation PR merges.

## Open Work

- Extend the SDF writer contract beyond the first conservative subset when
  tests can prove additional joint aliases or edge cases, shapes, mesh material
  variants, destination-aware resource rewriting, default/world gravity, or
  other world-level data round-trip correctly.
- Decide the next implementation target: URDF writer or PLAN-101 project
  save/load.
- Keep YAML out of the first implementation target unless a durable
  project/scene schema is accepted first.
