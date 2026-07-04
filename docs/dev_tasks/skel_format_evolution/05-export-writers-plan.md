# Phase 5 Export Writers Plan

## Current State

DART 7 `dart::io` is read-only today:

- `dart::io::readSkeleton()` is the public front door for Skeleton imports.
- `dart::utils::UrdfParser` exposes file/string parsing, package resolution,
  and conversion from URDF data into a `Skeleton`.
- `dart::utils::SdfParser` exposes SDF model parsing into a `Skeleton`.
- `dart::utils::MjcfParser` is present, but `dart::io::readSkeleton()` does not
  expose direct MJCF skeleton loading yet.
- USD read-side work is active in `docs/dev_tasks/usd_scene_loader/`; early USD
  phases are explicitly read-only.

The export gap is therefore real implementation work. This planning note does
not complete Phase 5.

## Decision

Phase 5 should be implemented as a separate round-trip effort, not as a SKEL
replacement. The accepted writer scope is:

1. **Project/scene save-load** for DART-owned editor state, owned by PLAN-101
   and the dartsim scene model. Current durable GUI docs lean toward JSON for
   tooling simplicity; YAML remains rejected unless a later durable owner
   accepts a versioned schema and round-trip contract.
2. **SDF writer** for portable simulation interchange, because SDF can represent
   worlds/models and already has version/conversion semantics through
   libsdformat.
3. **URDF writer** for robot-link trees that fit URDF's model constraints.
4. **MJCF/USD writers** only after their read-side semantics are mature enough
   in DART to define a truthful round-trip contract.

## Implementation Shape

Phase 5 should start with a narrow writer API instead of a single mega-exporter:

- Add writer options separately from `ReadOptions`; do not overload the import
  API with write-only policy.
- Make unsupported DART constructs explicit diagnostics, not silent drops.
- Preserve resource URI strategy: mesh paths, `package://` rewriting, and
  generated relative paths need deterministic rules.
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

- Decide the first implementation target: SDF writer, URDF writer, or PLAN-101
  project save/load.
- Define the comparison helper for read/write/read tests so future formats do
  not copy ad hoc assertions.
- Decide whether writer APIs live under `dart::io`, parser-specific utility
  namespaces, or a DART 7 scene/project layer.
- Keep YAML out of the first implementation target unless a durable
  project/scene schema is accepted first.
