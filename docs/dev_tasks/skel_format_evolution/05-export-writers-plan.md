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
writer for a conservative `Skeleton` subset. It builds libsdformat DOM objects
and serializes through sdformat; DART owns the `Skeleton`-to-SDF semantic
mapping and diagnostics. It writes BodyNode links, root FreeJoint/WeldJoint
placement,
revolute/continuous/prismatic/weld/screw/universal child joints, passive joint
dynamics metadata (damping, Coulomb friction, spring reference, and spring
stiffness), sdformat-normalized screw thread pitch (legacy `<thread_pitch>`
before SDF 1.10 and modern `<screw_thread_pitch>` for SDF 1.10+),
topology-only ball child joints, SDF 1.11+ mimic metadata for axis/axis2
follower joints with motor
enforcement, explicit parent-world root joints for supported SDF joint types
(revolute, continuous, prismatic, screw, universal, and topology-only ball),
multiple root FreeJoint trees, mixed implicit FreeJoint plus explicit
parent-world root models, model static/mobile state, model self-collision,
non-default skeleton gravity through root SDF world gravity, link gravity mode,
inertial parameters, local joint/shape poses,
box/sphere/cylinder/capsule/cone/ellipsoid/mesh visual or collision geometry,
visual shadow state as SDF `<cast_shadows>`, hidden visual state as zero-valued
SDF `<visibility_flags>`, visual alpha as SDF `<transparency>`, explicit visual
material colors as SDF `<diffuse>` values, and PBR metallic/roughness factors as
SDF `<pbr><metal>` values. Model static/mobile state round-trips through SDF
`<static>`, and model-level self-collision round-trips through SDF
`<self_collide>`.
Non-default skeleton gravity writes a root SDF `<world>` with `<gravity>` and
re-parses through the same `sdf::World` DOM.
Shape-level and body-level collision disable state round-trips through SDF
`<surface><contact><collide_bitmask>` for DART's lossless zero-bitmask subset;
zero-threshold bounce restitution round-trips through SDF
`<surface><bounce>` coefficient values; ODE friction coefficients, first
friction direction, and slip compliance round-trip through SDF
`<surface><friction><ode>` values when the first friction direction is
representable in the collision frame. Non-finite slip compliance and
non-collision-frame friction directions fail with targeted diagnostics.
Absolute non-file mesh URI
preservation is covered through a custom retriever-backed write/read test, and
URI-backed mesh material variants are covered by a write/read test that
preserves the source `sdf::Mesh` URI and re-loads the original multi-material
glTF resource. Targetless relative mesh references, URI-less in-memory mesh
material variants, and relative or host-qualified `file` mesh URIs are rejected
because the writer has no destination SDF URI for resource resolution or
generated asset placement. `WriteOptions`
visual/collision filtering is covered by focused tests. Unsupported constructs,
missing mesh URIs, non-finite mesh scales, pre-SDF-1.11 mimic output,
coupler-style mimic enforcement, non-finite material colors, invalid PBR
material factors, non-default visual reflectance, non-default DART mesh
color/alpha render policies, NaN joint position limits, non-finite screw pitch,
non-finite skeleton gravity, shape poses, inertial data, or joint axes, invalid
collision-surface
friction, slip, restitution, or
friction-direction frame, unsupported ball-joint metadata, unsupported DART
`SoftBodyNode` export, and non-finite joint dynamics return `common::Result`
errors instead of being silently dropped.
DART `PlaneShape`, `HeightmapShape`, and `ConvexMeshShape` now report targeted
policy diagnostics for the missing finite SDF plane size, source heightmap
URI/resource policy, or destination URI for generated mesh resources.
Additional DART-native geometry families that do not map to an sdformat-owned
primitive or targetless resource, including `PyramidShape`, `ArrowShape`,
`MultiSphereConvexHullShape`, `PointCloudShape`, `LineSegmentShape`, and
`VoxelGridShape`, also fail with targeted diagnostics instead of falling back to
generic unsupported-shape or mesh-URI errors.

The SDF writer integration test now uses
`tests/helpers/io_round_trip_helpers.hpp` for reusable body, joint, DoF,
inertia, and shape assertions. Future writer tests should extend that helper
instead of copying SDF-specific ad hoc checks.

Parser-side SDF import now uses libsdformat DOM objects for top-level model or
first-world model selection, world gravity, ambiguous `.xml` SDF dispatch
through `dart::io::readSkeleton()`, and standard model/link/joint/aspect
traversal. Ambiguous XML dispatch asks sdformat to classify root-model and
world-contained SDF documents before the non-SDF URDF/MJCF XML-root fallback
runs. The remaining authored/default presence checks use sdformat's
`Element::GetExplicitlySetInFile()` signal with non-mutating direct child
traversal for standard SDF inertial, material diffuse, legacy
`use_parent_model_frame`, and joint-axis dynamics/limits. sdformat Element
presence reads remain only for that authored/default bridge and DART-specific
soft-body extension fields; the compatibility boolean value is read through
sdformat typed element access. Modern `axis/xyz@expressed_in` frame annotations are
resolved through `sdf::JointAxis::ResolveXyz()`, and authored diffuse values are
read from `sdf::Material::Diffuse()`. Visual shadow and zero-visibility hidden
state are read through `sdf::Visual` DOM values. Authored collision-surface
contact bitmask, bounce restitution, and ODE friction values are read through
`sdf::Collision`, `sdf::Surface`, `sdf::Contact`, `sdf::Friction`, and
`sdf::ODE` DOM values plus sdformat Element typed access for the bounce schema
field that sdformat 16 does not expose as a high-level DOM class. Writer
diagnostics now cover non-finite collision-surface friction directions before
they can produce malformed SDF. Visual transparency reads use
`sdf::Visual::Transparency()`. Those values map into DART visual, collision,
and dynamics aspect fields. The SDF-specific
helper surface has been narrowed accordingly: it no longer exposes generic XML
attribute, string, boolean, vector2/vectorX, child enumerator, scalar, vector,
or pose parser APIs that duplicate sdformat traversal/parsing. Retained helpers
cover only authored element presence/lookup for fields the high-level DOM does
not expose; remaining DART-specific soft-body extension values use local
sdformat typed `Element::Get<T>` calls instead of DART-side XML text fallback
parsing.
Writer-side preservation of disabled link gravity and schema-preferred SDF
1.10+ screw pitch still patches sdformat's serialized element tree, but the
link/joint names and values now come from the typed `sdf::Model`,
`sdf::Link`, and `sdf::Joint` DOM rather than from serialized XML attributes,
and required patch targets are found by sdformat `Element::FindElement()`
lookup plus sdformat sibling iteration for repeated generated children.
The SDF writer tests also keep semantic checks on libsdformat by reparsing
emitted SDF through `sdf::Root::LoadSdfString()` and inspecting typed DOM
objects. Serialized XML substring assertions are reserved for element-name
contracts that sdformat normalizes semantically, such as legacy
`<thread_pitch>` versus modern `<screw_thread_pitch>`.

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
  tests can prove additional joint aliases or edge cases beyond continuous
  revolute joints, additional sdformat-owned shapes with complete resource
  policies such as SDF heightmaps, URI-less or generated mesh material
  variants, additional collision-surface fields beyond the already-covered
  contact bitmask, zero-threshold bounce restitution, and ODE friction/slip,
  destination-aware resource rewriting, or other world-level data beyond
  gravity round-trip correctly.
- Decide the next implementation target: URDF writer or PLAN-101 project
  save/load.
- Keep YAML out of the first implementation target unless a durable
  project/scene schema is accepted first.
