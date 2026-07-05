# SKEL Format Evolution — Dev Task

## Current Status

- [x] Phase 0: Capture strategic decision + prototype reference.
- [x] Phase 1: Convert live SKEL fixtures used by examples, tests, and
      benchmarks to SDF. PR
      [#3065](https://github.com/dartsim/dart/pull/3065) landed on `main` at
      `5dbe995d221`, adding SDF replacements for the referenced
      `single_pendulum`, `cube`, `shapes`, and `test_shapes` fixtures, moving
      tests and resource retriever coverage off `dart://sample/skel`, and
      verifying examples/tutorials/python examples have no SKEL fixture loads.
- [x] Phase 2: Remove SKEL from DART 7 `main`: parser implementation,
      `dart::io` format inference and dispatch, bindings/stubs, parser-specific
      tests, `.skel` sample fixtures, and user-facing DART 7 docs. The Phase 2
      branch relocates the non-SKEL Kima Collada fixtures from `data/skel/kima/`
      to `data/mesh/kima/` before deleting the legacy `data/skel/` tree.
- [x] Phase 3: Optional YAML model/scene format. Decision recorded in
      [`03-yaml-decision.md`](03-yaml-decision.md): do not add YAML as a DART 7
      `dart::io` model format or as a front-end syntax for URDF/SDF/MJCF. If
      YAML is revisited, it must be a DART-owned project or scene
      serialization format with import/export round-trip tests, not SKEL syntax
      in YAML.
- [x] Phase 4: USD support coordination. Decision recorded in
      [`04-usd-coordination.md`](04-usd-coordination.md): this task uses USD as
      design pressure, while USD loader/viewer/dartpy implementation remains
      owned by [`usd_scene_loader/`](../usd_scene_loader/).
- [ ] Phase 5: Export writers for accepted portable/project formats so DART can
      round-trip a scene back to text. Planning is recorded in
      [`05-export-writers-plan.md`](05-export-writers-plan.md). The first SDF
      writer slice is implemented locally for a conservative `Skeleton` subset
      using libsdformat DOM serialization; link gravity mode, passive joint
      dynamics metadata (damping, Coulomb friction, spring reference, and spring
      stiffness), symmetric axis effort/velocity limits,
      sdformat-normalized screw thread pitch, SDF 1.11+ axis/axis2 mimic
      metadata, universal two-axis joints, continuous revolute joints,
      explicit parent-world root joints for supported SDF joint types,
      including continuous revolute roots, multiple root FreeJoint trees, mixed
      implicit FreeJoint plus explicit parent-world root models,
      topology-only ball joints, model static/mobile state, model
      self-collision, non-default skeleton gravity through root SDF world
      gravity, capsule/cone/ellipsoid geometry,
      visual shadow/hidden state,
      explicit visual material colors, PBR metallic/roughness factors, visual
      transparency, shape-level and body-level collision disable bitmasks,
      zero-threshold bounce restitution, and ODE friction/slip
      metadata now round-trip as SDF `<self_collide>`, root
      `<world><gravity>`, link `<gravity>`,
      `<axis>/<axis2><dynamics>`, `<thread_pitch>`/`<screw_thread_pitch>`,
      `<mimic>`, local
      `<pose>`, `<capsule>`, `<cone>`, `<ellipsoid>`, `<cast_shadows>`,
      zero-valued `<visibility_flags>`, `<transparency>`, `<diffuse>`, and
      `<pbr><metal>` values plus `<surface><contact><collide_bitmask>` disables
      plus `<surface><bounce>` restitution coefficients and
      `<surface><friction><ode>` `mu`, `mu2`, `fdir1`, `slip1`, and `slip2`,
      including a combined collision-surface entry that carries contact,
      friction/slip, and bounce metadata together.
      Absolute non-file mesh URIs now have writer read/write/read coverage
      through a custom retriever, and URI-backed mesh material variants now have
      writer read/write/read coverage through preserved source mesh URIs. Writer
      options for excluding visuals or
      collisions, empty skeleton diagnostics, empty or malformed SDF version
      diagnostics, missing mesh URI and non-finite mesh scale diagnostics,
      pre-SDF-1.11 mimic diagnostics, unsupported coupler-style mimic
      diagnostics, and relative/generated mesh resource diagnostics now have
      focused coverage. NaN joint position limits now fail with explicit
      diagnostics instead of being treated like unbounded SDF limits. Symmetric
      joint-axis effort/velocity limits now have read/write/read coverage, and
      asymmetric or NaN effort/velocity limits fail with explicit diagnostics
      because SDF stores those fields as maximum absolute values. Invalid
      collision-surface
      friction, slip, restitution, and non-collision-frame friction-direction
      values, non-default visual reflectance, and non-default DART mesh
      color/alpha render policies now fail with explicit diagnostics.
      Non-finite skeleton gravity, shape poses, non-positive body masses,
      non-finite inertial data, and joint axes are also covered as writer
      diagnostics instead of being serialized into malformed SDF. Unsupported
      child `FreeJoint` writer attempts and
      unsupported `EulerJoint`, `PlanarJoint`, `TranslationalJoint2D`, and
      `TranslationalJoint` writer attempts now fail with targeted diagnostics
      instead of relying on generic unsupported-joint fallbacks. DART
      `SoftBodyNode` writer attempts now fail with a targeted diagnostic
      instead of being serialized as ordinary SDF links with point-mass, spring,
      damping, and soft mesh topology semantics dropped. The
      SDF writer also rejects DART-only/generated geometry families such as
      `PyramidShape`, `ArrowShape`, `MultiSphereConvexHullShape`,
      `PointCloudShape`, `LineSegmentShape`, and `VoxelGridShape` with
      targeted diagnostics until a destination-aware generated-resource policy
      exists. The
      writer test now uses a shared IO round-trip assertion helper for body,
      joint, DoF,
      inertia, and shape comparisons.
      DART `PlaneShape`, `HeightmapShape`, and `ConvexMeshShape` writer
      attempts now fail with targeted diagnostics explaining the missing finite
      SDF plane size, source heightmap URI/resource policy, or target URI for
      generated mesh resources instead of relying on a generic
      unsupported-shape fallback.
      Parser-side SDF model selection, including parser-specific
      `SdfParser::Options::mModelName` lookup of named world models through
      `sdf::World::ModelByName()`, ambiguous `.xml` SDF dispatch through
      `dart::io::readSkeleton()`, and standard model/link/joint/aspect traversal
      now use libsdformat `sdf::Root`, `sdf::Model`, `sdf::World`,
      `sdf::Link`, `sdf::Joint`,
      `sdf::Visual`, and `sdf::Collision` DOM values rather than raw XML-level
      enumeration. Ambiguous XML dispatch asks sdformat to classify root-model
      and world-contained SDF documents via `sdf::Root` model/world APIs before
      the non-SDF URDF/MJCF XML-root fallback runs. Standard
      visual/collision/material reads use libsdformat DOM
      values for names, local poses, geometry, cast-shadow state, zero
      visibility flags, transparency, diffuse colors, and PBR metal workflow
      factors; material authored/default checks only preserve whether diffuse
      was authored, using sdformat's explicit-authored element flags instead of
      child-existence probing. Standard joint reads use DOM values
      for joint name/type,
      parent/child links, local pose, axis vectors including
      `axis/xyz@expressed_in` frame resolution, axis dynamics, finite limits,
      mimic metadata, and sdformat-normalized screw pitch values; sdformat
      Element presence checks remain only where DART needs authored/default
      presence, DART-specific soft-body extension fields, or the legacy
      `use_parent_model_frame` presence check; authored/default checks now use sdformat
      `Element::GetExplicitlySetInFile()`, and the legacy boolean value itself
      uses sdformat typed element access.
      The SDF-specific helper layer no longer exposes generic XML attribute,
      string, boolean, vector2/vectorX, child-enumerator, scalar, vector, or
      pose parser APIs; retained helpers are limited to authored sdformat
      element presence/lookup for fields the high-level DOM does not expose.
      That bridge uses sdformat explicit-authored flags for standard SDF
      presence checks, locates retained bridge children with sdformat
      `Element::FindElement()`, and converts remaining DART-specific soft-body
      extension values through sdformat typed `Element::Get<T>` calls rather
      than XML text reparsing.
      A lint/check-lint guard, `pixi run check-sdf-sdformat-boundary`, now
      rejects TinyXML/raw XML parser APIs and generic SDF element text-parsing
      helpers in `dart/utils/sdf` so the sdformat boundary is enforced instead
      of only documented.
      Writer-side preservation of disabled link gravity and SDF 1.10+
      `<screw_thread_pitch>` now derives names and values from typed
      `sdf::Model` / `sdf::Link` / `sdf::Joint` DOM objects instead of reading
      serialized XML attributes back out of sdformat's element tree, and its
      required element-tree patching now uses sdformat `Element::FindElement()`
      lookups for child selection plus sdformat sibling iteration for repeated
      generated children.
      SDF writer tests validate standard semantics by reparsing emitted SDF
      through `sdf::Root::LoadSdfString()` and inspecting libsdformat DOM
      objects; literal XML substring checks are retained only for serialized
      schema element-name contracts that sdformat normalizes semantically, such
      as legacy `<thread_pitch>` versus modern `<screw_thread_pitch>`.
      Existing shipped SDF fixture coverage now loads the converted SKEL
      fixtures (`single_pendulum.sdf`, `cube.sdf`, `shapes.sdf`, and
      `test_shapes.sdf`) plus the native root-model `quad.sdf` quadruped
      fixture, the native two-link revolute SDF fixture
      (`two_link_revolute_model.sdf`), the root-model issue fixtures
      (`test_issue1583.model`, `test_issue1596.model`, and
      `test_issue1683.model`), the included relative mesh model fixture
      (`include_relative_mesh/included_model/model.sdf`), the top-level
      `ground.world` fixture, and the
      world-contained
      `issue1193_revolute*.sdf`, `high_version.world`, and
      `single_bodynode_skeleton.world` fixtures plus
      `test_skeleton_joint.world`, `force_torque_test.world`, and
      `force_torque_test2.world`, plus the selected
      `pendulum_with_base_mimic_slow_follows_fast` model from
      `mimic_fast_slow_pendulums_world.sdf` and the selected
      `double_pendulum_with_base` model from `double_pendulum*.world`, writes
      them back through
      `SdfParser::tryWriteSkeletonToString()`, reloads the emitted text, and
      compares body, joint, inertial, mobility, gravity, axis-limit,
      joint-dynamics, visual geometry, collision geometry, model-pose, and
      joint-offset semantics of the original parsed skeletons against the
      re-parsed writer outputs. The issue fixture coverage proves fixed
      parent-world root semantics recovered through `RootJointType::Fixed`,
      model poses, child revolute axes and limits, parent-world and child
      universal joints, and box/sphere/cylinder visual and collision geometry.
      The relative mesh fixture coverage proves parser-resolved source mesh URI
      preservation for visual and collision mesh geometry. The top-level
      `ground.world` fixture proves imported SDF plane-as-DART-box semantics,
      static world model state, disabled visual shadow state, and high ODE
      friction metadata survive writer read/write/read. The force-torque
      world fixture coverage is
      limited to DART skeleton semantics imported from the in-file models; it
      does not claim SDF sensor or physics metadata preservation. The selected
      mimic world fixture proves the sdformat-backed parser selector can cover
      a non-first world model while preserving SDF 1.11 axis mimic metadata.
      The selected double-pendulum world fixtures prove the same
      sdformat-backed parser selector covers non-first world models with
      parent-world revolute roots, child revolute joints, visual-only pendulum
      geometry, and visual/collision pendulum geometry without XML-level SDF
      model enumeration.
      The first URDF writer slice is also implemented locally on
      `dart::utils::UrdfParser::tryWriteSkeletonToString` for one-root URDF
      trees with identity root FreeJoint/WeldJoint placement validation, child
      revolute/continuous/prismatic/fixed joints, continuous joint
      velocity/effort limit metadata, standard-plane planar and floating child
      joints with uniform scalar limit/dynamics metadata, passive
      damping/friction metadata, single-DoF motor-style mimic metadata,
      zero-offset coupler mimic metadata through paired URDF
      `SimpleTransmission` entries, inertial data, local visual/collision
      poses, box/sphere/cylinder/absolute or package URI mesh geometry,
      explicit visual colors, implicit default visual color omission,
      default-RGB alpha overrides, and visual/collision include options. URDF
      does not serialize
      root parent-joint metadata, so root joint name/type are parser-default
      choices on reparse rather than writer output.
      `INTEGRATION_io_UrdfWriter`
      validates a write/read/read subset and diagnostics for empty skeletons,
      multiple root trees, unsupported root joint families, non-identity root
      placement, unsupported child joint families, arbitrary planar axes,
      non-uniform multi-DoF limit/dynamics metadata, non-identity child joint
      frames, unbounded finite-requiring URDF limits, missing mimic references,
      coupler mimic offsets, plus missing mesh URIs, relative or host-qualified
      file mesh URIs, and non-finite mesh scales. It also covers continuous
      revolute joint dynamics round-trip, shipped
      `joint_properties.urdf`, `issue838.urdf`, `KR5/ground.urdf`, and
      `wam.urdf` plus the package-mesh `drchubo.urdf` robot
      read/write/read coverage for revolute and continuous joint
      limits/dynamics, visual boxes, global material color references,
      imported root-`world` frame semantics, collision boxes, and visual and
      collision package mesh URI preservation through `UrdfParser` package
      resolution. The full `KR5 sixx R650.urdf` fixture now has shipped-fixture
      diagnostic coverage proving its non-identity root-link pose is rejected
      explicitly. It also covers non-positive mass, non-finite local
      center-of-mass, visual
      material color, shape pose, joint axis, and asymmetric velocity/effort
      limit diagnostics, targeted diagnostics for non-default visual
      reflectance, disabled collision aspects, and collision dynamics metadata
      that URDF cannot represent, and targeted DART `SoftBodyNode` diagnostics.
      Writer APIs stay format-owned for now: the SDF writer remains on
      `dart::utils::SdfParser`, the URDF writer remains on
      `dart::utils::UrdfParser`, `dart::io` stays read-side, and project/editor
      save-load belongs to the scene/project layer. Broader SDF/URDF coverage
      and project save/load remain open. Add YAML only if a durable project/scene
      schema is accepted first.

## Goal

Close out [issue #496](https://github.com/dartsim/dart/issues/496) (2015
proposal to convert SKEL XML to YAML) with a decision _not_ to YAML-ify SKEL,
and instead remove SKEL before the DART 7 release. Invest the same engineering
budget into verified fixture migration, URDF/SDF/MJCF coverage, the YAML
non-adoption decision, USD adoption, and format-export so DART scenes can leave
the engine in portable formats.

## Non-Goals

- Re-implementing SKEL as YAML. Issue #496's literal proposal is rejected;
  the rationale is in _Key Decisions_ below.
- Keeping SKEL support in DART 7 for compatibility. Legacy users should stay on
  a `release-6.*` branch until their assets are migrated.
- Building a YAML schema in this task. YAML remains rejected for DART 7
  `dart::io` unless a future durable project/scene owner accepts its
  maintenance, export, and round-trip contracts.

## Key Decisions

- **SKEL is legacy and should leave DART 7**. The format is verbose,
  string-based, and has no installed-base outside DART. Industry has converged
  on URDF (ROS), SDF (Gazebo), and MJCF (MuJoCo); DART 7 should not carry a
  DART-only XML format forward.
- **YAML is not a DART 7 `dart::io` model format**. Phase 3 rejects
  `ModelFormat::Yaml`, YAML wrappers over URDF/SDF/MJCF, and any SKEL syntax
  carryover. If YAML is revisited, it belongs in a durable project/scene
  serialization design with schema/versioning rules and import/export
  round-trip tests.
- **USD is the medium-term scene format**. Pixar / NVIDIA convergence makes
  USD the most likely cross-engine scene format. Loader work is tracked
  separately under [`usd_scene_loader/`](../usd_scene_loader/); Phase 4 records
  the coordination boundary so SKEL evolution does not duplicate that surface.
- **Export is part of the work**. URDF/SDF writers, plus any accepted
  DART-owned project/scene writer, turn DART into a round-trip engine rather
  than read-only, which unblocks save / load in the dartsim editor (PLAN-101)
  and downstream pipelines like gz-physics.

## Prototype Reference

The original prototype lived on the now-deleted `feature/skel_yaml` branch.
Its tip commit was:

- `1dd83e31586 wip` — added 6 files (1,991 lines total) under
  `docs/dev_tasks/skel_format/`: `README.md`, `migration-guide.md`,
  `phase-01-deprecation.md`, `phase-02-yaml.md`, `phase-03-usd.md`, and
  `phase-04-export.md`. The detailed phase planning, deprecation timing
  table, YAML library tradeoff matrix, USD schema notes, and export round-
  trip strategy lived there.

Current reality: this worktree no longer has the prototype object in reflog or
`git fsck --unreachable`; `git show 1dd83e31586:...` fails here. Treat the SHA
as archaeology only unless another clone still has the object. Phase 3 and Phase
5 must be decided from current DART 7 requirements and fresh evidence, not by
assuming the old SKEL-YAML prototype is recoverable.

If another clone still has the object, these paths may be useful context:

```bash
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-01-deprecation.md
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-02-yaml.md
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-03-usd.md
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-04-export.md
git show 1dd83e31586:docs/dev_tasks/skel_format/migration-guide.md
```

This compact dev task is the durable strategic frame. Do not recreate the old
SKEL-YAML direction just because the prototype once existed.

## Immediate Next Steps

1. Land the Phase 2 removal branch (`feature/remove-skel-dart7-phase2`) after
   review. Add the PR link to the DART 7 changelog entry before merge.
2. Continue Phase 5 from
   [`05-export-writers-plan.md`](05-export-writers-plan.md): extend SDF or URDF
   writer coverage beyond the first conservative subsets, or move to PLAN-101
   project save/load, and keep read/write/read tests attached to every expanded
   contract. Absolute non-file mesh URI preservation is covered; targetless
   relative/generated mesh references are rejected until a future file/project
   writer defines a destination URI and copy/rewrite policy. Heightmap export
   also needs a source heightmap URI and destination-aware resource policy before
   DART can use `sdf::Heightmap` DOM serialization. Keep writer APIs
   format-owned unless a later multi-format write API is reviewed. YAML can
   enter that slice only after a durable project/scene schema is accepted.

## Verification Gates

- Phase 1 (landed via PR #3065): focused IO/simulation/resource retriever tests
  for the converted SDF fixtures; `rg` checks proving examples/tests no longer
  reference live SKEL fixtures; `pixi run check-docs-policy`; `pixi run lint`.
- Phase 2: focused tests proving `.skel` no longer dispatches through
  `dart::io`, parser-specific SKEL tests are removed, and the source tree no
  longer ships DART 7 sample `.skel` assets. Existing non-SKEL mesh tests now
  load the relocated Kima fixtures from `data/mesh/kima/*.dae`.
- Phase 3: decision-only gate recorded in
  [`03-yaml-decision.md`](03-yaml-decision.md): live issue state, current
  `dart::io` source/docs, upstream format docs, USD task ownership, and PLAN-101
  project round-trip evidence all support not implementing a YAML parser now.
  Any future accepted YAML design needs focused parser, export, and round-trip
  tests before implementation is considered complete.
- Phase 4: decision-only coordination gate recorded in
  [`04-usd-coordination.md`](04-usd-coordination.md): PR #3109 is merged, the
  active `usd_scene_loader/` task owns remaining USD implementation, and this
  task must not duplicate the USD loader/viewer/dartpy surface.
- Phase 5: planning gate recorded in
  [`05-export-writers-plan.md`](05-export-writers-plan.md). The first SDF writer
  slice is covered by `INTEGRATION_io_SdfWriter`, which writes and re-reads a
  representative supported `Skeleton` subset and checks unsupported-shape
  diagnostics plus non-finite visual material, invalid PBR material, and
  unsupported visual reflectance diagnostics. It also proves link-level gravity
  mode for gravity-disabled links and passive joint dynamics metadata plus
  symmetric axis effort/velocity limits and sdformat-normalized screw thread
  pitch for supported single-axis joints, two-axis universal joints, SDF 1.11+
  axis/axis2 mimic metadata with motor enforcement, continuous revolute joints,
  explicit parent-world root
  joints for supported SDF joint types, including continuous revolute roots,
  multiple root FreeJoint trees, mixed implicit FreeJoint plus explicit
  parent-world root models, topology-only ball joints, model static/mobile
  state, model self-collision, non-default skeleton gravity through root SDF
  world gravity,
  plus local root, joint, and shape poses, plus capsule/cone/ellipsoid geometry,
  visual shadow state, zero visibility flags, visual transparency, and absolute
  non-file mesh URI
  preservation through a custom retriever plus URI-backed mesh material
  variants through preserved source mesh URIs. It also covers sdformat-owned
  ambiguous `.xml` SDF dispatch for root-model and world-contained models,
  `WriteOptions`
  visual/collision filtering, empty skeleton diagnostics, empty or malformed
  SDF version diagnostics, the empty mesh URI and non-finite mesh scale
  diagnostics, pre-SDF-1.11 mimic
  diagnostics, and unsupported coupler-style mimic diagnostics, plus targetless
  relative/generated mesh resource diagnostics. Plane-shaped infinite DART
  geometry, DART heightmaps without source heightmap URIs, and generated convex
  meshes now have targeted diagnostics for the finite SDF plane-size and
  destination-URI/resource policy gaps. Non-default DART mesh color/alpha
  render policies now have targeted diagnostics because SDF mesh DOM objects
  preserve URI/scale/submesh selection, not DART's runtime render-policy modes.
  DART `SoftBodyNode` export now has a targeted diagnostic because this SDF
  writer has no sdformat-backed contract for DART point masses, springs,
  damping, or soft mesh topology.
  DART-only/generated geometry families such as `PyramidShape`, `ArrowShape`,
  `MultiSphereConvexHullShape`, `PointCloudShape`, `LineSegmentShape`, and
  `VoxelGridShape` now have targeted diagnostics for missing SDF primitives or
  targetless generated-resource policies.
  Shipped-fixture read/write/read coverage now includes the native
  `two_link_revolute_model.sdf` sample, proving the current sdformat-backed
  writer preserves a root link, child revolute joint, finite axis
  position/velocity/effort limits, inertias, and box/cylinder visual and
  collision geometry without XML-level SDF parsing. It also includes the
  world-contained `issue1193_revolute*.sdf` fixtures, proving sdformat-backed
  model-pose, root transform, child joint-offset, zero-world-gravity, and
  visual box/sphere semantics survive read/write/read. The shipped
  `quad.sdf` fixture adds read/write/read coverage for a root-model quadruped
  with 17 links, 16 revolute joints, finite axis velocity/effort limits, visual
  material colors, box visual/collision geometry, and foot sphere visuals. The
  shipped `test_issue1583.model`, `test_issue1596.model`, and
  `test_issue1683.model` fixtures add read/write/read coverage for root-model
  issue fixtures with fixed parent-world root semantics recovered through
  `RootJointType::Fixed`, model poses, child revolute axes and limits,
  parent-world and child universal joints, and box/sphere/cylinder visual and
  collision geometry. The
  shipped `include_relative_mesh/included_model/model.sdf` fixture adds
  read/write/read coverage for visual/collision mesh geometry with
  parser-resolved source mesh URI preservation. The top-level `ground.world`
  fixture adds read/write/read coverage for imported SDF plane-as-DART-box
  semantics, static world model state, disabled visual shadow state, and high
  ODE friction metadata. The
  shipped
  `high_version.world` and `single_bodynode_skeleton.world` fixtures add
  read/write/read coverage for simple world-contained single-body models with
  high-version SDF input, default-inertial fallback, root-joint semantics,
  gravity, and box/cylinder visual and collision geometry. The shipped
  `test_skeleton_joint.world` fixture adds read/write/read coverage for a
  world-contained mixed-joint model with prismatic, revolute, screw, and
  revolute2/universal joints plus cylinder visual and collision geometry. The
  shipped
  `force_torque_test.world` and `force_torque_test2.world` fixtures add
  read/write/read coverage for the DART skeleton semantics imported from
  sensor-bearing world files: explicit parent-world root semantics, child
  revolute joints, finite axis limits/dynamics, and sphere/box visual and
  collision geometry across two-link and three-link models. They do not claim
  SDF sensor or physics metadata preservation. The shipped
  `mimic_fast_slow_pendulums_world.sdf` fixture now adds read/write/read
  coverage for a selected non-first world model with SDF 1.11 axis mimic
  metadata, using sdformat DOM lookup rather than XML-level model enumeration.
  The shipped `double_pendulum.world` and `double_pendulum_with_base.world`
  fixtures now add read/write/read coverage for selected non-first world
  models with parent-world revolute roots, child revolute joints, visual-only
  pendulum geometry, and visual/collision pendulum geometry, again using
  sdformat DOM lookup rather than XML-level model enumeration.
  URDF shipped-fixture read/write/read coverage now includes
  `joint_properties.urdf`, `issue838.urdf`, `KR5/ground.urdf`, `wam.urdf`, and
  `drchubo.urdf`,
  proving the current conservative URDF writer preserves revolute and
  continuous joint topology, axes, finite velocity/effort limits, passive
  damping/friction metadata, inertias, visual box geometry, visual material
  colors imported from global URDF material references, collision box geometry,
  visual/collision package mesh geometry and URIs, and parser-normalized
  root-`world` inertial-frame semantics. The full `KR5 sixx R650.urdf` robot
  remains unsupported because its imported root joint has a non-identity
  root-link pose, and that shipped fixture now has explicit diagnostic coverage.
  NaN position limits on SDF-supported single-axis joints and topology-only ball
  joints now have targeted diagnostics, while infinite limits remain the
  unbounded SDF joint contract.
  Non-finite skeleton gravity, shape pose, non-positive body mass, inertial
  local center-of-mass data, joint axis inputs, unsupported child `FreeJoint`s,
  and unsupported
  `EulerJoint`, `PlanarJoint`, `TranslationalJoint2D`, and
  `TranslationalJoint` families now have focused diagnostics coverage.
  Shape-level and body-level
  collision-surface contact bitmask disables, zero-threshold bounce restitution,
  ODE friction
  coefficients, collision-frame first friction direction, and slip compliance
  now have read/write/read coverage both independently and in a combined
  collision-surface entry, plus invalid friction, slip, restitution,
  friction-direction value/frame, and non-default visual-reflectance
  diagnostics. The
  assertions are
  factored
  through
  `tests/helpers/io_round_trip_helpers.hpp` so future writer formats can reuse
  the body, joint, DoF, inertia, and shape comparison helpers. Completion still
  requires broader accepted-format writer APIs and round-trip tests that load a
  scene from each accepted format, write it back, and compare the re-parsed
  models. SDF helper cleanup is covered by `test_sdf_helpersNone`, which now
  validates only the retained sdformat Element presence/lookup bridge instead
  of the removed generic parser helper APIs, including sdformat
  `Element::FindElement()` lookup, explicit-authored presence checks, and the
  removed SDF-specific boolean, scalar, vector, pose, and text fallback parsers.
  SDF writer patching of disabled link gravity, modern screw pitch, and bounce
  restitution is covered by `INTEGRATION_io_SdfWriter` and now uses sdformat
  `Element::FindElement()` lookup for child selection.
