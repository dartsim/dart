# Resume: SKEL Format Evolution

## Current Resume Checkpoint (2026-07-05)

`work/skel-format-yaml-decision` is the stacked writer/decision branch. It now
has `origin/main` merged locally after Phase 2 landed as PR
[#3288](https://github.com/dartsim/dart/pull/3288) at
`5c75381f79a`. The remaining branch owns Phase 3/4/5 decision and writer
closeout work on top of the merged SKEL-removal baseline.

The Phase 2 branch's only merge conflict against `origin/main` was in
`docs/dev_tasks/usd_scene_loader/README.md`; it was resolved by keeping the new
OpenUSD blocker/specification-intake text from `main` while removing SKEL from
the DART 7 parity/front-door wording. The local 2026-07-05 Phase 2 validation
pass completed:

- lightweight invariants: SKEL parser/source files absent, `data/skel` absent,
  Kima Collada meshes present under `data/mesh/kima/*.dae`;
- `pixi run python -m pytest python/tests/unit/test_check_dartpy_import_layout.py python/tests/unit/utils/test_utils_stub_import.py python/tests/unit/test_check_dart7_world_promotion_blockers.py`
  — 28 passed;
- `pixi run check-ai-commands`;
- `pixi run check-dartpy-import-layout`;
- `pixi run run-cpp-target INTEGRATION_io_Read` — 7 passed, including
  `Read.SkelIsNotSupported`;
- `pixi run run-cpp-target UNIT_dynamics_MeshShape` — 49 passed;
- `pixi run run-cpp-target test_mesh_loaderNone` — 5 passed;
- `pixi run lint`;
- `pixi run build`;
- `pixi run test-all` — 6/6 phases passed: linting, build, unit tests,
  simulation tests, Python tests, and documentation;
- `pixi run -e cuda test-all` — 7/7 phases passed on the host RTX 5000 Ada
  GPU, including CUDA tests and benchmark smoke.

The sdformat-boundary checker is not part of the Phase 2 removal branch; it
lives on this stacked writer branch, which owns the SDF IO boundary work.

The post-PR-#3288 stack refresh on 2026-07-05 resolved conflicts by keeping the
sdformat-backed ambiguous SDF XML classifier on this branch, keeping the SDF
writer integration target, and updating this task to treat PR #3288 as merged.
Validation for that refresh passed:

- `git diff --check`;
- `git diff --cached --check`;
- `pixi run check-sdf-sdformat-boundary`;
- `pixi run lint`.

The earlier 2026-07-05 local stack-refresh validation also passed on
`work/skel-format-yaml-decision` after merging the refreshed Phase 2 branch:

- `git diff --check`;
- `git diff --cached --check`;
- `pixi run check-sdf-sdformat-boundary`;
- `pixi run run-cpp-target INTEGRATION_io_Read` - 7 passed;
- `pixi run run-cpp-target INTEGRATION_io_SdfParser` - 42 passed;
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter` - 97 passed;
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` - 45 passed;
- `pixi run lint`;
- `pixi run build`.

## Phase 2 PR Record (merged)

PR [#3288](https://github.com/dartsim/dart/pull/3288) merged on 2026-07-05.
The historical PR body was:

Title:

```text
Remove legacy SKEL format from DART 7
```

Base/head:

```text
base: main
head: feature/remove-skel-dart7-phase2
milestone: DART 7.0
```

Body:

```markdown
## Summary

- Remove the legacy SKEL model format from DART 7 and route users to URDF,
  SDF, MJCF, or a `release-6.*` branch for old SKEL assets.

## Motivation / Problem

- SKEL is a DART-only legacy XML format. DART 7 should not carry it forward or
  redesign it as YAML while the maintained interchange paths are URDF, SDF,
  MJCF, and future project/scene serialization.

## Changes / Key Changes

- Removed `SkelParser`, `ModelFormat::Skel`, `.skel` / `<skel>` format
  inference, and direct SKEL dispatch from `dart::io`.
- Removed `.skel` sample assets, parser-specific tests, dartpy bindings/stubs,
  and user-facing SKEL docs.
- Moved the non-SKEL Kima Collada meshes from `data/skel/kima/` to
  `data/mesh/kima/`.
- Added coverage that `.skel` files are not supported through
  `dart::io::readSkeleton()`.
- Updated DART 7 docs and changelog guidance for migrating SKEL assets to URDF,
  SDF, or MJCF.

## Testing

- `pixi run python -m pytest python/tests/unit/test_check_dartpy_import_layout.py python/tests/unit/utils/test_utils_stub_import.py python/tests/unit/test_check_dart7_world_promotion_blockers.py`
- `pixi run check-ai-commands`
- `pixi run check-dartpy-import-layout`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run run-cpp-target UNIT_dynamics_MeshShape`
- `pixi run run-cpp-target test_mesh_loaderNone`
- `pixi run lint`
- `pixi run build`
- `pixi run test-all`
- `pixi run -e cuda test-all`

## Breaking Changes

- [ ] None
- Removes DART 7 SKEL loading and the `SkelParser` API. Migrate assets to URDF,
  SDF, or MJCF, or use the active `release-6.*` branch for legacy SKEL
  compatibility.

## Related Issues / PRs (backports)

- Closes https://github.com/dartsim/dart/issues/496
- Phase 1 fixture conversion: https://github.com/dartsim/dart/pull/3065

---

#### Checklist

- [ ] Milestone set (DART 7.0 for `main`, branch-matching DART 6.x release
      milestone for the active DART 6 LTS branch)
- [x] CHANGELOG.md updated per `docs/onboarding/changelog.md` if required
- [x] Add unit tests for new functionality
- [x] Document new methods and classes
- [x] Add Python bindings (dartpy) if applicable
```

The SKEL-removal changelog bullet now links to PR #3288.

Phase 3 is decided and committed locally on `work/skel-format-yaml-decision`.
The decision record is [`03-yaml-decision.md`](03-yaml-decision.md): do not add
YAML as a DART 7 `dart::io` model format, do not build a YAML wrapper over
URDF/SDF/MJCF, and do not carry SKEL syntax forward. If YAML returns, it must be
a DART-owned project/scene serialization design with schema/versioning rules
plus import, export, and round-trip tests.

Phase 4 coordination is also recorded locally in
[`04-usd-coordination.md`](04-usd-coordination.md): PR #3109 already landed the
OFF-by-default USD `dart::io` scaffold, and remaining USD loader/viewer/dartpy
implementation stays in `docs/dev_tasks/usd_scene_loader/` instead of this SKEL
task.

Phase 5 is complete for the bounded DART 7 writer scope recorded in
[`05-export-writers-plan.md`](05-export-writers-plan.md): parser-specific SDF and
URDF `Skeleton` writers exist with read/write/read coverage, PLAN-101
project/scene save-load is already implemented in the dartsim engine layer, and
future SDF/URDF writer expansion is parked behind the durable criteria in
`docs/onboarding/io-parsing.md`.

The SDF writer implementation is local on
`work/skel-format-yaml-decision`: `dart::utils::SdfParser` exposes
`tryWriteSkeletonToString()` for a conservative `Skeleton` subset, with
the writer building libsdformat DOM objects and serializing through sdformat.
`INTEGRATION_io_SdfWriter` proves write/read round-trip for links, root
FreeJoint/WeldJoint placement,
revolute/continuous/prismatic/weld/screw/universal child joints, inertial data,
box/sphere/cylinder/capsule/cone/ellipsoid/mesh geometry, link gravity mode,
non-default skeleton gravity through root SDF world gravity, passive joint
dynamics metadata (damping, Coulomb friction, spring reference, and spring
stiffness), symmetric axis effort/velocity limits, sdformat-normalized screw
thread pitch, SDF 1.11+ axis/axis2 mimic metadata with motor enforcement,
explicit parent-world root
joints for supported SDF joint types including continuous revolute roots,
multiple root FreeJoint trees, mixed implicit FreeJoint plus explicit
parent-world root models, topology-only ball child joints, model static/mobile
state, model self-collision, local root/joint/shape poses, visual
shadow/hidden state, explicit
visual material colors, PBR metallic/roughness factors, collision-surface
contact disable bitmasks for both shape-level and body-level DART collision
disable state, visual transparency, zero-threshold bounce restitution, ODE
friction coefficients, first friction direction, slip compliance, combined
collision-surface metadata entries, and absolute non-file mesh URI preservation
through a custom retriever, plus URI-backed mesh material
variants through preserved source mesh URIs. Continuous SDF joints now
parse as unbounded DART
`RevoluteJoint`s, and unbounded DART revolute joints write back as SDF
`continuous` while finite-limit revolute joints stay `revolute`. Targetless
relative/generated mesh references, including relative or host-qualified file
URI forms, now return explicit diagnostics until a future file/project writer
defines a destination URI and resource copy/rewrite policy. It also checks
`WriteOptions` visual/collision filtering, unsupported-shape diagnostics, empty
skeleton diagnostics, empty or malformed SDF version diagnostics, missing mesh
URI and non-finite mesh scale diagnostics, DART `PlaneShape` finite-size
diagnostics, `HeightmapShape`
source-URI/resource-policy diagnostics, `ConvexMeshShape` generated-resource
diagnostics,
DART-only/generated geometry diagnostics for `PyramidShape`, `ArrowShape`,
`MultiSphereConvexHullShape`, `PointCloudShape`, `LineSegmentShape`, and
`VoxelGridShape`, pre-SDF-1.11 mimic diagnostics,
unsupported coupler-style mimic diagnostics, non-finite visual material
diagnostics, invalid PBR material diagnostics, unsupported visual reflectance
diagnostics, non-default DART mesh color/alpha render-policy diagnostics,
NaN joint position-limit diagnostics, non-finite screw pitch diagnostics,
asymmetric or NaN joint effort/velocity limit diagnostics,
non-finite skeleton gravity, shape-pose, non-positive body-mass,
inertial-data, and joint-axis diagnostics, invalid collision-surface friction,
friction-direction values, slip, restitution, and friction-direction-frame
diagnostics, unsupported
ball-joint metadata, unsupported child `FreeJoint` diagnostics, unsupported
`EulerJoint`, `PlanarJoint`, `TranslationalJoint2D`, and `TranslationalJoint`
diagnostics, unsupported DART `SoftBodyNode` diagnostics, and non-finite joint
dynamics diagnostics. This closes the current SDF side of the bounded Phase 5
scope; broader SDF work is durably parked until a destination-aware resource
policy, libsdformat-backed semantic mappings, and new read/write/read tests are
accepted.

SDF shipped-fixture writer coverage now includes the native converted SKEL
fixtures, `quad.sdf`, `two_link_revolute_model.sdf`,
`test_issue1583.model`, `test_issue1596.model`, `test_issue1683.model`,
the included relative mesh model fixture, the top-level `ground.world` fixture,
world-contained
`issue1193_revolute*.sdf`, `high_version.world`,
`single_bodynode_skeleton.world`, `test_skeleton_joint.world`, and the
sensor-bearing `force_torque_test.world` / `force_torque_test2.world` fixtures.
These tests load through the libsdformat-backed DART SDF parser, write with
`SdfParser::tryWriteSkeletonToString()`, reparse the emitted SDF, and compare
imported `Skeleton` semantics only. The issue fixture coverage uses
`RootJointType::Fixed` on reparse to recover the fixed parent-world root
semantics represented by SDF static model state. The issue fixture coverage
also includes parent-world and child universal joints from
`test_issue1596.model`. The relative mesh model coverage compares
parser-resolved source mesh URIs for visual and collision mesh geometry. The
top-level include-driver relative mesh world fixture now proves included-model
relative mesh resources resolve against `sdf::Model::Uri()` rather than the
outer world URI before writer read/write/read. The top-level `ground.world`
coverage compares imported SDF plane-as-DART-box
semantics, static world model state, disabled visual shadow state, and high ODE
friction metadata. The
force-torque coverage does not claim SDF sensor or physics metadata
preservation.

Current SDF IO rule: keep SDF parsing, dispatch, semantic reads, and writing on
libsdformat APIs. The retained `sdf::Element` bridge is only for
authored/default checks, DART extension fields, and schema fields missing from
the high-level DOM; do not add DART-side XML tokenization, child enumeration,
or text reparsing for SDF semantics.

The URDF writer implementation is also local on
`work/skel-format-yaml-decision`: `dart::utils::UrdfParser` exposes
`tryWriteSkeletonToString()` for one-root URDF trees with identity root
FreeJoint or WeldJoint placement validation, child
revolute/continuous/prismatic/fixed joints,
standard-plane planar and floating child joints with uniform scalar
limit/dynamics metadata, passive damping/friction metadata, single-DoF
motor-style mimic metadata, inertial data, zero-offset coupler mimic metadata
through paired URDF `SimpleTransmission` entries, local visual/collision poses,
box/sphere/cylinder/absolute or package URI mesh geometry, explicit visual
colors, implicit default visual color omission, default-RGB alpha overrides, and
visual/collision include options. URDF does not serialize root parent-joint
metadata, so root joint name/type are parser-default choices on reparse rather
than writer output. `INTEGRATION_io_UrdfWriter` proves write/read/read
round-trip for that subset and covers explicit diagnostics for multiple root
trees, unsupported root joint families, non-identity root placement,
unsupported child joint families, arbitrary planar axes, non-uniform multi-DoF
limit/dynamics metadata, non-identity child joint frames, unbounded
finite-requiring URDF limits, missing mimic references, coupler mimic offsets,
plus missing mesh URIs, relative or host-qualified file mesh URIs, and
non-finite mesh scales. Additional focused coverage proves
unbounded DART revolute joints write as URDF `continuous` joints and preserve
passive dynamics metadata through reparse, proves visual and collision
`package://` mesh URIs serialize and reparse through `UrdfParser` package
resolution, and proves zero-offset DART coupler mimic relationships serialize as
URDF `SimpleTransmission` groups and reparse with coupler enforcement intact.
The Atlas `data/sdf/atlas/atlas_v5_no_head.urdf` fixture now has
read/write/read coverage for a 36-body, 35-DoF shipped robot fixture with
relative visual/collision meshes, representative revolute joints, fixed camera
joints, and body/inertia/mesh URI comparisons. This proves the current
conservative URDF writer handles a larger robot-scale fixture without changing
the writer contract.
Implicit default visual colors now serialize without authored URDF materials,
explicit default colors stay authored, and default RGB with an alpha override
still serializes a material so alpha round-trips. Non-positive mass, non-finite
local center-of-mass, visual material color, shape pose, joint axis, and
asymmetric velocity/effort limits now have focused diagnostics coverage.
Non-default visual reflectance, disabled DART collision aspects, and collision
dynamics metadata now fail with targeted diagnostics because URDF has no
equivalent material reflectance, collidable-disable, or
surface/contact-dynamics fields. DART `SoftBodyNode` writer attempts also fail
with a targeted diagnostic instead of being serialized as ordinary URDF links
with point-mass, spring, damping, and soft mesh topology semantics dropped.
Together with the SDF writer and PLAN-101 project save/load evidence, this
closes the current DART 7 Phase 5 writer scope; broader URDF work is parked
until a new representable contract and focused round-trip tests exist.

Current SDF audit result: `dart/utils/sdf/` has no TinyXML/raw XML parser path
for SDF semantics. SDF content loading, ambiguous `.xml` SDF classification,
standard model/link/joint/visual/collision/material reads, and writer semantic
checks stay on libsdformat `sdf::Root` and typed DOM APIs. Remaining
`sdf::Element` access is the documented bridge for authored/default checks,
DART extension fields, sdformat fields not exposed by high-level DOM, and
post-serialization patching where sdformat's DOM omits or legacy-names a value.
TinyXML remains only in `dart::io` as the non-SDF URDF/MJCF root classifier and
in the URDF writer, where URDF XML is the target format.
This boundary now has a local lint/check-lint guard:
`pixi run check-sdf-sdformat-boundary` rejects TinyXML/raw XML parser APIs,
generic SDF element text parsing, and SDF helper-surface expansion beyond
authored sdformat element lookup in `dart/utils/sdf`.

The supported SDF geometry reader paths now load through libsdformat
`sdf::Geometry` DOM objects before mapping sphere, box, cylinder, capsule,
cone, ellipsoid, plane, and mesh shapes into DART shapes; this preserves the
existing plane-as-thin-box and resource-retrieved mesh behavior without adding
new raw XML-level parsing.
The model reader now selects the top-level model through libsdformat
`sdf::Root::Model()` or the first model in the first `sdf::World`, and carries
the resulting `sdf::Model` DOM through standard model, link, joint, visual,
collision, and material traversal instead of rediscovering those standard
children through raw XML-level enumeration.
`dart::io::readSkeleton()` now also asks sdformat to recognize ambiguous `.xml`
SDF content, including root-model and world-contained model documents, by
loading through `sdf::Root::LoadSdfString()` and classifying through
`sdf::Root` model/world APIs before falling back to TinyXML root sniffing for
non-SDF URDF/MJCF dispatch.
Model/link body-node reads use `sdf::Model` and `sdf::Link` DOM values for
model name/static/pose and link name/gravity/pose plus inertial mass, center of
mass, and inertia tensor values. Visual and collision shape nodes traverse
`sdf::Model`, `sdf::Link`, `sdf::Visual`, `sdf::Collision`, `sdf::Geometry`, and
`sdf::Material` DOM values for shape names, local poses, geometry, cast-shadow
state, zero visibility flags, transparency, and diffuse material colors plus
`sdf::Pbr` metal workflow factors. Diffuse material values now come from
`sdf::Material::Diffuse()`; authored/default checks use sdformat's
`Element::GetExplicitlySetInFile()` signal so absent diffuse colors do not
reset DART's default visual color. Collision-surface contact bitmasks now read
through `sdf::Collision`, `sdf::Surface`, and `sdf::Contact` DOM values into
DART `CollisionAspect` collidable state for the lossless zero-bitmask subset,
while ODE friction values read through `sdf::Friction` and `sdf::ODE` DOM
values into DART `DynamicsAspect` friction, slip, and first friction-direction
fields while preserving defaults for unauthored fields.
SDF bounce restitution reads through sdformat Element authored flags and direct
typed `Element::Get<double>` access for the
`<surface><bounce><restitution_coefficient>` schema field because sdformat 16
does not expose a high-level `Bounce` DOM class.
DART `BodyNode::setCollidable(false)` now writes per-collision
`sdf::Contact::SetCollideBitmask(0)` values and re-parses as disabled collision
aspects. This preserves the effective disabled-collision state through
sdformat's contact bitmask DOM because SDF has no link-level collidable flag.
Standard SDF joint reads traverse `sdf::Model`,
`sdf::Joint`, and `sdf::JointAxis` DOM values for joint enumeration, joint
name/type,
parent/child links, local pose, axis vectors including
`axis/xyz@expressed_in` frame resolution, axis dynamics, finite position limits,
mimic metadata, sdformat-normalized screw pitch values, and authored
axis-limit effort/velocity values mapped into symmetric DART force/velocity
limits. sdformat Element presence checks remain only where DART preserves
authored/default distinctions for existing diagnostics, reads DART-specific
soft-body extension fields, or supports the legacy `use_parent_model_frame`
presence check. Those standard SDF
authored/default checks now use sdformat explicit-authored flags and
sdformat `Element::FindElement()` lookup instead of raw child-existence
probing. The compatibility boolean value now uses sdformat typed element access
rather than DART's SDF helper layer.
The SDF detail helper API has been narrowed to that remaining bridge: generic
XML attribute reads, string reads, vector2/vectorX parsing, child enumeration,
boolean value parsing, and direct helper tests for those deleted APIs are gone.
Retained helpers now cover only authored element presence/lookup for fields the
high-level DOM does not expose; standard presence checks use
`Element::FindElement()` plus `GetExplicitlySetInFile()`, and DART-specific
soft-body extension values are converted locally through sdformat typed
`Element::Get<T>` calls instead of shared DART-side scalar, vector, or pose
parsers.
The SDF writer's post-serialization preservation path now uses typed
`sdf::Model`, `sdf::Link`, and `sdf::Joint` DOM values for link/joint names,
disabled link gravity modes, and screw pitch values. It still patches
sdformat's element tree for fields that `Root::ToElement()` omits or emits
under the deprecated tag, but it no longer reads serialized XML attributes back
from that tree, and the patch path now finds generated sdformat children by
`Element::FindElement()` child lookup plus sdformat sibling iteration for
repeated generated children.
Model static/mobile state now reads through `sdf::Model::Static()` and writes
through `sdf::Model::SetStatic()`, with writer coverage that preserves static
state through an implicit FreeJoint root. Model-level self-collision now reads
through `sdf::Model::SelfCollide()` and writes through
`sdf::Model::SetSelfCollide()`, with parser and writer coverage for SDF
`<self_collide>` round-trip.
Non-default skeleton gravity now reads through `sdf::World::Gravity()` and
writes through `sdf::World::SetGravity()` by wrapping the model in a root SDF
`<world>` only when the skeleton gravity differs from DART's default gravity.
DART `HeightmapShape` writer attempts now fail with a targeted diagnostic
instead of falling through to the generic unsupported-shape path. This is
intentional: sdformat exposes `sdf::Heightmap`, but DART's in-memory heightmap
shape does not carry the source heightmap URI that SDF requires, and the current
targetless string writer has no destination URI or generated-resource policy.
DART visual reflectance also fails with a targeted diagnostic because the
current sdformat-backed material mapping preserves diffuse color plus SDF PBR
metal workflow factors, but has no equivalent DART reflectance field to write
losslessly.
Non-default DART `MeshShape` color and alpha modes now fail with targeted
diagnostics on SDF visual export. sdformat's typed `sdf::Mesh` DOM preserves
URI, scale, and submesh selection, but not DART's runtime mesh render-policy
modes, so accepting those meshes would silently drop visual behavior.
DART `SoftBodyNode` export now fails with a targeted diagnostic before the
writer builds an `sdf::Link`. This avoids serializing a soft body as an
ordinary link and dropping DART point-mass, spring, damping, or soft mesh
topology semantics.
DART-only/generated geometry families such as `PyramidShape`,
`ArrowShape`, `MultiSphereConvexHullShape`, `PointCloudShape`,
`LineSegmentShape`, and `VoxelGridShape` now fail with targeted diagnostics
instead of the generic unsupported-shape or mesh-URI fallback. SDF has no direct
pyramid, arrow, multi-sphere convex-hull, point-cloud, line-segment, or
occupancy-grid geometry primitives in the
writer's current contract, and the targetless string writer has no destination
URI or generated-resource policy for converting those DART-side data
structures into SDF-owned mesh or resource artifacts.
URI-backed mesh material variants now have focused read/write/read coverage:
the writer preserves the `sdf::Mesh` URI, and the reparse uses DART's normal
mesh resource loader to recover submesh material assignments, PBR factors,
diffuse colors, and texture paths from the original multi-material glTF asset.
This does not add SDF XML material synthesis for URI-less or generated
in-memory mesh materials; those still need a destination-aware file/project
writer policy.

The SDF writer integration test now uses
`tests/helpers/io_round_trip_helpers.hpp` for reusable body, joint, DoF,
inertia, and shape comparisons, and `tests/CMakeLists.txt` includes the helper
in the formatted test-helper list. This closes the Phase 5 comparison-helper
infrastructure item, but it does not broaden the public writer contract by
itself.
Its SDF semantic assertions now parse the emitted string back through
`sdf::Root::LoadSdfString()` and inspect libsdformat DOM objects for standard
model, link, joint, visual, collision, material, mesh, and geometry values.
Literal XML substring checks remain only for schema element-name contracts that
sdformat normalizes semantically, such as legacy `<thread_pitch>` versus modern
`<screw_thread_pitch>`.

The writer API home decision is now recorded in
`docs/onboarding/io-parsing.md` and
[`05-export-writers-plan.md`](05-export-writers-plan.md): keep export APIs
format-owned until more than one accepted writer contract exists. The SDF
writer stays on `dart::utils::SdfParser`, `dart::io` remains read-side, and
project/editor save-load belongs to the scene/project layer.

PLAN-101 project save/load is already delivered by the dartsim engine, not by
the SKEL replacement path. The durable project format is the versioned
`dartsim-scene` text contract in `dartsim::engine::scene_io`, with
`SimEngine::saveProject()` / `SimEngine::loadProject()` owning file lifecycle
state. Existing headless tests cover stable scene serialization, malformed
input rejection, dirty-state/project-path behavior, failed load/save
preservation, and UI project actions such as save/open/new/recent-project and
dirty-replacement guards. This keeps editor metadata out of SDF/URDF
interchange export and does not introduce YAML or SKEL syntax.

Validation for this slice:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run run-cpp-target test_io_readNone`
- `pixi run lint`
- `pixi run build`

Additional validation for the comparison-helper refactor:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for the writer-API-home docs:

- `git diff --check`
- `pixi run lint`

Additional validation for PLAN-101 project save/load evidence:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target UNIT_dartsim_engine && CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target UNIT_dartsim_ui_ProjectActions && ./build/default/cpp/Release/bin/UNIT_dartsim_engine --gtest_filter="SceneIO.*:SimEngine.ProjectFileRoundTrip:SimEngine.ProjectDirtyStateTracksSavedSceneSnapshot:SimEngine.LoadAndNewProjectResetProjectState:SimEngine.FailedProjectSaveAndLoadPreserveCurrentState:RecorderPlayer.RecordingRoundTripsThroughStream" && ./build/default/cpp/Release/bin/UNIT_dartsim_ui_ProjectActions'`

Additional validation for included SDF model relative mesh resources:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfParser && CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfParser --gtest_filter=SdfParser.IncludedModelRelativeMeshUsesIncludedModelUri && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingRelativeMeshIncludeWorldFixture'`

Changelog decision:

- Mode: draft
- Base evidence: local diff on `work/skel-format-yaml-decision`, compared
  against the DART 7 `CHANGELOG.md` IO and Parsing section.
- Scope evidence: `dart/utils/sdf/sdf_parser.cpp`,
  `tests/integration/io/test_sdf_parser.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder.
- Decision: entry required.
- Target section: `CHANGELOG.md` -> `DART 7.0.0 (TBD)` -> `IO and Parsing`.
- Entry text: Fixed SDF world includes so relative meshes inside included
  models resolve against the included model URI and survive SDF writer
  read/write/read round-trips.
- PR-body note: Record this as an SDF parser resource-base fix plus
  include-driver world writer round-trip coverage if a PR is opened.
- Follow-up: add PR link after PR creation, pending explicit approval to push
  or update the PR.

Additional validation for absolute non-file mesh URI writer coverage:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for URI-backed mesh material variant writer coverage:

- Focused coverage passed without writer implementation changes, proving the
  current sdformat `sdf::Mesh` URI path preserves the source mesh resource and
  lets DART recover submesh material assignments, PBR factors, diffuse colors,
  and texture paths on reparse.

Additional validation for sdformat-backed writer assertions:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for collision-surface friction-direction and slip
diagnostics:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for non-finite collision-surface friction-direction
diagnostics:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.NonFiniteCollisionSurfaceFrictionDirectionReturnsError'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `git diff --check`
- `pixi run lint`
- `pixi run build`

Additional validation for SDF writer version diagnostics:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.EmptySdfVersionReturnsError'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `git diff --check`
- `pixi run lint`
- `pixi run build`

Additional validation for sdformat-backed malformed SDF writer version
diagnostics:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.MalformedSdfVersionReturnsSdformatError:SdfWriter.EmptySdfVersionReturnsError'`

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice
  hardens SDF writer version validation inside the existing unreleased
  conservative SDF writer contract; the DART 7 IO and Parsing entry already
  covers the writer and explicit unsupported diagnostics.
- Follow-up: none

Additional validation for multiple root FreeJoint tree SDF writer coverage:

- Focused candidate passed without production writer changes, proving the
  current sdformat `sdf::Model` link representation preserves multiple
  unjointed root links and re-parses them as DART root FreeJoint trees.

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsMultipleRootFreeJointTrees'
  ```

- Full writer target and local gates passed.

  ```bash
  git diff --check
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  pixi run lint
  pixi run build
  ```

Additional validation for mixed implicit/explicit root SDF writer coverage:

- Focused candidate passed without production writer changes, proving the
  current sdformat `sdf::Model` representation preserves a model that mixes an
  unjointed implicit FreeJoint root link with an explicit parent-world revolute
  root joint and re-parses both root trees with topology, transforms, dynamics,
  and shapes intact.

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsMixedImplicitAndParentWorldRoots'
  ```

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder
- Decision: no entry required; this is additional evidence for the existing
  conservative SDF writer contract before the implementation PR exists, not a
  separate user-visible API or format capability.
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice adds
  focused read/write/read coverage and documentation for the existing DART 7 IO
  and Parsing SDF writer contract; it does not add a new public writer API or
  new serialization behavior.
- Follow-up: none

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `tests/integration/io/test_sdf_writer.cpp`
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice adds
  focused test coverage and task evidence for existing SDF writer surface
  metadata diagnostics; it does not add a new public writer API or
  serialization behavior.
- Follow-up: none

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice adds
  coverage and documentation for the existing DART 7 IO and Parsing SDF writer
  and mesh/GLTF material import entries; it does not add a new public writer
  API or serialization behavior.
- Follow-up: none

Additional validation for writer options and missing mesh URI diagnostics:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for SDF 1.11 mimic metadata writer coverage:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for targetless relative/generated mesh URI diagnostics:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for non-finite mesh scale diagnostics:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for targeted PlaneShape and ConvexMeshShape writer
diagnostics:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`

Additional validation for targeted HeightmapShape writer diagnostics:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`

Additional validation for DART mesh render-policy diagnostics:

- Focused control failed before the writer change:

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter="SdfWriter.MeshColorModeReturnsError:SdfWriter.MeshAlphaModeReturnsError"'
  ```

- Focused candidate passed with the same command.

- Full writer target: passed.

  ```bash
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  ```

- Local gates: passed.

  ```bash
  pixi run lint
  pixi run build
  pixi run test-unit
  ```

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice
  tightens explicit unsupported diagnostics under the existing DART 7 IO and
  Parsing SDF writer entry, which already describes unsupported constructs being
  reported explicitly.
- Follow-up: none

Additional validation for DART `SoftBodyNode` writer diagnostics:

- Focused control failed before the writer change because the writer reported a
  lower-level unsupported shape instead of naming the lost soft-body contract.

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.SoftBodyNodeReturnsExplicitUnsupportedError'
  ```

- Focused candidate passed with the same command.

- Full writer target: passed.

  ```bash
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  ```

- Local gates: passed.

  ```bash
  git diff --check
  pixi run lint
  pixi run build
  pixi run test-unit
  ```

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice
  tightens explicit unsupported diagnostics under the existing DART 7 IO and
  Parsing SDF writer entry, which already describes unsupported constructs being
  reported explicitly.
- Follow-up: none

Additional validation for DART-only/generated geometry diagnostics:

- Focused control failed before the writer change because `PyramidShape`,
  `PointCloudShape`, `LineSegmentShape`, `VoxelGridShape`, and
  `MultiSphereConvexHullShape` all reached the generic unsupported-shape
  fallback instead of naming the missing SDF primitive or generated-resource
  policy.
- Follow-up coverage added the `ArrowShape` diagnostic because it inherits from
  `MeshShape` and previously reached the generic missing-URI mesh error instead
  of naming the missing SDF arrow primitive and generated-resource policy.

  ```bash
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  ```

- Focused candidates passed with the same command, including the ArrowShape
  follow-up.

- Full writer target and local gates passed.

  ```bash
  git diff --check
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  pixi run lint
  pixi run build
  pixi run test-unit
  ```

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice
  tightens explicit unsupported diagnostics under the existing DART 7 IO and
  Parsing SDF writer entry, which already describes unsupported constructs being
  reported explicitly.
- Follow-up: none

Additional validation for body-level collision disable SDF writing:

- Control: direct filtered `INTEGRATION_io_SdfWriter` run failed before the
  writer change because the emitted collision kept sdformat's default bitmask
  `255` and reparsed as collidable.
- Focused candidate: passed.

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsBodyLevelCollidableAsCollisionBitmask'
  ```

- Full writer target: passed.

  ```bash
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  ```

- Local gates: passed.

  ```bash
  pixi run lint
  pixi run build
  pixi run test-unit
  ```

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry; this broadens an unreleased SDF
  writer contact-bitmask subset and is covered by the broader DART 7 SDF
  writer/export work.
- Follow-up: none

Additional validation for unsupported visual reflectance SDF writing:

- Control: direct filtered `INTEGRATION_io_SdfWriter` run failed before the
  writer change because `tryWriteSkeletonToString()` succeeded after dropping
  the non-default DART visual reflectance value.
- Focused candidate: passed.

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.UnsupportedVisualReflectanceReturnsError'
  ```

- Full writer target: passed.

  ```bash
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  ```

- Local gates: passed.

  ```bash
  pixi run lint
  pixi run build
  pixi run test-unit
  ```

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry; this hardens an unreleased SDF
  writer unsupported-material diagnostic and is covered by the broader DART 7
  SDF writer/export work.
- Follow-up: none

Additional validation for continuous SDF joint read/write/read coverage:

- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for model/link/inertial SDF DOM reads:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for visual/collision/material SDF DOM reads:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for joint SDF DOM reads:

- `pixi run run-cpp-target INTEGRATION_io_SdfParser`

Additional validation for ellipsoid SDF DOM geometry reads and writes:

- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for SDF root/model/link/joint/aspect DOM traversal:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run lint`
- `pixi run build`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_Read`

Additional validation for pruning the unused SDF parser helper API:

- `git diff --check`
- `pixi run run-cpp-target test_sdf_helpersNone`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run lint`
- `pixi run build`

Additional validation for capsule/cone SDF DOM geometry reads and writes:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run lint`
- `pixi run build`

Additional validation for SDF 1.10+ screw pitch writer output:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`

Additional validation for sdformat-normalized screw pitch IO:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run lint`
- `pixi run build`

Additional validation for SDF PBR material factor IO:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`

Additional validation for SDF joint-axis expressed-in frame resolution:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`

Additional validation for SDF model self-collision IO:

- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `git diff --check`
- `pixi run lint`
- `pixi run build`

Additional validation for SDF model static/mobile writer coverage:

- Focused implicit-root candidate passed, proving the current sdformat
  `sdf::Model::Static()` path preserves `Skeleton::isMobile() == false`
  independently from explicit fixed-root topology. The emitted SDF model has no
  explicit root joint, and reparse keeps the root as a DART FreeJoint while
  restoring the non-mobile skeleton state.

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsModelStaticStateWithImplicitRoot'
  ```

- Full writer target and local gates passed.

  ```bash
  git diff --check
  pixi run run-cpp-target INTEGRATION_io_SdfWriter
  pixi run lint
  pixi run build
  ```

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder
- Decision: no entry required; this adds focused coverage and documentation for
  existing unreleased SDF writer model-state behavior under the broader DART 7
  IO and Parsing writer entry.
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice proves
  existing model static/mobile state preservation and does not add a new public
  API or serialization surface.
- Follow-up: none

Additional validation for SDF world gravity IO:

- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `git diff --check`
- `pixi run lint`
- `pixi run build`

Additional validation for SDF material diffuse DOM reads:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`

Additional validation for removing the SDF helper boolean parser:

- `git diff --check`
- `pixi run run-cpp-target test_sdf_helpersNone`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`

Additional validation for removing SDF helper XML text fallback parsing:

- `git diff --check`
- `pixi run run-cpp-target test_sdf_helpersNone`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`

Additional validation for removing SDF writer serialized-attribute reads:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for sdformat-authored SDF presence checks:

- `pixi run run-cpp-target test_sdf_helpersNone`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`

Additional validation for sdformat `Element::FindElement()` lookup bridge:

- `git diff --check`
- `pixi run run-cpp-target test_sdf_helpersNone`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target test_io_readNone`

Additional validation for collision-surface ODE friction IO:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for collision-surface contact bitmask IO:

- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for collision-surface bounce restitution IO:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for visual shadow/hidden metadata IO:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for visual transparency IO:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`

Additional validation for direct sdformat bounce typed access:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`

Additional validation for removing SDF helper value parsers:

- `pixi run run-cpp-target test_sdf_helpersNone`
- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`

Additional validation for direct SDF writer patch traversal:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`

Additional validation for authored-only sdformat Element bridge narrowing:

- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `SCCACHE_GHA_ENABLED=false pixi run run-cpp-target test_sdf_helpersNone`
- `SCCACHE_GHA_ENABLED=false pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`
- `git diff --check`

Additional validation for explicit parent-world root joint writing, including
continuous revolute roots:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run lint`
- `pixi run build`

Changelog decision:

- Mode: draft
- Base evidence: `origin/main`
- Scope evidence: local diff adds the parser-specific SDF writer API,
  integration round-trip coverage, IO docs, and SKEL-format task handoff.
- Decision: entry required
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing
- Entry text: present as the conservative SDF writer bullet, without a PR link
  until the implementation PR exists.
- PR-body note: N/A
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push. No additional changelog
  entry is needed for the comparison-helper refactor because it only changes
  test infrastructure and task handoff docs. No additional changelog entry is
  needed for the absolute non-file mesh URI coverage because it only strengthens
  test evidence for the existing SDF writer entry. No additional entry is needed
  for writer-option and missing-mesh-URI coverage because it hardens the same
  existing writer surface before the implementation PR exists. No additional
  entry is needed for non-finite mesh scale coverage because it hardens the same
  typed `sdf::Mesh` writer surface before the implementation PR exists. No
  additional entry is needed for SDF 1.11 mimic metadata coverage because it
  broadens the same conservative SDF writer capability before the implementation
  PR exists.
  No additional entry is needed for targetless relative/generated mesh URI
  diagnostics because it hardens the same SDF writer resource contract before
  the implementation PR exists. No additional entry is needed for continuous SDF
  joint coverage because it broadens the same conservative SDF writer capability
  before the implementation PR exists. No additional entry is needed for
  visual/collision/material SDF DOM reads because they harden the same
  conservative SDF writer/parser round-trip surface before the implementation PR
  exists. No additional entry is needed for joint SDF DOM reads because they
  harden the same conservative SDF writer/parser round-trip surface before the
  implementation PR exists. No additional separate entry is needed for
  ellipsoid SDF geometry reads and writes because the existing conservative SDF
  writer changelog bullet now includes the broadened geometry subset before the
  implementation PR exists. No additional separate entry is needed for
  capsule/cone SDF geometry reads and writes because they broaden the same
  sdformat-DOM-backed parser/writer geometry subset before the implementation
  PR exists. No additional entry is needed for SDF
  root/model/link/joint/aspect DOM traversal because it hardens the same
  conservative SDF parser/writer round-trip surface before the implementation
  PR exists. No additional entry is needed for pruning unused SDF parser helper
  APIs because this is an internal parser cleanup that narrows the detail helper
  surface while preserving the existing parser/writer changelog scope. No
  additional entry is needed for SDF 1.10+ screw pitch writer output because it
  hardens the same conservative SDF writer capability before the implementation
  PR exists. No additional separate entry is needed for sdformat-normalized
  screw pitch IO because it keeps the same parser/writer capability on
  libsdformat semantics before the implementation PR exists. No additional
  separate entry is needed for SDF PBR material factors because the existing
  conservative SDF writer changelog bullet now includes broadened visual
  material round-trip coverage before the implementation PR exists. No
  additional separate entry is needed for SDF joint-axis expressed-in frame
  resolution because the existing DART 7 libsdformat dependency entry now covers
  this parser-side normalization. No additional separate entry is needed for
  SDF material diffuse DOM reads because they keep the same parser-side material
  normalization under the existing DART 7 libsdformat dependency entry while
  preserving DART's authored/default behavior. No additional separate entry is
  needed for removing the SDF helper XML text fallback parser because it is an
  internal detail-helper cleanup that moves the existing DART extension bridge
  onto sdformat typed `Element::Get<T>` access under the same libsdformat
  normalization entry. No additional separate entry is needed for removing SDF
  writer serialized-attribute reads because it is an internal implementation
  cleanup that keeps the existing conservative SDF writer behavior while using
  typed sdformat DOM values to drive the same serialization patch. No
  additional separate entry is needed for targeted PlaneShape and
  ConvexMeshShape writer diagnostics because they harden the same conservative
  SDF writer unsupported-resource contract before the implementation PR exists.
  No additional separate entry is needed for sdformat-authored SDF presence
  checks because this is an internal parser hardening that preserves existing
  authored/default behavior while replacing child-existence probing with
  sdformat `Element::FindElement()` lookup and explicit-authored metadata.
  No additional separate entry is needed for replacing DART-side SDF child
  lookup loops with sdformat `Element::FindElement()` because it is an internal
  implementation cleanup that preserves the same SDF parser/writer behavior
  while keeping the remaining bridge on libsdformat APIs.
  No additional separate entry is needed for collision-surface ODE friction IO
  because it broadens the same conservative SDF writer/parser round-trip
  capability before the implementation PR exists; the existing DART 7 IO and
  Parsing bullet now includes collision surface ODE friction/slip metadata.
  No additional separate entry is needed for collision-surface contact bitmask
  IO because it broadens the same conservative SDF writer/parser round-trip
  capability before the implementation PR exists; the existing DART 7 IO and
  Parsing bullet now includes collision surface contact disable bitmasks.
  No additional separate entry is needed for collision-surface bounce
  restitution IO because it broadens the same conservative SDF writer/parser
  round-trip capability before the implementation PR exists; the existing DART
  7 IO and Parsing bullet now includes zero-threshold bounce restitution.
  No additional separate entry is needed for visual shadow/hidden metadata IO
  because it broadens the same conservative SDF writer/parser round-trip
  capability before the implementation PR exists; the existing DART 7 IO and
  Parsing bullet now includes visual shadow/hidden state. No additional
  separate entry is needed for visual transparency IO because it broadens the
  same conservative SDF writer/parser round-trip capability before the
  implementation PR exists; the existing DART 7 IO and Parsing bullet now
  includes visual shadow/hidden/transparency state. No additional separate
  entry is needed for direct sdformat bounce typed access because it is an
  internal parser hardening for the already-covered collision-surface bounce
  restitution field. No additional separate entry is needed for removing SDF
  helper value parsers because it is an internal helper-surface narrowing that
  keeps remaining DART extension parsing on sdformat typed element access.
  No additional separate entry is needed for direct SDF writer patch traversal
  because it is an internal implementation cleanup that keeps the existing
  conservative SDF writer behavior while avoiding schema lookup in the
  sdformat element patch path.
  No additional separate entry is needed for authored-only sdformat Element
  bridge narrowing because it is an internal helper-surface cleanup that
  preserves existing SDF parser/writer behavior while keeping SDF semantics on
  libsdformat DOM and typed access.
  No additional separate entry is needed for explicit parent-world root joint
  writing because it broadens the same conservative SDF writer capability before
  the implementation PR exists; the existing DART 7 IO and Parsing bullet now
  includes explicit parent-world root joints for supported SDF joint types,
  including continuous revolute roots.
  No additional separate entry is needed for SDF model self-collision IO because
  it broadens the same conservative SDF writer/parser round-trip capability
  before the implementation PR exists; the existing DART 7 IO and Parsing
  bullet now includes model self-collision state.
  No additional separate entry is needed for SDF world gravity IO because it
  broadens the same conservative SDF writer/parser round-trip capability before
  the implementation PR exists; the existing DART 7 IO and Parsing bullet now
  includes non-default skeleton gravity through SDF world gravity.
  No additional separate entry is needed for the targeted ArrowShape writer
  diagnostic because it hardens the same conservative SDF writer
  unsupported-resource contract before the implementation PR exists.
  No additional separate entry is needed for NaN SDF joint position-limit
  diagnostics because it hardens the same conservative SDF writer contract
  before the implementation PR exists.
  No additional separate entry is needed for non-finite SDF structural
  diagnostics because they harden the same conservative SDF writer contract
  before the implementation PR exists.

Additional validation for NaN SDF joint position-limit diagnostics:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (55 tests)
- `pixi run lint`
- `pixi run build`

Additional validation for non-finite SDF structural diagnostics:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (59 tests)
- `pixi run lint`
- `pixi run build`

Additional validation for sdformat-based ambiguous `.xml` SDF dispatch:

- Focused candidate for the tightened SDF/non-SDF dispatch split: passed.

  ```bash
  pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target test_io_readNone && ./build/default/cpp/Release/bin/test_io_readNone --gtest_filter=ReadUnit.InfersSdfWorldAmbiguousXmlUriWithSdformat'
  ```

- `cmake --build build/default/cpp/Release --target test_io_readNone -j2`
- `build/default/cpp/Release/bin/test_io_readNone --gtest_filter='ReadUnit.InfersSdfAmbiguousXmlUriWithSdformat:ReadUnit.ReturnsNullForInvalidAmbiguousXmlContent:ReadUnit.InfersUrdfFromAmbiguousXmlRootBeforeParseFailure'`
- `pixi run run-cpp-target test_io_readNone`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- `build/default/cpp/Release/bin/INTEGRATION_io_Read`
- `git diff --check`

Changelog decision:

- Mode: decide
- Base evidence: `origin/main` inferred for
  `work/skel-format-yaml-decision`; no open PR exists for the branch.
- Scope evidence: local diff keeps ambiguous `.xml` SDF detection in
  `dart::io::readSkeleton()` on `sdf::Root::LoadSdfString()`, separates the
  TinyXML fallback as URDF/MJCF-only root classification, and adds focused
  read-unit coverage for world-contained SDF models behind `.xml` URIs.
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No additional changelog entry is needed because this is an
  internal dispatch hardening under the existing DART 7 IO and Parsing
  libsdformat-normalization entry; valid SDF imports remain on the same public
  `readSkeleton()` path.
- Follow-up: none

Additional validation for unsupported DART joint-family SDF writer diagnostics:

- `git diff --check`
- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.ChildFreeJointReturnsExplicitUnsupportedError:SdfWriter.RootEulerJointReturnsExplicitUnsupportedError:SdfWriter.EulerJointReturnsExplicitUnsupportedError:SdfWriter.PlanarJointReturnsExplicitUnsupportedError:SdfWriter.TranslationalJoint2DReturnsExplicitUnsupportedError:SdfWriter.TranslationalJointReturnsExplicitUnsupportedError'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`

Changelog decision:

- Mode: decide
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder
- Decision: no entry required
- Target section: N/A
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice
  tightens explicit unsupported diagnostics under the existing unreleased SDF
  writer contract; the DART 7 IO and Parsing entry already says unsupported
  constructs are reported explicitly.
- Follow-up: none

Additional validation for SDF joint-axis effort/velocity limit IO:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfParser && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfParser --gtest_filter=SdfParser.JointAxisLimitsEffortAndDamping'`
- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsJointAxisVelocityAndEffortLimits:SdfWriter.AsymmetricJointVelocityLimitReturnsError:SdfWriter.AsymmetricJointForceLimitReturnsError:SdfWriter.NaNJointVelocityLimitReturnsError:SdfWriter.NaNJointForceLimitReturnsError'`

Additional validation for first URDF writer slice:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter'`

Additional validation for URDF mimic metadata writer coverage:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.RoundTripsMimicMetadata:UrdfWriter.MimicWithoutReferenceReturnsError:UrdfWriter.CouplerMimicReturnsError'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (15 tests)

Additional validation for URDF continuous-joint dynamics writer coverage:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.RoundTripsContinuousJointDynamics'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (15 tests)

Additional validation for URDF package mesh URI writer coverage:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.PreservesPackageMeshUri'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (15 tests)

Additional validation for URDF mesh resource diagnostics:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.MeshWithoutUriReturnsError:UrdfWriter.RelativeMeshUriReturnsError:UrdfWriter.NonFiniteMeshScaleReturnsError'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (15 tests)

Additional validation for URDF soft-body writer diagnostics:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.SoftBodyNodeReturnsExplicitUnsupportedError'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (15 tests)

Additional validation for URDF collision package meshes and structural
diagnostics:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter="UrdfWriter.PreservesCollisionPackageMeshUri:UrdfWriter.NonPositiveMassReturnsError:UrdfWriter.NonFiniteLocalComReturnsError:UrdfWriter.NonFiniteVisualMaterialColorReturnsError:UrdfWriter.NonFiniteShapePoseReturnsError:UrdfWriter.AsymmetricVelocityLimitReturnsError:UrdfWriter.AsymmetricEffortLimitReturnsError:UrdfWriter.NonFiniteJointAxisReturnsError"'`

Additional validation for URDF default visual material preservation:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter="UrdfWriter.OmitsImplicitDefaultVisualMaterial:UrdfWriter.PreservesExplicitDefaultVisualMaterial:UrdfWriter.PreservesDefaultVisualAlphaOverride"'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (26 tests)

Additional validation for URDF coupler mimic transmission writing:

- Control before the writer change:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.CouplerMimicReturnsError'`
- Focused candidate:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter="UrdfWriter.RoundTripsCouplerMimicThroughTransmissions:UrdfWriter.CouplerMimicWithOffsetReturnsError"'`
- Full writer target: `pixi run run-cpp-target INTEGRATION_io_UrdfWriter`
- Local gates: `git diff --check`, `pixi run lint`, `pixi run build`

Changelog decision:

- Mode: draft
- Base evidence: `origin/main` inferred for
  `work/skel-format-yaml-decision`; no open PR exists for the branch.
- Scope evidence: focused diff in `dart/utils/urdf/urdf_writer.cpp`,
  `dart/utils/urdf/urdf_parser.hpp`,
  `tests/integration/io/test_urdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, `CHANGELOG.md`, and this SKEL evolution
  task folder.
- Decision: entry required as a revision to the existing DART 7 URDF writer
  bullet because this broadens the unreleased public parser-owned writer
  contract to include zero-offset coupler mimic transmission output.
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing existing URDF
  writer bullet.
- Entry text: the existing conservative URDF writer bullet now includes
  zero-offset coupler mimic transmissions; no separate bullet is needed.
- PR-body note: N/A
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push.

Additional validation for URDF planar/floating joint writer coverage:

- Control before the writer change:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.UnsupportedPlanarJointReturnsError'`
- Focused candidate:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter="UrdfWriter.RoundTripsPlanarAndFloatingJoints:UrdfWriter.NonUniformPlanarJointLimitsReturnsError:UrdfWriter.NonUniformFloatingJointDynamicsReturnsError:UrdfWriter.ArbitraryPlanarJointReturnsError"'`
- SDF boundary check after maintainer guidance:
  `pixi run check-sdf-sdformat-boundary`
- Full writer target: `pixi run run-cpp-target INTEGRATION_io_UrdfWriter`
- Local gates: `git diff --check`, `pixi run lint`, `pixi run build`,
  `pixi run test-unit`

Changelog decision:

- Mode: draft
- Base evidence: `origin/main` inferred for
  `work/skel-format-yaml-decision`; no open PR exists for the branch.
- Scope evidence: focused diff in `dart/utils/urdf/urdf_writer.cpp`,
  `dart/utils/urdf/urdf_parser.hpp`,
  `tests/integration/io/test_urdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, `CHANGELOG.md`, and this SKEL evolution
  task folder.
- Decision: entry required as a revision to the existing DART 7 URDF writer
  bullet because this broadens the unreleased public parser-owned writer
  contract to include standard-plane planar and floating child joint output
  with uniform scalar limit/dynamics metadata.
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing existing URDF
  writer bullet.
- Entry text: the existing conservative URDF writer bullet now includes
  planar/floating child joints with uniform scalar limits and dynamics; no
  separate bullet is needed.
- PR-body note: N/A
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push.

Changelog decision:

- Mode: decide
- Base evidence: `origin/main` inferred for
  `work/skel-format-yaml-decision`; no open PR exists for the branch.
- Scope evidence: focused diff in `dart/utils/urdf/urdf_writer.cpp`,
  `dart/utils/urdf/urdf_parser.hpp`,
  `tests/integration/io/test_urdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Decision: no additional changelog entry.
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing existing URDF
  writer bullet.
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice
  refines default-material preservation inside the existing unreleased
  conservative URDF writer contract; the DART 7 IO and Parsing entry already
  covers URDF writer visual colors and explicit diagnostics for unsupported or
  lossy constructs.
- Follow-up: none

Additional validation for URDF visual/collision metadata diagnostics:

- Control before the writer change failed because the writer succeeded after
  dropping non-default DART visual reflectance, disabled collision aspect state,
  and collision dynamics metadata:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter="UrdfWriter.UnsupportedVisualReflectanceReturnsError:UrdfWriter.DisabledCollisionAspectReturnsError:UrdfWriter.CollisionDynamicsMetadataReturnsError"'`
- Focused candidate passed with the same command.
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (33 tests)

Changelog decision:

- Mode: decide
- Base evidence: `origin/main` inferred for
  `work/skel-format-yaml-decision`; no open PR exists for the branch.
- Scope evidence: local diff committed as
  `a849cf29b27 Reject lossy URDF writer metadata` in
  `dart/utils/urdf/urdf_writer.cpp`,
  `tests/integration/io/test_urdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder; nearby
  `CHANGELOG.md` DART 7 IO and Parsing URDF writer bullet inspected.
- Decision: no additional changelog entry.
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing existing URDF
  writer bullet.
- Entry text: N/A
- PR-body note: No separate changelog entry is needed because this slice
  tightens explicit unsupported/lossy diagnostics inside the existing
  unreleased conservative URDF writer contract; the DART 7 IO and Parsing entry
  already says the URDF writer reports unsupported or lossy DART constructs
  explicitly.
- Follow-up: none

Additional validation for URDF root-joint policy coverage:

- Focused root-policy candidate passed, proving the writer accepts identity
  root `WeldJoint` input as a bare URDF root link, rejects non-identity root
  placement, and rejects unsupported root joint families instead of silently
  dropping parent-joint metadata:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter="UrdfWriter.IdentityRootWeldWritesRootLinkWithoutParentJointMetadata:UrdfWriter.RootJointPlacementReturnsError:UrdfWriter.UnsupportedRootJointTypeReturnsError"'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (36 tests)
- SDF boundary check passed:
  `pixi run check-sdf-sdformat-boundary`
- Local formatting gate passed:
  `pixi run lint`

Changelog decision:

- Mode: decide
- Base evidence: `origin/main` inferred for
  `work/skel-format-yaml-decision`; no open PR exists for the branch.
- Scope evidence: local diff in `tests/integration/io/test_urdf_writer.cpp`,
  `dart/utils/urdf/urdf_parser.hpp`, `CHANGELOG.md`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Decision: no additional changelog entry; revise the existing unreleased DART
  7 URDF writer bullet to say identity root FreeJoint/WeldJoint validation
  rather than root metadata preservation.
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing existing URDF
  writer bullet.
- Entry text: N/A beyond the wording correction in the existing bullet.
- PR-body note: No separate changelog entry is needed because this slice adds
  focused root-policy coverage and corrects wording inside the existing
  unreleased conservative URDF writer entry; it does not add a new public
  writer API or serialization behavior.
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push.

Additional validation for SDF sdformat-boundary lint coverage:

- `pixi run check-sdf-sdformat-boundary`
- `pixi run run-cpp-target test_sdf_helpersNone` (3 tests)
- `pixi run run-cpp-target INTEGRATION_io_SdfParser` (39 tests)
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (76 tests)
- `pixi run lint`
- `pixi run check-lint`
- `pixi run build`

Changelog decision:

- Mode: draft
- Base evidence: `origin/main` inferred for
  `work/skel-format-yaml-decision`; no open PR exists for the branch.
- Scope evidence: local diff adds `scripts/check_sdf_sdformat_boundary.py`,
  wires it into `pixi run lint` / `pixi run check-lint`, and updates IO/task
  docs to record the sdformat boundary.
- Decision: entry required because this changes the contributor quality-gate
  surface.
- Target section: `CHANGELOG.md` -> DART 7 -> Tests, Benchmarks, and Quality
  Gates.
- Entry text: present as the SDF sdformat-boundary lint/check-lint guard
  bullet, without a PR link until the implementation PR exists.
- PR-body note: N/A
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push.

Changelog decision:

- Mode: decide
- Base evidence: current local diff only adds URDF writer tests and task /
  onboarding documentation for behavior already in the public URDF writer
  contract.
- Decision: no additional changelog entry. The existing DART 7 URDF writer
  bullet already covers package URI meshes, visual colors, unsupported/lossy
  construct diagnostics, and invalid mesh resources; this slice hardens
  coverage without changing the user-visible API or writer behavior.
- PR-body note: record as test coverage / task progress if a PR is opened.

Changelog decision:

- Mode: draft
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/sdf/sdf_parser.cpp`,
  `dart/utils/sdf/sdf_writer.cpp`, `tests/integration/io/test_sdf_parser.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  `CHANGELOG.md`, and this SKEL evolution task folder
- Decision: entry required as a revision to the existing DART 7 SDF writer
  bullet
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing
- Entry text: the existing conservative SDF writer bullet now includes
  symmetric effort/velocity limits; no separate bullet is needed.
- PR-body note: N/A
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push.

Changelog decision:

- Mode: draft
- Base evidence: `origin/main`
- Scope evidence: focused diff in `dart/utils/urdf/urdf_parser.hpp`,
  `dart/utils/urdf/urdf_writer.cpp`,
  `tests/integration/io/test_urdf_writer.cpp`,
  `tests/integration/CMakeLists.txt`, `docs/onboarding/io-parsing.md`,
  `CHANGELOG.md`, and this SKEL evolution task folder
- Decision: entry required as a new DART 7 URDF writer bullet because this adds
  a public parser-owned export API
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing
- Entry text: the new conservative URDF writer bullet describes the supported
  first tree subset, continuous joints, passive dynamics, single-DoF mimic
  metadata, package mesh URI preservation, invalid mesh resource diagnostics,
  soft-body rejection, and explicit diagnostics.
- PR-body note: N/A
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push.

Additional validation for empty URDF writer input diagnostics:

- Control: direct filtered `INTEGRATION_io_UrdfWriter` run with the new
  `UrdfWriter.EmptySkeletonReturnsError` expectation failed before the writer
  fix because an empty skeleton returned the generic multiple-root diagnostic:
  `Cannot write URDF for Skeleton [empty] with multiple root trees.`
- Intermediate guard-order candidate failed because `Skeleton::getRootBodyNode`
  throws on empty skeletons before returning a nullable pointer.
- Focused candidate passed, proving empty skeletons now return the intended
  structured writer error while multiple-root skeletons keep their existing
  diagnostic:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter="UrdfWriter.EmptySkeletonReturnsError:UrdfWriter.MultipleRootTreesReturnError"'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_UrdfWriter` (37 tests)
- SDF boundary check passed:
  `pixi run check-sdf-sdformat-boundary`
- Local formatting gate passed:
  `pixi run lint`
- Build gate passed:
  `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `dart/utils/urdf/urdf_writer.cpp`,
  `tests/integration/io/test_urdf_writer.cpp`, and this SKEL evolution task
  folder.
- Decision: no additional changelog entry. The existing unreleased DART 7 URDF
  writer bullet already covers unsupported/lossy construct diagnostics; this
  slice makes the empty-input structured diagnostic reachable and adds focused
  regression coverage without adding a new public API.
- PR-body note: record as URDF writer diagnostic hardening if a PR is opened.

Additional validation for empty SDF writer input diagnostics:

- Control: direct filtered `INTEGRATION_io_SdfWriter` run with the new
  `SdfWriter.EmptySkeletonReturnsError` expectation failed before the writer
  fix because the SDF writer succeeded on an empty skeleton.
- Focused candidate passed, proving empty skeletons now return a structured
  writer error while empty SDF version validation remains first:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter="SdfWriter.EmptySkeletonReturnsError:SdfWriter.EmptySdfVersionReturnsError"'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (77 tests)
- SDF boundary check passed:
  `pixi run check-sdf-sdformat-boundary`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Decision: no additional changelog entry. The existing unreleased DART 7 SDF
  writer bullet already says unsupported constructs are reported explicitly;
  this slice makes empty-input rejection explicit and covered without adding a
  new public API.
- PR-body note: record as SDF writer diagnostic hardening if a PR is opened.

Additional validation for non-positive SDF inertial mass diagnostics:

- Control: direct filtered `INTEGRATION_io_SdfWriter` run with the new
  `SdfWriter.NonPositiveMassReturnsError` expectation failed before the writer
  fix because the SDF writer succeeded on a zero-mass body.
- Focused candidate passed, proving non-positive body mass now returns a
  structured SDF writer error while the existing non-finite inertial-data
  diagnostic remains distinct:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter="SdfWriter.NonPositiveMassReturnsError:SdfWriter.NonFiniteInertialDataReturnsError"'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (78 tests)
- SDF boundary check passed:
  `pixi run check-sdf-sdformat-boundary`
- Broader local gates passed:
  `pixi run lint`; `pixi run build`; `pixi run test-unit`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in `dart/utils/sdf/sdf_writer.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer and explicit
  unsupported/invalid construct diagnostics.
- Decision: no additional changelog entry. This slice prevents zero-mass DART
  inertial data from serializing through the existing unreleased SDF writer
  contract and adds focused regression coverage without adding a new public API.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as SDF writer diagnostic hardening if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for the SDF sdformat routing boundary:

- `scripts/check_sdf_sdformat_boundary.py` now also guards
  `dart/io/read.cpp` so ambiguous XML SDF content is classified with
  `sdf::Root::LoadSdfString()` before the legacy URDF/MJCF TinyXML root
  fallback can run.
- Boundary check passed:
  `pixi run check-sdf-sdformat-boundary`

Additional validation for existing SDF fixture read/write/read coverage:

- Added `SdfWriter.RoundTripsExistingSinglePendulumFixture`, which loads
  `dart://sample/sdf/test/single_pendulum.sdf`, writes it through
  `SdfParser::tryWriteSkeletonToString()`, reloads the emitted SDF, and
  compares the body, root joint, inertial, damping, visual box, and collision
  box semantics against the original parsed skeleton.
- Initial focused draft caught a bad hand-coded transform-sign expectation for
  the source SDF; the final test compares original parsed DART semantics to the
  re-parsed writer output instead of interpreting XML-level pose text itself.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingSinglePendulumFixture'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (79 tests)
- SDF boundary check passed:
  `pixi run check-sdf-sdformat-boundary`
- Broader local gates passed:
  `pixi run lint`; `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer. The DART 7 Tests,
  Benchmarks, and Quality Gates section also already records the libsdformat
  boundary lint/check-lint guard.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped SDF sample without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as SDF writer fixture round-trip verification if a PR is
  opened.
- Follow-up: none until an implementation PR exists.

Additional validation for the remaining converted SKEL SDF fixtures:

- Added `SdfWriter.RoundTripsConvertedSkelBoxFixtures`, which covers the other
  shipped SDF fixtures converted from legacy SKEL:
  `dart://sample/sdf/test/cube.sdf`, `dart://sample/sdf/test/shapes.sdf`, and
  `dart://sample/sdf/test/test_shapes.sdf`.
- The test writes each parsed skeleton through
  `SdfParser::tryWriteSkeletonToString()`, reloads the emitted SDF, and compares
  skeleton name, mobility, body and joint counts, root-joint type and
  transforms, body inertia, visual box size/pose, and collision box size/pose
  against the original parsed skeleton.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsConvertedSkelBoxFixtures'`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer and the legacy SKEL
  conversion/removal entries already cover the fixture migration direction.
- Decision: no additional changelog entry. This slice extends fixture-level
  read/write/read verification across the already-shipped converted SDF samples
  without adding a public API or broadening the writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as converted SDF fixture round-trip verification if a PR
  is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for native two-link revolute SDF fixture coverage:

- Added `SdfWriter.RoundTripsExistingTwoLinkRevoluteFixture`, which loads
  `dart://sample/sdf/test/two_link_revolute_model.sdf`, writes it through
  `SdfParser::tryWriteSkeletonToString()`, reloads the emitted SDF, and
  compares the re-parsed DART semantics against the original parsed skeleton.
- The test covers a shipped SDF model with a generated root FreeJoint, a child
  revolute joint, finite axis position/velocity/effort limits, body inertias,
  and box/cylinder visual and collision geometry. The assertions stay at the
  DART object and typed shape/joint level, avoiding XML-level SDF parsing.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingTwoLinkRevoluteFixture'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (81 tests)
- Boundary/lint/build gates passed:
  `pixi run check-sdf-sdformat-boundary`; `pixi run lint`; `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer, and this slice
  adds fixture-level read/write/read verification without adding a new public
  API or broadening the documented writer contract.
- Decision: no additional changelog entry.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as native two-link SDF fixture round-trip verification
  if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for world-contained SDF revolute fixture coverage:

- Added `SdfWriter.RoundTripsExistingWorldRevoluteFixtures`, which covers
  `dart://sample/sdf/test/issue1193_revolute_test.sdf` and
  `dart://sample/sdf/test/issue1193_revolute_with_offset_test.sdf`.
- The test reads each world-contained SDF fixture through libsdformat-backed
  SDF parsing, writes the parsed skeleton with
  `SdfParser::tryWriteSkeletonToString()`, reloads the emitted SDF, and
  compares DART semantics instead of XML text.
- The fixtures prove the current writer preserves model-pose-derived root
  transforms, explicit child joint pose offsets, zero world gravity, revolute
  axes, body inertias, visual box/sphere geometry, and visual shape poses.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingWorldRevoluteFixtures'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (82 tests)
- Boundary/lint/build gates passed:
  `pixi run check-sdf-sdformat-boundary`; `pixi run lint`; `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer and the DART 7
  Tests, Benchmarks, and Quality Gates section already records the sdformat
  boundary guard.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for existing shipped SDF world fixtures without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as world-contained SDF fixture round-trip verification
  if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for force-torque SDF world fixture coverage:

- Added `SdfWriter.RoundTripsExistingForceTorqueWorldFixture`, which covers
  `dart://sample/sdf/test/force_torque_test.world`.
- The test reads the shipped world file through libsdformat-backed SDF parsing,
  writes the parsed in-file model skeleton with
  `SdfParser::tryWriteSkeletonToString()`, reloads the emitted SDF, and
  compares DART skeleton semantics instead of XML text.
- The fixture proves the current writer preserves the DART semantics imported
  from a sensor-bearing SDF world file: explicit parent-world revolute root,
  child revolute joint, finite axis limits/dynamics, body inertias, skeleton
  gravity, visual sphere/box geometry, collision sphere/box geometry, and shape
  poses. It does not claim SDF sensor or physics metadata preservation because
  those fields are not part of the imported DART `Skeleton`.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingForceTorqueWorldFixture'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (83 tests)
- Boundary/lint/build gates passed:
  `pixi run check-sdf-sdformat-boundary`; `pixi run lint`; `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer and the DART 7
  Tests, Benchmarks, and Quality Gates section already records the sdformat
  boundary guard.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped SDF world fixture
  without adding a new public API, broadening the documented writer contract, or
  claiming preservation for SDF fields outside DART `Skeleton` semantics.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as force-torque SDF world fixture round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for force-torque SDF chain world fixture coverage:

- Added `SdfWriter.RoundTripsExistingForceTorqueChainWorldFixture`, which
  covers `dart://sample/sdf/test/force_torque_test2.world`.
- The test reads the shipped world file through libsdformat-backed SDF parsing,
  writes the parsed in-file `boxes` model skeleton with
  `SdfParser::tryWriteSkeletonToString()`, reloads the emitted SDF through the
  normal SDF parser, and compares DART skeleton semantics instead of XML text.
- The fixture proves the current writer preserves the DART semantics imported
  from a sensor-bearing three-link/two-joint SDF world file: parent-world root
  semantics, child revolute joint topology, axes, finite dynamics metadata, body
  inertias, skeleton gravity, and box visual/collision geometry plus shape
  poses. It does not claim SDF sensor, include, or physics metadata
  preservation because those fields are not part of the imported DART
  `Skeleton`.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingForceTorqueChainWorldFixture'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (84 tests)
- Boundary/lint/build gates passed:
  `pixi run check-sdf-sdformat-boundary`; `pixi run lint`; `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer and the DART 7
  Tests, Benchmarks, and Quality Gates section already records the sdformat
  boundary guard.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped SDF world fixture
  without adding a new public API, broadening the documented writer contract, or
  claiming preservation for SDF fields outside DART `Skeleton` semantics.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as second force-torque SDF world fixture round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for simple SDF world fixture coverage:

- Added `SdfWriter.RoundTripsExistingSingleBodyWorldFixtures`, which covers
  `dart://sample/sdf/test/high_version.world` and
  `dart://sample/sdf/test/single_bodynode_skeleton.world`.
- The test reads both shipped world files through libsdformat-backed SDF
  parsing, writes the parsed single-body skeletons with
  `SdfParser::tryWriteSkeletonToString()`, reloads the emitted SDF through the
  normal SDF parser, and compares DART skeleton semantics instead of XML text.
- The fixtures prove the current writer preserves high-version SDF world input,
  default-inertial fallback, imported root-joint semantics, skeleton gravity,
  body inertia, and box/cylinder visual and collision geometry plus shape poses.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingSingleBodyWorldFixtures'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (85 tests)
- Boundary/lint/build gates passed:
  `pixi run check-sdf-sdformat-boundary`; `pixi run lint`; `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  inspected; it already covers the conservative SDF writer and the DART 7
  Tests, Benchmarks, and Quality Gates section already records the sdformat
  boundary guard.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for existing shipped SDF world fixtures without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as simple SDF world fixture round-trip verification if a
  PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for mixed-joint SDF world fixture coverage:

- Added `SdfWriter.RoundTripsExistingMixedJointWorldFixture`, which covers
  `dart://sample/sdf/test/test_skeleton_joint.world`.
- The test reads the shipped world file through libsdformat-backed SDF parsing,
  writes the parsed skeleton with `SdfParser::tryWriteSkeletonToString()`,
  reloads the emitted SDF through the normal SDF parser, and compares DART
  skeleton semantics instead of XML text.
- The fixture proves the current writer preserves imported root-joint semantics,
  skeleton gravity, body inertia, prismatic/revolute/screw/universal joint
  types, axes, limits, passive dynamics, screw pitch, and cylinder visual and
  collision geometry plus shape poses for a mixed-joint world file.
- Focused fixture test passed:
  `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingMixedJointWorldFixture'`
- Full writer target passed:
  `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (86 tests)
- Boundary/lint/build gates passed:
  `pixi run check-sdf-sdformat-boundary`; `pixi run lint`; `pixi run build`

Changelog decision:

- Mode: decide
- Base evidence: `origin/main` as fetched locally for this branch.
- Scope evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped SDF world fixture without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as mixed-joint SDF world fixture round-trip verification
  if a PR is opened.
- Follow-up: none until an implementation PR exists.

## Previous Resume Checkpoint (2026-07-03)

Phase 2 is implemented on `feature/remove-skel-dart7-phase2` and awaiting PR
creation/review/merge. The branch removes the C++ SKEL parser, `dart::io`
`.skel` and `<skel>` inference/dispatch, `ModelFormat::Skel`,
parser-specific tests, dartpy SKEL bindings/stubs, user-facing SKEL docs, and
all sample `.skel` fixtures. Non-SKEL Kima Collada meshes were relocated from
`data/skel/kima/` to `data/mesh/kima/` and the mesh tests were updated to load
that path.

Phase 2 validation run on the branch:

- `pixi run python -m pytest python/tests/unit/test_check_dartpy_import_layout.py python/tests/unit/utils/test_utils_stub_import.py python/tests/unit/test_check_dart7_world_promotion_blockers.py`
- `pixi run check-ai-commands`
- `pixi run check-dartpy-import-layout`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run run-cpp-target UNIT_dynamics_MeshShape`
- `pixi run run-cpp-target test_mesh_loaderNone`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-py`
- `pixi run test-all`
- `CUDAARCHS=89 DART_CUDA_ARCHITECTURES=89 pixi run -e cuda test-all`

The first CUDA full-gate attempt hit a stale/default generated CMake CUDA
architecture (`compute_52`) before target-level `DART_CUDA_ARCHITECTURES`
could apply. Clearing `build/cuda/cpp/Release` and setting both
`CUDAARCHS=89` and `DART_CUDA_ARCHITECTURES=89` matched this host's RTX 5000
Ada GPU and the CUDA full gate passed.

The DART 7 changelog has a draft Breaking Changes entry without a PR link; add
the link after the Phase 2 PR exists.

Additional validation for URDF joint-properties fixture coverage:

- Added `UrdfWriter.RoundTripsExistingJointPropertiesFixture`, which loads
  `data/urdf/test/joint_properties.urdf`, writes it through
  `UrdfParser::tryWriteSkeletonToString()`, reparses the emitted URDF, and
  compares body inertias, visual box geometry, revolute and continuous joint
  topology, axes, finite velocity/effort limits, and passive damping/friction
  metadata.
- Extended the URDF writer to emit `<limit velocity="..." effort="...">` for
  continuous joints when both symmetric finite metadata fields are present;
  lower/upper remain omitted for continuous joints, matching URDF's model.
- SDF policy check: `pixi run check-sdf-sdformat-boundary` passed, so this
  slice keeps SDF IO on libsdformat and does not add DART-side SDF XML parsing.

Validation commands:

- `pixi run check-sdf-sdformat-boundary`
- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.RoundTripsExistingJointPropertiesFixture'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter`
- `pixi run lint`
- `pixi run build`

Changelog decision:

- Mode: draft
- Base evidence: current local diff extends the unreleased conservative URDF
  writer contract and adds fixture-level read/write/read coverage.
- Decision: entry required as a revision to the existing DART 7 URDF writer
  bullet.
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing existing URDF
  writer bullet.
- Entry text: the existing conservative URDF writer bullet now includes
  continuous joint velocity/effort limits.

Additional validation for URDF material and ground fixture coverage:

- Added `UrdfWriter.RoundTripsExistingIssue838Fixture`, which loads
  `data/urdf/test/issue838.urdf`, writes it through
  `UrdfParser::tryWriteSkeletonToString()`, reparses the emitted URDF, and
  compares body inertias, visual box geometry, visual colors imported from
  global URDF material references, fixed-joint topology, and revolute joint
  limits.
- Added `UrdfWriter.RoundTripsExistingGroundFixture`, which loads
  `data/urdf/KR5/ground.urdf`, writes it through
  `UrdfParser::tryWriteSkeletonToString()`, reparses the emitted URDF, and
  compares the DART skeleton semantics imported from the fixture: the root
  `world` link remains an inertial-frame placeholder rather than a body node,
  and the `ground_link` inertial, visual box, and collision box semantics
  survive the writer round trip.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.RoundTripsExistingIssue838Fixture:UrdfWriter.RoundTripsExistingGroundFixture:UrdfWriter.RoundTripsExistingJointPropertiesFixture'`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_urdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing URDF writer bullet
  already covers visual colors, primitive geometry, visual/collision poses, and
  explicit unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for existing shipped URDF fixtures without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as URDF material and ground fixture round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for URDF WAM package-mesh fixture coverage:

- Generalized the URDF writer fixture assertions in
  `tests/integration/io/test_urdf_writer.cpp` so read/write/read coverage can
  compare supported box, sphere, cylinder, and mesh shape semantics rather than
  only box fixtures.
- Added `UrdfWriter.RoundTripsExistingWamFixture`, which loads
  `data/urdf/wam/wam.urdf` with `UrdfParser` package resolution for
  `herb_description`, writes it through
  `UrdfParser::tryWriteSkeletonToString()`, reparses the emitted URDF, and
  compares the DART skeleton semantics imported from the fixture: root
  `world` remains an inertial-frame placeholder, body inertias survive,
  visual/collision package mesh geometry and URIs survive, visual material
  colors survive, and the `/j1` revolute joint keeps topology, axis, and limit
  metadata.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.RoundTripsExistingWamFixture:UrdfWriter.RoundTripsExistingIssue838Fixture:UrdfWriter.RoundTripsExistingGroundFixture:UrdfWriter.RoundTripsExistingJointPropertiesFixture'`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter`
- `pixi run check-sdf-sdformat-boundary`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_urdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing URDF writer bullet
  already covers package URI meshes, primitive geometry, visual/collision
  poses, visual colors, and explicit unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped package-mesh URDF robot
  without adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as WAM package-mesh URDF fixture round-trip verification
  if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for URDF drchubo and KR5 root-pose fixture coverage:

- Added `UrdfWriter.RoundTripsExistingDrchuboFixture`, which loads
  `data/urdf/drchubo/drchubo.urdf` with `UrdfParser` package resolution for
  `drchubo`, writes it through `UrdfParser::tryWriteSkeletonToString()`,
  reparses the emitted URDF, and compares the DART skeleton semantics imported
  from the fixture: body inertias, visual/collision package mesh geometry and
  URIs, visual material colors, and representative revolute joint topology,
  axes, limits, and dynamics.
- Added `UrdfWriter.ExistingKr5FixtureWithRootPoseReturnsError`, which loads
  `data/urdf/KR5/KR5 sixx R650.urdf` and proves the writer rejects its imported
  non-identity root-link pose with the targeted `URDF has no root-link pose`
  diagnostic instead of attempting a lossy write.
- SDF policy check: `pixi run check-sdf-sdformat-boundary` passed before this
  slice; no SDF code changed, and SDF IO remains bounded to libsdformat rather
  than DART-side XML parsing.

Validation commands:

- `pixi run check-sdf-sdformat-boundary`
- `pixi run run-cpp-target INTEGRATION_io_UrdfWriter`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_urdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing URDF writer bullet
  already covers package URI meshes, visual/collision poses, visual colors, and
  explicit unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification plus one shipped-fixture diagnostic without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as drchubo URDF package-mesh round-trip coverage plus
  KR5 root-pose diagnostic coverage if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for SDF quad root-model fixture coverage:

- Added `SdfWriter.RoundTripsExistingQuadFixture`, which loads
  `dart://sample/sdf/quad.sdf`, writes it through
  `SdfParser::tryWriteSkeletonToString()`, reparses the emitted SDF, and
  compares the DART skeleton semantics imported from the fixture: 17 body
  nodes, 16 child revolute joints plus the implicit root joint, body inertias,
  finite joint axis limits, joint topology and axes, repeated visual/collision
  boxes, foot sphere visuals, shape poses, visual material colors, mobility,
  and gravity.
- The test stays on the libsdformat-backed DART SDF parser/writer/reparser
  path. It does not add XML-level SDF parsing or inspect emitted XML text.
- Expected source-fixture warnings from libsdformat are the legacy
  `use_parent_model_frame` element warnings in `quad.sdf`.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingQuadFixture'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (87 tests)
- `pixi run check-sdf-sdformat-boundary`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  already covers the conservative SDF writer, visual material colors, primitive
  visual/collision geometry, finite effort/velocity limits, and explicit
  unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped SDF quadruped model
  without adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as quad SDF root-model fixture round-trip verification
  if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for SDF root-model issue fixture coverage:

- Added `SdfWriter.RoundTripsExistingSimpleJointModelFixtures`, which loads
  `dart://sample/sdf/test/test_issue1583.model` and
  `dart://sample/sdf/test/test_issue1683.model`, writes them through
  `SdfParser::tryWriteSkeletonToString()`, reparses the emitted SDF with
  `RootJointType::Fixed`, and compares the DART skeleton semantics imported
  from the fixtures: model names, mobility, gravity, body inertias, fixed
  parent-world root joints, child revolute joint topology/axes/limits, and
  cylinder/box visual geometry.
- The test stays on the libsdformat-backed DART SDF parser/writer/reparser
  path. It does not add XML-level SDF parsing or inspect emitted XML text.
- Expected source-fixture warnings are the missing explicit inertia warnings
  for links that provide mass but no inertia tensor.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingSimpleJointModelFixtures'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (88 tests)
- `pixi run check-sdf-sdformat-boundary`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  already covers the conservative SDF writer, fixed/root joint behavior,
  primitive visual/collision geometry, finite limits, and explicit
  unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for existing shipped SDF issue models without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as SDF root-model issue fixture round-trip verification
  if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for SDF universal-joint issue fixture coverage:

- Added `SdfWriter.RoundTripsExistingUniversalJointModelFixture`, which loads
  `dart://sample/sdf/test/test_issue1596.model`, writes it through
  `SdfParser::tryWriteSkeletonToString()`, reparses the emitted SDF through the
  normal `SdfParser` path, and compares the DART skeleton semantics imported
  from the fixture: model name, mobility, gravity, body inertias, parent-world
  and child universal joint topology/axes/limits/dynamics, and box/sphere
  visual plus box collision geometry.
- The test stays on the libsdformat-backed DART SDF parser/writer/reparser
  path. It does not add XML-level SDF parsing or inspect emitted XML text.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingUniversalJointModelFixture'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter` (89 tests)
- `pixi run check-sdf-sdformat-boundary`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  already covers the conservative SDF writer, universal joints, root-joint
  behavior, primitive visual/collision geometry, finite limits/dynamics, and
  explicit unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped SDF issue model without
  adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as SDF universal-joint issue fixture round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for SDF relative-mesh model fixture coverage:

- Added `SdfWriter.RoundTripsExistingRelativeMeshModelFixture`, which loads
  `dart://sample/sdf/test/include_relative_mesh/included_model/model.sdf`,
  writes it through `SdfParser::tryWriteSkeletonToString()`, reparses the
  emitted SDF through the normal `SdfParser` path, and compares the DART
  skeleton semantics imported from the fixture: model name, mobility, gravity,
  body inertia, implicit root joint, visual/collision mesh shape poses, mesh
  scale, and parser-resolved source mesh URIs.
- The test stays on the libsdformat-backed DART SDF parser/writer/reparser
  path. It does not add XML-level SDF parsing or inspect emitted XML text.
- The fixture proves preservation after the SDF parser has resolved the source
  mesh URI to a retrievable absolute URI. It does not change the writer policy
  that targetless relative mesh references authored directly on DART
  `MeshShape`s remain unsupported until a destination-aware file/project writer
  exists.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingRelativeMeshModelFixture'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run check-sdf-sdformat-boundary`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  already covers the conservative SDF writer, mesh visual/collision geometry,
  and explicit unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing shipped SDF relative-mesh model
  without adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as SDF relative-mesh model fixture round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for combined SDF collision-surface metadata coverage:

- Added `SdfWriter.RoundTripsCombinedCollisionSurfaceMetadata`, which writes one
  collision carrying disabled-collision bitmask state, ODE friction/slip
  metadata, collision-frame first friction direction, and bounce restitution.
- The test validates the emitted SDF with libsdformat `sdf::Root`,
  `sdf::Surface`, `sdf::Contact`, `sdf::Friction`, and `sdf::ODE` DOM values,
  using the existing narrow `sdf::Element` bridge only for the bounce schema
  values that sdformat 16 does not expose through a high-level DOM class.
- It then reparses the emitted SDF through `SdfParser` and verifies the combined
  metadata survives as DART collision and dynamics aspects. No XML-level SDF
  parsing or emitted-text inspection was added.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsCombinedCollisionSurfaceMetadata'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run check-sdf-sdformat-boundary`
- `git diff --check`
- `pixi run build`
- `pixi run lint`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  already covers the conservative SDF writer, collision-surface contact
  bitmasks, bounce restitution, ODE friction/slip metadata, and explicit
  unsupported/lossy diagnostics.
- Decision: no additional changelog entry. This slice adds combined
  read/write/read verification for an existing unreleased SDF writer contract
  without adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as combined SDF collision-surface metadata verification
  if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for top-level SDF `ground.world` fixture coverage:

- Added `SdfWriter.RoundTripsExistingGroundWorldFixture`, which loads the
  shipped top-level `dart://sample/sdf/ground.world` fixture through
  `SdfParser`, writes it with `SdfParser::tryWriteSkeletonToString()`, and
  reparses the emitted SDF through `SdfParser`.
- The test validates the emitted SDF with libsdformat `sdf::Root`,
  `sdf::Model`, `sdf::Link`, `sdf::Visual`, `sdf::Collision`,
  `sdf::Geometry`, `sdf::Surface`, `sdf::Friction`, and `sdf::ODE` DOM values.
  It does not add XML-level SDF parsing or emitted-text substring checks.
- The read/write/read assertions cover the DART semantics imported from this
  world fixture: static model state, plane geometry imported as DART box
  geometry, disabled visual shadow state, and high ODE friction metadata.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsExistingGroundWorldFixture'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run check-sdf-sdformat-boundary`
- `pixi run build`
- `pixi run lint`
- `git diff --check`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: `CHANGELOG.md` DART 7 IO and Parsing SDF writer bullet
  already covers the conservative SDF writer, collision-surface ODE metadata,
  visual shadow state, shipped-fixture coverage, and explicit unsupported/lossy
  diagnostics.
- Decision: no additional changelog entry. This slice adds fixture-level
  read/write/read verification for an existing unreleased SDF writer contract
  without adding a new public API or broadening the documented writer contract.
- Target section: N/A
- Entry text: N/A
- PR-body note: record as `ground.world` SDF fixture round-trip verification if
  a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for selected SDF mimic world model coverage:

- Added parser-specific `SdfParser::Options::mModelName` for named SDF model
  selection. It uses libsdformat `sdf::Root`/`sdf::World::ModelByName` DOM
  lookup; empty model names keep the previous top-level / first-world-first-model
  behavior.
- Added `SdfParser.SelectsNamedWorldModel` and
  `SdfParser.MissingNamedWorldModelReturnsNull`.
- Added `SdfWriter.RoundTripsSelectedMimicWorldFixture`, selecting
  `pendulum_with_base_mimic_slow_follows_fast` from
  `dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf`, writing
  SDF 1.11, validating emitted `sdf::JointAxis::Mimic()` metadata through the
  libsdformat world/model DOM, and reparsing through `SdfParser`.
- No XML-level SDF parsing, model enumeration, or emitted-text substring checks
  were added.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfParser --gtest_filter=SdfParser.SelectsNamedWorldModel:SdfParser.MissingNamedWorldModelReturnsNull && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsSelectedMimicWorldFixture'`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run check-sdf-sdformat-boundary`
- `pixi run build`
- `pixi run lint`
- `git diff --check`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in `SdfParser::Options`,
  `tests/integration/io/test_sdf_parser.cpp`,
  `tests/integration/io/test_sdf_writer.cpp`,
  `docs/onboarding/io-parsing.md`, and this SKEL evolution task folder.
- Scope evidence: this slice adds a parser-specific SDF loading option that
  users can call directly to select named models from multi-model SDF world
  files; it also broadens fixture-level SDF writer validation for existing
  SDF 1.11 mimic metadata.
- Decision: update the existing DART 7 IO and Parsing loading bullet for
  parser-specific SDF world model selection, and fold the selected mimic
  fixture coverage into the existing SDF writer documentation rather than
  adding a separate writer changelog bullet.
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing
- Entry text: present as the updated URDF/SDF/MJCF loading APIs bullet, without
  a PR link until an implementation PR exists.
- PR-body note: record as parser-specific SDF named model selection plus
  selected mimic world fixture round-trip verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Changelog decision:

- Mode: draft
- Base evidence: `origin/main`
- Scope evidence: `git diff --stat origin/main...HEAD` removes the SKEL parser,
  `.skel` fixtures, SKEL docs, parser tests, and dartpy SKEL bindings/stubs.
- Decision: entry required
- Target section: `CHANGELOG.md` -> DART 7 -> Breaking Changes
- Entry text: present as the legacy SKEL removal bullet, without a PR link
  until the Phase 2 PR exists.
- Follow-up: add the Phase 2 PR link to the same bullet before merge.

Post-commit completion audit cleared stale active-doc references that still
listed SKEL as a DART 7 model-loading/parser target. The remaining tracked
SKEL-format references outside this task folder are the DART 7 removal
changelog entry and historical DART 6 changelog entries.

Additional validation for selected SDF double-pendulum world model coverage:

- Added `SdfWriter.RoundTripsSelectedDoublePendulumWorldFixtures`, selecting
  `double_pendulum_with_base` from `dart://sample/sdf/double_pendulum.world`
  and `dart://sample/sdf/double_pendulum_with_base.world` through
  `SdfParser::Options::mModelName`.
- The test writes each selected model back through
  `SdfParser::tryWriteSkeletonToString()`, validates the emitted model through
  `sdf::Root` / `sdf::World` / `sdf::Model` DOM lookup, and reparses through
  `SdfParser`.
- Coverage proves selected non-first world models round-trip parent-world
  revolute roots, child revolute joints, visual-only pendulum geometry, and
  visual/collision pendulum geometry without XML-level model enumeration or
  emitted-text substring checks.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsSelectedDoublePendulumWorldFixtures'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run check-sdf-sdformat-boundary`
- `pixi run build`
- `pixi run lint`
- `git diff --check`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder.
- Scope evidence: this slice broadens fixture-level SDF writer validation for
  selected existing world models under the same unreleased conservative SDF
  writer/parser round-trip contract.
- Decision: no additional changelog entry.
- Target section: not applicable.
- PR-body note: record as selected double-pendulum SDF world fixture
  round-trip verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for selected second-world SDF model coverage:

- Added `SdfWriter.RoundTripsSelectedModelFromSecondWorld`, using a synthetic
  SDF document with two `<world>` entries and selecting
  `selected_second_world_model` from the second world through
  `SdfParser::Options::mModelName`.
- The test writes the selected model through
  `SdfParser::tryWriteSkeletonToString()`, validates the emitted SDF through
  `sdf::Root` / `sdf::World` / `sdf::Model` / `sdf::Link` DOM lookup, and
  reparses through `SdfParser`.
- Coverage proves named model selection searches sdformat world DOM objects
  beyond the first world and preserves the selected world's gravity,
  visual/collision box geometry, and visual material color without XML-level
  world/model enumeration or emitted-text substring checks.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsSelectedModelFromSecondWorld'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run check-sdf-sdformat-boundary`
- `pixi run build`
- `pixi run lint`
- `git diff --check`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder.
- Scope evidence: this slice broadens test evidence for the existing
  parser-specific SDF named-model selection and unreleased conservative SDF
  writer contract, without adding a new public API or user-visible export
  capability.
- Decision: no additional changelog entry.
- Target section: not applicable.
- PR-body note: record as selected second-world SDF model round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for selected shipped many-model SDF cube coverage:

- Added `SdfWriter.RoundTripsSelectedIssue1624CubeFixture`, selecting
  `model_0_0_1` from `dart://sample/sdf/test/issue1624_cubes.sdf` through
  `SdfParser::Options::mModelName`.
- The test writes the selected cube through
  `SdfParser::tryWriteSkeletonToString()`, validates the emitted SDF through
  typed `sdf::Root` / `sdf::Model` / `sdf::Link` / `sdf::Visual` /
  `sdf::Collision` / `sdf::Box` DOM lookup, and reparses through `SdfParser`.
- Coverage proves named model selection can skip the default first model in a
  shipped many-model SDF world and preserve model pose, mass/inertia, red visual
  material color, and visual/collision box geometry without XML-level model
  enumeration or emitted-text substring checks.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsSelectedIssue1624CubeFixture'`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run check-sdf-sdformat-boundary`
- `pixi run build`
- `pixi run lint`
- `git diff --check`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder.
- Scope evidence: this slice broadens shipped-fixture test evidence for the
  existing parser-specific SDF named-model selection and unreleased
  conservative SDF writer contract, without adding a new public API or
  user-visible export capability.
- Decision: no additional changelog entry.
- Target section: not applicable.
- PR-body note: record as selected `issue1624_cubes.sdf` SDF cube
  round-trip verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for shipped `benchmark.world` selected-model coverage:

- Extended `SdfWriter.RoundTripsSelectedDoublePendulumWorldFixtures` with
  `dart://sample/sdf/benchmark.world`, selecting
  `double_pendulum_with_base10b` through `SdfParser::Options::mModelName`.
- Coverage proves the SDF parser/writer path can skip unresolved includes and
  unrelated in-world models, including a static plane model, then write,
  validate through libsdformat `sdf::Root` / `sdf::World` / `sdf::Model` DOM,
  and reparse the selected two-link pendulum skeleton without adding XML-level
  SDF enumeration or text parsing.

Validation command:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_SdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_SdfWriter --gtest_filter=SdfWriter.RoundTripsSelectedDoublePendulumWorldFixtures'`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_sdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder.
- Scope evidence: this slice broadens shipped-fixture test evidence for the
  existing sdformat-backed SDF writer contract, without adding a new public API
  or user-visible export capability.
- Decision: no additional changelog entry.
- Target section: not applicable.
- PR-body note: record as selected `benchmark.world` SDF writer round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for shipped URDF primitive geometry fixture coverage:

- Added `UrdfWriter.RoundTripsExistingPrimitiveGeometryFixture`, loading
  `dart://sample/urdf/test/primitive_geometry.urdf` through `UrdfParser`,
  writing the imported skeleton through
  `UrdfParser::tryWriteSkeletonToString()`, and reparsing the emitted URDF.
- Coverage proves the parser-selected primitive visual geometry imported into
  DART (`base_link` with one visual sphere and no collisions) survives
  write/read/read with body, inertia, visual geometry, and collision-count
  comparison. It does not claim preservation of the fixture's raw source XML
  sibling geometry tags beyond the skeleton semantics imported by `UrdfParser`.

Validation commands:

- `pixi run bash -lc 'CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter && ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.RoundTripsExistingPrimitiveGeometryFixture'`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_urdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder.
- Scope evidence: this slice broadens shipped-fixture test evidence for the
  existing parser-specific URDF writer contract, without adding a new public
  API or user-visible export capability.
- Decision: no additional changelog entry.
- Target section: not applicable.
- PR-body note: record as `primitive_geometry.urdf` URDF writer round-trip
  verification if a PR is opened.
- Follow-up: none until an implementation PR exists.

Additional validation for shipped URDF Atlas fixture coverage:

- Added `UrdfWriter.RoundTripsExistingAtlasV5NoHeadFixture`, loading
  `dart://sample/sdf/atlas/atlas_v5_no_head.urdf` through `UrdfParser`, writing
  the imported skeleton through `UrdfParser::tryWriteSkeletonToString()`, and
  reparsing the emitted URDF from a memory URI.
- Coverage proves the current conservative URDF writer preserves the shipped
  Atlas fixture's 36 body nodes, 35 DoFs, body inertias, visual/collision mesh
  geometry and resolved relative mesh URIs, representative revolute joint
  topology/axes/limits/dynamics, and fixed camera joints through write/read/read.
  It compares DART skeleton semantics imported by `UrdfParser`; it does not
  claim preservation of raw URDF-only tags outside that skeleton contract.

Validation command:

- `pixi run bash -lc 'set -euo pipefail; CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_io_UrdfWriter; ./build/default/cpp/Release/bin/INTEGRATION_io_UrdfWriter --gtest_filter=UrdfWriter.RoundTripsExistingAtlasV5NoHeadFixture'`

Changelog decision:

- Mode: decide
- Base evidence: current local diff in
  `tests/integration/io/test_urdf_writer.cpp`, `docs/onboarding/io-parsing.md`,
  and this SKEL evolution task folder.
- Scope evidence: this slice broadens shipped-fixture test evidence for the
  existing parser-specific URDF writer contract, without adding a new public API
  or user-visible export capability.
- Decision: no additional changelog entry.
- Target section: not applicable.
- Entry text: N/A.
- PR-body note: record as Atlas URDF writer round-trip verification if a PR is
  opened.
- Follow-up: none until an implementation PR exists.

Additional validation for Phase 5 bounded writer closeout:

- Marked Phase 5 complete for the current DART 7 writer scope in this task's
  README and export-writer plan. Future SDF/URDF writer expansion is parked
  behind durable criteria in `docs/onboarding/io-parsing.md` instead of keeping
  the phase open-ended.
- Existing evidence remains the parser-specific SDF and URDF writer APIs,
  read/write/read coverage in `INTEGRATION_io_SdfWriter` and
  `INTEGRATION_io_UrdfWriter`, unsupported-case diagnostics, PLAN-101
  project/scene save-load coverage in the dartsim engine layer, and the existing
  DART 7 `CHANGELOG.md` IO/Parsing bullets for SDF and URDF writers.
- Validation commands for this docs-only closeout:
  - `git diff --check`
  - `pixi run check-sdf-sdformat-boundary`
  - `pixi run lint`
- Changelog decision: no additional changelog entry. This slice changes task
  status and durable follow-up criteria only; the unreleased user-visible SDF and
  URDF writer capabilities already have DART 7 changelog entries.

## Previous Resume Checkpoint (2026-07-03)

Phase 1 landed on `main` via PR
[#3065](https://github.com/dartsim/dart/pull/3065), merge commit
`5dbe995d221` (2026-06-19): all SKEL files that were referenced by examples,
tests, or benchmarks have SDF replacements and the live
test/resource-retriever references have moved off `dart://sample/skel`. The
converted fixtures are:

- `data/skel/test/single_pendulum.skel` -> `data/sdf/test/single_pendulum.sdf`
- `data/skel/cube.skel` -> `data/sdf/test/cube.sdf`
- `data/skel/shapes.skel` -> `data/sdf/test/shapes.sdf`
- `data/skel/test/test_shapes.skel` -> `data/sdf/test/test_shapes.sdf`

The only remaining `.skel` strings under tests were parser-specific temporary
paths in `tests/integration/io/test_skel_parser.cpp`; they were removed with the
parser in Phase 2.

## Last Session Summary

Captured the strategic decision to close out
[issue #496](https://github.com/dartsim/dart/issues/496) by _not_ converting
SKEL to YAML, and to remove SKEL before the DART 7 release. Optional YAML work
should be a new DART 7 format decision or a front-end over existing URDF/SDF
semantics, not SKEL syntax in YAML. USD adoption remains tracked via
[`usd_scene_loader/`](../usd_scene_loader/), and export writers remain part of
the longer-term round-trip story. The retired `feature/skel_yaml` branch carried
a 1,991-line draft plan; that draft is preserved by SHA so useful phase details
can be recovered without adopting the old SKEL YAML direction.

## Current Branch

`work/skel-format-yaml-decision` — Phase 3 YAML decision, Phase 4 USD
coordination, Phase 5 bounded writer-scope closeout, and accumulated SDF/URDF
writer slices are local on top of merged PR #3288.

## Immediate Next Step

Prepare the final dev-task cleanup change. Phase 3, Phase 4, and Phase 5 are
already recorded on this stacked branch; Phase 5 is closed for the current DART
7 writer scope unless a maintainer reopens writer scope.

## Context That Would Be Lost

- The 2015 issue #496 proposed converting SKEL XML to YAML. That proposal is
  rejected here; the rationale lives in `README.md` under _Key Decisions_.
- YAML is rejected as a DART 7 `dart::io` model format and as a front-end over
  third-party canonical formats. If added later, it must be a DART-owned
  project/scene serialization format, not SKEL syntax in YAML.
- USD work is tracked separately under
  [`usd_scene_loader/`](../usd_scene_loader/) — do not duplicate.
- Phase 4 is a coordination boundary, not a claim that USD implementation is
  complete. The USD task still owns the OpenUSD-enabled loader, viewer, dartpy,
  dependency, and CI work.
- Export was part of the work, not a follow-up. SDF/URDF writer round-trip is
  the portable interchange path, while dartsim editor save/load is already
  covered by PLAN-101's project format. Phase 5 is complete for the bounded
  DART 7 writer scope; future SDF/URDF expansion is parked in
  `docs/onboarding/io-parsing.md`.
- The old `feature/skel_yaml` prototype SHA (`1dd83e31586`) is no longer
  reachable in this worktree. Do not depend on it for Phase 3 or Phase 5 unless
  another clone still has the object.

## How to Resume

```bash
git checkout work/skel-format-yaml-decision
git status && git log -4 --oneline
pixi run run-cpp-target INTEGRATION_io_SdfWriter

# Optional archaeology only if another clone still has the object:
# git show 1dd83e31586:docs/dev_tasks/skel_format/phase-02-yaml.md
# git show 1dd83e31586:docs/dev_tasks/skel_format/phase-04-export.md
```

Then continue with the remaining task work by preparing dev-task cleanup. If
Phase 5 is reopened later, start from this folder's
`README.md`, `03-yaml-decision.md`, `04-usd-coordination.md`,
`05-export-writers-plan.md`, `docs/onboarding/io-parsing.md`, and current
DART 7 requirements, not from the old prototype.
