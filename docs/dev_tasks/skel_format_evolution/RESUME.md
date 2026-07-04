# Resume: SKEL Format Evolution

## Current Resume Checkpoint (2026-07-04 UTC)

Phase 3 is decided and committed locally on `work/skel-format-yaml-decision`,
stacked on the Phase 2 removal branch. The decision record is
[`03-yaml-decision.md`](03-yaml-decision.md): do not add YAML as a DART 7
`dart::io` model format, do not build a YAML wrapper over URDF/SDF/MJCF, and do
not carry SKEL syntax forward. If YAML returns, it must be a DART-owned
project/scene serialization design with schema/versioning rules plus import,
export, and round-trip tests.

Phase 2 remains implemented on `feature/remove-skel-dart7-phase2` and awaiting
PR creation/review/merge. The stacked Phase 3 branch is documentation-only and
should not be pushed as the Phase 2 removal PR.

Phase 4 coordination is also recorded locally in
[`04-usd-coordination.md`](04-usd-coordination.md): PR #3109 already landed the
OFF-by-default USD `dart::io` scaffold, and remaining USD loader/viewer/dartpy
implementation stays in `docs/dev_tasks/usd_scene_loader/` instead of this SKEL
task.

Phase 5 export-writer planning is recorded locally in
[`05-export-writers-plan.md`](05-export-writers-plan.md). The plan keeps export
as a separate implementation phase: choose SDF writer, URDF writer, or PLAN-101
project save/load first; define deterministic resource handling and comparison
helpers; and complete the phase only with writer APIs plus read/write/read
tests. This planning record does not complete Phase 5.

The first SDF writer implementation slice is now local on
`work/skel-format-yaml-decision`: `dart::utils::SdfParser` exposes
`tryWriteSkeletonToString()` for a conservative `Skeleton` subset, with
the writer building libsdformat DOM objects and serializing through sdformat.
`INTEGRATION_io_SdfWriter` proves write/read round-trip for links, root
FreeJoint/WeldJoint placement,
revolute/continuous/prismatic/weld/screw/universal child joints, inertial data,
box/sphere/cylinder/capsule/cone/ellipsoid/mesh geometry, link gravity mode,
passive joint dynamics metadata (damping, Coulomb friction, spring reference,
and spring stiffness), sdformat-normalized screw thread pitch, SDF 1.11+
axis/axis2 mimic metadata with motor enforcement, topology-only ball child
joints, local root/joint/shape poses, visual shadow/hidden state, explicit
visual material colors, PBR metallic/roughness factors, collision-surface
contact disable bitmasks, visual transparency, zero-threshold bounce
restitution, ODE friction
coefficients, first friction direction, slip compliance, and absolute non-file
mesh URI preservation through a custom retriever. Continuous SDF joints now
parse as unbounded DART
`RevoluteJoint`s, and unbounded DART revolute joints write back as SDF
`continuous` while finite-limit revolute joints stay `revolute`. Targetless
relative/generated mesh references, including relative or host-qualified file
URI forms, now return explicit diagnostics until a future file/project writer
defines a destination URI and resource copy/rewrite policy. It also checks
`WriteOptions` visual/collision filtering, unsupported-shape diagnostics,
missing mesh URI diagnostics, DART `PlaneShape` finite-size diagnostics,
`ConvexMeshShape` generated-resource diagnostics, pre-SDF-1.11 mimic
diagnostics, unsupported coupler-style mimic diagnostics, non-finite visual
material diagnostics, invalid PBR material diagnostics, non-finite screw pitch
diagnostics, invalid collision-surface friction/restitution diagnostics,
unsupported ball-joint metadata, and
non-finite joint dynamics diagnostics. This is real Phase 5 progress, but Phase
5 is still open until broader SDF coverage plus the remaining accepted writer
targets are implemented or durably deferred.

The supported SDF geometry reader paths now load through libsdformat
`sdf::Geometry` DOM objects before mapping sphere, box, cylinder, capsule,
cone, ellipsoid, plane, and mesh shapes into DART shapes; this preserves the
existing plane-as-thin-box and resource-retrieved mesh behavior without adding
new raw XML-level parsing.
The model reader now selects the top-level model through libsdformat
`sdf::Root::Model()` and carries the resulting `sdf::Model` DOM through standard
model, link, joint, visual, collision, and material traversal instead of
rediscovering those standard children through raw XML-level enumeration.
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
Standard SDF joint reads traverse `sdf::Model`,
`sdf::Joint`, and `sdf::JointAxis` DOM values for joint enumeration, joint
name/type,
parent/child links, local pose, axis vectors including
`axis/xyz@expressed_in` frame resolution, axis dynamics, finite position limits,
mimic metadata, and sdformat-normalized screw pitch values. XML helper checks
remain only where DART preserves authored/default distinctions for existing
diagnostics, reads DART-specific soft-body extension fields, or supports the
legacy `use_parent_model_frame` presence check. Those standard SDF
authored/default checks now use sdformat explicit-authored flags and
non-mutating direct child traversal instead of raw child-existence probing. The
compatibility boolean value now uses sdformat typed element access rather than
DART's SDF helper layer.
The SDF detail helper API has been narrowed to that remaining bridge: generic
XML attribute reads, string reads, vector2/vectorX parsing, child enumeration,
boolean value parsing, and direct helper tests for those deleted APIs are gone.
Retained helpers now cover only element presence and child lookup; standard
presence checks use `GetExplicitlySetInFile()`, lookup walks authored child
elements directly, and
DART-specific soft-body extension values are converted locally through
sdformat typed `Element::Get<T>` calls instead of shared DART-side scalar,
vector, or pose parsers.
The SDF writer's post-serialization preservation path now uses typed
`sdf::Model`, `sdf::Link`, and `sdf::Joint` DOM values for link/joint names,
gravity modes, and screw pitch values. It still patches sdformat's element tree
for fields that `Root::ToElement()` omits or emits under the deprecated tag,
but it no longer reads serialized XML attributes back from that tree, and the
patch path now finds generated sdformat children by direct child/sibling
traversal instead of `FindElement` schema lookup.

The SDF writer integration test now uses
`tests/helpers/io_round_trip_helpers.hpp` for reusable body, joint, DoF,
inertia, and shape comparisons, and `tests/CMakeLists.txt` includes the helper
in the formatted test-helper list. This closes the Phase 5 comparison-helper
infrastructure item, but it does not broaden the public writer contract by
itself.

The writer API home decision is now recorded in
`docs/onboarding/io-parsing.md` and
[`05-export-writers-plan.md`](05-export-writers-plan.md): keep export APIs
format-owned until more than one accepted writer contract exists. The SDF
writer stays on `dart::utils::SdfParser`, `dart::io` remains read-side, and
project/editor save-load belongs to the scene/project layer.

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

Additional validation for absolute non-file mesh URI writer coverage:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for writer options and missing mesh URI diagnostics:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for SDF 1.11 mimic metadata writer coverage:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for targetless relative/generated mesh URI diagnostics:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for targeted PlaneShape and ConvexMeshShape writer
diagnostics:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run lint`
- `pixi run build`

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

Additional validation for pruning the unused SDF XML helper API:

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
  entry is needed for SDF 1.11 mimic metadata coverage because it broadens the
  same conservative SDF writer capability before the implementation PR exists.
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
  PR exists. No additional entry is needed for pruning unused SDF XML helper
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
  sdformat explicit-authored metadata.
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
coordination, Phase 5 export-writer planning, and the first SDF writer slice are
local on top of the Phase 2 removal commit. The underlying Phase 2 branch
remains `feature/remove-skel-dart7-phase2`.

## Immediate Next Step

Continue with the remaining real task work: land Phase 2 after maintainer
approval, then continue Phase 5 from
[`05-export-writers-plan.md`](05-export-writers-plan.md) by extending SDF writer
coverage beyond the current material, link, joint, geometry, mimic, pose, and
resource subset or choosing the next accepted writer target (URDF or PLAN-101
project save/load).

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
- Export is part of the work, not a follow-up: round-trip enables save /
  load in the dartsim editor (PLAN-101). The current Phase 5 file is a plan,
  and the current SDF writer is only the first implementation slice, not an
  implementation-complete gate.
- The old `feature/skel_yaml` prototype SHA (`1dd83e31586`) is no longer
  reachable in this worktree. Do not depend on it for Phase 3 or Phase 5 unless
  another clone still has the object.

## How to Resume

```bash
git checkout work/skel-format-yaml-decision
git status && git log -4 --oneline
pixi run run-cpp-target INTEGRATION_io_SdfWriter

# To return to the Phase 2 PR-ready branch:
# git checkout feature/remove-skel-dart7-phase2

# Optional archaeology only if another clone still has the object:
# git show 1dd83e31586:docs/dev_tasks/skel_format/phase-02-yaml.md
# git show 1dd83e31586:docs/dev_tasks/skel_format/phase-04-export.md
```

Then continue with the remaining task work. For Phase 5, start from this
folder's `README.md`, `03-yaml-decision.md`, `04-usd-coordination.md`,
`05-export-writers-plan.md`, and current DART 7 requirements, not from the old
prototype.
