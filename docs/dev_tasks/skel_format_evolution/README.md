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
      stiffness), sdformat-normalized screw thread pitch, SDF 1.11+ axis/axis2
      mimic metadata, universal two-axis joints, continuous revolute joints,
      topology-only ball joints, capsule/cone/ellipsoid geometry, explicit
      visual material colors, and PBR metallic/roughness factors now round-trip
      as SDF `<gravity>`,
      `<axis>/<axis2><dynamics>`, `<thread_pitch>`/`<screw_thread_pitch>`,
      `<mimic>`, local
      `<pose>`, `<capsule>`, `<cone>`, `<ellipsoid>`, `<diffuse>`, and
      `<pbr><metal>` values.
      Absolute non-file mesh URIs now have writer read/write/read coverage
      through a custom retriever. Writer options for excluding visuals or
      collisions, missing mesh URI diagnostics, pre-SDF-1.11 mimic diagnostics,
      unsupported coupler-style mimic diagnostics, and relative/generated mesh
      resource diagnostics now have focused coverage. The writer test now uses a
      shared IO round-trip assertion helper for body, joint, DoF, inertia, and
      shape comparisons.
      Parser-side SDF model selection and standard model/link/joint/aspect
      traversal now use libsdformat `sdf::Root`, `sdf::Model`, `sdf::Link`,
      `sdf::Joint`, `sdf::Visual`, and `sdf::Collision` DOM values rather than
      raw XML-level enumeration. Standard visual/collision/material reads use
      libsdformat DOM values for names, local poses, geometry, diffuse colors,
      and PBR metal workflow factors; material XML checks only preserve whether
      diffuse was authored. Standard joint reads use DOM values for joint
      name/type,
      parent/child links, local pose, axis vectors including
      `axis/xyz@expressed_in` frame resolution, axis dynamics, finite limits,
      mimic metadata, and sdformat-normalized screw pitch values; XML checks
      remain only where DART needs authored/default presence, DART-specific
      soft-body extension fields, or the legacy `use_parent_model_frame`
      presence check; the legacy boolean value itself now uses sdformat typed
      element access.
      The SDF-specific helper layer no longer exposes generic XML attribute,
      string, boolean, vector2/vectorX, or child-enumerator APIs; retained
      helpers are limited to the small extension/presence bridge still used by
      the parser, and remaining extension values are converted through
      sdformat typed parameters rather than XML text reparsing.
      Writer-side preservation of disabled link gravity and SDF 1.10+
      `<screw_thread_pitch>` now derives names and values from typed
      `sdf::Model` / `sdf::Link` / `sdf::Joint` DOM objects instead of reading
      serialized XML attributes back out of sdformat's element tree.
      Writer APIs stay format-owned for now: the SDF writer remains on
      `dart::utils::SdfParser`, `dart::io` stays read-side, and project/editor
      save-load belongs to the scene/project layer. Broader SDF coverage, URDF
      export, and project save/load remain open. Add YAML only if a durable
      project/scene schema is accepted first.

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
   [`05-export-writers-plan.md`](05-export-writers-plan.md): extend SDF writer
   coverage beyond the first conservative subset, decide the next writer target
   (URDF or PLAN-101 project save/load), and keep read/write/read tests attached
   to every expanded contract. Absolute non-file mesh URI preservation is
   covered; targetless relative/generated mesh references are rejected until a
   future file/project writer defines a destination URI and copy/rewrite policy.
   Keep writer APIs format-owned unless a later multi-format write API is
   reviewed. YAML can enter that slice only after a durable project/scene schema
   is accepted.

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
  diagnostics plus non-finite visual material and invalid PBR material
  diagnostics. It also proves link-level gravity mode for gravity-disabled
  links and passive joint dynamics metadata plus sdformat-normalized screw
  thread pitch for supported single-axis
  joints, two-axis universal joints, SDF 1.11+ axis/axis2 mimic metadata with
  motor enforcement, continuous revolute joints, topology-only ball joints,
  plus local root, joint, and shape poses, plus capsule/cone/ellipsoid geometry
  and absolute non-file mesh URI preservation through a custom retriever. It
  also covers
  `WriteOptions`
  visual/collision filtering, the empty mesh URI diagnostic, pre-SDF-1.11 mimic
  diagnostics, and unsupported coupler-style mimic diagnostics, plus targetless
  relative/generated mesh resource diagnostics. The assertions are factored
  through
  `tests/helpers/io_round_trip_helpers.hpp` so future writer formats can reuse
  the body, joint, DoF, inertia, and shape comparison helpers. Completion still
  requires broader accepted-format writer APIs and round-trip tests that load a
  scene from each accepted format, write it back, and compare the re-parsed
  models. SDF helper cleanup is covered by `test_sdf_helpersNone`, which now
  validates only the retained parser bridge instead of the removed generic XML
  helper APIs, including the removed SDF-specific boolean value parser and XML
  text fallback parser.
