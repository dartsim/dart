# Phase 3 YAML Decision

## Decision

Do not add YAML as a DART 7 `dart::io` model format, `ModelFormat::Yaml`, or
front-end syntax for URDF/SDF/MJCF.

If YAML is revisited, it must be a DART-owned project or scene serialization
format with its own durable owner doc, schema/versioning rules, and import plus
export round-trip tests. It must not carry SKEL syntax forward, and it must not
be a thin alternate syntax for a third-party format that already owns its
semantics and parser.

## Evidence Refreshed 2026-07-04 UTC

- [Issue #496](https://github.com/dartsim/dart/issues/496) is closed as
  WONTFIX. The closing direction says DART is not planning to evolve SKEL or add
  new formats there, and that new model-loading work should stay focused on
  widely used URDF/SDF/MJCF formats.
- Current DART 7 `dart::io` is a skeleton-loading front door, not a whole-world
  scene/project serializer. The local source of truth is
  [`docs/onboarding/io-parsing.md`](../../onboarding/io-parsing.md) and
  `dart/io/read.{hpp,cpp}`.
- [URDF](https://github.com/ros/urdfdom) remains the ROS robot-description lane
  with its own parser and XML data model.
- [SDFormat](https://gazebosim.org/libs/sdformat/) / [the SDFormat
  specification](https://sdformat.org/spec/) owns Gazebo-style models, worlds,
  physics, sensors, and conversion/versioning semantics.
- [MJCF](https://mujoco.readthedocs.io/en/latest/XMLreference.html) is MuJoCo's
  native modeling language with its own XML model compiler and reference
  semantics.
- [OpenUSD Physics](https://openusd.org/dev/api/usd_physics_page_front.html)
  already owns the medium-term scene graph / rigid-body interchange pressure
  that motivated the Phase 4 lane. DART tracks that work separately in
  [`docs/dev_tasks/usd_scene_loader/`](../usd_scene_loader/).
- PLAN-101 already names human-readable project round-trip as an editor need.
  That is a project save/load problem, not a reason to add a YAML skeleton
  loader to `dart::io`.

## Rationale

The original YAML proposal solved two old SKEL problems: hand-authoring XML was
unpleasant, and the SKEL parser carried DART-owned parsing machinery. DART 7 is
solving that at a different level by removing SKEL and keeping maintained
interchange formats at the `dart::io` front door.

A YAML wrapper over URDF/SDF/MJCF would create a second syntax whose semantics
still belong to those upstream formats. That makes every parser rule, default,
version change, include/resource behavior, and error diagnostic a parity burden
for DART without improving interchange. The historical issue discussion also
identified YAML-specific concerns that still matter for a model format:
aliases/cycles, parser subset differences, and emitter style stability can leak
into validation and round-trip behavior.

A new DART-owned YAML scene format is only worth considering if it solves a
DART-owned problem that URDF, SDF, MJCF, and USD do not solve. The likely such
problem is editor project save/load: preserving authored DART 7 World state,
GUI/project metadata, and round-trip editing intent. That belongs under the
dartsim project model or a future scene-serialization design, with writers from
day one.

## Consequences

- Do not add `yaml-cpp`, `ModelFormat::Yaml`, `YamlParser`, `.yaml`/`.yml`
  inference, or YAML parser tests as part of this dev task.
- Do not implement a SKEL-to-YAML migration path. Legacy SKEL assets should
  migrate to URDF, SDF, or MJCF, or stay on a DART 6 release branch.
- Keep USD work in `docs/dev_tasks/usd_scene_loader/`.
- Treat Phase 5 export as a separate round-trip design/implementation slice.
  YAML can enter that slice only if a durable project/scene schema is accepted
  first.

## Reconsideration Bar

Before any future YAML implementation starts, the durable owner doc must define:

1. The user workflow YAML uniquely enables.
2. Whether the target is `Skeleton`, DART 7 `World`, or a dartsim project file.
3. A versioned schema and migration policy.
4. A restricted YAML profile, including how aliases, anchors, merge keys,
   multi-document streams, duplicate keys, and cycles are rejected or normalized.
5. Import tests for valid files and malformed inputs.
6. Export tests proving stable, readable output.
7. Round-trip tests that load, write, re-load, and compare the DART-owned model.
8. A changelog/migration note explaining why URDF, SDF, MJCF, and USD are not
   sufficient for the accepted workflow.
