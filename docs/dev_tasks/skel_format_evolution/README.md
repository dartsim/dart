# SKEL Format Evolution — Dev Task

## Current Status

- [x] Phase 0: Capture strategic decision + prototype reference.
- [x] Phase 1: Convert live SKEL fixtures used by examples, tests, and
      benchmarks to SDF. This branch adds SDF replacements for the referenced
      `single_pendulum`, `cube`, `shapes`, and `test_shapes` fixtures, moves
      tests and resource retriever coverage off `dart://sample/skel`, and
      verifies examples/tutorials/python examples/benchmarks have no SKEL file
      loads.
- [ ] Phase 2: Remove SKEL from DART 7 `main`: parser implementation,
      `dart::io` format inference and dispatch, bindings/stubs, parser-specific
      tests, `.skel` sample fixtures, and user-facing DART 7 docs. Preserve or
      relocate non-SKEL mesh assets such as `data/skel/kima/*.dae` before
      deleting any parent directory.
- [ ] Phase 3: Optional YAML model/scene format. Decide whether YAML should be
      a new DART-owned scene format or a front-end over URDF/SDF semantics; do
      not implement SKEL syntax in YAML.
- [ ] Phase 4: USD support. Coordinate with
      [`usd_scene_loader/`](../usd_scene_loader/) so SKEL evolution and USD
      adoption share design pressure on the unified `dart::io` front door
- [ ] Phase 5: Export writers for URDF, SDF, and YAML so DART can round-trip
      a scene back to portable text

## Goal

Close out [issue #496](https://github.com/dartsim/dart/issues/496) (2015
proposal to convert SKEL XML to YAML) with a decision _not_ to YAML-ify SKEL,
and instead remove SKEL before the DART 7 release. Invest the same engineering
budget into verified fixture migration, URDF/SDF/MJCF coverage, optional new
YAML design, USD adoption, and format-export so DART scenes can leave the
engine in portable formats.

## Non-Goals

- Re-implementing SKEL as YAML. Issue #496's literal proposal is rejected;
  the rationale is in _Key Decisions_ below.
- Keeping SKEL support in DART 7 for compatibility. Legacy users should stay on
  a `release-6.*` branch until their assets are migrated.
- Building a YAML schema before deciding its maintenance, export, and
  interchange contracts. YAML remains optional and must not be SKEL-in-YAML.

## Key Decisions

- **SKEL is legacy and should leave DART 7**. The format is verbose,
  string-based, and has no installed-base outside DART. Industry has converged
  on URDF (ROS), SDF (Gazebo), and MJCF (MuJoCo); DART 7 should not carry a
  DART-only XML format forward.
- **YAML is optional, but not SKEL YAML**. A YAML front-end over `urdfdom` and
  `libsdformat` reuses canonical models and authoring ecosystems. A new
  DART-owned YAML scene format may still be worth evaluating, but it must be a
  clean DART 7 format with explicit semantics, tests, and export strategy.
- **USD is the medium-term scene format**. Pixar / NVIDIA convergence makes
  USD the most likely cross-engine scene format. Loader work is tracked
  separately under [`usd_scene_loader/`](../usd_scene_loader/); SKEL
  evolution should not duplicate that surface.
- **Export is part of the work**. SDF / URDF / YAML writers turn DART into a
  round-trip engine rather than read-only, which unblocks save / load in
  the dartsim editor (PLAN-101) and downstream pipelines like gz-physics.

## Prototype Reference

The original prototype lived on the now-deleted `feature/skel_yaml` branch.
Tip commit SHA (recoverable from reflog and `git fsck --unreachable` until
natural expiry):

- `1dd83e31586 wip` — adds 6 files (1,991 lines total) under
  `docs/dev_tasks/skel_format/`: `README.md`, `migration-guide.md`,
  `phase-01-deprecation.md`, `phase-02-yaml.md`, `phase-03-usd.md`, and
  `phase-04-export.md`. The detailed phase planning, deprecation timing
  table, YAML library tradeoff matrix, USD schema notes, and export round-
  trip strategy live there.

To inspect the verbose prototype during a phase pickup:

```bash
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-01-deprecation.md
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-02-yaml.md
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-03-usd.md
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-04-export.md
git show 1dd83e31586:docs/dev_tasks/skel_format/migration-guide.md
```

This compact dev task is the durable strategic frame; the prototype's
verbose phase docs are reusable seed material for the agent that picks each
phase up.

## Immediate Next Steps

1. Finish this Phase 1 conversion branch: run the focused IO/simulation/resource
   tests, docs policy, and lint.
2. Start Phase 2 on a follow-up branch: remove `SkelParser`, `.skel`
   inference/dispatch, parser-specific tests, bindings/stubs, and `.skel`
   sample fixtures. Preserve or move non-SKEL mesh assets such as
   `data/skel/kima/*.dae` before deleting any parent directory.
3. After SKEL removal is mechanically under control, decide whether Phase 3 is
   a YAML front-end over existing formats or a new DART-owned scene format.
4. Keep Phase 4 USD work coordinated with
   [`usd_scene_loader/`](../usd_scene_loader/) rather than duplicating that
   loader surface here.

## Verification Gates

- Phase 1: focused IO/simulation/resource retriever tests for the converted SDF
  fixtures; `rg` checks proving examples/tests/benchmarks no longer reference
  `dart://sample/skel` or `data/skel`; `pixi run check-docs-policy`;
  `pixi run lint`.
- Phase 2: focused tests proving `.skel` no longer dispatches through
  `dart::io`, parser-specific SKEL tests are removed, and the source tree no
  longer ships DART 7 sample `.skel` assets. Existing non-SKEL mesh tests that
  load `data/skel/kima/*.dae` must keep passing, either from their current path
  or after an explicit fixture relocation.
- Phase 3: focused parser tests for any accepted YAML design, including parity
  with the canonical format it maps to or explicit tests for a new DART-owned
  schema.
- Phase 4: see `usd_scene_loader/` gates.
- Phase 5: round-trip tests that load a scene from each format and write it
  back, comparing the re-parsed models.
