# SKEL Format Evolution — Dev Task

## Current Status

- [ ] Phase 0: Capture strategic decision + prototype reference (this PR)
- [ ] Phase 1: SKEL deprecation. Add a runtime deprecation warning to
      `dart::utils::SkelParser`, update tutorials and onboarding to point at
      URDF / SDF / MJCF, and remove SKEL examples that have a URDF/SDF twin
- [ ] Phase 2: YAML front-ends for URDF and SDF (not for SKEL). Decide on a
      YAML parser (rapidyaml is the prototype's pick) and add the front-ends
      behind `dart::io::ModelFormat::UrdfYaml` and `SdfYaml`, mirroring the
      existing format dispatch
- [ ] Phase 3: USD support. Coordinate with
      [`usd_scene_loader/`](../usd_scene_loader/) so SKEL evolution and USD
      adoption share design pressure on the unified `dart::io` front door
- [ ] Phase 4: Export writers for URDF, SDF, and YAML so DART can round-trip
      a scene back to portable text

## Goal

Close out [issue #496](https://github.com/dartsim/dart/issues/496) (2015
proposal to convert SKEL XML to YAML) with a decision _not_ to YAML-ify SKEL,
and instead invest the same engineering budget into deprecating SKEL,
enhancing URDF and SDF, adopting USD, and adding format-export so DART scenes
can leave the engine in the same standards-track formats they came in.

## Non-Goals

- Re-implementing SKEL as YAML. Issue #496's literal proposal is rejected;
  the rationale is in _Key Decisions_ below.
- Forcing existing SKEL users off the format on a hard deadline. Read-only
  SKEL parsing stays until DART 8 removes deprecated DART 6 / 7 APIs.
- Building DART-specific YAML schemas that diverge from the URDF and SDF
  semantics. YAML front-ends are alternate syntax over the same models.

## Key Decisions

- **SKEL is legacy, not evolving**. The format is verbose, string-based, and
  has no installed-base outside DART. Industry has converged on URDF (ROS)
  and SDF (Gazebo); MuJoCo and Drake have YAML alternatives over those same
  models, not over a fork-specific schema.
- **YAML belongs on URDF and SDF, not on SKEL**. A YAML front-end over
  `urdfdom` and `libsdformat` reuses the canonical models and authoring
  ecosystems. A SKEL-only YAML schema would create a third format DART has
  to maintain.
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

1. Land this dev-task folder.
2. When Phase 1 starts, fork a feature branch off `main`, run the
   deprecation warning + tutorial update as a single small PR, and remove
   the SKEL example files that have URDF / SDF twins.

## Verification Gates

- Phase 1: `pixi run lint`, `pixi run test-py` (parser warnings) and
  `pixi run check-docs-policy` (tutorial references stay accurate).
- Phase 2: focused tests under `tests/integration/io/` for the YAML
  front-ends, parity with the XML originals.
- Phase 3: see `usd_scene_loader/` gates.
- Phase 4: round-trip tests that load a scene from each format and write it
  back, comparing the re-parsed models.
