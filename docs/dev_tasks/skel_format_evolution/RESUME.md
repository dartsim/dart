# Resume: SKEL Format Evolution

## Current Resume Checkpoint

Phase 1 is implemented locally on `feature/remove-skel-dart7`: all SKEL files
that were referenced by examples, tests, or benchmarks have SDF replacements and
the live test/resource-retriever references have moved off `dart://sample/skel`.
The converted fixtures are:

- `data/skel/test/single_pendulum.skel` -> `data/sdf/test/single_pendulum.sdf`
- `data/skel/cube.skel` -> `data/sdf/test/cube.sdf`
- `data/skel/shapes.skel` -> `data/sdf/test/shapes.sdf`
- `data/skel/test/test_shapes.skel` -> `data/sdf/test/test_shapes.sdf`

The only remaining `.skel` strings under tests are parser-specific temporary
paths in `tests/integration/io/test_skel_parser.cpp`; remove those with the
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

`feature/remove-skel-dart7` — local Phase 1 fixture conversion, not pushed.

## Immediate Next Step

Run the remaining Phase 1 gates if they have not already passed in the current
session: focused IO read tests, `UNIT_simulation_world_SkeletonLoader`, resource
retriever tests, `pixi run check-docs-policy`, and `pixi run lint`. After this
conversion branch lands, pick Phase 2: remove `SkelParser`, `.skel`
inference/dispatch, SKEL tests, bindings/stubs, and `.skel` sample fixtures.
Do not delete the whole `data/skel` tree without first preserving or relocating
non-SKEL mesh assets such as `data/skel/kima/*.dae`, which are still loaded by
live mesh tests.

## Context That Would Be Lost

- The 2015 issue #496 proposed converting SKEL XML to YAML. That proposal is
  rejected here; the rationale lives in `README.md` under _Key Decisions_.
- YAML, if added, must be a new DART 7 format decision or a front-end over
  third-party canonical formats, not SKEL syntax in YAML.
- USD work is tracked separately under
  [`usd_scene_loader/`](../usd_scene_loader/) — do not duplicate.
- Export is part of the work, not a follow-up: round-trip enables save /
  load in the dartsim editor (PLAN-101).

## How to Resume

```bash
git checkout feature/remove-skel-dart7

# Inspect the prototype plan (while reflog still holds it):
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-01-deprecation.md
```

Then run the remaining verification gates and prepare the follow-up Phase 2
removal branch.
