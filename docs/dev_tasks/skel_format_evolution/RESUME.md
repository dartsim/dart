# Resume: SKEL Format Evolution

## Current Resume Checkpoint (2026-07-03)

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

`main` — Phase 1 is merged. Start Phase 2 from a fresh branch off current
`main`.

## Immediate Next Step

Pick Phase 2: remove `SkelParser`, `.skel` inference/dispatch, SKEL tests,
bindings/stubs, and `.skel` sample fixtures. Do not delete the whole
`data/skel` tree without first preserving or relocating non-SKEL mesh assets
such as `data/skel/kima/*.dae`, which are still loaded by live mesh tests.

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
git checkout main
git pull --ff-only origin main
git checkout -b feature/remove-skel-dart7-phase2

# Inspect the prototype plan (while reflog still holds it):
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-01-deprecation.md
```

Then start the Phase 2 removal work from this folder's `README.md`.
