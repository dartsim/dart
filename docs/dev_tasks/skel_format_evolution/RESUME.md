# Resume: SKEL Format Evolution

## Last Session Summary

Captured the strategic decision to close out
[issue #496](https://github.com/dartsim/dart/issues/496) by _not_ converting
SKEL to YAML, and to invest instead in SKEL deprecation, URDF / SDF YAML
front-ends, USD adoption (via [`usd_scene_loader/`](../usd_scene_loader/)),
and export writers. The retired `feature/skel_yaml` branch carried a
1,991-line draft plan; that draft is preserved by SHA so the verbose phase
docs can seed each phase as it picks up.

## Current Branch

`main` — no implementation work after the prototype plan.

## Immediate Next Step

Pick Phase 1 (SKEL deprecation). Open a focused PR that adds a runtime
deprecation warning to `dart::utils::SkelParser`, updates tutorials and
onboarding to point at URDF / SDF / MJCF, and removes SKEL examples that
have a URDF / SDF twin.

## Context That Would Be Lost

- The 2015 issue #496 proposed converting SKEL XML to YAML. That proposal is
  rejected here; the rationale lives in `README.md` under _Key Decisions_.
- YAML belongs on URDF and SDF (third-party canonical models), not on SKEL.
- USD work is tracked separately under
  [`usd_scene_loader/`](../usd_scene_loader/) — do not duplicate.
- Export is part of the work, not a follow-up: round-trip enables save /
  load in the dartsim editor (PLAN-101).

## How to Resume

```bash
git checkout main
git pull --ff-only origin main
git checkout -b feature/skel-deprecation-phase1

# Inspect the prototype plan (while reflog still holds it):
git show 1dd83e31586:docs/dev_tasks/skel_format/phase-01-deprecation.md
```

Then translate the prototype's Phase 1 plan into a deprecation-warning PR.
