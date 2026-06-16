# Resume: Python SIMBICON Locomotion

## Current Resume Checkpoint (2026-06-16, Foot Tilt Comparison Utility)

Current branch: `feature/simbicon-foot-tilt-metric`.

This branch starts from current `origin/main` and keeps the large
`feature/dart7-unified-contact-warm-start` branch as a checkpoint only. It ports
the small Atlas SIMBICON pose-window comparison utility and adds per-foot pose
metrics: transform translation, trace-vertical position, local z-axis tilt
relative to the trace vertical axis, and stance/swing aliases when the trace
state identifies the active limb.

Validation so far:

- `pixi run python -m pytest python/tests/unit/test_compare_atlas_simbicon_pose_window.py -q`
- `pixi run lint`

Immediate next step: run `scripts/compare_atlas_simbicon_pose_window.py` against
the late state-`0` DART 6/DART 7 trace JSONs once those artifacts are present
locally, then compare stance-foot local z-axis tilt with support-row counts
around steps `3970`, `4000`, `4070`, and `4100` before trying another stance
hip-roll or torso-reaction probe. Do not return to global native surface
tolerance or blunt swing `hpx`/`hpy` target clamps without new evidence.

How to resume:

```bash
git checkout feature/simbicon-foot-tilt-metric
git status --short --branch
pixi run python -m pytest python/tests/unit/test_compare_atlas_simbicon_pose_window.py -q
```

Then run the compare script with appropriate `--dart6-json`, `--dart7-json`,
`--mesh-dir`, and `--steps 3970,4000,4070,4100` arguments.

## Last Session Summary

The robot-agnostic Python SIMBICON controller and its three py-demos scenes
(`atlas_simbicon`, `g1_simbicon`, `simbicon_duo`) landed in PR #2786, including
an evidence-driven robustness pass that fixed the dominant fall mode (height
sink) with stance-leg height regulation. Both robots still topple over long
horizons via a lateral (coronal) instability, so `simbicon_duo` does not hold
up; this folder captures the diagnosis so the lateral-balance work can resume.

## Current Branch

`main` — clean. PR #2786 is merged; the controller code is on `main`. There is
**no** in-progress feature branch for the remaining lateral-balance work yet.

## Immediate Next Step

Reproduce the duo failure and characterize it: run the long-horizon
single-robot harness in `01-diagnosis.md` for `g1` and `atlas` (≥3000 steps),
then run the `simbicon_duo_trace.py` harness from the same file. Confirm whether
the lateral topple is per-robot or an interaction in the shared world. Do
**not** trust a short headless render — it looks upright well before the topple.

## Context That Would Be Lost

- The failure is **lateral (coronal)**, not sagittal: traces show `psag` stays
  small while `d_cor`/`v_cor` blow up and the swing foot flails high instead of
  planting (`swing_z` far above ground). Sagittal/torso control is fine.
- `height_kp=2.0` is a real, verified win and is already baked into both configs
  — do not remove it. It is near a sweet spot; higher values over-correct the
  knee and destabilize (Atlas blows up by `height_kp` 4–8).
- Coronal **gain tuning is a dead end** (measured): `cv_coronal` up and a
  per-side `coronal_swing_sign` flip both made things worse. The next attempt
  must be structural (foot placement / contact timing), not a new gain.
- Balance feedback currently uses the hip midpoint as a COM proxy even though
  dartpy exposes skeleton COM position (`getCOM` / `get_com`). The remaining
  gap is COM linear velocity: use a finite-difference velocity estimate or add a
  focused `getCOMLinearVelocity` binding before spending time on a custom COM
  position implementation.
- The exact sweep/trace harnesses used last session lived in `/tmp` and are
  reproduced verbatim in `01-diagnosis.md` (they were not committed).

## How to Resume

```bash
git checkout main && git pull --ff-only
# Build dartpy if needed (the demos import the compiled extension):
pixi run build
# Verify current behavior with the fall-step harness (see 01-diagnosis.md):
#   PYTHONPATH="build/default/cpp/Release/python:python" \
#     pixi run python /tmp/simbicon_param_sweep.py g1 height_kp 2 3000
#   PYTHONPATH="build/default/cpp/Release/python:python" \
#     pixi run python /tmp/simbicon_duo_trace.py 3000
git status && git log -3 --oneline
```

Then: create a feature branch and pursue the lateral-balance fix per
`01-diagnosis.md` ("Restart / next-attempt options"). Re-verify over ≥3000 steps
per robot **and** the duo before claiming robustness.
