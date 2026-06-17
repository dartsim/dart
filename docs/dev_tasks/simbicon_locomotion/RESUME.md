# Resume: Python SIMBICON Locomotion

## Current Resume Checkpoint (2026-06-17, Warm-Start Checkpoint Retirement)

Current branch: `docs/retire-warm-start-checkpoint`.

PR [#3043](https://github.com/dartsim/dart/pull/3043) is merged on `main`; the
small Atlas SIMBICON pose-window comparison utility no longer needs the old
checkpoint branch. This branch documents the remaining durable evidence from
`feature/dart7-unified-contact-warm-start` in
[`02-warm-start-branch-retirement.md`](02-warm-start-branch-retirement.md) so
that checkpoint can be removed after this note lands and the maintainer
explicitly approves deleting the local and remote branch.

Validation so far:

- `pixi run python -m pytest python/tests/unit/test_compare_atlas_simbicon_pose_window.py -q`
- `pixi run lint`

Immediate next step: finish the docs-only retirement PR, then run
`scripts/compare_atlas_simbicon_pose_window.py` against the late state-`0`
DART 6/DART 7 trace JSONs from current `main`. Compare stance-foot local z-axis
tilt with support-row counts around steps `3970`, `4000`, `4070`, and `4100`
before trying another stance hip-roll or torso-reaction probe. Do not return to
global native surface tolerance, seed-depth changes, reactive state-2 support
hold clamps, or blunt swing `hpx`/`hpy` target clamps without new evidence.

How to resume:

```bash
git checkout docs/retire-warm-start-checkpoint
git status --short --branch
pixi run lint
```

Then open a docs-only PR for this retirement note. After that PR lands, ask the
maintainer before deleting `feature/dart7-unified-contact-warm-start` locally
and remotely.

The older resume notes below predate PR #3043 and this retirement audit. Keep
them as SIMBICON task history, but do not treat their branch instructions as
current.

## Last Session Summary

The robot-agnostic Python SIMBICON controller and its three py-demos scenes
(`atlas_simbicon`, `g1_simbicon`, `simbicon_duo`) landed in PR #2786, including
an evidence-driven robustness pass that fixed the dominant fall mode (height
sink) with stance-leg height regulation. Both robots still topple over long
horizons via a lateral (coronal) instability, so `simbicon_duo` does not hold
up; this folder captures the diagnosis so the lateral-balance work can resume.

## Historical Baseline Branch

`main` — clean. PR #2786 is merged; the controller code is on `main`. There is
**no** in-progress feature branch for the remaining lateral-balance work yet.

## Original SIMBICON Next Step

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
