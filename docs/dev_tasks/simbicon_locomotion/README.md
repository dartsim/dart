# Python SIMBICON Locomotion — Dev Task

## Latest Focus (2026-06-17, Late State-0 Stance Diagnostics)

Branch: `main` or a fresh branch from current `main`.

PR [#3043](https://github.com/dartsim/dart/pull/3043) landed the reusable Atlas
SIMBICON pose-window comparison utility on `main`, including per-foot
transform translation, trace-vertical position, local z-axis tilt relative to
the trace vertical axis, and stance/swing aliases. PR
[#3047](https://github.com/dartsim/dart/pull/3047) landed the retirement audit
for the old `feature/dart7-unified-contact-warm-start` checkpoint, and the
local and remote checkpoint branches have been deleted.

Immediate next step: run `scripts/compare_atlas_simbicon_pose_window.py` on the
late state-`0` trace window from current `main`. Use the stance-foot tilt
numbers together with support-row counts around steps `3970`, `4000`, `4070`,
and `4100` before trying another stance-reaction or torso-reaction probe.

## Current Status

- [x] Robot-agnostic SIMBICON controller landed (PR #2786): FSM, world-frame
      torso/swing-hip control, `theta_d = theta_d0 + c_d*d + c_v*v` balance
      feedback, per-robot configs for Atlas + Unitree G1, and three py-demos
      scenes (`atlas_simbicon`, `g1_simbicon`, `simbicon_duo`).
- [x] Pose-window comparison utility landed (PR #3043): use it from `main` for
      late state-`0` stance-foot pose/tilt comparisons instead of resuming the
      old warm-start checkpoint branch.
- [x] Warm-start checkpoint retired (PR #3047 + branch deletion): the old
      `feature/dart7-unified-contact-warm-start` branch is no longer a resume
      surface.
- [x] Diagnosed and partly fixed the dominant fall mode (gradual height sink)
      via stance-leg height regulation (`height_kp`); torso control completed as
      a true PD.
- [ ] **Robust walking / standing.** Both robots still topple over long
      horizons via a lateral (coronal) instability, so the **`simbicon_duo`
      scene does not hold up** — the headline bug this task tracks.
- [ ] Decide the fate of `simbicon_duo` (keep as a short in-place showcase vs.
      gate it behind robust balance vs. redesign the lateral controller).

## Goal

Both Atlas and the Unitree G1 balance/step (ideally walk) robustly and
indefinitely under the shared SIMBICON controller — including together in the
`simbicon_duo` scene — without toppling.

## Non-Goals (for now)

- Matching the paper's exact per-robot gaits or parameters.
- Forward-locomotion speed/quality targets (get robust balance-in-place first).
- New dartpy bindings — work within the current Python surface.

## Key Findings (from the landing session — see `01-diagnosis.md`)

- **Dominant fall mode is a height sink, not a sagittal topple.** The stance leg
  creeps into a deeper crouch each gait cycle until the pelvis leaves the
  control window. `height_kp` (stance-knee height regulation) is the verified
  lever: Atlas survival ~2807 → 3000+ steps; G1 ~1170 → 2204 steps.
- **Residual fall mode is lateral (coronal) instability** plus the swing foot
  failing to plant. Parameter tuning does **not** fix it — sweeping the coronal
  feedback gain and a per-side hip-roll sign flip both made it worse. This needs
  a structural fix (lateral foot placement / contact-timed swing-down), not more
  gain tuning.
- `torso_kd` damping is paper-faithful but is not the robustness lever; keep it
  modest.

## Immediate Next Steps

1. Run the pose-window comparison utility on late state-`0` DART 6/DART 7 trace
   JSONs for steps `3970`, `4000`, `4070`, and `4100`; compare stance-foot
   tilt, stance pose, support-row counts, and support patch location.
2. Attack lateral balance structurally rather than by gain tuning — candidate
   approaches are enumerated in `01-diagnosis.md` ("Restart / next-attempt
   options"). Re-verify with the fall-step harness over ≥3000 steps per robot
   **and** the duo, not a short render.

## References

- Paper: SIMBICON (Yin, Loken, van de Panne, SIGGRAPH 2007) —
  https://www.cs.ubc.ca/~van/papers/simbicon.htm
- Code: `python/examples/demos/scenes/_simbicon.py` (controller),
  `python/examples/demos/scenes/_simbicon_robots.py` (configs + loaders),
  `python/examples/demos/scenes/{atlas_simbicon,g1_simbicon,simbicon_duo}.py`.
- Landed in PR #2786.
- Pose-window comparison utility landed in PR #3043.
- Warm-start checkpoint retirement:
  [`02-warm-start-branch-retirement.md`](02-warm-start-branch-retirement.md).
