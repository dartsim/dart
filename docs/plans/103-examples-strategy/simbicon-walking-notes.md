# SIMBICON Walking — World-Native Port Notes

Durable evidence home for the `planned_simbicon_walking` row
(`python/examples/demos/scenes/planned.py::SIMBICON_WALKING`). This file replaces
the retired `docs/dev_tasks/simbicon_locomotion/` working folder: the
robustness diagnosis below is the part a future World-native port should not have
to rediscover.

## Current state

- **`atlas_simbicon`** (`python/examples/demos/scenes/atlas_simbicon.py`) is an
  open-loop SIMBICON **target-cycling preview** on the DART 7 `World`
  (`_robot_model_world.load_atlas_world`). It blends a four-state pose cycle
  through named leg joints; it has **no closed-loop balance feedback**. Its
  docstring already states that the robust controller is deferred until the
  lateral-balance gap is closed.
- **`planned_simbicon_walking`** is the visible roadmap placeholder for the
  closed-loop walking controller. `unblocker` = lateral balance + robust dynamic
  walking evidence; `retire_when` = a closed-loop walking row replaces the
  target-preview route.
- **`scripts/compare_atlas_simbicon_pose_window.py`** (+
  `python/tests/unit/test_compare_atlas_simbicon_pose_window.py`) remains a
  landed, tested utility for diffing stance/swing foot pose, trace-vertical
  position, and local z-axis tilt between two SIMBICON trace JSONs. Reuse it for
  late-state stance / support-geometry comparisons.

## What was removed, and why not to resurrect it

PR [#2868](https://github.com/dartsim/dart/pull/2868) ("Prune demos to World
catalog") deleted the DART 6-era closed-loop stack that the earlier work built
on: the robot-agnostic controller (`_simbicon.py`), the per-robot configs and
skeleton loaders (`_simbicon_robots.py`), the G1 scene (`g1_simbicon.py`), and
the two-robot `simbicon_duo.py`. That prune is the locked PLAN-103 decision to
keep the demo catalog World-native. **Do not restore those skeleton-based scenes.**
The closed-loop walking controller should be rebuilt fresh against the World API
(the surviving `atlas_simbicon` preview and `_robot_model_world` are the seeds).

## Verified robustness findings (from the PR #2786 landing investigation)

These were measured on the now-pruned DART 6 skeleton controller, but the fall
modes are controller/physics facts a World-native port will meet again.

- **Dominant fall mode is a height sink, not a sagittal topple.** Sagittal/torso
  control held (`psag` stayed small); the stance leg crept into a deeper crouch
  each gait cycle until the pelvis dropped below the control window.
  **Stance-leg height regulation (`height_kp`) is the verified lever** — near
  `height_kp ≈ 2.0` for both robots. Fall step vs. `height_kp`, 3000-step
  horizon (higher is better):

  | height_kp | Atlas             | G1       |
  | --------- | ----------------- | -------- |
  | 0         | 2807              | 1170     |
  | 1.0       | 2162              | 1519     |
  | 2.0       | **survived 3000** | **2204** |
  | 4.0       | 1481              | 1191     |
  | 8.0       | 1151              | 848      |

  Too much over-corrects the knee and destabilizes; keep it near the sweet spot.

- **Residual fall mode is lateral (coronal) instability, and it is NOT
  gain-limited.** Once the sink was controlled, `pcor`/`v_cor` eventually blew up
  while the swing foot flailed high instead of planting. Measured dead ends
  (`height_kp=2.0`, G1, 3000 steps): sweeping the lateral velocity gain
  `cv_coronal` and flipping a per-side hip-roll sign both made survival **worse**,
  not better. The next attempt must change the **structure** of lateral control,
  not its gains. Other probes that also failed on the pruned controller and
  should not be retried blind: blunt state-0/state-1 swing `hpx`/`hpy` target
  clamps, reactive late state-2 support-hold or hip-pitch clamps (too late /
  destabilizing), and global native-surface-tolerance or seed-depth contact
  tweaks (those belong to
  [`../080-rigid-body-dynamics-solver.md`](../080-rigid-body-dynamics-solver.md),
  not the controller).

- `torso_kd` (torso damping) is paper-faithful but is not the robustness lever;
  keep it modest.

## Structural next-attempt options (for the World-native port)

Ordered roughly by expected value / effort. None verified.

1. **Lateral foot placement that actually plants.** Drive the swing-down target
   toward the ground and toward the projected COM in the coronal plane (SIMBICON
   places the swing foot to catch the COM). Verify the foot reaches the
   contact-height proxy before the swing-down timeout.
2. **Better contact detection.** Swing-down advanced on a foot-height proxy; if
   it mis-fires the gait desyncs and lateral balance is lost. Prefer a per-body
   contact query (filter the world contact results by the foot body) over a
   height proxy.
3. **Real COM proxy.** Balance feedback used the hip midpoint as a COM proxy even
   though dartpy exposes `getCOM`/`get_com`. Test the real COM position with a
   finite-difference velocity estimate first; if useful, add a focused
   `getCOMLinearVelocity` binding rather than reimplementing COM in Python.
4. **Coronal-specific gains/targets** (only after a structural change): give the
   coronal plane its own `cd`/`cv` and a nonzero default stance width — lateral
   SIMBICON is sensitive to stance width.
5. **Multi-robot coupling.** If a two-robot showcase returns, confirm the robots
   are not coupling through shared-world contacts/solver; separate them further
   or give each its own world and compose only for rendering.

## Diagnostic method (durable)

- Use a **long-horizon fall-step harness — ≥3000 steps per robot and the duo.**
  A short headless render looks upright well before the topple; do not trust it.
- Trace FSM state, pelvis sagittal/coronal lean, COM offset `d` and velocity `v`
  relative to the stance ankle, pelvis height, and swing-foot height. Watch
  `d_cor`/`v_cor` grow and the swing foot stay high near the fall.
- For late-state stance/support-geometry questions, diff two trace JSONs with
  `scripts/compare_atlas_simbicon_pose_window.py` (stance-foot tilt vs.
  support-row counts and support-patch placement).

## References

- SIMBICON: Yin, Loken, van de Panne, SIGGRAPH 2007 —
  <https://www.cs.ubc.ca/~van/papers/simbicon.htm>
- PR [#2786](https://github.com/dartsim/dart/pull/2786): landed the (now-pruned)
  robot-agnostic controller + robustness pass.
- PR [#2868](https://github.com/dartsim/dart/pull/2868): pruned the closed-loop
  scenes into the World catalog.
- PR [#3043](https://github.com/dartsim/dart/pull/3043): pose-window comparison
  utility. PR [#3047](https://github.com/dartsim/dart/pull/3047): warm-start
  checkpoint retirement audit.
