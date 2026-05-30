# SIMBICON Locomotion — Failure Diagnosis & Restart Notes

Evidence gathered while landing PR #2786. "Fall step" = the simulation step at
which the pelvis leaves the control window (`min_pelvis_height`), at a 0.001 s
time step unless noted. Numbers are for balance-in-place configs (no forward
lean).

## How it fails

State traces (FSM state, pelvis sagittal/coronal lean, COM offset `d` and
velocity `v` relative to the stance ankle, pelvis height, swing-foot height,
sampled every 100 steps) show a consistent story for **both** robots:

1. **Sagittal is fine.** `psag` stays small (G1 ≤ ~0.18 rad) the whole run; the
   robot never topples front/back. The world-frame torso control works.
2. **Dominant fall mode = gradual height sink.** The pelvis height drifts down
   cycle over cycle (the stance leg crouches a little deeper each step) until it
   drops below `min_pelvis_height` and the controller cuts out. Addressed by
   `height_kp` (stance-knee height regulation).
3. **Residual fall mode = lateral (coronal) instability.** Once the sink is
   controlled, the robot survives longer but eventually `pcor` and `v_cor` blow
   up (G1: `pcor` → ~-1.0 rad, `v_cor` → ~-5 at the fall) and the swing foot
   **flails high instead of planting** (`swing_z` ~0.8 m above ground in
   swing-down), so it never catches the falling COM. This is what topples the
   `simbicon_duo` robots.

## Sweep evidence

`height_kp` (stance-leg height regulation), 3000-step horizon:

| height_kp | Atlas               | G1                  |
| --------- | ------------------- | ------------------- |
| 0         | FELL@2807           | FELL@1170           |
| 0.5       | FELL@1671 (blew up) | FELL@1261           |
| 1.0       | FELL@2162           | FELL@1519           |
| 2.0       | **SURVIVED 3000**   | **FELL@2204**       |
| 4.0       | FELL@1481           | FELL@1191 (blew up) |
| 8.0       | FELL@1151           | FELL@848            |

`height_kp=2.0` is the sweet spot for both. Atlas at a 6000-step horizon:
FELL@4778 (so it is robust to ~4.8k steps, then the lateral mode catches it).

Approaches that did **not** help (all with `height_kp=2.0`, G1, 3000 steps):

| Knob                                          | Values → fall step                               | Verdict       |
| --------------------------------------------- | ------------------------------------------------ | ------------- |
| `coronal_swing_sign` (per-side hip-roll flip) | 0→2204, +1→1463, -1→2161                         | worse         |
| `cv_coronal` (lateral velocity gain)          | 0.2→2204, 0.5→2378 (blew up), 1.0→1462, 1.5→1381 | worse         |
| `torso_kd` (torso damping), Atlas             | 1→2807, 100→2631, 200→2445, 400→2796, 800→392    | neutral→worse |

Conclusion: **coronal balance is not gain-limited.** A per-side sign flip and a
stronger velocity term both made it worse, so the next attempt must change the
_structure_ of lateral control, not its gains. (The `coronal_swing_sign` /
`cd_coronal` / `cv_coronal` knobs explored here were reverted before merge since
they added inert config with no win — re-add deliberately if a structural change
needs them.)

## Restart / next-attempt options (lateral balance)

Ordered roughly by expected value / effort. None verified yet.

1. **Lateral foot placement that actually plants.** The swing foot flails high
   in swing-down. Make the swing-down target drive the foot toward the ground
   and toward the projected COM in the coronal plane (SIMBICON places the swing
   foot to catch the COM; the current swing-down knee/hip targets may not bring
   it down on the lighter/different G1 geometry). Verify the foot reaches the
   contact-height proxy before the swing-down timeout.
2. **Better contact detection.** Swing-down advances on a foot-height proxy (no
   per-body collision query in dartpy). If the proxy mis-fires, the gait desyncs
   and the robot loses lateral balance. Consider using the world's contact
   results filtered by the foot body if a per-body query can be exposed.
3. **COM proxy accuracy.** Balance feedback uses the hip-midpoint as the COM
   proxy (no `getCOM` in dartpy). For lateral balance this approximation may be
   too coarse; consider a mass-weighted COM from body transforms + inertias, or
   expose `getCOM`/`getCOMLinearVelocity` in dartpy.
4. **Coronal-specific gains/targets** (only after a structural change): give the
   coronal plane its own `cd`/`cv` and a nonzero default stance width so the
   support polygon is wider; lateral SIMBICON is sensitive to stance width.
5. **Duo-specific:** confirm the two robots in the shared world are not coupling
   through contacts/solver; if they are, separate them further or give each its
   own world and compose for rendering.

Decision still open: keep `simbicon_duo` as a short in-place showcase, gate it
behind robust balance, or redesign lateral control first.

## Reproduction harnesses

These lived in `/tmp` during the landing session (not committed). Run with
`PYTHONPATH="build/default/cpp/Release/python:python" pixi run python <script>`.

Fall-step sweep — `simbicon_param_sweep.py <atlas|g1> <field> <v,v,...> [steps]`:

```python
import sys
import numpy as np

sys.path.insert(0, "python")
sys.path.insert(0, "build/default/cpp/Release/python")

from examples.demos.scenes._simbicon_robots import (
    build_simbicon_setup, load_atlas_skeleton, load_g1_skeleton,
    make_atlas_config, make_g1_config,
)

ROBOT, FIELD = sys.argv[1], sys.argv[2]
VALUES = [float(x) for x in sys.argv[3].split(",")]
STEPS = int(sys.argv[4]) if len(sys.argv) > 4 else 3000
loader, make_cfg = (
    (load_atlas_skeleton, make_atlas_config) if ROBOT == "atlas"
    else (load_g1_skeleton, make_g1_config)
)

def run(val):
    cfg = make_cfg()
    setattr(cfg, FIELD, val)
    world, controllers, pre_step = build_simbicon_setup([(loader, cfg, 0.0)])
    pelvis = controllers[0]._pelvis
    pz = lambda: float(np.asarray(pelvis.get_world_transform().matrix())[2, 3])
    px = lambda: float(np.asarray(pelvis.get_world_transform().matrix())[0, 3])
    x0, fell = px(), -1
    for i in range(STEPS):
        pre_step(); world.step()
        z = pz()
        if not np.isfinite(z) or z < cfg.min_pelvis_height:
            fell = i; break
    tag = f"FELL@{fell}" if fell >= 0 else f"SURVIVED({STEPS})"
    print(f"{ROBOT} {FIELD}={val}: {tag} dx={px()-x0:+.3f} z={pz():+.3f}", flush=True)

for v in VALUES:
    run(v)
```

Failure trace — `simbicon_trace.py <atlas|g1> [steps] [torso_kd]` (prints
state/lean/COM-offset/velocity/heights every 100 steps until it falls). Same
imports/preamble as above; per step, sample `ctl._pelvis_angle(sagittal=...)`,
`ctl._balance_d_v()`, pelvis `z`, and the swing foot's world `z` (the swing foot
is `ctl._right_foot if ctl._swing_is_right() else ctl._left_foot`). Watch
`d_cor`/`v_cor` grow and `swing_z` stay high near the fall.

Redirect stderr to `/dev/null` — the solver prints many LCP-fallback warnings
that bury the result lines.
