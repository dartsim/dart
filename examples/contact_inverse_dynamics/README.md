# Contact-Aware Inverse Dynamics Example

This example demonstrates the `dart::dynamics::ContactInverseDynamics` API together with DART's inverse kinematics to compute contact forces and joint torques for a biped performing a squat motion.

## Overview

The demo shows:
- **IK-generated motion**: A squat cycle is generated using DART's `WholeBodyIK` with foot and pelvis targets
- **Contact-aware inverse dynamics**: Each frame, `ContactInverseDynamics` computes joint torques and ground reaction forces
- **Real-time visualization**: Contact forces are shown as colored arrows (green = feasible, red = infeasible)

## Running

```bash
pixi run ex contact_inverse_dynamics
```

Or build manually:
```bash
pixi run config
cmake --build build/default/cpp/Release --target contact_inverse_dynamics --parallel 12
./build/default/cpp/Release/bin/contact_inverse_dynamics
```

## Controls

The ImGui widget provides:

- **Play/Pause**: Toggle motion playback
- **Playback Speed**: Adjust speed (0.1–2.0×)
- **Squat Depth**: Control squat depth (0.0–0.25 m); regenerate keyframes after changing
- **Friction Coefficient**: Contact friction parameter (0.0–1.5)
- **Force Arrow Scale**: Adjust visualization scale for contact force arrows
- **Regenerate Keyframes**: Re-run IK with current squat depth

On HiDPI/scaled displays the panel can appear tiny because the GUI works in
framebuffer pixels. Pass `--gui-scale <factor>` (for example `--gui-scale 2`,
accepted range 0.75-4.0) or set the `DART_GUI_SCALE` environment variable to
scale the panel, fonts, and window.

## Status Display

- **FEASIBLE/INFEASIBLE**: Whether contacts can realize the motion within friction limits
- **Residual Norm**: Infinity-norm of unactuated residual forces
- **Total Vertical Force**: Sum of vertical contact forces vs. body weight
- **Contact Forces**: Magnitude of force at each of the 4 contact points (left/right heel and toe)

## Plots

- **Joint Torques**: Real-time torque history for 3 representative leg joints
- **Total Vertical GRF**: Total vertical ground reaction force over time

## Frame-Limited Mode

For testing or automation (still opens a window, so a display or Xvfb is
required):

```bash
./contact_inverse_dynamics --frames 30 --screenshot /tmp/demo.png
```

This runs 30 frames and captures the final frame before exiting. The OSG
screen-capture handler appends a context/frame suffix to the base name, so the
file is written as `/tmp/demo_0_<frame>.png`. `--screenshot` has no effect
without `--frames`.
