# Experimental Differentiable GUI

Differentiable-simulation demo for the experimental slice (PLAN-110). Before the
first frame is drawn the example runs a gradient-based **trajectory
optimization**: a ballistic body is thrown under gravity and its **initial
velocity** is optimized by plain gradient descent so the converged rollout lands
on a fixed target marker. The gradient comes from
`dart::simulation::experimental::diff::rollout` plus
`RolloutTrajectory::rolloutVjp` (the whole-rollout reverse-mode VJP), reusing the
throw-to-target loop from
`tests/unit/simulation/experimental/diff/test_diff_optimization.cpp`.

The Filament viewer then visualizes the result: a ground plane, the yellow
target marker, the faint gray pre-optimization trajectory (the rest throw, which
falls short), the blue optimized-arc trail, and the red projectile sphere that
animates the converged rollout frame by frame.

This example only builds when the GUI, experimental simulation, and the opt-in
differentiable build are all enabled
(`DART_BUILD_GUI`, `DART_BUILD_SIMULATION_EXPERIMENTAL`, and `DART_BUILD_DIFF`).

Run the interactive demo:

```bash
DART_BUILD_DIFF_OVERRIDE=ON pixi run ex experimental_differentiable_gui
```

Press `r` or use the `Reset Playback` panel button to restart the converged
rollout from its first state.

Run the headless capture (matches the CTest smoke):

```bash
DART_BUILD_DIFF_OVERRIDE=ON pixi run ex experimental_differentiable_gui -- \
  --headless --frames 30 --width 640 --height 480 \
  --screenshot /tmp/experimental_differentiable_gui.ppm
```

`--screenshot` writes a single final PPM image; `--out <dir>` writes a numbered
frame sequence. The optimizer prints its convergence summary
(`[throw-opt] iters=... final loss=... distance=...`) on startup, on both the
interactive and headless paths.
