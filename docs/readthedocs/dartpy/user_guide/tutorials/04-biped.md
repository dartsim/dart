# 04. Biped (dartpy)

## Overview

This tutorial translates the biped control lessons into dartpy.  The structure is
identical to the C++ document:

1. Load the `fullbody1` model and build a floor.
2. Implement the gravity-compensated SPD controller.
3. Add an ankle strategy to recover from small pushes.
4. Apply external forces from keyboard input (and optionally drive skateboard
   wheels).

Run the tutorial with:

```bash
pixi run py-tu-biped
```

The code lives in `python/tutorials/04_biped/`, with `main.py` and
`main_finished.py` mirroring the lesson layout.

## Lesson 1 – Load the scene

The tutorial uses the SKEL file shipped with DART:

```python
world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
world.setGravity([0.0, -9.81, 0.0])
biped = world.getSkeleton("fullbody1")
```

Several DOFs are then set to reproduce the starting pose from the original
sample (pelvis rotation, thigh/knee angles, heel angles, abdomen roll).  The
floor is created by welding a thin 10×10×0.01 m box to the world and translating
it down along Y so that the biped’s feet rest on top of it.

## Lesson 2 – SPD controller

`MyWorldNode` precomputes diagonal `Kp`/`Kd` matrices and stores the nominal pose
`q_d`.  The first six rows (floating base) are zeroed out just like in the C++
tutorial.  Every `customPreStep()` call executes the following steps:

1. Grab the current positions/velocities (`q`, `dq`).
2. Compute `invM = (M + Kd * dt)^-1`.
3. Build the position/velocity error terms.
4. Solve for `ddq` and compute the torque vector `p + d - Kd * ddq * dt`.
5. Multiply by `0.8` for safety.

All of this math is identical to Lesson 3 in the C++ tutorial, but uses numpy and
`dartpy` accessors.

## Lesson 3 – Ankle strategy

After applying SPD torques, the tutorial adds the ankle strategy described in the
original document.  It computes the COM offset relative to a point near the left
heel and distributes corrective torques across the heel and toe DOFs on both
feet.  The gains (`k1`, `k2`, `kd`) and thresholds match the ones documented in
Lesson 4 of the C++ tutorial.

## Lesson 4 – External pushes and visualization

`InputHandler` listens for `,` and `.` and calls `set_external_force()` with a
force along ±X.  The node applies that force to `h_spine` and visualizes it with
an `ArrowShape` attached to a `SimpleFrame`.  When the countdown expires the
arrow is hidden and the force vector is reset.

## Lesson 5 – Wheel commands (optional)

For the optional skateboard extension, `Controller.setWheelCommands()` zeroes out
the PD gains for the wheel joints and sets velocity commands via
`Skeleton.setCommand()`.  `changeWheelSpeed()` increments `self.speed` and prints
the current value so you can monitor it while holding `a`/`s`.

## Keyboard reference

| Key | Effect |
| --- | ------ |
| `space` | Toggle simulation |
| `p` | Replay |
| `.` | Apply a forward push |
| `,` | Apply a backward push |
| `a` | Increase skateboard speed (optional) |
| `s` | Decrease skateboard speed (optional) |

Experiment with the same flows as the C++ tutorial: let the SPD/ankle controller
stabilize the character, then apply pushes or drive the wheels.  Because the
sample is written entirely in dartpy, adding new input bindings or logging COM
trajectories is as simple as editing the python script.
