# 03. Dominoes (dartpy)

## Overview

This tutorial mirrors the domino controller sample in pure python.  It walks
through the same lessons as the C++ document:

1. Create the domino, floor, and manipulator skeletons.
2. Add, lean, and remove dominoes while preventing collisions.
3. Implement the gravity-compensated PD controller and the operational-space
   controller (OSC) that pushes the first domino.
4. Apply additional body forces to the human using keyboard input.

All code lives in `python/tutorials/03_dominoes/`.  Launch it with:

```bash
pixi run py-tu-dominoes
```

## Lesson 1 – Create the world

### 1a. Base domino and floor

`create_domino()` constructs a `FreeJoint`/`BodyNode` pair and attaches a thin
box. The `Joint_pos_z` DOF is set to `default_domino_height / 2`, which keeps the
box resting on the floor.  `create_floor()` uses a `WeldJoint` plus a large box
shape to create the ground plane—identical to the C++ setup.

### 1b. Manipulator

`create_manipulator()` loads `dart://sample/urdf/KR5/KR5 sixx R650.urdf` via
`dart.utils.DartLoader`.  The base joint is translated backward so the arm sits
behind the domino stack, matching the initial pose in `tutorialDominoes.cpp`.

## Lesson 2 – Domino editing

`DominoEventHandler` manages a deque of domino skeletons and the cumulative angle
of the line.  Pressing `q`, `w`, or `e` calls `attempt_to_create_domino(angle)`.
The method:

1. Clones the template domino (`self.first_domino.clone("domino_n")`).
2. Computes a displacement along the current heading using `default_distance *
   [cos(total_angle), sin(total_angle), 0]` and applies it to the clone.
3. Adjusts the yaw (`pose[2]`) by the requested lean angle.
4. Removes the floor from the world collision group, builds a new collision group
   containing the clone, and checks for intersections.  If the clone would
   penetrate anything, the method prints a warning and discards it; otherwise it
   adds the clone to the world and tracks it in `self.dominoes`.

Pressing `d` removes the most recently added domino from both the world and the
history vectors, mirroring Lesson 2d from the C++ tutorial.

## Lesson 3 – Controller setup

`Controller` stores the manipulator, the first domino, the end-effector
(`mEndEffector` in C++), the target frame, and an offset from the wrist frame to
 the contact point.  Reducing the tutorial to python does not change the math: we
still rely on `SimpleFrame` to hold the target transform and use the same gains.

### 3a. Gravity-compensated PD

`set_pd_forces()` implements the SPD controller described in the tutorial.  It
copies the manipulator’s positions/velocities into numpy arrays, advances them by
one time step, and forms the error terms.  The mass matrix and the
Coriolis/gravity vectors come from `dart.dynamics.Skeleton`.  The forces are then
applied via `Skeleton.setForces(...)`.

### 3b. Operational-space controller (OSC)

Holding `r` enables OSC.  The method computes the six-dimensional Jacobian at the
wrist offset, forms the pseudo-inverse the same way the C++ sample does, and
constructs the angular and translational error terms using
`dart.math.AngleAxis`.  A small feed-forward wrench is added along +X to “push”
the domino.  The final torques are the sum of the OSC output, the PD forces, and
the gravity/coriolis compensation terms.

## Lesson 4 – External pushes

`MyWorldNode` tracks an external force vector and a countdown timer.  Pressing `,`
or `.` calls `set_external_force(force, duration)`.  During `customPreStep()` the
force is applied to `h_spine` with `BodyNode.addExtForce()`.  An `ArrowShape`
attached to a `SimpleFrame` is updated so the push is visible—exactly like the
“disembodied force” described in the C++ tutorial.  When the timer expires the
arrow is hidden and the stored force resets to zero.

## Lesson 5 – Keyboard reference

| Key | Effect |
| --- | ------ |
| `q` / `w` / `e` | Add leaning dominoes before the simulation starts |
| `d` | Delete the last domino |
| `space` | Start / pause the simulation |
| `p` | Replay |
| `f` | Push the first domino with a disembodied force |
| `r` | Engage the OSC controller on the manipulator arm |
| `,` / `.` | Apply backward / forward pushes to the torso |
| `v` | Toggle contact-force visualization |

Use the tutorial the same way you would use the C++ version: place dominoes,
start the simulation, press `f` or `r` to knock them over, and experiment with
the torso pushes.  Because the entire sample is written in dartpy you can script
new behaviors (for example, auto-generating a domino spiral) with only a few
lines of Python.
