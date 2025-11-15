# 01. Multi-Pendulum (dartpy)

## Overview

This tutorial mirrors the five lessons in `tutorialMultiPendulum.cpp`, but every
step is implemented with dartpy. Along the way you will:

- create articulated chains programmatically
- attach visualization, collision, and dynamics aspects to each body node
- apply joint torques or body forces from keyboard input
- modify implicit springs and dampers while the simulation is running
- attach/detach constraints using the dartpy constraint solver

All code lives in `python/tutorials/01_multi_pendulum/`. Each directory contains
`main.py` (a scaffold) and `main_finished.py` (the reference solution). Run the
python tutorial with:

```bash
pixi run py-tu-multi-pendulum
```

## Lesson 0 – Build and visualize the pendulum

### 0a. Root body

`make_root_body()` creates the root `BallJoint`/`BodyNode` pair. After calling
`pendulum.createBallJointAndBodyNodePair(None, joint_prop, body_prop)` the helper
invokes `set_geometry()` which:

1. Creates a `BoxShape` sized `default_width × default_depth × default_height`.
2. Centers the shape on the local COM and attaches visual/collision/dynamics
   aspects.
3. Sets the body mass/inertia from the box volume.

This is the direct python translation of Lesson 0’s instructions.

### 0b. Additional links

`add_body()` appends a `RevoluteJoint` attached to the previously created body.
The joint axis is aligned with `UnitY`, and both the parent and child offsets are
set so that the two bodies meet halfway along `default_height`. Each body again
calls `set_geometry()` so the entire chain shares the same dimensions.

### 0c. Viewer plumbing

`main()` constructs a `dart.simulation.World`, adds the skeleton, and wires in a
`Controller`, `PendulumEventHandler`, and `CustomWorldNode`.  The world node is a
subclass of `dart.gui.osg.RealTimeWorldNode`; overriding `customPreStep()` lets
us call `controller.update()` before every physics step.  The event handler maps
keyboard events onto controller actions.  Finally, `viewer.run()` starts the
interactive simulation.

## Lesson 1 – Change shapes and apply forces

The controller stores the same state as the C++ sample:

- `force_countdown`: array of integers, one per DOF/body. A positive entry means
  “apply a force for N more frames.”
- `positive_sign`: toggled with `-` to flip the direction of torques/forces.
- `apply_body_force`: toggled with `f` to switch between joint torques and body
  forces.
- `body_force_visuals`: `(ArrowShape, VisualAspect)` pairs used to visualize
  external pushes.

### 1a. Reset visuals

At the start of every update the controller loops over the body nodes, restores
blue coloring on the first two shapes (joint capsule + body box), and hides any
arrow visuals.  This reproduces Lesson 1a’s instructions.

### 1b. Apply joint torques

If `apply_body_force` is `False`, the controller iterates over the DOFs and looks
for positive countdown entries.  Each active DOF receives `default_torque` (or
its negative) through `DegreeOfFreedom.setForce()`.  The child body is colored
red (`getShapeNode(0)`) so it is obvious which joint is being actuated.  Keys `1`
through `0` map to DOFs `0`–`9`, matching the original tutorial.

### 1c. Apply body forces

If `apply_body_force` is `True`, the same countdown drives calls to
`BodyNode.addExtForce()`.  Each body owns a `SimpleFrame` with an `ArrowShape`. We
update the arrow’s head/tail vectors and call `visual.show()` while the force is
active, then hide it when the countdown hits zero. This matches Lesson 1c’s
“attach an arrow when applying an external force” requirement.

## Lesson 2 – Change implicit spring/damper settings

The python tutorial exposes the same hotkeys as the C++ version:

- `q` / `a`: increase/decrease every DOF’s rest position by 10°.  After the loop,
the controller manually resets the X/Z axes of the root ball joint to zero so
only the middle axis curls.
- `w` / `s`: adjust spring stiffness in increments of 10, clamped at zero.
- `e` / `d`: adjust damping in increments of 1, again clamped at zero.

Each helper uses dartpy’s `DegreeOfFreedom` accessors (`getRestPosition()`,
`setSpringStiffness()`, etc.), so the code looks different but the behavior is
identical to Lesson 2 in the C++ tutorial.

## Lesson 3 – Add and remove constraints

The final lesson adds a `dart.constraint.BallJointConstraint` between the world
and the last body.  `Controller.add_constraint()` computes the attachment point
by transforming `[0, 0, default_height]` through the tip’s body transform and
passes it to the constraint constructor.  `Controller.remove_constraint()` calls
`World.getConstraintSolver().removeConstraint(...)` and clears the stored pointer.
The event handler maps the `r` key to toggle these calls so you can switch
between free-swinging and constrained modes while applying forces.

## Keyboard reference

| Key | Action |
| --- | ------ |
| `space` | Toggle simulation |
| `p` | Replay |
| `1`–`0` | Apply torques or body forces to links 1–10 |
| `-` | Flip the torque/force direction |
| `f` | Switch between joint torques and body forces |
| `q` / `a` | Increase / decrease rest positions |
| `w` / `s` | Increase / decrease stiffness |
| `e` / `d` | Increase / decrease damping |
| `r` | Attach / detach the constraint on the chain tip |

Follow the lessons sequentially just like the C++ guide: build the passive chain,
exercise the torque/body-force controls, experiment with springs/dampers, and
finally attach/detach the constraint. Because everything is done through dartpy
you can script additional experiments—logging applied forces, fading colors
based on torque magnitude, etc.—with only a few lines of Python.
