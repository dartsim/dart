# 01. Multi-Pendulum (dartpy)

## Overview

This tutorial recreates the five-lesson multi-pendulum walkthrough entirely in
Python. By following along you will:

- construct articulated chains with ball/revolute joints
- attach visual, collision, and dynamics aspects to each body
- apply joint torques and body forces from keyboard input
- modify implicit spring / damping properties during runtime
- toggle constraints on and off using dartpy’s constraint solver

All code lives in `python/tutorials/01_multi_pendulum/`.  Every directory
contains a scaffold (`main.py`) and a completed reference solution
(`main_finished.py`).  Run any revision with:

```bash
pixi run py-tu-multi-pendulum
# or
python python/tutorials/01_multi_pendulum/main.py
```

The python implementation mirrors the structure of the original tutorial:

- `make_root_body()` / `add_body()` build the rigid links
- `Controller` stores per-link state, applies forces, and manages constraints
- `PendulumEventHandler` translates keyboard events into controller calls
- `CustomWorldNode` hooks the controller into the osg viewer loop

## Lesson 0: Simulate a passive multi-pendulum

Lesson 0 focuses on skeleton construction.

### 0.1 Create the root body

The root link is attached with a `BallJoint`.  Its helper looks like:

```python
joint_prop = dart.dynamics.BallJointProperties()
joint_prop.mName = f"{name}_joint"
joint_prop.mRestPositions = np.ones(3) * default_rest_position
body_prop = dart.dynamics.BodyNodeProperties(
    dart.dynamics.BodyNodeAspectProperties(name)
)
joint, body = pendulum.createBallJointAndBodyNodePair(
    None,
    joint_prop,
    body_prop,
)
```

`set_geometry(body)` then attaches a `BoxShape`, centers it on the COM, assigns
visual/collision aspects, and sets the mass using the box volume.  This mirrors
`tutorialMultiPendulum.cpp` but uses python-friendly helpers.

### 0.2 Append additional links

Each additional body is connected with a `RevoluteJoint` that rotates about the
Y axis:

```python
joint_prop = dart.dynamics.RevoluteJointProperties()
joint_prop.mAxis = [0.0, 1.0, 0.0]
joint_prop.mT_ParentBodyToJoint.set_translation([0.0, 0.0, default_height])
joint, body = pendulum.createRevoluteJointAndBodyNodePair(
    parent,
    joint_prop,
    body_prop,
)
```

Rest positions, stiffness, and damping are initialized to shared defaults so the
chain behaves predictably.

### 0.3 Launch the viewer

`main()` finishes by creating a `dart.simulation.World`, adding the pendulum,
creating a `Controller`, wiring up the event handler, and running the osg view:

```python
controller = Controller(pendulum, world)
viewer = dart.gui.osg.Viewer()
viewer.addWorldNode(CustomWorldNode(world, controller))
viewer.addEventHandler(PendulumEventHandler(controller))
viewer.run()
```

`CustomWorldNode.customPreStep()` calls `controller.update()` before each frame
so the controller can inject forces.

## Lesson 1: Change shapes and apply forces

### 1a – Reset visuals

Every frame begins with a cleanup pass that restores colors and hides the arrow
visuals used for body forces:

```python
for index in range(self.pendulum.getNumBodyNodes()):
    body = self.pendulum.getBodyNode(index)
    for j in range(min(2, body.getNumShapeNodes())):
        body.getShapeNode(j).getVisualAspect().setColor([0.0, 0.0, 1.0, 1.0])
    _, visual = self.body_force_visuals[index]
    visual.hide()
```

### 1b – Joint torques

`force_countdown` mirrors the `std::vector<int>` from the C++ sample.  When a
user presses keys `1` through `0`, the handler sets an entry to
`default_countdown`.  While the entry is positive the controller applies a
constant torque and highlights the joint’s visualization shape:

```python
if self.force_countdown[i] > 0:
    dof = self.pendulum.getDof(i)
    torque = default_torque if self.positive_sign else -default_torque
    dof.setForce(torque)
    child = dof.getChildBodyNode()
    child.getShapeNode(0).getVisualAspect().setColor([1.0, 0.0, 0.0, 1.0])
    self.force_countdown[i] -= 1
```

### 1c – Body forces

Pressing `f` toggles `apply_body_force`.  In this mode the same countdown array
controls external forces applied with `BodyNode.addExtForce()`.  Each body owns a
`SimpleFrame`/`ArrowShape` pair that makes the force visible:

```python
force = np.array([default_force, 0.0, 0.0])
location = np.array([-default_width / 2.0, 0.0, default_height / 2.0])
if not self.positive_sign:
    force *= -1.0
    location[0] *= -1.0
body.addExtForce(force, location, True, True)
arrow, visual = self.body_force_visuals[i]
arrow.setPositions(*self._arrow_positions())
visual.show()
```

## Lesson 2: Adjust implicit springs

`change_rest_position()`, `change_stiffness()`, and `change_damping()` loop over
the DOFs and update the corresponding property.

- Rest positions (`q`/`a`) move by ±10° and are clamped to ±90°.  The X and Z
  axes of the root ball joint are forced back to zero so only the Y axis curls.
- Stiffness (`w`/`s`) increments/decrements by 10 but never drops below zero.
- Damping (`e`/`d`) adjusts in increments of 1.0, again clamped to zero.

These functions closely follow the logic in the C++ tutorial but use dartpy’s
`DegreeOfFreedom` accessors.

## Lesson 3: Toggle constraints

The final lesson attaches the pendulum tip to the world when you press `r`.  The
controller stores the active `dart.constraint.BallJointConstraint` and adds or
removes it from the world’s solver:

```python
def add_constraint(self):
    tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
    location = tip.getTransform().multiply([0.0, 0.0, default_height])
    self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
    self.world.getConstraintSolver().addConstraint(self.ball_constraint)
```

Removing the constraint simply calls `removeConstraint()` and clears the stored
pointer.  Toggling the constraint while forces are active makes it easy to see
how implicit springs and constraints interact.

## Keyboard reference

| Key | Action |
| --- | ------ |
| `space` | Toggle simulation |
| `p` | Replay |
| `1`–`0` | Apply torques or body forces to links 1–10 |
| `-` | Flip the torque/force direction |
| `f` | Switch between joint torque mode and body-force mode |
| `q` / `a` | Increase / decrease joint rest positions |
| `w` / `s` | Increase / decrease stiffness |
| `e` / `d` | Increase / decrease damping |
| `r` | Attach / detach the ball joint constraint |

Work through the lessons in order: build the passive chain, experiment with the
force modes, tweak the spring parameters, and finally attach/detach the
constraint while applying forces.  Everything you touch uses the dartpy API, so
extending the tutorial with new experiments is only a few lines of Python away.
