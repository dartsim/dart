# Multi Pendulum

## Overview

Build a five-body pendulum, apply joint torques and body forces, visualize
those forces, change joint springs and damping, and attach the pendulum's tip
to the world with a dynamic constraint.

Start with [main.py](https://github.com/dartsim/dart/blob/main/python/tutorials/multi_pendulum/main.py)
and compare your answers with
[main_finished.py](https://github.com/dartsim/dart/blob/main/python/tutorials/multi_pendulum/main_finished.py).
Follow the [setup instructions](../tutorials.rst) and launch the solution with
`pixi run tu-multi-pendulum-fi`. This scene uses Z as the upward direction.
The snippets below use `import dartpy as dart`, `import numpy as np`, and
`import math`, as in the scripts.

| Key | Action |
| --- | --- |
| Space | Start or pause simulation |
| `1`–`9`, `0` | Apply force to the corresponding joint coordinate or body, when present |
| `-` | Reverse the force direction |
| `f` | Switch between joint torque and body force |
| `q` / `a` | Increase / decrease spring rest angle |
| `w` / `s` | Increase / decrease spring stiffness |
| `e` / `d` | Increase / decrease damping |
| `r` | Attach / release the tip constraint |
| `p` | Toggle replay of recorded simulation |

## Lesson 0: Simulate a passive multi-pendulum

An articulated model is a `dart.dynamics.Skeleton`. It contains `BodyNode`
objects connected by joints. Each body has a parent joint, including the root
body whose joint connects it to the world. The pendulum's root uses a
`BallJoint` with three rotational degrees of freedom; the remaining bodies
use `RevoluteJoint` objects with one rotational degree of freedom each.

```python
pendulum = dart.dynamics.Skeleton("pendulum")
properties = dart.dynamics.BallJointProperties()
body_properties = dart.dynamics.BodyNodeProperties(
    dart.dynamics.BodyNodeAspectProperties("body1")
)
joint, body = pendulum.createBallJointAndBodyNodePair(
    None, properties, body_properties
)
```

`None` means there is no parent body. To append a body, pass its parent to
`createRevoluteJointAndBodyNodePair(parent, properties, body_properties)`.
`make_root_body` and `add_body` in the scripts set the geometry, transforms,
and joint parameters. The joints are drawn as spheres or cylinders and the
bodies as boxes. An initial root angle of 120 degrees starts the pendulum
swinging under gravity.

Create a world and add the skeleton:

```python
world = dart.simulation.World()
world.addSkeleton(pendulum)
```

`build_scene()` returns a custom `dart.gui.osg.RealTimeWorldNode` containing
this world, its controller, and its event handler. `main()` adds the node and
handler to a `dart.gui.osg.Viewer`. Override `customPreStep()` to update the
controller before each simulation step. This is where sensors, actuators,
and interactive control belong. `customPostStep()` records the resulting
state using `world.bake()`.

## Lesson 1: Change shapes and apply forces

The controller applies a force for `default_countdown = 200` simulation steps
after a numeric key is pressed. Forces expire automatically. It also resets
the colors and force arrows each step before highlighting active forces.

### Lesson 1a: Reset everything to default appearance

Find the controller's `update` method. The first two shape nodes of each body
represent its joint and its box, in that order. Reset their visual aspects to
blue:

```python
for i in range(pendulum.getNumBodyNodes()):
    body = pendulum.getBodyNode(i)
    shapes = body.getShapeNodes()
    for shape in shapes[:2]:
        shape.getVisualAspect().setColor([0.0, 0.0, 1.0])
```

Create force-arrow shape nodes once and hide their visual aspects when no
force is active. Show them while the force is applied. This gives the same
appearance as adding and removing arrows while keeping the shape-node count
constant and their Python references valid.

### Lesson 1b: Apply joint torques based on user input

In joint-torque mode, each countdown entry refers to a degree of freedom. Get
that degree of freedom and apply a torque of 15 N·m with the selected sign:

```python
dof = pendulum.getDof(i)
dof.setForce(default_torque if positive_sign else -default_torque)
body = self.dof_bodies[i]
body.getShapeNodes()[0].getVisualAspect().setColor([1.0, 0.0, 0.0])
```

The controller caches `dof_bodies` by iterating each body and repeating it
for each coordinate in its parent joint. This maps the selected coordinate
to its child body without requiring another dartpy accessor. The red joint
shape indicates which coordinate is being actuated. DART clears commanded forces after each simulation step, so expired
commands do not persist. The `-` key reverses the sign used by subsequent
updates.

### Lesson 1c: Apply body forces based on user input

In body-force mode, a countdown entry selects a body instead of a degree of
freedom. Use NumPy arrays to describe the force and its application point:

```python
body = pendulum.getBodyNode(i)
force = np.array([default_force, 0.0, 0.0])
location = np.array([-default_width / 2.0, 0.0, default_height / 2.0])
if not positive_sign:
    force = -force
    location[0] = -location[0]
body.addExtForce(force, location, True, True)
body.getShapeNodes()[1].getVisualAspect().setColor([1.0, 0.0, 0.0])
```

The force has magnitude 15 N. Its location is the center of the body's
negative-X face, as if a finger were pushing that face. Reversing the force
also changes the face being pushed. The two `True` arguments specify that
both vectors are expressed in the body's local frame. Color the box red and
show its persistent `dart.dynamics.ArrowShape` to visualize the push.

## Lesson 2: Set spring and damping properties for joints

DART joints support implicit springs and linear damping. The spring force
follows Hooke's law, {math}`\tau_s = -k(q-q_0)`, and the damping force is
{math}`\tau_d = -d\dot q`. Their implicit integration improves numerical stability.
The tutorial starts with zero spring stiffness and damping coefficient 5.

### Lesson 2a: Set joint spring rest position

The `q` and `a` keys call the controller's rest-position adjustment with
increments of {math}`\pm10` degrees. Add the increment to every coordinate's rest
position and clamp the result to {math}`[-\pi/2,\pi/2]`:

```python
for i in range(pendulum.getNumDofs()):
    dof = pendulum.getDof(i)
    rest = np.clip(dof.getRestPosition() + delta, -math.pi / 2, math.pi / 2)
    dof.setRestPosition(float(rest))
pendulum.getDof(0).setRestPosition(0.0)
pendulum.getDof(2).setRestPosition(0.0)
```

The last two lines keep the root BallJoint's other axes at zero so the rest
pose curls in the X-Z plane. Excessively curled rest poses can destabilize
the system.

### Lesson 2b: Set joint spring stiffness

Rest positions have no effect until the spring stiffness is positive. The
`w` and `s` keys adjust each coordinate by 10:

```python
for i in range(pendulum.getNumDofs()):
    dof = pendulum.getDof(i)
    dof.setSpringStiffness(max(0.0, dof.getSpringStiffness() + delta))
```

Clamp stiffness to zero because a negative value would push the joint away
from its rest position and add energy.

### Lesson 2c: Set joint damping

Damping resists motion in proportion to joint velocity. The `e` and `d` keys
adjust the damping coefficient by 1:

```python
for i in range(pendulum.getNumDofs()):
    dof = pendulum.getDof(i)
    dof.setDampingCoefficient(max(0.0, dof.getDampingCoefficient() + delta))
```

A negative coefficient adds energy instead of dissipating it, so clamp it to
zero as well.

## Lesson 3: Add and remove dynamic constraints

The joints in a skeleton form a tree. Dynamic constraints can connect bodies
across that tree, close a loop, or connect a body directly to the world.
Pressing `r` attaches the pendulum's last body to the world at its tip:

```python
tip = pendulum.getBodyNode(pendulum.getNumBodyNodes() - 1)
transform = tip.getTransform()
location = transform.rotation() @ np.array([0.0, 0.0, default_height])
location += transform.translation()
constraint = dart.constraint.BallJointConstraint(tip, location)
world.getConstraintSolver().addConstraint(constraint)
```

The location is expressed in world coordinates. Keep the constraint in a
controller member so the same object can be removed when `r` is pressed again:

```python
world.getConstraintSolver().removeConstraint(constraint)
constraint = None
```

Try changing spring and damping parameters with the tip attached. Use `p` to
replay the recorded poses; Space leaves replay and resumes normal simulation.
Replay reads `world.getRecording().getConfig(frame, skeleton_index)` and sets
the skeleton's positions without running the controller or advancing physics.
