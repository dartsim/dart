# Collisions

## Overview
This tutorial will show you how to programmatically create different kinds of
bodies and set initial conditions for Skeletons. It will also demonstrate some
use of DART's Frame Semantics.

The tutorial consists of five Lessons covering the following topics:

- Creating a rigid body
- Creating a soft body
- Setting initial conditions and taking advantage of Frames
- Setting joint spring and damping properties
- Creating a closed kinematic chain

Please reference the source code in [**python/tutorials/02_collisions/main.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/02_collisions/main.py) and [**python/tutorials/02_collisions/main_finished.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/02_collisions/main_finished.py).

## Lesson 1: Creating a rigid body

Start by opening the Skeleton code in [``python/tutorials/02_collisions/main.py``](https://github.com/dartsim/dart/blob/main/python/tutorials/02_collisions/main.py).
Find the function named ``add_rigid_body``. You will notice that this helper
accepts the Skeleton that you want to build, a joint type token (``"free"``,
``"ball"``, or ``"revolute"``), the name of the new BodyNode, the shape type,
and an optional parent BodyNode. Different Joint types in DART are managed by
different classes, so we keep the helper generic by picking the matching
``JointProperties`` class at runtime.

### Lesson 1a: Setting joint properties

The first thing we'll want to do is set the Joint properties for our new body.
Whenever we create a BodyNode, we must also create a parent Joint for it. A
BodyNode needs a parent Joint, even if that BodyNode is the root of the Skeleton,
because we need its parent Joint to describe how it's attached to the world. A
root BodyNode could be attached to the world by any kind of Joint. Most often,
it will be attached by either a FreeJoint (if the body should be completely
free to move with respect to the world) or a WeldJoint (if the body should be
rigidly attached to the world, unable to move at all), but *any* Joint type
is permissible.

Joint properties are managed in nested classes inside each joint type. For
example, ``dart.dynamics.RevoluteJointProperties`` contains the data we need for
revolute joints, and ``dart.dynamics.BallJointProperties`` does the same for ball
joints. We construct whichever properties object matches the requested joint type
and fill in the shared fields:

```python
if joint_type == "revolute":
    joint_prop = dart.dynamics.RevoluteJointProperties()
    joint_prop.mAxis = [0.0, 1.0, 0.0]
elif joint_type == "ball":
    joint_prop = dart.dynamics.BallJointProperties()
else:
    joint_prop = dart.dynamics.FreeJointProperties()
joint_prop.mName = f"{name}_joint"
```

Next we'll want to deal with offsetting the new BodyNode from its parent BodyNode.
We can use the following to check if there is a parent BodyNode:

```python
if parent is not None:
    # TODO: offset the child from its parent
```

Inside the branch, we'll want to create the offset between bodies:

```python
parent_tf = dart.math.Isometry3()
parent_tf.set_translation([0.0, 0.0, default_shape_height / 2.0])
child_tf = dart.math.Isometry3()
child_tf.set_translation([0.0, 0.0, -default_shape_height / 2.0])
```

We can then offset the parent and child BodyNodes of this Joint using this
transform:

```python
joint_prop.mT_ParentBodyToJoint = parent_tf
joint_prop.mT_ChildBodyToJoint = child_tf
```

Remember that all of that code should go inside the ``if parent`` condition.
We do not want to create this offset for root BodyNodes, because later on we
will rely on the assumption that the root Joint origin is lined up with the
root BodyNode origin.

### Lesson 1b: Create a Joint and BodyNode pair

A single function is used to simultaneously create a new Joint and its child
BodyNode. It's important to note that a Joint cannot be created without a
child BodyNode to accompany it, and a BodyNode cannot be created with parent
Joint to attach it to something. A parent Joint without a child BodyNode or
vice-versa would be non-physical and nonsensical, so we don't allow it.

Use the following to create a new Joint & BodyNode, and obtain a pointer to
that new BodyNode:

```python
body_prop = dart.dynamics.BodyNodeProperties(
    dart.dynamics.BodyNodeAspectProperties(name)
)
joint, body = skeleton.createBallJointAndBodyNodePair(
    parent, joint_prop, body_prop
)
```

The python bindings provide dedicated creation helpers for each joint type. When
we ask to create a root BodyNode, we pass ``None`` as the parent pointer. The
``joint`` return value is unused inside this helper, so we only keep the body
reference.

### Lesson 1c: Make a shape for the body

We'll allow the user to specify three types of Shapes: ``"box"``,
``"cylinder"``, and ``"ellipsoid"``.

```python
if shape_type == "box":
    dims = [default_shape_width, default_shape_width, default_shape_height]
    shape = dart.dynamics.BoxShape(dims)
elif shape_type == "cylinder":
    shape = dart.dynamics.CylinderShape(
        default_shape_width / 2.0, default_shape_height
    )
else:
    dims = default_shape_height * np.ones(3)
    shape = dart.dynamics.EllipsoidShape(dims)
```

Now we want to add this shape as a visualization **and** collision shape for
the BodyNode:

```python
shape_node = body.createShapeNode(shape)
visual = shape_node.createVisualAspect()
visual.setColor([0.8, 0.8, 0.8, 1.0])
shape_node.createCollisionAspect()
shape_node.createDynamicsAspect().setRestitutionCoeff(default_restitution)
```

### Lesson 1d: Set up the inertia properties for the body

For the simulations to be physically accurate, it's important for the inertia
properties of the body to match up with the geometric properties of the shape.
We can create an ``Inertia`` object and set its values based on the shape's
geometry, then give that ``Inertia`` to the BodyNode.

```python
inertia = dart.dynamics.Inertia()
mass = default_shape_density * shape.getVolume()
inertia.setMass(mass)
inertia.setMoment(shape.computeInertia(mass))
body.setInertia(inertia)
```

### Lesson 1e: Set the coefficient of restitution

This is very easily done with the following function:

```python
shape_node.getDynamicsAspect().setRestitutionCoeff(default_restitution)
```

### Lesson 1f: Set the damping coefficient

In real life, joints have friction. This pulls energy out of systems over time,
and makes those systems more stable. In our simulation, we'll ignore air
friction, but we'll add friction in the joints between bodies in order to have
better numerical and dynamic stability:

```python
if parent is not None:
    parent_joint = body.getParentJoint()
    for i in range(parent_joint.getNumDofs()):
        parent_joint.getDof(i).setDampingCoefficient(
            default_damping_coefficient
        )
```

If this BodyNode has a parent BodyNode, then we set damping coefficients of its
Joint to a default value.

## Lesson 2: Creating a soft body

At the time of writing, the dartpy bindings do not expose
``SoftBodyNodeHelper`` yet, so the tutorial follows the recommended approximation:
build a chain of light rigid bodies, tint them transparent, and give their parent
joint very soft springs/dampers.

### Lesson 2a: Set the Joint properties

This portion is exactly the same as Lesson 1a because ``add_soft_body`` simply
delegates to ``add_rigid_body``. The helper decides whether to create a box,
cylinder, or sphere based on ``SoftShapeType`` and passes that through to
``add_rigid_body``. This keeps the joint creation logic identical for rigid and
soft segments.

### Lesson 2b: Zero the mass/inertia

After ``add_rigid_body`` returns, ``add_soft_body`` removes almost all of its
inertia so the resulting bodies move like a soft skin:

```python
inertia = dart.dynamics.Inertia()
inertia.setMass(1e-8)
inertia.setMoment(np.eye(3) * 1e-8)
body.setInertia(inertia)
```

### Lesson 2c: Fade the visuals

Soft parts should be visually distinct from rigid parts, so we lower their alpha:

```python
body.setAlpha(0.4)
```

### Lesson 2d: Assemble a pseudo-soft chain

``create_soft_body`` calls ``add_soft_body`` to create a root body, then adds a
couple of ball-jointed links:

```python
soft = dart.dynamics.Skeleton("soft_approx")
body = add_soft_body(soft, "free", "soft_core", SOFT_BOX)
for i in range(2):
    body = add_soft_body(soft, "ball", f"soft_link_{i}", SOFT_ELLIPSOID, body)
    joint = body.getParentJoint()
    for dof_idx in range(joint.getNumDofs()):
        dof = joint.getDof(dof_idx)
        dof.setSpringStiffness(5.0)
        dof.setDampingCoefficient(0.1)
```

Those light springs/dampers approximate the compliance that would normally come
from a true SoftBodyNode.

### Lesson 2e: Create a hybrid body

``create_hybrid_body`` combines the “soft” chain with an attached rigid chunk:

```python
hybrid = dart.dynamics.Skeleton("hybrid")
body = add_soft_body(hybrid, "free", "soft_base", SOFT_ELLIPSOID)
body = add_soft_body(hybrid, "ball", "soft_link", SOFT_BOX, body)
_, rigid = hybrid.createWeldJointAndBodyNodePair(body)
rigid.setName("rigid_box")
```

The welded body receives a green box visual plus a physically meaningful inertia,
so you can see both behaviors in one chain.

## Lesson 3: Setting initial conditions with frames

Lesson 3 explains how to place and launch new objects. ``add_object`` clones a
template skeleton, positions it above the ground, performs a collision check,
and then tosses it at the wall.

### Lesson 3a: Set the starting pose

``CollisionsEventHandler._initial_position`` returns a six-element vector
containing the desired FreeJoint pose. Randomization toggles the lateral offset:

```python
positions = np.zeros(6)
if self.randomize:
    positions[4] = default_spawn_range * self.rng.uniform(-1.0, 1.0)
positions[5] = default_start_height
obj.getJoint(0).setPositions(positions)
```

### Lesson 3b: Give the copy a unique name

The tutorial appends a monotonically increasing suffix so each spawned skeleton
is easy to track:

```python
obj.setName(f"{obj.getName()}_{self.spawn_index}")
self.spawn_index += 1
```

### Lesson 3c: Reject objects that start in collision

Before inserting the skeleton into the world we collide it against the current
scene:

```python
solver = self.world.getConstraintSolver()
detector = solver.getCollisionDetector()
world_group = solver.getCollisionGroup()
new_group = detector.createCollisionGroup(obj)
collision = world_group.collide(new_group, option, result)
if collision:
    return False
```

### Lesson 3d: Create reference frames for the COM

Once the skeleton is considered safe to add, ``_launch_object`` creates two
``SimpleFrame`` instances:

1. ``center`` is a world-attached frame placed at the object’s COM. Its classic
   linear/angular derivatives store the launch velocities.
2. ``ref`` is a child frame that matches the transform of the root BodyNode.

```python
center_tf = dart.math.Isometry3()
center_tf.set_translation(obj.getCOM())
center = dart.dynamics.SimpleFrame(
    dart.dynamics.Frame.World(), "center", center_tf
)
ref = dart.dynamics.SimpleFrame(center, "root_reference")
ref.setRelativeTransform(obj.getBodyNode(0).getTransform(center))
```

### Lesson 3e: Compute the launch velocity

Randomization perturbs the launch angle, linear speed, and angular speed:

```python
angle = default_launch_angle
speed = default_start_v
angular_speed = default_start_w
if self.randomize:
    angle = self.rng.uniform(
        minimum_launch_angle, maximum_launch_angle
    )
    speed = self.rng.uniform(minimum_start_v, maximum_start_v)
    angular_speed = self.rng.uniform(-maximum_start_w, maximum_start_w)

linear = np.array([math.cos(angle), 0.0, math.sin(angle)]) * speed
angular = np.array([0.0, 1.0, 0.0]) * angular_speed
center.setClassicDerivatives(linear, angular)
```

### Lesson 3f: Apply the velocity to the root joint

Finally, the root FreeJoint receives the spatial velocity stored in ``ref``:

```python
obj.getJoint(0).setVelocities(ref.getSpatialVelocity())
```

## Lesson 4: Setting joint spring and damping properties

``setup_ring`` configures the rigid chain so it behaves like a closed ring with
reasonable compliance.

### Lesson 4a: Set the spring and damping coefficients

The first six DOFs correspond to the root FreeJoint, so we skip them when
assigning spring and damping values:

```python
for idx in range(6, ring.getNumDofs()):
    dof = ring.getDof(idx)
    dof.setSpringStiffness(ring_spring_stiffness)
    dof.setDampingCoefficient(ring_damping_coefficient)
```

### Lesson 4b: Set the rest positions of the joints

Each BallJoint needs to curl by the exterior angle of an N-sided polygon. We
convert that desired rotation into BallJoint coordinates:

```python
num_edges = ring.getNumBodyNodes()
angle = 2 * math.pi / num_edges
for i in range(1, ring.getNumJoints()):
    joint = ring.getJoint(i)
    rotation = dart.math.eulerXYZToMatrix([0.0, angle, 0.0])
    rest = dart.dynamics.BallJoint.convertToPositions(rotation)
    for axis in range(3):
        joint.setRestPosition(axis, rest[axis])
```

### Lesson 4c: Initialize the ring at its rest pose

Once the rest positions are stored, the same DOFs are set to those values so
the ring starts in equilibrium:

```python
for idx in range(6, ring.getNumDofs()):
    dof = ring.getDof(idx)
    dof.setPosition(dof.getRestPosition())
```

## Lesson 5: Creating a closed kinematic chain

Closing the chain is the final step. After calling ``setup_ring`` and launching
the rigid ring, ``add_ring`` binds the first and last bodies with a
``dart.constraint.BallJointConstraint``.

```python
head = ring.getBodyNode(0)
tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
offset = np.array([0.0, 0.0, default_shape_height / 2.0])
offset = tail.getWorldTransform().multiply(offset)
constraint = dart.constraint.BallJointConstraint(head, tail, offset)
self.world.getConstraintSolver().addConstraint(constraint)
self.joint_constraints.append(constraint)
```

Constraints are removed when a skeleton is deleted by scanning ``joint_constraints``
and calling ``removeConstraint`` on any entry that references the departing
skeleton.

## Keyboard reference

| Key | Action |
| --- | ------ |
| `1` | Toss a rigid ball |
| `2` | Toss the pseudo-soft body |
| `3` | Toss the hybrid body |
| `4` | Toss a rigid chain |
| `5` | Toss the rigid ring and attach a constraint |
| `d` | Delete the oldest spawned object |
| `r` | Toggle randomization for spawn offsets and launch speeds |

**Tips**

- Let objects settle before tossing a new one, otherwise the collision solver
  can accumulate large impulses.
- Experiment with the ring spring/damping values to see how stiffness affects
  the constrained loop.
- Because everything is implemented in Python, you can easily add new templates
  (e.g., textured boxes or meshes) or collect statistics about impact forces by
  instrumenting `CollisionsEventHandler`.

Working through each lesson gives you the same understanding as the original C++
tutorial, but now the entire workflow—from creating bodies to managing
constraints—lives in dartpy.
