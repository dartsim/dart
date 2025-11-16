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

## Lesson 2 – Creating a soft body

Python bindings for DART’s true soft body helpers are still in progress, so this
tutorial implements the suggested approximation: a chain of light rigid bodies
with translucent visuals. `add_soft_body()` simply calls `add_rigid_body()`,
zeros out the mass and inertia, and sets `BodyNode.setAlpha(0.4)`.  The helper
accepts a `SoftShapeType` enumeration that decides whether to build a box,
cylinder, or sphere. `create_soft_body()` uses it to assemble a flexible ring
while `create_hybrid_body()` welds a rigid box onto the soft chain so you can
see the hybrid behavior from Lesson 2f in the C++ doc.

## Lesson 3 – Setting initial conditions with frames

Lesson 3 explains how to place and launch new objects.  `add_object()` clones a
template skeleton, positions it above the ground, and then sets up two
`SimpleFrame`s:

1. `center` is attached to the world and positioned at the object’s COM.  Its
   linear/ angular velocities are filled using either the deterministic defaults
   or the randomized values generated when the user toggles randomization.
2. `root_reference` is parented to `center` and matches the transform of the
   skeleton’s root body.  Calling `ref.getSpatialVelocity()` produces the
   correct twist in world coordinates, which is then passed to
   `obj.getJoint(0).setVelocities(...)`.

These frames mirror the ones used in `tutorialCollisions.cpp`, so the python
version produces the same launch trajectories.

## Lesson 4 – Setting joint spring and damping properties

`setupRing(ring)` iterates over every DOF past the translational DOFs and
assigns the spring/damping coefficients specified in the tutorial.  It also
computes the rest positions required to keep the ring closed.  All of the math
matches the “Lesson 4” section from the C++ document, but it is expressed with
python arrays (`Eigen::Vector3d` equivalents are simple NumPy arrays).

Randomization (`r` key) is also implemented here. The handler draws a random
multiplier in `[−1, 1]` and scales the spawn offset, speed, launch angle, and
angular velocity accordingly. That reproduces the “random toss” mode from the
original tutorial.

## Lesson 5 – Creating a closed kinematic chain

Closing the chain is the final step. After calling `setupRing()` and launching
the rigid ring, the handler computes an anchor position using
`tail.getWorldTransform().multiply([0, 0, default_shape_height/2])` and creates a
`dart.constraint.BallJointConstraint` between the first and last body. The
constraint is stored in `self.joint_constraints` so that deleting the ring later
removes the constraint from the solver (otherwise the solver would keep a
dangling pointer).

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
  the constrainted loop.
- Because everything is implemented in Python, you can easily add new templates
  (e.g., textured boxes or meshes) or collect statistics about impact forces by
  instrumenting `CollisionsEventHandler`.

Working through each lesson gives you the same understanding as the original C++
tutorial, but now the entire workflow—from creating bodies to managing
constraints—lives in dartpy.
