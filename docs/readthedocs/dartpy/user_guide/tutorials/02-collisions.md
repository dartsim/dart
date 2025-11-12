# 02. Collisions (dartpy)

## Overview

This tutorial shows how to create rigid and pseudo-soft bodies, set their initial
conditions, and manage constraints/collisions entirely from Python.  The lessons
mirror the original C++ guide but use dartpy idioms.

You will learn how to:

- construct rigid bodies with different joint types
- approximate soft bodies and hybrid soft/rigid structures
- randomize spawn locations / velocities and perform pre-spawn collision tests
- add and remove constraints (e.g., to close a kinematic chain)

All code resides in `python/tutorials/02_collisions/` with scaffold and finished
versions of each lesson.  Run the tutorial via:

```bash
pixi run py-tu-collisions
```

`CollisionsEventHandler` orchestrates the demo: it spawns objects, deletes them,
and maintains the list of constraints associated with each skeleton.

## Lesson 1: Creating a rigid body

### 1a – Configure joint properties

`add_rigid_body()` is the python analogue of the templated helper from
`tutorialCollisions.cpp`.  It accepts a joint type (`"free"`, `"ball"`, or
`"revolute"`) and builds the corresponding `JointProperties` structure.

```python
if joint_type == "revolute":
    joint_prop = dart.dynamics.RevoluteJointProperties()
    joint_prop.mAxis = [0.0, 1.0, 0.0]
else:
    joint_prop = dart.dynamics.FreeJointProperties()  # or BallJointProperties
joint_prop.mName = f"{name}_joint"
```

If the new body has a parent, the code offsets the joint origin so that the two
bodies meet at the halfway point along `default_shape_height`:

```python
if parent is not None:
    parent_tf = dart.math.Isometry3()
    parent_tf.set_translation([0.0, 0.0, default_shape_height / 2.0])
    joint_prop.mT_ParentBodyToJoint = parent_tf
    child_tf = dart.math.Isometry3()
    child_tf.set_translation([0.0, 0.0, -default_shape_height / 2.0])
    joint_prop.mT_ChildBodyToJoint = child_tf
```

### 1b – Create the joint/body pair

`createFreeJointAndBodyNodePair`, `createBallJointAndBodyNodePair`, and
`createRevoluteJointAndBodyNodePair` are thin wrappers around the templated C++
function.  The helper picks the appropriate method, captures the returned body
pointer, and leaves the joint pointer unused.

### 1c – Construct shapes

The tutorial supports box, cylinder, and ellipsoid shapes.  Each body creates a
`ShapeNode`, adds visual/collision aspects, and sets the mass properties:

```python
shape = dart.dynamics.BoxShape([
    default_shape_width,
    default_shape_width,
    default_shape_height,
])
node = body.createShapeNode(shape)
node.createVisualAspect().setColor([0.8, 0.8, 0.8, 1.0])
node.createCollisionAspect()
node.createDynamicsAspect().setRestitutionCoeff(default_restitution)
```

`setRestitutionCoeff` matches the original tutorial’s instruction to make the
objects “bouncy”.

### 1d – Set inertia and damping

`dart.dynamics.Inertia()` is filled using `shape.getVolume()` so that the mass
matches the geometry.  If the body has a parent, the helper adds a bit of joint
damping (`default_damping_coefficient`) to keep chains stable.

## Lesson 2: Creating a soft body

Official soft-body bindings are not exposed in dartpy yet, so the tutorial uses a
“rigid mesh” approximation.  `add_soft_body()` simply calls `add_rigid_body()`
with an ellipsoid or box, zeros out the inertia, and adjusts the alpha for a
translucent appearance:

```python
body = add_rigid_body(skeleton, joint_type, name, shape_type, parent)
inertia = dart.dynamics.Inertia()
inertia.setMass(1e-8)
inertia.setMoment(np.eye(3) * 1e-8)
body.setInertia(inertia)
body.setAlpha(0.4)
```

`create_soft_body()` uses this helper to build a flexible loop, while
`create_hybrid_body()` attaches a rigid block with `createWeldJointAndBodyNodePair`
so you can see rigid/soft interaction just like in the C++ version.

## Lesson 3: Taking advantage of frames

`add_object()` showcases DART’s frame semantics when setting initial conditions.
It clones the blueprint skeleton, positions it using a relative translation, and
then sets up two `SimpleFrame`s (`center` and `root_reference`) that mirror the
C++ reference frames.  Their velocities determine the initial linear/angular
velocity applied to the root joint:

```python
center_tf = dart.math.Isometry3()
center_tf.set_translation(obj.getCOM())
center = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "center", center_tf)
center.setClassicDerivatives(linear_velocity, angular_velocity)
ref = dart.dynamics.SimpleFrame(center, "root_reference")
ref.setRelativeTransform(obj.getBodyNode(0).getTransform(center))
obj.getJoint(0).setVelocities(ref.getSpatialVelocity())
```

## Lesson 4: Setting joint properties on the fly

Before launching an object, the handler optionally randomizes its spawn position
and velocity.  The randomness mirrors the `std::uniform_real_distribution` logic
from the original tutorial and is toggled with the `r` key.  Stiffness and
rest-position configuration occurs inside `setup_ring(ring)`, matching Lesson 4
instructions from the C++ writeup.

## Lesson 5: Creating a closed kinematic chain

`add_ring()` demonstrates how to close the rigid chain into a loop.  After the
ring bodies are spawned, the handler adds a `dart.constraint.BallJointConstraint`
between the first and last body nodes:

```python
head = ring.getBodyNode(0)
tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
offset = tail.getWorldTransform().multiply([0.0, 0.0, default_shape_height / 2.0])
constraint = dart.constraint.BallJointConstraint(head, tail, offset)
self.world.getConstraintSolver().addConstraint(constraint)
self.joint_constraints.append(constraint)
```

When an object is deleted, any constraints that reference it are removed from the
solver to keep the system consistent.

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

Let each object settle before spawning another to avoid compounding collisions.
With these lessons complete you now have full control over rigid/soft object
creation, collision filtering, and constraints directly from dartpy.
