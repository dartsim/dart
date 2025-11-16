# 02. Collisions (dartpy)

## Overview

This tutorial walks through the same five lessons as the C++ “Collisions”
sample, but every example is implemented with dartpy. You will learn how to:

- create rigid bodies programmatically with different joint types
- approximate soft bodies and hybrid chains despite the current lack of python
  bindings for `SoftBodyNodeHelper`
- use DART frames to set initial positions, orientations, and launch velocities
- randomize spawn conditions and perform collision checks before inserting new
  skeletons into the world
- build and tear down constraints to close a kinematic chain

Run the tutorial from the repository root:

```bash
pixi run py-tu-collisions
```

The code lives in `python/tutorials/02_collisions/`. Each directory contains a
`main.py` skeleton with TODO blocks plus a `main_finished.py` reference answer.
`CollisionsEventHandler` orchestrates everything: it owns the templates, spawns
new objects, keeps a FIFO queue for deletions, and tracks any constraints that
must be removed when a skeleton goes away.

## Lesson 1 – Creating a rigid body

Lesson 1 mirrors the `addRigidBody` walkthrough from the C++ tutorial and is
split into the same sub-lessons.

### 1a. Setting joint properties

`add_rigid_body()` accepts the parent skeleton, an optional parent body node, a
friendly name, the joint type (`"free"`, `"ball"`, `"revolute"`), and the shape
type. The python bindings expose the same `JointProperties` classes, so the code
creates the appropriate object and sets the shared parameters:

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

When a parent body node exists we also offset the joint so that the two bodies
meet in the middle.  The helper constructs two `Isometry3` transforms—one for
the parent and one for the child—and assigns them to
`mT_ParentBodyToJoint` / `mT_ChildBodyToJoint`. This is the python equivalent of
the math in the C++ tutorial that keeps bodies evenly spaced along the Z axis.

### 1b. Creating the joint/body pair

In C++ we called `createJointAndBodyNodePair`. In python that function has been
split into more explicit helpers, so `add_rigid_body()` dispatches to the
matching method (`createFreeJointAndBodyNodePair`, etc.) and keeps the returned
body pointer:

```python
joint, body = skeleton.createRevoluteJointAndBodyNodePair(
    parent,
    joint_prop,
    dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(name)
    ),
)
```

The joint pointer is unused because the helper continues to work through the
`BodyNode`.

### 1c. Creating a shape

The tutorial supports three shapes. Each branch constructs the shape, creates a
`ShapeNode`, and attaches visual, collision, and dynamics aspects. The dynamics
aspect sets the restitution coefficient so that newly spawned objects bounce off
the ground and wall.

```python
if shape_type == "box":
    shape = dart.dynamics.BoxShape([...])
elif shape_type == "cylinder":
    shape = dart.dynamics.CylinderShape(default_shape_width / 2.0, default_shape_height)
else:
    shape = dart.dynamics.EllipsoidShape(
        np.ones(3) * default_shape_height
    )
node = body.createShapeNode(shape)
node.createVisualAspect().setColor([0.8, 0.8, 0.8, 1.0])
node.createCollisionAspect()
node.createDynamicsAspect().setRestitutionCoeff(default_restitution)
```

### 1d. Setting inertia and damping

The helper mirrors the original instructions by computing the inertia from the
shape volume and density.  If the body hangs off a parent the code also loops
over the parent joint’s DOFs and assigns `default_damping_coefficient`. That
prevents the rigid chain from exploding when it slams into the wall.

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
