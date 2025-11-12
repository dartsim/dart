# 02. Collisions (dartpy)

## Overview

This tutorial recreates the “toss random objects at a wall” demo entirely in
Python.  You will learn how to:

- build rigid, pseudo-soft, and hybrid skeletons using helper functions
- launch objects with configurable velocity and randomization
- perform ad-hoc collision checks before adding a skeleton to the world
- attach and remove constraints at runtime

The code lives in `python/tutorials/02_collisions/`.  Each folder contains a
skeleton version (`main.py`) and a finished reference (`main_finished.py`).
Run any version with:

```bash
pixi run py-tu-collisions
```

`CollisionsEventHandler` owns the state of the demo and exposes methods for
spawning, deleting, and randomizing objects.

## Lesson 1 – Rigid objects

`add_rigid_body()` builds a `BodyNode`/joint pair and assigns geometry,
collision shapes, and dynamics aspects.  The helper supports three joint types
(`free`, `ball`, `revolute`) and three shapes (`box`, `cylinder`, `ellipsoid`).

```python
joint_prop = dart.dynamics.RevoluteJointProperties()
joint_prop.mName = f"{name}_joint"
joint_prop.mAxis = [0.0, 1.0, 0.0]
joint_prop.mT_ParentBodyToJoint.set_translation([0.0, 0.0, default_height])
joint, body = skeleton.createRevoluteJointAndBodyNodePair(
    parent,
    joint_prop,
    dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(name)
    ),
)
```

After the joint is created the tutorial attaches a shape, colors it, and sets up
inertia:

```python
shape = dart.dynamics.BoxShape([
    default_shape_width,
    default_shape_width,
    default_shape_height,
])
shape_node = body.createShapeNode(shape)
shape_node.createVisualAspect().setColor([0.8, 0.8, 0.8, 1.0])
shape_node.createCollisionAspect()
shape_node.createDynamicsAspect().setRestitutionCoeff(default_restitution)
```

The helper also adds a small amount of joint damping whenever a parent is
present, which keeps chains stable when repeatedly tossed into the wall.

`create_ball()`, `create_rigid_chain()`, and `create_rigid_ring()` all use
`add_rigid_body()` to construct reusable blueprints.

## Lesson 2 – Soft and hybrid bodies (python approximation)

dartpy does not yet expose `SoftBodyNodeHelper`, so the tutorial approximates
soft bodies by chaining lightweight rigid links.  The helper `add_soft_body()`
wraps `add_rigid_body()` and then zeros out the inertia and alpha:

```python
def add_soft_body(...):
    body = add_rigid_body(skeleton, joint_type, name, shape_type, parent)
    inertia = dart.dynamics.Inertia()
    inertia.setMass(1e-8)
    inertia.setMoment(np.eye(3) * 1e-8)
    body.setInertia(inertia)
    body.setAlpha(0.4)
    return body
```

`create_soft_body()` uses this helper twice to build a flexible tube, while
`create_hybrid_body()` welds a rigid box to the soft chain with
`createWeldJointAndBodyNodePair`.  The behaviour mimics the original tutorial
closely even though the underlying implementation differs.

## Lesson 3 – Spawning and launching

`CollisionsEventHandler.add_object()` clones one of the blueprint skeletons,
positions it, and launches it toward the wall.  The method performs four steps:

1. **Clone** – `skeleton.clone("unique_name")` ensures each spawned object has a
   unique name.
2. **Pose** – the object’s root joint is positioned slightly above the ground
   and optionally offset along the Y axis when `randomize` is enabled.
3. **Collision check** – the tutorial uses `world.getConstraintSolver()` to grab
   the main collision group and `createCollisionGroup(new_skeleton)` for the
   candidate.  The floor is temporarily removed from the world group before the
   check to avoid false positives.
4. **Launch** – two `SimpleFrame`s (`center` and `root_reference`) mirror the
   reference frames used in the original sample.  Their velocities are set based
   on a random launch angle, and the resulting spatial velocity is copied into
   the skeleton’s root joint.

```python
def _launch_object(self, obj):
    center_tf = dart.math.Isometry3()
    center_tf.set_translation(obj.getCOM())
    center = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "center", center_tf)
    center.setClassicDerivatives(linear_velocity, angular_velocity)
    ref = dart.dynamics.SimpleFrame(center, "root_reference")
    ref.setRelativeTransform(obj.getBodyNode(0).getTransform(center))
    obj.getJoint(0).setVelocities(ref.getSpatialVelocity())
```

Objects are only added to the world if the collision test passes.  Otherwise the
handler prints a warning and discards the clone.

## Lesson 4 – Rings and constraints

`add_ring()` creates a ring of rigid bodies, calls `setup_ring()` to configure
spring/damping values, and then closes the loop with a
`dart.constraint.BallJointConstraint` linking the first and last body nodes.
Constraints are stored in `self.joint_constraints` so they can be removed when a
skeleton is deleted.

```python
def add_ring(self, ring):
    setup_ring(ring)
    if not self.add_object(ring):
        return
    head = ring.getBodyNode(0)
    tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
    offset = tail.getWorldTransform().multiply([0.0, 0.0, default_shape_height / 2.0])
    constraint = dart.constraint.BallJointConstraint(head, tail, offset)
    self.world.getConstraintSolver().addConstraint(constraint)
    self.joint_constraints.append(constraint)
```

When a user presses `d`, the oldest spawned skeleton is removed from the world
and any associated constraints are detached automatically.

## Lesson 5 – User interface summary

| Key | Action |
| --- | ------ |
| `1` | Toss a rigid ball |
| `2` | Toss the pseudo-soft body |
| `3` | Toss the hybrid body |
| `4` | Toss a rigid chain |
| `5` | Toss the rigid ring and attach a constraint |
| `d` | Delete the oldest spawned object |
| `r` | Toggle randomization for spawn offsets and launch speeds |

Let objects settle before spawning a new one to avoid unrealistic impulses from
stacked collisions.  Every aspect of the sample—from geometry creation to
constraint management—is now exposed through dartpy, so you can prototype new
interactions entirely from Python.
