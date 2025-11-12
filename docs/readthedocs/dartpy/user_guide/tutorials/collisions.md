# Collisions (dartpy)

## Overview

This tutorial walks through the python equivalents of the classic DART collisions
lessons. You will learn how to:

- build rigid, pseudo-soft, and hybrid bodies entirely from code
- configure launch velocities and randomization
- manage collision groups using the world’s constraint solver
- attach and remove joint constraints at runtime

All code lives in `python/tutorials/collisions/`.  The starter file
(`main.py`) mirrors the structure of the legacy tutorial, and the finished version
(`main_finished.py`) contains a complete implementation.  Run the tutorial with:

```bash
pixi run py tu-collisions
```

The viewer binds keyboard events to spawn objects, delete them, and toggle
randomization.  Each lesson below references concrete python snippets from the
starter file.

## Lesson 1 – Rigid bodies

The helper ``add_rigid_body()`` accepts a skeleton, a joint type (`"free"`,
`"ball"`, or `"revolute"`), a name, and a shape type (`"box"`, `"cylinder"`,
`"ellipsoid"`).  It configures joint transforms, attaches a shape node, and sets
inertia based on the volume of the geometry.

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

Shapes are added via ``createShapeNode`` and always receive visual, collision
and dynamics aspects.  The dynamics aspect is used to set the restitution
coefficient so that collisions look lively when objects hit the ground/wall.

`create_ball()`, `create_rigid_chain()`, and `create_rigid_ring()` call
``add_rigid_body()`` repeatedly to build the base library of objects that the
user can spawn.

## Lesson 2 – Soft bodies and hybrids (python approximation)

The original tutorial uses `SoftBodyNodeHelper`.  dartpy does not yet expose the soft
body API, so the python tutorial approximates soft bodies using chains of rigid
links with extremely small masses and `setAlpha(0.4)` to make them semi
transparent.  The helper ``add_soft_body()`` simply calls ``add_rigid_body()``
with the desired shape, zeros out the inertia, and drops the alpha value.

```python
def add_soft_body(...):
    body = add_rigid_body(skeleton, joint_type, name, shape, parent)
    inertia = dart.dynamics.Inertia()
    inertia.setMass(1e-8)
    inertia.setMoment(np.eye(3) * 1e-8)
    body.setInertia(inertia)
    body.setAlpha(0.4)
    return body
```

`create_soft_body()` uses this helper twice to build a flexible chain, and
`create_hybrid_body()` welds a rigid box to the soft chain via
``createWeldJointAndBodyNodePair``.  The overall behaviour mirrors the legacy
lessons even though the internal implementation differs.

## Lesson 3 – Spawning, launching, and randomization

`CollisionsEventHandler.add_object()` clones one of the blueprint skeletons,
positions it above the ground, checks for collisions, and launches it toward the
wall.

1. **Clone and name** – ``skeleton.clone("name")`` copies the template and
   ensures unique names.
2. **Initial pose** – the new skeleton inherits the last domino’s pose; a random
   offset (when ``randomize`` is true) is added along the Y axis.
3. **Collision check** – use ``world.getConstraintSolver().getCollisionGroup()``
   and ``createCollisionGroup(new_object)`` to perform a one-off collision test
   before adding the skeleton to the world.  If the new body would overlap with
   anything (other than the floor), it is discarded and a warning is printed.
4. **Launch velocity** – a pair of `SimpleFrame` objects (``center`` and
   ``root_reference``) mimic the reference frames used in the original tutorial.  The
   code sets linear and angular velocities on the frames, then copies the
   resulting spatial velocity into the root joint.

```python
def _launch_object(self, obj):
    center_tf = dart.math.Isometry3()
    center_tf.set_translation(obj.getCOM())
    center = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "center", center_tf)
    center.setClassicDerivatives(linear, angular)
    ref = dart.dynamics.SimpleFrame(center, "root")
    ref.setRelativeTransform(obj.getBodyNode(0).getTransform(center))
    obj.getJoint(0).setVelocities(ref.getSpatialVelocity())
```

The keyboard handler supports the same hotkeys as the original version: `1`–`5` spawn
various objects, `d` deletes the oldest spawned body, and `r` toggles the
randomizer.

## Lesson 4 – Rigid rings and constraints

`add_ring()` creates a ring by cloning the rigid chain template, calling
``setup_ring()`` to configure spring/damping values, and adding a
`dart.constraint.BallJointConstraint` between the first and last links.  The
constraint is tracked so it can be removed if the user deletes the associated
skeleton.

```python
def add_ring(self, ring):
    setup_ring(ring)
    if not self.add_object(ring):
        return
    head = ring.getBodyNode(0)
    tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
    offset = np.array([0.0, 0.0, default_shape_height / 2.0])
    offset = tail.getWorldTransform().multiply(offset)
    constraint = dart.constraint.BallJointConstraint(head, tail, offset)
    self.world.getConstraintSolver().addConstraint(constraint)
    self.joint_constraints.append(constraint)
```

Deleting a skeleton removes any constraints that reference its body nodes to
keep the solver state clean.

## User interface summary

| Key | Action |
| --- | ------ |
| `1` | Toss a rigid ball |
| `2` | Toss a pseudo-soft body |
| `3` | Toss a hybrid body |
| `4` | Toss a rigid chain |
| `5` | Toss a rigid ring (with constraint) |
| `d` | Delete the oldest user-spawned object |
| `r` | Toggle randomization for spawn height/velocity |

Let each object settle before spawning another; otherwise the collision solver
can report large impulses.  With these lessons complete you now have full
control of object creation, collision filtering, and constraint management from
pure python.
