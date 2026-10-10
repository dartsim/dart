# Collisions

## Overview

Create rigid bodies, soft skins, hybrid bodies, articulated chains, and closed
rings. Set initial conditions using DART's frame semantics, then toss the
objects at a wall.

Start with [main.py](https://github.com/dartsim/dart/blob/main/python/tutorials/collisions/main.py)
and compare it with
[main_finished.py](https://github.com/dartsim/dart/blob/main/python/tutorials/collisions/main_finished.py).
Follow the [setup instructions](../tutorials.rst) and run
`pixi run tu-collisions-fi` for the solution. This scene uses Z as the upward
axis. The snippets use `import dartpy as dart`, `import numpy as np`, and
`import math`.

| Key | Action |
| --- | --- |
| Space | Start or pause simulation |
| `1` | Toss a rigid ball |
| `2` | Toss a soft body with a rigid core |
| `3` | Toss a hybrid soft/rigid body |
| `4` | Toss a rigid chain |
| `5` | Toss a closed rigid ring |
| `d` | Delete the oldest tossed object |
| `r` | Toggle randomized launch conditions |

Allow objects to settle before tossing more. Large deformations and stacked
contacts can make this demonstration unstable; if the viewer stops
responding, close it and restart with fewer objects.

## Lesson 1: Creating a rigid body

Find `add_rigid_body(chain, name, shape_type, parent=None)`. It creates a
FreeJoint for a root body and a BallJoint for a child body. The supported
shape names are `"box"`, `"cylinder"`, and `"ellipsoid"`.

### Lesson 1a: Setting joint properties

Every body needs a parent joint, even when it is the skeleton's root. A root
FreeJoint permits unrestricted motion, while a WeldJoint fixes the body to
the world. Other joint types can also serve as roots.

Python exposes property classes such as `FreeJointProperties`,
`BallJointProperties`, `RevoluteJointProperties`, and
`PrismaticJointProperties` under `dart.dynamics`. They combine common joint
properties with the joint type's own parameters, such as a revolute axis.
Create the appropriate object and give the joint a unique name:

```python
properties = (
    dart.dynamics.FreeJointProperties()
    if parent is None
    else dart.dynamics.BallJointProperties()
)
properties.mName = name + "_joint"
```

For a child body, offset the parent and child joint frames by half a body
height so they meet between the body centers:

```python
if parent is not None:
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, default_shape_height / 2.0])
    properties.mT_ParentBodyToJoint = tf
    properties.mT_ChildBodyToJoint = tf.inverse()
```

`Isometry3()` starts as an identity rigid transform. Leave root offsets at
identity because the launch code assumes the root joint and body origins
coincide. DART assigns names if empty or duplicate names are provided, but
explicit unique names make model inspection easier.

### Lesson 1b: Create a Joint and BodyNode pair

A body and its parent joint are created together. The named Python creation
methods replace C++ template arguments and return a two-element tuple:

```python
body_properties = dart.dynamics.BodyNodeProperties(
    dart.dynamics.BodyNodeAspectProperties(name)
)
if parent is None:
    joint, body = chain.createFreeJointAndBodyNodePair(
        parent, properties, body_properties
    )
else:
    joint, body = chain.createBallJointAndBodyNodePair(
        parent, properties, body_properties
    )
```

The skeleton owns the returned joint and body. `None` makes a root;
otherwise, `parent` is the existing body to attach the new body beneath.

### Lesson 1c: Make a shape for the body

Create a box using its three side lengths, a cylinder using radius and
height, or an ellipsoid using its three full axis lengths:

```python
if shape_type == "box":
    shape = dart.dynamics.BoxShape(
        [default_shape_width, default_shape_width, default_shape_height]
    )
elif shape_type == "cylinder":
    shape = dart.dynamics.CylinderShape(
        default_shape_width / 2.0, default_shape_height
    )
elif shape_type == "ellipsoid":
    shape = dart.dynamics.EllipsoidShape(default_shape_height * np.ones(3))
else:
    raise ValueError(f"Unknown rigid shape: {shape_type}")
```

Equal ellipsoid axes produce a sphere. Add the shape to the body and create
its visual, collision, and dynamics aspects:

```python
shape_node = body.createShapeNode(shape)
shape_node.createVisualAspect()
shape_node.createCollisionAspect()
dynamics = shape_node.createDynamicsAspect()
```

A visual-only shape is rendered without participating in collisions. These
objects need all three aspects.

### Lesson 1d: Set up the inertia properties for the body

Match the body's mass and rotational inertia to its geometry:

```python
mass = default_shape_density * shape.getVolume()
inertia = dart.dynamics.Inertia()
inertia.setMass(mass)
inertia.setMoment(shape.computeInertia(mass))
body.setInertia(inertia)
```

The tutorial uses density 1000 kg/m³, height 0.1 m, and width 0.03 m. Geometry
alone does not supply the intended body inertia; assign it explicitly.

### Lesson 1e: Set the restitution coefficient

Restitution controls how much relative normal velocity is restored after an
impact. Zero is inelastic and one is perfectly elastic. Set the object's
coefficient to 0.6 on its shape's dynamics aspect:

```python
dynamics.setRestitutionCoeff(default_restitution)
```

The wall uses 0.2. Try changing these values to compare bouncing and settling.

### Lesson 1f: Set joint damping

Ignore air friction but dissipate energy in joints between consecutive bodies:

```python
if parent is not None:
    joint = body.getParentJoint()
    for i in range(joint.getNumDofs()):
        joint.setDampingCoefficient(i, default_damping_coefficient)
```

The coefficient is 0.001. Do not damp the FreeJoint root here: it represents
motion through the world rather than an internal actuator.

## Lesson 2: Creating a soft body

`add_soft_body` creates a FreeJoint and a `SoftBodyNode`. A soft body's point
masses form a deformable skin around the underlying body frame. The helper
factories provide box, cylinder, and ellipsoid skins.

### Lesson 2a: Set the Joint properties

Create `dart.dynamics.FreeJointProperties()`, assign its name, and use the
same parent/child frame offsets as Lesson 1a when a parent is present.

### Lesson 2b: Set the properties of the soft body

A `SoftBodyNodeUniqueProperties` object stores the skin's point masses,
connections, and softness coefficients. `dart.dynamics.SoftBodyNodeHelper`
creates these properties for common surfaces. The script's `SOFT_BOX`,
`SOFT_CYLINDER`, and `SOFT_ELLIPSOID` constants select a factory.

For a wide, short box, compute its skin mass from surface area, density, and
skin thickness, then use four subdivisions per dimension:

```python
helper = dart.dynamics.SoftBodyNodeHelper
width, height = default_shape_height, 2 * default_shape_width
dims = np.array([width, width, height])
area = 2 * (dims[0] * dims[1] + dims[0] * dims[2] + dims[1] * dims[2])
mass = area * default_shape_density * default_skin_thickness
soft_properties = helper.makeBoxProperties(
    dims, dart.math.Isometry3(), [4, 4, 4], mass
)
```

For a cylinder, include both its side and its two caps. The subdivision
arguments are slices, stacks, and rings:

```python
radius, height = default_shape_height / 2.0, 2 * default_shape_width
area = 2 * math.pi * radius * height + 2 * math.pi * radius**2
mass = area * default_shape_density * default_skin_thickness
soft_properties = helper.makeCylinderProperties(radius, height, 8, 3, 2, mass)
```

For a spherical ellipsoid, use its full axis lengths and surface area:

```python
radius = default_shape_height / 2.0
mass = 4 * math.pi * radius**2 * default_shape_density * default_skin_thickness
soft_properties = helper.makeEllipsoidProperties(2 * radius * np.ones(3), 6, 6, mass)
```

The dimensions and mass must be finite and positive. Cylinder and ellipsoid
slices must be at least 3 and stacks at least 2; cylinder rings must be at
least 1. Invalid values raise `ValueError`. Box subdivisions follow the
native helper's clamping behavior.

Set vertex stiffness, edge stiffness, and damping:

```python
soft_properties.mKv = default_vertex_stiffness  # 1000
soft_properties.mKe = default_edge_stiffness    # 1
soft_properties.mDampCoeff = default_soft_damping  # 5
```

Vertex stiffness attaches the skin to the underlying rigid frame; edge
stiffness connects neighboring skin points. Try changing them separately to
see their different effects.

### Lesson 2c: Create the Joint and Soft Body pair

Combine ordinary body properties with the skin properties, then create the
joint and soft body together:

```python
body_properties = dart.dynamics.SoftBodyNodeProperties(
    dart.dynamics.BodyNodeProperties(dart.dynamics.BodyNodeAspectProperties(name)),
    soft_properties,
)
joint, body = chain.createFreeJointAndSoftBodyNodePair(
    parent, joint_properties, body_properties
)
```

`body` is a `SoftBodyNode`, so it supports ordinary body operations as well
as soft-body inspection such as `getNumPointMasses()`.

### Lesson 2d: Zero out the BodyNode inertia

A soft body has both underlying rigid-body inertia and skin point-mass
inertia. To use only the skin's inertia, make the underlying inertia tiny
rather than exactly zero, which could make the dynamics singular:

```python
inertia = dart.dynamics.Inertia()
inertia.setMoment(1e-8 * np.eye(3))
inertia.setMass(1e-8)
body.setInertia(inertia)
```

### Lesson 2e: Make the shape transparent

Make the skin transparent so rigid portions are visible through it:

```python
body.setAlpha(0.4)
```

A newly created soft body already has a soft-shape visualizer. Adjusting its
alpha to 0.4 distinguishes the soft skin from its rigid core.

### Lesson 2f: Give a hard bone to the SoftBodyNode

In `create_soft_body()`, add a box scaled to 60% of the skin dimensions:

```python
dims = 0.6 * np.array(
    [default_shape_height, default_shape_height, 2 * default_shape_width]
)
box = dart.dynamics.BoxShape(dims)
shape_node = body.createShapeNode(box)
shape_node.createVisualAspect()
shape_node.createCollisionAspect()
shape_node.createDynamicsAspect()
inertia = dart.dynamics.Inertia()
inertia.setMass(default_shape_density * box.getVolume())
inertia.setMoment(box.computeInertia(inertia.getMass()))
body.setInertia(inertia)
```

This supplies the rigid bone's inertia. It remains independent of the skin's
point-mass inertia.

### Lesson 2g: Add a rigid body attached by a WeldJoint

In `create_hybrid_body()`, attach a rigid box to a soft ellipsoid:

```python
joint, rigid_body = hybrid.createWeldJointAndBodyNodePair(soft_body)
rigid_body.setName("rigid box")
box = dart.dynamics.BoxShape(default_shape_height * np.ones(3))
shape_node = rigid_body.createShapeNode(box)
shape_node.createVisualAspect()
shape_node.createCollisionAspect()
shape_node.createDynamicsAspect()
tf = dart.math.Isometry3()
tf.set_translation([default_shape_height / 2.0, 0.0, 0.0])
joint.setTransformFromParentBodyNode(tf)
```

The offset makes the box protrude. Assign its mass and moment of inertia
from `box.getVolume()` and `box.computeInertia(mass)` as in Lesson 1d.

## Lesson 3: Setting initial conditions and taking advantage of Frames

`CollisionsEventHandler.add_object()` places a cloned object, rejects
intersecting spawn positions, and gives it a launch velocity.

### Lesson 3a: Set the starting position for the object

A FreeJoint configuration has six components: the first three describe
rotation using an angle-axis log map and the last three describe
translation. NumPy slicing makes this distinction explicit:

```python
positions = np.zeros(6)
if self.randomize:
    positions[4] = self.rng.uniform(-default_spawn_range, default_spawn_range)
positions[5] = default_start_height
obj.getJoint(0).setPositions(positions)
```

The object starts at height 0.4 m; randomization changes its Y position.
Only the root joint is set because articulated objects have more than six
coordinates in total.

### Lesson 3b: Set the object's name

Give every thrown skeleton a unique name before adding it to the world:

```python
obj.setName(obj.getName() + str(self.skeleton_count))
self.skeleton_count += 1
```

### Lesson 3c: Add the object to the world without collisions

Interpenetrating initial states can generate excessive forces. Test the
proposed object against the world's collision group before adding it:

```python
solver = self.world.getConstraintSolver()
new_group = solver.getCollisionDetector().createCollisionGroup()
new_group.addShapeFramesOf(obj)
option = dart.collision.CollisionOption()
result = dart.collision.CollisionResult()
if solver.getCollisionGroup().collide(new_group, option, result):
    print("The new object spawned in a collision. It will not be added.")
    return False
self.world.addSkeleton(obj)
```

Create an empty group and populate it with `addShapeFramesOf`; this is the
bound Python API. Existing contacts elsewhere in the world do not reject a
spawn because this query only compares the new object against the world.

### Lesson 3d: Creating reference frames

A BodyNode's motion is determined by its generalized coordinates. A
`SimpleFrame` instead lets you set an arbitrary transform and its
kinematic derivatives relative to a parent frame. Place one at the
object's COM:

```python
center_tf = dart.math.Isometry3()
center_tf.set_translation(obj.getCOM())
center = dart.dynamics.SimpleFrame(
    dart.dynamics.Frame.World(), "center", center_tf
)
```

### Lesson 3e: Set the center-of-mass velocity

Use launch angle 45 degrees, speed 3.5 m/s, and angular speed {math}`3\pi` rad/s.
With randomization enabled, choose an angle between 30 and 70 degrees,
speed between 2.5 and 4 m/s, and angular speed between {math}`-6\pi` and {math}`6\pi`:

```python
v = speed * np.array([math.cos(angle), 0.0, math.sin(angle)])
w = np.array([0.0, angular_speed, 0.0])
center.setClassicDerivatives(v, w)
```

These are classical linear and angular velocities relative to the center
frame's parent, the world. Classical and spatial velocity differ; the
FreeJoint needs the latter.

### Lesson 3f: Transfer motion from COM to the root

Create a child frame aligned with the root body, then transfer its spatial
velocity to the FreeJoint:

```python
ref = dart.dynamics.SimpleFrame(center, "root_reference")
ref.setRelativeTransform(obj.getBodyNode(0).getTransform(center))
obj.getJoint(0).setVelocities(ref.getSpatialVelocity())
```

This accounts for the offset between the root and COM, so the rotating
skeleton's center has the intended linear velocity. Keep both frames alive
until this calculation finishes.

## Lesson 4: Setting joint spring and damping properties

`setup_ring()` curls a chain into a polygon and gives its internal joints
spring and damping forces.

### Lesson 4a: Set the spring and damping coefficients

Skip the first six floating-root coordinates so the ring moves freely
through the world:

```python
for i in range(6, ring.getNumDofs()):
    dof = ring.getDof(i)
    dof.setSpringStiffness(ring_spring_stiffness)
    dof.setDampingCoefficient(ring_damping_coefficient)
```

The ring uses stiffness 0.5 and damping 0.05.

### Lesson 4b: Set the rest positions of the joints

A polygon with {math}`n` edges has exterior angle {math}`2\pi/n`. BallJoint positions
are angle-axis coordinates, so use its conversion helper:

```python
angle = 2 * math.pi / ring.getNumBodyNodes()
rotation = dart.math.AngleAxis(angle, [0.0, 1.0, 0.0]).rotation()
rest = dart.dynamics.BallJoint.convertToPositions(rotation)
for i in range(1, ring.getNumJoints()):
    for j in range(3):
        ring.getJoint(i).setRestPosition(j, rest[j])
```

EulerJoint and FreeJoint also provide conversion helpers for their own
coordinate conventions.

### Lesson 4c: Set the Joints to be in their rest positions

Start the internal joints at their spring rest positions:

```python
for i in range(6, ring.getNumDofs()):
    dof = ring.getDof(i)
    dof.setPosition(dof.getRestPosition())
```

## Lesson 5: Create a closed kinematic chain

`CollisionsEventHandler.add_ring()` first configures and successfully spawns
the ring. Then connect its first and last bodies at the tail's endpoint:

```python
head = ring.getBodyNode(0)
tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
offset = tail.getWorldTransform().multiply([0.0, 0.0, default_shape_height / 2.0])
constraint = dart.constraint.BallJointConstraint(head, tail, offset)
self.world.getConstraintSolver().addConstraint(constraint)
self.joint_constraints.append((ring, constraint))
```

The endpoint is expressed in world coordinates. Keep each constraint paired
with its ring. When deleting that skeleton, remove its constraint from the
solver first and remove the saved pair before removing the skeleton. A
rejected spawn must not leave a constraint behind. Now all five object types
are ready to throw at the wall.
