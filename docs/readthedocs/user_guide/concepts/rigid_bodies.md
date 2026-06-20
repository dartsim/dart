# Rigid bodies & shapes

A **rigid body** is a single solid object: a falling box, a ball, a table, a
floor. It is the simplest thing you can simulate in DART and the building block
for everything else. This page covers how to create rigid bodies, give them
shape, and read and write their state.

## Creating a rigid body

Describe the body with a `RigidBodyOptions` value object, then add it to the
world by name. `add_rigid_body` returns a **handle** you keep to interact with
the body later:

```python
import dartpy as dart
import numpy as np

opts = dart.RigidBodyOptions()
opts.mass = 2.0
opts.position = np.array([0.0, 0.0, 1.5])           # initial world position
opts.linear_velocity = np.array([1.0, 0.0, 0.0])    # initial linear velocity
opts.angular_velocity = np.array([0.0, 2.0, 0.0])   # initial spin

box = world.add_rigid_body("falling_box", opts)
```

Names are unique within a world, so `"falling_box"` both identifies the body and
lets you look it up later.

### Static bodies

A body marked **static** has effectively infinite mass and never moves. Ground
planes, walls, and fixtures are static:

```python
ground_opts = dart.RigidBodyOptions()
ground_opts.is_static = True
ground_opts.position = np.array([0.0, 0.0, 0.0])
ground = world.add_rigid_body("ground", ground_opts)
```

## Giving a body a collision shape

A body with no shape has mass and motion but cannot touch anything. Attach a
**collision shape** so it participates in contact. The shape factories take
their natural parameters — boxes take **half extents**, spheres take a radius:

```python
ground.set_collision_shape(dart.CollisionShape.box(np.array([2.0, 2.0, 0.05])))
box.set_collision_shape(dart.CollisionShape.sphere(0.2))
```

Half extents mean the ground box above spans 4 m × 4 m × 0.1 m. Collision shapes
are what {doc}`collision detection <../interaction/collisions_and_contacts>`
tests against.

## Reading and writing state

After each `world.step()`, a body's state is fresh. The handle exposes its pose
and motion in the world frame:

```python
box.transform          # 4x4 homogeneous transform
box.translation        # xyz position
box.linear_velocity    # world-frame linear velocity
box.angular_velocity   # world-frame angular velocity
box.kinetic_energy     # scalar kinetic energy
```

You can also set these directly — for example, to reset a body to a known state
before a new rollout:

```python
box.linear_velocity = np.array([0.0, 0.0, 0.0])
box.angular_velocity = np.array([0.0, 0.0, 0.0])
```

## Surface material

Friction and restitution (bounciness) control how a body behaves on contact.
Tune them per body to get sliding, sticking, or bouncing:

```python
box.friction = 0.85       # higher = grippier
box.restitution = 0.2     # higher = bouncier (0 = no bounce)
```

A low-friction, low-restitution body slides and settles; a high-restitution body
rebounds. These pair with the contact solver described in
{doc}`../interaction/solvers`.

## Next

One body is a start. Connect several with joints and you have a robot — see
{doc}`articulated systems <articulated_systems>`.
