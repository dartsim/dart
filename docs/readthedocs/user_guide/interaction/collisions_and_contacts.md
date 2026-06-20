# Collisions & contacts

When two shapes overlap, DART generates **contacts** and the contact solver
resolves them so bodies push apart instead of passing through each other. This
page explains how collision works and how to query it.

## Shapes make contact possible

Only bodies with a **collision shape** take part in contact. A body with mass but
no shape still moves under gravity and forces, but nothing can touch it. Attach a
shape when you want a body to collide:

```python
import dartpy as dart
import numpy as np

ground.set_collision_shape(dart.CollisionShape.box(np.array([2.0, 2.0, 0.05])))
ball.set_collision_shape(dart.CollisionShape.sphere(0.15))
```

Box shapes take **half extents**; spheres take a radius. Keep collision shapes as
simple as your scene allows — primitive shapes are faster and more robust than
dense meshes.

## How contact is resolved during a step

On each `world.step()`, DART:

1. **Detects** overlapping shape pairs (collision detection).
2. **Generates** contact points with positions and normals.
3. **Solves** for contact impulses that prevent interpenetration and apply
   friction, then integrates the result.

Because the whole sequence happens inside `step()`, you normally don't manage it
by hand — you set up shapes and materials, and the world does the rest.

## Querying contacts yourself

You can run collision detection on demand — useful for diagnostics, sensors, or
motion-planning checks — with `world.collide()`, which returns the current
contacts:

```python
contacts = world.collide()
print(f"{len(contacts)} contact(s) this frame")
```

This is a **query**: it reports contacts without integrating impulses or
advancing time, so it pairs naturally with the kinematics-only workflow from
{doc}`../concepts/world`.

## Friction and restitution

Each body's surface material shapes the contact response. **Friction** resists
sliding; **restitution** controls bounce. They are properties of the bodies in
contact:

```python
ball.friction = 0.4
ball.restitution = 0.6     # a lively bounce
ground.friction = 0.9
```

DART combines the materials of the two bodies in a contact to decide the net
behavior, so a bouncy ball on a grippy floor still bounces but does not slide
away.

## Next

Contact behavior also depends on _which solver_ runs it. Pick the right one in
{doc}`choosing solvers <solvers>`.
