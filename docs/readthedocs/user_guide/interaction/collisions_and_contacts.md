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

## Bounding baked rigid contact storage

Scenes that need hard bounds on the baked rigid candidate and emitted-contact
buffers can set independent limits when constructing the world:

```python
world = dart.World(
    rigid_collision_candidate_pair_capacity=2048,
    rigid_collision_contact_capacity=4096,
)
```

The candidate limit counts overlapping shape-AABB pairs before body and joint
filters. The contact limit counts the aggregate contact points emitted after
filtering and narrow phase. A value of `0` (the default) asks DART to derive a
conservative limit from the design-mode shape layout: every unordered shape
pair is allowed as a candidate, with up to four contacts for a primitive pair
and up to 1000 for a mesh-involving pair. The resolved values are available as
`world.rigid_collision_candidate_pair_capacity` and
`world.rigid_collision_contact_capacity` after the first collision bake.
Explicit contact limits that cannot represent the solver's derived two tangent
rows per contact are rejected when the World is constructed.

The limits lock when simulation mode is entered. Explicit limits reserve
their storage exactly, so a separated scene may become fully dense later
without growing its collision/contact buffers. Automatic limits are rejection
thresholds only: the complete shape-pair envelope grows quadratically with the
shape count, so DART reserves at most a fixed budget (65,536 candidate pairs
and 16,384 contacts) and lets the buffers grow, allocating, between the budget
and the envelope. Set explicit limits when a large scene must step without
allocating. If a limit is exceeded, `enter_simulation_mode()` or the
built-in `step()` fails before velocities, forces, poses, diagnostics, time, or
the frame counter are changed. The two limits are separate on purpose: a
candidate need not collide, while one shape pair may produce a multi-point
manifold.

These limits are construction-time memory policy, not simulated state. Binary
snapshots do not serialize them: loading uses the destination world's limits
and is transactional if the snapshot does not fit. `world.clear()` also keeps
the policy for the rebuilt contents, just as it keeps the world's allocator.
Replay stays in the same world and retains its already locked policy.
Deformable surface contact has a separate, per-deformable-body candidate
boundary; these rigid limits do not cover it.

### Allocation-safe primitive envelope

After a successful bake, with the shape layout unchanged, the AABB candidate
snapshot, aggregate contact buffer, reverse-order dispatch, and result
manifolds reuse reserved storage. The following unordered native primitive
pairs have fixed-size narrow-phase workspaces in either argument order:

| First shape | Allocation-safe partners              |
| ----------- | ------------------------------------- |
| Sphere      | Sphere, Box, Capsule, Cylinder, Plane |
| Box         | Box, Capsule, Plane                   |
| Capsule     | Capsule, Cylinder, Plane              |
| Cylinder    | Cylinder, Plane                       |
| Plane       | Plane (always produces no contact)    |

This includes rotated Box--Box contact, which uses fixed face-clipping and
contact-reduction arrays. Box--Cylinder is intentionally not in the guaranteed
set: some orientations fall back to convex GJK/EPA, whose support-function and
polytope workspaces can allocate. Convex/GJK and mesh paths can also allocate
internal temporary storage even though their emitted contacts remain bounded
by the configured contact limit.

The guarantee applies to the internal baked collision path, not to changing
collision geometry after the bake or to the returned Python/C++ container from
`world.collide()` (that public return value is a copy and may allocate). Other
enabled simulation features must establish their own allocation contract; for
example, this statement does not make a blanket no-allocation promise for a
custom step pipeline or an independently configured deactivation solver.

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
