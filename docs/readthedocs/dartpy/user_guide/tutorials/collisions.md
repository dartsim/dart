# Collisions (dartpy)

This tutorial demonstrates how to build rigid objects, spawn them into a scene,
and manage collisions/constraints from dartpy. The steps mirror the C++
counterpart, but the python version approximates soft bodies with articulated
chains because SoftBody helpers are not yet exposed in dartpy.

- Starter code: `python/tutorials/collisions/main.py`
- Finished code: `python/tutorials/collisions/main_finished.py`
- Run the tutorial: `python python/tutorials/collisions/main.py`

## Lesson 1 – Rigid bodies

`add_rigid_body()` is the python port of the templated C++ helper. It builds the
joint properties, creates the joint/body node pair (calling
`createFreeJointAndBodyNodePair`, `createBallJointAndBodyNodePair`, etc.), and
attaches the requested geometry:

```python
shape = dart.dynamics.BoxShape(
    [default_shape_width, default_shape_width, default_shape_height]
)
shape_node = body.createShapeNode(shape)
shape_node.createVisualAspect().setColor([0.8, 0.8, 0.8, 1.0])
shape_node.createCollisionAspect()
shape_node.createDynamicsAspect()
```

The inertia, restitution coefficient, and joint damping are applied exactly like
the C++ code.

## Lesson 2 – Soft bodies (approximation)

`add_soft_body()` and `create_soft_body()` build a chain of low-mass articulated
links that behave like a soft skin. They follow the same lesson structure even
though the implementation is an approximation. The key steps are:

- create a new chain with `add_soft_body()`
- zero out the inertia (`Inertia.setMass(1e-8)`) to keep the skin light
- tune the transparency via `BodyNode.setAlpha(0.4)`

`create_hybrid_body()` attaches a rigid `BoxShape` to the soft chain using a
`WeldJoint`, matching Lesson 2g.

## Lesson 3 – Spawning objects

`CollisionsEventHandler.add_object()` handles the entire spawning pipeline:

1. clone the blueprint skeleton (`blueprint.clone("name")`)
2. randomize the starting transform
3. check for collisions before adding it to the world
4. compute the launch velocity using a pair of `SimpleFrame`s

Collision checks rely on `constraint_solver.getCollisionGroup()` /
`getCollisionDetector()` the same way the C++ tutorial does.

## Lesson 4/5 – Rings and constraints

`add_ring()` calls `setup_ring()` to configure the spring/damping values for the
loop, spawns the rigid chain, and then creates a
`dart.constraint.BallJointConstraint` to close the loop. Constraints are tracked
so `remove_skeleton()` can clean them up when a skeleton is deleted.

## Running the demo

The event handler uses the same keyboard layout as the C++ tutorial:

- `1`: rigid ball
- `2`: “soft” body
- `3`: hybrid body
- `4`: rigid chain
- `5`: rigid ring
- `d`: delete the oldest spawned object
- `r`: toggle randomised launch parameters

Let each object settle before launching the next one to avoid unstable
collisions. Use the `pixi run py_tutorial_collisions` target if you prefer
running inside the configured build tree.
