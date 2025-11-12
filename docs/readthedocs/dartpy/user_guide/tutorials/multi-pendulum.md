# Multi-Pendulum (dartpy)

The dartpy version of the multi-pendulum tutorial mirrors the C++ lessons: you
will build a five-link pendulum, learn how to apply joint/body forces, tweak
spring-damper properties, and attach/detach constraints during simulation.

- Starter code: `python/tutorials/multi_pendulum/main.py`
- Finished code: `python/tutorials/multi_pendulum/main_finished.py`
- Run the tutorial: `python python/tutorials/multi_pendulum/main.py`

The python sources follow the same lesson structure used in the
[C++ tutorial](../../../dart/user_guide/tutorials/multi-pendulum.md). Each
section below calls out the functions that must be edited on the dartpy side.

## Lesson 0 – Build and run the passive pendulum

`main()` constructs the pendulum by combining `make_root_body()` and `add_body()`
calls, adds it to a `dart.simulation.World`, and starts a GUI viewer:

```python
pendulum = dart.dynamics.Skeleton("pendulum")
body = make_root_body(pendulum, "body1")
for name in ["body2", "body3", "body4", "body5"]:
    body = add_body(pendulum, body, name)

world = dart.simulation.World()
world.addSkeleton(pendulum)
```

Once this is running you should see the passive chain falling under gravity.

## Lesson 1 – Appearance and forces

`Controller.update()` is the core of Lesson 1. The first block resets each shape
to its default blue color and hides the SimpleFrame arrows that visualise body
forces:

```python
for idx in range(self.pendulum.getNumBodyNodes()):
    body = self.pendulum.getBodyNode(idx)
    for j in range(min(2, body.getNumShapeNodes())):
        body.getShapeNode(j).getVisualAspect().setColor([0.0, 0.0, 1.0, 1.0])
    _, arrow_visual = self.body_force_visuals[idx]
    arrow_visual.hide()
```

Lesson 1b (joint torques) and Lesson 1c (body forces) live in the two branches
of `Controller.update()`. They use the `mForceCountDown` array to keep track of
active forces and either call `DegreeOfFreedom.setForce()` or
`BodyNode.addExtForce()`. The SimpleFrame arrows inside `body_force_visuals`
replace the C++ `ArrowShape` nodes that were attached/detached manually.

## Lesson 2 – Joint properties

`change_rest_position()`, `change_stiffness()`, and `change_damping()` iterate
over every DOF and update the desired property. Pay attention to units—just like
the C++ version we clamp the rest position to ±90° using
`math.radians(90.0)`.

## Lesson 3 – Constraints

`add_constraint()` and `remove_constraint()` show how to access the world’s
constraint solver from dartpy:

```python
tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
location = tip.getTransform().multiply([0.0, 0.0, default_height])
self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
self.world.getConstraintSolver().addConstraint(self.ball_constraint)
```

Key `'r'` toggles these calls via `PendulumEventHandler`.

## Keyboard reference

- `space`: start/stop simulation
- `p`: replay
- `1`–`0`: apply torques/forces to each joint or body
- `-`: flip the torque/body-force direction
- `f`: switch between joint torque mode and body-force mode
- `q/a`: increase/decrease joint rest positions
- `w/s`: increase/decrease stiffness
- `e/d`: increase/decrease damping
- `r`: toggle the constraint on the last link
