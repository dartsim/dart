# Multi-Pendulum (dartpy)

## Overview

This tutorial mirrors the classic multi-pendulum exercise entirely in Python.
You will:

- build articulated chains programmatically
- attach shapes, collision proxies, and inertia properties
to each body
- write a controller capable of both joint torques and body forces
- dynamically adjust spring/damper parameters
- toggle constraints at runtime

The code for every lesson lives in `python/tutorials/multi_pendulum/`.  The
``main.py`` file contains blank sections marked with comments, while
``main_finished.py`` shows one possible solution.  To run the tutorial at any
point:

```bash
pixi run py tu-multi-pendulum
# or, if pixi is not available
python python/tutorials/multi_pendulum/main.py
```

Each lesson below describes the relevant functions in detail, quotes the Python
snippets you are expected to fill in, and explains the runtime behaviour you
should observe.

## Lesson 0 – Passive simulation

`main()` creates a five-link pendulum by repeatedly calling
``make_root_body()`` and ``add_body()``.  The helper functions wrap the verbose
setup of joints, geometry, and mass properties so that the lessons can focus on
control logic.

```python
pendulum = dart.dynamics.Skeleton("pendulum")
body = make_root_body(pendulum, "body1")
for index in range(2, 6):
    body = add_body(pendulum, body, f"body{index}")

pendulum.getDof(1).setPosition(math.radians(120))
world = dart.simulation.World()
world.addSkeleton(pendulum)
```

`make_root_body()` constructs a `BallJoint`.  Its ``BodyNode`` receives a box
shape with visual, collision, and dynamics aspects attached via
``createShapeNode``.  `add_body()` appends a `RevoluteJoint` whose axis is the Y
axis, so each link rotates in the sagittal plane.  The transforms mirror the
layout used in the legacy tutorial.

Once the skeleton exists, the rest of `main()` wires the controller into a GUI
viewer:

```python
controller = Controller(pendulum, world)
handler = PendulumEventHandler(controller)
node = CustomWorldNode(world, controller)
viewer = dart.gui.osg.Viewer()
viewer.addWorldNode(node)
viewer.addEventHandler(handler)
viewer.run()
```

``CustomWorldNode.customPreStep()`` calls ``controller.update()`` every frame,
which gives the controller an opportunity to inject forces before DART advances
one time step.  ``PendulumEventHandler`` listens for keyboard events and flips
controller flags.  With Lesson 0 complete you should see a passive chain that
simply swings under gravity.

## Lesson 1 – Appearance and forces

Lesson 1 focuses on `Controller.update()`. The object maintains:

- ``force_countdown`` – a list whose positive entries keep forces alive for a
  configurable number of steps.
- ``positive_sign`` – toggled by the ``-`` key to flip torque direction.
- ``apply_body_force`` – toggled by the ``f`` key to switch between joint and
  body forces.
- ``body_force_visuals`` – a list of `(ArrowShape, VisualAspect)` pairs used to
  display external pushes.

### Lesson 1a – Reset visuals

Every update starts by returning body visuals to their default state:

```python
for index in range(self.pendulum.getNumBodyNodes()):
    body = self.pendulum.getBodyNode(index)
    for j in range(min(2, body.getNumShapeNodes())):
        body.getShapeNode(j).getVisualAspect().setColor([0.0, 0.0, 1.0, 1.0])
    _, visual = self.body_force_visuals[index]
    visual.hide()
```

The tutorial deliberately attaches three visuals during Lesson 1c (joint shape,
body shape, arrow).  Hiding the `SimpleFrame` visual is the simplest way to
remove the arrow without touching the scene graph directly.

### Lesson 1b – Joint torques

When ``apply_body_force`` is ``False`` the controller iterates over the DOFs and
checks the countdown array:

```python
if self.force_countdown[i] > 0:
    dof = self.pendulum.getDof(i)
    torque = default_torque if self.positive_sign else -default_torque
    dof.setForce(torque)
    child = dof.getChildBodyNode()
    child.getShapeNode(0).getVisualAspect().setColor([1.0, 0.0, 0.0, 1.0])
    self.force_countdown[i] -= 1
```

Pressing ``1`` through ``0`` queues torques on consecutive joints.  The first
visual for each body node corresponds to the joint capsule, so recolouring it
makes it obvious which joint is currently actuated.  Holding ``-`` flips
``positive_sign`` and therefore the torque direction.

### Lesson 1c – Body forces and arrows

If ``apply_body_force`` is ``True`` the same countdown values determine whether a
body should receive an external force:

```python
force = np.array([default_force, 0.0, 0.0])
location = np.array([-default_width / 2.0, 0.0, default_height / 2.0])
if not self.positive_sign:
    force *= -1.0
    location[0] *= -1.0
body.addExtForce(force, location, True, True)
```

A `SimpleFrame` holding an `ArrowShape` is attached to each body when the force
is active.  Updating the arrow’s head/tail vectors and calling ``visual.show()``
closely mimics the behaviour of the attach/detach logic used in the legacy GUI.

## Lesson 2 – Implicit spring parameters

Lesson 2 implements ``change_rest_position()``, ``change_stiffness()``, and
``change_damping()``.  Each method iterates over ``self.pendulum.getNumDofs()``
and updates the desired property.  The patterns are intentionally similar so you
only need to learn the dartpy accessors once.

### Lesson 2a – Rest positions

`q` and `a` adjust the rest position for every DOF by ±10 degrees.  To keep the
implicit springs stable we clamp the result to ±90°.  After the loop completes,
DOFs ``0`` and ``2`` (the X/Z axes of the root ball joint) are reset to zero so
the chain curls in a single plane.

### Lesson 2b – Stiffness

`w` and `s` increment or decrement the spring stiffness for each DOF.  The
minimum value is zero, so the code simply computes ``max(0.0, stiffness +
delta)``.  Because stiffness and rest positions interact, experiment with both
sets of hotkeys to feel the effect.

### Lesson 2c – Damping

`e` and `d` update the damping coefficients.  Once again the values cannot go
below zero.  You will notice that lowering damping causes the pendulum to bounce
around dramatically when torques or body forces are applied.

## Lesson 3 – Constraints

The final lesson turns the last pendulum link into a temporary constraint
anchor.  The python controller keeps a reference to the active
``dart.constraint.BallJointConstraint`` and adds/removes it from the world’s
constraint solver when requested.

```python
def add_constraint(self):
    tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
    location = tip.getTransform().multiply([0.0, 0.0, default_height])
    self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
    self.world.getConstraintSolver().addConstraint(self.ball_constraint)
```

Removing the constraint is equally simple:

```python
def remove_constraint(self):
    if self.ball_constraint is None:
        return
    solver = self.world.getConstraintSolver()
    solver.removeConstraint(self.ball_constraint)
    self.ball_constraint = None
```

The `r` key toggles between these states.  Try freezing the final link and then
applying joint torques—it becomes obvious how the implicit springs interact with
the world-space anchor.

## Keyboard reference

| Key | Effect |
| --- | ------ |
| `space` | Toggle simulation |
| `p` | Replay |
| `1`–`0` | Apply torques/forces to individual joints/bodies |
| `-` | Flip the torque/force direction |
| `f` | Switch between joint torques and body forces |
| `q` / `a` | Increase / decrease rest positions |
| `w` / `s` | Increase / decrease stiffness |
| `e` / `d` | Increase / decrease damping |
| `r` | Attach / detach the ball joint constraint |

Spend time experimenting with each control.  The dartpy tutorial contains the
same pedagogical steps as the original guide but every operation—shape
creation, controller logic, arrow visualization, and constraint management—is
performed through Python.
