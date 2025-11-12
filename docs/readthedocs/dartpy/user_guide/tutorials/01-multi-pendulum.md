# 01. Multi-Pendulum (dartpy)

## Overview

This tutorial mirrors the classic multi-pendulum exercise entirely in Python.
By the time you finish the lessons you will know how to:

- create and configure articulated skeletons with dartpy
- combine visual, collision, and dynamics aspects on each body
- write controllers that apply torques or external forces in real time
- adjust implicit spring/damper parameters while the simulation is running
- toggle constraints at runtime directly from GUI events

All code lives in `python/tutorials/01_multi_pendulum/`.  Each directory
contains a `main.py` scaffold and a fully implemented `main_finished.py`.  Run
any revision with:

```bash
pixi run py-tu-multi-pendulum
# or, if pixi is unavailable
python python/tutorials/01_multi_pendulum/main.py
```

Throughout the tutorial you will edit the following helper types:

- `make_root_body()` / `add_body()` – create joints, attach shapes, and set
  inertia values for each link in the chain.
- `Controller` – stores the pendulum skeleton, applies forces, and manages
  constraints.
- `PendulumEventHandler` – handles keyboard events and calls into the controller.
- `CustomWorldNode` – plugs the controller into the osg viewer’s pre-step hook.

## Lesson 0 – Build and simulate a passive chain

A five-link pendulum is assembled by chaining a ball joint followed by several
revolute joints.  The helper functions encapsulate the verbose setup code.  In
`main()` the skeleton creation looks like this:

```python
pendulum = dart.dynamics.Skeleton("pendulum")
body = make_root_body(pendulum, "body1")
for index in range(2, 6):
    body = add_body(pendulum, body, f"body{index}")
```

### 0.1 Root joint and body

`make_root_body()` instantiates a `BallJoint` and a body node, then adds both
visual and collision geometry:

```python
joint_prop = dart.dynamics.BallJointProperties()
joint_prop.mName = f"{name}_joint"
joint_prop.mRestPositions = np.ones(3) * default_rest_position
body_prop = dart.dynamics.BodyNodeProperties(
    dart.dynamics.BodyNodeAspectProperties(name)
)
joint, body = pendulum.createBallJointAndBodyNodePair(
    None,
    joint_prop,
    body_prop,
)
```

A helper called `set_geometry()` attaches a `BoxShape`, centers it on the COM,
and assigns matching collision/dynamics aspects.  The box dimensions and mass
properties match the constants defined at the top of the tutorial.

### 0.2 Additional links

`add_body()` appends a `RevoluteJoint` whose axis is the Y axis so each link
sweeps inside the sagittal plane:

```python
joint_prop = dart.dynamics.RevoluteJointProperties()
joint_prop.mAxis = [0.0, 1.0, 0.0]
joint_prop.mT_ParentBodyToJoint.set_translation([0.0, 0.0, default_height])
joint, body = pendulum.createRevoluteJointAndBodyNodePair(
    parent,
    joint_prop,
    body_prop,
)
```

Each joint inherits the default spring and damping coefficients to keep the
chain numerically stable.

### 0.3 World, viewer, and controller plumbing

With the skeleton ready we can wire it into an osg viewer:

```python
world = dart.simulation.World()
world.addSkeleton(pendulum)
controller = Controller(pendulum, world)
handler = PendulumEventHandler(controller)
node = CustomWorldNode(world, controller)
viewer = dart.gui.osg.Viewer()
viewer.addWorldNode(node)
viewer.addEventHandler(handler)
viewer.run()
```

`CustomWorldNode.customPreStep()` calls `controller.update()` before every frame
advance, while `PendulumEventHandler` translates keyboard input into controller
operations.

## Lesson 1 – Appearance and forces

The controller tracks several pieces of state:

- `force_countdown`: list of integers whose positive entries mean “keep applying
  a force for this many frames”.
- `positive_sign`: toggled when the user presses `-`, flipping force direction.
- `apply_body_force`: toggled by `f`, switching between joint torques and body
  forces.
- `body_force_visuals`: one `(ArrowShape, VisualAspect)` pair per body node used
  to display external pushes.

### Lesson 1a – Reset visuals

Every update begins by restoring the default appearance and hiding any arrow
visuals:

```python
for index in range(self.pendulum.getNumBodyNodes()):
    body = self.pendulum.getBodyNode(index)
    for j in range(min(2, body.getNumShapeNodes())):
        body.getShapeNode(j).getVisualAspect().setColor([0.0, 0.0, 1.0, 1.0])
    _, visual = self.body_force_visuals[index]
    visual.hide()
```

Limiting the loop to the first two shape nodes works because each body exposes
one joint capsule and one body box.  The third shape is reserved for the arrow
visual.

### Lesson 1b – Joint torques

When `apply_body_force` is `False`, the controller iterates over the DOFs and
looks for active countdown entries:

```python
if self.force_countdown[i] > 0:
    dof = self.pendulum.getDof(i)
    torque = default_torque if self.positive_sign else -default_torque
    dof.setForce(torque)
    child = dof.getChildBodyNode()
    child.getShapeNode(0).getVisualAspect().setColor([1.0, 0.0, 0.0, 1.0])
    self.force_countdown[i] -= 1
```

Keys `1` through `0` map to DOFs `0` through `9`.  Highlighting
`child.getShapeNode(0)` makes it obvious which joint is currently actuated.

### Lesson 1c – External body forces

If `apply_body_force` is `True`, the same countdown slots determine whether a
body receives an external push.  The arrow visual is shown or hidden alongside
the force:

```python
force = np.array([default_force, 0.0, 0.0])
location = np.array([-default_width / 2.0, 0.0, default_height / 2.0])
if not self.positive_sign:
    force *= -1.0
    location[0] *= -1.0
body = self.pendulum.getBodyNode(i)
body.addExtForce(force, location, True, True)
arrow, visual = self.body_force_visuals[i]
arrow.setPositions(*self._arrow_positions())
visual.show()
self.force_countdown[i] -= 1
```

Each arrow lives inside a `SimpleFrame`.  Updating the frame is cheaper than
adding/removing shapes from the scene graph every frame.

## Lesson 2 – Implicit springs and damping

Hotkeys `q/a`, `w/s`, and `e/d` modify the pendulum’s implicit springs.  Every
function follows the same pattern: iterate over the DOFs, compute a new value,
and clamp it to a valid range before writing it back.

### Lesson 2a – Rest positions

`change_rest_position()` increments all rest positions by ±10 degrees.  To keep
an implicit spring system stable, the values are clamped to ±90 degrees.
Finally, DOFs `0` and `2` (the X/Z axes of the root ball joint) are forced back
to zero so that only the middle axis curls.

### Lesson 2b – Stiffness

`change_stiffness()` adds `delta_stiffness` to each DOF and clamps the result to
be non-negative.  This lets you experiment with softer or stiffer chains while
watching the viewer update in real time.

### Lesson 2c – Damping

`change_damping()` mirrors the stiffness code but operates on the damping
coefficients.  Reducing damping causes the pendulum to oscillate wildly when you
inject torques, whereas large damping values settle almost immediately.

## Lesson 3 – Constraints

The last lesson shows how to attach the pendulum tip to the world with a
`dart.constraint.BallJointConstraint`.  The controller keeps a reference to the
active constraint so it can add or remove it from the solver:

```python
def add_constraint(self):
    tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
    location = tip.getTransform().multiply([0.0, 0.0, default_height])
    self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
    self.world.getConstraintSolver().addConstraint(self.ball_constraint)

def remove_constraint(self):
    if self.ball_constraint is None:
        return
    solver = self.world.getConstraintSolver()
    solver.removeConstraint(self.ball_constraint)
    self.ball_constraint = None
```

Keyboard key `r` toggles between the attached and free states.  Try freezing the
pendulum tip, applying torques, and then unfreezing it to appreciate the effect
on the implicit springs.

## Keyboard reference

| Key | Action |
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

Experiment with different stiffness/damping combinations while toggling the
constraint and injecting forces.  The entire workflow—from building the model
through interacting with it—now lives comfortably inside Python.
