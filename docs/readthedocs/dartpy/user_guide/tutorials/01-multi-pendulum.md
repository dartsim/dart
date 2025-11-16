# Multi-Pendulum (dartpy)

## Overview

This dartpy tutorial demonstrates how to interact with DART's dynamics API from Python while
following the same multi-pendulum exercise as the C++ guide. You will:

- Create a basic program to simulate a dynamic system.
- Change the colors of shapes or hide/show visuals at runtime.
- Apply torques at joints or forces on bodies.
- Tune the implicit spring and damping properties of joints.
- Add or remove dynamic constraints.

Use the source in
[python/tutorials/01_multi_pendulum/main.py](https://github.com/dartsim/dart/blob/main/python/tutorials/01_multi_pendulum/main.py)
and
[python/tutorials/01_multi_pendulum/main_finished.py](https://github.com/dartsim/dart/blob/main/python/tutorials/01_multi_pendulum/main_finished.py)
as you follow along. Run the finished demo with:

```bash
pixi run py-tu-multi-pendulum
```

## Lesson 0: Simulate a passive multi-pendulum

This warmup lesson shows how to set up a simulation program in dartpy. The demo is the
same five-link pendulum used in the C++ version, and the skeleton is built
programmatically so you can add or remove bodies later.

The articulated model lives inside a `dart.dynamics.Skeleton`. In `main()` we first create
an empty skeleton named `pendulum`:

```python
pendulum = dart.dynamics.Skeleton("pendulum")
```

Skeletons are composed of `BodyNode` objects connected by `Joint`s. Even the root body is
attached via a `Joint`—here a ball joint. In `make_root_body` we create the joint/body
pair and attach it to the empty skeleton:

```python
joint_prop = dart.dynamics.BallJointProperties()
joint_prop.mName = f"{name}_joint"
joint, body = pendulum.createBallJointAndBodyNodePair(
    None, joint_prop, dart.dynamics.BodyNodeProperties()
)
```

Passing `None` for the parent produces a root body that connects directly to the world.
Additional bodies are added with `add_body`, which appends a revolute joint aligned with
the Y axis so the chain swings in the X–Z plane:

```python
joint_prop = dart.dynamics.RevoluteJointProperties()
joint_prop.mName = f"{name}_joint"
joint_prop.mAxis = [0.0, 1.0, 0.0]
joint_prop.mT_ParentBodyToJoint.set_translation([0.0, 0.0, default_height])
joint, body = pendulum.createRevoluteJointAndBodyNodePair(
    parent, joint_prop, dart.dynamics.BodyNodeProperties()
)
joint.setRestPosition(0, default_rest_position)
joint.setSpringStiffness(0, default_stiffness)
joint.setDampingCoefficient(0, default_damping)
```

Once the skeleton exists, we place it inside a `dart.simulation.World` and use a subclass
of `dart.gui.osg.RealTimeWorldNode` to run our controller before every physics step. The
viewer/hotkey wiring mirrors the C++ `SimWindow` setup:

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

`CustomWorldNode.customPreStep()` calls `controller.update()` every frame so the rest of
the lessons can inspect input state, apply forces, and tweak visualization state.

## Lesson 1: Change shapes and apply forces

We want to visualize what is happening while applying forces. The `Controller` stores a
countdown per degree of freedom (DOF), the sign toggle, the body-force toggle, and cached
arrow visuals. The following sub-lessons mirror `MyWindow::timeStepping()` from the C++
guide.

### Lesson 1a: Reset everything to the default appearance

Each `BodyNode` contains two primary `ShapeNode`s: index 0 for the parent joint and index 1
for the body. At the beginning of `Controller.update()` we reset the colors to blue and
hide the arrow visuals:

```python
def _reset_visuals(self):
    for idx in range(self.pendulum.getNumBodyNodes()):
        body = self.pendulum.getBodyNode(idx)
        for j in range(min(2, body.getNumShapeNodes())):
            visual = body.getShapeNode(j).getVisualAspect()
            visual.setColor([0.0, 0.0, 1.0, 1.0])
        _, arrow_visual = self.body_force_visuals[idx]
        arrow_visual.hide()
```

This ensures that any highlights or arrows added in prior frames disappear before we apply
new forces.

### Lesson 1b: Apply joint torques based on user input

When a digit key is pressed the corresponding entry in `force_countdown` is set to
`default_countdown`. While the entry stays positive we apply a torque to that DOF and turn
the joint capsule red:

```python
def _apply_joint_torques(self):
    torque = default_torque if self.positive_sign else -default_torque
    for i in range(self.pendulum.getNumDofs()):
        if self.force_countdown[i] <= 0:
            continue
        dof = self.pendulum.getDof(i)
        dof.setForce(torque)
        child = dof.getChildBodyNode()
        if child.getNumShapeNodes() > 0:
            joint_visual = child.getShapeNode(0).getVisualAspect()
            joint_visual.setColor([1.0, 0.0, 0.0, 1.0])
        self.force_countdown[i] -= 1
```

This is a direct translation of the C++ snippet: `DegreeOfFreedom.setForce()` pushes into
the generalized force vector, `positive_sign` toggles with `-`, and the zeroth `ShapeNode`
represents the joint geometry.

### Lesson 1c: Apply body forces based on user input

If `apply_body_force` is true we apply external forces instead of torques. For each body we
build the force vector and point of application, call `BodyNode.addExtForce()`, tint the
body red, and show the appropriate arrow:

```python
def _apply_body_forces(self):
    tail, head = self._arrow_positions()
    for i in range(min(self.pendulum.getNumBodyNodes(), len(self.force_countdown))):
        if self.force_countdown[i] <= 0:
            continue
        body = self.pendulum.getBodyNode(i)
        force = np.array([default_force, 0.0, 0.0])
        location = np.array([-default_width / 2.0, 0.0, default_height / 2.0])
        if not self.positive_sign:
            force *= -1.0
            location[0] *= -1.0
        body.addExtForce(force, location, True, True)
        if body.getNumShapeNodes() > 1:
            body_visual = body.getShapeNode(1).getVisualAspect()
            body_visual.setColor([1.0, 0.0, 0.0, 1.0])
        arrow, arrow_visual = self.body_force_visuals[i]
        arrow.setPositions(tail, head)
        arrow_visual.show()
        self.force_countdown[i] -= 1
```

The two `True` flags inform dartpy that both vectors are expressed in the body frame, and
the cached `SimpleFrame` arrow visuals save us from recreating geometry each frame.

## Lesson 2: Set spring and damping properties for joints

DART joints can store implicit springs and dampers that are evaluated with an implicit
integrator for stability. As in the C++ tutorial we use `q/a` to adjust rest positions,
`w/s` to modify stiffness, and `e/d` to modify damping.

### Lesson 2a: Set joint spring rest position

`Controller.change_rest_position(delta)` loops over every DOF, adds `delta`, clamps to
±90°, and zeroes out the extra axes of the root ball joint so the pendulum stays in the
X–Z plane:

```python
def change_rest_position(self, delta):
    limit = math.radians(90.0)
    for i in range(self.pendulum.getNumDofs()):
        dof = self.pendulum.getDof(i)
        q0 = dof.getRestPosition() + delta
        q0 = max(-limit, min(limit, q0))
        dof.setRestPosition(q0)
    self.pendulum.getDof(0).setRestPosition(0.0)
    self.pendulum.getDof(2).setRestPosition(0.0)
```

This closely follows `MyWindow::changeRestPosition` with python syntax.

### Lesson 2b: Set joint spring stiffness

We adjust the implicit spring stiffness with exactly the same loop, clamping at zero to
avoid negative stiffness:

```python
def change_stiffness(self, delta):
    for i in range(self.pendulum.getNumDofs()):
        dof = self.pendulum.getDof(i)
        stiffness = max(0.0, dof.getSpringStiffness() + delta)
        dof.setSpringStiffness(stiffness)
```

### Lesson 2c: Set joint damping

Damping draws energy out of the system and also must stay non-negative. The helper mirrors
the stiffness routine:

```python
def change_damping(self, delta):
    for i in range(self.pendulum.getNumDofs()):
        dof = self.pendulum.getDof(i)
        damping = max(0.0, dof.getDampingCoefficient() + delta)
        dof.setDampingCoefficient(damping)
```

Try combining spring/damping changes with applied torques to reproduce every experiment
from the C++ walkthrough.

## Lesson 3: Add and remove dynamic constraints

Dynamic constraints let you attach `BodyNode`s together (or to the world) at runtime. We
follow the same steps as the C++ tutorial, just expressed with dartpy syntax.

1. **Grab the tip body node.**

   ```python
   tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
   ```

2. **Compute the world-space location at the end of the last link.**

   ```python
   location = tip.getTransform().multiply([0.0, 0.0, default_height])
   ```

3. **Create the ball-joint constraint and register it with the solver.**

   ```python
   self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
   self.world.getConstraintSolver().addConstraint(self.ball_constraint)
   ```

4. **Remove the constraint when `r` is pressed again.**

   ```python
   self.world.getConstraintSolver().removeConstraint(self.ball_constraint)
   self.ball_constraint = None
   ```

With these steps in place the dartpy version behaves exactly like the C++ one: tapping `r`
toggles the attachment, and you can continue applying torques, body forces, springs, and
dampers while the constraint is active.

## Keyboard reference

| Key | Action |
| --- | ------ |
| `space` | Toggle the simulation on/off |
| `p` | Replay from the initial configuration |
| `1`–`0` | Apply torques or body forces to links 1–10 |
| `-` | Flip the torque/force direction |
| `f` | Toggle between joint torques and body forces |
| `q` / `a` | Increase / decrease every DOF’s rest position |
| `w` / `s` | Increase / decrease spring stiffness |
| `e` / `d` | Increase / decrease damping |
| `r` | Attach / detach the constraint on the chain tip |

**Now you are ready to run the demo!** Keep the C++ tutorial handy if you want to compare
line-for-line behavior—the dartpy snippets above follow the same structure while showing
the Python APIs you'll need in your own projects.
