# 01. Multi-Pendulum (dartpy)

## Overview

This tutorial mirrors the classic multi-pendulum example from the C++ guide, but every
step is implemented with dartpy. As you progress you will:

- Set up a `dart.simulation.World`, create a pendulum skeleton, and drive it from Python.
- Change the colors and visibility of shapes inside each body node.
- Apply joint torques or external body forces interactively.
- Tune implicit spring and damping properties on every degree of freedom (DOF).
- Add or remove dynamic constraints with the dartpy constraint solver.

The reference implementation lives in `python/tutorials/01_multi_pendulum/main_finished.py`.
Run it with:

```bash
pixi run py-tu-multi-pendulum
```

Open `main.py` if you want to fill in the TODOs yourself—the lesson structure and
hotkeys are identical to the C++ tutorial so you can follow along line by line.

## Source layout and runtime helpers

The Python version uses the same conceptual pieces as the C++ sample:

- `make_root_body()` / `add_body()` build the articulated chain.
- `Controller` replaces the `MyWindow` state in the C++ tutorial. It stores the countdown
  timers, applies torques or forces, and keeps track of the optional constraint.
- `PendulumEventHandler` mirrors the GLUT key handler mapping digits, `-`, `f`, `q/a`,
  `w/s`, `e/d`, and `r` to controller methods.
- `CustomWorldNode` plays the role of `SimWindow` by calling `controller.update()` from
  `customPreStep()` before every physics tick.
- `main()` wires everything together, prints the instruction overlay, and calls
  `viewer.run()`.

Keep the C++ tutorial open while reading this document—the paragraphs below follow the
same structure, but the code snippets show the dartpy equivalents.

## Lesson 0: Simulate a passive multi-pendulum

We start by assembling a pendulum with five rigid bodies swinging under gravity. Just like
in the C++ version, everything is built programmatically so you can scale the number of
links or swap in different shapes later.

### Create the skeleton and root body

Inside `main()` we construct an empty skeleton called `pendulum` and immediately attach the
root ball joint/body pair:

```python
pendulum = dart.dynamics.Skeleton("pendulum")
root = make_root_body(pendulum, "body1")
```

`make_root_body()` calls `pendulum.createBallJointAndBodyNodePair(None, joint_prop,
body_prop)`. Passing `None` indicates that this body is the root and should be attached to
the world. The helper populates the `BallJointProperties` (rest positions, stiffness, and
damping) and then uses `set_geometry()` to attach a blue box aligned with the local COM.
It also creates a small ellipsoid so the base joint is easy to identify in the viewer.

### Append additional links

Every additional link is created with `add_body(pendulum, parent, name)`:

```python
joint_prop = dart.dynamics.RevoluteJointProperties()
joint_prop.mAxis = [0.0, 1.0, 0.0]
joint_prop.mT_ParentBodyToJoint.set_translation([0.0, 0.0, default_height])
joint, body = pendulum.createRevoluteJointAndBodyNodePair(parent, joint_prop, body_prop)
```

The revolute joints use the Y axis so the chain swings within the X–Z plane. The helper
also adds a short cylinder to visualize the joint capsule, calls `set_geometry()` to attach
the main body box, and seeds the joint’s rest position, spring stiffness, and damping with
the module-level defaults. Repeating this call four times gives us the five-link pendulum
used throughout the tutorial.

### Hook the skeleton into the viewer

With the skeleton assembled we move on to the run loop:

```python
world = dart.simulation.World()
world.addSkeleton(pendulum)
controller = Controller(pendulum, world)
handler = PendulumEventHandler(controller)
node = CustomWorldNode(world, controller)
viewer = dart.gui.osg.Viewer()
viewer.addWorldNode(node)
viewer.addEventHandler(handler)
```

`CustomWorldNode` derives from `dart.gui.osg.RealTimeWorldNode`; overriding
`customPreStep()` lets the controller update forces and visuals before each physics step.
The viewer prints hotkeys exactly like the C++ `MyWindow` class, adds a grid attachment,
positions the camera, and starts the GUI loop via `viewer.run()`.

## Lesson 1: Change shapes and applying forces

We now want to apply forces during simulation and visualize what is happening. The Python
controller stores the same state as the C++ tutorial: force countdowns per DOF, a sign bit
that toggles with the minus key, a flag that switches between joint torques and body
forces, and cached arrow visuals for external forces.

### Lesson 1a: Reset everything to default appearance

At the top of `Controller.update()` we reset the appearance so each frame starts from
neutral colors and no arrows:

```python
for idx in range(self.pendulum.getNumBodyNodes()):
    body = self.pendulum.getBodyNode(idx)
    for j in range(min(2, body.getNumShapeNodes())):
        body.getShapeNode(j).getVisualAspect().setColor([0, 0, 1, 1])
    _, arrow_visual = self.body_force_visuals[idx]
    arrow_visual.hide()
```

This mirrors the instructions from Lesson 1a of the C++ guide: the first shape node
corresponds to the joint, the second corresponds to the body, and any attached arrow should
be removed before we process new input.

### Lesson 1b: Apply joint torques based on user input

`PendulumEventHandler` maps the digit keys to DOF indices. When a digit is pressed the
controller sets the corresponding `force_countdown` entry to `default_countdown`. While the
countdown is positive we apply a torque and color the associated joint red:

```python
torque = default_torque if self.positive_sign else -default_torque
for i in range(self.pendulum.getNumDofs()):
    if self.force_countdown[i] <= 0:
        continue
    dof = self.pendulum.getDof(i)
    dof.setForce(torque)
    child = dof.getChildBodyNode()
    child.getShapeNode(0).getVisualAspect().setColor([1, 0, 0, 1])
    self.force_countdown[i] -= 1
```

The logic is identical to the C++ tutorial: `positive_sign` flips with `-`,
`DegreeOfFreedom.setForce()` writes into the generalized force vector, and coloring shape 0
highlights the joint that is being actuated.

### Lesson 1c: Apply body forces based on user input

Toggling `apply_body_force` with `f` switches to the external force path. For each body we
compute the force vector and point of application, call `BodyNode.addExtForce()`, color the
body red, and display the cached arrow:

```python
force = np.array([default_force, 0.0, 0.0])
location = np.array([-default_width / 2.0, 0.0, default_height / 2.0])
if not self.positive_sign:
    force *= -1
    location[0] *= -1
body.addExtForce(force, location, True, True)
arrow, visual = self.body_force_visuals[i]
arrow.setPositions(*self._arrow_positions())
visual.show()
self.force_countdown[i] -= 1
```

As in the C++ code, the two booleans tell DART that both vectors are expressed in the body
frame. The helper `_create_force_visuals()` builds the arrow frames during initialization so
we only need to move and show them here.

## Lesson 2: Set joint spring and damping properties

DART joints can store implicit springs and dampers. Just like the C++ tutorial, we use `q/a`
to change rest positions, `w/s` to adjust stiffness, and `e/d` to adjust damping. The python
helpers call the same APIs (`DegreeOfFreedom::getRestPosition()` → `dof.getRestPosition()`,
etc.), so the code reads almost identically to the C++ snippets.

### Lesson 2a: Set joint spring rest position

`change_rest_position(delta)` loops over every DOF, adds `delta`, clamps to ±90°, and zeroes
the extra axes of the root ball joint so the chain stays in the X–Z plane. Compare this to
`MyWindow::changeRestPosition` in C++—the structure is the same:

```python
for i in range(self.pendulum.getNumDofs()):
    dof = self.pendulum.getDof(i)
    q0 = dof.getRestPosition() + delta
    if abs(q0) > math.radians(90.0):
        q0 = math.copysign(math.radians(90.0), q0)
    dof.setRestPosition(q0)

self.pendulum.getDof(0).setRestPosition(0.0)
self.pendulum.getDof(2).setRestPosition(0.0)
```

Python’s `abs`/`math.copysign` take the place of the `if (std::abs(q0) > limit)` guard in the
C++ version; everything else is a direct translation.

### Lesson 2b: Set joint spring stiffness

Stiffness adjustments mirror the C++ code block from Lesson 2b. We still iterate over every
DOF, bump the spring stiffness, and clamp at zero to prevent instability:

```python
for i in range(self.pendulum.getNumDofs()):
    dof = self.pendulum.getDof(i)
    stiffness = dof.getSpringStiffness() + delta
    if stiffness < 0.0:
        stiffness = 0.0
    dof.setSpringStiffness(stiffness)
```

### Lesson 2c: Set joint damping

The damping helper is copy-paste equivalent to the C++ tutorial’s Lesson 2c snippet—only the
language syntax changes. Again we clamp at zero to avoid negative damping:

```python
for i in range(self.pendulum.getNumDofs()):
    dof = self.pendulum.getDof(i)
    damping = dof.getDampingCoefficient() + delta
    if damping < 0.0:
        damping = 0.0
    dof.setDampingCoefficient(damping)
```

Combining these adjustments with the rest-position controls lets you reproduce every
experiment proposed in the C++ guide (curl the chain, increase stiffness, add damping, and
observe how the implicit forces interact with explicit torques or body forces).

## Lesson 3: Add and remove dynamic constraints

Dynamic constraints let you create closed loops or pin bodies to the world without editing
the skeleton. We follow the exact same steps as the C++ tutorial, but with dartpy syntax.

1. **Grab the tip body node.**

   ```python
   tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
   ```

2. **Compute the world-space location at the very end of the last link.**

   ```python
   location = tip.getTransform().multiply([0.0, 0.0, default_height])
   ```

3. **Build the ball-joint constraint and add it to the world’s solver.**

   ```python
   self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
   self.world.getConstraintSolver().addConstraint(self.ball_constraint)
   ```

   - This mirrors `mBallConstraint = std::make_shared<BallJointConstraint>(tip, location);`
     followed by `mWorld->getConstraintSolver()->addConstraint(mBallConstraint);`.

4. **Remove the constraint when `r` is pressed again.**

   ```python
   self.world.getConstraintSolver().removeConstraint(self.ball_constraint)
   self.ball_constraint = None
   ```

   - Equivalent to `mWorld->getConstraintSolver()->removeConstraint(mBallConstraint);` and
     `mBallConstraint = nullptr;` in the C++ tutorial.

With these steps in place the dartpy tutorial behaves exactly like the C++ version: tapping
`r` toggles the world-space attachment, and you can still apply torques, body forces, springs,
and dampers while the constraint is active.

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

## Going further

Everything above keeps parity with the C++ tutorial. Once you have walked through the
lessons, try extending the python version:

1. **Change the number of links.** Add or remove calls to `add_body()` and watch how the
   keyboard mapping behaves with longer chains.
2. **Record applied forces.** Log the torque/force commands each frame so you can plot them
   against joint positions afterward.
3. **Animate different materials.** Instead of a binary blue/red toggle, color the bodies
   proportionally to the applied torque or force magnitude.
4. **Script hotkeys.** Replace manual key presses with a schedule and run repeatable
   experiments from Python.
5. **Tune world parameters.** Adjust gravity, the integrator time step, or constraint solver
   settings through `dart.simulation.World` and observe how the motion changes.

## Troubleshooting tips

- Make sure the viewer window has focus when pressing keys, otherwise events will not reach
  `PendulumEventHandler`.
- If the simulation explodes, double-check that you did not allow negative stiffness or
  damping values when experimenting with custom code.
- When running over SSH, enable X forwarding (e.g., `ssh -X`) so the viewer window can open.
- If you add more than ten bodies, extend the `digit_keys` map in `PendulumEventHandler`
  so every DOF remains accessible from the keyboard.

Following these steps gives you a dartpy tutorial that closely tracks the C++ walkthrough
while highlighting the python-specific APIs you will use in larger projects.

## Implementation checklist for `main.py`

If you are following the exercises in `main.py`, work through the TODOs using the checklist
below. Each step corresponds to a lesson subsection so you know when you can move on.

1. **Lesson 1a:** Loop over every body, reset the first two visual aspects to blue, and
   hide the arrow visuals. Verify by toggling between lessons—the entire pendulum should be
   blue unless you press a force key.
2. **Lesson 1b:** Inside the “not body force” branch, retrieve each DOF via
   `self.pendulum.getDof(i)`, apply the torque, and tint `shapeNode(0)` red. Watch the viewer
   while holding keys `1`, `2`, or `3` to confirm the highlights cycle correctly.
3. **Lesson 1c:** Implement the external-force path. Remember to adjust both the force vector
   and the application point when the sign flips. Also update the cached arrow so the visual
   matches the applied push.
4. **Lesson 2a:** Modify `change_rest_position` to adjust every DOF and clamp the values. Don’t
   forget to zero the extra axes of the root ball joint so the chain stays planar.
5. **Lesson 2b:** Update `change_stiffness` with the max-at-zero pattern, then test by holding
   `w` while applying torques. You should feel the pendulum snap back harder as stiffness
   increases.
6. **Lesson 2c:** Mirror the stiffness logic for damping. Holding `e` should make the pendulum
   settle quickly after you release the torque keys.
7. **Lesson 3:** Implement `add_constraint` and `remove_constraint`. The easiest way to see the
   effect is to apply a body force near the tip with the constraint active—it should stay
   anchored to the world.
8. **Bonus experiments:** Extend `digit_keys` beyond `0`–`9`, change the default dimensions to
   create thicker links, or add logging to the controller so you can replay force sequences.

Checking off each item ensures your python implementation covers everything described in the
C++ tutorial while matching the viewer experience.

## FAQ

**Why does the tutorial use `numpy` vectors?**  Dartpy accepts Python lists in most places,
 but using numpy arrays makes it easier to perform element-wise operations (sign flips, dot
 products, etc.) when you start experimenting.

**Can I reuse the controller with a different skeleton?**  Yes. As long as the skeleton uses
 the same conventions (ball-jointed root, revolute children, consistent shape ordering), you
 can plug in a different model or even load one from URDF/SDF before instantiating
 `Controller`.

**How do I slow the simulation down for demos?**  Call `world.setTimeStep(new_dt)` before you
 pass the world to `CustomWorldNode`. Doubling the time step speeds things up, halving it
 slows everything down and improves numerical stability.

**What if I want to script the controller instead of using hotkeys?**  Replace the event
 handler with a coroutine that calls `controller.apply_force()` or `controller.toggle_body_force()`
 according to your own schedule, then let `controller.update()` run untouched.

**Does the python version expose everything from C++?**  Yes for this tutorial. The exercise
 touches skeleton construction, joint properties, forces, collision/visual aspects, the GUI
 viewer, and the constraint solver—all of which are covered by dartpy bindings.

## Additional exercises

1. **Closed-loop chain:** After attaching the world constraint, connect the tip back to the
   root using a `WeldJointConstraint` (available in dartpy) to create a loop, then study how
   the solver reacts when you apply torques in the middle of the chain.
2. **Energy monitoring:** Compute kinetic and potential energies each frame using
   `pendulum.getKineticEnergy()` / `getPotentialEnergy()` and plot them to see how damping
   dissipates energy over time.
3. **Controller variants:** Implement PID control in python to hold the pendulum at different
   angles and compare it to the manual hotkey workflow.
4. **Different geometry:** Replace the boxes with `CapsuleShape` or `EllipsoidShape` to mimic
   robot arms, and update `set_geometry()` so the COM offsets still line up.
5. **Multi-window visualization:** Use `dart.gui.osg.Viewer`’s screenshot API to capture frames
   automatically, then stitch them into a video demonstrating each lesson.

Documenting your experiments alongside the code helps future readers understand how the
python bindings match the C++ baseline and encourages contributions when new bindings are
needed.


## References

- C++ tutorial source: `tutorials/tutorial_multi_pendulum/main.cpp`
- Python reference implementation: `python/tutorials/01_multi_pendulum/main_finished.py`
- dartpy API docs: https://dartsim.github.io/dartpy/ (browse `dart.dynamics`, `dart.simulation`, and `dart.gui.osg`).

Keep these handy while experimenting so you can jump between languages whenever you wonder
how a particular method is spelled or which namespace it belongs to.
