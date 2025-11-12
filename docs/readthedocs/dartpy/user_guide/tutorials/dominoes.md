# Dominoes (dartpy)

This tutorial combines rigid-body spawning with a simple operational-space
controller that uses a robotic arm to push the first domino. The python sources
match the lesson layout from the C++ tutorial while using numpy for the
controllers.

- Starter code: `python/tutorials/dominoes/main.py`
- Finished code: `python/tutorials/dominoes/main_finished.py`
- Run the tutorial: `python python/tutorials/dominoes/main.py`

## Controller setup (Lessons 2 & 3)

`Controller.__init__()` grabs the manipulator skeleton, stores the initial joint
configuration as the PD target, builds a `SimpleFrame` named `target`, and
computes the end-effector offset.

`set_pd_forces()` implements the gravity-compensated PD controller described in
Lesson 2. The SPD/operational-space controller from Lesson 3 lives in
`set_operational_space_forces()`:

```python
J = self.end_effector.getWorldJacobian(self.offset)
pinv_J = J.T @ np.linalg.inv(J @ J.T + 0.0025 * np.eye(6))
...
relative = self.target.getTransform(self.end_effector).rotation()
aa = dart.math.AngleAxis(relative)
angular_error = np.asarray(aa.axis()).reshape(3) * aa.angle()
```

The code mirrors the math from the C++ writeup—only the syntax changes.

## Lesson 1 – Spawning dominoes

`DominoEventHandler.attempt_to_create_domino()` clones the reference skeleton,
computes the new pose relative to the previous domino, and uses the world’s
collision group to check for intersections. Removing the floor from the collision
group before running `collide()` matches the behaviour from the C++ tutorial.

`delete_last_domino()` removes the most recently added skeleton and updates
`total_angle`, allowing the user to undo mistakes before the simulation starts.

## Lesson 4 – External pushes

`DominoEventHandler.update()` advances two timers:

- `force_countdown` triggers `BodyNode.addExtForce()` on the first domino
- `push_countdown` toggles between the PD controller and the operational-space
  controller

The external force is visualised with a shared `ArrowShape` (`body_force_visuals`
in the code) so that the tutorial matches the visual cues shown in the C++
screenshots.

## Running the tutorial

- Before pressing `space`, use `q`, `w`, or `e` to add left/forward/right
  dominoes and `d` to delete the last one.
- After the simulation starts:
  - `f`: apply a disembodied push to the first domino
  - `r`: have the manipulator arm push the domino using the OSC controller
  - `space`: pause/resume, `p`: replay

The dartpy version loads the manipulator from the same URDF file as the C++
example (`dart://sample/urdf/KR5/KR5 sixx R650.urdf`), so you can follow along
with the original tutorial text for additional background.
