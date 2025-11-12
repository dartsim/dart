# Biped (dartpy)

The dartpy biped tutorial focuses on the core control lessons (PD/SPD tracking,
ankle strategy, and external pushes) while matching the workflow from the C++
guide. Skateboard/velocity-actuator lessons from the original tutorial are not
implemented because the python assets currently target the `fullbody1` model,
but the surrounding structure (event handlers, SPD gains, ankle controller)
remains the same.

- Starter code: `python/tutorials/biped/main.py`
- Finished code: `python/tutorials/biped/main_finished.py`
- Run the tutorial: `python python/tutorials/biped/main.py`

## Lessons 1 & 2 – Loading and posing the biped

`main()` loads `dart://sample/skel/fullbody1.skel`, sets joint limits implicitly
through the SKEL configuration, and then calls a small block of DOF assignments
to create the “ready” pose. This matches the work done by `loadBiped()` and
`setInitialPose()` in the C++ tutorial.

## Lesson 3 – SPD controller

Inside `MyWorldNode.customPreStep()` the first block implements the stable PD
controller using numpy:

```python
invM = np.linalg.inv(self.skel.getMassMatrix() + self.Kd * self.timestep)
p = -self.Kp @ (q + dq * self.timestep - self.q_d)
d = -self.Kd @ dq
ddq = invM @ (
    -self.skel.getCoriolisAndGravityForces() + p + d + constraint_forces
)
self.torques = p + d + (-self.Kd @ ddq) * self.timestep
```

The gains in `self.Kp` / `self.Kd` mirror the C++ values (floating base rows set
to zero, large gains elsewhere).

## Lesson 4 – Ankle strategy

The next block applies the sagittal-plane ankle strategy. It measures the COM
offset relative to the left heel/ankle frames and adjusts the toe/heel DOFs:

```python
offset = com[0] - cop[0]
if 0.0 < offset < 0.1:
    self.torques[self.left_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
    ...
elif -0.2 < offset < -0.05:
    ...
```

This directly ports the logic from the C++ tutorial to python.

## Lesson 5 – External pushes

`InputHandler` listens for `'.'` (forward) and `','` (backward) key presses and
calls `MyWorldNode.set_external_force()`. An `ArrowShape` attached to a
`SimpleFrame` visualises the applied force and is toggled inside
`customPreStep()`. You can change the magnitude/duration through
`DEFAULT_FORCE` and `DEFAULT_COUNTDOWN`.

## Keyboard reference

- `space`: pause/resume simulation
- `p`: replay
- `'.'`: push the torso forward
- `','`: push the torso backward

The SPD and ankle controllers run continuously, so the character will return to
its nominal pose after each perturbation. Follow along with the original tutorial
text for a deeper explanation of each control block—the python file keeps the
same structure to make cross-referencing easy.
