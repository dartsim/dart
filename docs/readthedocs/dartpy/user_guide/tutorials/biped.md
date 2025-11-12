# Biped (dartpy)

## Overview

The biped tutorial demonstrates stabilizing a human model with PD / SPD
controllers, ankle strategies, and external pushes.  The python version mirrors
all conceptual lessons from the original guide but uses the `fullbody1` skeleton
and the dartpy API throughout.

Run the tutorial via:

```bash
pixi run py tu-biped
```

The code lives in `python/tutorials/biped/`.  Each section below references
functions in `main.py` (the starter file) and highlights the edits required to
complete the exercise.

## Lesson 1 – Loading the biped and floor

The tutorial uses two skeletons:

- `fullbody1` – loaded with `dart.utils.SkelParser.readWorld(...)`
- `floor` – created manually with a `WeldJoint` and a thin box shape

`main()` loads the world, extracts the `fullbody1` skeleton, and adjusts a few
DOFs to create the initial pose:

```python
world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
world.setGravity([0.0, -9.81, 0.0])
biped = world.getSkeleton("fullbody1")
biped.getDof("j_pelvis_rot_y").setPosition(-0.2)
# (additional DOF tweaks omitted for brevity)
```

The floor skeleton is welded to the world, assigned a 10×10×0.01 m box shape,
and translated downward so the biped stands on top.

## Lesson 2 – Basic PD controller

`MyWorldNode` stores diagonal `Kp`/`Kd` matrices and a target pose `q_d`.  The
first six rows (floating base) are zeroed out the same way they were in the
original walkthrough.  The method ``customPreStep()`` computes the SPD torques:

```python
q = self.skel.getPositions()
dq = self.skel.getVelocities()
invM = np.linalg.inv(self.skel.getMassMatrix() + self.Kd * self.timestep)
p = -self.Kp @ (q + dq * self.timestep - self.q_d)
d = -self.Kd @ dq
constraint = self.skel.getConstraintForces()
cor_grav = self.skel.getCoriolisAndGravityForces()
ddq = invM @ (-cor_grav + p + d + constraint)
self.torques = p + d + (-self.Kd @ ddq) * self.timestep
```

The computed torques are scaled by ``0.8`` before being applied to the skeleton,
mirroring the “safety margin” implemented in the reference solution.

## Lesson 3 – Ankle strategy

After the SPD block the tutorial adds the ankle strategy used to recover from
small pushes.  The code measures the horizontal offset between the COM and the
left heel’s COP, then applies corrective torques to the ankle joints:

```python
com = self.skel.getCOM()
cop = self.left_heel.getTransform().multiply([0.05, 0.0, 0.0])
offset = com[0] - cop[0]
if 0.0 < offset < 0.1:
    k1, k2, kd = 200.0, 100.0, 10.0
    self.torques[self.left_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
    # (repeat for the other three ankle DOFs)
elif -0.2 < offset < -0.05:
    k1, k2, kd = 2000.0, 100.0, 100.0
    # apply backward-recovery torques
```

The strategy writes into the same ``self.torques`` array computed by the SPD
controller, so the final torques contain both joint-space and task-space
contributions.

## Lesson 4 – External pushes and visualization

`InputHandler` listens for comma and period key presses, which apply pushes along
the ±X directions.  `MyWorldNode` stores the currently active force and visualises it
with an `ArrowShape` attached to a `SimpleFrame`.  Each frame the code updates the
arrow to extend from the spine COM backward along the applied force.  When the
force timer expires the arrow is hidden and the force is reset to zero.

## Lesson 5 – Wheel commands (optional)

The original tutorial eventually adds skateboard wheels driven by velocity
commands.  The dartpy tutorial keeps the API surface identical: ``setWheelCommands``
sets velocity commands for four wheel joints while zeroing out the PD gains for
those rows.  Toggling the wheel speed with `a`/`s` is left as an experiment in
`Controller.changeWheelSpeed()`.

## Keyboard reference

| Key | Effect |
| --- | ------ |
| `space` | Toggle simulation |
| `p` | Replay |
| `.` | Apply a forward push to the torso |
| `,` | Apply a backward push to the torso |
| `a` | Increase skateboard speed |
| `s` | Decrease skateboard speed |

Use the ankle strategy and SPD controller together to keep the biped upright
while experimenting with the external push hotkeys.
