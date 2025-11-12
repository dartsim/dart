# 04. Biped (dartpy)

## Overview

This tutorial focuses on stabilizing the `fullbody1` skeleton with incremental
controllers:

- classic PD / SPD tracking for joint-space regulation
- an ankle strategy that reacts to COM offsets
- external pushes visualized with `ArrowShape`s
- (optional) wheel commands that mimic the skateboard extension from the original
  sample

All code lives in `python/tutorials/04_biped/`.  Run the sample with:

```bash
pixi run py-tu-biped
```

`MyWorldNode` encapsulates the control logic while `InputHandler` routes keyboard
input to the node.

## Lesson 1 – Loading the biped and environment

`main()` loads the SKEL file via `dart.utils.SkelParser`, retrieves the biped
skeleton, and tweaks a few DOFs to get into a neutral standing pose:

```python
world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
world.setGravity([0.0, -9.81, 0.0])
biped = world.getSkeleton("fullbody1")
biped.getDof("j_pelvis_rot_y").setPosition(-0.20)
# ... set a few more DOFs for the knees, hips, and ankles ...
```

A small helper builds a 10×10×0.01 m floor using a `WeldJoint`, a `BoxShape`,
and a translation of `-1.0` along the Y axis so the floor sits just below the
character’s feet.

## Lesson 2 – SPD controller

`MyWorldNode` allocates diagonal `Kp`/`Kd` matrices and stores the reference pose
`q_d`.  The first six rows (floating base) are set to zero while the remaining
rows use large gains.  `customPreStep()` computes SPD torques:

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
self.skel.setForces(self.torques * 0.8)
```

Multiplying the final torques by `0.8` prevents numerical spikes when external
forces are introduced later.

## Lesson 3 – Ankle strategy

The ankle strategy nudges the COM back over the support polygon.  The tutorial
computes the difference between the COM’s X coordinate and a point near the left
heel, then distributes corrective torques across the heel and toe joints on both
feet:

```python
com = self.skel.getCOM()
cop = self.left_heel.getTransform().multiply([0.05, 0.0, 0.0])
offset = com[0] - cop[0]
if 0.0 < offset < 0.1:
    k1, k2, kd = 200.0, 100.0, 10.0
    self.torques[self.left_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
    self.torques[self.left_foot[1]] += -k2 * offset + kd * (self.pre_offset - offset)
    # mirror across the right foot ...
elif -0.2 < offset < -0.05:
    k1, k2, kd = 2000.0, 100.0, 100.0
    # apply backward-recovery torques
```

After the adjustment `self.pre_offset` is updated so the derivative term can
react to the next frame’s change in offset.

## Lesson 4 – External pushes and visualization

`InputHandler` listens for comma (`-X push`) and period (`+X push`) key presses.
Each call to `set_external_force()` stores the requested force vector and
duration.  Inside `customPreStep()` the force is applied to the spine via
`BodyNode.addExtForce()` and an `ArrowShape` attached to a `SimpleFrame` is used
for visualization:

```python
spine = self.skel.getBodyNode("h_spine")
spine.addExtForce(self.ext_force)
arrow_head = spine.getTransform().translation()
arrow_tail = arrow_head - self.ext_force / 30.0
self.ext_force_arrow_shape.setPositions(arrow_tail, arrow_head)
```

When the timer expires the arrow is hidden and the stored force is cleared.

## Lesson 5 – Wheel commands (optional)

To mimic the skateboard extension, the tutorial exposes `setWheelCommands()` and
`changeWheelSpeed()`.  Calling `setWheelCommands()` zeroes out the PD gains for
the four wheel joints and sends velocity commands via `Skeleton.setCommand()`.
`changeWheelSpeed()` simply increments `self.speed` and prints the new value so
you can see the adjustments in the console.

## Keyboard reference

| Key | Effect |
| --- | ------ |
| `space` | Toggle simulation |
| `p` | Replay |
| `.` | Apply a forward push to the torso |
| `,` | Apply a backward push to the torso |
| `a` | Increase skateboard speed |
| `s` | Decrease skateboard speed |

Experiment with different gain values, wheel speeds, and push directions.  All
of the behaviour from the original sample is now exposed through dartpy, so you
can prototype new controllers directly in Python.
