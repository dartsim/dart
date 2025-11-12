# 04. Biped (dartpy)

## Overview

This tutorial ports the biped control exercises to dartpy.  You will:

- load the `fullbody1` SKEL model and create a simple floor
- implement a gravity-compensated SPD controller
- add an ankle strategy to recover from COM excursions
- visualize external pushes applied through keyboard input
- optionally control skateboard wheels via velocity commands

All code lives in `python/tutorials/04_biped/`.  Run it with:

```bash
pixi run py-tu-biped
```

`MyWorldNode` contains the control logic while `InputHandler` listens for key
presses (`.`, `,`, `a`, `s`).

## Lesson 1: Load the scene

`main()` loads `dart://sample/skel/fullbody1.skel`, retrieves the biped
skeleton, and tweaks several DOFs to create the initial pose (pelvis rotation,
thighs, shins, heels, abdomen).  `createFloor()` builds a 10×10×0.01 m box and
attaches it to the world via a `WeldJoint`.  The floor is translated down by one
meter so the biped stands on it when the simulation starts.

## Lesson 2: SPD controller

`MyWorldNode` computes SPD torques every frame.  The gains are diagonal matrices
where the first six rows (floating base) are set to zero while the remaining
rows have aggressive gains (`Kp = 400`, `Kd = 40`).

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

The `0.8` multiplier matches the “safety margin” used in the original tutorial.

## Lesson 3: Ankle strategy

After SPD torques are computed, the tutorial adds ankle torques that depend on
the COM/heel offset.  The feet DOF indices are captured during initialization so
we can write directly into `self.torques`:

```python
com = self.skel.getCOM()
cop = self.left_heel.getTransform().multiply([0.05, 0.0, 0.0])
offset = com[0] - cop[0]
if 0.0 < offset < 0.1:
    k1, k2, kd = 200.0, 100.0, 10.0
    self.torques[self.left_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
    self.torques[self.left_foot[1]] += -k2 * offset + kd * (self.pre_offset - offset)
    # mirror on the right foot
elif -0.2 < offset < -0.05:
    k1, k2, kd = 2000.0, 100.0, 100.0
    # apply backward-recovery torques
```

`self.pre_offset` stores the previous offset so the derivative component can be
computed implicitly.

## Lesson 4: External pushes and visualization

`InputHandler` reacts to `,` and `.` by calling `set_external_force()` with a
vector along ±X.  `MyWorldNode` keeps track of the active force, applies it to
`h_spine`, and visualizes it via an `ArrowShape` attached to a `SimpleFrame`:

```python
spine = self.skel.getBodyNode("h_spine")
spine.addExtForce(self.ext_force)
arrow_head = spine.getTransform().translation()
arrow_tail = arrow_head - self.ext_force / 30.0
self.ext_force_arrow_shape.setPositions(arrow_tail, arrow_head)
```

When the countdown expires the arrow is hidden and the stored force is reset.

## Lesson 5: Wheel commands (optional)

If you want to recreate the skateboard extension, `Controller.setWheelCommands()`
zeroes out the PD gains for the wheel joints and drives them via
`Skeleton.setCommand()`.  `changeWheelSpeed()` increments `self.speed` and prints
the new value so the user can monitor adjustments while holding `a` or `s`.

## Keyboard reference

| Key | Effect |
| --- | ------ |
| `space` | Toggle simulation |
| `p` | Replay |
| `.` | Apply a forward push to the torso |
| `,` | Apply a backward push to the torso |
| `a` | Increase skateboard speed (optional) |
| `s` | Decrease skateboard speed (optional) |

Experiment with different pushes and ankle gains.  Because the tutorial is built
entirely on dartpy, you can modify controller parameters on the fly and script
new perturbation experiments without touching any C++ code.
