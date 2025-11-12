# 03. Dominoes (dartpy)

## Overview

The domino tutorial mixes runtime skeleton editing with an operational-space
controller that uses a robotic arm to push the first domino.  You will learn
how to:

- clone skeletons, position them relative to existing ones, and check for
  collisions before adding them to the world
- load a URDF manipulator and drive it with a PD + OSC controller
- trigger external forces from keyboard events and visualize those forces

All code lives in `python/tutorials/03_dominoes/`.  Run it with:

```bash
pixi run py-tu-dominoes
```

Before the simulation starts you can insert or remove dominoes.  After the
simulation is running you can knock over the first domino using either a
virtual “finger” or the manipulator arm.

## Lesson 1 – Managing domino skeletons

`DominoEventHandler` owns the list of dominoes.  Pressing `q`, `w`, or `e`
creates a new domino that leans left, forward, or right.  The handler clones the
reference skeleton, positions it relative to the previously placed domino, and
performs a collision test before adding it to the world:

```python
def attempt_to_create_domino(self, angle):
    new_domino = self.first_domino.clone(f"domino_{len(self.dominoes) + 1}")
    last_domino = self.dominoes[-1] if self.dominoes else self.first_domino
    displacement = default_distance * np.array([
        math.cos(self.total_angle),
        math.sin(self.total_angle),
        0.0,
    ])
    pose = last_domino.getPositions().copy()
    pose[3:] += displacement
    pose[2] = self.total_angle + angle
    new_domino.setPositions(pose)
```

To keep the scene valid, the handler uses the world’s `ConstraintSolver` to
perform a one-off collision test between the new domino and everything else in
the world (with the floor temporarily removed from the collision group).  If the
result is collision-free, the domino is added to the world and recorded in
`self.dominoes` along with the requested lean angle.  Pressing `d` deletes the
most recently created domino and rewinds the cumulative angle.

## Lesson 2 – Loading the manipulator and targeting a domino

`create_manipulator()` loads the KR5 URDF via `dart.utils.DartLoader` and moves
its base closer to the domino stack.  The `Controller` class stores a
`SimpleFrame` named `target` so that the OSC portion knows where to push:

```python
self.target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target")
target_tf = dart.math.Isometry3()
target_tf.set_translation([0.0, 0.0, default_domino_height / 2.0])
relative = self.end_effector.getTransform(domino.getBodyNode(0))
target_tf.set_rotation(relative.rotation())
self.target.setTransform(target_tf, domino.getBodyNode(0))
```

The `offset` vector stores the translation from the wrist frame to the contact
point on the end effector so that both controllers apply forces at the same
location.

## Lesson 3 – Controllers

`Controller` exposes two entry points: `set_pd_forces()` and
`set_operational_space_forces()`.  The PD component keeps the manipulator near a
comfortable reference pose:

```python
dt = self.manipulator.getTimeStep()
q = self.manipulator.getPositions().copy()
dq = self.manipulator.getVelocities()
q += dq * dt
q_err = self.q_desired - q
dq_err = -dq
Cg = self.manipulator.getCoriolisAndGravityForces()
M = self.manipulator.getMassMatrix()
self.forces = M @ (self.kp_pd * q_err + self.kd_pd * dq_err) + Cg
self.manipulator.setForces(self.forces)
```

When the user holds `r`, the OSC controller takes over.  It computes the
Jacobian, its pseudo-inverse, and uses `AngleAxis` math to determine the angular
error relative to the `target` frame:

```python
J = self.end_effector.getWorldJacobian(self.offset)
pinv_J = J.T @ np.linalg.inv(J @ J.T + 0.0025 * np.eye(6))
relative = self.target.getTransform(self.end_effector).rotation()
aa = dart.math.AngleAxis(relative)
angular_error = np.asarray(aa.axis()).reshape(3) * aa.angle()
translation_error = (
    self.target.getWorldTransform().translation()
    - self.end_effector.getWorldTransform().multiply(self.offset)
)
```

The final wrench combines the OSC correction, a small feed-forward push along
`+X`, and the gravity-compensated PD torques computed earlier.

## Lesson 4 – External pushes and visualization

`InputHandler` listens for comma and period key presses.  Each key calls
`set_external_force()` on `MyWorldNode`, which stores the requested force for a
fixed duration and uses an `ArrowShape` attached to a `SimpleFrame` to display
it on the torso.  The arrow’s tail/head vectors are updated each frame while the
force is active.

## Keyboard reference

| Key | Effect |
| --- | ------ |
| `q` / `w` / `e` | Add a leaning domino before the simulation starts |
| `d` | Delete the last domino |
| `space` | Start or pause the simulation |
| `p` | Replay |
| `f` | Push the first domino with a disembodied force |
| `r` | Push the first domino with the manipulator |
| `,` / `.` | Apply backward / forward pushes to the human torso |
| `v` | Toggle contact force visualization |

Experiment with different sequences of domino lean angles, then try both push
modes to see how the arm controller differs from the direct body-force method.
