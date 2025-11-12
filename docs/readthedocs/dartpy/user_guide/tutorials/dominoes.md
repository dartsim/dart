# Dominoes (dartpy)

## Overview

The dominoes tutorial combines interactive model editing with operational-space
control (OSC). You will:

- clone skeletons at runtime and validate their collision-free placement
- drive a manipulator using a gravity-compensated PD controller and OSC task
- trigger external pushes from keyboard events

Starter and finished files live in `python/tutorials/dominoes/`.  Launch the
viewer with:

```bash
pixi run py tu-dominoes
```

Before starting the simulation you can add or remove dominoes; once the
simulation begins you can push the first domino with a disembodied force or with
the manipulator arm.

## Lesson 1 – Managing domino skeletons

`DominoEventHandler` exposes two methods: ``attempt_to_create_domino`` and
``delete_last_domino``.  Pressing `q`, `w`, or `e` creates a domino that leans
left, forward, or right.  The method clones the base skeleton, positions it
relative to the previous domino, and uses the collision detector to ensure the
new body does not overlap with existing objects.

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

To detect overlaps the handler fetches the main collision group from the world’s
constraint solver, removes the floor, and tests the new domino against the
remaining shapes.  The floor entries are re-added afterward so the rest of the
simulation continues unaffected.

```python
solver = self.world.getConstraintSolver()
collision_group = solver.getCollisionGroup()
new_group = solver.getCollisionDetector().createCollisionGroup(new_domino)
collision_group.removeShapeFramesOf(self.floor)
domino_collision = collision_group.collide(new_group)
collision_group.addShapeFramesOf(self.floor)
```

If the test passes the domino is added to the world, recorded in a list, and the
cumulative angle is updated.  Pressing `d` deletes the last domino by removing
its skeleton from the world and adjusting the stored angles.

## Lesson 2 – Loading the manipulator and initial pose

`create_manipulator()` demonstrates how to load a URDF directly from dartpy:

```python
loader = dart.utils.DartLoader()
manipulator = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
manipulator.setName("manipulator")
base_tf = dart.math.Isometry3()
base_tf.set_translation([-0.65, 0.0, 0.0])
manipulator.getJoint(0).setTransformFromParentBodyNode(base_tf)
```

`setInitialPose()` (Lesson 2 in the original tutorial) is implemented inline in
`main()` by setting a handful of DOFs on the human skeleton so that the biped
starts upright.  The domino manipulator reuses the same pose from the URDF.

## Lesson 3 – Controller structure

`Controller` keeps three important members:

- ``q_desired`` – the joint configuration used by the PD controller.
- ``target`` – a `SimpleFrame` positioned at the top of the first domino.
- ``offset`` – a vector from the wrist frame to the contact point.

### PD controller

``set_pd_forces()`` implements the gravity-compensated PD controller:

```python
q = self.manipulator.getPositions().copy()
dq = self.manipulator.getVelocities()
dt = self.manipulator.getTimeStep()
q += dq * dt
q_err = self.q_desired - q
dq_err = -dq
Cg = self.manipulator.getCoriolisAndGravityForces()
M = self.manipulator.getMassMatrix()
self.forces = M @ (self.kp_pd * q_err + self.kd_pd * dq_err) + Cg
self.manipulator.setForces(self.forces)
```

The gains mirror the values from the original tutorial: floating-base rows are
zeroed out, while the remaining joints use aggressive gains.

### Operational space controller

`set_operational_space_forces()` computes the OSC task used to push the first
domino:

1. compute the analytic Jacobian and its pseudo-inverse
2. compute linear and angular error relative to the target frame
3. compute the time derivative of the error using
   ``getSpatialVelocity``
4. add damping (``kd_os``) and feed-forward forces to follow the OSC target

```python
J = self.end_effector.getWorldJacobian(self.offset)
pinv_J = J.T @ np.linalg.inv(J @ J.T + 0.0025 * np.eye(6))
relative = self.target.getTransform(self.end_effector).rotation()
axis_angle = dart.math.AngleAxis(relative)
angular_error = np.asarray(axis_angle.axis()).reshape(3) * axis_angle.angle()
translation_error = (
    self.target.getWorldTransform().translation()
    - self.end_effector.getWorldTransform().multiply(self.offset)
)
```

The final wrench includes a feed-forward push along +X to knock the domino.
`PendulumEventHandler` uses the `f` key to switch between PD-only mode and OSC
mode so you can compare the two behaviours.

## Lesson 4 – External pushes

`InputHandler` listens for comma and period key presses.  Each press calls
``set_external_force(force, duration)`` on ``MyWorldNode``.  Forces are applied
at the spine body node and visualized with an `ArrowShape` attached to a
`SimpleFrame`.  Holding the key sustains the force for the configured duration.

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

The dartpy tutorial now mirrors the original walkthrough entirely in Python,
from runtime skeleton cloning to OSC-driven pushes.
