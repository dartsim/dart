# 03. Dominoes (dartpy)

## Overview

The domino tutorial combines runtime skeleton editing with an operational-space
controller (OSC).  By following the lessons you will:

- clone domino skeletons, position them relative to existing ones, and ensure
  they are collision free before adding them to the world
- load a URDF manipulator and target the top of the first domino
- implement both a gravity-compensated PD controller and an OSC task
- visualize disembodied pushes with `ArrowShape`s driven from keyboard input

All code resides in `python/tutorials/03_dominoes/`.  Run the tutorial with:

```bash
pixi run py-tu-dominoes
```

## Lesson 1: Creating the world

### 1a – Base domino

`create_domino()` instantiates a `FreeJoint`/`BodyNode` pair and attaches a thin
box shape:

```python
domino = dart.dynamics.Skeleton("domino")
_, body = domino.createFreeJointAndBodyNodePair()
shape = dart.dynamics.BoxShape([
    default_domino_depth,
    default_domino_width,
    default_domino_height,
])
node = body.createShapeNode(shape)
node.createVisualAspect().setColor([0.9, 0.3, 0.1, 1.0])
node.createCollisionAspect()
node.createDynamicsAspect()
```

The mass is computed from the box volume and the root DOF `Joint_pos_z` is set to
`default_domino_height / 2` so the domino rests on the floor.

### 1b – Floor and manipulator

`create_floor()` uses a `WeldJoint` and a large box to create the ground plane.
`create_manipulator()` loads the KR5 URDF via `dart.utils.DartLoader` and moves
the base backwards so the arm sits behind the domino line.

## Lesson 2: Domino editing

`DominoEventHandler` manages user-created dominoes.  Pressing `q`, `w`, or `e`
creates a new domino leaning left, straight, or right.  The handler:

1. Clones the base domino skeleton (`skeleton.clone("domino_n")`).
2. Computes a displacement along the current heading using `default_distance`.
3. Adjusts the yaw (`pose[2]`) by the requested lean angle.
4. Temporarily removes the floor from the world collision group, builds a new
   collision group containing the clone, and checks for intersections.  If the
   clone collides with any existing object the spawn is aborted and a warning is
   printed.

Deleted dominoes are removed from both the world and the internal history so
angles remain consistent.

## Lesson 3: Controller setup

`Controller` stores the manipulator skeleton, the active domino skeleton, the
end-effector pointer, and a `SimpleFrame` named `target`.  The target is
positioned at the top center of the first domino and rotated so it matches the
end-effector orientation.

### 3a – PD controller

`set_pd_forces()` copies the structure from the C++ tutorial but uses numpy:

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

### 3b – Operational-space controller

When the user holds `r`, `set_operational_space_forces()` takes over.  It computes
the Jacobian at the wrist offset, the pseudo-inverse, and the angular/linear
errors relative to `self.target`:

```python
J = self.end_effector.getWorldJacobian(self.offset)
pinv_J = J.T @ np.linalg.inv(J @ J.T + 0.0025 * np.eye(6))
relative = self.target.getTransform(self.end_effector).rotation()
angle_axis = dart.math.AngleAxis(relative)
angular_error = np.asarray(angle_axis.axis()).reshape(3) * angle_axis.angle()
translation_error = (
    self.target.getWorldTransform().translation()
    - self.end_effector.getWorldTransform().multiply(self.offset)
)
```

A small feed-forward wrench (`f_desired[3] = default_push_force`) completes the
OSC task.  The handler toggles between PD and OSC modes so you can compare them
interactively.

## Lesson 4: External pushes

`MyWorldNode` keeps track of an external force vector and a countdown timer.
Pressing `,` or `.` calls `set_external_force()` with a ±X push.  During each
`customPreStep()` call the node applies the force to `h_spine` and updates an
`ArrowShape` attached to a `SimpleFrame` so the push is visible in the viewer.
When the timer expires the arrow is hidden and the force is cleared.

## Lesson 5: User interface summary

| Key | Effect |
| --- | ------ |
| `q` / `w` / `e` | Add leaning dominoes before the simulation starts |
| `d` | Delete the last domino |
| `space` | Start / pause the simulation |
| `p` | Replay |
| `f` | Push the first domino with a disembodied force |
| `r` | Use the manipulator arm (OSC controller) to push the domino |
| `,` / `.` | Apply backward / forward pushes to the human torso |
| `v` | Toggle contact force visualization |

Follow the lessons sequentially: place dominoes, start the simulation, and
experiment with both push mechanisms.  Because everything is written in Python,
you can easily extend the tutorial—for example by changing the OSC target or by
adding new domino placement modes.
