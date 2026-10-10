# Dominoes

## Overview

Clone and place dominoes, load a URDF manipulator, and compare a predicted-state
PD controller with an operational-space controller. The robot pushes the
first domino to start the chain reaction.

Start with [main.py](https://github.com/dartsim/dart/blob/main/python/tutorials/dominoes/main.py)
and compare it with
[main_finished.py](https://github.com/dartsim/dart/blob/main/python/tutorials/dominoes/main_finished.py).
Follow the [setup instructions](../tutorials.rst) and run
`pixi run tu-dominoes-fi` for the solution. This scene uses Z as the upward
axis. The scripts use `import dartpy as dart`, `import numpy as np`, and
`import math`.

| Key | Action |
| --- | --- |
| `q` / `w` / `e` | Add a domino turning left / straight / right before simulation starts |
| `d` | Delete the last added domino before simulation starts |
| Space | Start or pause simulation; placement is disabled after the first start |
| `f` | Push the first domino with an external force after starting |
| `r` | Push the first domino with the robot after starting |
| `p` | Toggle replay |
| `v` | Toggle contact-force arrows, including during replay |

## Lesson 1: Cloning Skeletons

Cloning creates an independent model with the original skeleton's geometry,
properties, and state. A new copy still needs a unique world name and a
collision-free initial pose.

### Lesson 1a: Create a new domino

Find `DominoEventHandler.attemptToCreateDomino(angle)`. The handler retains
`mFirstDomino` as the original. Clone it and select the last placed domino
as the placement reference:

```python
new_domino = self.mFirstDomino.clone()
new_domino.setName("domino #" + str(len(self.mDominoes) + 1))
last_domino = self.mDominoes[-1] if self.mDominoes else self.mFirstDomino
```

Use `mTotalAngle` to compute the offset along the current line of dominoes:

```python
dx = default_distance * np.array(
    [math.cos(self.mTotalAngle), math.sin(self.mTotalAngle), 0.0]
)
positions = last_domino.getPositions().copy()
positions[3:] += dx
positions[2] = self.mTotalAngle + angle
new_domino.setPositions(positions)
```

The root FreeJoint has six coordinates: three for angle-axis orientation,
then three for translation. `positions[3:]` updates translation and
`positions[2]` sets rotation around Z for this planar layout. Domino spacing
is 0.15 m and each left/right turn is 20 degrees. Add the skeleton to the
world only after the next lesson's collision check succeeds.

### Lesson 1b: Make sure no dominoes are in collision

As in the [Collisions tutorial](collisions.md), test the new skeleton against
the existing world. Floor contact is intended, so temporarily exclude floor
shapes from this query:

```python
solver = self.mWorld.getConstraintSolver()
collision_group = solver.getCollisionGroup()
new_group = solver.getCollisionDetector().createCollisionGroup()
new_group.addShapeFramesOf(new_domino)
collision_group.removeShapeFramesOf(self.mFloor)
try:
    domino_collision = collision_group.collide(new_group)
finally:
    collision_group.addShapeFramesOf(self.mFloor)
```

The `finally` block restores floor collision checking even if the query
fails. A collision rejects the placement without changing its history.
Otherwise add the skeleton and record its turn:

```python
if domino_collision:
    print("The new domino would penetrate something. It will not be added.")
    return
self.mWorld.addSkeleton(new_domino)
self.mAngles.append(angle)
self.mDominoes.append(new_domino)
self.mTotalAngle += angle
```

This tests only the proposed domino against the existing scene, so unrelated
contacts do not reject placement.

### Lesson 1c: Delete the last domino added

The original domino stays in place. If the history contains a clone, remove
that last clone and undo its contribution to the line's total turn:

```python
if self.mDominoes:
    self.mWorld.removeSkeleton(self.mDominoes.pop())
    self.mTotalAngle -= self.mAngles.pop()
```

Keep the angle and skeleton histories synchronized. Python references and
DART's skeleton ownership handle lifetime management; no manual deletion
is needed. Add and delete dominoes with `q`, `w`, `e`, and `d`. Once physics
has run, placement is disabled even while paused, because the last simulated
pose is no longer a reliable layout reference.

### Lesson 1d: Apply a force to the first domino

In `DominoEventHandler.update()`, apply an 8 N push for 200 simulation steps
when the user presses `f`:

```python
self.mFirstDomino.getBodyNode(0).addExtForce(
    [default_push_force, 0.0, 0.0],
    [0.0, 0.0, default_domino_height / 2.0],
)
self.mForceCountDown -= 1
```

With the default `addExtForce` arguments, the force is expressed in world
coordinates and its location in body coordinates. The force points along X
and is applied at the top of the first domino.

## Lesson 2: Loading and controlling a robotic manipulator

Replace the disembodied push with a six-axis KR5 manipulator. A controller
first holds the robot's initial configuration against gravity, then reaches
and pushes using operational-space control.

### Lesson 2a: Load a URDF file

`createManipulator()` uses `dart.utils.DartLoader` to load the URDF sample:

```python
loader = dart.utils.DartLoader()
manipulator = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
if manipulator is None:
    raise RuntimeError("Could not load the KR5 manipulator sample URDF.")
manipulator.setName("manipulator")
```

For URDF resources with `package://` paths, use
`loader.addPackageDirectory(package_name, directory)` to supply package
locations. The sample uses DART's installed resources.

Position the base and set a useful configuration:

```python
tf = dart.math.Isometry3()
tf.set_translation([-0.65, 0.0, 0.0])
manipulator.getJoint(0).setTransformFromParentBodyNode(tf)
manipulator.getDof(1).setPosition(math.radians(140.0))
manipulator.getDof(2).setPosition(math.radians(-140.0))
```

Return this loaded skeleton. Without control it will fall under gravity.

### Lesson 2b: Grab the desired joint angles

In the controller constructor, preserve the initial joint configuration:

```python
self.mQDesired = manipulator.getPositions().copy()
```

`getPositions()` returns all generalized positions as a NumPy array. The
same vectorized skeleton interface supplies velocities and accepts forces.

### Lesson 2c: Write a stable PD controller for the manipulator

In `setPDForces()`, predict the next position with the current velocity.
Assume desired velocity is zero, then scale the error terms by the mass
matrix:

```python
q = self.mManipulator.getPositions()
dq = self.mManipulator.getVelocities()
dt = self.mManipulator.getTimeStep()
q_err = self.mQDesired - (q + dq * dt)
dq_err = -dq
mass = self.mManipulator.getMassMatrix()
self.mForces = mass @ (self.mKpPD * q_err + self.mKdPD * dq_err)
self.mManipulator.setForces(self.mForces)
```

The scalar gains are 200 and 20. The control law is
{math}`\tau=M[k_p(q_{desired}-q-h\dot q)-k_d\dot q]`.
Compare the result with ordinary PD by removing the prediction term
`dq * dt`. This lesson uses the original tutorial's predicted-state,
mass-scaled controller; the [Biped tutorial](biped.md) demonstrates the
implicit acceleration solve for stable PD.

### Lesson 2d: Compensate for gravity and Coriolis forces

DART provides the combined Coriolis and gravity term directly. Add it to
the previous force law:

```python
cg = self.mManipulator.getCoriolisAndGravityForces()
self.mForces = mass @ (self.mKpPD * q_err + self.mKdPD * dq_err) + cg
self.mManipulator.setForces(self.mForces)
```

Compare the robot's pose error with and without this compensation. It can
hold its target more accurately without relying on position error to
counteract gravity.

## Lesson 3: Writing an operational space controller

Operational-space control expresses a task in terms of the end effector's
pose and force, then maps it to joint forces. Here the task is to reach the
top of the first domino and exert a push.

### Lesson 3a: Set up the information needed for an OS controller

In the constructor, select the last body as the end effector and offset its
tool point 0.05 m along its local X axis:

```python
self.mEndEffector = manipulator.getBodyNode(manipulator.getNumBodyNodes() - 1)
self.mOffset = np.array([default_endeffector_offset, 0.0, 0.0])
self.mTarget = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target")
```

Place the target at the top of the domino and align its orientation with
the end effector's initial orientation:

```python
target_offset = dart.math.Isometry3()
target_offset.set_translation([0.0, 0.0, default_domino_height / 2.0])
target_offset.set_rotation(
    self.mEndEffector.getTransform(domino.getBodyNode(0)).rotation()
)
self.mTarget.setTransform(target_offset, domino.getBodyNode(0))
```

Specifying the transform relative to the domino prevents a mismatched
orientation from making the manipulator approach at an unwanted angle.
Keep the target frame alive as a controller member.

### Lesson 3b: Computing forces for OS Controller

In `setOperationalSpaceForces()`, obtain the mass matrix and the tool-point
world Jacobian. Use a damped pseudoinverse to reduce sensitivity near
singularities:

```python
mass = self.mManipulator.getMassMatrix()
jacobian = self.mEndEffector.getWorldJacobian(self.mOffset)
pinv_j = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + 0.0025 * np.eye(6))
derivative = self.mEndEffector.getJacobianClassicDeriv(self.mOffset)
pinv_dj = derivative.T @ np.linalg.inv(
    derivative @ derivative.T + 0.0025 * np.eye(6)
)
```

This retains the tutorial's controller equations. The regularizer 0.0025
keeps these inverses defined when the Jacobian loses rank. For larger
applications, consider linear solves or SVD-based pseudoinverses. The
classical Jacobian derivative is used here; spatial-vector algorithms can
instead use `getJacobianSpatialDeriv`.

Build a six-component pose error with angular components first and linear
components last:

```python
error = np.zeros(6)
end_tf = self.mEndEffector.getWorldTransform()
error[3:] = self.mTarget.getWorldTransform().translation() - (
    end_tf.rotation() @ self.mOffset + end_tf.translation()
)
aa = dart.math.AngleAxis(self.mTarget.getTransform(self.mEndEffector).rotation())
error[:3] = aa.angle() * aa.axis()
de = -self.mEndEffector.getSpatialVelocity(
    self.mOffset, self.mTarget, dart.dynamics.Frame.World()
)
```

The angle-axis form represents rotational error without subtracting rotation
matrices. `de` is the negative current relative tool velocity because the
desired velocity is zero.

Convert scalar gains to matrices, compensate for gravity and Coriolis forces,
and map the desired 8 N X-axis push into joint forces with {math}`J^T`:

```python
cg = self.mManipulator.getCoriolisAndGravityForces()
kp = self.mKpOS * np.eye(6)
kd = self.mKdOS * np.eye(self.mManipulator.getNumDofs())
f_desired = np.zeros(6)
f_desired[3] = default_push_force
dq = self.mManipulator.getVelocities()
self.mForces = (
    mass @ (pinv_j @ kp @ de + pinv_dj @ kp @ error)
    - kd @ dq
    + kd @ pinv_j @ kp @ error
    + cg
    + jacobian.T @ f_desired
)
self.mManipulator.setForces(self.mForces)
```

The operational-space gains are 5 and 0.01. After starting physics, press
`r` to run this controller for 1000 steps. The handler uses the PD controller
again after the push expires.

## Replay and contact forces

The custom world node calls the handler before each step, then records the
result with `world.bake()` afterward. Press `p` to replay poses at a stride of
16 recorded frames per viewer refresh. Replay pauses physics and applies
each skeleton's saved configuration:

```python
recording = world.getRecording()
for i in range(world.getNumSkeletons()):
    world.getSkeleton(i).setPositions(recording.getConfig(frame, i))
```

Check `getNumFrames()`, `getNumSkeletons()`, and `getNumDofs(i)` before
replaying. A topology change invalidates old configurations; stop replay
rather than assigning arrays with incompatible dimensions. The world owns
its recording; retain the world while using it. Invalid frame, skeleton, or
contact indices raise `IndexError`.

For live contact arrows, read `world.getLastCollisionResult()` and each
contact's `point` and `force`. For recorded arrows, use
`recording.getNumContacts(frame)`, `getContactPoint(frame, i)`, and
`getContactForce(frame, i)`. The handler retains a pool of visual-only
`SimpleFrame` arrow shapes and hides unused arrows, so toggling `v` does not
create new collidable geometry. Force vectors are scaled by 0.1 for display.
