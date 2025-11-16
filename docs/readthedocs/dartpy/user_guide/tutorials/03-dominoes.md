# Dominoes

## Overview

This tutorial demonstrates some of the more advanced features of DART's
python bindings. You will learn how to:

- Clone Skeletons
- Load a URDF manipulator
- Write a stable PD controller with gravity/coriolis compensation
- Write an operational space controller (OSC)

Please reference the source code in
[**python/tutorials/03_dominoes/main.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/03_dominoes/main.py)
and
[**python/tutorials/03_dominoes/main_finished.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/03_dominoes/main_finished.py).

## Lesson 1: Cloning Skeletons

There are often times where you might want to create an exact replica of an
existing Skeleton. DART offers cloning functionality that allows you to do this
very easily.

### Lesson 1a: Create a new domino

Creating a new domino is straightforward. Find the function
``attempt_to_create_domino`` in ``DominoEventHandler``. The handler has a
member called ``first_domino`` which is the original domino created when the
program starts up. To make a new one, we can just clone it:

```python
new_domino = self.first_domino.clone(
    f"domino_{len(self.dominoes) + 1}"
)
```

But keep in mind that every Skeleton that gets added to a world requires its own
unique name. Creating a clone will keep the original name, so we should give the
new copy its own name (the ``clone`` helper accepts the new name for us).

So the easy part is finished, but now we need to get the domino to the correct
position. First, let's grab the last domino that was placed in the environment:

```python
last_domino = self.dominoes[-1] if self.dominoes else self.first_domino
```

Now we should compute what we want its position to be. The handler keeps a
member called ``total_angle`` which tracks how much the line of dominoes has
turned so far. We'll use that to figure out what translational offset the
new domino should have from the last domino:

```python
displacement = default_distance * np.array(
    [math.cos(self.total_angle), math.sin(self.total_angle), 0.0]
)
```

And now we can compute the total position of the new domino. First, we'll copy
the positions of the last domino:

```python
pose = last_domino.getPositions().copy()
```

And then we'll add the translational offset to it along with the new yaw angle:

```python
pose[3:] += displacement
pose[2] = self.total_angle + angle
new_domino.setPositions(pose)
```

Remember that the domino's root joint is a FreeJoint which has six degrees of
freedom: the first three are for orientation and last three are for translation.
We only need to touch the final three entries when shifting it down the line.

Now we'll add the Skeleton to the world once we've decided to keep it:

```python
self.world.addSkeleton(new_domino)
```

### Lesson 1b: Make sure no dominoes are in collision

Similar to **Lesson 3** of the **Collisions** tutorial, we'll want to make sure
that the newly inserted Skeleton is not starting out in collision with anything,
because this could make for a very ugly (perhaps even broken) simulation.

First, we'll grab the world's constraint solver and collision detector:

```python
solver = self.world.getConstraintSolver()
collision_group = solver.getCollisionGroup()
detector = solver.getCollisionDetector()
new_group = detector.createCollisionGroup(new_domino)
```

We remove the floor from the collision group before performing the test so that
contact with the ground does not count as a collision. After the test we add it
back:

```python
collision_group.removeShapeFramesOf(self.floor)
domino_collision = collision_group.collide(new_group)
collision_group.addShapeFramesOf(self.floor)
```

The only object that could possibly have collided with something else is the
new domino, because we don't allow the application to create new things except
for the dominoes. So if this registered as true, then we should skip adding the
new domino to the world. Otherwise, we record it and update ``total_angle``:

```python
if not domino_collision:
    self.world.addSkeleton(new_domino)
    self.dominoes.append(new_domino)
    self.angles.append(angle)
    self.total_angle += angle
```

### Lesson 1c: Delete the last domino added

Ordinarily, removing a Skeleton from a scene is just a matter of calling the
``World.removeSkeleton`` function, but we have a little bit of bookkeeping to
take care of for our particular application. First, we check whether there are
any dominoes to actually remove:

```python
if not self.dominoes:
    return
```

Then we grab the last domino in the history, remove it from the history,
and then take it out of the world:

```python
last_domino = self.dominoes.pop()
self.world.removeSkeleton(last_domino)
self.total_angle -= self.angles.pop()
```

### Lesson 1d: Apply a force to the first domino

But just setting up dominoes isn't much fun without being able to knock them
down. We can quickly and easily knock down the dominoes by magically applying
a force to the first one. Inside ``DominoEventHandler.update`` there is code
which runs whenever ``force_countdown`` is positive. That value gets set
whenever the user presses ``f``:

```python
force = default_push_force * np.array([1.0, 0.0, 0.0])
location = np.array([0.0, 0.0, default_domino_height / 2.0])
self.first_domino.getBodyNode(0).addExtForce(force, location)
```

## Lesson 2: Loading and controlling a robotic manipulator

Striking something with a magical force is convenient, but not very believable.
Instead, let's load a robotic manipulator and have it push over the first domino.

### Lesson 2a: Load a URDF file

Our manipulator is going to be loaded from a URDF file. URDF files are loaded
by ``dart.utils.DartLoader``. First, create a loader and parse the file into a
Skeleton:

```python
loader = dart.utils.DartLoader()
manipulator = loader.parseSkeleton(
    "dart://sample/urdf/KR5/KR5 sixx R650.urdf"
)
manipulator.setName("manipulator")
```

Now we'll want to initialize the location and configuration of the manipulator.
Experimentation has demonstrated that the following setup is good for our purposes:

```python
base_tf = dart.math.Isometry3()
base_tf.set_translation([-0.65, 0.0, 0.0])
manipulator.getJoint(0).setTransformFromParentBodyNode(base_tf)

manipulator.getDof(1).setPosition(math.radians(140.0))
manipulator.getDof(2).setPosition(math.radians(-140.0))
```

### Lesson 2b: Grab the desired joint angles

To make the manipulator actually useful, we'll want the ``Controller`` class to
hold the reference joint configuration. This is easily done in its constructor:

```python
self.q_desired = self.manipulator.getPositions().copy()
```

### Lesson 2c: Write a stable PD controller for the manipulator

Now that we know what configuration we want the manipulator to hold, we can
write a PD controller that keeps it in place. Find the function ``set_pd_forces``.

First, we'll grab the current positions and velocities and integrate the
positions forward one time step:

```python
dt = self.manipulator.getTimeStep()
q = self.manipulator.getPositions().copy()
dq = self.manipulator.getVelocities()
q += dq * dt
```

Then we compute our joint position error and velocity error:

```python
q_err = self.q_desired - q
dq_err = -dq
```

Now we can grab our mass matrix, which we will use to scale our force terms, and
the Coriolis+gravity vector so we can add the compensation term later:

```python
mass = self.manipulator.getMassMatrix()
cg = self.manipulator.getCoriolisAndGravityForces()
```

And then combine all this into a PD controller that computes forces to minimize
our error:

```python
self.forces = mass @ (self.kp_pd * q_err + self.kd_pd * dq_err) + cg
self.manipulator.setForces(self.forces)
```

### Lesson 2d: Compensate for gravity and Coriolis forces

One of the key features of DART is the ability to easily compute the gravity and
Coriolis forces, allowing you to write much higher quality controllers. The term
``cg`` computed above is added directly to the PD torques so the manipulator can
hold a pose even when its links are heavy or not perfectly balanced.

## Lesson 3: Writing an operational space controller

While PD controllers are simple and handy, operational space controllers can be
much more elegant and useful for performing tasks. Operational space controllers
allow us to unify geometric tasks (like getting the end effector to a particular
spot) and dynamics tasks (like applying a certain force with the end effector).

### Lesson 3a: Set up the information needed for an OS controller

Unlike PD controllers, an operational space controller needs more information
than just desired joint angles. The Controller constructor prepares it by finding
the end effector, storing a wrist offset, and creating the target ``SimpleFrame``:

```python
self.end_effector = self.manipulator.getBodyNode(
    self.manipulator.getNumBodyNodes() - 1
)
self.offset = np.array([default_endeffector_offset, 0.0, 0.0])
self.target = dart.dynamics.SimpleFrame(
    dart.dynamics.Frame.World(), "target"
)
```

The target frame is positioned on top of the first domino and aligned with the
end effector's coordinate frame:

```python
target_tf = dart.math.Isometry3()
target_tf.set_translation([0.0, 0.0, default_domino_height / 2.0])
relative = self.end_effector.getTransform(domino.getBodyNode(0))
target_tf.set_rotation(relative.rotation())
self.target.setTransform(target_tf, domino.getBodyNode(0))
```

### Lesson 3b: Computing forces for the OS controller

Find the function ``set_operational_space_forces``. This is where we'll compute
the forces for our operational space controller.

One of the key ingredients in an operational space controller is the mass matrix.
We can get this easily, just like we did for the PD controller:

```python
mass = self.manipulator.getMassMatrix()
```

Next we'll want the Jacobian of the tool offset in the end effector and its
Moore-Penrose pseudo-inverse:

```python
J = self.end_effector.getWorldJacobian(self.offset)
JJt = J @ J.T + 0.0025 * np.eye(6)
pinv_J = J.T @ np.linalg.inv(JJt)
```

We also compute the time derivative of the Jacobian and its pseudo-inverse:

```python
dJ = self.end_effector.getJacobianClassicDeriv(self.offset)
dJdJt = dJ @ dJ.T + 0.0025 * np.eye(6)
pinv_dJ = dJ.T @ np.linalg.inv(dJdJt)
```

Now we can compute the linear and angular components of error:

```python
target_tf = self.target.getWorldTransform()
end_tf = self.end_effector.getWorldTransform()
translation_error = target_tf.translation() - end_tf.multiply(self.offset)
relative = self.target.getTransform(self.end_effector).rotation()
aa = dart.math.AngleAxis(relative)
angular_error = np.asarray(aa.axis()).reshape(3) * aa.angle()
```

Then the time derivative of error, assuming our desired velocity is zero:

```python
de = -self.end_effector.getSpatialVelocity(
    self.offset, self.target, dart.dynamics.Frame.World()
)
```

Like with the PD controller, we mix in terms to compensate for gravity and
Coriolis forces, and define the controller gains:

```python
cg = self.manipulator.getCoriolisAndGravityForces()
Kp = self.kp_os * np.eye(6)
dofs = self.manipulator.getNumDofs()
Kd = self.kd_os * np.eye(dofs)
```

We also add a small feed-forward wrench in +X so the arm actually pushes the
domino once it is in position:

```python
f_desired = np.zeros(6)
f_desired[3] = default_push_force
feedforward = J.T @ f_desired
```

Now we can mix everything together into the single control law:

```python
dq = self.manipulator.getVelocities()
self.forces = (
    mass @ (pinv_J @ (Kp @ de) + pinv_dJ @ (Kp @ e))
    - Kd @ dq
    + Kd @ (pinv_J @ (Kp @ e))
    + cg
    + feedforward
)
self.manipulator.setForces(self.forces)
```

Pressing ``r`` engages the OSC for a short duration so the arm reaches for and
pushes the first domino, while pressing ``f`` continues to apply the magical body
force described in Lesson 1d. Press `space` to let the simulation run and watch
the dominoes fall.
