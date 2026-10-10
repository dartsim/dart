# Biped

## Overview

Learn joint limits and self-collision, PD and stable PD control, ankle
feedback, skeleton editing, velocity actuators, and Jacobian-based inverse
kinematics. The final scene balances a biped on a skateboard.

Start with [main.py](https://github.com/dartsim/dart/blob/main/python/tutorials/biped/main.py)
and compare it with
[main_finished.py](https://github.com/dartsim/dart/blob/main/python/tutorials/biped/main_finished.py).
Follow the [setup instructions](../tutorials.rst) and run
`pixi run tu-biped-fi` for the solution. The model uses **Y as the upward
axis**, including its gravity, ground, and camera. Do not substitute Z for Y
when adapting the other tutorials' code.

| Key | Action |
| --- | --- |
| Space | Start or pause simulation |
| `.` / `,` | Apply a forward / backward push |
| `a` or `A` | Increase wheel speed by 0.5 |
| `s` or `S` | Decrease wheel speed by 0.5 |

The snippets use `import dartpy as dart` and `import numpy as np`.

## Lesson 1: Joint limits and self-collision

`load_biped()` uses the SKEL parser to load the world containing the biped:

```python
world = dart.utils.SkelParser.readWorld("dart://sample/skel/biped.skel")
biped = world.getSkeleton("biped")
```

SKEL is an XML format describing a world and its skeletons. The sample URI
resolves DART's installed model assets. Check that loading returned a world
and that the named skeleton exists before using it.

Without control, the biped collapses like a ragdoll. Its knees should flex
without extending backward. The knee description in `biped.skel` contains
bounds:

```xml
<joint type="revolute" name="j_shin_right">
  <parent>h_thigh_right</parent>
  <child>h_shin_right</child>
  <axis>
    <xyz>0.0 0.0 1.0</xyz>
    <limit>
      <lower>-3.14</lower>
      <upper>0.0</upper>
    </limit>
  </axis>
</joint>
```

You can also set coordinate bounds with `setPositionLowerLimit` and
`setPositionUpperLimit`. Bounds are enforced only after enabling enforcement
on each joint:

```python
for i in range(biped.getNumJoints()):
    biped.getJoint(i).setLimitEnforcement(True)
biped.enableSelfCollisionCheck()
biped.disableAdjacentBodyCheck()
```

Self-collision is disabled by default. Enabling it prevents bodies within the
same skeleton from penetrating one another; disabling adjacent-body checks
avoids collisions between directly connected bodies. The figure still falls,
but its joints and contacts now obey the intended physical restrictions.

## Lesson 2: Proportional-derivative control

A PD controller applies
{math}`\tau=-k_p(q-q_{target})-k_d\dot q` to hold a target pose. In
`set_initial_pose()`, set a roughly balanced crouching stance:

```python
for name, position in (
    ("j_thigh_left_z", 0.15),
    ("j_thigh_right_z", 0.15),
    ("j_shin_left", -0.4),
    ("j_shin_right", -0.4),
    ("j_heel_left_1", 0.25),
    ("j_heel_right_1", 0.25),
):
    biped.getDof(name).setPosition(position)
```

Angles are in radians. A degree of freedom also exposes
`getIndexInSkeleton()` if you need its index; `setPositions(array)` sets the
entire configuration at once.

In the controller constructor, allocate NumPy gain matrices. The root's six
free coordinates are unactuated, so their gains must be zero:

```python
dofs = biped.getNumDofs()
kp = np.diag([0.0] * 6 + [1000.0] * (dofs - 6))
kd = np.diag([0.0] * 6 + [50.0] * (dofs - 6))
target_positions = biped.getPositions().copy()
forces = np.zeros(dofs)
```

`add_pd_forces()` uses NumPy's `@` operator for matrix-vector products:

```python
q = biped.getPositions()
dq = biped.getVelocities()
p = -kp @ (q - target_positions)
d = -kd @ dq
forces += p + d
biped.setForces(forces)
```

The controller adds these terms to its force accumulator so other feedback
can contribute in the same step. Reset that accumulator before the next
update. Try ordinary PD first: gains of 1000 and 50 can produce a numerically
unstable simulation. Reducing gains helps, but suitable values depend on
body inertia, joint properties, configuration, and timestep.

## Lesson 3: Stable PD control

[Stable PD control](http://www.cc.gatech.edu/~jtan34/project/spd.html) uses a
prediction of the next state instead of only the current state. DART supplies
the mass matrix {math}`M`, Coriolis and gravity forces {math}`C_g`, and generalized
constraint forces {math}`f_c`. With timestep {math}`h`, compute

```{math}
p=-K_p(q+h\dot q-q_{target}),\qquad d=-K_d\dot q,
```

```{math}
\ddot q=(M+hK_d)^{-1}(-C_g+p+d+f_c),\qquad
\tau=p+d-hK_d\ddot q.
```

Implement `add_spd_forces()` with a linear solve rather than an explicit
matrix inverse:

```python
q = biped.getPositions()
dq = biped.getVelocities()
dt = biped.getTimeStep()
p = -kp @ (q + dq * dt - target_positions)
d = -kd @ dq
acceleration = np.linalg.solve(
    biped.getMassMatrix() + kd * dt,
    -biped.getCoriolisAndGravityForces()
    + p + d + biped.getConstraintForces(),
)
forces += p + d - kd @ acceleration * dt
biped.setForces(forces)
```

`getCoriolisForces()` and `getGravityForces()` expose the separate terms.
Constraint forces include contacts, joint limits, and user constraints such
as the ball constraint in the multi-pendulum lesson. Stable PD allows the
original gains over a much wider stable range. A standing target can remain
balanced, but external pushes still require feedback that adapts to motion.

## Lesson 4: Ankle strategy

An ankle strategy reacts to the horizontal deviation between center of mass
(COM) and an approximate center of pressure (COP). A linear feedback rule is
{math}`\theta_a=-k_p(x-p)-k_d(\dot x-\dot p)`; here the controller adds corresponding
heel and toe torques. Find `add_ankle_strategy_forces()` and compute the
sagittal deviation:

```python
heel = biped.getBodyNode("h_heel_left")
offset = np.array([0.05, 0.0, 0.0])
cop = heel.getTransform().multiply(offset)
diff = biped.getCOM()[0] - cop[0]
dcop = heel.getLinearVelocity(offset)
ddiff = biped.getCOMLinearVelocity()[0] - dcop[0]
```

`getTransform()` defaults to world coordinates; a frame argument requests a
transform relative to another frame. `getCOM()` and the velocity APIs also
support frame arguments. Useful APIs include:

| Method | Quantity |
| --- | --- |
| `getSpatialVelocity` | Spatial velocity in the requested coordinates |
| `getLinearVelocity` / `getAngularVelocity` | Classical linear / angular velocity |
| `getSpatialAcceleration` | Spatial acceleration |
| `getLinearAcceleration` / `getAngularAcceleration` | Classical linear / angular acceleration |

The solution uses separate forward and backward recovery gains. For
{math}`0\leq diff<0.1`, heel/toe/derivative gains are 200/100/10; for
{math}`-0.2<diff<-0.05`, they are 2000/100/100. Each relevant coordinate receives
`-gain * diff - derivative_gain * ddiff`. These gains are tuned for this
model; experiment with moderate pushes using `.` and `,`.

## Lesson 5: Skeleton editing

Load `dart://sample/skel/skateboard.skel`, then move its root and entire
subtree under the biped's left heel using an Euler joint:

```python
world = dart.utils.SkelParser.readWorld("dart://sample/skel/skateboard.skel")
skateboard = world.getSkeleton(0)
properties = dart.dynamics.EulerJointProperties()
child_transform = dart.math.Isometry3()
child_transform.set_translation([0.0, 0.1, 0.0])
properties.mT_ChildBodyToJoint = child_transform
joint = skateboard.getRootBodyNode().moveToEulerJoint(
    biped.getBodyNode("h_heel_left"), properties
)
```

The 0.1 m Y offset makes room between the board and the heel. The returned
joint belongs to the destination skeleton. Replacing the parent joint deletes
the original joint, so discard any references to it and use the returned joint.
This edits the model topology:
the board is now part of the biped rather than a separate world skeleton.

Other body-node editing operations include:

| Method | Effect |
| --- | --- |
| `remove()` | Remove a body and its subtree into a separate skeleton |
| `moveTo(parent)` | Move a subtree under a new parent |
| `moveToEulerJoint(parent, properties)` | Move a subtree and replace its parent joint with an Euler joint |
| `split(name)` | Move a subtree into a new named skeleton |
| `copyTo(parent)` | Clone a subtree under a new parent |
| `copyAs(name)` | Clone a subtree into a new named skeleton |

## Lesson 6: Actuator types

Each joint selects its actuator type:

| Type | Behavior |
| --- | --- |
| `FORCE` | Use the commanded joint force to compute acceleration |
| `PASSIVE` | Apply no actuator force |
| `ACCELERATION` | Compute force to achieve commanded acceleration |
| `VELOCITY` | Compute force to achieve commanded velocity |
| `LOCKED` | Keep velocity and acceleration at zero |

Set all four wheel joints to velocity control in `set_velocity_actuators()`:

```python
for name in (
    "joint_front_left", "joint_front_right",
    "joint_back_left", "joint_back_right",
):
    biped.getJoint(name).setActuatorType(dart.dynamics.ActuatorType.VELOCITY)
```

In `set_wheel_commands()`, clear the wheel coordinates' PD gains and command
their velocities every step:

```python
first = biped.getDof("joint_front_left_1").getIndexInSkeleton()
kp[first:, first:] = 0.0
kd[first:, first:] = 0.0
for name in (
    "joint_front_left_2", "joint_front_right_2",
    "joint_back_left", "joint_back_right",
):
    biped.getDof(name).setCommand(speed)
```

A command applies to the current timestep, so it must be repeated for
continuous motion. Zeroing the wheel gains keeps velocity actuators out of
the SPD force calculation. Use `a` and `s` to change wheel speed. The earlier
two-foot target pose is insufficient for balancing on one foot; use IK to
find a better pose next.

## Lesson 7: Inverse kinematics

`solve_ik()` finds a pose with COM supported over the left foot and the foot
corners level. Minimize horizontal COM-to-foot deviation and vertical
corner-to-ground error:

![IK objective](biped/IKObjective.png)

```{math}
E=\tfrac12\|c-p\|^2+\tfrac12\sum_{i=1}^4(y_i-y_{ground})^2.
```

Here {math}`c` and {math}`p` are COM and left-foot COM projected onto the X-Z plane;
{math}`y_i` is a heel or toe corner's height. The target height in this model is
{math}`y_{ground}=-0.8`. The solution first bends the right knee and raises the
arms, then performs 4500 gradient-descent iterations.

A Jacobian gives each point's derivative with respect to generalized
coordinates. The derivative of COM-to-foot deviation is

```python
heel = biped.getBodyNode("h_heel_left")
local_com = heel.getCOM(heel)
jacobian = biped.getCOMLinearJacobian() - biped.getLinearJacobian(
    heel, local_com
)
```

Use rows 0 and 2 for the horizontal error. The corner-height gradient is row
1 because Y is vertical:

```python
offset = np.array([0.0, -0.04, -0.03])
error = heel.getTransform().multiply(offset)[1] - (-0.8)
gradient = biped.getLinearJacobian(heel, offset)[1]
```

Repeat this for both heel corners and both toe corners, accumulate a descent
direction with step size 0.2, update positions, and call
`computeForwardKinematics(True, False, False)` before evaluating the next
iteration. `solve_ik()` returns the resulting target configuration.

| Jacobian method | Quantity |
| --- | --- |
| `getJacobian` | Generalized spatial Jacobian |
| `getLinearJacobian` / `getAngularJacobian` | Linear / angular point Jacobian |
| `getJacobianSpatialDeriv` | Spatial time derivative |
| `getJacobianClassicDeriv` | Classical time derivative |
| `getLinearJacobianDeriv` / `getAngularJacobianDeriv` | Linear / angular classical time derivative |

The finished scene applies SPD, ankle feedback, and wheel commands before
each world step. Try gentle skateboard acceleration and braking, then
external pushes, to explore when the one-foot balance controller recovers.
