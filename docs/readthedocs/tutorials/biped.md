# Overview
This tutorial demonstrates the dynamic features in DART useful for
developing controllers for bipedal or wheel-based robots. The tutorial
consists of seven Lessons covering the following topics:

- Joint limits and self-collision.
- Actuators types and management.
- APIs for Jacobian matrices and other kinematic quantities.
- APIs for dynamic quantities.
- Skeleton editing.

Please reference the source code in [**tutorialBiped.cpp**](https://github.com/dartsim/dart/blob/release-5.1/tutorials/tutorialBiped.cpp) and [**tutorialBiped-Finished.cpp**](https://github.com/dartsim/dart/blob/release-5.1/tutorials/tutorialBiped-Finished.cpp).

# Lesson 1: Joint limits and self-collision
Let's start by locating the ``main`` function in tutorialBiped.cpp. We first create a floor
and call ``loadBiped`` to load a bipedal figure described in SKEL
format, which is an XML format representing a robot model. A SKEL file
describes a ``World`` with one or more ``Skeleton``s in it. Here we
load in a World from [**biped.skel**](https://github.com/dartsim/dart/blob/release-5.1/data/skel/biped.skel) and assign the bipedal figure to a
``Skeleton`` pointer called *biped*.

```cpp
SkeletonPtr loadBiped()
{
...
    WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/biped.skel");
    SkeletonPtr biped = world->getSkeleton("biped");
...
}
```

Running the skeleton code (hit the spacebar) without any modification, you should see a
human-like character collapse on the ground and fold in on
itself. Before we attempt to control the biped, let's first make the
biped a bit more realistic by enforcing more human-like joint limits.

DART allows the user to set upper and lower bounds on each degree of
freedom in the SKEL file or using provided APIs. For example, you
should see the description of the right knee joint in **biped.skel**:

```cpp
<joint type="revolute" name="j_shin_right">
...
    <axis>
        <xyz>0.0 0.0 1.0</xyz>
        <limit>
            <lower>-3.14</lower>
            <upper>0.0</upper>
        </limit>                  
    </axis>
...
</joint>
```
The &lt;upper> and &lt;lower> tags make sure that the knee can only flex but
not extend. Alternatively, you can directly specify the joint limits
in the code using
``setPositionUpperLimit`` and ``setPositionLowerLimit``.

In either case, the joint limits on the biped will not be activated
until you call ``setPositionLimited``:

```cpp
SkeletonPtr loadBiped()
{
...
    for(size_t i = 0; i < biped->getNumJoints(); ++i)
        biped->getJoint(i)->setPositionLimited(true);
...
}
```
Once the joint limits are set, the next task is to enforce
self-collision. By default, DART does not check self-collision within
a skeleton. You can enable self-collision checking on the biped by
```cpp
SkeletonPtr loadBiped()
{
...
    biped->enableSelfCollision();
...
}
```
This function will enable self-collision on every non-adjacent pair of
body nodes. If you wish to also enable self-collision on adjacent body
nodes, set the optional parameter to true:
```cpp
biped->enableSelfCollision(true);
```
Running the program again, you should see that the character is still
floppy like a ragdoll, but now the joints do not bend backward and the
body nodes do not penetrate each other anymore.

# Lesson 2: Proportional-derivative control

To actively control its own motion, the biped must exert internal
forces using actuators. In this Lesson, we will design one of the
simplest controllers to produce internal forces that make the biped
hold a target pose. The proportional-derivative (PD) control computes
control force by &Tau; = -k<sub>p</sub> (&theta; -
&theta;<sub>target</sub>) - k<sub>d</sub> &theta;&#775;, where &theta;
and &theta;&#775; are the current position and velocity of a degree of
freedom, &theta;<sub>target</sub> is the target position set by the
controller, and k<sub>p</sub> and k<sub>d</sub> are the stiffness and
damping coefficients. The detailed description of a PD controller can
be found [here](https://en.wikipedia.org/wiki/PID_controller).

The first task is to set the biped to a particular configuration. You
can use ``setPosition`` to set each degree of freedom individually:

```cpp
void setInitialPose(SkeletonPtr biped)
{
...
    biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
...
}
```
Here the degree of freedom named "j_thigh_left_z" is set to 0.15
radian. Note that each degree of freedom in a skeleton has a numerical
index which can be accessed by
``getIndexInSkeleton``. You
can also set the entire configuration using a vector that holds the
positions of all the degreed of freedoms using
``setPositions``.

We continue to set more degrees of freedoms in the lower
body to create a roughly stable standing pose.

```cpp
biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
biped->setPosition(biped->getDof("j_thigh_right_z")->getIndexInSkeleton(), 0.15);
biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);
biped->setPosition(biped->getDof("j_heel_left_1")->getIndexInSkeleton(), 0.25);
biped->setPosition(biped->getDof("j_heel_right_1")->getIndexInSkeleton(), 0.25);
```

Now the biped will start in this configuration, but will not maintain
this configuration as soon as the simulation starts. We need a
controller to make this happen. Let's take a look at the constructor of our ``Controller`` in the
skeleton code:

```cpp
Controller(const SkeletonPtr& biped)
{
...
    for(size_t i = 0; i < 6; ++i)
    {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }

    for(size_t i = 6; i < mBiped->getNumDofs(); ++i)
    {
        mKp(i, i) = 1000;
        mKd(i, i) = 50;
    }

    setTargetPositions(mBiped->getPositions());
}    
```

Here we arbitrarily define the stiffness and damping coefficients to
1000 and 50, except for the first six degrees of freedom. Because the
global translation and rotation of the biped are not actuated, the
first six degrees of freedom at the root do not exert any internal
force. Therefore, we set the stiffness and damping coefficients to
zero. At the end of the constructor, we set the target position of the PD
controller to the current configuration of the biped.

With these settings, we can compute the forces generated by the PD
controller and add them to the internal forces of biped using ``setForces``:

```cpp
void addPDForces()
{
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();
    
    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    
    mForces += p + d;
    mBiped->setForces(mForces);
}
```
Note that the PD control force is *added* to the current internal force
stored in mForces instead of overriding it.

Now try to run the program and see what happens. The skeleton
disappears almost immediately as soon as you hit the space bar! This
is because our stiffness and damping coefficients are set way too
high. As soon as the biped deviates from the target position, huge
internal forces are generated to cause the numerical simulation to
blow up.

So let's lower those coefficients a bit. It turns out that each of the
degrees of freedom needs to be individually tuned depending on many
factors, such as the inertial properties of the body nodes, the type
and properties of joints, and the current configuration of the
system. Figuring out an appropriate set of coefficients can be a
tedious process difficult to generalize across new tasks or different
skeletons. In the next Lesson, we will introduce a much more efficient
way to stabilize the PD controllers without endless tuning and
trial-and-errors.

# Lesson 3: Stable PD control

SPD is a variation of PD control proposed by
[Jie Tan](http://www.cc.gatech.edu/~jtan34/project/spd.html). The
basic idea of SPD is to compute control force using the predicted
state at the next time step, instead of the current state. This Lesson
will only demonstrate the implementation of SPD using DART without
going into details of SPD derivation.

The implementation of SPD involves accessing the current dynamic
quantities in Lagrange's equations of motion. Fortunately, these
quantities are readily available via DART API, which makes the full
implementation of SPD simple and concise:

```cpp
void addSPDForces()
{
    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();

    Eigen::MatrixXd invM = (mBiped->getMassMatrix() + mKd * mBiped->getTimeStep()).inverse();
    Eigen::VectorXd p = -mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot = invM * (-mBiped->getCoriolisAndGravityForces() + p + d + mBiped->getConstraintForces());

    mForces += p + d - mKd * qddot * mBiped->getTimeStep();
    mBiped->setForces(mForces);
}
```

You can get mass matrix, Coriolis force, gravitational force, and
constraint force projected onto generalized coordinates using function
calls ``getMassMatrix``,
``getCoriolisForces``,
``getGravityForces``,
and
``getConstraintForces``,
respectively. Constraint forces include forces due to contacts, joint
limits, and other joint constraints set by the user (e.g. the weld
joint constraint in the multi-pendulum tutorial).

With SPD, a wide range of stiffness and damping coefficients will all
result in stable motion. In fact, you can just leave them to our
original values: 1000 and 50. By holding the target pose, now the biped
can stand on the ground in balance indefinitely. However, if you apply
an external push force on the biped (hit ',' or '.' key to apply a
backward or forward push), the biped loses its balance quickly. We
will demonstrate a more robust feedback controller in the next Lesson.


# Lesson 4: Ankle strategy

Ankle (or hip) strategy is an effective way to maintain standing
balance. The idea is to adjust the target position of ankles according
to the deviation between the center of mass and the center of pressure
projected on the ground. A simple linear feedback rule is used to
update the target ankle position: &theta;<sub>a</sub> = -k<sub>p</sub>
(x - p) - k<sub>d</sub> (x&#775; - p&#775;), where x and p indicate the
center of mass and center of pressure in the anterior-posterior
axis. k<sub>p</sub> and k<sub>d</sub> are the feedback gains defined
by the user.

To implement ankle strategy, let's first compute the deviation between
the center of mass and an approximated center of pressure in the
anterior-posterior axis:

```cpp
void addAnkleStrategyForces()
{
    Eigen::Vector3d COM = mBiped->getCOM();
    Eigen::Vector3d offset(0.05, 0, 0);
    Eigen::Vector3d COP = mBiped->getBodyNode("h_heel_left")->getTransform() * offset;
    double diff = COM[0] - COP[0];
...
}
```

DART provides various APIs to access useful kinematic information. For
example, ``getCOM`` returns the center of mass of the skeleton and
``getTransform`` returns transformation of the body node with
respect to any coordinate frame specified by the parameter (world
coordinate frame as default). DART APIs also come in handy when
computing the derivative term,  -k<sub>d</sub> (x&#775; - p&#775;):

```cpp
void addAnkleStrategyForces()
{
...
    Eigen::Vector3d dCOM = mBiped->getCOMLinearVelocity();
    Eigen::Vector3d dCOP =  mBiped->getBodyNode("h_heel_left")->getLinearVelocity(offset);
    double dDiff = dCOM[0] - dCOP[0];
...
}
```

The linear/angular velocity/acceleration of any point in any coordinate
frame can be easily accessed in DART. The full list of the APIs for accessing
various velocities/accelerations can be found in the [API Documentation](http://dartsim.github.io/dart/). The 
following table summarizes the essential APIs.

| Function Name          | Description |
|------------------------|-------------|
| getSpatialVelocity     | Return the spatial velocity of this BodyNode in the coordinates of the BodyNode. |
| getLinearVelocity      | Return the linear portion of classical velocity of the BodyNode relative to some other BodyNode. |
| getAngularVelocity     | Return the angular portion of classical velocity of this BodyNode relative to some other BodyNode. |
| getSpatialAcceleration | Return the spatial acceleration of this BodyNode in the coordinates of the BodyNode. |
| getLinearAcceleration  | Return the linear portion of classical acceleration of the BodyNode relative to some other BodyNode. |
| getAngularAcceleration | Return the angular portion of classical acceleration of this BodyNode relative to some other BodyNode. |

The remaining of the ankle strategy implementation is just the matter
of parameters tuning. We found that using different feedback rules for
falling forward and backward result in more stable controller.

# Lesson 5: Skeleton editing

DART provides various functions to copy, delete, split, and merge
parts of skeletons to alleviate the pain of building simulation models from
scratch. In this Lesson, we will load a skateboard model from a SKEL
file and merge our biped with the skateboard to create a wheel-based
robot.

We first load a skateboard from **skateboard.skel**:

```cpp
void modifyBipedWithSkateboard(SkeletonPtr biped)
{
    WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/skateboard.skel");
    SkeletonPtr skateboard = world->getSkeleton(0);
...
}
```

Our goal is to make the skateboard Skeleton a subtree of the biped
Skeleton connected to the left heel BodyNode via a newly created
Euler joint. To do so, you need to first create an instance of
``EulerJoint::Properties`` for this new joint.

```cpp
void modifyBipedWithSkateboard(SkeletonPtr biped)
{
...
    EulerJoint::Properties properties = EulerJoint::Properties();
    properties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0, 0.1, 0);
...
}
```

Here we increase the vertical distance between the child BodyNode and
the joint by 0.1m to give some space between the skateboard and the
left foot. Now you can merge the skateboard and the biped using this new Euler
joint by

```cpp
void modifyBipedWithSkateboard(SkeletonPtr biped)
{
...
    skateboard->getRootBodyNode()->moveTo<EulerJoint>(biped->getBodyNode("h_heel_left"), properties);
}
```

There are many other functions you can use to edit skeletons. Here is
a table of some relevant functions for quick references.

| Function Name         |      Example                                  | Description |
|-----------------------|-----------------------------------------------|-------------|
| remove                | bd1->remove()                                 | Remove the BodyNode bd1 and its subtree from their Skeleton. |
| moveTo                | bd1->moveTo(bd2)                              | Move the BodyNode bd1 and its subtree under the BodyNode bd2. |
| split                 | auto newSkel = bd1->split("new skeleton")`    | Remove the BodyNode bd1 and its subtree from their current Skeleton and move them into a newly created Skeleton with "new skeleton" name. |
| changeParentJointType | bd1->changeParentJointType&lt;BallJoint&gt;() | Change the Joint type of the BodyNode bd1's parent joint to BallJoint |
| copyTo                | bd1->copyTo(bd2)                              | Create clones of the BodyNode bd1 and its subtree and attach the clones to the specified the BodyNode bd2. |
| copyAs                | auto newSkel = bd1->copyAs("new skeleton")    | Create clones of the BodyNode bd1 and its subtree and create a new Skeleton with "new skeleton" name to attach them to. |


# Lesson 6: Actuator types

DART provides five types of actuator. Each joint can select its own
actuator type.

| Type         | Description |
|--------------|-------------|
| FORCE        | Take joint force and return the resulting joint acceleration. |
| PASSIVE      | Take nothing (joint force = 0) and return the resulting joint acceleration. |
| ACCELERATION | Take desired joint acceleration and return the joint force to achieve the acceleration.  |
| VELOCITY     | Take desired joint velocity and return the joint force to achieve the velocity. |
| LOCKED       | Lock the joint by setting the joint velocity and acceleration to zero and return the joint force to lock the joint. |

In this Lesson, we will switch the actuator type of the wheels
from the default FORCE type to VELOCITY type.

```cpp
void setVelocityAccuators(SkeletonPtr biped)
{
    Joint* wheel1 = biped->getJoint("joint_front_left");
    wheel1->setActuatorType(Joint::VELOCITY);
...
}
```

Once all four wheels are set to VELOCITY actuator type, you can
command them by directly setting the desired velocity:

```cpp
void setWheelCommands()
{
...
    int index1 = mBiped->getDof("joint_front_left_2")->getIndexInSkeleton();
    mBiped->setCommand(index1, mSpeed);
...
}
```

Note that ``setCommand`` only exerts commanding force in the current time step. If you wish the
wheel to continue spinning at a particular speed, ``setCommand``
needs to be called at every time step.

We also set the stiffness and damping coefficients for the wheels to zero.

```cpp
void setWheelCommands()
{
    int wheelFirstIndex = mBiped->getDof("joint_front_left_1")->getIndexInSkeleton();
    for (size_t i = wheelFirstIndex; i < mBiped->getNumDofs(); ++i)
    {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
...
}
```

This is because we do not want the velocity-based actuators to
incorrectly affect the computation of SPD. If we use simple PD
control scheme, the values of these spring and damping coefficients do not
affect the dynamics of the system.

Let's simulate what we've got so far. The biped now is connecting to the
skateboard through a Euler joint. Once the simulation starts, you can
use 'a' and 's' to increase or decrease the wheel speed. However, the
biped falls on the floor immediately because the current target pose is not
balanced for one-foot stance. We need to find a better target
pose.

# Lesson 7: Inverse kinematics

Instead of manually designing a target pose, this time we will solve for
a balanced pose by formulating an inverse kinematics (IK) problem and
solving it using gradient descent method. In this example, a balanced
pose is defined as a pose where the center of mass is well supported
by the ground contact and the left foot lies flat on the ground. As
such, we cast IK as an optimization problem that minimizes the
horizontal deviation between the center of mass and the center of the
left foot, as well as the vertical distance of the four corners of the
left foot from the ground:

<img src="IKObjective.png" width="180">

where <b>c</b> and <b>p</b> indicate the projected center of mass and center of
pressure on the ground, and *p<sub>i</sub>* indicates the vertical height of one
corner of the left foot.

To compute the gradient of the above objective function, we need to evaluate
the partial derivatives of each objective term with respect to the
degrees of freedom, i.e., the computation of Jacobian matrix. DART
provides a comprensive set of APIs for accessing various types of
Jacobian. In this example, computing the gradient of the first term of
the objective function requires the Jacobian of the
center of mass of the Skeleton, as well as the Jacobian of the center
of mass of a BodyNode:

```cpp
Eigen::VectorXd solveIK(SkeletonPtr biped)
{
...
    Eigen::Vector3d localCOM = leftHeel->getCOM(leftHeel);
    LinearJacobian jacobian = biped->getCOMLinearJacobian() - biped->getLinearJacobian(leftHeel, localCOM);
...
}
```

``getCOMLinearJacobian`` returns the linear Jacobian of the
center of mass of the Skeleton, while ``getLinearJacobian``
returns the Jacobian of a point on a BodyNode. The BodyNode and the
local coordinate of the point are specified as parameters to this
function. Here the point of interest is the center of mass of the left
foot, which local coordinates can be accessed by ``getCOM``
with a parameter indicating the left foot being the frame of
reference. We use ``getLinearJacobian`` again to compute the
gradient of the second term of the objective function:

```cpp
Eigen::VectorXd solveIK(SkeletonPtr biped)
{
...
    Eigen::Vector3d offset(0.0, -0.04, -0.03);
    gradient = biped->getLinearJacobian(leftHeel, offset).row(1);
...
}
```

The full list of Jacobian APIs can be found in the [API Documentation](http://dartsim.github.io/dart/). The 
following table summarizes the essential APIs.

| Function Name           | Description |
|-------------------------|-------------|
| getJacobian             | Return the generalized Jacobian targeting the origin of the BodyNode. The Jacobian is expressed in the Frame of this BodyNode. |
| getLinearJacobian       | Return the linear Jacobian targeting the origin of the BodyNode. You can specify a coordinate Frame to express the Jacobian in. |
| getAngularJacobian      | Return the angular Jacobian targeting the origin of the BodyNode. You can specify a coordinate Frame to express the Jacobian in. |
| getJacobianSpatialDeriv | Return the spatial time derivative of the generalized Jacobian targeting the origin of the BodyNode. The Jacobian is expressed in the BodyNode's coordinate Frame. |
| getJacobianClassicDeriv | Return the classical time derivative of the generalized Jacobian targeting the origin of the BodyNode. The Jacobian is expressed in the World coordinate Frame. |
| getLinearJacobianDeriv  | Return the linear Jacobian (classical) time derivative, in terms of any coordinate Frame. |
| getAngularJacobianDeriv | Return the angular Jacobian (classical) time derivative, in terms of any coordinate Frame. |

This Lesson concludes the entire Biped tutorial. You should see a biped
standing stably on the skateboard. With moderate
acceleration/deceleration on the skateboard, the biped is able to
maintain balance and hold the one-foot stance pose.

<div id="fb-root"></div>
<script>(function(d, s, id) {
  var js, fjs = d.getElementsByTagName(s)[0];
  if (d.getElementById(id)) return;
  js = d.createElement(s); js.id = id;
  js.src = "//connect.facebook.net/en_US/sdk.js#xfbml=1&version=v2.4";
  fjs.parentNode.insertBefore(js, fjs);
}(document, 'script', 'facebook-jssdk'));</script>

<div class="fb-like" data-href="http://dart.readthedocs.org/en/release-5.1/tutorials/biped/" data-layout="button_count" data-action="like" data-show-faces="true" data-share="true"></div>

<div id="disqus_thread"></div>
<script type="text/javascript">
    /* * * CONFIGURATION VARIABLES * * */
    var disqus_shortname = 'dartsim';
    
    /* * * DON'T EDIT BELOW THIS LINE * * */
    (function() {
        var dsq = document.createElement('script'); dsq.type = 'text/javascript'; dsq.async = true;
        dsq.src = '//' + disqus_shortname + '.disqus.com/embed.js';
        (document.getElementsByTagName('head')[0] || document.getElementsByTagName('body')[0]).appendChild(dsq);
    })();
</script>
<noscript>Please enable JavaScript to view the <a href="https://disqus.com/?ref_noscript" rel="nofollow">comments powered by Disqus.</a></noscript>
