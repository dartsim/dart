# Overview
This tutorial demonstrates the dynamic features in DART to facilitate
the development of controllers for bipedal and wheel-based robots.

The tutorial consists of seven Lessons covering the following
topics:

- SKEL  file format.
- Joint limits and self-collision.
- Actuators types and management.
- APIs for Jacobian matrices and other kinematic quantities.
- APIs for dynamic quantities.
- Skeleton editing.

# Lesson 1: Joint limits and self-collision

Let's start by opening the skeleton code
[tutorialBiped.cpp](http://). After creating a floor in
[main()](http://), [loadBiped()](http://) is called and a bipedal figure described
in SKEL format is loaded. Each SKEL file describes a
[World](http://dartsim.github.io/dart/d7/d41/classdart_1_1simulation_1_1World.html). Here
we load in a World from biped.skel and assign the bipedal figure to a
[Skeleton](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html)
pointer called *biped*.


    WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/biped.skel");
    assert(world != nullptr);

    SkeletonPtr biped = world->getSkeleton("biped");

Running the skeleton code without any modification, you should see a
human-like character dropping on the ground and collapsing into
itself. Therefore, the first task in this Lesson is to enforce more realistic
joint limits.

DART allows the user to set upper and lower bounds on each degree of
freedom in the SKEL file or using provided APIs. In biped.skel, you
should see the description of the right knee joint in this format:

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

The &lt;upper> and &lt;lower> tags make sure that the knee can only flex but
not extend in one direction. Alternatively, you can directly specify the joint limits
in the code using
[setPositionUpperLimit](http://dartsim.github.io/dart/d6/d5b/classdart_1_1dynamics_1_1Joint.html#aa0635643a0a8c1f22edb8243e86ea801)
and [setPositionLowerLimit](http://dartsim.github.io/dart/d6/d5b/classdart_1_1dynamics_1_1Joint.html#adadee231309b62cd3e3d904f75f2a969).

In either case, the joint limits on the biped will not be activated until you call [setPositionLimited](http://dartsim.github.io/dart/d6/d5b/classdart_1_1dynamics_1_1Joint.html#a3212ca5f7893cfd9a5422ab17df4038b)

    for(size_t i = 0; i < biped->getNumJoints(); ++i)
        biped->getJoint(i)->setPositionLimited(true);

Once the joint limits are set, the next task is to enforce
self-collision. By default, DART does not check self-collision within
a skeleton. You can enable self-collision checking on the biped by

    biped->enableSelfCollision();

This function will enable self-collision on every non-adjacent pair of
body nodes. If you wish to also enable self-collision on adjacent body
nodes, set the optional parameter to true:

    biped->enableSelfCollision(true);

Running the program again, you should see that the character is still
floppy like a ragdoll, but now the joints do not bend backward and the
body nodes do not penetrate each other anymore.

# Lesson 2: Proportional-derivative (PD) control

To actively control its own motion, the biped must exert internal
forces using actuators. In this Lesson, we will design one of the
simplest controllers to produce internal forces that make the biped
hold a target pose. The proportional-derivative (PD) control has
the form of &Tau; = -k<sub>p</sub> (&theta; -
&theta;<sub>target</sub>) - k<sub>d</sub> &theta;&#775;, where &theta;
and &theta;&#775; are the current position and velocity of a degree of
freedom, &theta;<sub>target</sub> is the target position set by the
controller, and k<sub>p</sub> and k<sub>d</sub> are the stiffness and
damping coefficients. The detailed description of a PD controller can
be found [here](https://en.wikipedia.org/wiki/PID_controller).

The first task is to set the biped to a desired pose in
[setInitialPose()](htt[:\\). To set
each degree of freedom individually, you can use [setPosition](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html#ac2036ea4998f688173d19ace0edab841):

     biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);

Here the degree of freedom named "j_thigh_left_z" is set to 0.15
radian. Note that each degree of freedom in a skeleton has a numerical
index which can be obtained by
[getIndexInSkeleton](http://dartsim.github.io/dart/de/db7/classdart_1_1dynamics_1_1DegreeOfFreedom.html#add2ec1d2f979e9056b466b1be5ee1a86). You
can also set the entire configuration using a vector that holds the
positions of all the degreed of freedoms using
[setPositions](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html#aee6d1a2be46c277602fae2f1d47762ef).

Let's set more degrees of freedoms in the lower
body to create a roughly stable standing pose.

    biped->setPosition(biped->getDof("j_thigh_left_z")->getIndexInSkeleton(), 0.15);
    biped->setPosition(biped->getDof("j_thigh_right_z")->getIndexInSkeleton(), 0.15);
    biped->setPosition(biped->getDof("j_shin_left")->getIndexInSkeleton(), -0.4);
    biped->setPosition(biped->getDof("j_shin_right")->getIndexInSkeleton(), -0.4);
    biped->setPosition(biped->getDof("j_heel_left_1")->getIndexInSkeleton(), 0.25);
    biped->setPosition(biped->getDof("j_heel_right_1")->getIndexInSkeleton(), 0.25);

Now let's take a look at the constructor of our Controller in the
skeleton code:

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
    ...
    
Here we arbitrarily define the stiffness and damping coefficients to
1000 and 50, except for the first six degrees of freedom. Because the
global translation and rotation of the biped are not actuated, the
first six degrees of freedom at the root should not exert any internal
force. Therefore, we set the stiffness and damping coefficients to
zero.

At the end of the constructor, we set the target position of the PD
controller to the current configuration of the biped:

    setTargetPositions(mBiped->getPositions());

[Controller::addForces()](http://) computes the forces generated by the PD
controller and adds these forces to the internal forces of biped using [setForces](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html#a9a6a9b792fa39639d3af613907d2d8ed):

    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();
    
    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    
    mForces += p + d;
    mBiped->setForces(mForces);

Note that the PD control force is *added* to the current internal force
stored in mForces instead of overriding it.

Now try to run the program and see what happens. The skeleton
disappears almost immediately as soon as you hit the space bar! This
is because our stiffness and damping coefficients are set way too
high. As soon as the biped deviates from the target position, huge
internal forces are generated to cause the numerical simulation to
blow up.

So let's lower those coefficients a bit. It turns out that each of
the degrees of freedom needs to be individually tuned depending on the
many factors such as the inertial properties of the body nodes, the
type of joints, and the current configuration of the system. Figuring
out an appropriate set of coefficients can be a tedious process
difficult to generalize across new tasks or different skeletons. In
the next Lesson, we will introduce a much more efficient way to
stabilize the PD controllers without endless tuning and
trial-and-errors.

# Lesson 3: Stable proportional-derivative (SPD) control

SPD is a variation of PD control proposed by
[Jie Tan](http://www.cc.gatech.edu/~jtan34/project/spd.html). The
basic idea of SPD is to compute control force using the predicted
state at the next time step, instead of the current state. This Lesson
will only demonstrates the implementation of SPD using DART.

The implementation of SPD involves accessing dynamic quantities in Lagrange's equations of motion. Fortunately, these quantities are readily available via DART APIs. [Controller::addSPDForces()](\http://) shows the full implementation of SPD:

    Eigen::VectorXd q = mBiped->getPositions();
    Eigen::VectorXd dq = mBiped->getVelocities();

    Eigen::MatrixXd invM = (mBiped->getMassMatrix() + mKd * mBiped->getTimeStep()).inverse();
    Eigen::VectorXd p = -mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot = invM * (-mBiped->getCoriolisAndGravityForces() + p + d + mBiped->getConstraintForces());

    mForces += p + d - mKd * qddot * mBiped->getTimeStep();
    mBiped->setForces(mForces);

You can get mass matrix, Coriolis force, gravitational force, and
constraint force projected onto generalized coordinates using function
calls [getMassMatrix](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html#a1998cb27dd892d259da109509f313830),
[getCoriolisForces](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html#aeffe03aff506e206f79c5074b3886f08),
[getGravityForces](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html#a0d278dc0365a99729fdbbee7acf0bcd3),
and
[getConstraintForces](http://dartsim.github.io/dart/d3/d19/classdart_1_1dynamics_1_1Skeleton.html#a4b46912a4f3966efb2e54f1f5a29a77b),
respectively. Constraint forces include forces due to contacts, joint
limits, and other joint constraints set by the user (e.g. the weld
joint constraint in the multi-pendulum tutorial).
