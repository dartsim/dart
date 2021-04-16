# Overview

This tutorial will demonstrate some of the more advanced features of DART's
dynamics API which allow you to write robust controllers that work for real
dynamic systems, such as robotic manipulators. We will show you how to:

- Clone Skeletons
- Load a URDF
- Write a stable PD controller w/ gravity and coriolis compensation
- Write an operational space controller

Please reference the source code in [**tutorialDominoes.cpp**](https://github.com/dartsim/dart/blob/release-5.1/tutorials/tutorialDominoes.cpp) and [**tutorialDominoes-Finished.cpp**](https://github.com/dartsim/dart/blob/release-5.1/tutorials/tutorialDominoes-Finished.cpp).

# Lesson 1: Cloning Skeletons

There are often times where you might want to create an exact replica of an
existing Skeleton. DART offers cloning functionality that allows you to do this
very easily.

### Lesson 1a: Create a new domino

Creating a new domino is straightforward. Find the function ``attemptToCreateDomino``
in the ``MyWindow`` class. The class has a member called ``mFirstDomino`` which
is the original domino created when the program starts up. To make a new one,
we can just clone it:

```cpp
SkeletonPtr newDomino = mFirstDomino->clone();
```

But keep in mind that every Skeleton that gets added to a world requires its own
unique name. Creating a clone will keep the original name, so we should we give
the new copy its own name:

```cpp
newDomino->setName("domino #" + std::to_string(mDominoes.size() + 1));
```

So the easy part is finished, but now we need to get the domino to the correct
position. First, let's grab the last domino that was placed in the environment:

```cpp
const SkeletonPtr& lastDomino = mDominoes.size() > 0 ?
      mDominoes.back() : mFirstDomino;
```

Now we should compute what we want its position to be. The ``MyWindow`` class
keeps a member called ``mTotalAngle`` which tracks how much the line of dominoes
has turned so far. We'll use that to figure out what translational offset the
new domino should have from the last domino:

```cpp
Eigen::Vector3d dx = default_distance * Eigen::Vector3d(
      cos(mTotalAngle), sin(mTotalAngle), 0.0);
```

And now we can compute the total position of the new domino. First, we'll copy
the positions of the last domino:

```cpp
Eigen::Vector6d x = lastDomino->getPositions();
```

And then we'll add the translational offset to it:

```cpp
x.tail<3>() += dx;
```

Remember that the domino's root joint is a FreeJoint which has six degrees of
freedom: the first three are for orientation and last three are for translation.

Finally, we should add on the change in angle for the new domino:

```cpp
x[2] = mTotalAngle + angle;
```

Be sure to uncomment the ``angle`` argument of the function.

Now we can use ``x`` to set the positions of the domino:

```cpp
newDomino->setPositions(x);
```

The root FreeJoint is the only joint in the domino's Skeleton, so we can just
use the ``Skeleton::setPositions`` function to set it.

Now we'll add the Skeleton to the world:

```cpp
mWorld->addSkeleton(newDomino);
```

### Lesson 1b: Make sure no dominoes are in collision

Similar to **Lesson 3** of the **Collisions** tutorial, we'll want to make sure
that the newly inserted Skeleton is not starting out in collision with anything,
because this could make for a very ugly (perhaps even broken) simulation. 

First, we'll tell the world to compute collisions:

```cpp
dart::collision::CollisionDetector* detector =
    mWorld->getConstraintSolver()->getCollisionDetector();
detector->detectCollision(true, true);
```

Now we'll look through and see if any dominoes are in collision with anything
besides the floor. We ignore collisions with the floor because, mathemetically
speaking, if they are in contact with the floor then they register as being in
collision. But we want the dominoes to be in contact with the floor, so this is
okay.

```cpp
bool dominoCollision = false;
size_t collisionCount = detector->getNumContacts();
for(size_t i = 0; i < collisionCount; ++i)
{
  // If neither of the colliding BodyNodes belongs to the floor, then we
  // know the new domino is in contact with something it shouldn't be
  const dart::collision::Contact& contact = detector->getContact(i);
  if(contact.bodyNode1.lock()->getSkeleton() != mFloor
     && contact.bodyNode2.lock()->getSkeleton() != mFloor)
  {
    dominoCollision = true;
    break;
  }
}
```

The only object that could possibly have collided with something else is the
new domino, because we don't allow the application to create new things except
for the dominoes. So if this registered as true, then we should take the new
domino out of the world:

```cpp
if(dominoCollision)
{
  // Remove the new domino, because it is penetrating an existing one
  mWorld->removeSkeleton(newDomino);
}
```

Otherwise, if the new domino is in an okay position, we should add it to the
history:

```cpp
else
{
  // Record the latest domino addition
  mAngles.push_back(angle);
  mDominoes.push_back(newDomino);
  mTotalAngle += angle;
}
```

### Lesson 1c: Delete the last domino added

Ordinarily, removing a Skeleton from a scene is just a matter of calling the
``World::removeSkeleton`` function, but we have a little bit of bookkeeping to
take care of for our particular application. First, we should check whether
there are any dominoes to actually remove:

```cpp
if(mDominoes.size() > 0)
{
  // TODO: Remove Skeleton
}
```

Then we should grab the last domino in the history, remove it from the history,
and then take it out of the world:

```cpp
SkeletonPtr lastDomino = mDominoes.back();
mDominoes.pop_back();
mWorld->removeSkeleton(lastDomino);
```

The ``SkeletonPtr`` class is really a ``std::shared_ptr<Skeleton>`` so we don't
need to worry about ever calling ``delete`` on it. Instead, its resources will
be freed when ``lastDomino`` goes out of scope.

We should also make sure to do the bookkeepping for the angles:

```cpp
mTotalAngle -= mAngles.back();
mAngles.pop_back();
```

**Now we can add and remove dominoes from the scene. Feel free to give it a try.**

### Lesson 1d: Apply a force to the first domino

But just setting up dominoes isn't much fun without being able to knock them
down. We can quickly and easily knock down the dominoes by magically applying
a force to the first one. In the ``timeStepping`` function of ``MyWindow`` there
is a label for **Lesson 1d**. This spot will get visited whenever the user
presses 'f', so we'll apply an external force to the first domino here:

```cpp
Eigen::Vector3d force = default_push_force * Eigen::Vector3d::UnitX();
Eigen::Vector3d location =
    default_domino_height / 2.0 * Eigen::Vector3d::UnitZ();
mFirstDomino->getBodyNode(0)->addExtForce(force, location);
```

# Lesson 2: Loading and controlling a robotic manipulator

Striking something with a magical force is convenient, but not very believable.
Instead, let's load a robotic manipulator and have it push over the first domino.

### Lesson 2a: Load a URDF file

Our manipulator is going to be loaded from a URDF file. URDF files are loaded
by the ``dart::io::DartLoader`` class (pending upcoming changes to DART's
loading system). First, create a loader:

```cpp
dart::io::DartLoader loader;
```

Note that many URDF files use ROS's ``package:`` scheme to specify the locations
of the resources that need to be loaded. We won't be using this in our example,
but in general you should use the function ``DartLoader::addPackageDirectory``
to specify the locations of these packages, because DART does not have the same
package resolving abilities of ROS.

Now we'll have ``loader`` parse the file into a Skeleton:

```cpp
SkeletonPtr manipulator =
    loader.parseSkeleton(DART_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf");
```

And we should give the Skeleton a convenient name:

```cpp
manipulator->setName("manipulator");
```

Now we'll want to initialize the location and configuration of the manipulator.
Experimentation has demonstrated that the following setup is good for our purposes:

```cpp
// Position its base in a reasonable way
Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
tf.translation() = Eigen::Vector3d(-0.65, 0.0, 0.0);
manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

// Get it into a useful configuration
manipulator->getDof(1)->setPosition(140.0 * M_PI / 180.0);
manipulator->getDof(2)->setPosition(-140.0 * M_PI / 180.0);
```

And lastly, be sure to return the Skeleton that we loaded rather than the dummy
Skeleton that was originally there:

```cpp
return manipulator;
```

**Feel free to load up the application to see the manipulator in the scene,
although all it will be able to do is collapse pitifully onto the floor.**

### Lesson 2b: Grab the desired joint angles

To make the manipulator actually useful, we'll want to have the ``Controller``
control its joint forces. For it to do that, the ``Controller`` class will need
to be informed of what we want the manipulator's joint angles to be. This is 
easily done in the constructor of the ``Controller`` class:

```cpp
mQDesired = mManipulator->getPositions();
```

The function ``Skeleton::getPositions`` will get all the generalized coordinate
positions of all the joints in the Skeleton, stacked in a single vector. These
Skeleton API functions are useful when commanding or controlling an entire
Skeleton with a single mathematical expression.

### Lesson 2c: Write a stable PD controller for the manipulator

Now that we know what configuration we want the manipulator to hold, we can
write a PD controller that keeps them in place. Find the function ``setPDForces``
in the ``Controller`` class.

First, we'll grab the current positions and velocities:

```cpp
Eigen::VectorXd q = mManipulator->getPositions();
Eigen::VectorXd dq = mManipulator->getVelocities();
```

Additionally, we'll integrate the position forward by one timestep:

```cpp
q += dq * mManipulator->getTimeStep();
```

This is not necessary for writing a regular PD controller, but instead this is
to write a "stable PD" controller which has some better numerical stability
properties than an ordinary discrete PD controller. You can try running with and
without this line to see what effect it has on the stability.

Now we'll compute our joint position error:

```cpp
Eigen::VectorXd q_err = mQDesired - q;
```

And our joint velocity error, assuming our desired joint velocity is zero:

```cpp
Eigen::VectorXd dq_err = -dq;
```

Now we can grab our mass matrix, which we will use to scale our force terms:

```cpp
const Eigen::MatrixXd& M = mManipulator->getMassMatrix();
```

And then combine all this into a PD controller that computes forces to minimize
our error:

```cpp
mForces = M * (mKpPD * q_err + mKdPD * dq_err);
```

Now we're ready to set these forces on the manipulator:

```cpp
mManipulator->setForces(mForces);
```

**Feel free to give this PD controller a try to see how effective it is.**

### Lesson 2d: Compensate for gravity and Coriolis forces

One of the key features of DART is the ability to easily compute the gravity and
Coriolis forces, allowing you to write much higher quality controllers than you
would be able to otherwise. This is easily done like so:

```cpp
const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();
```

And now we can update our control law by just slapping this term onto the end
of the equation:

```cpp
mForces = M * (mKpPD * q_err + mKdPD * dq_err) + Cg;
```

**Give this new PD controller a try to see how its performance compares to the
one without compensation**

# Lesson 3: Writing an operational space controller

While PD controllers are simply and handy, operational space controllers can be
much more elegant and useful for performing tasks. Operational space controllers
allow us to unify geometric tasks (like getting the end effector to a particular
spot) and dynamics tasks (like applying a certain force with the end effector)
all while remaining stable and smooth.

### Lesson 3a: Set up the information needed for an OS controller

Unlike PD controllers, an operational space controller needs more information
than just desired joint angles.

First, we'll grab the last BodyNode on the manipulator and treat it as an end
effector:

```cpp
mEndEffector = mManipulator->getBodyNode(mManipulator->getNumBodyNodes() - 1);
```

But we don't want to use the origin of the BodyNode frame as the origin of our
Operational Space controller; instead we want to use a slight offset, to get to
the tool area of the last BodyNode:

```cpp
mOffset = default_endeffector_offset * Eigen::Vector3d::UnitX();
```

Also, our target will be the spot on top of the first domino, so we'll create a
reference frame and place it there. First, create the SimpleFrame:

```cpp
mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target");
```

Then compute the transform needed to get from the center of the domino to the
top of the domino:

```cpp
Eigen::Isometry3d target_offset(Eigen::Isometry3d::Identity());
target_offset.translation() =
    default_domino_height / 2.0 * Eigen::Vector3d::UnitZ();
```

And then we should rotate the target's coordinate frame to make sure that lines
up with the end effector's reference frame, otherwise the manipulator might try
to push on the domino from a very strange angle:

```cpp
target_offset.linear() =
    mEndEffector->getTransform(domino->getBodyNode(0)).linear();
```

Now we'll set the target so that it has a transform of ``target_offset`` with
respect to the frame of the domino:

```cpp
mTarget->setTransform(target_offset, domino->getBodyNode(0));
```

And this gives us all the information we need to write an Operational Space
controller.

### Lesson 3b: Computing forces for OS Controller

Find the function ``setOperationalSpaceForces()``. This is where we'll compute
the forces for our operational space controller.

One of the key ingredients in an operational space controller is the mass matrix.
We can get this easily, just like we did for the PD controller:

```cpp
const Eigen::MatrixXd& M = mManipulator->getMassMatrix();
```

Next we'll want the Jacobian of the tool offset in the end effector. We can get
it easily with this function:

```cpp
Jacobian J = mEndEffector->getWorldJacobian(mOffset);
```

But operational space controllers typically use the Moore-Penrose pseudoinverse
of the Jacobian rather than the Jacobian itself. There are many ways to compute
the pseudoinverse of the Jacobian, but a simple way is like this:

```cpp
Eigen::MatrixXd pinv_J = J.transpose() * (J * J.transpose()
                       + 0.0025 * Eigen::Matrix6d::Identity()).inverse();
```

Note that this pseudoinverse is also damped so that it behaves better around
singularities. This is method for computing the pseudoinverse is not very
efficient in terms of the number of mathematical operations it performs, but
it is plenty fast for our application. Consider using methods based on Singular
Value Decomposition if you need to compute the pseudoinverse as fast as possible.

Next we'll want the time derivative of the Jacobian, as well as its pseudoinverse:

```cpp
// Compute the Jacobian time derivative
Jacobian dJ = mEndEffector->getJacobianClassicDeriv(mOffset);

// Comptue the pseudo-inverse of the Jacobian time derivative
Eigen::MatrixXd pinv_dJ = dJ.transpose() * (dJ * dJ.transpose()
                        + 0.0025 * Eigen::Matrix6d::Identity()).inverse();
```

Notice that here we're compute the **classic** derivative, which means the
derivative of the Jacobian with respect to time in classical coordinates rather
than spatial coordinates. If you use spatial vector arithmetic, then you'll want
to use ``BodyNode::getJacobianSpatialDeriv`` instead.

Now we can compute the linear components of error:

```cpp
Eigen::Vector6d e;
e.tail<3>() = mTarget->getWorldTransform().translation()
            - mEndEffector->getWorldTransform() * mOffset;
```

And then the angular components of error:

```cpp
Eigen::AngleAxisd aa(mTarget->getTransform(mEndEffector).linear());
e.head<3>() = aa.angle() * aa.axis();
```

Then the time derivative of error, assuming our desired velocity is zero:

```cpp
Eigen::Vector6d de = -mEndEffector->getSpatialVelocity(
      mOffset, mTarget.get(), Frame::World());
```

Like with the PD controller, we can mix in terms to compensate for gravity and
Coriolis forces:

```cpp
const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();
```

The gains for the operational space controller need to be in matrix form, but
we're storing the gains as scalars, so we'll need to conver them:

```cpp
Eigen::Matrix6d Kp = mKpOS * Eigen::Matrix6d::Identity();

size_t dofs = mManipulator->getNumDofs();
Eigen::MatrixXd Kd = mKdOS * Eigen::MatrixXd::Identity(dofs, dofs);
```

And we'll need to compute the joint forces needed to achieve our desired end
effector force. This is easily done using the Jacobian transpose:

```cpp
Eigen::Vector6d fDesired = Eigen::Vector6d::Zero();
fDesired[3] = default_push_force;
Eigen::VectorXd f = J.transpose() * fDesired;
```

And now we can mix everything together into the single control law:

```cpp
Eigen::VectorXd dq = mManipulator->getVelocities();
mForces = M * (pinv_J * Kp * de + pinv_dJ * Kp * e)
          - Kd * dq + Kd * pinv_J * Kp * e + Cg + f;
```

Then don't forget to pass the forces into the manipulator:

```cpp
mManipulator->setForces(mForces);
```

**Now you're ready to try out the full dominoes app!**

<div id="fb-root"></div>
<script>(function(d, s, id) {
  var js, fjs = d.getElementsByTagName(s)[0];
  if (d.getElementById(id)) return;
  js = d.createElement(s); js.id = id;
  js.src = "//connect.facebook.net/en_US/sdk.js#xfbml=1&version=v2.4";
  fjs.parentNode.insertBefore(js, fjs);
}(document, 'script', 'facebook-jssdk'));</script>

<div class="fb-like" data-href="http://dart.readthedocs.org/en/release-5.1/tutorials/dominoes/" data-layout="button_count" data-action="like" data-show-faces="true" data-share="true"></div>

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
