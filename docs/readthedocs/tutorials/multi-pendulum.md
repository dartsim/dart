# Overview

This tutorial will demonstrate some basic interaction with DART's dynamics
API during simulation. This will show you how to:

- Create a basic program to simulate a dynamic system
- Change the colors of shapes
- Add/remove shapes from visualization
- Apply internal forces in the joints
- Apply external forces to the bodies
- Alter the implicit spring and damping properties of joints
- Add/remove dynamic constraints

Please reference the source code in [**tutorialMultiPendulum.cpp**](https://github.com/dartsim/dart/blob/release-5.1/tutorials/tutorialMultiPendulum.cpp) and [**tutorialMultiPendulum-Finished.cpp**](https://github.com/dartsim/dart/blob/release-5.1/tutorials/tutorialMultiPendulum-Finished.cpp).

# Lesson 0: Simulate a passive multi-pendulum

This is a warmup lesson that demonstrates how to set up a simulation
program in DART. The example we will use throughout this tutorial is a
pendulum with five rigid bodies swinging under gravity. DART allows
the user to build various articulated rigid/soft body systems from
scratch. It also loads models in URDF, SDF, and SKEL formats as
demonstrated in the later tutorials.

In DART, an articulated dynamics model is represented by a
``Skeleton``. In the ``main`` function, we first create an empty
skeleton named *pendulum*.

```cpp
SkeletonPtr pendulum = Skeleton::create("pendulum");
```

A Skeleton is a structure that consists of ``BodyNode``s (bodies) which are 
connected by ``Joint``s. Every Joint has a child BodyNode, and every BodyNode 
has a parent Joint. Even the root BodyNode has a Joint that attaches it to the 
World. In the function ``makeRootBody``, we create a pair of a
``BallJoint``  and a BodyNode, and attach this pair to the currently
empty pendulum skeleton.

```cpp
BodyNodePtr bn = pendulum->createJointAndBodyNodePair<BallJoint>(
      nullptr, properties, BodyNode::AspectProperties(name)).second;
```

Note that the first parameters is a nullptr, which indicates that
this new BodyNode is the root of the pendulum. If we wish to append
the new BodyNode to an existing BodyNode in the pendulum,
we can do so by passing the pointer of the existing BodyNode as
the first parameter. In fact, this is how we add more BodyNodes to
the pendulum in the function ``addBody``:

```cpp
BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
      parent, properties, BodyNode::AspectProperties(name)).second;
```
The simplest way to set up a simulation program in DART is to use
``SimWindow`` class. A SimWindow owns an instance of ``World``  and
simulates all the Skeletons in the World. In this example, we create a World with the
pendulum skeleton in it, and assign the World to an instance of
``MyWindow``, a subclass derived from SimWindow.

```cpp
WorldPtr world(new World);
world->addSkeleton(pendulum);
MyWindow window(world);
```

Every single time step, the ``MyWindow::timeStepping`` function will be called
and the state of the World will be simulated. The user can override
the default timeStepping function to customize the simulation
routine. For example, one can incorporate sensors, actuators, or user
interaction in the forward simulation.


# Lesson 1: Change shapes and applying forces

We have a pendulum with five bodies, and we want to be able to apply forces to
them during simulation. Additionally, we want to visualize these forces so we
can more easily interpret what is happening in the simulation. For this reason,
we'll discuss visualizing and forces at the same time.

### Lesson 1a: Reset everything to default appearance

At each step, we'll want to make sure that everything starts out with its default
appearance. The default is for everything to be blue and there not to be any
arrow attached to any body.

Find the function named ``timeStepping`` in the ``MyWindow`` class. The top of
this function is where we will want to reset everything to the default appearance.

Each BodyNode contains visualization ``Shape``s that will be rendered during
simulation. In our case, each BodyNode has two shapes:

- One shape to visualize the parent joint
- One shape to visualize the body

The default appearance for everything is to be colored blue, so we'll want to
iterate through these two Shapes in each BodyNode, setting their colors to blue.

```cpp
for(size_t i = 0; i < mPendulum->getNumBodyNodes(); ++i)
{
  BodyNode* bn = mPendulum->getBodyNode(i);
  for(size_t j = 0; j < 2; ++j)
  {
    const ShapePtr& shape = bn->getVisualizationShape(j);

    shape->setColor(dart::Color::Blue());
  }

  // TODO: Remove any arrows
}
```

Additionally, there is the possibility that some BodyNodes will have an arrow
shape attached if the user had been applying an external body force to it. By
default, this arrow should not be attached, so in the outer for-loop, we should
check for arrows and remove them:

```cpp
if(bn->getNumVisualizationShapes() == 3)
{
  bn->removeVisualizationShape(mArrow);
}
```

Now everything will be reset to the default appearance.

### Lesson 1b: Apply joint torques based on user input

The ``MyWindow`` class in this tutorial has a variable called ``mForceCountDown``
which is a ``std::vector<int>`` whose entries get set to a value of
``default_countdown`` each time the user presses a number key. If an entry in
``mForceCountDown`` is greater than zero, then that implies that the user wants
a force to be applied for that entry.

There are two ways that forces can be applied:

- As an internal joint force
- As an external body force

First we'll consider applying a Joint force. Inside the for-loop that goes
through each ``DegreeOfFreedom`` using ``getNumDofs()``, there is an 
if-statement for ``mForceCountDown``. In that if-statement, we'll grab the
relevant DegreeOfFreedom and set its generalized (joint) force:

```cpp
DegreeOfFreedom* dof = mPendulum->getDof(i);
dof->setForce( mPositiveSign? default_torque : -default_torque );
```

The ``mPositiveSign`` boolean gets toggled when the user presses the minus sign
'-' key. We use this boolean to decide whether the applied force should be
positive or negative.

Now we'll want to visualize the fact that a Joint force is being applied. We'll
do this by highlighting the joint with the color red. First we'll grab the Shape
that corresponds to this Joint:

```cpp
BodyNode* bn = dof->getChildBodyNode();
const ShapePtr& shape = bn->getVisualizationShape(0);
```

Because of the way the pendulum bodies were constructed, we trust that the
zeroth indexed visualization shape will be the shape that depicts the joint.
So now we will color it red:

```cpp
shape->setColor(dart::Color::Red());
```

### Lesson 1c: Apply body forces based on user input

If mBodyForce is true, we'll want to apply an external force to the body instead
of an internal force in the joint. First, inside the for-loop that iterates
through each ``BodyNode`` using ``getNumBodyNodes()``, there is an if-statement
for ``mForceCountDown``. In that if-statement, we'll grab the relevant BodyNode:

```cpp
BodyNode* bn = mPendulum->getBodyNode(i);
```

Now we'll create an ``Eigen::Vector3d`` that describes the force and another one
that describes the location for that force. An ``Eigen::Vector3d`` is the Eigen
C++ library's version of a three-dimensional mathematical vector. Note that the
``d`` at the end of the name stands for ``double``, not for "dimension". An
Eigen::Vector3f would be a three-dimensional vector of floats, and an
Eigen::Vector3i would be a three-dimensional vector of integers.

```cpp
Eigen::Vector3d force = default_force * Eigen::Vector3d::UnitX();
Eigen::Vector3d location(-default_width / 2.0, 0.0, default_height / 2.0);
```

The force will have a magnitude of ``default_force`` and it will point in the
positive x-direction. The location of the force will be in the center of the
negative x side of the body, as if a finger on the negative side is pushing the
body in the positive direction. However, we need to account for sign changes:

```cpp
if(!mPositiveSign)
{
  force = -force;
  location[0] = -location[0];
}
```

That will flip the signs whenever the user is requesting a negative force.

Now we can add the external force:

```cpp
bn->addExtForce(force, location, true, true);
```

The two ``true`` booleans at the end are indicating to DART that both the force
and the location vectors are being expressed with respect to the body frame.

Now we'll want to visualize the force being applied to the body. First, we'll
grab the Shape for the body and color it red:

```cpp
const ShapePtr& shape = bn->getVisualizationShape(1);
shape->setColor(dart::Color::Red());
```

Last time we grabbed the 0-index visualization shape, because we trusted that
it was the shape that represented the parent Joint. This time we're grabbing
the 1-index visualization shape, because we trust that it is the shape for the
body.

Now we'll want to add an arrow to the visualization shapes of the body to
represent the applied force. The ``MyWindow`` class already provides the arrow
shape; we just need to add it:

```cpp
bn->addVisualizationShape(mArrow);
```

# Lesson 2: Set spring and damping properties for joints

DART allows Joints to have implicit spring and damping properties. By default,
these properties are zeroed out, so a joint will only exhibit the forces that
are given to it by the ``Joint::setForces`` function. However, you can give a
non-zero spring coefficient to a joint so that it behaves according to Hooke's
Law, and you can give a non-zero damping coefficient to a joint which will
result in linear damping. These forces are computed using implicit methods in
order to improve numerical stability.

### Lesson 2a: Set joint spring rest position

First let's see how to get and set the rest positions.

Find the function named ``changeRestPosition`` in the ``MyWindow`` class. This
function will be called whenever the user presses the 'q' or 'a' button. We want
those buttons to curl and uncurl the rest positions for the pendulum. To start,
we'll go through all the generalized coordinates and change their rest positions
by ``delta``:

```cpp
for(size_t i = 0; i < mPendulum->getNumDofs(); ++i)
{
  DegreeOfFreedom* dof = mPendulum->getDof(i);
  double q0 = dof->getRestPosition() + delta;

  dof->setRestPosition(q0);
}
```

However, it's important to note that the system can become somewhat unstable if
we allow it to curl up too much, so let's put a limit on the magnitude of the
rest angle. Right before ``dof->setRestPosition(q0);`` we can put:

```cpp
if(std::abs(q0) > 90.0 * M_PI / 180.0)
  q0 = (q0 > 0)? (90.0 * M_PI / 180.0) : -(90.0 * M_PI / 180.0);
```

And there's one last thing to consider: the first joint of the pendulum is a
BallJoint. BallJoints have three degrees of freedom, which means if we alter
the rest positions of *all* of the pendulum's degrees of freedom, then the
pendulum will end up curling out of the x-z plane. You can allow this to happen
if you want, or you can prevent it from happening by zeroing out the rest
positions of the BallJoint's other two degrees of freedom:

```cpp
mPendulum->getDof(0)->setRestPosition(0.0);
mPendulum->getDof(2)->setRestPosition(0.0);
```

### Lesson 2b: Set joint spring stiffness

Changing the rest position does not accomplish anything without having any
spring stiffness. We can change the spring stiffness as follows:

```cpp
for(size_t i = 0; i < mPendulum->getNumDofs(); ++i)
{
  DegreeOfFreedom* dof = mPendulum->getDof(i);
  double stiffness = dof->getSpringStiffness() + delta;
  dof->setSpringStiffness(stiffness);
}
```

However, it's important to realize that if the spring stiffness were ever to
become negative, we would get some very nasty explosive behavior. It's also a
bad idea to just trust the user to avoid decrementing it into being negative.
So before the line ``dof->setSpringStiffness(stiffness);`` you'll want to put:

```cpp
if(stiffness < 0.0)
  stiffness = 0.0;
```

### Lesson 2c: Set joint damping

Joint damping can be thought of as friction inside the joint actuator. It
applies a resistive force to the joint which is proportional to the generalized
velocities of the joint. This draws energy out of the system and generally
results in more stable behavior.

The API for getting and setting the damping is just like the API for stiffness:

```cpp
for(size_t i = 0; i < mPendulum->getNumDofs(); ++i)
{
  DegreeOfFreedom* dof = mPendulum->getDof(i);
  double damping = dof->getDampingCoefficient() + delta;
  if(damping < 0.0)
    damping = 0.0;
  dof->setDampingCoefficient(damping);
}
```

Again, we want to make sure that the damping coefficient is never negative. In
fact, a negative damping coefficient would be far more harmful than a negative
stiffness coefficient.

# Lesson 3: Add and remove dynamic constraints

Dynamic constraints in DART allow you to attach two BodyNodes together according
to a selection of a few different Joint-style constraints. This allows you to
create closed loop constraints, which is not possible using standard Joints.
You can also create a dynamic constraint that attaches a BodyNode to the World
instead of to another BodyNode.

In our case, we want to attach the last BodyNode to the World with a BallJoint
style constraint whenever the function ``addConstraint()`` gets called. First,
let's grab the last BodyNode in the pendulum:

```cpp
BodyNode* tip  = mPendulum->getBodyNode(mPendulum->getNumBodyNodes() - 1);
```

Now we'll want to compute the location that the constraint should have. We want
to connect the very end of the tip to the world, so the location would be:

```cpp
Eigen::Vector3d location =
    tip->getTransform() * Eigen::Vector3d(0.0, 0.0, default_height);
```

Now we can create the BallJointConstraint:

```cpp
mBallConstraint =
    std::make_shared<dart::constraint::BallJointConstraint>(tip, location);
```

And then add it to the world:

```cpp
mWorld->getConstraintSolver()->addConstraint(mBallConstraint);
```

Now we also want to be able to remove this constraint. In the function
``removeConstraint()``, we can put the following code:

```cpp
mWorld->getConstraintSolver()->removeConstraint(mBallConstraint);
mBallConstraint = nullptr;
```

Setting mBallConstraint to a nullptr will allow its smart pointer to delete it.

**Now you are ready to run the demo!**

<div id="fb-root"></div>
<script>(function(d, s, id) {
  var js, fjs = d.getElementsByTagName(s)[0];
  if (d.getElementById(id)) return;
  js = d.createElement(s); js.id = id;
  js.src = "//connect.facebook.net/en_US/sdk.js#xfbml=1&version=v2.4";
  fjs.parentNode.insertBefore(js, fjs);
}(document, 'script', 'facebook-jssdk'));</script>

<div class="fb-like" data-href="http://dart.readthedocs.org/en/release-5.1/tutorials/multi-pendulum/" data-layout="button_count" data-action="like" data-show-faces="true" data-share="true"></div>

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
