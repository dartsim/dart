# Multi Pendulum

## Overview

This tutorial will demonstrate some basic interaction with DART's dynamics
API during simulation. This will show you how to:

- Create a basic program to simulate a dynamic system
- Change the colors of shapes
- Add/remove shapes from visualization
- Apply internal forces in the joints
- Apply external forces to the bodies
- Alter the implicit spring and damping properties of joints
- Add/remove dynamic constraints

Please reference the source code in [**tutorials/tutorial_multi_pendulum/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_multi_pendulum/main.cpp) and [**tutorials/tutorial_multi_pendulum_finished/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_multi_pendulum_finished/main.cpp).

## Lesson 0: Simulate a passive multi-pendulum

This is a warmup lesson that demonstrates how to set up a simulation
program in DART. The example we will use throughout this tutorial is a
pendulum with five rigid bodies swinging under gravity. DART allows
the user to build various articulated rigid/soft body systems from
scratch. It also loads models in URDF, SDF, and SKEL formats as
demonstrated in the later tutorials.

In DART, an articulated dynamics model is represented by a
`Skeleton`. In the `main` function, we first create an empty
skeleton named _pendulum_.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         SkeletonPtr pendulum = Skeleton::create("pendulum");

   .. tab:: Python

      .. code-block:: python

         pendulum = dart.dynamics.Skeleton("pendulum")
```

A Skeleton is a structure that consists of `BodyNode`s (bodies) which are
connected by `Joint`s. Every Joint has a child BodyNode, and every BodyNode
has a parent Joint. Even the root BodyNode has a Joint that attaches it to the
World. In the function `makeRootBody`, we create a pair of a
`BallJoint` and a BodyNode, and attach this pair to the currently
empty pendulum skeleton.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         BodyNodePtr bn = pendulum->createJointAndBodyNodePair<BallJoint>(
               nullptr, properties, BodyNode::AspectProperties(name)).second;

   .. tab:: Python

      .. code-block:: python

         joint_prop = dart.dynamics.BallJointProperties()
         joint_prop.mName = f"{name}_joint"
         joint, body = pendulum.createBallJointAndBodyNodePair(
             None,
             joint_prop,
             dart.dynamics.BodyNodeProperties(
                 dart.dynamics.BodyNodeAspectProperties(name)
             ),
         )
```

Note that the first parameters is a nullptr, which indicates that
this new BodyNode is the root of the pendulum. If we wish to append
the new BodyNode to an existing BodyNode in the pendulum,
we can do so by passing the pointer of the existing BodyNode as
the first parameter. In fact, this is how we add more BodyNodes to
the pendulum in the function `addBody`:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(
               parent, properties, BodyNode::AspectProperties(name)).second;

   .. tab:: Python

      .. code-block:: python

         joint_prop = dart.dynamics.RevoluteJointProperties()
         joint_prop.mName = f"{name}_joint"
         joint_prop.mAxis = [0.0, 1.0, 0.0]
         joint_prop.mT_ParentBodyToJoint.set_translation([0.0, 0.0, default_height])
         joint, body = pendulum.createRevoluteJointAndBodyNodePair(
             parent,
             joint_prop,
             dart.dynamics.BodyNodeProperties(
                 dart.dynamics.BodyNodeAspectProperties(name)
             ),
         )
```
These tutorials use the OSG viewer utilities under ``dart::gui``. After we
create the ``World`` and pendulum skeleton, we instantiate a ``Controller``
plus a GUI event handler, wrap the world in a ``RealTimeWorldNode``, and send
that node to a ``Viewer``:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum/main.cpp
         :language: cpp
         :start-after: // Create controller and event handler
         :end-before: // Print instructions

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: world.addSkeleton(pendulum)
         :end-before: viewer.addInstructionText("space bar
```

The `Viewer` drives the world through `CustomWorldNode::customPreStep()`,
which we override to call `Controller::update()` each time step:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum/main.cpp
         :language: cpp
         :start-after: class CustomWorldNode
         :end-before: protected:

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: class CustomWorldNode
         :end-before: def set_geometry
```

## Lesson 1: Change shapes and applying forces

We have a pendulum with five bodies, and we want to be able to apply forces to
them during simulation. Additionally, we want to visualize these forces so we
can more easily interpret what is happening in the simulation. For this reason,
we'll discuss visualizing and forces at the same time.

### Lesson 1a: Reset everything to default appearance

At each step, we'll want to make sure that everything starts out with its default
appearance. The default is for everything to be blue and there not to be any
arrow attached to any body.

Find the `Controller::update` function. The top of this function is where we
reset everything to the default appearance before advancing the simulation.

Each BodyNode contains visualization `Shape`s that will be rendered during
simulation. In our case, each BodyNode has two shapes:

- One shape to visualize the parent joint
- One shape to visualize the body

The default appearance for everything is to be colored blue, so we'll want to
iterate through these two Shapes in each BodyNode, setting their colors to blue.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson1a-reset-start
         :end-before: // snippet:cpp-lesson1a-reset-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson1a-reset-start
         :end-before: # snippet:py-lesson1a-reset-end
```

Additionally, there is the possibility that some BodyNodes will have an arrow
shape attached if the user had been applying an external body force to it. By
default, this arrow should not be attached, so in the outer for-loop, we should
check for arrows and remove them:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson1a-remove-arrow-start
         :end-before: // snippet:cpp-lesson1a-remove-arrow-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson1a-remove-arrow-start
         :end-before: # snippet:py-lesson1a-remove-arrow-end
```

Now everything will be reset to the default appearance.

### Lesson 1b: Apply joint torques based on user input

The `Controller` class keeps a `mForceCountDown` vector whose entries get
set to `default_countdown` whenever `PendulumEventHandler` sees a numeric key
press. If an entry in `mForceCountDown` is greater than zero, then that implies
that the user wants a force to be applied for that entry.

There are two ways that forces can be applied:

- As an internal joint force
- As an external body force

First we'll consider applying a Joint force. Inside the for-loop that goes
through each `DegreeOfFreedom` using `getNumDofs()`, there is an
if-statement for `mForceCountDown`. In that if-statement, we'll grab the
relevant DegreeOfFreedom and set its generalized (joint) force:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson1b-joint-force-start
         :end-before: // snippet:cpp-lesson1b-joint-force-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson1b-joint-force-start
         :end-before: # snippet:py-lesson1b-joint-force-end
```

The `mPositiveSign` boolean gets toggled when the user presses the minus sign
'-' key. We use this boolean to decide whether the applied force should be
positive or negative.

Now we'll want to visualize the fact that a Joint force is being applied. We'll
do this by highlighting the joint with the color red. First we'll grab the Shape
that corresponds to this Joint:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         BodyNode* bn = dof->getChildBodyNode();
         auto* shapeNode = bn->getShapeNodeWith<VisualAspect>(0);

   .. tab:: Python

      .. code-block:: python

         child = dof.getChildBodyNode()
         joint_visual = child.getShapeNode(0).getVisualAspect()
```

Because of the way the pendulum bodies were constructed, we trust that the
zeroth indexed visualization shape will be the shape that depicts the joint.
So now we will color it red:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         shapeNode->getVisualAspect()->setColor(dart::Color::Red());

   .. tab:: Python

      .. code-block:: python

         joint_visual.setColor([1.0, 0.0, 0.0, 1.0])
```

### Lesson 1c: Apply body forces based on user input

If mBodyForce is true, we'll want to apply an external force to the body instead
of an internal force in the joint. First, inside the for-loop that iterates
through each `BodyNode` using `getNumBodyNodes()`, there is an if-statement
for `mForceCountDown`. In that if-statement, we'll grab the relevant BodyNode:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         BodyNode* bn = mPendulum->getBodyNode(i);

   .. tab:: Python

      .. code-block:: python

         body = self.pendulum.getBodyNode(i)
```

Now we'll create an `math::Vector3d` that describes the force and another one
that describes the location for that force. An `math::Vector3d` is the Eigen
C++ library's version of a three-dimensional mathematical vector. Note that the
`d` at the end of the name stands for `double`, not for "dimension". An
math::Vector3f would be a three-dimensional vector of floats, and an
math::Vector3i would be a three-dimensional vector of integers.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         math::Vector3d force = default_force * math::Vector3d::UnitX();
         math::Vector3d location(-default_width / 2.0, 0.0, default_height / 2.0);

   .. tab:: Python

      .. code-block:: python

         force = np.array([default_force, 0.0, 0.0])
         location = np.array([-default_width / 2.0, 0.0, default_height / 2.0])
```

The force will have a magnitude of `default_force` and it will point in the
positive x-direction. The location of the force will be in the center of the
negative x side of the body, as if a finger on the negative side is pushing the
body in the positive direction. However, we need to account for sign changes:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         if(!mPositiveSign)
         {
           force = -force;
           location[0] = -location[0];
         }

   .. tab:: Python

      .. code-block:: python

         if not self.positive_sign:
             force *= -1.0
             location[0] *= -1.0
```

That will flip the signs whenever the user is requesting a negative force.

Now we can add the external force:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         bn->addExtForce(force, location, true, true);

   .. tab:: Python

      .. code-block:: python

         body.addExtForce(force, location, True, True)
```

The two `true` booleans at the end are indicating to DART that both the force
and the location vectors are being expressed with respect to the body frame.

Now we'll want to visualize the force being applied to the body. First, we'll
grab the Shape for the body and color it red:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
         shapeNodes[1]->getVisualAspect()->setColor(dart::Color::Red());

   .. tab:: Python

      .. code-block:: python

         body_visual = body.getShapeNode(1).getVisualAspect()
         body_visual.setColor([1.0, 0.0, 0.0, 1.0])
```

Last time we grabbed the 0-index visualization shape, because we trusted that
it was the shape that represented the parent Joint. This time we're grabbing
the 1-index visualization shape, because we trust that it is the shape for the
body.

Now we'll want to add an arrow to the visualization shapes of the body to
represent the applied force. The `Controller` already owns an `ArrowShape`
that visualizes the currently applied body force; we just need to show it:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         bn->createShapeNodeWith<VisualAspect>(mArrow);

   .. tab:: Python

      .. code-block:: python

         arrow, arrow_visual = self.body_force_visuals[i]
         arrow_visual.show()
```

## Lesson 2: Set spring and damping properties for joints

DART allows Joints to have implicit spring and damping properties. By default,
these properties are zeroed out, so a joint will only exhibit the forces that
are given to it by the `Joint::setForces` function. However, you can give a
non-zero spring coefficient to a joint so that it behaves according to Hooke's
Law, and you can give a non-zero damping coefficient to a joint which will
result in linear damping. These forces are computed using implicit methods in
order to improve numerical stability.

### Lesson 2a: Set joint spring rest position

First let's see how to get and set the rest positions.

Find the function named `Controller::changeRestPosition`. This function is
triggered whenever `PendulumEventHandler` sees the 'q' or 'a' key. We want
those buttons to curl and uncurl the rest positions for the pendulum. To start,
we'll go through all the generalized coordinates and change their rest positions
by `delta`:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson2a-rest-position-start
         :end-before: // snippet:cpp-lesson2a-rest-position-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson2a-rest-position-start
         :end-before: # snippet:py-lesson2a-rest-position-end
```

However, it's important to note that the system can become somewhat unstable if
we allow it to curl up too much, so let's put a limit on the magnitude of the
rest angle. Right before `dof->setRestPosition(q0);` we can put:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // The system becomes numerically unstable
         :end-before:       dof->setRestPosition(q0);

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: q0 = dof.getRestPosition() + delta
         :end-before:         dof.setRestPosition(q0)
```

And there's one last thing to consider: the first joint of the pendulum is a
BallJoint. BallJoints have three degrees of freedom, which means if we alter
the rest positions of _all_ of the pendulum's degrees of freedom, then the
pendulum will end up curling out of the x-z plane. You can allow this to happen
if you want, or you can prevent it from happening by zeroing out the rest
positions of the BallJoint's other two degrees of freedom:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: cpp

         mPendulum->getDof(0)->setRestPosition(0.0);
         mPendulum->getDof(2)->setRestPosition(0.0);

   .. tab:: Python

      .. code-block:: python

         self.pendulum.getDof(0).setRestPosition(0.0)
         self.pendulum.getDof(2).setRestPosition(0.0)
```

### Lesson 2b: Set joint spring stiffness

Changing the rest position does not accomplish anything without having any
spring stiffness. We can change the spring stiffness as follows:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson2b-stiffness-start
         :end-before: // snippet:cpp-lesson2b-stiffness-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson2b-stiffness-start
         :end-before: # snippet:py-lesson2b-stiffness-end
```

However, it's important to realize that if the spring stiffness were ever to
become negative, we would get some very nasty explosive behavior. It's also a
bad idea to just trust the user to avoid decrementing it into being negative.
So before the line `dof->setSpringStiffness(stiffness);` you'll want to put:

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

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson2c-damping-start
         :end-before: // snippet:cpp-lesson2c-damping-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson2c-damping-start
         :end-before: # snippet:py-lesson2c-damping-end
```

Again, we want to make sure that the damping coefficient is never negative. In
fact, a negative damping coefficient would be far more harmful than a negative
stiffness coefficient.

## Lesson 3: Add and remove dynamic constraints

Dynamic constraints in DART allow you to attach two BodyNodes together according
to a selection of a few different Joint-style constraints. This allows you to
create closed loop constraints, which is not possible using standard Joints.
You can also create a dynamic constraint that attaches a BodyNode to the World
instead of to another BodyNode.

In our case, we want to attach the last BodyNode to the World with a BallJoint
style constraint whenever the function `addConstraint()` gets called. First,
let's grab the last BodyNode in the pendulum:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson3-add-constraint-start
         :end-before: Eigen::Vector3d location

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson3-add-constraint-start
         :end-before: location = tip.getTransform()
```

Now we'll want to compute the location that the constraint should have. We want
to connect the very end of the tip to the world, so the location would be:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // Attach the last link to the world
         :end-before: mBallConstraint =

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: location = tip.getTransform().multiply
         :end-before: self.ball_constraint = dart.constraint.BallJointConstraint
```

Now we can create the BallJointConstraint:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: Eigen::Vector3d location
         :end-before: mWorld->getConstraintSolver()->addConstraint

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: self.ball_constraint = dart.constraint.BallJointConstraint
         :end-before: self.world.getConstraintSolver().addConstraint
```

And then add it to the world:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: mBallConstraint =
         :end-before: /// Remove any existing constraint

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: self.world.getConstraintSolver().addConstraint
         :end-before: def remove_constraint
```

Now we also want to be able to remove this constraint. In the function
`removeConstraint()`, we can put the following code:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: /// Remove any existing constraint
         :end-before: bool hasConstraint() const

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: def remove_constraint
         :end-before: def has_constraint
```

Setting mBallConstraint to a nullptr will allow its smart pointer to delete it.

**Now you are ready to run the demo!**
