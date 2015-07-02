# Overview

This tutorial will demonstrate some basic interaction with DART's dynamics
API during simulation. This will show you how to:

- Change the colors of shapes
- Add/remove shapes from visualization
- Apply internal forces in the joints
- Apply external forces to the bodies
- Alter the implicit spring and damping properties of joints
- Add/remove dynamic constraints

# Lesson 1: Changing shapes and applying forces

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

In DART, an articulated dynamics model is represented by a ``Skeleton``. A 
Skeleton is a structure that consists of ``BodyNode``s (bodies) which are 
connected by ``Joint``s. Every Joint has a child BodyNode, and every BodyNode 
has a parent Joint. Even the root BodyNode has a Joint that attaches it to the 
World.

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




















