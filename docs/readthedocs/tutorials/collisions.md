# Collisions

## Overview

This tutorial will show you how to programmatically create different kinds of
bodies and set initial conditions for Skeletons. It will also demonstrate some
use of DART's Frame Semantics.

The tutorial consists of five Lessons covering the following topics:

- Creating a rigid body
- Creating a soft body
- Setting initial conditions and taking advantage of Frames
- Setting joint spring and damping properties
- Creating a closed kinematic chain

Please reference the source code in [**tutorial_collisions/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_collisions/main.cpp) and [**tutorial_collisions_finished/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_collisions_finished/main.cpp).

## Lesson 1: Creating a rigid body

Start by opening the Skeleton code in [`tutorials/tutorial_collisions/main.cpp`](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_collisions/main.cpp).
Find the function named `addRigidBody`. You will notice that this is a templated
function. If you're not familiar with templates, that's okay; we won't be doing
anything too complicated with them. Different Joint types in DART are managed by
a bunch of different classes, so we need to use templates if we want the same
function to work with a variety of Joint types.

### Lesson 1a: Setting joint properties

The first thing we'll want to do is set the Joint properties for our new body.
Whenever we create a BodyNode, we must also create a parent Joint for it. A
BodyNode needs a parent Joint, even if that BodyNode is the root of the Skeleton,
because we need its parent Joint to describe how it's attached to the world. A
root BodyNode could be attached to the world by any kind of Joint. Most often,
it will be attached by either a FreeJoint (if the body should be completely
free to move with respect to the world) or a WeldJoint (if the body should be
rigidly attached to the world, unable to move at all), but _any_ Joint type
is permissible.

Joint properties are managed in a nested class, which means it's a class which
is defined inside of another class. For example, `RevoluteJoint` properties are
managed in a class called `RevoluteJoint::Properties` while `PrismaticJoint`
properties are managed in a class called `PrismaticJoint::Properties`. However,
both `RevoluteJoint` and `PrismaticJoint` inherit the `SingleDofJoint` class
so the `RevoluteJoint::Properties` and `PrismaticJoint::Properties` classes
both inherit the `SingleDofJoint::Properties` class. The difference is that
`RevoluteJoint::Properties` also inherits `RevoluteJoint::UniqueProperties`
whereas `PrismaticJoint::Properties` inherits `PrismaticJoint::UniqueProperties`
instead. Many DART classes contain nested `Properties` classes like this which
are compositions of their base class's nested `Properties` class and their own
`UniqueProperties` class. As you'll see later, this is useful for providing a
consistent API that works cleanly for fundamentally different types of classes.

To create a `Properties` class for our Joint type, we instantiate the nested
struct for the joint, assign it a unique name, and, if needed, offset the joint
from its parent BodyNode. The block below shows the complete setup, including
the transforms that align the joint halfway between the two bodies:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson1a-properties-start
         :end-before: // snippet:cpp-collisions-lesson1a-properties-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson1a-properties-start
         :end-before: # snippet:py-collisions-lesson1a-properties-end
```

We need to include the `typename` keywords because of how the syntax works for
templated functions. Leaving it out should make your compiler complain.

From here, we can set the Joint properties in any way we'd like. There are only
a few things we care about right now: First, the Joint's name. Every Joint in a
Skeleton needs to have a non-empty unique name. Those are the only restrictions
that are placed on Joint names. If you try to make a Joint's name empty, it will
be given a default name. If you try to make a Joint's name non-unique, DART will
append a number tag to the end of the name in order to make it unique. It will
also print out a warning during run time, which can be an eyesore (because it
wants you to be aware when you are being negligent about naming things). For the
sake of simplicity, let's just give it a name based off its child BodyNode.

Next we'll want to deal with offsetting the new BodyNode from its parent BodyNode.
An `Isometry3` (python) or `Isometry3d` (C++) is the Eigen library's version
of a homogeneous transformation matrix. Initialize it to the identity to ensure
the contents are well-defined before applying translations. We can easily compute
the center point between the origins of the two bodies using our default height
value and then assign that translation to both the parent- and child-to-joint
transforms.

Remember that all of that code should go inside the `if(parent)` condition.
We do not want to create this offset for root BodyNodes, because later on we
will rely on the assumption that the root Joint origin is lined up with the
root BodyNode origin.

### Lesson 1b: Create a Joint and BodyNode pair

A single function is used to simultaneously create a new Joint and its child
BodyNode. It's important to note that a Joint cannot be created without a
child BodyNode to accompany it, and a BodyNode cannot be created with parent
Joint to attach it to something. A parent Joint without a child BodyNode or
vice-versa would be non-physical and nonsensical, so we don't allow it.

Use the following to create a new Joint & BodyNode, and obtain a pointer to
that new BodyNode:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson1b-joint-pair-start
         :end-before: // snippet:cpp-collisions-lesson1b-joint-pair-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson1b-joint-pair-start
         :end-before: # snippet:py-collisions-lesson1b-joint-pair-end
```

There's a lot going on in this function, so let's break it down for a moment:

`chain->createJointAndBodyNodePair<JointType>`

This is a Skeleton member function that takes template arguments. The first
template argument specifies the type of Joint that you want to create. In our
case, the type of Joint we want to create is actually a template argument of
our current function, so we just pass that argument along. The second template
argument of `createJointAndBodyNodePair` allows us to specify the BodyNode
type that we want to create, but the default argument is a standard rigid
BodyNode, so we can leave the second argument blank.

`(parent, joint_properties, BodyNode::AspectProperties(name))`

Now for the function arguments: The first specifies the parent BodyNode. In the
event that you want to create a root BodyNode, you can simply pass in a nullptr
as the parent. The second argument is a `JointType::Properties` struct, so we
pass in the `joint_properties` object that we created earlier. The third argument is
a `BodyNode::Properties` struct, but we're going to set the BodyNode properties
later, so we'll just toss the name in by wrapping it up in a
`BodyNode::AspectProperties` object and leave the rest as default values.

Now notice the very last thing on this line of code:

`.second;`

The function actually returns a `std::pair` of pointers to the new Joint and
new BodyNode that were just created, but we only care about grabbing the
BodyNode once the function is finished, so we can append `.second` to the end
of the line so that we just grab the BodyNode pointer and ignore the Joint
pointer. The joint will of course still be created; we just have no need to
access it at this point.

### Lesson 1c: Make a shape for the body

We'll take advantage of the Shape::ShapeType enumeration to specify what kind of
Shape we want to produce for the body. In particular, we'll allow the user to
specify three types of Shapes: `Shape::BOX`, `Shape::CYLINDER`, and
`Shape::ELLIPSOID`.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson1c-shape-selection-start
         :end-before: // snippet:cpp-collisions-lesson1c-shape-selection-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson1c-shape-selection-start
         :end-before: # snippet:py-collisions-lesson1c-shape-selection-end
```

`ShapePtr` is simply a typedef for `std::shared_ptr<Shape>`. DART has this
typedef in order to improve space usage and readability, because this type gets
used very often.

Now we want to construct each of the Shape types within their conditional
statements. Each constructor is a bit different.

For box shapes we pass a vector that contains the three dimensions of the box.
The cylinder branch uses a radius and height, and the ellipsoid branch consumes
the three axis lengths. Since we actually want a sphere, all three axis lengths
will be equal, so we can create a vector filled with ones and then multiply it
by the length that we actually want for the three components.

Finally, we want to add this shape as a visualization **and** collision shape for
the BodyNode:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson1c-shape-node-start
         :end-before: // snippet:cpp-collisions-lesson1c-shape-node-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson1c-shape-node-start
         :end-before: # snippet:py-collisions-lesson1c-shape-node-end
```

We want to do this no matter which type was selected, so those two lines of code
should be after all the condition statements.

### Lesson 1d: Set up the inertia properties for the body

For the simulations to be physically accurate, it's important for the inertia
properties of the body to match up with the geometric properties of the shape.
We can create an `Inertia` object and set its values based on the shape's
geometry, then give that `Inertia` to the BodyNode.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson1d-inertia-start
         :end-before: // snippet:cpp-collisions-lesson1d-inertia-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson1d-inertia-start
         :end-before: # snippet:py-collisions-lesson1d-inertia-end
```

### Lesson 1e: Set the coefficient of restitution

This is very easily done with the following function:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson1e-restitution-start
         :end-before: // snippet:cpp-collisions-lesson1e-restitution-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson1e-restitution-start
         :end-before: # snippet:py-collisions-lesson1e-restitution-end
```

### Lesson 1f: Set the damping coefficient

In real life, joints have friction. This pulls energy out of systems over time,
and makes those systems more stable. In our simulation, we'll ignore air
friction, but we'll add friction in the joints between bodies in order to have
better numerical and dynamic stability:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson1f-damping-start
         :end-before: // snippet:cpp-collisions-lesson1f-damping-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson1f-damping-start
         :end-before: # snippet:py-collisions-lesson1f-damping-end
```

If this BodyNode has a parent BodyNode, then we set damping coefficients of its
Joint to a default value.

## Lesson 2: Creating a soft body

Find the templated function named `addSoftBody`. This function will have a
role identical to the `addRigidBody` function from earlier.

```{note}
Soft body helpers are currently exposed only in the C++ API. The dartpy snippets
approximate the same structure by reusing rigid BodyNodes with matching geometry
and transparency so you can follow along with each lesson.
```

### Lesson 2a: Set the Joint properties

This portion is exactly the same as Lesson 1a. You can even copy the code
directly from there if you'd like to.

### Lesson 2b: Set the properties of the soft body

Last time we set the BodyNode properties after creating it, but this time
we'll set them beforehand.

First, let's create a struct for the properties that are unique to SoftBodyNodes.
Up above we defined an enumeration for a couple different SoftBodyNode types.
There is no official DART-native enumeration for this, we created our own to use
for this function. We'll want to fill in the `SoftBodyNode::UniqueProperties`
struct based off of this enumeration.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson2b-soft-properties-start
         :end-before: // snippet:cpp-collisions-lesson2b-soft-properties-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson2b-soft-properties-start
         :end-before: # snippet:py-collisions-lesson2b-soft-properties-end
```

Each of the C++ conditionals leverages `SoftBodyNodeHelper` to construct the
skin and its mass distribution (feel free to experiment with slice/stack counts,
keeping in mind the minimum values noted in the comments). The dartpy
implementation currently falls back to creating a rigid shell with matching
geometry because SoftBody nodes are not yet exposed, but the remainder of the
lesson still applies.

### Lesson 2c: Create the Joint and Soft Body pair

This step is very similar to Lesson 1b, except now we'll want to specify
that we're creating a soft BodyNode. First, let's create a full
`SoftBodyNode::Properties`. This will combine the `UniqueProperties` of the
SoftBodyNode with the standard properties of a BodyNode. Now we can pass the
whole thing into the creation function:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson2c-soft-node-start
         :end-before: // snippet:cpp-collisions-lesson2c-soft-node-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson2c-soft-node-start
         :end-before: # snippet:py-collisions-lesson2c-soft-node-end
```

Notice that this time it will return a `SoftBodyNode` pointer rather than a
normal `BodyNode` pointer. This is one of the advantages of templates!

### Lesson 2d: Zero out the BodyNode inertia

A SoftBodyNode has two sources of inertia: the underlying inertia of the
standard BodyNode class, and the point mass inertias of its soft skin. In our
case, we only want the point mass inertias, so we should zero out the standard
BodyNode inertia. However, zeroing out inertia values can be very dangerous,
because it can easily result in singularities. So instead of completely zeroing
them out, we will just make them small enough that they don't impact the
simulation:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson2d-soft-inertia-start
         :end-before: // snippet:cpp-collisions-lesson2d-soft-inertia-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson2d-soft-inertia-start
         :end-before: # snippet:py-collisions-lesson2d-soft-inertia-end
```

### Lesson 2e: Make the shape transparent

To help us visually distinguish between the soft and rigid portions of a body,
we can make the soft part of the shape transparent. Upon creation, a SoftBodyNode
will have exactly one visualization shape: the soft shape visualizer. We can
reduce the value of its alpha channel (or directly call `setAlpha` in dartpy)
so it stands out from the rigid pieces:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson2e-soft-alpha-start
         :end-before: // snippet:cpp-collisions-lesson2e-soft-alpha-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson2e-soft-alpha-start
         :end-before: # snippet:py-collisions-lesson2e-soft-alpha-end
```

### Lesson 2f: Give a hard bone to the SoftBodyNode

SoftBodyNodes are intended to be used as soft skins that are attached to rigid
bones. We can create a rigid shape, place it in the SoftBodyNode, and give some
inertia to the SoftBodyNode's base BodyNode class, to act as the inertia of the
bone.

Find the function `createSoftBody()`. Underneath the call to `addSoftBody`,
we can create a box shape that matches the dimensions of the soft box, but scaled
down:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson2f-rigid-core-start
         :end-before: // snippet:cpp-collisions-lesson2f-rigid-core-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson2f-rigid-core-start
         :end-before: # snippet:py-collisions-lesson2f-rigid-core-end
```

After adding the rigid geometry, set its inertia so the simulation stays
physically reasonable. The dartpy version follows the same pattern using the
fallback rigid node.

Note that the inertia of the inherited BodyNode is independent of the inertia
of the SoftBodyNode's skin.

### Lesson 2g: Add a rigid body attached by a WeldJoint

To make a more interesting hybrid shape, we can attach a protruding rigid body
to a SoftBodyNode using a WeldJoint. Find the `createHybridBody()` function
and see where we call the `addSoftBody` function. Just below this, we'll
create a new rigid body with a WeldJoint attachment:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson2g-welded-rigid-start
         :end-before: // snippet:cpp-collisions-lesson2g-welded-rigid-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson2g-welded-rigid-start
         :end-before: # snippet:py-collisions-lesson2g-welded-rigid-end
```

Now we can give the new rigid BodyNode a box, offset it away from the center of
its parent so it protrudes, and set its inertia so that collisions behave as
expected.

## Lesson 3: Setting initial conditions and taking advantage of Frames

Find the `addObject` function in the `MyWorld` class. This function will
be called whenever the user requests for an object to be added to the world.
In this function, we want to set up the initial conditions for the object so
that it gets thrown at the wall. We also want to make sure that it's not in
collision with anything at the time that it's added, because that would result
in problems for the simulation.

### Lesson 3a: Set the starting position for the object

We want to position the object in a reasonable place for us to throw it at the
wall. We also want to have the ability to randomize its location along the y-axis.

First, let's create a zero vector for the position:

```cpp
math::Vector6d positions(math::Vector6d::Zero());
```

You'll notice that this is an math::Vector**6**d rather than the usual
math::Vector**3**d. This vector has six components because the root BodyNode
has 6 degrees of freedom: three for orientation and three for translation.
Because we follow Roy Featherstone's Spatial Vector convention, the **first**
three components are for **orientation** using a logmap (also known as angle-axis)
and the **last** three components are for **translation**.

First, if randomness is turned on, we'll set the y-translation to a randomized
value and then set the height to the default value before applying the vector to
the root FreeJoint. The helper below shows the full block.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson3a-initial-position-start
         :end-before: // snippet:cpp-collisions-lesson3a-initial-position-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson3a-initial-position-start
         :end-before: # snippet:py-collisions-lesson3a-initial-position-end
```

We trust that the root Joint is a FreeJoint with 6 degrees of freedom because
of how we constructed all the objects that are going to be thrown at the wall:
They were all given a FreeJoint between the world and the root BodyNode.

### Lesson 3b: Set the object's name

Every object in the world is required to have a non-empty unique name. Just like
Joint names in a Skeleton, if we pass a Skeleton into a world with a non-unique
name, the world will print out a complaint to us and then rename it. So avoid the
ugly printout, we'll make sure the new object has a unique name ahead of time:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson3b-name-start
         :end-before: // snippet:cpp-collisions-lesson3b-name-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson3b-name-start
         :end-before: # snippet:py-collisions-lesson3b-name-end
```

### Lesson 3c: Add the object to the world without collisions

Before we add the Skeleton to the world, we want to make sure that it
isn't actually placed inside of something accidentally. If an object in a
simulation starts off inside of another object, it can result in extremely
non-physical simulations, perhaps even breaking the simulation entirely.
We can access the world's collision detector directly to check make sure the
new object is collision-free. If it isn't, we refuse to add it and print a
message explaining why.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson3c-collision-check-start
         :end-before: // snippet:cpp-collisions-lesson3c-collision-check-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson3c-collision-check-start
         :end-before: # snippet:py-collisions-lesson3c-collision-check-end
```

### Lesson 3d: Creating reference frames

DART has a unique feature that we call Frame Semantics. The Frame Semantics of
DART allow you to create reference frames and use them to get and set data
relative to arbitrary frames. There are two crucial Frame types currently used
in DART: `BodyNode`s and `SimpleFrame`s.

The BodyNode class does not allow you to explicitly set its transform, velocity,
or acceleration properties, because those are all strictly functions of the
degrees of freedom that the BodyNode depends on. Because of this, the BodyNode
is not a very convenient class if you want to create an arbitrary frame of
reference. Instead, DART offers the `SimpleFrame` class which gives you the
freedom of arbitrarily attaching it to any parent Frame and setting its transform,
velocity, and acceleration to whatever you'd like. This makes SimpleFrame useful
for specifying arbitrary reference frames.

We're going to set up a couple SimpleFrames and use them to easily specify the
velocity properties that we want the Skeleton to have. First, we'll place a
SimpleFrame at the Skeleton's center of mass:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson3d-reference-frame-start
         :end-before: // snippet:cpp-collisions-lesson3d-reference-frame-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson3d-reference-frame-start
         :end-before: # snippet:py-collisions-lesson3d-reference-frame-end
```

Calling `object->getCOM()` will tell us the center of mass location with
respect to the World Frame. We use that to set the translation of the
SimpleFrame's relative transform so that the origin of the SimpleFrame will be
located at the object's center of mass.

Now we'll set what we want the object's angular and linear speeds to be. We just
use the default values unless randomization is turned on, then convert those
speeds into directional velocities and store them on the SimpleFrame:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson3e-launch-velocity-start
         :end-before: // snippet:cpp-collisions-lesson3e-launch-velocity-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson3e-launch-velocity-start
         :end-before: # snippet:py-collisions-lesson3e-launch-velocity-end
```

The `SimpleFrame::setClassicDerivatives()` allows you to set the classic linear
and angular velocities and accelerations of a SimpleFrame with respect to its
parent Frame, which in this case is the World Frame. In DART, classic velocity and
acceleration vectors are explicitly differentiated from spatial velocity and
acceleration vectors. If you are unfamiliar with the term "spatial vector", then
you'll most likely want to work in terms of classic velocity and acceleration.

Now we want to create a new SimpleFrame that will be a child of the previous one,
align it with the root BodyNode, and then use it to set the root joint's spatial
velocity:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson3f-apply-velocity-start
         :end-before: // snippet:cpp-collisions-lesson3f-apply-velocity-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson3f-apply-velocity-start
         :end-before: # snippet:py-collisions-lesson3f-apply-velocity-end
```

Note that the FreeJoint uses spatial velocity and spatial acceleration for its
degrees of freedom.

Now we're ready to toss around objects!

## Lesson 4: Setting joint spring and damping properties

Find the `setupRing` function. This is where we'll setup a chain of BodyNodes
so that it behaves more like a closed ring.

### Lesson 4a: Set the spring and damping coefficients

We'll want to set the stiffness and damping coefficients of only the
DegreesOfFreedom that are **between** two consecutive BodyNodes. The first
six degrees of freedom are between the root BodyNode and the World, so we don't
want to change the stiffness of them, or else the object will hover unnaturally
in the air. But all the rest of the degrees of freedom should be set:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson4a-ring-stiffness-start
         :end-before: // snippet:cpp-collisions-lesson4a-ring-stiffness-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson4a-ring-stiffness-start
         :end-before: # snippet:py-collisions-lesson4a-ring-stiffness-end
```

### Lesson 4b: Set the rest positions of the joints

We want to make sure that the ring's rest position works well for the structure
it has. Using basic geometry, we know we can compute the exterior angle on each
edge of a polygon like so:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson4b-ring-rest-start
         :end-before: // snippet:cpp-collisions-lesson4b-ring-rest-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson4b-ring-rest-start
         :end-before: # snippet:py-collisions-lesson4b-ring-rest-end
```

This loop converts each desired rotation into BallJoint positions and sets the
rest pose component-wise.

### Lesson 4c: Set the Joints to be in their rest positions

Finally, we should set the ring so that all the degrees of freedom (past the
root BodyNode) start out in their rest positions:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson4c-ring-rest-state-start
         :end-before: // snippet:cpp-collisions-lesson4c-ring-rest-state-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson4c-ring-rest-state-start
         :end-before: # snippet:py-collisions-lesson4c-ring-rest-state-end
```

## Lesson 5: Create a closed kinematic chain

Find the `CollisionsEventHandler::addRing` function. In here, we'll want to
create a dynamic constraint that attaches the first and last BodyNodes of the chain
together by a BallJoint-style constraint.

First we'll grab the BodyNodes that we care about, compute the offset where the
BallJoint constraint should be located, create the constraint, register it with
the world's solver, and track it locally:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_collisions_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-collisions-lesson5-closed-chain-start
         :end-before: // snippet:cpp-collisions-lesson5-closed-chain-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/02_collisions/main_finished.py
         :language: python
         :start-after: # snippet:py-collisions-lesson5-closed-chain-start
         :end-before: # snippet:py-collisions-lesson5-closed-chain-end
```

And that's it! You're ready to run the full tutorialCollisions application!

**When running the application, keep in mind that the dynamics of collisions are
finnicky, so you may see some unstable and even completely non-physical behavior.
If the application freezes, you may need to force quit out of it.**
