# Dominoes

## Overview

This tutorial will demonstrate some of the more advanced features of DART's
dynamics API which allow you to write robust controllers that work for real
dynamic systems, such as robotic manipulators. We will show you how to:

- Clone Skeletons
- Load a URDF
- Write a stable PD controller w/ gravity and coriolis compensation
- Write an operational space controller

Please reference the source code in [**tutorials/tutorial_dominoes/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_dominoes/main.cpp) and [**tutorials/tutorial_dominoes_finished/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_dominoes_finished/main.cpp).

## Lesson 1: Cloning Skeletons

There are often times where you might want to create an exact replica of an
existing Skeleton. DART offers cloning functionality that allows you to do this
very easily.

### Lesson 1a: Create a new domino

Creating a new domino is straightforward. Find the function ``attemptToCreateDomino``
in the ``MyWindow`` class. The class has a member called ``mFirstDomino`` which
is the original domino created when the program starts up. To make a new one,
we can just clone it:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-clone-start
         :end-before: // snippet:cpp-dominoes-lesson1a-clone-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-clone-start
         :end-before: # snippet:py-dominoes-lesson1a-clone-end
```

But keep in mind that every Skeleton that gets added to a world requires its own
unique name. Creating a clone will keep the original name, so we should we give
the new copy its own name:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-name-start
         :end-before: // snippet:cpp-dominoes-lesson1a-name-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-clone-start
         :end-before: # snippet:py-dominoes-lesson1a-clone-end
```

So the easy part is finished, but now we need to get the domino to the correct
position. First, let's grab the last domino that was placed in the environment:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-last-start
         :end-before: // snippet:cpp-dominoes-lesson1a-last-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-last-start
         :end-before: # snippet:py-dominoes-lesson1a-last-end
```

Now we should compute what we want its position to be. The ``MyWindow`` class
keeps a member called ``mTotalAngle`` which tracks how much the line of dominoes
has turned so far. We'll use that to figure out what translational offset the
new domino should have from the last domino:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-offset-start
         :end-before: // snippet:cpp-dominoes-lesson1a-offset-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-offset-start
         :end-before: # snippet:py-dominoes-lesson1a-offset-end
```

And now we can compute the total position of the new domino. First, we'll copy
the positions of the last domino:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-copy-start
         :end-before: // snippet:cpp-dominoes-lesson1a-copy-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-copy-start
         :end-before: # snippet:py-dominoes-lesson1a-copy-end
```

And then we'll add the translational offset to it:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-translate-start
         :end-before: // snippet:cpp-dominoes-lesson1a-translate-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-translate-start
         :end-before: # snippet:py-dominoes-lesson1a-translate-end
```

Remember that the domino's root joint is a FreeJoint which has six degrees of
freedom: the first three are for orientation and last three are for translation.

Finally, we should add on the change in angle for the new domino:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-angle-start
         :end-before: // snippet:cpp-dominoes-lesson1a-angle-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-angle-start
         :end-before: # snippet:py-dominoes-lesson1a-angle-end
```

Be sure to uncomment the ``angle`` argument of the function.

Now we can use ``x`` to set the positions of the domino:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1a-set-positions-start
         :end-before: // snippet:cpp-dominoes-lesson1a-set-positions-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1a-set-positions-start
         :end-before: # snippet:py-dominoes-lesson1a-set-positions-end
```

The root FreeJoint is the only joint in the domino's Skeleton, so we can just
use the ``Skeleton::setPositions`` function to set it.

We'll hold off on adding the domino to the world until we confirm that it isn't
going to spawn inside another object.

### Lesson 1b: Make sure no dominoes are in collision

Similar to **Lesson 3** of the **Collisions** tutorial, we'll want to make sure
that the newly inserted Skeleton is not starting out in collision with anything,
because this could make for a very ugly (perhaps even broken) simulation. 

We grab the world's constraint solver, create a temporary collision group that
contains just the new domino, and temporarily remove the floor from the world's
collision group (otherwise every domino would appear to be colliding). After
running the collision test we add the floor back so the dominoes can rest on it
during simulation.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1b-collision-check-start
         :end-before: // snippet:cpp-dominoes-lesson1b-collision-check-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1b-collision-check-start
         :end-before: # snippet:py-dominoes-lesson1b-collision-check-end
```

If nothing else occupies that space, we can safely add the domino to the world
and update our history. Otherwise we print a warning so the user can remove a
domino and try again.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1b-result-start
         :end-before: // snippet:cpp-dominoes-lesson1b-result-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1b-result-start
         :end-before: # snippet:py-dominoes-lesson1b-result-end
```

### Lesson 1c: Delete the last domino added

Ordinarily, removing a Skeleton from a scene is just a matter of calling the
``World::removeSkeleton`` function, but we have a little bit of bookkeeping to
take care of for our particular application. First, we should check whether
there are any dominoes to actually remove:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1c-delete-start
         :end-before: // snippet:cpp-dominoes-lesson1c-delete-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1c-delete-start
         :end-before: # snippet:py-dominoes-lesson1c-delete-end
```

**Now we can add and remove dominoes from the scene. Feel free to give it a try.**

### Lesson 1d: Apply a force to the first domino

But just setting up dominoes isn't much fun without being able to knock them
down. We can quickly and easily knock down the dominoes by magically applying
a force to the first one. In the ``timeStepping`` function of ``MyWindow`` there
is a label for **Lesson 1d**. This spot will get visited whenever the user
presses 'f', so we'll apply an external force to the first domino here:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson1d-force-start
         :end-before: // snippet:cpp-dominoes-lesson1d-force-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson1d-force-start
         :end-before: # snippet:py-dominoes-lesson1d-force-end
```

## Lesson 2: Loading and controlling a robotic manipulator

Striking something with a magical force is convenient, but not very believable.
Instead, let's load a robotic manipulator and have it push over the first domino.

### Lesson 2a: Load a URDF file

Our manipulator is going to be loaded from a URDF file. URDF files are loaded
by the ``dart::io::DartLoader`` class (pending upcoming changes to DART's
loading system). First, create a loader:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2a-loader-start
         :end-before: // snippet:cpp-dominoes-lesson2a-loader-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2a-loader-start
         :end-before: # snippet:py-dominoes-lesson2a-loader-end
```

Note that many URDF files use ROS's ``package:`` scheme to specify the locations
of the resources that need to be loaded. We won't be using this in our example,
but in general you should use the function ``DartLoader::addPackageDirectory``
to specify the locations of these packages, because DART does not have the same
package resolving abilities of ROS.

Now we'll have ``loader`` parse the file into a Skeleton:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2a-parse-start
         :end-before: // snippet:cpp-dominoes-lesson2a-parse-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2a-parse-start
         :end-before: # snippet:py-dominoes-lesson2a-parse-end
```

And we should give the Skeleton a convenient name:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2a-name-start
         :end-before: // snippet:cpp-dominoes-lesson2a-name-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2a-name-start
         :end-before: # snippet:py-dominoes-lesson2a-name-end
```

Now we'll want to initialize the location and configuration of the manipulator.
Experimentation has demonstrated that the following setup is good for our purposes:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2a-base-start
         :end-before: // snippet:cpp-dominoes-lesson2a-base-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2a-base-start
         :end-before: # snippet:py-dominoes-lesson2a-base-end
```

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2a-configuration-start
         :end-before: // snippet:cpp-dominoes-lesson2a-configuration-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2a-configuration-start
         :end-before: # snippet:py-dominoes-lesson2a-configuration-end
```

And lastly, be sure to return the Skeleton that we loaded rather than the dummy
Skeleton that was originally there:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2a-return-start
         :end-before: // snippet:cpp-dominoes-lesson2a-return-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2a-return-start
         :end-before: # snippet:py-dominoes-lesson2a-return-end
```

**Feel free to load up the application to see the manipulator in the scene,
although all it will be able to do is collapse pitifully onto the floor.**

### Lesson 2b: Grab the desired joint angles

To make the manipulator actually useful, we'll want to have the ``Controller``
control its joint forces. For it to do that, the ``Controller`` class will need
to be informed of what we want the manipulator's joint angles to be. This is 
easily done in the constructor of the ``Controller`` class:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2b-desired-positions-start
         :end-before: // snippet:cpp-dominoes-lesson2b-desired-positions-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2b-desired-positions-start
         :end-before: # snippet:py-dominoes-lesson2b-desired-positions-end
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

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2c-state-start
         :end-before: // snippet:cpp-dominoes-lesson2c-state-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2c-state-start
         :end-before: # snippet:py-dominoes-lesson2c-state-end
```

Additionally, we'll integrate the position forward by one timestep:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2c-integrate-start
         :end-before: // snippet:cpp-dominoes-lesson2c-integrate-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2c-integrate-start
         :end-before: # snippet:py-dominoes-lesson2c-integrate-end
```

This is not necessary for writing a regular PD controller, but instead this is
to write a "stable PD" controller which has some better numerical stability
properties than an ordinary discrete PD controller. You can try running with and
without this line to see what effect it has on the stability.

Now we'll compute our joint position error:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2c-q-error-start
         :end-before: // snippet:cpp-dominoes-lesson2c-q-error-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2c-q-error-start
         :end-before: # snippet:py-dominoes-lesson2c-q-error-end
```

And our joint velocity error, assuming our desired joint velocity is zero:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2c-dq-error-start
         :end-before: // snippet:cpp-dominoes-lesson2c-dq-error-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2c-dq-error-start
         :end-before: # snippet:py-dominoes-lesson2c-dq-error-end
```

Now we can grab our mass matrix, which we will use to scale our force terms:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2c-mass-start
         :end-before: // snippet:cpp-dominoes-lesson2c-mass-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2c-mass-start
         :end-before: # snippet:py-dominoes-lesson2c-mass-end
```

And then combine all this into a PD controller that computes forces to minimize
our error:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2c-force-law-start
         :end-before: // snippet:cpp-dominoes-lesson2c-force-law-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2c-force-law-start
         :end-before: # snippet:py-dominoes-lesson2c-force-law-end
```

Now we're ready to set these forces on the manipulator:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2c-apply-start
         :end-before: // snippet:cpp-dominoes-lesson2c-apply-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2c-apply-start
         :end-before: # snippet:py-dominoes-lesson2c-apply-end
```

**Feel free to give this PD controller a try to see how effective it is.**

### Lesson 2d: Compensate for gravity and Coriolis forces

One of the key features of DART is the ability to easily compute the gravity and
Coriolis forces, allowing you to write much higher quality controllers than you
would be able to otherwise. This is easily done like so:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson2d-cg-start
         :end-before: // snippet:cpp-dominoes-lesson2d-cg-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson2d-cg-start
         :end-before: # snippet:py-dominoes-lesson2d-cg-end
```

And now we can update our control law by just slapping this term onto the end
of the equation:

```cpp
mForces = M * (mKpPD * q_err + mKdPD * dq_err) + Cg;
```

**Give this new PD controller a try to see how its performance compares to the
one without compensation**

## Lesson 3: Writing an operational space controller

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

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3a-end-effector-start
         :end-before: // snippet:cpp-dominoes-lesson3a-end-effector-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3a-end-effector-start
         :end-before: # snippet:py-dominoes-lesson3a-end-effector-end
```

But we don't want to use the origin of the BodyNode frame as the origin of our
Operational Space controller; instead we want to use a slight offset, to get to
the tool area of the last BodyNode:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3a-offset-start
         :end-before: // snippet:cpp-dominoes-lesson3a-offset-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3a-offset-start
         :end-before: # snippet:py-dominoes-lesson3a-offset-end
```

Also, our target will be the spot on top of the first domino, so we'll create a
reference frame and place it there. First, create the SimpleFrame:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3a-target-frame-start
         :end-before: // snippet:cpp-dominoes-lesson3a-target-frame-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3a-end-effector-start
         :end-before: # snippet:py-dominoes-lesson3a-end-effector-end
```

Then compute the transform needed to get from the center of the domino to the
top of the domino:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3a-target-offset-start
         :end-before: // snippet:cpp-dominoes-lesson3a-target-offset-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3a-target-offset-start
         :end-before: # snippet:py-dominoes-lesson3a-target-offset-end
```

And then we should rotate the target's coordinate frame to make sure that lines
up with the end effector's reference frame, otherwise the manipulator might try
to push on the domino from a very strange angle:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3a-target-rotation-start
         :end-before: // snippet:cpp-dominoes-lesson3a-target-rotation-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3a-target-rotation-start
         :end-before: # snippet:py-dominoes-lesson3a-target-rotation-end
```

Now we'll set the target so that it has a transform of ``target_offset`` with
respect to the frame of the domino:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3a-target-set-start
         :end-before: // snippet:cpp-dominoes-lesson3a-target-set-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3a-target-set-start
         :end-before: # snippet:py-dominoes-lesson3a-target-set-end
```

And this gives us all the information we need to write an Operational Space
controller.

### Lesson 3b: Computing forces for OS Controller

Find the function ``setOperationalSpaceForces()``. This is where we'll compute
the forces for our operational space controller.

One of the key ingredients in an operational space controller is the mass matrix.
We can get this easily, just like we did for the PD controller:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-mass-start
         :end-before: // snippet:cpp-dominoes-lesson3b-mass-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-mass-start
         :end-before: # snippet:py-dominoes-lesson3b-mass-end
```

Next we'll want the Jacobian of the tool offset in the end effector. We can get
it easily with this function:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-jacobian-start
         :end-before: // snippet:cpp-dominoes-lesson3b-jacobian-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-jacobian-start
         :end-before: # snippet:py-dominoes-lesson3b-jacobian-end
```

But operational space controllers typically use the Moore-Penrose pseudoinverse
of the Jacobian rather than the Jacobian itself. There are many ways to compute
the pseudoinverse of the Jacobian, but a simple way is like this:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-jacobian-start
         :end-before: // snippet:cpp-dominoes-lesson3b-jacobian-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-jacobian-start
         :end-before: # snippet:py-dominoes-lesson3b-jacobian-end
```

Note that this pseudoinverse is also damped so that it behaves better around
singularities. This is method for computing the pseudoinverse is not very
efficient in terms of the number of mathematical operations it performs, but
it is plenty fast for our application. Consider using methods based on Singular
Value Decomposition if you need to compute the pseudoinverse as fast as possible.

Next we'll want the time derivative of the Jacobian, as well as its pseudoinverse:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-jacobian-deriv-start
         :end-before: // snippet:cpp-dominoes-lesson3b-jacobian-deriv-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-jacobian-deriv-start
         :end-before: # snippet:py-dominoes-lesson3b-jacobian-deriv-end
```

Notice that here we're compute the **classic** derivative, which means the
derivative of the Jacobian with respect to time in classical coordinates rather
than spatial coordinates. If you use spatial vector arithmetic, then you'll want
to use ``BodyNode::getJacobianSpatialDeriv`` instead.

Now we can compute the linear components of error:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-linear-error-start
         :end-before: // snippet:cpp-dominoes-lesson3b-linear-error-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-linear-error-start
         :end-before: # snippet:py-dominoes-lesson3b-linear-error-end
```

And then the angular components of error:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-angular-error-start
         :end-before: // snippet:cpp-dominoes-lesson3b-angular-error-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-angular-error-start
         :end-before: # snippet:py-dominoes-lesson3b-angular-error-end
```

Then the time derivative of error, assuming our desired velocity is zero:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-error-derivative-start
         :end-before: // snippet:cpp-dominoes-lesson3b-error-derivative-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-error-derivative-start
         :end-before: # snippet:py-dominoes-lesson3b-error-derivative-end
```

Like with the PD controller, we can mix in terms to compensate for gravity and
Coriolis forces:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-cg-start
         :end-before: // snippet:cpp-dominoes-lesson3b-cg-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-cg-start
         :end-before: # snippet:py-dominoes-lesson3b-cg-end
```

The gains for the operational space controller need to be in matrix form, but
we're storing the gains as scalars, so we'll need to conver them:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-gains-kp-start
         :end-before: // snippet:cpp-dominoes-lesson3b-gains-kp-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-gains-kp-start
         :end-before: # snippet:py-dominoes-lesson3b-gains-kp-end
```

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-gains-kd-start
         :end-before: // snippet:cpp-dominoes-lesson3b-gains-kd-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-gains-kd-start
         :end-before: # snippet:py-dominoes-lesson3b-gains-kd-end
```

And we'll need to compute the joint forces needed to achieve our desired end
effector force. This is easily done using the Jacobian transpose:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-feedforward-start
         :end-before: // snippet:cpp-dominoes-lesson3b-feedforward-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-feedforward-start
         :end-before: # snippet:py-dominoes-lesson3b-feedforward-end
```

And now we can mix everything together into the single control law:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-control-law-start
         :end-before: // snippet:cpp-dominoes-lesson3b-control-law-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-control-law-start
         :end-before: # snippet:py-dominoes-lesson3b-control-law-end
```

Then don't forget to pass the forces into the manipulator:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_dominoes_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-dominoes-lesson3b-apply-start
         :end-before: // snippet:cpp-dominoes-lesson3b-apply-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/03_dominoes/main_finished.py
         :language: python
         :start-after: # snippet:py-dominoes-lesson3b-apply-start
         :end-before: # snippet:py-dominoes-lesson3b-apply-end
```

**Now you're ready to try out the full dominoes app!**
