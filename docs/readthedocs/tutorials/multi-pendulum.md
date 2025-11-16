# Multi Pendulum

This walkthrough combines the original C++ and dartpy tutorials. The prose is
shared, while the code samples are presented in tabs so you can focus on your
preferred language. Each snippet pulls directly from the **finished** tutorial
sources to stay in sync with the code you run locally.

Source links:

- [tutorials/tutorial_multi_pendulum/main.cpp](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_multi_pendulum/main.cpp)
- [python/tutorials/01_multi_pendulum/main.py](https://github.com/dartsim/dart/blob/main/python/tutorials/01_multi_pendulum/main.py)

## Lesson 1: Apply forces during simulation

We reset each body's visuals every frame, then either apply joint torques or
body forces based on user input.

### Lesson 1a – Reset visuals

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

### Lesson 1b – Apply joint torques

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

### Lesson 1c – Apply body forces

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson1c-body-force-start
         :end-before: // snippet:cpp-lesson1c-body-force-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson1c-body-force-start
         :end-before: # snippet:py-lesson1c-body-force-end
```

## Lesson 2: Implicit spring and damping properties

The next step lets you change the pendulum's implicit spring/damper settings and
rest positions while the simulation is running.

### Lesson 2a – Rest positions

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

### Lesson 2b – Spring stiffness

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

### Lesson 2c – Damping

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

## Lesson 3: Toggle constraints

Finally, you can pin the last link of the pendulum to the world with a
`BallJointConstraint` and remove it again to return to free motion.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson3-add-constraint-start
         :end-before: // snippet:cpp-lesson3-add-constraint-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson3-add-constraint-start
         :end-before: # snippet:py-lesson3-add-constraint-end
```

Removing the constraint simply unregisters it from the world:

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_multi_pendulum_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-lesson3-remove-constraint-start
         :end-before: // snippet:cpp-lesson3-remove-constraint-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/01_multi_pendulum/main_finished.py
         :language: python
         :start-after: # snippet:py-lesson3-remove-constraint-start
         :end-before: # snippet:py-lesson3-remove-constraint-end
```

## Next steps

The finished source files include the remaining boilerplate: building the
pendulum skeleton, hooking up the viewer, and handling keyboard input. Because
these pieces are mostly identical between languages, refer to the full sources
when you need to inspect them. Future iterations of this page will add snippet
coverage for the initialization sections as we continue consolidating the
tutorials.
