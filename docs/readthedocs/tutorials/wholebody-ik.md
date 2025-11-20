# Whole-Body Inverse Kinematics

## Overview

This tutorial walks through building a whole-body inverse kinematics (IK)
controller for the Atlas humanoid. You will:

- Load Atlas and configure a comfortable standing pose
- Create hand end effectors with offsets that point to the palm centers
- Configure the IK error bounds and solver for reliable convergence
- Bias the gradient method so the floating base and joints move smoothly
- Interact with the robot through drag-and-drop or drive IK targets headlessly

Reference implementations live in:

- C++: [`tutorials/tutorial_wholebody_ik/main.cpp`](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_wholebody_ik/main.cpp) and [`tutorials/tutorial_wholebody_ik_finished/main.cpp`](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_wholebody_ik_finished/main.cpp)
- Python: [`python/tutorials/wholebody_ik/main.py`](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main.py) and [`python/tutorials/wholebody_ik/main_finished.py`](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main_finished.py)

## Run the tutorial

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. code-block:: bash

         # Build all tutorial targets (exercise + finished)
         pixi run build

         # Interactive viewer
         ./build/default/cpp/Release/bin/tutorial_wholebody_ik

         # Headless diagnostics (finished version)
         ./build/default/cpp/Release/bin/tutorial_wholebody_ik_finished \
             --headless --steps=200 --radius=0.10

   .. tab:: Python

      .. code-block:: bash

         # Interactive viewer (exercise or finished)
         pixi run python python/tutorials/wholebody_ik/main.py --mode gui
         pixi run python python/tutorials/wholebody_ik/main_finished.py --mode gui

         # Scripted headless loop with diagnostics
         pixi run python python/tutorials/wholebody_ik/main_finished.py \
             --mode headless --headless-steps=200 --trajectory-radius=0.10
```

## Lesson 1 – Load Atlas and define a resting pose

We load `atlas_v3_no_head.urdf`, bend the knees into a natural crouch, and raise
the knee joint limits to prevent hyperextension.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_wholebody_ik_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-load-atlas-start
         :end-before: // snippet:cpp-load-atlas-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/wholebody_ik/main_finished.py
         :language: python
         :start-after: # snippet:py-load-atlas-start
         :end-before: # snippet:py-load-atlas-end
```

## Lesson 2 – Create end effectors with palm offsets

Atlas’s hand frame sits near the wrist. We create a custom end effector on each
hand and shift it ±12 cm along the Y axis so the IK target aligns with the palm.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_wholebody_ik_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-create-hand-start
         :end-before: // snippet:cpp-create-hand-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/wholebody_ik/main_finished.py
         :language: python
         :start-after: # snippet:py-create-hand-start
         :end-before: # snippet:py-create-hand-end
```

## Lesson 3 – Configure IK bounds and solver settings

`InverseKinematics::ErrorMethod` only reports a non-zero error when the desired
pose falls outside the bounds, so we clamp both translation and rotation to
±1e-8. We also switch on whole-body support and tighten the solver tolerances.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_wholebody_ik_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-setup-ik-start
         :end-before: // snippet:cpp-setup-ik-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/wholebody_ik/main_finished.py
         :language: python
         :start-after: # snippet:py-setup-ik-start
         :end-before: # snippet:py-setup-ik-end
```

## Lesson 4 – Bias the gradient method for smoother motion

To keep the torso steady, we down-weight the floating-base DOFs, clamp gradient
updates, and add damping when the backend uses a DLS Jacobian solver.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_wholebody_ik_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-smooth-motion-start
         :end-before: // snippet:cpp-smooth-motion-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/wholebody_ik/main_finished.py
         :language: python
         :start-after: # snippet:py-smooth-motion-start
         :end-before: # snippet:py-smooth-motion-end
```

## Lesson 5 – Interactive drag-and-drop

The GUI variant wraps the world in a `RealTimeWorldNode`, registers a custom
event handler, and enables drag-and-drop handles on both hands so you can steer
the IK solver visually.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_wholebody_ik_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-gui-start
         :end-before: // snippet:cpp-gui-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/wholebody_ik/main_finished.py
         :language: python
         :start-after: # snippet:py-gui-start
         :end-before: # snippet:py-gui-end
```

## Lesson 6 – Headless trajectory tracking

`runHeadlessDemo` (C++) and `run_headless_demo` (Python) attach `SimpleFrame`
targets to the IK modules, march the hands along small trajectories, and log
joint vectors plus solve statistics for each step.

```{eval-rst}
.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../tutorials/tutorial_wholebody_ik_finished/main.cpp
         :language: cpp
         :start-after: // snippet:cpp-headless-start
         :end-before: // snippet:cpp-headless-end

   .. tab:: Python

      .. literalinclude:: ../../../python/tutorials/wholebody_ik/main_finished.py
         :language: python
         :start-after: # snippet:py-headless-start
         :end-before: # snippet:py-headless-end
```

## Next steps

- Add foot end effectors and support polygons to practice balance control
- Stack additional IK objectives (e.g., torso orientation, head pose)
- Integrate collision constraints or joint-limit penalties when solving
- Feed the IK outputs into a controller or physics simulation for validation
