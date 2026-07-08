Examples
========

DART 7's Python examples use the ``dartpy`` World facade. The smallest useful
example builds a scene in code, steps it, and inspects the body state:

.. code-block:: python

   import dartpy as dart

   world = dart.World(time_step=1.0 / 1000.0)

   ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
   ground.is_static = True
   ground.set_collision_shape(dart.CollisionShape.box((1.0, 1.0, 0.05)))

   box = world.add_rigid_body("box", mass=1.0, position=(0.0, 0.0, 1.0))
   box.set_collision_shape(dart.CollisionShape.box((0.1, 0.1, 0.1)))

   world.enter_simulation_mode()
   for _ in range(100):
       world.step()

   print(f"t = {world.time:.3f} s, box z = {float(box.translation[2]):.3f} m")

For the same first step with more explanation, see
:doc:`/user_guide/getting_started/hello_dart`.

Interactive demos
-----------------

The source tree's maintained Python example surface is the ``py-demos`` runner
under ``python/examples/demos``. It hosts World scenes in the same Filament
viewer used by the C++ demo app:

.. code-block:: bash

   pixi run py-demos                                # open the default rigid_body scene
   pixi run py-demos -- --scene rigid_solver_compare # compare solver behavior
   pixi run py-demos -- --list                      # print the scene catalog
   pixi run py-demo-capture -- --scene rigid_body --frames 2 --width 640 --height 360

The demo catalog includes rigid bodies, multibody contact, solver comparisons,
deformable examples, differentiable simulation scenes, and visual debugging
packets. Start with ``python/examples/README.md`` and
``python/examples/demos/README.md`` in the source checkout when you need the
current scene list and command-line options.

Legacy DART 6 examples
----------------------

If you need ``Skeleton``/``BodyNode``/``Joint`` examples such as URDF loading
with ``World.addSkeleton()``, use the stable DART 6 documentation at
https://dart.readthedocs.io/en/stable/. Those APIs are not the DART 7
Python-first path documented here.
